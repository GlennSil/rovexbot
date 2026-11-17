#!/usr/bin/env python3
"""
Interface de réglage des PID de vitesse du RovexBot.

À utiliser avec le code Arduino firmware/pid_tuning/pid_tuning.ino.

    python3 pid_tuner.py            # avec l'Arduino branché
    python3 pid_tuner.py --demo     # sans Arduino : moteurs simulés, pour essayer l'interface

Dépendances : sudo apt install python3-tk python3-serial
"""
import math
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

# ---------- Apparence ----------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID = "#e4e3df"
COLOR = ["#2a78d6", "#eb6834"]          # 0 = roue droite, 1 = roue gauche
WHEEL_NAME = ["Droite", "Gauche"]
WHEEL_LETTER = ["r", "l"]

WINDOW_S = 15.0                          # durée affichée sur les courbes
PERIOD_S = 0.1                           # période d'envoi de l'Arduino

DEFAULT_GAINS = [{"p": 11.5, "i": 7.5, "d": 0.1},
                 {"p": 12.8, "i": 8.3, "d": 0.1}]


# =====================================================================
#  Liaison avec l'Arduino (ou avec le simulateur en mode --demo)
# =====================================================================
def parse_line(line):
    """'consigne_d:5.00,mesure_d:4.90,...' -> dict, ou None si ce n'est pas une ligne de mesure."""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    sample = {}
    for part in line.split(","):
        if ":" not in part:
            return None
        key, value = part.split(":", 1)
        try:
            sample[key.strip()] = float(value)
        except ValueError:
            return None
    needed = ("consigne_d", "mesure_d", "consigne_g", "mesure_g", "pwm_d_sur10", "pwm_g_sur10")
    if not all(k in sample for k in needed):
        return None
    return sample


class FakeArduino:
    """Simule pid_tuning.ino et deux moteurs, pour essayer l'interface sans matériel."""

    def __init__(self):
        self.kp = [g["p"] for g in DEFAULT_GAINS]
        self.ki = [g["i"] for g in DEFAULT_GAINS]
        self.kd = [g["d"] for g in DEFAULT_GAINS]
        self.setpoint = [0.0, 0.0]
        self.speed = [0.0, 0.0]          # vitesse réelle
        self.measure = [0.0, 0.0]        # vitesse vue par l'encodeur
        self.last_measure = [0.0, 0.0]
        self.integral = [0.0, 0.0]
        self.output = [0.0, 0.0]
        self.auto = [False, False]
        self.selected = [True, False]
        self.motor_gain = [0.125, 0.109]  # rad/s par unité de PWM (gauche un peu plus faible)
        self.dead_zone = [13.0, 16.0]
        self.replies = []

    def command(self, text):
        text = text.strip()
        if not text:
            return
        cmd = text[0]
        arg = text[1:].strip()
        if cmd == "w":
            self.selected = [arg[:1] in ("r", "b"), arg[:1] in ("l", "b")]
            return
        if cmd == "x":
            for w in range(2):
                self._set_setpoint(w, 0.0)
            return
        try:
            value = float(arg)
        except ValueError:
            return
        for w in range(2):
            if not self.selected[w]:
                continue
            if cmd == "s":
                self._set_setpoint(w, value)
            elif cmd == "p":
                self.kp[w] = value
            elif cmd == "i":
                self.ki[w] = value
            elif cmd == "d":
                self.kd[w] = value

    def _set_setpoint(self, w, value):
        self.setpoint[w] = max(0.0, value)
        if value <= 0.0:
            self.auto[w] = False
            self.output[w] = 0.0
        elif not self.auto[w]:
            self.auto[w] = True
            self.integral[w] = 0.0
            self.last_measure[w] = self.measure[w]

    def step(self):
        """Avance de 100 ms et renvoie la ligne que l'Arduino aurait envoyée."""
        dt = PERIOD_S
        quantum = 10 * 2 * math.pi / 385.0
        for w in range(2):
            # Moteur : premier ordre avec zone morte
            target = max(0.0, self.motor_gain[w] * (self.output[w] - self.dead_zone[w]))
            self.speed[w] += (target - self.speed[w]) * (1 - math.exp(-dt / 0.15))
            self.measure[w] = round(self.speed[w] / quantum) * quantum
            # PID (même algorithme que la bibliothèque PID_v1)
            if self.auto[w]:
                error = self.setpoint[w] - self.measure[w]
                self.integral[w] += self.ki[w] * dt * error
                self.integral[w] = min(255.0, max(0.0, self.integral[w]))
                d_input = self.measure[w] - self.last_measure[w]
                out = self.kp[w] * error + self.integral[w] - (self.kd[w] / dt) * d_input
                self.output[w] = min(255.0, max(0.0, out))
                self.last_measure[w] = self.measure[w]
        return ("consigne_d:%.2f,mesure_d:%.2f,consigne_g:%.2f,mesure_g:%.2f,"
                "pwm_d_sur10:%.2f,pwm_g_sur10:%.2f" % (
                    self.setpoint[0], self.measure[0], self.setpoint[1], self.measure[1],
                    self.output[0] / 10.0, self.output[1] / 10.0))


class Link(threading.Thread):
    """Lit les mesures et envoie les commandes, dans un fil séparé de l'interface."""

    def __init__(self, port, demo):
        super().__init__(daemon=True)
        self.port = port
        self.demo = demo
        self.samples = queue.Queue()
        self.commands = queue.Queue()
        self.messages = queue.Queue()     # texte pour la barre d'état
        self.running = True
        self.serial = None
        self.fake = FakeArduino() if demo else None

    def send(self, text):
        self.commands.put(text)

    def run(self):
        if self.demo:
            self._run_demo()
        else:
            self._run_serial()

    def _run_demo(self):
        next_tick = time.monotonic()
        while self.running:
            while not self.commands.empty():
                text = self.commands.get()
                self.fake.command(text)
                if text[:1] in ("p", "i", "d"):
                    for w, name in enumerate(("droite", "gauche")):
                        self.messages.put("%s : Kp=%.2f Ki=%.2f Kd=%.2f" % (
                            name, self.fake.kp[w], self.fake.ki[w], self.fake.kd[w]))
            sample = parse_line(self.fake.step())
            if sample:
                self.samples.put(sample)
            next_tick += PERIOD_S
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def _run_serial(self):
        try:
            while self.running:
                raw = self.serial.readline()
                if raw:
                    line = raw.decode("utf-8", errors="ignore")
                    sample = parse_line(line)
                    if sample:
                        self.samples.put(sample)
                    elif line.strip().startswith("#"):
                        self.messages.put(line.strip("# \r\n"))
                # Une commande à la fois, espacées, pour ne pas saturer le tampon de l'Arduino
                if not self.commands.empty():
                    self.serial.write((self.commands.get() + "\n").encode("utf-8"))
                    time.sleep(0.02)
        except Exception as error:                      # port débranché, etc.
            self.messages.put("Liaison perdue : %s" % error)
        finally:
            try:
                self.serial.close()
            except Exception:
                pass

    def stop(self):
        self.running = False


# =====================================================================
#  Mesures sur le dernier essai (réponse à un échelon de consigne)
# =====================================================================
class StepMetrics:
    def __init__(self):
        self.active = False
        self.setpoint = 0.0
        self.t0 = 0.0
        self.rise = None
        self.peak = 0.0
        self.recent = []
        self.frozen = None           # résultat conservé après l'arrêt de la roue

    def start(self, setpoint, now):
        self.frozen = None
        self.active = setpoint > 0
        self.setpoint = setpoint
        self.t0 = now
        self.rise = None
        self.peak = 0.0
        self.recent = []

    def update(self, measure, now):
        if not self.active:
            return
        if self.rise is None and measure >= 0.9 * self.setpoint:
            self.rise = now - self.t0
        self.peak = max(self.peak, measure)
        self.recent.append(measure)
        self.recent = self.recent[-10:]

    def freeze(self, now):
        """À l'arrêt de la roue : on garde le résultat de l'essai tel qu'il était."""
        if self.active:
            self.frozen = self.summary(now)
            self.active = False

    def summary(self, now):
        """Renvoie (texte, conseil)."""
        if self.frozen is not None:
            return self.frozen
        if not self.active:
            return "Pas encore d'essai.", ""
        elapsed = now - self.t0
        rise = "%.1f s" % self.rise if self.rise is not None else "…"
        overshoot = max(0.0, (self.peak - self.setpoint) / self.setpoint * 100.0)
        settled = elapsed > 2.5 and len(self.recent) == 10
        error = self.setpoint - sum(self.recent) / len(self.recent) if settled else None
        text = "Montée : %s  ·  Dépassement : %.0f %%  ·  Erreur finale : %s" % (
            rise, overshoot, ("%.2f rad/s" % error) if error is not None else "…")

        advice = ""
        if settled:
            spread = max(self.recent) - min(self.recent)
            if spread > 0.25 * self.setpoint and spread > 0.5:
                advice = "La vitesse oscille : baisse Kp (ou Ki)."
            elif overshoot > 15:
                advice = "Dépassement élevé : baisse Ki, puis Kp."
            elif abs(error) > 0.3:
                advice = "La vitesse n'atteint pas la consigne : augmente Ki."
            elif self.rise is not None and self.rise > 1.0:
                advice = "Réponse lente : augmente Kp."
            else:
                advice = "Bon réglage : rapide, peu de dépassement, pas d'erreur finale."
        elif elapsed > 2.5 and self.rise is None:
            advice = "La roue n'atteint pas 90 % de la consigne : augmente Kp, puis Ki."
        return text, advice


# =====================================================================
#  Graphique simple dessiné sur un Canvas (pas de dépendance à matplotlib)
# =====================================================================
def nice_step(span, target=5):
    raw = span / float(target)
    power = 10 ** math.floor(math.log10(raw))
    for mult in (1, 2, 5, 10):
        if raw <= mult * power:
            return mult * power
    return 10 * power


class Plot(tk.Canvas):
    def __init__(self, parent, title, unit, fixed_max=None, **kwargs):
        super().__init__(parent, background=SURFACE, highlightthickness=0, **kwargs)
        self.title = title
        self.unit = unit
        self.fixed_max = fixed_max
        self.series = []            # liste de (nom, couleur, pointillé, [(t, y), ...])
        self.hover_x = None
        self.bind("<Motion>", self._on_motion)
        self.bind("<Leave>", self._on_leave)
        self.bind("<Configure>", lambda e: self.redraw())

    def set_series(self, series):
        self.series = series
        self.redraw()

    def _on_motion(self, event):
        self.hover_x = event.x
        self.redraw()

    def _on_leave(self, event):
        self.hover_x = None
        self.redraw()

    def redraw(self):
        self.delete("all")
        width = self.winfo_width()
        height = self.winfo_height()
        if width < 80 or height < 80:
            return
        left, right, top, bottom = 46, 14, 44, 24
        x0, x1, y0, y1 = left, width - right, top, height - bottom

        # Échelle verticale
        if self.fixed_max is not None:
            y_max = self.fixed_max
            step = nice_step(y_max)
        else:
            data_max = max([y for s in self.series for (_, y) in s[3]] + [1.0])
            step = nice_step(max(data_max * 1.15, 4.0))
            y_max = math.ceil(max(data_max * 1.15, 4.0) / step) * step

        def to_x(t):                 # t en secondes, 0 = maintenant, négatif = passé
            return x1 + (t / WINDOW_S) * (x1 - x0)

        def to_y(y):
            return y1 - (min(y, y_max) / y_max) * (y1 - y0)

        # Titre
        self.create_text(x0, 12, text=self.title, anchor="w", fill=INK,
                         font=("TkDefaultFont", 10, "bold"))

        # Grille et graduations
        value = 0.0
        while value <= y_max + 1e-9:
            y = to_y(value)
            self.create_line(x0, y, x1, y, fill=GRID)
            self.create_text(x0 - 6, y, text="%g" % value, anchor="e", fill=INK_SECONDARY,
                             font=("TkDefaultFont", 8))
            value += step
        for seconds in range(0, int(WINDOW_S) + 1, 5):
            x = to_x(-seconds)
            self.create_line(x, y1, x, y1 + 4, fill=GRID)
            label = "maintenant" if seconds == 0 else "-%d s" % seconds
            self.create_text(x, y1 + 12, text=label, anchor="e" if seconds == 0 else "center",
                             fill=INK_SECONDARY, font=("TkDefaultFont", 8))
        self.create_line(x0, y1, x1, y1, fill=INK_SECONDARY)

        # Légende
        lx = x0
        for name, color, dashed, _ in self.series:
            self.create_line(lx, 30, lx + 22, 30, fill=color, width=2,
                             dash=(5, 3) if dashed else None)
            item = self.create_text(lx + 27, 30, text=name, anchor="w", fill=INK_SECONDARY,
                                    font=("TkDefaultFont", 8))
            lx = self.bbox(item)[2] + 16

        # Courbes
        for name, color, dashed, points in self.series:
            coords = []
            for t, y in points:
                if t >= -WINDOW_S:
                    coords.extend((to_x(t), to_y(y)))
            if len(coords) >= 4:
                self.create_line(*coords, fill=color, width=2,
                                 dash=(5, 3) if dashed else None, joinstyle="round")

        # Lecture au survol
        if self.hover_x is not None and x0 <= self.hover_x <= x1:
            t_hover = (self.hover_x - x1) / (x1 - x0) * WINDOW_S
            self.create_line(self.hover_x, y0, self.hover_x, y1, fill=INK_SECONDARY, dash=(2, 2))
            parts = ["%.1f s" % t_hover]
            for name, color, dashed, points in self.series:
                if not points:
                    continue
                t, y = min(points, key=lambda p: abs(p[0] - t_hover))
                if abs(t - t_hover) < 0.3:
                    parts.append("%s : %.2f %s" % (name, y, self.unit))
                    if not dashed:
                        px, py = to_x(t), to_y(y)
                        self.create_oval(px - 4, py - 4, px + 4, py + 4, fill=color,
                                         outline=SURFACE, width=2)
            self.create_text(x1, 12, text="    ".join(parts), anchor="e", fill=INK,
                             font=("TkDefaultFont", 8))


# =====================================================================
#  Fenêtre principale
# =====================================================================
class App:
    def __init__(self, root, demo=False, port="/dev/ttyACM0"):
        self.root = root
        self.demo = demo
        self.link = None
        self.ready = False               # liaison établie et gains envoyés
        self.gains = [dict(g) for g in DEFAULT_GAINS]
        self.running = [False, False]
        self.metrics = [StepMetrics(), StepMetrics()]
        self.history = []                # liste de (instant, échantillon)
        self.confirmed = {}              # derniers gains confirmés par l'Arduino
        self.retest_job = None

        root.title("RovexBot — Réglage des PID" + ("  (démonstration)" if demo else ""))
        root.geometry("1220x720")
        root.minsize(1040, 640)
        root.protocol("WM_DELETE_WINDOW", self.close)

        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Go.TButton", font=("TkDefaultFont", 11, "bold"), padding=8)
        style.configure("Go.TButton", background=COLOR[0], foreground="#ffffff")
        style.map("Go.TButton", background=[("disabled", "#c9c8c3"), ("active", "#1c5cab")])
        style.configure("Stop.TButton", font=("TkDefaultFont", 11, "bold"), padding=8)
        style.configure("Wheel.TRadiobutton", font=("TkDefaultFont", 10), padding=4)

        self._build_top_bar(port)
        body = ttk.Frame(root, padding=(10, 0, 10, 10))
        body.pack(fill="both", expand=True)
        controls = ttk.Frame(body, width=380)
        controls.pack(side="left", fill="y", padx=(0, 10))
        controls.pack_propagate(False)
        self._build_controls(controls)
        plots = ttk.Frame(body)
        plots.pack(side="left", fill="both", expand=True)
        self.speed_plot = Plot(plots, "Vitesse des roues (rad/s)", "rad/s")
        self.speed_plot.pack(fill="both", expand=True, pady=(0, 8))
        self.pwm_plot = Plot(plots, "Commande envoyée aux moteurs (PWM, 0 à 255)", "", fixed_max=250,
                             height=190)
        self.pwm_plot.pack(fill="x")
        self._build_results(plots)

        self._load_gains_into_sliders()
        self._refresh_gain_summary()
        self._set_controls_state(False)
        self.root.after(100, self._tick)
        if demo:
            self.connect()

    # ---------------- Construction de l'interface ----------------
    def _build_top_bar(self, port):
        bar = ttk.Frame(self.root, padding=10)
        bar.pack(fill="x")
        ttk.Label(bar, text="Port de l'Arduino").pack(side="left")
        self.port_var = tk.StringVar(value=port)
        self.port_entry = ttk.Entry(bar, textvariable=self.port_var, width=16)
        self.port_entry.pack(side="left", padx=6)
        self.connect_button = ttk.Button(bar, text="Connecter", command=self.toggle_connection)
        self.connect_button.pack(side="left")
        self.status_var = tk.StringVar(value="Non connecté. Branche l'Arduino puis clique sur Connecter.")
        ttk.Label(bar, textvariable=self.status_var, foreground=INK_SECONDARY).pack(side="left", padx=12)
        if self.demo:
            self.port_entry.configure(state="disabled")
            self.connect_button.configure(state="disabled")

    def _build_controls(self, parent):
        # 1. Roue
        box = ttk.LabelFrame(parent, text="1. Roue à régler", padding=8)
        box.pack(fill="x", pady=(0, 8))
        self.wheel_var = tk.StringVar(value="r")
        self.wheel_buttons = []
        for text, value in (("Droite", "r"), ("Gauche", "l"), ("Les deux", "b")):
            button = ttk.Radiobutton(box, text=text, value=value, variable=self.wheel_var,
                                     style="Wheel.TRadiobutton", command=self._on_wheel_change)
            button.pack(side="left", expand=True)
            self.wheel_buttons.append(button)

        # 2. Consigne
        box = ttk.LabelFrame(parent, text="2. Vitesse demandée (rad/s)", padding=8)
        box.pack(fill="x", pady=(0, 8))
        self.setpoint_var = tk.DoubleVar(value=5.0)
        self.setpoint_scale = self._slider(box, self.setpoint_var, 0.0, 20.0, 0.5, self._on_setpoint_change)
        row = ttk.Frame(box)
        row.pack(fill="x", pady=(6, 0))
        self.start_button = ttk.Button(row, text="▶  Démarrer", style="Go.TButton", command=self.start)
        self.start_button.pack(side="left", expand=True, fill="x", padx=(0, 4))
        self.stop_button = ttk.Button(row, text="■  Arrêter", style="Stop.TButton", command=self.stop)
        self.stop_button.pack(side="left", expand=True, fill="x", padx=(4, 0))

        # 3. Gains
        box = ttk.LabelFrame(parent, text="3. Gains du PID (appliqués tout de suite)", padding=8)
        box.pack(fill="x", pady=(0, 8))
        self.gain_vars = {}
        self.gain_scales = {}
        for letter, label, maximum, step in (("p", "Kp  —  réactivité", 40.0, 0.1),
                                             ("i", "Ki  —  supprime l'erreur finale", 40.0, 0.1),
                                             ("d", "Kd  —  amortissement (laisser à 0)", 2.0, 0.01)):
            ttk.Label(box, text=label).pack(anchor="w", pady=(4, 0))
            var = tk.DoubleVar(value=0.0)
            self.gain_vars[letter] = var
            self.gain_scales[letter] = self._slider(
                box, var, 0.0, maximum, step, lambda l=letter: self._on_gain_change(l))
        self.auto_retest_var = tk.BooleanVar(value=True)
        self.auto_retest_check = ttk.Checkbutton(
            box, variable=self.auto_retest_var,
            text="Refaire l'essai après chaque changement\n(arrêt d'une seconde, puis redémarrage)")
        self.auto_retest_check.pack(anchor="w", pady=(8, 0))

        # 5. Gains actuels
        box = ttk.LabelFrame(parent, text="Gains actuels", padding=8)
        box.pack(fill="x")
        self.summary_var = tk.StringVar()
        ttk.Label(box, textvariable=self.summary_var, font=("TkFixedFont", 9), justify="left").pack(
            side="left", anchor="w")
        self.copy_button = ttk.Button(box, text="Copier", command=self.copy_gains)
        self.copy_button.pack(side="right")

    def _build_results(self, parent):
        box = ttk.LabelFrame(parent, text="Résultat du dernier essai", padding=8)
        box.pack(fill="x", pady=(8, 0))
        self.result_vars = []
        self.advice_vars = []
        for w in range(2):
            column = ttk.Frame(box)
            column.pack(side="left", fill="x", expand=True, anchor="n")
            row = ttk.Frame(column)
            row.pack(fill="x")
            swatch = tk.Canvas(row, width=14, height=14, highlightthickness=0)
            swatch.create_rectangle(1, 4, 13, 10, fill=COLOR[w], outline="")
            swatch.pack(side="left")
            ttk.Label(row, text="Roue " + WHEEL_NAME[w].lower(),
                      font=("TkDefaultFont", 9, "bold")).pack(side="left", padx=(4, 0))
            var = tk.StringVar(value="")
            self.result_vars.append(var)
            ttk.Label(column, textvariable=var, wraplength=360, justify="left",
                      font=("TkDefaultFont", 8)).pack(anchor="w", padx=(18, 0))
            advice = tk.StringVar(value="")
            self.advice_vars.append(advice)
            ttk.Label(column, textvariable=advice, foreground=INK_SECONDARY, wraplength=360,
                      justify="left", font=("TkDefaultFont", 8, "italic")).pack(anchor="w", padx=(18, 0))

    def _slider(self, parent, variable, minimum, maximum, step, on_change):
        """Curseur + boutons − / + pour les réglages fins. Renvoie la liste de ses widgets."""
        row = ttk.Frame(parent)
        row.pack(fill="x")

        def nudge(direction):
            value = min(maximum, max(minimum, variable.get() + direction * step))
            variable.set(round(value, 3))
            on_change()

        minus = ttk.Button(row, text="−", width=3, command=lambda: nudge(-1))
        minus.pack(side="left")
        scale = tk.Scale(row, variable=variable, from_=minimum, to=maximum, resolution=step,
                         orient="horizontal", showvalue=True, highlightthickness=0, length=230)
        scale.pack(side="left", fill="x", expand=True, padx=4)
        scale.bind("<ButtonRelease-1>", lambda e: on_change())
        scale.bind("<KeyRelease>", lambda e: on_change())
        plus = ttk.Button(row, text="+", width=3, command=lambda: nudge(+1))
        plus.pack(side="left")
        return [minus, scale, plus]

    def _set_controls_state(self, enabled):
        state = "normal" if enabled else "disabled"
        widgets = self.wheel_buttons + [self.start_button, self.stop_button, self.auto_retest_check]
        widgets += self.setpoint_scale
        for group in self.gain_scales.values():
            widgets += group
        for widget in widgets:
            widget.configure(state=state)

    # ---------------- Connexion ----------------
    def toggle_connection(self):
        if self.link is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        if self.demo:
            self.link = Link(None, demo=True)
            self.link.start()
            self.status_var.set("Mode démonstration : moteurs simulés, aucun Arduino n'est utilisé.")
            self._send_all_gains()
            return
        try:
            import serial
        except ImportError:
            self.status_var.set("Module série manquant : sudo apt install python3-serial")
            return
        try:
            port = serial.Serial(port=self.port_var.get().strip(), baudrate=115200, timeout=0.05)
        except Exception as error:
            self.status_var.set("Connexion impossible : %s" % error)
            return
        self.link = Link(self.port_var.get(), demo=False)
        self.link.serial = port
        self.link.start()
        self.connect_button.configure(text="Déconnecter")
        self.status_var.set("Connexion… l'Arduino redémarre (3 secondes).")
        # L'ouverture du port redémarre l'Arduino : on attend avant d'envoyer les gains
        self.root.after(3000, self._send_all_gains)

    def _send_all_gains(self):
        if self.link is None:
            return
        self.link.send("x")
        for w in range(2):
            self.link.send("w " + WHEEL_LETTER[w])
            for letter in ("p", "i", "d"):
                self.link.send("%s %.3f" % (letter, self.gains[w][letter]))
        self.ready = True
        self._set_controls_state(True)
        if not self.demo:
            self.status_var.set("Connecté. Choisis une roue, une vitesse, puis clique sur Démarrer.")

    def disconnect(self):
        if self.link is not None:
            self.link.send("x")
            time.sleep(0.15)
            self.link.stop()
            self.link = None
        self.ready = False
        self.running = [False, False]
        self._set_controls_state(False)
        self.connect_button.configure(text="Connecter")
        self.status_var.set("Déconnecté.")

    # ---------------- Actions ----------------
    def _selected_wheels(self):
        return {"r": [0], "l": [1], "b": [0, 1]}[self.wheel_var.get()]

    def _on_wheel_change(self):
        self._load_gains_into_sliders()

    def _load_gains_into_sliders(self):
        wheel = self._selected_wheels()[0]
        for letter in ("p", "i", "d"):
            self.gain_vars[letter].set(self.gains[wheel][letter])

    def _on_gain_change(self, letter):
        if not self.ready:
            return
        value = round(self.gain_vars[letter].get(), 3)
        for w in self._selected_wheels():
            if self.gains[w][letter] != value:
                self.gains[w][letter] = value
                self.link.send("w " + WHEEL_LETTER[w])
                self.link.send("%s %.3f" % (letter, value))
                self.status_var.set("Envoyé : K%s = %g, roue %s. Attente de la confirmation de l'Arduino…"
                                    % (letter, value, WHEEL_NAME[w].lower()))
        self._refresh_gain_summary()
        # Un gain change la façon d'atteindre la consigne, pas la vitesse une fois stabilisée :
        # pour voir son effet, il faut refaire un essai (arrêt puis redémarrage).
        if self.auto_retest_var.get() and any(self.running[w] for w in self._selected_wheels()):
            self._schedule_retest()

    def _schedule_retest(self):
        if self.retest_job is not None:
            self.root.after_cancel(self.retest_job)
        wheels = self._selected_wheels()
        self.link.send("x")
        now = time.monotonic()
        for w in range(2):
            self.metrics[w].freeze(now)
        self.retest_job = self.root.after(1200, lambda: self._retest(wheels))

    def _retest(self, wheels):
        self.retest_job = None
        if self.ready and self._selected_wheels() == wheels:
            self.start()

    def _on_setpoint_change(self):
        # Si la roue tourne déjà, la nouvelle consigne s'applique tout de suite
        if self.ready and any(self.running[w] for w in self._selected_wheels()):
            self.start()

    def start(self):
        if not self.ready:
            return
        setpoint = self.setpoint_var.get()
        now = time.monotonic()
        for w in self._selected_wheels():
            self.link.send("w " + WHEEL_LETTER[w])
            self.link.send("s %.2f" % setpoint)
            self.running[w] = setpoint > 0
            self.metrics[w].start(setpoint, now)

    def stop(self):
        if self.retest_job is not None:
            self.root.after_cancel(self.retest_job)
            self.retest_job = None
        if self.link is not None:
            self.link.send("x")
        now = time.monotonic()
        for w in range(2):
            self.metrics[w].freeze(now)
        self.running = [False, False]

    def copy_gains(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.summary_var.get())
        self.status_var.set("Gains copiés dans le presse-papiers.")

    def _refresh_gain_summary(self):
        lines = []
        for w in range(2):
            g = self.gains[w]
            lines.append("%-7s Kp = %-6g Ki = %-6g Kd = %g" % (WHEEL_NAME[w], g["p"], g["i"], g["d"]))
        self.summary_var.set("\n".join(lines))

    # ---------------- Rafraîchissement ----------------
    def _tick(self):
        now = time.monotonic()
        if self.link is not None:
            while not self.link.samples.empty():
                sample = self.link.samples.get()
                self.history.append((now, sample))
                self.metrics[0].update(sample["mesure_d"], now)
                self.metrics[1].update(sample["mesure_g"], now)
            while not self.link.messages.empty():
                message = self.link.messages.get()
                if message.startswith("Liaison perdue"):
                    self.status_var.set(message)
                    self.disconnect()
                    break
                for side in ("droite", "gauche"):
                    if message.startswith(side + " :"):
                        self.confirmed[side] = message
                        self.status_var.set("Confirmé par l'Arduino  —  " + "   |   ".join(
                            self.confirmed[k] for k in ("droite", "gauche") if k in self.confirmed))
            self.history = [(t, s) for (t, s) in self.history if now - t <= WINDOW_S + 1.0]

        def points(key, scale=1.0):
            return [(t - now, s[key] * scale) for (t, s) in self.history]

        self.speed_plot.set_series([
            ("Droite, mesure", COLOR[0], False, points("mesure_d")),
            ("Droite, consigne", COLOR[0], True, points("consigne_d")),
            ("Gauche, mesure", COLOR[1], False, points("mesure_g")),
            ("Gauche, consigne", COLOR[1], True, points("consigne_g")),
        ])
        self.pwm_plot.set_series([
            ("Droite", COLOR[0], False, points("pwm_d_sur10", 10.0)),
            ("Gauche", COLOR[1], False, points("pwm_g_sur10", 10.0)),
        ])
        for w in range(2):
            text, advice = self.metrics[w].summary(now)
            self.result_vars[w].set(text)
            self.advice_vars[w].set(advice)
        self.root.after(100, self._tick)

    def close(self):
        if self.link is not None:
            self.link.send("x")
            time.sleep(0.15)
            self.link.stop()
        self.root.destroy()


def main():
    demo = "--demo" in sys.argv
    ports = [a for a in sys.argv[1:] if not a.startswith("--")]
    root = tk.Tk()
    App(root, demo=demo, port=ports[0] if ports else "/dev/ttyACM0")
    root.mainloop()


if __name__ == "__main__":
    main()
