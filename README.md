# RovexBot

Robot mobile à conduite différentielle sous **ROS 2 Humble**, développé en simulation (Gazebo) puis porté sur un robot réel piloté par une Jetson Orin Nano et un Arduino Uno.

Le même code de contrôle tourne dans les deux cas : seule la couche matérielle change (plugin Gazebo en simulation, liaison série vers l'Arduino sur le robot réel), sélectionnée par l'argument `is_sim` de l'URDF.

## Crédits

Ce projet s'appuie sur le travail d'**Antonio Brandi** et de son robot **Bumperbot**, que j'ai suivi à travers ses cours *Self Driving and ROS 2 – Learn by Doing*.

- Dépôt Bumperbot : https://github.com/AntoBrandi/Bumper-Bot
- Cours Odometry & Control : https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control
- Cours Map & Localization : https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Map-Localization

L'architecture des packages, les nodes de contrôle, de localisation et de cartographie, ainsi que la base de la hardware interface et du firmware Arduino proviennent de ce travail.

Ce que j'ai fait de mon côté :

- adaptation à mon propre robot : modèle URDF, maillages, dimensions (roues de 65 mm, entraxe de 160 mm) ;
- hardware interface `ros2_control` (`RovexbotInterface`) : corrections de robustesse et sélection simulation / robot réel par argument `is_sim` ;
- firmware Arduino : lecture des encodeurs sécurisée vis-à-vis des interruptions, gestion explicite du sens de rotation, PID réglés sur le robot ;
- outil graphique de réglage des PID (`rovexbot_firmware/tools/pid_tuner.py`) ;
- driver MPU6050 porté sur la Jetson Orin Nano : bus I2C paramétrable, lecture en bloc, covariances, calibration du biais du gyroscope ;
- fusion odométrie des roues + IMU par EKF, en source unique de la pose ;
- fichiers de lancement `rovexbot_bringup` pour la simulation et le robot réel.

Les mondes Gazebo `small_house` et `small_warehouse` viennent des modèles AWS RoboMaker.

## Matériel

| Élément | Rôle |
|---|---|
| Jetson Orin Nano | Ordinateur embarqué (ROS 2 Humble) |
| Arduino Uno R3 | Commande des moteurs, lecture des encodeurs, PID de vitesse |
| L298N | Pont en H pour les deux moteurs |
| 2 moteurs CC avec encodeurs en quadrature | Roues motrices (385 impulsions par tour de roue) |
| MPU6050 | Centrale inertielle (I2C) |

## Packages

| Package | Contenu |
|---|---|
| `rovexbot_description` | URDF/xacro, maillages, mondes Gazebo, lancement de la simulation |
| `rovexbot_controller` | Configuration `diff_drive_controller`, contrôleurs pédagogiques, téléopération (manette, `twist_mux`) |
| `rovexbot_firmware` | Hardware interface `ros2_control` (C++), codes Arduino, driver MPU6050, outil de réglage des PID |
| `rovexbot_localization` | Filtre de Kalman, EKF (`robot_localization`), AMCL |
| `rovexbot_mapping` | Cartographie à poses connues, `slam_toolbox`, cartes |
| `rovexbot_utils` | Arrêt de sécurité |
| `rovexbot_bringup` | Lancement complet : simulation ou robot réel |

## Architecture

### Chaîne de commande

```
clavier / manette
      │  Twist
      ▼
  twist_mux ──▶ twist_relay ──▶ diff_drive_controller
                                      │  vitesses des roues (rad/s)
                                      ▼
                         RovexbotInterface (ros2_control)
                                      │  port série, 115200 bauds
                                      ▼
                         Arduino : PID de vitesse ──▶ L298N ──▶ moteurs
                                      ▲
                                 encodeurs
```

### Chaîne d'estimation de la pose

```
diff_drive_controller ──▶ /rovexbot_controller/odom ─┐
                                                     ├──▶ EKF ──▶ TF odom → base_footprint
mpu6050_driver ──────────▶ /imu/out ─────────────────┘   (robot_localization)
```

L'EKF est la **seule** source de la transformation `odom → base_footprint` : `diff_drive_controller`
a `enable_odom_tf: false`, sans quoi deux nodes publieraient la même transformation.

Le MPU6050 n'a pas de magnétomètre, donc pas d'orientation absolue : seule la vitesse de
rotation autour de z (`vyaw`) est fusionnée, avec les vitesses linéaires des roues. C'est
précisément ce qui compense le patinage en rotation, invisible pour les encodeurs seuls.

### Protocole série

| Sens | Format | Exemple |
|---|---|---|
| Jetson → Arduino (consignes) | `r<signe><vitesse>,l<signe><vitesse>,` | `rp05.00,ln03.20,` |
| Arduino → Jetson (mesures, toutes les 100 ms) | `r<signe><vitesse>,l<signe><vitesse>,` | `rp4.73,ln2.10,` |

`r` / `l` : roue droite / gauche. `p` / `n` : sens positif / négatif. Vitesses en rad/s.

Le signe renvoyé par l'Arduino vient du **sens commandé**, pas de la phase B des encodeurs
(voir *Problèmes rencontrés*).

## Câblage

**Arduino Uno**

| Broche | Connexion |
|---|---|
| 9 | L298N `ENA` (PWM roue gauche) |
| 12, 13 | L298N `IN1`, `IN2` (sens roue gauche) |
| 11 | L298N `ENB` (PWM roue droite) |
| 7, 8 | L298N `IN3`, `IN4` (sens roue droite) |
| 3, 5 | Encodeur droit, phases A et B |
| 2, 4 | Encodeur gauche, phases A et B |

Les deux moteurs sont câblés en sens inverse de la marche avant du robot ; c'est corrigé
dans le firmware par `RIGHT_MOTOR_INVERT` et `LEFT_MOTOR_INVERT`.

Les cavaliers `ENA` et `ENB` du L298N sont retirés, et les masses de l'Arduino et du L298N
sont reliées. Le cavalier 5 V du L298N doit rester retiré quand la batterie est branchée.

**MPU6050 sur la Jetson Orin Nano** (connecteur 40 broches)

| MPU6050 | Jetson |
|---|---|
| VCC | Broche 1 (3,3 V) |
| GND | Broche 6 |
| SDA | Broche 3 |
| SCL | Broche 5 |

Ces broches correspondent au bus `/dev/i2c-7` (et non `/dev/i2c-1` comme sur une Raspberry Pi).
Vérification : `i2cdetect -y -r 7` doit afficher `68`.

## Installation

```bash
mkdir -p ~/rovexbot_ws && cd ~/rovexbot_ws
git clone https://github.com/GlennSil/rovexbot.git src
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Pour le robot réel, téléverser `rovexbot_firmware/firmware/robot_control/robot_control.ino`
sur l'Arduino (bibliothèque **PID** de Brett Beauregard requise).

## Utilisation

**Simulation**

```bash
ros2 launch rovexbot_bringup simulated_robot.launch.py
```

Arguments : `world_name` et `map_name` (par défaut `small_house` pour les deux — le monde
Gazebo et la carte d'AMCL doivent représenter le même environnement).

**Robot réel** (sur la Jetson)

```bash
ros2 launch rovexbot_bringup real_robot.launch.py
```

Arguments : `port` (par défaut `/dev/ttyACM0`) et `i2c_bus` (par défaut `7`).

Le robot doit être **immobile au démarrage** : le driver MPU6050 mesure le biais du
gyroscope pendant les 2 premières secondes.

**Pilotage au clavier**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=key_vel -p speed:=0.1 -p turn:=0.5
```

## Réglage des PID de vitesse

Chaque roue a son propre PID sur l'Arduino, à 10 Hz (période de 100 ms, la même que la
mesure des encodeurs).

### Outil

`rovexbot_firmware/tools/pid_tuner.py` est une interface graphique qui pilote le croquis
`firmware/pid_tuning/pid_tuning.ino` : choix de la roue, consigne, curseurs Kp / Ki / Kd
appliqués à chaud, courbes en direct de la vitesse et du PWM, et mesure automatique du
temps de montée, du dépassement et de l'erreur finale.

```bash
sudo apt install python3-tk python3-serial
python3 rovexbot_firmware/tools/pid_tuner.py           # Arduino branché
python3 rovexbot_firmware/tools/pid_tuner.py --demo    # moteurs simulés, sans matériel
```

### Méthode suivie

1. Ki et Kd à 0, augmenter Kp jusqu'à l'oscillation entretenue, puis revenir à 60 %.
2. Augmenter Ki jusqu'à ce que la vitesse rejoigne la consigne en moins d'une seconde.
3. Laisser Kd à 0 : la mesure avance par pas de 0,16 rad/s (une impulsion d'encodeur par
   fenêtre de 100 ms), et le terme dérivé prend ces sauts pour de vraies variations.
4. Vérifier à 2, 5 et 10 rad/s.

### Gains retenus

| Roue | Kp | Ki | Kd | Kp critique mesuré |
|---|---|---|---|---|
| Droite | 16,5 | 16,0 | 0 | 27,4 |
| Gauche | 16,0 | 16,0 | 0 | 28,8 |

Résultat : montée en 0,4 s, dépassement de 4 %, erreur finale nulle, de 2 à 10 rad/s.

Un point instructif : avec Ki = 6,7, la roue atteignait 90 % de la consigne en 0,4 s mais
mettait 5 s à se stabiliser. En régime établi le moteur demande environ 55 de PWM, que le
terme proportionnel ne fournit plus quand l'erreur devient petite — c'est l'intégrale qui
doit les produire, et elle ne montait que d'environ 7 points de PWM par seconde. Passer
Ki à 16 a supprimé ce traînage.

## Problèmes rencontrés

Les erreurs qui ont coûté le plus de temps, et ce qu'elles ont appris.

### Firmware et électronique

| Problème | Symptôme | Cause et correction |
|---|---|---|
| Moteurs et encodeurs croisés | Une roue s'emballe à fond, l'autre ralentit puis s'arrête | Le canal A du L298N pilotait la roue gauche alors que le code le croyait à droite. Chaque PID commandait un moteur en lisant l'encodeur de l'autre roue : voyant sa mesure à 0, il poussait le PWM au maximum. Diagnostic fait avec `pid_tuner.py` (PWM à 250 d'un côté, mesure à 0, et la courbe de l'autre roue qui monte). |
| Sens de marche inversé | `i` au clavier fait reculer le robot, `j` tourne à droite | Les deux moteurs sont câblés à l'envers. Corrigé par `RIGHT_MOTOR_INVERT` / `LEFT_MOTOR_INVERT` plutôt qu'en recâblant. |
| Signe de vitesse incohérent | Rotations erratiques, odométrie incohérente | Le sens était déduit de la phase B, lue une seule fois par front montant de la phase A : faux dès que la roue vibre ou change de sens. Le signe vient maintenant du sens commandé ; l'encodeur ne fournit plus que la valeur absolue. |
| Compteur d'encodeur jamais remis à zéro | La « vitesse » augmente sans fin à vitesse constante | On mesurait le nombre total d'impulsions depuis l'allumage, pas une vitesse. Remise à zéro après chaque fenêtre de 100 ms. |
| Variables d'interruption non protégées | Valeurs aberrantes, blocages aléatoires | Compteurs non déclarés `volatile`, lus en deux temps sur 16 bits, et `String` alloué dans une routine d'interruption. Corrigé par `volatile`, lecture atomique entre `noInterrupts()` / `interrupts()`, et `char` au lieu de `String`. |
| `Serial.readString()` | Environ 1 s de latence sur chaque commande | La fonction attend l'expiration du timeout. Remplacée par une lecture caractère par caractère ; le node ROS ajoute un `\n`. |
| Cavalier `ENA` en place | Le moteur tourne à fond, le PWM n'a aucun effet | `ENA` forcé à 5 V. Cavalier retiré. |
| Alimentation des moteurs coupée | Tout a l'air de fonctionner mais rien ne bouge | L'USB alimente l'Arduino, pas les moteurs : le robot répond en série sans bouger. |
| Puce USB de l'Arduino détruite | La LED reste allumée, mais plus aucun `/dev/ttyACM*` ni entrée `lsusb` | Carte remplacée. Causes probables : conflit entre le 5 V du L298N et celui de l'USB, ou parasites des moteurs remontant par la masse commune. |

### ROS 2

| Problème | Symptôme | Cause et correction |
|---|---|---|
| URDF sans branche robot réel | Le plugin `RovexbotInterface` n'est jamais chargé | Les deux branches de `rovexbot_ros2_control.xacro` chargeaient le plugin Gazebo. Ajout d'un argument `is_sim` et d'une branche série. |
| Limites de vitesse des roues | Robot plafonné à environ 3 cm/s | `min`/`max` à ±1 rad/s, alors que 0,7 m/s demandent ±21,5 rad/s. Portées à ±22. |
| `reserve()` au lieu de `resize()` | Fonctionne par chance, comportement indéfini | `export_state_interfaces` prenait l'adresse de cases qui n'existaient pas encore. |
| `read()` sans `try`/`catch` | Le `controller_manager` plante au démarrage | La première ligne série après l'ouverture du port est souvent tronquée ; `std::stod` levait une exception. Lignes incomplètes ignorées avec un avertissement. |
| Topic de la manette | Les commandes de la manette n'arrivent jamais | `twist_relay` publiait sur `input_joy/cmd_vel` et `twist_mux` écoutait `joy_vel`. |
| Monde et carte différents | AMCL ne converge pas | Gazebo démarrait dans `empty` pendant qu'AMCL chargeait `small_house`. Arguments `world_name` et `map_name` ajoutés au bringup. |
| Particules AMCL invisibles | Rien ne s'affiche dans RViz | AMCL ne publie qu'après un déplacement d'au moins 25 cm ou 0,2 rad. Il fallait simplement faire rouler le robot. |
| Carte `small_warehouse` | `map_server` échoue | `map.yaml` référençait `map.png` au lieu de `map.pgm`. |
| EKF sur l'odométrie bruitée | L'EKF ne reçoit rien | `odom0` pointait sur `odom_noisy`, un topic d'exercice, et sur le préfixe `bumperbot_`. |
| `slam.launch.py` | Le lancement plante | `{"free_thresh_default", "0.196"}` crée un ensemble Python, pas un dictionnaire : virgules au lieu de deux-points. |

### Capteurs

| Problème | Symptôme | Cause et correction |
|---|---|---|
| Mauvais bus I2C | MPU6050 introuvable, node silencieux | Le code ouvrait le bus 1 (convention Raspberry Pi) ; les broches 3/5 de l'Orin Nano sont sur le bus 7. Bus devenu paramétrable, et un avertissement est désormais affiché en cas d'échec. |
| Gyroscope 8 fois trop faible | Rotations sous-estimées | `GYRO_CONFIG = 24` règle la plage sur ±2000 °/s, mais la conversion utilisait le facteur de ±250 °/s. Plage et facteur sont maintenant liés dans le code. |
| Biais du gyroscope | Environ 0,18 rad/s au repos, l'EKF croit que le robot tourne | Mesure du biais sur 200 échantillons au démarrage, robot immobile, puis soustraction. |
| Covariances nulles | L'EKF prend les mesures pour parfaites | Variances réalistes, et `orientation_covariance[0] = -1` (convention REP-145) pour signaler que le capteur ne fournit pas d'orientation. |

### Méthode

| Problème | Symptôme | Leçon |
|---|---|---|
| Fichiers édités en double | Les corrections semblent sans effet | L'IDE Arduino ouvrait une copie du croquis située hors du workspace. Vérifier avec *Sketch → Show Sketch Folder*. |
| Éditeur qui écrase les fichiers | Une correction disparaît après un « Save All » | Un onglet resté ouvert réécrit son ancien contenu sur le disque. Fermer l'onglet sans enregistrer avant de modifier un fichier à l'extérieur. |
| Simulation et robot réel en même temps | Deux `controller_manager` du même nom sur le réseau | Les deux machines partagent le même réseau ROS : arrêter la simulation avant de lancer le robot. |

## Banc de test

| Croquis Arduino | Usage |
|---|---|
| `firmware/robot_control/` | Firmware de production : réception des consignes, PID, renvoi des vitesses |
| `firmware/pid_tuning/` | Réglage des PID, piloté par `tools/pid_tuner.py` |
| `firmware/two_wheels_forward_test/` | Les deux roues en avant à PWM fixe, sans PID : vérification du câblage |
| `firmware/simple_encoder_reader/` | Lecture d'un encodeur seul |
| `firmware/simple_motor_control/` | Commande d'un moteur seul depuis le port série |

## État d'avancement

- [x] Simulation Gazebo : contrôle, odométrie, localisation AMCL, SLAM
- [x] Robot réel : hardware interface, PID de vitesse réglés, téléopération
- [x] IMU MPU6050 sur la Jetson, avec calibration du biais du gyroscope
- [x] Fusion odométrie des roues + IMU par EKF, source unique de la pose
- [ ] Calibration de l'odométrie (vérifier qu'un mètre annoncé est un mètre parcouru)
- [ ] Limitation de l'accélération pour éviter le patinage sur sol lisse
- [ ] Lidar et SLAM sur le robot réel
- [ ] Navigation autonome (Nav2)
