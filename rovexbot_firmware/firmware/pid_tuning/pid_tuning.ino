// Réglage des PID de vitesse, une roue à la fois, sans retéléverser.
//
// Moniteur série : 115200 bauds, fin de ligne "Nouvelle ligne".
// Commandes :
//   w r | w l | w b   roue sélectionnée : droite, gauche, les deux
//   s 5               consigne de vitesse (rad/s) pour la sélection
//   p 10              Kp pour la sélection
//   i 5               Ki pour la sélection
//   d 0               Kd pour la sélection
//   x                 arrêt des deux roues
//   ?                 affiche les gains actuels
//
// Sortie toutes les 100 ms, lisible dans le Traceur série :
//   consigne_d, mesure_d, consigne_g, mesure_g (rad/s), pwm_d_sur10, pwm_g_sur10 (PWM / 10)

#include <PID_v1.h>

// ---------- L298N ----------
// Câblage réel du robot (vérifié avec l'outil de réglage des PID) :
// Roue gauche : canal A du L298N
#define LEFT_PWM 9     // ENA
#define LEFT_IN1 12    // IN1
#define LEFT_IN2 13    // IN2
// Roue droite : canal B du L298N
#define RIGHT_PWM 11   // ENB
#define RIGHT_IN1 7    // IN3
#define RIGHT_IN2 8    // IN4

// ---------- Encodeurs ----------
#define right_encoder_phaseA 3
#define right_encoder_phaseB 5
#define left_encoder_phaseA 2
#define left_encoder_phaseB 4

const double ENCODER_PPR = 385.0;
const unsigned long interval = 100;   // ms
const double TICKS_TO_RAD_S = (1000.0 / interval) * (2.0 * PI / ENCODER_PPR);

volatile unsigned int right_encoder_counter = 0;
volatile unsigned int left_encoder_counter = 0;

unsigned long last_millis = 0;

// Indice 0 = droite, 1 = gauche
double setpoint[2] = {0.0, 0.0};   // rad/s
double measure[2]  = {0.0, 0.0};   // rad/s
double output[2]   = {0.0, 0.0};   // PWM 0-255
double kp[2] = {11.5, 12.8};
double ki[2] = {7.5, 8.3};
double kd[2] = {0.1, 0.1};

PID pidRight(&measure[0], &output[0], &setpoint[0], kp[0], ki[0], kd[0], DIRECT);
PID pidLeft(&measure[1], &output[1], &setpoint[1], kp[1], ki[1], kd[1], DIRECT);
PID* pid[2] = {&pidRight, &pidLeft};

bool selected[2] = {true, false};   // roue droite sélectionnée au départ

char buffer[20];
uint8_t buffer_idx = 0;


void printGains() {
  for (uint8_t w = 0; w < 2; w++) {
    Serial.print(w == 0 ? F("# droite : Kp=") : F("# gauche : Kp="));
    Serial.print(kp[w]);
    Serial.print(F(" Ki="));
    Serial.print(ki[w]);
    Serial.print(F(" Kd="));
    Serial.println(kd[w]);
  }
}

// Consigne nulle -> PID en manuel et sortie à 0 : l'intégrale repart de zéro au prochain essai
void applySetpoint(uint8_t w, double value) {
  setpoint[w] = value;
  if (value <= 0.0) {
    setpoint[w] = 0.0;
    pid[w]->SetMode(MANUAL);
    output[w] = 0.0;
  } else {
    pid[w]->SetMode(AUTOMATIC);
  }
}

void processCommand() {
  char cmd = buffer[0];
  double value = atof(buffer + 1);

  if (cmd == 'w') {
    char* c = buffer + 1;
    while (*c == ' ') c++;
    selected[0] = (*c == 'r' || *c == 'b');
    selected[1] = (*c == 'l' || *c == 'b');
    Serial.print(F("# selection :"));
    if (selected[0]) Serial.print(F(" droite"));
    if (selected[1]) Serial.print(F(" gauche"));
    Serial.println();
  }
  else if (cmd == 'x') {
    applySetpoint(0, 0.0);
    applySetpoint(1, 0.0);
  }
  else if (cmd == '?') {
    printGains();
  }
  else {
    for (uint8_t w = 0; w < 2; w++) {
      if (!selected[w]) continue;
      if (cmd == 's') applySetpoint(w, value);
      else if (cmd == 'p') kp[w] = value;
      else if (cmd == 'i') ki[w] = value;
      else if (cmd == 'd') kd[w] = value;
      pid[w]->SetTunings(kp[w], ki[w], kd[w]);
    }
    // Confirmation : affiche les gains après chaque changement
    if (cmd == 'p' || cmd == 'i' || cmd == 'd') printGains();
  }
}


void setup() {
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  // Sens avant pour les deux roues
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);

  for (uint8_t w = 0; w < 2; w++) {
    pid[w]->SetSampleTime(interval);
    pid[w]->SetOutputLimits(0, 255);
    pid[w]->SetMode(MANUAL);
  }

  pinMode(right_encoder_phaseA, INPUT);
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseA, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);

  Serial.begin(115200);
}


void loop() {
  // ---------- Lecture des commandes, ligne par ligne ----------
  while (Serial.available()) {
    char chr = Serial.read();
    if (chr == '\n' || chr == '\r') {
      if (buffer_idx > 0) {
        buffer[buffer_idx] = '\0';
        processCommand();
        buffer_idx = 0;
      }
    } else if (buffer_idx < sizeof(buffer) - 1) {
      buffer[buffer_idx++] = chr;
    }
  }

  // ---------- Mesure + PID toutes les 100 ms ----------
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    last_millis = current_millis;

    noInterrupts();
    unsigned int right_count = right_encoder_counter;
    unsigned int left_count = left_encoder_counter;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    interrupts();

    measure[0] = right_count * TICKS_TO_RAD_S;
    measure[1] = left_count * TICKS_TO_RAD_S;

    pidRight.Compute();
    pidLeft.Compute();

    analogWrite(RIGHT_PWM, (int)output[0]);
    analogWrite(LEFT_PWM, (int)output[1]);

    Serial.print(F("consigne_d:"));    Serial.print(setpoint[0]);
    Serial.print(F(",mesure_d:"));     Serial.print(measure[0]);
    Serial.print(F(",consigne_g:"));   Serial.print(setpoint[1]);
    Serial.print(F(",mesure_g:"));     Serial.print(measure[1]);
    Serial.print(F(",pwm_d_sur10:"));  Serial.print(output[0] / 10.0);
    Serial.print(F(",pwm_g_sur10:"));  Serial.println(output[1] / 10.0);
  }
}


void rightEncoderCallback() {
  right_encoder_counter++;
}

void leftEncoderCallback() {
  left_encoder_counter++;
}
