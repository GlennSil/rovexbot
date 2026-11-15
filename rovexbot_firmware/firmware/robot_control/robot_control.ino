#include <PID_v1.h>

// ---------- L298N ----------
#define L298N_enA 9    // PWM roue droite
#define L298N_in1 12   // Sens roue droite
#define L298N_in2 13   // Sens roue droite
#define L298N_enB 11   // PWM roue gauche
#define L298N_in3 7    // Sens roue gauche
#define L298N_in4 8    // Sens roue gauche

// ---------- Encodeurs ----------
#define right_encoder_phaseA 3   // Interruption
#define right_encoder_phaseB 5
#define left_encoder_phaseA 2    // Interruption
#define left_encoder_phaseB 4

const double ENCODER_PPR = 385.0;       // fronts montants de la phase A par tour de roue
const unsigned long interval = 100;     // période de mesure et de PID (ms)
const double TICKS_TO_RAD_S = (1000.0 / interval) * (2.0 * PI / ENCODER_PPR);

// Variables partagées avec les interruptions -> volatile
volatile unsigned int right_encoder_counter = 0;
volatile unsigned int left_encoder_counter = 0;
volatile char right_wheel_sign = 'p';   // 'p' = positif, 'n' = négatif
volatile char left_wheel_sign = 'p';

unsigned long last_millis = 0;

// ---------- Lecture du protocole série ----------
// Format reçu depuis RovexbotInterface::write() : "rp05.00,ln03.20,"
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
char value[] = "00.00";
uint8_t value_idx = 0;

// ---------- PID ----------
// Consigne (rad/s, valeur absolue ; le sens est géré par in1..in4)
double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;
// Mesure (rad/s, valeur absolue)
double right_wheel_meas_vel = 0.0;
double left_wheel_meas_vel = 0.0;
// Sortie (PWM 0-255)
double right_wheel_cmd = 0.0;
double left_wheel_cmd = 0.0;
// Réglages
double Kp_r = 11.5, Ki_r = 7.5, Kd_r = 0.1;
double Kp_l = 12.8, Ki_l = 8.3, Kd_l = 0.1;

PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);


void setRightDirection(bool forward) {
  digitalWrite(L298N_in1, forward ? HIGH : LOW);
  digitalWrite(L298N_in2, forward ? LOW : HIGH);
}

void setLeftDirection(bool forward) {
  digitalWrite(L298N_in3, forward ? HIGH : LOW);
  digitalWrite(L298N_in4, forward ? LOW : HIGH);
}

void resetValue() {
  value_idx = 0;
  strcpy(value, "00.00");
}


void setup() {
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  setRightDirection(true);
  setLeftDirection(true);

  rightMotor.SetSampleTime(interval);
  leftMotor.SetSampleTime(interval);
  rightMotor.SetOutputLimits(0, 255);
  leftMotor.SetOutputLimits(0, 255);
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  pinMode(right_encoder_phaseA, INPUT);
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseA, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);

  Serial.begin(115200);
}


void loop() {
  // ---------- 1. Lecture des commandes (un caractère par tour) ----------
  if (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'r') {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      resetValue();
    }
    else if (chr == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      resetValue();
    }
    else if (chr == 'p' || chr == 'n') {
      bool forward = (chr == 'p');
      if (is_right_wheel_cmd) setRightDirection(forward);
      else if (is_left_wheel_cmd) setLeftDirection(forward);
    }
    else if (chr == ',') {
      if (is_right_wheel_cmd) right_wheel_cmd_vel = atof(value);
      else if (is_left_wheel_cmd) left_wheel_cmd_vel = atof(value);
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = false;
      resetValue();
    }
    else if ((chr >= '0' && chr <= '9') || chr == '.') {
      if (value_idx < 5) {
        value[value_idx++] = chr;
      }
    }
    // Tout autre caractère (\r, \n...) est ignoré
  }

  // ---------- 2. Mesure + PID toutes les 100 ms ----------
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    last_millis = current_millis;

    // Copie + remise à zéro sans être interrompu
    noInterrupts();
    unsigned int right_count = right_encoder_counter;
    unsigned int left_count = left_encoder_counter;
    char right_sign = right_wheel_sign;
    char left_sign = left_wheel_sign;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    interrupts();

    right_wheel_meas_vel = right_count * TICKS_TO_RAD_S;
    left_wheel_meas_vel = left_count * TICKS_TO_RAD_S;

    rightMotor.Compute();
    leftMotor.Compute();

    // Consigne nulle -> moteur coupé (évite le grésillement à l'arrêt)
    if (right_wheel_cmd_vel == 0.0) right_wheel_cmd = 0.0;
    if (left_wheel_cmd_vel == 0.0) left_wheel_cmd = 0.0;

    analogWrite(L298N_enA, (int)right_wheel_cmd);
    analogWrite(L298N_enB, (int)left_wheel_cmd);

    // ---------- 3. Envoi des vitesses mesurées ----------
    // Format lu par RovexbotInterface::read() : "rp4.73,ln2.10,"
    Serial.print('r');
    Serial.print(right_sign);
    Serial.print(right_wheel_meas_vel, 2);
    Serial.print(",l");
    Serial.print(left_sign);
    Serial.print(left_wheel_meas_vel, 2);
    Serial.println(',');
  }
}


// ---------- Interruptions encodeurs ----------
void rightEncoderCallback() {
  right_wheel_sign = (digitalRead(right_encoder_phaseB) == HIGH) ? 'p' : 'n';
  right_encoder_counter++;
}

// Moteur gauche monté en miroir -> signe inversé
void leftEncoderCallback() {
  left_wheel_sign = (digitalRead(left_encoder_phaseB) == HIGH) ? 'n' : 'p';
  left_encoder_counter++;
}
