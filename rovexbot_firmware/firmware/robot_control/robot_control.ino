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

// Sens de rotation : true si la roue tourne à l'envers de ce que demande ROS.
// Les deux moteurs sont câblés en sens inverse de la marche avant du robot.
const bool RIGHT_MOTOR_INVERT = true;
const bool LEFT_MOTOR_INVERT  = true;

// ---------- Encodeurs ----------
#define right_encoder_phaseA 3   // Interruption
#define right_encoder_phaseB 5   // non utilisée : le sens vient de la commande
#define left_encoder_phaseA 2    // Interruption
#define left_encoder_phaseB 4

const double ENCODER_PPR = 385.0;       // fronts montants de la phase A par tour de roue
const unsigned long interval = 100;     // période de mesure et de PID (ms)
const double TICKS_TO_RAD_S = (1000.0 / interval) * (2.0 * PI / ENCODER_PPR);

// Variables partagées avec les interruptions -> volatile
volatile unsigned int right_encoder_counter = 0;
volatile unsigned int left_encoder_counter = 0;

// Sens demandé par ROS, utilisé pour signer la vitesse renvoyée.
// La phase B n'est pas assez fiable pour ça : lue une seule fois par front
// montant de la phase A, elle donne un sens faux dès que la roue vibre ou
// change de sens, ce qui rend l'odométrie incohérente.
bool right_wheel_forward = true;
bool left_wheel_forward = true;

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
// Gains relevés avec tools/pid_tuner.py (robot surélevé, 2 / 5 / 10 rad/s)
double Kp_r = 16.5, Ki_r = 16.0, Kd_r = 0.0;
double Kp_l = 16.0, Ki_l = 16.0, Kd_l = 0.0;

PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);


void setRightDirection(bool forward) {
  right_wheel_forward = forward;
  bool level = (forward != RIGHT_MOTOR_INVERT);   // inverse le sens si nécessaire
  digitalWrite(RIGHT_IN1, level ? HIGH : LOW);
  digitalWrite(RIGHT_IN2, level ? LOW : HIGH);
}

void setLeftDirection(bool forward) {
  left_wheel_forward = forward;
  bool level = (forward != LEFT_MOTOR_INVERT);
  digitalWrite(LEFT_IN1, level ? HIGH : LOW);
  digitalWrite(LEFT_IN2, level ? LOW : HIGH);
}

void resetValue() {
  value_idx = 0;
  strcpy(value, "00.00");
}


void setup() {
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

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
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    interrupts();

    // L'encodeur donne la vitesse, le sens vient de la commande
    char right_sign = right_wheel_forward ? 'p' : 'n';
    char left_sign = left_wheel_forward ? 'p' : 'n';

    right_wheel_meas_vel = right_count * TICKS_TO_RAD_S;
    left_wheel_meas_vel = left_count * TICKS_TO_RAD_S;

    // Consigne nulle -> moteur coupé et PID en manuel : l'intégrale repart de zéro au prochain départ
    if (right_wheel_cmd_vel == 0.0) right_wheel_cmd = 0.0;
    if (left_wheel_cmd_vel == 0.0) left_wheel_cmd = 0.0;
    rightMotor.SetMode(right_wheel_cmd_vel == 0.0 ? MANUAL : AUTOMATIC);
    leftMotor.SetMode(left_wheel_cmd_vel == 0.0 ? MANUAL : AUTOMATIC);

    rightMotor.Compute();
    leftMotor.Compute();

    analogWrite(RIGHT_PWM, (int)right_wheel_cmd);
    analogWrite(LEFT_PWM, (int)left_wheel_cmd);

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
// On compte seulement les impulsions : le sens vient de la commande (voir plus haut)
void rightEncoderCallback() {
  right_encoder_counter++;
}

void leftEncoderCallback() {
  left_encoder_counter++;
}
