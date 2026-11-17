// Test : fait tourner les deux roues en avant à PWM fixe
// et envoie les vitesses mesurées par les encodeurs sur le port série.
// Format envoyé toutes les 100 ms : "rp4.73,lp4.60,"

// ---------- L298N ----------
// Câblage réel du robot (vérifié avec l'outil de réglage des PID) :
// Roue gauche : canal A du L298N
#define LEFT_PWM 9     // ENA, fil blanc
#define LEFT_IN1 12    // IN1, fil blanc pâle
#define LEFT_IN2 13    // IN2, fil violet
// Roue droite : canal B du L298N
#define RIGHT_PWM 11   // ENB, fil jaune
#define RIGHT_IN1 7    // IN3
#define RIGHT_IN2 8    // IN4

// ---------- Encodeurs ----------
#define right_encoder_phaseA 3   // Interruption
#define right_encoder_phaseB 5
#define left_encoder_phaseA 2    // Interruption
#define left_encoder_phaseB 4

const int TEST_PWM = 100;               // 0-255 : vitesse du test
const double ENCODER_PPR = 385.0;       // fronts montants de la phase A par tour de roue
const unsigned long interval = 100;     // période de mesure (ms)
const double TICKS_TO_RAD_S = (1000.0 / interval) * (2.0 * PI / ENCODER_PPR);

volatile unsigned int right_encoder_counter = 0;
volatile unsigned int left_encoder_counter = 0;
volatile char right_wheel_sign = 'p';
volatile char left_wheel_sign = 'p';

unsigned long last_millis = 0;


void setup() {
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  // Sens "avant" pour les deux roues.
  // Les moteurs sont câblés en sens inverse de la marche avant du robot,
  // d'où IN1 = LOW et IN2 = HIGH (voir RIGHT_MOTOR_INVERT dans robot_control.ino).
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);

  pinMode(right_encoder_phaseA, INPUT);
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseA, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);

  Serial.begin(115200);

  analogWrite(RIGHT_PWM, TEST_PWM);
  analogWrite(LEFT_PWM, TEST_PWM);
}


void loop() {
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    last_millis = current_millis;

    noInterrupts();
    unsigned int right_count = right_encoder_counter;
    unsigned int left_count = left_encoder_counter;
    char right_sign = right_wheel_sign;
    char left_sign = left_wheel_sign;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    interrupts();

    Serial.print('r');
    Serial.print(right_sign);
    Serial.print(right_count * TICKS_TO_RAD_S, 2);
    Serial.print(",l");
    Serial.print(left_sign);
    Serial.print(left_count * TICKS_TO_RAD_S, 2);
    Serial.println(',');
  }
}


void rightEncoderCallback() {
  right_wheel_sign = (digitalRead(right_encoder_phaseB) == HIGH) ? 'p' : 'n';
  right_encoder_counter++;
}

// Moteur gauche monté en miroir -> signe inversé
void leftEncoderCallback() {
  left_wheel_sign = (digitalRead(left_encoder_phaseB) == HIGH) ? 'n' : 'p';
  left_encoder_counter++;
}
