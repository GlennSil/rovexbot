// Test : fait tourner les deux roues en avant à PWM fixe
// et envoie les vitesses mesurées par les encodeurs sur le port série.
// Format envoyé toutes les 100 ms : "rp4.73,lp4.60,"

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
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Sens "avant" pour les deux roues
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  pinMode(right_encoder_phaseA, INPUT);
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseA, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);

  Serial.begin(115200);

  analogWrite(L298N_enA, TEST_PWM);
  analogWrite(L298N_enB, TEST_PWM);
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
