#define L298N_enA 9
#define L298N_in1 12
#define L298N_in2 13
#define right_encoder_phaseA 3
#define right_encoder_phaseB 5
#define L298N_enB 7
#define L298N_in3 8
#define L298N_in4 11
#define left_encoder_phaseA 2
#define left_encoder_phaseB 4

double cmd = 0;
volatile unsigned int right_encoder_counter = 0;
volatile char right_encoder_sign = 'p';
double right_encoder_meas_vel = 0.0;
volatile unsigned int left_encoder_counter = 0;
volatile char left_encoder_sign = 'p';
double left_encoder_meas_vel = 0.0;


unsigned long last_millis = 0;
const unsigned long interval = 100;   // ms

void setup() {
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);
  pinMode(right_encoder_phaseA, INPUT);
  pinMode(right_encoder_phaseB, INPUT);

  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);

  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  Serial.begin(115200);
}

void loop() {
  unsigned long now = millis();
  if (now - last_millis >= interval) {
    last_millis = now;

    // Copie + remise à zéro sans être interrompu
    noInterrupts();
    unsigned int count = right_encoder_counter;
    char sign = right_encoder_sign;
    right_encoder_counter = 0;
    interrupts();

    right_encoder_meas_vel = 10 * count * (60 / 385.0) * 0.10472;   // rad/s

    Serial.print(sign);
    Serial.println(right_encoder_meas_vel);
  }

  analogWrite(L298N_enA, 100);
}

void rightEncoderCallback() {
  right_encoder_counter++;
  right_encoder_sign = (digitalRead(right_encoder_phaseB) == HIGH) ? 'p' : 'n';
}