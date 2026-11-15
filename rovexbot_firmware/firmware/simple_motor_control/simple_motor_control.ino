#define L298N_enA 9
#define L298N_in1 12
#define L298N_in2 13

double cmd = 0;   // globale : garde sa valeur entre deux tours de loop()

void setup() {
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n').toDouble();
  }
  int pwm = constrain(cmd * 100, 0, 255);
  analogWrite(L298N_enA, pwm);
}