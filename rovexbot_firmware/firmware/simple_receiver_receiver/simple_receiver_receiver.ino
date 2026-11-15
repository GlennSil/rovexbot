#define LED_PIN 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();                    // enlève \r, espaces
    if (msg.length() == 0) return; // ignore les lignes vides

    if (msg == "0") {
      digitalWrite(LED_PIN, LOW);
    } else if (msg == "1") {
      digitalWrite(LED_PIN, HIGH);
    }
    // tout autre message est ignoré
  }
}