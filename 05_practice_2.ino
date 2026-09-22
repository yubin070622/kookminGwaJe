#define LED_PIN 7

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, 0);
  delay(1000);

  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, 1);
    delay(100);
    digitalWrite(LED_PIN, 0);
    delay(100);
  }

  digitalWrite(LED_PIN, 1);
  while (1) {}
}
