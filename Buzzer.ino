#define BUZZER_PIN 11 

void setup() {
  pinMode(BUZZER_PIN, OUTPUT); // Set the buzzer pin as output
}

void loop() {
  // Play a frequency for 500ms
  tone(BUZZER_PIN, 1000); // Play a 1kHz tone
  delay(500);             // Wait for 500ms

  noTone(BUZZER_PIN);     // Stop the tone
  delay(500);             // Wait for 500ms

  // Play another frequency for 500ms
  tone(BUZZER_PIN, 1500); // Play a 1.5kHz tone
  delay(500);

  noTone(BUZZER_PIN);     // Stop the tone
  delay(500);
}
