#define BATTERY_PIN 1 // ADC pin connected to the voltage divider
#define R1 15000       // Resistor R1 value in ohms
#define R2 4700        // Resistor R2 value in ohms

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // ESP32 ADC has 12-bit resolution
}

void loop() {
  // Read the raw ADC value
  int adcValue = analogRead(BATTERY_PIN);

  // Convert ADC value to voltage (scaled for 3.3V reference)
  float scaledVoltage = (adcValue / 4095.0) * 3.3;

  // Calculate the actual battery voltage using the voltage divider
  float batteryVoltage = scaledVoltage * (R1 + R2) / R2;

  // Print the battery voltage
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage);
  Serial.println("V");

  delay(1000); // Update every second
}
