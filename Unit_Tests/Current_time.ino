#include <WiFi.h>
#include "time.h"

const char* ssid = "Nav";
const char* password = "password";

// NTP server details
const char* ntpServer = "pool.ntp.org"; 
const long gmtOffset_sec = 0;           // Offset in seconds for GMT (e.g., 3600 for GMT+1)
const int daylightOffset_sec = 0;       // Offset in seconds for daylight saving time

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Initialize time using NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Time synchronized using NTP");
}

void loop() {
  // Get the current time
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    delay(2000);
    return;
  }

  // Print the current time
  //Serial.print("Current time: ");
  //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S"); // Day, Month Date Year HH:MM:SS
  Serial.println(&timeinfo, "%H:%M:%S"); // Day, Month Date Year HH:MM:SS
  delay(1000);
}
