#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "time.h"

const char* ssid = "Nav";
const char* password = "password";

float latitude = 51.488625; 
float longitude = -0.204775; 

String weatherCondition;

// NTP Server
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;           
const int daylightOffset_sec = 0;

String sunriseTime, sunsetTime;

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

  // Initialize NTP for current time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Fetch sunrise and sunset times
  getSunriseSunset();

  // Fetch current weather conditions
  getWeather();
}

void loop() {
  // Get current time
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    delay(2000);
    return;
  }

  // Print current time
  char currentTime[9];
  strftime(currentTime, 9, "%H:%M:%S", &timeinfo);
  Serial.print("Current time: ");
  Serial.println(currentTime);

  // Check if it's dark and print the result
  if (isDarkOutside(currentTime)) {
    if (weatherCondition == "rain" || weatherCondition == "showers") {
      Serial.println("It's dark outside because it's raining.");
    } else {
      Serial.println("It's dark outside.");
    }
  } else {
    Serial.println("It's light outside.");
  }

  delay(60000); // Check every minute
}

void getSunriseSunset() {
  HTTPClient http;

  String url = "https://api.sunrise-sunset.org/json?lat=" + String(latitude, 6) + "&lng=" + String(longitude, 6) + "&formatted=0";

  Serial.println("Requesting sunrise and sunset times...");
  http.begin(url);

  int httpCode = http.GET();
  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println("Response: " + payload);

    // Parse JSON response
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    sunriseTime = doc["results"]["sunrise"].as<String>();
    sunsetTime = doc["results"]["sunset"].as<String>();

    Serial.print("Sunrise (UTC): ");
    Serial.println(sunriseTime);
    Serial.print("Sunset (UTC): ");
    Serial.println(sunsetTime);
  } else {
    Serial.print("HTTP request failed: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

void getWeather() {
  HTTPClient http;

  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(latitude, 6) + "&longitude=" + String(longitude, 6) + "&current_weather=true";

  Serial.println("Requesting weather data...");
  http.begin(url);

  int httpCode = http.GET();
  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println("Weather Response: " + payload);

    // Parse JSON response
    DynamicJsonDocument doc(2048);
    deserializeJson(doc, payload);

    // Extract current weather condition
    weatherCondition = doc["current_weather"]["weathercode"].as<String>();

    // Convert weather code to human-readable condition
    if (weatherCondition == "61" || weatherCondition == "63" || weatherCondition == "65") {
      weatherCondition = "rain";
    } else {
      weatherCondition = "clear";
    }

    Serial.print("Current weather: ");
    Serial.println(weatherCondition);
  } else {
    Serial.print("Weather request failed: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

bool isDarkOutside(String currentTime) {
  struct tm sunriseTm, sunsetTm, currentTm;

  // Parse sunrise and sunset times into tm structs
  strptime(sunriseTime.c_str(), "%Y-%m-%dT%H:%M:%S", &sunriseTm);
  strptime(sunsetTime.c_str(), "%Y-%m-%dT%H:%M:%S", &sunsetTm);

  // Get current time as a tm struct
  struct tm timeinfo;
  getLocalTime(&timeinfo);

  // Compare current time with sunrise and sunset
  return difftime(mktime(&timeinfo), mktime(&sunriseTm)) < 0 || difftime(mktime(&timeinfo), mktime(&sunsetTm)) > 0;
}
