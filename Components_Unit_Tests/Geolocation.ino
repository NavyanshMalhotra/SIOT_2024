#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "Nav";         
const char* password = "password"; 

// Replace with your Google API key
const char* apiKey = "API KEY REMOVED";    

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
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Google Geolocation API endpoint
    String url = "https://www.googleapis.com/geolocation/v1/geolocate?key=" + String(apiKey);

    // Build the Wi-Fi access point JSON object
    String requestBody = buildWiFiRequestBody();
    Serial.println("Request Body: " + requestBody);

    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.POST(requestBody);

    if (httpCode > 0) {
      String payload = http.getString();
      Serial.println("Response: " + payload);

      // Parse JSON response
      DynamicJsonDocument doc(2048);
      deserializeJson(doc, payload);

      double lat = doc["location"]["lat"];
      double lng = doc["location"]["lng"];
      float accuracy = doc["accuracy"];

      // Print the results with high precision
      Serial.print("Latitude: ");
      Serial.println(lat, 8);  
      Serial.print("Longitude: ");
      Serial.println(lng, 8); 
      Serial.print("Accuracy (meters): ");
      Serial.println(accuracy, 2); 
    } else {
      Serial.print("Error on HTTP request: ");
      Serial.println(http.errorToString(httpCode));
    }

    http.end();
  } else {
    Serial.println("Wi-Fi disconnected!");
  }

  delay(10000); // Query every 10 seconds
}

String buildWiFiRequestBody() {
  String requestBody = "{\"wifiAccessPoints\":[";

  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    if (i > 0) requestBody += ",";
    requestBody += "{";
    requestBody += "\"macAddress\":\"" + WiFi.BSSIDstr(i) + "\",";
    requestBody += "\"signalStrength\":" + String(WiFi.RSSI(i)) + "";
    requestBody += "}";
  }

  requestBody += "]}";
  return requestBody;
}
