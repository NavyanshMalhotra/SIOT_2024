#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_GFX.h>
#include <Adafruit_DotStarMatrix.h>
#include <Adafruit_DotStar.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "time.h"

#define BUZZER_PIN_LEFT 12 
#define BUZZER_PIN_RIGHT 22
#define DATAPIN 10
#define CLOCKPIN 11
#define PIR_PIN_LEFT 3  
#define PIR_PIN_RIGHT 2

// Wi-Fi credentials
const char* ssid = "Nav";
const char* password = "password";

// NTP server details
const char* ntpServer = "pool.ntp.org"; 
const long gmtOffset_sec = 0;           // GMT offset
const int daylightOffset_sec = 0;       // Daylight saving offset

// Thinger.io settings
#define HOST "backend.thinger.io"
#define PORT 80  
#define AUTH_TOKEN "Bearer <Thinger Token Removed>" 

// Geolocation API details
const char* apiKey = "AIzaSyCntEmrdrHb5rv4k_iPllArjv7CliwJtpM";    
float latitude = 0.0;
float longitude = 0.0;

// Adafruit DotStar Matrix
Adafruit_DotStarMatrix matrix = Adafruit_DotStarMatrix(
  8, 8, DATAPIN, CLOCKPIN,
  DS_MATRIX_TOP + DS_MATRIX_RIGHT + DS_MATRIX_COLUMNS + DS_MATRIX_PROGRESSIVE,
  DOTSTAR_BRG);

const uint16_t yellow_arrow = matrix.Color(125, 125, 0); // Dim Yellow


MPU6050 mpu;
bool dmpReady = false; 
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float ypr_filtered[3];
float alpha = 0.0;
unsigned long prevTime = 0;
float deltaTime = 0.02;
int imu_data[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
const float confidence_rate = 0.65f;

// PIR Sensor Setup
int valR = 0;       
int valL = 0;          
int sensor = 3;    
int state = LOW;       
int val = 0;          

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Initialize NTP Time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Initialize PIR sensor
  pinMode(PIR_PIN_LEFT, INPUT);
  pinMode(PIR_PIN_RIGHT, INPUT);
  pinMode(sensor, INPUT); 

 
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected");
  } else {
    Serial.println("MPU6050 connection failed");
  }


  alpha = deltaTime / (deltaTime + (1.0 / 0.75)); // Low-pass filter setup

  matrix.begin();
  matrix.setBrightness(40);


  getGeolocation();
}

void loop() {
  static int temp_arr[3] = {0};

  getGeolocation();

  // Get time and check weather/time condition
  if (isSafeToDisplay()) {
    matrix.fillScreen(0);
    matrix.fillScreen(yellow_arrow); // Show dim yellow on conditions
  }

  // Read PIR sensor for motion detection
  valL = digitalRead(PIR_PIN_LEFT);
  if (valL == HIGH) {
    playBuzzer(BUZZER_PIN_LEFT); 
  }

  valR = digitalRead(PIR_PIN_RIGHT);
  if (valR == HIGH) {
    playBuzzer(BUZZER_PIN_RIGHT);
  }

  // Check PIR sensor on right side (pin 3)
  val = digitalRead(sensor);  // Read PIR sensor
  if (val == HIGH) {  
    playBuzzer(BUZZER_PIN_RIGHT); 
  }

  displayArrow(detectTurn());

  if (detectCrash()) {
    sendTelegramMessage("Crash detected! Location: " + String(latitude, 8) + ", " + String(longitude, 8));
  }

  uploadToThingerIO("Data uploaded successfully");

  delay(1000);
}

bool isSafeToDisplay() {
  bool isRaining = false;  
  bool isDayTime = false; 
  
  return isRaining || !isDayTime;
}

void displayArrow(bool isLeftTurn) {
  matrix.fillScreen(0);
  if (isLeftTurn) {
    matrix.drawBitmap(0, 0, leftArrow, 8, 8, yellow_arrow); // Left Arrow
  } else {
    matrix.drawBitmap(0, 0, rightArrow, 8, 8, yellow_arrow); // Right Arrow
  }
  matrix.show();
}

bool detectTurn() {
  calculateInference(); 
  return getTurnDirection(); 
}

bool detectCrash() {
  float acceleration = 3.0; 
  return acceleration > 2.0;
}

void sendTelegramMessage(String message) {
  String telegramApiUrl = "https://api.telegram.org/bot<Bot Token removed>/sendMessage?chat_id=<Chat ID removed>&text=" + message;
  
  HTTPClient http;
  http.begin(telegramApiUrl);
  int httpResponseCode = http.GET();
  
  if (httpResponseCode == 200) {
    Serial.println("Telegram message sent successfully!");
  } else {
    Serial.println("Error sending Telegram message");
  }
  http.end();
}

void uploadToThingerIO(String message) {
  HTTPClient http;
  http.begin("http://backend.thinger.io/v3/users/NavM/devices/xiaosensorremote/callback/data");
  http.addHeader("Authorization", AUTH_TOKEN);
  http.addHeader("Content-Type", "application/json");

  String payload = "{\"lat\":" + String(latitude, 8) + ", \"lng\":" + String(longitude, 8) + ", \"turn\":\"" + (detectTurn() ? "left" : "right") + "\", \"motion\": \"" + (val == HIGH ? "detected" : "none") + "\"}";

  int httpResponseCode = http.POST(payload);

  http.end();
}

void playBuzzer(int buzzerPin) {
  tone(buzzerPin, 1000); 
  delay(500);          
  noTone(buzzerPin);   
  delay(500);             
}

void calculateInference() {
  ei_impulse_result_t result = {0};

  signal_t features_signal;
  features_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  features_signal.get_data = &raw_feature_get_data;

  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
  if (res != EI_IMPULSE_OK) {
    Serial.printf("ERR: Failed to run classifier (%d)\n", res);
    return;
  }

  print_inference_result(result);
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

void print_inference_result(ei_impulse_result_t result) {
  float max_val = 0.0f;
  int max_index = -1;

  Serial.println("Inference results:");
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.printf("%s: %.2f\n", result.classification[i].label, result.classification[i].value);
    if (result.classification[i].value > max_val) {
      max_val = result.classification[i].value;
      max_index = i;
    }
  }

  if (max_val > confidence_rate) {
    Serial.printf("Detected: %s (confidence: %.2f)\n", result.classification[max_index].label, max_val);
  }
}


void getGeolocation() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String url = "https://www.googleapis.com/geolocation/v1/geolocate?key=" + String(apiKey);

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

      latitude = doc["location"]["lat"];
      longitude = doc["location"]["lng"];
      float accuracy = doc["accuracy"];

      Serial.print("Latitude: ");
      Serial.println(latitude, 8);  
      Serial.print("Longitude: ");
      Serial.println(longitude, 8); 
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
