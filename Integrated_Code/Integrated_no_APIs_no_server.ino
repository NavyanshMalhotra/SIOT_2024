/* 
To run code:
- Update SSID credentials and 
- Generate and update API keys. Search "Update tokens" across code 
- Conenct sensor and actuator pins as defined below 
- Use 4.7V (3.7-5V) battery to Vin pin on ESP32 
*/

// Include necessary libraries
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SIOT_helmet_motion_inferencing.h>
#include <Adafruit_GFX.h>
#include <Adafruit_DotStarMatrix.h>
#include <Adafruit_DotStar.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "time.h"

// Pin declarations
#define IMU_SDA 21
#define IMU_SCL 18
#define BUZZER_PIN_LEFT 12
#define BUZZER_PIN_RIGHT 22
#define DATAPIN 10
#define CLOCKPIN 11
#define PIR_PIN_LEFT 3
#define PIR_PIN_RIGHT 2
#define BATTERY_V_READ 4

// Voltage divider circuit to read battery volatgae as an indicator for battery capacity
#define R1 15000  // Resistor R1 value in ohms
#define R2 4700   // Resistor R2 value in ohms

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Sensor variables and states
float battery_voltage = 0;
bool left_PIR = 0;
bool right_PIR = 0;
String motion = "none";

// Orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];           // Raw yaw, pitch, roll
float ypr_filtered[3];  // Filtered yaw, pitch, roll

// Filter coefficient and variables
float alpha = 0.0;  // Low-pass filter coefficient
unsigned long prevTime = 0;
float deltaTime = 0.05;  // Initial estimate of sampling interval (20 Hz)

// Classification variables
int imu_data[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
const float confidence_rate = 0.65f;

int raw_feature_get_data(size_t offset, size_t length, float* out_ptr);
void calculateInference();
void print_inference_result(ei_impulse_result_t result);

Adafruit_DotStarMatrix matrix = Adafruit_DotStarMatrix(
  8, 8, DATAPIN, CLOCKPIN,
  DS_MATRIX_TOP + DS_MATRIX_RIGHT + DS_MATRIX_COLUMNS + DS_MATRIX_PROGRESSIVE,
  DOTSTAR_BRG);

const uint16_t colors[] = {
  matrix.Color(175, 0, 0),  // Red for arrows
  matrix.Color(80, 80, 0)   // Yellow for low light conditions
};

// Bitmap for a right-pointing arrow
const uint8_t rightArrow[] = {
  0b00000000,  // Row 1: Empty
  0b00001000,  // Row 2: Small arrow tip
  0b00001100,  // Row 3: Arrowhead part 1
  0b11111111,  // Row 4: Line with arrowhead
  0b11111111,  // Row 5: Line with arrowhead
  0b00001100,  // Row 6: Arrowhead part 2
  0b00001000,  // Row 7: Small arrow tip
  0b00000000   // Row 8: Empty
};

// Bitmap for a left-pointing arrow
const uint8_t leftArrow[] = {
  0b00000000,  // Row 1: Empty
  0b00010000,  // Row 2: Small arrow tip
  0b00110000,  // Row 3: Arrowhead part 1
  0b11111111,  // Row 4: Line with arrowhead
  0b11111111,  // Row 5: Line with arrowhead
  0b00110000,  // Row 6: Arrowhead part 2
  0b00010000,  // Row 7: Small arrow tip
  0b00000000   // Row 8: Empty
};

// Bitmap for all on
const uint8_t allOn[] = {
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111
};

// Wi-Fi/Network credentials
// Update tokens
const char* ssid = "<WLAN SSID>";
const char* password = "<WLAN Password>>";

void setup() {
  Serial.begin(115200);

  // Wait till Serial is available
  while (!Serial)
    ;
  delay(1000);

  // Initialize I2C communication
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);  // Set I2C clock speed to 400kHz

  analogReadResolution(12);  // ESP32 ADC has 12-bit resolution

  pinMode(BUZZER_PIN_LEFT, OUTPUT);
  pinMode(BUZZER_PIN_RIGHT, OUTPUT);
  pinMode(PIR_PIN_LEFT, INPUT);
  pinMode(PIR_PIN_RIGHT, INPUT);

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");

  // Initialize DMP
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  // Calibrated using calibration code
  mpu.setXGyroOffset(-558);
  mpu.setYGyroOffset(51);
  mpu.setZGyroOffset(128);
  mpu.setZAccelOffset(1266);


  // Check if DMP initialization was successful
  if (devStatus == 0) {
    Serial.println("DMP initialized successfully");
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;

    // Get expected packet size for DMP
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // DMP initialization failed
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  float cutoffFreq = 0.75;  // Low-pass filter cutoff frequency in Hz
  alpha = deltaTime / (deltaTime + (1.0 / cutoffFreq));

  matrix.begin();
  matrix.setBrightness(40);
}

void loop() {
  static int temp_arr[3] = { 0 };
  if (!dmpReady) return;
  fifoCount = mpu.getFIFOCount();

  // Handle FIFO overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
    return;
  }

  // Wait for correct data length
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  // Read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  // Track FIFO count here in case there is > 1 packet available
  fifoCount -= packetSize;

  // Parse quaternion and calculate Yaw, Pitch, Roll
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // Apply low-pass filter
  for (int i = 0; i < 3; i++) {
    ypr_filtered[i] = alpha * ypr[i] + (1.0 - alpha) * ypr_filtered[i];
  }

  // Convert filtered values to degrees
  temp_arr[0] = ypr_filtered[0] * 180 / M_PI;
  temp_arr[1] = ypr_filtered[1] * 180 / M_PI;
  temp_arr[2] = ypr_filtered[2] * 180 / M_PI;

  // Shift data into the inference buffer
  for (int i = 0; i < (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME); i++) {
    imu_data[i] = imu_data[i + EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME];
    features[i] = (float)imu_data[i];
  }
  for (int i = 0; i < EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME; i++) {
    imu_data[(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) + i] = temp_arr[i];
    features[(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) + i] = (float)temp_arr[i];
  }

  // Perform inference every 0.75 of a second
  if (millis() - prevTime >= 750) {
    motion = return_inference_result();
    prevTime = millis();
  }

  // Left PIR sensor and buzzer
  if (digitalRead(PIR_PIN_LEFT)) {
    left_PIR = 1;
    buzz(BUZZER_PIN_LEFT);
  }

  // Right PIR sensor and buzzer
  if (digitalRead(PIR_PIN_RIGHT)) {
    right_PIR = 1;
    buzz(BUZZER_PIN_RIGHT);
  }

  battery_voltage = batteryVoltage(BATTERY_V_READ);
  Serial.print("Battery voltage: ");
  Serial.print(BATTERY_V_READ);
  Serial.println(" V");

  if (motion == "right"){
    right_arrow();
  }

  if (motion == "left"){
    left_arrow();
  }
}

void calculateInference() {
  ei_impulse_result_t result = { 0 };

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

String return_inference_result() {
  ei_impulse_result_t result = { 0 };

  signal_t features_signal;
  features_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  features_signal.get_data = &raw_feature_get_data;

  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);

  float max_val = 0.0f;
  int max_index = -1;

  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    //Serial.printf("%s: %.2f\n", result.classification[i].label, result.classification[i].value);
    if (result.classification[i].value > max_val) {
      max_val = result.classification[i].value;
      max_index = i;
    }
  }

  if (max_val > confidence_rate) {
    //Serial.printf("Detected: %s (confidence: %.2f)\n", result.classification[max_index].label, max_val);
    return result.classification[max_index].label;
  } else {
    return "none";
  }
}

int raw_feature_get_data(size_t offset, size_t length, float* out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

// Function for buzzer warnign sound
// Paramater either BUZZER_PIN_LEFT or BUZZER_PIN_RIGHT
void buzz(int buzzerPin) {
  for (int i = 0; i < 2; i++) {
    tone(buzzerPin, 1000);
    delay(200);

    noTone(buzzerPin);
    delay(200);

    tone(buzzerPin, 1500);
    delay(200);

    noTone(buzzerPin);
    delay(200);
  }

  if (buzzerPin == BUZZER_PIN_LEFT) {
    left_PIR = 0;
  } else if (buzzerPin == BUZZER_PIN_RIGHT) {
    right_PIR = 0;
  }
}

float batteryVoltage(int battery_Pin) {
  int adcValue = analogRead(battery_Pin);

  // Convert ADC value to voltage (scaled for 3.3V reference)
  float scaledVoltage = (adcValue / 4095.0) * 3.3;

  // Calculate the actual battery voltage using the voltage divider
  float batteryVoltage = scaledVoltage * (R1 + R2) / R2;

  // Print the battery voltage
  Serial.print(batteryVoltage);

  return batteryVoltage;
}

void right_arrow() {
  matrix.fillScreen(0);
  matrix.drawBitmap(0, 0, rightArrow, 8, 8, colors[0]);
  matrix.show();
  delay(4000);
  matrix.fillScreen(0);
}

void left_arrow() {
  matrix.fillScreen(0);
  matrix.drawBitmap(0, 0, leftArrow, 8, 8, colors[0]);
  matrix.show();
  delay(4000);
  matrix.fillScreen(0);
}

void all_on() {
  matrix.fillScreen(0);
  matrix.drawBitmap(0, 0, allOn, 8, 8, colors[1]);
  matrix.show();
}