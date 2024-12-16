// Include necessary libraries
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SIOT_helmet_motion_inferencing.h>

// MPU6050 instance
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];          // Raw yaw, pitch, roll
float ypr_filtered[3]; // Filtered yaw, pitch, roll

// Filter coefficient and variables
float alpha = 0.0;     // Low-pass filter coefficient
unsigned long prevTime = 0;
float deltaTime = 0.02; // Initial estimate of sampling interval (50 Hz)

// Classification variables
int imu_data[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
const float confidence_rate = 0.65f;

// Function prototypes
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr);
void calculateInference();
void print_inference_result(ei_impulse_result_t result);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  delay(1000);

  // Initialize I2C communication
  Wire.begin(21, 18);     // SDA, SCL
  Wire.setClock(400000);  // Set I2C clock speed to 400kHz

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");

  // Initialize DMP
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  // Calibrate the MPU6050 (adjust these values as needed)
  mpu.setXGyroOffset(-558);
  mpu.setYGyroOffset(51);
  mpu.setZGyroOffset(128);
  mpu.setZAccelOffset(1266); // Adjust based on your device

  // Check if DMP initialization was successful
  if (devStatus == 0) {
    Serial.println("DMP initialized successfully");

    // Enable DMP
    mpu.setDMPEnabled(true);

    // Set interrupt enabled
    mpuIntStatus = mpu.getIntStatus();

    // Set DMP ready flag
    dmpReady = true;

    // Get expected packet size for DMP
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // DMP initialization failed
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  // Initialize filter coefficient
  float cutoffFreq = 0.75; // Low-pass filter cutoff frequency in Hz
  alpha = deltaTime / (deltaTime + (1.0 / cutoffFreq));
}

void loop() {
  static int temp_arr[3] = {0};

  // If DMP is not ready, do nothing
  if (!dmpReady) return;

  // Check for DMP data in FIFO
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

  // Perform inference every second
  if (millis() - prevTime >= 1000) {
    calculateInference();
    prevTime = millis();
  }
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

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}
