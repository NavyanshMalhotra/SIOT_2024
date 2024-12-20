#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // Set true if DMP initialized successfully
uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  delay(1000);

  Wire.begin(21, 18); // SDA, SCL
  Wire.setClock(400000); 

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");

  // Initialize DMP
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  // Calibrate the MPU6050
  mpu.setXGyroOffset(-558);
  mpu.setYGyroOffset(51);
  mpu.setZGyroOffset(128);
  mpu.setZAccelOffset(1266);

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
}

void loop() {
  // If DMP is not ready, do nothing
  if (!dmpReady) return;

  // Check for DMP data in FIFO
  fifoCount = mpu.getFIFOCount();

  // Check for FIFO overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO(); // Reset FIFO if overflowed
    //Serial.println("FIFO overflow!");
    return;
  }

  // Wait for correct data length
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // Read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // Track FIFO count here in case there is > 1 packet available
  fifoCount -= packetSize;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // Print Yaw, Pitch, Roll in degrees
  //Serial.print("Yaw: ");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print('\t');
  //Serial.print(", Pitch: ");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print('\t');
  //Serial.print(", Roll: ");
  Serial.println(ypr[2] * 180 / M_PI);

  delay(50);
}
