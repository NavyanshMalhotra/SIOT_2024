#include "Wire.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t ax, ay, az, gx, gy, gz;
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial to connect (if needed)

  Serial.println("Initializing I2C...");
  Wire.begin(21, 18); 

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  Serial.println("Testing connection...");
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected.");
  } else {
    Serial.println("MPU6050 connection failed.");
    while (1)
      ; // Halt if connection fails
  }

  Serial.println("Calibrating MPU6050...");
  calibrateMPU();
}

void loop() {
  // Display the calibrated offsets
  Serial.print("Accel Offsets: ");
  Serial.print(ax_offset);
  Serial.print(", ");
  Serial.print(ay_offset);
  Serial.print(", ");
  Serial.println(az_offset);

  Serial.print("Gyro Offsets: ");
  Serial.print(gx_offset);
  Serial.print(", ");
  Serial.print(gy_offset);
  Serial.print(", ");
  Serial.println(gz_offset);

  delay(1000); // Update once per second
}

void calibrateMPU() {
  const int numSamples = 1000; // Number of samples for calibration

  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  // Read multiple samples for calibration
  for (int i = 0; i < numSamples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    axSum += ax;
    aySum += ay;
    azSum += (az - 16384); // Subtract gravity (1g = 16384 in MPU6050)
    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    delay(2); // Small delay between samples
  }

  // Calculate average offsets
  ax_offset = axSum / numSamples;
  ay_offset = aySum / numSamples;
  az_offset = azSum / numSamples;
  gx_offset = gxSum / numSamples;
  gy_offset = gySum / numSamples;
  gz_offset = gzSum / numSamples;

  // Set offsets in the MPU
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);

  Serial.println("Calibration complete!");
}


/*
Accel Offsets: -2077, -703, 2166
Gyro Offsets: -558, 51, 128
*/
