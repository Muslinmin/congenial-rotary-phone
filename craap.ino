#include "FastIMU.h"
#include <Wire.h>

MPU6500 mpu;  // Define mpu object from class MPU6500
AccelData accelOut;
GyroData gyroOut;
calData cal = { 0 };  // Calibration data

float pitch = 0.0, roll = 0.0, yaw = 0.0; // Euler angles
const float alpha = 0.98; // Complementary filter coefficient (between 0 and 1)
unsigned long prevTime = 0;

void setup() {
  cal.valid = false;  // Mark calibration as invalid initially
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);  // Set the I2C clock to 400 kHz
  delay(2000);

  // Initialize MPU6500 with the specified address
  int err = mpu.init(cal, 0x68);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;  // Stop execution if initialization fails
    }
  } else {
    Serial.println("MPU6500 Initialized");
  }

  // Perform calibration (optional)
  Serial.println("Performing Calibration...");
  mpu.calibrateAccelGyro(&cal);
  Serial.println("Calibration done!");

  // Reinitialize with the new calibration data
  mpu.init(cal, 0x68);
}

void loop() {
    mpu.update();
    mpu.getAccel(&accelOut);
    mpu.getGyro(&gyroOut);

    // Calculate time difference
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    prevTime = currentTime;

    // **Calculate pitch and roll from accelerometer data**
    float accelPitch = atan2(accelOut.accelY, sqrt(accelOut.accelX * accelOut.accelX + accelOut.accelZ * accelOut.accelZ)) * 180.0 / PI;
    float accelRoll = atan2(-accelOut.accelX, sqrt(accelOut.accelY * accelOut.accelY + accelOut.accelZ * accelOut.accelZ)) * 180.0 / PI;

    // **Integrate gyroscope data to calculate yaw**
    //yaw += gyroOut.gyroZ * dt; // Assuming gyroZ is in degrees per second

    // **Apply the complementary filter**
    pitch = alpha * (pitch + gyroOut.gyroX * dt) + (1.0 - alpha) * accelPitch;
    roll = alpha * (roll + gyroOut.gyroY * dt) + (1.0 - alpha) * accelRoll;

    // Print Euler angles
    Serial.print("Pitch: "); Serial.println(pitch);
    Serial.print(" Roll: "); Serial.println(roll);
    //Serial.print(" Yaw: "); Serial.println(yaw);

    delay(100); // Add some delay for stability
}
