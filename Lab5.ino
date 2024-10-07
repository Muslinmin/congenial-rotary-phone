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
    //Serial.print("Error initializing IMU: ");
    //Serial.println(err);
    while (true) {
      ;  // Stop execution if initialization fails
    }
  } else {
    Serial.println("MPU6500 Initialized");
  }

  // Perform calibration (optional)
  //Serial.println("Performing Calibration...");
  mpu.calibrateAccelGyro(&cal);
  //Serial.println("Calibration done!");

  // Reinitialize with the new calibration data
  mpu.init(cal, 0x68);
}

void transmitToPythonSimulationApp(){
    // Serial communication routine
    if (Serial.available()) {
      char rx_char = Serial.read();
      if (rx_char == '.') {
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.print(roll, 2);
        Serial.print(", ");
        Serial.println(yaw, 2); // Make sure to use yaw here if needed
      }
    }
}

// void transmitToMatLab() {
//   // Transmit the start header "111"
//   Serial.print("111");

//   // Transmit the accelerometer data
//   Serial.print("AccX="); Serial.print(accelOut.accelX); Serial.print(", ");
//   Serial.print("AccY="); Serial.print(accelOut.accelY); Serial.print(", ");
//   Serial.print("AccZ="); Serial.print(accelOut.accelZ); Serial.print(", ");
  
//   // Transmit the gyroscope data
//   Serial.print("GyrX="); Serial.print(gyroOut.gyroX); Serial.print(", ");
//   Serial.print("GyrY="); Serial.print(gyroOut.gyroY); Serial.print(", ");
//   Serial.print("GyrZ="); Serial.print(gyroOut.gyroZ);
  
//   // Transmit the end header "FFF"
//   Serial.println("FFF");
// }

void transmitToMatLab() {
    // Start header: send '111'
    Serial.print("111");

    // Transmit the IMU data as comma-separated values
    Serial.print(accelOut.accelX, 2);  // X acceleration with 2 decimal places
    Serial.print(",");
    Serial.print(accelOut.accelY, 2);  // Y acceleration with 2 decimal places
    Serial.print(",");
    Serial.print(accelOut.accelZ, 2);  // Z acceleration with 2 decimal places
    Serial.print(",");
    Serial.print(gyroOut.gyroX, 2);  // X gyroscope data
    Serial.print(",");
    Serial.print(gyroOut.gyroY, 2);  // Y gyroscope data
    Serial.print(",");
    Serial.print(gyroOut.gyroZ, 2);  // Z gyroscope data
    Serial.print(",");

    // End header: send 'FFF'
    Serial.print("FFF");
}


void loop() {
    mpu.update();
    mpu.getAccel(&accelOut);
    mpu.getGyro(&gyroOut);

    // Calculate time difference
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    prevTime = currentTime;

    // Calculate pitch and roll from accelerometer data
    float accelPitch = atan2(accelOut.accelY, sqrt(accelOut.accelX * accelOut.accelX + accelOut.accelZ * accelOut.accelZ)) * 180.0 / PI;
    float accelRoll = atan2(-accelOut.accelX, sqrt(accelOut.accelY * accelOut.accelY + accelOut.accelZ * accelOut.accelZ)) * 180.0 / PI;

    // Integrate gyroscope data to calculate yaw
    yaw += gyroOut.gyroZ * dt; // Uncomment this if you want to use yaw

    // Apply the complementary filter
    pitch = alpha * (pitch + gyroOut.gyroX * dt) + (1.0 - alpha) * accelPitch;
    roll = alpha * (roll + gyroOut.gyroY * dt) + (1.0 - alpha) * accelRoll;
    transmitToMatLab();
    delay(100); // Add some delay for stability
}
