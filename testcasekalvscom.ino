#include "FastIMU.h"
#include <Wire.h>
#include <Servo.h>

MPU6500 mpu;  // Define mpu object from class MPU6500
AccelData accelOut;
GyroData gyroOut;
calData cal = {0};  // Calibration data

float pitch = 0.0, roll = 0.0, yaw = 0.0;  // Euler angles
float complementaryPitch = 0.0;  // Variable for complementary filter's pitch
const float alpha = 0.97;  // Complementary filter coefficient
unsigned long prevTime = 0;

// Kalman filter class and objects
class Kalman {
public:
  float Q_angle;
  float Q_bias;
  float R_measure;
  float angle;
  float bias;
  float rate;
  float P[2][2];

  Kalman() {
    Q_angle = 0.003f;
    Q_bias = 0.003f;
    R_measure = 0.03f;
    angle = 0.0f;
    bias = 0.0f;
    rate = 0.0f;
    P[0][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
  }

  float getAngle(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    return angle;
  }
};

// Kalman filter objects
Kalman kalmanPitch;
Kalman kalmanRoll;

// Function to plot the pitch values using Kalman, complementary, and raw accelerometer
void plotAllFilters(float kalmanPitch, float complementaryPitch, float accelPitch) {
    Serial.print(kalmanPitch);
    Serial.print("\t");
    Serial.print(complementaryPitch);
    Serial.print("\t");
    Serial.println(accelPitch);  // End the line with a newline character
}

void setup() {
  cal.valid = false;
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  delay(2000);

  // Initialize MPU6500
  int err = mpu.init(cal, 0x68);
  if (err != 0) {
    while (true) {
      ;
    }
  }

  // Perform calibration
  mpu.calibrateAccelGyro(&cal);
  mpu.init(cal, 0x68);
}

void loop() {
  mpu.update();
  mpu.getAccel(&accelOut);
  mpu.getGyro(&gyroOut);

  // Calculate time difference
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;  // Convert to seconds
  prevTime = currentTime;

  // Calculate pitch from accelerometer data
  float accelPitch = atan2(accelOut.accelY, sqrt(accelOut.accelX * accelOut.accelX + accelOut.accelZ * accelOut.accelZ)) * 180.0 / PI;

  // Use the Kalman filter to estimate pitch
  float kalmanPitchValue = kalmanPitch.getAngle(accelPitch, gyroOut.gyroX, dt);

  // Use the complementary filter to estimate pitch
  complementaryPitch = alpha * (complementaryPitch + gyroOut.gyroX * dt) + (1.0 - alpha) * accelPitch;

  // Plot Kalman pitch, complementary pitch, and raw accelerometer pitch
  plotAllFilters(kalmanPitchValue, complementaryPitch, accelPitch);

  delay(10);  // Add some delay for stability
}
