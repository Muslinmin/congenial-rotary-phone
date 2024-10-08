#include "FastIMU.h"
#include <Wire.h>
#include <Servo.h>

MPU6500 mpu;  // Define mpu object from class MPU6500
AccelData accelOut;
GyroData gyroOut;
calData cal = {0};  // Calibration data

float pitch = 0.0, roll = 0.0, yaw = 0.0;  // Euler angles
const float alpha = 0.97;  // Complementary filter coefficient (between 0 and 1)
unsigned long prevTime = 0;

// Servo objects for plane control
Servo aileronLeft;
Servo aileronRight;
Servo elevatorLeft;
Servo elevatorRight;

// Servo pins
int aileronLeftPin = 9;
int aileronRightPin = 10;
int elevatorLeftPin = 5;
int elevatorRightPin = 6;

int prevAileronLeftAngle = 90;
int prevAileronRightAngle = 90;
int prevElevatorLeftAngle = 90;
int prevElevatorRightAngle = 90;
class Kalman {
public:
  float Q_angle;   // Process noise variance for the accelerometer
  float Q_bias;    // Process noise variance for the gyroscope bias
  float R_measure; // Measurement noise variance

  float angle;     // The angle calculated by the Kalman filter
  float bias;      // The gyro bias calculated by the Kalman filter
  float rate;      // Unbiased rate calculated by subtracting bias from gyro rate

  float P[2][2];   // Error covariance matrix

  Kalman() {
    Q_angle = 0.001f;
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
    // Predict
    rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
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

void transmitToMatLab(float pitchT, float rollT) {
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
    Serial.print(pitchT, 2);  // Z gyroscope data
    Serial.print(",");
    Serial.print(rollT, 2);  // Z gyroscope data
    Serial.print(",");

    // End header: send 'FFF'
    Serial.print("FFF");
}

void setup() {
  cal.valid = false;  // Mark calibration as invalid initially
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);  // Set the I2C clock to 400 kHz
  delay(2000);

  // Initialize MPU6500 with the specified address
  int err = mpu.init(cal, 0x68);
  if (err != 0) {
    while (true) {
      ;  // Stop execution if initialization fails
    }
  } else {
    Serial.println("MPU6500 Initialized");
  }

  // Perform calibration
  mpu.calibrateAccelGyro(&cal);

  // Reinitialize with the new calibration data
  mpu.init(cal, 0x68);

  // Attach servos to pins
  aileronLeft.attach(aileronLeftPin);
  aileronRight.attach(aileronRightPin);
  elevatorLeft.attach(elevatorLeftPin);
  elevatorRight.attach(elevatorRightPin);
}

void servoControl(float pit, float rol) {
  // Convert the float parameters to int
  int pitchInt = static_cast<int>(pit);
  int rollInt = static_cast<int>(rol);

  // Calculate new servo angles
  int aileronLeftAngle = 90 + rollInt;
  int aileronRightAngle = 90 - rollInt;
  int elevatorLeftAngle = 90 + pitchInt;
  int elevatorRightAngle = 90 + pitchInt;

  // Check for changes beyond 5 degrees for each servo
  if (abs(aileronLeftAngle - prevAileronLeftAngle) >= 1) {
    // aileronLeft.write(aileronLeftAngle);
    // aileronRight.write(aileronLeftAngle); // emulate
    prevAileronLeftAngle = aileronLeftAngle;  // Update previous angle
    // Serial.print("Aileron Left Angle: ");
    // Serial.println(aileronLeftAngle);
  }

  if (abs(aileronRightAngle - prevAileronRightAngle) >= 1) {
    // aileronRight.write(aileronRightAngle);
    prevAileronRightAngle = aileronRightAngle;  // Update previous angle
    Serial.print("Aileron Right Angle: ");
    Serial.println(aileronRightAngle);
  }

  if (abs(elevatorLeftAngle - prevElevatorLeftAngle) >= 1) {
    // elevatorLeft.write(elevatorLeftAngle);
    // aileronRight.write(elevatorLeftAngle); // emulate
    prevElevatorLeftAngle = elevatorLeftAngle;  // Update previous angle
    // Serial.print("Elevator Left Angle: ");
    // Serial.println(elevatorLeftAngle);
  }

  if (abs(elevatorRightAngle - prevElevatorRightAngle) >= 1) {
    // elevatorRight.write(elevatorRightAngle);
    prevElevatorRightAngle = elevatorRightAngle;  // Update previous angle
    // Serial.print("Elevator Right Angle: ");
    // Serial.println(elevatorRightAngle);
  }
}

Kalman kalmanPitch;
Kalman kalmanRoll;

void loop() {
  mpu.update();
  mpu.getAccel(&accelOut);
  mpu.getGyro(&gyroOut);

  // Calculate time difference
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;  // Convert to seconds
  prevTime = currentTime;

  // Calculate pitch and roll from accelerometer data
  float accelPitch = atan2(accelOut.accelY, sqrt(accelOut.accelX * accelOut.accelX + accelOut.accelZ * accelOut.accelZ)) * 180.0 / PI;
  float accelRoll = atan2(-accelOut.accelX, sqrt(accelOut.accelY * accelOut.accelY + accelOut.accelZ * accelOut.accelZ)) * 180.0 / PI;

  // Use the Kalman filter to estimate pitch and roll angles
  pitch = kalmanPitch.getAngle(accelPitch, gyroOut.gyroX, dt);
  roll = kalmanRoll.getAngle(accelRoll, gyroOut.gyroY, dt);

  // Control servos based on pitch and roll
  // servoControl(pitch, roll);
  transmitToMatLab(pitch, roll);
  delay(1);  // Add some delay for stability
}

