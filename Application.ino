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
    // Serial.print("Aileron Right Angle: ");
    // Serial.println(aileronRightAngle);
  }

  if (abs(elevatorLeftAngle - prevElevatorLeftAngle) >= 1) {
    // elevatorLeft.write(elevatorLeftAngle);
    aileronRight.write(elevatorLeftAngle); // emulate
    prevElevatorLeftAngle = elevatorLeftAngle;  // Update previous angle
    Serial.print("Elevator Left Angle: ");
    Serial.println(elevatorLeftAngle);
  }

  if (abs(elevatorRightAngle - prevElevatorRightAngle) >= 1) {
    // elevatorRight.write(elevatorRightAngle);
    prevElevatorRightAngle = elevatorRightAngle;  // Update previous angle
    // Serial.print("Elevator Right Angle: ");
    // Serial.println(elevatorRightAngle);
  }
}

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
  // float accelRoll = atan2(accelOut.accelY, accelOut.accelZ) * 180.0 / PI;
  // Integrate gyroscope data to calculate yaw
  yaw += gyroOut.gyroZ * dt;

  // Apply the complementary filter
  pitch = alpha * (pitch + gyroOut.gyroX * dt) + (1.0 - alpha) * accelPitch;
  roll = alpha * (roll + gyroOut.gyroY * dt) + (1.0 - alpha) * accelRoll;

  // Control servos based on pitch and roll
  servoControl(pitch, roll);

  delay(1);  // Add some delay for stability
}
