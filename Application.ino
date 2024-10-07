#include "FastIMU.h"
#include <Wire.h>
#include <Servo.h>

MPU6500 mpu;  // Define mpu object from class MPU6500
AccelData accelOut;
GyroData gyroOut;
calData cal = {0};  // Calibration data

float pitch = 0.0, roll = 0.0, yaw = 0.0;  // Euler angles
const float alpha = 0.98;  // Complementary filter coefficient (between 0 and 1)
unsigned long prevTime = 0;

// Servo objects
Servo servo1;
// Servo servo2;
// Servo servo3;
// Servo servo4;

// Servo pins
int servopin1 = 9;
// int servopin2 = 10;
// int servopin3 = 5;
// int servopin4 = 6;


int prevServo1Angle = 90;
int prevServo2Angle = 90;
int prevServo3Angle = 90;
int prevServo4Angle = 90;

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
  servo1.attach(servopin1);
  // servo2.attach(servopin2);
  // servo3.attach(servopin3);
  // servo4.attach(servopin4);
}

void servoControl(float pit, float rol) {
  // Convert the float parameters to int
  int pitchInt = static_cast<int>(pit);
  int rollInt = static_cast<int>(rol);

  // Calculate new servo angles
  int servo1Angle = 90 + rollInt;
  int servo2Angle = 90 - rollInt;
  int servo3Angle = 90 + pitchInt;
  int servo4Angle = 90 - pitchInt;

  // Check for changes beyond 5 degrees for each servo
  if (abs(servo1Angle - prevServo1Angle) >= 5) {
    servo1.write(servo1Angle);
    prevServo1Angle = servo1Angle;  // Update previous angle
    Serial.print("Servo1 Angle: ");
    Serial.println(servo1Angle);
  }

  if (abs(servo2Angle - prevServo2Angle) >= 5) {
    // servo2.write(servo2Angle);
    prevServo2Angle = servo2Angle;  // Update previous angle
    Serial.print("Servo2 Angle: ");
    Serial.println(servo2Angle);
  }

  if (abs(servo3Angle - prevServo3Angle) >= 5) {
    // servo3.write(servo3Angle);
    prevServo3Angle = servo3Angle;  // Update previous angle
    Serial.print("Servo3 Angle: ");
    Serial.println(servo3Angle);
  }

  if (abs(servo4Angle - prevServo4Angle) >= 5) {
    // servo4.write(servo4Angle);
    prevServo4Angle = servo4Angle;  // Update previous angle
    Serial.print("Servo4 Angle: ");
    Serial.println(servo4Angle);
  }
}

void transmitToPythonSimulationApp() {
  // Serial communication routine
  if (Serial.available()) {
    char rx_char = Serial.read();
    if (rx_char == '.') {
      Serial.print(pitch, 2);
      Serial.print(", ");
      Serial.print(roll, 2);
      Serial.print(", ");
      Serial.println(yaw, 2); // Transmit yaw if needed
    }
  }
}

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

  // Integrate gyroscope data to calculate yaw
  yaw += gyroOut.gyroZ * dt;

  // Apply the complementary filter
  pitch = alpha * (pitch + gyroOut.gyroX * dt) + (1.0 - alpha) * accelPitch;
  roll = alpha * (roll + gyroOut.gyroY * dt) + (1.0 - alpha) * accelRoll;

  //function that has the control signal over here
  // Transmit data to Python simulation app
  // transmitToPythonSimulationApp();
  servoControl(pitch, roll);
  // Transmit data to MATLAB (optional)
  // transmitToMatLab();

  delay(100);  // Add some delay for stability
}
