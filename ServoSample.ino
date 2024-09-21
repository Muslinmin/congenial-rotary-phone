#include <Servo.h>
Servo myServo;  // Create a Servo object

void setup() {
  // put your setup code here, to run once:
  myServo.attach(9);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int pos = 0; pos <= 180; pos += 1) {  // Move from 0 to 180 degrees
    myServo.write(pos);  // Tell servo to go to position in variable 'pos'
    delay(30);  // Wait 15 milliseconds for the servo to reach the position
  }
  for (int pos = 180; pos >= 0; pos -= 1) {  // Move from 180 to 0 degrees
    myServo.write(pos);  // Tell servo to go to position in variable 'pos'
    delay(30);  // Wait 15 milliseconds for the servo to reach the position
  }
  // myServo.write(0);
}
