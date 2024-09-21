#include <stdio.h>

#include <Servo.h>

#define trig 20
#define echo 21

float t=0, d=0, scale=1e-4; //add scaling calculation here
char charVal[10];
String stringVal = " cm";
Servo myServo;  // Create a Servo object

void setup() {
  // put your setup code here, to run once:
  // Setup baud ratep
  Serial.begin(9600);
  // Configure GPIO
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
  myServo.attach(9);
  delay(50);
}
void readSensor(){
    // put your main code here, to run repeatedly:
  //Generate trig pulse to start
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  //measure TOF and compute distance
  t = pulseIn(echo, HIGH);
  d = scale* t * 340 / 2;
  // Use serial print to display result on serial monitor
  Serial.print("Distance: ");
  Serial.print(d);
  Serial.println("cm");
  delay(50);
}
void loop() {
    for (int pos = 0; pos <= 180; pos += 1) {  // Move from 0 to 180 degrees
    myServo.write(pos);  // Tell servo to go to position in variable 'pos'
    delay(30);  // Wait 15 milliseconds for the servo to reach the position
    readSensor();
  }
  for (int pos = 180; pos >= 0; pos -= 1) {  // Move from 180 to 0 degrees
    myServo.write(pos);  // Tell servo to go to position in variable 'pos'
    delay(30);  // Wait 15 milliseconds for the servo to reach the position
    readSensor();
  }
}
