#include <stdio.h>
#include <Servo.h>
#define trig 20
#define echo 21

// Linear eqn derived from excel, y=0.9713x+0.0442
// Polynomial eqn derived from excel, y=0.0006x^2+0.9093x+1.3387

float t=0, d=0, scale=1e-4; //add scaling calculation here
// Eqn gradient m and constant c
float m = 0.9713, c = 0.0442;
char charVal[10];
String stringVal = " cm";
Servo myServo;  // Create a Servo object

void setup() {
  // Setup baud rate
  Serial.begin(9600);
  // Configure GPIO
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

void loop() {
//Generate trig pulse to start
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);

  //measure TOF and compute distance
  t = pulseIn(echo, HIGH);
  d = scale* t * 340 / 2;

  // Apply linear eqn to optimize distance
  // d = (m * d) + c;

  // Using polynomial for more precision
  d = 0.0006 * d * d + 0.9093 * d + 1.3387;

  // Use serial print to display result on serial monitor
  Serial.print("Optimized Distance: ");
  Serial.print(d);
  Serial.println("cm");
  delay(50);
}