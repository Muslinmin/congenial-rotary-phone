#include <stdio.h>
#define trig 20
#define echo 21

float t=0, d=0, scale=1e-4; //add scaling calculation here
char charVal[10];
String stringVal = " cm";


void setup() {
  // put your setup code here, to run once:
  // Setup baud ratep
  Serial.begin(9600);
  // Configure GPIO
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
}

void loop() {
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
  delay (1000);
}
