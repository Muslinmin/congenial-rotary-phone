#include <Servo.h>

#define echo 21
#define trig 20

#define echo_2 19
#define trig_2 18

//Y axis - distance measured
//X axis - yaw or panning
//Z axis - tilt (UP OR DOWN)

// Linear eqn derived from excel, y=0.9713x+0.0442
// Polynomial eqn derived from excel, y=0.0006x^2+0.9093x+1.3387

float t=0, distanceX=0, distanceZ=0,hypotenuse=0, scale=1e-4; //add scaling calculation here
// Eqn gradient m and constant c
float m = 0.9713, c = 0.0442;

Servo myServo;  // Panning servo X axis
Servo zServo;

void setup() {

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  //second ultrasonic sensor for Z axis
  pinMode(trig_2, OUTPUT);
  pinMode(echo_2, INPUT);

  Serial.begin(9600);
  myServo.attach(9);// X axis panning
  zServo.attach(6); // z axis panning
}
void readSensor(int xAxisDeg, int zAxisDeg){
  //Generate trig pulse to start
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  //measure TOF and compute distance
  t = pulseIn(echo, HIGH);
  distanceX = scale* t * 343 / 2;


  digitalWrite(trig_2,LOW);
  delayMicroseconds(2);
  digitalWrite(trig_2,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_2,LOW);
  //measure TOF and compute distance
  t = pulseIn(echo_2, HIGH);
  distanceZ = scale* t * 343 / 2;

  hypotenuse = sqrt((distanceX*distanceX) + (distanceZ*distanceZ)); //^2 doesnt work in this case...

  Serial.print(xAxisDeg); 
  Serial.print(","); 
  Serial.print(zAxisDeg);
  Serial.print(","); 
  Serial.print(hypotenuse);
  Serial.print(":");
  //e.g. 90,45,139: "90 degrees from left, 45 degrees from ground, 139cm away"
}

void debug(){
  Serial.print(12); 
  Serial.print(","); 
  Serial.print(10);
  Serial.print(","); 
  Serial.print(15);
  Serial.write(58);
}
  void loop() {
    for (int pos = 15; pos <= 180; pos += 1) {
    // myServo.write(pos);
    delay(15);  
      for(int zPos = 15; zPos <= 180; zPos+=1){
        // zServo.write(pos);  
        delay(15); 
        //readSensor(pos, zPos);
        debug();
      }
    }
    for (int pos = 180; pos >= 15; pos -= 1) {
    // myServo.write(pos);
      delay(15);  
      for(int zPos = 180; zPos >= 15; zPos-=1){
        // zServo.write(pos);  
        delay(15); 
        // readSensor(pos, zPos);
        debug();
      }
    }
}
