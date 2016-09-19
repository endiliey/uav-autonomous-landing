// Created for DIP UAV Project 2016/17 Semester 1 under Prof. Goh Wang Ling
// Defining all trigger and echo pins for all 3 sonars on Arduino UNO
// Sonars used are HC-SR04
#define trigPin 3 
#define echoPin 2
#define buzzer 4
#define trigPin2 5
#define echoPin2 7
#define trigPin3 1
#define echoPin3 13


#include <SoftwareSerial.h> // We need SoftwareSerialLibrary to allow serial communication to take place on other digital pins of the board

SoftwareSerial mySerial(8, 9); //TX,RX

void setup(){ // start of code, setting defined trigger pin as output and echo pin as input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  Serial.begin(9600);  // setting bps to computer as 9600 bits per second and beginning Serial Monitor (to debug)
  }

//stabilize all trig pin low to zero volt before trigerring
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin2, LOW);
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);

//Initializing all variables to store time and distance for all sonars
int duration, distance;
int duration2, distance2;
int duration3, distance3;

//Sonar 1 Function
void sonar1(){
    // Raise trig pin HIGH for 1000us to trigger ultrasound pulse
    digitalWrite (trigPin, HIGH);
    delayMicroseconds (1000);
    digitalWrite (trigPin, LOW);
    duration = pulseIn (echoPin, HIGH);
    distance = (duration/2) /29.1; // Based on data sheet, Sound travels 340 m/s or around 29.1 microsecond per centimeter
}

//Sonar 2 Function
void sonar2(){
    // Raise trig pin HIGH for 1000us to trigger ultrasound pulse
    digitalWrite (trigPin2, HIGH);
    delayMicroseconds (1000);
    digitalWrite (trigPin2, LOW);
    duration2 = pulseIn (echoPin2, HIGH);
    distance2 = (duration2/2) /29.1;
}

//Sonar 3 Function
void sonar3(){
    // Raise trig pin HIGH for 1000us to trigger ultrasound pulse
    digitalWrite (trigPin3, HIGH);
    delayMicroseconds (1000);
    digitalWrite (trigPin3, LOW);
    duration3 = pulseIn (echoPin3, HIGH);
    distance3 = (duration3/2) /29.1;
}

//Loop functions to keep the sonar detection running
void loop(){
  sonar1();
  sonar2();
  sonar3();

Serial.println("Sonar 1 Sonar 2 Sonar 3"); //

  if (distance > 400) {
    Serial.println("Wrong");
  }
  else
  {
    Serial.print (distance,DEC);
    Serial.println("cm\t,");
  }

  if (distance2 > 400) {
    Serial.println("Wrong\t");
  }
  else
  {
    Serial.print (distance2,DEC);
    Serial.println("cm\t,");
  }

  if (distance3 > 400) {
    Serial.println("Wrong\t");
  }
  else
  {
    Serial.print (distance3,DEC);
    Serial.println("cm\t,");
  }

  //If copters are within 100cm distance of all sonars
  if (distance < 100 && distance2 <100 && distance3 <100 ) {
    Serial.println("land");
    //Do something to tell PC
   //mySerial.write(88);
    //Serial.write("land");
  // We can add some sound here when landing. Example : tone(4,100,200);
  }
  else if (distance < 200 && distance2 <200 && distance3 <200 ) {
    Serial.println("fly");
  //We can add some sound here when landing. Example : tone(4,300,1000);
}
  /*else noTone(4);
  
  delay(3000);
  }
  */

