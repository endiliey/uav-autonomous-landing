/*  
 *  Do not change anything if you don't know what you are doing
 *  12/10/16
 *  This program is the code to be uploaded to Arduino
 Line 8 - 90 : Getting PWM Value and Pixy Object 
 */

#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

#define yawPin 5 //ch1
#define pitchPin 6 //ch2
#define throttlePin 3 //ch3
#define rollPin 7 //ch4
#define ch5Pin 9 //ch5
#define ch6Pin 4 //ch6


//int ch1;
//int ch2;
//int ch3;
//int ch4;
//int ch5;
//int ch6;
int x;
int y;
int height;
int width;

void setup() {
  // put your setup code here, to run once:
pinMode(yawPin, INPUT);
pinMode(pitchPin, INPUT);
pinMode(throttlePin, INPUT);
pinMode(rollPin, INPUT);
pinMode(ch5Pin, INPUT);
pinMode(ch6Pin, INPUT);

pixy.init();
Serial.begin(9600);
Serial.println("Starting soon.. ");
}

void loop() {

  //ch1 = pulseIn(yawPin,HIGH); //yaw - >(most left = 2005, most right = 959)
  //ch2 = pulseIn(pitchPin,HIGH); //pitch -> (most top 2006, most bottom = 939)
  //ch3 = pulseIn(throttlePin,HIGH); // throttle -> (most top = 979, most bottom = 2034)
  //ch4 = pulseIn(rollPin,HIGH); //roll -> (most left = 985, most right = 1998)
  //ch5 = pulseIn(ch5Pin,HIGH); //ch5 -> ( on(toward us) = 2018 , off = 929)
  //ch6 = pulseIn(ch6Pin,HIGH); //ch6 - > (on = 2018, off = 929)

  Serial.println("Yaw     Pitch   Throttle     Roll       CH5     CH6"); // Print the value of 
  Serial.print(pulseIn(yawPin,HIGH));
  Serial.print("     ");
  Serial.print(pulseIn(pitchPin,HIGH));
  Serial.print("      ");
  Serial.print(pulseIn(throttlePin,HIGH));
  Serial.print("      ");
  Serial.print(pulseIn(rollPin,HIGH));
  Serial.print("      ");
  Serial.print(pulseIn(ch5Pin,HIGH));
  Serial.print("      ");
  Serial.print(pulseIn(ch6Pin,HIGH));
  Serial.println("");

  if (pixy.getBlocks() > 0)
  {

        x = (pixy.blocks[0].x) - 160 ;
        y = -1 * ((pixy.blocks[0].y) - 100) ;
        height = pixy.blocks[0].height;
        width = pixy.blocks[0].width;
        
  Serial.println("Detected: ");
            Serial.println("x    y    height  width");
            Serial.print(x); //this will be the x value of object detected by Pixy
            Serial.print("    ");
            Serial.print(y); // this will be the y value of object detected by Pixy
            Serial.print("    ");
            Serial.print(height);// this will be the width of object detected by pixy
            Serial.print("    "); 
            Serial.println(width); // this will be the height of object detected by pixy
  }    
  
  delay(500);

}
