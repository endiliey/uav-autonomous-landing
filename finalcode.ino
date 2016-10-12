/* This program is to get the x,y,height and width value of an object detected by Pixy
 * Please do not change this program without permission
 * By DIP Group B - NTU EEE Semester 1 AY.2016-2016
*/ 

#include <SPI.h>  
#include <Pixy.h>

// Create instances of pixy as an initializer 
Pixy pixy;

#define yawPin 5 //ch1
#define pitchPin 6 //ch2
#define throttlePin 3 //ch3
#define rollPin 11 //ch4
#define ch5Pin 9 //ch5
#define ch6Pin 10 //ch6


int ch1;
int ch2;
int ch3;
int ch4;
int ch5;
int ch6;

void setup()
{
  pinMode(yawPin, INPUT);
  pinMode(pitchPin, INPUT);
  pinMode(throttlePin, INPUT);
  pinMode(rollPin, INPUT);
  pinMode(ch5Pin, INPUT);
  pinMode(ch6Pin, INPUT);

  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();
}

void loop()
{ 
  // All this line is for Pixy main detecting purpose //
  
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  int x; //this will be the x value of object detected by Pixy
  int y; // this will be the y value of object detected by Pixy
  int width; // this will be the width of object detected by pixy
  int height; // this will be the height of object detected by pixy

  // We get all the object detected by Pixy with Pixy in-built method function -> getBlocks()
  blocks = pixy.getBlocks();
  
  // Since Pixy can detect lot of signature, just print any of them !
  if (blocks)
  {
    i++;
    
    // do this (print) every 50 frames (anything faster could bog down the arduino
    if (i%50==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        x = pixy.blocks[j].x - 160 ;
        y = -1 * (pixy.blocks[j].y -100) ;
        height = pixy.blocks[j].height;
        width = pixy.blocks[j].width;
        
        Serial.println("x    y    height     width");
        Serial.print("           ");
        Serial.print(x);
        Serial.print("    ");
        Serial.print(y);
        Serial.print("    ");
        Serial.print(height);
        Serial.print("    ");
        Serial.println(width);

        
      }
    }
  } 

 ch1 = pulseIn(yawPin,HIGH); //yaw - >(most left = 2005, most right = 959)
  ch2 = pulseIn(pitchPin,HIGH); //pitch -> (most top 2006, most bottom = 939)
  ch3 = pulseIn(throttlePin,HIGH); // throttle -> (most top = 979, most bottom = 2034)
  ch4 = pulseIn(rollPin,HIGH); //roll -> (most left = 985, most right = 1998)
  ch5 = pulseIn(ch5Pin,HIGH); //ch5 -> ( on(toward us) = 2018 , off = 929)
  ch6 = pulseIn(ch6Pin,HIGH); //ch6 - > (on = 2018, off = 929)

  Serial.println("Yaw        Pitch      Throttle         Roll       CH5        CH6"); // Print the value of 
  Serial.print(ch1);
  Serial.print("         ");
  Serial.print(ch2);
  Serial.print("         ");
  Serial.print(ch3);
  Serial.print("         ");
  Serial.print(ch4);
  Serial.print("         ");
  Serial.print(ch5);
  Serial.print("         ");
  Serial.print(ch6);
  Serial.println("");
   
}
