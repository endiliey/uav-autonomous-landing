#include <SoftwareSerial.h>
SoftwareSerial bluetooth(2, 3); //TX,RX
#define thrin 4
//#define rolin 12
//#define pitin 11
//#define yawin 10
//#define ax1in 9
#define throut 5
//#define rolout 6
//#define pitout 5
//#define yawout 4
#define ax1out 2
//#define thrinanalog A1
int thr, rol, pit, yaw;
boolean ax1;
#define buzzer 8

void setup() {
  pinMode(thrin, INPUT);
//  pinMode(thrinanalog, INPUT);
//  pinMode(rolin, INPUT);
//  pinMode(pitin, INPUT);
//  pinMode(yawin, INPUT);
//  pinMode(ax1in, INPUT);
  pinMode(throut, OUTPUT);
//  pinMode(rolout, OUTPUT);
//  pinMode(pitout, OUTPUT);
//  pinMode(yawout, OUTPUT);
//  pinMode(ax1out, OUTPUT);
  Serial.begin(115200);
  Serial.print("Hello");
}


int BluetoothData;
void loop() {
//  ax1 = digitalRead(ax1in);
  thr = digitalRead(thrin);
  if(1)
  {
    thr = digitalRead(thrin);
    digitalWrite(throut, thr);
    /*rol = digitalRead(rolin);
    digitalWrite(rolout, rol);
    pit = digitalRead(pitin);
    digitalWrite(pitout, pit);
    yaw = digitalRead(yawin);
    digitalWrite(yawout, yaw);*/
//    ax1 = digitalRead(ax1in);
//digitalWrite(ax1out, ax1);
  } else
  {
    thr= 1000;
    digitalWrite(throut, thr);
  }
}
