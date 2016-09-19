#include <SoftwareSerial.h>

#define AUX_SIGNAL_IN 0 // INTERRUPT 0 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
#define AUX_SIGNAL_IN_PIN 2 // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead

#define NEUTRAL_THROTTLE 1500 // this is the duration in microseconds of neutral throttle on an electric RC Car
#define REAL_THROTTLE_IN 4
#define REAL_THROTTLE_OUT 5
#define REAL_AX1_OUT 8

volatile int nThrottleIn = NEUTRAL_THROTTLE; // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartPeriod = 0; // set in the interrupt
volatile boolean bNewThrottleSignal = false; // set in the interrupt and read in the loop
// we could use nThrottleIn = 0 in loop instead of a separate variable, but using bNewThrottleSignal to indicate we have a new signal 
// is clearer for this first example
SoftwareSerial BTSerial(6, 7);
int btdata = 0;
int btdata1 =0  ;

void setup()
{
  // tell the Arduino we want the function calcInput to be called whenever INT0 (digital pin 2) changes from HIGH to LOW or LOW to HIGH
  // catching these changes will allow us to calculate how long the input pulse is
  attachInterrupt(AUX_SIGNAL_IN,calcInput,CHANGE);
  pinMode(REAL_THROTTLE_IN, INPUT);
  pinMode(REAL_THROTTLE_OUT, OUTPUT);
  pinMode(REAL_AX1_OUT, OUTPUT);
  Serial.begin(38400); 
  BTSerial.begin(38400);
  btdata1 = 0;
}

void loop()
{
   btdata = BTSerial.read();
   if(btdata==5)
   btdata1=5;//to make sure that it keep in autoland mode although it is not detected by sonar anymore
   
 //  Serial.println(btdata); 
 // if a new throttle signal has been measured, lets print the value to serial, if not our code could carry on with some other processing
 if(bNewThrottleSignal)
 {
   Serial.println(nThrottleIn);  

   // set this back to false when we have finished
   // with nThrottleIn, while true, calcInput will not update
   // nThrottleIn
   bNewThrottleSignal = false;
 }
 if (nThrottleIn < 1000 && nThrottleIn >900)//manual throttling 
 {
  digitalWrite(REAL_THROTTLE_OUT, digitalRead(REAL_THROTTLE_IN));
  digitalWrite(REAL_AX1_OUT, digitalRead(AUX_SIGNAL_IN_PIN));
  btdata1=0;
 }
 else 
 {
  if(btdata1==5) //if detected by sonar
  {
  digitalWrite(REAL_THROTTLE_OUT, HIGH);
  delayMicroseconds(1375);//CHANGE THIS IF YOU WANT TO CHANGE THE THROTTLE VALUE
  digitalWrite(REAL_THROTTLE_OUT, LOW);
  delay(10);
  }
  else// if inside autoland mode but did not detected 
  {
  digitalWrite(REAL_THROTTLE_OUT, digitalRead(REAL_THROTTLE_IN));
  digitalWrite(REAL_AX1_OUT, digitalRead(AUX_SIGNAL_IN_PIN));
  }
 }
} 

void calcInput()//calculate the PWM value
{
  // if the pin is high, its the start of an interrupt
  if(digitalRead(AUX_SIGNAL_IN_PIN) == HIGH)
  { 
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its 
    // easy to understand and works very well
    ulStartPeriod = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the 
    // start time ulStartPeriod from the current time returned by micros()
    if(ulStartPeriod && (bNewThrottleSignal == false))
    {
      nThrottleIn = (int)(micros() - ulStartPeriod);
      ulStartPeriod = 0;

      // tell loop we have a new signal on the throttle channel
      // we will not update nThrottleIn until loop sets
      // bNewThrottleSignal back to false
      bNewThrottleSignal = true;
    }
  }
}
