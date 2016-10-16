/* Programmed by Endilie
 *  Do not change anything if you don't know what you are doing
 *  11/10/16
 *  The purpose of this program is to get the PWM Value from Radio Controller
 */

#include <PinChangeInterrupt.h>

const byte channel_pin[] = {2,3,4,5,6,7};
volatile unsigned long rising_start[] = {0,0,0,0,0,0};
volatile long channel_length[] = {0,0,0,0,0,0};

void setup() {
  Serial.begin(9600);

  pinMode(channel_pin[0], INPUT);
  pinMode(channel_pin[1], INPUT);
  pinMode(channel_pin[2], INPUT);
  pinMode(channel_pin[3], INPUT);
  pinMode(channel_pin[4], INPUT);
  pinMode(channel_pin[5], INPUT);
  
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[0]), onRising0, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[1]), onRising1, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[2]), onRising2, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[3]), onRising3, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[4]), onRising4, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[5]), onRising5, CHANGE);
}

void processPin(byte pin) {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(channel_pin[pin]));

  if(trigger == RISING) {
    rising_start[pin] = micros();
  } else if(trigger == FALLING) {
    channel_length[pin] = micros() - rising_start[pin];
  }
}

void onRising0(void) {
  processPin(0);
}

void onRising1(void) {
  processPin(1);
}

void onRising2(void) {
  processPin(2);
}

void onRising3(void) {
  processPin(3);
}

void onRising4(void) {
  processPin(4);
}

void onRising5(void) {
  processPin(5);
}

void loop() {
  //Serial.println("Yaw | Pitch | Ch6 | Roll | Ch5 | Throttle |");
  Serial.print(channel_length[0]); //yaw
  Serial.print(" | ");
  Serial.print(channel_length[1]); //pitch
  Serial.print(" | ");
  Serial.print(channel_length[2]); // ch6
  Serial.print(" | ");
  Serial.print(channel_length[3]); // roll
  Serial.print(" | ");
  Serial.print(channel_length[4]); // ch5
  Serial.print(" | ");
  Serial.print(channel_length[5]); //throttle
  Serial.println(""); 
  
  //delay(100);
}
