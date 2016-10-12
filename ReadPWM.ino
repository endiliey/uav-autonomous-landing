/* Programmed by Endilie
 *  Do not change anything if you don't know what you are doing
 *  11/10/16
 *  The purpose of this program is to get the PWM Value from Radio Controller
 */

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

void setup() {
  // put your setup code here, to run once:
pinMode(yawPin, INPUT);
pinMode(pitchPin, INPUT);
pinMode(throttlePin, INPUT);
pinMode(rollPin, INPUT);
pinMode(ch5Pin, INPUT);
pinMode(ch6Pin, INPUT);

Serial.begin(9600);
Serial.println("Starting soon.. ");
}

void loop() {

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
  
  delay(150);

}
