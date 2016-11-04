/*
  DIP Unmanned Aerial Vehicles- Arduino Nano Code

  created 14 Sept 2016
  Last modified 4 Nov 2016 3:11 pm
  
  This code is the on-board code for our Arduino Nano that
  acts as an intermediary between Naze32 Flight controller and
  transmitter (Turnigy 6X). 

  This code has four main functions :
  1. Reading multiple PWM (Pulse Width Modulation) signals from Turnigy 
  6x (Transmitter) with the help of Turnigy XR7000 receiver module
  2. Multiplexing received PWM signals into 1 PPM (Pulse Position Modulation)
  and outputting it to the Naze32 (flight controller)
  3. Computer Vision - Getting x,y and height value of detected landing pad
  4. Auto-landing algorithm with the concept of PID Tuning

  More can be found on
  https://github.com/endiliey/DIP-UAV

  Coder in-charge: Endilie Yacop Sucipto
  Contributor : Tee Chin Kiat and Dou Zi

*/

/*header files*/
#include <SPI.h>  // include Serial Peripheral Interface (SPI) library to enable SPI bus. note that by using this, pin 10,11,12,13 cannot be used
#include <Pixy.h> // include Pixy library to be able to communicate with Pixy camera
#include <PID_v1.h> // include PID (proportional, integral and derivative) library
#include <PinChangeInterrupt.h> // include arduino hardware interrupt (pin change interrupt) library

/*defining macros for easier configuration*/
#define CHANNEL_NUMBER 6  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default pwm value
#define SWITCH_OFF_VALUE 900 // set the default "off" switch value
#define SWITCH_ON_VALUE 1800 // set the default "on" switch value
#define FRAME_LENGTH 15000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length (spacing) between each pulse in PPM
#define THROTTLE_LANDING_VALUE 900 // set the throttle landing value
#define LANDING_HEIGHT_VALUE 300 // set the desired height for the the drone to land
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 8  //set PPM signal output pin on the arduino
#define LANDING_X_VALUE 10 // set the desired landing area (more of error-tolerance within X)
#define LANDING_Y_VALUE 10 // set the desired landing area (Y error-tolerance)
#define TURNING_SPEED 70 // set the desired maximum turning speed. The bigger it is, the faster the drones may turn/ move itself for autolanding
#define LANDING_SPEED 25 // set the desired landing speed (make sure its around 100. Bigger numbers = faster throttle down for autolanding
#define X_CENTER ((PIXY_MAX_X-PIXY_MIN_X)/2) // set the center "X" value of pixy       
#define Y_CENTER ((PIXY_MAX_Y-PIXY_MIN_Y)/2) // set the center "Y" value of pixy
#define ledPin 9 // set LED output pin on the arduino
#define P_CONSTANT 2.5 // set the proportional gain constant
#define I_CONSTANT 6 // set the integral gain constant
#define D_CONSTANT 0.8 // set the derivative gain constant



/*this array is the global variables needed to get and store PWM valuee*/
const byte channel_pin[] = {2,3,4,5,6,7}; // we use pin 2,3,4,5,6,7 for PWM input
volatile unsigned long rising_start[] = {0,0,0,0,0,0};
volatile long channel_length[] = {0,0,0,0,0,0}; //
volatile long yaw; // store received yaw PWM value
volatile long throttle; // store received throttle PWM value
volatile long roll; // store received roll PWM value
volatile long pitch; // store received pitch PWM value
volatile long ch5; // store received ch5 PWM value
volatile long ch6; // store received ch6 PWM value

/*these are the setup and global variables needed for Computer Vision & autoLanding algorithm */ 
Pixy pixy; // Create an instances of Pixy class named pixy
int x; // store x value from Pixy camera
int y; // store y value from Pixy camera
int height; // store y value from Pixy camera 
bool autoLand = false; // store boolean to check whether drone is in autoland mode
bool objectFound = false; // store boolean to check whether drone detect an object (the landing pad) 
int throttleLast; // store the last throttle value before going to autoLand mode 
bool descendBefore = false ; // boolean to check whether drone have descend before

/*this array holds the PWM values for the generated PPM signal 
index 0 = throttle, 1 = pitch, 2 = roll, 3 = yaw, 4 = ch5, 5 = ch6 */
int ppm[CHANNEL_NUMBER]; //

/*these are the global variables needed for our PID tuning*/
double rollInput; // Error of roll, in this case is "X", we want it to be equal to setpoint
double pitchInput; // Error of pitch, in this case is "Y", we want it to be equal to setpoint
double Kp = P_CONSTANT ; // Proportional gain constant
double Ki = I_CONSTANT; // Integral gain constant 
double Kd = D_CONSTANT; // Derivative gain constant
double Setpoint = 0; // desired setpoint (we want the drone to be exactly in the middle)
double rollOutput; // the PID adjusted PWM value of roll to achieve input to be equal to setpoint
double pitchOutput; // the PID adjusted PWM value of pitch to achieve input to be equal to setpoint
double rollFinal; // store CHANNEL_DEFAULT_VALUE + rollOutput PWM value
double pitchFinal; // store CHANNEL_DEFAULT_VALUE + pitchOutput PWM value

/*Construct two classes of PID and passing it several pointers */
PID rollPID(&rollInput, &rollOutput, &Setpoint, Kp, Ki, Kd, REVERSE); 
PID pitchPID(&pitchInput, &pitchOutput, &Setpoint, Kp, Ki, Kd, REVERSE  );


void setup() 
{ 
    /*initialize default ppm values */
    for(int i=0; i<CHANNEL_NUMBER; i++)
      {
          if ( i == 5 || i == 6){
            ppm[i] = SWITCH_OFF_VALUE;
          }
          else{
            ppm[i]= CHANNEL_DEFAULT_VALUE;  
          }
      }
          
    /*this is the initializer needed for PPM Signal Generation */
    pinMode(sigPin, OUTPUT);
    digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)

    /*This is the setup needed for Timer for PPM Generation, first we disable global interrupts,
    enable timer using Arduino register and enable global interrupts later on*/
    cli(); // disabling global interrupts to prevent error
    TCCR1A = 0; // set TCCR1A register to 0
    TCCR1B = 0; // set TCCR1B register to 0 (hence TCCR register is all 0)
    OCR1A = 100;  // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC (Clear Timer on Compare) mode
    TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei(); // enabling global interrupts
  
    /*seting pinMode of each pin that will be used to read PWM values */
    pinMode(channel_pin[0], INPUT);
    pinMode(channel_pin[1], INPUT);
    pinMode(channel_pin[2], INPUT);
    pinMode(channel_pin[3], INPUT);
    pinMode(channel_pin[4], INPUT);
    pinMode(channel_pin[5], INPUT);
    
    /*setup for PinChangeInterrupt library,it will attach interrupt to each pin as well as calling the PWM reading function*/   
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[0]), onRising0, CHANGE); 
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[1]), onRising1, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[2]), onRising2, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[3]), onRising3, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[4]), onRising4, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[5]), onRising5, CHANGE);

    /*Initialization for our pixy camera*/
    pixy.init();

    /*Setup needed for autolanding*/
    pinMode(ledPin, OUTPUT); // set the pin for led to be output
    rollPID.SetMode(AUTOMATIC); // set the mode of roll PID to be automatic
    pitchPID.SetMode(AUTOMATIC); // set the mode of pitch PID to be automatic 
    rollPID.SetSampleTime(5);  // set sample time of PID to be 5 ms
    pitchPID.SetSampleTime(5); // set sample time of PID to be 5 ms 
    rollPID.SetOutputLimits(-1 * TURNING_SPEED, TURNING_SPEED); // set max,min limit of roll PID
    pitchPID.SetOutputLimits(-1 * TURNING_SPEED, TURNING_SPEED); // set max,min limit of pitch PID

    /*This is only for debugging (using Serial port to see if it's working properly, it will be commented out later on
    Serial.begin(9600);
    Serial.println("Starting soon.. ");
    */  
}

  /*This is processPin function that get the PWM value from each pin by recording 
  from lowest value to its peak (the change is the pulse width value)*/
  void processPin(byte pin) {
    uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(channel_pin[pin]));
    
    if(trigger == RISING) {
       rising_start[pin] = micros();
       } 
    else if(trigger == FALLING) {
       channel_length[pin] = micros() - rising_start[pin];
       }
  }

  /*void functions to call processPin function*/
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


  //timer that will handle the PPM Signal Generation//
  ISR(TIMER1_COMPA_vect){  
  static boolean state = true;
  
  TCNT1 = 0;
  
    if (state) { 
        //start pulse
        digitalWrite(sigPin, onState);
        OCR1A =  PULSE_LENGTH * 2;
        state = false;
      } 
    else { 
        //end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;
      
        digitalWrite(sigPin, !onState);
        state = true;
    
                if(cur_chan_numb >= CHANNEL_NUMBER){
                  cur_chan_numb = 0;
                  calc_rest = calc_rest + PULSE_LENGTH;// 
                  OCR1A = (FRAME_LENGTH - calc_rest) * 2;
                  calc_rest = 0;
                }
                else {
                  OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
                  calc_rest = calc_rest + ppm[cur_chan_numb];
                  cur_chan_numb++;
                }     
       }
    }



void loop() {
  
  /*This store the PWM Value that we get to its respective variable (just for easier readability)*/
  throttle = channel_length[0];
  roll = channel_length[1];
  pitch = channel_length[2];
  yaw = channel_length[3];
  ch5 = channel_length[4];
  ch6 = channel_length[5];
  
     /*Check if pixy camera detect object (landing pad)*/
     if ( pixy.getBlocks() > 0 ){
      objectFound = true;
      digitalWrite(ledPin, HIGH); // turn on LED light
  
      /* Here we get the x,y, and height values of object (landing pad) detected */
      x = X_CENTER - pixy.blocks[0].x  ;
      y = (pixy.blocks[0].y) - Y_CENTER ;
      height = pixy.blocks[0].height;
  
      rollInput = (double)x; // set x as the roll PID input
      pitchInput = (double)y; // set y as the pitch PID input
      
     }  
     else {
      objectFound = false;
      digitalWrite(ledPin, LOW); // turn of LED light
     }  
  
     /*Check if switch for auto land mode is on */
     if ( ch5 <= CHANNEL_DEFAULT_VALUE){
      autoLand = false;
     }
     else if ( ch5 >= CHANNEL_DEFAULT_VALUE )
     {
          if (autoLand == false){
            autoLand = true; 
            throttleLast = throttle; // store last throttle value  
          }
          else {
            autoLand = true;
          } 
     }

  /*This part is only for debugging, we will comment this out later on */
            /*Serial.println("Detected: ");
            Serial.println("x    y    height  width");
            Serial.print(x); //this will be the x value of object detected by Pixy
            Serial.print("    ");
            Serial.print(y); // this will be the y value of object detected by Pixy
            Serial.print("|");
            Serial.println(height);// this will be the width of object detected by pixy
            Serial.print("|"); 
            */
  }    

   /*
    Here we modify ppm array (ppm[0] until ppm[5]) and set any channel to desired value
    (in our DIP case, we will use between 900 and 2000 for the four main channel, and 
    either 900/2000 for CH5 and CH6). 
    Timer running in the background will take care of the rest and automatically 
    generate PPM signal on output pin using values in ppm array
    */
  
   /*if auto land is false, arduino will just pass the received PWM value directly to the
   flight controller (manual mode)*/
   if ( autoLand == false ){  
      ppm[0] = throttle;
      ppm[1] = roll;
      ppm[2] = pitch;
      ppm[3] = yaw;
      ppm[4] = ch5;
      ppm[5] = ch6;
   }
   else
   {
    ppm[3] = yaw;
    ppm[4] = ch5;
    ppm[5] = ch6;
    Serial.println(height);

           /*if UAV is within desired height, it will throttle down and cut off power*/
           if (height >= LANDING_HEIGHT_VALUE){
                ppm[0] = THROTTLE_LANDING_VALUE;
                ppm[1] = CHANNEL_DEFAULT_VALUE;
                ppm[2] = CHANNEL_DEFAULT_VALUE;
              }
           else {                    
                /*PID Tuning to make UAV centered */
                rollPID.Compute(); // roll PID will compute adjusted PWM value and store it to roll Output
                pitchPID.Compute(); // pitch PID will compute the adjusted PWM value and store it to pitchOutput
                rollFinal = CHANNEL_DEFAULT_VALUE + rollOutput; // adjust final PWM value that will be ou
                pitchFinal = CHANNEL_DEFAULT_VALUE + pitchOutput; // adjust final PWM value that will be outputted

                /*To make sure our UAV is turning within our desired turning speed limit*/
                if (rollFinal > CHANNEL_DEFAULT_VALUE + TURNING_SPEED){
                  rollFinal = CHANNEL_DEFAULT_VALUE + TURNING_SPEED;
                }
                else if (rollFinal < CHANNEL_DEFAULT_VALUE - TURNING_SPEED){
                  rollFinal = CHANNEL_DEFAULT_VALUE - TURNING_SPEED;
                }
                 if (pitchFinal > CHANNEL_DEFAULT_VALUE + TURNING_SPEED){
                  pitchFinal = CHANNEL_DEFAULT_VALUE + TURNING_SPEED;
                }
                else if (pitchFinal < CHANNEL_DEFAULT_VALUE - TURNING_SPEED){
                  pitchFinal = CHANNEL_DEFAULT_VALUE - TURNING_SPEED;
                }
                    
                ppm[1] = (int)rollFinal; // Arduino output adjusted roll PWM value to flight controller
                ppm[2] = (int)pitchFinal; // Arduino output adjusted pitch PWM value to flight controller

                /*if copter is within desired area, it will descend down slowly */
                if ( x >= (-1 *LANDING_X_VALUE) && x <= LANDING_X_VALUE && y >= (-1 * LANDING_Y_VALUE) && y <= LANDING_Y_VALUE ){
                    descendBefore = true;
                    ppm[0] = throttleLast - LANDING_SPEED; // decrease throttle down abit to make it descend    
                 }
                else {
                /*if copter is not within desired area and it descend before, it will try to maintain its altitude*/
                     if (descendBefore == true){
                        ppm[0] = ppm[0] + LANDING_SPEED; // increase throttle value back to maintain its altitude
                        descendBefore = false;
                     }
                     else {
                        ppm[0] = throttleLast; // set throttle value to be last throttle value
                     }
                 
                }
   
              }
                          
  }

}
