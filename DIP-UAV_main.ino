//////////////////////READ THIS///////////////////////////////
 /* Do not change anything if you don't know what you are doing
 *  31/10/16
 *  This is the on-board code for our drone
 *  It have 3 functions : getting PWM value from receiver (6 channels input), getting object coordinates (Computer Vision input) , and outputting PPM Signal to flight controller (1 output only) with some algorithm to make sure it can autoland / controlled manually
 *  Designed for DIP Project B Team 
 *  Coder In-charge : Endilie Yacop Sucipto
 *  Contributor : Tee Chin Kiat, Dou Zi
 */ 
//////////////////////CONFIGURATION///////////////////////////////
#define CHANNEL_NUMBER 6  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define SWITCH_OFF_VALUE 1500 // set the default "off" switch value, in this case we use 900 because the Turnigy range from 900-2000
#define SWITCH_ON_VALUE 1800 // set the default "on" switch value, in this case we use 2000
#define FRAME_LENGTH 15000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define THROTTLE_LANDING_VALUE 900 // please change this with the value of throttle when its going to DESCEND (THE 900 VALUE IS ASSUMPTION BY ENDI)
#define LANDING_HEIGHT_VALUE 75 // please change this with the value of object height when the drones are very near with the object (like ~20cm distant apart)
#define LANDING_WIDTH_VALUE 200 // please change this with the value of object width when the drones are very near with the object (like ~20cm distant apart)
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 8  //set PPM signal output pin on the arduino
#define LANDING_X_VALUE 30 // set the desired landing area (more of error-tolerance within X)
#define LANDING_Y_VALUE 30 // set the desired landing area (Y error-tolerance)
#define TURNING_SPEED 50 // set the desired turning speed. The bigger it is, the faster the drones turn/ move itself for autolanding
#define LANDING_SPEED 25 // set the desired landing speed (make sure its around 100. Bigger numbers = faster throttle down for autolanding
#include <SPI.h>  // include SPI interface, note that by using this, pin 10,11,12,13 cannot be used (take note for our project seriously)
#include <Pixy.h> // include Pixy header files, this is the basic header to get Pixy method functioning
#include <PinChangeInterrupt.h> // include hardware interrupt libraries to get PWM input without lot of delay (usually we use PulseIn which gives lot of delay)
#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)
#define POLARITY -1

Pixy pixy; // Create an instances of Pixy class named pixy

class PWMLoop
{
 public:
 PWMLoop(int32_t pgain, int32_t dgain);

 void update(int32_t error);

 int32_t m_pos;
 int32_t m_prevError;
 int32_t m_pgain;
 int32_t m_dgain;
};

PWMLoop rollLoop(500, 500);
PWMLoop pitchLoop(500, 500);

PWMLoop::PWMLoop(int32_t pgain, int32_t dgain)
{
  m_pos = 1500;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void PWMLoop::update(int32_t error)
{
  long int vel;
  if (m_prevError!=0x80000000)
  {  
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    m_pos += vel;
    if (m_pos> 1600) 
      m_pos = 1600; 
    else if (m_pos < 1400) 
      m_pos = 1400;
  }
  m_prevError = error;
}

/*this array is the global variable needed to get and store PWM valuee*/
const byte channel_pin[] = {2,3,4,5,6,7}; // we use pin 2,3,4,5,6,7 for PWM input
volatile unsigned long rising_start[] = {0,0,0,0,0,0};
volatile long channel_length[] = {0,0,0,0,0,0};

/*this array holds the x,y,height and width value of the landing platform detected by our camera as well as the autoLand & objectFound boolean variable */
int x;
int y;
int height;
int width;
bool autoLand = false;
bool objectFound = false;
int throttleLast;
bool descendBefore = false ;
/*To make things readable, we create an arrays to hold the yaw, throttle, pitch, roll and ch5,ch6 PWM Value */
volatile long yaw;
volatile long throttle;
volatile long roll;
volatile long pitch;
volatile long ch5;
volatile long ch6;

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[CHANNEL_NUMBER];

void setup() 
{
    //initialize default ppm values
    for(int i=0; i<CHANNEL_NUMBER; i++)
    {
        if ( i == 5 || i == 6)
        {
          ppm[i] = SWITCH_OFF_VALUE;
        }
        else
        {
          ppm[i]= CHANNEL_DEFAULT_VALUE;  
        }
    }
    
    /*this is the initializer needed for PPM Signal Generation */
    pinMode(sigPin, OUTPUT);
    digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)

    /*This is the setup needed for Timer for PPM Generation, first we disable global interrupts, enable timer using Arduino register and enable global interrupts later on*/
    cli(); // disabling global interrupts
    TCCR1A = 0; // set TCCR1A register to 0
    TCCR1B = 0; // set TCCR1B register to 0 (hence TCCR register is all 0)
    OCR1A = 100;  // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC (Clear Timer on Compare) mode
    TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei(); // enabling global interrupts
  
    /*This is all the initializer needed for reading PWM Value from Receiver (changing pin Mode and attaching Interrupt mode on each pin)*/
    pinMode(channel_pin[0], INPUT);
    pinMode(channel_pin[1], INPUT);
    pinMode(channel_pin[2], INPUT);
    pinMode(channel_pin[3], INPUT);
    pinMode(channel_pin[4], INPUT);
    pinMode(channel_pin[5], INPUT);
  
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[0]), onRising0, CHANGE); //this will attach interrupt to the pin as well as calling the PWM reading function
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[1]), onRising1, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[2]), onRising2, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[3]), onRising3, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[4]), onRising4, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[5]), onRising5, CHANGE);

    /* Initialization for our pixy camera*/
    pixy.init();

    /*This is only for debugging (using Serial port to see if it's working properly, it will be commented out later on*/
    Serial.begin(9600);
    Serial.println("Starting soon.. ");
}

  /*This is processPin function that get the PWM value from each pin by recording from lowest value to its peak (the change is the pulse width value)*/
  void processPin(byte pin)
  {
    uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(channel_pin[pin]));
  
      if(trigger == RISING) 
        {
          rising_start[pin] = micros();
        } 
      else if(trigger == FALLING)
        {
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


//Leave this alone, it's the timer that will handle the PPM Signal Generation//
ISR(TIMER1_COMPA_vect)
{  
  static boolean state = true;
  
  TCNT1 = 0;
  
    if (state) 
      { 
        //start pulse
        digitalWrite(sigPin, onState);
        OCR1A = PULSE_LENGTH * 2;
        state = false;
      } 
    else
      { 
        //end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;
      
        digitalWrite(sigPin, !onState);
        state = true;
    
                if(cur_chan_numb >= CHANNEL_NUMBER)
                {
                  cur_chan_numb = 0;
                  calc_rest = calc_rest + PULSE_LENGTH;// 
                  OCR1A = (FRAME_LENGTH - calc_rest) * 2;
                  calc_rest = 0;
                }
                else
                {
                  OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
                  calc_rest = calc_rest + ppm[cur_chan_numb];
                  cur_chan_numb++;
                }     
      }
}

void loop() {


  /*This store the PWM Value that we get to its respective variable (just for easier readability), rather than having channel_length[0], we use yaw instead*/

  throttle = channel_length[0];
  roll = channel_length[1];
  pitch = channel_length[2];
  yaw = channel_length[3];
  ch5 = channel_length[4];
  ch6 = channel_length[5];
  int32_t rollError, pitchError;
  /*This is only for debugging, it will be commented out later on*/
 /* Serial.print(throttle); //yaw
  Serial.print(" | ");
  Serial.print(roll); 
  Serial.print(" | ");
  Serial.print(pitch); 
  Serial.print(" | ");
  Serial.print(yaw); 
  Serial.print(" | ");
  Serial.print(ch5); 
  Serial.print(" | ");
  Serial.print(ch6);
  Serial.println(""); 
  */
  
         if ( pixy.getBlocks() > 0 )
         {
          objectFound = true;
          //Serial.println("Object initialization found");
         }  
         else
         {
          objectFound = false;
          //Serial.println("Object not found");
         }  
 
         if ( ch5 <= SWITCH_OFF_VALUE)
         {
          autoLand = false;
         }
         else if ( ch5 >= SWITCH_ON_VALUE )
         {
              if (autoLand == false)
              {
                autoLand = true; 
                throttleLast = throttle;
              }
              else
              {
                autoLand = true;
              } 
         }
 
  /* Here we get the x,y, height and coordinate values of object detected */
  if (objectFound == true && autoLand == true)
  {

        x = (pixy.blocks[0].x) - 160 ;
        y = -1 * ((pixy.blocks[0].y) - 100) ;
        height = pixy.blocks[0].height;
        width = pixy.blocks[0].width;

  /*This part is only for debugging, we will comment this out later on */
/* Serial.println("Detected: ");
            Serial.println("x    y    height  width");
            Serial.print(x); //this will be the x value of object detected by Pixy
            Serial.print("    ");
            Serial.print(y); // this will be the y value of object detected by Pixy
            Serial.print("    ");
            Serial.print(height);// this will be the width of object detected by pixy
            Serial.print("    "); 
            Serial.println(width); // this will be the height of object detected by pixy
            */
 
  }    

   /*
    Here we can modify ppm array (ppm[0] until ppm[5]) and set any channel to value (in our DIP case, we will use between 900 and 2000 for the four main channel, and either 900/2000 for CH5 and CH6). 
    Timer running in the background will take care of the rest and automatically 
    generate PPM signal on output pin using values in ppm array
    */

   if ( objectFound == false )
   {
      Serial.println("Object not found, manual mode");
      ppm[0] = throttle;
      ppm[1] = roll;
      ppm[2] = pitch;
      ppm[3] = yaw;
      ppm[4] = ch5;
      ppm[5] = ch6;
   }
   else if ( autoLand == true && objectFound == true)
   {
         ppm[3] = CHANNEL_DEFAULT_VALUE;
         ppm[4] = ch5;
         ppm[5] = ch6;
        
    
      if ( x >= (-1 *LANDING_X_VALUE) && x <= LANDING_X_VALUE && y >= (-1 * LANDING_Y_VALUE) && y <= LANDING_Y_VALUE )
      {
          descendBefore = true;
          ppm[2] = CHANNEL_DEFAULT_VALUE;
          ppm[1] = CHANNEL_DEFAULT_VALUE;
          Serial.println("Will land with throttle speed of");
          if (height >= LANDING_HEIGHT_VALUE)
          {
            ppm[0] = THROTTLE_LANDING_VALUE;
          }
          else
          {
            ppm[0] = throttleLast - LANDING_SPEED; 
          }
          
          Serial.println(ppm[0]);

      }
      else 
      {
           if (descendBefore == true)
           {
            ppm[0] = throttleLast + LANDING_SPEED;
            descendBefore = false;
           }
           else
           {
            ppm[0] = throttleLast;
           }
            
           rollError = POLARITY * (X_CENTER-pixy.blocks[0].x);
           pitchError = POLARITY * (pixy.blocks[0].y - Y_CENTER);

           rollLoop.update(rollError);
           pitchLoop.update(pitchError);

           ppm[1] = rollLoop.m_pos;
           ppm[2] = pitchLoop.m_pos;
           
       /*   if (x >= LANDING_X_VALUE)
          {
            ppm[1] = CHANNEL_DEFAULT_VALUE - TURNING_SPEED;
            Serial.println("Will roll to the left");
            Serial.println(ppm[1]);
          }
          else if (x <= (-1 * LANDING_X_VALUE) )
          {
            ppm[1] = CHANNEL_DEFAULT_VALUE + TURNING_SPEED;
            Serial.println("Will roll to the right");
            Serial.println(ppm[1]);
          }
          else
          {
            ppm[1] = CHANNEL_DEFAULT_VALUE;
          }
  
          
          if (y >= LANDING_Y_VALUE)
          {
             ppm[2] = CHANNEL_DEFAULT_VALUE - TURNING_SPEED; 
             Serial.println("Will pitch to bottom");  
             Serial.println(ppm[2]);  
          }
          else if (y <= (-1 * LANDING_Y_VALUE) )
          {
            ppm[2] = CHANNEL_DEFAULT_VALUE + TURNING_SPEED;
            Serial.println("Will pitch to up");
            Serial.println(ppm[2]);
          }
          else
          {
            ppm[2] = CHANNEL_DEFAULT_VALUE;
            Serial.println(ppm[2]);
          }
        */         
       }
   
   }

}
