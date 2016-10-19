//////////////////////CONFIGURATION///////////////////////////////
#define CHANNEL_NUMBER 6  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define SWITCH_OFF_VALUE 900 // set the default "off" switch value, in this case we use 900 because the Turnigy range from 900-2000
#define SWITCH_ON_VALUE 2000 // set the default "on" switch value, in this case we use 2000
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 9  //set PPM signal output pin on the arduino

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[CHANNEL_NUMBER];

void setup(){  

  //initiallize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++){
      if ( i == 5 || i == 6)
      {
      ppm[i] = SWITCH_OFF_VALUE;
      }
      else
      {
      ppm[i]= CHANNEL_DEFAULT_VALUE;  
      }
      
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

}

void loop(){
  
  /*
    Here we can modify ppm array and set any channel to value (in our DIP case, we will use between 900 and 2000 for the four main channel, and either 900/2000 for CH5 and CH6). 
    Timer running in the background will take care of the rest and automatically 
    generate PPM signal on output pin using values in ppm array
  */
  
}

ISR(TIMER1_COMPA_vect)
{  //leave this alone
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
