// -----------------------------------------------------------------------------
// Orrobot Attacker Game-on
// Version: 0.1
// Author: Jono / Aditya
// Date: Dec 03 2015
// 12/01 JS - Updated to use int for location and goallocation, not float
// 12/01 5pm JS - Updated to clean up (needs testing, especially RF)
// 12/01 9pm JS - added Red and Blue LED's for COMM test
// 12/03 10pm JS - added puck detection
// 12/04 4:30p JS - made ADC_init(), case/switch for buffer read, **created puck_detect() **TEST!!!
//                      **updated goal_calibrate to use internal quadrant** TEST!!
// 12/05 10am JS - #define RFOVERRIDE AND OVERSTATE
// 12/05 5pm JS - fixed puck detect
// 12/07 2:30 JS - starting with a very very simple version
// 12/07 5:30 JS - added to robocked/go2goal "maxangle" and "minangle" TEST AGAIN!
// -----------------------------------------------------------------------------

#define F_CPU 16000000UL

#include "m_robockey.h"
//Includes all the other necessary files

void init();
void ADC_init();
void send(int x, int y, int theta); //**REQUIRES that TXaddress, packet_length already set
//Sends x, y, theta location info via RF to other M2
void print_stuff(int*locate, int*goal_locate, int*ADC_read, int puckangle);
//~~Display Readings for location, goal location, ADC phototransistors and puckangle~~~~~~~~~


#define RFOVERRIDE GO2GOAL
                // override RF listening mode to start in desired state
#define OVERSTATE OFF  
                //change OVERSTATE to desired state, 0 means no OVERRIDE
#define USB_DEBUG ON
#define packet_length 10
#define channel 1
#define PUCK2GOAL 50

volatile char timer0_flag=0;
volatile char front_switch = 0;
volatile char back_switch = 0;
volatile char read_flag=0;      //Triggered whenever RF reads something
int TXaddress = 0x4F;
int RXaddress = 0x4C;
unsigned char buffer[packet_length];

//     ^ quad guide:
// II  |   I
//     +y
//<---- -+x-->
//     |
// III |  IV


int main()
{
    m_red(ON); //If only red is on, still initializing
    m_wait(50); //Wait to be sure no hands are above the mWii
    int locate[4];  //Stores X, Y, angle value for the bot location based on mWii readings
    int goal_locate[3]; //Stores X, Y, angle to the goal
    int centerpt[3] = {0, 0, 0};     //Stores X, Y, angle to center point
    int state = COMM;
    int blue_flag = 0;
    int ADC_read[8];    //Stores values from the ADC readings
    int ADC_track[3] = {0,0,0}; //[0] is adc counter, [1] is maximum ADC reading , [2] is ADC max location
    int puckangle = 0; int i; int j = 0;
    int quadrant = 0;
    
    init(); //INITIALIZE all timers, ADC setup, m_usb, m_wii, m_rf
    
    for (i=0; i < 6; i++){ //Initialize with a few mWii readings to filter out noise
        localize(locate);}
    quadrant = goalcalibrate(locate, goal_locate);	//Calibrate goal location  
    
        

    while(1)
    {
        //m_red(OFF); //m_red is turned off if the bot starts in the L side of the rink
        if(timer0_flag==1)
        {
            timer0_flag=0; //Reset timer flag
            m_green(TOGGLE);    //Toggle green as timer is run
            localize(locate);   //Run localize to determine the bot's location
			
            //send(locate[0], locate[1], locate[2]); //Sends location data to another M2
            
            if (read_flag)  //If RF signal is received, read to buffer
            {
                m_rf_read(buffer, packet_length); //Read from RF
                read_flag = 0;
                
                //Display the reading in serial
                m_usb_tx_string("Buffer reading is: "); m_usb_tx_int(buffer[0]); m_usb_tx_string(" \n");
                
                switch(buffer[0]){ //Respond to game controller commands
                    case COMM:
                        state = COMM; break;
                    case PLAY:
                        state = SEESPUCK; break;
                    case PAUSE:
                        state = PAUSE; break;}           
            }
            
            if (RFOVERRIDE){ //~~~~RF READING OVERRIDE~~~~Enabled in #define section up top
                state = RFOVERRIDE;} //Override state >> SEARCH1
            
            //Send quadrant location to the 7 Segment to display
             sevensegdispl(locate[3]);
            
            
            //~~Rerun goal calibration~~~~~~
            if (j < 50){ //Set up counter to reset every X milliseconds
                j++;}
            
            else   //Rerun goal calibrate every X milliseconds cycle
            {
                j=0; //Reset counter
                goal_locate[2]=180+atan2(locate[1]-goal_locate[1],locate[0]-goal_locate[0])*180/3.14;	
                //Calibrate goal location to be positive angle
			    
                print_stuff(locate, goal_locate, ADC_read, puckangle); //~~Display Readings~~~~~~~~~
               

                
                if (blue_flag){ //Called only in COMM mode
                    if (check(PORTD,5)){ //Toggle blue LED (D5)
                    clear(PORTD,5);}
                    else set(PORTD,5);
                    blue_flag++;} //Toggle BLUE led to indicate COMM mode
                if (blue_flag > 6){ //Turn off Blue LED after it blinks three times
                        blue_flag = 0;}
            } //~~END Goal calibration re-run
            
        } //~~~~~~END TIMER 0 LOOP~~~~~~~~~~~~~~
        
        //~~~~~PUCK DETECTION CODE~~~~~~~~~~~~~
         //NEW2 IR Readings:   (F5  F6  D7 D6 F0 F7 F1  F4) **On Attacker, F6 is not reading
        //NEW2 ANGLES: (0 +45 +90 +135 +180 -135 -135 -90 -45)
        puckangle = puck_detect(ADC_read, ADC_track, puckangle);
        
        //~~~SWITCH STATEMENT~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        switch(state){
            case COMM:  //Listen for signal sent by the game
                motor_stop();
                blue_flag = 1;
                sevensegdispl(8); //Number 8 means STOP!
                break;
                
            case PAUSE:  //Listen for signal sent by the game
                motor_stop();
                sevensegdispl(8); //Number 8 means STOP!
                break;
            
            case SEESPUCK:                 //Go directly to the puck
                go2puck(puckangle);
                /*if (ADC_read[0]>980){     //If center ADC reads HIGHEST VALUE, head to the goal
                        state=GO2GOAL;}
                if(ADC_track[1]<20){        //If ADC readings drop too low, search again
                        state=SEARCH1;}*/
                break;
                
            case GO2GOAL: //Head to the goal
                go2goal(locate,goal_locate[2]);
                sevensegdispl(9); //Number 9 means GO!
                //Going R? Move toward R goal, small turn radii
                //Going L? Move toward R goal, small turn radii
                break;
                
            /* SIMPLIFY FOR NOW
            case SEARCH1:       //Go to the center of the rink in search of the puck
                centerpt[2]=180+atan2(locate[1]-goal_locate[1],locate[0]-goal_locate[0])*180/3.14;	
                go2goal(locate,centerpt[2]); //Start moving toward the center to find the puck
                if (ADC_track[1]>100){  //If largest ADC value is above 100, go find the puck
                        state = SEESPUCK;}
                break;
                        
            case PUCK2GOAL:
                if (ADC_read[0]<700){   //if center ADC reading drops too low, go to puck
                    state = SEESPUCK;}
                go2goal(locate,goal_locate[2]);
                sevensegdispl(9); //Number 9 means GO!
                break;
            
                */ //END SIMPLIFATION
                
            default:
                    // search mode
                break;   
        }   //~~~~~~~~~~~~~~~~~~END SWITCH STATEMENT~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        if(OVERSTATE){  //Override code to keep the bot in one particular state
            state = OVERSTATE;}
        OCR1A = 170; OCR1B = 170;
    } //~~~~END MAIN WHILE LOOP~~~~~~~~~~~~~~~~~~~~~~~~
}

void init()
{
    int i;
    m_clockdivide(0);
    for (i=0; i < 10; i++)
    {
        buffer[i] = 0;
    }
    m_usb_init();
    m_wii_open();
    timer0_init();
    timer1_init();
    m_rf_open(channel, RXaddress, packet_length); //[WHY ISNT THIS WORKING?!]
    
    sei();  //Enable interrupts!!
    
    set(DDRC,7);    //Set pin C7 for L motor direction as output
    set(DDRC,6);    //Set pin C6 for R motor direction as output
    
    set(DDRD,4);    //Set pin D4 for red LED as output
    set(DDRD,5);    //Set pin D5 for blue LED as output
    
    ADC_init(); //disable JTAG, set up ADC, disable digital input for D6-D7, F0-F7
    
}

void ADC_init()
{
    //~~~ADC INIT~~~~
        
    m_disableJTAG(); //Necessary to read pins F4 - F7
    
    clear(ADMUX,REFS1); //Set to internal VCC
    set(ADMUX,REFS0);   // 0 1

    set(ADCSRA,ADPS2);      // Prescaler set to /128 (for 16MHz clock, 125 kHz)
    set(ADCSRA,ADPS1);    // 1 1 1
    set(ADCSRA,ADPS0);

    clear(ADCSRA,ADATE); //DISABLE FREE RUNNING MODE!
    
    set(DIDR2,ADC10D);       //Disables digital input at PIN D7 (ADC10)
    set(DIDR2,ADC9D);       //Disables digital input at PIN D6 (ADC9)
    set(DIDR0,ADC0D);       //Disables digital input at PIN F0 (ADC0)
    set(DIDR0,ADC1D);       //Disables digital input at PIN F1 (ADC1)
    set(DIDR0,ADC4D);       //Disables digital input at PIN F4 (ADC4)
    set(DIDR0,ADC5D);       //Disables digital input at PIN F5 (ADC5)
    set(DIDR0,ADC6D);       //Disables digital input at PIN F6 (ADC6)
    set(DIDR0,ADC7D);       //Disables digital input at PIN F7 (ADC7)
    
    D6_read();  //Starts with D6 reading
    
    set(ADCSRA,ADEN);       //Enable conversions
    set(ADCSRA,ADSC);       //Start conversions 
    //~~ADC NOW INITIALIZED
}

//~~Display Readings for location, goal location, ADC phototransistors and puckangle~~~~~~~~~
void print_stuff(int*locate, int*goal_locate, int*ADC_read, int puckangle)
{
    int i;
    m_usb_tx_string("Present Loc (X,Y, theta): ("); 
    m_usb_tx_int(locate[0]); m_usb_tx_string(", "); m_usb_tx_int(locate[1]); 
    m_usb_tx_string(", "); m_usb_tx_int(locate[2]); m_usb_tx_string(") \n");
    m_usb_tx_string("Goal angle (X,Y, theta): ("); m_usb_tx_int(goal_locate[0]);        
    m_usb_tx_string(", "); m_usb_tx_int(goal_locate[1]);
    m_usb_tx_string(", "); m_usb_tx_int(goal_locate[2]);
    m_usb_tx_string(") \n"); m_usb_tx_string("Present angle is: "); 
    m_usb_tx_int(locate[2]); m_usb_tx_string("\n");
    m_usb_tx_string("Goal angle is: "); 
    m_usb_tx_int(goal_locate[2]);m_usb_tx_string("\n");
    m_usb_tx_string("Difference between angles is: "); 
    m_usb_tx_int(goal_locate[2]-locate[2]); m_usb_tx_string(" \n");

    m_usb_tx_string("IR Readings:   (F5  F6  D7  D6  F0  F7  F1  F4) \n"); 
    m_usb_tx_string("ADC reading is: ("); 
    for (i=0; i < 8; i++){ //Print out all 8 ADC readings
        m_usb_tx_int(ADC_read[i]); m_usb_tx_string("  "); }
    m_usb_tx_string(")\n");
    m_usb_tx_string("Puck angle calc is:  ");  m_usb_tx_int(puckangle); m_usb_tx_string("\n");
}
