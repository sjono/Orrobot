// -----------------------------------------------------------------------------
// Orrobot Attacker Game-on
// Version: 0.1
// Author: Jono / Aditya
// Date: Dec 03 2015
// 12/08 5p JS - working on motor_run again
// 12/08 8:30p JS - updated for tonights pool match
// 12/09 9:40a JS - testing SEESPUCK >> PUCK2GOAL, reading cases with 7SEGMENT
// 12/09 10a JS - Sees puck and switches to PUCK2GOAL, but turns TOO SLOWLY!
// 12/09 10:30 JS - fixing motor_pd, to use distance_error >> not fast enough to turn back
// 12/09 1:31 JS - included puck_detect locally (specific to attacker)
// 12/09 1:45 JS - Rolling back motor_pd() version to simplify for pool play
// 12/09 2:30 JS - updated version for Pool Play (solenoid fires in Puck2Goal)
// 12/09 4:14p JS - removed OCR1A & OCR1B override - DUH! >> hopeless version
// 12/09 9:50p JS - using PD code from 12/08
// 12/09 11:14 JS - setting LED color with #define COLOR
// -----------------------------------------------------------------------------

#define F_CPU 16000000UL

#include "m_robockey.h"
//Includes all the other necessary files

void init();
void ADC_init();
void motor_run(int* location, int* goallocation, int motordir);
//Runs the bot toward the goal location based upon the angle
int motor_pd(int*locate, int*goal_locate, int*locate_old);
//Looks at the location with respect to the goal location and rotates based on PD feedback (12/09 01h21 version JS)
void go2pduck(int puckangle, int* locate, int*locate_old);
//Looks at puck angle and determines where to go (12/09 09h21 version JS)
void send(int x, int y, int theta); //**REQUIRES that TXaddress, packet_length already set
//Sends x, y, theta location info via RF to other M2
void print_stuff(int*locate, int*goal_locate, int*ADC_read, int puckangle, int state);
//~~Display Readings for location, goal location, ADC phototransistors and puckangle~~~~~~~~~

int puck_detect(int* ADC_read, int* ADC_track, int puckangle); //SPECIFIC TO ATTACKER
//Reads from ADC and stores into ADC_read, after 8 readings, finds the max value and stores in ADC_track


#define RFOVERRIDE OFF
                // override RF listening mode to start in desired state
#define OVERSTATE OFF  
                //change OVERSTATE to desired state, 0 means no OVERRIDE
#define USB_DEBUG ON
#define packet_length 10
#define channel 1
#define DUTYMAX 170

volatile char timer0_flag=0;
volatile char front_switch = 0;
volatile char back_switch = 0;
volatile char read_flag=0;      //Triggered whenever RF reads something
volatile int frontswitch=0;    //Front switch flag
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
    int color = BLUE;
    m_red(ON); //If only red is on, still initializing
    m_wait(50); //Wait to be sure no hands are above the mWii
    int locate[4];  //Stores X, Y, angle value for the bot location based on mWii readings
    int locate_old[3]; //Stores old location values
    int goal_locate[3]; //Stores X, Y, angle to the goal
    int centerpt[3] = {0, 0, 0};     //Stores X, Y, angle to center point
    int clearpt[4] = {-150, -20, 0, 0};     //Clearing start point : make sure this is a good location!
    int state = PAUSE;          //INTIALIZE in PAUSE mode
    int blue_flag = 0;
    int ADC_read[8];    //Stores values from the ADC readings
    int ADC_track[4] = {0,0,0,0}; //[0] is adc counter, [1] is maximum ADC reading , [2] is ADC max location, [3] ADC count
    int puckangle = 0; int i; int j = 0;
    int quadrant = 0;
    int motordir = 0;     //Motor driving counter
    
    init(); //INITIALIZE all timers, ADC setup, m_usb, m_wii, m_rf
    
    for (i=0; i < 6; i++){ //Initialize with a few mWii readings to filter out noise
        localize(locate);}
    quadrant = goalcalibrate(locate, goal_locate);	//Calibrate goal location  
    
    if (RFOVERRIDE){ //~~~~RF READING OVERRIDE~~~~Enabled in #define section up top
            state = RFOVERRIDE;} //Override state >> SEARCH1
           
    while(1)
    {
        m_red(OFF); //m_red is turned off after intialization
        if(timer0_flag==1)
        {
            timer0_flag=0; //Reset timer flag
            m_green(TOGGLE);    //Toggle green as timer is run
            for (i=0; i < 4; i++){ //Store old localization values
                locate_old[i] = locate[i];}
            localize(locate);//Run localize to determine the bot's location

			
            //send(locate[0], locate[1], locate[2]); //Sends location data to another M2
            
            //~~~~~~~~TESTING CODE INSIDE TIMER0~~~~~
            if (!check(PINB,7))
            {
                frontswitch+=1;
                m_usb_tx_string("SWITCH IS PRESSED, COUNT IS: \n"); m_usb_tx_int(frontswitch);   
                m_usb_tx_string("\n");   
            }
            else frontswitch = 0;

            //~~~~~~~~END TESTING CODE INSIDE TIMER0~~~~~
            if (read_flag)  //If RF signal is received, read to buffer
            {
                m_rf_read(buffer, packet_length); //Read from RF
                read_flag = 0;
                
                //Display the reading in serial
                m_usb_tx_string("Buffer reading is: "); m_usb_tx_int(buffer[0]); m_usb_tx_string(" \n");
                
                switch(buffer[0]){ //Respond to game controller commands
                    case COMM:
                        state = COMM; blue_flag = 1; break;
                    case PLAY:
                        state = SEESPUCK; break;
                    case PAUSE:
                        state = PAUSE; break;}           
            }
            
                        
            //~~Rerun goal calibration~~~~~~
            if (j < 50){ //Set up counter to reset every X milliseconds
                j++;}
            
            else   //Rerun goal calibrate every X milliseconds cycle
            {
                j=0; //Reset counter
                goal_locate[2]=180+atan2(locate[1]-goal_locate[1],locate[0]-goal_locate[0])*180/3.14;	
                //Calibrate goal location to be positive angle
                clearpt[2] = 180+atan2(locate[1]-clearpt[1],locate[0]-clearpt[0])*180/3.14;
                centerpt[2] = 180+atan2(locate[1]-centerpt[1],locate[0]-centerpt[0])*180/3.14;
                //~~Display Readings for location, goal location, ADC phototransistors and puckangle~~~~~~~~~
                print_stuff(locate, goal_locate, ADC_read, puckangle, state);
                               
                if (blue_flag) //Called only in COMM mode
                {
                    if (color == BLUE)
                    {                        
                        if (check(PORTD,4)){ //Toggle blue LED (D4)
                        clear(PORTD,4);}
                        else set(PORTD,4);
                        blue_flag++; //Toggle BLUE led to indicate COMM mode
                    }
                    if (color == RED)
                    {
                        if (check(PORTD,5)){ //Toggle red LED (D5)
                        clear(PORTD,5);}
                        else set(PORTD,5);
                        blue_flag++;
                    }
                } // END blue_flag
                else
                {
                    if (color == BLUE){ //Leaves LED on after blinking
                        set(PORTD,4);}
                    if (color == RED){
                        set(PORTD,5);}
                }
                
                //Resets the solenoid if it has been fired ~~~~~~~~~~~~
                if (check(PORTB,4)){ 
                    clear(PORTB,4);}
                //else set(PORTB,4); //This would trigger the solenoid to fire whenever goal calibrate runs
                   
            }   //~~END Goal calibration re-run
            
        } //~~~~~~END TIMER 0 LOOP~~~~~~~~~~~~~~
        
        //~~~~~PUCK DETECTION CODE~~~~~~~~~~~~~
         //NEW2 IR Readings:   (F5  F6  D7 D6 F0 F7 F1  F4) **On Attacker, F6 is not reading
        //NEW2 ANGLES: (0 +45 +90 +135 +180 -135 -135 -90 -45)
        puckangle = puck_detect(ADC_read, ADC_track, puckangle);
        
        //~~~SWITCH STATEMENT~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        switch(state){
            case COMM:  //Listen for signal sent by the game
                motor_stop();
                if (blue_flag > 6){
                    blue_flag =0;
                    state = PAUSE;}
                sevensegdispl(8); //Number 8 means STOP!
                break;
                
            case PAUSE:  //Listen for signal sent by the game
                motor_stop();
                sevensegdispl(8); //Number 8 means STOP!
                break;
                
            case SEARCH1:       //Go to the center of the rink in search of the puck
                centerpt[2]=180+atan2(locate[1]-goal_locate[1],locate[0]-goal_locate[0])*180/3.14;	
                go2goal(locate,centerpt[2]); //Start moving toward the center to find the puck
                if (ADC_track[1]>100){  //If largest ADC value is above 100, go find the puck
                        state = SEESPUCK;}
                break;
            
            case GO2GOAL: //Head to the goal
                motor_pd(locate, goal_locate, locate_old); //TURNED OFF FOR TESTIN!!!!
                set(DDRB,5);set(DDRB,6);    //Make sure motors are on
                set(PORTC,6); set(PORTC,7); //Currently only drives forward
                sevensegdispl(9); //Number 9 means GO!
                //Going R? Move toward R goal, small turn radii
                //Going L? Move toward R goal, small turn radii
                break;
    
            case SEESPUCK:                 //Go directly to the puck
                go2puck(puckangle);
                
                if(frontswitch > 20){
                        state = PUCK2GOAL;}
                if(ADC_track[1]>1000){ //Count if ADC readings are LARGE (means close to puck)
                        ADC_track[3]+=1;}
                if(ADC_track[3] > 500){ //DIAL THIS IN ~~ (100 = too soon), 500?
                        state = PUCK2GOAL;
                        ADC_track[3] = 0;} //Reset ADC counter
                /*if(ADC_track[1]<20){        //If ADC readings drop too low, search again
                        ADC_track[3]-=1;}
                if(ADC_track[3] < -50){
                    state=SEARCH1;
                    ADC_track[3] = 0;} //Reset ADC counter      */
                /*if (frontswitch > 200){     //When the puck has been on the bot for a while
                    set(PORTB,4);}            //Fire solenoid (moved to the Puck2Goal state)*/           
                sevensegdispl(4); //#4 Looks like a lower case c
                OCR1A = 180, OCR1B = 180;
                break;            
                
            case PUCK2GOAL:
                motor_pd(locate,goal_locate, locate_old); //TURNED OFF FOR TESTIN!!!!
                set(DDRB,5);set(DDRB,6);    //Make sure motors are on
                set(PORTC,6); set(PORTC,7); //Currently only drives forward                
                if (ADC_read[0]<700){   //if center ADC reading drops too low, go to puck ~~ lower threshhold?!
                    ADC_track[3] -= 1;}
                if (ADC_track[3] < -100){
                    state = SEESPUCK;
                    ADC_track[3] = 0;}      //Reset ADC counter                go2goal(locate,goal_locate[2]);
                if (frontswitch > 200){     //When the puck has been on the bot for a while
                    set(PORTB,4);}            //Fire solenoid (12/09 10a - fires too soon!)

                sevensegdispl(2); //#2 looks like a 9
                break;
                
            case CLEARGOAL: //Go towards GOAL and CLEAR!
                if (motor_pd(locate, clearpt, locate_old) < 20){ //FIRST GO TO CLEAR POINT
                    clearpt[3] += 1;
                    sevensegdispl(2);} //#2 looks like a 9
                if (clearpt[3] > 100){      //NEXT SWEEP!
                    clearpt[1] = -clearpt[0]; clearpt[0]+=10; clearpt[2] = 100; //Go to opposite Y-value
                    clearpt[3] = 0;}
                if(clearpt[1] > 0){
                    sevensegdispl(2);} //#2 looks like a 9
                else sevensegdispl(4); //#4 Looks like a lower case c;
                set(DDRB,5);set(DDRB,6);    //Make sure motors are on
                set(PORTC,6); set(PORTC,7); //Currently only drives forward
                break;
            default:
                
                    // search mode
                break;   
        }   //~~~~~~~~~~~~~~~~~~END SWITCH STATEMENT~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        if(OVERSTATE){  //Override code to keep the bot in one particular state
            state = OVERSTATE;}
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
    
    set(DDRB,4); //Set pin for Solenoid 1 & 2 (front facing)
    clear(PORTB,4); //Keep Solenoid pin LOW
        
    clear(DDRB,7); //Set pin for back switch (B7 = PCINT7)
    set(PORTB,7); //Enable pullup resistor on E6
    
    //set(PCICR,PCIE0); //
    //set(PCMSK0,PCINT7);
    
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

//****************MOTOR RUN FUNCTION********************************//
void motor_run(int*location, int*goallocation, int motordir)
{ //Looks at the location with respect to the goal location and rotates if necesary
    int angle_diff;
    int j;
    
    int variation = 30; //variation in degrees that we accept
    
    angle_diff = goallocation[2]-(location[2]);
    while (angle_diff > 180){
        angle_diff -= 360;}
    while (angle_diff < -180){
        angle_diff += 360;}

    int quad = location[3];
      
    clear(DDRB,5);
    clear(DDRB,6);
    for(j=0;j<30000;j++); //Pause for a certain amount of time
    
    if((angle_diff>(-variation)) && (angle_diff<variation)) //If angle is acceptable tolerance, keep going straight
    {
        set(PORTC,6); // GO STRAIGHT!!
        set(PORTC,7);  
    }

    
    else{    
        //m_usb_tx_string("Angle diff is"); m_usb_tx_int(angle_diff); m_usb_tx_string("\n");   
        if (angle_diff<(-variation)){
           // m_usb_tx_string("GO COUNTER-CLOCKWISE \n");   
        //    clear(PORTC,6); // Counter-clockwise rotation
        //    set(PORTC,7);
//~~~~~try it backwards~~~~~~
            set(PORTC,6); // Clockwise rotation
            clear(PORTC,7);

        }

        if (angle_diff>variation){
            //m_usb_tx_string("GO CLOCKWISE \n");   
            //set(PORTC,6); // Clockwise rotation
            //clear(PORTC,7);
//~~~~~try it backwards~~~~~~
            clear(PORTC,6); // Counter-clockwise rotation
            set(PORTC,7);

        }
            
    }
        

    clear(DDRB,5);
    clear(DDRB,6);
    OCR1A = 130;
    OCR1B = 130;

}
int motor_pd(int*locate, int*goal_locate, int*locate_old)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~ PD CODE TESTIN~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
{ //Looks at the location with respect to the goal location and rotates if necesary
    
    //~~~VARIABLES: 
    //location_pd[0] = angular feedback L wheel, location_pd[1] = angular feedback R wheel;
    //location_pd[2] = directional feedback BOTH wheel

//~~~~~~~~~~~STEP1: compare angles~~~~~~~~~~~~~~~~~~
    float Kp = 2; float Kd = 0.5; 
    int angle_pd = 0; int dir_pd;
    int left_step; int right_step; int fwd_step;
    int angle_target; int angle_change; int magnitude; int mag_old;
    int lincrement = OCR1A; int rincrement = OCR1B;
    
    angle_target = goal_locate[2]-locate[2];
    angle_change = locate[2]-locate_old[2];
    
    angle_pd = (float) Kp * angle_target - (float) Kd * angle_change;
    
    angle_pd = angle_pd*100/360; //Gives a reading from 0 to 100
        
                                    //Left and right Confirmed for attacker (12/08 6pm JS)
    left_step = -(float) angle_pd*2/10;    //+Left means turning CCW, decreases angle (by max of 2)
    right_step = (float) angle_pd*2/10;          //+Right means CW, increases angle (by max of 2)
        
//~~~~~~~~~~~STEP2: compare location~~~~~~~~~~~~~~~~~~
    
    magnitude = (float)(locate[0] - goal_locate[0])/1000*(locate[0] - goal_locate[0]); //Divide by 1000 to keep < 64000
    magnitude += (locate[1] - goal_locate[1])/1000*(locate[1] - goal_locate[1]);
    magnitude = (float) 10 * sqrt(magnitude);
    
    mag_old = (float)(locate[0] - locate_old[0])*(locate[0] - locate_old[0]);
    mag_old -= (locate[1] - locate_old[1])*(locate[1] - locate_old[1]);
    mag_old = (float) sqrt(mag_old);

    
    
    dir_pd = (float) Kp*magnitude - (float) Kp*mag_old; //Ranges from -500 to 500
    
    fwd_step = dir_pd*2/100; //Gives a reading from 0 to 100
    int a=1; //Parameters 
    int b=2; //...........to test
    int c=1; //...................coefficients
    
    lincrement += a*(b*left_step+c*fwd_step); //TRY TWEAKING RATIOS
    rincrement += a*(b*right_step+c*fwd_step);
//~~~~~~~~~~~STEP3: update variables~~~~~~~~~~~~~~~~~~
    
    if (lincrement < DUTYMAX)
    {
        if ((lincrement) < 0){
                OCR1A = 0;}
        else OCR1A = lincrement;
    }
    else OCR1A = DUTYMAX;
    if (rincrement < DUTYMAX)
    {
        if ((rincrement) < 0){  
                OCR1B = 0;}
        else OCR1B = rincrement;
    }
    else OCR1B = DUTYMAX;
    
    
    //~~~~~~~~~~~~PRINT STUFF~~~~~~~~~~~~~~
    
    m_usb_tx_string("Present Loc (X,Y, theta): ("); 
    m_usb_tx_int(locate[0]); m_usb_tx_string(", "); m_usb_tx_int(locate[1]); 
    m_usb_tx_string(", "); m_usb_tx_int(locate[2]); m_usb_tx_string(") \n");
    m_usb_tx_string("Goal angle (X,Y, theta): ("); m_usb_tx_int(goal_locate[0]);        
    m_usb_tx_string(", "); m_usb_tx_int(goal_locate[1]);
    m_usb_tx_string(", "); m_usb_tx_int(goal_locate[2]);
    m_usb_tx_string(") \n");
    m_usb_tx_string("Magnitude:  ");  m_usb_tx_int(magnitude); m_usb_tx_string("\n");
    m_usb_tx_string("Mag_old:  ");  m_usb_tx_int(mag_old); m_usb_tx_string("\n");
    m_usb_tx_string("Dir_pd [ KP*mag - KP*mag_old ]:  ");  m_usb_tx_int(dir_pd); m_usb_tx_string("\n");
    m_usb_tx_string("Angle PD:  ");  m_usb_tx_int(angle_pd); m_usb_tx_string("\n");
    m_usb_tx_string("Left Step:  ");  m_usb_tx_int(left_step); m_usb_tx_string("\n");
    m_usb_tx_string("Right Step:  ");  m_usb_tx_int(right_step); m_usb_tx_string("\n");
    m_usb_tx_string("FWD Step:  ");  m_usb_tx_int(fwd_step); m_usb_tx_string("\n");
    m_usb_tx_string("OCR1A (L wheel):  ");  m_usb_tx_int(OCR1A); m_usb_tx_string("\n");
    m_usb_tx_string("OCR1B (R wheel):  ");  m_usb_tx_int(OCR1B); m_usb_tx_string("\n");

    return magnitude;
}
void go2pduck(int puckangle, int* locate, int*locate_old) //12/09 9h21 version JS
//Move towards the puck with proportional and derivative feedback
{
	int j;
      
    //~~~~~~~~~~~STEP1: compare angles~~~~~~~~~~~~~~~~~~
    float Kp = 2; float Kd = 0.5; 
    int angle_pd = 0;
    int old1A=OCR1A; int old1B=OCR1B;
    int left_step; int right_step;
    int angle_change;
    int lincrement = OCR1A; int rincrement = OCR1B; //Stores old DUTY cycle value
    
    angle_change = locate[2]-locate_old[2];
    
    angle_pd = (float) Kp * puckangle - (float) Kd * angle_change; //Gives a reading from 0 to 90
    
    if(!check(PORTC,6)){
        old1A = -OCR1A;}
   if(!check(PORTC,7)){
        old1B = -OCR1B;}
                                    //Left and right Confirmed for attacker (12/08 6pm JS)
    left_step = old1A+(float) angle_pd*5/100;    //+Left means turning CCW, decreases angle (by max of 2)
    right_step = old1B-(float) angle_pd*5/100;          //+Right means CW, increases angle (by max of 2)

        
        //~~~~~~~~~~~STEP2: compare location~~~~~~~~~~~~~~~~~~
        //Code to set OCR1A and INV_pins as necessary if left_step +/- and so on
    
    if ((left_step < DUTYMAX) && (left_step > -DUTYMAX))
        {
            m_usb_tx_string("DUTYMAX CHECK WORKED for LEFT \n");
            if (left_step > 0){     //FORWARD
                set(PORTC,6);       //Left is C6 is OCR1A (B5)
                OCR1A = left_step;} 
            else{                   //BACKWARDS
                clear(PORTC,6);   
                OCR1A = -left_step;}
        }


    if ((right_step < DUTYMAX) && (right_step > -DUTYMAX))
        {
            m_usb_tx_string("DUTYMAX CHECK WORKED for RIGHT \n");    
            if (right_step > 0){        //FORWARD
                set(PORTC,7);           //Right is C7 is OCR1B (B6)
                OCR1B = right_step;} 
            else{                   //BACKWARDS
                clear(PORTC,7);
                OCR1B = -right_step;}
        }
      
      m_usb_tx_string("angle_change:  ");  m_usb_tx_int(angle_change); m_usb_tx_string("\n");
    m_usb_tx_string("angle_pd:  ");  m_usb_tx_int(angle_pd); m_usb_tx_string("\n");
    m_usb_tx_string("left_step:  ");  m_usb_tx_int(left_step); m_usb_tx_string("\n");
    m_usb_tx_string("right step:  ");  m_usb_tx_int(right_step); m_usb_tx_string("\n");
    m_usb_tx_string("OCR1A (L wheel):  ");  m_usb_tx_int(OCR1A); 
    if (check(PORTC,6)){ m_usb_tx_string("    FORWARD\n");}
    else m_usb_tx_string("    BACKWARD\n");
    m_usb_tx_string("OCR1B (R wheel):  ");  m_usb_tx_int(OCR1B);
    if (check(PORTC,7)){ m_usb_tx_string("    FORWARD\n");}
    else m_usb_tx_string("    BACKWARD\n");
    
}

int puck_detect(int* ADC_read, int* ADC_track, int puckangle) //FOR ATTACKER
//NEW2 IR Readings:   (F5  F6  D7 D6 F0 F7 F1  F4) **On Attacker, F6 is not reading
//NEW1 ANGLES: (0 +45 +90 +135 +180 -135 -135 -90 -45)

{
    int i;
    
    if(check(ADCSRA,ADIF) && (ADC_track[0] < 8)) //IF ADC IS READY, READS VALUE AND SETS NEXT PIN TO READ
        {    
                ADC_read[ADC_track[0]] = ADC;
                ADC_track[0]++;
        
                set(ADCSRA,ADIF);     //Clears the pin
                clear(ADCSRA,ADEN);     //Disables the ADC system while changing settings

                switch(ADC_track[0])
                {
                case 1:            //Set ADC to read from F0
                F6_read();        
                break;
                case 2:            //Set ADC to read from F7
                D7_read();
                break;
                case 3:            //Set ADC to read from F1
                D6_read();
                break;
                case 4:            //Set ADC to read from F4
                F0_read();
                break;
                case 5:            //Set ADC to read from F5
                F7_read();
                break;
                case 6:            //Set ADC to read from D7
                F1_read();
                break;
                case 7:            //Set ADC to read from F6
                F4_read();
                break;
                case 8:            //Set ADC to read from D6
                F5_read();
                break;
                }
                set(ADCSRA,ADEN);       //Enable conversions
                set(ADCSRA,ADSC);       //Start conversions
        }    
               
            
        if (ADC_track[0]>7)   //Use full ADC array to determine angle (use max point)
        {
            ADC_track[0] = 0;   //Reset ADC counter
            
            for (i=0; i < 8; i++){   //FIND MAX POINT IN ADC_read              
                if (ADC_read[i] > ADC_track[1]){
                    ADC_track[2] = i;   //Store counter location into ADC max location
                    ADC_track[1] = ADC_read[i];}} //Store max reading into ADC_max location
            
            
            if (ADC_track[2] > 4){ //IF MAX POINT IS (+) turn right
                puckangle = -90;}
            else if(ADC_track[2] == 0){    //MAX POINT IS 0 MEANS GO STRAIGHT!
                 puckangle = 0;}
            else puckangle = 90; //IF MAX POINT IS (-) turn left
            ADC_track[1] = 0;          //RESET ADC_max VALUE and FLAG
        }  
        
        return puckangle;
}



void send(int x, int y, int z) //
{
    char buffer_out[10] = {x, y, z, x, y, z, x, y, z, z};
    m_rf_send(TXaddress, buffer_out, packet_length);
}

ISR(TIMER0_COMPA_vect)
{
	timer0_flag=1;
}

ISR(INT2_vect)
{
    read_flag=1;
}


void print_stuff(int*locate, int*goal_locate, int*ADC_read, int puckangle, int state)
{
    int i;
    int angle = goal_locate[2]-(locate[2]-90);
    while (angle > 180){
        angle -= 360;}
    while (angle < -180){
        angle += 360;}

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
    m_usb_tx_int(angle); m_usb_tx_string(" \n");

    m_usb_tx_string("IR Readings:   (F5  F6  D7  D6  F0  F7  F1  F4) \n"); 
    m_usb_tx_string("ADC reading is: ("); 
    for (i=0; i < 8; i++){ //Print out all 8 ADC readings
        m_usb_tx_int(ADC_read[i]); m_usb_tx_string("  "); }
    m_usb_tx_string(")\n");
    m_usb_tx_string("Puck angle calc is:  ");  m_usb_tx_int(puckangle); m_usb_tx_string("\n");
    m_usb_tx_string("Quadrant is:  ");  m_usb_tx_int(locate[3]); m_usb_tx_string("\n");
    
    m_usb_tx_string("STATE IS "); m_usb_tx_int(state); m_usb_tx_string("\n");   
}


