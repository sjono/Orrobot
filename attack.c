// -----------------------------------------------------------------------------
// Orrobot Attacker
// Version: 0.1
// Author: Jono Sanders
// Date: Nov 17 2015
// -----------------------------------------------------------------------------

#include "m_usb.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_robockey.h"

#define USB_DEBUG ON
#define packet_length 10
#define channel 1
#define PLAY 161
#define PAUSE 164
#define COMM 160
#define SEARCH1 20
#define SEARCH2 25
#define PUCKF 30
#define GOALF 35
#define SHOOT 40

void Timer0_init();  //Sets up Timer0 at 100 Hz (for readings)
void Timer1_init();  //Sets up Timer1 at 7kHz (best PWM for motors)
void Motor_run();   //Reads Duty cycle >>Turns on PWM output for B6 (L motor) and B5 (R motor)
void Motor_off();   //Turns off PWM for pins B6 and B5
void Motor_turn();    //Reads Duty, direction >> turns CW or CCW as desired
void init();
void printF();
    

int RXaddress = 0x4C;
char BTN_flag = 0;
volatile int i = 0;
volatile char read_flag=0;
volatile char state;
char buffer[packet_length];
int TXaddress = 0x4D;
volatile char timer0_flag = 0;

int main()
{
    int value;
    m_green(OFF);
    m_red(ON);
    init();
    while(1){
        m_red(OFF);
        m_green(ON);
        if(USB_DEBUG & m_usb_rx_available()){ //When a button is pressed, send data
            value = m_usb_rx_char();
            printF(value, buffer);    //Asks for input to determine which signal to send
        }   
        if (timer0_flag)    // To run every 10 ms, based on Timer0
        {
            if (read_flag){
                m_rf_read(buffer, packet_length);
                printF(0, buffer[0]);
                read_flag = 0;
            }
            timer0_flag = 0;
            
         }
        switch(state){
            case PLAY:  //Signal sent by the game
                //Check initial location and begin Search1 or Search2
                break;
            case PAUSE:  //Signal sent by the game
                //stop motion
                break;
            case COMM:  //Signal sent by the game
                //Going R? (X=high) Flash blue positioning LED
                //Going L? (X=0) Flash red positioning LED
                break;
            case SEARCH1:
                //Dictated by starting position
                //Main search pattern
                break;
            case SEARCH2;
                //Dictated by starting position
                //Alternative search pattern
                break;
            case PUCKF;
                //Go directly to the puck
                break;
            case GOALF;
                //Going R? Move toward R goal, small turn radii
                //Going L? Move toward R goal, small turn radii
                break;
            case SHOOT;
                //stop moving
                //fire solenoid
                break;
            default:
                    // search mode
                break;
    }
}


void init()
{
    int i;
    for (i=0; i < 10; i++)
    {
        buffer[i] = 0;
    }
    sei();
    m_clockdivide(1);
    m_rf_open(channel, RXaddress, packet_length);
    m_usb_init();
    Timer0_init(); 
    Timer1_init();
}

void Timer0_init()      // For readings every 10 ms
{
    TCNT0 = 0;
    
    set(TCCR0B,CS02);     // set prescaler to 1024 >> 16 MHz / 1024 = 15,625 counts / sec
    clear(TCCR0B,CS01);       // 1 0 1
    set(TCCR0B,CS02);
    
    set(TCCR0B, WGM02);     //Up to OCR0A, PWM mode
    set(TCCR0A, WGM01);     // 1 1 1
    set(TCCR0A, WGM00);
    

    OCR0A = 156;            // Means 15,625 / 156 = 100 Hz or 10 ms per cycle
    set(TIMSK0,OCIE0A);          //Sets an interrupt to occur whenever OCR0A is reached
}

void Timer1_init()  //For driving the motors
{
    clear(TCCR1B,CS12); // set prescaler to 8
	set(TCCR1B,CS11); // 0 1 0
	clear(TCCR1B,CS10);
    
    clear(TCCR1B, WGM13);     //Mode 5: up to 0X00FF (256), PWM
	set(TCCR1B, WGM12);     //  0 1 0 1
	clear(TCCR1A, WGM11);   // Means 2,000,000 / 256 >> 7.8KHz
	set(TCCR1A, WGM10);
	
	set(TCCR1A,COM1A1);     // For channel A (B5) clear at OCR1A, set at rollover
	clear(TCCR1A,COM1A0);   // 1 0
	//OCR1A = 255*80/100;       // 50% duty cycle for 0x00FF (255)

	set(TCCR1A,COM1B1);     // For channel B (B6) clear at OCR1B, set at rollover
	clear(TCCR1A,COM1B0);   // 1 0
}
    
void printF(int key, int buff)
{
    if (key == 0)
    {
        m_usb_tx_string("    Buffer just read as: ");
        m_usb_tx_int(buff);
        m_usb_tx_string("\n");
    }
    else{
        m_usb_tx_string("\n");    
        m_usb_tx_int(key);
        m_usb_tx_string(" Buffer reading is: ");
        m_usb_tx_int(buff);
        m_usb_tx_string("\n");
    }
}

ISR(TIMER0_COMPA_vect)
{
    timer0_flag = 1;
}

ISR(INT2_vect)
{
    read_flag=1;
}
    
void Motor_run(int duty){
    OCR1A = abs(duty)*255/100;
	OCR1B = abs(duty)*255/100;
	 	
    set(PINB,5);//Left wheel on
    set(PINB,6);//Right wheel on
    
	/*if(duty > 0){
		set(PORTB,0);	//Clockwise
	}
	if(duty < 0){
		clear(PORTB,0);	//Counter clockwise
	}*/
}
    
void Motor_off()
{
    clear(PINB,5);//Left wheel off
    clear(PINB,6);//Right wheel off
}
    
void Motor_turn(char dir){
    if (dir == 1)  //Turn clockwise
    {
        set(PINB,6); //Left wheel on
        clear(PINB,5);//Right wheel off
    }
    if (dir == 2)  //Turn counterclockwise
    {
        set(PINB,5); //Left wheel off
        clear(PINB,6);//Right wheel on
    }
}
    
void localize(char* location)
{
    int X, Y;
    //run code to read from mWii
    location[0] = X; //Store X value in the location X spot
    location[1] = Y; //Store Y value in the location Y spot
}