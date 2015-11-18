// -----------------------------------------------------------------------------
// Orrobot Qualifying Test
// Version: 0.1
// Author: Jono Sanders
// Date: Nov 18 2015
// -----------------------------------------------------------------------------

#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_wii.h"
#include "m_robockey.h"

void loc_init();
void init();

#define USB_DEBUG ON
#define packet_length 10
#define channel 1

volatile char mwii_readflag=0;
int TXaddress = 0x4F;
int RXaddress = 0x4C;
char buffer[packet_length];

int main()
{
    int locate[3];  //Stores X, Y, omega value for the bot location based on mWii readings
    int value;
    int state = COMM;
    int i;
    char wii_read = 0;
    init();
    m_green(OFF);
    m_red(ON);
    for (i=0; i < 6; i++){ //Initialize with a few mWii readings to filter out noise
        localize(&locate[0]);
    }
    while(1)
    {
        m_red(OFF);
        m_green(TOGGLE);
        if(USB_DEBUG && m_usb_rx_available()){ //When a button is pressed, send data
            value = m_usb_rx_char();
            m_usb_tx_string("Location calc is (X,Y, theta): "); 
            m_usb_tx_int(locate[0]);        
            m_usb_tx_string(", ");
            m_usb_tx_int(locate[1]);
            m_usb_tx_string(", ");
            m_usb_tx_int(locate[2]*180/3.14);
            m_usb_tx_string(") \n");
        }   
        if(mwii_readflag==1)
        {
            localize(&locate[0]);
            m_rf_read(buffer, packet_length); //Read from RF
            mwii_readflag=0;
            
            //Look at read value to determine if correct state
            if(buffer[0] == PLAY && buffer[1] == PLAY){
            state = GO2GOAL;
            }
            //add many more if statements here!
            else{
                state = COMM;
            }
        }
        switch(state){
            case COMM:  //Listen for signal sent by the game
                motor_stop();
                state_LED(COMM);
                //Going R? (X=high) Flash blue positioning LED
                //Going L? (X=0) Flash red positioning LED
                break;
            case SEARCH1:
                //Dictated by starting position
                //Main search pattern
                break;
            case SEARCH2:
                //Dictated by starting position
                //Alternative search pattern
                break;
            case HASPUCK:
                //Go directly to the puck
                break;
            case GO2GOAL:
                //Going R? Move toward R goal, small turn radii
                //Going L? Move toward R goal, small turn radii
                break;
            case SHOOT:
                //stop moving
                //fire solenoid
                break;
            case DEFEND:
                // Back to our goal!
                break;
            default:
                    // search mode
                break;
    }
}

void init()
{
    m_clockdivide(1);
    m_usb_init();
    m_wii_open();
    timer0_init();
    timer1_init();
    m_rf_open(channel, RXaddress, packet_length);
    
}

ISR(TIMER0_COMPA_vect)
{
	mwii_readflag=1;
}