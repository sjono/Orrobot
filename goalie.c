//Goalie Puck detect code
/**** 12/04/2015*********/
//Author: Aditya PSR

#define F_CPU 16000000UL

#include "m_robockey.h"

#define OVERRIDE OFF
#define USB_DEBUG ON
#define packet_length 10
#define channel 1
#define CENTER 0
#define LEFT 2
#define RIGHT 1

int timer0_flag=0;
int read_flag=0;
int TXaddress = 0x4D;
int RXaddress = 0x4E;
unsigned char buffer[packet_length];
void goal_block(int*locate,int direction,int goal_extent_flag,int goal_extent_count,int value);
void goalie_stabilize(int*locate);
void send(int x, int y,int z);

void init()
{
	int i;
	m_clockdivide(0);
	for (i=0; i < 10; i++)
		buffer[i] = 0;

	m_usb_init();
	m_wii_open();
	timer0_init();
	timer1_init();
	m_disableJTAG();
	m_rf_open(channel, RXaddress, packet_length); //[WHY ISNT THIS WORKING?!]
	
	sei();  //Enable interrupts!!
	
	set(DDRC,7);    //Set pin C7 for L motor direction as output
	set(DDRC,6);    //Set pin C6 for R motor direction as output
	
	set(DDRD,4);    //Set pin D4 for red LED as output
	set(DDRD,5);    //Set pin D5 for blue LED as output
	
	//~~~ADC INIT~~~~

	
	clear(ADMUX,REFS1); //Set to internal VCC
	set(ADMUX,REFS0);   // 0 1

	set(ADCSRA,ADPS2);      // Prescaler set to /128 (for 16MHz clock, 125 kHz)
	set(ADCSRA,ADPS1);    // 1 1 1
	set(ADCSRA,ADPS0);

	clear(ADCSRA,ADATE); //DISABLE FREE RUNNING MODE!
	
	set(DIDR0,ADC0D);       //Disables digital input at PIN F0 (ADC0)
	set(DIDR0,ADC1D);       //Disables digital input at PIN F1 (ADC1)
	set(DIDR0,ADC4D);       //Disables digital input at PIN F4 (ADC4)
	
	F1_read();  //Starts with F4 reading
	
	set(ADCSRA,ADEN);       //Enable conversions
	set(ADCSRA,ADSC);       //Start conversions
	//~~ADC NOW INITIALIZED
}


int main()
{
	 int color = BLUE;
     int locate[4];
	 int goallocation[3];
	 int blue_flag = 0;
	 int goal_extent_flag=0;
	 int goal_extent_count=0;
	 m_red(ON); //If only red is on, still initializing
	 m_wait(50); //Wait to be sure no hands are above the mWii
	 int state=PAUSE;
	 int adc_ct = 0;     //Counts ADC readings during cycle through various pins
	 int ADC_read[3];    //Stores values from the ADC readings
	 int ADC_flag = 0;
	 unsigned int blobs[12];
	 int direction=0; int i; int j = 0;
	 int ADC_max = 0; int ADC_max_loc = 0;
	 int puck_detect_count=0;
	 
	 for (i=0; i < 6; i++)
	 { //Initialize with a few mWii readings to filter out noise
		localize(locate);
	 }

	 init();
	 goalcalibrate(locate,goallocation);
	 m_red(OFF);	 
	 while(1)
	 {
		if(timer0_flag==1)
		{
			
			m_green(TOGGLE);
			localize(locate);
			if(read_flag==1)
			{
				 m_rf_read(buffer, packet_length); //Read from RF
				read_flag = 0;
			 
				//Display the reading in serial
				/*m_usb_tx_string("Buffer reading is: ");
				 m_usb_tx_int(buffer[0]); m_usb_tx_string(" \n");*/
			 
				
			}
			//if(abs(locate[0])<250 || abs(locate[2])<165 || abs(locate[2])>15)
				//goalie_stabilize(locate);
			if(buffer[0] == PLAY)
				state = SEARCH1;
			else if(buffer[0] == PAUSE)
				state = PAUSE;
			else if(buffer[0] == COMM)
				state = PAUSE;
			
			//for(i=0;i<3;i++)
			sevensegdispl(locate[3]);
		
			if (j <50) //Counter to reset every X milliseconds
				j++;
			else
			{
				j=0;
				/*m_wii_read(blobs);
				m_usb_tx_int(blobs[0]); m_usb_tx_string(", "); m_usb_tx_int(blobs[3]); m_usb_tx_string(", ");
				m_usb_tx_int(blobs[6]); m_usb_tx_string(", "); m_usb_tx_int(blobs[9]); m_usb_tx_string(", ");
				m_usb_tx_int(blobs[1]); m_usb_tx_string(", "); m_usb_tx_int(blobs[4]); m_usb_tx_string(", ");
				m_usb_tx_int(blobs[7]); m_usb_tx_string(", "); m_usb_tx_int(blobs[10]); m_usb_tx_string("\n");*/

				m_usb_tx_string("Present Loc (X,Y, theta): (");
				m_usb_tx_int(locate[0]); m_usb_tx_string(", "); m_usb_tx_int(locate[1]);
				m_usb_tx_string(", "); m_usb_tx_int(locate[2]); m_usb_tx_string(") \n");
				m_usb_tx_string("IR Readings:   (F1  F0  F4) \n");
				m_usb_tx_string("ADC reading is: (");
				for (i=0; i < 3; i++){ //Print out all 8 ADC readings
				m_usb_tx_int(ADC_read[i]); m_usb_tx_string("  "); }
				m_usb_tx_string(")\n");
				m_usb_tx_int(goal_extent_flag);
				m_usb_tx_string("\n");
				m_usb_tx_int(direction);
				//m_usb_tx_string("Max ADC value is at :  ");
				//m_usb_tx_int(ADC_max_loc); m_usb_tx_string("\n");
				//m_usb_tx_string("Puck angle calc is:  ");
				//m_usb_tx_int(direction); m_usb_tx_string("\n");
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

			}
			//if(abs(locate[3])>15||abs(locate[3])<165)
				//goalie_stabilize(locate);
			timer0_flag=0;	
		}
		
		if(check(ADCSRA,ADIF) && (adc_ct < 3)) //IF ADC IS READY, READS VALUE AND SETS NEXT PIN TO READ
		{
			ADC_read[adc_ct] = ADC;
			adc_ct++;
			set(ADCSRA,ADIF);     //Clears the pin
			
			clear(ADCSRA,ADEN);     //Disables the ADC system while changing settings

			switch(adc_ct)
			{
				case 1:            //Set ADC to read from F0
				F0_read();
				break;
				case 2:            //Set ADC to read from F1
				F4_read();
				break;
				case 3:            //Set ADC to read from F4
				F1_read();
				break;
			}
			set(ADCSRA,ADEN);       //Enable conversions
			set(ADCSRA,ADSC);       //Start conversions
		}
		 if((adc_ct)>2)
		 { //RESETS ADC_CT IF FULL CYCLE
			 adc_ct = 0;
			 ADC_flag = 1;   //Flag is set to say that ADC array is full
		 }
		 if(ADC_flag==1)
		 {
			 for (i=0; i < 3; i++)
			 {   //FIND MAX POINT IN ADC_read
				 if (ADC_read[i] > ADC_max)
				 {
					 ADC_max_loc = i;
					ADC_max = ADC_read[i];
				}
			}
			
		}
		 
		switch(state)
		{
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
		 
			case SEARCH1:		//A goal sweep function from left to right when it cant see the puck
			if(goal_extent_flag || (direction==CENTER))
			{
				goal_extent_flag = 0;
				direction = (direction+1)%3;
			}
			goal_block(locate,direction,goal_extent_flag,goal_extent_count,200);
			if(ADC_max>=100){
				// ADD COUNTER HERE to have it only change to DEFEND after 100
                ADC_max = 0;
				state = DEFEND;
				if(ADC_max_loc==0){
					direction=CENTER;
					//m_usb_tx_string("CENTER \n");
				}
				if(ADC_max_loc==1){
					direction=RIGHT;
					//m_usb_tx_string("RIGHT \n");
				}
				if(ADC_max_loc==2){
					direction=LEFT;
					//m_usb_tx_string("LEFT \n");
				}
			}
			sevensegdispl(7);
			break;
			
			case DEFEND:		//Sees the puck and defend the goal to the side of the puck
			//m_usb_tx_string("I SEE THE PUCK");
			if(ADC_max>700)
			{	
				if(ADC_max_loc==0)
				{
					direction=CENTER;
					//m_usb_tx_string("CENTER \n");
				}
				if(ADC_max_loc==1)
				{
					direction=RIGHT;
					//m_usb_tx_string("RIGHT \n");
				}
				if(ADC_max_loc==2)
				{
					direction=LEFT;
					//m_usb_tx_string("LEFT \n");
				}
				ADC_max=0;
				goal_block(locate,direction,goal_extent_flag,goal_extent_count,200);
				for(i=0;i<3;i++)
				{
					send(CLEARGOAL,CLEARGOAL,CLEARGOAL);
					m_usb_tx_string("SENDING");	
				}
				
			}
			else
			{
				if(ADC_max_loc==0)
				{
					direction=CENTER;
					//m_usb_tx_string("CENTER \n");
				}
				if(ADC_max_loc==1)
				{
					direction=RIGHT;
					//m_usb_tx_string("RIGHT \n");
				}
				if(ADC_max_loc==2){
					direction=LEFT;
					//m_usb_tx_string("LEFT \n");
				}
				goal_block(locate,direction,goal_extent_flag,goal_extent_count,200);
				sevensegdispl(6);
				if(ADC_max<100)
				{
					ADC_max=0;
					state = SEARCH1;
				}
			}
			break;
		}
	 }
}


void goal_block(int*locate,int direction,int goal_extent_flag,int goal_extent_count,int value)
{
	int j;
	if(direction==CENTER)		//The puck is straight ahead
	{
		//m_usb_tx_string("CENTER\n");
		motor_stop();
		
	}
	if((direction == RIGHT)&&(locate[3]==2 || locate[3]==3))		//The puck is towards the right
	{
		if(locate[1]<=-120)//-120   //-90
		{
			//m_usb_tx_string("RIGHT STOP\n");
			//goal_extent_count++;
			//if(goal_extent_count>10)
			//{
				goal_extent_count=0;
				goal_extent_flag=1;
				motor_stop();	
			//}
			
		}
		else
		{
			//m_usb_tx_string("GOING RIGHT\n");
			clear(DDRB,5);
			clear(DDRB,6);
			for(j=0;j<30000;j++);
			clear(PORTC,6);
			clear(PORTC,7);
			set(DDRB,5);
			set(DDRB,6);
		} 
	}
	if((direction == RIGHT)&&(locate[3]==1 || locate[3]==4))
	{
		if(locate[1]>=120)//120 \\90
		{
			//goal_extent_count++;
			//if(goal_extent_count>10)
			//{
				goal_extent_count=0;
				goal_extent_flag=1;
				motor_stop();
			//}
		}
		
		else
		{
			//m_usb_tx_string("GOING RIGHT\n");
			clear(DDRB,5);
			clear(DDRB,6);
			for(j=0;j<30000;j++);
			set(PORTC,6);
			clear(PORTC,7);
			clear(DDRB,5);
			set(DDRB,5);
			set(DDRB,6);
		}
	}
	if((direction == LEFT)&&(locate[3]==2 || locate[3]==3))	//The puck is towards left
	{
		if(locate[1]>=80)//80  60
		{
			
				goal_extent_count=0;
				goal_extent_flag=1;
				motor_stop();

		}
		
		else
		{
			//m_usb_tx_string("GOING LEFT\n");
			clear(DDRB,5);
			clear(DDRB,6);
			for(j=0;j<30000;j++);
			set(PORTC,6);
			set(PORTC,7);
			set(DDRB,5);
			set(DDRB,6);
		}
	}
	if((direction == LEFT)&&(locate[3]==1 || locate[3]==4))
	{
		if(locate[1]<=-90)//-90		//-60
		{
			
				goal_extent_count=0;
				goal_extent_flag=1;
				motor_stop();

		}
		
		else
		{
			//m_usb_tx_string("GOING LEFT\n");
			clear(DDRB,5);
			clear(DDRB,6);
			for(j=0;j<30000;j++);
			set(PORTC,6);
			set(PORTC,7);
			set(DDRB,5);
			set(DDRB,6);
		}
	}
	OCR1A = value;
	OCR1B = value;
}

 ISR(TIMER0_COMPA_vect)
{
	timer0_flag=1;
}

 ISR(INT2_vect)
{
	read_flag=1;
}
void send(int x, int y, int z) //
{
	char buffer_out[10] = {x, y, z, x, y, z, x, y, z, z};
	m_rf_send(TXaddress, buffer_out, packet_length);
}
/*void goalie_stabilize()
{
	
}*/