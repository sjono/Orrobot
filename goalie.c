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
int TXaddress = 0x4F;
int RXaddress = 0x4C;
unsigned char buffer[packet_length];
void goal_block(int*locate,int direction);

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
	 int locate[4];
	 int blue_flag = 0;
	 int blobs[12];
	 m_red(ON); //If only red is on, still initializing
	 m_wait(50); //Wait to be sure no hands are above the mWii
	 int state = COMM;
	 int adc_ct = 0;     //Counts ADC readings during cycle through various pins
	 int ADC_read[3];    //Stores values from the ADC readings
	 int ADC_flag = 0;
	 int direction=0; int i; int j = 0;
	 int ADC_max = 0; int ADC_max_loc = 0;
	 for (i=0; i < 6; i++)
	 { //Initialize with a few mWii readings to filter out noise
		localize(locate);
	 }

	 init();
	 
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
				m_usb_tx_string("Buffer reading is: ");
				 m_usb_tx_int(buffer[0]); m_usb_tx_string(" \n");
			 
				if(buffer[0] == PLAY)
					state = SEARCH1;
			 
				else if(buffer[0] == PAUSE)
					state = PAUSE;
				
				else if(buffer[0] == COMM)
					state = COMM;
		
				else  //TESTING - JUST GET THIS TO MOVE! (doesn't wait for RF signal)
					state = SEARCH1;			 
			}
			if (locate[0] > 0 && locate[1] > 0){
			locate[3] = 1;}
			if (locate[0] < 0 && locate[1] > 0){
			locate[3] = 2;}
			if (locate[0] < 0 && locate[1] < 0){
			locate[3] = 3;}
			if (locate[0] > 0 && locate[1] < 0){
			locate[3] = 4;}
			sevensegdispl(locate[3]);
		
			if (j <50) //Counter to reset every X milliseconds
				j++;
			else
			{
				j=0;
				m_usb_tx_string("Present Loc (X,Y, theta): (");
				m_usb_tx_int(locate[0]); m_usb_tx_string(", "); m_usb_tx_int(locate[1]);
				m_usb_tx_string(", "); m_usb_tx_int(locate[2]); m_usb_tx_string(") \n");
				m_usb_tx_string("IR Readings:   (F1  F0  F4) \n");
				m_usb_tx_string("ADC reading is: (");
				for (i=0; i < 3; i++){ //Print out all 8 ADC readings
				m_usb_tx_int(ADC_read[i]); m_usb_tx_string("  "); }
				m_usb_tx_string(")\n");
				m_usb_tx_string("Max ADC value is at :  ");
				m_usb_tx_int(ADC_max_loc); m_usb_tx_string("\n");
				m_usb_tx_string("Puck angle calc is:  ");
				m_usb_tx_int(direction); m_usb_tx_string("\n");
				if (blue_flag)
				{ //Called only in COMM mode
					if (check(PORTD,4)) //Toggle red LED (D4)
						clear(PORTD,4);
					else 
						set(PORTD,4);
					if (check(PORTD,5)) //Toggle blue LED (D5)
						clear(PORTD,5);
					else
						set(PORTD,5);
					blue_flag = 0;
				}

			}
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
			if(ADC_max>=100){
				ADC_max = 0;
				state = DEFEND;
				if(ADC_max_loc==0)
					direction=CENTER;
				if(ADC_max_loc==1)
					direction=RIGHT;
				if(ADC_max_loc==2)
					direction=LEFT;
			}
				 
		 }
		switch(state)
		{
			case COMM:  //Listen for signal sent by the game
			motor_stop();
			blue_flag = 1;
			sevensegdispl(8); //Number 8 means STOP!
			break;
		 
			case PAUSE:  //Listen for signal sent by the game
			motor_stop();
			sevensegdispl(8); //Number 8 means STOP!
			break;
		 
			case SEARCH1:		//A goal sweep function from left to right when it cant see the puck
			if(abs(locate[1])>=50 || abs(locate[1])<=10)
				direction = (direction+1)%3;
			goal_block(locate,direction);
			sevensegdispl(7);
			break;
			
			case DEFEND:		//Sees the puck and defend the goal to the side of the puck
			goal_block(locate,direction);
			if(ADC_max>500)
				
			sevensegdispl(6);
			break;
		}
	 }
}


void goal_block(int*locate,int direction)
{
	int j;
	if(direction==CENTER)
	{
		motor_stop();
		
	}
	if((direction == RIGHT)&&(locate[3]==2 || locate[3]==3))
	{
		if(locate[1]>=-50)
			motor_stop();
		else
		{
			clear(DDRB,5);
			clear(DDRB,6);
			for(j=0;j<30000;j++);
			set(PORTC,6);
			set(PORTC,7);
			set(DDRB,5);
			set(DDRB,6);
		} 
	}
	if((direction == RIGHT)&&(locate[3]==1 || locate[3]==4))
	{
		if(locate[1]>=50)
		motor_stop();
		else
		{
			clear(DDRB,5);
			clear(DDRB,6);
			for(j=0;j<30000;j++);
			set(PORTC,6);
			clear(PORTC,7);
			clear(DDRB,5);
			set(DDRB,6);
		}
	}
	if((direction == LEFT)&&(locate[3]==2 || locate[3]==3))
	{
		if(locate[1]>=50)
		motor_stop();
		else
		{
			clear(DDRB,5);
			clear(DDRB,6);
			for(j=0;j<30000;j++);
			clear(PORTC,6);
			clear(PORTC,7);
			set(DDRB,5);
			set(DDRB,6);
		}
	}
	if((direction == LEFT)&&(locate[3]==1 || locate[3]==4))
	{
		if(locate[1]>=-50)
		motor_stop();
		else
		{
			clear(DDRB,5);
			clear(DDRB,6);
			for(j=0;j<30000;j++);
			set(PORTC,6);
			set(PORTC,7);
			set(DDRB,5);
			set(DDRB,6);
		}
	}
	OCR1A = 150;
	OCR1B = 150;
}

 ISR(TIMER0_COMPA_vect)
{
	timer0_flag=1;
}

 ISR(INT2_vect)
{
	read_flag=1;
}