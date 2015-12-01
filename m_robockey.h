// -----------------------------------------------------------------------------
// Orrobot Library
// Version: 1.0
// Author: Jono Sanders, Aditya Pinapala
// Date Created: Nov 18 2015
// Update: 11/21 JS - ratio comparison to include 20 ratios, not just 16 (!), double maxratio (not int!)
// Update: 12/01 JS - added code to throw out readings of 1023
// -----------------------------------------------------------------------------

#include "m_usb.h"
#include "m_general.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_wii.h"

#define PLAY 161
#define PAUSE 164
#define COMM 160
#define SEARCH1 20
#define SEARCH2 25
#define HASPUCK 30
#define GO2GOAL 35
#define SHOOT 40
#define DEFEND 45

char localize(int* locate);
//Reads from mWii and writes X, Y values to locate[0] and locate[1] respectively
void timer0_init();
//Initializes Timer0 for 100 readings per second
void timer1_init();
//Initialized Timer1 to 7800 Hz, for PWM output to motors
void sevensegdispl(int state);
//Takes input state as a number from 0 - 9 and outputs to pins B0 - B3 to 7seg driver IC
    
// -----------------------------------------------------------------------------

char localize(int* locate)
{
    int i;
    unsigned int blobs[12];
    int PtA[2]={0,0};
    int PtD[2]={0,0};
    float theta=0; 
    int center[2] = {1024/2, 768/2};
    int ADcent[2];    
    int anglXY[2];
    int rinkXY[2] = {0, 0};
    int lost_ct = 0;

    m_wii_read(&blobs[0]); //read from m_wii **Be sure that m_wii is initialized! [mwii_open()]
    int Pt1[2] = {blobs[0], blobs[1]};
    int Pt2[2] = {blobs[3], blobs[4]};
    int Pt3[2] = {blobs[6], blobs[7]};
    int Pt4[2] = {blobs[9], blobs[10]};
    
    //~~~Check if any readings should be thrown out (greater than 1022)~~~
    if(Pt1[1] == 1023)){
        for (i=0; i < 2; i++){  // Store Pt1 with values from Pt2
            Pt1[i] = Pt2[i];}
        lost_ct++;}
    
    if(Pt2[1] == 1023){
        for (i=0; i < 2; i++){   // Store Pt2 with values from Pt3
            Pt2[i] = Pt3[i];}
        lost_ct++;}
    
    if(Pt3[1] == 1023){
        for (i=0; i < 2; i++){   // Store Pt3 with values from Pt4
            Pt3[i] = Pt4[i];}
        lost_ct++;}
    
    if(Pt4[1] == 1023){
        for (i=0; i < 2; i++){   // Store Pt4 with values from Pt1
            Pt4[i] = Pt1[i];}
        lost_ct++;}
    
    if(lost_ct > 1){        // If more than one reading is bad, return 0
        return 0;}
    //~~~END READING CHECK~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    float Pt1to2=0, Pt2to3=0, Pt3to4=0, Pt4to1=0, Pt2to4=0, Pt3to1=0;
    // Calculate magnitude of connecting line sqrt [(X1-X2)^2+(Y2-Y1)^2 ]
    for (i=0; i < 2; i++)   // Calculate sum of squares (X1-X2)^2+(Y2-Y1)^2
    {
        Pt1to2 += (Pt1[i]-Pt2[i])*(Pt1[i]-Pt2[i]);
        Pt2to3 += (Pt2[i]-Pt3[i])*(Pt2[i]-Pt3[i]);
        Pt3to4 += (Pt3[i]-Pt4[i])*(Pt3[i]-Pt4[i]);
        Pt4to1 += (Pt4[i]-Pt1[i])*(Pt4[i]-Pt1[i]);
        Pt2to4 += (Pt2[i]-Pt4[i])*(Pt2[i]-Pt4[i]);
        Pt3to1 += (Pt3[i]-Pt1[i])*(Pt3[i]-Pt1[i]);
    }
    
    //Calculate the SQUARE ROOTS!
    Pt1to2 = sqrt(Pt1to2);  Pt2to3 = sqrt(Pt2to3); Pt3to4 = sqrt(Pt3to4);
    Pt4to1 = sqrt(Pt4to1); Pt2to4 = sqrt(Pt2to4);  Pt3to1 = sqrt(Pt3to1);

    float ratio[24];        //Calculate all of the ratios (and their inverses)
    
    ratio[0]=Pt4to1/Pt3to1; //(12) Inv is Pt3to1/Pt4to1
    ratio[1]=Pt4to1/Pt3to4; //(13) Inv is Pt3to4/Pt4to1
    ratio[2]=Pt4to1/Pt1to2; //(14) Inv is Pt1to2/Pt4to1
    ratio[3]=Pt4to1/Pt2to4; //(15) Inv is Pt2to4/Pt4to1
    ratio[4]=Pt3to1/Pt2to3; //(16) Inv is Pt2to3/Pt3to1
    ratio[5]=Pt3to1/Pt3to4; //(17) Inv is Pt3to4/Pt3to1
    ratio[6]=Pt3to1/Pt1to2; //(18) Inv is Pt1to2/Pt3to1
    ratio[7]=Pt2to3/Pt3to4; //(19) Inv is Pt3to4/Pt2to3
    ratio[8]=Pt2to3/Pt1to2; //(20) Inv is Pt1to2/Pt2to3
    ratio[9]=Pt2to3/Pt2to4; //(21) Inv is Pt2to4/Pt2to3
    ratio[10]=Pt3to4/Pt2to4; //(22) Inv is Pt2to4/Pt3to4
    ratio[11]=Pt1to2/Pt2to4; //(23) Inv is Pt2to4/Pt1to2
          
    for (i=0; i < 12; i++){ //Store the inverse of each ratio
        ratio[i+12]=1/ratio[i];}
    
    double maxratio=0;  //Store the maximum of the ratios
    int maxpt=0;        //Store location in the array of max ratio
    
    for (i=0; i < 24; i++){   // Sum of the squares (X^2+Y^2)
        if(ratio[i] > maxratio) //Largest ratio should be DA/AB
        {
            maxpt = i;
            maxratio = ratio[i];
        }}
    
    switch (maxpt){  //Determine which point is D and A from max
        case 0: //D is Pt4, A is Pt1
            for (i=0; i<2; i++){
                PtD[i]=Pt4[i]; PtA[i]=Pt1[i];}
            break;
        case 1: //D is Pt1, A is Pt4
           for (i=0; i<2; i++){
                PtD[i]=Pt1[i]; PtA[i]=Pt4[i];}
            break;
        case 2: //D is Pt4, A is Pt1
           for (i=0; i<2; i++){
                PtD[i]=Pt4[i]; PtA[i]=Pt1[i];}
            break;
        case 3: //D is Pt1, A is Pt4
          for (i=0; i<2; i++){
                PtD[i]=Pt1[i]; PtA[i]=Pt4[i];}
            break;
        case 4: //D is Pt1, A is Pt3
          for (i=0; i<2; i++){
                PtD[i]=Pt1[i]; PtA[i]=Pt3[i];}
            break;
        case 5: //D is Pt1, A is Pt3
          for (i=0; i<2; i++){
                PtD[i]=Pt1[i]; PtA[i]=Pt3[i];}
            break;
        case 6: //D is Pt3, A is Pt1
          for (i=0; i<2; i++){
                PtD[i]=Pt3[i]; PtA[i]=Pt1[i];}
            break;
        case 7: //D is Pt2, A is Pt3
          for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt3[i];}
            break;
        case 8: //D is Pt3, A is Pt2
            for (i=0; i<2; i++){
                PtD[i]=Pt3[i]; PtA[i]=Pt2[i];}
            break;
        case 9: //D is Pt3, A is Pt2
          for (i=0; i<2; i++){
                PtD[i]=Pt3[i]; PtA[i]=Pt2[i];}
            break;
        case 10: //D is Pt3, A is Pt4
          for (i=0; i<2; i++){
                PtD[i]=Pt3[i]; PtA[i]=Pt4[i];}
            break;
        case 11: //D is Pt1, A is Pt2
           for (i=0; i<2; i++){
                PtD[i]=Pt1[i]; PtA[i]=Pt2[i];}
            break;
        case 12: //D is Pt3, A is Pt1
           for (i=0; i<2; i++){
                PtD[i]=Pt3[i]; PtA[i]=Pt1[i];}
            break;
        case 13: //D is Pt3, A is Pt4
            for (i=0; i<2; i++){
                PtD[i]=Pt3[i]; PtA[i]=Pt4[i];}
            break;
        case 14: //D is Pt2, A is Pt1
            for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt1[i];}
            break;
        case 15: //D is Pt2, A is Pt4
           for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt4[i];}
            break;
        case 16: //D is Pt2, A is Pt3
          for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt3[i];}
            break;
        case 17:  //D is Pt4, A is Pt3
          for (i=0; i<2; i++){
                PtD[i]=Pt4[i]; PtA[i]=Pt3[i];}
            break;
        case 18: //D is Pt2, A is Pt1
        for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt1[i];}       
            break;
        case 19: //D is Pt4, A is Pt3
        for (i=0; i<2; i++){
            PtD[i]=Pt4[i]; PtA[i]=Pt3[i];}       
            break;
        case 20: //D is Pt1, A is Pt2
        for (i=0; i<2; i++){
            PtD[i]=Pt1[i]; PtA[i]=Pt2[i];}       
            break;
        case 21: //D is Pt4, A is Pt2
        for (i=0; i<2; i++){
            PtD[i]=Pt4[i]; PtA[i]=Pt2[i];}       
            break;
        case 22: //D is Pt2, A is Pt4
        for (i=0; i<2; i++){
            PtD[i]=Pt2[i]; PtA[i]=Pt4[i];}       
            break;
        case 23: //D is Pt4, A is Pt2
        for (i=0; i<2; i++){
            PtD[i]=Pt4[i]; PtA[i]=Pt2[i];}       
            break;}
        
        
    theta = -atan2(PtA[0]-PtD[0],PtA[1]-PtD[1]); //Calculate orientation angle w.r.t. chord AD
        
        
    for (i=0; i<2; i++) //Find the center of chord AD = rink center
    {
        ADcent[i] = (PtA[i]+PtD[i])/2;
    }
        
    for (i=0; i<2; i++) //Find the bot location coordinates with respect to ADcenter
    {
        anglXY[i] = center[i]-ADcent[i];
    }
        
    rinkXY[0] = -(cos(theta)*anglXY[0]+sin(theta)*anglXY[1]);      // x' = cos(theta)*x + sin(theta)*y
    rinkXY[1] = -sin(theta)*anglXY[0] + cos(theta)*anglXY[1];   // y' = -sin(theta)*x + cos(theta)*y

    //Store rinkLocate[0,1,2] in the bins for "locate" variables
    *locate = rinkXY[0]; //Tested
    *(locate+1) = rinkXY[1];
    *(locate+2) = (float) theta*180/3.14;

    return 1;    //Return 1 to say that localization X, Y and angle have been stored
}

void timer0_init()	//For mWii Read
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

void timer1_init()	//For motors to run at ~8KHz
{
    clear(TCCR1B,CS12); // set prescaler to 8
	set(TCCR1B,CS11); // 0 1 0
	clear(TCCR1B,CS10);
    
    clear(TCCR1B, WGM13);     //Mode 5: up to 0X00FF (256), PWM
	set(TCCR1B, WGM12);     //  0 1 0 1
	clear(TCCR1A, WGM11);   // Means 2,000,000 / 256 counts >> 7.8KHz
	set(TCCR1A, WGM10);
	
	set(TCCR1A,COM1A1);     // For channel A (B5) clear at OCR1A, set at rollover
	clear(TCCR1A,COM1A0);   // 1 0
	//OCR1A = 255*80/100;       // 50% duty cycle for 0x00FF (255)

	set(TCCR1A,COM1B1);     // For channel B (B6) clear at OCR1B, set at rollover
	clear(TCCR1A,COM1B0);   // 1 0
    
    
	OCR1A = 256/10*4;    //Initialize w 40% duty cycle
	OCR1B = OCR1A;
}

void sevensegdispl(int state)
{	set(DDRB,0);
	set(DDRB,1);
	set(DDRB,2);
	set(DDRB,3);
	switch(state)
	{
	case 0:
	clear(PORTB,0);
	clear(PORTB,1);
	clear(PORTB,2);
	clear(PORTB,3);
	break;
	case 1:
	set(PORTB,0);
	clear(PORTB,1);
	clear(PORTB,2);
	clear(PORTB,3);
	break;
	case 2:
	clear(PORTB,0);
	set(PORTB,1);
	clear(PORTB,2);
	clear(PORTB,3);
	break;
	case 3:
	set(PORTB,0);
	set(PORTB,1);
	clear(PORTB,2);
	clear(PORTB,3);
	break;
	case 4:
	clear(PORTB,0);
	clear(PORTB,1);
	set(PORTB,2);
	clear(PORTB,3);
	break;
	case 5:
	set(PORTB,0);
	clear(PORTB,1);
	set(PORTB,2);
	clear(PORTB,3);
	break;
	case 6:
	clear(PORTB,0);
	set(PORTB,1);
	set(PORTB,2);
	clear(PORTB,3);
	break;
	case 7:
	set(PORTB,0);
	set(PORTB,1);
	set(PORTB,2);
	clear(PORTB,3);
	break;
	case 8:
	clear(PORTB,0);
	clear(PORTB,1);
	clear(PORTB,2);
	set(PORTB,3);
	break;
	case 9:
	set(PORTB,0);
	clear(PORTB,1);
	clear(PORTB,2);
	set(PORTB,3);
	break;	
	default:
	set(PORTB,0);
	set(PORTB,1);
	set(PORTB,2);
	set(PORTB,3);
	break;
	}
	
}