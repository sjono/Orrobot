// -----------------------------------------------------------------------------
// Orrobot Library
// Version: 0.1
// Author: Jono Sanders
// Date: Nov 18 2015
// -----------------------------------------------------------------------------

#include "m_usb.h"

void localize(int* locate);
//Reads from mWii and writes X, Y values to locate[0] and locate[1] respectively

// -----------------------------------------------------------------------------

void localize(int* locate)
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
    

    m_wii_read(&blobs[0]); //read from m_wii **Be sure that m_wii is initialized!
    int Pt1[2] = {blobs[0], blobs[1]};
    int Pt2[2] = {blobs[3], blobs[4]};
    int Pt3[2] = {blobs[6], blobs[7]};
    int Pt4[2] = {blobs[9], blobs[10]};
    float Pt1to2=0, Pt2to3=0, Pt3to4=0, Pt4to1=0, Pt2to4=0, Pt3to1=0;
    for (i=0; i < 2; i++)   // Sum of the squares (X^2+Y^2)
    {
        Pt1to2 += (Pt1[i]-Pt2[i])*(Pt1[i]-Pt2[i]);
        Pt2to3 += (Pt2[i]-Pt3[i])*(Pt2[i]-Pt3[i]);
        Pt3to4 += (Pt3[i]-Pt4[i])*(Pt3[i]-Pt4[i]);
        Pt4to1 += (Pt4[i]-Pt1[i])*(Pt4[i]-Pt1[i]);
        Pt2to4 += (Pt2[i]-Pt4[i])*(Pt2[i]-Pt4[i]);
        Pt3to1 += (Pt3[i]-Pt1[i])*(Pt3[i]-Pt1[i]);
    }
    
    Pt1to2 = sqrt(Pt1to2);
    Pt2to3 = sqrt(Pt2to3);
    Pt3to4 = sqrt(Pt3to4);
    Pt4to1 = sqrt(Pt4to1);
    Pt2to4 = sqrt(Pt2to4);
    Pt3to1 = sqrt(Pt3to1);

        //Calculate all of the ratios
    float ratio[16];
    
    ratio[0]=Pt4to1/Pt1to2;
    ratio[1]=Pt1to2/Pt3to1;
    ratio[2]=Pt1to2/Pt2to3;
    ratio[3]=Pt1to2/Pt2to4;
    ratio[4]=Pt2to3/Pt3to4;
    ratio[5]=Pt2to3/Pt3to1;
    ratio[6]=Pt3to4/Pt4to1;
    ratio[7]=Pt2to4/Pt4to1;
      
      for (i=0; i < 8; i++) //Include the inverse ratios
      {
        ratio[i+8]=1/ratio[i];
      }
    
    int maxpt=0, maxratio=0;
    for (i=0; i < 16; i++)   // Sum of the squares (X^2+Y^2)
    {
        if(ratio[i] > maxratio) //Usually the ratio DA/AB
        {
            maxpt = i;
            maxratio = ratio[i];
        }
    }
    switch (maxpt){  //Determine which point is D and A from max
        case 0: //D is Pt4, A is Pt1
            for (i=0; i<2; i++){
                PtD[i]=Pt4[i]; PtA[i]=Pt1[i];}
            break;
        case 1: //D is Pt2, A is Pt1
           for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt1[i];}
            break;
        case 2: //D is Pt1, A is Pt2
           for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt2[i];}
            break;
        case 3: //D is Pt1, A is Pt2
          for (i=0; i<2; i++){
                PtD[i]=Pt1[i]; PtA[i]=Pt2[i];}
            break;
        case 4: //D is Pt2, A is Pt3
          for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt3[i];}
            break;
        case 5: //D is Pt2, A is Pt3
          for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt3[i];}
            break;
        case 6: //D is Pt3, A is Pt4
          for (i=0; i<2; i++){
                PtD[i]=Pt3[i]; PtA[i]=Pt4[i];}
            break;
        case 7: //D is Pt2, A is Pt4
          for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt4[i];}
            break;
        case 8: //D is Pt2, A is Pt1
          for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=PtA[i];}
            break;
        case 9: //D is Pt3, A is Pt1
           for (i=0; i<2; i++){
                PtD[i]=Pt3[i]; PtA[i]=Pt1[i];}
            break;
        case 10: //D is Pt3, A is Pt2
           for (i=0; i<2; i++){
                PtD[i]=Pt3[i]; PtA[i]=Pt2[i];}
            break;
        case 11: //D is Pt2, A is Pt4
            for (i=0; i<2; i++){
                PtD[i]=Pt2[i]; PtA[i]=Pt4[i];}
            break;
        case 12: //D is Pt4, A is Pt3
            for (i=0; i<2; i++){
                PtD[i]=Pt4[i]; PtA[i]=Pt3[i];}
            break;
        case 13: //D is Pt1, A is Pt3
           for (i=0; i<2; i++){
                PtD[i]=Pt1[i]; PtA[i]=Pt3[i];}
            break;
        case 14: //D is Pt1, A is Pt4
          for (i=0; i<2; i++){
                PtD[i]=Pt1[i]; PtA[i]=Pt4[i];}
            break;
        case 15:  //D is Pt1, A is Pt4
          for (i=0; i<2; i++){
                PtD[i]=Pt1[i]; PtA[i]=Pt4[i];}
            break;
    }
        
        
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
        if (rinkXY[0] < 0){
            rinkXY[1] = -rinkXY[1];}
    
        //set locate variable to look at rinkLocate(1&2)
        *locate = rinkXY[0]; //Tested
        *(locate+1) = rinkXY[1];
        *(locate+2) = (float) theta*180/3.14;
    
        /*m_usb_tx_string("mWii reading is: ");    //PRINT OUT CODE, if desired
        for (i=0; i<12; i++){
                m_usb_tx_int(blobs[i]);
                m_usb_tx_string(", ");
                }
            m_usb_tx_string("\n");*/
            
            /*m_usb_tx_string("Location in the rink is \n"); //PRINT OUT CALCULATIONS
            m_usb_tx_string("X: ");
            m_usb_tx_int(rinkXY[0]);        
            m_usb_tx_string(", Y: ");
            m_usb_tx_int(rinkXY[1]);
            m_usb_tx_string(", Angle: ");
            m_usb_tx_int(theta*180/3.14);
            m_usb_tx_string("\n");
            m_usb_tx_string("\n");*/
            
}