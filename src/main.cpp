
/*------------------------------ Librairies ---------------------------------*/
#include "LibDreamT.h" // Vos propres librairies


/*---------------------------- fonctions "Main" -----------------------------*/

void setup() 
{
 InitDream();

};






/* Boucle principale (infinie)*/
void loop() 
{ 
bnoRead BNO;
float tableau[4];
float *tableauEncoder= tableau;
tableauEncoder[0]=0;
tableauEncoder[1]=0;
tableauEncoder[2]=0;
tableauEncoder[3]=0;

while(1)
{



 
    if(shouldRead_){
    readMsg();
  }
    if(shouldSend_){
    sendMsg();
  }
    if(shouldPulse_){
    startPulse();
  }

  // Magnet
    Magnet(MagnetOn);

  // BNO
  
    //BNO.setOmega();

    //Serial.println(BNO.setOmega());
    

  // Motor

    getDataEncoder(tableauEncoder);

    Serial.print("Time:");
    Serial.println(tableauEncoder[0]);
    Serial.print("Position:");
    Serial.println(tableauEncoder[1]);
    Serial.print("Vitesse:");
    Serial.println(tableauEncoder[2]);
    Serial.print("Acceleration:");
    Serial.println(tableauEncoder[3]);

    if (tableauEncoder[1]<500 )
    {
      AX_.setMotorPWM(0,0.12);
    }
    else
    {   AX_.setMotorPWM(0,0);}

/*
    // mise a jour des chronometres
    timerSendMsg_.update();
    timerPulse_.update();
  
    // mise à jour du PID
    pid_.run();*/
  }
}