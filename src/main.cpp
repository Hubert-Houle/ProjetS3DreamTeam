
/*------------------------------ Librairies ---------------------------------*/
#include "LibDreamT.h" // Vos propres librairies

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() 
{
 InitDream();

};






void loop() 
{ 
BNO = new bnoRead;
encodeur Denis_codeur;


  while(1)
  {
    //PID_absorbtion(BNO, -0.03, -0.0001, -0.0001);


    Serial.print("Position "); Serial.println(Denis_codeur.getPosition(),4);

    Serial.print("Vitesse "); Serial.println(Denis_codeur.getVitesse(),4);
    
    Serial.print("Acceleration "); Serial.println(Denis_codeur.getAccel(),4);




    //BNO->setOmega();
    //BNO->setAlpha();
    //BNO->getAngle();

    //Serial.print(">Omega:");
    //Serial.println(BNO->getOmega());
    //Serial.print(">Alpha:");
    //Serial.println(BNO->getAlpha());  
    //Serial.print(">Angle:");
    //Serial.println(BNO->getAngle());




 
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
    Magnet(MagnetOff);

  // BNO
  
    //BNO.setOmega();

    //Serial.println(BNO.setOmega());
    



    // mise a jour des chronometres
    //timerSendMsg_.update();
    //timerPulse_.update();
  
    // mise à jour du PID
    pid_.run();
    //allo++;
  }
}