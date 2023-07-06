
/*------------------------------ Librairies ---------------------------------*/
#include "LibDreamT.h" // Vos propres librairies

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() 
{
 InitDream();

};






void loop() 
{
  bnoRead BNO;


  while(1)
  {

    BNO.setOmega();
    BNO.setAlpha();
    BNO.getAngle();

    Serial.print(">Omega:");
    Serial.println(BNO.getOmega());
    Serial.print(">Alpha:");
    Serial.println(BNO.getAlpha());  
    Serial.print(">Angle:");
    Serial.println(BNO.getAngle());

  }


  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }


Magnet(MagnetOn);

// BNO








  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  
  // mise à jour du PID
  pid_.run();
}