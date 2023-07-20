
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
DenisCodeur = new encodeur;


  while(1)
  {
    PID_absorbtion(BNO, DenisCodeur, 0, 0.15, 0.0001, 0.5);
    PID_absorbtion(BNO, DenisCodeur, 0, 0.15, 0.0001, 0);
    //PID_absorbtion(BNO, -0.03, -0.0001, -0.0001);

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

    // mise a jour des chronometres
    timerSendMsg_.update();
    timerPulse_.update();
  
    // mise à jour du PID
    pid_.run();
    //allo++;
  }
}