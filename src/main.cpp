
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
float SP_position=-1;
DT_pid *ptrPID_position= new DT_pid(&SP_position,(float*) &(DenisCodeur->Position), (float) 1.75 ,(float) 0.00001, (float) 0.00001);




  while(1)
  {
    PID_absorbtion(BNO, 0.15, 0.0001, 0.5);
    PID_absorbtion(BNO, 0.15, 0.0001, 0);
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
  }
}
