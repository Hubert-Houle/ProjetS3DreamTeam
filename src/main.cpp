
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
float SP_position=0.5;
DT_pid *ptrPID_position= new DT_pid(&SP_position, &DenisCodeur.position, 0.1,0.001,0);



  while(1)
  {
    PID_absorbtion(BNO, 0.1, 0.0001, 0.0001);
	
    fct_PID_position(ptrPID_position);
 
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
    //pid_.run();
    //allo++;
  }
}
