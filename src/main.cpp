
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
float SP_Vitesse=0;
DT_pid *ptrPID_position= new DT_pid(&SP_position,(float*) &(DenisCodeur->Position), (float) 1.75 ,(float) 0.00001, (float) 0.00001);
DT_pid *ptrPID_vitesse= new DT_pid(&SP_Vitesse,(float*) &(DenisCodeur->Vitesse), (float) 1 ,(float) 0.00001, (float) 0.00001);



  while(1)
  {
    //PID_absorbtion(BNO, 0.1, 0.0001, 0.0001);
	
    fct_PID_position(DenisCodeur, ptrPID_position,ptrPID_vitesse);
    
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
