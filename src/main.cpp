
/*------------------------------ Librairies ---------------------------------*/
#include "LibDreamT.h" // Vos propres librairies


/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
 InitDream();
}
//______________________________________________________________________







/* Boucle principale (infinie)*/
void loop() {

  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  
  // mise à jour du PID
  pid_.run();
}