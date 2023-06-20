
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
  sensors_event_t event;
bno.getEvent( &event );


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


AnglesPendule = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
AnglesPendule.x();
OmegaPendule = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
OmegaPendule.x();



  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  
  // mise à jour du PID
  pid_.run();
}