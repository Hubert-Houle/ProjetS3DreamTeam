
/*------------------------------ Librairies ---------------------------------*/
#include "LibDreamT.h" // Vos propres librairies



/*---------------------------- Definitions du parcour -----------------------------*/
#define POSITION_DEPOS -0.3


/*---------------------------- fonctions "Main" -----------------------------*/

void setup() 
{
 InitDream();

};






void loop() 
{ 
BNO = new bnoRead;
DenisCodeur = new encodeur;
float SP_position=0;
DT_pid *ptrPID_omega_fetch= new DT_pid(&SP_position,(float*) &(BNO->currentOmega), (float) 0.005,(float) 0.0000, (float) 0.0001);
DT_pid *ptrPID_omega_ship= new DT_pid(&SP_position,(float*) &(BNO->currentOmega), (float) 0.003,(float) 0.0000, (float) 0.0001);
DT_pid *ptrPID_position= new DT_pid(&SP_position,(float*) &(DenisCodeur->Position), (float) 5,(float) 0.00001, (float) 0.00001);
DT_pid *ptrPID_oscille = new DT_pid(&SP_position,(float*) &(DenisCodeur->Position), (float) 5,(float) 0.00001, (float) 0.00001);

  while(1)
  {
    BNO->setOmega();
    
   switch (Etat)
    {
        case 0 :
          AX_.setMotorPWM(0 , 0);
          Magnet(MagnetOn);
            break;
        case 1 : 
          //Avance
          SP_position= 0.30 ;
          fct_PID_position(DenisCodeur , ptrPID_position);

          if(DenisCodeur->Position > 0.28 && DenisCodeur->Position < 0.42)
            {
              Etat = 2;
            }
            break;

        case 2 :
          //Stabilise
          fct_PID_omega(DenisCodeur , ptrPID_omega_fetch);
          //PID_absorbtion(BNO,DenisCodeur,0, 0.15, 0.0001, 0.5);
            break;

        case 3 : 
          //Avance
          SP_position= 0.40 ;
          fct_PID_position(DenisCodeur , ptrPID_position);

          if(DenisCodeur->Position < 0.41)
            {
              Etat = 4;
            }
            break;

        case 4 :
          //Stabilise
          fct_PID_omega(DenisCodeur , ptrPID_omega_fetch);
          //PID_absorbtion(BNO,DenisCodeur,0, 0.15, 0.0001, 0.5);
            break;

        case 5 :
          //Oscillation
          SP_position=0.10*sin(2.0*PI*0.75*millis()/1000.0)+0.40;
          fct_PID_oscille(DenisCodeur , ptrPID_position);
          if(BNO->getAngle() > 40)
          {
            Etat = 6;
          }
            break;

        case 6 :
          //Traverse
          SP_position= -0.20;
          fct_PID_position(DenisCodeur , ptrPID_position);
          if(DenisCodeur->Position < -0.19)
          {
            //AX_.setMotorPWM(0 , -0.2);
            //delay(750);
            Etat = 7;
          }
            break;
          
        case 7 :
          SP_position= -0.25;
          fct_PID_position(DenisCodeur , ptrPID_position);
          if(DenisCodeur->Position < -0.24  && BNO->currentOmega > 0)
          {
            //AX_.setMotorPWM(0 , -0.2);
            //delay(750);
            Etat = 8;
          }
            break;
            
        case 8 :
          //Stabilise
          fct_PID_omega(DenisCodeur , ptrPID_omega_ship);
          break;
        case 9 :
          //Stabilise
          Magnet(MagnetOff);
          break;
    }
   // SP_position=0.25*sin(2*PI*1*millis()/1000.0);
    //fct_PID_position(DenisCodeur , ptrPID_position ,ptrPID_position); 
    //PID_absorbtion(BNO,DenisCodeur,0, 0.15, 0.0001, 0.5); 
    
   // PID_absorbtion(BNO, 0.15, 0.0001, 0.5);
    //PID_absorbtion(BNO, 0.15, 0.0001, 0);
    //PID_absorbtion(BNO, -0.03, -0.0001, -0.0001);
    if(Serial.available() > 0)
    {
      serialEvent();
    }
    if(shouldRead_){
    Serial.println("LIS MON OSTI DE MESSAGE CALAAISESESESESE");
    readMsg();
    }
    if(shouldSend_){
    sendMsg();
  }
    if(shouldPulse_){
    startPulse();
  }

  // Magnet
   // Magnet(MagnetOff);

    // mise a jour des chronometres
    timerSendMsg_.update();
    timerPulse_.update();
    

  }
}
