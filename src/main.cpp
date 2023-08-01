
/*------------------------------ Librairies ---------------------------------*/
#include "LibDreamT.h" // Vos propres librairies



/*---------------------------- Definitions du parcour -----------------------------*/
#define POSITION_DEPOS -0.30
#define OBSTACLE -0.10
#define BUCKET -0.30
#define PICK 0.30
#define CENTRE 0.0
#define EDGE 0.60
#define OFFSET_OSCILLATION 0.35

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

DT_pid *ptrPID_omega_fetch= new DT_pid(&SP_position,(float*) &(BNO->currentOmega), (float) 0.005,(float) 0.0000, (float) 0.0003);
DT_pid *ptrPID_omega_ship= new DT_pid(&SP_position,(float*) &(BNO->currentOmega), (float) 0.003,(float) 0.0000, (float) 0.0001);
DT_pid *ptrPID_position= new DT_pid(&SP_position,(float*) &(DenisCodeur->Position), (float) 5,(float) 0.00001, (float) 0.00001);
DT_pid *ptrPID_oscille = new DT_pid(&SP_position,(float*) &(DenisCodeur->Position), (float) 5,(float) 0.00001, (float) 0.00001);

/*------------------------A ESSAYER/FAIRE
  1- rajouter du D a omega_fetch pour qu'il ralentisse plus tot
  2- Ajuster la traverse pour mieu amortir le pendule rendu l'autre coter
  3- Lorsque le pickup du sapin est set up: mettre une condition pour switcher de l'etat 4 a 5
*/
  while(1)
  {
    BNO->setOmega();
    Barriere_Virtuelle_global=GAIN_BARRIERE_VIRTUELLE * ((1/(POSITION_FIN_RAIL-DenisCodeur->getPosition())+K_OFFSETPOSITIF)*(DenisCodeur->getVitesse()>0)+(1/(POSITION_DEBUT_RAIL-DenisCodeur->getPosition())+K_OFFSETNEGATIF)*(DenisCodeur->getVitesse()<0));
    Serial.println(DenisCodeur->LaserTag);
   switch (Etat)
    {
        case 0 :
          AX_.setMotorPWM(0 , 0);
          Magnet(MagnetOff);
            break;
        
        case 11 : 
          //Avance au centre rapidement
          SP_position= CENTRE ;
          fct_PID_position(DenisCodeur , ptrPID_position);

          if(DenisCodeur->Position > -0.05 && DenisCodeur->Position < 0.1)
            {
              Etat = 1;
            }
            break;

        case 12 :
          //Stabilise
          fct_PID_omega(DenisCodeur , ptrPID_omega_fetch);
          //PID_absorbtion(BNO,DenisCodeur,0, 0.15, 0.0001, 0.5);
            if( abs(BNO->getOmega()) < 5)
              {
                Etat = 1;
              }
            break;

        case 1 : 
          //Recule lentement jusqu'au bout du rail
          AX_.setMotorPWM(0 , 0.2);
          Magnet(MagnetOn);
          if(DenisCodeur->Position > EDGE )
            {
              Etat = 2;
            }
            break;


        case 2 :
          //Stabilise
          fct_PID_omega(DenisCodeur , ptrPID_omega_fetch);
          //PID_absorbtion(BNO,DenisCodeur,0, 0.15, 0.0001, 0.5);
            if( abs(BNO->getOmega()) < 5)
              {
                Etat = 3;
              }
            break;

        case 3 : 
          //Avance vers le centre d'oscillation

          //SP_position= OBSTACLE + OFFSET_OSCILLATION  + 0.15;
          //fct_PID_position(DenisCodeur , ptrPID_position);

          AX_.setMotorPWM(0 , -0.2);

          if(DenisCodeur->Position < (OBSTACLE + OFFSET_OSCILLATION))
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
          SP_position=0.10*sin(2.0*PI*0.75*millis()/1000.0) + (OBSTACLE + OFFSET_OSCILLATION) ;
          fct_PID_oscille(DenisCodeur , ptrPID_position);
          if(BNO->getAngle() > 45)
          {
            Etat = 6;
          }
            break;

        case 6 :
          //Traverse
          SP_position= ( OBSTACLE - 0.25 );
          fct_PID_position(DenisCodeur , ptrPID_position);
          if(DenisCodeur->Position < ( OBSTACLE - 0.27 ) )
          {
            //AX_.setMotorPWM(0 , -0.2);
            //delay(750);
            Etat = 7;
          }
            break;
          
        
            
        case 7 :
          //Stabilise
          fct_PID_omega(DenisCodeur , ptrPID_omega_ship);
          if( abs(BNO->getOmega()) < 8 && abs(BNO->getAngle()) < 5)
              {
                Etat = 8;
              }
          break;

        case 8 :
          SP_position= BUCKET;
          fct_PID_position(DenisCodeur , ptrPID_position);
          if(DenisCodeur->Position < (BUCKET)  && BNO->currentOmega > OBSTACLE )
          {
            //AX_.setMotorPWM(0 , -0.2);
            //delay(750);
            Etat = 9;
          }
            break;

        case 9 :
          //Stabilise
          fct_PID_omega(DenisCodeur , ptrPID_omega_ship);
          if( abs(BNO->getOmega()) < 5 && abs(BNO->getAngle()) < 0.5)
              {
                Etat = 10;
              }
          break;

        case 10 :
          //Stabilise
          Magnet(MagnetOff);
          delay(10);
          Etat = 0;
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
