#ifndef LIBDREAMT_H
#define LIBDREAMT_H

#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include "RB-See-473.h"
#include "bnoRead.h"
#include "encodeur.h"
#include "DT_pid.h"
#include "math.h"




//------------------------------ Constantes ---------------------------------

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  10           // Periode (ms) d'envoie d'etat general

#define MAGPIN          32          // J18     Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

#define MagnetOn        1
#define MagnetOff       0

#define POSITION_FIN_RAIL 1.7
#define POSITION_DEBUT_RAIL 0
#define GAIN_BARRIERE_VIRTUELLE 0.05
#define K_OFFSETPOSITIF -0.03
#define K_OFFSETNEGATIF 0.04

#define ROWOFDATA 50
#define COLOFDATA 7
#define FINRAIL 1.7
#define DEBUTRAIL 0

//---------------------------- variables globales ---------------------------
ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_;                           // objet PID
bnoRead *BNO;                       // objet BNO
encodeur *DenisCodeur;              // objet encodeur

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 0;                // Amplitude de la tension au moteur [-1,1]

float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

// MEF
int Etat = 0;

//Adafruit_BNO055 bno = Adafruit_BNO055(55);
//imu::Vector<3> AnglesPendule;
//imu::Vector<3> OmegaPendule;


float Barriere_Virtuelle_global;
float CV_luts[ROWOFDATA]={-1.000000000000000,-0.959183673469388,-0.918367346938775,-0.877551020408163,-0.836734693877551,-0.795918367346939,-0.755102040816326,-0.714285714285714,-0.673469387755102,-0.632653061224490,-0.591836734693878,-0.551020408163265,-0.510204081632653,-0.469387755102041,-0.428571428571429,-0.387755102040816,-0.346938775510204,-0.306122448979592,-0.265306122448980,-0.224489795918367,-0.183673469387755,-0.142857142857143,-0.102040816326531,-0.061224489795918,-0.020408163265306, 0.020408163265306, 0.061224489795918, 0.102040816326531, 0.142857142857143, 0.183673469387755, 0.224489795918367, 0.265306122448980, 0.306122448979592, 0.346938775510204, 0.387755102040816, 0.428571428571429, 0.469387755102041, 0.510204081632653, 0.551020408163265, 0.591836734693878, 0.632653061224490, 0.673469387755102, 0.714285714285714, 0.755102040816326, 0.795918367346939, 0.836734693877551, 0.877551020408163, 0.918367346938775, 0.959183673469388, 1.000000000000000};
double COEFFS[COLOFDATA][ROWOFDATA]={ { -1.586827596759865,  -1.522059123422732,  -1.457290650085590,  -1.392522176748469,  -1.327753703411311,  -1.262985230074186,  -1.198216756737043,  -1.133448283399900,  -1.068679810062772,  -1.003911336725625,  -0.939142863388496,  -0.874374390051354,  -0.809605916714216,  -0.744837443377080,  -0.680068970039948,  -0.615300496702809,  -0.550532023365673,  -0.485763550028531,  -0.420995076691394,  -0.356226603354258,  -0.291458130017118,  -0.226689656679983,  -0.161921183342844,  -0.097152710005706,  -0.032384236668569,   0.032384236668569,   0.097152710005706,   0.161921183342844,   0.226689656679983,   0.291458130017118,   0.356226603354258,   0.420995076691394,   0.485763550028531,   0.550532023365673,   0.615300496702809,   0.680068970039948,   0.744837443377080,   0.809605916714216,   0.874374390051354,   0.939142863388496,   1.003911336725625,   1.068679810062772,   1.133448283399900,   1.198216756737043,   1.262985230074186,   1.327753703411311,   1.392522176748469,   1.457290650085590,   1.522059123422732,   1.586827596759865}
 , { 6.188627627363953,   5.683433535334240,   5.936030581349106,   5.430836489319420,   5.178239443304518,   4.925642397289692,   4.673045351274825,   4.420448305259960,   4.167851259245123,   3.915254213230249,   3.662657167215409,   3.410060121200543,   3.157463075185688,   2.904866029170833,   2.652268983155991,   2.399671937141133,   2.147074891126279,   1.894477845111415,   1.641880799096561,   1.389283753081708,   1.136686707066848,   0.884089661051997,   0.631492615037139,   0.378895569022283,   0.126298523007428,  -0.126298523007428,  -0.378895569022283,  -0.631492615037139,  -0.884089661051997,  -1.136686707066848,  -1.389283753081708,  -1.641880799096561,  -1.894477845111415,  -2.147074891126279,  -2.399671937141133,  -2.652268983155991,  -2.904866029170833,  -3.157463075185688,  -3.410060121200543,  -3.662657167215409,  -3.915254213230249,  -4.167851259245123,  -4.420448305259960,  -4.673045351274825,  -4.925642397289692,  -5.178239443304518,  -5.430836489319420,  -5.683433535334240,  -5.936030581349106,  -6.188627627363953}
 , { 0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0}
 , { -17.431301150409375, -16.008337791192275, -16.719819470800843, -15.296856111583782, -14.585374431975168, -13.873892752366663, -13.162411072758102, -12.450929393149545, -11.739447713541027, -11.027966033932453, -10.316484354323926,  -9.605002674715365,  -8.893520995106822,  -8.182039315498274,  -7.470557635889746,  -6.759075956281200,  -6.047594276672654,  -5.336112597064093,  -4.624630917455553,  -3.913149237847008,  -3.201667558238456,  -2.490185878629916,  -1.778704199021365,  -1.067222519412819,  -0.355740839804273,   0.355740839804273,   1.067222519412819,   1.778704199021365,   2.490185878629916,   3.201667558238456,   3.913149237847008,   4.624630917455553,   5.336112597064093,   6.047594276672654,   6.759075956281200,   7.470557635889746,   8.182039315498274,   8.893520995106822,   9.605002674715365,  10.316484354323926,  11.027966033932453,  11.739447713541027,  12.450929393149545,  13.162411072758102,  13.873892752366663,  14.585374431975168,  15.296856111583782,  16.008337791192275,  16.719819470800843,  17.431301150409375}
 , { 19.586088873418664,  17.987224475588558,  18.786656674503629,  17.187792276673555,  16.388360077758456,  15.588927878843439,  14.789495679928379,  13.990063481013326,  13.190631282098300,  12.391199083183229,  11.591766884268196,  10.792334685353138,   9.992902486438092,   9.193470287523043,   8.394038088608008,   7.594605889692962,   6.795173690777915,   5.995741491862854,   5.196309292947813,   4.396877094032765,   3.597444895117713,   2.798012696202670,   1.998580497287619,   1.199148298372571,   0.399716099457524,  -0.399716099457524,  -1.199148298372571,  -1.998580497287619,  -2.798012696202670,  -3.597444895117713,  -4.396877094032765,  -5.196309292947813,  -5.995741491862854,  -6.795173690777915,  -7.594605889692962,  -8.394038088608008,  -9.193470287523043,  -9.992902486438092, -10.792334685353138, -11.591766884268196, -12.391199083183229, -13.190631282098300, -13.990063481013326, -14.789495679928379, -15.588927878843439, -16.388360077758456, -17.187792276673555, -17.987224475588558, -18.786656674503629, -19.586088873418664}
 , { -7.786576168928998,  -7.150937297996014,  -7.468756733462511,  -6.833117862529535,  -6.515298427063035,  -6.197478991596552,  -5.879659556130059,  -5.561840120663570,  -5.244020685197084,  -4.926201249730590,  -4.608381814264102,  -4.290562378797611,  -3.972742943331121,  -3.654923507864630,  -3.337104072398142,  -3.019284636931654,  -2.701465201465165,  -2.383645765998672,  -2.065826330532184,  -1.748006895065694,  -1.430187459599203,  -1.112368024132714,  -0.794548588666224,  -0.476729153199735,  -0.158909717733245,   0.158909717733245,   0.476729153199735,   0.794548588666224,   1.112368024132714,   1.430187459599203,   1.748006895065694,   2.065826330532184,   2.383645765998672,   2.701465201465165,   3.019284636931654,   3.337104072398142,   3.654923507864630,   3.972742943331121,   4.290562378797611,   4.608381814264102,   4.926201249730590,   5.244020685197084,   5.561840120663570,   5.879659556130059,   6.197478991596552,   6.515298427063035,   6.833117862529535,   7.150937297996014,   7.468756733462511,   7.786576168928998}
 , { -0.000000000000003,  -0.000000000000004,  -0.000000000000004,  -0.000000000000004,  -0.000000000000003,  -0.000000000000003,  -0.000000000000003,  -0.000000000000003,  -0.000000000000003,  -0.000000000000002,  -0.000000000000002,  -0.000000000000002,  -0.000000000000002,  -0.000000000000002,  -0.000000000000002,  -0.000000000000002,  -0.000000000000002,  -0.000000000000001,  -0.000000000000001,  -0.000000000000001,  -0.000000000000001,  -0.000000000000001,  -0.000000000000000,  -0.000000000000000,  -0.000000000000000,   0.000000000000000,   0.000000000000000,   0.000000000000000,   0.000000000000001,   0.000000000000001,   0.000000000000001,   0.000000000000001,   0.000000000000001,   0.000000000000002,   0.000000000000002,   0.000000000000002,   0.000000000000002,   0.000000000000002,   0.000000000000002,   0.000000000000002,   0.000000000000002,   0.000000000000003,   0.000000000000003,   0.000000000000003,   0.000000000000003,   0.000000000000003,   0.000000000000004,   0.000000000000004,   0.000000000000004,   0.000000000000003}};



float BarriereVirtuelle(float cv_Actuel,float x);



//------------------------- Prototypes de fonctions -------------------------
void InitDream();
void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();
  // Magnet
void digitalWrite(uint8_t pin, uint8_t val);
void Magnet();
  // PID
double PIDmeasurement();
void PIDcommand(double cmd);
void PIDgoalReached();
bool PID_absorbtion(bnoRead* bNo, DT_pid* pid);
  //Motor
void getDataEncoder(double tableauEncodor[4]);
//int32_t ArduinoX::readEncoder(uint8_t id);
//int32_t ArduinoX::readResetEncoder(uint8_t id);
//void ArduinoX::resetEncoder(uint8_t id);


//________________________Definition de fonctions ______________________________
void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
  AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}


//-------------------------------------------------------------------------------
void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(0,0);
  AX_.setMotorPWM(1,0);
  timerPulse_.disable();
  isInPulse_ = false;
}


//-------------------------------------------------------------------------------
void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pid_.getGoal();
  doc["measurements"] = PIDmeasurement();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent(); 
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
  doc["accelX"] = imu_.getAccelX();
  doc["accelY"] = imu_.getAccelY();
  doc["accelZ"] = imu_.getAccelZ();
  doc["gyroX"] = imu_.getGyroX();
  doc["gyroY"] = imu_.getGyroY();
  doc["gyroZ"] = imu_.getGyroZ();
  doc["isGoal"] = pid_.isAtGoal();
  doc["actualTime"] = pid_.getActualDt();
  doc["Angle"] = BNO->getAngle();
  doc["Vitesse Angulaire"] = BNO->getOmega();
  doc["Alpha"] = BNO->getAlpha();
  doc["Position"] = DenisCodeur->getPosition();
  doc["Vitesse"] = DenisCodeur->getVitesse();
  doc["Acceleration"] = DenisCodeur->getAccel();
  doc["Etat"] = Etat;
  doc["barriere"] = Barriere_Virtuelle_global;
  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println(); //NE PAS COMMENTER, IMPORTANT POUR LE FORMAT DU MESSAGE
  shouldSend_ = false;
}


//-------------------------------------------------------------------------------
void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Analyse des éléments du message message
  parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     pulsePWM_ = doc["pulsePWM"].as<float>();
  }

  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     pulseTime_ = doc["pulseTime"].as<float>();
  }

  parse_msg = doc["pulse"];
  if(!parse_msg.isNull()){
     shouldPulse_ = doc["pulse"];
  }
  parse_msg = doc["setGoal"];
  if(!parse_msg.isNull()){
    pid_.disable();
    pid_.setGains(doc["setGoal"][0], doc["setGoal"][1], doc["setGoal"][2]);
    pid_.setEpsilon(doc["setGoal"][3]);
    pid_.setGoal(doc["setGoal"][4]);
    pid_.enable();
  }
  parse_msg = doc["Etat"];
  if(!parse_msg.isNull())
    {
      Etat = doc["Etat"].as<int>();
    }
  
}


//--  Init  ~~~~____~~~~____~~~~____~~~~____~~~~____~~~~_____~~~~____~~~~____~~~~
void InitDream()
{

  AX_.init();                       // initialisation de la carte ArduinoX 
 

  //bno.setExtCrystalUse(true);
  Serial.begin(BAUD);               // initialisation de la communication serielle
  Serial.println("criss");
  //imu_.init();   
   Serial.println("criss2");                   // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);
   Serial.println("criss3");            // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
   Serial.println("criss4");
  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
  
  // Initialisation du PID
  pid_.setGains(0.25,0.1 ,0);
  // Attache des fonctions de retour
  pid_.setMeasurementFunc(PIDmeasurement);
  pid_.setCommandFunc(PIDcommand);
  pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(0.001);
  pid_.setPeriod(200);

  //IMU-------------
    //BNO = new bnoRead;
  //PinMode
  pinMode(MAGPIN, OUTPUT); // Definition du IO




  return ;
}

//--  PID  -----------------------------------------------------------------------
double PIDmeasurement(){
  double GROSSE_PUTE = 0;
  return GROSSE_PUTE;
}
void PIDcommand(double cmd){
  // To do
}
void PIDgoalReached(){
  // To do
}
bool PID_absorbtion(bnoRead* bNo, encodeur* denis, bool magnet, float kp, float ki, float kd){
  double tot;
  float valeur[3] = {kp, ki, kd};
  if(magnet==1){valeur[0]=kp*0.25;valeur[1]=ki*0.25;valeur[2]=kd*0.25;}
  float zero = 0;
  BNO->setOmega();
  float angle = bNo->getAngle();
  float omega = bNo->getOmega();
  float pos_goal = denis->getPosition();
  float pos = denis->getPosition();
  DT_pid* PID_angle = new DT_pid(&zero, &angle, 2*valeur[0]/3, 2*valeur[1]/3, 2*valeur[2]/3);
  DT_pid* PID_omega = new DT_pid(&zero, &omega, valeur[0]/3, valeur[1]/3, valeur[2]/3);
  DT_pid* PID_pos = new DT_pid(&pos_goal, &pos, valeur[0]/3, valeur[1]/3, valeur[2]/3);

  while((angle<-7 || angle>7) && (omega>10 || omega<-10)){
    tot = PID_angle->response_Sum() + PID_omega->response_Sum();
    Serial.println(PID_pos->response_Sum());
    if(tot>1){tot=1;}
    if(tot<-1){tot=-1;}
    AX_.setMotorPWM(0, tot);

    angle = bNo->getAngle();
    omega = bNo->getOmega();
    pos = denis->getPosition();
  }
  AX_.setMotorPWM(0, 0);
  //Serial.println("Bien joue le bot!");
  delete PID_angle;
  delete PID_omega;
  return 1;
}


//--  Aimant  -----------------------------------------------------------------------
void Magnet(bool StateMagnet)
{
  if (StateMagnet == 1 )
  {
    digitalWrite(MAGPIN , HIGH);
  }
  else 
  {
   digitalWrite(MAGPIN , LOW );
  }
}

//oks


//--  PID_lineaire -------------------------------------
 bool fct_PID_position(encodeur* enCodeur, DT_pid* pid1){

  float CV;

  

	CV=pid1->response_Sum(); 

  CV=BarriereVirtuelle(CV,enCodeur->Position);
  
	AX_.setMotorPWM(0, (CV>1) ? 1: ((CV<-1) ? -1 : CV));  // retourner CV si entre 1 et -1 sinon retourner 1 ou -1
	
	return 1;

 	
 }

 bool fct_PID_omega(encodeur* enCodeur, DT_pid* pid1){

  float CV;

  CV=BarriereVirtuelle(CV,enCodeur->Position);

	CV=pid1->response_Sum();

  
	AX_.setMotorPWM(0, (CV>1) ? 1: ((CV<-1) ? -1 : CV));  // retourner CV si entre 1 et -1 sinon retourner 1 ou -1
	
	return 1;

 	
 }

  bool fct_PID_oscille(encodeur* enCodeur, DT_pid* pid1){

  float CV;

  

	CV=pid1->response_Sum();

  
	AX_.setMotorPWM(0, (CV>1) ? 1: ((CV<-1) ? -1 : CV));  // retourner CV si entre 1 et -1 sinon retourner 1 ou -1
	
	return 1;

 	
 }
//--  Motor -------------------------------------
//  tableau[0] = previus Time
//  tableau[1] = previus pulse
//  tableau[2] = previus motor speed
//  tableau[3] = previus motor acceleration




void getDataEncoder(double *tableauEncoder)
{  
  double R = 0.035;
  double C = 3.1416*(2*R);
  double Kg = 18.75/2; 
  double ppt = 64;
  double dpp = C/(ppt*Kg);
  

    double PrevTime = tableauEncoder[0];
    double PrevPulse = tableauEncoder[1];
    double PrevVitesse = tableauEncoder[2];
    double millisecondes = millis();

    //Serial.println("Millis: ");
    //Serial.println(millisecondes , 16);
    double LiveTime = millisecondes/1000.0;

    //Serial.println("Livetime: ");
    //Serial.println(LiveTime , 16);

    //Serial.println(PrevPulse , 16);

    double LivePulse = AX_.readEncoder(0)*dpp;
  
    double DeltaTime = LiveTime - PrevTime;

    //Serial.println("Prevtime: ");
    //Serial.println(PrevTime , 16);
    //Serial.println(DeltaTime);


      // vitesse
    double DeltaPulse = LivePulse - PrevPulse;
    double LiveVitesse = DeltaPulse / DeltaTime;
    
      // acceleration
    double DeltaVitesse = LiveVitesse - PrevVitesse;

    //Serial.println("DeltaVitesse: ");
    //Serial.println(DeltaVitesse , 16);

    //Serial.println("DeltaTime: ");
    //Serial.println(DeltaTime , 16);
    double AccMotor = DeltaVitesse / DeltaTime;
    //Serial.println("AccMotor: ");
    //Serial.println(AccMotor , 16);

    tableauEncoder[0] = LiveTime;
    tableauEncoder[1] = LivePulse;
    tableauEncoder[2] = LiveVitesse;
    tableauEncoder[3] = AccMotor;
}

float BarriereVirtuelle(float cv_Actuel, float x)
{
  x=-1*x;
	if(x>FINRAIL || x<DEBUTRAIL){return 0;}

	float erreurabs[ROWOFDATA];
	float erreurmin= 100;
	int Ind;
	for(int i=0;i<ROWOFDATA;i++)
	{
		erreurabs[i]= abs(CV_luts[i]-cv_Actuel);
		
		if (erreurabs[i]<erreurmin)
		{
			erreurmin=erreurabs[i];
			Ind=i;
		}
		
	}
	

	
	return COEFFS[0][Ind]*pow(x,6)+COEFFS[1][Ind]*pow(x,5)+COEFFS[2][Ind]*pow(x,4)+COEFFS[3][Ind]*pow(x,3)+COEFFS[4][Ind]*pow(x,2)+COEFFS[5][Ind]*x+COEFFS[6][Ind];
}



#endif


/*
                                                                          _..._       .-'''-.                                                       
_______                                                                .-'_..._''.   '   _    \ _______                                             
\  ___ `'.         __.....__        _..._   .--.                     .' .'      '.\/   /` '.   \\  ___ `'.         __.....__                        
 ' |--.\  \    .-''         '.    .'     '. |__|                    / .'          .   |     \  ' ' |--.\  \    .-''         '.                      
 | |    \  '  /     .-''"'-.  `. .   .-.   ..--.                   . '            |   '      |  '| |    \  '  /     .-''"'-.  `.           .-,.--.  
 | |     |  '/     /________\   \|  '   '  ||  |                   | |            \    \     / / | |     |  '/     /________\   \          |  .-. | 
 | |     |  ||                  ||  |   |  ||  |     _             | |             `.   ` ..' /  | |     |  ||                  |  _    _  | |  | | 
 | |     ' .'\    .-------------'|  |   |  ||  |   .' |            . '                '-...-'`   | |     ' .'\    .-------------' | '  / | | |  | | 
 | |___.' /'  \    '-.____...---.|  |   |  ||  |  .   | /           \ '.          .              | |___.' /'  \    '-.____...---..' | .' | | |  '-  
/_______.'/    `.             .' |  |   |  ||__|.'.'| |//            '. `._____.-'/             /_______.'/    `.             .' /  | /  | | |      
\_______|/       `''-...... -'   |  |   |  |  .'.'.-'  /               `-.______ /              \_______|/       `''-...... -'  |   `'.  | | |      
                                 |  |   |  |  .'   \_.'                         `                                               '   .'|  '/|_|      
                                 '--'   '--'                                                                                     `-'  `--'      
*/


/*
ooooooooooooooollllllloolloooooooooooolloooooolllllllllllloolooooooollc:;,,,;;;,''........',:clllllllllllllllcclllccccccccccccccclccccccccccllc:;:::::
oooooooooooooooooooooooooooollooooooooooooooooooooollllllloolollc:;;,,''',,,,,'''............',;cllllllccccccllcccccccccclllllllllollllllllllol:::::::
oooooooooooooooooooooooooooollooooooooooooooooooooooooollollc:;,'.'''''''.......'...............,;cllooolloolllllllllllllodddddoooddoodooooddddlcccccc
ooodoooooodoooooooooooooooooooooooooooooooooooooooooooooll:;;,,,,,,'..................'',,,,,,,,,;;cllooooolloollllllllclodddddoooddoooooodddxdlcccccc
ooooooooooooooooooooooooooooooooooooooooooooooooooooolc:;:::c::;;;,,;;;;;;,,,,,,,,,,,,;;;,,''',,,,,;::clllllolllllllllccclddoodoooddoooooodddxdlcccccc
ooooooooooooooooooooooooooooooooooooooooooooooooooooc:,,,;::::::::cccccc:::::::::::;;,;;;,,''..''....',;:cllllllllccccccclddoddoooddoooooddddxdlcccccc
ooooooooooooooooooooooooooloooooooooooooooooooooooc;''...'',,;;;;,,,;::::::::;;;;;,''''................'',;;clllllccccccclddoodoooddoooodddddddlcccccc
oooooooooooooooooolllloolllllllollllllllllooooooc,......'''.....'...'',''''',,,''''............... .......''';clollllllcclddodddoodddooodddddddllccccc
ooooooooooooooooooloooooooooooooooooooooooooooo:'................'''.....    ............       .   ..........';lllllllccoddddddooddddoodddddddllccccc
oooooooooooooooooooooooooooooooolloooooooooooo:'................''''''.......''...........      ...  .........'.,:lllllclodddoddoodddddddddddddlllcccc
ooooooooooooooooooooooooooooooooooooooooooool;..  ..........';:clooollllllccccll:;,'........... ..............''.';llllccoddooddooodddddddddddollccccc
oooooooooooooooooooooooooooooooooooooooooooo:'....',,'',;:codxkkOOOOOOOOOkkkkxxxdoolcc::;;;,'.....................';cllccldddoddddddddodddddddolcccccc
oooooooooooooooooooooooooollcclllloolooooool:'.'',,;;;;coxkkkkkkOOOOOOOOOOOOOOOOkkkxxxxddoolc:::;;,,;,,'............;:::::ldddddddddddoodddddddlcccccc
ooooolllooooooollloolllllccc::clllollllllolc;'...',;;:coxkkkkkkOOOOOOOOOOOOOOOOOOOkkkkkkxxxdddollcc:::;,'............,;:::ldddddddooddddddddddoc::cccc
lccllllllllllllllclllcc:;;:llcloooooooooool;'....',;:cloxkkkkkOOOOOOOOOOOOOOOOOOOOOkkkkkkxxxdddoollc:::;,...   ..   ..;c::oddddddddoooodddddddol:;;;::
ccclccclccclccclccccc:::cclolclooooddddoooc,'....',:ccldxkkkkOOOO0000000OOOOOOOOOOOkkkkkxxxxxdddoolc:::;;,....       .,c::oddddddddddoooddddddoooc;,,;
lllllllllllloooooolllc:llllolclooooddddoooc,.....',;clooxkkkOOO0000000000OOOOOOOOOOkkkkkkxxxxxddooolc::;;;'.....     .,c::odddddddooooooddddddooooc:''
ooooooooododddddddooollooloolclooodddddddoc,...'',;::clodkkkOO00000000000OOOOOOOOOkkkkkkkkxxxdddooolc::;;;,'....      'c::odddddddoooooododdddoodoll,.
odooooooooodddddddoloolooloolcloodddddddooc,,,,,;;;::cldxkkkOO000000000OOOOOOOOOkkkkkkkkkkkxxxddoooolc:;;;;'....      'c::odddddddooooooddddoooodool;'
dddooooddddddddooooooooooooolcloooddddddool;;,,,,,;:codxkkkkOOO000000000OOOOkOOkkkkkkkkkkkkxxxdddooolc:;;;;'....     .'c::oddddddoooooooodddddoodool;'
dddddddddddddooolooloooooooolcloodddddddddo:;''''',:okkkkkkkOOOOO00000000OOOOOkkkkOkkOOOkkkkxxddooolcc:;;;,....    ..';c::odddodddooooooddddddooddol;,
oododdddddddollllooooooodooolclodddddddddddl;'...',cxkkkkkkkOOOO0000KKK0000OOOOOOOOOOOOOOOkkkxxdooolc:;;;;,.....  ..,:ll::odddodddoooooodddooooodool;,
ddddxxddddddollllooooooodooooclddddddddddddo:'..'';okkkkkkkkkxxkO000KKK0000OOOOOOOOO00OOOOkkkxxdoool::;;;;,..  ....,ccll::odddddddoooooooodooooodool::
ddddddooddddollloooooooodoodoclodddddddddddxdc;,,,:oxkOkkxdol:::cloodxkkOOOOOOkkOOkkOOOkkOkkkxxddolc::;;;;,.    ...:cclc::odddddddoooooooooooooodollcc
ddxdddddddddollllooooooododdollodddddddddddxxkxc,;:cloxkxdlc;;;::::;;;;:codxxxxkOkxxxkxxddddoollcc:;,,,,,;;.    ..:llclc;:oddddddooolooooooooooodollcc
dxddddddddddollloooolooododdollodddddddddoxkkOko;,ldoc::lllllodxxxdlc:::ccloodxkOkxdodolc:;,,,,'''.....,,;;.   ..,lolllc::oddddddoooooooooodoooodllocc
dddddddddxddollooooooooododdolloddddddddooxOOOOd::dxkkdoolcccccloc,.';c::ccllooxkdoc:::;;;;:::cc:;,'..';;;;.  ..':oolllc::odddoodooooodooooooolooolocc
ooddddoddxddolloooooooooddddolclododdooooodO0OOxodxxkOOOOkdlcccoxdlccloc:loddook0Oxc'..',c:,'';:::;,,,,;::,...,;cooolclc::odddoodoooooddooooooloollocl
ooddddddddddolloooooooooddoddlclooodoooooook0OOkxxxxkOOOOOkkxxkkkkxxdooodxkkxxkO0Oxl,'',lddl:;::,'',,,;:;,'.',looooolccc;:odddoooooooodoooooooloollocl
odxdddddddddolloooooooooddoddlcloooooooooook0OOkxkxxxkOOOOOkkOOOOOOOOOOOOOOOOkOOOkxl;,,:oxxddlc:;,,,,;;;,''..,oddddolclc;:oddoooooooooddoooodoloolloll
ddxdoodddxxddlllloooooooddoddlcloooooooooookOOkxxkxdxkkOO000OOOOOOOOOOOOOOOOOOOOkxdl:;:lxkkkxdolc:::::;;;,,';coddddolclc;:oddooooooooodooooodoloooooll
xxddoodddxdddollloooooooddoddocloooooooooodkOOxxkkxxxkkOO0000000OOOOOOOOOOOOOOOOkxdlc:cldxkkOkkkxxddol:;;,,cooooodoocccc;:odddoooooooooooooodoloooooll
ddddodddddxxdolllooooooodddddocloooooooooodO0OxxkxxxxkkkOO00KKK00OOkxxxkkkOOOOOOOkdoc;;:lxOOkkkkkxxdlc;;,,;loooooooocccc;:odddoooooooooooooddoloooooll
dodddddddxxxdollooooollodddddocloooddddoooxOOkxxkxxxkkkkOO0KKK0OkxolodxkkOOOOOOOOkxl:;;,:okOOOOOkkxdl:;,,,;looooddoocclc::odddoooooooooooooodolodoooll
oddddddddddddolllooooooodddddocloddddddooodkOkxxxxxxkkkkOOO00OkxoccoxkkkOOOOOO00Okdl:;:;,;okOOOOkkdol:;,,,;looooddoocclc;:odddoooooooodooooodolodoooll
dddooddddddddolllooooooodooooocloddddddoooodkxxxxxxxkkkkkOOOkkdoccdkOkkxoooddxxxdoc;,,;;,';okOOOkxdlc:;,,,;looooddolcclc;:odddoooooooooooooddooooololo
dddooddddddddollooooooooddoodollodooooddooooodxxxxxxkkkkkkkkxdl:coxkkkkxxxxxdollc;'...''',,cxOOkkdolc;,,,,:loooodoolcclc;:odddooooooooddoooodooooooolo
dddooddddxxddollooooooooddoodollooooooooooooodxxxxxxkkkkkkkkxoccodkkkkkkkkkkxdodxoc:;;,,,;;cokkkxdol:;,,,,:oooooooolcclc;coddddoooooooddoooooolooooolo
oddddddddddddollooooooooodooddlcooooooooooooodxxxxxkkkkkkkkkxoclodxkkkkkkkkkkkxxxxdolc:;;::coxkxddlc;,,,,;loooooooolcclc;coddddooooooooooooooolodooolo
oddddddddddddollooooooooodooddlcooooooooooooooxxxxxkkkkkkkkkkdlcloddoooooooooddollcc:;,,,;codxkxdoc:,,,,,:loolloooolccc:;cddoddoooooooodoooooolooooolo
dddooddddxxddolllooooooooddoddlclooooooooooooodxxdxxkkkkkkkkkxoldkOOkxxxkkxxxddolc::;'''';odxkkxdlc;,,,,;looloooodolccc:;cdddddoooooooodoooooolooooolo
dddoodddddxddolllooooolloddoddlclooooooooooolcldxddxxkkOkkkkkxddkOOOOOOOOOOkkkxddolc;;;;:odxkOkxoc:;,,,;clooooooooolccl:;ldddddooooooooooooooolooooolo
dddddddddddxxdlllooooolloddoddoclooooddolc:;,.,oxxdddxxkkkkkkkxkkOOOOOOOOOkkkxddoolcc:cllodkOkxol:;;,,;cllooooooddolccl:;lddddddoooooooooooooolooooolo
ddddddddddddddllloooooooodddddolloool:;'.......;oxxdddxxxkkkkkkkkOOOO000OOOOkkxxdddoolllllokkxdoc:;,,;ldolllooooddolccl:;cddddddooooooodoooooolooooolo
dddddddddddxxdllloooooolodddddocc:;'.....'''''..;oxxddddxxxxxxkkkOOOOOOOOOOOOkkxxdooooolcclxxdol:;,,'.,::cloooooddolcclc;cdddddoooooooooooooooloollolo
dddddddddxxxddllloooooooodddoc;'.......'''..'''..;oxxddddxxxddxxkkkOOOOO00OOOOkxdoollllc:;coooc;,,,'.   ..',;cldxdolcllc;ldddddooooodooodoooolloollolo
ddddddddddddddllloooooooolc;'.........'''...''''..,ldxdddddddddddxkkkOOOOOkkkxxddolccc:;,,;::;,,,,'.        ...,:cllcllc:lddddddoooodooodoooolloollolo
ddddddddddxdddolooooooc:,'....................'''..':oddddddxdddooddxxkkkxddddddolc:;;,,',,,,,,,,,.              ..'';:;;lddddddoooodooodooooolooooolo
dddddddddxxdddollcc:;,.........................'''...:xxdoooddddddooooooooooooolc:;;,,,,,,,,,,,;;.                    ...',;:cloooooddoooooooolooooolo
dddddddxxdollc:,,'..................'...........''''''cOOxdoooddddddollccc:::::;;;,,,,,,,,,,,;;ll.                         ....',;:cloodddoddolodooolo
ddddoolc:;,''.......................'...........'''''''l0XKOxoooooooooollcc:::;,,,,,,,,,,,;;;coxd,                            .......',:codddooooolooo
ol:;,''...'.....................................''''''''l0NNX0kdooollllllccc::;;;;;,,;;;;;:lodxxx;                          .............',:cllodooooo
,'...............'...............................''''''''lKWWWNX0kdoooollcc:::;;,,;;;;;:codddxxxx:.                      .....................,;cloood
..........''''.'''................................''''''',dNWWWWNNKOkxdoolccc:;;;;;;:clddxxxxxxxxc.                   ...........................',:ld
.......'''''''''''''................................'''''':0WWWWWNNXXK0Oxdoll::;;:cloxxkkkOOOOOkxc.                 ................................';
......'''''''''''''.'..............................''''''''oXWWWWWNNXXXXXKOkdlccodkO0KKKKKXXXXKOxl. ....  .......  ...................................
.'...'''''''''''''..'.............................''''''''.;kNNWWWWNNNNNNNXX0xx0XNNNNNNNNNNNNXKOxl......  .......  ...................................
*/


//lol