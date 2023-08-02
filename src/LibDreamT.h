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
double COEFFS[COLOFDATA][ROWOFDATA]= {{-0.065680201331172,  -0.062999376787044,  -0.060318552242914,  -0.057637727698783,  -0.054956903154654,  -0.052276078610524,  -0.049595254066394,  -0.046914429522268,  -0.044233604978137,  -0.041552780434007,  -0.038871955889879,  -0.036191131345748,  -0.033510306801619,  -0.030829482257490,  -0.028148657713360,  -0.025467833169231,  -0.022787008625101,  -0.020106184080971,  -0.017425359536842,  -0.014744534992712,  -0.012063710448583,  -0.009382885904453,  -0.006702061360324,  -0.004021236816194,  -0.001340412272065,   0.001340412272065,   0.004021236816194,   0.006702061360324,   0.009382885904453,   0.012063710448583,   0.014744534992712,   0.017425359536842,   0.020106184080971,   0.022787008625101,   0.025467833169231,   0.028148657713360,   0.030829482257490,   0.033510306801619,   0.036191131345748,   0.038871955889879,   0.041552780434007,   0.044233604978137,   0.046914429522268,   0.049595254066394,   0.052276078610524,   0.054956903154654,   0.057637727698783,   0.060318552242914,   0.062999376787044,   0.065680201331172}
 ,{0.942742701577726,   0.904263407635780,   0.865784113693833,   0.827304819751880,   0.788825525809933,   0.750346231867985,   0.711866937926035,   0.673387643984096,   0.634908350042144,   0.596429056100194,   0.557949762158250,   0.519470468216298,   0.480991174274351,   0.442511880332405,   0.404032586390456,   0.365553292448508,   0.327073998506559,   0.288594704564609,   0.250115410622663,   0.211636116680715,   0.173156822738766,   0.134677528796818,   0.096198234854870,   0.057718940912922,   0.019239646970974,  -0.019239646970974,  -0.057718940912922,  -0.096198234854870,  -0.134677528796818,  -0.173156822738766,  -0.211636116680715,  -0.250115410622663,  -0.288594704564609,  -0.327073998506559,  -0.365553292448508,  -0.404032586390456,  -0.442511880332405,  -0.480991174274351,  -0.519470468216298,  -0.557949762158250,  -0.596429056100194,  -0.634908350042144,  -0.673387643984096,  -0.711866937926035,  -0.750346231867985,  -0.788825525809933,  -0.827304819751880,  -0.865784113693833,  -0.904263407635780,  -0.942742701577726}
 ,{0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0}
 ,{  -6.520975240086754,  -6.254812985389339,  -5.988650730691925,  -5.722488475994494,  -5.456326221297078,  -5.190163966599660,  -4.924001711902234,  -4.657839457204838,  -4.391677202507410,  -4.125514947809988,  -3.859352693112578,  -3.593190438415150,  -3.327028183717732,  -3.060865929020320,  -2.794703674322901,  -2.528541419625481,  -2.262379164928060,  -1.996216910230636,  -1.730054655533221,  -1.463892400835803,  -1.197730146138384,  -0.931567891440965,  -0.665405636743545,  -0.399243382046128,  -0.133081127348709,   0.133081127348709,   0.399243382046128,   0.665405636743545,   0.931567891440965,   1.197730146138384,   1.463892400835803,   1.730054655533221,   1.996216910230636,   2.262379164928060,   2.528541419625481,   2.794703674322901,   3.060865929020320,   3.327028183717732,   3.593190438415150,   3.859352693112578,   4.125514947809988,   4.391677202507410,   4.657839457204838,   4.924001711902234,   5.190163966599660,   5.456326221297078,   5.722488475994494,   5.988650730691925,   6.254812985389339,   6.520975240086754}
 ,{10.352909274358135,   9.930341548874132,   9.507773823390130,   9.085206097906115,   8.662638372422109,   8.240070646938102,   7.817502921454091,   7.394935195970106,   6.972367470486094,   6.549799745002084,   6.127232019518088,   5.704664294034073,   5.282096568550069,   4.859528843066069,   4.436961117582064,   4.014393392098057,   3.591825666614049,   3.169257941130037,   2.746690215646035,   2.324122490162031,   1.901554764678025,   1.478987039194019,   1.056419313710012,   0.633851588226008,   0.211283862742003,  -0.211283862742003,  -0.633851588226008,  -1.056419313710012,  -1.478987039194019,  -1.901554764678025,  -2.324122490162031,  -2.746690215646035,  -3.169257941130037,  -3.591825666614049,  -4.014393392098057,  -4.436961117582064,  -4.859528843066069,  -5.282096568550069,  -5.704664294034073,  -6.127232019518088,  -6.549799745002084,  -6.972367470486094,  -7.394935195970106,  -7.817502921454091,  -8.240070646938102,  -8.662638372422109,  -9.085206097906115,  -9.507773823390130,  -9.930341548874132, -10.352909274358135}
 ,{-5.695643704190692,  -5.463168450958419,  -5.230693197726146,  -4.998217944493871,  -4.765742691261598,  -4.533267438029323,  -4.300792184797049,  -4.068316931564781,  -3.835841678332507,  -3.603366425100233,  -3.370891171867962,  -3.138415918635686,  -2.905940665403414,  -2.673465412171142,  -2.440990158938870,  -2.208514905706596,  -1.976039652474322,  -1.743564399242047,  -1.511089146009775,  -1.278613892777502,  -1.046138639545229,  -0.813663386312956,  -0.581188133080682,  -0.348712879848410,  -0.116237626616137,   0.116237626616137,   0.348712879848410,   0.581188133080682,   0.813663386312956,   1.046138639545229,   1.278613892777502,   1.511089146009775,   1.743564399242047,   1.976039652474322,   2.208514905706596,   2.440990158938870,   2.673465412171142,   2.905940665403414,   3.138415918635686,   3.370891171867962,   3.603366425100233,   3.835841678332507,   4.068316931564781,   4.300792184797049,   4.533267438029323,   4.765742691261598,   4.998217944493871,   5.230693197726146,   5.463168450958419,   5.695643704190692}
 ,{-0.000000000000002,  -0.000000000000003,  -0.000000000000002,  -0.000000000000003,  -0.000000000000001,  -0.000000000000001,  -0.000000000000002,  -0.000000000000001,  -0.000000000000001,  -0.000000000000001,  -0.000000000000001,  -0.000000000000002,  -0.000000000000001,  -0.000000000000001,  -0.000000000000001,  -0.000000000000001,  -0.000000000000000,  -0.000000000000001,  -0.000000000000000,  -0.000000000000001,  -0.000000000000001,  -0.000000000000000,  -0.000000000000000,  -0.000000000000000,  -0.000000000000000,   0.000000000000000,   0.000000000000000,   0.000000000000000,   0.000000000000000,   0.000000000000001,   0.000000000000001,   0.000000000000000,   0.000000000000001,   0.000000000000000,   0.000000000000001,   0.000000000000001,   0.000000000000001,   0.000000000000001,   0.000000000000002,   0.000000000000001,   0.000000000000001,   0.000000000000001,   0.000000000000001,   0.000000000000002,   0.000000000000001,   0.000000000000001,   0.000000000000003,   0.000000000000002,   0.000000000000003,   0.000000000000002}};


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