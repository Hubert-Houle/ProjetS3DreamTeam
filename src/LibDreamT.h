#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include "RB-See-473.h"


//------------------------------ Constantes ---------------------------------

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          32          // J18     Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

#define MagnetOn        1
#define MagnetOff       0

//---------------------------- variables globales ---------------------------
ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_;                           // objet PID
ArduinoX deez;                      // objet moeur

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

Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> AnglesPendule;
imu::Vector<3> OmegaPendule;

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
bool PIDabsorbtion();

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

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
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
}


//--  Init  ---------------------------------------------------------------------
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
  //timerPulse_.setCallback(endPulse);
  
  // Initialisation du PID
  pid_.setGains(0.25,0.1 ,0);
  // Attache des fonctions de retour
  pid_.setMeasurementFunc(PIDmeasurement);
  pid_.setCommandFunc(PIDcommand);
  pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(0.001);
  pid_.setPeriod(200);
  //IMU-------------

  //PinMode
  pinMode(MAGPIN, OUTPUT); // Definition du IO

  deez.init();
  //return ;
}

//--  PID  -----------------------------------------------------------------------
double PIDmeasurement(){
  // To do
  return 0;
}
void PIDcommand(double cmd){
  // To do
}
void PIDgoalReached(){
  // To do
}

bool PIDabsorbtion(){
  delay(50);
  float mesure_angle; // A ENLEVER ET REMPLACER PAR VALEUR DU BNO   en deg
  float mesure_vitesse_angulaire; // A ENLEVER ET REMPLACER PAR VALEUR DU BNO   en deg/s

  float erreur_live = 0;
  float erreur_tot = 0;
  float erreur_avant = 0;
  float temp = 0;
  float pid = 0;
  float kp = 0.004125;
  float ki = 0.0001;
  float kd = 0.004125;

  while(mesure_angle<-10 && mesure_angle>10 && mesure_vitesse_angulaire>20){
    erreur_live = mesure_angle + mesure_vitesse_angulaire/2;
    erreur_tot = erreur_tot + erreur_live;
    erreur_avant = temp - erreur_live;
    temp = erreur_live;

    pid = kp*erreur_live + ki*erreur_tot + kd*erreur_avant;
    if(pid>1){pid=1;}
    if(pid<-1){pid=-1;}
    deez.setMotorPWM(0, pid);
  }
  deez.setMotorPWM(0, 0);
  Serial.println("Bien joue le bot");
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

//--  BNO055  ---(fuck jo)--------------------------------------------------------------------
/*float AlphaPendule(float TableauBNO[4] )
{
  float LiveTime = millis();

  float PrevOmega = TableauBNO[0];
  float PrevTime = TableauBNO[1];
  float LiveOmega = TableauBNO[2];

  float DeltaTime = LiveTime - PrevTime;
  float DeltaOmega = LiveOmega - PrevOmega;
  float Alpha = DeltaOmega/DeltaTime;

  TableauBNO[0]= LiveOmega;
  TableauBNO[1]= LiveTime;
  TableauBNO[3]= Alpha;

  return TableauBNO;

}*/



//--  Rb-see-473  ---(fuck jo)-----------------------------------------------------

