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

//Adafruit_BNO055 bno = Adafruit_BNO055(55);
//imu::Vector<3> AnglesPendule;
//imu::Vector<3> OmegaPendule;

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
  doc["Position"] = DenisCodeur->getPosition();
  doc["Vitesse"] = DenisCodeur->getVitesse();
  doc["Acceleration"] = DenisCodeur->getAccel();
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
  Serial.println("Bien joue le bot!");
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




//--  Motor -------------------------------------

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
    Serial.println("AccMotor: ");
    Serial.println(AccMotor , 16);

    tableauEncoder[0] = LiveTime;
    tableauEncoder[1] = LivePulse;
    tableauEncoder[2] = LiveVitesse;
    tableauEncoder[3] = AccMotor;
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