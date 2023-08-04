#include "bnoRead.h"

bnoRead::bnoRead()
{

    bno = Adafruit_BNO055(55);

    bno.begin(OPERATION_MODE_IMUPLUS);

    bno.setExtCrystalUse(true);
    


    delay(1000);

}

bnoRead::~bnoRead()
{
  
}

float bnoRead::getAngle()
{
  
  sensors_event_t event; 
  bno.getEvent(&event);

  AnglesPendule = bno.getVector(Adafruit_BNO055::VECTOR_EULER);


  if (AnglesPendule.x() >= 180)
  {
    correctAngle = 360 - AnglesPendule.x();
  }

  if (AnglesPendule.x() <= 180)
  {
    correctAngle = 0 - AnglesPendule.x();
  }
  return correctAngle;
  
}


void bnoRead::setOmega()
{
  
  sensors_event_t event; 
  bno.getEvent(&event);
  OmegaPendule = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  currentOmega = OmegaPendule.z();

}

float bnoRead::getOmega()
{
  return currentOmega;
}


void bnoRead::setAlpha()
{
    sensors_event_t event; 
    bno.getEvent(&event);
    liveTime = millis();

    float DeltaTime = liveTime - prevTime;
    float DeltaOmega = currentOmega - prevOmega;

    prevOmega = currentOmega;
    prevTime = liveTime;

    Accel = DeltaOmega/DeltaTime;

}

float bnoRead::getAlpha()
{

  return Accel;
}


