#include "bnoRead.h"

bnoRead::bnoRead()
{

    bno = Adafruit_BNO055(55);
    bno.setExtCrystalUse(true);

}

bnoRead::~bnoRead()
{

}

void bnoRead::setOmega()
{

  OmegaPendule = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  currentOmega = OmegaPendule.x();

}

float bnoRead::setAlpha()
{
    float Accel = 0;
    liveTime = millis();

    float DeltaTime = liveTime - prevTime;
    float DeltaOmega = currentOmega - prevOmega;

    prevOmega = currentOmega;

    float Accel = DeltaOmega/DeltaTime;


    return Accel;
}


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
