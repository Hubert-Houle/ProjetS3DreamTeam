#include "encodeur.h"


encodeur::encodeur()
{
    LePlusBeauEncodeurDuMonde = new ArduinoX;


    LePlusBeauEncodeurDuMonde->setMotorPWM(0, 0);
    LePlusBeauEncodeurDuMonde->setMotorPWM(1, 0);

    LePlusBeauEncodeurDuMonde->setMotorPWM(0,0);
    LePlusBeauEncodeurDuMonde->setMotorPWM(1,0);


}

encodeur::~encodeur()
{

}



double encodeur::getPosition()
{

    Position = LePlusBeauEncodeurDuMonde->readEncoder(0)*dpp;
    //Position = AX_.readEncoder(0)*dpp;
    PrevPosition = Position;

    return Position;
}



double encodeur::getVitesse()
{

    double PositionTemp = PrevPosition;
    getPosition();

    double millisecondes = millis();
    LiveTime = millisecondes/1000.0;

    double DeltaPosition = Position - PositionTemp;
    double DeltaTime = LiveTime - PrevTime;

    Vitesse = DeltaPosition / DeltaTime;

    PrevTime = LiveTime;
    PrevVitesse = Vitesse;

    return Vitesse;
}



double encodeur::getAccel()
{

    double VitesseTemp = PrevVitesse;
    double PrevTimeTemporaire = PrevTime;
    getVitesse();

    double DeltaVitesse = Vitesse - VitesseTemp;
    double DeltaTime = LiveTime - PrevTimeTemporaire;

    Acceleration = DeltaVitesse / DeltaTime;

    return Acceleration;
}
