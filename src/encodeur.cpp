#include "encodeur.h"


encodeur::encodeur()
{
    LePlusBeauEncodeurDuMonde = new ArduinoX;
    LePlusBeauEncodeurDuMonde->init();


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
    double millisecondes = millis();
    LiveTime = millisecondes/1000.0;    
    double DeltaTime = LiveTime - PrevTime;
    if(DeltaTime > 0.01)
    {
        double PositionTemp = PrevPosition;
        getPosition();

        double DeltaPosition = Position - PositionTemp;
 
        Vitesse = DeltaPosition / DeltaTime;

        PrevTime = LiveTime;
        PrevVitesse = Vitesse;
    }
    
    return Vitesse;
}



double encodeur::getAccel()
{
    double millisecondes = millis();
    LiveTime = millisecondes/1000.0;  
    double PrevTimeTemporaire = PrevTime;
    double DeltaTime = LiveTime - PrevTimeTemporaire;



   if(DeltaTime > 0.01)
    {        
        double VitesseTemp = PrevVitesse;
        getVitesse();

        double DeltaVitesse = Vitesse - VitesseTemp;

        Acceleration = DeltaVitesse / DeltaTime;
    }

    return Acceleration;
}
