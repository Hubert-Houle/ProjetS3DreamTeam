#ifndef ENCODEUR_H
#define ENCODEUR_H

#include <Arduino.h>
#include <Wire.h>
#include <LibS3GRO.h>


class encodeur
{
    public:
        double LiveTime = 0;
        double PrevTime = 0;
        double PrevPosition = 0;
        double Position = 0;
        double Vitesse = 0;
        double PrevVitesse = 0;
        double Acceleration = 0;

        ArduinoX *LePlusBeauEncodeurDuMonde;

        double R = 0.036;
        double C = 3.1416*(2.00*R);
        double Kg = 25.0; 
        double ppt = 64.00;
        double dpp = C/(ppt*Kg);

        encodeur();
        ~encodeur();
        double getPosition();
        double getVitesse();
        double getAccel();



};







#endif