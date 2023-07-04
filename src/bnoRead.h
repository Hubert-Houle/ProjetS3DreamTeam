#ifndef BNOREAD_H
#define BNOREAD_H

#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>



class bnoRead
{
public:
    bnoRead();
    ~bnoRead();

    float getAlpha();
    float getOmega();
    imu::Vector<3> AnglesPendule;
    imu::Vector<3> OmegaPendule;

private:
    float liveTime = 0;
    float prevTime = 0;
    float currenttime = 0;
    float prevOmega

};



#endif