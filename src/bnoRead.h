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

    float setAlpha();
    void setOmega();
    float getOmega();
    imu::Vector<3> AnglesPendule;
    imu::Vector<3> OmegaPendule;
    Adafruit_BNO055 bno;

private:
    float liveTime = 0;
    float prevTime = 0;
    float currenttime = 0;
    float currentOmega = 0;
    float prevOmega = 0;

};



#endif