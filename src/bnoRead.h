#ifndef BNOREAD_H
#define BNOREAD_H

#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>


class bnoRead
{
public:
    bnoRead();
    ~bnoRead();

    void setAlpha();
    float getAlpha();
    void setOmega();
    float getOmega();
    float getAngle();
    imu::Vector<3> AnglesPendule;
    imu::Vector<3> OmegaPendule;
    Adafruit_BNO055 bno;
    //sensors_event_t event;

private:
    float liveTime = 0;
    float prevTime = 0;
    float currenttime = 0;
    float currentOmega = 0;
    float prevOmega = 0;
    float Accel = 0;
    float correctAngle = 0;

};



#endif