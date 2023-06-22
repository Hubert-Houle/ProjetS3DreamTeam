#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <Arduino.h>
  
  class Lineaire:public IMU9DOF
 {
  public:
  double GetAccel_X();
  double GetAccel_Y();
  double GetAccel_Z();
 };