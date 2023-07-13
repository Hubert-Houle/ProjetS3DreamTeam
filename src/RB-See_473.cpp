#include "RB-See-473.h"
double Lineaire::GetAccel_X()
{
  double Accel = 0;
  Accel = Lineaire::getAccelX()*9.80665;
  return Accel;
}

double Lineaire::GetAccel_Y()
{
  double Accel = 0;
  Accel = Lineaire::getAccelY()*9.80665;
  return Accel;
}

double Lineaire::GetAccel_Z()
{
  double Accel = 0;
  Accel = Lineaire::getAccelZ()*9.80665;
  return Accel;
}