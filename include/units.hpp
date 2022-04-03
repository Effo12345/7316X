#include "main.h"

#ifndef UNITS_HPP
#define UNITS_HPP

//Class to hold all unit conversion functions for ease of access
class Units{
private:
  const double trackingCircumference = 2.798 * PI; //Circumference of tracking wheel
  const double wheelConversionFactor = 360 / trackingCircumference;
public:
  double in_to_m(double in) {
    return in / 39.37;
  }

  double m_to_in(double m) {
    return m * 39.27;
  }

  //Used to convert between degrees of rotation and inches of movement
  double InToDeg(double in) {
    return in * wheelConversionFactor;
  }

  double DegToIn(double deg) {
    return deg / wheelConversionFactor;
  }

  //Convert between degrees and radians
  double RadToDeg(double rad) {
    return rad*57.29577951308232286;
  }

  double DegToRad(double deg) {
    return deg*0.01745329251994329547;
  }

  //Convert between degrees per second (dps) and revolutions per minute (rpm)
  double DPSToRPM(double dps) {
    return dps / 0.166667;
  }
};

static Units Units;

#endif //UNITS_HPP
