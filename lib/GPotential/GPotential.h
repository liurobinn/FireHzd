#ifndef Gpotential_h
#define Gpotential_h
#include "Arduino.h"

class GPotential {
  public:

  double altitude;
  double mass;
  double energy;

  double GetPotentialEnergy(double altitude, double mass);
};

#endif