
#include "GPotential.h"
#include "Arduino.h"

double GPotential::GetPotentialEnergy(double altitude, double mass){
    energy = altitude * mass *9.81;
    return energy;
}