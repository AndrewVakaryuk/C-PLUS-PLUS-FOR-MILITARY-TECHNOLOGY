#ifndef HOMEWORK_07_BALLISTICS_HPP
#define HOMEWORK_07_BALLISTICS_HPP

#include "domain_types.hpp"

double computeTimeOfFlight(double mass, double drag, double lift, double initialSpeed, double altitude);

double computeHorizontalDistance(double mass, double drag, double lift, double flightTime, double initialSpeed);

bool computeDropToAim(const Coord &dronePos,
                      const Coord &aimPoint,
                      float altitude,
                      const AmmoParams &ammo,
                      float attackSpeed,
                      float accelerationPath,
                      DropSolution &result);

#endif
