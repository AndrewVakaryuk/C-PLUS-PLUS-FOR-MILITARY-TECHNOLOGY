#ifndef HOMEWORK_07_DRONE_KINEMATICS_HPP
#define HOMEWORK_07_DRONE_KINEMATICS_HPP

#include "domain_types.hpp"

double wrapAngle(double radians);

double angleDelta(double fromRadians, double toRadians);

double computeTimeToStop(DroneState state,
                         double speed,
                         double acceleration,
                         double angularSpeed,
                         double remainingTurnRadians);

void updateDrone(double simTimeStep,
                 double desiredDirection,
                 double turnThresholdRadians,
                 double attackSpeed,
                 double accelerationPath,
                 double angularSpeedRadiansPerSec,
                 DroneState &state,
                 double &speed,
                 double &direction,
                 Coord &position);

#endif
