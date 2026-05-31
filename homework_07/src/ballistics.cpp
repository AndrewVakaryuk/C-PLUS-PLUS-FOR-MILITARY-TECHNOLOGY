#include "../include/ballistics.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
constexpr double kGravity = 9.81;
constexpr double kDistanceEpsilon = 1e-9;
constexpr double kModelEpsilon = 1e-12;

bool applyManeuver(double targetX,
                   double targetY,
                   double &droneX,
                   double &droneY,
                   double &distance,
                   double &maneuverX,
                   double &maneuverY,
                   double horizontalReach,
                   double accelerationPath)
{
  if (distance < kDistanceEpsilon) {
    maneuverX = targetX - (horizontalReach + accelerationPath);
    maneuverY = targetY;
  }
  else if (horizontalReach + accelerationPath > distance) {
    maneuverX = targetX - (targetX - droneX) * (horizontalReach + accelerationPath) / distance;
    maneuverY = targetY - (targetY - droneY) * (horizontalReach + accelerationPath) / distance;
  }
  else {
    return false;
  }

  droneX = maneuverX;
  droneY = maneuverY;
  distance = horizontalReach + accelerationPath;
  return true;
}
}  // namespace

double computeTimeOfFlight(double mass, double drag, double lift, double initialSpeed, double altitude)
{
  const double a = drag * kGravity * mass - 2.0 * drag * drag * lift * initialSpeed;
  const double b = -3.0 * kGravity * mass * mass + 3.0 * drag * lift * mass * initialSpeed;
  const double c = 6.0 * mass * mass * altitude;

  if (std::fabs(a) < kModelEpsilon) {
    return -1.0;
  }

  const double p = -(b * b) / (3.0 * a * a);
  const double q = (2.0 * b * b * b) / (27.0 * a * a * a) + c / a;

  if (p >= -kModelEpsilon || std::fabs(p) < kModelEpsilon) {
    return -1.0;
  }

  const double arccosArg = (3.0 * q / (2.0 * p)) * std::sqrt(-3.0 / p);
  if (arccosArg < -1.0 || arccosArg > 1.0) {
    return -1.0;
  }

  const double phi = std::acos(arccosArg);
  const double t = 2.0 * std::sqrt(-p / 3.0) * std::cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a);
  if (t <= 0.0) {
    return -1.0;
  }
  return t;
}

double computeHorizontalDistance(double mass, double drag, double lift, double flightTime, double initialSpeed)
{
  if (mass < kModelEpsilon) {
    return 0.0;
  }

  const double liftSq = lift * lift;
  const double liftFourth = liftSq * liftSq;
  const double onePlusLiftSq = 1.0 + liftSq;

  const double term1 = initialSpeed * flightTime;
  const double term2 = -flightTime * flightTime * drag * initialSpeed / (2.0 * mass);
  const double term3 = flightTime * flightTime * flightTime *
                       (6.0 * drag * kGravity * lift * mass - 6.0 * drag * drag * (liftSq - 1.0) * initialSpeed) / (36.0 * mass * mass);
  const double term4Den = 36.0 * onePlusLiftSq * onePlusLiftSq * mass * mass * mass;
  const double term4 = flightTime * flightTime * flightTime * flightTime *
                       (-6.0 * drag * drag * kGravity * lift * (1.0 + liftSq + liftFourth) * mass +
                        3.0 * drag * drag * drag * liftSq * onePlusLiftSq * initialSpeed +
                        6.0 * drag * drag * drag * liftFourth * onePlusLiftSq * initialSpeed) /
                       term4Den;
  const double term5Den = 36.0 * onePlusLiftSq * mass * mass * mass * mass;
  const double term5 = flightTime * flightTime * flightTime * flightTime * flightTime *
                       (3.0 * drag * drag * drag * kGravity * lift * lift * lift * mass -
                        3.0 * drag * drag * drag * drag * liftSq * onePlusLiftSq * initialSpeed) /
                       term5Den;

  return term1 + term2 + term3 + term4 + term5;
}

bool computeDropToAim(const Coord &dronePos,
                      const Coord &aimPoint,
                      float altitude,
                      const AmmoParams &ammo,
                      float attackSpeed,
                      float accelerationPath,
                      DropSolution &result)
{
  result = {};
  result.ok = false;

  if (attackSpeed <= 0.0f || accelerationPath < 0.0f || altitude <= 0.0f) {
    return false;
  }

  const double mass = static_cast<double>(ammo.mass);
  const double drag = static_cast<double>(ammo.drag);
  const double lift = static_cast<double>(ammo.lift);
  const double initialSpeed = static_cast<double>(attackSpeed);

  const double flightTime = computeTimeOfFlight(mass, drag, lift, initialSpeed, static_cast<double>(altitude));
  if (flightTime < 0.0) {
    return false;
  }

  const double horizontalReach = computeHorizontalDistance(mass, drag, lift, flightTime, initialSpeed);

  double droneX = static_cast<double>(dronePos.x);
  double droneY = static_cast<double>(dronePos.y);
  const double aimX = static_cast<double>(aimPoint.x);
  const double aimY = static_cast<double>(aimPoint.y);
  const double startDroneX = droneX;
  const double startDroneY = droneY;

  double distance = std::hypot(aimX - droneX, aimY - droneY);
  double maneuverX = 0.0;
  double maneuverY = 0.0;

  const bool maneuverApplied =
    applyManeuver(aimX, aimY, droneX, droneY, distance, maneuverX, maneuverY, horizontalReach, static_cast<double>(accelerationPath));

  if (distance < kDistanceEpsilon) {
    return false;
  }

  const double ratio = (distance - horizontalReach) / distance;
  result.dropPoint.x = static_cast<float>(droneX + (aimX - droneX) * ratio);
  result.dropPoint.y = static_cast<float>(droneY + (aimY - droneY) * ratio);

  double travelDistance = 0.0;
  if (maneuverApplied) {
    travelDistance = std::hypot(droneX - startDroneX, droneY - startDroneY) +
                     std::hypot(static_cast<double>(result.dropPoint.x) - droneX, static_cast<double>(result.dropPoint.y) - droneY);
  }
  else {
    travelDistance =
      std::hypot(static_cast<double>(result.dropPoint.x) - startDroneX, static_cast<double>(result.dropPoint.y) - startDroneY);
  }

  result.travelTime = travelDistance / initialSpeed;
  result.impactTime = result.travelTime + flightTime;
  result.ok = true;
  return true;
}
