#include "../include/analytical_solver.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
constexpr double kGravity = 9.81;
constexpr double kDistanceEpsilon = 1e-9;
constexpr double kModelEpsilon = 1e-12;

double computeTimeOfFlight(double m, double d, double l, double v0, double z0)
{
  const double a = d * kGravity * m - 2.0 * d * d * l * v0;
  const double b = -3.0 * kGravity * m * m + 3.0 * d * l * m * v0;
  const double c = 6.0 * m * m * z0;

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

double computeHorizontalDistance(double m, double d, double l, double t, double v0)
{
  if (m < kModelEpsilon) {
    return 0.0;
  }

  const double liftSq = l * l;
  const double liftFourth = liftSq * liftSq;
  const double onePlusLiftSq = 1.0 + liftSq;

  const double term1 = v0 * t;
  const double term2 = -t * t * d * v0 / (2.0 * m);
  const double term3 = t * t * t * (6.0 * d * kGravity * l * m - 6.0 * d * d * (liftSq - 1.0) * v0) / (36.0 * m * m);
  const double term4Den = 36.0 * onePlusLiftSq * onePlusLiftSq * m * m * m;
  const double term4 = t * t * t * t *
                       (-6.0 * d * d * kGravity * l * (1.0 + liftSq + liftFourth) * m + 3.0 * d * d * d * liftSq * onePlusLiftSq * v0 +
                        6.0 * d * d * d * liftFourth * onePlusLiftSq * v0) /
                       term4Den;
  const double term5Den = 36.0 * onePlusLiftSq * m * m * m * m;
  const double term5 =
    t * t * t * t * t * (3.0 * d * d * d * kGravity * l * l * l * m - 3.0 * d * d * d * d * liftSq * onePlusLiftSq * v0) / term5Den;

  return term1 + term2 + term3 + term4 + term5;
}

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

DropSolution AnalyticalSolver::solve(const Coord &dronePos,
                                     const TargetSnapshot &target,
                                     float altitude,
                                     const AmmoParams &ammo,
                                     float attackSpeed,
                                     float accelerationPath) const
{
  DropSolution result{};
  result.ok = false;

  if (attackSpeed <= 0.0f || accelerationPath < 0.0f || altitude <= 0.0f) {
    return result;
  }

  const double m = static_cast<double>(ammo.mass);
  const double d = static_cast<double>(ammo.drag);
  const double l = static_cast<double>(ammo.lift);
  const double v0 = static_cast<double>(attackSpeed);
  const double z0 = static_cast<double>(altitude);

  const double flightTime = computeTimeOfFlight(m, d, l, v0, z0);
  if (flightTime < 0.0) {
    return result;
  }

  const double horizontalReach = computeHorizontalDistance(m, d, l, flightTime, v0);

  double droneX = static_cast<double>(dronePos.x);
  double droneY = static_cast<double>(dronePos.y);
  const double targetX = static_cast<double>(target.position.x);
  const double targetY = static_cast<double>(target.position.y);

  double distance = std::hypot(targetX - droneX, targetY - droneY);
  double maneuverX = 0.0;
  double maneuverY = 0.0;
  const double startDroneX = droneX;
  const double startDroneY = droneY;

  const bool maneuverApplied =
    applyManeuver(targetX, targetY, droneX, droneY, distance, maneuverX, maneuverY, horizontalReach, static_cast<double>(accelerationPath));

  if (distance < kDistanceEpsilon) {
    return result;
  }

  const double ratio = (distance - horizontalReach) / distance;
  result.dropPoint.x = static_cast<float>(droneX + (targetX - droneX) * ratio);
  result.dropPoint.y = static_cast<float>(droneY + (targetY - droneY) * ratio);

  double travelDistance = 0.0;
  if (maneuverApplied) {
    travelDistance = std::hypot(droneX - startDroneX, droneY - startDroneY) +
                     std::hypot(static_cast<double>(result.dropPoint.x) - droneX, static_cast<double>(result.dropPoint.y) - droneY);
  }
  else {
    travelDistance =
      std::hypot(static_cast<double>(result.dropPoint.x) - startDroneX, static_cast<double>(result.dropPoint.y) - startDroneY);
  }

  result.travelTime = travelDistance / v0;
  result.impactTime = result.travelTime + flightTime;
  result.ok = true;
  return result;
}
