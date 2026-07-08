#include "table_solver.hpp"

#include <cmath>
#include <iostream>

#include "ballistics.hpp"
#include "interfaces/i_target_provider.hpp"

#ifndef HW7_BALLISTIC_TABLE_PATH
#define HW7_BALLISTIC_TABLE_PATH "data/ballistic_table.txt"
#endif

namespace {
constexpr double kDistanceEpsilon = 1e-9;
constexpr double kLeadConvergence = 1e-4;
constexpr int kLeadIterations = 10;

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

TableSolver::TableSolver() : TableSolver(HW7_BALLISTIC_TABLE_PATH) {}

TableSolver::TableSolver(std::string tablePath) : table_(), initialized_(table_.load(tablePath))
{
  if (!initialized_) {
    std::cerr << "Warning: failed to load ballistic table from " << tablePath
              << "; TableSolver will use analytical fallback" << std::endl;
  }
}

DropSolution TableSolver::solve(const Coord &dronePos,
                                const TargetSnapshot &target,
                                float altitude,
                                const AmmoParams &ammo,
                                float attackSpeed,
                                float accelerationPath) const
{
  DropSolution result{};
  result.ok = false;
  // Do not require table initialization: projectileMetrics can fall back analytically.
  if (attackSpeed <= 0.0f || accelerationPath < 0.0f || altitude <= 0.0f) {
    return result;
  }

  double flightTime = 0.0;
  double horizontalRange = 0.0;
  if (!projectileMetrics(altitude, ammo, attackSpeed, flightTime, horizontalRange)) {
    return result;
  }

  double droneX = static_cast<double>(dronePos.x);
  double droneY = static_cast<double>(dronePos.y);
  const double aimX = static_cast<double>(target.position.x);
  const double aimY = static_cast<double>(target.position.y);
  const double startDroneX = droneX;
  const double startDroneY = droneY;

  double distance = std::hypot(aimX - droneX, aimY - droneY);
  double maneuverX = 0.0;
  double maneuverY = 0.0;
  const bool maneuverApplied =
    applyManeuver(aimX, aimY, droneX, droneY, distance, maneuverX, maneuverY, horizontalRange, static_cast<double>(accelerationPath));
  if (distance < kDistanceEpsilon) {
    return result;
  }

  const double ratio = (distance - horizontalRange) / distance;
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

  result.travelTime = travelDistance / static_cast<double>(attackSpeed);
  result.impactTime = result.travelTime + flightTime;
  result.ok = true;
  return result;
}

bool TableSolver::projectileMetrics(float altitude,
                                    const AmmoParams &ammo,
                                    float dropSpeed,
                                    double &flightTime,
                                    double &horizontalRange) const
{
  if (altitude <= 0.0f || dropSpeed <= 0.0f) {
    return false;
  }

  if (initialized_) {
    // Always use table lookup (with edge clamp) so mission logic matches the web simulator.
    const BallisticTable::Result result = table_.lookup(altitude, dropSpeed, ammo.mass, ammo.drag, ammo.lift);
    if (result.t > 0.0 && result.hDist > 0.0) {
      flightTime = result.t;
      horizontalRange = result.hDist;
      return true;
    }
    return false;
  }

  // Table file missing: fall back to the analytic model.
  flightTime = computeTimeOfFlight(ammo.mass, ammo.drag, ammo.lift, dropSpeed, altitude);
  if (flightTime < 0.0) {
    return false;
  }
  horizontalRange = computeHorizontalDistance(ammo.mass, ammo.drag, ammo.lift, flightTime, dropSpeed);
  return horizontalRange > 0.0;
}

bool TableSolver::solveLead(const Coord &dronePos,
                            double currentTimeSeconds,
                            int targetIndex,
                            const ITargetProvider &targetProvider,
                            float altitude,
                            const AmmoParams &ammo,
                            float attackSpeed,
                            float accelerationPath,
                            DropSolution &result) const
{
  double flightTime = 0.0;
  double horizontalRange = 0.0;
  if (!projectileMetrics(altitude, ammo, attackSpeed, flightTime, horizontalRange)) {
    return false;
  }
  (void)horizontalRange;

  double travelTime = 0.0;
  for (int iter = 0; iter < kLeadIterations; iter++) {
    const double predictedTime = currentTimeSeconds + travelTime + flightTime;
    Coord aim{};
    if (!targetProvider.extrapolateTargetPosition(targetIndex, currentTimeSeconds, predictedTime, aim)) {
      return false;
    }

    DropSolution iterationResult = solve(dronePos, TargetSnapshot{aim, {0.0f, 0.0f}}, altitude, ammo, attackSpeed, accelerationPath);
    if (!iterationResult.ok) {
      return false;
    }

    const double newTravelTime = iterationResult.travelTime;
    if (std::fabs(newTravelTime - travelTime) < kLeadConvergence) {
      result = iterationResult;
      return true;
    }
    travelTime = newTravelTime;
  }

  const double predictedTime = currentTimeSeconds + travelTime + flightTime;
  Coord aim{};
  if (!targetProvider.extrapolateTargetPosition(targetIndex, currentTimeSeconds, predictedTime, aim)) {
    return false;
  }
  result = solve(dronePos, TargetSnapshot{aim, {0.0f, 0.0f}}, altitude, ammo, attackSpeed, accelerationPath);
  return result.ok;
}
