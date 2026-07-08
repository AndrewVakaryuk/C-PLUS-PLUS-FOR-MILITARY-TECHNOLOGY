#include "analytical_solver.hpp"

#include <cmath>

#include "ballistics.hpp"
#include "interfaces/i_target_provider.hpp"

namespace {
constexpr double kLeadConvergence = 1e-4;
constexpr int kLeadIterations = 10;
}  // namespace

DropSolution AnalyticalSolver::solve(const Coord &dronePos,
                                     const TargetSnapshot &target,
                                     float altitude,
                                     const AmmoParams &ammo,
                                     float attackSpeed,
                                     float accelerationPath) const
{
  DropSolution result{};
  computeDropToAim(dronePos, target.position, altitude, ammo, attackSpeed, accelerationPath, result);
  return result;
}

bool AnalyticalSolver::solveLead(const Coord &dronePos,
                                 double currentTimeSeconds,
                                 int targetIndex,
                                 const ITargetProvider &targetProvider,
                                 float altitude,
                                 const AmmoParams &ammo,
                                 float attackSpeed,
                                 float accelerationPath,
                                 DropSolution &result) const
{
  const double mass = static_cast<double>(ammo.mass);
  const double drag = static_cast<double>(ammo.drag);
  const double lift = static_cast<double>(ammo.lift);
  const double initialSpeed = static_cast<double>(attackSpeed);

  const double flightTime = computeTimeOfFlight(mass, drag, lift, initialSpeed, static_cast<double>(altitude));
  if (flightTime < 0.0) {
    return false;
  }

  double travelTime = 0.0;
  for (int iter = 0; iter < kLeadIterations; iter++) {
    const double predictedTime = currentTimeSeconds + travelTime + flightTime;
    Coord aim{};
    if (!targetProvider.extrapolateTargetPosition(targetIndex, currentTimeSeconds, predictedTime, aim)) {
      return false;
    }

    DropSolution iterationResult{};
    if (!computeDropToAim(dronePos, aim, altitude, ammo, attackSpeed, accelerationPath, iterationResult)) {
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
  return computeDropToAim(dronePos, aim, altitude, ammo, attackSpeed, accelerationPath, result);
}
