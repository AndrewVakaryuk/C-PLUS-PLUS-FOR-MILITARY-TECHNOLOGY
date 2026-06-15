#include "../include/mission_simulator.hpp"

#include <cmath>
#include <iostream>

#include "../include/ballistics.hpp"
#include "../include/drone_kinematics.hpp"

namespace {
constexpr int kMaxSimulationSteps = 10000;
constexpr std::size_t kMaxSimulationRecords = 10002;
constexpr double kEpsilon = 1e-9;

bool projectileMetricsAtSpeed(double mass,
                              double drag,
                              double lift,
                              double dropSpeed,
                              double altitude,
                              double &flightTime,
                              double &horizontalRange)
{
  if (dropSpeed <= kEpsilon) {
    return false;
  }

  flightTime = computeTimeOfFlight(mass, drag, lift, dropSpeed, altitude);
  if (flightTime < 0.0) {
    return false;
  }

  horizontalRange = computeHorizontalDistance(mass, drag, lift, flightTime, dropSpeed);
  return true;
}
}  // namespace

MissionSimulator::MissionSimulator(IConfigLoader *configLoader, ITargetProvider *targetProvider, IBallisticSolver *solver)
  : configLoader_(configLoader)
  , targetProvider_(targetProvider)
  , solver_(solver)
  , initialized_(false)
{}

bool MissionSimulator::init(const char *configSource)
{
  initialized_ = false;
  config_ = DroneConfig{};
  ammo_ = AmmoParams{};

  if (configLoader_ == nullptr || targetProvider_ == nullptr || solver_ == nullptr || configSource == nullptr) {
    return false;
  }
  if (!configLoader_->load(configSource)) {
    return false;
  }
  if (!configLoader_->getConfig(config_)) {
    return false;
  }
  if (!configLoader_->getAmmoParams(config_.ammoName, ammo_)) {
    return false;
  }
  if (targetProvider_->getTargetCount() <= 0) {
    return false;
  }

  initialized_ = true;
  return true;
}

bool MissionSimulator::run(std::vector<SimStep> &steps)
{
  steps.clear();
  if (!initialized_) {
    return false;
  }

  const int targetCount = targetProvider_->getTargetCount();
  const double attackSpeed = static_cast<double>(config_.attackSpeed);
  const double accelerationPath = static_cast<double>(config_.accelPath);
  const double altitude = static_cast<double>(config_.altitude);

  if (attackSpeed <= kEpsilon || accelerationPath <= kEpsilon || config_.simTimeStep <= 0.0f || config_.arrayTimeStep <= 0.0f ||
      config_.angularSpeed <= 0.0f) {
    std::cerr << "Error: invalid non-positive simulation parameters" << std::endl;
    return false;
  }

  const double mass = static_cast<double>(ammo_.mass);
  const double drag = static_cast<double>(ammo_.drag);
  const double lift = static_cast<double>(ammo_.lift);

  if (computeTimeOfFlight(mass, drag, lift, attackSpeed, altitude) < 0.0) {
    std::cerr << "Error: invalid ballistic flight time at attack speed" << std::endl;
    return false;
  }

  const double simTimeStep = static_cast<double>(config_.simTimeStep);
  const double angularSpeed = static_cast<double>(config_.angularSpeed);
  const double turnThreshold = static_cast<double>(config_.turnThreshold);
  const double hitRadius = static_cast<double>(config_.hitRadius);
  const double acceleration = (attackSpeed * attackSpeed) / (2.0 * accelerationPath);

  Coord dronePos = config_.startPos;
  DroneState state = DRONE_STOPPED;
  double speed = 0.0;
  double direction = static_cast<double>(config_.initialDir);
  double currentTime = 0.0;
  int activeTarget = -1;

  int tick = 0;
  bool hit = false;

  while (tick < kMaxSimulationSteps) {
    const double linearStopTime = computeTimeToStop(state, speed, acceleration, angularSpeed, 0.0);

    double bestScore = 1e300;
    int bestIndex = -1;
    Coord bestFirePoint{0.0f, 0.0f};

    for (int i = 0; i < targetCount; i++) {
      DropSolution leadSolution{};
      if (!solver_->solveLead(
            dronePos, currentTime, i, *targetProvider_, config_.altitude, ammo_, config_.attackSpeed, config_.accelPath, leadSolution)) {
        continue;
      }

      double penalty = 0.0;
      if (i != activeTarget) {
        if (state == DRONE_TURNING) {
          const double aimDirection = std::atan2(leadSolution.dropPoint.y - dronePos.y, leadSolution.dropPoint.x - dronePos.x);
          penalty = std::fabs(angleDelta(direction, aimDirection)) / angularSpeed;
        }
        else {
          penalty = linearStopTime;
        }
      }

      const double score = leadSolution.impactTime + penalty;
      if (score < bestScore) {
        bestScore = score;
        bestIndex = i;
        bestFirePoint = leadSolution.dropPoint;
      }
    }

    if (bestIndex < 0) {
      std::cerr << "Error: no reachable target / ballistics failure" << std::endl;
      return false;
    }

    const double desiredDirection = std::atan2(bestFirePoint.y - dronePos.y, bestFirePoint.x - dronePos.x);

    double flightTimeNow = 0.0;
    double rangeNow = 0.0;
    const bool canAimNow =
      projectileMetricsAtSpeed(mass, drag, lift, speed, altitude, flightTimeNow, rangeNow);

    Coord aimPointNow = dronePos;
    if (canAimNow) {
      aimPointNow = {static_cast<float>(dronePos.x + rangeNow * std::cos(direction)),
                     static_cast<float>(dronePos.y + rangeNow * std::sin(direction))};
    }

    Coord predictedTargetNow = dronePos;
    if (canAimNow &&
        !targetProvider_->interpolateTargetPosition(bestIndex, currentTime + flightTimeNow, predictedTargetNow)) {
      std::cerr << "Error: failed to predict target position" << std::endl;
      return false;
    }

    if (steps.size() >= kMaxSimulationRecords) {
      std::cerr << "Error: simulation record buffer overflow" << std::endl;
      return false;
    }

    SimStep record{};
    record.pos = dronePos;
    record.direction = static_cast<float>(direction);
    record.state = static_cast<int>(state);
    record.targetIdx = bestIndex;
    record.dropPoint = bestFirePoint;
    record.aimPoint = aimPointNow;
    record.predictedTarget = predictedTargetNow;
    steps.push_back(record);

    const double missNow = std::hypot(aimPointNow.x - predictedTargetNow.x, aimPointNow.y - predictedTargetNow.y);
    if (canAimNow && state == DRONE_MOVING && missNow <= hitRadius) {
      if (steps.size() == 1 && steps.size() < kMaxSimulationRecords) {
        steps.push_back(record);
      }
      hit = true;
      break;
    }

    updateDrone(
      simTimeStep, desiredDirection, turnThreshold, attackSpeed, accelerationPath, angularSpeed, state, speed, direction, dronePos);

    const double timeAfterStep = currentTime + simTimeStep;

    double flightTimeAfter = 0.0;
    double rangeAfter = 0.0;
    const bool canAimAfter =
      projectileMetricsAtSpeed(mass, drag, lift, speed, altitude, flightTimeAfter, rangeAfter);

    Coord aimPointAfter = dronePos;
    if (canAimAfter) {
      aimPointAfter = {static_cast<float>(dronePos.x + rangeAfter * std::cos(direction)),
                       static_cast<float>(dronePos.y + rangeAfter * std::sin(direction))};
    }

    Coord predictedTargetAfter = dronePos;
    if (canAimAfter &&
        !targetProvider_->interpolateTargetPosition(bestIndex, timeAfterStep + flightTimeAfter, predictedTargetAfter)) {
      std::cerr << "Error: failed to predict target position after step" << std::endl;
      return false;
    }

    const double missAfter = std::hypot(aimPointAfter.x - predictedTargetAfter.x, aimPointAfter.y - predictedTargetAfter.y);
    if (canAimAfter && state == DRONE_MOVING && missAfter <= hitRadius) {
      if (steps.size() >= kMaxSimulationRecords) {
        std::cerr << "Error: simulation record buffer overflow" << std::endl;
        return false;
      }
      SimStep afterRecord = record;
      afterRecord.pos = dronePos;
      afterRecord.direction = static_cast<float>(direction);
      afterRecord.state = static_cast<int>(state);
      afterRecord.aimPoint = aimPointAfter;
      afterRecord.predictedTarget = predictedTargetAfter;
      steps.push_back(afterRecord);
      hit = true;
      break;
    }

    activeTarget = bestIndex;
    currentTime += simTimeStep;
    tick++;
  }

  if (!hit) {
    std::cerr << "Error: hit radius not reached within MAX_STEPS" << std::endl;
    return false;
  }

  return true;
}
