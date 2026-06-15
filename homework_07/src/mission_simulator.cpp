#include "../include/mission_simulator.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

#include "../include/ballistics.hpp"
#include "../include/drone_kinematics.hpp"

namespace {
constexpr int kMaxSimulationSteps = 10000;
constexpr double kEpsilon = 1e-9;
}  // namespace

MissionSimulator::MissionSimulator(IConfigLoader *configLoader, ITargetProvider *targetProvider, IBallisticSolver *solver)
  : configLoader_(configLoader)
  , targetProvider_(targetProvider)
  , solver_(solver)
  , initialized_(false)
{
  std::memset(&config_, 0, sizeof(config_));
  std::memset(&ammo_, 0, sizeof(ammo_));
}

bool MissionSimulator::init(const char *configSource)
{
  initialized_ = false;
  std::memset(&config_, 0, sizeof(config_));
  std::memset(&ammo_, 0, sizeof(ammo_));

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

bool MissionSimulator::run(SimStep *steps, int maxSteps, int &recordCount)
{
  recordCount = 0;
  if (!initialized_ || steps == nullptr || maxSteps <= 0) {
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

  const double projectileFlightTime = computeTimeOfFlight(mass, drag, lift, attackSpeed, altitude);
  if (projectileFlightTime < 0.0) {
    std::cerr << "Error: invalid ballistic flight time" << std::endl;
    return false;
  }
  const double projectileRange = computeHorizontalDistance(mass, drag, lift, projectileFlightTime, attackSpeed);

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
    const Coord aimPointNow{static_cast<float>(dronePos.x + projectileRange * std::cos(direction)),
                            static_cast<float>(dronePos.y + projectileRange * std::sin(direction))};
    Coord predictedTargetNow{};
    if (!targetProvider_->interpolateTargetPosition(bestIndex, currentTime + projectileFlightTime, predictedTargetNow)) {
      std::cerr << "Error: failed to predict target position" << std::endl;
      return false;
    }

    if (recordCount >= maxSteps) {
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
    steps[recordCount++] = record;

    const double missNow = std::hypot(aimPointNow.x - predictedTargetNow.x, aimPointNow.y - predictedTargetNow.y);
    if (missNow <= hitRadius) {
      if (recordCount == 1 && recordCount < maxSteps) {
        steps[recordCount++] = record;
      }
      hit = true;
      break;
    }

    updateDrone(
      simTimeStep, desiredDirection, turnThreshold, attackSpeed, accelerationPath, angularSpeed, state, speed, direction, dronePos);

    const double timeAfterStep = currentTime + simTimeStep;
    const Coord aimPointAfter{static_cast<float>(dronePos.x + projectileRange * std::cos(direction)),
                              static_cast<float>(dronePos.y + projectileRange * std::sin(direction))};
    Coord predictedTargetAfter{};
    if (!targetProvider_->interpolateTargetPosition(bestIndex, timeAfterStep + projectileFlightTime, predictedTargetAfter)) {
      std::cerr << "Error: failed to predict target position after step" << std::endl;
      return false;
    }

    const double missAfter = std::hypot(aimPointAfter.x - predictedTargetAfter.x, aimPointAfter.y - predictedTargetAfter.y);
    if (missAfter <= hitRadius) {
      if (recordCount >= maxSteps) {
        std::cerr << "Error: simulation record buffer overflow" << std::endl;
        return false;
      }
      SimStep afterRecord = record;
      afterRecord.pos = dronePos;
      afterRecord.direction = static_cast<float>(direction);
      afterRecord.state = static_cast<int>(state);
      afterRecord.aimPoint = aimPointAfter;
      afterRecord.predictedTarget = predictedTargetAfter;
      steps[recordCount++] = afterRecord;
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
