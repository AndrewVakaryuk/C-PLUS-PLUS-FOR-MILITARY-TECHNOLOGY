#include "mission_simulator.hpp"

#include <cmath>
#include <iostream>

#include "ballistics.hpp"
#include "drone_kinematics.hpp"
#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"

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
  , targetCount_(0)
  , attackSpeed_(0.0)
  , accelerationPath_(0.0)
  , altitude_(0.0)
  , mass_(0.0)
  , drag_(0.0)
  , lift_(0.0)
  , simTimeStep_(0.0)
  , angularSpeed_(0.0)
  , turnThreshold_(0.0)
  , hitRadius_(0.0)
  , acceleration_(0.0)
  , droneState_(DRONE_STOPPED)
  , speed_(0.0)
  , direction_(0.0)
  , currentTime_(0.0)
  , activeTarget_(-1)
  , tick_(0)
  , hit_(false)
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

bool MissionSimulator::validateSimulationParameters() const
{
  if (attackSpeed_ <= kEpsilon || accelerationPath_ <= kEpsilon || config_.simTimeStep <= 0.0f || config_.arrayTimeStep <= 0.0f ||
      config_.angularSpeed <= 0.0f) {
    std::cerr << "Error: invalid non-positive simulation parameters" << std::endl;
    return false;
  }

  if (computeTimeOfFlight(mass_, drag_, lift_, attackSpeed_, altitude_) < 0.0) {
    std::cerr << "Error: invalid ballistic flight time at attack speed" << std::endl;
    return false;
  }

  return true;
}

void MissionSimulator::resetSimulationState()
{
  targetCount_ = targetProvider_->getTargetCount();
  attackSpeed_ = static_cast<double>(config_.attackSpeed);
  accelerationPath_ = static_cast<double>(config_.accelPath);
  altitude_ = static_cast<double>(config_.altitude);
  mass_ = static_cast<double>(ammo_.mass);
  drag_ = static_cast<double>(ammo_.drag);
  lift_ = static_cast<double>(ammo_.lift);
  simTimeStep_ = static_cast<double>(config_.simTimeStep);
  angularSpeed_ = static_cast<double>(config_.angularSpeed);
  turnThreshold_ = static_cast<double>(config_.turnThreshold);
  hitRadius_ = static_cast<double>(config_.hitRadius);
  acceleration_ = (attackSpeed_ * attackSpeed_) / (2.0 * accelerationPath_);

  dronePos_ = config_.startPos;
  droneState_ = DRONE_STOPPED;
  speed_ = 0.0;
  direction_ = static_cast<double>(config_.initialDir);
  currentTime_ = 0.0;
  activeTarget_ = -1;
  tick_ = 0;
  hit_ = false;
}

bool MissionSimulator::hasNext() const
{
  return initialized_ && !hit_ && tick_ < kMaxSimulationSteps;
}

bool MissionSimulator::selectBestTarget(int &bestIndex, Coord &bestFirePoint) const
{
  const double linearStopTime = computeTimeToStop(droneState_, speed_, acceleration_, angularSpeed_, 0.0);

  double bestScore = 1e300;
  bestIndex = -1;
  bestFirePoint = {0.0f, 0.0f};

  for (int i = 0; i < targetCount_; i++) {
    DropSolution leadSolution{};
    if (!solver_->solveLead(
          dronePos_, currentTime_, i, *targetProvider_, config_.altitude, ammo_, config_.attackSpeed, config_.accelPath, leadSolution)) {
      continue;
    }

    double penalty = 0.0;
    if (i != activeTarget_) {
      if (droneState_ == DRONE_TURNING) {
        const double aimDirection = std::atan2(leadSolution.dropPoint.y - dronePos_.y, leadSolution.dropPoint.x - dronePos_.x);
        penalty = std::fabs(angleDelta(direction_, aimDirection)) / angularSpeed_;
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

  return bestIndex >= 0;
}

SimulationStepResult MissionSimulator::step(std::vector<SimStep> &steps)
{
  int bestIndex = -1;
  Coord bestFirePoint{0.0f, 0.0f};
  if (!selectBestTarget(bestIndex, bestFirePoint)) {
    std::cerr << "Error: no reachable target / ballistics failure" << std::endl;
    return SimulationStepResult::Failed;
  }

  const double desiredDirection = std::atan2(bestFirePoint.y - dronePos_.y, bestFirePoint.x - dronePos_.x);

  double flightTimeNow = 0.0;
  double rangeNow = 0.0;
  const bool canAimNow = projectileMetricsAtSpeed(mass_, drag_, lift_, speed_, altitude_, flightTimeNow, rangeNow);

  Coord aimPointNow = dronePos_;
  if (canAimNow) {
    aimPointNow = {static_cast<float>(dronePos_.x + rangeNow * std::cos(direction_)),
                   static_cast<float>(dronePos_.y + rangeNow * std::sin(direction_))};
  }

  Coord predictedTargetNow = dronePos_;
  if (canAimNow && !targetProvider_->interpolateTargetPosition(bestIndex, currentTime_ + flightTimeNow, predictedTargetNow)) {
    std::cerr << "Error: failed to predict target position" << std::endl;
    return SimulationStepResult::Failed;
  }

  if (steps.size() >= kMaxSimulationRecords) {
    std::cerr << "Error: simulation record buffer overflow" << std::endl;
    return SimulationStepResult::Failed;
  }

  SimStep record{};
  record.pos = dronePos_;
  record.direction = static_cast<float>(direction_);
  record.state = static_cast<int>(droneState_);
  record.targetIdx = bestIndex;
  record.dropPoint = bestFirePoint;
  record.aimPoint = aimPointNow;
  record.predictedTarget = predictedTargetNow;
  steps.push_back(record);

  const double missNow = std::hypot(aimPointNow.x - predictedTargetNow.x, aimPointNow.y - predictedTargetNow.y);
  if (canAimNow && droneState_ == DRONE_MOVING && missNow <= hitRadius_) {
    if (steps.size() == 1 && steps.size() < kMaxSimulationRecords) {
      steps.push_back(record);
    }
    hit_ = true;
    return SimulationStepResult::Hit;
  }

  updateDrone(simTimeStep_,
              desiredDirection,
              turnThreshold_,
              attackSpeed_,
              accelerationPath_,
              angularSpeed_,
              droneState_,
              speed_,
              direction_,
              dronePos_);

  const double timeAfterStep = currentTime_ + simTimeStep_;

  double flightTimeAfter = 0.0;
  double rangeAfter = 0.0;
  const bool canAimAfter = projectileMetricsAtSpeed(mass_, drag_, lift_, speed_, altitude_, flightTimeAfter, rangeAfter);

  Coord aimPointAfter = dronePos_;
  if (canAimAfter) {
    aimPointAfter = {static_cast<float>(dronePos_.x + rangeAfter * std::cos(direction_)),
                     static_cast<float>(dronePos_.y + rangeAfter * std::sin(direction_))};
  }

  Coord predictedTargetAfter = dronePos_;
  if (canAimAfter &&
      !targetProvider_->interpolateTargetPosition(bestIndex, timeAfterStep + flightTimeAfter, predictedTargetAfter)) {
    std::cerr << "Error: failed to predict target position after step" << std::endl;
    return SimulationStepResult::Failed;
  }

  const double missAfter = std::hypot(aimPointAfter.x - predictedTargetAfter.x, aimPointAfter.y - predictedTargetAfter.y);
  if (canAimAfter && droneState_ == DRONE_MOVING && missAfter <= hitRadius_) {
    if (steps.size() >= kMaxSimulationRecords) {
      std::cerr << "Error: simulation record buffer overflow" << std::endl;
      return SimulationStepResult::Failed;
    }
    SimStep afterRecord = record;
    afterRecord.pos = dronePos_;
    afterRecord.direction = static_cast<float>(direction_);
    afterRecord.state = static_cast<int>(droneState_);
    afterRecord.aimPoint = aimPointAfter;
    afterRecord.predictedTarget = predictedTargetAfter;
    steps.push_back(afterRecord);
    hit_ = true;
    return SimulationStepResult::Hit;
  }

  activeTarget_ = bestIndex;
  currentTime_ += simTimeStep_;
  tick_++;
  return SimulationStepResult::Continue;
}

bool MissionSimulator::run(std::vector<SimStep> &steps)
{
  steps.clear();
  if (!initialized_) {
    return false;
  }

  resetSimulationState();
  if (!validateSimulationParameters()) {
    return false;
  }

  while (hasNext()) {
    const SimulationStepResult result = step(steps);
    if (result == SimulationStepResult::Failed) {
      return false;
    }
    if (result == SimulationStepResult::Hit) {
      return true;
    }
  }

  std::cerr << "Error: hit radius not reached within MAX_STEPS" << std::endl;
  return false;
}
