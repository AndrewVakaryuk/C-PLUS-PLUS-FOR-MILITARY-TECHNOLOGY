#include "drone_physics.hpp"

#include <cmath>
#include <iostream>

#include "simulation_pacing.hpp"

namespace {
constexpr double kStepRatioEpsilon = 1e-3;
}  // namespace

DronePhysics::DronePhysics()
  : clock_(0.01)
  , state_(createInitialDroneState())
{}

void DronePhysics::init(const DroneConfig &config)
{
  config_ = config;
  clock_ = SimulationClock(static_cast<double>(config.physicsTimeStep));
  timeScale_ = static_cast<double>(config.timeScale);
  turnThreshold_ = static_cast<double>(config.turnThreshold);
  attackSpeed_ = static_cast<double>(config.attackSpeed);
  accelerationPath_ = static_cast<double>(config.accelPath);
  angularSpeed_ = static_cast<double>(config.angularSpeed);
  pos_ = config.startPos;
  speed_ = 0.0;
  direction_ = static_cast<double>(config.initialDir);
  desiredDirection_ = config.initialDir;
  hasDesiredDirection_ = true;
  state_ = createInitialDroneState();
  commands_.clear();
  initialized_ = validateParameters();
  refreshTelemetry();
}

bool DronePhysics::validateParameters() const
{
  if (config_.physicsTimeStep <= 0.0f || config_.simTimeStep <= 0.0f || config_.attackSpeed <= 0.0f || config_.accelPath <= 0.0f ||
      config_.angularSpeed <= 0.0f) {
    std::cerr << "Error: invalid non-positive physics parameters" << std::endl;
    return false;
  }

  const double ratio = static_cast<double>(config_.simTimeStep) / static_cast<double>(config_.physicsTimeStep);
  const double rounded = std::round(ratio);
  if (rounded < 1.0 || std::fabs(ratio - rounded) > kStepRatioEpsilon) {
    std::cerr << "Error: simTimeStep must be a positive integer multiple of physicsTimeStep" << std::endl;
    return false;
  }

  return true;
}

void DronePhysics::pushCommand(DroneCommand command)
{
  commands_.push(command);
}

void DronePhysics::applyPendingCommand()
{
  DroneCommand command{};
  while (commands_.tryPop(command)) {
    desiredDirection_ = command.desiredDirection;
    hasDesiredDirection_ = true;
  }
}

void DronePhysics::refreshTelemetry()
{
  telemetry_.pos = pos_;
  telemetry_.direction = static_cast<float>(direction_);
  telemetry_.speed = static_cast<float>(speed_);
  telemetry_.stateCode = state_ != nullptr ? state_->code() : 0;
  telemetry_.canRelease = state_ != nullptr && state_->canRelease();
  telemetry_.isTurning = state_ != nullptr && state_->isTurning();
  telemetry_.timeSecSinceStart = static_cast<float>(clock_.now());
}

DroneTelemetry DronePhysics::getTelemetry() const
{
  std::lock_guard<std::mutex> lock(stateMutex_);
  return telemetry_;
}

double DronePhysics::estimateStopTime() const
{
  std::lock_guard<std::mutex> lock(stateMutex_);
  double speed = speed_;
  double direction = direction_;
  Coord pos = pos_;
  DroneContext ctx{clock_.stepDt(),
                   static_cast<double>(desiredDirection_),
                   turnThreshold_,
                   attackSpeed_,
                   accelerationPath_,
                   angularSpeed_,
                   speed,
                   direction,
                   pos};
  return computeTimeToStop(*state_, ctx);
}

bool DronePhysics::stepOnce()
{
  if (!initialized_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(stateMutex_);
  applyPendingCommand();
  const double heading = hasDesiredDirection_ ? static_cast<double>(desiredDirection_) : direction_;
  DroneContext ctx{clock_.stepDt(), heading, turnThreshold_, attackSpeed_, accelerationPath_, angularSpeed_, speed_, direction_, pos_};
  updateDrone(state_, ctx);
  clock_.advance();
  refreshTelemetry();
  return true;
}

bool DronePhysics::isThreadReady() const
{
  return ready_.load();
}

void DronePhysics::start()
{
  running_.store(true);
}

void DronePhysics::stop()
{
  stopRequested_.store(true);
}

void DronePhysics::pause()
{
  paused_.store(true);
}

void DronePhysics::resume()
{
  paused_.store(false);
}

void DronePhysics::run()
{
  ready_.store(true);
  while (!running_.load()) {
    if (stopRequested_.load()) {
      return;
    }
    paceSimulation(0.001, timeScale_);
  }

  while (!stopRequested_.load()) {
    if (paused_.load()) {
      paceSimulation(0.001, timeScale_);
      continue;
    }
    if (!stepOnce()) {
      break;
    }
    paceSimulation(clock_.stepDt(), timeScale_);
  }
}
