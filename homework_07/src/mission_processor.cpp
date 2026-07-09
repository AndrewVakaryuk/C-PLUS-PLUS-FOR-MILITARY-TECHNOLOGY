#include "mission_processor.hpp"

#include <cmath>
#include <iostream>
#include <utility>

#include "drone_physics.hpp"
#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"
#include "drone_kinematics.hpp"
#include "simulation_pacing.hpp"

namespace {
constexpr int kMaxSimulationSteps = 10000;
constexpr std::size_t kMaxSimulationRecords = 10002;
constexpr double kEpsilon = 1e-9;
constexpr double kMinReleaseSpeedRatio = 0.5;

bool projectileMetricsAtSpeed(const IBallisticSolver &solver,
                              const AmmoParams &ammo,
                              float altitude,
                              float dropSpeed,
                              double &flightTime,
                              double &horizontalRange)
{
  return solver.projectileMetrics(altitude, ammo, dropSpeed, flightTime, horizontalRange);
}

bool isReadyToRelease(const DroneTelemetry &telemetry, double attackSpeed)
{
  if (!telemetry.canRelease) {
    return false;
  }
  return static_cast<double>(telemetry.speed) >= attackSpeed * kMinReleaseSpeedRatio - kEpsilon;
}

double displayDropSpeed(double speed, double attackSpeed)
{
  if (speed <= kEpsilon) {
    return attackSpeed;
  }
  return speed;
}

bool computeAimAndPrediction(const IBallisticSolver &solver,
                             const AmmoParams &ammo,
                             float dropSpeed,
                             float altitude,
                             const Coord &dronePos,
                             double direction,
                             double predictionTime,
                             int targetIndex,
                             const ITargetProvider &targetProvider,
                             Coord &aimPoint,
                             Coord &predictedTarget)
{
  double flightTime = 0.0;
  double range = 0.0;
  if (!projectileMetricsAtSpeed(solver, ammo, altitude, dropSpeed, flightTime, range)) {
    return false;
  }

  aimPoint = {static_cast<float>(dronePos.x + range * std::cos(direction)),
              static_cast<float>(dronePos.y + range * std::sin(direction))};
  return targetProvider.interpolateTargetPosition(targetIndex, predictionTime + flightTime, predictedTarget);
}

bool computeAimAndPredictionWithFallback(const IBallisticSolver &solver,
                                         const AmmoParams &ammo,
                                         float preferredDropSpeed,
                                         float fallbackDropSpeed,
                                         float altitude,
                                         const Coord &dronePos,
                                         double direction,
                                         double predictionTime,
                                         int targetIndex,
                                         const ITargetProvider &targetProvider,
                                         Coord &aimPoint,
                                         Coord &predictedTarget)
{
  if (computeAimAndPrediction(solver,
                              ammo,
                              preferredDropSpeed,
                              altitude,
                              dronePos,
                              direction,
                              predictionTime,
                              targetIndex,
                              targetProvider,
                              aimPoint,
                              predictedTarget)) {
    return true;
  }

  if (std::fabs(preferredDropSpeed - fallbackDropSpeed) <= kEpsilon) {
    return false;
  }

  return computeAimAndPrediction(solver,
                               ammo,
                               fallbackDropSpeed,
                               altitude,
                               dronePos,
                               direction,
                               predictionTime,
                               targetIndex,
                               targetProvider,
                               aimPoint,
                               predictedTarget);
}
}  // namespace

MissionProcessor::MissionProcessor(std::unique_ptr<IConfigLoader> configLoader,
                                   std::unique_ptr<ITargetProvider> targetProvider,
                                   std::unique_ptr<IBallisticSolver> solver)
  : configLoader_(std::move(configLoader))
  , targetProvider_(std::move(targetProvider))
  , solver_(std::move(solver))
  , physics_(nullptr)
  , initialized_(false)
  , targetCount_(0)
  , attackSpeed_(0.0)
  , simTimeStep_(0.0)
  , hitRadius_(0.0)
  , angularSpeed_(0.0)
  , currentTime_(0.0)
  , activeTarget_(-1)
  , tick_(0)
  , hit_(false)
  , missionSuccess_(false)
{}

void MissionProcessor::setPhysics(DronePhysics *physics)
{
  physics_ = physics;
}

bool MissionProcessor::init(const char *configSource)
{
  initialized_ = false;
  config_ = DroneConfig{};
  ammo_ = AmmoParams{};

  if (configLoader_ == nullptr || targetProvider_ == nullptr || solver_ == nullptr || physics_ == nullptr ||
      configSource == nullptr) {
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

  physics_->init(config_);
  initialized_ = true;
  return true;
}

bool MissionProcessor::validateSimulationParameters() const
{
  if (attackSpeed_ <= kEpsilon || config_.accelPath <= 0.0f || config_.simTimeStep <= 0.0f || config_.arrayTimeStep <= 0.0f ||
      config_.angularSpeed <= 0.0f || config_.physicsTimeStep <= 0.0f) {
    std::cerr << "Error: invalid non-positive simulation parameters" << std::endl;
    return false;
  }

  if (physicsStepsPerMission() < 1) {
    std::cerr << "Error: simTimeStep must be a positive integer multiple of physicsTimeStep" << std::endl;
    return false;
  }

  double flightTime = 0.0;
  double horizontalRange = 0.0;
  if (!solver_->projectileMetrics(config_.altitude, ammo_, config_.attackSpeed, flightTime, horizontalRange)) {
    std::cerr << "Error: invalid ballistic flight time at attack speed" << std::endl;
    return false;
  }

  return true;
}

void MissionProcessor::resetSimulationState()
{
  targetCount_ = targetProvider_->getTargetCount();
  attackSpeed_ = static_cast<double>(config_.attackSpeed);
  simTimeStep_ = static_cast<double>(config_.simTimeStep);
  hitRadius_ = static_cast<double>(config_.hitRadius);
  angularSpeed_ = static_cast<double>(config_.angularSpeed);
  currentTime_ = 0.0;
  activeTarget_ = -1;
  tick_ = 0;
  hit_ = false;
  missionSuccess_ = false;
  recordedSteps_.clear();
  physics_->init(config_);
}

int MissionProcessor::physicsStepsPerMission() const
{
  const double ratio = static_cast<double>(config_.simTimeStep) / static_cast<double>(config_.physicsTimeStep);
  const double rounded = std::round(ratio);
  if (rounded < 1.0 || std::fabs(ratio - rounded) > 1e-3) {
    return 0;
  }
  return static_cast<int>(rounded);
}

void MissionProcessor::advancePhysicsForMissionStep()
{
  const int steps = physicsStepsPerMission();
  for (int i = 0; i < steps; ++i) {
    physics_->stepOnce();
  }
}

void MissionProcessor::waitForPhysicsAdvance(double targetTimeSec)
{
  constexpr int kMaxWaitIterations = 100000;
  for (int i = 0; i < kMaxWaitIterations; ++i) {
    if (stopRequested_.load()) {
      return;
    }
    if (physics_->getTelemetry().timeSecSinceStart + 1e-4f >= static_cast<float>(targetTimeSec)) {
      return;
    }
    paceSimulation(static_cast<double>(config_.physicsTimeStep), static_cast<double>(config_.timeScale));
  }
}

bool MissionProcessor::selectBestTarget(int &bestIndex, Coord &bestFirePoint, const DroneTelemetry &telemetry) const
{
  const double linearStopTime = physics_->estimateStopTime();

  double bestScore = 1e300;
  bestIndex = -1;
  bestFirePoint = {0.0f, 0.0f};

  for (int i = 0; i < targetCount_; i++) {
    DropSolution leadSolution{};
    if (!solver_->solveLead(telemetry.pos,
                            currentTime_,
                            i,
                            *targetProvider_,
                            config_.altitude,
                            ammo_,
                            config_.attackSpeed,
                            config_.accelPath,
                            leadSolution)) {
      continue;
    }

    double penalty = 0.0;
    if (i != activeTarget_) {
      if (telemetry.isTurning) {
        const double aimDirection =
          std::atan2(leadSolution.dropPoint.y - telemetry.pos.y, leadSolution.dropPoint.x - telemetry.pos.x);
        penalty = std::fabs(angleDelta(static_cast<double>(telemetry.direction), aimDirection)) / angularSpeed_;
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

SimulationStepResult MissionProcessor::stepOnce(std::vector<SimStep> &steps)
{
  if (useExternalPhysicsThread_) {
    physics_->pause();
  }

  const DroneTelemetry telemetry = physics_->getTelemetry();
  currentTime_ = static_cast<double>(telemetry.timeSecSinceStart);
  const double direction = static_cast<double>(telemetry.direction);
  const double speed = static_cast<double>(telemetry.speed);

  int bestIndex = -1;
  Coord bestFirePoint{0.0f, 0.0f};
  if (!selectBestTarget(bestIndex, bestFirePoint, telemetry)) {
    std::cerr << "Error: no reachable target / ballistics failure" << std::endl;
    return SimulationStepResult::Failed;
  }

  const double desiredDirection = std::atan2(bestFirePoint.y - telemetry.pos.y, bestFirePoint.x - telemetry.pos.x);
  physics_->pushCommand(DroneCommand{static_cast<float>(desiredDirection)});

  const double recordDropSpeed = displayDropSpeed(speed, attackSpeed_);
  Coord aimPointNow = telemetry.pos;
  Coord predictedTargetNow = telemetry.pos;
  if (!computeAimAndPredictionWithFallback(*solver_,
                                           ammo_,
                                           static_cast<float>(recordDropSpeed),
                                           config_.attackSpeed,
                                           config_.altitude,
                                           telemetry.pos,
                                           direction,
                                           currentTime_,
                                           bestIndex,
                                           *targetProvider_,
                                           aimPointNow,
                                           predictedTargetNow)) {
    std::cerr << "Error: failed to predict target position" << std::endl;
    return SimulationStepResult::Failed;
  }

  Coord hitAimPointNow = aimPointNow;
  Coord hitPredictedTargetNow = predictedTargetNow;
  const bool canHitNow = isReadyToRelease(telemetry, attackSpeed_) &&
                         computeAimAndPrediction(*solver_,
                                                 ammo_,
                                                 static_cast<float>(speed),
                                                 config_.altitude,
                                                 telemetry.pos,
                                                 direction,
                                                 currentTime_,
                                                 bestIndex,
                                                 *targetProvider_,
                                                 hitAimPointNow,
                                                 hitPredictedTargetNow);

  if (steps.size() >= kMaxSimulationRecords) {
    std::cerr << "Error: simulation record buffer overflow" << std::endl;
    return SimulationStepResult::Failed;
  }

  SimStep record{};
  record.pos = telemetry.pos;
  record.direction = telemetry.direction;
  record.state = telemetry.stateCode;
  record.targetIdx = bestIndex;
  record.dropPoint = bestFirePoint;
  record.aimPoint = aimPointNow;
  record.predictedTarget = predictedTargetNow;
  record.timeSecSinceStart = telemetry.timeSecSinceStart;
  steps.push_back(record);

  const double missNow = std::hypot(hitAimPointNow.x - hitPredictedTargetNow.x, hitAimPointNow.y - hitPredictedTargetNow.y);
  if (canHitNow && missNow <= hitRadius_) {
    steps.back().aimPoint = hitAimPointNow;
    steps.back().predictedTarget = hitPredictedTargetNow;
    if (steps.size() == 1 && steps.size() < kMaxSimulationRecords) {
      steps.push_back(steps.back());
    }
    hit_ = true;
    return SimulationStepResult::Hit;
  }

  DroneTelemetry telemetryAfter{};
  const double targetPhysicsTime = currentTime_ + simTimeStep_;
  if (useExternalPhysicsThread_) {
    physics_->resume();
    waitForPhysicsAdvance(targetPhysicsTime);
    physics_->pause();
    telemetryAfter = physics_->getTelemetry();
  }
  else {
    advancePhysicsForMissionStep();
    telemetryAfter = physics_->getTelemetry();
  }

  const double directionAfter = static_cast<double>(telemetryAfter.direction);
  const double speedAfter = static_cast<double>(telemetryAfter.speed);
  const double timeAfterStep = currentTime_ + simTimeStep_;

  const double recordDropSpeedAfter = displayDropSpeed(speedAfter, attackSpeed_);
  Coord aimPointAfter = telemetryAfter.pos;
  Coord predictedTargetAfter = telemetryAfter.pos;
  if (!computeAimAndPredictionWithFallback(*solver_,
                                           ammo_,
                                           static_cast<float>(recordDropSpeedAfter),
                                           config_.attackSpeed,
                                           config_.altitude,
                                           telemetryAfter.pos,
                                           directionAfter,
                                           timeAfterStep,
                                           bestIndex,
                                           *targetProvider_,
                                           aimPointAfter,
                                           predictedTargetAfter)) {
    std::cerr << "Error: failed to predict target position after step" << std::endl;
    return SimulationStepResult::Failed;
  }

  Coord hitAimPointAfter = aimPointAfter;
  Coord hitPredictedTargetAfter = predictedTargetAfter;
  const bool canHitAfter = isReadyToRelease(telemetryAfter, attackSpeed_) &&
                           computeAimAndPrediction(*solver_,
                                                   ammo_,
                                                   static_cast<float>(speedAfter),
                                                   config_.altitude,
                                                   telemetryAfter.pos,
                                                   directionAfter,
                                                   timeAfterStep,
                                                   bestIndex,
                                                   *targetProvider_,
                                                   hitAimPointAfter,
                                                   hitPredictedTargetAfter);

  const double missAfter =
    std::hypot(hitAimPointAfter.x - hitPredictedTargetAfter.x, hitAimPointAfter.y - hitPredictedTargetAfter.y);
  if (canHitAfter && missAfter <= hitRadius_) {
    if (steps.size() >= kMaxSimulationRecords) {
      std::cerr << "Error: simulation record buffer overflow" << std::endl;
      return SimulationStepResult::Failed;
    }
    SimStep afterRecord = record;
    afterRecord.pos = telemetryAfter.pos;
    afterRecord.direction = telemetryAfter.direction;
    afterRecord.state = telemetryAfter.stateCode;
    afterRecord.aimPoint = hitAimPointAfter;
    afterRecord.predictedTarget = hitPredictedTargetAfter;
    afterRecord.timeSecSinceStart = telemetryAfter.timeSecSinceStart;
    steps.push_back(afterRecord);
    hit_ = true;
    return SimulationStepResult::Hit;
  }

  activeTarget_ = bestIndex;
  tick_++;
  return SimulationStepResult::Continue;
}

bool MissionProcessor::executeMission(std::vector<SimStep> &steps)
{
  steps.clear();
  if (!initialized_) {
    return false;
  }

  resetSimulationState();
  if (!validateSimulationParameters()) {
    return false;
  }

  if (useExternalPhysicsThread_) {
    physics_->pause();
  }

  while (!hit_ && tick_ < kMaxSimulationSteps && !stopRequested_.load()) {
    const SimulationStepResult result = stepOnce(steps);
    if (result == SimulationStepResult::Failed) {
      return false;
    }
    if (result == SimulationStepResult::Hit) {
      return true;
    }
  }

  if (!hit_) {
    std::cerr << "Error: hit radius not reached within MAX_STEPS" << std::endl;
    return false;
  }

  return true;
}

bool MissionProcessor::run(std::vector<SimStep> &steps)
{
  useExternalPhysicsThread_ = false;
  return executeMission(steps);
}

bool MissionProcessor::isThreadReady() const
{
  return ready_.load();
}

void MissionProcessor::start()
{
  running_.store(true);
}

void MissionProcessor::stop()
{
  stopRequested_.store(true);
}

void MissionProcessor::runThreaded()
{
  useExternalPhysicsThread_ = true;
  ready_.store(true);
  while (!running_.load()) {
    if (stopRequested_.load()) {
      return;
    }
    paceSimulation(0.001, static_cast<double>(config_.timeScale));
  }

  missionSuccess_ = executeMission(recordedSteps_);
}

const std::vector<SimStep> &MissionProcessor::steps() const
{
  return recordedSteps_;
}

bool MissionProcessor::succeeded() const
{
  return missionSuccess_;
}
