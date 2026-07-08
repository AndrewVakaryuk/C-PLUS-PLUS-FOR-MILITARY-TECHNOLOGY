#include "mission_simulator.hpp"

#include <cmath>
#include <iostream>
#include <utility>

#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"

namespace {
constexpr int kMaxSimulationSteps = 10000;
constexpr std::size_t kMaxSimulationRecords = 10002;
constexpr double kEpsilon = 1e-9;
// Drop only near attack speed; avoids low-speed "vertical" releases while still allowing
// high-speed accel/decel phases that the extreme scenario needs.
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

bool isReadyToRelease(const IDroneState &state, double speed, double attackSpeed)
{
  if (!state.canRelease()) {
    return false;
  }
  return speed >= attackSpeed * kMinReleaseSpeedRatio - kEpsilon;
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

MissionSimulator::MissionSimulator(std::unique_ptr<IConfigLoader> configLoader,
                                   std::unique_ptr<ITargetProvider> targetProvider,
                                   std::unique_ptr<IBallisticSolver> solver)
  : configLoader_(std::move(configLoader))
  , targetProvider_(std::move(targetProvider))
  , solver_(std::move(solver))
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
  , droneState_(createInitialDroneState())
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

  double flightTime = 0.0;
  double horizontalRange = 0.0;
  if (!solver_->projectileMetrics(config_.altitude, ammo_, config_.attackSpeed, flightTime, horizontalRange)) {
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
  droneState_ = createInitialDroneState();
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
  DroneContext stopCtx{simTimeStep_,
                       direction_,
                       turnThreshold_,
                       attackSpeed_,
                       accelerationPath_,
                       angularSpeed_,
                       const_cast<double &>(speed_),
                       const_cast<double &>(direction_),
                       const_cast<Coord &>(dronePos_)};
  const double linearStopTime = computeTimeToStop(*droneState_, stopCtx);

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
      if (droneState_->isTurning()) {
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

  const double recordDropSpeed = displayDropSpeed(speed_, attackSpeed_);
  Coord aimPointNow = dronePos_;
  Coord predictedTargetNow = dronePos_;
  if (!computeAimAndPredictionWithFallback(*solver_,
                                           ammo_,
                                           static_cast<float>(recordDropSpeed),
                                           config_.attackSpeed,
                                           config_.altitude,
                                           dronePos_,
                                           direction_,
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
  const bool canHitNow = isReadyToRelease(*droneState_, speed_, attackSpeed_) &&
                         computeAimAndPrediction(*solver_,
                                                 ammo_,
                                                 static_cast<float>(speed_),
                                                 config_.altitude,
                                                 dronePos_,
                                                 direction_,
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
  record.pos = dronePos_;
  record.direction = static_cast<float>(direction_);
  record.state = droneState_->code();
  record.targetIdx = bestIndex;
  record.dropPoint = bestFirePoint;
  record.aimPoint = aimPointNow;
  record.predictedTarget = predictedTargetNow;
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

  DroneContext droneCtx{simTimeStep_,
                        desiredDirection,
                        turnThreshold_,
                        attackSpeed_,
                        accelerationPath_,
                        angularSpeed_,
                        speed_,
                        direction_,
                        dronePos_};
  updateDrone(droneState_, droneCtx);

  const double timeAfterStep = currentTime_ + simTimeStep_;

  const double recordDropSpeedAfter = displayDropSpeed(speed_, attackSpeed_);
  Coord aimPointAfter = dronePos_;
  Coord predictedTargetAfter = dronePos_;
  if (!computeAimAndPredictionWithFallback(*solver_,
                                           ammo_,
                                           static_cast<float>(recordDropSpeedAfter),
                                           config_.attackSpeed,
                                           config_.altitude,
                                           dronePos_,
                                           direction_,
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
  const bool canHitAfter = isReadyToRelease(*droneState_, speed_, attackSpeed_) &&
                           computeAimAndPrediction(*solver_,
                                                   ammo_,
                                                   static_cast<float>(speed_),
                                                   config_.altitude,
                                                   dronePos_,
                                                   direction_,
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
    afterRecord.pos = dronePos_;
    afterRecord.direction = static_cast<float>(direction_);
    afterRecord.state = droneState_->code();
    afterRecord.aimPoint = hitAimPointAfter;
    afterRecord.predictedTarget = hitPredictedTargetAfter;
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
