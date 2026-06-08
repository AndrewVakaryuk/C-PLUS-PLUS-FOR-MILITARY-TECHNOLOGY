#include "../include/mission_processor.hpp"

#include <cstring>

MissionProcessor::MissionProcessor(IConfigLoader *configLoader, ITargetProvider *targetProvider, IBallisticSolver *solver)
  : configLoader_(configLoader)
  , targetProvider_(targetProvider)
  , solver_(solver)
  , currentTargetIndex_(0)
  , targetCount_(0)
  , initialized_(false)
{
  std::memset(&config_, 0, sizeof(config_));
  std::memset(&ammo_, 0, sizeof(ammo_));
}

bool MissionProcessor::init(const char *configSource)
{
  initialized_ = false;
  currentTargetIndex_ = 0;
  targetCount_ = 0;
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

  targetCount_ = targetProvider_->getTargetCount();
  if (targetCount_ <= 0) {
    return false;
  }

  initialized_ = true;
  return true;
}

bool MissionProcessor::hasNext() const
{
  return initialized_ && currentTargetIndex_ < targetCount_;
}

bool MissionProcessor::step(DropSolution &result)
{
  if (!hasNext()) {
    return false;
  }

  TargetSnapshot target{};
  if (!targetProvider_->getTarget(currentTargetIndex_, target)) {
    return false;
  }

  result = solver_->solve(config_.startPos, target, config_.altitude, ammo_, config_.attackSpeed, config_.accelPath);
  currentTargetIndex_++;
  return true;
}

void MissionProcessor::reset()
{
  currentTargetIndex_ = 0;
}

void MissionProcessor::changeSolver(IBallisticSolver *solver)
{
  if (solver != nullptr) {
    solver_ = solver;
  }
}
