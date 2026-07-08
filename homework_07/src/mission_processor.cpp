#include "mission_processor.hpp"

#include <utility>

#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"

MissionProcessor::MissionProcessor(std::unique_ptr<IConfigLoader> configLoader,
                                   std::unique_ptr<ITargetProvider> targetProvider,
                                   std::unique_ptr<IBallisticSolver> solver)
  : configLoader_(std::move(configLoader))
  , targetProvider_(std::move(targetProvider))
  , solver_(std::move(solver))
  , currentTargetIndex_(0)
  , targetCount_(0)
  , initialized_(false)
{}

bool MissionProcessor::init(const char *configSource)
{
  initialized_ = false;
  currentTargetIndex_ = 0;
  targetCount_ = 0;
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

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> solver)
{
  if (solver != nullptr) {
    solver_ = std::move(solver);
  }
}
