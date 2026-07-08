#pragma once

#include <memory>

#include "domain_types.hpp"

class IBallisticSolver;
class IConfigLoader;
class ITargetProvider;

// Unit-test helper: one static drop per target from the configured start position.
// Production path uses MissionSimulator (time-stepped mission until hit).
class MissionProcessor {
public:
  MissionProcessor(std::unique_ptr<IConfigLoader> configLoader,
                   std::unique_ptr<ITargetProvider> targetProvider,
                   std::unique_ptr<IBallisticSolver> solver);

  bool init(const char *configSource);
  bool hasNext() const;
  bool step(DropSolution &result);
  void reset();
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);

private:
  std::unique_ptr<IConfigLoader> configLoader_;
  std::unique_ptr<ITargetProvider> targetProvider_;
  std::unique_ptr<IBallisticSolver> solver_;
  DroneConfig config_;
  AmmoParams ammo_;
  int currentTargetIndex_;
  int targetCount_;
  bool initialized_;
};

