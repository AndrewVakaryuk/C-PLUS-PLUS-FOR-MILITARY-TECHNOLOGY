#pragma once

#include "domain_types.hpp"

class IBallisticSolver;
class IConfigLoader;
class ITargetProvider;

// Unit-test helper: one static drop per target from the configured start position.
// Production path uses MissionSimulator (time-stepped mission until hit).
class MissionProcessor {
public:
  MissionProcessor(IConfigLoader *configLoader, ITargetProvider *targetProvider, IBallisticSolver *solver);

  bool init(const char *configSource);
  bool hasNext() const;
  bool step(DropSolution &result);
  void reset();
  void changeSolver(IBallisticSolver *solver);

private:
  IConfigLoader *configLoader_;
  ITargetProvider *targetProvider_;
  IBallisticSolver *solver_;
  DroneConfig config_;
  AmmoParams ammo_;
  int currentTargetIndex_;
  int targetCount_;
  bool initialized_;
};

