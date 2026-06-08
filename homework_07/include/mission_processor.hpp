#ifndef HOMEWORK_07_MISSION_PROCESSOR_HPP
#define HOMEWORK_07_MISSION_PROCESSOR_HPP

#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"

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

#endif
