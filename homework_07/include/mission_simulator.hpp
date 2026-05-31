#ifndef HOMEWORK_07_MISSION_SIMULATOR_HPP
#define HOMEWORK_07_MISSION_SIMULATOR_HPP

#include "domain_types.hpp"
#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"

class MissionSimulator {
public:
  MissionSimulator(IConfigLoader *configLoader, ITargetProvider *targetProvider, IBallisticSolver *solver);

  bool init(const char *configSource);
  bool run(SimStep *steps, int maxSteps, int &recordCount);

private:
  IConfigLoader *configLoader_;
  ITargetProvider *targetProvider_;
  IBallisticSolver *solver_;
  DroneConfig config_;
  AmmoParams ammo_;
  bool initialized_;
};

#endif
