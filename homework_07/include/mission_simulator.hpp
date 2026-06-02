#pragma once

#include <vector>

#include "domain_types.hpp"
#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"

// Full mission loop: lead targeting, drone kinematics, hit detection, SimStep recording.
// Used by runMissionDemo / homework_07_baseline.
class MissionSimulator {
public:
  MissionSimulator(IConfigLoader *configLoader, ITargetProvider *targetProvider, IBallisticSolver *solver);

  bool init(const char *configSource);
  bool run(std::vector<SimStep> &steps);

private:
  IConfigLoader *configLoader_;
  ITargetProvider *targetProvider_;
  IBallisticSolver *solver_;
  DroneConfig config_;
  AmmoParams ammo_;
  bool initialized_;
};
