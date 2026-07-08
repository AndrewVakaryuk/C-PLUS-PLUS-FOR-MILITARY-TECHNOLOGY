#pragma once

#include <memory>
#include <vector>

#include "domain_types.hpp"

class IBallisticSolver;
class IConfigLoader;
class ITargetProvider;

enum class SimulationStepResult {
  Continue,
  Hit,
  Failed,
};

// Full mission loop: lead targeting, drone kinematics, hit detection, SimStep recording.
// Used by runMissionDemo / homework_07_baseline.
class MissionSimulator {
public:
  MissionSimulator(std::unique_ptr<IConfigLoader> configLoader,
                   std::unique_ptr<ITargetProvider> targetProvider,
                   std::unique_ptr<IBallisticSolver> solver);

  bool init(const char *configSource);
  bool hasNext() const;
  SimulationStepResult step(std::vector<SimStep> &steps);
  bool run(std::vector<SimStep> &steps);

private:
  bool validateSimulationParameters() const;
  void resetSimulationState();
  bool selectBestTarget(int &bestIndex, Coord &bestFirePoint) const;

  std::unique_ptr<IConfigLoader> configLoader_;
  std::unique_ptr<ITargetProvider> targetProvider_;
  std::unique_ptr<IBallisticSolver> solver_;
  DroneConfig config_;
  AmmoParams ammo_;
  bool initialized_;

  int targetCount_;
  double attackSpeed_;
  double accelerationPath_;
  double altitude_;
  double mass_;
  double drag_;
  double lift_;
  double simTimeStep_;
  double angularSpeed_;
  double turnThreshold_;
  double hitRadius_;
  double acceleration_;

  Coord dronePos_;
  DroneState droneState_;
  double speed_;
  double direction_;
  double currentTime_;
  int activeTarget_;
  int tick_;
  bool hit_;
};
