#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "domain_types.hpp"

class DronePhysics;
class IBallisticSolver;
class IConfigLoader;
class ITargetProvider;

enum class SimulationStepResult {
  Continue,
  Hit,
  Failed,
};

class MissionProcessor {
public:
  MissionProcessor(std::unique_ptr<IConfigLoader> configLoader,
                   std::unique_ptr<ITargetProvider> targetProvider,
                   std::unique_ptr<IBallisticSolver> solver);

  void setPhysics(DronePhysics *physics);

  bool init(const char *configSource);
  bool run(std::vector<SimStep> &steps);

  bool isThreadReady() const;
  void start();
  void stop();
  void runThreaded();
  const std::vector<SimStep> &steps() const;
  bool succeeded() const;

private:
  bool validateSimulationParameters() const;
  void resetSimulationState();
  bool selectBestTarget(int &bestIndex, Coord &bestFirePoint, const DroneTelemetry &telemetry) const;
  SimulationStepResult stepOnce(std::vector<SimStep> &steps);
  bool executeMission(std::vector<SimStep> &steps);
  int physicsStepsPerMission() const;
  void advancePhysicsForMissionStep();
  void waitForPhysicsAdvance(double targetTimeSec);

  std::unique_ptr<IConfigLoader> configLoader_;
  std::unique_ptr<ITargetProvider> targetProvider_;
  std::unique_ptr<IBallisticSolver> solver_;
  DronePhysics *physics_;
  DroneConfig config_;
  AmmoParams ammo_;
  bool initialized_;

  int targetCount_;
  double attackSpeed_;
  double simTimeStep_;
  double hitRadius_;
  double angularSpeed_;
  double currentTime_;
  int activeTarget_;
  int tick_;
  bool hit_;

  bool useExternalPhysicsThread_{false};
  std::vector<SimStep> recordedSteps_;
  bool missionSuccess_;

  std::atomic<bool> ready_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> stopRequested_{false};
};
