#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "domain_types.hpp"
#include "drone_kinematics.hpp"
#include "simulation_clock.hpp"
#include "thread_safe_queue.hpp"

class DronePhysics {
public:
  DronePhysics();

  void init(const DroneConfig &config);
  void pushCommand(DroneCommand command);
  DroneTelemetry getTelemetry() const;
  double estimateStopTime() const;

  bool stepOnce();
  bool isThreadReady() const;
  void start();
  void stop();
  void pause();
  void resume();
  void run();

private:
  void applyPendingCommand();
  void refreshTelemetry();
  bool validateParameters() const;

  DroneConfig config_{};
  SimulationClock clock_{0.01};
  ThreadSafeQueue<DroneCommand> commands_;
  std::unique_ptr<IDroneState> state_;
  Coord pos_{};
  double speed_{0.0};
  double direction_{0.0};
  double turnThreshold_{0.0};
  double attackSpeed_{0.0};
  double accelerationPath_{0.0};
  double angularSpeed_{0.0};
  double timeScale_{10.0};
  float desiredDirection_{0.0f};
  bool hasDesiredDirection_{false};
  bool initialized_{false};

  mutable std::mutex stateMutex_;
  DroneTelemetry telemetry_{};

  std::atomic<bool> ready_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> stopRequested_{false};
  std::atomic<bool> paused_{true};
};
