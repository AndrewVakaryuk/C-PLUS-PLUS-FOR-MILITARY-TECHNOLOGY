#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "interfaces/i_target_provider.hpp"
#include "simulation_clock.hpp"

class ThreadSafeTargetProvider : public ITargetProvider {
public:
  explicit ThreadSafeTargetProvider(const char *dataSourceDir);
  ~ThreadSafeTargetProvider() override;

  int getTargetCount() const override;
  bool getTarget(int index, TargetSnapshot &target) const override;
  bool interpolateTargetPosition(int targetIndex, double timeSeconds, Coord &position) const override;
  bool extrapolateTargetPosition(int targetIndex,
                                 double fromTimeSeconds,
                                 double toTimeSeconds,
                                 Coord &position) const override;

  bool isThreadReady() const;
  void start();
  void stop();
  void run();
  void advanceOnce();

private:
  ThreadSafeTargetProvider(const ThreadSafeTargetProvider &) = delete;
  ThreadSafeTargetProvider &operator=(const ThreadSafeTargetProvider &) = delete;

  static std::string buildPath(const std::string &dir, const std::string &fileName);
  bool load(const char *dataSourceDir);
  void clear();
  void updateSnapshots(double timeSeconds);
  bool velocityFromPastSamples(int targetIndex, double atTimeSeconds, Coord &velocity) const;
  bool interpolateUnlocked(int targetIndex, double timeSeconds, Coord &position) const;

  std::vector<std::vector<Coord>> targets_;
  std::vector<TargetSnapshot> snapshots_;
  int timeSteps_;
  float arrayTimeStep_;
  double timeScale_;
  SimulationClock clock_;
  bool loaded_;

  mutable std::mutex mutex_;

  std::atomic<bool> ready_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> stopRequested_{false};
};
