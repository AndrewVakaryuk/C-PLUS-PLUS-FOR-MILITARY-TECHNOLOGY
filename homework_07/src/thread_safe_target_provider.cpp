#include "thread_safe_target_provider.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "json.hpp"
#include "simulation_pacing.hpp"

namespace {
using json = nlohmann::json;
}  // namespace

ThreadSafeTargetProvider::ThreadSafeTargetProvider(const char *dataSourceDir)
  : timeSteps_(0)
  , arrayTimeStep_(1.0f)
  , timeScale_(10.0)
  , clock_(10.0)
  , loaded_(false)
{
  load(dataSourceDir);
}

ThreadSafeTargetProvider::~ThreadSafeTargetProvider()
{
  stopRequested_.store(true);
}

void ThreadSafeTargetProvider::clear()
{
  targets_.clear();
  snapshots_.clear();
  timeSteps_ = 0;
  arrayTimeStep_ = 1.0f;
  loaded_ = false;
}

std::string ThreadSafeTargetProvider::buildPath(const std::string &dir, const std::string &fileName)
{
  return (std::filesystem::path(dir) / fileName).string();
}

bool ThreadSafeTargetProvider::load(const char *dataSourceDir)
{
  clear();

  if (dataSourceDir == nullptr) {
    std::cerr << "Error: data source path invalid" << std::endl;
    return false;
  }

  const std::string sourceDir = dataSourceDir;
  const std::string targetsPath = buildPath(sourceDir, "targets.json");
  const std::string configPath = buildPath(sourceDir, "config.json");

  std::ifstream targetsFile(targetsPath);
  if (!targetsFile.is_open()) {
    std::cerr << "Error: could not open " << targetsPath << std::endl;
    return false;
  }

  std::ifstream configFile(configPath);
  if (!configFile.is_open()) {
    std::cerr << "Error: could not open " << configPath << std::endl;
    return false;
  }

  json targetsJson;
  json configJson;
  try {
    targetsFile >> targetsJson;
    configFile >> configJson;

    const int targetCount = targetsJson.at("targetCount").get<int>();
    timeSteps_ = targetsJson.at("timeSteps").get<int>();
    arrayTimeStep_ = configJson.at("targetArrayTimeStep").get<float>();
    timeScale_ = configJson["simulation"].value("timeScale", 10.0f);

    if (targetCount <= 0 || timeSteps_ <= 1 || arrayTimeStep_ <= 0.0f) {
      std::cerr << "Error: invalid target provider config values" << std::endl;
      clear();
      return false;
    }

    targets_.resize(static_cast<std::size_t>(targetCount));
    for (int i = 0; i < targetCount; i++) {
      std::vector<Coord> &trajectory = targets_[static_cast<std::size_t>(i)];
      trajectory.resize(static_cast<std::size_t>(timeSteps_));
      for (int t = 0; t < timeSteps_; t++) {
        trajectory[static_cast<std::size_t>(t)].x =
          targetsJson.at("targets").at(i).at("positions").at(t).at("x").get<float>();
        trajectory[static_cast<std::size_t>(t)].y =
          targetsJson.at("targets").at(i).at("positions").at(t).at("y").get<float>();
      }
    }

    clock_ = SimulationClock(static_cast<double>(arrayTimeStep_));
    snapshots_.resize(static_cast<std::size_t>(targetCount));
    updateSnapshots(0.0);
  }
  catch (const std::exception &e) {
    std::cerr << "Error: invalid target data: " << e.what() << std::endl;
    clear();
    return false;
  }

  loaded_ = true;
  return true;
}

void ThreadSafeTargetProvider::updateSnapshots(double timeSeconds)
{
  for (int i = 0; i < static_cast<int>(snapshots_.size()); ++i) {
    Coord position{};
    Coord velocity{};
    if (!interpolateUnlocked(i, timeSeconds, position)) {
      continue;
    }
    if (!velocityFromPastSamples(i, timeSeconds, velocity)) {
      velocity = {0.0f, 0.0f};
    }
    snapshots_[static_cast<std::size_t>(i)].position = position;
    snapshots_[static_cast<std::size_t>(i)].velocity = velocity;
  }
}

void ThreadSafeTargetProvider::advanceOnce()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!loaded_) {
    return;
  }
  clock_.advance();
  updateSnapshots(clock_.now());
}

int ThreadSafeTargetProvider::getTargetCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return loaded_ ? static_cast<int>(targets_.size()) : 0;
}

bool ThreadSafeTargetProvider::getTarget(int index, TargetSnapshot &target) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!loaded_ || index < 0 || index >= static_cast<int>(snapshots_.size())) {
    return false;
  }
  target = snapshots_[static_cast<std::size_t>(index)];
  return true;
}

bool ThreadSafeTargetProvider::interpolateUnlocked(int targetIndex, double timeSeconds, Coord &position) const
{
  if (!loaded_ || targetIndex < 0 || targetIndex >= static_cast<int>(targets_.size()) || timeSteps_ <= 1 ||
      arrayTimeStep_ <= 0.0f) {
    return false;
  }

  const std::vector<Coord> &trajectory = targets_[static_cast<std::size_t>(targetIndex)];
  if (static_cast<int>(trajectory.size()) != timeSteps_) {
    return false;
  }

  const double cycle = static_cast<double>(timeSteps_) * static_cast<double>(arrayTimeStep_);
  if (cycle <= 0.0) {
    return false;
  }

  double wrappedTime = std::fmod(timeSeconds, cycle);
  if (wrappedTime < 0.0) {
    wrappedTime += cycle;
  }

  const int index = static_cast<int>(std::floor(wrappedTime / static_cast<double>(arrayTimeStep_))) % timeSteps_;
  const int nextIndex = (index + 1) % timeSteps_;
  const double fraction =
    (wrappedTime - static_cast<double>(index) * static_cast<double>(arrayTimeStep_)) / static_cast<double>(arrayTimeStep_);

  const Coord &p0 = trajectory[static_cast<std::size_t>(index)];
  const Coord &p1 = trajectory[static_cast<std::size_t>(nextIndex)];
  position.x = static_cast<float>(p0.x + (p1.x - p0.x) * fraction);
  position.y = static_cast<float>(p0.y + (p1.y - p0.y) * fraction);
  return true;
}

bool ThreadSafeTargetProvider::interpolateTargetPosition(int targetIndex, double timeSeconds, Coord &position) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return interpolateUnlocked(targetIndex, timeSeconds, position);
}

bool ThreadSafeTargetProvider::velocityFromPastSamples(int targetIndex, double atTimeSeconds, Coord &velocity) const
{
  Coord posNow{};
  Coord posPast{};
  if (!interpolateUnlocked(targetIndex, atTimeSeconds, posNow)) {
    return false;
  }
  if (!interpolateUnlocked(targetIndex, atTimeSeconds - static_cast<double>(arrayTimeStep_), posPast)) {
    return false;
  }

  const double dt = static_cast<double>(arrayTimeStep_);
  velocity.x = static_cast<float>((posNow.x - posPast.x) / dt);
  velocity.y = static_cast<float>((posNow.y - posPast.y) / dt);
  return true;
}

bool ThreadSafeTargetProvider::extrapolateTargetPosition(int targetIndex,
                                                         double fromTimeSeconds,
                                                         double toTimeSeconds,
                                                         Coord &position) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!loaded_ || targetIndex < 0 || targetIndex >= static_cast<int>(targets_.size()) || arrayTimeStep_ <= 0.0f) {
    return false;
  }

  Coord posNow{};
  Coord velocity{};
  if (!interpolateUnlocked(targetIndex, fromTimeSeconds, posNow)) {
    return false;
  }
  if (!velocityFromPastSamples(targetIndex, fromTimeSeconds, velocity)) {
    return false;
  }

  const double deltaT = toTimeSeconds - fromTimeSeconds;
  position.x = static_cast<float>(posNow.x + velocity.x * deltaT);
  position.y = static_cast<float>(posNow.y + velocity.y * deltaT);
  return true;
}

bool ThreadSafeTargetProvider::isThreadReady() const
{
  return ready_.load();
}

void ThreadSafeTargetProvider::start()
{
  running_.store(true);
}

void ThreadSafeTargetProvider::stop()
{
  stopRequested_.store(true);
}

void ThreadSafeTargetProvider::run()
{
  ready_.store(true);
  while (!running_.load()) {
    if (stopRequested_.load()) {
      return;
    }
    paceSimulation(0.001, timeScale_);
  }

  while (!stopRequested_.load()) {
    advanceOnce();
    paceSimulation(clock_.stepDt(), timeScale_);
  }
}
