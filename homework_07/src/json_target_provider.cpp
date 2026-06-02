#include "../include/json_target_provider.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "../include/json.hpp"

namespace {
using json = nlohmann::json;
}  // namespace

JsonTargetProvider::JsonTargetProvider(const char *dataSourceDir)
  : timeSteps_(0)
  , arrayTimeStep_(1.0f)
  , loaded_(false)
{
  load(dataSourceDir);
}

void JsonTargetProvider::clear()
{
  targets_.clear();
  timeSteps_ = 0;
  arrayTimeStep_ = 1.0f;
  loaded_ = false;
}

std::string JsonTargetProvider::buildPath(const std::string &dir, const std::string &fileName)
{
  return (std::filesystem::path(dir) / fileName).string();
}

bool JsonTargetProvider::load(const char *dataSourceDir)
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
  }
  catch (const std::exception &e) {
    std::cerr << "Error: invalid target data: " << e.what() << std::endl;
    clear();
    return false;
  }

  loaded_ = true;
  return true;
}

int JsonTargetProvider::getTargetCount() const
{
  return loaded_ ? static_cast<int>(targets_.size()) : 0;
}

bool JsonTargetProvider::getTarget(int index, TargetSnapshot &target) const
{
  if (!loaded_ || index < 0 || index >= static_cast<int>(targets_.size())) {
    return false;
  }

  const std::vector<Coord> &trajectory = targets_[static_cast<std::size_t>(index)];
  if (trajectory.size() < 2) {
    return false;
  }

  target.position = trajectory[0];
  const Coord &p0 = trajectory[0];
  const Coord &p1 = trajectory[1];
  target.velocity.x = (p1.x - p0.x) / arrayTimeStep_;
  target.velocity.y = (p1.y - p0.y) / arrayTimeStep_;
  return true;
}

bool JsonTargetProvider::interpolateTargetPosition(int targetIndex, double timeSeconds, Coord &position) const
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
