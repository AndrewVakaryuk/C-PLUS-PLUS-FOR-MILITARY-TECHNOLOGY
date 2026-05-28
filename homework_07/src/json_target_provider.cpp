#include "../include/json_target_provider.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>

#include "../include/json.hpp"

namespace {
using json = nlohmann::json;
constexpr int kMaxPathLen = 512;
}  // namespace

JsonTargetProvider::JsonTargetProvider(const char *dataSourceDir)
  : targets_(nullptr)
  , targetCount_(0)
  , timeSteps_(0)
  , arrayTimeStep_(1.0f)
  , loaded_(false)
{
  load(dataSourceDir);
}

JsonTargetProvider::~JsonTargetProvider()
{
  clear();
}

void JsonTargetProvider::clear()
{
  if (targets_ != nullptr) {
    for (int i = 0; i < targetCount_; i++) {
      delete[] targets_[i];
      targets_[i] = nullptr;
    }
    delete[] targets_;
    targets_ = nullptr;
  }
  targetCount_ = 0;
  timeSteps_ = 0;
  arrayTimeStep_ = 1.0f;
  loaded_ = false;
}

bool JsonTargetProvider::buildPath(const char *dir, const char *fileName, char *outPath, int outPathSize) const
{
  if (dir == nullptr || fileName == nullptr || outPath == nullptr || outPathSize <= 0) {
    return false;
  }

  const int dirLen = static_cast<int>(std::strlen(dir));
  const int fileLen = static_cast<int>(std::strlen(fileName));
  const bool hasSlash = (dirLen > 0 && dir[dirLen - 1] == '/');
  const int requiredLen = dirLen + (hasSlash ? 0 : 1) + fileLen + 1;
  if (requiredLen > outPathSize) {
    return false;
  }

  const int written = std::snprintf(outPath, static_cast<std::size_t>(outPathSize), hasSlash ? "%s%s" : "%s/%s", dir, fileName);
  return written > 0 && written < outPathSize;
}

bool JsonTargetProvider::load(const char *dataSourceDir)
{
  clear();

  char targetsPath[kMaxPathLen] = {};
  char configPath[kMaxPathLen] = {};
  if (!buildPath(dataSourceDir, "targets.json", targetsPath, kMaxPathLen) ||
      !buildPath(dataSourceDir, "config.json", configPath, kMaxPathLen)) {
    std::cerr << "Error: data source path too long or invalid" << std::endl;
    return false;
  }

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
  int allocatedRows = 0;
  try {
    targetsFile >> targetsJson;
    configFile >> configJson;

    targetCount_ = targetsJson.at("targetCount").get<int>();
    timeSteps_ = targetsJson.at("timeSteps").get<int>();
    arrayTimeStep_ = configJson.at("targetArrayTimeStep").get<float>();

    if (targetCount_ <= 0 || timeSteps_ <= 1 || arrayTimeStep_ <= 0.0f) {
      std::cerr << "Error: invalid target provider config values" << std::endl;
      clear();
      return false;
    }

    targets_ = new Coord *[targetCount_];
    for (int i = 0; i < targetCount_; i++) {
      targets_[i] = nullptr;
    }

    for (int i = 0; i < targetCount_; i++) {
      targets_[i] = new Coord[timeSteps_];
      allocatedRows++;
      for (int t = 0; t < timeSteps_; t++) {
        targets_[i][t].x = targetsJson.at("targets").at(i).at("positions").at(t).at("x").get<float>();
        targets_[i][t].y = targetsJson.at("targets").at(i).at("positions").at(t).at("y").get<float>();
      }
    }
  }
  catch (const std::exception &e) {
    std::cerr << "Error: invalid target data: " << e.what() << std::endl;
    if (targets_ != nullptr) {
      for (int i = 0; i < allocatedRows; i++) {
        delete[] targets_[i];
        targets_[i] = nullptr;
      }
      delete[] targets_;
      targets_ = nullptr;
    }
    targetCount_ = 0;
    timeSteps_ = 0;
    arrayTimeStep_ = 1.0f;
    loaded_ = false;
    return false;
  }

  loaded_ = true;
  return true;
}

int JsonTargetProvider::getTargetCount() const
{
  return loaded_ ? targetCount_ : 0;
}

bool JsonTargetProvider::getTarget(int index, TargetSnapshot &target) const
{
  if (!loaded_ || targets_ == nullptr || index < 0 || index >= targetCount_) {
    return false;
  }

  target.position = targets_[index][0];
  const Coord p0 = targets_[index][0];
  const Coord p1 = targets_[index][1];
  target.velocity.x = (p1.x - p0.x) / arrayTimeStep_;
  target.velocity.y = (p1.y - p0.y) / arrayTimeStep_;
  return true;
}
