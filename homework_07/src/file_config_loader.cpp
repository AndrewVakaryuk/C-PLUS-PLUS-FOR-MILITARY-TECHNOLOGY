#include "file_config_loader.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

#include "json.hpp"

namespace {
using json = nlohmann::json;
}  // namespace

FileConfigLoader::FileConfigLoader()
  : loaded_(false)
{}

std::string FileConfigLoader::buildPath(const std::string &dir, const std::string &fileName)
{
  return (std::filesystem::path(dir) / fileName).string();
}

bool FileConfigLoader::loadConfigJson(const std::string &configPath)
{
  std::ifstream inputFile(configPath);
  if (!inputFile.is_open()) {
    std::cerr << "Error: could not open " << configPath << std::endl;
    return false;
  }

  json j;
  try {
    inputFile >> j;
    config_.startPos.x = j.at("drone").at("position").at("x").get<float>();
    config_.startPos.y = j.at("drone").at("position").at("y").get<float>();
    config_.altitude = j.at("drone").at("altitude").get<float>();
    config_.initialDir = j.at("drone").at("initialDirection").get<float>();
    config_.attackSpeed = j.at("drone").at("attackSpeed").get<float>();
    config_.accelPath = j.at("drone").at("accelerationPath").get<float>();
    config_.angularSpeed = j.at("drone").at("angularSpeed").get<float>();
    config_.turnThreshold = j.at("drone").at("turnThreshold").get<float>();
    config_.simTimeStep = j.at("simulation").at("timeStep").get<float>();
    config_.hitRadius = j.at("simulation").at("hitRadius").get<float>();
    config_.arrayTimeStep = j.at("targetArrayTimeStep").get<float>();
    config_.ammoName = j.at("ammo").get<std::string>();
  }
  catch (const std::exception &e) {
    std::cerr << "Error: invalid " << configPath << ": " << e.what() << std::endl;
    return false;
  }

  return true;
}

bool FileConfigLoader::loadAmmoJson(const std::string &ammoPath)
{
  std::ifstream ammoFile(ammoPath);
  if (!ammoFile.is_open()) {
    std::cerr << "Error: could not open " << ammoPath << std::endl;
    return false;
  }

  json j;
  try {
    ammoFile >> j;
    if (!j.is_array()) {
      std::cerr << "Error: ammo config must be array" << std::endl;
      return false;
    }

    if (j.empty()) {
      std::cerr << "Error: ammo config is empty" << std::endl;
      return false;
    }

    ammoByName_.clear();
    for (const auto &entry : j) {
      AmmoParams ammo{};
      ammo.name = entry.at("name").get<std::string>();
      ammo.mass = entry.at("mass").get<float>();
      ammo.drag = entry.at("drag").get<float>();
      ammo.lift = entry.at("lift").get<float>();
      ammoByName_[ammo.name] = ammo;
    }
  }
  catch (const std::exception &e) {
    std::cerr << "Error: invalid " << ammoPath << ": " << e.what() << std::endl;
    return false;
  }

  return true;
}

bool FileConfigLoader::load(const char *configSource)
{
  loaded_ = false;
  config_ = DroneConfig{};
  ammoByName_.clear();

  if (configSource == nullptr) {
    std::cerr << "Error: config source path invalid" << std::endl;
    return false;
  }

  const std::string sourceDir = configSource;
  const std::string configPath = buildPath(sourceDir, "config.json");
  const std::string ammoPath = buildPath(sourceDir, "ammo.json");

  if (!loadConfigJson(configPath)) {
    return false;
  }
  if (!loadAmmoJson(ammoPath)) {
    ammoByName_.clear();
    return false;
  }

  loaded_ = true;
  return true;
}

bool FileConfigLoader::getConfig(DroneConfig &config) const
{
  if (!loaded_) {
    return false;
  }
  config = config_;
  return true;
}

bool FileConfigLoader::getAmmoParams(const std::string &ammoName, AmmoParams &ammo) const
{
  if (!loaded_) {
    return false;
  }

  const auto it = ammoByName_.find(ammoName);
  if (it == ammoByName_.end()) {
    return false;
  }

  ammo = it->second;
  return true;
}
