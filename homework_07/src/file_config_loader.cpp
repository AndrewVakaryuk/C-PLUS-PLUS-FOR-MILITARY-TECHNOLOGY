#include "../include/file_config_loader.hpp"

#include <cstring>
#include <fstream>
#include <iostream>

#include "../include/json.hpp"

namespace {
using json = nlohmann::json;
constexpr int kMaxPathLen = 512;
}  // namespace

FileConfigLoader::FileConfigLoader()
  : ammo_(nullptr)
  , ammoCount_(0)
  , loaded_(false)
{
  std::memset(&config_, 0, sizeof(config_));
}

FileConfigLoader::~FileConfigLoader()
{
  clearAmmo();
}

void FileConfigLoader::clearAmmo()
{
  delete[] ammo_;
  ammo_ = nullptr;
  ammoCount_ = 0;
}

bool FileConfigLoader::buildPath(const char *dir, const char *fileName, char *outPath, int outPathSize) const
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

  std::strcpy(outPath, dir);
  if (!hasSlash) {
    std::strcat(outPath, "/");
  }
  std::strcat(outPath, fileName);
  return true;
}

bool FileConfigLoader::loadConfigJson(const char *configPath)
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

    const json::string_t &ammoJson = j.at("ammo").get_ref<const json::string_t &>();
    std::strncpy(config_.ammoName, ammoJson.c_str(), sizeof(config_.ammoName) - 1);
    config_.ammoName[sizeof(config_.ammoName) - 1] = '\0';
  }
  catch (const std::exception &e) {
    std::cerr << "Error: invalid " << configPath << ": " << e.what() << std::endl;
    return false;
  }

  return true;
}

bool FileConfigLoader::loadAmmoJson(const char *ammoPath)
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

    const int parsedCount = static_cast<int>(j.size());
    if (parsedCount <= 0) {
      std::cerr << "Error: ammo config is empty" << std::endl;
      return false;
    }

    AmmoParams *parsedAmmo = new AmmoParams[parsedCount];
    for (int i = 0; i < parsedCount; i++) {
      const json::string_t &nameJson = j.at(i).at("name").get_ref<const json::string_t &>();
      std::strncpy(parsedAmmo[i].name, nameJson.c_str(), sizeof(parsedAmmo[i].name) - 1);
      parsedAmmo[i].name[sizeof(parsedAmmo[i].name) - 1] = '\0';
      parsedAmmo[i].mass = j.at(i).at("mass").get<float>();
      parsedAmmo[i].drag = j.at(i).at("drag").get<float>();
      parsedAmmo[i].lift = j.at(i).at("lift").get<float>();
    }

    clearAmmo();
    ammo_ = parsedAmmo;
    ammoCount_ = parsedCount;
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
  std::memset(&config_, 0, sizeof(config_));
  clearAmmo();

  char configPath[kMaxPathLen] = {};
  char ammoPath[kMaxPathLen] = {};

  if (!buildPath(configSource, "config.json", configPath, kMaxPathLen) || !buildPath(configSource, "ammo.json", ammoPath, kMaxPathLen)) {
    std::cerr << "Error: config source path too long or invalid" << std::endl;
    return false;
  }

  if (!loadConfigJson(configPath)) {
    return false;
  }
  if (!loadAmmoJson(ammoPath)) {
    clearAmmo();
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

bool FileConfigLoader::getAmmoParams(const char *ammoName, AmmoParams &ammo) const
{
  if (!loaded_ || ammoName == nullptr || ammo_ == nullptr) {
    return false;
  }

  for (int i = 0; i < ammoCount_; i++) {
    if (std::strcmp(ammo_[i].name, ammoName) == 0) {
      ammo = ammo_[i];
      return true;
    }
  }
  return false;
}
