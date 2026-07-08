#pragma once

#include <string>
#include <unordered_map>

#include "interfaces/i_config_loader.hpp"

class FileConfigLoader : public IConfigLoader {
public:
  FileConfigLoader();
  ~FileConfigLoader() override = default;

  bool load(const char *configSource) override;
  bool getConfig(DroneConfig &config) const override;
  bool getAmmoParams(const std::string &ammoName, AmmoParams &ammo) const override;

private:
  FileConfigLoader(const FileConfigLoader &) = delete;
  FileConfigLoader &operator=(const FileConfigLoader &) = delete;

  static std::string buildPath(const std::string &dir, const std::string &fileName);
  bool loadConfigJson(const std::string &configPath);
  bool loadAmmoJson(const std::string &ammoPath);

  DroneConfig config_;
  std::unordered_map<std::string, AmmoParams> ammoByName_;
  bool loaded_;
};
