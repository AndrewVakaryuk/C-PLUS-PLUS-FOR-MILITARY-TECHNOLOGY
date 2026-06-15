#ifndef HOMEWORK_07_FILE_CONFIG_LOADER_HPP
#define HOMEWORK_07_FILE_CONFIG_LOADER_HPP

#include "interfaces/i_config_loader.hpp"

class FileConfigLoader : public IConfigLoader {
public:
  FileConfigLoader();
  ~FileConfigLoader() override;

  bool load(const char *configSource) override;
  bool getConfig(DroneConfig &config) const override;
  bool getAmmoParams(const char *ammoName, AmmoParams &ammo) const override;

private:
  FileConfigLoader(const FileConfigLoader &) = delete;
  FileConfigLoader &operator=(const FileConfigLoader &) = delete;

  void clearAmmo();
  bool loadConfigJson(const char *configPath);
  bool loadAmmoJson(const char *ammoPath);
  bool buildPath(const char *dir, const char *fileName, char *outPath, int outPathSize) const;

  DroneConfig config_;
  AmmoParams *ammo_;
  int ammoCount_;
  bool loaded_;
};

#endif
