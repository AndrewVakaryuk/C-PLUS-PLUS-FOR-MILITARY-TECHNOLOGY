#pragma once

#include <string>

#include "domain_types.hpp"

class IConfigLoader {
public:
  virtual ~IConfigLoader() {}

  virtual bool load(const char *configSource) = 0;
  virtual bool getConfig(DroneConfig &config) const = 0;
  virtual bool getAmmoParams(const std::string &ammoName, AmmoParams &ammo) const = 0;
};
