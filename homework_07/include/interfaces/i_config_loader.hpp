#ifndef HOMEWORK_07_INTERFACES_I_CONFIG_LOADER_HPP
#define HOMEWORK_07_INTERFACES_I_CONFIG_LOADER_HPP

#include "../domain_types.hpp"

class IConfigLoader {
public:
  virtual ~IConfigLoader() {}

  virtual bool load(const char *configSource) = 0;
  virtual bool getConfig(DroneConfig &config) const = 0;
  virtual bool getAmmoParams(const char *ammoName, AmmoParams &ammo) const = 0;
};

#endif
