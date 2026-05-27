#ifndef HOMEWORK_07_INTERFACES_I_TARGET_PROVIDER_HPP
#define HOMEWORK_07_INTERFACES_I_TARGET_PROVIDER_HPP

#include "domain_types.hpp"

class ITargetProvider {
public:
  virtual ~ITargetProvider() {}

  virtual int getTargetCount() const = 0;
  virtual bool getTarget(int index, TargetSnapshot &target) const = 0;
};

#endif
