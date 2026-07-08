#ifndef HOMEWORK_07_INTERFACES_I_TARGET_PROVIDER_HPP
#define HOMEWORK_07_INTERFACES_I_TARGET_PROVIDER_HPP

#include "domain_types.hpp"

class ITargetProvider {
public:
  virtual ~ITargetProvider() {}

  virtual int getTargetCount() const = 0;
  virtual bool getTarget(int index, TargetSnapshot &target) const = 0;
  // Replay position on the known cyclic trajectory (used for visualization / server parity).
  virtual bool interpolateTargetPosition(int targetIndex, double timeSeconds, Coord &position) const = 0;
  // Predict future position from velocity estimated at fromTimeSeconds using past samples only.
  virtual bool extrapolateTargetPosition(int targetIndex,
                                         double fromTimeSeconds,
                                         double toTimeSeconds,
                                         Coord &position) const = 0;
};

#endif
