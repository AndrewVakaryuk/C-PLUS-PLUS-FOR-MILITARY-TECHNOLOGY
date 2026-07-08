#ifndef HOMEWORK_07_DRONE_KINEMATICS_HPP
#define HOMEWORK_07_DRONE_KINEMATICS_HPP

#include <memory>

#include "domain_types.hpp"

constexpr int DRONE_STATE_STOPPED = 0;
constexpr int DRONE_STATE_ACCELERATING = 1;
constexpr int DRONE_STATE_DECELERATING = 2;
constexpr int DRONE_STATE_TURNING = 3;
constexpr int DRONE_STATE_MOVING = 4;

struct DroneContext {
  double simTimeStep;
  double desiredDirection;
  double turnThresholdRadians;
  double attackSpeed;
  double accelerationPath;
  double angularSpeedRadiansPerSec;
  double &speed;
  double &direction;
  Coord &position;
};

class IDroneState {
public:
  virtual ~IDroneState() = default;
  virtual std::unique_ptr<IDroneState> execute(DroneContext &ctx) const = 0;
  virtual const char *name() const = 0;
  virtual int code() const = 0;
  virtual bool canRelease() const = 0;
  virtual bool isTurning() const = 0;
  virtual double estimateStopTime(const DroneContext &ctx) const = 0;
};

double wrapAngle(double radians);
double angleDelta(double fromRadians, double toRadians);
double computeTimeToStop(const IDroneState &state, const DroneContext &ctx);
std::unique_ptr<IDroneState> createInitialDroneState();
void updateDrone(std::unique_ptr<IDroneState> &state, DroneContext &ctx);

#endif
