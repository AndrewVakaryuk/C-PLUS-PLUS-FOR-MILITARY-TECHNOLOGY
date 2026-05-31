#include "../include/drone_kinematics.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
constexpr double kEpsilon = 1e-9;

double rotateToward(double currentDir, double desiredDir, double maxStepRad)
{
  double delta = angleDelta(currentDir, desiredDir);
  if (std::fabs(delta) <= maxStepRad) {
    return wrapAngle(desiredDir);
  }
  return wrapAngle(currentDir + std::copysign(maxStepRad, delta));
}

void advanceAlongHeading(double averageSpeed, double direction, double simTimeStep, Coord &position)
{
  position.x += static_cast<float>(averageSpeed * std::cos(direction) * simTimeStep);
  position.y += static_cast<float>(averageSpeed * std::sin(direction) * simTimeStep);
}

void decelerateThisStep(double simTimeStep, double acceleration, DroneState &state, double &speed, double direction, Coord &position)
{
  if (acceleration <= kEpsilon) {
    speed = 0.0;
    state = DRONE_TURNING;
    return;
  }
  const double speedBefore = speed;
  const double speedAfter = std::max(0.0, speedBefore - acceleration * simTimeStep);
  advanceAlongHeading(0.5 * (speedBefore + speedAfter), direction, simTimeStep, position);
  speed = speedAfter;
  state = (speed <= 1e-6) ? DRONE_TURNING : DRONE_DECELERATING;
  if (speed <= 1e-6) {
    speed = 0.0;
  }
}
}  // namespace

double wrapAngle(double radians)
{
  while (radians > M_PI) {
    radians -= 2.0 * M_PI;
  }
  while (radians < -M_PI) {
    radians += 2.0 * M_PI;
  }
  return radians;
}

double angleDelta(double fromRadians, double toRadians)
{
  return wrapAngle(toRadians - fromRadians);
}

double computeTimeToStop(DroneState state,
                         double speed,
                         double acceleration,
                         double angularSpeed,
                         double remainingTurnRadians)
{
  switch (state) {
    case DRONE_STOPPED:
      return 0.0;
    case DRONE_ACCELERATING:
    case DRONE_DECELERATING:
    case DRONE_MOVING:
      if (acceleration <= kEpsilon) {
        return 1e9;
      }
      return speed / acceleration;
    case DRONE_TURNING:
      if (angularSpeed <= kEpsilon) {
        return 1e9;
      }
      return std::fabs(remainingTurnRadians) / angularSpeed;
    default:
      return 0.0;
  }
}

void updateDrone(double simTimeStep,
                 double desiredDirection,
                 double turnThresholdRadians,
                 double attackSpeed,
                 double accelerationPath,
                 double angularSpeedRadiansPerSec,
                 DroneState &state,
                 double &speed,
                 double &direction,
                 Coord &position)
{
  const double acceleration = (accelerationPath > kEpsilon) ? (attackSpeed * attackSpeed) / (2.0 * accelerationPath) : 0.0;

  double delta = angleDelta(direction, desiredDirection);
  const double angularStep = angularSpeedRadiansPerSec * simTimeStep;

  switch (state) {
    case DRONE_STOPPED:
      if (std::fabs(delta) > turnThresholdRadians) {
        state = DRONE_TURNING;
      }
      else {
        state = DRONE_ACCELERATING;
      }
      break;
    case DRONE_TURNING:
      direction = rotateToward(direction, desiredDirection, angularStep);
      delta = angleDelta(direction, desiredDirection);
      if (std::fabs(delta) < 1e-4) {
        state = DRONE_ACCELERATING;
      }
      break;
    case DRONE_DECELERATING:
      decelerateThisStep(simTimeStep, acceleration, state, speed, direction, position);
      break;
    case DRONE_ACCELERATING:
      if (acceleration <= kEpsilon) {
        speed = attackSpeed;
        state = DRONE_MOVING;
        break;
      }
      if (std::fabs(delta) > turnThresholdRadians && speed > 1e-3) {
        decelerateThisStep(simTimeStep, acceleration, state, speed, direction, position);
        break;
      }
      {
        const double speedBefore = speed;
        const double speedAfter = std::min(attackSpeed, speedBefore + acceleration * simTimeStep);
        advanceAlongHeading(0.5 * (speedBefore + speedAfter), direction, simTimeStep, position);
        speed = speedAfter;
        if (speed >= attackSpeed - 1e-6) {
          speed = attackSpeed;
          state = DRONE_MOVING;
        }
      }
      break;
    case DRONE_MOVING:
      if (std::fabs(delta) > turnThresholdRadians) {
        decelerateThisStep(simTimeStep, acceleration, state, speed, direction, position);
        break;
      }
      direction = rotateToward(direction, desiredDirection, angularStep);
      advanceAlongHeading(speed, direction, simTimeStep, position);
      break;
    default:
      break;
  }
}
