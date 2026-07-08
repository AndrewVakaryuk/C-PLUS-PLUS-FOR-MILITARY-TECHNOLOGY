#include "drone_kinematics.hpp"

#include <cmath>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
constexpr double kEpsilon = 1e-9;

class StateStopped;
class StateAccelerating;
class StateDecelerating;
class StateTurning;
class StateMoving;

std::unique_ptr<IDroneState> makeStopped();
std::unique_ptr<IDroneState> makeAccelerating();
std::unique_ptr<IDroneState> makeDecelerating();
std::unique_ptr<IDroneState> makeTurning();
std::unique_ptr<IDroneState> makeMoving();

double currentAcceleration(const DroneContext &ctx)
{
  if (ctx.accelerationPath <= kEpsilon) {
    return 0.0;
  }
  return (ctx.attackSpeed * ctx.attackSpeed) / (2.0 * ctx.accelerationPath);
}

double rotateToward(double currentDir, double desiredDir, double maxStepRad)
{
  const double delta = angleDelta(currentDir, desiredDir);
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

class StateStopped final : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext &ctx) const override
  {
    const double delta = angleDelta(ctx.direction, ctx.desiredDirection);
    if (std::fabs(delta) > ctx.turnThresholdRadians) {
      return makeTurning();
    }
    return makeAccelerating();
  }

  const char *name() const override { return "Stopped"; }
  int code() const override { return DRONE_STATE_STOPPED; }
  bool canRelease() const override { return false; }
  bool isTurning() const override { return false; }
  double estimateStopTime(const DroneContext &) const override { return 0.0; }
};

class StateTurning final : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext &ctx) const override
  {
    const double angularStep = ctx.angularSpeedRadiansPerSec * ctx.simTimeStep;
    ctx.direction = rotateToward(ctx.direction, ctx.desiredDirection, angularStep);
    const double delta = angleDelta(ctx.direction, ctx.desiredDirection);
    if (std::fabs(delta) < 1e-4) {
      return makeAccelerating();
    }
    return nullptr;
  }

  const char *name() const override { return "Turning"; }
  int code() const override { return DRONE_STATE_TURNING; }
  bool canRelease() const override { return false; }
  bool isTurning() const override { return true; }
  double estimateStopTime(const DroneContext &ctx) const override
  {
    if (ctx.angularSpeedRadiansPerSec <= kEpsilon) {
      return 1e9;
    }
    return std::fabs(angleDelta(ctx.direction, ctx.desiredDirection)) / ctx.angularSpeedRadiansPerSec;
  }
};

class StateDecelerating final : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext &ctx) const override
  {
    const double acceleration = currentAcceleration(ctx);
    if (acceleration <= kEpsilon) {
      ctx.speed = 0.0;
      return makeTurning();
    }

    const double speedBefore = ctx.speed;
    const double speedAfter = std::max(0.0, speedBefore - acceleration * ctx.simTimeStep);
    advanceAlongHeading(0.5 * (speedBefore + speedAfter), ctx.direction, ctx.simTimeStep, ctx.position);
    ctx.speed = speedAfter;
    if (ctx.speed <= 1e-6) {
      ctx.speed = 0.0;
      return makeTurning();
    }
    return nullptr;
  }

  const char *name() const override { return "Decelerating"; }
  int code() const override { return DRONE_STATE_DECELERATING; }
  bool canRelease() const override { return true; }
  bool isTurning() const override { return false; }
  double estimateStopTime(const DroneContext &ctx) const override
  {
    const double acceleration = currentAcceleration(ctx);
    if (acceleration <= kEpsilon) {
      return 1e9;
    }
    return ctx.speed / acceleration;
  }
};

class StateMoving final : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext &ctx) const override
  {
    const double delta = angleDelta(ctx.direction, ctx.desiredDirection);
    if (std::fabs(delta) > ctx.turnThresholdRadians) {
      const double acceleration = currentAcceleration(ctx);
      if (acceleration <= kEpsilon) {
        ctx.speed = 0.0;
        return makeTurning();
      }

      const double speedBefore = ctx.speed;
      const double speedAfter = std::max(0.0, speedBefore - acceleration * ctx.simTimeStep);
      advanceAlongHeading(0.5 * (speedBefore + speedAfter), ctx.direction, ctx.simTimeStep, ctx.position);
      ctx.speed = speedAfter;
      if (ctx.speed <= 1e-6) {
        ctx.speed = 0.0;
        return makeTurning();
      }
      return makeDecelerating();
    }

    const double angularStep = ctx.angularSpeedRadiansPerSec * ctx.simTimeStep;
    ctx.direction = rotateToward(ctx.direction, ctx.desiredDirection, angularStep);
    advanceAlongHeading(ctx.speed, ctx.direction, ctx.simTimeStep, ctx.position);
    return nullptr;
  }

  const char *name() const override { return "Moving"; }
  int code() const override { return DRONE_STATE_MOVING; }
  bool canRelease() const override { return true; }
  bool isTurning() const override { return false; }
  double estimateStopTime(const DroneContext &ctx) const override
  {
    const double acceleration = currentAcceleration(ctx);
    if (acceleration <= kEpsilon) {
      return 1e9;
    }
    return ctx.speed / acceleration;
  }
};

class StateAccelerating final : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext &ctx) const override
  {
    const double acceleration = currentAcceleration(ctx);
    const double delta = angleDelta(ctx.direction, ctx.desiredDirection);

    if (acceleration <= kEpsilon) {
      ctx.speed = ctx.attackSpeed;
      return makeMoving();
    }

    if (std::fabs(delta) > ctx.turnThresholdRadians && ctx.speed > 1e-3) {
      const double speedBefore = ctx.speed;
      const double speedAfter = std::max(0.0, speedBefore - acceleration * ctx.simTimeStep);
      advanceAlongHeading(0.5 * (speedBefore + speedAfter), ctx.direction, ctx.simTimeStep, ctx.position);
      ctx.speed = speedAfter;
      if (ctx.speed <= 1e-6) {
        ctx.speed = 0.0;
        return makeTurning();
      }
      return makeDecelerating();
    }

    const double speedBefore = ctx.speed;
    const double speedAfter = std::min(ctx.attackSpeed, speedBefore + acceleration * ctx.simTimeStep);
    advanceAlongHeading(0.5 * (speedBefore + speedAfter), ctx.direction, ctx.simTimeStep, ctx.position);
    ctx.speed = speedAfter;
    if (ctx.speed >= ctx.attackSpeed - 1e-6) {
      ctx.speed = ctx.attackSpeed;
      return makeMoving();
    }
    return nullptr;
  }

  const char *name() const override { return "Accelerating"; }
  int code() const override { return DRONE_STATE_ACCELERATING; }
  bool canRelease() const override { return true; }
  bool isTurning() const override { return false; }
  double estimateStopTime(const DroneContext &ctx) const override
  {
    const double acceleration = currentAcceleration(ctx);
    if (acceleration <= kEpsilon) {
      return 1e9;
    }
    return ctx.speed / acceleration;
  }
};

std::unique_ptr<IDroneState> makeStopped() { return std::make_unique<StateStopped>(); }
std::unique_ptr<IDroneState> makeAccelerating() { return std::make_unique<StateAccelerating>(); }
std::unique_ptr<IDroneState> makeDecelerating() { return std::make_unique<StateDecelerating>(); }
std::unique_ptr<IDroneState> makeTurning() { return std::make_unique<StateTurning>(); }
std::unique_ptr<IDroneState> makeMoving() { return std::make_unique<StateMoving>(); }
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

double computeTimeToStop(const IDroneState &state, const DroneContext &ctx)
{
  return state.estimateStopTime(ctx);
}

std::unique_ptr<IDroneState> createInitialDroneState()
{
  return makeStopped();
}

void updateDrone(std::unique_ptr<IDroneState> &state, DroneContext &ctx)
{
  if (!state) {
    state = createInitialDroneState();
  }
  std::unique_ptr<IDroneState> next = state->execute(ctx);
  if (next) {
    state = std::move(next);
  }
}
