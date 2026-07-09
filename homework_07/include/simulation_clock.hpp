#pragma once

class SimulationClock {
public:
  explicit SimulationClock(double stepDt)
    : stepDt_(stepDt)
  {}

  void advance()
  {
    simTime_ += stepDt_;
  }

  void reset()
  {
    simTime_ = 0.0;
  }

  double now() const
  {
    return simTime_;
  }

  double stepDt() const
  {
    return stepDt_;
  }

private:
  double simTime_{0.0};
  double stepDt_;
};
