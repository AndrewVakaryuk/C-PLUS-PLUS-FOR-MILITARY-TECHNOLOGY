#pragma once

#include <string>
#include <vector>

#include "interfaces/i_target_provider.hpp"

class JsonTargetProvider : public ITargetProvider {
public:
  explicit JsonTargetProvider(const char *dataSourceDir);
  ~JsonTargetProvider() override = default;

  int getTargetCount() const override;
  bool getTarget(int index, TargetSnapshot &target) const override;
  bool interpolateTargetPosition(int targetIndex, double timeSeconds, Coord &position) const override;

private:
  JsonTargetProvider(const JsonTargetProvider &) = delete;
  JsonTargetProvider &operator=(const JsonTargetProvider &) = delete;

  static std::string buildPath(const std::string &dir, const std::string &fileName);
  bool load(const char *dataSourceDir);
  void clear();

  std::vector<std::vector<Coord>> targets_;
  int timeSteps_;
  float arrayTimeStep_;
  bool loaded_;
};
