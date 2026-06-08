#ifndef HOMEWORK_07_JSON_TARGET_PROVIDER_HPP
#define HOMEWORK_07_JSON_TARGET_PROVIDER_HPP

#include "interfaces/i_target_provider.hpp"

class JsonTargetProvider : public ITargetProvider {
public:
  explicit JsonTargetProvider(const char *dataSourceDir);
  ~JsonTargetProvider() override;

  int getTargetCount() const override;
  bool getTarget(int index, TargetSnapshot &target) const override;
  bool interpolateTargetPosition(int targetIndex, double timeSeconds, Coord &position) const override;

private:
  JsonTargetProvider(const JsonTargetProvider &) = delete;
  JsonTargetProvider &operator=(const JsonTargetProvider &) = delete;

  bool load(const char *dataSourceDir);
  void clear();
  bool buildPath(const char *dir, const char *fileName, char *outPath, int outPathSize) const;

  Coord **targets_;
  int targetCount_;
  int timeSteps_;
  float arrayTimeStep_;
  bool loaded_;
};

#endif
