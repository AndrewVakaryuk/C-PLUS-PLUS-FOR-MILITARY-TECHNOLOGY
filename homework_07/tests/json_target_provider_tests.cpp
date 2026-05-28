#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "../include/json_target_provider.hpp"

namespace fs = std::filesystem;

TEST(Homework07JsonTargetProvider, LoadsBaseCircleAndExposesCount)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  JsonTargetProvider provider(baseCircleDir.c_str());

  EXPECT_EQ(provider.getTargetCount(), 5);
}

TEST(Homework07JsonTargetProvider, ReadsFirstTargetPositionAndVelocity)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  JsonTargetProvider provider(baseCircleDir.c_str());

  TargetSnapshot target{};
  ASSERT_TRUE(provider.getTarget(0, target));
  EXPECT_FLOAT_EQ(target.position.x, 340.0f);
  EXPECT_FLOAT_EQ(target.position.y, 250.0f);

  // Based on first two points in base-circle with array dt = 10:
  // x: (339.95 - 340.0) / 10  = -0.005
  // y: (252.09 - 250.0) / 10  =  0.209
  EXPECT_NEAR(target.velocity.x, -0.005f, 1e-5f);
  EXPECT_NEAR(target.velocity.y, 0.209f, 1e-5f);
}

TEST(Homework07JsonTargetProvider, RejectsInvalidIndex)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  JsonTargetProvider provider(baseCircleDir.c_str());

  TargetSnapshot target{};
  EXPECT_FALSE(provider.getTarget(-1, target));
  EXPECT_FALSE(provider.getTarget(provider.getTargetCount(), target));
}

TEST(Homework07JsonTargetProvider, MissingDirectoryProducesEmptyProvider)
{
  const std::string missingDir = (fs::path(HW7_SCENARIOS_ROOT) / "does-not-exist").string();
  JsonTargetProvider provider(missingDir.c_str());

  EXPECT_EQ(provider.getTargetCount(), 0);
  TargetSnapshot target{};
  EXPECT_FALSE(provider.getTarget(0, target));
}
