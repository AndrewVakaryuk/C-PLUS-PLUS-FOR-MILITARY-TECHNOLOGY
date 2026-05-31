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

TEST(Homework07JsonTargetProvider, InterpolatesTargetPositionAtKnownTimes)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  JsonTargetProvider provider(baseCircleDir.c_str());

  Coord atZero{};
  ASSERT_TRUE(provider.interpolateTargetPosition(0, 0.0, atZero));
  EXPECT_FLOAT_EQ(atZero.x, 340.0f);
  EXPECT_FLOAT_EQ(atZero.y, 250.0f);

  Coord atTen{};
  ASSERT_TRUE(provider.interpolateTargetPosition(0, 10.0, atTen));
  EXPECT_NEAR(atTen.x, 339.95f, 0.01f);
  EXPECT_NEAR(atTen.y, 252.09f, 0.01f);
}

TEST(Homework07JsonTargetProvider, InterpolatesWithPeriodicWrap)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  JsonTargetProvider provider(baseCircleDir.c_str());

  Coord atCycle{};
  const double cycle = 120.0 * 10.0;
  ASSERT_TRUE(provider.interpolateTargetPosition(0, cycle, atCycle));
  EXPECT_FLOAT_EQ(atCycle.x, 340.0f);
  EXPECT_FLOAT_EQ(atCycle.y, 250.0f);
}

TEST(Homework07JsonTargetProvider, RejectsInvalidInterpolationRequest)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  JsonTargetProvider provider(baseCircleDir.c_str());

  Coord position{};
  EXPECT_FALSE(provider.interpolateTargetPosition(-1, 0.0, position));
  EXPECT_FALSE(provider.interpolateTargetPosition(provider.getTargetCount(), 0.0, position));
}
