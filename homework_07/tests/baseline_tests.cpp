#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

// Temporary characterization harness: include the current monolithic HW3 port
// directly so we can lock down behavior before the real refactor starts.
#define ENABLE_LOG 0
#define ENABLE_DEBUG 0
#define main homework_07_baseline_entrypoint
#include "../src/main.cpp"
#undef main

namespace fs = std::filesystem;

TEST(Homework07Baseline, ReadsBaselineConfig)
{
  DroneConfig config{};

  ASSERT_TRUE(readConfigJson(config));
  EXPECT_FLOAT_EQ(config.startPos.x, 150.0f);
  EXPECT_FLOAT_EQ(config.startPos.y, 150.0f);
  EXPECT_FLOAT_EQ(config.altitude, 100.0f);
  EXPECT_FLOAT_EQ(config.attackSpeed, 10.0f);
  EXPECT_STREQ(config.ammoName, "VOG-17");
  EXPECT_FLOAT_EQ(config.arrayTimeStep, 10.0f);
  EXPECT_FLOAT_EQ(config.simTimeStep, 0.1f);
  EXPECT_FLOAT_EQ(config.hitRadius, 3.0f);
}

TEST(Homework07Baseline, LoadsAmmoTableAndKnownAmmo)
{
  AmmoParams *ammo = nullptr;
  int ammoCount = 0;

  ASSERT_TRUE(readAmmoJson(ammo, ammoCount));
  ASSERT_NE(ammo, nullptr);
  EXPECT_EQ(ammoCount, 5);

  double mass = 0.0;
  double drag = 0.0;
  double lift = 0.0;
  EXPECT_TRUE(lookupAmmo("VOG-17", ammo, ammoCount, mass, drag, lift));
  EXPECT_NEAR(mass, 0.35, 1e-6);
  EXPECT_NEAR(drag, 0.07, 1e-6);
  EXPECT_NEAR(lift, 0.0, 1e-6);

  delete[] ammo;
}

TEST(Homework07Baseline, RejectsUnknownAmmo)
{
  AmmoParams *ammo = nullptr;
  int ammoCount = 0;

  ASSERT_TRUE(readAmmoJson(ammo, ammoCount));
  ASSERT_NE(ammo, nullptr);

  double mass = 0.0;
  double drag = 0.0;
  double lift = 0.0;
  EXPECT_FALSE(lookupAmmo("UNKNOWN-AMMO", ammo, ammoCount, mass, drag, lift));

  delete[] ammo;
}

TEST(Homework07Baseline, LoadsTargetTrajectoryData)
{
  Coord **targets = nullptr;
  int targetCount = 0;
  int timeSteps = 0;

  ASSERT_TRUE(readTargetsJson(targets, targetCount, timeSteps));
  ASSERT_NE(targets, nullptr);
  EXPECT_EQ(targetCount, 5);
  EXPECT_EQ(timeSteps, 120);

  EXPECT_FLOAT_EQ(targets[0][0].x, 340.0f);
  EXPECT_FLOAT_EQ(targets[0][0].y, 250.0f);
  EXPECT_FLOAT_EQ(targets[0][30].x, 300.0f);
  EXPECT_FLOAT_EQ(targets[0][30].y, 290.0f);

  freeTargets(targets, targetCount);
}

TEST(Homework07Baseline, BaselineRunProducesSimulationJson)
{
  const fs::path outputPath = fs::path(HW7_OUTPUT_DIR) / "simulation.json";
  std::error_code createDirError;
  fs::create_directories(outputPath.parent_path(), createDirError);
  std::error_code removeError;
  fs::remove(outputPath, removeError);

  ASSERT_EQ(homework_07_baseline_entrypoint(), 0);
  ASSERT_TRUE(fs::exists(outputPath));

  std::ifstream input(outputPath);
  ASSERT_TRUE(input.is_open());

  json simulation;
  ASSERT_NO_THROW(input >> simulation);
  ASSERT_TRUE(simulation.contains("totalSteps"));
  ASSERT_TRUE(simulation.contains("steps"));
  ASSERT_TRUE(simulation.at("steps").is_array());
  ASSERT_FALSE(simulation.at("steps").empty());

  const json &firstStep = simulation.at("steps").at(0);
  EXPECT_TRUE(firstStep.contains("position"));
  EXPECT_TRUE(firstStep.contains("direction"));
  EXPECT_TRUE(firstStep.contains("state"));
  EXPECT_TRUE(firstStep.contains("targetIndex"));
  EXPECT_TRUE(firstStep.contains("dropPoint"));
  EXPECT_TRUE(firstStep.contains("aimPoint"));
  EXPECT_TRUE(firstStep.contains("predictedTarget"));
}
