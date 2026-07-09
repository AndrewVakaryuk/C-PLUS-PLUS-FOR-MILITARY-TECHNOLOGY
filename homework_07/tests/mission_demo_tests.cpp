#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "json.hpp"
#include "mission_demo.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

std::vector<std::string> allScenarioNames()
{
  return {
    "base-circle",
    "elliptical-trajectories",
    "figure-eight-trajectory",
    "star-trajectories",
    "Lissajous-trajectory",
    "fast-drone_slow-targets",
    "heavy-ammo",
    "gliding-ammo",
    "cardioid-epitrochoid",
    "extreme",
  };
}

void assertSimulationJsonShape(const json &simulation)
{
  ASSERT_TRUE(simulation.contains("totalSteps"));
  ASSERT_TRUE(simulation.contains("steps"));
  ASSERT_TRUE(simulation.at("steps").is_array());

  const int totalSteps = simulation.at("totalSteps").get<int>();
  EXPECT_GT(totalSteps, 10);
  EXPECT_EQ(static_cast<int>(simulation.at("steps").size()), totalSteps);

  const json &firstStep = simulation.at("steps").at(0);
  EXPECT_TRUE(firstStep.contains("position"));
  EXPECT_TRUE(firstStep.contains("direction"));
  EXPECT_TRUE(firstStep.contains("state"));
  EXPECT_TRUE(firstStep.contains("targetIndex"));
  EXPECT_TRUE(firstStep.contains("dropPoint"));
  EXPECT_TRUE(firstStep.contains("aimPoint"));
  EXPECT_TRUE(firstStep.contains("predictedTarget"));
  EXPECT_TRUE(firstStep.contains("timeSecSinceStart"));
}

}  // namespace

class Homework07ScenarioE2ETest : public ::testing::TestWithParam<std::string> {};

TEST_P(Homework07ScenarioE2ETest, RunMissionDemoProducesSimulationJson)
{
  const std::string scenarioName = GetParam();
  const std::string dataDir = (fs::path(HW7_SCENARIOS_ROOT) / scenarioName).string();
  const fs::path outputDir = fs::path(HW7_OUTPUT_DIR) / "scenarios" / scenarioName;
  const fs::path outputFile = outputDir / "simulation.json";

  ASSERT_TRUE(fs::exists(fs::path(dataDir) / "config.json")) << scenarioName;
  ASSERT_TRUE(fs::exists(fs::path(dataDir) / "ammo.json")) << scenarioName;
  ASSERT_TRUE(fs::exists(fs::path(dataDir) / "targets.json")) << scenarioName;

  std::error_code ec;
  fs::create_directories(outputDir, ec);
  fs::remove(outputFile, ec);

  setenv("HW7_SYNC_MISSION", "1", 1);
  ASSERT_EQ(runMissionDemo(dataDir.c_str(), outputDir.string().c_str()), 0) << scenarioName;
  ASSERT_TRUE(fs::exists(outputFile)) << scenarioName;

  std::ifstream input(outputFile);
  ASSERT_TRUE(input.is_open()) << scenarioName;

  json simulation;
  ASSERT_NO_THROW(input >> simulation) << scenarioName;
  assertSimulationJsonShape(simulation);
}

INSTANTIATE_TEST_SUITE_P(Homework07AllScenarios, Homework07ScenarioE2ETest, ::testing::ValuesIn(allScenarioNames()));
