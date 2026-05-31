#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "../include/json.hpp"
#include "../include/mission_demo.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

TEST(Homework07MissionDemo, WritesLegacySimulationJsonForBaseCircle)
{
  const std::string dataDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  const fs::path outputDir = fs::path(HW7_OUTPUT_DIR) / "mission-demo";
  const fs::path outputFile = outputDir / "simulation.json";

  std::error_code ec;
  fs::create_directories(outputDir, ec);
  fs::remove(outputFile, ec);

  ASSERT_EQ(runMissionDemo(dataDir.c_str(), outputDir.string().c_str()), 0);
  ASSERT_TRUE(fs::exists(outputFile));

  std::ifstream input(outputFile);
  ASSERT_TRUE(input.is_open());

  json simulation;
  ASSERT_NO_THROW(input >> simulation);
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
}
