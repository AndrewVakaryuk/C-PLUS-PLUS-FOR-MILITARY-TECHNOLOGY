#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "../include/json.hpp"
#include "../include/mission_demo.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

TEST(Homework07MissionDemo, WritesMissionResultsForBaseCircle)
{
  const std::string dataDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  const fs::path outputDir = fs::path(HW7_OUTPUT_DIR) / "mission-demo";
  const fs::path outputFile = outputDir / "mission_results.json";

  std::error_code ec;
  fs::create_directories(outputDir, ec);
  fs::remove(outputFile, ec);

  ASSERT_EQ(runMissionDemo(dataDir.c_str(), outputDir.string().c_str()), 0);
  ASSERT_TRUE(fs::exists(outputFile));

  std::ifstream input(outputFile);
  ASSERT_TRUE(input.is_open());

  json doc;
  ASSERT_NO_THROW(input >> doc);
  ASSERT_TRUE(doc.contains("totalSteps"));
  ASSERT_TRUE(doc.contains("steps"));
  ASSERT_TRUE(doc.at("steps").is_array());
  EXPECT_EQ(doc.at("totalSteps").get<int>(), 5);
  ASSERT_EQ(doc.at("steps").size(), 5);

  const json &first = doc.at("steps").at(0);
  EXPECT_TRUE(first.at("ok").get<bool>());
  EXPECT_TRUE(first.contains("dropPoint"));
  EXPECT_TRUE(first.contains("travelTime"));
  EXPECT_TRUE(first.contains("impactTime"));
}
