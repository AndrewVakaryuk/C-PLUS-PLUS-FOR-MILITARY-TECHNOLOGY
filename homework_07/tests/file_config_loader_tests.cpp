#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "file_config_loader.hpp"

namespace fs = std::filesystem;

TEST(Homework07FileConfigLoader, LoadsBaseCircleConfigAndAmmo)
{
  FileConfigLoader loader;
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();

  ASSERT_TRUE(loader.load(baseCircleDir.c_str()));

  DroneConfig config{};
  ASSERT_TRUE(loader.getConfig(config));
  EXPECT_FLOAT_EQ(config.startPos.x, 150.0f);
  EXPECT_FLOAT_EQ(config.startPos.y, 150.0f);
  EXPECT_FLOAT_EQ(config.altitude, 100.0f);
  EXPECT_FLOAT_EQ(config.attackSpeed, 10.0f);
  EXPECT_EQ(config.ammoName, "VOG-17");

  AmmoParams ammo{};
  ASSERT_TRUE(loader.getAmmoParams("VOG-17", ammo));
  EXPECT_FLOAT_EQ(ammo.mass, 0.35f);
  EXPECT_FLOAT_EQ(ammo.drag, 0.07f);
  EXPECT_FLOAT_EQ(ammo.lift, 0.0f);
}

TEST(Homework07FileConfigLoader, RejectsMissingConfigSource)
{
  FileConfigLoader loader;
  const std::string missingDir = (fs::path(HW7_SCENARIOS_ROOT) / "does-not-exist").string();

  EXPECT_FALSE(loader.load(missingDir.c_str()));

  DroneConfig config{};
  EXPECT_FALSE(loader.getConfig(config));

  AmmoParams ammo{};
  EXPECT_FALSE(loader.getAmmoParams("VOG-17", ammo));
}

TEST(Homework07FileConfigLoader, RejectsUnknownAmmoLookup)
{
  FileConfigLoader loader;
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  ASSERT_TRUE(loader.load(baseCircleDir.c_str()));

  AmmoParams ammo{};
  EXPECT_FALSE(loader.getAmmoParams("UNKNOWN-AMMO", ammo));
}
