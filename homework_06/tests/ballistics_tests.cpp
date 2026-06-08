#include "ballistics.hpp"

#include <gtest/gtest.h>

TEST(Ballistics, ComputesKnownDropPoint)
{
  const BallisticsInput input{
    .drone_x = 100.0,
    .drone_y = 100.0,
    .drone_z = 100.0,
    .target_x = 200.0,
    .target_y = 200.0,
    .attack_speed = 10.0,
    .acceleration_path = 10.0,
    .ammo_name = "VOG-17",
  };

  const BallisticsResult result = compute_drop_solution(input);
  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.solution.fire_x, 173.759, 0.01);
  EXPECT_NEAR(result.solution.fire_y, 173.759, 0.01);
  EXPECT_FALSE(result.solution.maneuver_needed);
}

TEST(Ballistics, RejectsUnknownAmmo)
{
  const BallisticsInput input{
    .drone_x = 100.0,
    .drone_y = 100.0,
    .drone_z = 100.0,
    .target_x = 200.0,
    .target_y = 200.0,
    .attack_speed = 10.0,
    .acceleration_path = 10.0,
    .ammo_name = "UNKNOWN-AMMO",
  };

  const BallisticsResult result = compute_drop_solution(input);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, BallisticsError::UnknownAmmo);
}

TEST(Ballistics, SampleM67ManeuverAboveTarget)
{
  // homework_06/data/sample_m67.txt — drone vertically above target
  const BallisticsInput input{
    .drone_x = 543.0,
    .drone_y = 232.0,
    .drone_z = 120.0,
    .target_x = 543.0,
    .target_y = 232.0,
    .attack_speed = 13.0,
    .acceleration_path = 12.0,
    .ammo_name = "M67",
  };

  const BallisticsResult result = compute_drop_solution(input);
  ASSERT_TRUE(result.ok);
  EXPECT_TRUE(result.solution.maneuver_needed);
  EXPECT_NEAR(result.solution.maneuver_x, 478.496, 0.01);
  EXPECT_NEAR(result.solution.maneuver_y, 232.0, 0.01);
  EXPECT_NEAR(result.solution.fire_x, 490.496, 0.01);
  EXPECT_NEAR(result.solution.fire_y, 232.0, 0.01);
}

TEST(Ballistics, SampleRkg3ManeuverWithOffset)
{
  // homework_06/data/sample_rkg-3.txt — short offset, still needs maneuver
  const BallisticsInput input{
    .drone_x = 543.0,
    .drone_y = 232.0,
    .drone_z = 120.0,
    .target_x = 553.0,
    .target_y = 242.0,
    .attack_speed = 13.0,
    .acceleration_path = 12.0,
    .ammo_name = "RKG-3",
  };

  const BallisticsResult result = compute_drop_solution(input);
  ASSERT_TRUE(result.ok);
  EXPECT_TRUE(result.solution.maneuver_needed);
  EXPECT_NEAR(result.solution.maneuver_x, 504.6, 0.01);
  EXPECT_NEAR(result.solution.maneuver_y, 193.6, 0.01);
  EXPECT_NEAR(result.solution.fire_x, 513.085, 0.01);
  EXPECT_NEAR(result.solution.fire_y, 202.085, 0.01);
}

TEST(Ballistics, SampleGlidingVogLongRange)
{
  // homework_06/data/sample_gliding-vog.txt
  const BallisticsInput input{
    .drone_x = 0.0,
    .drone_y = 0.0,
    .drone_z = 100.0,
    .target_x = 300.0,
    .target_y = 300.0,
    .attack_speed = 20.0,
    .acceleration_path = 50.0,
    .ammo_name = "GLIDING-VOG",
  };

  const BallisticsResult result = compute_drop_solution(input);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.solution.maneuver_needed);
  EXPECT_NEAR(result.solution.fire_x, 242.711, 0.01);
  EXPECT_NEAR(result.solution.fire_y, 242.711, 0.01);
}

TEST(Ballistics, SampleGlidingRkgLongRange)
{
  // homework_06/data/sample_gliding-rkg.txt
  const BallisticsInput input{
    .drone_x = 543.0,
    .drone_y = 232.0,
    .drone_z = 120.0,
    .target_x = 1034.0,
    .target_y = 432.0,
    .attack_speed = 13.0,
    .acceleration_path = 12.0,
    .ammo_name = "GLIDING-RKG",
  };

  const BallisticsResult result = compute_drop_solution(input);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.solution.maneuver_needed);
  EXPECT_NEAR(result.solution.fire_x, 966.534, 0.01);
  EXPECT_NEAR(result.solution.fire_y, 404.519, 0.01);
}
