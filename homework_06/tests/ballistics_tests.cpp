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

TEST(Ballistics, GlidingAmmoProducesFiniteDropPoint)
{
  const BallisticsInput input{
    .drone_x = 100.0,
    .drone_y = 100.0,
    .drone_z = 100.0,
    .target_x = 200.0,
    .target_y = 200.0,
    .attack_speed = 10.0,
    .acceleration_path = 10.0,
    .ammo_name = "GLIDING-VOG",
  };

  const BallisticsResult result = compute_drop_solution(input);
  ASSERT_TRUE(result.ok);
  EXPECT_GT(result.solution.fire_x, 0.0);
  EXPECT_GT(result.solution.fire_y, 0.0);
}
