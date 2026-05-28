#include <cstring>

#include <gtest/gtest.h>

#include "../include/analytical_solver.hpp"

namespace {
AmmoParams makeAmmo(const char *name, float mass, float drag, float lift)
{
  AmmoParams ammo{};
  std::strncpy(ammo.name, name, sizeof(ammo.name) - 1);
  ammo.name[sizeof(ammo.name) - 1] = '\0';
  ammo.mass = mass;
  ammo.drag = drag;
  ammo.lift = lift;
  return ammo;
}
}  // namespace

TEST(Homework07AnalyticalSolver, ComputesKnownVog17DropPoint)
{
  AnalyticalSolver solver;
  const Coord dronePos{100.0f, 100.0f};
  const TargetSnapshot target{{200.0f, 200.0f}, {0.0f, 0.0f}};
  const AmmoParams ammo = makeAmmo("VOG-17", 0.35f, 0.07f, 0.0f);

  const DropSolution solution = solver.solve(dronePos, target, 100.0f, ammo, 10.0f, 10.0f);
  ASSERT_TRUE(solution.ok);
  EXPECT_NEAR(solution.dropPoint.x, 173.759f, 0.01f);
  EXPECT_NEAR(solution.dropPoint.y, 173.759f, 0.01f);
  EXPECT_GT(solution.travelTime, 0.0);
  EXPECT_GT(solution.impactTime, solution.travelTime);
}

TEST(Homework07AnalyticalSolver, ComputesM67ManeuverCase)
{
  AnalyticalSolver solver;
  const Coord dronePos{543.0f, 232.0f};
  const TargetSnapshot target{{543.0f, 232.0f}, {0.0f, 0.0f}};
  const AmmoParams ammo = makeAmmo("M67", 0.6f, 0.1f, 0.0f);

  const DropSolution solution = solver.solve(dronePos, target, 120.0f, ammo, 13.0f, 12.0f);
  ASSERT_TRUE(solution.ok);
  EXPECT_NEAR(solution.dropPoint.x, 490.496f, 0.01f);
  EXPECT_NEAR(solution.dropPoint.y, 232.0f, 0.01f);
}

TEST(Homework07AnalyticalSolver, RejectsInvalidAltitude)
{
  AnalyticalSolver solver;
  const Coord dronePos{100.0f, 100.0f};
  const TargetSnapshot target{{200.0f, 200.0f}, {0.0f, 0.0f}};
  const AmmoParams ammo = makeAmmo("VOG-17", 0.35f, 0.07f, 0.0f);

  const DropSolution solution = solver.solve(dronePos, target, 0.0f, ammo, 10.0f, 10.0f);
  EXPECT_FALSE(solution.ok);
}

TEST(Homework07AnalyticalSolver, RejectsNonPositiveAttackSpeed)
{
  AnalyticalSolver solver;
  const Coord dronePos{100.0f, 100.0f};
  const TargetSnapshot target{{200.0f, 200.0f}, {0.0f, 0.0f}};
  const AmmoParams ammo = makeAmmo("VOG-17", 0.35f, 0.07f, 0.0f);

  const DropSolution solution = solver.solve(dronePos, target, 100.0f, ammo, 0.0f, 10.0f);
  EXPECT_FALSE(solution.ok);
}
