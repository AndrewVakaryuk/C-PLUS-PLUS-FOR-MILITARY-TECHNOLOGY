#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "analytical_solver.hpp"
#include "json_target_provider.hpp"

namespace {
AmmoParams makeAmmo(const std::string &name, float mass, float drag, float lift)
{
  AmmoParams ammo{};
  ammo.name = name;
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

TEST(Homework07AnalyticalSolver, SolveLeadForBaseCircleAtMissionStart)
{
  const std::string baseCircleDir = (std::filesystem::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  JsonTargetProvider provider(baseCircleDir.c_str());
  AnalyticalSolver solver;
  const AmmoParams ammo = makeAmmo("VOG-17", 0.35f, 0.07f, 0.0f);
  const Coord dronePos{150.0f, 150.0f};

  DropSolution lead{};
  ASSERT_TRUE(solver.solveLead(dronePos, 0.0, 2, provider, 100.0f, ammo, 10.0f, 10.0f, lead));
  EXPECT_NEAR(lead.dropPoint.x, 211.017f, 0.05f);
  EXPECT_NEAR(lead.dropPoint.y, 222.600f, 0.05f);
  EXPECT_GT(lead.travelTime, 0.0);
  EXPECT_GT(lead.impactTime, lead.travelTime);
}

TEST(Homework07AnalyticalSolver, SolveLeadFailsForInvalidTargetIndex)
{
  const std::string baseCircleDir = (std::filesystem::path(HW7_SCENARIOS_ROOT) / "base-circle").string();
  JsonTargetProvider provider(baseCircleDir.c_str());
  AnalyticalSolver solver;
  const AmmoParams ammo = makeAmmo("VOG-17", 0.35f, 0.07f, 0.0f);

  DropSolution lead{};
  EXPECT_FALSE(solver.solveLead({150.0f, 150.0f}, 0.0, -1, provider, 100.0f, ammo, 10.0f, 10.0f, lead));
  EXPECT_FALSE(solver.solveLead({150.0f, 150.0f}, 0.0, provider.getTargetCount(), provider, 100.0f, ammo, 10.0f, 10.0f, lead));
}
