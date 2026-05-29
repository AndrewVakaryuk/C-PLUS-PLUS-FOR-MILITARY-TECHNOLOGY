#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "../include/analytical_solver.hpp"
#include "../include/factories.hpp"
#include "../include/mission_processor.hpp"

namespace fs = std::filesystem;

namespace {
void deleteMissionComponents(IConfigLoader *&loader, ITargetProvider *&provider, IBallisticSolver *&solver)
{
  delete loader;
  delete provider;
  delete solver;
  loader = nullptr;
  provider = nullptr;
  solver = nullptr;
}
}  // namespace

TEST(Homework07MissionProcessor, ProcessesAllTargetsViaFactoryComponents)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();

  IConfigLoader *loader = createLoader(LoaderType::FILE);
  ITargetProvider *provider = createProvider(ProviderType::JSON, baseCircleDir.c_str());
  IBallisticSolver *solver = createSolver(SolverType::ANALYTICAL);
  ASSERT_NE(loader, nullptr);
  ASSERT_NE(provider, nullptr);
  ASSERT_NE(solver, nullptr);

  MissionProcessor mission(loader, provider, solver);
  ASSERT_TRUE(mission.init(baseCircleDir.c_str()));

  int processed = 0;
  int successful = 0;
  while (mission.hasNext()) {
    DropSolution stepResult{};
    ASSERT_TRUE(mission.step(stepResult));
    processed++;
    if (stepResult.ok) {
      successful++;
    }
  }

  EXPECT_EQ(processed, 5);
  EXPECT_EQ(successful, 5);

  deleteMissionComponents(loader, provider, solver);
}

TEST(Homework07MissionProcessor, ResetAllowsSecondPass)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();

  IConfigLoader *loader = createLoader(LoaderType::FILE);
  ITargetProvider *provider = createProvider(ProviderType::JSON, baseCircleDir.c_str());
  IBallisticSolver *solver = createSolver(SolverType::ANALYTICAL);
  ASSERT_NE(loader, nullptr);
  ASSERT_NE(provider, nullptr);
  ASSERT_NE(solver, nullptr);

  MissionProcessor mission(loader, provider, solver);
  ASSERT_TRUE(mission.init(baseCircleDir.c_str()));

  int firstPass = 0;
  while (mission.hasNext()) {
    DropSolution stepResult{};
    ASSERT_TRUE(mission.step(stepResult));
    firstPass++;
  }
  EXPECT_EQ(firstPass, 5);

  mission.reset();
  EXPECT_TRUE(mission.hasNext());

  int secondPass = 0;
  while (mission.hasNext()) {
    DropSolution stepResult{};
    ASSERT_TRUE(mission.step(stepResult));
    secondPass++;
  }
  EXPECT_EQ(secondPass, 5);

  deleteMissionComponents(loader, provider, solver);
}

TEST(Homework07MissionProcessor, ChangeSolverAffectsResults)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();

  IConfigLoader *loader = createLoader(LoaderType::FILE);
  ITargetProvider *provider = createProvider(ProviderType::JSON, baseCircleDir.c_str());
  IBallisticSolver *solverA = createSolver(SolverType::ANALYTICAL);
  IBallisticSolver *solverB = createSolver(SolverType::ANALYTICAL);
  ASSERT_NE(loader, nullptr);
  ASSERT_NE(provider, nullptr);
  ASSERT_NE(solverA, nullptr);
  ASSERT_NE(solverB, nullptr);

  MissionProcessor mission(loader, provider, solverA);
  ASSERT_TRUE(mission.init(baseCircleDir.c_str()));

  DropSolution first{};
  ASSERT_TRUE(mission.step(first));
  ASSERT_TRUE(first.ok);

  mission.changeSolver(solverB);
  mission.reset();

  DropSolution second{};
  ASSERT_TRUE(mission.step(second));
  ASSERT_TRUE(second.ok);
  EXPECT_NEAR(first.dropPoint.x, second.dropPoint.x, 1e-3f);
  EXPECT_NEAR(first.dropPoint.y, second.dropPoint.y, 1e-3f);

  delete loader;
  delete provider;
  delete solverA;
  delete solverB;
}
