#include <filesystem>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "analytical_solver.hpp"
#include "factories.hpp"
#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"
#include "mission_processor.hpp"

namespace fs = std::filesystem;

TEST(Homework07MissionProcessor, ProcessesAllTargetsViaFactoryComponents)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();

  std::unique_ptr<IConfigLoader> loader = createLoader(LoaderType::FILE);
  std::unique_ptr<ITargetProvider> provider = createProvider(ProviderType::JSON, baseCircleDir.c_str());
  std::unique_ptr<IBallisticSolver> solver = createSolver(SolverType::ANALYTICAL);
  ASSERT_NE(loader, nullptr);
  ASSERT_NE(provider, nullptr);
  ASSERT_NE(solver, nullptr);

  MissionProcessor mission(std::move(loader), std::move(provider), std::move(solver));
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

}

TEST(Homework07MissionProcessor, ResetAllowsSecondPass)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();

  std::unique_ptr<IConfigLoader> loader = createLoader(LoaderType::FILE);
  std::unique_ptr<ITargetProvider> provider = createProvider(ProviderType::JSON, baseCircleDir.c_str());
  std::unique_ptr<IBallisticSolver> solver = createSolver(SolverType::ANALYTICAL);
  ASSERT_NE(loader, nullptr);
  ASSERT_NE(provider, nullptr);
  ASSERT_NE(solver, nullptr);

  MissionProcessor mission(std::move(loader), std::move(provider), std::move(solver));
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

}

TEST(Homework07MissionProcessor, ChangeSolverAffectsResults)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();

  std::unique_ptr<IConfigLoader> loader = createLoader(LoaderType::FILE);
  std::unique_ptr<ITargetProvider> provider = createProvider(ProviderType::JSON, baseCircleDir.c_str());
  std::unique_ptr<IBallisticSolver> solverA = createSolver(SolverType::ANALYTICAL);
  std::unique_ptr<IBallisticSolver> solverB = createSolver(SolverType::ANALYTICAL);
  ASSERT_NE(loader, nullptr);
  ASSERT_NE(provider, nullptr);
  ASSERT_NE(solverA, nullptr);
  ASSERT_NE(solverB, nullptr);

  MissionProcessor mission(std::move(loader), std::move(provider), std::move(solverA));
  ASSERT_TRUE(mission.init(baseCircleDir.c_str()));

  DropSolution first{};
  ASSERT_TRUE(mission.step(first));
  ASSERT_TRUE(first.ok);

  mission.changeSolver(std::move(solverB));
  mission.reset();

  DropSolution second{};
  ASSERT_TRUE(mission.step(second));
  ASSERT_TRUE(second.ok);
  EXPECT_NEAR(first.dropPoint.x, second.dropPoint.x, 1e-3f);
  EXPECT_NEAR(first.dropPoint.y, second.dropPoint.y, 1e-3f);

}
