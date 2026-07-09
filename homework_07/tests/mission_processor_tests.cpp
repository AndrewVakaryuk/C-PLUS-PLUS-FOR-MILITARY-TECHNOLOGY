#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "domain_types.hpp"
#include "drone_physics.hpp"
#include "factories.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"
#include "mission_processor.hpp"

namespace fs = std::filesystem;

TEST(Homework10DronePhysics, IntegratesWithFixedSimulationTime)
{
  DroneConfig config{};
  config.startPos = {150.0f, 150.0f};
  config.initialDir = 0.0f;
  config.attackSpeed = 10.0f;
  config.accelPath = 10.0f;
  config.angularSpeed = 1.0f;
  config.turnThreshold = 0.1f;
  config.simTimeStep = 0.1f;
  config.physicsTimeStep = 0.01f;
  config.timeScale = 10.0f;

  DronePhysics physics;
  physics.init(config);
  physics.pushCommand(DroneCommand{0.0f});

  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(physics.stepOnce());
  }

  const DroneTelemetry telemetry = physics.getTelemetry();
  EXPECT_NEAR(telemetry.timeSecSinceStart, 1.0f, 1e-4f);
  EXPECT_GT(telemetry.speed, 0.0f);
}

TEST(Homework10MissionProcessor, CompletesBaseCircleSynchronously)
{
  const std::string baseCircleDir = (fs::path(HW7_SCENARIOS_ROOT) / "base-circle").string();

  std::unique_ptr<IConfigLoader> loader = createLoader(LoaderType::FILE);
  std::unique_ptr<ITargetProvider> provider = createProvider(ProviderType::THREAD_SAFE, baseCircleDir.c_str());
  std::unique_ptr<IBallisticSolver> solver = createSolver(SolverType::TABLE);
  auto physics = std::make_unique<DronePhysics>();
  ASSERT_NE(loader, nullptr);
  ASSERT_NE(provider, nullptr);
  ASSERT_NE(solver, nullptr);

  MissionProcessor mission(std::move(loader), std::move(provider), std::move(solver));
  mission.setPhysics(physics.get());
  ASSERT_TRUE(mission.init(baseCircleDir.c_str()));

  std::vector<SimStep> steps;
  ASSERT_TRUE(mission.run(steps));
  EXPECT_GT(steps.size(), 10U);
  EXPECT_GT(steps.back().timeSecSinceStart, 0.0f);
}
