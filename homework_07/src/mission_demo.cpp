#include "mission_demo.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "drone_physics.hpp"
#include "factories.hpp"
#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"
#include "mission_processor.hpp"
#include "simulation_json_writer.hpp"
#include "thread_safe_target_provider.hpp"

namespace {
constexpr auto kReadyPollInterval = std::chrono::milliseconds(1);

bool waitUntilReady(const ThreadSafeTargetProvider &provider, const DronePhysics &physics, const MissionProcessor &mission)
{
  while (!provider.isThreadReady() || !physics.isThreadReady() || !mission.isThreadReady()) {
    std::this_thread::sleep_for(kReadyPollInterval);
  }
  return true;
}
}  // namespace

int runMissionDemo(const char *dataDir, const char *outputDir)
{
  std::unique_ptr<IConfigLoader> loader = createLoader(LoaderType::FILE);
  std::unique_ptr<ITargetProvider> provider = createProvider(ProviderType::THREAD_SAFE, dataDir);
  const char *solverOverride = std::getenv("HW7_SOLVER");
  const SolverType solverType =
    (solverOverride != nullptr && std::string(solverOverride) == "analytical") ? SolverType::ANALYTICAL : SolverType::TABLE;
  std::unique_ptr<IBallisticSolver> solver = createSolver(solverType);
  auto physics = std::make_unique<DronePhysics>();

  if (loader == nullptr || provider == nullptr || solver == nullptr) {
    std::cerr << "Error: failed to create mission components" << std::endl;
    return 1;
  }

  auto *threadedProvider = dynamic_cast<ThreadSafeTargetProvider *>(provider.get());
  if (threadedProvider == nullptr) {
    std::cerr << "Error: mission demo requires ThreadSafeTargetProvider" << std::endl;
    return 1;
  }

  MissionProcessor processor(std::move(loader), std::move(provider), std::move(solver));
  processor.setPhysics(physics.get());
  if (!processor.init(dataDir)) {
    std::cerr << "Error: mission init failed" << std::endl;
    return 1;
  }

  const char *syncOverride = std::getenv("HW7_SYNC_MISSION");
  if (syncOverride != nullptr && std::string(syncOverride) == "1") {
    std::vector<SimStep> steps;
    if (!processor.run(steps)) {
      return 1;
    }
    if (!writeSimulationJson(outputDir, steps)) {
      return 1;
    }
    return 0;
  }

  std::thread providerThread(&ThreadSafeTargetProvider::run, threadedProvider);
  std::thread physicsThread(&DronePhysics::run, physics.get());
  std::thread missionThread(&MissionProcessor::runThreaded, &processor);

  if (!waitUntilReady(*threadedProvider, *physics, processor)) {
    return 1;
  }

  threadedProvider->start();
  physics->start();
  processor.start();

  missionThread.join();
  physics->stop();
  threadedProvider->stop();
  physicsThread.join();
  providerThread.join();

  if (!writeSimulationJson(outputDir, processor.steps())) {
    return 1;
  }

  return processor.succeeded() ? 0 : 1;
}
