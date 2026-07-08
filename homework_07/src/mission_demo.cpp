#include "mission_demo.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "factories.hpp"
#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"
#include "mission_simulator.hpp"
#include "simulation_json_writer.hpp"

int runMissionDemo(const char *dataDir, const char *outputDir)
{
  std::unique_ptr<IConfigLoader> loader = createLoader(LoaderType::FILE);
  std::unique_ptr<ITargetProvider> provider = createProvider(ProviderType::JSON, dataDir);
  const char *solverOverride = std::getenv("HW7_SOLVER");
  const SolverType solverType =
    (solverOverride != nullptr && std::string(solverOverride) == "analytical") ? SolverType::ANALYTICAL : SolverType::TABLE;
  std::unique_ptr<IBallisticSolver> solver = createSolver(solverType);

  if (loader == nullptr || provider == nullptr || solver == nullptr) {
    std::cerr << "Error: failed to create mission components" << std::endl;
    return 1;
  }

  MissionSimulator simulator(std::move(loader), std::move(provider), std::move(solver));
  if (!simulator.init(dataDir)) {
    std::cerr << "Error: mission init failed" << std::endl;
    return 1;
  }

  std::vector<SimStep> steps;
  if (!simulator.run(steps)) {
    return 1;
  }

  if (!writeSimulationJson(outputDir, steps)) {
    return 1;
  }

  return 0;
}
