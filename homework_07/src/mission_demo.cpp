#include "../include/mission_demo.hpp"

#include <iostream>
#include <vector>

#include "../include/factories.hpp"
#include "../include/mission_simulator.hpp"
#include "../include/simulation_json_writer.hpp"

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

int runMissionDemo(const char *dataDir, const char *outputDir)
{
  IConfigLoader *loader = createLoader(LoaderType::FILE);
  ITargetProvider *provider = createProvider(ProviderType::JSON, dataDir);
  IBallisticSolver *solver = createSolver(SolverType::ANALYTICAL);

  if (loader == nullptr || provider == nullptr || solver == nullptr) {
    std::cerr << "Error: failed to create mission components" << std::endl;
    deleteMissionComponents(loader, provider, solver);
    return 1;
  }

  MissionSimulator simulator(loader, provider, solver);
  if (!simulator.init(dataDir)) {
    std::cerr << "Error: mission init failed" << std::endl;
    deleteMissionComponents(loader, provider, solver);
    return 1;
  }

  std::vector<SimStep> steps;
  if (!simulator.run(steps)) {
    deleteMissionComponents(loader, provider, solver);
    return 1;
  }

  if (!writeSimulationJson(outputDir, steps)) {
    deleteMissionComponents(loader, provider, solver);
    return 1;
  }

  deleteMissionComponents(loader, provider, solver);
  return 0;
}
