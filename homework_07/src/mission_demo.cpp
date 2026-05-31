#include "../include/mission_demo.hpp"

#include <iostream>

#include "../include/factories.hpp"
#include "../include/mission_simulator.hpp"
#include "../include/simulation_json_writer.hpp"

namespace {
constexpr int kMaxSimulationRecords = 10002;

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

  SimStep *steps = new SimStep[kMaxSimulationRecords];
  int recordCount = 0;
  if (!simulator.run(steps, kMaxSimulationRecords, recordCount)) {
    delete[] steps;
    deleteMissionComponents(loader, provider, solver);
    return 1;
  }

  if (!writeSimulationJson(outputDir, steps, recordCount)) {
    delete[] steps;
    deleteMissionComponents(loader, provider, solver);
    return 1;
  }

  delete[] steps;
  deleteMissionComponents(loader, provider, solver);
  return 0;
}
