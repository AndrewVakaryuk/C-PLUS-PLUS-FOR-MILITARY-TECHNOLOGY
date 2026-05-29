#include "../include/mission_demo.hpp"

#include <fstream>
#include <iostream>

#include "../include/factories.hpp"
#include "../include/json.hpp"
#include "../include/mission_processor.hpp"

namespace {
using json = nlohmann::json;

bool writeMissionResults(const char *outputDir, const json &out)
{
  std::string path = outputDir;
  if (!path.empty() && path.back() != '/') {
    path += '/';
  }
  path += "mission_results.json";

  std::ofstream file(path);
  if (!file.is_open()) {
    std::cerr << "Error: could not open mission output file: " << path << std::endl;
    return false;
  }
  file << out.dump(2);
  return static_cast<bool>(file);
}

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

  MissionProcessor mission(loader, provider, solver);
  if (!mission.init(dataDir)) {
    std::cerr << "Error: mission init failed" << std::endl;
    deleteMissionComponents(loader, provider, solver);
    return 1;
  }

  json out;
  out["steps"] = json::array();

  int stepIndex = 0;
  while (mission.hasNext()) {
    DropSolution solution{};
    if (!mission.step(solution)) {
      std::cerr << "Error: mission step failed" << std::endl;
      deleteMissionComponents(loader, provider, solver);
      return 1;
    }

    json step;
    step["index"] = stepIndex++;
    step["ok"] = solution.ok;
    step["dropPoint"] = {{"x", solution.dropPoint.x}, {"y", solution.dropPoint.y}};
    step["travelTime"] = solution.travelTime;
    step["impactTime"] = solution.impactTime;
    out["steps"].push_back(step);
  }

  out["totalSteps"] = stepIndex;

  if (!writeMissionResults(outputDir, out)) {
    deleteMissionComponents(loader, provider, solver);
    return 1;
  }

  deleteMissionComponents(loader, provider, solver);
  return 0;
}
