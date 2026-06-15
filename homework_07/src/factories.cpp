#include "factories.hpp"

#include "analytical_solver.hpp"
#include "file_config_loader.hpp"
#include "json_target_provider.hpp"

IBallisticSolver *createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return new AnalyticalSolver();
    default:
      return nullptr;
  }
}

ITargetProvider *createProvider(ProviderType type, const char *param)
{
  switch (type) {
    case ProviderType::JSON:
      if (param == nullptr) {
        return nullptr;
      }
      return new JsonTargetProvider(param);
    default:
      return nullptr;
  }
}

IConfigLoader *createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::FILE:
      return new FileConfigLoader();
    default:
      return nullptr;
  }
}
