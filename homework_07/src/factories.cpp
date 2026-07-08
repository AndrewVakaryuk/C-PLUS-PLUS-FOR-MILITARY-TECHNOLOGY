#include "factories.hpp"

#include <memory>

#include "analytical_solver.hpp"
#include "file_config_loader.hpp"
#include "json_target_provider.hpp"
#include "table_solver.hpp"

std::unique_ptr<IBallisticSolver> createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return std::make_unique<AnalyticalSolver>();
    case SolverType::TABLE:
      return std::make_unique<TableSolver>();
    default:
      return nullptr;
  }
}

std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const char *param)
{
  switch (type) {
    case ProviderType::JSON:
      if (param == nullptr) {
        return nullptr;
      }
      return std::make_unique<JsonTargetProvider>(param);
    default:
      return nullptr;
  }
}

std::unique_ptr<IConfigLoader> createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::FILE:
      return std::make_unique<FileConfigLoader>();
    default:
      return nullptr;
  }
}
