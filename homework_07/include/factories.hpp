#ifndef HOMEWORK_07_FACTORIES_HPP
#define HOMEWORK_07_FACTORIES_HPP

#include "interfaces/i_ballistic_solver.hpp"
#include "interfaces/i_config_loader.hpp"
#include "interfaces/i_target_provider.hpp"

enum class SolverType {
  ANALYTICAL
};

enum class ProviderType {
  JSON
};

enum class LoaderType {
  FILE
};

IBallisticSolver *createSolver(SolverType type);
ITargetProvider *createProvider(ProviderType type, const char *param);
IConfigLoader *createLoader(LoaderType type);

#endif
