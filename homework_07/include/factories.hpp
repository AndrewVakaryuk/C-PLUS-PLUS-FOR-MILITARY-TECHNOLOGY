#pragma once

#include <memory>

class IBallisticSolver;
class IConfigLoader;
class ITargetProvider;

enum class SolverType { ANALYTICAL, TABLE };

enum class ProviderType { JSON, THREAD_SAFE };

enum class LoaderType { FILE };

std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const char *param);
std::unique_ptr<IConfigLoader> createLoader(LoaderType type);
