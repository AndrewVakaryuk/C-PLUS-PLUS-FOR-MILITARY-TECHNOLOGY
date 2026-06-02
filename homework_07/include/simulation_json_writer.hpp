#pragma once

#include <vector>

#include "domain_types.hpp"

bool writeSimulationJson(const char *outputDir, const std::vector<SimStep> &steps);
