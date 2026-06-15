#include "../include/simulation_json_writer.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "json.hpp"

namespace {
using json = nlohmann::json;
}  // namespace

bool writeSimulationJson(const char *outputDir, const SimStep steps[], int stepCount)
{
  json out;
  out["totalSteps"] = stepCount;
  out["steps"] = json::array();

  for (int i = 0; i < stepCount; i++) {
    json step;
    step["position"] = {{"x", steps[i].pos.x}, {"y", steps[i].pos.y}};
    step["direction"] = steps[i].direction;
    step["state"] = steps[i].state;
    step["targetIndex"] = steps[i].targetIdx;
    step["dropPoint"] = {{"x", steps[i].dropPoint.x}, {"y", steps[i].dropPoint.y}};
    step["aimPoint"] = {{"x", steps[i].aimPoint.x}, {"y", steps[i].aimPoint.y}};
    step["predictedTarget"] = {{"x", steps[i].predictedTarget.x}, {"y", steps[i].predictedTarget.y}};
    out["steps"].push_back(step);
  }

  std::string path = outputDir;
  if (!path.empty() && path.back() != '/') {
    path += '/';
  }
  path += "simulation.json";

  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(outputDir), ec);

  std::ofstream file(path);
  if (!file.is_open()) {
    std::cerr << "Error: could not open simulation JSON output file: " << path << std::endl;
    return false;
  }
  file << out.dump(2);
  return static_cast<bool>(file);
}
