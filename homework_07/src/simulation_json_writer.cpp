#include "../include/simulation_json_writer.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "json.hpp"

namespace {
using json = nlohmann::json;
}  // namespace

bool writeSimulationJson(const char *outputDir, const std::vector<SimStep> &steps)
{
  json out;
  out["totalSteps"] = steps.size();
  out["steps"] = json::array();

  for (const SimStep &step : steps) {
    json stepJson;
    stepJson["position"] = {{"x", step.pos.x}, {"y", step.pos.y}};
    stepJson["direction"] = step.direction;
    stepJson["state"] = step.state;
    stepJson["targetIndex"] = step.targetIdx;
    stepJson["dropPoint"] = {{"x", step.dropPoint.x}, {"y", step.dropPoint.y}};
    stepJson["aimPoint"] = {{"x", step.aimPoint.x}, {"y", step.aimPoint.y}};
    stepJson["predictedTarget"] = {{"x", step.predictedTarget.x}, {"y", step.predictedTarget.y}};
    out["steps"].push_back(stepJson);
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
