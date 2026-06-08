#include "ballistics.hpp"

#include <fstream>
#include <iostream>
#include <span>
#include <string>
#include <string_view>

namespace {

const char *error_message(BallisticsError error)
{
  switch (error) {
    case BallisticsError::UnknownAmmo:
      return "unknown ammo type";
    case BallisticsError::InvalidModel:
      return "ballistics model invalid for these inputs";
    case BallisticsError::NonPositiveFlightTime:
      return "non-positive time of flight";
  }

  return "ballistics error";
}

bool read_input_file(std::string_view path, BallisticsInput &input, std::string &ammo_storage)
{
  std::ifstream input_file{std::string(path)};
  if (!input_file.is_open()) {
    std::cerr << "Error: could not open input file '" << path << "'\n";
    return false;
  }

  input_file >> input.drone_x >> input.drone_y >> input.drone_z;
  input_file >> input.target_x >> input.target_y;
  input_file >> input.attack_speed >> input.acceleration_path;
  input_file >> ammo_storage;

  if (!input_file) {
    std::cerr << "Error: could not parse input file '" << path << "'\n";
    return false;
  }

  input.ammo_name = ammo_storage;
  return true;
}

bool write_output_file(std::string_view path, const DropSolution &solution)
{
  std::ofstream output_file{std::string(path)};
  if (!output_file.is_open()) {
    std::cerr << "Error: could not open output file '" << path << "'\n";
    return false;
  }

  if (solution.maneuver_needed) {
    output_file << solution.maneuver_x << ' ' << solution.maneuver_y << '\n';
  }
  output_file << solution.fire_x << ' ' << solution.fire_y << '\n';
  return static_cast<bool>(output_file);
}

}  // namespace

int main(int argc, char **argv)
{
  const std::span args(argv, static_cast<std::size_t>(argc));

  if (args.size() != 2) {
    const std::string_view program = args.empty() ? std::string_view{"ballistics_cli"} : std::string_view{args.front()};
    std::cerr << "Usage: " << program << " <input-file>\n";
    return 1;
  }

  const std::string_view input_path{args[1]};

  BallisticsInput input{};
  std::string ammo_storage;
  if (!read_input_file(input_path, input, ammo_storage)) {
    return 1;
  }

  const BallisticsResult result = compute_drop_solution(input);
  if (!result.ok) {
    std::cerr << "Error: " << error_message(result.error) << '\n';
    return 1;
  }

  if (!write_output_file("output.txt", result.solution)) {
    return 1;
  }

  return 0;
}
