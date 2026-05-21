#pragma once

#include <cstdint>
#include <string_view>

struct BallisticsInput {
  double drone_x = 0.0;
  double drone_y = 0.0;
  double drone_z = 0.0;
  double target_x = 0.0;
  double target_y = 0.0;
  double attack_speed = 0.0;
  double acceleration_path = 0.0;
  std::string_view ammo_name;
};

struct DropSolution {
  double fire_x = 0.0;
  double fire_y = 0.0;
  bool maneuver_needed = false;
  double maneuver_x = 0.0;
  double maneuver_y = 0.0;
};

struct AmmoCoefficients {
  double mass = 0.0;
  double drag = 0.0;
  double lift = 0.0;
};

enum class BallisticsError : std::uint8_t {
  UnknownAmmo,
  InvalidModel,
  NonPositiveFlightTime,
};

struct BallisticsResult {
  bool ok = false;
  BallisticsError error = BallisticsError::UnknownAmmo;
  DropSolution solution{};
};

bool lookup_ammo_params(std::string_view ammo_name, AmmoCoefficients &coeffs);

BallisticsResult compute_drop_solution(const BallisticsInput &input);
