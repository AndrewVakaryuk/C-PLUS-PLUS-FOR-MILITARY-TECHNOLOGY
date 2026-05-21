#include "ballistics.hpp"

#include <cmath>

namespace {

constexpr double kGravity = 9.81;
constexpr double kDistanceEpsilon = 1e-9;
constexpr double kModelEpsilon = 1e-12;

struct FlightConditions {
  double attack_speed = 0.0;
  double altitude = 0.0;
};

bool solve_time_of_flight(const AmmoCoefficients &coeffs, const FlightConditions &flight, double &time_out)
{
  const double cubic_a = coeffs.drag * kGravity * coeffs.mass - 2.0 * coeffs.drag * coeffs.drag * coeffs.lift * flight.attack_speed;
  const double cubic_b = -3.0 * kGravity * coeffs.mass * coeffs.mass + 3.0 * coeffs.drag * coeffs.lift * coeffs.mass * flight.attack_speed;
  const double cubic_c = 6.0 * coeffs.mass * coeffs.mass * flight.altitude;

  if (std::abs(cubic_a) < kModelEpsilon) {
    return false;
  }

  const double depressed_p = -(cubic_b * cubic_b) / (3.0 * cubic_a * cubic_a);
  const double depressed_q = (2.0 * cubic_b * cubic_b * cubic_b) / (27.0 * cubic_a * cubic_a * cubic_a) + cubic_c / cubic_a;

  // Cardano setup requires p < 0 and non-zero for divisions and sqrt(-3/p).
  if (depressed_p >= -kModelEpsilon || std::abs(depressed_p) < kModelEpsilon) {
    return false;
  }

  const double arccos_arg = (3.0 * depressed_q / (2.0 * depressed_p)) * std::sqrt(-3.0 / depressed_p);
  if (arccos_arg < -1.0 || arccos_arg > 1.0) {
    return false;
  }

  const double phi = std::acos(arccos_arg);
  const double time = 2.0 * std::sqrt(-depressed_p / 3.0) * std::cos((phi + 4.0 * M_PI) / 3.0) - cubic_b / (3.0 * cubic_a);

  if (time <= 0.0) {
    return false;
  }

  time_out = time;
  return true;
}

double horizontal_distance(const AmmoCoefficients &coeffs, double attack_speed, double time)
{
  if (coeffs.mass < kModelEpsilon) {
    return 0.0;
  }

  const double lift_factor = coeffs.lift;
  const double lift_sq = lift_factor * lift_factor;
  const double lift_fourth = lift_sq * lift_sq;
  const double one_plus_lift_sq = 1.0 + lift_sq;

  const double term1 = attack_speed * time;
  const double term2 = -time * time * coeffs.drag * attack_speed / (2.0 * coeffs.mass);
  const double term3 =
    time * time * time *
    (6.0 * coeffs.drag * kGravity * lift_factor * coeffs.mass - 6.0 * coeffs.drag * coeffs.drag * (lift_sq - 1.0) * attack_speed) /
    (36.0 * coeffs.mass * coeffs.mass);
  const double term4_denominator = 36.0 * one_plus_lift_sq * one_plus_lift_sq * coeffs.mass * coeffs.mass * coeffs.mass;
  if (term4_denominator < kModelEpsilon) {
    return 0.0;
  }
  const double term4 = time * time * time * time *
                       (-6.0 * coeffs.drag * coeffs.drag * kGravity * lift_factor * (1.0 + lift_sq + lift_fourth) * coeffs.mass +
                        3.0 * coeffs.drag * coeffs.drag * coeffs.drag * lift_sq * one_plus_lift_sq * attack_speed +
                        6.0 * coeffs.drag * coeffs.drag * coeffs.drag * lift_fourth * one_plus_lift_sq * attack_speed) /
                       term4_denominator;
  const double term5_denominator = 36.0 * one_plus_lift_sq * coeffs.mass * coeffs.mass * coeffs.mass * coeffs.mass;
  if (term5_denominator < kModelEpsilon) {
    return 0.0;
  }
  const double term5 = time * time * time * time * time *
                       (3.0 * coeffs.drag * coeffs.drag * coeffs.drag * kGravity * lift_factor * lift_factor * lift_factor * coeffs.mass -
                        3.0 * coeffs.drag * coeffs.drag * coeffs.drag * coeffs.drag * lift_sq * one_plus_lift_sq * attack_speed) /
                       term5_denominator;

  return term1 + term2 + term3 + term4 + term5;
}

}  // namespace

bool lookup_ammo_params(std::string_view ammo_name, AmmoCoefficients &coeffs)
{
  if (ammo_name == "VOG-17") {
    coeffs.mass = 0.35;
    coeffs.drag = 0.07;
    coeffs.lift = 0.0;
    return true;
  }
  if (ammo_name == "M67") {
    coeffs.mass = 0.60;
    coeffs.drag = 0.10;
    coeffs.lift = 0.0;
    return true;
  }
  if (ammo_name == "RKG-3") {
    coeffs.mass = 1.20;
    coeffs.drag = 0.10;
    coeffs.lift = 0.0;
    return true;
  }
  if (ammo_name == "GLIDING-VOG") {
    coeffs.mass = 0.45;
    coeffs.drag = 0.10;
    coeffs.lift = 1.0;
    return true;
  }
  if (ammo_name == "GLIDING-RKG") {
    coeffs.mass = 1.40;
    coeffs.drag = 0.10;
    coeffs.lift = 1.0;
    return true;
  }
  return false;
}

BallisticsResult compute_drop_solution(const BallisticsInput &input)
{
  BallisticsResult result{};

  AmmoCoefficients coeffs{};
  if (!lookup_ammo_params(input.ammo_name, coeffs)) {
    result.error = BallisticsError::UnknownAmmo;
    return result;
  }

  double time = 0.0;
  const FlightConditions flight{.attack_speed = input.attack_speed, .altitude = input.drone_z};
  if (!solve_time_of_flight(coeffs, flight, time)) {
    result.error = BallisticsError::InvalidModel;
    return result;
  }

  const double horiz_dist = horizontal_distance(coeffs, input.attack_speed, time);

  double drone_x = input.drone_x;
  double drone_y = input.drone_y;

  double distance =
    std::sqrt((input.target_x - drone_x) * (input.target_x - drone_x) + (input.target_y - drone_y) * (input.target_y - drone_y));

  if (distance < kDistanceEpsilon) {
    result.solution.maneuver_needed = true;
    result.solution.maneuver_x = input.target_x - (horiz_dist + input.acceleration_path);
    result.solution.maneuver_y = input.target_y;

    drone_x = result.solution.maneuver_x;
    drone_y = result.solution.maneuver_y;
    distance = horiz_dist + input.acceleration_path;
  }
  else if (distance >= kDistanceEpsilon && horiz_dist + input.acceleration_path > distance) {
    const double retreat = horiz_dist + input.acceleration_path;
    result.solution.maneuver_needed = true;
    result.solution.maneuver_x = input.target_x - (input.target_x - drone_x) * retreat / distance;
    result.solution.maneuver_y = input.target_y - (input.target_y - drone_y) * retreat / distance;

    drone_x = result.solution.maneuver_x;
    drone_y = result.solution.maneuver_y;
    distance = horiz_dist + input.acceleration_path;
  }

  if (distance < kDistanceEpsilon) {
    result.error = BallisticsError::InvalidModel;
    return result;
  }

  const double ratio = (distance - horiz_dist) / distance;
  result.solution.fire_x = drone_x + (input.target_x - drone_x) * ratio;
  result.solution.fire_y = drone_y + (input.target_y - drone_y) * ratio;
  result.ok = true;
  return result;
}
