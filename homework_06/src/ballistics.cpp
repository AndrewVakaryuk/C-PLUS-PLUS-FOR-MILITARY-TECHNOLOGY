#include "ballistics.hpp"

#include <cmath>

namespace {

constexpr double kGravity = 9.81;
constexpr double kDistanceEpsilon = 1e-9;

bool solve_time_of_flight(double mass, double drag, double lift, double attack_speed, double altitude,
                          double& time_out) {
    const double a = drag * kGravity * mass - 2.0 * drag * drag * lift * attack_speed;
    const double b = -3.0 * kGravity * mass * mass + 3.0 * drag * lift * mass * attack_speed;
    const double c = 6.0 * mass * mass * altitude;

    const double p = -(b * b) / (3.0 * a * a);
    const double q = (2.0 * b * b * b) / (27.0 * a * a * a) + c / a;

    const double arccos_arg = (3.0 * q / (2.0 * p)) * std::sqrt(-3.0 / p);
    if (arccos_arg < -1.0 || arccos_arg > 1.0) {
        return false;
    }

    const double phi = std::acos(arccos_arg);
    const double time =
        2.0 * std::sqrt(-p / 3.0) * std::cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a);

    if (time <= 0.0) {
        return false;
    }

    time_out = time;
    return true;
}

double horizontal_distance(double mass, double drag, double lift, double attack_speed, double time) {
    const double l = lift;
    const double l2 = l * l;
    const double l4 = l2 * l2;
    const double one_plus_l2 = 1.0 + l2;

    const double term1 = attack_speed * time;
    const double term2 = -time * time * drag * attack_speed / (2.0 * mass);
    const double term3 =
        time * time * time * (6.0 * drag * kGravity * l * mass - 6.0 * drag * drag * (l2 - 1.0) * attack_speed) /
        (36.0 * mass * mass);
    const double term4 = time * time * time * time *
                         (-6.0 * drag * drag * kGravity * l * (1.0 + l2 + l4) * mass +
                          3.0 * drag * drag * drag * l2 * one_plus_l2 * attack_speed +
                          6.0 * drag * drag * drag * l4 * one_plus_l2 * attack_speed) /
                         (36.0 * one_plus_l2 * one_plus_l2 * mass * mass * mass);
    const double term5 = time * time * time * time * time *
                         (3.0 * drag * drag * drag * kGravity * l * l * l * mass -
                          3.0 * drag * drag * drag * drag * l2 * one_plus_l2 * attack_speed) /
                         (36.0 * one_plus_l2 * mass * mass * mass * mass);

    return term1 + term2 + term3 + term4 + term5;
}

}  // namespace

bool lookup_ammo_params(std::string_view ammo_name, double& mass, double& drag, double& lift) {
    if (ammo_name == "VOG-17") {
        mass = 0.35;
        drag = 0.07;
        lift = 0.0;
        return true;
    }
    if (ammo_name == "M67") {
        mass = 0.60;
        drag = 0.10;
        lift = 0.0;
        return true;
    }
    if (ammo_name == "RKG-3") {
        mass = 1.20;
        drag = 0.10;
        lift = 0.0;
        return true;
    }
    if (ammo_name == "GLIDING-VOG") {
        mass = 0.45;
        drag = 0.10;
        lift = 1.0;
        return true;
    }
    if (ammo_name == "GLIDING-RKG") {
        mass = 1.40;
        drag = 0.10;
        lift = 1.0;
        return true;
    }
    return false;
}

BallisticsResult compute_drop_solution(const BallisticsInput& input) {
    BallisticsResult result{};

    double mass = 0.0;
    double drag = 0.0;
    double lift = 0.0;
    if (!lookup_ammo_params(input.ammo_name, mass, drag, lift)) {
        result.error = BallisticsError::UnknownAmmo;
        return result;
    }

    double time = 0.0;
    if (!solve_time_of_flight(mass, drag, lift, input.attack_speed, input.drone_z, time)) {
        result.error = BallisticsError::InvalidModel;
        return result;
    }

    const double h = horizontal_distance(mass, drag, lift, input.attack_speed, time);

    double drone_x = input.drone_x;
    double drone_y = input.drone_y;

    double distance =
        std::sqrt((input.target_x - drone_x) * (input.target_x - drone_x) +
                  (input.target_y - drone_y) * (input.target_y - drone_y));

    if (distance < kDistanceEpsilon) {
        result.solution.maneuver_needed = true;
        result.solution.maneuver_x = input.target_x - (h + input.acceleration_path);
        result.solution.maneuver_y = input.target_y;

        drone_x = result.solution.maneuver_x;
        drone_y = result.solution.maneuver_y;
        distance = h + input.acceleration_path;
    } else if (h + input.acceleration_path > distance) {
        result.solution.maneuver_needed = true;
        result.solution.maneuver_x =
            input.target_x - (input.target_x - drone_x) * (h + input.acceleration_path) / distance;
        result.solution.maneuver_y =
            input.target_y - (input.target_y - drone_y) * (h + input.acceleration_path) / distance;

        drone_x = result.solution.maneuver_x;
        drone_y = result.solution.maneuver_y;
        distance = h + input.acceleration_path;
    }

    const double ratio = (distance - h) / distance;
    result.solution.fire_x = drone_x + (input.target_x - drone_x) * ratio;
    result.solution.fire_y = drone_y + (input.target_y - drone_y) * ratio;
    result.ok = true;
    return result;
}
