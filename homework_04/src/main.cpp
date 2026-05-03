#include <cmath>
#include <fstream>
#include <iostream>
#include <numbers>

int main(int argc, char** argv) {
    // The program expects exactly one argument: a path to telemetry samples.
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }

    std::ifstream input{argv[1]};
    if (!input) {
        std::cerr << "error: failed to open input file: " << argv[1] << '\n';
        return 2;
    }

    // Robot model — fixed by the assignment so results are comparable.
    constexpr long ticks_per_revolution = 1024;
    constexpr double wheel_radius_m = 0.3;
    constexpr double wheelbase_m = 1.0;
    constexpr double distance_per_tick =
        2.0 * std::numbers::pi * wheel_radius_m / ticks_per_revolution;

    // Each row carries five whitespace-separated values:
    //   timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
    // The first row is the initial state at t = 0 and is not printed.
    long timestamp_ms = 0;
    long fl_ticks = 0, fr_ticks = 0, bl_ticks = 0, br_ticks = 0;
    if (!(input >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks)) {
        std::cerr << "error: input file is empty or malformed\n";
        return 3;
    }

    long prev_fl = fl_ticks, prev_fr = fr_ticks;
    long prev_bl = bl_ticks, prev_br = br_ticks;

    double x = 0.0, y = 0.0, theta = 0.0;

    while (input >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks) {
        // Per-wheel tick deltas since the previous row.
        const long d_fl = fl_ticks - prev_fl;
        const long d_fr = fr_ticks - prev_fr;
        const long d_bl = bl_ticks - prev_bl;
        const long d_br = br_ticks - prev_br;

        // Front and back wheel of one side are assumed to rotate in sync;
        // averaging them collapses the 4-wheel input into a 2-wheel model.
        const double d_left = (d_fl + d_bl) / 2.0;
        const double d_right = (d_fr + d_br) / 2.0;

        const double dL = d_left * distance_per_tick;
        const double dR = d_right * distance_per_tick;

        // Differential drive: linear progress of the center and heading change.
        const double d = (dL + dR) / 2.0;
        const double dtheta = (dR - dL) / wheelbase_m;

        // Midpoint integration: project along the average heading on this step.
        x += d * std::cos(theta + dtheta / 2.0);
        y += d * std::sin(theta + dtheta / 2.0);
        theta += dtheta;

        std::cout << timestamp_ms << ' ' << x << ' ' << y << ' ' << theta << '\n';

        prev_fl = fl_ticks;
        prev_fr = fr_ticks;
        prev_bl = bl_ticks;
        prev_br = br_ticks;
    }

    return 0;
}
