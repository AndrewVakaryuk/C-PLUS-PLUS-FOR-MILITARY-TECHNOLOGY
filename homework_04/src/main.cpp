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
        //TODO: apply logic
        std::cout << timestamp_ms << " " << fl_ticks << " " << fr_ticks << " " << bl_ticks << " " << br_ticks << '\n';
    }

    return 0;
}
