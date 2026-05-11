#include <cmath>
#include <fstream>
#include <iostream>
#include <numbers>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }

    std::ifstream input{argv[1]};
    if (!input) {
        std::cerr << "error: failed to open input file: " << argv[1] << '\n';
        return 2;
    }

    constexpr long kTicksPerRevolution = 1024;
    constexpr double kWheelRadiusM = 0.3;
    constexpr double kWheelbaseM = 1.0;
    constexpr double kDistancePerTick =
        2.0 * std::numbers::pi * kWheelRadiusM / kTicksPerRevolution;

    long timestamp_ms = 0;
    long fl_ticks = 0;
    long fr_ticks = 0;
    long bl_ticks = 0;
    long br_ticks = 0;
    if (!(input >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks)) {
        std::cerr << "error: input file is empty or malformed\n";
        return 3;
    }

    long prev_fl = fl_ticks;
    long prev_fr = fr_ticks;
    long prev_bl = bl_ticks;
    long prev_br = br_ticks;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    while (input >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks) {
        long d_fl = fl_ticks - prev_fl;
        long d_fr = fr_ticks - prev_fr;
        long d_bl = bl_ticks - prev_bl;
        long d_br = br_ticks - prev_br;

        double d_left_m = static_cast<double>(d_fl + d_bl) / 2.0 * kDistancePerTick;
        double d_right_m = static_cast<double>(d_fr + d_br) / 2.0 * kDistancePerTick;

        // Differential drive: linear progress of the center and heading change.
        double d = (d_left_m + d_right_m) / 2.0;
        double dtheta = (d_right_m - d_left_m) / kWheelbaseM;

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
