#include <iostream>

int main(int argc, char** argv) {
    // Програма очікує рівно один аргумент: шлях до файлу з telemetry samples.
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }

    // TODO: реалізувати wheel odometry для 4-колісної differential-drive UGV.
    //
    // Параметри моделі:
    //   ticks_per_revolution = 1024
    //   wheel_radius_m       = 0.3
    //   wheelbase_m          = 1.0
    //
    // Вхід: текстовий файл з 5 числами в кожному рядку:
    //         timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
    // Вихід: табличний формат у stdout, починаючи з другого sample:
    //         timestamp_ms x y theta

    return 0;
}
