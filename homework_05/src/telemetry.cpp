#include "telemetry.hpp"

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iostream>

// Starter had intentional runtime defects. Remaining issues include unsafe
// frame-rate math, empty logs, and stricter validation from the homework spec.

const int EXPECTED_FIELD_COUNT = 7;
const int MAX_LINE_LENGTH = 256;

int split_line(char line[], char* fields[], int max_fields) {
    int count = 0;
    char* cursor = line;

    while (*cursor != '\0' && count < max_fields) {
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
            *cursor = '\0';
            ++cursor;
        }

        if (*cursor == '\0') {
            break;
        }

        fields[count] = cursor;
        ++count;

        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' &&
               *cursor != '\r') {
            ++cursor;
        }
    }

    return count;
}

bool parse_long(const char* text, long& out, int line_number) {
    if (text == nullptr || *text == '\0') {
        std::cerr << "error: invalid frame at line " << line_number << ": empty integer field\n";
        return false;
    }

    char* end = nullptr;
    errno = 0;
    const long value = std::strtol(text, &end, 10);

    if (end == text) {
        std::cerr << "error: invalid frame at line " << line_number << ": invalid integer: " << text
                  << '\n';
        return false;
    }

    if (*end != '\0') {
        std::cerr << "error: invalid frame at line " << line_number << ": invalid integer: " << text
                  << '\n';
        return false;
    }

    if (errno == ERANGE) {
        std::cerr << "error: invalid frame at line " << line_number << ": integer out of range: "
                  << text << '\n';
        return false;
    }

    out = value;
    return true;
}

bool parse_int(const char* text, int& out, int line_number) {
    long value = 0;
    if (!parse_long(text, value, line_number)) {
        return false;
    }

    out = static_cast<int>(value);
    return true;
}

bool parse_double(const char* text, double& out, int line_number) {
    if (text == nullptr || *text == '\0') {
        std::cerr << "error: invalid frame at line " << line_number << ": empty number field\n";
        return false;
    }

    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(text, &end);

    if (end == text) {
        std::cerr << "error: invalid frame at line " << line_number << ": invalid number: " << text
                  << '\n';
        return false;
    }

    if (*end != '\0') {
        std::cerr << "error: invalid frame at line " << line_number << ": invalid number: " << text
                  << '\n';
        return false;
    }

    if (errno == ERANGE) {
        std::cerr << "error: invalid frame at line " << line_number << ": number out of range: "
                  << text << '\n';
        return false;
    }

    out = value;
    return true;
}

bool parse_frame(char line[], Frame& frame, int line_number) {
    char* fields[EXPECTED_FIELD_COUNT + 1] = {};
    const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT + 1);
    if (field_count != EXPECTED_FIELD_COUNT) {
        std::cerr << "error: line " << line_number << ": expected " << EXPECTED_FIELD_COUNT
                  << " fields, got " << field_count << '\n';
        return false;
    }

    // `&&` short-circuits: later fields are not parsed after the first failure.
    return parse_long(fields[0], frame.timestamp_ms, line_number) &&
           parse_int(fields[1], frame.seq, line_number) &&
           parse_double(fields[2], frame.voltage_v, line_number) &&
           parse_double(fields[3], frame.current_a, line_number) &&
           parse_double(fields[4], frame.temperature_c, line_number) &&
           parse_int(fields[5], frame.gps_fix, line_number) &&
           parse_int(fields[6], frame.satellites, line_number);
}

double compute_frame_rate_hz(const Frame frames[], int frame_count) {
    const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;

    return static_cast<double>((frame_count - 1) * 1000 / elapsed_ms);
}

int read_frames(const char* path, Frame frames[], int max_frames) {
    std::ifstream input{path};
    if (!input) {
        std::cerr << "error: failed to open input file: " << path << '\n';
        return 0;
    }

    int frame_count = 0;
    int line_number = 0;
    char line[MAX_LINE_LENGTH];

    while (input.getline(line, MAX_LINE_LENGTH)) {
        ++line_number;

        if (line[0] == '\0') {
            continue;
        }

        if (frame_count < max_frames) {
            Frame frame{};
            if (!parse_frame(line, frame, line_number)) {
                return -1;
            }

            frames[frame_count] = frame;
            ++frame_count;
        }
    }

    return frame_count;
}

Summary summarize(const Frame frames[], int frame_count) {
    Summary summary{};
    summary.frames_total = frame_count;
    summary.frames_valid = frame_count;
    summary.voltage_min = frames[0].voltage_v;
    summary.voltage_max = frames[0].voltage_v;
    summary.low_voltage_frames = 0;

    double temperature_sum = 0.0;

    for (int i = 0; i < frame_count; ++i) {
        if (frames[i].voltage_v < summary.voltage_min) {
            summary.voltage_min = frames[i].voltage_v;
        }

        if (frames[i].voltage_v > summary.voltage_max) {
            summary.voltage_max = frames[i].voltage_v;
        }

        temperature_sum += frames[i].temperature_c;

        if (frames[i].voltage_v < 22.0) {
            ++summary.low_voltage_frames;
        }
    }

    const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
    summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
    summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
    return summary;
}

void print_summary(const Summary& summary) {
    std::cout << "frames_total " << summary.frames_total << '\n';
    std::cout << "frames_valid " << summary.frames_valid << '\n';
    std::cout << "voltage_min " << summary.voltage_min << '\n';
    std::cout << "voltage_max " << summary.voltage_max << '\n';
    std::cout << "temperature_avg " << summary.temperature_avg << '\n';
    std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
    std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
