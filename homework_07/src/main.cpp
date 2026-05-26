#include <iostream>
#include <fstream>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <string>
#include "json.hpp"

using json = nlohmann::json;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef ENABLE_LOG
#define ENABLE_LOG 1
#endif

#ifndef ENABLE_DEBUG
#define ENABLE_DEBUG 0
#endif

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

struct Coord {
    float x;
    float y;

    Coord operator+(const Coord &other) const {
        return {x + other.x, y + other.y};
    }

    Coord operator-(const Coord &other) const {
        return {x - other.x, y - other.y};
    }

    Coord operator*(float s) const {
        return {x * s, y * s};
    }

    Coord operator/(float s) const {
        return {x / s, y / s};
    }

    bool operator==(const Coord &other) const {
        constexpr float kCoordEps = 1e-6f;
        return std::fabs(x - other.x) <= kCoordEps
            && std::fabs(y - other.y) <= kCoordEps;
    }
};

struct AmmoParams {
    char name[32];
    float mass;
    float drag;
    float lift;
};

struct DroneConfig {
    Coord startPos;
    float altitude;
    float initialDir;
    float attackSpeed;
    float accelPath;
    char ammoName[32];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

struct SimStep {
    Coord pos;
    float direction;
    int state;
    int targetIdx;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};

float length(Coord c) {
    return static_cast<float>(std::hypot(c.x, c.y));
}

Coord normalize(Coord c) {
    const float len = length(c);
    if (len <= 1e-9f) {
        return {0.0f, 0.0f};
    }
    return c / len;
}

constexpr int    MAX_STEPS = 10000;
constexpr int    MAX_RECORDS = MAX_STEPS + 2;
constexpr double EPS = 1e-9;
constexpr int    DEBUG_STEPS_LIMIT = 120;

#ifndef HW7_DATA_DIR
#define HW7_DATA_DIR "."
#endif

#ifndef HW7_OUTPUT_DIR
#define HW7_OUTPUT_DIR "."
#endif

std::string joinPath(const char *directory, const char *fileName) {
    std::string path = directory;
    if (!path.empty() && path.back() != '/') {
        path += '/';
    }
    path += fileName;
    return path;
}

bool readConfigJson(DroneConfig &config) {
    const std::string configPath = joinPath(HW7_DATA_DIR, "config.json");
    std::ifstream inputFile(configPath);
    if (!inputFile.is_open()) {
        std::cerr << "Error: could not open " << configPath << std::endl;
        return false;
    }

    json j;
    try {
        inputFile >> j;
        config.startPos.x = j.at("drone").at("position").at("x").get<float>();
        config.startPos.y = j.at("drone").at("position").at("y").get<float>();
        config.altitude = j.at("drone").at("altitude").get<float>();
        config.initialDir = j.at("drone").at("initialDirection").get<float>();
        config.attackSpeed = j.at("drone").at("attackSpeed").get<float>();
        config.accelPath = j.at("drone").at("accelerationPath").get<float>();
        config.angularSpeed = j.at("drone").at("angularSpeed").get<float>();
        config.turnThreshold = j.at("drone").at("turnThreshold").get<float>();
        config.simTimeStep = j.at("simulation").at("timeStep").get<float>();
        config.hitRadius = j.at("simulation").at("hitRadius").get<float>();
        config.arrayTimeStep = j.at("targetArrayTimeStep").get<float>();

        const auto &ammoJson = j.at("ammo").get_ref<const json::string_t&>();
        std::strncpy(config.ammoName, ammoJson.c_str(), sizeof(config.ammoName) - 1);
        config.ammoName[sizeof(config.ammoName) - 1] = '\0';
    } catch (const std::exception &e) {
        std::cerr << "Error: invalid " << configPath << ": " << e.what() << std::endl;
        return false;
    }

    return true;
}

bool readInput(double &xd, double &yd, double &zd,
               float &initialDir,
               double &attackSpeed, double &accelerationPath,
               char (&ammo_name)[50],
               float &arrayTimeStep, float &simTimeStep,
               float &hitRadius, float &angularSpeed, float &turnThreshold) {
    std::ifstream inputFile("input.txt");
    if (!inputFile.is_open()) {
        std::cerr << "Error: could not open input.txt" << std::endl;
        return false;
    }
    inputFile >> xd >> yd >> zd;
    inputFile >> initialDir;
    inputFile >> attackSpeed >> accelerationPath;
    inputFile >> ammo_name;
    inputFile >> arrayTimeStep >> simTimeStep;
    inputFile >> hitRadius >> angularSpeed >> turnThreshold;
    if (inputFile.fail()) {
        std::cerr << "Error: invalid input.txt contents" << std::endl;
        return false;
    }
    inputFile.close();
    return true;
}

bool readTargetsJson(Coord **&targets, int &targetCount, int &timeSteps) {
    const std::string targetsPath = joinPath(HW7_DATA_DIR, "targets.json");
    std::ifstream targetFile(targetsPath);
    if (!targetFile.is_open()) {
        std::cerr << "Error: could not open " << targetsPath << std::endl;
        return false;
    }

    json j;
    int allocatedRows = 0;
    try {
        targetFile >> j;
        targetCount = j.at("targetCount").get<int>();
        timeSteps = j.at("timeSteps").get<int>();
        if (targetCount <= 0 || timeSteps <= 0) {
            std::cerr << "Error: invalid targetCount/timeSteps in targets.json" << std::endl;
            return false;
        }

        targets = new Coord*[targetCount];
        for (int i = 0; i < targetCount; i++) {
            targets[i] = nullptr;
        }
        for (int i = 0; i < targetCount; i++) {
            targets[i] = new Coord[timeSteps];
            allocatedRows++;
            for (int t = 0; t < timeSteps; t++) {
                targets[i][t].x = j.at("targets").at(i).at("positions").at(t).at("x").get<float>();
                targets[i][t].y = j.at("targets").at(i).at("positions").at(t).at("y").get<float>();
            }
        }
    } catch (const std::exception &e) {
        if (targets != nullptr) {
            for (int i = 0; i < allocatedRows; i++) {
                delete[] targets[i];
                targets[i] = nullptr;
            }
            delete[] targets;
            targets = nullptr;
        }
        targetCount = 0;
        timeSteps = 0;
        std::cerr << "Error: invalid " << targetsPath << ": " << e.what() << std::endl;
        return false;
    }

    return true;
}

void freeTargets(Coord **&targets, int targetCount) {
    if (targets == nullptr) {
        return;
    }
    for (int i = 0; i < targetCount; i++) {
        delete[] targets[i];
        targets[i] = nullptr;
    }
    delete[] targets;
    targets = nullptr;
}

void cleanupResources(Coord **&targets, int targetCount, AmmoParams *&ammo, SimStep *&simSteps) {
    freeTargets(targets, targetCount);
    delete[] simSteps;
    simSteps = nullptr;
    delete[] ammo;
    ammo = nullptr;
}

bool readAmmoJson(AmmoParams *&ammo, int &ammoCount) {
    const std::string ammoPath = joinPath(HW7_DATA_DIR, "ammo.json");
    std::ifstream ammoFile(ammoPath);
    if (!ammoFile.is_open()) {
        std::cerr << "Error: could not open " << ammoPath << std::endl;
        return false;
    }

    json j;
    try {
        ammoFile >> j;
        if (!j.is_array()) {
            std::cerr << "Error: ammo.json root must be an array" << std::endl;
            return false;
        }
        ammoCount = static_cast<int>(j.size());
        if (ammoCount <= 0) {
            std::cerr << "Error: ammo.json is empty" << std::endl;
            return false;
        }

        ammo = new AmmoParams[ammoCount];
        for (int i = 0; i < ammoCount; i++) {
            const auto &nameJson = j.at(i).at("name").get_ref<const json::string_t&>();
            std::strncpy(ammo[i].name, nameJson.c_str(), sizeof(ammo[i].name) - 1);
            ammo[i].name[sizeof(ammo[i].name) - 1] = '\0';
            ammo[i].mass = j.at(i).at("mass").get<float>();
            ammo[i].drag = j.at(i).at("drag").get<float>();
            ammo[i].lift = j.at(i).at("lift").get<float>();
        }
    } catch (const std::exception &e) {
        delete[] ammo;
        ammo = nullptr;
        ammoCount = 0;
        std::cerr << "Error: invalid " << ammoPath << ": " << e.what() << std::endl;
        return false;
    }

    return true;
}

bool lookupAmmo(const char ammoName[],
                const AmmoParams *ammo, int ammoCount,
                double &m, double &d, double &l) {
    for (int i = 0; i < ammoCount; i++) {
        if (std::strcmp(ammoName, ammo[i].name) == 0) {
            m = static_cast<double>(ammo[i].mass);
            d = static_cast<double>(ammo[i].drag);
            l = static_cast<double>(ammo[i].lift);
            return true;
        }
    }
    std::cerr << "Error: unknown ammo type '" << ammoName << "'" << std::endl;
    return false;
}

enum DroneState {
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

double computeTimeOfFlight(double m, double d, double l, double V0, double Z0) {
    const double g = 9.81;

    double a = d * g * m - 2.0 * d * d * l * V0;
    double b = -3.0 * g * m * m + 3.0 * d * l * m * V0;
    double c = 6.0 * m * m * Z0;

    double p = -(b * b) / (3.0 * a * a);
    double q = (2.0 * b * b * b) / (27.0 * a * a * a) + c / a;

    double arccos_arg = (3.0 * q / (2.0 * p)) * sqrt(-3.0 / p);

    if (arccos_arg < -1.0 || arccos_arg > 1.0) {
        return -1.0;
    }

    double phi = acos(arccos_arg);
    double t   = 2.0 * sqrt(-p / 3.0) * cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a);

    if (t <= 0) {
        return -1.0;
    }

    return t;
}

double computeHorizontalDistance(double m, double d, double l, double t, double V0) {
    const double g = 9.81;

    double term1 = V0 * t;
    double term2 = -t*t * d * V0 / (2.0 * m);
    double term3 = t*t*t * (6.0*d*g*l*m - 6.0*d*d*(l*l - 1.0)*V0) / (36.0 * m*m);
    double term4 = t*t*t*t * (-6.0*d*d*g*l*(1.0+l*l+l*l*l*l)*m + 3.0*d*d*d*l*l*(1.0+l*l)*V0 + 6.0*d*d*d*l*l*l*l*(1.0+l*l)*V0) / (36.0 * (1.0+l*l)*(1.0+l*l) * m*m*m);
    double term5 = t*t*t*t*t * (3.0*d*d*d*g*l*l*l*m - 3.0*d*d*d*d*l*l*(1.0+l*l)*V0) / (36.0 * (1.0+l*l) * m*m*m*m);

    return term1 + term2 + term3 + term4 + term5;
}

bool applyManeuver(double targetX, double targetY,
                   double &xd, double &yd, double &D,
                   double &xd_new, double &yd_new,
                   double h, double accelerationPath) {
    if (D < 1e-9) {
        xd_new = targetX - (h + accelerationPath);
        yd_new = targetY;
    } else if (h + accelerationPath > D) {
        xd_new = targetX - (targetX - xd) * (h + accelerationPath) / D;
        yd_new = targetY - (targetY - yd) * (h + accelerationPath) / D;
    } else {
        return false;
    }

    xd = xd_new;
    yd = yd_new;
    D  = h + accelerationPath;
    return true;
}

double wrapTrajectoryTime(double t, double arrayTimeStep, int timeSteps) {
    const double cycle = static_cast<double>(timeSteps) * arrayTimeStep;
    if (cycle <= 0.0) {
        return 0.0;
    }
    t = std::fmod(t, cycle);
    if (t < 0.0) {
        t += cycle;
    }
    return t;
}

Coord interpolateTarget(double t,
                        Coord **targets,
                        int targetIndex,
                        int timeSteps,
                        float arrayTimeStep) {
    t = wrapTrajectoryTime(t, arrayTimeStep, timeSteps);
    int idx  = static_cast<int>(std::floor(t / arrayTimeStep)) % timeSteps;
    int next = (idx + 1) % timeSteps;
    double frac = (t - static_cast<double>(idx) * arrayTimeStep) / arrayTimeStep;
    Coord p0 = targets[targetIndex][idx];
    Coord p1 = targets[targetIndex][next];
    return {
        static_cast<float>(p0.x + (p1.x - p0.x) * frac),
        static_cast<float>(p0.y + (p1.y - p0.y) * frac)
    };
}

bool computeDropPoint(Coord dronePos, double zd,
                      Coord aimPoint,
                      double m, double d, double l,
                      double attackSpeed, double accelerationPath,
                      Coord &firePoint,
                      double &travelDistance,
                      int debugStep) {
    double V0 = attackSpeed;
    double tFlight = computeTimeOfFlight(m, d, l, V0, zd);
    if (tFlight < 0.0) {
        return false;
    }
    double h = computeHorizontalDistance(m, d, l, tFlight, V0);

    double xdWork = dronePos.x;
    double ydWork = dronePos.y;
    double D = std::hypot(aimPoint.x - xdWork, aimPoint.y - ydWork);

    double xdNew = 0.0;
    double ydNew = 0.0;
    const bool maneuver = applyManeuver(aimPoint.x, aimPoint.y, xdWork, ydWork, D, xdNew, ydNew, h, accelerationPath);

    if (D < EPS) {
        return false;
    }
    double ratio = (D - h) / D;
    firePoint.x = static_cast<float>(xdWork + (aimPoint.x - xdWork) * ratio);
    firePoint.y = static_cast<float>(ydWork + (aimPoint.y - ydWork) * ratio);

    if (maneuver) {
        const double toManeuver = std::hypot(xdWork - dronePos.x, ydWork - dronePos.y);
        const double maneuverToFire = std::hypot(firePoint.x - xdWork, firePoint.y - ydWork);
        travelDistance = toManeuver + maneuverToFire;
    } else {
        travelDistance = std::hypot(firePoint.x - dronePos.x, firePoint.y - dronePos.y);
    }

    if (debugStep < DEBUG_STEPS_LIMIT) {
        DEBUG(std::fixed << std::setprecision(3)
              << "[DROP] step=" << debugStep
              << " D=" << D
              << " h+acc=" << (h + accelerationPath)
              << " maneuver=" << (maneuver ? 1 : 0)
              << " aim=(" << aimPoint.x << "," << aimPoint.y << ")"
              << " fire=(" << firePoint.x << "," << firePoint.y << ")"
              << " travelDist=" << travelDistance);
    }
    return true;
}

bool leadTargetingForTarget(Coord dronePos, double zd,
                             double currentTime,
                             int targetIndex,
                             Coord **targets,
                             int timeSteps,
                             float arrayTimeStep,
                             double m, double d, double l,
                             double attackSpeed, double accelerationPath,
                             Coord &outFirePoint,
                             double &outTravelTime, double &outImpactTime,
                             int debugStep) {
    const double tFlight = computeTimeOfFlight(m, d, l, attackSpeed, zd);
    if (tFlight < 0.0) {
        return false;
    }

    double travelTime = 0.0;

    for (int iter = 0; iter < 10; iter++) {
        double predTime = currentTime + travelTime + tFlight;
        Coord aim = interpolateTarget(predTime, targets, targetIndex, timeSteps, arrayTimeStep);
        Coord firePoint{0.0f, 0.0f};
        double travelDistance = 0.0;
        if (!computeDropPoint(dronePos, zd, aim, m, d, l,
                              attackSpeed, accelerationPath, firePoint, travelDistance, debugStep)) {
            return false;
        }

        double newTravel = travelDistance / attackSpeed;

        if (debugStep < DEBUG_STEPS_LIMIT) {
            DEBUG(std::fixed << std::setprecision(3)
                  << "[LEAD] step=" << debugStep
                  << " target=" << targetIndex
                  << " iter=" << iter
                  << " predT=" << predTime
                  << " aim=(" << aim.x << "," << aim.y << ")"
                  << " fire=(" << firePoint.x << "," << firePoint.y << ")"
                  << " travelDist=" << travelDistance
                  << " travelT=" << newTravel
                  << " flightT=" << tFlight);
        }

        if (std::fabs(newTravel - travelTime) < 1e-4) {
            outFirePoint = firePoint;
            outTravelTime = newTravel;
            outImpactTime = newTravel + tFlight;
            return true;
        }
        travelTime = newTravel;
    }

    double predTime = currentTime + travelTime + tFlight;
    Coord aim = interpolateTarget(predTime, targets, targetIndex, timeSteps, arrayTimeStep);
    double travelDistance = 0.0;
    if (!computeDropPoint(dronePos, zd, aim, m, d, l,
                          attackSpeed, accelerationPath, outFirePoint, travelDistance, debugStep)) {
        return false;
    }
    outTravelTime = travelDistance / attackSpeed;
    outImpactTime = outTravelTime + tFlight;
    return true;
}

double wrapAngle(double a) {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

double angleDelta(double fromRad, double toRad) {
    return wrapAngle(toRad - fromRad);
}

double computeTimeToStop(DroneState state,
                         double speed,
                         double acceleration,
                         double angularSpeed,
                         double remainingTurnAngleRad) {
    switch (state) {
        case STOPPED:
            return 0.0;
        case ACCELERATING:
        case DECELERATING:
        case MOVING:
            if (acceleration <= EPS) {
                return 1e9;
            }
            return speed / acceleration;
        case TURNING:
            if (angularSpeed <= EPS) {
                return 1e9;
            }
            return std::fabs(remainingTurnAngleRad) / angularSpeed;
        default:
            return 0.0;
    }
}

double rotateToward(double currentDir, double desiredDir, double maxStepRad) {
    double delta = angleDelta(currentDir, desiredDir);
    if (std::fabs(delta) <= maxStepRad) {
        return wrapAngle(desiredDir);
    }
    return wrapAngle(currentDir + std::copysign(maxStepRad, delta));
}

void advanceAlongHeading(double vAvg, double dir, double simTimeStep, Coord &pos) {
    pos.x += static_cast<float>(vAvg * std::cos(dir) * simTimeStep);
    pos.y += static_cast<float>(vAvg * std::sin(dir) * simTimeStep);
}

void decelerateThisStep(double simTimeStep,
                        double accel,
                        DroneState &state,
                        double &speed,
                        double dir,
                        Coord &pos) {
    if (accel <= EPS) {
        speed = 0.0;
        state = TURNING;
        return;
    }
    double v0 = speed;
    double v1 = std::max(0.0, v0 - accel * simTimeStep);
    advanceAlongHeading(0.5 * (v0 + v1), dir, simTimeStep, pos);
    speed = v1;
    state = (speed <= 1e-6) ? TURNING : DECELERATING;
    if (speed <= 1e-6) {
        speed = 0.0;
    }
}

void updateDrone(double simTimeStep,
                 double desiredDir,
                 double turnThresholdRad,
                 double attackSpeed,
                 double accelerationPath,
                 double angularSpeedRadPerSec,
                 DroneState &state,
                 double &speed,
                 double &dir,
                 Coord &pos) {
    const double accel = (accelerationPath > EPS)
        ? (attackSpeed * attackSpeed) / (2.0 * accelerationPath)
        : 0.0;

    double delta = angleDelta(dir, desiredDir);
    double wstep = angularSpeedRadPerSec * simTimeStep;

    switch (state) {
        case STOPPED: {
            if (std::fabs(delta) > turnThresholdRad) {
                state = TURNING;
            } else {
                state = ACCELERATING;
            }
            break;
        }
        case TURNING: {
            dir = rotateToward(dir, desiredDir, wstep);
            delta = angleDelta(dir, desiredDir);
            if (std::fabs(delta) < 1e-4) {
                state = ACCELERATING;
            }
            break;
        }
        case DECELERATING: {
            decelerateThisStep(simTimeStep, accel, state, speed, dir, pos);
            break;
        }
        case ACCELERATING: {
            if (accel <= EPS) {
                speed = attackSpeed;
                state = MOVING;
                break;
            }
            if (std::fabs(delta) > turnThresholdRad && speed > 1e-3) {
                decelerateThisStep(simTimeStep, accel, state, speed, dir, pos);
                break;
            }
            double v0 = speed;
            double v1 = std::min(attackSpeed, v0 + accel * simTimeStep);
            advanceAlongHeading(0.5 * (v0 + v1), dir, simTimeStep, pos);
            speed = v1;
            if (speed >= attackSpeed - 1e-6) {
                speed = attackSpeed;
                state = MOVING;
            }
            break;
        }
        case MOVING: {
            if (std::fabs(delta) > turnThresholdRad) {
                decelerateThisStep(simTimeStep, accel, state, speed, dir, pos);
                break;
            }
            dir = rotateToward(dir, desiredDir, wstep);
            advanceAlongHeading(speed, dir, simTimeStep, pos);
            break;
        }
        default:
            break;
    }
}

bool writeSimulationJson(const char *path,
                         const SimStep steps[],
                         int stepCount) {
    json out;
    out["totalSteps"] = stepCount;
    out["steps"] = json::array();

    for (int i = 0; i < stepCount; i++) {
        json step;
        step["position"] = {
            {"x", steps[i].pos.x},
            {"y", steps[i].pos.y}
        };
        step["direction"] = steps[i].direction;
        step["state"] = steps[i].state;
        step["targetIndex"] = steps[i].targetIdx;
        step["dropPoint"] = {
            {"x", steps[i].dropPoint.x},
            {"y", steps[i].dropPoint.y}
        };
        step["aimPoint"] = {
            {"x", steps[i].aimPoint.x},
            {"y", steps[i].aimPoint.y}
        };
        step["predictedTarget"] = {
            {"x", steps[i].predictedTarget.x},
            {"y", steps[i].predictedTarget.y}
        };
        out["steps"].push_back(step);
    }

    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: could not open simulation JSON output file" << std::endl;
        return false;
    }
    file << out.dump(2);
    file.close();
    return true;
}

int main() {
    AmmoParams *ammo = nullptr;
    SimStep *simSteps = nullptr;
    int ammoCount = 0;
    Coord **targets = nullptr;
    int targetCount = 0;
    int timeSteps = 0;

    DroneConfig config{};
    if (!readConfigJson(config)) {
        return 1;
    }
    LOG("Config loaded: speed=" << config.attackSpeed
        << " ammo=" << config.ammoName);

    Coord dronePos = config.startPos;
    double zd = config.altitude;
    float initialDir = config.initialDir;
    double attackSpeed = config.attackSpeed;
    double accelerationPath = config.accelPath;
    char ammo_name[50] = {};
    std::strncpy(ammo_name, config.ammoName, sizeof(ammo_name) - 1);
    ammo_name[sizeof(ammo_name) - 1] = '\0';
    float arrayTimeStep = config.arrayTimeStep;
    float simTimeStep = config.simTimeStep;
    float hitRadius = config.hitRadius;
    float angularSpeed = config.angularSpeed;
    float turnThreshold = config.turnThreshold;

    if (attackSpeed <= EPS || accelerationPath <= EPS || simTimeStep <= 0.0f
        || arrayTimeStep <= 0.0f || angularSpeed <= 0.0f) {
        std::cerr << "Error: invalid non-positive simulation parameters" << std::endl;
        return 1;
    }
    if (!readAmmoJson(ammo, ammoCount)) {
        return 1;
    }
    if (!readTargetsJson(targets, targetCount, timeSteps)) {
        cleanupResources(targets, targetCount, ammo, simSteps);
        return 1;
    }

    double m = 0.0, d = 0.0, l = 0.0;
    if (!lookupAmmo(ammo_name, ammo, ammoCount, m, d, l)) {
        cleanupResources(targets, targetCount, ammo, simSteps);
        return 1;
    }

    const double projectileFlightTime = computeTimeOfFlight(m, d, l, attackSpeed, zd);
    if (projectileFlightTime < 0.0) {
        std::cerr << "Error: invalid ballistic flight time" << std::endl;
        cleanupResources(targets, targetCount, ammo, simSteps);
        return 1;
    }
    const double projectileRange = computeHorizontalDistance(m, d, l, projectileFlightTime, attackSpeed);

    const auto dt = static_cast<double>(simTimeStep);
    const double accel = (attackSpeed * attackSpeed) / (2.0 * accelerationPath);

    DroneState state = STOPPED;
    double speed = 0.0;
    double dir = initialDir;

    double currentTime = 0.0;
    int activeTarget = -1;

    simSteps = new SimStep[MAX_RECORDS];
    int recCount = 0;

    int steps = 0;
    bool hit = false;

    while (steps < MAX_STEPS) {
        const double linearStopTime = computeTimeToStop(
            state, speed, accel,
            static_cast<double>(angularSpeed), 0.0);

        double bestScore = 1e300;
        int bestIndex = -1;
        Coord bestFirePoint{0.0f, 0.0f};

        for (int i = 0; i < targetCount; i++) {
            Coord firePoint{0.0f, 0.0f};
            double travelTime = 0.0;
            double impactTime = 0.0;
            if (!leadTargetingForTarget(dronePos, zd, currentTime, i,
                                        targets, timeSteps,
                                        arrayTimeStep,
                                        m, d, l, attackSpeed, accelerationPath,
                                        firePoint, travelTime, impactTime, steps)) {
                continue;
            }

            double penalty = 0.0;
            if (i != activeTarget) {
                if (state == TURNING) {
                    const double aimDir = std::atan2(firePoint.y - dronePos.y, firePoint.x - dronePos.x);
                    const double remTurn = std::fabs(angleDelta(dir, aimDir));
                    penalty = remTurn / static_cast<double>(angularSpeed);
                } else {
                    penalty = linearStopTime;
                }
            }
            const double score = impactTime + penalty;
            if (score < bestScore) {
                bestScore = score;
                bestIndex = i;
                bestFirePoint = firePoint;
            }
        }

        if (bestIndex < 0) {
            std::cerr << "Error: no reachable target / ballistics failure" << std::endl;
            cleanupResources(targets, targetCount, ammo, simSteps);
            return 1;
        }

        const double desiredDir = std::atan2(bestFirePoint.y - dronePos.y, bestFirePoint.x - dronePos.x);
        Coord aimPointNow{
            static_cast<float>(dronePos.x + projectileRange * std::cos(dir)),
            static_cast<float>(dronePos.y + projectileRange * std::sin(dir))
        };
        Coord predictedTargetNow = interpolateTarget(currentTime + projectileFlightTime, targets, bestIndex, timeSteps, arrayTimeStep);
        if (steps < DEBUG_STEPS_LIMIT) {
            DEBUG(std::fixed << std::setprecision(3)
                  << "[STEP] step=" << steps
                  << " t=" << currentTime
                  << " pos=(" << dronePos.x << "," << dronePos.y << ")"
                  << " state=" << static_cast<int>(state)
                  << " target=" << bestIndex
                  << " drop=(" << bestFirePoint.x << "," << bestFirePoint.y << ")"
                  << " dist=" << std::hypot(bestFirePoint.x - dronePos.x, bestFirePoint.y - dronePos.y)
                  << " heading=" << dir
                  << " desired=" << desiredDir);
        }

        if (recCount >= MAX_RECORDS) {
            std::cerr << "Error: simulation record buffer overflow" << std::endl;
            cleanupResources(targets, targetCount, ammo, simSteps);
            return 1;
        }
        simSteps[recCount].pos = dronePos;
        simSteps[recCount].direction = static_cast<float>(dir);
        simSteps[recCount].state = static_cast<int>(state);
        simSteps[recCount].targetIdx = bestIndex;
        simSteps[recCount].dropPoint = bestFirePoint;
        simSteps[recCount].aimPoint = aimPointNow;
        simSteps[recCount].predictedTarget = predictedTargetNow;
        recCount++;

        const double missNow = std::hypot(aimPointNow.x - predictedTargetNow.x, aimPointNow.y - predictedTargetNow.y);
        if (missNow <= static_cast<double>(hitRadius)) {
            if (recCount == 1) {
                simSteps[recCount].pos = dronePos;
                simSteps[recCount].direction = static_cast<float>(dir);
                simSteps[recCount].state = static_cast<int>(state);
                simSteps[recCount].targetIdx = bestIndex;
                simSteps[recCount].dropPoint = bestFirePoint;
                simSteps[recCount].aimPoint = aimPointNow;
                simSteps[recCount].predictedTarget = predictedTargetNow;
                recCount++;
            }
            hit = true;
            break;
        }

        updateDrone(dt, desiredDir, static_cast<double>(turnThreshold),
                    attackSpeed, accelerationPath,
                    static_cast<double>(angularSpeed),
                    state, speed, dir, dronePos);

        const double timeAfterStep = currentTime + dt;
        Coord aimPointAfter{
            static_cast<float>(dronePos.x + projectileRange * std::cos(dir)),
            static_cast<float>(dronePos.y + projectileRange * std::sin(dir))
        };
        Coord predictedTargetAfter = interpolateTarget(timeAfterStep + projectileFlightTime, targets, bestIndex, timeSteps, arrayTimeStep);
        const double missAfter = std::hypot(aimPointAfter.x - predictedTargetAfter.x, aimPointAfter.y - predictedTargetAfter.y);
        if (missAfter <= static_cast<double>(hitRadius)) {
            if (recCount >= MAX_RECORDS) {
                std::cerr << "Error: simulation record buffer overflow" << std::endl;
                cleanupResources(targets, targetCount, ammo, simSteps);
                return 1;
            }
            simSteps[recCount].pos = dronePos;
            simSteps[recCount].direction = static_cast<float>(dir);
            simSteps[recCount].state = static_cast<int>(state);
            simSteps[recCount].targetIdx = bestIndex;
            simSteps[recCount].dropPoint = bestFirePoint;
            simSteps[recCount].aimPoint = aimPointAfter;
            simSteps[recCount].predictedTarget = predictedTargetAfter;
            recCount++;
            hit = true;
            break;
        }

        activeTarget = bestIndex;
        currentTime += dt;
        steps++;
    }

    if (!hit) {
        std::cerr << "Error: hit radius not reached within MAX_STEPS" << std::endl;
        cleanupResources(targets, targetCount, ammo, simSteps);
        return 1;
    }

    const std::string simPath = joinPath(HW7_OUTPUT_DIR, "simulation.json");
    if (!writeSimulationJson(simPath.c_str(), simSteps, recCount)) {
        cleanupResources(targets, targetCount, ammo, simSteps);
        return 1;
    }

    cleanupResources(targets, targetCount, ammo, simSteps);
    return 0;
}
