#ifndef HOMEWORK_07_DOMAIN_TYPES_HPP
#define HOMEWORK_07_DOMAIN_TYPES_HPP

struct Coord {
  float x;
  float y;
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

struct TargetSnapshot {
  Coord position;
  Coord velocity;
};

struct DropSolution {
  Coord dropPoint;
  double travelTime;
  double impactTime;
  bool ok;
};

#endif
