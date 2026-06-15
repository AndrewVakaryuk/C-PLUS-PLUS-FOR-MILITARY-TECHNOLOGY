#ifndef HOMEWORK_07_INTERFACES_I_BALLISTIC_SOLVER_HPP
#define HOMEWORK_07_INTERFACES_I_BALLISTIC_SOLVER_HPP

#include "domain_types.hpp"

class ITargetProvider;

class IBallisticSolver {
public:
  virtual ~IBallisticSolver() {}

  virtual DropSolution solve(const Coord &dronePos,
                             const TargetSnapshot &target,
                             float altitude,
                             const AmmoParams &ammo,
                             float attackSpeed,
                             float accelerationPath) const = 0;

  virtual bool solveLead(const Coord &dronePos,
                         double currentTimeSeconds,
                         int targetIndex,
                         const ITargetProvider &targetProvider,
                         float altitude,
                         const AmmoParams &ammo,
                         float attackSpeed,
                         float accelerationPath,
                         DropSolution &result) const = 0;
};

#endif
