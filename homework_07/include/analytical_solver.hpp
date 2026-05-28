#ifndef HOMEWORK_07_ANALYTICAL_SOLVER_HPP
#define HOMEWORK_07_ANALYTICAL_SOLVER_HPP

#include "interfaces/i_ballistic_solver.hpp"

class AnalyticalSolver : public IBallisticSolver {
public:
  DropSolution solve(const Coord &dronePos,
                     const TargetSnapshot &target,
                     float altitude,
                     const AmmoParams &ammo,
                     float attackSpeed,
                     float accelerationPath) const override;
};

#endif
