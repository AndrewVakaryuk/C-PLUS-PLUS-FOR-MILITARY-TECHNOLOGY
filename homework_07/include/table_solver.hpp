#pragma once

#include <string>

#include "ballistic_table.hpp"
#include "interfaces/i_ballistic_solver.hpp"

class TableSolver : public IBallisticSolver {
public:
  TableSolver();
  explicit TableSolver(std::string tablePath);

  DropSolution solve(const Coord &dronePos,
                     const TargetSnapshot &target,
                     float altitude,
                     const AmmoParams &ammo,
                     float attackSpeed,
                     float accelerationPath) const override;
  bool projectileMetrics(float altitude,
                         const AmmoParams &ammo,
                         float dropSpeed,
                         double &flightTime,
                         double &horizontalRange) const override;
  bool solveLead(const Coord &dronePos,
                 double currentTimeSeconds,
                 int targetIndex,
                 const ITargetProvider &targetProvider,
                 float altitude,
                 const AmmoParams &ammo,
                 float attackSpeed,
                 float accelerationPath,
                 DropSolution &result) const override;

private:
  BallisticTable table_;
  bool initialized_;
};
