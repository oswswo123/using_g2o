#include "g2o/core/block_solver.h"

std::unique_ptr<g2o::BlockSolverX> createBSX();
std::unique_ptr<g2o::BlockSolver_6_3> createBS63();
std::unique_ptr<g2o::BlockSolver_7_3> createBS73();
std::unique_ptr<g2o::BlockSolver_3_2> createBS32();
