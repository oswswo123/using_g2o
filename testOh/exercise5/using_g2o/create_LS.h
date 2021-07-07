#include "g2o/core/block_solver.h"

std::unique_ptr<g2o::BlockSolverX::LinearSolverType> create_BX_LS();
std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> create_B63_LS();
std::unique_ptr<g2o::BlockSolver_7_3::LinearSolverType> create_B73_LS();
std::unique_ptr<g2o::BlockSolver_3_2::LinearSolverType> create_B32_LS();
