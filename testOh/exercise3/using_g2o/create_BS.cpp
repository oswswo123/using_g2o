#include "g2o/core/block_solver.h"

#include "create_LS.h"
#include "create_BS.h"

std::unique_ptr<g2o::BlockSolverX> createBSX(){
	return std::make_unique<g2o::BlockSolverX>(std::move(create_BX_LS()));
}

std::unique_ptr<g2o::BlockSolver_6_3> createBS63(){
	return std::make_unique<g2o::BlockSolver_6_3>(std::move(create_B63_LS()));
}

std::unique_ptr<g2o::BlockSolver_7_3> createBS73(){
	return std::make_unique<g2o::BlockSolver_7_3>(std::move(create_B73_LS()));
}

std::unique_ptr<g2o::BlockSolver_3_2> createBS32(){
	return std::make_unique<g2o::BlockSolver_3_2>(std::move(create_B32_LS()));
}
