#include "g2o/core/block_solver.h"

#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "create_LS.h"

std::unique_ptr<g2o::BlockSolverX::LinearSolverType> create_BX_LS(){
	std::cout << "Select linear solver." << std::endl;
	std::cout << "Example) Dense, Cholmod, CSparse " << std::endl;

	std::string inputbuffer;
	bool repeat = true;
	while(repeat){
		std::cin >> inputbuffer;

		if(inputbuffer.compare("Dense") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
		}
		else if(inputbuffer.compare("Cholmod") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
		}
		else if(inputbuffer.compare("CSparse") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
		}
		else
			std::cout << "Invalid input. Please enter the correct value." << std::endl;
	}

	return NULL;
}

std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> create_B63_LS(){
	std::cout << "Select linear solver." << std::endl;
	std::cout << "Example) Dense, Cholmod, CSparse " << std::endl;

	std::string inputbuffer;
	bool repeat = true;
	while(repeat){
		std::cin >> inputbuffer;

		if(inputbuffer.compare("Dense") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
		}
		else if(inputbuffer.compare("Cholmod") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
		}
		else if(inputbuffer.compare("CSparse") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>>();
		}
		else
			std::cout << "Invalid input. Please enter the correct value." << std::endl;
	}

	return NULL;
}

std::unique_ptr<g2o::BlockSolver_7_3::LinearSolverType> create_B73_LS(){	
	std::cout << "Select linear solver." << std::endl;
	std::cout << "Example) Dense, Cholmod, CSparse " << std::endl;

	std::string inputbuffer;
	bool repeat = true;
	while(repeat){
		std::cin >> inputbuffer;

		if(inputbuffer.compare("Dense") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_7_3::PoseMatrixType>>();
		}
		else if(inputbuffer.compare("Cholmod") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_7_3::PoseMatrixType>>();
		}
		else if(inputbuffer.compare("CSparse") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_7_3::PoseMatrixType>>();
		}
		else
			std::cout << "Invalid input. Please enter the correct value." << std::endl;
	}

	return NULL;
}

std::unique_ptr<g2o::BlockSolver_3_2::LinearSolverType> create_B32_LS(){
	std::cout << "Select linear solver." << std::endl;
	std::cout << "Example) Dense, Cholmod, CSparse " << std::endl;

	std::string inputbuffer;
	bool repeat = true;
	while(repeat){
		std::cin >> inputbuffer;

		if(inputbuffer.compare("Dense") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_3_2::PoseMatrixType>>();
		}
		else if(inputbuffer.compare("Cholmod") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_3_2::PoseMatrixType>>();
		}
		else if(inputbuffer.compare("CSparse") == 0){
			repeat = false;
			return std::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_3_2::PoseMatrixType>>();
		}
		else
			std::cout << "Invalid input. Please enter the correct value." << std::endl;
	}

	return NULL; 
}
