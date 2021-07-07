#include "g2o/core/block_solver.h"

#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"

#include "create_OA.h"
#include "create_BS.h"

g2o::OptimizationAlgorithmLevenberg* createLM(){
	std::cout << "Select block solver." << std::endl;
	std::cout << "Example) X, 6_3, 7_3, 3_2" << std::endl;

	std::string inputbuffer;
	bool repeat = true;
	while(repeat){
		std::cin >> inputbuffer;

		if(inputbuffer.compare("X") == 0){
			repeat = false;
			return new g2o::OptimizationAlgorithmLevenberg(std::move(createBSX()));
		}
		else if(inputbuffer.compare("6_3") == 0){
			repeat = false;
			return new g2o::OptimizationAlgorithmLevenberg(std::move(createBS63()));
		}
		else if(inputbuffer.compare("7_3") == 0){
			repeat = false;
			return new g2o::OptimizationAlgorithmLevenberg(std::move(createBS73()));
		}
		else if(inputbuffer.compare("3_2") == 0){
			repeat = false;
			return new g2o::OptimizationAlgorithmLevenberg(std::move(createBS32()));
		}
		else
			std::cout << "Invalid input. Please enter the correct value." << std::endl;
	}

	return NULL;
}

g2o::OptimizationAlgorithmGaussNewton* createGN(){
	std::cout << "Select block solver." << std::endl;
	std::cout << "Example) X, 6_3, 7_3, 3_2" << std::endl;

	std::string inputbuffer;
	bool repeat = true;
	while(repeat){
		std::cin >> inputbuffer;

		if(inputbuffer.compare("X") == 0){
			repeat = false;
			return new g2o::OptimizationAlgorithmGaussNewton(std::move(createBSX()));
		}
		else if(inputbuffer.compare("6_3") == 0){
			repeat = false;
			return new g2o::OptimizationAlgorithmGaussNewton(std::move(createBS63()));
		}
		else if(inputbuffer.compare("7_3") == 0){
			repeat = false;
			return new g2o::OptimizationAlgorithmGaussNewton(std::move(createBS73()));
		}
		else if(inputbuffer.compare("3_2") == 0){
			repeat = false;
			return new g2o::OptimizationAlgorithmGaussNewton(std::move(createBS32()));
		}
		else
			std::cout << "Invalid input. Please enter the correct value." << std::endl;
	}

	return NULL;
}
