#include "g2o/core/sparse_optimizer.h"

#include "create_OA.h"
#include "create_optimizer.h"

g2o::SparseOptimizer* create_sparseoptimizer(){
	g2o::SparseOptimizer* optimizer = new g2o::SparseOptimizer;

	std::cout << "Select nonlinear solver." << std::endl;
	std::cout << "Example) LM : Levenberg-Marquardt, GN : Gauss-Newton " << std::endl;

	std::string inputbuffer;
	bool repeat = true;
	while(repeat){
		std::cin >> inputbuffer;

		if(inputbuffer.compare("LM") == 0){
			optimizer->setAlgorithm(createLM());
			repeat = false;
		}
		else if(inputbuffer.compare("GN") == 0){
			optimizer->setAlgorithm(createGN());
			repeat = false;
		}
		else
			std::cout << "Invalid input. Please enter the correct value." << std::endl;
	}

	optimizer->setVerbose(true);

	return optimizer;
}
