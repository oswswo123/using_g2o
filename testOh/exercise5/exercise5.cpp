#include "using_optimizer.h"

#include <iostream>

int main(int argc, char* argv[]){

	if(argc != 3){
		std::cerr << "argc is " << argc << "."  << std::endl;
		std::cerr << "Invalid input. Please enter the correct value." << std::endl;
		return -1;
	}

	/* create optimizer part */

	UsingOptimizer usingoptimizer;

	/* input data load in optimizer part */

	std::string filename = "../data/" + std::string(argv[1]);
	usingoptimizer.add_data(filename, std::string(argv[2]));

	/* do optimize part */

	usingoptimizer.do_optimize();

	/* save optimizing data part */

	filename = "../data/after_" + std::string(argv[1]);
	usingoptimizer.file_save(filename);

	/* all vertex and edge data deleate */

	usingoptimizer.optimizer_clear();

	return 0;
}
