#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam3d/vertex_se3.h"

#include "create_optimizer.h"
#include "using_optimizer.h"

G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_TYPE_GROUP(slam2d);

// constructor
UsingOptimizer::UsingOptimizer(){
	optimizer = create_sparseoptimizer();
	maxiteration = 50;
}

void UsingOptimizer::add_data(std::string filename, std::string vertexSE_type){
	if(!optimizer->load(filename.c_str())){
		std::cout << "Error loading graph" << std::endl;
		return;
	}
	else{
		std::cout << "Loaded " << optimizer->vertices().size() << " vertices" << std::endl;
		std::cout << "Loaded " << optimizer->edges().size() << " edges" << std::endl;
	}

	if(vertexSE_type.compare("VertexSE2") == 0){
		g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(optimizer->vertex(0));
		firstRobotPose->setFixed(true);
	}
	else if(vertexSE_type.compare("VertexSE3") == 0){
		g2o::VertexSE3* firstRobotPose = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex(0));
		firstRobotPose->setFixed(true);
	}
	else{
		std::cout << "Error loading first pose" << std::endl;
		return;
	}
}

void UsingOptimizer::set_maxiter(int n) { maxiteration = n; }

void UsingOptimizer::do_optimize(){
	optimizer->initializeOptimization();
	std::cerr << "Optimizing ... " << std::endl;
	optimizer->optimize(maxiteration);
	std::cerr << "done." << std::endl;
}

void UsingOptimizer::file_save(std::string filename){ optimizer->save(filename.c_str()); }
void UsingOptimizer::optimizer_clear(){ optimizer->clear(); }

