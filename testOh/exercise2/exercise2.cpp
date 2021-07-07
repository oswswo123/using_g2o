#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam3d/vertex_se3.h"

G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_TYPE_GROUP(slam2d);

#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace std;
using namespace g2o;

#define MAXITERATION 100

int main(int agrc, char* argv[]){
	// create the linear solver
	std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver = 
		g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();

    // create the block solver on the top of the linear solver
	std::unique_ptr<g2o::BlockSolverX> blockSolver = 
		g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));

    //create the algorithm to carry out the optimization
	g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = 
		new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

	SparseOptimizer optimizer;

	string filename = "../data/" + string(argv[1]);
	cout << filename << endl;

	if(!optimizer.load(filename.c_str())){
	//if(!optimizer.load("../data/manhattanOlson3500.g2o")){
	//if(!optimizer.load("../data/sphere_bignoise_vertex3.g2o")){
		cout << "Error loading graph" << endl;
		return -1;
	}
	else{
		cout << "Loaded " << optimizer.vertices().size() << " vertices" << endl;
		cout << "Loaded " << optimizer.edges().size() << " edges" << endl;
	}

	if(strcmp(argv[2], "VertexSE2") == 0){
		VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
		firstRobotPose->setFixed(true);
	}
	else if(strcmp(argv[2], "VertexSE3") == 0){
		VertexSE3* firstRobotPose = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
		firstRobotPose->setFixed(true);
	}
	else{
		cout << "Error loading first pose" << endl;
		return -1;
	}

	optimizer.setAlgorithm(optimizationAlgorithm);
	optimizer.setVerbose(true);
	optimizer.initializeOptimization();
	cerr << "Optimizing ... " << endl;
	optimizer.optimize(MAXITERATION);
	cerr << "done." << endl;

	filename = "../data/after_" + string(argv[1]);
	optimizer.save(filename.c_str());
	//optimizer.save("../data/manhattanOlson3500_after.g2o");
	//optimizer.save("../data/sphere_after.g2o");
	optimizer.clear();

	return 0;
}
