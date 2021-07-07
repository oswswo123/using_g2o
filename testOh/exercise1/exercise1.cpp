#include <iostream>
#include <unistd.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"

#include "g2o/solvers/dense/linear_solver_dense.h"

#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/factory.h"

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

// g2o/solver/ : linear equation으로 근사화된 문제를 풀 수 있는 알고리즘
// g2o/core/optimization_algorithm_~~~ " : optimization algorithm을 선택(non linear equation 해결)
// g2o/types : SLAM이나 BA에서 자주 사용되는 상태 표현들의 클래스
// G2O_USE_TYPE_GROUP(slam3d); : 그래프를 *.g2o 파일에서 읽어올 때 필수

int getNewID();
void addPoseVertex(g2o::SparseOptimizer* oprimizer, g2o::SE3Quat& pose, bool set_fixed);
void addEdgePosePose(g2o::SparseOptimizer* optimizer, int id0, int id1, g2o::SE3Quat& relpose);

int main(){

	// step 1. create linear solver
	std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linear_solver =
		g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();

	// step 2. create block solver
	std::unique_ptr<g2o::BlockSolverX> block_solver = 
		g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));

	// step 3. create optimization algorithm
	g2o::OptimizationAlgorithm* algorithm =
		new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

	// step 4. create optimizer
	g2o::SparseOptimizer* optimizer = new g2o::SparseOptimizer;
	optimizer->setAlgorithm(algorithm);
	optimizer->setVerbose(true);		// to print optimization process

	std::vector<g2o::SE3Quat> gt_poses;

	// 1. Set fixed vertices(시작점, 첫점 설정)
	{
		Eigen::Vector3d tran;
		Eigen::Quaterniond quat;

		// first vertex at origin
		tran = Eigen::Vector3d(0, 0, 0);
		quat.setIdentity();
		g2o::SE3Quat pose0(quat, tran);
		addPoseVertex(optimizer, pose0, true);
		gt_poses.push_back(pose0);

		// second vertex at 1, 0, 0
		tran = Eigen::Vector3d(1, 0, 0);
		quat.setIdentity();
		g2o::SE3Quat pose1(quat, tran);
		addPoseVertex(optimizer, pose1, true);
		gt_poses.push_back(pose1);
	}

	// 2. Set variable vertices(이후의 vertex들은 최적화 대상이 됨)
	{
		const double PI = 3.141592653589793;
		const int CIRCLE_NODES = 8;
		const double CIRCLE_RADIUS = 2;
		double angle = 2.*PI/double(CIRCLE_NODES);
		
		// quaternion에 오일러 회전각 입력
		Eigen::Quaterniond quat;
		quat = Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 0, 1));

		// 3d vector에 x, y 위치 변화량 입력
		Eigen::Vector3d tran = 
			Eigen::Vector3d(CIRCLE_RADIUS*sin(angle), CIRCLE_RADIUS - CIRCLE_RADIUS*cos(angle), 0.);

		//relative pose between consecutive pose along the circle
		g2o::SE3Quat relpose(quat, tran);
		for(int i=0; i<CIRCLE_NODES; i++){
			g2o::SE3Quat abspose = gt_poses.back() * relpose;
			addPoseVertex(optimizer, abspose, false);
			gt_poses.push_back(abspose);
		}
	}

	// 3. Set edges between vertices
	{
		g2o::SE3Quat relpose;
		// add edge between poses
		for(size_t i=1; i<gt_poses.size(); i++){
			// relpose: pose[i-1] w.r.t pose[i]
			relpose = gt_poses[i-1].inverse() * gt_poses[i];
			addEdgePosePose(optimizer, i-1, i, relpose);
		}

		// the last pose supposed to be the same as gt_poses[1]
		relpose = gt_poses[1].inverse() * gt_poses.back();
		addEdgePosePose(optimizer, 1, int(gt_poses.size()-1), relpose);
	}

	// 4. Optimize and write files
	{
		// 최적화 준비
		optimizer->initializeOptimization();

		std::string filename;
		char *PRJ_PATH;
		PRJ_PATH = (char *)malloc(sizeof(char) * 256);
		getcwd(PRJ_PATH, 256);

		filename = std::string(PRJ_PATH) + "/../igdata/before_opt.g2o";
		optimizer->save(filename.c_str());

		std::cout << "before file path: " << filename << std::endl;

		// 최적화 개시
		optimizer->optimize(100);

		filename = std::string(PRJ_PATH) + "/../igdata/after_opt.g2o";
		optimizer->save(filename.c_str());

		std::cout << "after file path : " << filename << std::endl;
	}

	return 0;
}

int getNewID(){
	static int vertex_id = 0;
	return vertex_id++;
}

void addPoseVertex(g2o::SparseOptimizer* optimizer, g2o::SE3Quat& pose, bool set_fixed){
	g2o::VertexSE3* v_se3 = new g2o::VertexSE3;
	v_se3->setId(getNewID());
	if(set_fixed)
		v_se3->setEstimate(pose);
	v_se3->setFixed(set_fixed);
	optimizer->addVertex(v_se3);
}

void addEdgePosePose(g2o::SparseOptimizer* optimizer, int id0, int id1, g2o::SE3Quat& relpose){
	g2o::EdgeSE3* edge = new g2o::EdgeSE3;
	edge->setVertex(0, optimizer->vertices().find(id0)->second);
	edge->setVertex(1, optimizer->vertices().find(id1)->second);
	edge->setMeasurement(relpose);
	Eigen::MatrixXd info_matrix = Eigen::MatrixXd::Identity(6, 6) * 10.;
	edge->setInformation(info_matrix);
	optimizer->addEdge(edge);
}
