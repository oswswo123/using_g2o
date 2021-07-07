#include "g2o/core/sparse_optimizer.h"

class UsingOptimizer{
private:
	g2o::SparseOptimizer* optimizer;
	int maxiteration;
	
public:
	UsingOptimizer();
	void add_data(std::string filename, std::string vertexSE_type);
	void set_maxiter(int n);
	void do_optimize();
	void file_save(std::string filename);
	void optimizer_clear();
};
