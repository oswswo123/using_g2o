#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"

g2o::OptimizationAlgorithmLevenberg* createLM();
g2o::OptimizationAlgorithmGaussNewton* createGN();
