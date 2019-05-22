#ifndef BEZIERINTERPOLATION_H_INCLUDED
#define BEZIERINTERPOLATION_H_INCLUDED

#include "ObstacleAvoidance.h"
#include "ObstacleReconstruction.h"

void test_link();

std::vector<Eigen::MatrixXf> compute_bezier(Eigen::MatrixXf const& XY);

Eigen::MatrixXf border_to_vertices(Border const& obs);


#endif // BEZIERINTERPOLATION_H_INCLUDED
