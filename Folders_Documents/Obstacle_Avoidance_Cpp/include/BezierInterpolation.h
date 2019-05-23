#ifndef BEZIERINTERPOLATION_H_INCLUDED
#define BEZIERINTERPOLATION_H_INCLUDED

#include "ObstacleAvoidance.h"
#include "ObstacleReconstruction.h"

std::vector<Eigen::MatrixXf> compute_bezier(Eigen::MatrixXf const& XY);

Eigen::MatrixXf border_to_vertices(Border const& obs);

State get_projection_on_bezier(State const& state_robot, std::vector<Eigen::MatrixXf> const& pts_bezier);

#endif // BEZIERINTERPOLATION_H_INCLUDED
