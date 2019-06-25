#ifndef BEZIERINTERPOLATION_H_INCLUDED
#define BEZIERINTERPOLATION_H_INCLUDED

#include "ObstacleAvoidance.h"
#include "ObstacleReconstruction.h"

std::vector<Eigen::MatrixXf> compute_bezier(Eigen::MatrixXf const& XY);

Eigen::MatrixXf border_to_vertices(Border const& obs);

State get_projection_on_bezier(State const& state_robot, std::vector<Eigen::MatrixXf> const& pts_bezier);

Eigen::Matrix<float, 1, 3> get_distances_surface_bezier(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float, 1, 2> const& proj_attractor, std::vector<Eigen::MatrixXf> const& pts_bezier);

float get_max_gamma_distance_bezier(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float,1,2> const& normal, std::vector<Eigen::MatrixXf> const& pts_bezier); // maximum gamma distance that can be reached following the normal

Eigen::Matrix<float, 4, 1> test_next_step_special(State const& state_robot, State const& state_attractor, Border const& border);

State test_next_step_special_weighted(State const& state_robot, State const& state_attractor, std::vector<Border> const& borders, float const& size_of_cells);

Eigen::Matrix<float, 4, 1> next_step_classic(State const& state_robot, State const& state_attractor, State const& state_reference, Border const& border);

#endif // BEZIERINTERPOLATION_H_INCLUDED
