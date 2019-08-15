#ifndef BEZIERINTERPOLATION_H_INCLUDED
#define BEZIERINTERPOLATION_H_INCLUDED

#include "ObstacleAvoidance.h"
#include "ObstacleReconstruction.h"

/**
 * Interpolate the surface of an obstacle with cubic Bezier curves based on a set of points that defines this surface
 *
 * @param XY Eigen matrix of size (N,2) containing the (x,y) position in the occupancy grid of the N points associated to this surface
 * @return Vector of Eigen matrices containing 2 matrices of size (N,M)
 *         Column i of the first matrix contains the x coordinates of the M points discretizing the interpolation of the surface between input points i and i+1 (with input point N+1 being input point 0)
 *         Column i of the second matrix contains the y coordinates of the M points discretizing the interpolation of the surface between input points i and i+1 (with input point N+1 being input point 0)
 *         Each section of the interpolation is discretized by M points (constant value set in the function)
 */
std::vector<Eigen::MatrixXf> compute_bezier(Eigen::MatrixXf const& XY);

/**
 * Convert a list of surface cells into a set of points that will be used as the input of the bezier interpolation.
 *
 * @param obs Eigen matrix of size (N,5) containing information about the N surface cells of an obstacle with [x, y, type, charac1, charac2] format
 *        x and y are the coordinates of the cell in the occupancy grid
 *        type is 1 for a straight cell, 2 for an outer corner, 3 for an inner corner (see report)
 *        charac1 and charac2 contain information specific to each type of cell (see report)
 * @return Eigen matrix of size (N,2) containing the (x,y) position in the occupancy grid of the N points associated to this surface
 *         Each surface cell generates 2 points and since adjacent cells share a point it makes a total of N unique points
 */
Eigen::MatrixXf border_to_vertices(Border const& obs);

/**
 * Return the projection of a point on a surface interpolated with cubic Bezier curves. The projected point is the closest point belonging to the surface.
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param pts_bezier Vector of Eigen matrices containing information about the interpolated surface of the obstacle (see function compute_bezier)
 * @return Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the projection of the point on the interpolated surface. Theta is not used.
 */
State get_projection_on_bezier(State const& state_robot, std::vector<Eigen::MatrixXf> const& pts_bezier);

/**
 * Returns the perimeter of a surface interpolated with cubic Bezier curves, the minimum distance along the surface between the projection of the robot and
 * the projection of the attractor as well as the corresponding direction to follow (clockwise or counter-clockwise).
 *
 * @param proj_robot Eigen matrix of size (1,2) containing the (x,y) position of the projection of the robot on the interpolated surface.
 * @param proj_attractor Eigen matrix of size (1,2) containing the (x,y) position of the projection of attractor on the interpolated surface.
 * @param pts_bezier Vector of Eigen matrices containing information about the interpolated surface of the obstacle (see function compute_bezier)
 * @return Eigen matrix of size (1,3) containing the perimeter (position 0), the minimum distance (position 1) and the direction (position 2)
 *         Direction is -1 for counter-clockwise and 1 for clockwise
 */
Eigen::Matrix<float, 1, 3> get_distances_surface_bezier(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float, 1, 2> const& proj_attractor, std::vector<Eigen::MatrixXf> const& pts_bezier);

/**
 * Returns the maximum gamma distance that can be reached by following the normal to the surface and before the projection of the robot switches to another part of the obstacle
 *
 * @param proj_robot Eigen matrix of size (1,2) containing the (x,y) position of the projection of the robot on the interpolated surface.
 * @param normal Eigen matrix of size (1,2) containing the (x,y) information about the normal vector to the surface where the projection of the robot is
 * @param pts_bezier Vector of Eigen matrix containing information about the interpolated surface of the obstacle (see function compute_bezier)
 * @return Maximum Gamma distance for this projection of the robot on the surface
 */
float get_max_gamma_distance_bezier(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float,1,2> const& normal, std::vector<Eigen::MatrixXf> const& pts_bezier);

/**
 * Returns the velocity command and Gamma distance for a single obstacle using cubic Bezier curves. Should be similar to next_step_special function except for added Bezier functions
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param border Eigen matrix of size (N,5) containing information about the N surface cells of an obstacle with [x, y, type, charac1, charac2] format
 *        x and y are the coordinates of the cell in the occupancy grid
 *        type is 1 for a straight cell, 2 for an outer corner, 3 for an inner corner (see report)
 *        charac1 and charac2 contain information specific to each type of cell (see report)
 * @return Eigen matrix of size (4,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta, gamma distance of the robot for this obstacle]
 */
Eigen::Matrix<float, 4, 1> test_next_step_special(State const& state_robot, State const& state_attractor, Border const& border);

/**
 * Returns the velocity command for a list of obstacles using cubic Bezier curves. Should be similar to next_step_special_weighted function except for added Bezier functions
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param borders Vector of Eigen matrices of size (Ni,5) containing information about the Ni surface cells of obstacle number i with [x, y, type, charac1, charac2] format
 *        x and y are the coordinates of the cell in the occupancy grid
 *        type is 1 for a straight cell, 2 for an outer corner, 3 for an inner corner (see report)
 *        charac1 and charac2 contain information specific to each type of cell (see report)
 * @return Eigen matrix of size (3,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta]
 */
State test_next_step_special_weighted(State const& state_robot, State const& state_attractor, std::vector<Border> const& borders, float const& size_of_cells);

/**
 * Returns the velocity command and Gamma distance with (Huber et al., 2019)'s method for star-shaped obstacles (no projection into circle space)
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param state_reference Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the reference point. Theta is not used.
 * @param border Eigen matrix of size (N,5) containing information about the N surface cells of an obstacle with [x, y, type, charac1, charac2] format
 *        x and y are the coordinates of the cell in the occupancy grid
 *        type is 1 for a straight cell, 2 for an outer corner, 3 for an inner corner (see report)
 *        charac1 and charac2 contain information specific to each type of cell (see report)
 * @return Eigen matrix of size (4,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta, gamma distance of the robot for this obstacle]
 */
Eigen::Matrix<float, 4, 1> next_step_classic(State const& state_robot, State const& state_attractor, State const& state_reference, Border const& border);

/**
 * Compute the velocity command of the robot depending on its relative position to several obstacles (version with limit distance to consider obstacles)
 * Reconstruction of surfaces based on straight lines, inner and outer corners
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param borders Vector of Eigen matrices of size (Ni,5) containing information about the Ni surface cells of obstacle number i with [x, y, type, charac1, charac2] format
 *        x and y are the coordinates of the cell in the occupancy grid
 *        type is 1 for a straight cell, 2 for an outer corner, 3 for an inner corner (see report)
 *        charac1 and charac2 contain information specific to each type of cell (see report)
 * @param size_of_cells Size of the cells of the occupancy grid in meters
 * @param corrected_velocity Enable the new method to get the velocity command back in initial space (default: false)
 * @param bezier_enabled Enable the reconstruction of the surface of obstacles with Bezier interpolation (default: false)
 * @return Eigen matrix of size (3,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta]
 */
State get_next_velocity_command_weighted(State const& state_robot, State const& state_attractor, std::vector<Border> const& borders, float const& size_of_cells, bool const& corrected_velocity=false, bool const& bezier_enabled=false);

/**
 * Compute the velocity command of the robot depending on its relative position to single obsacle (version with limit distance to consider obstacles)
 * Reconstruction of the surface is based on straight lines, inner and outer corners
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @param corrected_velocity Enable the new method to get the velocity command back in initial space (default: false)
 * @param bezier_enabled Enable the reconstruction of the surface of obstacles with Bezier interpolation (default: false)
 * @return Eigen matrix of size (4,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta, gamma distance] for the considered obstacle
 */
Eigen::Matrix<float, 4, 1> get_next_velocity_command(State const& state_robot, State const& state_attractor, Border const& border, bool const& corrected_velocity=false, bool const& bezier_enabled=false);

/**
 * Compute the projection of a point in circle space for a given position in the initial space
 * Bezier interpolation version of point_from_initial_to_circle
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @return Eigen matrix of size (10,1) containing [ gamma distance of the robot in circle space, (x,y,theta) position of the robot in circle space, (x,y,theta) position of the attractor in circle space,
 *          (x,y,theta) of the reference vector in circle space] with theta always being 0
 */
Eigen::Matrix<float, 10, 1> point_from_initial_to_circle_bezier(State const& state_robot, State const& state_attractor, std::vector<Eigen::MatrixXf> & pts_bezier);

/**
 * Compute a numerical approximation of the d X^c / d X^i matrix for a given position in the initial space
 * c means circle space, i means initial space
 * d X^c / d X^i = [ dxc/dxi  dxc/dyi
 *                   dyc/dxi  dyc/dyi ]
 * Bezier interpolation version of get_derivation_matrix
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @return Eigen matrix of size (2,2) containing an estimation of the variation of the circle space depending on the variation in the initial space
 */
Eigen::Matrix<double, 2, 2> get_derivation_matrix_bezier(State const& state_robot, State const& state_attractor, std::vector<Eigen::MatrixXf> & pts_bezier, Eigen::Matrix<float, 10, 1> const& point_robot);


#endif // BEZIERINTERPOLATION_H_INCLUDED
