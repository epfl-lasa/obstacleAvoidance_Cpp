/*
Implementation by Pierre-Alexandre Léziart of the method described in (Huber and al., 2019)
LASA laboratory, EPFL, Spring 2019
Mail: pierre-alexandre.leziart [at] epfl [dot] ch
Contains functions for obstacle avoidance algorithm
*/

#ifndef OBSTACLEAVOIDANCE_H_INCLUDED
#define OBSTACLEAVOIDANCE_H_INCLUDED

#include <iostream>            // In-Out streams
#include <fstream>             // To write data into files
#include <eigen3/Eigen/Dense>  // For Linear Algebra
#include <cmath>               // Basic math functions

const int number_states = 3;   // States are [x, y, phi]
const int method_weights = 2;  // 1 to consider all the obstacles, 2 to consider only the obstacles within the limit_dist range
const float limit_dist = 2.5;
/* limit gamma distance for method_weights=2, set it to -1 if you use method_weights=1
 * Gamma_distance = 1 + euclidian_distance so euclidian_distance = Gamma_distance - 1
 * If the gamma limit is set to 6 then the euclidian limit will be (6-1) = 5
 * In an occupancy grid that means the euclidian limit will be 5 cells
 * If cells have a size of 20 cm then the real world limit distance will be 1 meter to consider obstacles
 */

using State = Eigen::Matrix<float, number_states, 1>; // State is an alias to represent a column vector with three components
using Obstacle = Eigen::Matrix<float, 10, 1>;         // Obstacle is an alias to represent ellipses [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]

/*  Disclaimer: there may be some references to "epsilon" because initially I thought it was one instead of a xi
 *
 *   Latex equation of an ellipse: \dfrac {((x-h)\cos(A)+(y-k)\sin(A))^2}{(a^2)}+\dfrac{((x-h) \sin(A)-(y-k) \cos(A))^2}{(b^2)}=1
 *   where h,k and a,b are the shifts and semi-axis in the x and y directions respectively and A is the angle measured from x axis.
 *
 *   The following functions are defined in the paper "Avoidance of Convex and Concave Obstacles with Convergence ensured through Contraction"
 *   Each step has its own function. The function that regroups all the other is next_step_single_obstacle
 */

/**
 * Compute the initial velocity command for a given position of the robot and of the attractor.
 * This velocity command is the one the algorithm starts with, with the velocity field not deformed by obstacles
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the robot
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @return Eigen matrix of size (3,1), the velocity command that the robot has to follow if there is no obstacle in its surroundings
 */
State f_epsilon(State const& state_robot, State const& state_attractor); // compute attractor function

/**
 * Compute gamma(xi) for an obstacle and a given position of the robot
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot.
 * @param obs Eigen matrix of size (10,1) containing information about the elliptic obstacle with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @return Value of Gamma distance from the ellipse
 */
float gamma(State const& state_robot, Obstacle const& obs, bool is_radius=false);

/**
 * Compute lambda_r(xi) for an obstacle and a given position of the robot, used to computed D(xi) matrix
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot.
 * @param obs Eigen matrix of size (10,1) containing information about the elliptic obstacle with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @param limit_distance Optionnal value when we want to consider an obstacle only when the Gamma value of the robot for this obstacle is inferior to limit_distance
 * @return Value of the lambda_r coefficient
 */
float lambda_r(State const& state_robot, Obstacle const& obs, float limit_distance=-1);

/**
 * Compute lambda_e(xi) for an obstacle and a given position of the robot, used to computed D(xi) matrix
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot
 * @param obs Eigen matrix of size (10,1) containing information about an elliptic obstacle with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @param limit_distance optionnal value when we want to consider an obstacle only when the Gamma value of the robot for this obstacle is inferior to limit_distance
 * @return Value of the lambda_e coefficient
 */
float lambda_e(State const& state_robot, Obstacle const& obs, float limit_distance=-1);

/**
 * Compute a D(xi) matrix as defined in the method of (Huber and al., 2019)
 *
 * @param lambda_r Value of the lambda_r coefficient for the considered obstacle
 * @param lambda_e Value of the lambda_e coefficient for the considered obstacle
 * @return D(xi) Eigen matrix of size (number_states, number_states)
 */
Eigen::Matrix<float, number_states, number_states> D_epsilon( float const& lamb_r, float const& lamb_e);

/**
 * Compute a r(xi) vector as defined in the method of (Huber and al., 2019) that is used to get the E(xi) matrix
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot
 * @param obs Eigen matrix of size (10,1) containing information about an elliptic obstacle with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @return Eigen matrix of size (3,1) containing the (x,y,theta) data of the r(xi) vector.
 */
State r_epsilon(State const& state_robot, Obstacle const& obs);

/**
 * Compute the norm-2 distance between two Eigen matrices of size (3,1) containing the (x,y,theta) information about two points of the workspace
 *
 * @param state1 Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot
 * @param state2 Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot
 * @return Norm-2 distance between the two input state vectors
 */
float distance(State const& state1, State const& state2);

/**
 * Compute the partial derivative of the Gamma-distance for an elliptic obstacle and a given direction (x, y or theta)
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot
 * @param obs Eigen matrix of size (10,1) containing information about an elliptic obstacle with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @param direction Direction of the partial derivative (0 for x, 1 for y, 2 for theta)
 * @return Value of the partial derivative along the input direction
 */
float partial_derivative(State const& state_robot, Obstacle const& obs, int const& direction);

/**
 * Compute the gradient of the Gamma-distance at a given point of the workspace for an elliptic obstacle
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot
 * @param obs Eigen matrix of size (10,1) containing information about an elliptic obstacle with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @return Eigen matrix of size (3,1) containing the [partial derivative along x, partial derivative along y, partial derivative along theta]
 */
State gradient(State const& state_robot, Obstacle const& obs);

/**
 * Compute the orthogonal basis of an hyperplane which is orthogonal to the input vector, with the Gram-Schmidt method
 *
 * @param gradient_vector Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot.
 * @return Eigen matrix of size (3,2) containing two state vectors which are orthogonal with each other and with the input vector
 */
Eigen::Matrix<float, number_states, number_states-1> gram_schmidt(State const& gradient_vector);

/**
 * Compute the projection of a vector on another vector.
 *
 * @param vec1 Eigen matrix of size (3,1), vector that will be projected on the other one
 * @param vec2 Eigen matrix of size (3,1)
 * @return Eigen matrix of size (3,1), result of the projection of vec1 on vec2
 */
State projection(State const& vec1, State const& vec2);

/**
 * Compute the E(xi) matrix as defined in the method of (Huber and al., 2019)
 *
 * @param r_eps_vector Eigen matrix of size (3,1), reference vector r(xi)
 * @param ortho_basis Eigen matrix of size (3,2), orthogonal basis
 * @return Matrix E(xi), an Eigen matrix of size (3,3)
 */
Eigen::Matrix<float, number_states, number_states> E_epsilon(State const& r_eps_vector, Eigen::Matrix<float, number_states, number_states-1> const& ortho_basis);

/**
 * Compute the M(xi) matrix as defined in the method of (Huber and al., 2019)
 *
 * @param D_eps Matrix D(xi), an Eigen matrix of size (3,3)
 * @param E_eps Matrix E(xi), an Eigen matrix of size (3,3)
 * @return Matrix M(xi), an Eigen matrix of size (3,3)
 */
Eigen::Matrix<float, number_states, number_states> M_epsilon(Eigen::Matrix<float, number_states, number_states> const& D_eps, Eigen::Matrix<float, number_states, number_states> const& E_eps);

/**
 * Compute xi_dot(xi) as defined in the method of (Huber and al., 2019), with M(xi) and f(xi)
 *
 * @param M_eps Matrix M(xi), an Eigen matrix of size (3,3) which deforms the velocity field
 * @param f_eps Matrix f(xi), an Eigen matrix of size (3,1), the velocity command before deformation
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot
 * @param obs Eigen matrix of size (10,1) containing information about an elliptic obstacle with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @return Eigen matrix of size (3,1), the velocity command after deformation by the obstacle
 */
State epsilon_dot(Eigen::Matrix<float, number_states, number_states> const& M_eps, State const& f_eps, State const& state_robot, Obstacle const& obs);

/**
 * Apply the method of (Huber and al., 2019) for a single elliptic obstacle and a given position of the robot and of the attractor
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the robot
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @param obs Eigen matrix of size (10,1) containing information about an elliptic obstacle with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @return Eigen matrix of size (3,1), the velocity command that the robot has to follow
 */
State next_step_single_obstacle(State const& state_robot, State const& state_attractor, Obstacle const& obs);

/**
 * Apply the method of (Huber and al., 2019) to get the velocity commands generated by several elliptic obstacles and a given position of the robot and of the attractor
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the robot
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @param mat_obs Eigen matrix of size (10,N) containing information about N elliptic obstacles with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format.
 *             Each column contains information about one obstacle.
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @return Eigen matrix of size (3,N), the velocity commands generated by all obstacles independently. Column i contains the velocity command the robot would have
 *         to follow if there was just obstacle i in its surroundings.
 */
Eigen::MatrixXf velocities(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_obs);

/**
 * Compute the respective weight of several elliptic obstacles for a given position of the robot
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the robot
 * @param mat_obs Eigen matrix of size (10,N) containing information about N elliptic obstacles with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format.
 *             Each column contains information about one obstacle.
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @param method Selection of the weighting method. 1 for the method of (Huber and al., 2019), 2 for a method that assign a weight of 0 to obstacles that are too far from the robot
 * @return Eigen matrix of size (1,N), respective weight of each obstacle. Sum of weights is equal to 1.
 */
Eigen::MatrixXf weights(State const& state_robot, Eigen::MatrixXf const& mat_obs, int const& method, float const& limit_distance=-1);

/**
 * Compute the magnitude of the final velocity command after weighting all obstacles.
 *
 * @param mat_weights Eigen matrix of size (1,N), weight of each obstacle. Sum of weights is equal to 1.
 * @param mat_magnitudes Eigen matrix of size (1,N), magnitude of the velocity command generated by each obstacle independently of each other.
 * @return Value of the magnitude of the final velocity command after the weighting process.
 */
float weighted_magnitude(Eigen::MatrixXf const& mat_weights, Eigen::MatrixXf const& mat_magnitudes);

/**
 * Compute the direction of the final velocity command after weighting all obstacles. Simplified formulas for the 2D case (only x and y matter).
 * See (Huber and al., 2019) for the formula with more dimensions.
 *
 * @param mat_norm_velocities Eigen matrix of size (3,N), normalized velocity command generated by each obstacle independently of each other.
 * @param mat_weights Eigen matrix of size (1,N), weight of each obstacle. Sum of weights is equal to 1.
 * @return Direction of the final velocity command after the weighting process.
 */
State n_bar_2D(Eigen::MatrixXf const& mat_norm_velocities, Eigen::MatrixXf const& mat_weights);

/**
 * Compute the next velocity command that the robot has to follow for several elliptic obstacles according to the method of (Huber and al., 2019)
 * It calls the next_step_single_obstacle function for each obstacle and weights the different velocity commands.
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the robot
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @param mat_obs Eigen matrix of size (10,N) containing information about N elliptic obstacles with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format.
 *             Each column contains information about one obstacle.
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @return Eigen matrix of size (3,1), the velocity command that the robot has to follow after weighting all obstacles
 */
State one_step_2D(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_obs);

/**
 * Compute the next velocity command that the robot has to follow for several elliptic obstacles without weighting.
 * It gets the deformation matrix associated with each obstacle and applies it directly to the velocity field.
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the robot
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @param mat_obs Eigen matrix of size (10,N) containing information about N elliptic obstacles with the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] format.
 *             Each column contains information about one obstacle.
 *             x_c, y_c, phi are the position and orientation
 *             a1, a2 are the length of the half-axes of the ellipse
 *             p1, p2 are the power in the equation of the ellipse
 *             v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @return Eigen matrix of size (3,1), the velocity command that the robot has to follow after deforming the velocity field
 */
State direct_multiplication_2D(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_obs);

/**
 * Compute the rotation matrix to align the input velocity command with the first axis of the frame.
 * It is the R matrix evoked in (Huber and al., 2019)
 * Implemented for the 2D dimensional case (only x and y matter), need some work if you want it for the d-dimensional case
 * This function is currently not used. Since we limited ourselves to the 2D case, I did not finished the d-dimensional implementation (need to finish n_bar_matrix)
 *
 * @param f_eps Eigen matrix of size (3,1) containing a (x,y,theta) state vector
 * @return Eigen matrix of size (3,3), the rotation matrix that will align the input vector with the first axis of the frame
 */
Eigen::Matrix<float, number_states, number_states> R_matrix(State const& f_eps);

/**
 * Compute the kappa matrix evoked in (Huber and al., 2019)
 * This function is currently not used. Since we limited ourselves to the 2D case, I did not finished the d-dimensional implementation (need to finish n_bar_matrix)
 *
 * @param mat_n_hat Eigen matrix of size (3,N) containing n_hat information about N obstacles. See (Huber and al., 2019), mat_n_hat = R_mat * mat_norm_velocities
 * @return Eigen matrix of size (2,3), the kappa matrix evoked in (Huber and al., 2019)
 */
Eigen::MatrixXf kappa_matrix(Eigen::MatrixXf const& mat_n_hat);

/**
 * Compute the n_bar matrix evoked in (Huber and al., 2019) with the R and kappa matrices.
 * This function is currently not used and not finished. Since we limited ourselves to the 2D case, I did not finished the d-dimensional implementation (need to finish n_bar_matrix)
 *
 * @param mat_kappa_bar Eigen matrix of size (3,N) containing n_hat information about N obstacles. See (Huber and al., 2019) to see what is the kappa_bar matrix
 * @param R_mat rotation matrix to align the current velocity command with the first axis of the frame
 * @return Eigen matrix of size (3,N), the n_bar matrix evoked in (Huber and al., 2019)
 */
Eigen::MatrixXf n_bar_matrix(Eigen::MatrixXf const& mat_kappa_bar, Eigen::Matrix<float, number_states, number_states> const& R_mat);

/**
 * Update the position of elliptic obstacles based on their linear and angular velocities
 *
 * @param mat_obs Eigen matrix of size (N,10) containing the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] information about N obstacles.
 *            x_c, y_c, phi are the position and orientation
 *            a1, a2 are the length of the half-axes of the ellipse
 *            p1, p2 are the power in the equation of the ellipse
 *            v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @param time_step Time step that has to be used to update the obstacles [s]
 */
void update_obstacles(Eigen::MatrixXf & mat_obs, float const& time_step);

/**
 * Create a grid of the workspace and compute the velocity command for each point of this grid for several elliptic obstacles with the method of (Huber and al., 2019)
 * Results are stored in a text file that has to be opened with Python to plot a quiver graph with matplotlib
 *
 * @param limits Eigen matrix of size (5,1) containing the [x_min, x_max, y_min, y_max, step] information that defines the grid
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @param mat_obs Eigen matrix of size (N,10) containing the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] information about N obstacles.
 *            x_c, y_c, phi are the position and orientation
 *            a1, a2 are the length of the half-axes of the ellipse
 *            p1, p2 are the power in the equation of the ellipse
 *            v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 */
void compute_quiver(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, Eigen::MatrixXf const& mat_obs);

/**
 * Create a grid of the workspace and compute the velocity command for each point of this grid for several elliptic obstacles with the direct_multiplication_2D function
 * Save data in a text file that will be opened with Python to plot a quiver graph with matplotlib
 *
 * @param limits Eigen matrix of size (5,1) containing the [x_min, x_max, y_min, y_max, step] information that defines the grid
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @param mat_obs Eigen matrix of size (N,10) containing the [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot] information about N obstacles.
 *            x_c, y_c, phi are the position and orientation
 *            a1, a2 are the length of the half-axes of the ellipse
 *            p1, p2 are the power in the equation of the ellipse
 *            v_x, v_y, w_rot are the linear and angular velocities of the ellipse
 * @see direct_multiplication_2D
 */
void compute_quiver_multiplication(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, Eigen::MatrixXf const& mat_obs);

/**
 * Return a velocity command whose linear and angular velocities have been limited for security reasons. The aim of this function is to limit the norm of the linear and angular velocities that are sent to the real robot. Limits are set in the function code.
 *
 * @param input_speed Eigen matrix of size (3,1) containing the (x_dot,y_dot,theta_dot) velocity command that will be sent to the robot
 * @return Eigen matrix of size (3,1) containing the input (x_dot,y_dot,theta_dot) velocity command whose linear and angular velocities may have been reduced if they were above the limits.
 */
State speed_limiter(State const& input_speed);


#endif // OBSTACLEAVOIDANCE_H_INCLUDED
