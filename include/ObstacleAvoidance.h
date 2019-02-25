#ifndef OBSTACLEAVOIDANCE_H_INCLUDED
#define OBSTACLEAVOIDANCE_H_INCLUDED

//#include "State.h"     // Import State class
//#include "Obstacle.h"  // Import Obstacle class
#include <iostream>
#include <Eigen> // For Linear Algebra
#include <cmath>
// #include "SolverFunctions.h" // For now the specific point is always the center of the ellipse so solver not needed


const int number_states = 3;
using State = Eigen::Matrix<float, number_states, 1>;
using Obstacle = Eigen::Matrix<float, 10, 1>; // [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]

State f_epsilon(State const& state_robot, State const& state_attractor); // compute attractor function

//float specific_radius(State state_robot, State center_point, State reference_point, Obstacle obs, int p); // compute specific radius

float gamma(State const& state_robot, Obstacle const& obs, bool is_radius=false); // compute gamma(epsilon) for an obstacle

float lambda_r(State const& state_robot, Obstacle const& obs); // for D(epsilon) matrix

float lambda_e(State const& state_robot, Obstacle const& obs); // for D(epsilon) matrix

Eigen::Matrix<float, number_states, number_states> D_epsilon( float const& lamb_r, float const& lamb_e); // create D(epsilon) matrix

State r_epsilon(State const& state_robot, Obstacle const& obs); // compute r(epsilon) vector for E(epsilon)

float distance(State const& state1, State const& state2); // compute distance between two states

float partial_derivative(State const& state_robot, Obstacle const& obs, int const& direction); // partial finite derivative of order 4

State gradient(State const& state_robot, Obstacle const& obs); // compute gradient of gamma(epsilon)

Eigen::Matrix<float, number_states, number_states-1> gram_schmidt(State const& gradient_vector); // orthogonal basis of an hyperplane with Gram-Schmidt

State projection(State const& vec1, State const& vec2); // projection of vec1 onto vec2

Eigen::Matrix<float, number_states, number_states> E_epsilon(State const& r_eps_vector, Eigen::Matrix<float, number_states, number_states-1> const& ortho_basis); // create E(epsilon) matrix

Eigen::Matrix<float, number_states, number_states> M_epsilon(Eigen::Matrix<float, number_states, number_states> const& D_eps, Eigen::Matrix<float, number_states, number_states> const& E_eps); // create M(epsilon) matrix

State epsilon_dot(Eigen::Matrix<float, number_states, number_states> const& M_eps, State const& f_eps, State const& state_robot, Obstacle const& obs); // compute epsilon_dot with M(epsilon) and f(epsilon)

State next_step_single_obstacle(State const& state_robot, State const& state_attractor, Obstacle const& obs); // use all previous functions to compute the next velocity state

// Functions to handle several obstacles

// Because the number of detected obstacles can vary I use dynamic-size matrices
Eigen::MatrixXf velocities(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_obs); // compute the next velocity for all obstacles

Eigen::MatrixXf weights(State const& state_robot, Eigen::MatrixXf const& mat_obs); // compute the weight of each obstacle

float weighted_magnitude(Eigen::MatrixXf const& mat_weights, Eigen::MatrixXf const& mat_magnitudes); // compute the magnitude of the epsilon velocity

State n_bar_2D(Eigen::MatrixXf const& mat_norm_velocities, Eigen::MatrixXf const& mat_weights); // compute the weighted directional interpolation (2D case)

State one_step_2D(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_obs);

Eigen::Matrix<float, number_states, number_states> R_matrix(State const& f_eps); // compute the R matrix to align the initial DS with the first axis

Eigen::MatrixXf kappa_matrix(Eigen::MatrixXf const& mat_n_hat); // compute the Kappa matrix

Eigen::MatrixXf n_bar_matrix(Eigen::MatrixXf const& mat_kappa_bar, Eigen::Matrix<float, number_states, number_states> const& R_mat); // compute n_bar with matrices R and kappa

// Other functions

void update_obstacles(Eigen::MatrixXf & mat_obs, float const& time_step);

#endif // OBSTACLEAVOIDANCE_H_INCLUDED
