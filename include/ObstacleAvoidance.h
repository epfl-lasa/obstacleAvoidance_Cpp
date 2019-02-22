#ifndef OBSTACLEAVOIDANCE_H_INCLUDED
#define OBSTACLEAVOIDANCE_H_INCLUDED

//#include "State.h"     // Import State class
//#include "Obstacle.h"  // Import Obstacle class
#include <Eigen> // For Linear Algebra
#include <cmath>
#include "SolverFunctions.h"


const int number_states = 3;
using State = Eigen::Matrix<float, number_states, 1>;
using Obstacle = Eigen::Matrix<float, 7, 1>; // [x_c, y_c, phi, a1, a2, p1, p2]

State f_epsilon(State state_robot, State state_attractor); // compute attractor function

float specific_radius(State state_robot, State center_point, State reference_point, Obstacle obs, int p); // compute specific radius

float gamma(State state_robot, State center_point, State reference_point, Obstacle obs, int p, bool is_radius=false); // compute gamma(epsilon) for an obstacle

float lambda_r(State state_robot, State center_point, State reference_point, Obstacle obs, int p); // for D(epsilon) matrix

float lambda_e(State state_robot, State center_point, State reference_point, Obstacle obs, int p); // for D(epsilon) matrix

Eigen::Matrix<float, number_states, number_states> D_epsilon( float lamb_r, float lamb_e); // create D(epsilon) matrix

State r_epsilon(State state_robot, State reference_point); // compute r(epsilon) vector for E(epsilon)

float distance(State state1, State state2); // compute distance between two states

float partial_derivative(State state_robot, State center_point, State reference_point, Obstacle obs, int p, int direction); // partial finite derivative of order 4

State gradient(State state_robot, State center_point, State reference_point, Obstacle obs, int p); // compute gradient of gamma(epsilon)

Eigen::Matrix<float, number_states, number_states-1> gram_schmidt(State gradient_vector); // orthogonal basis of an hyperplane with Gram-Schmidt

State projection(State vec1, State vec2); // projection of vec1 onto vec2

Eigen::Matrix<float, number_states, number_states> E_epsilon(State r_eps_vector, Eigen::Matrix<float, number_states, number_states-1> ortho_basis); // create E(epsilon) matrix

Eigen::Matrix<float, number_states, number_states> M_epsilon(Eigen::Matrix<float, number_states, number_states> D_eps, Eigen::Matrix<float, number_states, number_states> E_eps); // create M(epsilon) matrix

State epsilon_dot(Eigen::Matrix<float, number_states, number_states> M_eps, State f_eps); // compute epsilon_dot with M(epsilon) and f(epsilon)

State next_step_single_obstacle(State state_robot, State state_attractor, State center_point, State reference_point, Obstacle obs, int p); // use all previous functions to compute the next velocity state


#endif // OBSTACLEAVOIDANCE_H_INCLUDED
