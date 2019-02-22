/*
Implementation on C++ by Pierre-Alexandre Léziart
Mail: TODO
Contains functions for obstacle avoidance algorithm
*/

// TODO : Add constant reference for functions in order to reduce computational cost
// TODO : Replace distance by .norm() of Eigen in "r_epsilon" function

#include "ObstacleAvoidance.h"

State f_epsilon(State state_robot, State state_attractor)
{
    State res = state_attractor - state_robot;
    return res;
}

float specific_radius(State state_robot, State center_point, State reference_point, Obstacle obs, int p) // compute specific radius
{
    // First step: compute state_surface which is the point on the surface of the obstacle that is on the segment between state_robot and reference_point
    State state_surf;
    // TODO
    // Second step: compute the gamma distance of the surface point
    flot gamma_surf = gamma( state_surf, center_point, reference_point, obs, p);
    return 1.0;
}

float gamma(State state_robot, State center_point, State reference_point, Obstacle obs, int p)
{
    float radius = specific_radius(state_robot, center_point, reference_point, obs, p);
    float sum = 0;
    for (int i=0; i < 2; i++)
    {
        // Here only the first and second components (x and y) has a weight for the gamma distance
        // The third component is the angle. How can you define the angle of an obstacle? Doesn't really make sense so we just use x and y.
        sum += std::pow( (state_robot(i,0) - center_point(i,0)) / radius, 2*p);
    }
    return sum;
}

float lambda_r(State state_robot, State center_point, State reference_point, Obstacle obs, int p)
{
    float res = 1 - (1/gamma(state_robot, center_point, reference_point, obs, p));
    return res;
}

float lambda_e(State state_robot, State center_point, State reference_point, Obstacle obs, int p)
{
    float res = 1 + (1/gamma(state_robot, center_point, reference_point, obs, p));
    return res;
}

Eigen::Matrix<float, number_states, number_states> D_epsilon(float lamb_r, float lamb_e)
{
    Eigen::Matrix<float, number_states, number_states> D = Eigen::Matrix<float, number_states, number_states>::Zero();
    D(0,0) = lamb_r;
    for (int i=1; i < number_states; i++)
    {
        D(i,i) = lamb_e;
    }
    return D;
}

State r_epsilon(State state_robot, State reference_point)
{
    State res = (state_robot - reference_point) / distance(state_robot, reference_point);
    return res;
}

float distance(State state1, State state2)
{
    State diff = state1 - state2;
    Eigen::Matrix<float, 1, 1> res = diff.colwise().norm(); // norm of the diff vector
    return res(0,0);
}

float partial_derivative(State state_robot, State center_point, State reference_point, Obstacle obs, int p, int direction) // partial derivative
{
    float h = 0.01; // Change to global variable?
    State state_h = State::Zero();
    state_h(direction,0) = h;

    float sum = 0;
    // Fourth order central difference
    sum += 1 * gamma(  2 * state_h + state_robot, center_point, reference_point, obs, p);
    sum -= 4 * gamma(  1 * state_h + state_robot, center_point, reference_point, obs, p);
    sum += 6 * gamma(  0 * state_h + state_robot, center_point, reference_point, obs, p);
    sum -= 4 * gamma( -1 * state_h + state_robot, center_point, reference_point, obs, p);
    sum += 1 * gamma( -2 * state_h + state_robot, center_point, reference_point, obs, p);
    sum /= pow(h,4); // Dividing by h^order

    return sum;
}

State gradient(State state_robot, State center_point, State reference_point, Obstacle obs, int p)
{
    State res;
    for (int i=0; i < number_states; i++)
    {
        float grad = partial_derivative( state_robot, center_point, reference_point, obs, p, i); // derivate along direction i
        res(i,0) = grad;
    }
    return res;
}

Eigen::Matrix<float, number_states, number_states-1> gram_schmidt(State gradient_vector)
{
    // I used this method to create the initial basis https://www.quora.com/How-do-I-find-the-matrix-of-orthonormal-base-vectors-of-a-hyperplane
    Eigen::Matrix<float, number_states, number_states-1> initial_basis = Eigen::Matrix<float, number_states, number_states-1>::Zero();

    // Warning: the first coordinate of gradient_vector has to be non-zero
    for (int i=0; i < (number_states-1); i++)
    {
        initial_basis(0,i)   =   gradient_vector(i+1,0);
        initial_basis(i+1,i) = - gradient_vector(  0,0);
    }

    // Then I applied Gram-Schmidt on the initial basis to get an orthonormal basis
    Eigen::Matrix<float, number_states, number_states-1> ortho_basis = Eigen::Matrix<float, number_states, number_states-1>::Zero();
    ortho_basis.col(0) = initial_basis.col(0) / initial_basis.col(0).norm();

    for (int i=1; i < (number_states-1); i++)
    {
        ortho_basis.col(i) = initial_basis.col(i);
        for (int j=i; j > 0; j--)
        {
            // Remove the projection on the vector already in ortho_basis to get a new orthogonal vector
            ortho_basis.col(i) -= projection(initial_basis.col(j), ortho_basis.col(j-1));
        }
        ortho_basis.col(i) = ortho_basis.col(i) / ortho_basis.col(i).norm(); // Normalization step
    }

    return ortho_basis;
}

State projection(State vec1, State vec2)
{
    float magnitude = vec1.dot(vec2); // norm of the projected vector
    // projected vector is magnitude * (vec2 normalized)
    State new_vec = vec2 * (magnitude / vec2.norm());
    return new_vec;
}

Eigen::Matrix<float, number_states, number_states> E_epsilon(State r_eps_vector, Eigen::Matrix<float, number_states, number_states-1> ortho_basis)
{
    Eigen::Matrix<float, number_states, number_states> E_eps;
    // Horizontal stack of r_eps_vector and ortho_basis to get [r_eps e_1 e_2 ... e_d-1]
    E_eps.col(0) = r_eps_vector;
    E_eps.block<number_states, number_states-1>(0,1) = ortho_basis;
    return E_eps;
}

Eigen::Matrix<float, number_states, number_states> M_epsilon(Eigen::Matrix<float, number_states, number_states> D_eps, Eigen::Matrix<float, number_states, number_states> E_eps)
{
    Eigen::Matrix<float, number_states, number_states> M_eps = E_eps * D_eps * E_eps.inverse();
    return M_eps;
}

State epsilon_dot(Eigen::Matrix<float, number_states, number_states> M_eps, State f_eps)
{
    State eps_dot = M_eps * f_eps;
    return eps_dot;
}

State next_step_single_obstacle(State state_robot, State state_attractor, State center_point, State reference_point, Obstacle obs, int p)
{
    // Compute attractor function
    State f_eps = f_epsilon( state_robot, state_attractor);

    // Several steps to get D(epsilon) matrix
    float lamb_r = lambda_r( state_robot, center_point, reference_point, obs, p); // optimization -> include gamma as argument of lambda_r and lambda_e
    float lamb_e = lambda_e( state_robot, center_point, reference_point, obs, p);
    Eigen::Matrix<float, number_states, number_states> D_eps = D_epsilon( lamb_r, lamb_e);

    // Several steps to get E(epsilon) matrix
    State r_eps_vector = r_epsilon( state_robot, reference_point);
    State gradient_vector = gradient( state_robot, center_point, reference_point, obs, p);
    Eigen::Matrix<float, number_states, number_states-1> ortho_basis = gram_schmidt( gradient_vector);
    Eigen::Matrix<float, number_states, number_states> E_eps = E_epsilon( r_eps_vector, ortho_basis);

    // Compute M(epsilon)
    Eigen::Matrix<float, number_states, number_states> M_eps = M_epsilon( D_eps, E_eps);

    // Compute epsilon_dot
    State velocity_next = epsilon_dot( M_eps, f_eps);

    return velocity_next;
}


