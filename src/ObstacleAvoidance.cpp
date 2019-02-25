/*
Implementation on C++ by Pierre-Alexandre Léziart
Mail: TODO
Contains functions for obstacle avoidance algorithm
*/

// DONE : Add constant reference for functions in order to reduce computational cost
// TODO : Replace distance by .norm() of Eigen in "r_epsilon" function
// DONE : Center point and reference point should be part of the Obstacle
// TODO : Solver may give the wrong solution (two solutions), add checking step
#include "ObstacleAvoidance.h"

State f_epsilon(State const& state_robot, State const& state_attractor)
{
    State res = state_attractor - state_robot;
    return res;
}

// For now the specific point is always the center of the ellipse
/*float specific_radius(State state_robot, State center_point, State reference_point, Obstacle obs, int p) // compute specific radius
{
    // First step: compute state_surface which is the point on the surface of the obstacle that is on the segment between state_robot and reference_point
    State state_surf;

    // Equation of the line that goes through state_robot and reference_point is y=a*x+b
    float a_line = (state_robot(1,0) - reference_point(1,0))/(state_robot(0,0) - reference_point(0,0)); // a = deltaY / deltaX
    float b_line = state_robot(1,0) - a_line * state_robot(0,0);
    std::cout << "a_line : " << a_line << " & b_line : " << b_line << std::endl;
    // Creating parameters vectors for the ellipse
    // params = [ x_c, y_c, phi, a1, a2, p1, p2, a_line, b_line, x_robot, y_robot]
    Eigen::Matrix<float, 11, 1> params_ellipse;
    if ((a_line > 1000000) || (a_line < -1000000)) // Avoid infinite coefficient
    {
        params_ellipse << obs, std::pow(10,9), 0.0, state_robot(0,0), state_robot(1,0); // add state_robot
    }
    else
    {
        params_ellipse << obs, a_line, b_line, state_robot(0,0), state_robot(1,0); // add state_robot
    }


    // Find the surface point that is also on the line
    state_surf = findSurfacePointEllipse(params_ellipse);
    std::cout << "Point on surface found = " << state_surf.transpose() << std::endl;
    // Compute the gamma distance of the surface point
    float gamma_surf = gamma( state_surf, center_point, reference_point, obs, p, true);
    return gamma_surf;
}*/

float gamma(State const& state_robot, Obstacle const& obs, bool is_radius)
{
    //float radius = 1;
    /*if (!is_radius) // Do not call specific_radius() when we call gamma() from specific_radius()
    {
        float radius = specific_radius(state_robot, center_point, reference_point, obs, p);
    }*/

    /*float sum = 0;
    for (int i=0; i < 2; i++)
    {
        // Here only the first and second components (x and y) has a weight for the gamma distance
        // The third component is the angle. How can you define the angle of an obstacle? Doesn't really make sense so we just use x and y.
        sum += std::pow( (state_robot(i,0) - center_point(i,0)) / radius, 2);
    }*/

    // Distance is obtained with the equation of the ellipse
    float delta_x = state_robot(0,0) - obs(0,0);
    float delta_y = state_robot(1,0) - obs(1,0);
    float angle = obs(2,0);
    float sum = (std::pow( (delta_x*std::cos(angle) + delta_y*std::sin(angle)) / obs(3,0) , 2 * obs(5,0))) + (std::pow( (delta_x*std::sin(angle) - delta_y*std::cos(angle)) / obs(4,0) , 2 * obs(6,0)));
    return sum;
}

float lambda_r(State const& state_robot, Obstacle const& obs)
{
    float res = 1 - (1/gamma(state_robot, obs));
    return res;
}

float lambda_e(State const& state_robot, Obstacle const& obs)
{
    float res = 1 + (1/gamma(state_robot, obs));
    return res;
}

Eigen::Matrix<float, number_states, number_states> D_epsilon(float const& lamb_r, float const& lamb_e)
{
    Eigen::Matrix<float, number_states, number_states> D = Eigen::Matrix<float, number_states, number_states>::Zero();
    D(0,0) = lamb_r;
    for (int i=1; i < number_states; i++)
    {
        D(i,i) = lamb_e;
    }
    return D;
}

State r_epsilon(State const& state_robot, Obstacle const& obs)
{
    State reference_point;
    reference_point << obs(0,0), obs(1,0), 0; // reference point is center point of the ellipse
    State res = (state_robot - reference_point) / distance(state_robot, reference_point);
    return res;
}

float distance(State const& state1, State const& state2)
{
    State diff = state1 - state2;
    Eigen::Matrix<float, 1, 1> res = diff.colwise().norm(); // norm of the diff vector
    return res(0,0);
}

float partial_derivative(State const& state_robot, Obstacle const& obs, int const& direction) // partial derivative
{
    // RUNGE-KUTTA 4 Method (has issue with float precision and division by h^4)
    /*float h = 0.01; // Change to global variable?
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

    std::cout << "Direction gradient : " << direction << std::endl;
    std::cout << "Coefficient 1 : " << gamma(  2 * state_h + state_robot, center_point, reference_point, obs, p) << std::endl;
    std::cout << "Coefficient 2 : " << gamma(  1 * state_h + state_robot, center_point, reference_point, obs, p) << std::endl;
    std::cout << "Coefficient 3 : " << gamma(  0 * state_h + state_robot, center_point, reference_point, obs, p) << std::endl;
    std::cout << "Coefficient 4 : " << gamma( -1 * state_h + state_robot, center_point, reference_point, obs, p) << std::endl;
    std::cout << "Coefficient 5 : " << gamma( -2 * state_h + state_robot, center_point, reference_point, obs, p) << std::endl;
    std::cout << "Divided weighted sum : " << sum << std::endl;

    return sum;*/

    // ANALYTICAL SOLUTION (for ellipses)
    float delta_x = state_robot(0,0) - obs(0,0);
    float delta_y = state_robot(1,0) - obs(1,0);
    float angle = obs(2,0);

    if (direction==0)
    {
        float val = ( 2 * obs(5,0) * std::cos(angle) / std::pow(obs(3,0) , 2 * obs(5,0)) ) * (std::pow( (delta_x*std::cos(angle) + delta_y*std::sin(angle)) , 2 * obs(5,0) - 1)) + ( 2 * obs(6,0) * std::sin(angle) / std::pow(obs(4,0) , 2 * obs(6,0)) ) * (std::pow( (delta_x*std::sin(angle) - delta_y*std::cos(angle)) , 2 * obs(6,0) - 1));
        //std::cout << "Analytical solution is " << val << std::endl;
        return val;
    }
    else if (direction==1)
    {
        float val = ( 2 * obs(5,0) * std::sin(angle) / std::pow(obs(3,0) , 2 * obs(5,0)) ) * (std::pow( (delta_x*std::cos(angle) + delta_y*std::sin(angle)) , 2 * obs(5,0) - 1)) - ( 2 * obs(6,0) * std::cos(angle) / std::pow(obs(4,0) , 2 * obs(6,0)) ) * (std::pow( (delta_x*std::sin(angle) - delta_y*std::cos(angle)) , 2 * obs(6,0) - 1));
        //std::cout << "Analytical solution is " << val << std::endl;
        return val;
    }
    else
    {
        return 0.0;
    }

}

State gradient(State const& state_robot, Obstacle const& obs)
{
    State res;
    for (int i=0; i < number_states; i++)
    {
        float grad = partial_derivative( state_robot, obs, i); // derivate along direction i
        res(i,0) = grad;
    }
    return res;
}

Eigen::Matrix<float, number_states, number_states-1> gram_schmidt(State const& gradient_vector)
{
    // I used this method to create the initial basis https://www.quora.com/How-do-I-find-the-matrix-of-orthonormal-base-vectors-of-a-hyperplane
    Eigen::Matrix<float, number_states, number_states-1> initial_basis = Eigen::Matrix<float, number_states, number_states-1>::Zero();

    // Warning: the first coordinate of gradient_vector has to be non-zero
    if (gradient_vector(0,0) != 0)
    {
       for (int i=0; i < (number_states-1); i++)
        {
            initial_basis(0,i)   =   gradient_vector(i+1,0);
            initial_basis(i+1,i) = - gradient_vector(  0,0);
        }
    }
    else
    {
        for (int i=0; i < (number_states-1); i++)
        {
            if (i==0)
            {
                initial_basis(0,0)  = - gradient_vector(  1,0);
                initial_basis(1,0)  =   gradient_vector(  0,0);
            }
            else
            {
                initial_basis(1,i)   =   gradient_vector(i+1,0);
                initial_basis(i+1,i) = - gradient_vector(  1,0);
            }
        }
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

State projection(State const& vec1, State const& vec2)
{
    float magnitude = vec1.dot(vec2); // norm of the projected vector
    // projected vector is magnitude * (vec2 normalized)
    State new_vec = vec2 * (magnitude / vec2.norm());
    return new_vec;
}

Eigen::Matrix<float, number_states, number_states> E_epsilon(State const& r_eps_vector, Eigen::Matrix<float, number_states, number_states-1> const& ortho_basis)
{
    Eigen::Matrix<float, number_states, number_states> E_eps;
    // Horizontal stack of r_eps_vector and ortho_basis to get [r_eps e_1 e_2 ... e_d-1]
    E_eps.col(0) = r_eps_vector;
    E_eps.block<number_states, number_states-1>(0,1) = ortho_basis;
    return E_eps;
}

Eigen::Matrix<float, number_states, number_states> M_epsilon(Eigen::Matrix<float, number_states, number_states> const& D_eps, Eigen::Matrix<float, number_states, number_states> const& E_eps)
{
    Eigen::Matrix<float, number_states, number_states> M_eps = E_eps * D_eps * E_eps.inverse();
    return M_eps;
}

State epsilon_dot(Eigen::Matrix<float, number_states, number_states> const& M_eps, State const& f_eps, State const& state_robot, Obstacle const& obs)
{
    State eps_tilde = state_robot - obs.block(0,0,3,1);
    State eps_dot_L; eps_dot_L << obs(7,0), obs(8,0), 0; // [v_x, v_y,     0]
    State eps_dot_R; eps_dot_R << 0, 0, obs(9,0);       // [  0,   0, w_rot]
    State eps_tilde_dot = eps_dot_L + eps_dot_R.cwiseProduct(eps_tilde);
    State eps_dot = M_eps * (f_eps - eps_tilde_dot) + eps_tilde_dot;
    //State eps_dot = M_eps * f_eps;
    return eps_dot;
}

State next_step_single_obstacle(State const& state_robot, State const& state_attractor, Obstacle const& obs)
{
    // Compute attractor function
    State f_eps = f_epsilon( state_robot, state_attractor);
    //std::cout << "f_eps=" << f_eps << std::endl;

    // Several steps to get D(epsilon) matrix
    float lamb_r = lambda_r( state_robot, obs); // optimization -> include gamma as argument of lambda_r and lambda_e
    float lamb_e = lambda_e( state_robot, obs);
    Eigen::Matrix<float, number_states, number_states> D_eps = D_epsilon( lamb_r, lamb_e);
    //std::cout << "lambda r=" << lamb_r << std::endl;
    //std::cout << "lambda e=" << lamb_e << std::endl;

    // Several steps to get E(epsilon) matrix
    State r_eps_vector = r_epsilon( state_robot, obs);

    // Remove tail effect
    if (r_eps_vector.dot((state_attractor-state_robot)) > 0)
    {
        return f_eps;
    }

    State gradient_vector = gradient( state_robot, obs);
    Eigen::Matrix<float, number_states, number_states-1> ortho_basis = gram_schmidt( gradient_vector);
    Eigen::Matrix<float, number_states, number_states> E_eps = E_epsilon( r_eps_vector, ortho_basis);
    //std::cout << "r_eps_vector=" << std::endl << r_eps_vector << std::endl;
    //std::cout << "gradient_vector=" << std::endl << gradient_vector << std::endl;
    //std::cout << "ortho_basis=" << std::endl << ortho_basis << std::endl;
    // std::cout << "E_epsilon=" << E_eps << std::endl;

    // Compute M(epsilon)
    Eigen::Matrix<float, number_states, number_states> M_eps = M_epsilon( D_eps, E_eps);

    // Compute epsilon_dot
    State velocity_next = epsilon_dot( M_eps, f_eps, state_robot, obs);

    return velocity_next;
}

// Functions to handle several obstacles

Eigen::MatrixXf velocities(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_obs)
{
    const int number_obstacles = mat_obs.cols();

    // mat_velocities is a matrix with size "number_states x number_obstacles"
    Eigen::MatrixXf mat_velocities(number_states, number_obstacles); // "3 x number_of_obstacles"

    for (int i=0; i < number_obstacles; i++)
    {
        mat_velocities.col(i) = next_step_single_obstacle(state_robot, state_attractor, mat_obs.col(i));
    }
    return mat_velocities;
}

// Eigen::MatrixXf mat_magnitudes( 1, number_obstacles);
// mat_magnitudes = mat_velocities.colwise().norm();
// Eigen::MatrixXf mat_norm_velocities( number_states, number_obstacles);
// mat_norm_velocities = mat.colwise().normalized();

Eigen::MatrixXf weights(State const& state_robot, Eigen::MatrixXf const& mat_obs)
{
    const int number_obstacles = mat_obs.cols();

    // mat_obs is a matrix with size "7 x number_obstacles"
    Eigen::MatrixXf mat_weights(1, number_obstacles); // "1 x number_of_obstacles"
    Eigen::MatrixXf mat_dist(1, number_obstacles);    // "1 x number_of_obstacles"
    Eigen::MatrixXf mat_prod(1, number_obstacles);    // "1 x number_of_obstacles"

    // Fill the mat_dist matrix (compute only once gamma(eps) for each obstacle for optimization purpose)
    for (int i=0; i < number_obstacles; i++)
    {
        mat_dist(0,i) = gamma(state_robot, mat_obs.col(i));
    }

    // Fill the mat_prod matrix (products of gamma_i(eps) - 1)
    for (int i=0; i < number_obstacles; i++)
    {
        float product = 1;

        for (int j=0; j < number_obstacles; j++)
        {
            if (j!=i)
            {
                product *= (mat_dist(0,j) - 1);
            }
        }
        mat_prod(0,i) = product;
    }

    // Fill the mat_weights matrix
    float denominator = 0;
    for (int j=0; j < number_obstacles; j++)
    {
        denominator += mat_prod(0,j);
    }

    for (int i=0; i < number_obstacles; i++)
    {
        mat_weights(0,i) = mat_prod(0,i) / denominator;
    }

    return mat_weights;
}

float weighted_magnitude(Eigen::MatrixXf const& mat_weights, Eigen::MatrixXf const& mat_magnitudes)
{
    const int number_obstacles = mat_weights.cols();
    float mag = 0;
    for (int i=0; i < number_obstacles; i++)
    {
        mag += mat_weights(0,i) * mat_magnitudes(0,i);
    }
    return mag;
}

State n_bar_2D(Eigen::MatrixXf const& mat_norm_velocities, Eigen::MatrixXf const& mat_weights)
{
    const int number_obstacles = mat_weights.cols();

    float weighted_angle = 0;
    for (int i=0; i < number_obstacles; i++)
    {
        weighted_angle += mat_weights(0,i) * std::atan2(mat_norm_velocities(1,i), mat_norm_velocities(0,i));
    }

    State res;
    res << std::cos(weighted_angle), std::sin(weighted_angle), 0;
    return res;
}
// final_velocity = mag_weighted * n_weighted;


State one_step_2D(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_obs)
{
    const int number_obstacles = mat_obs.cols();
    //std::cout << "1" << std::endl;
    Eigen::MatrixXf mat_velocities(number_states, number_obstacles); // "3 x number_of_obstacles"
    mat_velocities = velocities(state_robot, state_attractor, mat_obs);
    //std::cout << "2" << std::endl;
    Eigen::MatrixXf mat_magnitudes( 1, number_obstacles);
    mat_magnitudes = mat_velocities.colwise().norm();
    //std::cout << "3" << std::endl;
    Eigen::MatrixXf mat_norm_velocities( number_states, number_obstacles);
    mat_norm_velocities = mat_velocities.colwise().normalized();
    //std::cout << "4" << std::endl;
    Eigen::MatrixXf mat_weights(1, number_obstacles); // "1 x number_of_obstacles"
    mat_weights = weights( state_robot, mat_obs);
    //std::cout << "5" << std::endl;
    float weighted_mag = weighted_magnitude( mat_weights, mat_magnitudes);
    //std::cout << "6" << std::endl;
    State weighted_direction;
    weighted_direction = n_bar_2D( mat_norm_velocities, mat_weights);

    return (weighted_mag * weighted_direction);
}


// ---------
Eigen::Matrix<float, number_states, number_states> R_matrix(State const& f_eps)
{
    Eigen::Matrix<float, number_states, number_states> R_mat;
    R_mat.col(0) = f_eps.colwise().normalized();
    R_mat.col(1) << - R_mat(0,1), R_mat(0,0), 0;
    R_mat.col(2) << 0, 0, 1;
    return R_mat;
}

// mat_n_hat = R_mat * mat_norm_velocities

Eigen::MatrixXf kappa_matrix(Eigen::MatrixXf const& mat_n_hat)
{
    const int number_obstacles = mat_n_hat.cols();

    Eigen::MatrixXf mat_kappa(number_states-1, number_obstacles);

    for (int i=0; i < number_obstacles; i++)
    {
        float angle = std::acos(mat_n_hat(i,0));

        float denominator = 0;
        for (int j=1; i < number_states; i++)
        {
            denominator += mat_n_hat(i,j);
        }

        mat_kappa.col(i) = (angle/denominator) *  mat_n_hat.block(0, 1, number_states-1, 1);
    }
    return mat_kappa;
}

Eigen::MatrixXf n_bar_matrix(Eigen::MatrixXf const& mat_kappa_bar, Eigen::Matrix<float, number_states, number_states> const& R_mat)
{
    const int number_obstacles = mat_kappa_bar.cols();

    Eigen::MatrixXf mat_n_bar(number_states, number_obstacles); // "number_of_states x number_of_obstacles"

    Eigen::ArrayXf mat_temp(1, number_obstacles);
    mat_temp = mat_kappa_bar.colwise().norm();
    mat_n_bar.row(0) = mat_temp.cos();

    // FINISH MATRIX

    return mat_n_bar;
}

void update_obstacles(Eigen::MatrixXf & mat_obs, float const& time_step)
{
    const int number_obstacles = mat_obs.cols();

    mat_obs.block(0,0,3,number_obstacles) = mat_obs.block(0,0,3,number_obstacles) + time_step * mat_obs.block(7,0,3,number_obstacles);
}
