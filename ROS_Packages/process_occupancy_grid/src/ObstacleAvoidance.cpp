/*
Implementation with C++ by Pierre-Alexandre Léziart
LASA laboratory, EPFL, Spring 2019
Mail: pierre-alexandre.leziart [at] ens-rennes [dot] fr
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

// For now the specific point is always the center of the ellipse so this function is not used
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
        // Here only the first and second components (x and y) have a weight for the gamma distance
        // The third component is the angle. How can you define the angle of an obstacle? Doesn't really make sense so we just use x and y.
        sum += std::pow( (state_robot(i,0) - center_point(i,0)) / radius, 2);
    }*/

    // Distance is obtained with the equation of the ellipse (simplification: reference point = center point)
    float delta_x = state_robot(0,0) - obs(0,0);
    float delta_y = state_robot(1,0) - obs(1,0);
    float angle = obs(2,0);
    // Inspired by the equation of the surface of an ellipse (for a point on the surface sum = 1)
    float sum = (std::pow( (delta_x*std::cos(angle) + delta_y*std::sin(angle)) / obs(3,0) , 2 * obs(5,0))) + (std::pow( (delta_x*std::sin(angle) - delta_y*std::cos(angle)) / obs(4,0) , 2 * obs(6,0)));
    return sum;
}

float lambda_r(State const& state_robot, Obstacle const& obs, float limit_distance)
{
    float res = 1.0; // default result
    if (method_weights == 1) // if method 1 is selected the standard formula is used
    {
       res = 1.0 - (1/gamma(state_robot, obs));
    }
    else if (limit_distance <= 1) // the limit distance must be greater than 1 because if it is not it means we have to be inside an ellipse obstacle in order to consider it (do not make sense)
    {
        throw std::invalid_argument( "Limit distance has to be greater than 1." );
    }
    else
    {
        float x = gamma(state_robot, obs); // gamma distance from the obstacle
        if (x < limit_distance) // if the distance is inferior to the limit distance we consider the obstacle
        {
            // We want lambda_r = 0 on the surface and lambda_r = 1 at the limit distance
            // Second order polynomial to get a derivative equals to zero at the limit_distance (lambda_r = a * (x - x_lim) + 1)
            float a = - 1 / std::pow(1 - limit_distance, 2);
            res = a * std::pow(x - limit_distance, 2) + 1;
        }
        // else 1.0 is returned (method 2 and obstacle too far so not considered)
    }
    return res;
}

float lambda_e(State const& state_robot, Obstacle const& obs, float limit_distance)
{
    // Comments for lambda_e are the same than for lambda_r (just a small change of formula)
    float res = 1.0;
    if (limit_distance == -1)
    {
       res = 1.0 + (1/gamma(state_robot, obs));
    }
    else if (limit_distance <= 1)
    {
        throw std::invalid_argument( "Limit distance has to be greater than 1." );
    }
    else
    {
        float x = gamma(state_robot, obs);
        if (x < limit_distance)
        {
            // Second order polynomial to get a derivative equals to zero at the limit_distance
            float a =  1 / std::pow(1 - limit_distance, 2);
            res = a * std::pow(x - limit_distance, 2) + 1;
        }
        // else 1.0 is returned
    }
    return res;
}

Eigen::Matrix<float, number_states, number_states> D_epsilon(float const& lamb_r, float const& lamb_e)
{
    // This matrix defined in the paper
    // Diagonal is [lambda_r lambda_e .... lambda_e]
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
    // This vector goes from the reference point of the obstacle to the robot with a norm-2 = 1
    State reference_point;
    reference_point << obs(0,0), obs(1,0), 0; // reference point is center point of the ellipse
    State res = (state_robot - reference_point) / distance(state_robot, reference_point);
    return res;
}

float distance(State const& state1, State const& state2)
{
    State diff = state1 - state2;
    Eigen::Matrix<float, 1, 1> res = diff.colwise().norm(); // norm-2 of the diff vector
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

    // To get this formulas : take this Latex equation \dfrac {((x-h)\cos(A)+(y-k)\sin(A))^2}{(a^2)}+\dfrac{((x-h) \sin(A)-(y-k) \cos(A))^2}{(b^2)}=1
    // where ℎ,k and a,b are the shifts and semi-axis in the x and y directions respectively and A is the angle measured from x axis.
    // Partial derivative of x for direction 0, partial derivate of y for direction 1
    if (direction==0)
    {
        float val = ( 2 * obs(5,0) * std::cos(angle) / std::pow(obs(3,0) , 2 * obs(5,0)) ) * (std::pow( (delta_x*std::cos(angle) + delta_y*std::sin(angle)) , 2 * obs(5,0) - 1)) + ( 2 * obs(6,0) * std::sin(angle) / std::pow(obs(4,0) , 2 * obs(6,0)) ) * (std::pow( (delta_x*std::sin(angle) - delta_y*std::cos(angle)) , 2 * obs(6,0) - 1));
        return val;
    }
    else if (direction==1)
    {
        float val = ( 2 * obs(5,0) * std::sin(angle) / std::pow(obs(3,0) , 2 * obs(5,0)) ) * (std::pow( (delta_x*std::cos(angle) + delta_y*std::sin(angle)) , 2 * obs(5,0) - 1)) - ( 2 * obs(6,0) * std::cos(angle) / std::pow(obs(4,0) , 2 * obs(6,0)) ) * (std::pow( (delta_x*std::sin(angle) - delta_y*std::cos(angle)) , 2 * obs(6,0) - 1));
        return val;
    }
    else if (direction==2) // no gradient according to phi, the angle of the robot
    {
        return 0.0;
    }
    else
    {
        throw std::invalid_argument( "This direction does not exist for the current implementation of the algorithm." );
    }
}

State gradient(State const& state_robot, Obstacle const& obs)
{
    State res;
    for (int i=0; i < number_states; i++) // for each dimension
    {
        float grad = partial_derivative( state_robot, obs, i); // derivative along dimension i (x, y or phi)
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
    else // If first coordinate is zero then we use the second vector as a reference. Can be improved by looping over all vectors (if first coordinate of second vector is also zero...)
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
    ortho_basis.col(0) = initial_basis.col(0) / initial_basis.col(0).norm(); // Normalization of the first vector

    for (int i=1; i < (number_states-1); i++)
    {
        ortho_basis.col(i) = initial_basis.col(i);
        for (int j=i; j > 0; j--)
        {
            // Remove the projection of the new vector on the vectors already in ortho_basis to get a new orthogonal vector
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
    // E matrix is defined in the paper
    Eigen::Matrix<float, number_states, number_states> E_eps;
    // Horizontal stack of r_eps_vector and ortho_basis to get [r_eps e_1 e_2 ... e_d-1]
    E_eps.col(0) = r_eps_vector;
    E_eps.block<number_states, number_states-1>(0,1) = ortho_basis;
    return E_eps;
}

Eigen::Matrix<float, number_states, number_states> M_epsilon(Eigen::Matrix<float, number_states, number_states> const& D_eps, Eigen::Matrix<float, number_states, number_states> const& E_eps)
{
    // M matrix is defined in the paper
    Eigen::Matrix<float, number_states, number_states> M_eps = E_eps * D_eps * E_eps.inverse();
    return M_eps;
}

State epsilon_dot(Eigen::Matrix<float, number_states, number_states> const& M_eps, State const& f_eps, State const& state_robot, Obstacle const& obs)
{
    // Version with moving obstacles (defined in the paper)
    State eps_tilde = state_robot - obs.block(0,0,3,1);
    State eps_dot_L; eps_dot_L << obs(7,0), obs(8,0), 0; // [v_x, v_y,     0]
    State eps_dot_R; eps_dot_R << 0, 0, obs(9,0);       // [  0,   0, w_rot]
    State eps_tilde_dot = eps_dot_L + eps_dot_R.cwiseProduct(eps_tilde);
    State eps_dot = M_eps * (f_eps - eps_tilde_dot) + eps_tilde_dot;

    // Version with non-moving obstacles (simplification)
    //State eps_dot = M_eps * f_eps;
    return eps_dot;
}

State next_step_single_obstacle(State const& state_robot, State const& state_attractor, Obstacle const& obs)
{
    // Compute all the steps for a single step and a single obstacle

    // Compute attractor function
    State f_eps = f_epsilon( state_robot, state_attractor);
    //std::cout << "f_eps=" << f_eps << std::endl;

    // Several steps to get D(epsilon) matrix
    float lamb_r = lambda_r( state_robot, obs, limit_dist); // optimization -> include gamma as argument of lambda_r and lambda_e (TODO?)
    float lamb_e = lambda_e( state_robot, obs, limit_dist);
    Eigen::Matrix<float, number_states, number_states> D_eps = D_epsilon( lamb_r, lamb_e);
    //std::cout << "lambda r=" << lamb_r << std::endl;
    //std::cout << "lambda e=" << lamb_e << std::endl;

    // Several steps to get E(epsilon) matrix
    State r_eps_vector = r_epsilon( state_robot, obs);

    // Remove tail effect (see the paper for a clean explanation)
    State tail_vector = (state_attractor-state_robot);
    if (r_eps_vector.dot(tail_vector.colwise().normalized()) > 0.3) // can be between 0 and 1, I chose > 0 because with thin ellipse I had some issues
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
        mat_velocities.col(i) = next_step_single_obstacle(state_robot, state_attractor, mat_obs.col(i)); // compute velocity command for each obstacle
    }
    return mat_velocities;
}

Eigen::MatrixXf weights(State const& state_robot, Eigen::MatrixXf const& mat_obs, int const& method, float const& limit_distance)
{
    const int number_obstacles = mat_obs.cols();

    // mat_obs is a matrix with size "7 x number_obstacles"
    Eigen::MatrixXf mat_weights(1, number_obstacles); // "1 x number_of_obstacles" one weight for each obstacle
    Eigen::MatrixXf mat_dist(1, number_obstacles);    // "1 x number_of_obstacles" one distance for each obstacle
    Eigen::MatrixXf mat_prod(1, number_obstacles);    // "1 x number_of_obstacles" one product for each obstacle

    // Fill the mat_dist matrix (compute only once gamma(eps) for each obstacle for optimization purpose)
    for (int i=0; i < number_obstacles; i++)
    {
        mat_dist(0,i) = gamma(state_robot, mat_obs.col(i));
    }

    if (method == 1) // Method described in the paper
    {
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
    }
    else if (method == 2) // Obstacles are not considered if they are too far from the robot
    {
        if (limit_distance == -1) {throw std::invalid_argument( "You have to set the limit distance to use this method." );}
        // Fill the mat_prod matrix (for this method it's actually filled by 1/(gamma_i-1), not by products)
        for (int i=0; i < number_obstacles; i++)
        {
            if (mat_dist(0,i) > limit_distance) // obstacle too far so it is not considered (0 weight)
            {
                mat_prod(0,i) = 0;
                //std::cout << i << " is out of range | ";
            }
            else if (mat_dist(0,i) > 1.01)
            {
                // Weight is infinity if the robot touches the surface of the obstacle and is 0 if the robot is at the limit distance
                // I used minus the natural log to get the +infinity with a shift to get 0 at the limit distance -log((x-1)/(xlim-1))
                mat_prod(0,i) = - std::log((mat_dist(0,i)-1)/(limit_distance-1));
                //std::cout << i << " is in the range | ";
            }
            else
            {
                mat_prod(0,i) = 100000; // very high number (problem with infinity when the distance is almost 1)
            }
        }
        //std::cout << std::endl;

    }
    else
    {
        throw std::invalid_argument( "No corresponding method for this number." );
    }

    // Fill the mat_weights matrix
    float denominator = 0;
    for (int j=0; j < number_obstacles; j++)
    {
        denominator += mat_prod(0,j);
    }

    // Relative importance of the weights to have them between [0,1]
    for (int i=0; i < number_obstacles; i++)
    {
        if (denominator == 0)
        {
            mat_weights(0,i) = 0;
        }
        else
        {
            mat_weights(0,i) = mat_prod(0,i) / denominator;
        }
    }

    return mat_weights;
}

float weighted_magnitude(Eigen::MatrixXf const& mat_weights, Eigen::MatrixXf const& mat_magnitudes)
{
    const int number_obstacles = mat_weights.cols();
    float mag = 0;
    for (int i=0; i < number_obstacles; i++)
    {
        mag += mat_weights(0,i) * mat_magnitudes(0,i); // weighted sum of the velocity commands of all obstacles
    }
    return mag;
}

State n_bar_2D(Eigen::MatrixXf const& mat_norm_velocities, Eigen::MatrixXf const& mat_weights)
{
    const int number_obstacles = mat_weights.cols();

    /*float weighted_angle = 0;
    for (int i=0; i < number_obstacles; i++)
    {
        weighted_angle += mat_weights(0,i) * std::atan2(mat_norm_velocities(1,i), mat_norm_velocities(0,i)); // simplification of the formula of the paper in the 2D case
        std::cout << "Angle: " << std::atan2(mat_norm_velocities(1,i), mat_norm_velocities(0,i)) << std::endl;
        std::cout << "Sum  : " << weighted_angle << std::endl;
    }*/
    // For instance if obstacle_0 makes the robot go east (0 angle) and obstacle_1 makes it go north (pi/2 angle),
    // then the robot will go in a direction between 0 and pi/2 depending on the relative distance of the obstacles

    Eigen::Matrix<float, 3, 1> sum_vectors; sum_vectors << 0, 0, 0;
    for (int i=0; i < number_obstacles; i++)
    {
        sum_vectors += mat_weights(0,i) * mat_norm_velocities.col(i); // simplification of the formula of the paper in the 2D case
    }

    State res;
    res = sum_vectors.colwise().normalized();
    //res << std::cos(weighted_angle), std::sin(weighted_angle), 0;
    return res;
}

State one_step_2D(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_obs)
{
    // Compute all the steps to get the velocity command considering several obstacles
    const int number_obstacles = mat_obs.cols();

    // Velocity command for each obstacle
    Eigen::MatrixXf mat_velocities(number_states, number_obstacles); // "3 x number_of_obstacles"
    mat_velocities = velocities(state_robot, state_attractor, mat_obs);

    // Norm of the velocity command for each obstacle
    Eigen::MatrixXf mat_magnitudes( 1, number_obstacles);
    mat_magnitudes = mat_velocities.colwise().norm();

    // Direction of the velocity command for each obstacle
    Eigen::MatrixXf mat_norm_velocities( number_states, number_obstacles);
    mat_norm_velocities = mat_velocities.colwise().normalized();

    // Relative weights of the obstacles depending on their distance from the robot
    Eigen::MatrixXf mat_weights(1, number_obstacles); // "1 x number_of_obstacles"
    mat_weights = weights( state_robot, mat_obs, method_weights, limit_dist);

    // Special case if all obstacles are too far away from the robot so none of them is considered
    if (mat_weights.maxCoeff() == 0)
    {
        // No obstacle in range so the robot just goes toward the attractor
        State cmd_velocity = f_epsilon( state_robot, state_attractor);
        cmd_velocity(2,0) = std::atan2(cmd_velocity(1,0),cmd_velocity(0,0)) - state_robot(2,0); // angle difference used for angular speed control (gain of 1)
        return speed_limiter(cmd_velocity);
    }

    // Magnitude of the weighted velocity command
    float weighted_mag = weighted_magnitude( mat_weights, mat_magnitudes);

    // Direction of the weighted velocity command
    State weighted_direction;
    weighted_direction = n_bar_2D( mat_norm_velocities, mat_weights);

    // Weighted velocity command (only for v_x and v_y as the angular speed is always 0 for our method). Ridgeback platform is holonomic so there is no problem with that
    State cmd_velocity = weighted_mag * weighted_direction;

    // Set the angular velocity to align the robot with the direction it moves (post process for better movement, thinner profile to go between obstacles)
    cmd_velocity(2,0) = std::atan2(cmd_velocity(1,0),cmd_velocity(0,0)) - state_robot(2,0); // angle difference used for angular speed control (gain of 1)

    /*std::cout << state_robot << std::endl;
    std::cout << state_attractor << std::endl;
    std::cout << mat_obs << std::endl;
    std::cout << mat_velocities << std::endl;
    std::cout << mat_norm_velocities << std::endl;
    std::cout << mat_weights << std::endl;
    std::cout << weighted_mag << std::endl;*/

    return speed_limiter(cmd_velocity);
}

State direct_multiplication_2D(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_obs)
{
    // Compute all the steps to get the velocity command considering several obstacles
    const int number_obstacles = mat_obs.cols();

    // Compute attractor function
    State velocity_next = f_epsilon( state_robot, state_attractor);

    for (int i_obs=0; i_obs < number_obstacles; i_obs++)
    {
        Obstacle obs = mat_obs.col(i_obs);

        // Several steps to get D(epsilon) matrix
        float lamb_r = lambda_r( state_robot, obs, limit_dist); // optimization -> include gamma as argument of lambda_r and lambda_e (TODO?)
        float lamb_e = lambda_e( state_robot, obs, limit_dist);
        Eigen::Matrix<float, number_states, number_states> D_eps = D_epsilon( lamb_r, lamb_e);

        // Several steps to get E(epsilon) matrix
        State r_eps_vector = r_epsilon( state_robot, obs);

        // Remove tail effect (see the paper for a clean explanation)
        /*State tail_vector = (state_attractor-state_robot);
        if (r_eps_vector.dot(tail_vector.colwise().normalized()) > 0.3) // can be between 0 and 1, I chose > 0 because with thin ellipse I had some issues
        {
            return f_eps;
        }*/

        State gradient_vector = gradient( state_robot, obs);
        Eigen::Matrix<float, number_states, number_states-1> ortho_basis = gram_schmidt( gradient_vector);
        Eigen::Matrix<float, number_states, number_states> E_eps = E_epsilon( r_eps_vector, ortho_basis);

        // Compute M(epsilon)
        Eigen::Matrix<float, number_states, number_states> M_eps = M_epsilon( D_eps, E_eps);

        // Compute epsilon_dot
        velocity_next = epsilon_dot( M_eps, velocity_next, state_robot, obs);
    }
    return velocity_next;
}

// 3 functions to use the weighting method in a d-dimensional case
// See the reference paper to understand them, I basically just implemented the formulas by playing a bit with matrices

Eigen::Matrix<float, number_states, number_states> R_matrix(State const& f_eps)
{
    // R matrix in the 3-dimensional case, need some work if you want it for the d-dimensional case
    Eigen::Matrix<float, number_states, number_states> R_mat;
    R_mat.col(0) = f_eps.colwise().normalized();
    R_mat.col(1) << - R_mat(0,1), R_mat(0,0), 0;
    R_mat.col(2) << 0, 0, 1;
    return R_mat;
}

// mat_n_hat = R_mat * mat_norm_velocities

Eigen::MatrixXf kappa_matrix(Eigen::MatrixXf const& mat_n_hat)
{
    //Kappa matrix, see the paper for the formula

    const int number_obstacles = mat_n_hat.cols();

    Eigen::MatrixXf mat_kappa(number_states-1, number_obstacles);

    for (int i=0; i < number_obstacles; i++)
    {
        float angle = std::acos(mat_n_hat(i,0)); // acos of the first vector

        float denominator = 0;
        for (int j=1; j < number_states; j++)
        {
            denominator += mat_n_hat(i,j); // denominator of the formula
        }

        mat_kappa.col(i) = (angle/denominator) *  mat_n_hat.block(0, 1, number_states-1, 1); // matrix with the vector except the first
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

// OTHER FUNCTIONS

void update_obstacles(Eigen::MatrixXf & mat_obs, float const& time_step)
{
    const int number_obstacles = mat_obs.cols();
    // I use the block command to take [x_c, y_c, phi] and to add it time_step * [v_x, v_y, w_rot]
    mat_obs.block(0,0,3,number_obstacles) = mat_obs.block(0,0,3,number_obstacles) + time_step * mat_obs.block(7,0,3,number_obstacles);
}

void compute_quiver(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, Eigen::MatrixXf const& mat_obs)
{
    // Compute the velocity command at the initial time for all points of a [x,y] grid
    // Save the result in a txt file and then use a Python script to plot the vector field (quiver) with matplotlib
    const int number_obstacles = mat_obs.cols();
    bool flag = false;

    std::ofstream myfile;
    myfile.open("quiver_data.txt");
    std::cout << "-- Quiver file opened --" << std::endl;
    for (float x=limits(0,0); x <= limits(1,0); x += limits(4,0)) // x direction of the grid
    {
        for (float y=limits(2,0); y <= limits(3,0); y += limits(4,0)) // y direction of the grid
        {
            //std::cout << x << " & " << y << std::endl;
            State state_robot; state_robot << x, y, 0; // the robot is set on the point of the grid
            flag = false;
            for (int i=0; i < number_obstacles; i++)
            {
                if (gamma(state_robot, mat_obs.col(i)) < 0.99)
                {
                    flag = true; // if the point/robot is inside an obstacle (distance < 1) then it makes no sense to compute its velocity command
                }
            }
            if (!flag) // if not inside an obstacle
            {
                State next_eps = one_step_2D( state_robot, state_attractor, mat_obs); // compute velocity command
                myfile << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file
            }
        }
    }
    myfile.close();
    std::cout << "-- Quiver file closed --" << std::endl;
}

void compute_quiver_multiplication(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, Eigen::MatrixXf const& mat_obs)
{
    // Compute the velocity command at the initial time for all points of a [x,y] grid
    // Save the result in a txt file and then use a Python script to plot the vector field (quiver) with matplotlib
    const int number_obstacles = mat_obs.cols();
    bool flag = false;

    std::ofstream myfile;
    myfile.open("./MultiplicationData/quiver_multiplication_data.txt");
    std::cout << "-- Quiver file opened --" << std::endl;
    for (float x=limits(0,0); x <= limits(1,0); x += limits(4,0)) // x direction of the grid
    {
        for (float y=limits(2,0); y <= limits(3,0); y += limits(4,0)) // y direction of the grid
        {
            //std::cout << x << " & " << y << std::endl;
            State state_robot; state_robot << x, y, 0; // the robot is set on the point of the grid
            flag = false;
            for (int i=0; i < number_obstacles; i++)
            {
                if (gamma(state_robot, mat_obs.col(i)) < 0.99)
                {
                    flag = true; // if the point/robot is inside an obstacle (distance < 1) then it makes no sense to compute its velocity command
                }
            }
            if (!flag) // if not inside an obstacle
            {
                State next_eps = direct_multiplication_2D( state_robot, state_attractor, mat_obs); // compute velocity command
                myfile << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file
            }
        }
    }
    myfile.close();
    std::cout << "-- Quiver file closed --" << std::endl;
}


State speed_limiter(State const& input_speed)
{
    const float limit_linear_speed = 0.05; // in meter/second
    const float limit_angular_speed = 5 * 0.01745; // in rad/second (0.01745 is to convert from degree to radian)

    State output_speed = input_speed;
    float norm_speed = std::sqrt(std::pow(output_speed(0,0),2) + std::pow(output_speed(1,0),2)); // speed of the robot
    if ( norm_speed > limit_linear_speed )
    {
        output_speed(0,0) = limit_linear_speed * output_speed(0,0) / norm_speed; // resize x direction
        output_speed(1,0) = limit_linear_speed * output_speed(1,0) / norm_speed; // resize y direction
    }
    output_speed(2,0) = std::max(std::min(output_speed(2,0), limit_angular_speed), -limit_angular_speed); // to be inside [-limit, +limit]
    return output_speed;
}


// Functions for polygons -> I adapted the functions used with ellipses to be able to use them with polygons defined by a list of (x,y) points

float polygon_gamma(State const& state_robot, Eigen::Matrix<float, 2, 1> closest_point)
{
    //Closest point is the closest point to the robot that is on the surface of the polygon
    float x_close = closest_point(0,0);
    float y_close = closest_point(1,0);

    float dist = 0;
    dist = 1 + std::pow(state_robot(0,0)-x_close,2) + std::pow(state_robot(1,0)-y_close,2); // minimum distance is 1 (robot touches the surface) then it goes to +infinity

    return dist;
}

void compute_quiver_polygon(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, Eigen::MatrixXf const& mat_points)
{
    // Compute quiver function but adapted for polygons
    const int number_points = mat_points.cols();
    bool flag = false;

    std::ofstream myfile;
    myfile.open("test_quiver_data.txt");
    std::cout << "-- Quiver test file opened --" << std::endl;
    for (float x=limits(0,0); x <= limits(1,0); x += limits(4,0))
    {
        for (float y=limits(2,0); y <= limits(3,0); y += limits(4,0))
        {
            //std::cout << x << " & " << y << std::endl;
            State state_robot; state_robot << x, y, 0;

            //float gamma_point = polygon_gamma( state_robot, mat_points);
            //myfile << x << "," << y << "," << gamma_point << "\n";

            State next_eps = polygon_next_step_single_obstacle( state_robot, state_attractor, mat_points);
            myfile << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n";

        }
    }
    myfile.close();
    std::cout << "-- Quiver test file closed --" << std::endl;
}

State polygon_next_step_single_obstacle(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_points)
{
    // Compute attractor function
    State f_eps = f_epsilon( state_robot, state_attractor);

    // Compute the closest point in the polygon
    Eigen::Matrix<float, 2, 1> closest_point;
    closest_point = polygon_get_closest( state_robot, mat_points);

    // Several steps to get D(epsilon) matrix
    float lamb_r = 1 - (1/polygon_gamma(state_robot, closest_point));
    float lamb_e = 1 + (1/polygon_gamma(state_robot, closest_point));
    Eigen::Matrix<float, number_states, number_states> D_eps = D_epsilon( lamb_r, lamb_e);

    // Several steps to get E(epsilon) matrix
    State r_eps_vector = polygon_r_epsilon( state_robot, closest_point);

    // Remove tail effect
    /*State tail_vector = (state_attractor-state_robot);
    if (r_eps_vector.dot(tail_vector.colwise().normalized()) > 0.3)
    {
        return f_eps;
    }*/

    State gradient_vector = polygon_gradient( state_robot, closest_point);
    Eigen::Matrix<float, number_states, number_states-1> ortho_basis = gram_schmidt( gradient_vector);
    Eigen::Matrix<float, number_states, number_states> E_eps = E_epsilon( r_eps_vector, ortho_basis);

    // Compute M(epsilon)
    Eigen::Matrix<float, number_states, number_states> M_eps = M_epsilon( D_eps, E_eps);

    // Compute epsilon_dot
    State velocity_next = M_eps * f_eps; // epsilon_dot( M_eps, f_eps, state_robot, obs);

    //std::cout << "f_eps=" << f_eps << std::endl;
    //std::cout << "lambda r=" << lamb_r << std::endl;
    //std::cout << "lambda e=" << lamb_e << std::endl;
    //std::cout << "r_eps_vector=" << std::endl << r_eps_vector << std::endl;
    //std::cout << "gradient_vector=" << std::endl << gradient_vector << std::endl;
    //std::cout << "ortho_basis=" << std::endl << ortho_basis << std::endl;
    // std::cout << "E_epsilon=" << E_eps << std::endl;

    return velocity_next;
}

State polygon_r_epsilon(State const& state_robot, Eigen::Matrix<float, 2, 1> closest_point)
{
    State reference_point;
    reference_point << closest_point(0,0), closest_point(1,0), 0; // reference point is center point of the ellipse
    State res = (state_robot - reference_point) / distance(state_robot, reference_point);
    return res;
}

float polygon_partial_derivative(State const& state_robot, Eigen::Matrix<float, 2, 1> const& closest_point, int const& direction) // partial derivative
{
    // ANALYTICAL SOLUTION
    float delta_x = state_robot(0,0) - closest_point(0,0);
    float delta_y = state_robot(1,0) - closest_point(1,0);
    float dist = std::sqrt(std::pow(delta_x,2)+std::pow(delta_y,2));
    float angle = std::atan2(delta_y, delta_x);

    // polygon_gamma distance increases with the square of the Euclidian distance "dist^2"
    // so the partial derivative along dist direction is 2*dist
    // the partial derivative is the direction which is orthogonal to the segment
    // so cos and sin are used to bring back the derivative to the x and y direction

    if (direction==0) // x direction
    {
        float val = 2 * dist * std::cos(angle);
        return val;
    }
    else if (direction==1) // y direction
    {
        float val = 2 * dist * std::sin(angle);
        return val;
    }
    else
    {
        return 0.0;
    }

}

State polygon_gradient(State const& state_robot, Eigen::Matrix<float, 2, 1> const& closest_point)
{
    State res;
    for (int i=0; i < number_states; i++)
    {
        float grad = polygon_partial_derivative( state_robot, closest_point, i); // derivate along direction i
        res(i,0) = grad;
    }
    return res;
}

Eigen::Matrix<float, 2, 1> polygon_get_closest(State const& state_robot, Eigen::MatrixXf const& mat_points)
{
    int number_points = mat_points.cols();
    // Distance between the robot and the points of the polygon (its corners)
    Eigen::MatrixXf mat_distance(1, number_points);
    for (int i=0; i < number_points; i++)
    {
        mat_distance(0,i) = std::pow(state_robot(0,0)-mat_points(0,i),2) + std::pow(state_robot(1,0)-mat_points(1,i),2);
    }
    // Get the two closest points
    float d_first = mat_distance(0,0);
    float i_first = 0;
    float d_second = mat_distance(0,1);
    float i_second = 1;
    if (d_second < d_first)
    {
        d_first = mat_distance(0,1);
        i_first = 1;
        d_second = mat_distance(0,0);
        i_second = 0;
    }
    for (int j=2; j < number_points; j++)
    {
        if (mat_distance(0,j)<d_first)
        {
            d_second = d_first;
            i_second = i_first;
            d_first = mat_distance(0,j);
            i_first = j;
        }
        else if (mat_distance(0,j)<d_second)
        {
            d_second = mat_distance(0,j);
            i_second = j;
        }
    }

    // Find the projection point on the segment
    float x_close = 0;
    float y_close = 0;
    if (std::abs(mat_points(1,i_first)-mat_points(1,i_second)) < 0.001 ) // a1 has to be non-zero (horizontal line)
    {
        x_close = state_robot(0,0);
        y_close = mat_points(1,i_first);
    } else if (std::abs(mat_points(0,i_first)-mat_points(0,i_second)) < 0.001 ) // avoid (x_first - x_second) = 0 (vertical line)
    {
        x_close = mat_points(0,i_first);
        y_close = state_robot(1,0);
    } else // If neither horizontal nor vertical line
    {
        float a1 = (mat_points(1,i_first)-mat_points(1,i_second))/(mat_points(0,i_first)-mat_points(0,i_second));
        float b1 = mat_points(1,i_first) - a1 * mat_points(0,i_first); // b = y - a * x
        float b2 = state_robot(1,0) + state_robot(0,0) / a1; // with y = a2 * x + b2 the equation of the line that goes through the robot and the point we want

        x_close = (b2 - b1)/(a1 + (1/a1));
        y_close = a1 * x_close + b1;
    }
    Eigen::Matrix<float, 2, 1> closest_point;
    closest_point << x_close, y_close;

    /*std::cout << "Robot (" << state_robot(0,0) << "," << state_robot(1,0) << ")" << std::endl;
    std::cout << "and closest (" << x_close << "," << y_close << ")" << std::endl;
    std::cout << "and first (" << mat_points(0,i_first) << "," << mat_points(1,i_first) << ")" << std::endl;
    std::cout << "and secon (" << mat_points(0,i_second) << "," << mat_points(1,i_second) << ")" << std::endl;*/

    return closest_point;
}
// Idea to smooth the rough changes of direction near the corners : interpolation at the angle between the walls -> ratio distance to second and third points??

// Test function when I was trying to call this library from ROS
float test_ros()
{
    return 42.0;
}

