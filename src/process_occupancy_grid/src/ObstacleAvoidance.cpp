/*
Implementation by Pierre-Alexandre Léziart of the method described in (Huber and al., 2019)
LASA laboratory, EPFL, Spring 2019
Mail: pierre-alexandre.leziart [at] epfl [dot] ch
Contains functions for obstacle avoidance algorithm
*/

#include "ObstacleAvoidance.h"

State f_epsilon(State const& state_robot, State const& state_attractor)
{
    State res = state_attractor - state_robot;
    return res;
}

float gamma(State const& state_robot, Obstacle const& obs, bool is_radius)
{
    // Here only the first and second components (x and y) have a weight for the gamma distance
    // The third component is the angle and is not used.
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
    float delta_x = state_robot(0,0) - obs(0,0);
    float delta_y = state_robot(1,0) - obs(1,0);
    float angle = obs(2,0);

    // ANALYTICAL SOLUTION FOR ELLIPTIC OBSTACLES
    // To get these formulas: take this Latex equation \dfrac {((x-h)\cos(A)+(y-k)\sin(A))^2}{(a^2)}+\dfrac{((x-h) \sin(A)-(y-k) \cos(A))^2}{(b^2)}=1
    // where h,k and a,b are the shifts and semi-axis in the x and y directions respectively and A is the angle measured from x axis.
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
    // E matrix is defined in the paper (Huber and al., 2019)
    Eigen::Matrix<float, number_states, number_states> E_eps;
    // Horizontal stack of r_eps_vector and ortho_basis to get [r_eps e_1 e_2 ... e_d-1]
    E_eps.col(0) = r_eps_vector;
    E_eps.block<number_states, number_states-1>(0,1) = ortho_basis;
    return E_eps;
}

Eigen::Matrix<float, number_states, number_states> M_epsilon(Eigen::Matrix<float, number_states, number_states> const& D_eps, Eigen::Matrix<float, number_states, number_states> const& E_eps)
{
    // M matrix is defined in the paper (Huber and al., 2019)
    Eigen::Matrix<float, number_states, number_states> M_eps = E_eps * D_eps * E_eps.inverse();
    return M_eps;
}

State epsilon_dot(Eigen::Matrix<float, number_states, number_states> const& M_eps, State const& f_eps, State const& state_robot, Obstacle const& obs)
{
    State eps_dot;

    bool moving_obstacles = false;
    if (moving_obstacles) // Version with moving obstacles, as defined in (Huber and al., 2019)
    {
        State eps_tilde = state_robot - obs.block(0,0,3,1);
        State eps_dot_L; eps_dot_L << obs(7,0), obs(8,0), 0; // [v_x, v_y,     0]
        State eps_dot_R; eps_dot_R << 0, 0, obs(9,0);        // [  0,   0, w_rot]
        State eps_tilde_dot = eps_dot_L + eps_dot_R.cwiseProduct(eps_tilde);
        eps_dot = M_eps * (f_eps - eps_tilde_dot) + eps_tilde_dot;
    }
    else // Version with non-moving obstacles (simplification)
    {
        eps_dot = M_eps * f_eps;
    }

    return eps_dot;
}

State next_step_single_obstacle(State const& state_robot, State const& state_attractor, Obstacle const& obs)
{
    // All steps to get the velocity command associated with a single elliptic obstacle

    // Compute attractor function
    State f_eps = f_epsilon( state_robot, state_attractor);

    // Several steps to get D(xi) matrix
    float lamb_r = lambda_r( state_robot, obs, limit_dist);
    float lamb_e = lambda_e( state_robot, obs, limit_dist);
    Eigen::Matrix<float, number_states, number_states> D_eps = D_epsilon( lamb_r, lamb_e);

    // Several steps to get E(xi) matrix
    State r_eps_vector = r_epsilon( state_robot, obs);

    // Remove tail effect (see the paper for a clean explanation)
    State tail_vector = (state_attractor-state_robot);
    if (r_eps_vector.dot(tail_vector.colwise().normalized()) > 0.3) // can be between 0 and 1, I chose > 0 because with thin ellipse I had some issues
    {
        return f_eps;
    }

    // Compute E(xi) matrix
    State gradient_vector = gradient( state_robot, obs);
    Eigen::Matrix<float, number_states, number_states-1> ortho_basis = gram_schmidt( gradient_vector);
    Eigen::Matrix<float, number_states, number_states> E_eps = E_epsilon( r_eps_vector, ortho_basis);

    // Compute M(xi) matrix
    Eigen::Matrix<float, number_states, number_states> M_eps = M_epsilon( D_eps, E_eps);

    // Compute xi_dot
    State velocity_next = epsilon_dot( M_eps, f_eps, state_robot, obs);

    return velocity_next;
}

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

    // mat_obs is a matrix with size "10 x number_obstacles"
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
        if (limit_distance == -1) {throw std::invalid_argument( "You have to set the limit_distance variable to use this method." );}

        // Fill the mat_prod matrix (for this method it's actually filled by 1/(gamma_i-1), not by products)
        for (int i=0; i < number_obstacles; i++)
        {
            if (mat_dist(0,i) > limit_distance) // obstacle too far so it is not considered (0 weight)
            {
                mat_prod(0,i) = 0;
            }
            else if (mat_dist(0,i) > 1.01)
            {
                // Weight is infinity if the robot touches the surface of the obstacle and is 0 if the robot is at the limit distance
                // I used minus the natural log to get the +infinity with a shift to get 0 at the limit distance -log((x-1)/(xlim-1))
                mat_prod(0,i) = - std::log((mat_dist(0,i)-1)/(limit_distance-1));
            }
            else
            {
                mat_prod(0,i) = 100000; // very high number (problem with infinity when the distance is almost 1)
            }
        }

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
        mag += mat_weights(0,i) * mat_magnitudes(0,i); // weighted sum of the magnitude of the velocity commands for all obstacles
    }
    return mag;
}

State n_bar_2D(Eigen::MatrixXf const& mat_norm_velocities, Eigen::MatrixXf const& mat_weights)
{
    const int number_obstacles = mat_weights.cols();

    // For instance if obstacle_0 makes the robot go east (0 angle) and obstacle_1 makes it go north (pi/2 angle),
    // then the robot will go in a direction between 0 and pi/2 depending on the relative distance of the obstacles

    Eigen::Matrix<float, 3, 1> sum_vectors; sum_vectors << 0, 0, 0;
    for (int i=0; i < number_obstacles; i++)
    {
        sum_vectors += mat_weights(0,i) * mat_norm_velocities.col(i); // simplification of the formula of the paper in the 2D case
    }

    State res;
    res = sum_vectors.colwise().normalized(); // Same as res << std::cos(weighted_angle), std::sin(weighted_angle), 0;
    return res;
}

State one_step_2D(State const& state_robot, State const& state_attractor, Eigen::MatrixXf const& mat_obs)
{
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
    const int number_obstacles = mat_obs.cols();

    // Compute attractor function (initial velocity command before deformation)
    State velocity_next = f_epsilon( state_robot, state_attractor);

    // For each obstacle
    for (int i_obs=0; i_obs < number_obstacles; i_obs++)
    {
        Obstacle obs = mat_obs.col(i_obs);

        // Several steps to get D(xi) matrix
        float lamb_r = lambda_r( state_robot, obs, limit_dist);
        float lamb_e = lambda_e( state_robot, obs, limit_dist);
        Eigen::Matrix<float, number_states, number_states> D_eps = D_epsilon( lamb_r, lamb_e);

        // Several steps to get E(xi) matrix
        State r_eps_vector = r_epsilon( state_robot, obs);
        State gradient_vector = gradient( state_robot, obs);
        Eigen::Matrix<float, number_states, number_states-1> ortho_basis = gram_schmidt( gradient_vector);
        Eigen::Matrix<float, number_states, number_states> E_eps = E_epsilon( r_eps_vector, ortho_basis);

        // Compute M(xi)
        Eigen::Matrix<float, number_states, number_states> M_eps = M_epsilon( D_eps, E_eps);

        // Compute xi_dot
        velocity_next = epsilon_dot( M_eps, velocity_next, state_robot, obs);
    }
    return velocity_next;
}

Eigen::Matrix<float, number_states, number_states> R_matrix(State const& f_eps)
{
    // R matrix in the 3-dimensional case, need some work if you want it for the d-dimensional case
    Eigen::Matrix<float, number_states, number_states> R_mat;
    R_mat.col(0) = f_eps.colwise().normalized();
    R_mat.col(1) << - R_mat(0,1), R_mat(0,0), 0;
    R_mat.col(2) << 0, 0, 1;
    return R_mat;
}

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

    // NOT FINISHED

    return mat_n_bar;
}

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
    const float limit_linear_speed = 0.15; // in meter/second
    const float limit_angular_speed = 10.0 * 0.01745; // in rad/second (0.01745 is to convert from degree to radian)

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

