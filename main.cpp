#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <Eigen> // For Linear Algebra
#include <array>

// Global variables
#define PI 3.1415;

// INCLUDE HEADERS
#include "Attractor.h"
#include "ObstacleAvoidance.h"
//#include "SolverFunctions.h"

using namespace std;

int test_functions();

int main()
{

    cout << "Hello world!" << endl;

    /*Eigen::MatrixXf mat(3,4);
    mat.col(0) << 1, 1, 1;
    mat.col(1) << 2, 2, 2;
    mat.col(2) << 0, 1, 0;
    mat.col(3) << 1, 2, 1;
    cout << "Mat : " << endl << mat << endl;

    Eigen::MatrixXf mat_bis(3,4);
    mat_bis = mat.colwise().normalized();
    cout << "Mat_bis : " << endl << mat_bis << endl;*/

    //test_functions();

    /*
    State s1, s2;
    s1 << 2, 2, 2;
    s2 << 1, 2, 2;
    cout << s1 << endl;

    cout << distance(s1, s2) << endl;
    cout << Eigen::Matrix<float, number_states, number_states>::Zero() << endl;
    State state_h = State::Zero();
    cout << state_h << endl;

    cout << "Colum: " << s1 << endl;
    cout << "Norm: "  << s1.norm() << endl;*/

    /*Eigen::Matrix<float,9,1> test;
    test << s1, s2, s1;
    cout << "Test:" << endl << test << endl;*/

    /*Eigen::Matrix<float,9,1> params_ellipse;
    params_ellipse << 1,2,3,4,5,6,7,8,9;
    findSurfacePointEllipse(params_ellipse);*/

    /*Obstacle obs1({{1.0,1.0}},{{1.0,1.0}},{{10.0,-8.0}},-40/180*PI,1.0,{{0.0, 0.0}},0.0,1.0,0.0);
    obs1.disp_params();

    Attractor att1(1.2, 2.3);
    att1.disp_params();

    Eigen::Matrix<int, 2, 3> mat1;
    mat1 << 1, 2, 3, 4, 5, 6;
    cout << mat1 << endl;
    mat1(1,2) = 9;
    cout << mat1;

    std::array<Obstacle, 2> test;
    test[0].disp_params();
    test[0] = obs1;
    test[1] = obs1;
    test[0].disp_params();*/

    /*State s1(1.1, 2.2, 3.3);
    s1.disp_params();
    State s2(s1);
    s2.disp_params();
    State s3 = s1;
    s3.disp_params();

    s3 = s1.sumStates(s2);
    s3.disp_params();
    State s4(s3);
    s4 = s1 + s3;
    s4.disp_params();
    s4 = s4 - s1;
    s4.disp_params();*/

    // ------------ TRAJECTORY GENERATION

    ofstream myfile;
    int num_file = 0;
    cout << "-- Starting solver --" << endl;
    for (float k=-2; k <= 4; k+=0.2)
    {
        myfile.open("trajectory_" + std::to_string(k) + ".txt");
        try
        {

            State state_robot;
            State state_attractor;
            Obstacle obs1; Obstacle obs2;

            state_robot     << -2, k, 0;
            state_attractor <<  4, 2, 0;
            obs1 << 0, 0, 0, 1, 1, 1, 1; // [x_c, y_c, phi, a1, a2, p1, p2]
            obs2 << 2, 2, 0, 1, 1, 1, 1; // [x_c, y_c, phi, a1, a2, p1, p2]

            Eigen::MatrixXf mat_obs(7,2);
            mat_obs.col(0) = obs1;
            mat_obs.col(1) = obs2;

            int N_steps = 500;
            float time_step = 0.01;

            for (int i=0; i<N_steps; i++)
            {
                myfile << state_robot(0,0) << "," << state_robot(1,0) << "," << state_robot(2,0) << "\n";
                //cout << "State robot: " << std::endl << state_robot << endl;
                //State next_eps = next_step_single_obstacle(state_robot, state_attractor, obs);
                State next_eps = one_step_2D( state_robot, state_attractor, mat_obs);
                //cout << "Next velocity: " << std::endl << next_eps << endl;
                state_robot += next_eps * time_step;
            }
        }
        catch (int e)
        {
            cout << "An exception occurred. Exception Nr. " << e << '\n';
        }
        myfile.close();
        num_file++;
    }
    cout << "-- Completed --" << endl;
    cout << "-- Trajectory files closed --" << endl;
    return 0;
}

int test_functions()
{
    State state_robot;
    State state_attractor;
    Obstacle obs;

    state_robot     << -2, 0, 0;
    state_attractor << 2, 0, 0;
    obs << 0, 0, 0, 1, 1, 1, 1; // [x_c, y_c, phi, a1, a2, p1, p2]

    // TEST F_EPSILON
    /* State f_eps = f_epsilon( state_robot, state_attractor);
    std::cout << "f_eps=" << std::endl << f_eps << std::endl; */

    // TEST SPECIFIC RADIUS
    /*float radius = 0;

    state_robot     << -2, 0, 0;
    std::cout << "state_robot = " << state_robot.transpose() << std::endl;
    radius = specific_radius(state_robot, center_point, reference_point, obs, p);
    std::cout << "specific radius = " << radius << std::endl;

    state_robot     << 0, 2, 0;
    std::cout << "state_robot = " << state_robot.transpose() << std::endl;
    radius = specific_radius(state_robot, center_point, reference_point, obs, p);
    std::cout << "specific radius = " << radius << std::endl;

    state_robot     << -1, 1, 0;
    std::cout << "state_robot = " << state_robot.transpose() << std::endl;
    radius = specific_radius(state_robot, center_point, reference_point, obs, p);
    std::cout << "specific radius = " << radius << std::endl;

    state_robot     << 0, 2, 0;
    reference_point << 0, 0.5, 0;
    std::cout << "state_robot = " << state_robot.transpose() << std::endl;
    radius = specific_radius(state_robot, center_point, reference_point, obs, p);
    std::cout << "specific radius = " << radius << std::endl;*/

    // TEST GAMMA
    float res_gamma = 0.0;

    state_robot     << -2, 0, 0;
    res_gamma = gamma(state_robot, obs);
    std::cout << "state_robot = " << state_robot.transpose() << std::endl;
    std::cout << "res_gamma = " << res_gamma << std::endl;

    state_robot     << 0, 2, 0;
    res_gamma = gamma(state_robot, obs);
    std::cout << "state_robot = " << state_robot.transpose() << std::endl;
    std::cout << "res_gamma = " << res_gamma << std::endl;

    state_robot     << 0, 1, 0;
    res_gamma = gamma(state_robot, obs);
    std::cout << "state_robot = " << state_robot.transpose() << std::endl;
    std::cout << "res_gamma = " << res_gamma << std::endl;

    // TEST D_EPSILON

    state_robot     << -2, 0, 0;

    std::cout << "state_robot = " << state_robot.transpose() << std::endl;
    float lamb_r = lambda_r( state_robot, obs); // optimization -> include gamma as argument of lambda_r and lambda_e
    float lamb_e = lambda_e( state_robot, obs);
    Eigen::Matrix<float, 3, 3> D_eps = D_epsilon( lamb_r, lamb_e);
    std::cout << "lambda r=" << lamb_r << std::endl;
    std::cout << "lambda e=" << lamb_e << std::endl;
    std::cout << "D_eps =" << std::endl << D_eps << std::endl;

    // TEST E_EPSILON
    std::cout << std::endl;
    const int number_states = 3;
    State r_eps_vector = r_epsilon( state_robot, obs);
    State gradient_vector = gradient( state_robot, obs);
    Eigen::Matrix<float, number_states, number_states-1> ortho_basis = gram_schmidt( gradient_vector);
    Eigen::Matrix<float, number_states, number_states> E_eps = E_epsilon( r_eps_vector, ortho_basis);
    std::cout << "r_eps_vector=" << std::endl << r_eps_vector << std::endl;
    std::cout << "gradient_vector=" << std::endl << gradient_vector << std::endl;
    std::cout << "ortho_basis=" << std::endl << ortho_basis << std::endl;
    std::cout << "E_epsilon=" << std::endl << E_eps << std::endl;

    // TEST M_EPSILON and EPSILON_DOT
    std::cout << std::endl;
    Eigen::Matrix<float, number_states, number_states> M_eps = M_epsilon( D_eps, E_eps);
    std::cout << "M_epsilon=" << std::endl << M_eps << std::endl;
    State f_eps = f_epsilon( state_robot, state_attractor);
    State velocity_next = epsilon_dot( M_eps, f_eps);
    std::cout << "epsilon_dot=" << std::endl << velocity_next << std::endl;

    return 0;
}
