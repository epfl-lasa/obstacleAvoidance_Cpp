#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <Eigen> // For Linear Algebra
#include <array>

// INCLUDE HEADERS
//#include "Attractor.h"
#include "ObstacleAvoidance.h"
#include "ObstacleReconstruction.h"
//#include "SolverFunctions.h"

using namespace std;

// Global variables
float PI = 3.1415;

int test_functions();



void trajectories_border()
{
    ofstream myfile;
    ofstream myobs;
    cout << "-- Starting solver --" << endl;

    Blob obst(23,2);

    obst.row(0) << 6, 4;
    obst.row(1) << 7, 4;
    obst.row(2) << 8, 4;
    obst.row(3) << 9, 4;
    obst.row(4) << 6, 5;
    obst.row(5) << 9, 5;
    obst.row(6) << 6, 6;
    obst.row(7) << 8, 6;
    obst.row(8) << 9, 6;
    obst.row(9) << 6, 7;
    obst.row(10) << 8, 7;
    obst.row(11) << 10, 5;
    obst.row(12) << 11, 5;
    obst.row(13) << 8, 8;
    obst.row(14) << 8, 9;
    obst.row(15) << 8, 10;
    obst.row(16) << 11, 6;
    obst.row(17) << 11, 7;
    obst.row(18) << 12, 7;
    obst.row(19) << 13, 8;
    obst.row(20) <<  5, 6;
    obst.row(21) <<  5, 7;
    obst.row(22) <<  4, 8;

    myobs.open("./Trajectories/obs_data.txt"); //each starting point has its own file
    for (int i=0; i<obst.rows(); i++)
    {
        myobs << obst(i,0) << "," << obst(i,1) << "\n";
    }
    myobs.close();

    Point center_blob = get_center(obst);
    Border border_out;
    border_out = compute_border( obst, center_blob);

    State state_robot;
    State state_attractor;


    state_attractor << 17,6, 0;

    Eigen::Matrix<float, 5, 1> limits_quiver;
    limits_quiver << 0, 20, 0, 20, 0.1;

    int N_steps = 2;
    float time_step = 0.01;


    for (float y_k=-2; y_k <= 8; y_k+=0.2)
    {
        float time_stamp = 0.0;
        myfile.open("./Trajectories/trajectory_" + std::to_string(y_k) + ".txt"); //each starting point has its own file
        try
        {
            state_robot     << 0, y_k, 0;

            for (int i=0; i<N_steps; i++)
            {
                myfile << time_stamp << "," << state_robot(0,0) << "," << state_robot(1,0) << "," << state_robot(2,0) << "\n";
                State next_eps = next_step_single_obstacle_border( state_robot, state_attractor, border_out);
                state_robot += next_eps * time_step;
                time_stamp += time_step;
            }
        }
        catch (int e)
        {
            cout << "An exception occurred. Exception Nr. " << e << '\n';
        }
        myfile.close();

    }
    cout << "-- Completed --" << endl;
    cout << "-- Trajectory files closed --" << endl;
}

int main()
{
    //growing_obstacle();
    //trajectories_border();
    //growing_several_obstacle();
    if (true)
    {
        Grid occupancy_grid(6,10);
        occupancy_grid.row(0) << 0,0,0,0,0,0,0,0,0,0;
        occupancy_grid.row(1) << 0,0,1,1,0,0,0,0,0,0;
        occupancy_grid.row(2) << 0,0,1,1,0,0,0,0,0,0;
        occupancy_grid.row(3) << 0,0,0,0,0,0,0,0,0,0;
        occupancy_grid.row(4) << 0,0,0,0,0,1,1,1,0,0;
        occupancy_grid.row(5) << 0,0,0,0,0,0,0,0,0,0;

        std::vector<Border> storage;
        storage = detect_borders( occupancy_grid );
        for (int iter=0; iter<storage.size(); iter++)
        {
            cout << "Obstacle " << iter << ":"<< endl;
            cout << storage[iter] << endl;
        }

    }

    if (true)
    {
        Blob obst(23,2);
        Blob res;
        obst.row(0) << 6, 4;
        obst.row(1) << 7, 4;
        obst.row(2) << 8, 4;
        obst.row(3) << 9, 4;
        obst.row(4) << 6, 5;
        obst.row(5) << 9, 5;
        obst.row(6) << 6, 6;
        obst.row(7) << 8, 6;
        obst.row(8) << 9, 6;
        obst.row(9) << 6, 7;
        obst.row(10) << 8, 7;
        obst.row(11) << 10, 5;
        obst.row(12) << 11, 5;
        obst.row(13) << 8, 8;
        obst.row(14) << 8, 9;
        obst.row(15) << 8, 10;
        obst.row(16) << 11, 6;
        obst.row(17) << 11, 7;
        obst.row(18) << 12, 7;
        obst.row(19) << 13, 8;
        obst.row(20) <<  5, 6;
        obst.row(21) <<  5, 7;
        obst.row(22) <<  4, 8;


        /*res = fill_gaps(obst);
        cout << "With gaps:" << endl;
        cout << obst << endl;
        cout << "Without gaps:" << endl;
        cout << res << endl;*/

        //float margin = 1;
        Point center_blob = get_random(obst);
        Border border_out;
        border_out = compute_border( obst, center_blob);

        /*Point pt_test;
        pt_test.row(0) << 8, 6;
        cout << check_direction(res, pt_test, 3) << endl;*/
        /*cout << border_out.rows() << endl;
        for (int i=0; i<border_out.rows();i++)
        {
            cout << border_out.row(i) << endl;
        }*/

        display_border(obst, border_out);

        Eigen::Matrix<float,1,2>  robot; robot << 2,8;//3.51,6.51;
        Eigen::MatrixXf closest(1,6);
        closest = find_closest_point(robot, border_out);
        cout << "Closest is: " << closest << endl;

        Eigen::Matrix<float, 4, 1> gamma_ref;
        gamma_ref = gamma_and_ref_vector( robot, closest);
        cout << "GammaRefVec is: " << gamma_ref.transpose() << endl;

        State state_attractor; state_attractor << 17,6,0;

        State my_robot; my_robot << robot(0,0), robot(0,1), 0;
        State next_eps = next_step_single_obstacle_border( my_robot, state_attractor, border_out);
        cout << "Velocity command is: " << next_eps.transpose() << endl;

        Eigen::Matrix<float, 5, 1> limits_quiver;
        limits_quiver << 0, 20, 0, 20, 0.1;
        compute_quiver_border(limits_quiver, state_attractor, obst);
        //expand_obstacle(obst, 2);

        /*robot << 8, 6;//5.5, 5.5;
        closest << 7, 7, 3, 0.7, 1, 4;
        cout << (gamma_and_ref_vector(robot, closest)) << endl;

        robot << 8, 8;//5.5, 5.5;
        closest << 7, 7, 3, 0.7, 2, 4;
        cout << (gamma_and_ref_vector(robot, closest)) << endl;

        robot << 6, 8;//5.5, 5.5;
        closest << 7, 7, 3, 0.7, 3, 4;
        cout << (gamma_and_ref_vector(robot, closest)) << endl;

        robot << 6, 6;//5.5, 5.5;
        closest << 7, 7, 3, 0.7, 0, 4;
        cout << (gamma_and_ref_vector(robot, closest)) << endl;*/

    }

    return 0;
    cout << "Hello world!!!!" << endl;

    /*Eigen::Matrix<float, 3, 1> mat1; mat1 << 1, 2, 3;
    Eigen::Matrix<float, 3, 1> mat2; mat2 << 1, 2, 4;
    cout << mat1.cwiseProduct(mat2) << endl;*/
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
    ofstream myfile_obstacles;
    myfile_obstacles.open("obstacles_trajectory.txt");
    bool flag_obstacles = true;
    bool flag_quiver = true; // Set to false to disable Quiver computation
    int num_file = 0;
    cout << "-- Starting solver --" << endl;

    for (float y_k=-2; y_k <= 8; y_k+=0.2)
    {
        float time_stamp = 0.0;
        myfile.open("./Trajectories/trajectory_" + std::to_string(y_k) + ".txt"); //each starting point has its own file
        try
        {

            State state_robot;
            State state_attractor;
            Obstacle obs1, obs2, obs3, obs4, obs5;

            /*state_robot     << -2, y_k, 0;
            state_attractor <<  8, 4, 0;
            obs1 << 0, 0, (45*PI/180), 0.6, 1, 1, 1, 0, 2, (-60*PI/180); // [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]
            obs2 << 2, 2, 0, 1, 1, 1, 1, 0, 0, (60*PI/180); // [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]
            obs3 << 6, 6, 0, 1, 1, 1, 1, 0, 0, 0;
            obs4 << 4, 5, 0, 0.3, 1, 1, 1, 0, -2, (60*PI/180);
            obs5 << 8, -2, 0, 1, 0.7, 1, 1, -0.5, 2, (60*PI/180);

            Eigen::MatrixXf mat_obs(10,5);
            mat_obs.col(0) = obs1;
            mat_obs.col(1) = obs2;
            mat_obs.col(2) = obs3;
            mat_obs.col(3) = obs4;
            mat_obs.col(4) = obs5;*/

            state_robot     << 0, 0, 0;
            state_attractor << 5.9,4.9,0;
            Eigen::MatrixXf mat_obs(10,4); // [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]
            mat_obs.col(0) << -4, 7, 0, 1.6, 1.6, 1, 1, 0, 0, 0;
            mat_obs.col(1) <<  2, 2, 0, 1.6, 1.6, 1, 1, 0, 0, 0;
            mat_obs.col(2) <<  10, -2, 0, 1.6, 1.6, 1, 1, 0, 0, 0;
            mat_obs.col(3) <<  4, 0.5, 0, 1.6, 1.6, 1, 1, 0, 0, 0;

            Eigen::Matrix<float, 5, 1> limits_quiver;
            limits_quiver << -2, 6, -2, 5, 0.1;


            if (flag_quiver)
            {
                Eigen::Matrix<float, 2, 4> mat_points;
                mat_points.row(0) << 2, 2, 3, 3;
                mat_points.row(1) << 2, 3, 2, 3;

                //compute_quiver(limits_quiver, state_attractor, mat_obs);
                compute_quiver_multiplication(limits_quiver, state_attractor, mat_obs);
                //compute_quiver_polygon(limits_quiver, state_attractor, mat_points);
                flag_quiver = false;
            }
            int N_steps = 500;
            float time_step = 0.01;

            for (int i=0; i<N_steps; i++)
            {
                if (flag_obstacles)
                {
                    for (int j=0; j<mat_obs.cols(); j++) // We want to save all [x_c, y_c, phi, a1, a2, p1, p2]
                    {
                        myfile_obstacles << time_stamp;
                        for (int n_row=0; n_row < 7; n_row++)
                        {
                            myfile_obstacles << "," << mat_obs(n_row,j);
                        }
                        myfile_obstacles << "\n";
                    }
                }
                myfile << time_stamp << "," << state_robot(0,0) << "," << state_robot(1,0) << "," << state_robot(2,0) << "\n";
                //cout << "State robot: " << std::endl << state_robot << endl;
                //State next_eps = next_step_single_obstacle(state_robot, state_attractor, obs);
                State next_eps = one_step_2D( state_robot, state_attractor, mat_obs);
                //cout << "Next velocity: " << std::endl << next_eps << endl;
                state_robot += next_eps * time_step;
                update_obstacles( mat_obs, time_step); // update [x_c, y_c, phi] of all obstacles
                time_stamp += time_step;
            }
            flag_obstacles = false; // We need to write the trajectory of the obstacles only once
            myfile_obstacles.close();
            //cout << "Final position: " << state_robot(0,0) << "," << state_robot(1,0) << endl;
            //cout << "Final step: " << N_steps << endl;
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

// Lots of stuff that I used to test the functions when I was developing them, to be deleted for the final version
int test_functions()
{
    State state_robot;
    State state_attractor;
    Obstacle obs;

    state_robot     << -2, 0, 0;
    state_attractor << 2, 0, 0;
    obs << 0, 0, 0, 1, 1, 1, 1, 0, 0, 0; // [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]

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
    /*float res_gamma = 0.0;

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
    std::cout << "epsilon_dot=" << std::endl << velocity_next << std::endl;*/

    return 0;
}
