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
#include "SolverFunctions.h"

using namespace std;

int main()
{
    cout << "Hello world!" << endl;

    State s1, s2;
    s1 << 2, 2, 2;
    s2 << 1, 2, 2;
    cout << s1 << endl;

    cout << distance(s1, s2) << endl;
    cout << Eigen::Matrix<float, number_states, number_states>::Zero() << endl;
    State state_h = State::Zero();
    cout << state_h << endl;

    cout << "Colum: " << s1 << endl;
    cout << "Norm: "  << s1.norm() << endl;

    /*Eigen::Matrix<float,9,1> test;
    test << s1, s2, s1;
    cout << "Test:" << endl << test << endl;*/

    Eigen::Matrix<float,9,1> params_ellipse;
    params_ellipse << 1,2,3,4,5,6,7,8,9;
    findSurfacePointEllipse(params_ellipse);
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

    ofstream myfile;
    myfile.open("trajectory.txt");
    try
    {

        State state_robot;
        State state_attractor;
        State center_point;
        State reference_point;
        Obstacle obs;
        int p = 1;

        state_robot     << -2, 0, 0;
        state_attractor << 2, 0, 0;
        center_point    << 0, 0, 0;
        reference_point << 0, 0, 0;
        obs << 0, 0, 0, 1, 1, 1, 1;
        int N_steps = 20;
        float time_step = 0.01;

        for (int i=0; i<N_steps; i++)
        {
            myfile << state_robot(0,0) << "," << state_robot(1,0) << "," << state_robot(2,0) << "\n";
            cout << "State robot: " << std::endl << state_robot << endl;
            State next_eps = next_step_single_obstacle(state_robot, state_attractor, center_point, reference_point, obs, p);
            cout << "Next velocity: " << std::endl << next_eps << endl;
            state_robot += next_eps * time_step;
        }
    }
    catch (int e)
    {
        cout << "An exception occurred. Exception Nr. " << e << '\n';
    }
    myfile.close();
    return 0;
}
