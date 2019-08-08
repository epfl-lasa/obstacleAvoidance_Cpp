#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <Eigen> // For Linear Algebra
#include <array>

// INCLUDE HEADERS
#include "BezierInterpolation.h"
#include "ObstacleAvoidance.h"
#include "ObstacleReconstruction.h"

#include <chrono>
#include <ctime>

#include <cstdlib>

using namespace std;

// Global variables
float PI = 3.1415;

int test_functions();         // Lots of stuff that I used to test the functions when I was developing them, to be deleted for the final version
void trajectory_generation(); // Old function to plot trajectories with ellipses obstacles
void plot_stream_gazebo();    // Create data to plot a stream with matplotlib for a given set of obstacles


// float limit_dist;

void test_draw_circle()
{
    Grid occupancy = Grid::Zero(14,16);
    draw_circle(occupancy, 7, 7, 5);
    cout << occupancy << endl;
}

void trajectory_comparison()
{
    float size_cell = 1;
    // Num 0
    /*Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(3) << 0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(4) << 0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(5) << 0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(6) << 0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(7) << 0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(8) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;*/
    // Num 1
    Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(3) << 0,1,1,1,1,1,1,1,1,1,0;
    occupancy.row(4) << 0,1,0,0,0,1,0,0,0,1,0;
    occupancy.row(5) << 0,1,0,0,0,1,0,0,0,1,0;
    occupancy.row(6) << 0,1,0,0,0,1,0,0,0,1,0;
    occupancy.row(7) << 0,1,0,0,0,1,0,0,0,1,0;
    occupancy.row(8) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;

    occupancy *= 100;

    int num = 1;

    // State of the robot
    State state_robot; state_robot << 5.1, 10.0, 0;

    // Detect expanded obstacles
    std::vector<Border> storage;
    storage = detect_borders( occupancy, state_robot);

    std::ofstream mycells;
    mycells.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+std::to_string(num)+"_cells.txt");
    for (int i=0; i<occupancy.rows(); i++)
    {
        for (int j=0; j<occupancy.cols(); j++)
        {
            if (occupancy(i,j)==100) {mycells << i << "," << j << "\n";}
        }
    }
    mycells.close();

    std::ofstream myobs;
    myobs.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+std::to_string(num)+"_obs.txt");
    for (int iter=0; iter<storage.size(); iter++)
    {
        Border border_out = storage[iter];
        for (int i=0; i<border_out.rows(); i++)
        {
            myobs << border_out(i,0) << "," << border_out(i,1) << "," << border_out(i,2) << "," << border_out(i,3) << "," << border_out(i,4) << "," << iter << "\n";
        }
    }
    myobs.close();

    std::ofstream mystream, mystream_bezier, mystream_classic;
    mystream.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+std::to_string(num)+"_circle.txt");
    mystream.close();
    mystream.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+std::to_string(num)+"_corrected_circle.txt");
    mystream.close();
    mystream.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+std::to_string(num)+"_params.txt");
    mystream.close();
    mystream.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+std::to_string(num)+"_corrected_circle_formula.txt");
    mystream.close();

    // Position of the attractor
    State state_attractor;
    state_attractor << 5, 0, 0;

    int N_steps = 5000;
    float time_step = 0.005; // time_step
    State robot_initial; robot_initial = state_robot;

    mystream.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+std::to_string(num)+"_normal.txt");

    for (int i=0; i<N_steps; i++)
    {
        if (std::remainder(i,100)==0) {std::cout << i << std::endl;}
        State next_eps = next_step_special_weighted( robot_initial, state_attractor, storage, size_cell); // compute special velocity with transform in circle space
        mystream << robot_initial(0,0) << "," << robot_initial(1,0) << "\n"; // write position of the point in the initial space
        robot_initial += next_eps * time_step;
    }

    mystream.close();

    std::cout << " ###### END OF STEP 1 - STARTING STEP 2 ###### " << std::endl;

    robot_initial = state_robot;

    mystream.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+std::to_string(num)+"_corrected.txt");

    for (int i=0; i<N_steps; i++)
    {
        if (std::remainder(i,100)==0) {std::cout << i << std::endl;}
        State next_eps = next_step_special_weighted( robot_initial, state_attractor, storage, size_cell, true); // compute special velocity with transform in circle space
        mystream << robot_initial(0,0) << "," << robot_initial(1,0) << "\n"; // write position of the point in the initial space
        robot_initial += next_eps * time_step;
    }

    mystream.close();

    std::cout << " ###### END OF STEP 2 - STARTING STEP 3 ###### " << std::endl;

    mystream.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/TrajectoryComparisonData/traj_data_"+std::to_string(num)+"_only_circle.txt");

    // Initialization
    Eigen::MatrixXf closest(1,6);
    Eigen::MatrixXf closest_attractor(1,6);
    Eigen::Matrix<float, 6, 1> gamma_norm_proj;
    Eigen::Matrix<float, 6, 1> gamma_norm_proj_attractor;
    State robot_vec;
    State normal_vec;

    Eigen::Matrix<float, 1, 2> robot = state_robot.block(0,0,2,1).transpose();
    Eigen::Matrix<float, 1, 2> attractor = state_attractor.block(0,0,2,1).transpose();

    Border border = storage[0];

    // Get closest cell for the robot
    closest = find_closest_point(robot, border);

    // Get gamma distance + normal vector + point projected on border for the robot
    gamma_norm_proj = gamma_normal_projection( robot, closest);

    // Get closest cell for the attractor
    closest_attractor = find_closest_point(attractor, border);

    // Get gamma distance + normal vector + point projected on border for the attractor
    gamma_norm_proj_attractor = gamma_normal_projection( attractor, closest_attractor);

    // To return NaN for points inside obstacle
    robot_vec << (robot(0,0)-gamma_norm_proj(4,0)),(robot(0,1)-gamma_norm_proj(5,0)),0;
    normal_vec << gamma_norm_proj(1,0),gamma_norm_proj(2,0),0;

    // Get minimal distance along surface (either by following the surface clockwise or counter-clockwise)
    // Distance between the projected point of the robot and the projected point of the attractor
    Eigen::Matrix<float, 1, 2> proj_robot;     proj_robot     << gamma_norm_proj(4,0), gamma_norm_proj(5,0);
    Eigen::Matrix<float, 1, 2> proj_attractor; proj_attractor << gamma_norm_proj_attractor(4,0), gamma_norm_proj_attractor(5,0);

    // Get information about the position of the project point on the boundary of the obstacle
    Eigen::Matrix<float, 1, 3> distances_surface = get_distances_surface(proj_robot, proj_attractor, border);

    // Get the maximum gamma distance in the initial space for the robot
    float max_gamma_robot = get_max_gamma_distance( proj_robot, gamma_norm_proj.block(1,0,2,1).transpose(), border);

    // Get the maximum gamma distance in the initial space for the attractor
    float max_gamma_attractor = get_max_gamma_distance( proj_attractor, gamma_norm_proj_attractor.block(1,0,2,1).transpose(), border);

    // Get the position of the robot in the circle space
    Eigen::Matrix<float, 10, 1> point_circle_space = get_point_circle_frame( distances_surface(0,1), distances_surface(0,0), gamma_norm_proj(0,0), max_gamma_robot, gamma_norm_proj_attractor(0,0), max_gamma_attractor, distances_surface(0,2));

    State robot_circle = point_circle_space.block(1,0,3,1);

    std::cout << "In circle space, attractor is at position (" << point_circle_space(4,0) << " , " << point_circle_space(5,0) << ")" << std::endl;

    N_steps = 32000;
    for (int i=0; i<N_steps; i++)
    {
        if (std::remainder(i,100)==0) {std::cout << i << std::endl;}
        Eigen::Matrix<float, 4, 1> output_circle = next_step_special_only_circle(point_circle_space);

        State next_eps; next_eps = output_circle.block(0,0,3,1);
        mystream << robot_circle(0,0) << "," << robot_circle(1,0) << "\n"; // write position of the point in the circle space
        robot_circle += next_eps * time_step;

        point_circle_space(0,0) = std::sqrt(std::pow(robot_circle(0,0),2)+std::pow(robot_circle(1,0),2));
        point_circle_space.block(1,0,3,1) = robot_circle;
        point_circle_space.block(7,0,3,1) = robot_circle.colwise().normalized();
    }

    mystream.close();

    std::cout << " ###### File closed ###### " << std::endl;
}

void quiver_bezier()

{
    float size_cell = 1;
    // Num 0
    /*Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(3) << 0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(4) << 0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(5) << 0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(6) << 0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(7) << 0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(8) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;*/
    // Num 1
    /*Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(3) << 0,0,0,0,0,1,0,0,0,0,0;
    occupancy.row(4) << 0,0,0,0,1,1,1,0,0,0,0;
    occupancy.row(5) << 0,0,0,1,1,1,1,1,0,0,0;
    occupancy.row(6) << 0,0,0,0,1,1,1,0,0,0,0;
    occupancy.row(7) << 0,0,0,0,0,1,0,0,0,0,0;
    occupancy.row(8) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;*/
    // Num 2
    /*Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,1,1,1,1,1,0,0,0;
    occupancy.row(3) << 0,0,0,1,0,0,0,1,0,0,0;
    occupancy.row(4) << 0,0,0,1,0,0,0,0,0,0,0;
    occupancy.row(5) << 0,0,0,1,0,0,0,0,0,0,0;
    occupancy.row(6) << 0,0,0,1,0,0,0,0,0,0,0;
    occupancy.row(7) << 0,0,0,1,0,0,0,1,0,0,0;
    occupancy.row(8) << 0,0,0,1,1,1,1,1,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;*/
    // Num 3
    /*Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,1,0,0,0,1,0,0;
    occupancy.row(2) << 0,0,0,1,1,1,1,1,0,0,0;
    occupancy.row(3) << 0,0,1,1,0,0,0,1,1,0,0;
    occupancy.row(4) << 0,0,0,1,0,0,0,0,1,0,0;
    occupancy.row(5) << 0,0,0,1,1,0,0,0,0,0,0;
    occupancy.row(6) << 0,0,0,0,1,0,0,0,0,0,0;
    occupancy.row(7) << 0,0,0,0,1,0,0,1,0,0,0;
    occupancy.row(8) << 0,0,0,1,1,1,1,1,1,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,1,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;*/
    // Num 4
    /*Grid occupancy = Grid::Zero(21,21);
    occupancy.row(0)  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1)  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2)  << 0,0,0,1,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(3)  << 0,0,0,1,0,0,1,0,0,1,0,0,0,0,0,1,0,0,0,0,0;
    occupancy.row(4)  << 0,0,0,1,0,0,1,0,0,1,0,0,0,0,1,1,1,0,0,0,0;
    occupancy.row(5)  << 0,0,0,1,0,0,1,0,0,1,0,0,0,1,1,1,1,1,0,0,0;
    occupancy.row(6)  << 0,0,0,1,0,0,1,0,0,1,0,0,0,0,1,1,1,0,0,0,0;
    occupancy.row(7)  << 0,0,0,0,1,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0;
    occupancy.row(8)  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9)  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(11) << 0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,1,1,0,0,0;
    occupancy.row(12) << 0,0,0,0,0,0,1,1,0,0,0,1,0,0,0,0,0,1,0,0,0;
    occupancy.row(13) << 0,0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,0,1,0,0,0;
    occupancy.row(14) << 0,0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,0,1,0,0,0;
    occupancy.row(15) << 0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0;
    occupancy.row(16) << 0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,0,0,0;
    occupancy.row(17) << 0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,0,0,0,0;
    occupancy.row(18) << 0,0,0,0,0,0,1,1,1,0,0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(19) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(20) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;*/
    // Num 5 : num 2 with attractor at (5,5)
    // Num 6 : num 0 with also classic
    // Num 7
    /*Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,1,1,0,0;
    occupancy.row(3) << 0,0,0,0,0,0,0,1,1,0,0;
    occupancy.row(4) << 0,0,1,1,1,1,1,1,1,0,0;
    occupancy.row(5) << 0,0,1,1,1,1,1,1,1,0,0;
    occupancy.row(6) << 0,0,1,1,0,0,0,1,1,0,0;
    occupancy.row(7) << 0,0,1,1,0,0,0,1,1,0,0;
    occupancy.row(8) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;*/
    // Num 8
    /*Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,1,1,0,0;
    occupancy.row(3) << 0,0,0,0,0,0,0,1,1,0,0;
    occupancy.row(4) << 0,0,1,1,1,1,1,1,1,0,0;
    occupancy.row(5) << 0,0,1,1,1,1,1,1,1,0,0;
    occupancy.row(6) << 0,0,1,1,1,1,1,1,1,0,0;
    occupancy.row(7) << 0,0,1,1,1,1,1,1,1,0,0;
    occupancy.row(8) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;*/
    // Num 9
    /*Grid occupancy = Grid::Zero(21,21);
    occupancy.row(0)  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1)  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2)  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(3)  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(4)  << 0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(5)  << 0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0;
    occupancy.row(6)  << 0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0;
    occupancy.row(7)  << 0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0;
    occupancy.row(8)  << 0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0;
    occupancy.row(9)  << 0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0;
    occupancy.row(10) << 0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0;
    occupancy.row(11) << 0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0;
    occupancy.row(12) << 0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0;
    occupancy.row(13) << 0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0;
    occupancy.row(14) << 0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0;
    occupancy.row(15) << 0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0;
    occupancy.row(16) << 0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(17) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(18) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(19) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(20) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;*/
    // Num 10
    /*Grid occupancy = Grid::Zero(21,21);
    occupancy.row(0)  << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1)  << 0,1,1,1,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0;
    occupancy.row(2)  << 0,1,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0;
    occupancy.row(3)  << 0,1,1,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(4)  << 0,0,0,0,0,0,0,0,0,0,1,0,0,1,1,1,1,0,0,0,0;
    occupancy.row(5)  << 0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,1,0,0,0;
    occupancy.row(6)  << 0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0;
    occupancy.row(7)  << 0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,0,0,0,0;
    occupancy.row(8)  << 0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,1,0,0,0,0,0;
    occupancy.row(9)  << 0,0,0,0,0,0,0,1,1,0,0,0,0,0,1,0,0,0,0,0,0;
    occupancy.row(10) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0;
    occupancy.row(11) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0;
    occupancy.row(12) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0;
    occupancy.row(13) << 0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0;
    occupancy.row(14) << 0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(15) << 0,0,0,1,1,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0;
    occupancy.row(16) << 0,0,0,1,1,0,0,0,1,1,0,0,0,0,0,0,1,1,1,0,0;
    occupancy.row(17) << 0,0,0,1,1,0,0,0,1,1,0,0,0,0,0,1,1,1,1,1,0;
    occupancy.row(18) << 0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,0,0;
    occupancy.row(19) << 0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0;
    occupancy.row(20) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;*/
    // Num 11
    Grid occupancy = Grid::Zero(13,13);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(3) << 0,0,1,1,1,1,1,1,1,1,1,0,0;
    occupancy.row(4) << 0,0,1,0,0,0,1,0,0,0,1,0,0;
    occupancy.row(5) << 0,0,1,0,0,0,1,0,0,0,1,0,0;
    occupancy.row(6) << 0,0,1,0,0,0,1,0,0,0,1,0,0;
    occupancy.row(7) << 0,0,0,0,0,0,0,0,0,0,1,0,0;
    occupancy.row(8) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(11)<< 0,0,0,0,1,1,1,1,0,0,0,0,0;
    occupancy.row(12)<< 0,0,0,0,0,0,0,0,0,0,0,0,0;


    occupancy *= 100;

    int num = 11;

    // State of the robot
    State state_robot; state_robot << 0.0, 0.0, 0;

    // Detect expanded obstacles
    std::vector<Border> storage;
    storage = detect_borders( occupancy, state_robot);

    std::ofstream mycells;
    mycells.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+std::to_string(num)+"_cells.txt");
    for (int i=0; i<occupancy.rows(); i++)
    {
        for (int j=0; j<occupancy.cols(); j++)
        {
            if (occupancy(i,j)==100) {mycells << i << "," << j << "\n";}
        }
    }
    mycells.close();

    std::ofstream myobs;
    myobs.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+std::to_string(num)+"_obs.txt");
    for (int iter=0; iter<storage.size(); iter++)
    {
        Border border_out = storage[iter];
        for (int i=0; i<border_out.rows(); i++)
        {
            myobs << border_out(i,0) << "," << border_out(i,1) << "," << border_out(i,2) << "," << border_out(i,3) << "," << border_out(i,4) << "," << iter << "\n";
        }
    }
    myobs.close();

    for (int iter=0; iter<storage.size(); iter++)
    {
        Eigen::MatrixXf res_bez = border_to_vertices(storage[iter]);
        std::vector<Eigen::MatrixXf> pts_bezier = compute_bezier(res_bez);
        const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
        std::ofstream my_bezier_border;
        my_bezier_border.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+std::to_string(num)+"_border_bezier_"+std::to_string(iter)+".txt");
        for (int i=0; i<pts_bezier.size(); i++)
        {
            my_bezier_border << (pts_bezier[i]).format(CSVFormat) << "\n";
        }
        my_bezier_border.close();
    }

    std::ofstream mystream, mystream_bezier, mystream_classic;
    mystream.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+std::to_string(num)+"_normal.txt");
    mystream_bezier.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+std::to_string(num)+"_bezier.txt");
    mystream_classic.open("D:/Mes documents/Devoirs/MasterThesis/catkin_project/StreamData/stream_data_"+std::to_string(num)+"_classic.txt");

    // Position of the attractor
    State state_attractor;
    state_attractor << 0,0,0;//7.2, -1.5, 0;

    State state_reference;
    state_reference << 5, 4.5, 0; // Num 6
    //state_reference << 4.5, 7.5, 0; // Num 7 & Num 8
    //state_reference << 10, 10, 0; // Num 9
    // Limits of stream
    Eigen::Matrix<float, 5, 1> limits;
    //limits << -2.02, 14.02, -2.02, 14.02, 0.25;
    limits << -0.02, 14.02, -0.02, 14.02, 0.25;
    /*for (float x=2; x<9; x+=0.5)
    {
        State next_eps;
        state_robot << x, 11, 0;
        std::cout << " ### x = " << x << std::endl;
        next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    }*/

    /*{

        State next_eps;
        state_robot << 7.9, 7.4, 0;
        next_eps = get_next_velocity_command_weighted( state_robot, state_attractor, storage, size_cell, true, true);

        state_robot << 9.5, 7.4, 0;
        next_eps = get_next_velocity_command_weighted( state_robot, state_attractor, storage, size_cell, true, true);


    }*/
    for (float x=limits(0,0); x <= limits(1,0); x += limits(4,0)) // x direction of the grid
    {
        //if ((x-std::floor(x))<0.4) {std::cout << " ### x = " << x << " ###" << std::endl;}
        std::cout << " ### x = " << x << std::endl;
        for (float y=limits(2,0); y <= limits(3,0); y += limits(4,0)) // y direction of the grid
        {

            // Position of the point
            State state_point;
            state_point << x, y, 0;

            // Compute velocity command
            State next_eps;
            //next_eps = get_next_velocity_command_weighted( state_point, state_attractor, storage, size_cell, true, false);
            //mystream << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file

            next_eps = get_next_velocity_command_weighted( state_point, state_attractor, storage, size_cell, true, true);
            mystream_bezier << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file
            /*
            Eigen::Matrix<float, 4, 1> output;
            output = next_step_classic( state_point, state_attractor, state_reference, storage[0]);
            mystream_classic << x << "," << y << "," << output(0,0) << "," << output(1,0) << "\n"; // write result in text file*/
        }
    }

    mystream.close();
    mystream_bezier.close();
    mystream_classic.close();
    std::cout << " ###### File closed ###### " << std::endl;
}

void test_bezier()
{
    Eigen::MatrixXf XY = Eigen::MatrixXf::Zero(4,2);
    XY.col(0) << 0,1,1,0;
    XY.col(1) << 0,0,1,1;

    std::vector<Eigen::MatrixXf> res = compute_bezier(XY);

    float size_cell = 1;

    Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(3) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(4) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(5) << 0,0,0,0,1,1,1,0,0,0,0;
    occupancy.row(6) << 0,0,0,0,1,1,1,0,0,0,0;
    occupancy.row(7) << 0,0,0,0,1,1,1,0,0,0,0;
    occupancy.row(8) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;
    occupancy *= 100;

    // State of the robot
    State state_robot; state_robot << 2, 5.7, 0;

    // State of the attractor
    State state_attractor; state_attractor << 2, 6.51, 0;

    //* Detect expanded obstacles
    std::vector<Border> storage;
    storage = detect_borders( occupancy, state_robot);

    Eigen::MatrixXf res_bez = border_to_vertices(storage[0]);

    //std::cout << res_bez << std::endl;

    State closest_robot = get_projection_on_bezier(state_robot, compute_bezier(res_bez));
    std::cout << "Closest from robot: " << closest_robot.transpose() << std::endl;

    State closest_attractor = get_projection_on_bezier(state_attractor, compute_bezier(res_bez));
    std::cout << "Closest from attractor: " << closest_attractor.transpose() << std::endl;

    Eigen::Matrix<float, 1, 2> proj_robot; proj_robot << closest_robot(0,0), closest_robot(1,0);
    Eigen::Matrix<float, 1, 2> proj_attractor; proj_attractor << closest_attractor(0,0), closest_attractor(1,0);

    Eigen::Matrix<float, 1, 3> distances = get_distances_surface_bezier( proj_robot, proj_attractor, compute_bezier(res_bez));

    std::cout << "Distance tot: " << distances(0,0) << std::endl;
    std::cout << "Distance min: " << distances(0,1) << std::endl;
    std::cout << "Direction: " << distances(0,2) << std::endl;

    Eigen::Matrix<float, 1, 2> normal; normal << state_robot(0,0) - proj_robot(0,0), state_robot(1,0) - proj_robot(0,1);
    normal = normal / std::sqrt(std::pow(normal(0,0),2) + std::pow(normal(0,1),2));

    float max_gamma = get_max_gamma_distance_bezier( proj_robot, normal, compute_bezier(res_bez));

    std::cout << "Gamma: " << max_gamma << std::endl;
}
void test_fill_holes()
{
    Grid occupancy = Grid::Zero(14,16);
    occupancy.row(4) << 0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0;
    occupancy.row(5) << 0,0,0,0,1,0,0,0,0,0,0,1,1,0,0,0;
    occupancy.row(6) << 0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0;
    occupancy.row(7) << 0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0;
    occupancy.row(8) << 0,0,0,1,0,0,0,0,0,0,0,1,1,0,0,0;
    occupancy.row(9) << 0,0,0,1,0,0,0,0,0,0,0,1,1,0,0,0;
    occupancy.row(10)<< 0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0;
    occupancy *= 100;
    State state_robot; state_robot << 8, 8, 0;
    Grid output = expand_occupancy_grid( occupancy, 1, state_robot, 10, 1);

    cout << output << endl;

    // Detect expanded obstacles
    std::vector<Border> storage;
    storage = detect_borders( output, state_robot);

    std::cout << storage.size() << " detected obstacle(s)" << std::endl;
    for (int i=0; i < storage.size(); i++)
    {
        //std::cout << storage[i] << std::endl;
    }
}

void call_morphing()
{
    Blob obst(23,2); // This obstacle is a U-shape obstace
    Blob res;
    obst.row(0) << 5, 4;
    obst.row(1) << 6, 4;
    obst.row(2) << 7, 4;
    obst.row(3) << 8, 4;
    obst.row(4) << 9, 4;
    obst.row(5) << 10, 4;
    obst.row(6) << 11, 4;
    obst.row(7) << 12, 4;
    obst.row(8) << 13, 4;
    obst.row(9) << 5, 5;
    obst.row(10) << 13, 5;
    obst.row(11) << 5, 6;
    obst.row(12) << 13, 6;
    obst.row(13) << 5, 7;
    obst.row(14) << 13, 7;
    obst.row(15) << 5, 8;
    obst.row(16) << 13, 8;
    obst.row(17) << 5, 9;
    obst.row(18) << 13, 9;
    obst.row(19) << 6, 9;
    obst.row(20) << 12, 9;
    obst.row(21) << 7, 9;
    obst.row(22) << 11, 9;

    Eigen::Matrix<float, 5, 1> limits_morphing;
    limits_morphing << 2.02, 16.02, 1.02, 12.02, 0.1;

    State state_attractor; state_attractor << 14.3,6.6,0;//10,6.8,0;//9.3, 7, 0; // 9.1,8.5, 0;//8, 1.6,0; // doesn't matter

    std::vector<Blob> stack_of_obst;
    stack_of_obst.push_back(obst);
    //compute_morphing( limits_morphing, state_attractor, stack_of_obst); // compute morphing between initial and circle spaces

    State robot; robot << 8,1.6,0; // 10.61, 6.45, 0;
    compute_trajectory_both_spaces(robot, state_attractor, stack_of_obst);
}
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
    //border_out = compute_border( obst, center_blob);

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

void test_various_functions()
{
    float size_cell = 1;

    Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(3) << 0,0,1,1,1,1,1,1,0,0,0;
    occupancy.row(4) << 0,0,1,1,1,1,1,1,0,0,0;
    occupancy.row(5) << 0,0,1,1,0,0,0,0,0,0,0;
    occupancy.row(6) << 0,0,1,1,0,0,0,0,0,0,0;
    occupancy.row(7) << 0,0,1,1,0,0,0,0,0,0,0;
    occupancy.row(8) << 0,0,1,1,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;
    /*occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,1,1,0,0,1,0,0,0;
    occupancy.row(2) << 0,0,0,1,1,0,0,1,0,0,0;
    occupancy.row(3) << 0,0,0,1,1,0,0,1,0,0,0;
    occupancy.row(4) << 0,0,0,0,1,1,1,1,1,0,0;
    occupancy.row(5) << 0,0,0,0,1,1,1,1,1,0,0;
    occupancy.row(6) << 0,0,0,1,1,1,1,1,0,0,0;
    occupancy.row(7) << 0,0,0,1,1,1,1,1,0,0,0;
    occupancy.row(8) << 0,0,0,0,1,1,1,1,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,1,1,1,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;*/
    /*occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(3) << 0,1,0,0,1,1,1,1,0,0,0;
    occupancy.row(4) << 0,1,0,0,1,1,1,1,0,0,0;
    occupancy.row(5) << 0,1,0,0,1,1,1,1,0,0,0;
    occupancy.row(6) << 0,1,0,0,1,1,1,1,0,0,0;
    occupancy.row(7) << 0,0,0,0,0,1,1,1,0,0,0;
    occupancy.row(8) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;*/
    /*occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,1,1,1,1,1,1,1,0,0,0;
    occupancy.row(2) << 0,0,0,1,1,1,1,1,1,0,0,0,0;
    occupancy.row(3) << 0,1,0,0,1,1,1,1,0,0,1,0,0;
    occupancy.row(4) << 0,1,1,0,0,1,1,1,0,1,1,0,0;
    occupancy.row(5) << 0,1,1,1,1,1,1,1,1,1,1,1,0;
    occupancy.row(6) << 0,1,1,1,1,1,1,1,1,0,1,0,0;
    occupancy.row(7) << 0,1,1,1,1,1,1,1,1,1,0,0,0;
    occupancy.row(8) << 0,1,1,1,0,1,1,1,1,1,1,0,0;
    occupancy.row(9) << 0,1,1,0,0,1,0,1,1,1,1,1,0;
    occupancy.row(10)<< 0,0,0,0,1,1,1,0,1,1,1,0,0;
    occupancy.row(11)<< 0,0,0,1,1,1,0,0,0,1,0,0,0;
    occupancy.row(12)<< 0,0,0,0,1,0,0,0,0,0,0,0,0;*/
    /*occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,1,1,1,1,1,1,1,0,0,0;
    occupancy.row(2) << 0,0,0,1,1,1,1,1,1,0,0,0,0;
    occupancy.row(3) << 0,1,1,1,1,1,1,1,0,0,1,0,0;
    occupancy.row(4) << 0,1,1,0,0,0,1,1,0,1,1,0,0;
    occupancy.row(5) << 0,0,0,0,0,0,0,1,1,1,1,1,0;
    occupancy.row(6) << 0,0,0,0,0,0,0,0,1,0,1,0,0;
    occupancy.row(7) << 0,0,0,0,0,0,0,1,1,1,0,0,0;
    occupancy.row(8) << 0,1,1,0,0,0,1,1,1,1,1,0,0;
    occupancy.row(9) << 0,1,1,1,1,1,1,1,1,1,1,1,0;
    occupancy.row(10)<< 0,0,0,1,1,1,1,0,1,1,1,0,0;
    occupancy.row(11)<< 0,0,0,1,1,1,0,0,0,1,0,0,0;
    occupancy.row(12)<< 0,0,0,0,1,0,0,0,0,0,0,0,0;*/
    occupancy *= 100;

    // State of the robot
    State state_robot; state_robot << 0.01, 0.01, 0;

    // State of the attractor
    State state_attractor; state_attractor << 0, 5, 0;

    //* Detect expanded obstacles
    std::vector<Border> storage;
    storage = detect_borders( occupancy, state_robot);

    //std::cout << storage[0] << std::endl;
    for (int iter=0; iter < storage.size(); iter++)
    {
        std::cout << "Border " << iter << ":"<< std::endl;
        std::cout << storage[iter] << std::endl;
    }

    // define the format you want, you only need one instance of this...
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

    /*std::ofstream mystream;
    mystream.open("/home/leziart/catkin_ws/StreamNode/test_to_delete.txt");
    mystream << (storage[0]).format(CSVFormat) << "\n";
    mystream.close();*/

    /*std::ofstream mystream;
    mystream.open("/home/leziart/catkin_ws/StreamNode/test_various_functions.txt");
    for (float x=0; x<11; x+=0.05)
    {
        for (float y=0; y<11; y+=0.05)
        {
            state_robot << x, y, 0;
            // Compute velocity command
            State next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
            //std::cout << "Velocity command: " << next_eps.transpose() << std::endl;

            mystream << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file

        }
    }
    mystream.close();*/
    /*
    mystream.open("/home/leziart/catkin_ws/StreamNode/test_various_functions_blob.txt");
    for (int x=0; x<occupancy.rows(); x++)
    {
        for (int y=0; y<occupancy.cols(); y++)
        {
            if (occupancy(x,y) == 100) {mystream << x << "," << y << "\n";}
        }
    }
    mystream.close();


    /*mystream.open("/home/leziart/catkin_ws/StreamNode/test_various_functions_vel.txt");
    for (float x=7; x<10; x+=0.001)
    {
        State next_eps;
        state_robot << x, 8, 0;
        next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
        mystream << x << "," << 8 << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file
    }
    mystream.close();*/

    /*State next_eps;
    state_robot << 6.2, 9, 0;
    std::cout << " ###### " << state_robot(0,0) << " " << state_robot(1,0) << " ###### " << std::endl;
    next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    std::cout << "Velocity command: " << next_eps.transpose() << std::endl;
    state_robot << 6.2, 9.98, 0;
    std::cout << " ###### " << state_robot(0,0) << " " << state_robot(1,0) << " ###### " << std::endl;
    next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    std::cout << "Velocity command: " << next_eps.transpose() << std::endl;
    state_robot << 6.2, 9.986, 0;
    std::cout << " ###### " << state_robot(0,0) << " " << state_robot(1,0) << " ###### " << std::endl;
    next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    std::cout << "Velocity command: " << next_eps.transpose() << std::endl;
    state_robot <<  6.2, 9.99, 0;
    std::cout << " ###### " << state_robot(0,0) << " " << state_robot(1,0) << " ###### " << std::endl;
    next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    std::cout << "Velocity command: " << next_eps.transpose() << std::endl;*/

    State next_eps;
    state_robot << 5.0f, 6.0f, 0;
    std::cout << " ###### " << state_robot(0,0) << " " << state_robot(1,0) << " ###### " << std::endl;
    next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    std::cout << "Velocity command: " << next_eps.transpose() << std::endl;
    state_robot << 5, 6.0, 0;
    std::cout << " ###### " << state_robot(0,0) << " " << state_robot(1,0) << " ###### " << std::endl;
    next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    std::cout << "Velocity command: " << next_eps.transpose() << std::endl;
    state_robot << 5, 4.45, 0;
    std::cout << " ###### " << state_robot(0,0) << " " << state_robot(1,0) << " ###### " << std::endl;
    next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    std::cout << "Velocity command: " << next_eps.transpose() << std::endl;
    state_robot << 5, 4, 0;
    std::cout << " ###### " << state_robot(0,0) << " " << state_robot(1,0) << " ###### " << std::endl;
    next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    std::cout << "Velocity command: " << next_eps.transpose() << std::endl;

    /*State next_eps;
    float x, y;
    for (x = 7.9, y = 3.4; x <= 8.0; x+=0.025, y+=0.025)
    {
        state_robot << x, y, 0;
        std::cout << " ###### " << state_robot(0,0) << " " << state_robot(1,0) << " ###### " << std::endl;
        next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
        std::cout << "Velocity command: " << next_eps.transpose() << std::endl;
    }


    extern bool logging_enabled;
    extern Eigen::MatrixXf log_matrix;

    if (logging_enabled)
    {
        std::cout << " ###### Log matrix ###### " << std::endl;
        std::cout << log_matrix << std::endl;
    }*/


}


int main()
{
    //Eigen::MatrixXi storage_line = Eigen::MatrixXi::Zero(0,2);
    //Line(1.0,1.0f,9.0f,4.0f,storage_line);
    //std::cout << storage_line << std::endl;

    // Num 11
    /*Grid occupancy = Grid::Zero(13,13);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,1,1,1,1,1,1,0,0,0,0,0;
    occupancy.row(3) << 0,0,1,0,0,0,0,1,0,0,0,0,0;
    occupancy.row(4) << 0,0,1,0,0,0,0,1,0,0,0,0,0;
    occupancy.row(5) << 0,0,1,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(6) << 0,0,1,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(7) << 0,0,1,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(8) << 0,0,1,0,0,0,0,1,0,0,0,0,0;
    occupancy.row(9) << 0,0,1,0,0,0,0,1,0,0,0,0,0;
    occupancy.row(10)<< 0,0,1,1,1,1,1,1,0,0,0,0,0;
    occupancy.row(11)<< 0,0,0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(12)<< 0,0,0,0,0,0,0,0,0,0,0,0,0;*/

    Grid occupancy = Grid::Zero(11,11);
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,1,1,1,1,1,0,0,0;
    occupancy.row(3) << 0,0,0,1,0,0,0,1,0,0,0;
    occupancy.row(4) << 0,0,0,1,0,0,0,0,0,0,0;
    occupancy.row(5) << 0,0,0,1,0,0,0,0,0,0,0;
    occupancy.row(6) << 0,0,0,1,0,0,0,0,0,0,0;
    occupancy.row(7) << 0,0,0,1,0,0,0,1,0,0,0;
    occupancy.row(8) << 0,0,0,1,1,1,1,1,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(0) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(1) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(2) << 0,0,0,0,0,0,0,1,1,0,0;
    occupancy.row(3) << 0,0,0,0,0,0,0,1,1,0,0;
    occupancy.row(4) << 0,0,1,1,1,1,1,1,1,0,0;
    occupancy.row(5) << 0,0,1,1,1,1,1,1,1,0,0;
    occupancy.row(6) << 0,0,1,1,0,0,0,1,1,0,0;
    occupancy.row(7) << 0,0,1,1,0,0,0,1,1,0,0;
    occupancy.row(8) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(9) << 0,0,0,0,0,0,0,0,0,0,0;
    occupancy.row(10)<< 0,0,0,0,0,0,0,0,0,0,0;
    occupancy *= 100;

    /*int num = 11;

    // State of the robot
    State state_robot; state_robot << 6.1,5.0,0;//3.7, 5.0, 0;
    State state_attractor; state_attractor << 4.0, 1.0, 0;

    // Detect expanded obstacles
    std::vector<Border> storage;
    storage = detect_borders( occupancy, state_robot);

    State next_eps;
    next_eps = test_next_step_special_weighted( state_robot, state_attractor, storage, 1.0);
    std::cout << "Vel cmd: " << next_eps(0,0) << "," << next_eps(1,0) << std::endl;*/

    /*const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    std::ofstream mystream;
    mystream.open("D:/Mes documents/Devoirs/MasterThesis/Article/Figures_EPS/nonstarshaped_obstacle.txt");
    mystream << (storage[0]).format(CSVFormat) << "\n";
    mystream.close();*/

    //trajectory_comparison();
    quiver_bezier();
    //test_bezier();
    //test_various_functions();
    //test_draw_circle();
    //call_morphing();
    //test_fill_holes();
    //growing_obstacle();
    //trajectories_border();
    //growing_several_obstacle();
    if (false)
    {
        float size_cell = 0.15;
        float radius_ridgeback = 0.6;
        int n_expansion = static_cast<int>(std::ceil(radius_ridgeback/size_cell));

        float limit_in_meters = 3;
        int   limit_in_cells  = static_cast<int>(std::ceil(limit_in_meters/size_cell));

        float start_cell_x = 133;
        float start_cell_y = 133;

        State state_attractor; state_attractor << 193,133,0;

        Eigen::MatrixXi occupancy_grid, obstacles;
        int ID_frame = 0;
        while(ID_frame < 1)
        {
            std::cout << "ID FRAME " << ID_frame << std::endl;

            // Declare file for recording
            std::ofstream mystream;
            if (ID_frame < 10)
            {
                std::cout << "ID FRAME < 10" << std::endl;
                mystream.open("/home/leziart/catkin_ws/StreamNode/stream_data_node_000" + std::to_string(ID_frame) + ".txt");
                std::string filename = "/home/leziart/catkin_ws/StreamNode/eigpeople_000" + std::to_string(ID_frame) + ".txt";
                occupancy_grid = readMatrix(filename);
                std::string filename2 = "/home/leziart/catkin_ws/StreamNode/obstacles_000" + std::to_string(ID_frame) + ".txt";
                obstacles = readMatrix(filename2);
            }
            else if (ID_frame < 100)
            {
                mystream.open("/home/leziart/catkin_ws/StreamNode/stream_data_node_00" + std::to_string(ID_frame) + ".txt");
                std::string filename = "/home/leziart/catkin_ws/StreamNode/eigpeople_00" + std::to_string(ID_frame) + ".txt";
                occupancy_grid = readMatrix(filename);
                filename = "/home/leziart/catkin_ws/StreamNode/obstacles_00" + std::to_string(ID_frame) + ".txt";
                obstacles = readMatrix(filename);
            }
            else if (ID_frame < 1000)
            {
                mystream.open("/home/leziart/catkin_ws/StreamNode/stream_data_node_0" + std::to_string(ID_frame) + ".txt");
                std::string filename = "/home/leziart/catkin_ws/StreamNode/eigpeople_0" + std::to_string(ID_frame) + ".txt";
                occupancy_grid = readMatrix(filename);
                filename = "/home/leziart/catkin_ws/StreamNode/obstacles_0" + std::to_string(ID_frame) + ".txt";
                obstacles = readMatrix(filename);
            }
            else
            {
                mystream.open("/home/leziart/catkin_ws/StreamNode/stream_data_node_" + std::to_string(ID_frame) + ".txt");
                std::string filename = "/home/leziart/catkin_ws/StreamNode/eigpeople_" + std::to_string(ID_frame) + ".txt";
                occupancy_grid = readMatrix(filename);
                filename = "/home/leziart/catkin_ws/StreamNode/obstacles_" + std::to_string(ID_frame) + ".txt";
                obstacles = readMatrix(filename);
            }
            ID_frame+=1;

            //std::cout << obstacles << std::endl;

            // Limits of stream
            Eigen::Matrix<float, 5, 1> limits;
            limits << 0.02, 10.02, -5.02, 5.02, 0.2;

            for (float x=limits(0,0); x <= limits(1,0); x += limits(4,0)) // x direction of the grid
            {
                std::cout << x << std::endl;
                for (float y=limits(2,0); y <= limits(3,0); y += limits(4,0)) // y direction of the grid
                {
                    float temp_cell_x = start_cell_x + std::round(x / size_cell);
                    float temp_cell_y = start_cell_y + std::round(y / size_cell);

                    // Position of the point
                    State state_point;
                    state_point << temp_cell_x, temp_cell_y, 0;

                    // Expand occupancy grid (obstacles in range)
                    Grid eig_expanded = expand_occupancy_grid( occupancy_grid, n_expansion, state_point, limit_in_cells, size_cell);

                    // Detect borders of obstacles in range
                    std::vector<Border> storage;
                    storage = detect_borders( eig_expanded, state_point );

                    bool flag = true;

                    for (int k=0; k<obstacles.rows(); k++)
                    {
                        if ((obstacles(k,0)==temp_cell_x)&&(obstacles(k,1)==temp_cell_y))
                        {
                            flag = false;
                            k=obstacles.rows();
                        }
                    }


                    if (flag)
                    {

                    // Compute velocity command
                    State next_eps;
                    next_eps = next_step_special_weighted( state_point, state_attractor, storage, size_cell);

                    mystream << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file
                    }
                    else
                    {
                       mystream << x << "," << y << "," << 0.0 << "," << 0.0 << "\n"; // write result in text file
                    }
                }
            }

            mystream.close();
        }
    }

    if (false)
    {
        Grid occupancy_grid(9,13);
        occupancy_grid.row(0) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
        occupancy_grid.row(1) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
        occupancy_grid.row(2) << 0,0,0,0,0,0,0,1,1,1,0,0,0;
        occupancy_grid.row(3) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
        occupancy_grid.row(4) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
        occupancy_grid.row(5) << 0,0,0,1,1,0,0,0,0,0,0,0,0;
        occupancy_grid.row(6) << 0,0,0,1,1,0,0,0,0,0,0,0,0;
        occupancy_grid.row(7) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
        occupancy_grid.row(8) << 0,0,0,0,0,0,0,0,0,0,0,0,0;
        occupancy_grid *= 100;
        std::vector<Border> storage;
        State placeholder_state; placeholder_state << 0, 0, 0;
        storage = detect_borders( occupancy_grid, placeholder_state );
        cout << storage.size() << " obstacles have been detected." << endl;
        for (int iter=0; iter<storage.size(); iter++)
        {
            cout << "Obstacle " << iter << ":"<< endl;
            cout << storage[iter] << endl;
        }



        /*Grid expanded_grid = expand_occupancy_grid( occupancy_grid, 1);
        cout << "Grid expanded by 1 cell." << endl;
        cout << expanded_grid << endl;
        expanded_grid = expand_occupancy_grid( occupancy_grid, 2, );
        cout << "Grid expanded by 2 cell." << endl;
        cout << expanded_grid << endl;
        cout << "- END -"<< endl;

        storage = detect_borders( expanded_grid );
        cout << storage.size() << " obstacles have been detected." << endl;
        for (int iter=0; iter<storage.size(); iter++)
        {
            cout << "Obstacle " << iter << ":"<< endl;
            cout << storage[iter] << endl;

        }*/
    }

    if (false)
    {
        // This obstacle has a random shape
        /*Blob obst(23,2);
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
        obst.row(22) <<  4, 8;*/

        // This obstacle is a filled rectangle
        /*Blob obst(12,2);
        Blob res;
        obst.row(0) << 6, 4;
        obst.row(1) << 7, 4;
        obst.row(2) << 8, 4;
        obst.row(3) << 9, 4;
        obst.row(4) << 6, 5;
        obst.row(5) << 7, 5;
        obst.row(6) << 8, 5;
        obst.row(7) << 9, 5;
        obst.row(8) << 6, 6;
        obst.row(9) << 7, 6;
        obst.row(10) << 8, 6;
        obst.row(11) << 9, 6;*/

        // This obstacle is a U-shaped obstacle
        Blob obst(23,2);
        Blob res;
        obst.row(0) << 5, 4;
        obst.row(1) << 6, 4;
        obst.row(2) << 7, 4;
        obst.row(3) << 8, 4;
        obst.row(4) << 9, 4;
        obst.row(5) << 10, 4;
        obst.row(6) << 11, 4;
        obst.row(7) << 12, 4;
        obst.row(8) << 13, 4;
        obst.row(9) << 5, 5;
        obst.row(10) << 13, 5;
        obst.row(11) << 5, 6;
        obst.row(12) << 13, 6;
        obst.row(13) << 5, 7;
        obst.row(14) << 13, 7;
        obst.row(15) << 5, 8;
        obst.row(16) << 13, 8;
        obst.row(17) << 5, 9;
        obst.row(18) << 13, 9;
        obst.row(19) << 6, 9;
        obst.row(20) << 12, 9;
        obst.row(21) << 7, 9;
        obst.row(22) << 11, 9;

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

        cout << "Border is: " << endl << border_out << endl;

        std::ofstream myobs;
        myobs.open("./Obstacles/article_border.txt");
        for (int i=0; i<border_out.rows(); i++)
        {
            myobs << border_out(i,0) << "," << border_out(i,1) << "," << border_out(i,2) << "," << border_out(i,3) << "," << border_out(i,4) << "\n";
        }
        myobs.close();


        Eigen::Matrix<float,1,2>  robot; robot << 8.4, 7.3;//6.5,8.5;//3.51,6.51;
        Eigen::MatrixXf closest(1,6);
        closest = find_closest_point(robot, border_out);
        cout << "Closest is: " << closest << endl;

        Eigen::Matrix<float, 4, 1> gamma_ref;
        gamma_ref = gamma_and_ref_vector( robot, closest);
        cout << "GammaRefVec is: " << gamma_ref.transpose() << endl;

        //State state_attractor; state_attractor << 17,6,0;
        State state_attractor; state_attractor << 8, 1.6,0;
        Eigen::Matrix<float,1,2> temp; temp << state_attractor(0,0), state_attractor(1,0);
        Eigen::MatrixXf closest_attract(1,6); closest_attract = find_closest_point(temp, border_out);
        cout << "Closest attractor is : " << closest_attract << endl;

        State my_robot; my_robot << robot(0,0), robot(0,1), 0;
        State next_eps = next_step_single_obstacle_border( my_robot, state_attractor, border_out);
        cout << "Velocity command is: " << next_eps.transpose() << endl;


        Blob other_obst_bis(10,2);
        other_obst_bis.row(0) << -3, 0;
        other_obst_bis.row(1) << -3, 1;
        other_obst_bis.row(2) << -3, 2;
        other_obst_bis.row(3) << -3, 3;
        other_obst_bis.row(4) << -3, -1;
        other_obst_bis.row(5) << -3, -2;
        other_obst_bis.row(6) << -2, 3;
        other_obst_bis.row(7) << -4, 3;
        other_obst_bis.row(8) << -2, -2;
        other_obst_bis.row(9) << -4, -2;

        Blob other_obst(5,2);
        other_obst.row(0) << 17, 4;
        other_obst.row(1) << 17, 5;
        other_obst.row(2) << 17, 6;
        other_obst.row(3) << 18, 4;
        other_obst.row(4) << 18, 5;
        Point center_other_blob = get_random(other_obst);
        Border border_other_out = compute_border( other_obst, center_other_blob);
        display_border(other_obst, border_other_out);


        float size_cell = 1;

        closest = find_closest_point(robot, border_other_out);
        cout << "Closest is: " << closest << endl;
        gamma_ref = gamma_and_ref_vector( robot, closest);
        cout << "GammaRefVec is: " << gamma_ref.transpose() << endl;
        //State state_attractor; state_attractor << 17,6,0;
        temp << state_attractor(0,0), state_attractor(1,0);
        closest_attract = find_closest_point(temp, border_other_out);
        cout << "Closest attractor is : " << closest_attract << endl;
        std::vector<Border> borders_test; borders_test.push_back(border_other_out);
        cout << "Next step weighted is : " << endl << next_step_special_weighted(my_robot, state_attractor, borders_test, size_cell) << endl;
        //cout << "Velocity command is: " << next_eps.transpose() << endl;




        /////

        cout << "Stacking obstacles" << endl;
        std::vector<Border> borders;
        borders.push_back(border_out); borders.push_back(border_other_out); borders.push_back(compute_border( other_obst_bis, get_random(other_obst_bis)));
        //next_step_special(my_robot, state_attractor, border_out);
        cout << "Next step weighted is : " << endl <<  next_step_special_weighted(my_robot, state_attractor, borders, size_cell) << endl;

        Eigen::Matrix<float, 5, 1> limits_quiver;
        limits_quiver << -9.02, 25.02, -9.02, 25.02, 0.2;


        std::vector<Blob> stack_of_obst;
        stack_of_obst.push_back(obst);
        stack_of_obst.push_back(other_obst);
        stack_of_obst.push_back(other_obst_bis);

        //compute_quiver_border(limits_quiver, state_attractor, other_obst);
        //compute_stream_border(limits_quiver, state_attractor, stack_of_obst);
        //expand_obstacle(obst, 2);


        plot_stream_gazebo();

    }
    return 0;
}

// Old function to plot trajectories for the first version of the algorithm
// Generate data in a /Trajectories folder that is use with matplotlib
void trajectory_generation()
{
    ofstream myfile;
    ofstream myfile_obstacles;
    myfile_obstacles.open("obstacles_trajectory.txt");
    bool flag_obstacles = true;

    bool flag_quiver = false; // Set to false to disable Quiver computation

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

            // Set of obstacle number 1
            /*state_robot     << -2, y_k, 0;
            state_attractor <<  8, 4, 0;

            Obstacle obs1, obs2, obs3, obs4, obs5;
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

            // Set of obstacle number 2
            state_robot     << 0  ,  y_k, 0;
            state_attractor << 5.9,  4.9, 0;
            Eigen::MatrixXf mat_obs(10,4); // [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]
            mat_obs.col(0) << -4, 7, 0, 1.6, 1.6, 1, 1, 0, 0, 0;
            mat_obs.col(1) <<  2, 2, 0, 1.6, 1.6, 1, 1, 0, 0, 0;
            mat_obs.col(2) <<  10, -2, 0, 1.6, 1.6, 1, 1, 0, 0, 0;
            mat_obs.col(3) <<  4, 0.5, 0, 1.6, 1.6, 1, 1, 0, 0, 0;

            Eigen::Matrix<float, 5, 1> limits_quiver;
            limits_quiver << -2, 6, -2, 5, 0.1;


            if (flag_quiver) // Compute quiver for the starting position of the obstacles
            {
                Eigen::Matrix<float, 2, 4> mat_points;
                mat_points.row(0) << 2, 2, 3, 3;
                mat_points.row(1) << 2, 3, 2, 3;

                //compute_quiver(limits_quiver, state_attractor, mat_obs);
                compute_quiver_multiplication(limits_quiver, state_attractor, mat_obs);
                //compute_quiver_polygon(limits_quiver, state_attractor, mat_points);
                flag_quiver = false;
            }

            int N_steps = 500; // Maximum number of steps
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
                myfile << time_stamp << "," << state_robot(0,0) << "," << state_robot(1,0) << "," << state_robot(2,0) << "\n"; // Write position of robot
                State next_eps = one_step_2D( state_robot, state_attractor, mat_obs);
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

// Create data to plot a stream with matplotlib for a given set of obstacles
void plot_stream_gazebo()
{
    // Initialization of four obstacles/blobs
    // Obstacles are stacked into a single structure stack_of_obsts
    // Initialization of the position of the attractor
    // Initialization of the limits of the quiver plot on matplotlib
    // Call function that computes the velocity command for all points in the limits of the quiver

    // Just change the initialization if you want to use your own obstacles

    Blob obst0(108,2);
    obst0.row(0) << 314, 312;
    obst0.row(1) << 315, 312;
    obst0.row(2) << 316, 312;
    obst0.row(3) << 317, 312;
    obst0.row(4) << 318, 312;
    obst0.row(5) << 319, 312;
    obst0.row(6) << 320, 312;
    obst0.row(7) << 321, 312;
    obst0.row(8) << 322, 312;
    obst0.row(9) << 323, 312;
    obst0.row(10) << 324, 312;
    obst0.row(11) << 325, 312;
    obst0.row(12) << 326, 312;
    obst0.row(13) << 326, 311;
    obst0.row(14) << 325, 311;
    obst0.row(15) << 324, 311;
    obst0.row(16) << 323, 311;
    obst0.row(17) << 322, 311;
    obst0.row(18) << 321, 311;
    obst0.row(19) << 320, 311;
    obst0.row(20) << 319, 311;
    obst0.row(21) << 318, 311;
    obst0.row(22) << 317, 311;
    obst0.row(23) << 316, 311;
    obst0.row(24) << 315, 311;
    obst0.row(25) << 316, 310;
    obst0.row(26) << 317, 310;
    obst0.row(27) << 318, 310;
    obst0.row(28) << 319, 310;
    obst0.row(29) << 320, 310;
    obst0.row(30) << 321, 310;
    obst0.row(31) << 322, 310;
    obst0.row(32) << 323, 310;
    obst0.row(33) << 324, 310;
    obst0.row(34) << 325, 310;
    obst0.row(35) << 326, 310;
    obst0.row(36) << 326, 309;
    obst0.row(37) << 325, 309;
    obst0.row(38) << 324, 309;
    obst0.row(39) << 323, 309;
    obst0.row(40) << 322, 309;
    obst0.row(41) << 321, 309;
    obst0.row(42) << 320, 309;
    obst0.row(43) << 320, 308;
    obst0.row(44) << 321, 308;
    obst0.row(45) << 322, 308;
    obst0.row(46) << 323, 308;
    obst0.row(47) << 324, 308;
    obst0.row(48) << 325, 308;
    obst0.row(49) << 326, 308;
    obst0.row(50) << 326, 307;
    obst0.row(51) << 325, 307;
    obst0.row(52) << 324, 307;
    obst0.row(53) << 323, 307;
    obst0.row(54) << 322, 307;
    obst0.row(55) << 321, 307;
    obst0.row(56) << 320, 307;
    obst0.row(57) << 319, 307;
    obst0.row(58) << 318, 307;
    obst0.row(59) << 319, 308;
    obst0.row(60) << 319, 306;
    obst0.row(61) << 320, 306;
    obst0.row(62) << 321, 306;
    obst0.row(63) << 322, 306;
    obst0.row(64) << 323, 306;
    obst0.row(65) << 324, 306;
    obst0.row(66) << 325, 306;
    obst0.row(67) << 324, 305;
    obst0.row(68) << 323, 305;
    obst0.row(69) << 322, 305;
    obst0.row(70) << 321, 305;
    obst0.row(71) << 320, 305;
    obst0.row(72) << 321, 304;
    obst0.row(73) << 322, 304;
    obst0.row(74) << 323, 304;
    obst0.row(75) << 317, 309;
    obst0.row(76) << 325, 313;
    obst0.row(77) << 324, 313;
    obst0.row(78) << 323, 313;
    obst0.row(79) << 322, 313;
    obst0.row(80) << 321, 313;
    obst0.row(81) << 320, 313;
    obst0.row(82) << 319, 313;
    obst0.row(83) << 318, 313;
    obst0.row(84) << 317, 313;
    obst0.row(85) << 316, 313;
    obst0.row(86) << 315, 313;
    obst0.row(87) << 316, 314;
    obst0.row(88) << 317, 314;
    obst0.row(89) << 318, 314;
    obst0.row(90) << 319, 314;
    obst0.row(91) << 320, 314;
    obst0.row(92) << 321, 314;
    obst0.row(93) << 322, 314;
    obst0.row(94) << 323, 314;
    obst0.row(95) << 324, 314;
    obst0.row(96) << 323, 315;
    obst0.row(97) << 322, 315;
    obst0.row(98) << 321, 315;
    obst0.row(99) << 320, 315;
    obst0.row(100) << 319, 315;
    obst0.row(101) << 318, 315;
    obst0.row(102) << 317, 315;
    obst0.row(103) << 318, 316;
    obst0.row(104) << 319, 316;
    obst0.row(105) << 320, 316;
    obst0.row(106) << 321, 316;
    obst0.row(107) << 322, 316;

    Blob obst1(84,2);
    obst1.row(0) << 319, 343;
    obst1.row(1) << 320, 343;
    obst1.row(2) << 321, 343;
    obst1.row(3) << 322, 343;
    obst1.row(4) << 323, 343;
    obst1.row(5) << 324, 343;
    obst1.row(6) << 325, 343;
    obst1.row(7) << 326, 343;
    obst1.row(8) << 327, 343;
    obst1.row(9) << 328, 343;
    obst1.row(10) << 328, 344;
    obst1.row(11) << 329, 344;
    obst1.row(12) << 329, 345;
    obst1.row(13) << 330, 345;
    obst1.row(14) << 330, 346;
    obst1.row(15) << 329, 346;
    obst1.row(16) << 328, 346;
    obst1.row(17) << 327, 346;
    obst1.row(18) << 326, 346;
    obst1.row(19) << 325, 346;
    obst1.row(20) << 324, 346;
    obst1.row(21) << 323, 346;
    obst1.row(22) << 322, 346;
    obst1.row(23) << 322, 345;
    obst1.row(24) << 323, 345;
    obst1.row(25) << 324, 345;
    obst1.row(26) << 325, 345;
    obst1.row(27) << 326, 345;
    obst1.row(28) << 327, 345;
    obst1.row(29) << 328, 345;
    obst1.row(30) << 327, 344;
    obst1.row(31) << 326, 344;
    obst1.row(32) << 325, 344;
    obst1.row(33) << 324, 344;
    obst1.row(34) << 323, 344;
    obst1.row(35) << 322, 344;
    obst1.row(36) << 321, 344;
    obst1.row(37) << 320, 344;
    obst1.row(38) << 321, 345;
    obst1.row(39) << 324, 347;
    obst1.row(40) << 325, 347;
    obst1.row(41) << 326, 347;
    obst1.row(42) << 327, 347;
    obst1.row(43) << 328, 347;
    obst1.row(44) << 329, 347;
    obst1.row(45) << 330, 347;
    obst1.row(46) << 330, 348;
    obst1.row(47) << 329, 348;
    obst1.row(48) << 328, 348;
    obst1.row(49) << 327, 348;
    obst1.row(50) << 326, 348;
    obst1.row(51) << 325, 348;
    obst1.row(52) << 324, 348;
    obst1.row(53) << 323, 348;
    obst1.row(54) << 324, 349;
    obst1.row(55) << 325, 349;
    obst1.row(56) << 326, 349;
    obst1.row(57) << 327, 349;
    obst1.row(58) << 328, 349;
    obst1.row(59) << 329, 349;
    obst1.row(60) << 328, 350;
    obst1.row(61) << 327, 350;
    obst1.row(62) << 326, 350;
    obst1.row(63) << 325, 350;
    obst1.row(64) << 326, 351;
    obst1.row(65) << 327, 351;
    obst1.row(66) << 327, 342;
    obst1.row(67) << 326, 342;
    obst1.row(68) << 325, 342;
    obst1.row(69) << 324, 342;
    obst1.row(70) << 323, 342;
    obst1.row(71) << 322, 342;
    obst1.row(72) << 321, 342;
    obst1.row(73) << 320, 342;
    obst1.row(74) << 321, 341;
    obst1.row(75) << 322, 341;
    obst1.row(76) << 323, 341;
    obst1.row(77) << 324, 341;
    obst1.row(78) << 325, 341;
    obst1.row(79) << 326, 341;
    obst1.row(80) << 325, 340;
    obst1.row(81) << 324, 340;
    obst1.row(82) << 323, 340;
    obst1.row(83) << 322, 340;

    Blob obst2(220,2);
    obst2.row(0) << 340, 333;
    obst2.row(1) << 341, 333;
    obst2.row(2) << 342, 333;
    obst2.row(3) << 343, 333;
    obst2.row(4) << 344, 333;
    obst2.row(5) << 345, 333;
    obst2.row(6) << 346, 333;
    obst2.row(7) << 347, 333;
    obst2.row(8) << 348, 333;
    obst2.row(9) << 349, 333;
    obst2.row(10) << 350, 333;
    obst2.row(11) << 351, 333;
    obst2.row(12) << 352, 333;
    obst2.row(13) << 353, 333;
    obst2.row(14) << 353, 334;
    obst2.row(15) << 352, 334;
    obst2.row(16) << 351, 334;
    obst2.row(17) << 350, 334;
    obst2.row(18) << 349, 334;
    obst2.row(19) << 348, 334;
    obst2.row(20) << 347, 334;
    obst2.row(21) << 346, 334;
    obst2.row(22) << 345, 334;
    obst2.row(23) << 344, 334;
    obst2.row(24) << 343, 334;
    obst2.row(25) << 342, 334;
    obst2.row(26) << 341, 334;
    obst2.row(27) << 340, 334;
    obst2.row(28) << 340, 335;
    obst2.row(29) << 341, 335;
    obst2.row(30) << 342, 335;
    obst2.row(31) << 343, 335;
    obst2.row(32) << 344, 335;
    obst2.row(33) << 345, 335;
    obst2.row(34) << 346, 335;
    obst2.row(35) << 346, 336;
    obst2.row(36) << 345, 336;
    obst2.row(37) << 344, 336;
    obst2.row(38) << 343, 336;
    obst2.row(39) << 342, 336;
    obst2.row(40) << 341, 336;
    obst2.row(41) << 340, 336;
    obst2.row(42) << 340, 337;
    obst2.row(43) << 341, 337;
    obst2.row(44) << 342, 337;
    obst2.row(45) << 343, 337;
    obst2.row(46) << 344, 337;
    obst2.row(47) << 345, 337;
    obst2.row(48) << 346, 337;
    obst2.row(49) << 347, 337;
    obst2.row(50) << 347, 338;
    obst2.row(51) << 346, 338;
    obst2.row(52) << 345, 338;
    obst2.row(53) << 344, 338;
    obst2.row(54) << 343, 338;
    obst2.row(55) << 342, 338;
    obst2.row(56) << 341, 338;
    obst2.row(57) << 342, 339;
    obst2.row(58) << 343, 339;
    obst2.row(59) << 344, 339;
    obst2.row(60) << 345, 339;
    obst2.row(61) << 346, 339;
    obst2.row(62) << 345, 340;
    obst2.row(63) << 344, 340;
    obst2.row(64) << 343, 340;
    obst2.row(65) << 344, 341;
    obst2.row(66) << 348, 335;
    obst2.row(67) << 349, 335;
    obst2.row(68) << 350, 335;
    obst2.row(69) << 351, 335;
    obst2.row(70) << 352, 335;
    obst2.row(71) << 351, 336;
    obst2.row(72) << 350, 336;
    obst2.row(73) << 349, 336;
    obst2.row(74) << 350, 337;
    obst2.row(75) << 354, 332;
    obst2.row(76) << 355, 332;
    obst2.row(77) << 356, 332;
    obst2.row(78) << 357, 332;
    obst2.row(79) << 357, 331;
    obst2.row(80) << 358, 331;
    obst2.row(81) << 358, 330;
    obst2.row(82) << 359, 330;
    obst2.row(83) << 358, 329;
    obst2.row(84) << 357, 329;
    obst2.row(85) << 356, 329;
    obst2.row(86) << 355, 329;
    obst2.row(87) << 354, 329;
    obst2.row(88) << 353, 329;
    obst2.row(89) << 352, 329;
    obst2.row(90) << 351, 329;
    obst2.row(91) << 351, 328;
    obst2.row(92) << 352, 328;
    obst2.row(93) << 353, 328;
    obst2.row(94) << 354, 328;
    obst2.row(95) << 355, 328;
    obst2.row(96) << 356, 328;
    obst2.row(97) << 357, 328;
    obst2.row(98) << 356, 327;
    obst2.row(99) << 355, 327;
    obst2.row(100) << 354, 327;
    obst2.row(101) << 353, 327;
    obst2.row(102) << 352, 327;
    obst2.row(103) << 351, 327;
    obst2.row(104) << 350, 327;
    obst2.row(105) << 350, 328;
    obst2.row(106) << 349, 329;
    obst2.row(107) << 348, 329;
    obst2.row(108) << 347, 329;
    obst2.row(109) << 346, 329;
    obst2.row(110) << 345, 329;
    obst2.row(111) << 344, 329;
    obst2.row(112) << 344, 330;
    obst2.row(113) << 345, 330;
    obst2.row(114) << 346, 330;
    obst2.row(115) << 347, 330;
    obst2.row(116) << 348, 330;
    obst2.row(117) << 349, 330;
    obst2.row(118) << 350, 330;
    obst2.row(119) << 350, 331;
    obst2.row(120) << 351, 331;
    obst2.row(121) << 351, 332;
    obst2.row(122) << 352, 332;
    obst2.row(123) << 353, 331;
    obst2.row(124) << 354, 331;
    obst2.row(125) << 355, 331;
    obst2.row(126) << 356, 331;
    obst2.row(127) << 356, 330;
    obst2.row(128) << 357, 330;
    obst2.row(129) << 355, 330;
    obst2.row(130) << 354, 330;
    obst2.row(131) << 353, 330;
    obst2.row(132) << 352, 330;
    obst2.row(133) << 350, 332;
    obst2.row(134) << 349, 332;
    obst2.row(135) << 348, 332;
    obst2.row(136) << 347, 332;
    obst2.row(137) << 346, 332;
    obst2.row(138) << 345, 332;
    obst2.row(139) << 344, 332;
    obst2.row(140) << 343, 332;
    obst2.row(141) << 342, 332;
    obst2.row(142) << 341, 332;
    obst2.row(143) << 342, 331;
    obst2.row(144) << 343, 331;
    obst2.row(145) << 344, 331;
    obst2.row(146) << 345, 331;
    obst2.row(147) << 346, 331;
    obst2.row(148) << 347, 331;
    obst2.row(149) << 348, 331;
    obst2.row(150) << 349, 331;
    obst2.row(151) << 343, 330;
    obst2.row(152) << 345, 328;
    obst2.row(153) << 346, 328;
    obst2.row(154) << 347, 328;
    obst2.row(155) << 348, 328;
    obst2.row(156) << 350, 326;
    obst2.row(157) << 351, 326;
    obst2.row(158) << 352, 326;
    obst2.row(159) << 353, 326;
    obst2.row(160) << 354, 326;
    obst2.row(161) << 355, 326;
    obst2.row(162) << 356, 326;
    obst2.row(163) << 357, 326;
    obst2.row(164) << 358, 326;
    obst2.row(165) << 358, 325;
    obst2.row(166) << 359, 325;
    obst2.row(167) << 359, 324;
    obst2.row(168) << 360, 324;
    obst2.row(169) << 360, 323;
    obst2.row(170) << 361, 323;
    obst2.row(171) << 360, 322;
    obst2.row(172) << 359, 322;
    obst2.row(173) << 358, 322;
    obst2.row(174) << 357, 322;
    obst2.row(175) << 356, 322;
    obst2.row(176) << 355, 322;
    obst2.row(177) << 354, 322;
    obst2.row(178) << 353, 322;
    obst2.row(179) << 352, 322;
    obst2.row(180) << 352, 323;
    obst2.row(181) << 353, 323;
    obst2.row(182) << 354, 323;
    obst2.row(183) << 355, 323;
    obst2.row(184) << 356, 323;
    obst2.row(185) << 357, 323;
    obst2.row(186) << 358, 323;
    obst2.row(187) << 359, 323;
    obst2.row(188) << 358, 324;
    obst2.row(189) << 357, 324;
    obst2.row(190) << 356, 324;
    obst2.row(191) << 355, 324;
    obst2.row(192) << 354, 324;
    obst2.row(193) << 353, 324;
    obst2.row(194) << 352, 324;
    obst2.row(195) << 351, 324;
    obst2.row(196) << 350, 324;
    obst2.row(197) << 350, 325;
    obst2.row(198) << 351, 325;
    obst2.row(199) << 352, 325;
    obst2.row(200) << 353, 325;
    obst2.row(201) << 354, 325;
    obst2.row(202) << 355, 325;
    obst2.row(203) << 356, 325;
    obst2.row(204) << 357, 325;
    obst2.row(205) << 351, 323;
    obst2.row(206) << 353, 321;
    obst2.row(207) << 354, 321;
    obst2.row(208) << 355, 321;
    obst2.row(209) << 356, 321;
    obst2.row(210) << 357, 321;
    obst2.row(211) << 358, 321;
    obst2.row(212) << 359, 321;
    obst2.row(213) << 358, 320;
    obst2.row(214) << 357, 320;
    obst2.row(215) << 356, 320;
    obst2.row(216) << 355, 320;
    obst2.row(217) << 354, 320;
    obst2.row(218) << 356, 333;
    obst2.row(219) << 355, 333;

    Blob obst3(85,2);
    obst3.row(0) << 350, 324;
    obst3.row(1) << 351, 324;
    obst3.row(2) << 352, 324;
    obst3.row(3) << 353, 324;
    obst3.row(4) << 354, 324;
    obst3.row(5) << 355, 324;
    obst3.row(6) << 356, 324;
    obst3.row(7) << 357, 324;
    obst3.row(8) << 357, 323;
    obst3.row(9) << 358, 323;
    obst3.row(10) << 357, 322;
    obst3.row(11) << 356, 322;
    obst3.row(12) << 355, 322;
    obst3.row(13) << 354, 322;
    obst3.row(14) << 353, 322;
    obst3.row(15) << 352, 322;
    obst3.row(16) << 352, 323;
    obst3.row(17) << 353, 323;
    obst3.row(18) << 354, 323;
    obst3.row(19) << 355, 323;
    obst3.row(20) << 356, 323;
    obst3.row(21) << 351, 323;
    obst3.row(22) << 353, 321;
    obst3.row(23) << 354, 321;
    obst3.row(24) << 355, 321;
    obst3.row(25) << 356, 321;
    obst3.row(26) << 355, 320;
    obst3.row(27) << 354, 320;
    obst3.row(28) << 356, 325;
    obst3.row(29) << 355, 325;
    obst3.row(30) << 354, 325;
    obst3.row(31) << 353, 325;
    obst3.row(32) << 352, 325;
    obst3.row(33) << 351, 325;
    obst3.row(34) << 350, 325;
    obst3.row(35) << 350, 326;
    obst3.row(36) << 351, 326;
    obst3.row(37) << 352, 326;
    obst3.row(38) << 353, 326;
    obst3.row(39) << 354, 326;
    obst3.row(40) << 355, 326;
    obst3.row(41) << 356, 326;
    obst3.row(42) << 356, 327;
    obst3.row(43) << 355, 327;
    obst3.row(44) << 354, 327;
    obst3.row(45) << 353, 327;
    obst3.row(46) << 352, 327;
    obst3.row(47) << 351, 327;
    obst3.row(48) << 350, 327;
    obst3.row(49) << 350, 328;
    obst3.row(50) << 351, 328;
    obst3.row(51) << 352, 328;
    obst3.row(52) << 353, 328;
    obst3.row(53) << 354, 328;
    obst3.row(54) << 355, 328;
    obst3.row(55) << 356, 328;
    obst3.row(56) << 357, 328;
    obst3.row(57) << 357, 329;
    obst3.row(58) << 358, 329;
    obst3.row(59) << 358, 330;
    obst3.row(60) << 359, 330;
    obst3.row(61) << 358, 331;
    obst3.row(62) << 357, 331;
    obst3.row(63) << 356, 331;
    obst3.row(64) << 355, 331;
    obst3.row(65) << 354, 331;
    obst3.row(66) << 353, 331;
    obst3.row(67) << 353, 330;
    obst3.row(68) << 354, 330;
    obst3.row(69) << 355, 330;
    obst3.row(70) << 356, 330;
    obst3.row(71) << 357, 330;
    obst3.row(72) << 356, 329;
    obst3.row(73) << 355, 329;
    obst3.row(74) << 354, 329;
    obst3.row(75) << 353, 329;
    obst3.row(76) << 352, 329;
    obst3.row(77) << 351, 329;
    obst3.row(78) << 352, 330;
    obst3.row(79) << 354, 332;
    obst3.row(80) << 355, 332;
    obst3.row(81) << 356, 332;
    obst3.row(82) << 357, 332;
    obst3.row(83) << 356, 333;
    obst3.row(84) << 355, 333;

    std::vector<Blob> stack_of_obst;
    stack_of_obst.push_back(obst0);
    stack_of_obst.push_back(obst1);
    stack_of_obst.push_back(obst2);
    stack_of_obst.push_back(obst3);

    State state_attractor; state_attractor << 380,331,0;

    Eigen::Matrix<float, 5, 1> limits_quiver;
    limits_quiver << 300.02, 420.02, 280.02, 400.02, 1;

    compute_stream_border(limits_quiver, state_attractor, stack_of_obst, -1);

}

