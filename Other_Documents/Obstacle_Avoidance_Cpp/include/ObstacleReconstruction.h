#ifndef OBSTACLERECONSTRUCTION_H_INCLUDED
#define OBSTACLERECONSTRUCTION_H_INCLUDED

#include <iostream> // In-Out streams
#include <fstream>  // To write data into files
#include <eigen3/Eigen/Dense> // For Linear Algebra
#include <cmath> // Basic math functions
#include <string>
#include <queue>
#include "ObstacleAvoidance.h"

// OpenCV functions
/*#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/core/eigen.hpp>*/



typedef Eigen::Matrix<float, Eigen::Dynamic, 5> MatrixX5f;

using Blob   = Eigen::MatrixX2i; // [x, y]
using Border = MatrixX5f;  // [x, y, type, charac_1, charac_2;]
using Point  = Eigen::Matrix<int, 1, 2>;
using Grid   = Eigen::MatrixXi;
using State  = Eigen::Matrix<float, 3, 1>; // State is an alias to represent a column vector with three components

struct PointFill // Simple structure of a 2D point
{
    int x;
    int y;
};


Point get_center(Blob const& blob); // get the center cell of a blob of cells

Point get_random(Blob const& blob, int const& limit=0); // get a random cell of a blob of cells

bool isPart(Blob const& blob, float const& x, float const& y); // check if a cell is part of a blob

bool isPartBorder(Border const& border, float const& x, float const& y); // check if a cell belongs to the input border starting from the first row

bool isPartBorderReverse(Border const& border, float const& x, float const& y); // check if a cell belongs to the input border starting from the last row

bool check_direction(Blob const& obstacle, Point const& point, int const& direction); // check if the targeted cell is part of the obstacle

Blob fill_gaps(Blob const& obstacle); // fill the 1-cell wide gaps in an obstacle

Blob fill_gaps_with_grid(Blob const& obstacle, Grid & occupancy_grid); // fill the 1-cell wide gaps in an obstacle and update occupancy grid

void update_border(Border & border, Point const& position, int const& previous_direction, int const& next_direction); // add elements to the border to follow the sides of the obstacles depending on the direction taken

void add_to_border(Border & border, int const& x, int const& y, int const& type, float const& charac_1, float const& charac_2); // add the given cell to the border with its characteristics

void update_position(Point & current_position, int const& direction); // update the position of a point after a step in a given direction

Border compute_border_and_fill(Blob const& obstacle, Point const& start_point, Grid & occupancy_grid); // follow the sides of the obstacle to create its border incrementally and update occupancy grid

Border compute_border(Blob const& obstacle, Point const& start_point, bool const& already_filled=false); // follow the sides of the obstacle to create its border incrementally

void display_border(Blob const& obstacle, Border const& border); // display the border in the console (simplified graphic representation)

void remove_end_duplicate(Border & border); // Function to remove the few duplicates at the beginning/end of the border creation because we come back to the same cell

Blob expand_obstacle(Blob const& obstacle, int const& n_cells); // expand an obstacle by a given number of cells
// idea: border detection -> fill the border -> border detection -> fill the border -> ...   n_cells times

Grid expand_occupancy_grid(Grid const& grid, int const& n_cells, State const& state_robot, float const& limit_range, float const& size_of_cells); // expand the occupied cells of an occupancy grid by a given number of cells

Eigen::Matrix<float, 1, 6> find_closest_point(Eigen::Matrix<float,1,2>  const& robot, Border const& border); // find the closest cell of a given border

Eigen::Matrix<float, 4, 1> gamma_and_ref_vector(Eigen::Matrix<float,1,2>  robot, Eigen::Matrix<float, 1, 6> data_closest); // return the Gamma distance of a point knowing its closest border cell
// as well as the reference vector (point - projected_point_on_the_border)

void compute_quiver_border(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, Blob const& obstacle); // compute the velocity command for all points of a given grid
// used with Python for visualization purpose (Quiver function of Matlab)

// Just like compute_quiver but with a small variation to be able to plot a "streamplot" with Matplotlib
void compute_stream_border(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, std::vector<Blob> obstacles, int const& ID=-1);

void compute_morphing(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, std::vector<Blob> obstacles); // compute morphing between initial and circle spaces

void compute_trajectory_both_spaces(State const& state_robot, State const& state_attractor, std::vector<Blob> obstacles); // compute trajectory in initial and circle spaces

Eigen::Matrix<int, 2, 1> get_cell(float const& x, float const& y, float const& size_side_cell); // for a given point it returns the center of the closest cell of the grid

State next_step_single_obstacle_border(State const& state_robot, State const& state_attractor, Border const& border); // compute the velocity command of the robot (version with no limit distance)
// based on its relative position to the border of a single obstacle

State next_step_several_obstacles_border( State const& state_robot, State const& state_attractor, std::vector<Border> const& borders); // compute the velocity command of the robot with several obstacles (version with no limit distance)

Eigen::Matrix<float, 1, 2> get_projection_on_border(Eigen::Matrix<float,1,2>  robot, Eigen::Matrix<float, 1, 6> data_closest, float const& angle); // project a point on the border of an obstacle
// for a given point and knowing the closest border cell (position and line/arc of circle), one can project the point onto the border
// it is used to get the normal vector to the surface since normal vector = point - projected_point

void growing_obstacle(); // create a single blob of occupied cells to test the border creation of "compute_border"

void growing_several_obstacle(); // create several blobs of occupied cells to test the obstacle detection algorithm "detect_borders"

std::vector<Border> detect_borders(Grid & occupancy_grid, State const& state_robot); // scan an occupancy grid for obstacles and returns a vector containing the borders of all detected obstacles

std::vector<Blob> detect_blobs( Grid & occupancy_grid ); // scan an occupancy grid for obstacles and returns a vector containing the blobs of all detected obstacles

int other_side_obstacle(Border const& border, int const& row, int const& col); // return the index of the column on the other side an obstacle thanks to its border
// the scan for obstacles is done from left to right so by giving (row, col_left) on the left of the obstacle the algorithm returns col_right
// then the scan resumes from the (row, col_right) cell

void explore_obstacle( Blob & obstacle, Grid & occupancy_grid, int const& row, int const& col); // explore an obstacle to detect all its cells
// start from one cell and a recursive function gradually explores nearby cells and checks if they are occupied (i.e part of the obstacle)

Eigen::MatrixXf weights_special(State const& state_robot, Eigen::MatrixXf const& mat_gamma, int const& method, float const& limit_distance, bool const& do_not_normalize=false); // compute the relative weights of obstacles in the limit range

State next_step_special_weighted(State const& state_robot, State const& state_attractor, std::vector<Border> const& borders, float const& size_of_cells); // compute the velocity command for a given position of the robot/attractor/obstacles

Eigen::Matrix<float, 4, 1> next_step_special(State const& state_robot, State const& state_attractor, Border const& border); // compute the next step with the special method for a single obstacle
// output is velocity_command stacked with gamma

Eigen::Matrix<float, 6, 1> gamma_normal_projection(Eigen::Matrix<float,1,2> const& robot, Eigen::Matrix<float, 1, 6> const& data_closest); // [gamma, normal vector, projected point on border]

Eigen::Matrix<float, 1, 3> get_distances_surface(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float, 1, 2> const& proj_attractor, Border const& border); // minimum distance along surface

float get_max_gamma_distance(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float,1,2> const& normal, Border const& border); // maximum gamma distance that can be reached following the normal

Eigen::Matrix<float, 10, 1> get_point_circle_frame( float const& distance_proj, float const& distance_tot, float const& gamma_robot, float const& gamma_max, float const& gamma_attractor, float const& gamma_max_attractor, int direction);

void fillGrid(PointFill orig, Grid & occupancy_grid); // fill the holes in the occupied blobs of an occupancy grid

void draw_circle(Grid & grid, int const& x, int const& y, int const& radius); // draw a disk of occupied cells in an occupancy grid

Eigen::MatrixXi readMatrix(std::string filename);

Eigen::MatrixXi Line( float x1, float y1, float x2, float y2);

bool check_if_on_way(Eigen::MatrixXi const& line, Border const& border);

#endif // OBSTACLERECONSTRUCTION_H_INCLUDED
