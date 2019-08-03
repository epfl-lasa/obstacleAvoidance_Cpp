/*
Implementation by Pierre-Alexandre Léziart of the method described in (Huber and al., 2019)
Generalization to non-star-shaped obstacles
LASA laboratory, EPFL, Spring 2019
Mail: pierre-alexandre.leziart [at] epfl [dot] ch
Contains functions for obstacle avoidance algorithm
*/

#ifndef OBSTACLERECONSTRUCTION_H_INCLUDED
#define OBSTACLERECONSTRUCTION_H_INCLUDED

#include <iostream>             // In-Out streams
#include <fstream>              // To write data into files
#include <eigen3/Eigen/Dense>   // For Linear Algebra
#include <cmath>                // Basic math functions
#include <string>               // Standard String type
#include <queue>                // Standard Queue type
#include "ObstacleAvoidance.h"  // Use some functions from ObstacleAvoidance.h

typedef Eigen::Matrix<float, Eigen::Dynamic, 5> MatrixX5f; /**< An Eigen matrix with 5 columns and a dynamic number of rows */

using Blob   = Eigen::MatrixX2i;           // Each row contains the [x, y] coordinates of a cell belonging to the blob of cells
using Border = MatrixX5f;                  // Each row contains the [x, y, type, charac_1, charac_2] information about a cell of the surface of an obstacle
using Point  = Eigen::Matrix<int, 1, 2>;   // [x, y] coordinates of a point
using Grid   = Eigen::MatrixXi;            // Eigen matrix used to represent an occupancy grid (0 for free cells and 100 for occupied cells)
using State  = Eigen::Matrix<float, 3, 1>; // State is an alias to represent a column vector with three components

/** Simple structure to represent a 2D point */
struct PointFill
{
    int x; /**< Coordinate of the point along the X axis */
    int y; /**< Coordinate of the point along the Y axis */
};

/**
 * Get the center cell of a blob of cells by averaging the X and Y coordinates of all cells
 *
 * @param blob Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the blob
 * @return A Point, an Eigen matrix of size (1,2) containing the [x,y] coordinates of the point in the middle of the blob
 */
Point get_center(Blob const& blob);

/**
 * Get the coordinates of a random cell belonging to a given blob of cells
 *
 * @param blob Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the blob
 * @param limit Restrict the random choice to the first cells of the blob
 *        If limit = 0 then any cell of the blob can be chosen randomly
 *        If limit = K then the random choice is made among the K first cells (row 0 to K-1)
 * @return A Point, an Eigen matrix of size (1,2) containing the [x,y] coordinates of a randomly chosen cell
 */
Point get_random(Blob const& blob, int const& limit=0);

/**
 * Check if a cell belongs to a given blob of cells
 *
 * @param blob Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the blob
 * @param x The X coordinate of the cell that has to be checked
 * @param y The Y coordinate of the cell that has to be checked
 * @return True if the cell with coordinates [x,y] belongs to the input blob, False otherwise
 */
bool isPart(Blob const& blob, float const& x, float const& y);

/**
 * Check if a cell belongs to the input border. Checking starts from the first row and goes down in the matrix.
 *
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @param x The X coordinate of the cell that has to be checked
 * @param y The Y coordinate of the cell that has to be checked
 * @return True if the cell with coordinates [x,y] belongs to the input border, False otherwise
 */
bool isPartBorder(Border const& border, float const& x, float const& y);

/**
 * Check if a cell belongs to the input border. Checking starts from the last row and goes up in the matrix.
 *
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @param x The X coordinate of the cell that has to be checked
 * @param y The Y coordinate of the cell that has to be checked
 * @return True if the cell with coordinates [x,y] belongs to the input border, False otherwise
 */
bool isPartBorderReverse(Border const& border, float const& x, float const& y);

/**
 * Check if the targeted cell belongs to the obstacle.
 *
 * @param obstacle Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the input obstacle
 * @param point A Point, an Eigen matrix of size (1,2) containing the [x,y] coordinates of the cell the iterator is on.
 * @param direction In which direction the targeted cell is.
 *                  Directions are:  3 2 1
 *                                   4   0
 *                                   5 6 7
 *                  With positive X toward the right and positive Y toward the top
 *                  For instance if point is at (2,2) and direction is 3 then it checks if cell (1,3) is part of the input blob "obstacle"
 * @return True if the targeted cell belongs to the input obstacle, False otherwise
 */
bool check_direction(Blob const& obstacle, Point const& point, int const& direction);

/**
 * Fill the 1-cell wide gaps in an obstacle
 * For instance . . . . . . becomes . . . . . . with . free cells and X occupied cells
 *              . X . X X .         . X X X X .
 *              . X . . X .         . X X X X .
 *              . X X X X .         . X X X X .
 *              . . . . . .         . . . . . .
 *
 * @param obstacle Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the input obstacle
 * @return A Blob, an Eigen matrix of size (N+M,2) containing the N cells of the obstacle and the M additional cells that have been filled
 */
Blob fill_gaps(Blob const& obstacle);

/**
 * Fill the 1-cell wide gaps in an obstacle and directly update the input occupancy grid
 * For instance . . . . . . becomes . . . . . . with . free cells and X occupied cells
 *              . X . X X .         . X X X X .
 *              . X . . X .         . X X X X .
 *              . X X X X .         . X X X X .
 *              . . . . . .         . . . . . .
 *
 * @param obstacle Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the input obstacle
 * @param occupancy_grid Eigen matrix that represents the occupancy grid the obstacle is in.
 * @return A Blob, an Eigen matrix of size (N+M,2) containing the N cells of the obstacle and the M additional cells that have been filled
 */
Blob fill_gaps_with_grid(Blob const& obstacle, Grid & occupancy_grid);

/**
 * Add new cells (lines or inner/outer corners) to the border depending on the previous direction taken to follow the surface and the next direction.
 * A big decision tree is used to handle all (previous, next) possibilities.
 *
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 *               At the end of the function, this border has been updated and is now an Eigen matrix of size (N+M,5) containing the N input surface cells and the M new cells that have just been added
 * @param point A Point, an Eigen matrix of size (1,2) containing the [x,y] coordinates of the cell the iterator is on
 * @param previous_direction The direction that was taken by the iterator to get to the current cell
 * @param next_direction The next direction that will be taken by the iterator to follow the surface of the obstacle
 */
void update_border(Border & border, Point const& position, int const& previous_direction, int const& next_direction); // add elements to the border to follow the sides of the obstacles depending on the direction taken

/**
 * Add a new cell to a border using its characteristics
 *
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 *               At the end of the function, this border has been updated and is now an Eigen matrix of size (N+1,5) containing the N input surface cells and the new cell that has just been added
 * @param x The X coordinate of the new cell
 * @param y The Y coordinate of the new cell
 * @param type The type of the new cell (1 for straight line, 2 for outer corner, 3 for inner corner)
 * @param charac_1 The first characteristic of the new cell (see report for more details)
 * @param charac_2 The second characteristic of the new cell (see report for more details)
 */
void add_to_border(Border & border, int const& x, int const& y, int const& type, float const& charac_1, float const& charac_2);

/**
 * Update the position of a point after a step in a given direction
 *
 * @param current_position A Point, an Eigen matrix of size (1,2) containing the [x,y] coordinates of the cell the iterator is on
 *                          At the end of the function, coordinates have been updated to the new position of the iterator
 * @param direction The direction taken by the iterator to follow the surface of the obstacle
 */
void update_position(Point & current_position, int const& direction);


/**
 * Run fill_gaps_with_grid function to fill the 1-cell wide gaps in an obstacle and directly update the input occupancy grid
 * Then run compute_border function on the filled obstacle and starting from start_point
 *
 * @param obstacle Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the input obstacle
 * @param point A Point, an Eigen matrix of size (1,2) containing the [x,y] coordinates of the cell the iterator starts on to follow the surface of the obstacle
 * @param occupancy_grid Eigen matrix that represents the occupancy grid the obstacle is in.
 * @see fill_gaps_with_grid
 * @see compute_border
 */
Border compute_border_and_fill(Blob const& obstacle, Point const& start_point, Grid & occupancy_grid);

/**
 * Follow the surface of an obstacle to reconstruct it incrementally
 *
 * @param obstacle Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the input obstacle
 * @param point A Point, an Eigen matrix of size (1,2) containing the [x,y] coordinates of the cell the iterator starts on to follow the surface of the obstacle
 * @param already_filled Boolean, true if the 1-cell wide gaps in the input obstacle have already been filled, false otherwise (default)
 *        If set to false then the fill_gaps function is called to fill the 1-cell wide gaps in the input obstacle
 * @see fill_gaps
 */
Border compute_border(Blob const& obstacle, Point const& start_point, bool const& already_filled=false);

/**
 * Display an obstacle and its reconstructed surface in the console (simplified graphic representation)
 *
 * @param obstacle Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the input obstacle
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 */
void display_border(Blob const& obstacle, Border const& border);

/**
 * Remove the few duplicates at the beginning/end of the surface reconstruction process because the iterator gets back to the starting cell
 *
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 *               At the end of the function, this border has been updated and is now an Eigen matrix of size (N-M,5) containing the input surface cells except the M removed duplicates
 */
void remove_end_duplicate(Border & border);

/**
 * Expand an obstacle by a given number of cells
 *
 * @param obstacle Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the input obstacle
 * @param n_cells Number of cells the obstacle should be expanded by.
 * @return A Blob, an Eigen matrix of size (N+M,2) with each row containing the [x,y] coordinates of a cell belonging to the input obstacle and the M new cells belonging to the expanded obstacle
 */
Blob expand_obstacle(Blob const& obstacle, int const& n_cells);

/**
 * Expand all the occupied cells of an occupancy grid by a given number of cells
 *
 * @param occupancy_grid Eigen matrix that represents the occupancy grid the obstacle is in.
 * @param n_cells Number of cells the obstacle should be expanded by.
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot.
 * @param limit_range Only obstacles for which the Gamma distance to the robot is inferior to limit_range are considered, the other ones are discarded
 * @param size_of_cells The size of cells in the occupancy grid space, always 1 [no unit] so not really used.
 * @return A Grid, an Eigen matrix that represents the occupancy grid the obstacle is in after expansion and with only obstacles in the limit range
 */
Grid expand_occupancy_grid(Grid const& grid, int const& n_cells, State const& state_robot, float const& limit_range, float const& size_of_cells);

/**
 * Find the closest cell from the robot and belonging to a given reconstructed surface/border
 *
 * @param robot Eigen matrix of size (1,2) containing the [x,y] coordinates of the robot
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @return An Eigen matrix of size (1,6) that contains information about the closest cell [x, y, type, charac_1, charac_2, square of euclidian distance from the robot]
 */
Eigen::Matrix<float, 1, 6> find_closest_point(Eigen::Matrix<float,1,2>  const& robot, Border const& border);

/**
 * Return the Gamma distance from the obstacle and the reference vector (normalized orthogonal vector to the surface that goes through the robot)
 *
 * @param robot Eigen matrix of size (1,2) containing the [x,y] coordinates of the robot
 * @param data_closest Eigen matrix of size (1,6) that contains information about the closest cell [x, y, type, charac_1, charac_2, square of euclidian distance from the robot]
 * @return An Eigen matrix of size (1,4) that contains [Gamma distance, x_ref_vector, y_ref_vector, theta_ref_vector]. theta_ref_vector is always equal to 0 as we work only in the (x,y) plane
 */
Eigen::Matrix<float, 4, 1> gamma_and_ref_vector(Eigen::Matrix<float,1,2>  robot, Eigen::Matrix<float, 1, 6> data_closest);

/**
 * Create a grid of the workspace and compute the velocity command for each point of this grid for a single obstacle using the generalised method
 * Results are stored in a text file that has to be opened with Python to plot a quiver graph with matplotlib
 *
 * @param limits Eigen matrix of size (5,1) containing the [x_min, x_max, y_min, y_max, step] information that defines the grid
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @param obstacle Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the input obstacle
 */
void compute_quiver_border(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, Blob const& obstacle);

/**
 * Create a grid of the workspace and compute the velocity command for each point of this grid for various obstacles using the generalised method
 * Results are stored in a text file that has to be opened with Python to plot a streamplot graph with matplotlib
 *
 * @param limits Eigen matrix of size (5,1) containing the [x_min, x_max, y_min, y_max, step] information that defines the grid
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @param obstacles Vector of Eigen matrices of size (Ni,2) with each row containing the [x,y] coordinates of one of the Ni cells of obstacle number i
 * @param ID Identification number of the current frame to make a .gif for instance
 */
void compute_stream_border(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, std::vector<Blob> obstacles, int const& ID=-1);

/**
 * Compute morphing between initial and circle spaces. The idea was first to get the position of all points in the workspace both in initial (t=0) and circle (t=1) spaces.
 * Then t could vary between 0 and 1 to do an interpolation of the position of the points and to have a nice visualization of the morphing with a video. It does not look good...
 * Results are stored in a text file that has to be opened with Python to do the interpolation with matplotlib
 *
 * @param limits Eigen matrix of size (5,1) containing the [x_min, x_max, y_min, y_max, step] information that defines the grid
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @param obstacles Vector of Eigen matrices of size (Ni,2) with each row containing the [x,y] coordinates of one of the Ni cells of obstacle number i
 */
void compute_morphing(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, std::vector<Blob> obstacles);

/**
 * Spawn a robot at a given position and make it follow the velocity stream with a given time step and a given number of iterations
 * Successive positions of the robot in initial and circle spaces are stored in a text file that has to be opened with Python
 *
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the robot
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor
 * @param obstacles Vector of Eigen matrices of size (Ni,2) with each row containing the [x,y] coordinates of one of the Ni cells of obstacle number i
 */
void compute_trajectory_both_spaces(State const& state_robot, State const& state_attractor, std::vector<Blob> obstacles);

/**
 * Return the center of the closest cell of the grid for a given position
 * For instance for cells with a side of 0.2 m, the cell (0,0) is centered on position (0,0) in real world and include all points with x in [-0.1, 0.1] and y in [-0.1, 0.1]
 * The cell (1,0) is centered on position (0.2,0) in real world and include all points with x in [0.1, 0.3] and y in [-0.1, 0.1]
 * The cell (0,1) is centered on position (0,0.2) in real world and include all points with x in [-0.1, 0.1] and y in [0.1, 0.3]
 *
 * @param x Position of the point along x in the real world
 * @param y Position of the point along y in the real world
 * @param size_side_cell Size of the cells of the grid
 * @return Eigen matrices of size (2,1) containing the position of the cell the point is standing on in the occupancy grid
 */
Eigen::Matrix<int, 2, 1> get_cell(float const& x, float const& y, float const& size_side_cell);

/**
 * Compute the velocity command of the robot depending on its relative position to a single obstacle (version with no limit distance to consider the obstacle)
 * Reconstruction of the surface based on straight lines, inner and outer corners
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @return Eigen matrix of size (3,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta]
 */
State next_step_single_obstacle_border(State const& state_robot, State const& state_attractor, Border const& border);

/**
 * Compute the velocity command of the robot depending on its relative position to several obstacles (version with no limit distance to consider the obstacle)
 * Reconstruction of surfaces based on straight lines, inner and outer corners
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param borders Vector of Eigen matrices of size (Ni,5) containing information about the Ni surface cells of obstacle number i with [x, y, type, charac1, charac2] format
 *        x and y are the coordinates of the cell in the occupancy grid
 *        type is 1 for a straight cell, 2 for an outer corner, 3 for an inner corner (see report)
 *        charac1 and charac2 contain information specific to each type of cell (see report)
 * @return Eigen matrix of size (3,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta]
 */
State next_step_several_obstacles_border( State const& state_robot, State const& state_attractor, std::vector<Border> const& borders);

/**
 * Project a point on the reconstructed surface of an obstacle. Knowing the position of the closest surface cell and if it is a line/arc of circle, one can project the point on the surface
 *
 * @param robot Eigen matrix of size (1,2) containing the [x,y] coordinates of the robot
 * @param data_closest Eigen matrix of size (1,6) that contains information about the closest cell [x, y, type, charac_1, charac_2, square of euclidian distance from the robot]
 * @param angle Angle of the normal vector to the surface that goes through the robot (in radian)
 * @return Eigen matrix of size (3,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta]
 */
Eigen::Matrix<float, 1, 2> get_projection_on_border(Eigen::Matrix<float,1,2>  robot, Eigen::Matrix<float, 1, 6> data_closest, float const& angle);

/**
 * Create a small and completely free occupancy grid. Then a single occupied cell is added in the middle of the grid (small obstacle). Successive iterations keeps adding occupied cells to
 * the obstacle which grows in a random way. At the end of each iteration, the occupancy grid is saved in a text file. The goal of this function is to have an arbitrarily shaped obstacle to
 * test the border reconstruction process (function compute_border)
 */
void growing_obstacle();

/**
 * Same as growing_obstacle function but with several obstacles. Create a small and completely free occupancy grid. Then occupied cells keep being added to the occupancy grid to grow obstacles
 * in a random way. At the end of each iteration, the occupancy grid is saved in a text file. The goal of this function is to have an arbitrarily shaped obstacles to test the obstacle detection
 * algorithm as well as border reconstruction process (function detect_borders)
 */
void growing_several_obstacle();

/**
 * Scan an occupancy grid to detect obstacles and return a vector containing the reconstructed surfaces of all detected obstacles
 *
 * @param occupancy_grid Eigen matrix that represents the occupancy grid the obstacle is in.
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @return Vector of Eigen matrices of size (Ni,5) containing information about the Ni surface cells of obstacle number i with [x, y, type, charac1, charac2] format
 *         x and y are the coordinates of the cell in the occupancy grid
 *         type is 1 for a straight cell, 2 for an outer corner, 3 for an inner corner (see report)
 *         charac1 and charac2 contain information specific to each type of cell (see report)
 */
std::vector<Border> detect_borders(Grid & occupancy_grid, State const& state_robot);

/**
 * Scan an occupancy grid to detect obstacles and return a vector containing occupied cells belonging to all detected obstacles
 *
 * @param occupancy_grid Eigen matrix that represents the occupancy grid the obstacle is in.
 * @return Vector of Eigen matrices of size (Ni,2) containing information about the Ni cells belonging to obstacle number i with [x, y] format
 *         For matrix number i, each row contains the [x,y] coordinates of a cell belonging to obstacle number i
 */
std::vector<Blob> detect_blobs( Grid & occupancy_grid );

/**
 * Return the index of the column on the other side an obstacle knowing which are its surface cells
 * the scan to detect obstacles is done from left to right so by giving (row, col_left) on the left of the obstacle the algorithm returns col_right value
 * That way the scan can resume from the (row, col_right) cell
 *   0 1 2 3 4 5
 * 0 . . . . . .
 * 1 . . . . . .
 * 2 . . x x . .
 * 3 . . x x . .
 * 4 . . . . . .
 * 5 . . . . . .
 * In this case (row, col_left) is equal to (2,1) and the algorithm will return 4 since the right side of the obstacle is (2,4)
 *
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @param row Current row the iterator is on
 * @param col Current col the iterator is on (left side of the obstacle)
 * @return Index of the column on the right side of the obstacle
 */
int other_side_obstacle(Border const& border, int const& row, int const& col);

/**
 * Explore an obstacle to detect all occupied cells that belong to it.
 * Exploration starts from one cell and a recursive function gradually explores nearby cells and checks if they are occupied (i.e part of the obstacle)
 *
 * @param obstacle Eigen matrix of size (N,2) with each row containing the [x,y] coordinates of a cell belonging to the blob.
 *               Initially it only contains the coordinate of the starting cell and is filled as the function gradually explores the obstacle
 * @param occupancy_grid Eigen matrix that represents the occupancy grid the obstacle is in.
 * @param row Row of the starting cell
 * @param col Column of the starting cell
 */
void explore_obstacle( Blob & obstacle, Grid & occupancy_grid, int const& row, int const& col);

/**
 * Return the respective weight of all considered obstacles depending on their distance to the robot
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param mat_gamma Eigen matrix of size (1,N) containing the respective Gamma distance of the robot for all obstacles in its surroundings
 * @param method Selection of the weighting method. 1 for the method of (Huber and al., 2019), 2 for a method that assign a weight of 0 to obstacles that are too far from the robot
 * @param do_not_normalize If true, do not make the sum of weights equal to 1 by dividing all weights by the sum of weights before output.
 * @return Eigen matrix of size (1,N), respective weight of each obstacle.
 */
Eigen::MatrixXf weights_special(State const& state_robot, Eigen::MatrixXf const& mat_gamma, int const& method, float const& limit_distance, bool const& do_not_normalize=false);

/**
 * Compute the velocity command of the robot depending on its relative position to several obstacles (version with limit distance to consider obstacles)
 * Reconstruction of surfaces based on straight lines, inner and outer corners
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param borders Vector of Eigen matrices of size (Ni,5) containing information about the Ni surface cells of obstacle number i with [x, y, type, charac1, charac2] format
 *        x and y are the coordinates of the cell in the occupancy grid
 *        type is 1 for a straight cell, 2 for an outer corner, 3 for an inner corner (see report)
 *        charac1 and charac2 contain information specific to each type of cell (see report)
 * @param size_of_cells Size of the cells of the occupancy grid in meters
 * @return Eigen matrix of size (3,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta]
 */
State next_step_special_weighted(State const& state_robot, State const& state_attractor, std::vector<Border> const& borders, float const& size_of_cells, bool const& corrected_velocity=false);

/**
 * Compute the velocity command of the robot depending on its relative position to single obsacle (version with limit distance to consider obstacles)
 * Reconstruction of the surface is based on straight lines, inner and outer corners
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @return Eigen matrix of size (4,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta, gamma distance] for the considered obstacle
 */
Eigen::Matrix<float, 4, 1> next_step_special(State const& state_robot, State const& state_attractor, Border const& border, bool const& corrected_velocity=false);

/**
 * Compute the Gamma distance of the robot for a given obstacle, the projection of the robot on the reconstructed surface as well as the normal vector to the surface
 *
 * @param robot Eigen matrix of size (1,2) containing the [x,y] coordinates of the robot
 * @param data_closest Eigen matrix of size (1,6) that contains information about the closest cell [x, y, type, charac_1, charac_2, square of euclidian distance from the robot]
 * @return Eigen matrix of size (6,1) [gamma distance, x coordinate of normal, y coordinate of normal, theta coordinate of normal (always 0), x coordinate of projectied point, y coordinate of projected point]
 */
Eigen::Matrix<float, 6, 1> gamma_normal_projection(Eigen::Matrix<float,1,2> const& robot, Eigen::Matrix<float, 1, 6> const& data_closest);

/**
 * Follow the reconstructed surface of an obstacle to get the minimum distance along the surface between the projection of the robot and the projection of the attractor
 * (either clockwise or counter-clockwise direction)
 *
 * @param proj_robot Eigen matrix of size (1,2) containing the [x,y] coordinates of the projection of the robot on the reconstructed surface
 * @param proj_attractor Eigen matrix of size (1,2) containing the [x,y] coordinates of the projection of the attractor on the reconstructed surface
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @return Eigen matrix of size (1,3) containing [perimeter of the reconstructed surface, minumum distance along surface, direction to follow]
 *         For the direction to follow, clockwise is 1 and counter-clockwise is -1 by convention
 */
Eigen::Matrix<float, 1, 3> get_distances_surface(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float, 1, 2> const& proj_attractor, Border const& border);

/**
 * Compute the maximum Gamma distance that can be reached before crossing a discontinuity by following a normal vector to the surface
 *
 * @param proj_robot Eigen matrix of size (1,2) containing the [x,y] coordinates of the projection of the robot on the reconstructed surface
 * @param normal Eigen matrix of size (1,2) containing the [x,y] coordinates of the normal vector to the surface at the position proj_robot
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @return Maximum Gamma distance that can be reached before crossing a discontinuity
 */
float get_max_gamma_distance(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float,1,2> const& normal, Border const& border);

/**
 * Compute the maximum distance that can be reached before crossing a discontinuity by following a normal vector to the surface
 *
 * @param distance_proj Minimum distance along the surface between the projection of the robot and the projection of the attractor on the reconstructed surface
 * @param distance_tot Perimeter of the reconstructed surface
 * @param gamma_robot Gamma distance of the robot for the considered obstacle
 * @param gamma_max Maximum Gamma distance that can be reached before crossing a discontinuity by following the normal vector to the surface where the projection of the robot is
 * @param gamma_attractor Gamma distance of the attractor for the considered obstacle
 * @param gamma_max_attractor Maximum Gamma distance that can be reached before crossing a discontinuity by following the normal vector to the surface where the projection of the attractor is
 * @param direction Direction to follow for the minimum distance along the surface between the projection of the robot and the projection of the attractor on the reconstructed surface
 *                   clockwise is 1 and counter-clockwise is -1 by convention
 * @return Eigen matrix of size (10,1) containing [ gamma distance of the robot in circle space, (x,y,theta) position of the robot in circle space, (x,y,theta) position of the attractor in circle space,
 *          (x,y,theta) of the reference vector in circle space] with theta always being 0
 */
Eigen::Matrix<float, 10, 1> get_point_circle_frame( float const& distance_proj, float const& distance_tot, float const& gamma_robot, float const& gamma_max, float const& gamma_attractor, float const& gamma_max_attractor, int direction);

/**
 * Fill the holes in all obstacles of the occupancy grid with a flood-like algorithm starting from the position of the robot
 * If there is a donut-shaped obstacle and the robot is outside the donut, then the hole in the donut will not be reached by the flood algorithm
 * All free cells that cannot be reached by the flood are marked as occupied since it means they belongs to a hole inside an obstacle
 * The input grid is directly updated.
 *
 * @param orig PointFill structure containing the [x,y] position of the origin of the flood
 * @param occupancy_grid Eigen matrix that represents the occupancy grid the obstacle is in
 */
void fillGrid(PointFill orig, Grid & occupancy_grid);

/**
 * Draw a disk of occupied cells centered on a given cell in an occupancy grid
 * The input grid is directly updated.
 *
 * @param grid Eigen matrix that represents the occupancy grid the obstacle is in
 * @param x Coordinate along X of the center cell
 * @param y Coordinate along Y of the center cell
 * @param radius Radius of the disk expressed in occupancy grid space (cells have a side of 1 [no unit])
 */
void draw_circle(Grid & grid, int const& x, int const& y, int const& radius);

/**
 * Read a text file containing integer values separated by white spaces and store them in an Eigen Matrix
 * 105 103
 * 179 234
 * 457  42 will output a matrix of size (3,2)
 * Mainly used to read the content of a Blob matrix with size (N,2) after it has been saved in a text file
 *
 * @param filename Path to the text file containing the data
 * @return An Eigen Matrix containing the integer values that are stored in the input text file
 */
Eigen::MatrixXi readMatrix(std::string filename);

/**
 * Return the indexes of all cells on a line between two points using Bresenham's line algorithm
 *
 * @param x1 Coordinate along X of the first point
 * @param y1 Coordinate along Y of the first point
 * @param x2 Coordinate along X of the second point
 * @param y2 Coordinate along Y of the second point
 * @return An Eigen Matrix of size (N,2) containing the [x,y] coordinates of the N cells that are on the line between the two points
 */
Eigen::MatrixXi Line( float x1, float y1, float x2, float y2);

/**
 * Check if a line crosses the reconstructed surface of an obstacle
 *
 * @param line Eigen Matrix of size (N,2) containing the [x,y] coordinates of the N cells that form the line
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @return True if the line crossed the surface, false otherwise
 */
bool check_if_on_way(Eigen::MatrixXi const& line, Border const& border);

/**
 * Compute the velocity command in circle space for a given position in circle space
 *
 * @param point_circle_space Eigen matrix of size (10,1) containing [ gamma distance of the robot in circle space, (x,y,theta) position of the robot in circle space, (x,y,theta) position of the attractor in circle space,
 *          (x,y,theta) of the reference vector in circle space] with theta always being 0
 * @return Eigen matrix of size (4,1) containing [vel cmd for x, vel cmd for y, vel cmd for theta, gamma distance] in circle space
 */
Eigen::Matrix<float, 4, 1> next_step_special_only_circle(Eigen::Matrix<float, 10, 1> point_circle_space);

/**
 * Compute the projection of a point in circle space for a given position in the initial space
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @return Eigen matrix of size (10,1) containing [ gamma distance of the robot in circle space, (x,y,theta) position of the robot in circle space, (x,y,theta) position of the attractor in circle space,
 *          (x,y,theta) of the reference vector in circle space] with theta always being 0
 */
Eigen::Matrix<float, 10, 1> point_from_initial_to_circle(State const& state_robot, State const& state_attractor, Border const& border);

/**
 * Compute a numerical approximation of the d X^c / d X^i matrix for a given position in the initial space
 * c means circle space, i means initial space
 * d X^c / d X^i = [ dxc/dxi  dxc/dyi
 *                   dyc/dxi  dyc/dyi ]
 *
 * @param state_robot Eigen matrix of size (3,1) containing the (x,y,theta) state vector of a point/robot. Theta is not used.
 * @param state_attractor Eigen matrix of size (3,1) containing the (x,y,theta) state vector of the attractor. Theta is not used.
 * @param border Eigen matrix of size (N,5) with each row containing the [x, y, type, charac_1, charac_2] information about a cell belonging to the surface of the input obstacle
 * @return Eigen matrix of size (2,2) containing an estimation of the variation of the circle space depending on the variation in the initial space
 */
Eigen::Matrix<double, 2, 2> get_derivation_matrix(State const& state_robot, State const& state_attractor, Border const& border);
#endif // OBSTACLERECONSTRUCTION_H_INCLUDED
