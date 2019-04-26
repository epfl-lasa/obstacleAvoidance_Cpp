#include "ObstacleReconstruction.h"
#include "ObstacleAvoidance.h"

float myRad = 0.1;
const float size_cell = 1.0;
const float margin = 0.25;

Point get_center(Blob const& blob) // Get the center of a group of cells by computing the mean of the positions of the cells along x and y
{
    Point res = blob.colwise().mean();
    return res;
}

Point get_random(Blob const& blob, int const& limit) // Get a random cell among a group of cells
{
    Point res;
    int i = 0;
    std::srand(42); // random seed
    if (limit==0) // default value of limit
    {
        i = rand() % blob.rows(); // between 0 and blob.rows - 1
    }
    else
    {
        i = rand() % limit; // between 0 and limit - 1
    }
    res(0,0) = blob(i,0); // assign x
    res(0,1) = blob(i,1); // assign y
    return res;
}

bool isPart(Blob const& blob, float const& x, float const& y) // check if a point is part of a blob
{
    for (int i=0; i<blob.rows(); i++) // for each cell in the blob
    {
        if ((x==blob(i,0)) && (y==blob(i,1))) // check if the cell has the same coordinates (i.e if it is the same cell)
        {
            return true; // is part of the input blob
        }
    }
    return false; // is not part of the input blob
}

bool isPartBorder(Border const& border, float const& x, float const& y) // check if a cell belongs to the input border starting from the first row
{
    for (int i=0; i<border.rows(); i++) // for each cell in the border starting from the first
    {
        if ((x==border(i,0)) && (y==border(i,1))) // check if the cell has the same coordinates (i.e if it is the same cell)
        {
            return true; // is part of the input border
        }
    }
    return false; // is not part of the input border
}

bool isPartBorderReverse(Border const& border, float const& x, float const& y) // check if a cell belongs to the input border starting from the first row
{
    for (int i=border.rows()-1; i>0; i--) // for each cell in the border starting from the last
    {
        if ((x==border(i,0)) && (y==border(i,1))) // check if the cell has the same coordinates (i.e if it is the same cell)
        {
            return true; // is part of the input border
        }
    }
    return false; // is not part of the input border
}


bool check_direction(Blob const& obstacle, Point const& point, int const& direction) // check if the targeted cell is part of the obstacle
{
    // Directions are:  3 2 1
    //                  4   0
    //                  5 6 7
    // With positive X towards the right and positive Y towards the top
    // For instance if point is at (2,2) and direction is 3 then it checks if cell (1,3) is part of the input blob "obstacle"
    switch(direction) {
    case 0 : if (isPart(obstacle, point(0,0)+1, point(0,1)  )) {return true;};
             break;       // exits the switch
    case 1 : if (isPart(obstacle, point(0,0)+1, point(0,1)+1)) {return true;};
             break;       // exits the switch
    case 2 : if (isPart(obstacle, point(0,0)  , point(0,1)+1)) {return true;};
             break;       // exits the switch
    case 3 : if (isPart(obstacle, point(0,0)-1, point(0,1)+1)) {return true;};
             break;       // exits the switch
    case 4 : if (isPart(obstacle, point(0,0)-1, point(0,1)  )) {return true;};
             break;       // exits the switch
    case 5 : if (isPart(obstacle, point(0,0)-1, point(0,1)-1)) {return true;};
             break;       // exits the switch
    case 6 : if (isPart(obstacle, point(0,0)  , point(0,1)-1)) {return true;};
             break;       // exits the switch
    case 7 : if (isPart(obstacle, point(0,0)+1, point(0,1)-1)) {return true;};
             break;       // exits the switch
    default: throw std::invalid_argument("Direction must be an integer between 0 and 7." ); // Not supposed to happen
             break;
    }
    return false;
}


Blob fill_gaps(Blob const& obstacle) // fill the 1-cell wide gaps in an obstacle
{
    // Finding a box which encloses the obstacle to avoid manipulating a really big matrix
    Eigen::Matrix<int, 1, 2> mini = obstacle.colwise().minCoeff(); // minimum coordinates along x and y
    Eigen::Matrix<int, 1, 2> maxi = obstacle.colwise().maxCoeff(); // maximum coordinates along x and y
    Eigen::MatrixXi grid = Eigen::MatrixXi::Zero(maxi(0,0)-mini(0,0)+1+2, maxi(0,1)-mini(0,1)+1+2); // box which encloses the obstacle with a 1-cell margin all around it

    // For instance a 2x2 obstacle will end in a box like that (. is free and x is occupied)
    // . . . .
    // . x x .
    // . x x .
    // . . . .

    // Fill a grid with occupied cells
    Blob filled_gaps;
    for (int i=0; i<obstacle.rows(); i++)
    {
        grid(obstacle(i,0)-mini(0,0)+1,obstacle(i,1)-mini(0,1)+1) = 1; // copy the obstacle from the huge occupancy map matrix to the small matrix that is just the right size to contain it
    }

    for (int i=1; i<(grid.rows()-1); i++) // first and last rows are margin so no need to scan them
    {
        for (int j=1; j<(grid.cols()-1); j++) // first and last cols are margin so no need to scan them
        {
            if ((grid(i,j)==0) && ((grid(i-1,j)+grid(i+1,j)+grid(i,j-1)+grid(i,j+1))>=3)) // if the cell is occupied and there is at least 3 occupied cells nearby it means it's a 1-cell wide gap
            {
                grid(i,j) = 1; // fill the gap
                filled_gaps.conservativeResize(filled_gaps.rows()+1, Eigen::NoChange);
                filled_gaps.row(filled_gaps.rows()-1) << i-1+mini(0,0),j-1+mini(0,1); // append the occupied cell to the obstacle
                i -= 1; // go 1 row up
                // . . .
                // x . x // this gap is not filled (sum == 2)
                // x . x // this gap is filled (sum == 3), but we need to recheck the row above because now the gap is going to be filled since the sum is not 2 anymore
                // x x x
                if (i<1) {i=1;} // if we were already at the first row
                j = 0;  // restart at beginning of the row (j++ will make it 1)
            }
        }
    }
    // All gap have been filled
    Blob result = obstacle;
    result.conservativeResize(obstacle.rows()+filled_gaps.rows(), Eigen::NoChange);
    result.block(obstacle.rows(),0,filled_gaps.rows(),2) = filled_gaps; // append filled gaps to the Blob
    /*std::cout << "---" << std::endl;
    std::cout << obstacle << std::endl;
    std::cout << "---" << std::endl;
    std::cout << filled_gaps << std::endl;
    std::cout << "---" << std::endl;
    std::cout << filled_gaps << std::endl;
    std::cout << "---" << std::endl;*/
    return result;
}

Blob fill_gaps_with_grid(Blob const& obstacle, Grid & occupancy_grid) // fill the 1-cell wide gaps in an obstacle and update occupancy grid
{
    // Finding a box which encloses the obstacle to avoid manipulating a really big matrix
    Eigen::Matrix<int, 1, 2> mini = obstacle.colwise().minCoeff(); // minimum coordinates along x and y
    Eigen::Matrix<int, 1, 2> maxi = obstacle.colwise().maxCoeff(); // maximum coordinates along x and y
    Eigen::MatrixXi grid = Eigen::MatrixXi::Zero(maxi(0,0)-mini(0,0)+1+2, maxi(0,1)-mini(0,1)+1+2); // box which encloses the obstacle with a 1-cell margin all around it

    // For instance a 2x2 obstacle will end in a box like that (. is free and x is occupied)
    // . . . .
    // . x x .
    // . x x .
    // . . . .

    // Fill a grid with occupied cells
    Blob filled_gaps;
    for (int i=0; i<obstacle.rows(); i++)
    {
        grid(obstacle(i,0)-mini(0,0)+1,obstacle(i,1)-mini(0,1)+1) = 1; // copy the obstacle from the huge occupancy map matrix to the small matrix that is just the right size to contain it
    }
    // TODO: Optimization using .block directly from the occupancy grid?

    for (int i=1; i<(grid.rows()-1); i++) // first and last rows are margin so no need to scan them
    {
        for (int j=1; j<(grid.cols()-1); j++) // first and last cols are margin so no need to scan them
        {
            if ((grid(i,j)==0) && ((grid(i-1,j)+grid(i+1,j)+grid(i,j-1)+grid(i,j+1))>=3)) // if the cell is occupied and there is at least 3 occupied cells nearby it means it's a 1-cell wide gap
            {
                grid(i,j) = 1; // fill the gap
                occupancy_grid(i+mini(0,0)-1,j+mini(0,1)-1) = 100; // update the occupancy_grid (occupied = 100, free = 0)
                filled_gaps.conservativeResize(filled_gaps.rows()+1, Eigen::NoChange);
                filled_gaps.row(filled_gaps.rows()-1) << i-1+mini(0,0),j-1+mini(0,1);
                i -= 1; // go 1 row up
                // . . .
                // x . x // this gap is not filled (sum == 2)
                // x . x // this gap is filled (sum == 3), but we need to recheck the row above because now the gap is going to be filled since the sum is not 2 anymore
                // x x x
                if (i<1) {i=1;}  // if we were already at the first row
                j = 0;  // restart at beginning of the row (j++ will make it 1)
            }
        }
    }
    // All gap have been filled
    Blob result = obstacle;
    result.conservativeResize(obstacle.rows()+filled_gaps.rows(), Eigen::NoChange);
    result.block(obstacle.rows(),0,filled_gaps.rows(),2) = filled_gaps; // append filled gaps to the Blob

    return result;
}

void update_border(Border & border, Point const& position, int const& previous_direction, int const& next_direction) // add elements to the border to follow the sides of the obstacles depending on the direction taken
{
    const int x = position(0,0); // position of the current cell
    const int y = position(0,1);
    // Directions are
    // 3 2 1
    // 4   0
    // 5 6 7
    // For instance if we have a 2x2 obstacle
    // . . . .
    // . x x .
    // . x x .
    // . . . .
    // If we are at the bottom right corner, the previous_previous direction is 0 (we went from bottom left to bottom right) and the next_direction is 2 (go to top right as we move counter-clockwise)
    // so we have to add three cell to the border to make the "corner"
    // . . . .
    // . x x .
    // . x x |
    // . . - /
    // The final border being
    // / - - \
    // | x x |
    // | x x |
    // \ - - /

    // Obstacle:
    // . . . . . . . .
    // . / - - - - \ .
    // . | x x x x | .
    // . | x x x x | .
    // . | x x / - / .
    // . \ - - / . . .
    // . . . . . . . .

    // Format of straight line : [x_cell, y_cell, 1, x_dir, y_dir]
    // "|" on the left  will have x_dir = -1 and y_dir = 0 because their normal vector points to the left
    // "|" on the right will have x_dir =  1 and y_dir = 0 because their normal vector points to the right
    // "-" on the top    will have x_dir = 0 and y_dir =  1 because their normal vector points to the top
    // "-" on the bottom will have x_dir = 0 and y_dir = -1 because their normal vector points to the bottom

    // Format of an outward arc of circle : [x_cell, y_cell, 2, unused, pos_center]
    // Pos center is a convention I defined to make a difference between the four quarters of circle
    // Pos 0  Pos 1  Pos 2  Pos 3
    //  - \    / -    | x    x |
    //  x |    | x    \ -    - /
    // The normal vector is going outward, from x to the \ or /
    // You use this type for convex corners like the four corners or a square

    // Format of an inward arc of circle : [x_cell, y_cell, 3, unused, pos_center]
    // Pos center is a convention I defined to make a difference between the four quarters of circle
    // Pos 0  Pos 1  Pos 2  Pos 3
    //  - \    / -    | x    x |
    //  x |    | x    \ -    - /
    // The normal vector is going inward, from \ or / to x
    // You use this type for concave corners like the two corners inside a U shaped obstacle

    // With the pos_center (six arcs of type 2 and two arcs of type 3)
    // . . . . . . . . .
    // . 1 - 0 . 1 - 0 .
    // . | x | . | x | .
    // . | x | . | x | .
    // . | x 0 - 1 x | .
    // . | x x x x x | .
    // . 2 - - - - - 3 .
    // . . . . . . . . .

    switch(previous_direction) { // depending on which direction was taken to get to the current cell
    case 0 : switch(next_direction) {
            case 0: add_to_border(border, x, y-1, 1, 0, -1); break;
            case 1: add_to_border(border, x, y-1, 1, 0, -1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 3, myRad, 1);  break;
            case 2: add_to_border(border, x, y-1, 1, 0, -1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 1, 1, 0);  break;
            case 3: add_to_border(border, x, y-1, 1, 0, -1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 1, 1, 0); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 3, myRad, 2); break;
            case 4: add_to_border(border, x, y-1, 1, 0, -1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 1, 1, 0); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 1, 0, 1); break;
            case 5: throw std::invalid_argument("Should not happen, case 0." ); break;
            case 6: throw std::invalid_argument("Should not happen, case 0." ); break;
            case 7: add_to_border(border, x, y-1, 3, myRad, 0); break;
            }
             break;       // exits the switch
    case 1 : switch(next_direction) {
            case 0: add_to_border(border, x, y-1, 3, myRad, 1); break;
            case 1: add_to_border(border, x, y-1, 3, myRad, 1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 3, myRad, 1);  break;
            case 2: add_to_border(border, x, y-1, 3, myRad, 1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 1, 1, 0);  break;
            case 3: add_to_border(border, x, y-1, 3, myRad, 1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 1, 1, 0); add_to_border(border, x+1, y+1, 2, myRad, 0);  break;
            case 4: add_to_border(border, x, y-1, 3, myRad, 1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 1, 1, 0); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 1, 0, 1); break;
            case 5: add_to_border(border, x, y-1, 3, myRad, 1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 1, 1, 0); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 1, 0, 1); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 3, myRad, 3); break;
            case 6: throw std::invalid_argument("Should not happen, case 1." ); break;
            case 7: throw std::invalid_argument("Should not happen, case 1." ); break;
            }
             break;       // exits the switch
    case 2 : switch(next_direction) { // ROTATION OF CASE 0
            case 0: throw std::invalid_argument("Should not happen, case 2." ); break;
            case 1: add_to_border(border, x+1, y, 3, myRad, 1); break;
            case 2: add_to_border(border, x+1, y, 1, 1, 0); break;
            case 3: add_to_border(border, x+1, y, 1, 1, 0); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 3, myRad, 2);  break;
            case 4: add_to_border(border, x+1, y, 1, 1, 0); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 1, 0, 1);  break;
            case 5: add_to_border(border, x+1, y, 1, 1, 0); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 1, 0, 1); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 3, myRad, 3); break;
            case 6: add_to_border(border, x+1, y, 1, 1, 0); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 1, 0, 1); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 1, -1, 0); break;
            case 7: throw std::invalid_argument("Should not happen, case 2." ); break;
            }
             break;       // exits the switch
    case 3 : switch(next_direction) { // ROTATION OF CASE 1
            case 0: throw std::invalid_argument("Should not happen, case 3." ); break;
            case 1: throw std::invalid_argument("Should not happen, case 3." ); break;
            case 2: add_to_border(border, x+1, y, 3, myRad, 2); break;
            case 3: add_to_border(border, x+1, y, 3, myRad, 2); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 3, myRad, 2);  break;
            case 4: add_to_border(border, x+1, y, 3, myRad, 2); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 1, 0, 1);  break;
            case 5: add_to_border(border, x+1, y, 3, myRad, 2); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 1, 0, 1); add_to_border(border, x-1, y+1, 2, myRad, 1);  break;
            case 6: add_to_border(border, x+1, y, 3, myRad, 2); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 1, 0, 1); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 1, -1, 0); break;
            case 7: add_to_border(border, x+1, y, 3, myRad, 2); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 1, 0, 1); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 1, -1, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 3, myRad, 0); break;
            }
             break;       // exits the switch
    case 4 : switch(next_direction) { // ROTATION OF CASE 2
            case 0: add_to_border(border, x, y+1, 1, 0, 1); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 1, -1, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 1, 0, -1); break;
            case 1: throw std::invalid_argument("Should not happen, case 4." ); break;
            case 2: throw std::invalid_argument("Should not happen, case 4." ); break;
            case 3: add_to_border(border, x, y+1, 3, myRad, 2); break;
            case 4: add_to_border(border, x, y+1, 1, 0, 1); break;
            case 5: add_to_border(border, x, y+1, 1, 0, 1); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 3, myRad, 3);  break;
            case 6: add_to_border(border, x, y+1, 1, 0, 1); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 1, -1, 0);  break;
            case 7: add_to_border(border, x, y+1, 1, 0, 1); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 1, -1, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 3, myRad, 0); break;
            }
             break;       // exits the switch
    case 5 : switch(next_direction) { // ROTATION OF CASE 3
            case 0: add_to_border(border, x, y+1, 3, myRad, 3); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 1, -1, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 1, 0, -1); break;
            case 1: add_to_border(border, x, y+1, 3, myRad, 3); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 1, -1, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 1, 0, -1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 3, myRad, 1); break;
            case 2: throw std::invalid_argument("Should not happen, case 5." ); break;
            case 3: throw std::invalid_argument("Should not happen, case 5." ); break;
            case 4: add_to_border(border, x, y+1, 3, myRad, 3); break;
            case 5: add_to_border(border, x, y+1, 3, myRad, 3); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 3, myRad, 3);  break;
            case 6: add_to_border(border, x, y+1, 3, myRad, 3); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 1, -1, 0);  break;
            case 7: add_to_border(border, x, y+1, 3, myRad, 3); add_to_border(border, x-1, y+1, 2, myRad, 1); add_to_border(border, x-1, y, 1, -1, 0); add_to_border(border, x-1, y-1, 2, myRad, 2);  break;
            }
             break;       // exits the switch
    case 6 : switch(next_direction) { // ROTATION OF CASE 4
            case 0: add_to_border(border, x-1, y, 1, -1, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 1, 0, -1);  break;
            case 1: add_to_border(border, x-1, y, 1, -1, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 1, 0, -1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 3, myRad, 1); break;
            case 2: add_to_border(border, x-1, y, 1, -1, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 1, 0, -1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 1, 1, 0); break;
            case 3: throw std::invalid_argument("Should not happen, case 6." ); break;
            case 4: throw std::invalid_argument("Should not happen, case 6." ); break;
            case 5: add_to_border(border, x-1, y, 3, myRad, 3); break;
            case 6: add_to_border(border, x-1, y, 1, -1, 0); break;
            case 7: add_to_border(border, x-1, y, 1, -1, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 3, myRad, 0);  break;
            }
             break;       // exits the switch
    case 7 : switch(next_direction) { // ROTATION OF CASE 5
            case 0: add_to_border(border, x-1, y, 3, myRad, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 1, 0, -1);  break;
            case 1: add_to_border(border, x-1, y, 3, myRad, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 1, 0, -1); add_to_border(border, x+1, y-1, 2, myRad, 3);  break;
            case 2: add_to_border(border, x-1, y, 3, myRad, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 1, 0, -1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 1, 1, 0); break;
            case 3: add_to_border(border, x-1, y, 3, myRad, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 1, 0, -1); add_to_border(border, x+1, y-1, 2, myRad, 3); add_to_border(border, x+1, y, 1, 1, 0); add_to_border(border, x+1, y+1, 2, myRad, 0); add_to_border(border, x, y+1, 3, myRad, 2); break;
            case 4: throw std::invalid_argument("Should not happen, case 7." ); break;
            case 5: throw std::invalid_argument("Should not happen, case 7." ); break;
            case 6: add_to_border(border, x-1, y, 3, myRad, 0); break;
            case 7: add_to_border(border, x-1, y, 3, myRad, 0); add_to_border(border, x-1, y-1, 2, myRad, 2); add_to_border(border, x, y-1, 3, myRad, 0);  break;
            }
             break;       // exits the switch
    default: throw std::invalid_argument("Direction must be an integer between 0 and 7." ); // Should not happen
             break;
    }

}

void add_to_border(Border & border, int const& x, int const& y, int const& type, float const& charac_1, float const& charac_2)  // add the given cell to the border with its characteristics
{
    if (!isPartBorderReverse( border, x, y)) // if not already part of the border
    {
        border.conservativeResize(border.rows()+1, Eigen::NoChange);
        border.row(border.rows()-1) << x, y, type, charac_1, charac_2; // append new border cell at the end
    }
}

void update_position(Point & current_position, int const& direction) // update the position of a point after a step in a given direction
{
    // Directions are:  3 2 1
    //                  4   0
    //                  5 6 7
    // With positive X towards the right and positive Y towards the top
    // If the point is at position (3,3) and we take a step in direction 1 then the point is now at position (4,4)
    // If the point is at position (3,3) and we take a step in direction 6 then the point is now at position (3,2)
    switch(direction) {
    case 0 : current_position(0,0) += 1;
             break;       // exits the switch
    case 1 : current_position(0,0) += 1; current_position(0,1) += 1;
             break;       // exits the switch
    case 2 : current_position(0,1) += 1;
             break;       // exits the switch
    case 3 : current_position(0,0) -= 1; current_position(0,1) += 1;
             break;       // exits the switch
    case 4 : current_position(0,0) -= 1;
             break;       // exits the switch
    case 5 : current_position(0,0) -= 1; current_position(0,1) -= 1;
             break;       // exits the switch
    case 6 : current_position(0,1) -= 1;
             break;       // exits the switch
    case 7 : current_position(0,0) += 1; current_position(0,1) -= 1;
             break;       // exits the switch
    default: throw std::invalid_argument("Direction must be an integer between 0 and 7." ); // Should not happen
             break;
    }
}

Border compute_border_and_fill(Blob const& obstacle, Point const& start_point, Grid & occupancy_grid) // follow the sides of the obstacle to create its border incrementally and update occupancy grid
{
    // Border border;
    Blob filled_obstacle = fill_gaps_with_grid(obstacle, occupancy_grid); // Fill the 1-wide gaps in the obstacle and update the occupancy grid accordingly

    return compute_border( filled_obstacle, start_point, true); // Compute the border of the filled obstacle (already_filled=true)
}

Border compute_border(Blob const& obstacle, Point const& start_point, bool const& already_filled) // follow the sides of the obstacle in a counter-clockwise direction to create its border incrementally
{

    Border border;
    // Fill the gaps in the obstacle
    Blob filled_obstacle;
    if (!already_filled)
    {
        filled_obstacle = fill_gaps(obstacle);
    }
    else
    {
        filled_obstacle = obstacle; // TODO: Use pointer to avoid copy?
    }
    // TODO: Check if it is still needed since if the function is called from compute_border_and_fill then the obstacle is already filled.
    // -> Done with already_filled check

    // Reach the right border by starting from start_point and going to the right till we reach a free cell (i.e the border)
    Point target = start_point;
    while (isPart(filled_obstacle, target(0,0), target(0,1))) // go right till we reach the border
    {
        target(0,0) += 1;
    }
    target(0,0) -= 1; // go on step to the left to get back in the obstacle
    //std::cout << "Starting following border from cell (" << target(0,0) << "," << target(0,1) << ")" << std::endl;
    // Now we want to know from which which is the previous direction to get to this cell if we were following the border
    // If the obstacle is like that (x occupied, . free, s starting point)
    // x x x .
    // x x x .
    // s x x .
    // x x . .
    // So we go right till we reach the . (c current cell, p previous cell if we were following the border counter clockwise)
    // x x x .
    // x x x .
    // x x c .
    // x p . .
    // If we were following the border, it means the previous_direction is 1 because we would have went to the top-right from the p cell.

    Point start = target;
    Point current_pos = target;
    int previous_dir = 2; // default value, should not matter I think
    int next_dir = 7; // direction 6 from c is a free cell since we stopped for this exact reason
    // so the first direction to check for to find the p cell is bottom right (7) then counter clockwise till every direction is checked
    // Once the previous cell has been found, we add 4 to the direction we used to find it since we would have been coming from that cell
    // With the example above, we find the p cell by checking direction 5 so it means the previous direction is 1 (5+4 modulo 8)

    // Retrieve previous direction from starting position to ensure that the border is correctly created for the first step
    bool flag_start = true;
    while (flag_start)
    {
        if (check_direction(filled_obstacle, current_pos, next_dir)) // check if the cell in that direction is occupied
        {
            previous_dir = next_dir; // if it is occupied we stop and assign the direction to previous_dir
            flag_start = false;
        }
        else
        {
            next_dir -= 1; // we check clockwise
            if (next_dir==0) {flag_start = false;} // should not happen or else it means the c cell is not surrounded by any cell
        }
    }
    previous_dir += 4;
    if (previous_dir >= 8) {previous_dir-=8;} // I am not using std::remainder because it returns a float (could cast it to int)
    int initial_previous_dir = previous_dir; // Used later
    next_dir = 1; // we start to check counter-clockwise from direction 1 since we already know that cell in direction 0 is free (we stopped because of that)
    flag_start = true;

    //check_direction(obstacle, current, next_dir);
    // Follow border till we come back to the initial position
    //std::cout << "(" << current_pos(0,0) << "," << current_pos(0,1) << ")" << std::endl;

    do
    {
        if (check_direction(filled_obstacle, current_pos, next_dir))
        {
            //std::cout << "(" << current_pos(0,0) << "," << current_pos(0,1) << ") and going from " << previous_dir << " to " << next_dir << std::endl;

            // go to next cell
            update_border(border, current_pos, previous_dir, next_dir); // add the correct elements to the border container
            update_position(current_pos, next_dir); // update the position of the current cell
            previous_dir = next_dir; // update previous direction taken
            next_dir = previous_dir - 1; // start by checking from previous dir - 1 because previous dir - 2 is free since we filled 1 wide gaps
            // x x ?
            // c . ?
            // In this case, we go from c to top-middle cell (so direction 1).
            // x c ?
            // x . ?
            // Direction 6 is obviously free or else we would have taken direction 0 during the previous step
            // Should we check direction 7? No because if the bottom-right cell is occupied then the bottom-middle cell is a 1-wide gap so it would have been filled during the fill_obstacle step
            // As bottom-middle is free, it implies that bottom-right is also free, the first direction to check is direction 0 (top-right cell). Direction 0 is indeed previous dir - 1 in this case
            // You can have the same reasoning for other cases and it's always previous_dir - 1 that you should check first

            if (next_dir<0) {next_dir = 7;}
            flag_start = false;
        }
        else
        {
            next_dir += 1; // we check counter-clockwise
            if (next_dir==8) {next_dir = 0;}
        }

    } while((flag_start)|| !((current_pos(0,0)==start(0,0))&&(current_pos(0,1)==start(0,1)&&(previous_dir==initial_previous_dir)))); // while we do not come back to the starting cell with the initial previous direction
    // initial previous direction is important because if the obstacle is very thin we may pass twice at the starting cell, for instance
    // . . . . . . .
    // . x . . . x .
    // . x . . . x .
    // . x x p s . .
    // . . . . . . .
    // With s the occupied starting cell, we will pass twice when following the border
    // previous initial dir in this case is 0 (coming from occupied cell p)

    // Perform one last step to properly close the border
    flag_start = true;
    while (flag_start)
    {
        if (check_direction(filled_obstacle, current_pos, next_dir))
        {
            // go to next
            update_border(border, current_pos, previous_dir, next_dir); // add the correct elements to the border container
            update_position(current_pos, next_dir); // update the position of the current cell
            previous_dir = next_dir; // update previous direction taken
            next_dir = previous_dir - 1; // start by checking from previous dir - 1
            if (next_dir<0) {next_dir = 7;}
            flag_start = false;
            //std::cout << "(" << current_pos(0,0) << "," << current_pos(0,1) << ")" << std::endl;
        }
        else
        {
            next_dir += 1;
            if (next_dir==8) {next_dir = 0;}
        }
    }

    remove_end_duplicate(border); // remove duplicates in the border container as the last step we take to properly close the border creates some duplicates

    return border;
}

void display_border(Blob const& obstacle, Border const& border) // display the border in the console (simplified graphic representation)
{
    Eigen::Matrix<int, 1, 2> mini = obstacle.colwise().minCoeff();
    Eigen::Matrix<int, 1, 2> maxi = obstacle.colwise().maxCoeff();
    Eigen::MatrixXi grid = Eigen::MatrixXi::Zero(maxi(0,0)-mini(0,0)+1+2, maxi(0,1)-mini(0,1)+1+2);

    std::string display_storage[maxi(0,0)-mini(0,0)+1+2][maxi(0,1)-mini(0,1)+1+2];
    for (int i=0; i<grid.rows();i++)
    {
        for (int j=0; j<grid.cols();j++)
        {
            display_storage[i][j] = "."; // all cells are initially free so "."
        }
    }


    for (int i=0; i<obstacle.rows(); i++)
    {
        grid(obstacle(i,0)-mini(0,0)+1,obstacle(i,1)-mini(0,1)+1) = 1;
        display_storage[obstacle(i,0)-mini(0,0)+1][obstacle(i,1)-mini(0,1)+1] = "\u25A1"; // if cell is occupied then we put a square symbol in it
    }
    for (int i=0; i<border.rows(); i++)
    {
        grid(border(i,0)-mini(0,0)+1,border(i,1)-mini(0,1)+1) = 2;
        switch((int)(border(i,2))) {
            case 1: if (border(i,3)==0) {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "\u007C";} // long - and | symbols
                    else {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "\u2015";}
                    break;
            case 2: if (border(i,4)==0) {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "/";} // corners, arc of circle characters are not displayed properly
                    else if (border(i,4)==1) {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "\\";}
                    else if (border(i,4)==2) {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "/";}
                    else  {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "\\";}
                    break;
            case 3: if (border(i,4)==2) {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "/";}
                    else if (border(i,4)==3) {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "\\";}
                    else if (border(i,4)==0) {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "/";}
                    else  {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "\\";}
                    break;
        }
    }

    for (int i=0; i<grid.rows();i++)
    {
        for (int j=0; j<grid.cols();j++)
        {
            std::cout << display_storage[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void remove_end_duplicate(Border & border) // Function to remove the few duplicates at the beginning/end of the border creation because we come back to the same cell
{
    bool flag_end = false;
    while (!flag_end)
    {
        int x = border(border.rows()-1,0); // load data of the current last row
        int y = border(border.rows()-1,1);
        for (int i=0;i<=border.rows()-2; i++)
        {
            if ((x==border(i,0))&&(y==border(i,1))) // duplicate has been detected
            {
                border.conservativeResize(border.rows()-1,border.cols()); // remove last row because it is a duplicate
                break;
            }
            if (i==border.rows()-2) {flag_end = true;} // if no duplicate has been detected we stop
        }
    }
}


Blob expand_obstacle(Blob const& obstacle, int const& n_cells) // expand an obstacle by a given number of cells
{
    Blob expanded_obs = fill_gaps(obstacle); // fill 1-wide gaps in the obstacle

    for (int i_expand=0; i_expand<n_cells; i_expand++) // loop once for each expansion
    {
        Point starting_point = get_random(expanded_obs);
        Border expanded_border = compute_border( expanded_obs, starting_point); // find the border of the current obstacle
        // the border is actually a 1-wide ring of free cells around the obstacle
        // so expanding the obstacle by 1 cell is just filling its border

        int N = expanded_obs.rows();
        expanded_obs.conservativeResize(N+expanded_border.rows(), Eigen::NoChange);
        for (int i_cell=0; i_cell<expanded_border.rows(); i_cell++)
        {
            expanded_obs.row(N+i_cell) << expanded_border(i_cell, 0), expanded_border(i_cell, 1); // [x,y] of the border cell
        }
    }

    //Border placeholder_border;
    //display_border(expanded_obs, placeholder_border);
    return expanded_obs;
}

int ID_data = 0; // variables used to save data to a file, may not be used
long counter_frame = 0;

Grid expand_occupancy_grid(Grid const& grid, int const& n_cells, State const& state_robot, float const& limit_range, float const& size_of_cells) // expand the occupied cells of an occupancy grid by a given number of cells
{
    // TODO: Optimization possible? We deal with big matrices so maybe something to do here

    // Only the cells in a given range around the robot has to been processed, the rest is discarded
    // Because out of this range the obstacles have a weight of 0 so what's the point of expanding them if
    // they are going to have no influence on the velocity of the robot anyway

    //limit_dist = limit_range; // modify the global variable set in ObstacleAvoidance.h

    float offset = std::sqrt(limit_dist-1);//limit_range / size_of_cells;
    int x_min = static_cast<int>(std::round(state_robot(0,0)-offset));
    int x_max = static_cast<int>(std::round(state_robot(0,0)+offset));
    int y_min = static_cast<int>(std::round(state_robot(1,0)-offset));
    int y_max = static_cast<int>(std::round(state_robot(1,0)+offset));

    // std::cout << " OFFSET: " << offset << std::endl;
    // std::cout << x_min << " | " << x_max << " | " << y_min << " | " << y_max << std::endl;


    // Expand all occupied cells of the occupancy grid by n cells
    Grid occupancy_res = grid;
    for (int n=0; n<n_cells; n++)
    {
        Grid occupancy_temp = occupancy_res; // occupancy grid at step n
        for (int i=1; i<(occupancy_res.rows()-1); i++) // scan row i
        {
           for (int j=1; j<(occupancy_res.cols()-1); j++) // scan col j
           {
               if (occupancy_temp(i,j) == 100) // if occupied we fill the 4 neightbour cells
               {
                   occupancy_res(i+1,j) = 100;
                   occupancy_res(i-1,j) = 100;
                   occupancy_res(i,j+1) = 100;
                   occupancy_res(i,j-1) = 100;
               }
           }
        }
    }

    // Output is an empty grid with the obstacle inside the limit distance
    Grid output = Grid::Zero(grid.rows(), grid.cols()); // empty grid the size of the input occupancy grid
    output.block(x_min, y_min, x_max-x_min+1, y_max-y_min+1) = occupancy_res.block(x_min, y_min, x_max-x_min+1, y_max-y_min+1); // copy the obstacle in the limit range

    // Detect all obstacles that have at least one cell in the limit square around the robot
    // For instance if one half of an obstacle is within the limit distance we also want
    // to take into account the part that is outside the limit square because the whole shape of the obstacle matters

    Grid temp_grid = occupancy_res;
    std::vector<Blob> all_detected = detect_blobs( temp_grid); // detect all blobs of occupied cells in the grid (i.e all obstacles)
    std::vector<Blob> inside_square;
    for (int i_blob=0; i_blob < all_detected.size(); i_blob++) // check each detected blob
    {
        bool flag = true;
        for (int i_cell=0; (flag)&&(i_cell < (all_detected[i_blob]).rows()); i_cell++) // check each cell of this blob
        {
            int x = (all_detected[i_blob])(i_cell,0);
            int y = (all_detected[i_blob])(i_cell,1);
            if ((x >= x_min) && (x <= x_max) && (y >= y_min) && (y <= y_max)) // if the considered blob has at least one cell in the limit range then we include it
            {
                inside_square.push_back(all_detected[i_blob]);
                flag = false;
            }
        }
    }


    // Add obstacles inside square to occupancy grid
    // TODO: maybe it could be optimized by only doing it for those are not already completely inside the square
    // Especially there is something not optimized with the line above "// copy the obstacle in the limit range"
    // I think something is done twice

    std::cout << all_detected.size() << " obstacles detected in total" << std::endl;
    std::cout << inside_square.size() << " obstacles partly inside the limit square" << std::endl;

    for (int i_blob=0; i_blob < inside_square.size(); i_blob++) // for each blob partly/completely inside the limit square
    {
        for (int i_cell=0; i_cell < (inside_square[i_blob]).rows(); i_cell++)
        {
            int x = (inside_square[i_blob])(i_cell,0);
            int y = (inside_square[i_blob])(i_cell,1);
            if (!((x >= x_min) && (x <= x_max) && (y >= y_min) && (y <= y_max))) // if outside the square we set the cell as occupied (if inside it is already done)
            {
                output(x,y) = 100;
            }
        }
        std::cout << std::endl;
    }
    // TODO: Here there may be something to optimize, maybe a way to avoid the for loop and pass the whole obstacle directly

    // Fill the holes in obstacles (free cells that are enclosed in an obstacle, they mess with the border detection algorithm)

    // We start from the cell the robot is standing one (which is a free cell)
    // Then it works like a flood algorithm to detect all the free cells of the map
    // Once all the free cells have been detected, by taking the negative of the map
    // we got all occupied cell + free cells enclosed inside obstacles
    // Then it's easy to scan this negative map to fill all enclosed free cells
    PointFill origin;
    origin.x = static_cast<int>(std::round(state_robot(0,0)));
    origin.y = static_cast<int>(std::round(state_robot(1,0)));
    std::cout << "Origin of flood: (" << origin.x << " , " << origin.y << ")" << std::endl;

    float mycount = 1;
    // If the cell the robot is standing one is not a free cell (if the robot touches an obstacle),
    // trying to find the closest free cell
    while (output(origin.x, origin.y) == 100)
    {
        if (output( static_cast<int>(origin.x+mycount), origin.y) != 100)
        {
            origin.x = static_cast<int>(origin.x+mycount);
            break;
        }
        else if (output( static_cast<int>(origin.x-mycount), origin.y) != 100)
        {
            origin.x = static_cast<int>(origin.x-mycount);
            break;
        }
        else if (output(origin.x, static_cast<int>(origin.y+mycount)) != 100)
        {
            origin.y = static_cast<int>(origin.y+mycount);
            break;
        }
        else if (output(origin.x, static_cast<int>(origin.y-mycount)) != 100)
        {
            origin.y = static_cast<int>(origin.y-mycount);
            break;
        }
        mycount += 1;
    }

    // TODO: Only do fillgrid for the square that include all obstacles partly inside the limit
    // Set limits inside fillGrid for the flood
    // IMPLEMENTATION BELOW
    Eigen::MatrixXi col_sum = output.colwise().sum();
    Eigen::MatrixXi row_sum = output.rowwise().sum();

    int i_col_min = col_sum.cols();
    int i_col_max = 0;
    int i_row_min = row_sum.cols();
    int i_row_max = 0;

    // Get the index of the first column with a non-zero element
    int k = 0;
    while (((k+1)<col_sum.cols()) && (col_sum(0,k+1)==0)) {k++;}
    if ((k+1)!=col_sum.cols()){k++;}
    i_col_min = k;

    // Get the index of the last column with a non-zero element
    k = col_sum.cols()-1;
    while (((k-1)>=0) && (col_sum(0,k-1)==0)) {k--;}
    if ((k-1)>=0){k--;}
    i_col_max = k;

    // Get the index of the first row with a non-zero element
    k = 0;
    while (((k+1)<row_sum.rows()) && (row_sum(k+1,0)==0)) {k++;}
    if ((k+1)<row_sum.rows()){k++;}
    i_row_min = k;

    // Get the index of the last row with a non-zero element
    k = row_sum.rows()-1;
    while (((k-1)>=0) && (row_sum(k-1,0)==0)) {k--;}
    if ((k-1)>=0){k--;}
    i_row_max = k;

    // TODO: Sum can be 0 if there is just the right number of unknown cells (-1) to nullify the occupied cells (100)

    //std::cout << row_sum << std::endl;

    //std::cout << i_row_min << " | " << i_row_max << " | " << i_col_min << " | " << i_col_max << std::endl;

    Grid toBeFilled = output.block(i_row_min, i_col_min, (i_row_max-i_row_min)+1, (i_col_max-i_col_min)+1);
    PointFill origin_toBeFilled;
    origin_toBeFilled.x = static_cast<int>(origin.x-i_row_min);
    origin_toBeFilled.y = static_cast<int>(origin.y-i_col_min);
    //std::cout << " PASS 2" << std::endl;

    fillGrid(origin_toBeFilled, toBeFilled); // function that performs the flood on the area of interest
    output.block(i_row_min, i_col_min, (i_row_max-i_row_min)+1, (i_col_max-i_col_min)+1) = toBeFilled;

    //std::cout << " PASS 3" << std::endl;
    //std::cout << "Before fillGrid: " << std::endl << occupancy_res << std::endl;

    //fillGrid(origin, output); // function that performs the flood

    //std::cout << "After fillGrid: " << std::endl << occupancy_res << std::endl;

    // For display (save data to text files for matplotlib)
    /*if (std::remainder(counter_frame, 30) < 0.1)
    {
        std::ofstream myrobotpos;
        if (ID_data==-1)
        {
            // Should not happen
        }
        else if (ID_data < 10)
        {
            myrobotpos.open("./StreamData/stream_posrobot_00" + std::to_string(ID_data) + ".txt");
        }
        else if (ID_data < 100)
        {
            myrobotpos.open("./StreamData/stream_posrobot_0" + std::to_string(ID_data) + ".txt");
        }
        else
        {
            myrobotpos.open("./StreamData/stream_posrobot_" + std::to_string(ID_data) + ".txt");
        }
        myrobotpos << state_robot(0,0) << "," << state_robot(1,0) << "\n";
        myrobotpos.close();


        Eigen::Matrix<float, 5, 1> limits;
        limits << 327.02, 377.02, 291.02, 341.02, 0.5;

        State state_attractor; state_attractor << 373, 296.33334, 0;

        compute_stream_border( limits, state_attractor, inside_square, ID_data);

        std::vector<Border> storage;
        storage = detect_borders( output);

        std::ofstream myobs;
        if (ID_data==-1)
        {
            // Should not happen
        }
        else if (ID_data < 10)
        {
            myobs.open("./StreamData/stream_obs_00" + std::to_string(ID_data) + ".txt");
        }
        else if (ID_data < 100)
        {
            myobs.open("./StreamData/stream_obs_0" + std::to_string(ID_data) + ".txt");
        }
        else
        {
            myobs.open("./StreamData/stream_obs_" + std::to_string(ID_data) + ".txt");
        }

        for (int iter=0; iter<storage.size(); iter++)
        {
            Border border_out = storage[iter];
            for (int i=0; i<border_out.rows(); i++)
            {
                myobs << border_out(i,0) << "," << border_out(i,1) << "," << border_out(i,2) << "," << border_out(i,3) << "," << border_out(i,4) << "\n";
            }
        }
        myobs.close();


        ID_data += 1;
    }
    counter_frame += 1;*/


    return output;
}


Eigen::Matrix<float, 1, 6> find_closest_point(Eigen::Matrix<float,1,2> const& robot, Border const& border) // find the closest cell of a given border
{
    // TODO: Is it right to use the Euclidian distance? For Gamma = 1 + euclidian dist to surface it's ok but for the others?

    float min_distance = std::pow(robot(0,0)-border(0,0),2) + std::pow(robot(0,1)-border(0,1),2); // distance to cell 0
    int i_closest = 0;
    for (int i=1; i<border.rows(); i++) // for each cell in the border (except the 0 one)
    {
        float distance = std::pow(robot(0,0)-border(i,0),2) + std::pow(robot(0,1)-border(i,1),2);
        if (distance < min_distance) // then it is closest then the current closest one
        {
            min_distance = distance;
            i_closest = i;
        }
    }
    Eigen::Matrix<float, 1, 6> res; res << border.row(i_closest), min_distance;
    return res; // return [x, y, type, charac_1, charac_2, distance_to_robot^2]
}

std::ofstream mypoints;

Eigen::Matrix<float, 4, 1> gamma_and_ref_vector(Eigen::Matrix<float,1,2>  robot, Eigen::Matrix<float, 1, 6> data_closest)// return the Gamma distance of a point knowing its closest border cell
{  // as well as the reference vector (point - projected_point_on_the_border, the normal vector to the surface that goes through the robot)
   Eigen::Matrix<float, 4, 1> output; // output is transpose of [gamma, x, y, phi]

   // Obstacle:
   // . . . . . . . .
   // . / - - - - \ .
   // . | x x x x | .
   // . | x x x x | .
   // . | x x / - / .
   // . \ - - / . . .
   // . . . . . . . .

   // Format of straight line : [x_cell, y_cell, 1, x_dir, y_dir]
   // "|" on the left  will have x_dir = -1 and y_dir = 0 because their normal vector points to the left
   // "|" on the right will have x_dir =  1 and y_dir = 0 because their normal vector points to the right
   // "-" on the top    will have x_dir = 0 and y_dir =  1 because their normal vector points to the top
   // "-" on the bottom will have x_dir = 0 and y_dir = -1 because their normal vector points to the bottom

   // Format of an outward arc of circle : [x_cell, y_cell, 2, unused, pos_center]
   // Pos center is a convention I defined to make a difference between the four quarters of circle
   // Pos 0  Pos 1  Pos 2  Pos 3
   //  - \    / -    | x    x |
   //  x |    | x    \ -    - /
   // The normal vector is going outward, from x to the \ or /
   // You use this type for convex corners like the four corners or a square

   // Format of an inward arc of circle : [x_cell, y_cell, 3, unused, pos_center]
   // Pos center is a convention I defined to make a difference between the four quarters of circle
   // Pos 0  Pos 1  Pos 2  Pos 3
   //  - \    / -    | x    x |
   //  x |    | x    \ -    - /
   // The normal vector is going inward, from \ or / to x
   // You use this type for concave corners like the two corners inside a U shaped obstacle

   // With the pos_center (six arcs of type 2 and two arcs of type 3)
   // . . . . . . . . .
   // . 1 - 0 . 1 - 0 .
   // . | x | . | x | .
   // . | x | . | x | .
   // . | x 0 - 1 x | .
   // . | x x x x x | .
   // . 2 - - - - - 3 .
   // . . . . . . . . .

   float angle = 0;
   if (data_closest(0,2)==1) // if the closest cell is a straight line "-" or "|"
   {
       output.block(1,0,2,1) = data_closest.block(0,3,1,2).transpose(); // assign x,y
   }
   else if ((data_closest(0,2)==2)||(data_closest(0,2)==3)) // if the closest cell is an arc of circle
   {
       switch (static_cast<int>(data_closest(0,4))) { // depending on the origin of the arc of circle
            case 0: angle = std::atan2(robot(0,1)-(data_closest(0,1)-0.5* size_cell), robot(0,0)-(data_closest(0,0)-0.5* size_cell)); break;
            case 1: angle = std::atan2(robot(0,1)-(data_closest(0,1)-0.5* size_cell), robot(0,0)-(data_closest(0,0)+0.5* size_cell)); break;
            case 2: angle = std::atan2(robot(0,1)-(data_closest(0,1)+0.5* size_cell), robot(0,0)-(data_closest(0,0)+0.5* size_cell)); break;
            case 3: angle = std::atan2(robot(0,1)-(data_closest(0,1)+0.5* size_cell), robot(0,0)-(data_closest(0,0)-0.5* size_cell)); break;
      }
        output.block(1,0,2,1) << std::cos(angle), std::sin(angle); // assign x,y
   }
   else
   {
        throw std::invalid_argument("Should not happen. Closest border cell has no type." );
   }
   output(3,0) = 0; // assign phi (always 0)

   Eigen::Matrix<float, 1, 2> projected;
   projected = get_projection_on_border( robot, data_closest, angle); // get the coordinates of the projection of the robot on the border

   // Checking if the robot is inside the obstacle (it can cross the boundary due to discretization)
   State normal_vec;
   switch (static_cast<int>(data_closest(0,2)))
   {
    case 1:
        normal_vec << data_closest(0,3), data_closest(0,4), 0; break;
    case 2:
        normal_vec << output(1,0), output(2,0), 0; break;
    case 3:
        normal_vec << output(1,0), output(2,0), 0; break;
    }
    // TODO: I think normal_vec << output(1,0), output(2,0), 0; works for the 3 cases so switch is useless


    // If the robot is inside the border than the scalar product between normal_vec and the vector (robot - projection) will be negative
    // as normal_point always points outward and (robot - projection) will point inwards
    State robot_vec; robot_vec << (robot(0,0)-projected(0,0)),(robot(0,1)-projected(0,1)),0;
    if (((data_closest(0,2)==1)||(data_closest(0,2)==2))&&(normal_vec.dot(robot_vec.colwise().normalized()) < 0))
    {
        // if true it means the robot is inside obstacle
        output(0,0) = 1; // we set gamma to 1 to trick the controller into thinking the robot is on the border
        return output;
    }
    else if ((data_closest(0,2)==3)&&(normal_vec.dot(robot_vec.colwise().normalized()) > 0)) // if true it means the robot is inside obstacle
    {
        output(0,0) = 1; // we set gamma to 1 to trick the controller into thinking the robot is on the border
        return output;
    }
    // End checking if robot is inside the obstacle


   //std::cout << "Projected: " << projected << std::endl;
   //mypoints << projected(0,0)  << "," << projected(0,1) << "\n";
   output(0,0) = 1 + std::pow(robot(0,0)-projected(0,0),2) + std::pow(robot(0,1)-projected(0,1),2); // gamma function that is used is 1 + euclidian_distance^2
   // TODO: Add call to a Gamma() function to centralize the computation of gamma at a single place
   // That way if we want to change gamma we do it at a single place without the need to change stuff everywhere

   return output;
}

void compute_quiver_border(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, Blob const& obstacle) // compute the velocity command for all points of a given grid
{   // used with Python for visualization purpose (Quiver function of Matlab)

    // Compute the velocity command at the initial time for all points of a [x,y] grid
    // Save the result in a txt file and then use a Python script to plot the vector field (quiver) with matplotlib
    //const int number_obstacles = mat_obs.cols();
    bool flag = false;

    std::ofstream myfile;
    myfile.open("quiver_data_border.txt");

    mypoints.open("quiver_points_border.txt");
    std::cout << "-- Quiver file opened --" << std::endl;

    Point center_blob = get_random(obstacle);
    Border border_out;
    border_out = compute_border( obstacle, center_blob);
    std::vector<Border> borders; borders.push_back(border_out);
    Blob filled_obstacle = fill_gaps(obstacle);

    for (float x=limits(0,0); x <= limits(1,0); x += limits(4,0)) // x direction of the grid
    {
        std::cout << x << std::endl;
        for (float y=limits(2,0); y <= limits(3,0); y += limits(4,0)) // y direction of the grid
        {
            //std::cout << x << " & " << y << std::endl;
            State state_robot; state_robot << x, y, 0; // the robot is set on the point of the grid
            flag = false;
            Eigen::Matrix<int, 2, 1> pos_cell;
            pos_cell = get_cell(x, y, size_cell);

            if (isPart(filled_obstacle, pos_cell(0,0), pos_cell(1,0)))// || isPartBorder(border_out, pos_cell(0,0), pos_cell(1,0)))
            {
                flag = true; // if the point/robot is inside an obstacle (distance < 1) then it makes no sense to compute its velocity command
            }

            if (!flag) // if not inside an obstacle
            {
                //State next_eps = next_step_single_obstacle_border( state_robot, state_attractor, border_out); // compute velocity command
                Eigen::Matrix<float, 4, 1> output = next_step_special( state_robot, state_attractor, border_out); // compute special velocity with transform in circle space
                State next_eps = output.block(0,0,3,1);
                myfile << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file
            }
        }
    }
    myfile.close();
    std::cout << "-- Quiver file closed --" << std::endl;
}

void compute_stream_border(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, std::vector<Blob> obstacles, int const& ID)
{   // Just like compute_quiver but with a small variation to be able to plot a "streamplot" with Matplotlib

    // Compute the velocity command at the initial time for all points of a [x,y] grid
    // Save the result in a txt file and then use a Python script to plot the vector field (quiver) with matplotlib
    //const int number_obstacles = mat_obs.cols();
    bool flag = false;

    std::vector<Border> borders;
    std::vector<Blob> filled_obstacles;

    std::ofstream myfile;
    if (ID==-1)
    {
        myfile.open("stream_data_border.txt");
        mypoints.open("stream_points_border.txt");
    }
    else if (ID < 10)
    {
        myfile.open("./StreamData/stream_data_border_00" + std::to_string(ID) + ".txt");
        mypoints.open("./StreamData/stream_points_border_00" + std::to_string(ID) + ".txt");
    }
    else if (ID < 100)
    {
        myfile.open("./StreamData/stream_data_border_0" + std::to_string(ID) + ".txt");
        mypoints.open("./StreamData/stream_points_border_0" + std::to_string(ID) + ".txt");
    }
    else
    {
        myfile.open("./StreamData/stream_data_border_" + std::to_string(ID) + ".txt");
        mypoints.open("./StreamData/stream_points_border_" + std::to_string(ID) + ".txt");
    }

    std::cout << "-- Stream file opened --" << std::endl;

    for (int i_obs=0; i_obs<obstacles.size(); i_obs++)
    {
        Point center_blob = get_random(obstacles[i_obs]);
        Border border_out = compute_border( obstacles[i_obs], center_blob);
        Blob filled_obstacle = fill_gaps(obstacles[i_obs]);

        borders.push_back(border_out);
        filled_obstacles.push_back(filled_obstacle);
    }

    for (float x=limits(0,0); x <= limits(1,0); x += limits(4,0)) // x direction of the grid
    {
        std::cout << x << std::endl;
        for (float y=limits(2,0); y <= limits(3,0); y += limits(4,0)) // y direction of the grid
        {
            State state_robot; state_robot << x, y, 0; // the robot is set on the point of the grid
            flag = false;
            Eigen::Matrix<int, 2, 1> pos_cell;
            pos_cell = get_cell(x, y, size_cell);

            for (int i=0; i < obstacles.size(); i++)
            {
                if (isPart(filled_obstacles[i], pos_cell(0,0), pos_cell(1,0)))// || isPartBorder(border_out, pos_cell(0,0), pos_cell(1,0)))
                {
                    flag = true; // if the point/robot is inside an obstacle (distance < 1) then it makes no sense to compute its velocity command
                }
            }

            if (!flag) // if not inside an obstacle
            {
                //State next_eps = next_step_single_obstacle_border( state_robot, state_attractor, border_out); // compute velocity command
                State next_eps = next_step_special_weighted( state_robot, state_attractor, borders, size_cell); // compute special velocity with transform in circle space
                myfile << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file
            }
            else
            {
                myfile << x << "," << y << "," << 0 << "," << 0 << "\n"; // write result in text file
            }
        }
    }
    myfile.close();
    std::cout << "-- Stream file closed --" << std::endl;
}

std::ofstream my_initial_space; // to store coordinates of points in the initial space
std::ofstream my_circle_space;  // to store coordinates of points in the circle space

void compute_morphing(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, std::vector<Blob> obstacles)
{   // compute morphing between initial and circle spaces
    // used with Python for visualization purpose

    // Compute the velocity command at the initial time for all points of a [x,y] grid
    // Save the result in a txt file and then use a Python script to plot the vector field (quiver) with matplotlib
    //const int number_obstacles = mat_obs.cols();
    bool flag = false;

    std::vector<Border> borders;
    std::vector<Blob> filled_obstacles;


    my_initial_space.open("Data_Initial_Space_bis.txt");
    my_circle_space.open("Data_Circle_Space_bis.txt");

    std::cout << "-- Morphing storage files opened --" << std::endl;

    for (int i_obs=0; i_obs<obstacles.size(); i_obs++)
    {
        Point center_blob = get_random(obstacles[i_obs]);
        Border border_out = compute_border( obstacles[i_obs], center_blob);
        Blob filled_obstacle = fill_gaps(obstacles[i_obs]);

        borders.push_back(border_out);
        filled_obstacles.push_back(filled_obstacle);
    }

    for (float x=limits(0,0); x <= limits(1,0); x += limits(4,0)) // x direction of the grid
    {
        std::cout << x << std::endl;
        for (float y=limits(2,0); y <= limits(3,0); y += limits(4,0)) // y direction of the grid
        {
            State state_robot; state_robot << x, y, 0; // the robot is set on the point of the grid
            flag = false;
            Eigen::Matrix<int, 2, 1> pos_cell;
            pos_cell = get_cell(x, y, size_cell);

            for (int i=0; i < obstacles.size(); i++)
            {
                if (isPart(filled_obstacles[i], pos_cell(0,0), pos_cell(1,0)))// || isPartBorder(border_out, pos_cell(0,0), pos_cell(1,0)))
                {
                    flag = true; // if the point/robot is inside an obstacle (distance < 1) then it makes no sense to compute its velocity command
                }
            }

            if (!flag) // if not inside an obstacle
            {
                //State next_eps = next_step_single_obstacle_border( state_robot, state_attractor, border_out); // compute velocity command
                State next_eps = next_step_special_weighted( state_robot, state_attractor, borders, size_cell); // compute special velocity with transform in circle space
                my_initial_space << x << "," << y << "\n"; // write position of the point in the initial space
            }
            else
            {
                //myfile << x << "," << y << "," << 0 << "," << 0 << "\n"; // write result in text file
            }
        }
    }
    my_initial_space.close();
    my_circle_space.close();
    std::cout << "-- Morphing storage files closed --" << std::endl;
}

void compute_trajectory_both_spaces(State const& state_robot, State const& state_attractor, std::vector<Blob> obstacles) // compute trajectory in initial and circle spaces
{   // used with Python for visualization purpose

    bool flag = false;

    std::vector<Border> borders;
    std::vector<Blob> filled_obstacles;

    my_initial_space.open("Trajectory_Initial_Space.txt");
    my_circle_space.open("Trajectory_Circle_Space.txt");

    std::cout << "-- Trajectory storage files opened --" << std::endl;

    for (int i_obs=0; i_obs<obstacles.size(); i_obs++)
    {
        Point center_blob = get_random(obstacles[i_obs]);
        Border border_out = compute_border( obstacles[i_obs], center_blob);
        Blob filled_obstacle = fill_gaps(obstacles[i_obs]);

        borders.push_back(border_out);
        filled_obstacles.push_back(filled_obstacle);
    }

    int N_steps = 5500;//2300;
    float time_step = 0.03; // time_step (does not have a physical sense)
    State robot; robot = state_robot;



    for (int i=0; i<N_steps; i++)
    {
        if (std::remainder(i,10)==0) {std::cout << i << std::endl;}
        State next_eps = next_step_special_weighted( robot, state_attractor, borders, size_cell); // compute special velocity with transform in circle space
        my_initial_space << robot(0,0) << "," << robot(1,0) << "\n"; // write position of the point in the initial space
        robot += next_eps * time_step;
    }
    my_initial_space.close();
    my_circle_space.close();
    std::cout << "-- Trajectory storage files closed --" << std::endl;
}


Eigen::Matrix<int, 2, 1> get_cell(float const& x, float const& y, float const& size_side_cell) // for a given point it returns the center of the closest cell of the grid
{
    // for instance for size_side_cell = 1, then the cell (0,0) is a square centered on (0,0) and its four
    // corners are (0.5, 0.5) (-0.5, 0.5) (-0.5,-0.5) (0.5,-0.5)
    float x_cell = std::floor((x + (size_side_cell/2)) / size_side_cell);
    float y_cell = std::floor((y + (size_side_cell/2)) / size_side_cell);
    int x_int = static_cast<int>(x_cell);
    int y_int = static_cast<int>(y_cell);
    Eigen::Matrix<int, 2, 1> out; out << x_int, y_int;
    return out;
}

State next_step_single_obstacle_border(State const& state_robot, State const& state_attractor, Border const& border)
{   // compute the velocity command of the robot (version with no limit distance)
    // based on its relative position to the border of a single obstacle

    // Compute all the steps for a single step and a single obstacle
    //std::cout << "PASS: " << state_robot.transpose() << std::endl;
    Eigen::Matrix<float, 1, 2> robot = state_robot.block(0,0,2,1).transpose();
    //std::cout << "Step 1" << std::endl;
    Eigen::MatrixXf closest(1,6);
    closest = find_closest_point(robot, border);
    //std::cout << "Step 2" << std::endl;
    Eigen::Matrix<float, 4, 1> gamma_ref;
    //std::cout << robot << " and " << closest << std::endl;
    gamma_ref = gamma_and_ref_vector( robot, closest);
    //std::cout << "Step 3" << std::endl;
    // Compute attractor function
    State f_eps = state_attractor - state_robot;
    f_eps = 4 * f_eps.normalized();
    //std::cout << "f_eps=" << f_eps << std::endl;
    //std::cout << "Step 4" << std::endl;
    // Several steps to get D(epsilon) matrix
    float lamb_r = 1 - (1/gamma_ref(0,0));//lambda_r( state_robot, obs, limit_dist); // optimization -> include gamma as argument of lambda_r and lambda_e (TODO?)
    float lamb_e = 1 + (1/gamma_ref(0,0));//lambda_e( state_robot, obs, limit_dist);
    Eigen::Matrix<float, number_states, number_states> D_eps = D_epsilon( lamb_r, lamb_e);

    // Several steps to get E(epsilon) matrix
    State r_eps_vector = gamma_ref.block(1,0,3,1);
    //std::cout << "Gammaref " << gamma_ref << std::endl;
    //Pstd::cout << lamb_r << " and " << lamb_e << std::endl;

    // Remove tail effect (see the paper for a clean explanation)
    /*State tail_vector = (state_attractor-state_robot);
    if (r_eps_vector.dot(tail_vector.colwise().normalized()) > 0.3) // can be between 0 and 1, I chose > 0 because with thin ellipse I had some issues
    {
        return f_eps;
    }*/

    State gradient_vector; gradient_vector <<  r_eps_vector(0,0), r_eps_vector(1,0), 0; //2 * std::sqrt(closest(0,5)) * r_eps_vector(0,0), 2 * std::sqrt(closest(0,5)) * r_eps_vector(1,0), 0;

    Eigen::Matrix<float, number_states, number_states-1> ortho_basis = gram_schmidt( gradient_vector);
    Eigen::Matrix<float, number_states, number_states> E_eps = E_epsilon( r_eps_vector, ortho_basis);
    //std::cout << "r_eps_vector=" << std::endl << r_eps_vector << std::endl;
    //std::cout << "gradient_vector=" << std::endl << gradient_vector << std::endl;
    //std::cout << "ortho_basis=" << std::endl << ortho_basis << std::endl;
    // std::cout << "E_epsilon=" << E_eps << std::endl;

    // Compute M(epsilon)
    Eigen::Matrix<float, number_states, number_states> M_eps = M_epsilon( D_eps, E_eps);

    // Compute epsilon_dot
    State velocity_next = M_eps * f_eps; // non moving obstacle

    return velocity_next;
}

State next_step_several_obstacles_border( State const& state_robot, State const& state_attractor, std::vector<Border> const& borders)
{   // compute the velocity command of the robot with several obstacles (version with no limit distance)
    // based on its relative position to the border of several obstacles

    // Compute all the steps for a single step and a single obstacle
    Eigen::Matrix<float, 1, 2> robot = state_robot.block(0,0,2,1).transpose();
    Eigen::MatrixXf closest(1,6);
    Eigen::Matrix<float, 4, 1> gamma_ref;
    State cmd_velocity;
    float lamb_r = 0;
    float lamb_e = 0;
    Eigen::Matrix<float, number_states, number_states> D_eps;
    State r_eps_vector;
    Eigen::Matrix<float, number_states, number_states-1> ortho_basis;
    Eigen::Matrix<float, number_states, number_states> E_eps;
    Eigen::Matrix<float, number_states, number_states> M_eps;

    // Compute attractor function
    cmd_velocity = state_attractor - state_robot;

    for (int iter=0; iter< borders.size(); iter++)
    {
        std::cout << "Obstacle " << iter << std::endl;
        //std::cout << storage[iter] << std::endl;

        closest = find_closest_point(robot, borders[iter]);
        gamma_ref = gamma_and_ref_vector( robot, closest);
        std::cout << " Distance: " << gamma_ref(0,0) << std::endl;

        //f_eps = 4 * f_eps.normalized();

        // Several steps to get D(epsilon) matrix
        lamb_r = 1 - (1/gamma_ref(0,0));//lambda_r( state_robot, obs, limit_dist); // optimization -> include gamma as argument of lambda_r and lambda_e (TODO?)
        lamb_e = 1 + (1/gamma_ref(0,0));//lambda_e( state_robot, obs, limit_dist);
         D_eps = D_epsilon( lamb_r, lamb_e);

        // Several steps to get E(epsilon) matrix
        r_eps_vector = gamma_ref.block(1,0,3,1);

        // Remove tail effect (see the paper for a clean explanation)
        /*State tail_vector = (state_attractor-state_robot);
        if (r_eps_vector.dot(tail_vector.colwise().normalized()) > 0.3) // can be between 0 and 1, I chose > 0 because with thin ellipse I had some issues
        {
            return f_eps;
        }*/

        State gradient_vector; gradient_vector <<  r_eps_vector(0,0), r_eps_vector(1,0), 0; //2 * std::sqrt(closest(0,5)) * r_eps_vector(0,0), 2 * std::sqrt(closest(0,5)) * r_eps_vector(1,0), 0;

        Eigen::Matrix<float, number_states, number_states-1> ortho_basis = gram_schmidt( gradient_vector);
        Eigen::Matrix<float, number_states, number_states> E_eps = E_epsilon( r_eps_vector, ortho_basis);
        Eigen::Matrix<float, number_states, number_states> M_eps = M_epsilon( D_eps, E_eps);
        cmd_velocity = M_eps * cmd_velocity; // non moving obstacle
    }
    // Set the angular velocity to align the robot with the direction it moves (post process for better movement, thinner profile to go between obstacles)
    cmd_velocity(2,0) = std::atan2(cmd_velocity(1,0),cmd_velocity(0,0)) - state_robot(2,0); // angle difference used for angular speed control (gain of 1)

    return speed_limiter(cmd_velocity);
}
Eigen::Matrix<float, 1, 2> get_projection_on_border(Eigen::Matrix<float,1,2>  robot, Eigen::Matrix<float, 1, 6> data_closest, float const& angle)
{   // Project a point on the border of an obstacle
    // For a given point and knowing the closest border cell (position and line/arc of circle), one can project the point onto the border
    // It is used to get the normal vector to the surface since normal vector = point - projected_point

    Eigen::Matrix<float, 1, 2> projected_point;
    projected_point << 0,0;

    /*if (true)
    {
        std::cout << " -- PROJECTION --" << std::endl;
        std::cout << "Robot:   " << robot << std::endl;
        std::cout << "Closest: " << data_closest << std::endl;
        std::cout << " --            --" << std::endl;
    }*/


    // Margin is a length between 0 and size_cell
    // If we get the obstacle
    // / - - \
    // | x x |
    // | x x |
    // \ - - /
    // Then if margin = 0 the border touches the surface of the obstacle
    // If margin = size cell the distance between the border and the surface is a whole cell


    switch (static_cast<int>(data_closest(0,2)))
    {
    case 1: // Straight lines
        if (data_closest(0,3)!=0) // normal vector along X axis
        {
            projected_point(0,1) = robot(0,1);
            projected_point(0,0) = data_closest(0,0) + data_closest(0,3) * (margin - (size_cell*0.5)); // center_cell +- half_side (depending on normal direction) + margin
        }
        else // normal vector along Y axis
        {
            projected_point(0,0) = robot(0,0);
            projected_point(0,1) = data_closest(0,1) + data_closest(0,4) * (margin - (size_cell*0.5));  // center_cell +- half_side (depending on normal direction) + margin
        }
        break;
    case 2: // Outward circles
        switch (static_cast<int>(data_closest(0,4)))
        {
        case 0: projected_point(0,0) = data_closest(0,0) - (size_cell*0.5) + (margin*std::cos(angle));
                projected_point(0,1) = data_closest(0,1) - (size_cell*0.5) + (margin*std::sin(angle)); break;
        case 1: projected_point(0,0) = data_closest(0,0) + (size_cell*0.5) + (margin*std::cos(angle));
                projected_point(0,1) = data_closest(0,1) - (size_cell*0.5) + (margin*std::sin(angle)); break;
        case 2: projected_point(0,0) = data_closest(0,0) + (size_cell*0.5) + (margin*std::cos(angle));
                projected_point(0,1) = data_closest(0,1) + (size_cell*0.5) + (margin*std::sin(angle)); break;
        case 3: projected_point(0,0) = data_closest(0,0) - (size_cell*0.5) + (margin*std::cos(angle));
                projected_point(0,1) = data_closest(0,1) + (size_cell*0.5) + (margin*std::sin(angle)); break;
        }
        break;
    case 3: // Inward circles
        switch (static_cast<int>(data_closest(0,4)))
        {
        case 0: projected_point(0,0) = data_closest(0,0) - (size_cell*0.5) + ((size_cell-margin)*std::cos(angle));
                projected_point(0,1) = data_closest(0,1) - (size_cell*0.5) + ((size_cell-margin)*std::sin(angle)); break;
        case 1: projected_point(0,0) = data_closest(0,0) + (size_cell*0.5) + ((size_cell-margin)*std::cos(angle));
                projected_point(0,1) = data_closest(0,1) - (size_cell*0.5) + ((size_cell-margin)*std::sin(angle)); break;
        case 2: projected_point(0,0) = data_closest(0,0) + (size_cell*0.5) + ((size_cell-margin)*std::cos(angle));
                projected_point(0,1) = data_closest(0,1) + (size_cell*0.5) + ((size_cell-margin)*std::sin(angle)); break;
        case 3: projected_point(0,0) = data_closest(0,0) - (size_cell*0.5) + ((size_cell-margin)*std::cos(angle));
                projected_point(0,1) = data_closest(0,1) + (size_cell*0.5) + ((size_cell-margin)*std::sin(angle)); break;
        }
        break;
    default: throw std::invalid_argument("Should not happen. Closest border cell has no type." ); break;
    }

    return projected_point;
}

// In gamma_and_ref -> get the projection on the border, easy with the info of closest

void growing_obstacle() // create a single blob of occupied cells to test the border creation of "compute_border"
{   // used with Python for visualization purpose

    // Basically it randomly grows a single obstacle in the middle of an empty occupancy grid

    std::ofstream myobs;
    std::srand(42);
    int n_max = 61;
    Blob obst(n_max,2);
    int n_obs = 2;
    obst.row(0) << 5, 5;
    obst.row(1) << 6, 5;
    while (n_obs < n_max)
    {
       for (int i_row = 0; (i_row < n_obs)&&(n_obs < n_max); i_row++)
       {

           float r_x = rand() % 3 - 1; // -1 or 0 or 1
           float r_y = rand() % 3 - 1; // -1 or 0 or 1
           float x = obst(i_row, 0) + r_x;
           float y = obst(i_row, 1) + r_y;
           if (!isPart( obst, x, y))
           {
                Point center_blob; //center_blob << 5, 5;
                center_blob = get_random(obst, n_obs);
                //std::cout << "Random (" << center_blob(0,0) << "," << center_blob(0,1) << ") for blob:" << std::endl;
                //std::cout << obst << std::endl;
                Border border_out;
                border_out = compute_border( obst, center_blob);
                //std::cout << border_out << std::endl;
                if (n_obs < 10)
                {
                    mypoints.open("./Obstacles/obs_0" + std::to_string(n_obs) + ".txt");
                    myobs.open("./Obstacles/border_0" + std::to_string(n_obs) + ".txt");
                }
                else
                {
                    mypoints.open("./Obstacles/obs_" + std::to_string(n_obs) + ".txt");
                    myobs.open("./Obstacles/border_" + std::to_string(n_obs) + ".txt");
                }

                /*for (float x_point=-5; x_point <= 15; x_point += 0.2) // x direction of the grid
                {
                    for (float y_point=-5; y_point <= 15; y_point += 0.2) // y direction of the grid
                    {
                        Eigen::Matrix<float,1,2>  robot; robot << x_point, y_point;
                        Eigen::MatrixXf closest(1,6);
                        closest = find_closest_point(robot, border_out);
                        Eigen::Matrix<float, 4, 1> gamma_ref;
                        gamma_ref = gamma_and_ref_vector( robot, closest);
                    }
                }*/

                for (int i=0; i<border_out.rows(); i++)
                {
                    myobs << border_out(i,0) << "," << border_out(i,1) << "," << border_out(i,2) << "," << border_out(i,3) << "," << border_out(i,4) << "\n";
                }
                myobs.close();

                /*for (float x_point = -5; x_point <=15; x_point+=0.2)
                {

                    Eigen::Matrix<float,1,2>  robot; robot << x_point, -5;
                    Eigen::MatrixXf closest(1,6);
                    closest = find_closest_point(robot, border_out);
                    Eigen::Matrix<float, 4, 1> gamma_ref;
                    gamma_ref = gamma_and_ref_vector( robot, closest);

                    robot << x_point, 15;
                    closest = find_closest_point(robot, border_out);
                    gamma_ref = gamma_and_ref_vector( robot, closest);
                }

                for (float y_point = -5; y_point <=15; y_point+=0.2)
                {
                    Eigen::Matrix<float,1,2>  robot; robot << -5, y_point;
                    Eigen::MatrixXf closest(1,6);
                    closest = find_closest_point(robot, border_out);
                    Eigen::Matrix<float, 4, 1> gamma_ref;
                    gamma_ref = gamma_and_ref_vector( robot, closest);

                    robot <<  15, y_point;
                    closest = find_closest_point(robot, border_out);
                    gamma_ref = gamma_and_ref_vector( robot, closest);
                }*/

                mypoints.close();

                obst.row(n_obs) << x, y;
                n_obs += 1;
                std::cout << n_obs << std::endl;
           }
       }
   }


    myobs.open("./Obstacles/growing_several_obs.txt"); //each starting point has its own file
    for (int i=0; i<obst.rows(); i++)
    {
        myobs << obst(i,0) << "," << obst(i,1) << "\n";
    }
    myobs.close();

}

void growing_several_obstacle() // create several blobs of occupied cells to test the obstacle detection algorithm "detect_borders"
{   // used with Python for visualization purpose

    // Basically it randomly grows several obstacles in the middle of an empty occupancy grid

    Grid occupancy_grid = Eigen::MatrixXi::Zero(20, 20);

    std::ofstream myobs;
    std::srand(49);
    int n_max = 81;
    int n_obs = 0;
    Blob obst(n_max,2);
    int r_x = 0;
    int r_y = 0;
    while (n_obs < n_max)
    {
        std::cout << n_obs << "/" << n_max << std::endl;
        do {
            r_x = rand() % 18 + 1;
            r_y = rand() % 18 + 1;
        } while (occupancy_grid(r_x, r_y)==100);
        occupancy_grid(r_x, r_y) = 100;
        obst(n_obs, 0) = r_x;
        obst(n_obs, 1) = r_y;
        n_obs +=1;

        std::cout << " PASS 1 " << std::endl;
        if (n_obs==49) {std::cout << occupancy_grid << std::endl;}
        std::vector<Border> storage;

        State placeholder_state; placeholder_state << 0, 0, 0;
        storage = detect_borders( occupancy_grid, placeholder_state);

        std::cout << " PASS 2 " << std::endl;

        if (n_obs < 10)
        {
            myobs.open("./Obstacles/several_border_0" + std::to_string(n_obs) + ".txt");
        }
        else
        {
            myobs.open("./Obstacles/several_border_" + std::to_string(n_obs) + ".txt");
        }

        std::cout << " PASS 3 " << std::endl;
        for (int iter=0; iter<storage.size(); iter++)
        {
            Border border_out = storage[iter];
            for (int i=0; i<border_out.rows(); i++)
            {
                myobs << border_out(i,0) << "," << border_out(i,1) << "," << border_out(i,2) << "," << border_out(i,3) << "," << border_out(i,4) << "\n";
            }
        }
        myobs.close();

        myobs.open("./Obstacles/growing_obs_" + std::to_string(n_obs) + ".txt"); //each starting point has its own file
        for (int i=0; i<obst.rows(); i++)
        {
            myobs << obst(i,0) << "," << obst(i,1) << "\n";
        }
        myobs.close();
   }

    // Previous pos of growing obs
}


std::vector<Border> detect_borders( Grid & occupancy_grid, State const& state_robot ) // scan an occupancy grid for obstacles and returns a vector containing the borders of all detected obstacles
{
    // Detect the limits of the smallest box which contains all the occupied cells
    Eigen::MatrixXi col_sum = occupancy_grid.colwise().sum();
    Eigen::MatrixXi row_sum = occupancy_grid.rowwise().sum();

    //std::cout << col_sum << std::endl;
    //std::cout << row_sum << std::endl;

    int i_col_min = col_sum.cols();
    int i_col_max = 0;
    int i_row_min = row_sum.cols();
    int i_row_max = 0;

    // Get the index of the first column with a non-zero element
    int k = 0;
    while (((k+1)<col_sum.cols()) && (col_sum(0,k+1)==0)) {k++;}
    k++;
    i_col_min = k;

    // Get the index of the last column with a non-zero element
    k = col_sum.cols()-1;
    while (((k-1)>=0) && (col_sum(0,k-1)==0)) {k--;}
    k--;
    i_col_max = k;

    // Get the index of the first row with a non-zero element
    k = 0;
    while (((k+1)<row_sum.rows()) && (row_sum(k+1,0)==0)) {k++;}
    k++;
    i_row_min = k;

    // Get the index of the last row with a non-zero element
    k = row_sum.rows()-1;
    while (((k-1)>=0) && (row_sum(k-1,0)==0)) {k--;}
    k--;
    i_row_max = k;
    // End detection of box limits


    std::vector<Border> detected_borders;

    Point start_obstacle;
    Border border_obstacle;

    for (int cursor_row=0; cursor_row < occupancy_grid.rows(); cursor_row++) // scan each row
    {
        for (int cursor_col=0; cursor_col < occupancy_grid.cols(); cursor_col++) // scan each column
        {
            if (occupancy_grid(cursor_row, cursor_col)==100) // if the cell is occupied -> it may be new obstacle
            {
                bool flag_not_in_border = true;
                for (int i_border=0; ((i_border<detected_borders.size())&&(flag_not_in_border == true)); i_border++)
                {
                    // cursor_col - 1 because we scan from left to right so we hit the left side of obstacle
                    // so the cell which may be part of a border is the cell on the left of the detected occupied cell
                    if (isPartBorder(detected_borders[i_border],cursor_row,cursor_col-1))
                    {
                        flag_not_in_border = false; // the detected cell is already part of a border
                        // set col to the other side of the obstacle by going around it
                        cursor_col = other_side_obstacle( detected_borders[i_border], cursor_row, cursor_col-1);
                    }
                }
                if (flag_not_in_border) // the detected cell is not part of a border, it means we have hit a new obstacle
                {
                    //std::cout << "New obstacle!" << std::endl;

                    start_obstacle << cursor_row, cursor_col;
                    Grid temp_grid = occupancy_grid;
                    Blob obstacle;
                    //std::cout << "Entering occupied cells detection" << std::endl;
                    explore_obstacle( obstacle, temp_grid, cursor_row, cursor_col); // detect all occupied cells that form the obstacle
                    //std::cout << "All occupied cells detected" << std::endl;

                    // Check if the obstacle encloses the robot (a closed room for instance)
                    int x_min = (obstacle.col(0)).minCoeff();
                    int x_max = (obstacle.col(0)).maxCoeff();
                    int y_min = (obstacle.col(1)).minCoeff();
                    int y_max = (obstacle.col(1)).maxCoeff();

                    //std::cout << i_row_min << " | " << i_row_max << " | " << i_col_min << " | " << i_col_max << std::endl;
                    //std::cout << x_min << " | " << x_max << " | " << y_min << " | " << y_max << std::endl;

                    if (false&&(x_min==i_row_min)&&(x_max==i_row_max)&&(y_min==i_col_min)&&(y_max==i_col_max))
                    {
                        std::cout << "Enclosing obstacle detected" << std::endl;

                        // Second Try

                        Eigen::Matrix<int, 2, 1> cell_robot = get_cell( state_robot(0,0), state_robot(1,0), size_cell);
                        int x_index_max = -1;//i_col_max+1;
                        for (int i_obs=0; i_obs<obstacle.rows(); i_obs++)
                        {
                            /*if ((obstacle(i_obs,1)==cell_robot(1,0)))
                            {
                                std::cout << "Considering (" << obstacle(i_obs,0) << "," << obstacle(i_obs,1) << ")" << std::endl;
                            }*/
                            if ((obstacle(i_obs,1)==cell_robot(1,0))&&(obstacle(i_obs,0)>x_index_max)&&(obstacle(i_obs,0) < cell_robot(0,0))) // same row as the robot
                            {
                                x_index_max = obstacle(i_obs,0); // we want the cell as left as possible
                            }
                            /*else if ((obstacle(i_obs,0)==cell_robot(0,0))&&(y_index_max==-1))
                            {
                                y_index_max = obstacle(i_obs,1); // we want the cell as left as possible
                            }*/
                            /*else if ((obstacle(i_obs,0)==cell_robot(0,0))&&(y_index_max!=-1)&&(obstacle(i_obs,1)<y_index_max))
                            {
                                y_index_max = obstacle(i_obs,1); // we want the cell as left as possible
                            }*/
                        }
                        if (x_index_max == -1) {throw std::logic_error("Obstacle has no occupied cell on the row of the robot");}
                        //std::cout << "cell robot: (" << cell_robot(0,0) << "," << cell_robot(1,0) << ")" << std::endl;
                        //std::cout << "x_index_max: " << x_index_max << std::endl;
                        Point start_special; start_special << x_index_max, cell_robot(1,0);

                        // First Try
                        /*bool continue_loop = true;
                        int i_x = x_min;
                        int i_y = y_min;
                        while ((continue_loop)&&(i_x<=x_max))
                        {
                            while ((continue_loop)&&(i_y<=y_max))
                            {
                                if (occupancy_grid(i_x, i_y)!=100)
                                {
                                    if (i_y==y_min) {i_x -= 1;}
                                    else {i_y--;}
                                    // x x x x x
                                    // x x x x x
                                    // . . . . x
                                    // . . . . x
                                    // x x x x x As the first . belongs to the first col we can't take col-1
                                    continue_loop = false;
                                }
                                i_y++;
                            }
                            i_x++;
                        }
                        i_x--; i_y--;
                        Point start_special; start_special << i_x, i_y;*/

                        border_obstacle = compute_border_and_fill( obstacle, start_special, occupancy_grid); // compute the border of the obstacle
                        detected_borders.push_back(border_obstacle); // add the new border to the list of borders
                        // no update of cursor_col
                        display_border( obstacle, border_obstacle);

                        for (int i_obs=0; i_obs<obstacle.rows(); i_obs++)
                        {
                            occupancy_grid(obstacle(i_obs,0),obstacle(i_obs,1)) = 0;
                        }
                        //std::cout << occupancy_grid << std::endl;

                    }
                    else if (obstacle.rows()>1) // if the obstacle is not a single occupied cell (noise)
                    {
                        //std::cout << "Entering compute border and fill" << std::endl;
                        border_obstacle = compute_border_and_fill( obstacle, start_obstacle, occupancy_grid); // compute the border of the obstacle
                        //std::cout << "Leaving compute border and fill" << std::endl;
                        detected_borders.push_back(border_obstacle); // add the new border to the list of borders
                        // set col to the other side of the obstacle by going around it in order to scan the rest of the row
                        cursor_col = other_side_obstacle( border_obstacle, cursor_row, cursor_col-1);
                    }
                    /*td::cout << border_obstacle << std::endl;
                    std::cout << cursor_row<< std::endl;
                    std::cout << cursor_col << std::endl;
                    return detected_borders;*/
                    //display_border( obstacle, border_obstacle);
                }
            }
        }
    }
    return detected_borders;
}

std::vector<Blob> detect_blobs( Grid & occupancy_grid ) // scan an occupancy grid for obstacles and returns a vector containing the blobs of all detected obstacles
{
    // The same as detect_borders but it returns the cells that form the obstacles instead of their borders.

    std::vector<Border> detected_borders;
    std::vector<Blob> detected_blobs;
    Point start_obstacle;
    Border border_obstacle;

    for (int cursor_row=0; cursor_row < occupancy_grid.rows(); cursor_row++) // scan each row
    {
        for (int cursor_col=0; cursor_col < occupancy_grid.cols(); cursor_col++) // scan each column
        {
            if (occupancy_grid(cursor_row, cursor_col)==100) // if the cell is occupied -> it may be new obstacle
            {
                bool flag_not_in_border = true;
                for (int i_border=0; ((i_border<detected_borders.size())&&(flag_not_in_border == true)); i_border++)
                {
                    // cursor_col - 1 because we scan from left to right so we hit the left side of obstacle
                    // so the cell which may be part of a border is the cell on the left of the detected cell
                    if (isPartBorder(detected_borders[i_border],cursor_row,cursor_col-1))
                    {
                        flag_not_in_border = false;
                        // set col to the other side of the obstacle by going around it
                        cursor_col = other_side_obstacle( detected_borders[i_border], cursor_row, cursor_col-1);
                    }
                }
                if (flag_not_in_border) // the detected cell is not part of a border, it means we have hit a new obstacle
                {
                    //std::cout << "New obstacle!" << std::endl;

                    start_obstacle << cursor_row, cursor_col;
                    Grid temp_grid = occupancy_grid;
                    Blob obstacle;
                    explore_obstacle( obstacle, temp_grid, cursor_row, cursor_col); // detect all occupied cells that form the obstacle

                    if (obstacle.rows()>1)
                    {
                        border_obstacle = compute_border_and_fill( obstacle, start_obstacle, occupancy_grid); // compute the border of the obstacle
                        detected_borders.push_back(border_obstacle); // add the new border to the list of borders
                        detected_blobs.push_back(obstacle); // add the new blob to the list of blobs
                        // set col to the other side of the obstacle by going around it in order to scan the rest of the row
                        cursor_col = other_side_obstacle( border_obstacle, cursor_row, cursor_col-1);
                    }
                }
            }
        }
    }
    return detected_blobs;
}


int other_side_obstacle(Border const& border, int const& row, int const& col)
{
    // return the index of the column on the other side an obstacle thanks to its border
    // the scan for obstacles is done from left to right so by giving (row, col_left) on the left of the obstacle the algorithm returns col_right
    // then the scan resumes from the (row, col_right) cell

    // in case there are several non convex obstacles that are kind of interlocked we need to get out as left as possible
    // for instance if there is a I-shaped obstacle in a U-shaped obstacle, we have to get out in the middle or the U and not on the other side:
    // . . . . . . . . .
    // . x . x x . x . .
    // . x . x x . x . .
    // . x . x x . x . .
    // . x . x x . x . .
    // . x . . . . x . .
    // . x x x x x x . .
    // . . . . . . . . .


    int mini = border.col(1).maxCoeff();
    if (mini <= col) { return (col+2); std::cout << "SPECIAL" << std::endl;} // test to avoid an issue when two obstacles have only a 1-cell gap between them
    for (int i=0; i<border.rows(); i++)
    {
        if ((row==border(i,0)) && (border(i,1)>col) && (mini>border(i,1)))
        {
            mini = border(i,1);
        }
    }
    return mini;
}

void explore_obstacle( Blob & obstacle, Grid & occupancy_grid, int const& row, int const& col) // explore an obstacle to detect all its cells
{
    // start from one cell and a recursive function gradually explores nearby cells and checks if they are occupied (i.e part of the obstacle)

    //std::cout << "(" << row << "," << col << ") for size (" << occupancy_grid.rows() << "," << occupancy_grid.cols() << ")  |";

    obstacle.conservativeResize(obstacle.rows()+1, Eigen::NoChange);
    obstacle.row(obstacle.rows()-1) << row, col;
    occupancy_grid(row,col) = 0;

    // 4 cardinal directions
    if ((row+1) < occupancy_grid.rows()) { if (occupancy_grid(row+1, col)==100) {explore_obstacle( obstacle, occupancy_grid, row+1, col  );}}
    if ((row-1) >= 0                   ) { if (occupancy_grid(row-1, col)==100) {explore_obstacle( obstacle, occupancy_grid, row-1, col  );}}
    if ((col+1) < occupancy_grid.cols()) { if (occupancy_grid(row, col+1)==100) {explore_obstacle( obstacle, occupancy_grid, row  , col+1);}}
    if ((col-1) >= 0                   ) { if (occupancy_grid(row, col-1)==100) {explore_obstacle( obstacle, occupancy_grid, row  , col-1);}}

    // 4 corners
    if (((row+1) < occupancy_grid.rows()) && ((col+1) < occupancy_grid.cols())) { if (occupancy_grid(row+1, col+1)==100) {explore_obstacle( obstacle, occupancy_grid, row+1, col+1);}}
    if (((row+1) < occupancy_grid.rows()) && ((col-1) >= 0                   )) { if (occupancy_grid(row+1, col-1)==100) {explore_obstacle( obstacle, occupancy_grid, row+1, col-1);}}
    if (((row-1) >= 0                   ) && ((col+1) < occupancy_grid.cols())) { if (occupancy_grid(row-1, col+1)==100) {explore_obstacle( obstacle, occupancy_grid, row-1, col+1);}}
    if (((row-1) >= 0                   ) && ((col-1) >= 0                   )) { if (occupancy_grid(row-1, col-1)==100) {explore_obstacle( obstacle, occupancy_grid, row-1, col-1);}}


    /*
    // 4 cardinal directions
    if (((row+1) < occupancy_grid.rows()) && (occupancy_grid(row+1, col)==100)) {explore_obstacle( obstacle, occupancy_grid, row+1, col  );}
    if (((row-1) >= 0                   ) && (occupancy_grid(row-1, col)==100)) {explore_obstacle( obstacle, occupancy_grid, row-1, col  );}
    if (((col+1) < occupancy_grid.cols()) && (occupancy_grid(row, col+1)==100)) {explore_obstacle( obstacle, occupancy_grid, row  , col+1);}
    if (((col-1) >= 0                   ) && (occupancy_grid(row, col-1)==100)) {explore_obstacle( obstacle, occupancy_grid, row  , col-1);}

    // 4 corners
    if (((row+1) < occupancy_grid.rows()) && ((col+1) < occupancy_grid.cols()) && (occupancy_grid(row+1, col+1)==100)) {explore_obstacle( obstacle, occupancy_grid, row+1, col+1);}
    if (((row+1) < occupancy_grid.rows()) && ((col-1) >= 0                   ) && (occupancy_grid(row+1, col-1)==100)) {explore_obstacle( obstacle, occupancy_grid, row+1, col-1);}
    if (((row-1) >= 0                   ) && ((col+1) < occupancy_grid.cols()) && (occupancy_grid(row-1, col+1)==100)) {explore_obstacle( obstacle, occupancy_grid, row-1, col+1);}
    if (((row-1) >= 0                   ) && ((col-1) >= 0                   ) && (occupancy_grid(row-1, col-1)==100)) {explore_obstacle( obstacle, occupancy_grid, row-1, col-1);}
    */

    // the obstacle is directly updated with the detected occupied cells
}

bool verbose = false; // to trigger verbose mode

/*State next_step_several_obstacles_border( State const& state_robot, State const& state_attractor, std::vector<Border> const& borders)
{
    // Compute all the steps for a single step and a single obstacle
    Eigen::Matrix<float, 1, 2> robot = state_robot.block(0,0,2,1).transpose();
    Eigen::MatrixXf closest(1,6);
    Eigen::Matrix<float, 4, 1> gamma_ref;
    State cmd_velocity;
    float lamb_r = 0;
    float lamb_e = 0;
    Eigen::Matrix<float, number_states, number_states> D_eps;
    State r_eps_vector;
    Eigen::Matrix<float, number_states, number_states-1> ortho_basis;
    Eigen::Matrix<float, number_states, number_states> E_eps;
    Eigen::Matrix<float, number_states, number_states> M_eps;

    // Compute attractor function
    cmd_velocity = state_attractor - state_robot;

    for (int iter=0; iter< borders.size(); iter++)
    {
    }
}*/

Eigen::MatrixXf weights_special(State const& state_robot, Eigen::MatrixXf const& mat_gamma, int const& method, float const& limit_distance)
{   // compute the relative weights of obstacles in the limit range

    const int number_obstacles = mat_gamma.cols();

    // mat_obs is a matrix with size "7 x number_obstacles"
    Eigen::MatrixXf mat_weights(1, number_obstacles); // "1 x number_of_obstacles" one weight for each obstacle
    Eigen::MatrixXf mat_dist(1, number_obstacles);    // "1 x number_of_obstacles" one distance for each obstacle
    Eigen::MatrixXf mat_prod(1, number_obstacles);    // "1 x number_of_obstacles" one product for each obstacle

    // Fill the mat_dist matrix (compute only once gamma(eps) for each obstacle for optimization purpose)
    for (int i=0; i < number_obstacles; i++)
    {
        mat_dist(0,i) = mat_gamma(0,i);
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
                //std::cout << i << " is in contact with the robot | ";
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
        if (denominator == 0) // no obstacle in range
        {
            mat_weights(0,i) = 0;
        }
        else
        {
            mat_weights(0,i) = mat_prod(0,i) / denominator; // relative weight of obstacles in range
        }
    }

    // /!\ TEST 16/04 to smooth changes of direction
    for (int i=0; i < number_obstacles; i++)
    {
        float tempo = mat_prod(0,i);
        mat_weights(0,i) =  mat_weights(0,i) * std::min(static_cast<float>(1.0), tempo);
    }

    return mat_weights;
}

State next_step_special_weighted(State const& state_robot, State const& state_attractor, std::vector<Border> const& borders, float const& size_of_cells)
{   // compute the velocity command for a given position of the robot/attractor/obstacles

    // Compute all the steps to get the velocity command considering several obstacles
    const int number_obstacles = borders.size();

    // Velocity command for each obstacle
    // mat_velocities is a matrix with size "number_states x number_obstacles"
    Eigen::MatrixXf mat_velocities(number_states, number_obstacles);  // "3 x number_of_obstacles" (x,y,phi)
    Eigen::MatrixXf mat_gamma(1, number_obstacles);                   // "1 x number_of_obstacles" (gamma distance)

    for (int i=0; i < borders.size(); i++)
    {
        Eigen::Matrix<float, 4, 1> output = next_step_special(state_robot, state_attractor, borders[i]); // compute velocity command for each obstacle
        mat_velocities(0,i) = output(0,0);
        mat_velocities(1,i) = output(1,0);
        mat_velocities(2,i) = output(2,0);
        // mat_velocities.col(i) = output.block(0,0,3,1); // TODO: Suddenly started to cause a Segmentation fault for no reason :O
        // Assigning the block does not work anymore but doing it coefficient by coefficient works... It makes no sense
        mat_velocities(2,i) = 0;
        mat_gamma(0,i) = output(3,0);
    }

    // Norm of the velocity command for each obstacle
    Eigen::MatrixXf mat_magnitudes( 1, number_obstacles);
    mat_magnitudes = mat_velocities.colwise().norm();

    // Direction of the velocity command for each obstacle
    Eigen::MatrixXf mat_norm_velocities( number_states, number_obstacles);
    mat_norm_velocities = mat_velocities.colwise().normalized();

    // Relative weights of the obstacles depending on their distance from the robot
    Eigen::MatrixXf mat_weights(1, number_obstacles); // "1 x number_of_obstacles"
    mat_weights = weights_special( state_robot, mat_gamma, method_weights, limit_dist);
    //std::cout << "Gammas : " << mat_gamma   << std::endl;
    //std::cout << "Weights: " << mat_weights << std::endl;

    // Special case if all obstacles are too far away from the robot so none of them is considered
    if ((mat_weights.size()==0) || (mat_weights.maxCoeff() == 0))
    {
        // No obstacle in range so the robot just goes toward the attractor
        State cmd_velocity = state_attractor - state_robot;
        cmd_velocity(2,0) = std::atan2(cmd_velocity(1,0),cmd_velocity(0,0)) - state_robot(2,0); // angle difference used for angular speed control (gain of 1)

        // Decrease speed when the robot get close to the attractor
        float distance_stop = 0.25;
        float distance_start_decreasing = 1 / size_of_cells; // numerical value is in meters, converted into the cell world
        float distance_to_attractor = std::sqrt(std::pow(state_robot(0,0)-state_attractor(0,0),2)+std::pow(state_robot(1,0)-state_attractor(1,0),2)); // Euclidian distance
        //std::cout << "Distance to attractor:   " << distance_to_attractor << std::endl;
        cmd_velocity.block(0,0,3,1) *= std::min(std::max(static_cast<float>(0.0),distance_to_attractor-distance_stop)/(distance_start_decreasing-distance_stop), static_cast<float>(1.0));

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
    //std::cout << "mat_weights:    " << mat_weights << std::endl;

    if (verbose)
    {
        std::cout << "mat_velocities:      " << std::endl << mat_velocities << std::endl;
        std::cout << "mat_norm_velocities: " << std::endl << mat_norm_velocities << std::endl;
        std::cout << "mat_weights:    " << mat_weights << std::endl;
        std::cout << "weighted_mag:   " << weighted_mag << std::endl;
        std::cout << "weighted_direction:   " << weighted_direction << std::endl;
        std::cout << "Min Gamma: " << mat_gamma.minCoeff() << std::endl;
        std::cout << "Limit: " << limit_dist << std::endl;

    }

    // Trying a new way to apply the effect of obstacles (smoother transition from no obstacle in range to 1 obstacle in range)
    /*State cmd_to_attractor = state_attractor - state_robot;
    cmd_to_attractor(2,0) = std::atan2(cmd_to_attractor(1,0),cmd_to_attractor(0,0)) - state_robot(2,0); // angle difference used for angular speed control (gain of 1)
    float coeff = (mat_gamma.minCoeff() - limit_dist)/(1 - limit_dist);
    cmd_velocity = (1 - coeff) * cmd_to_attractor + coeff * cmd_velocity;
    std::cout << "Coefficient: " << coeff << std::endl;
   */

    // Decrease speed when the robot get close to the attractor
    float distance_stop = 0.25;
    float distance_start_decreasing = 1 / size_of_cells; // numerical value is in meters, converted into the cell world
    float distance_to_attractor = std::sqrt(std::pow(state_robot(0,0)-state_attractor(0,0),2)+std::pow(state_robot(1,0)-state_attractor(1,0),2)); // Euclidian distance
    //std::cout << "Distance to attractor:   " << distance_to_attractor << std::endl;
    cmd_velocity.block(0,0,2,1) *= std::min(std::max(static_cast<float>(0.0),distance_to_attractor-distance_stop)/(distance_start_decreasing-distance_stop), static_cast<float>(1.0));

    return speed_limiter(cmd_velocity);
}

Eigen::Matrix<float, 4, 1> next_step_special(State const& state_robot, State const& state_attractor, Border const& border)
{   // compute the next step with the special method for a single obstacle
    // output is velocity_command stacked with gamma

    // Initialization
    Eigen::MatrixXf closest(1,6);
    Eigen::MatrixXf closest_attractor(1,6);
    Eigen::Matrix<float, 6, 1> gamma_norm_proj;
    Eigen::Matrix<float, 6, 1> gamma_norm_proj_attractor;
    State robot_vec;
    State normal_vec;

    State f_eps;
    float lamb_r = 0;
    float lamb_e = 0;
    Eigen::Matrix<float, number_states, number_states> D_eps;
    State r_eps_vector;
    State gradient_vector;
    Eigen::Matrix<float, number_states, number_states-1> ortho_basis;
    Eigen::Matrix<float, number_states, number_states> E_eps;
    Eigen::Matrix<float, number_states, number_states> M_eps;

    Eigen::Matrix2f circle_tranform; // from circle frame to standard frame, rotation matrix with minus the angle of the reference vector in circle shape
    Eigen::Matrix2f shape_tranform;  // from standard frame to shape frame, rotation matrix with the angle of the normal vector in shape space

    State velocity_circle_space;
    State velocity_shape_space;

    Eigen::Matrix<float, 1, 2> robot = state_robot.block(0,0,2,1).transpose();
    Eigen::Matrix<float, 1, 2> attractor = state_attractor.block(0,0,2,1).transpose();

    /* TEST - NOT USED
    // Compute all the steps for a single step and a single obstacle
    //for (int iter=0; iter< borders.size(); iter++)
    // {
       // Compute attractor function
        if (iter==0)
        {
            f_eps = attractor_circle_frame - robot_circle_frame;
        }
        else
        {
            // Rotation from the first axis to the new circle frame
            Eigen::Matrix2f first_to_circle_tranform;
            first_to_circle_tranform <<   ref_vec_circle_frame(0,0), (-1) * ref_vec_circle_frame(1,0),
                                          ref_vec_circle_frame(1,0),        ref_vec_circle_frame(0,0);

            // Retrieve velocity of previous obstacle, align it with first axis then rotate it in the new circle frame
            f_eps.block(0,0,2,1) = first_to_circle_tranform * circle_tranform * velocity_circle_space.block(0,0,2,1);
        }

     //}*/
    //Border border = borders[iter];

    // Get closest cell for the robot
    closest = find_closest_point(robot, border);
    //std::cout << robot << " and " << closest << std::endl;

    // Get gamma distance + normal vector + point projected on border for the robot
    gamma_norm_proj = gamma_normal_projection( robot, closest);

    // Get closest cell for the attractor
    closest_attractor = find_closest_point(attractor, border);
    //std::cout << robot << " and " << closest << std::endl;

    // Get gamma distance + normal vector + point projected on border for the attractor
    gamma_norm_proj_attractor = gamma_normal_projection( attractor, closest_attractor);

    // To return NaN for points inside obstacle
    robot_vec << (robot(0,0)-gamma_norm_proj(4,0)),(robot(0,1)-gamma_norm_proj(5,0)),0;
    normal_vec << gamma_norm_proj(1,0),gamma_norm_proj(2,0),0;
    if (verbose)
    {
        std::cout << "Closest         " << closest << std::endl;
        std::cout << "Projected robot " << gamma_norm_proj(4,0) << " " << gamma_norm_proj(5,0) << std::endl;
        std::cout << "Robot_vec       " << robot_vec.transpose() << std::endl;
        std::cout << "Normal_vec      " << normal_vec.transpose()  << std::endl;
        std::cout << "Dot product     " << normal_vec.dot(robot_vec.colwise().normalized()) << std::endl;
        std::cout << "Projected attractor " << gamma_norm_proj_attractor(4,0) << " " << gamma_norm_proj_attractor(5,0) << std::endl;
    }


    float speed_reverse = 0.5; // speed to get out of an obstacle

    if (((closest(0,2)==1)||(closest(0,2)==2))&&(normal_vec.dot(robot_vec.colwise().normalized()) < 0)) // if true it means the robot is inside obstacle
    {
        // If inside an obstacle, avoid display by returning NaN
        //Eigen::Matrix<float, 4, 1>  output; output << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), 0, std::numeric_limits<float>::quiet_NaN();

        // If inside an obstacle, velocity command is normal to the surface
        Eigen::Matrix<float, 4, 1>  output;
        float norm_vec = std::sqrt(std::pow(robot_vec(0,0),2) + std::pow(robot_vec(1,0),2));
        output << (- speed_reverse * robot_vec(0,0) / norm_vec), (- speed_reverse * robot_vec(1,0) / norm_vec), 0, 1; // [v_along_x, v_along_y, v_rotation, gamma_distance]
        my_circle_space << 1000 << "," << 1000 << "\n";   // write position of the point in the circle space (for matplotlib)
        return output;
    }
    else if ((closest(0,2)==3)&&(normal_vec.dot(robot_vec.colwise().normalized()) > 0)) // if true it means the robot is inside obstacle
    {
        // If inside an obstacle, avoid display by returning NaN
        //Eigen::Matrix<float, 4, 1>  output; output << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), 0, std::numeric_limits<float>::quiet_NaN();

        // If inside an obstacle, velocity command is normal to the surface
        Eigen::Matrix<float, 4, 1>  output;
        float norm_vec = std::sqrt(std::pow(robot_vec(0,0),2) + std::pow(robot_vec(1,0),2));
        output << (- speed_reverse * robot_vec(0,0) / norm_vec), (- speed_reverse * robot_vec(1,0) / norm_vec), 0, 1;
        my_circle_space << 1000 << "," << 1000 << "\n";   // write position of the point in the circle space (for matplotlib)
        return output;
    }

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
    float gamma_circle_frame = point_circle_space(0,0);
    State robot_circle_frame = point_circle_space.block(1,0,3,1);
    State attractor_circle_frame = point_circle_space.block(4,0,3,1);
    State ref_vec_circle_frame = point_circle_space.block(7,0,3,1);

    my_circle_space << robot_circle_frame(0,0) << "," << robot_circle_frame(1,0) << "\n"; // write position of the point in the circle space (for matplotlib)


    if (closest(0,2)==3)
    {
        ref_vec_circle_frame = (-1) * ref_vec_circle_frame;
    }

    // Compute attractor function
    f_eps = (attractor_circle_frame - robot_circle_frame);

    // Several steps to get D(epsilon) matrix
    lamb_r = 1 - (1/gamma_circle_frame);
    lamb_e = 1 + (1/gamma_circle_frame);
    D_eps = D_epsilon( lamb_r, lamb_e);

    // Several steps to get E(epsilon) matrix
    Obstacle obs;
    obs << 0, 0, 0, 1, 1, 1, 1, 0, 0, 0; // unit circle [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]
    r_eps_vector = ref_vec_circle_frame;
    gradient_vector = gradient( robot_circle_frame, obs);
    ortho_basis = gram_schmidt( gradient_vector);
    E_eps = E_epsilon( r_eps_vector, ortho_basis);

    // Compute M(epsilon)
    M_eps = M_epsilon( D_eps, E_eps);

    // Compute epsilon_dot
    velocity_circle_space = epsilon_dot( M_eps, f_eps, robot_circle_frame, obs);

    // Transform the velocity_command back into the shape-space
    circle_tranform <<   ref_vec_circle_frame(0,0), ref_vec_circle_frame(1,0),
                  (-1) * ref_vec_circle_frame(1,0), ref_vec_circle_frame(0,0);

    shape_tranform <<    gamma_norm_proj(1,0), (-1) * gamma_norm_proj(2,0),
                         gamma_norm_proj(2,0),        gamma_norm_proj(1,0);


    // Apply the two rotation matrices to get back to the shape-space
    velocity_shape_space.block(0,0,2,1) = shape_tranform * circle_tranform * velocity_circle_space.block(0,0,2,1);

    // Remove tail effect (see the paper for a clean explanation)
    /*State tail_vector = (attractor_circle_frame - robot_circle_frame);
    if (r_eps_vector.dot(tail_vector.colwise().normalized()) > 0.9) // can be between 0 and 1, I chose > 0 because with thin ellipse I had some issues
    {
        velocity_shape_space.block(0,0,2,1) = shape_tranform * circle_tranform * f_eps.block(0,0,2,1);
    }*/

    if (verbose)
    {
        std::cout << "Distance tot:             " << distances_surface(0,0) << " | Distance min: " << distances_surface(0,1) << std::endl;
        std::cout << "Gamma dist robot:         " << gamma_norm_proj(0,0) << std::endl;
        std::cout << "Normal vec robot:         " << gamma_norm_proj.block(1,0,2,1).transpose() << std::endl;
        std::cout << "Max gamma dist robot:     " << max_gamma_robot << std::endl;

        std::cout << "Gamma dist attractor:     " << gamma_norm_proj_attractor(0,0) << std::endl;
        std::cout << "Max gamma dist attractor: " << max_gamma_attractor << std::endl;

        std::cout << "Robot circle frame:       " << robot_circle_frame.transpose() << std::endl;
        std::cout << "Attractor circle frame:   " << attractor_circle_frame.transpose() << std::endl;
        std::cout << "Ref vec circle frame:     " << ref_vec_circle_frame.transpose() << std::endl;
        std::cout << "Feps circle frame:        " << f_eps.transpose() << std::endl;
        std::cout << "Gamma robot circle:       " << gamma_circle_frame << std::endl;

        std::cout << "Vel circle frame:         " << velocity_circle_space.block(0,0,2,1).transpose() << std::endl;

        std::cout << "Circle transform:         Shape transform: " << std::endl;
        std::cout << circle_tranform.row(0) << "        " << shape_tranform.row(0) << std::endl;
        std::cout << circle_tranform.row(1) << "        " << shape_tranform.row(1) << std::endl;
        std::cout << "Vel shape frame:          "  << velocity_shape_space.block(0,0,2,1).transpose() << std::endl;
    }

    //Eigen::Matrix<float, 1, 1> norm = velocity_shape_space.colwise().norm();
    //velocity_shape_space = velocity_shape_space / std::sqrt(std::pow(velocity_shape_space(0,0),2)+std::pow(velocity_shape_space(1,0),2));

    Eigen::Matrix<float, 4, 1> output;
    output.block(0,0,3,1) = velocity_shape_space;
    output(3,0) = gamma_norm_proj(0,0);
    return output;
}

Eigen::Matrix<float, 6, 1> gamma_normal_projection(Eigen::Matrix<float,1,2> const& robot, Eigen::Matrix<float, 1, 6> const& data_closest)
{
    //returns [gamma, normal vector, projected point on border]

    Eigen::Matrix<float, 6, 1> output; // output is transpose of [gamma, x, y, phi]

    float angle = 0;
    if (data_closest(0,2)==1)
    {
        output.block(1,0,2,1) = data_closest.block(0,3,1,2).transpose(); // assign x,y
    }
    else if ((data_closest(0,2)==2)||(data_closest(0,2)==3))
    {
        switch (static_cast<int>(data_closest(0,4)))
        {
        case 0: angle = std::atan2(robot(0,1)-(data_closest(0,1)-0.5* size_cell), robot(0,0)-(data_closest(0,0)-0.5* size_cell)); break;
        case 1: angle = std::atan2(robot(0,1)-(data_closest(0,1)-0.5* size_cell), robot(0,0)-(data_closest(0,0)+0.5* size_cell)); break;
        case 2: angle = std::atan2(robot(0,1)-(data_closest(0,1)+0.5* size_cell), robot(0,0)-(data_closest(0,0)+0.5* size_cell)); break;
        case 3: angle = std::atan2(robot(0,1)-(data_closest(0,1)+0.5* size_cell), robot(0,0)-(data_closest(0,0)-0.5* size_cell)); break;
        }
        output.block(1,0,2,1) << std::cos(angle), std::sin(angle); // assign x,y
        if (verbose)
        {
            std::cout << "Angle of normal is: " << angle << std::endl;
        }
    }
    else
    {
        throw std::invalid_argument("Should not happen. Closest border cell has no type." );
    }
    output(3,0) = 0; // assign phi

    Eigen::Matrix<float, 1, 2> projected;

    projected = get_projection_on_border( robot, data_closest, angle);
    output(4,0) = projected(0,0); // x of projected point on the border
    output(5,0) = projected(0,1); // y of projected point on the border

    // Checking if the robot is inside the obstacle (it can cross the boundary due to discretization)
    State normal_vec;
    switch (static_cast<int>(data_closest(0,2)))
    {
    case 1: normal_vec << data_closest(0,3), data_closest(0,4), 0; break;
    case 2: normal_vec << output(1,0), output(2,0), 0; break;
    case 3: normal_vec << output(1,0), output(2,0), 0; break;
    }

    State robot_vec;
    robot_vec << (robot(0,0)-projected(0,0)),(robot(0,1)-projected(0,1)),0;
    if (((data_closest(0,2)==1)||(data_closest(0,2)==2))&&(normal_vec.dot(robot_vec.colwise().normalized()) < 0))
    {
        // if true it means the robot is inside obstacle
        output(0,0) = 1; // we set gamma to 1 to trick the controller into thinking the robot is on the border
        return output;

    }
    else if ((data_closest(0,2)==3)&&(normal_vec.dot(robot_vec.colwise().normalized()) > 0)) // if true it means the robot is inside obstacle
    {
        output(0,0) = 1; // we set gamma to 1 to trick the controller into thinking the robot is on the border
        return output;
    }
    // End checking if robot is inside the obstacle

    //std::cout << "Projected: " << projected << std::endl;

    output(0,0) = 1 + std::pow(robot(0,0)-projected(0,0),2) + std::pow(robot(0,1)-projected(0,1),2); // gamma function that is used is 1 + euclidian_distance^2
    // TODO: Call a general function that compute the gamma function

    //std::cout << "Gamma: " << output(0,0) << std::endl;
    return output;
}

Eigen::Matrix<float, 1, 3> get_distances_surface(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float, 1, 2> const& proj_attractor, Border const& border)
{   // compute the minimum distance along surface (between 0 and half of the perimeter)

    Eigen::Matrix<float, 1, 3> distances_out;

    float distance_tot = 0;

    Eigen::Matrix<int, 2, 1> cell_proj_robot     = get_cell( proj_robot(0,0), proj_robot(0,1), size_cell);   // projection of robot on surface
    Eigen::Matrix<int, 2, 1> cell_proj_attractor = get_cell( proj_attractor(0,0), proj_attractor(0,1), size_cell);  // projection of attractor on surface

    int i_proj_robot = 0;
    int i_proj_attractor = 0;

    for (int i=0;i<border.rows(); i++)
    {
        switch (static_cast<int>(border(i,2))) {
        case 1: distance_tot += size_cell; break;
        case 2: distance_tot += 1.57075 * margin; break; // pi/2 * radius for the length of a quarter of a circle
        case 3: distance_tot += 1.57075 * (size_cell-margin); break;
        }

        if ((cell_proj_robot(0,0)==border(i,0)) && (cell_proj_robot(1,0)==border(i,1)))
        {
            i_proj_robot = i; // the projection of the robot is inside cell i
        }

        if ((cell_proj_attractor(0,0)==border(i,0)) && (cell_proj_attractor(1,0)==border(i,1)))
        {
            i_proj_attractor = i; // the projection of the attractor is inside cell i
        }

    }

    distances_out(0,0) = distance_tot; // distance_tot = perimeter of the border

    float distance_up = 0; // distance following the border in the counter-clockwise direction
    float distance_down = 0; // distance following the border in the clockwise direction

    if (i_proj_robot!=i_proj_attractor)
    {
        int i_up = i_proj_robot+1;
        if (i_up == border.rows()) {i_up = 0;}
        while (i_up != i_proj_attractor)
        {
            // LOOP, start at cell proj robot till cell proj attractor in up direction (to the bottom of the list, counter-clockwise)
            switch (static_cast<int>(border(i_up,2))) {
            case 1: distance_up += size_cell; break;
            case 2: distance_up += 1.57075 * margin; break; // pi/2 * radius for the length of a quarter of a circle
            case 3: distance_up += 1.57075 * (size_cell-margin); break;
            }

            i_up += 1;
            if (i_up == border.rows()) {i_up = 0;}
        }

        int i_down = i_proj_robot-1;
        if (i_down == (-1)) {i_down = border.rows()-1;}
        while (i_down != i_proj_attractor)
        {
            // LOOP, start at cell proj robot till cell proj attractor in down direction (to the top of the list, clockwise)
            switch (static_cast<int>(border(i_down,2))) {
            case 1: distance_down += size_cell; break;
            case 2: distance_down += 1.57075 * margin; break; // pi/2 * radius for the length of a quarter of a circle
            case 3: distance_down += 1.57075 * (size_cell-margin); break;
            }

            i_down -= 1;
            if (i_down == (-1)) {i_down = border.rows()-1;}
        }
    }
    if (false) {
    std::cout << "Passed position of robot:     " << proj_robot << std::endl;
    std::cout << "Passed position of attractor: " << proj_attractor << std::endl;
    std::cout << "Distance up before start/stop cells: " << distance_up << std::endl;
    std::cout << "Distance down before start/stop cells: " << distance_down << std::endl; }

    // Now we need to add to distance_up and distance_down the small distance that exists from the projected points to the border of their cell
    // for both directions
    // For instance if the projected point is something like that "----|----|-*--|----|----|" with "|----|" a cell and "*" the projected point,
    // then we need to add the distance "-" for the direction that goes left and "--" for the direction that goes right


    float add_up_robot = 0;
    float add_down_robot = 0;
    float add_up_attractor = 0;
    float add_down_attractor = 0;

    // For cell_proj_robot
    float angle = 0;
    int sign = 1;
    switch (static_cast<int>(border(i_proj_robot,2)))
    {
    case 1: // Straight line

        if (border(i_proj_robot,3)==0)
        {
            float direction = border(i_proj_robot,4);
            add_up_robot   =              (proj_robot(0,0) - cell_proj_robot(0,0))*direction + 0.5 * size_cell;
            add_down_robot = size_cell - ((proj_robot(0,0) - cell_proj_robot(0,0))*direction + 0.5 * size_cell);
        }
        else
        {
            float direction = border(i_proj_robot,3);
            add_down_robot =              (proj_robot(0,1) - cell_proj_robot(1,0))*direction + 0.5 * size_cell;
            add_up_robot   = size_cell - ((proj_robot(0,1) - cell_proj_robot(1,0))*direction + 0.5 * size_cell);
            //std::cout << direction << " " << add_up_robot << " " << add_down_robot << std::endl;
        }
        break;
    case 2: // Outward arc of circle
        switch (static_cast<int>(border(i_proj_robot,4)))
        {
        case 0:
            angle = std::atan2(proj_robot(0,1)-(cell_proj_robot(1,0)-0.5* size_cell), proj_robot(0,0)-(cell_proj_robot(0,0)-0.5* size_cell));
            break;
        case 1:
            angle = std::atan2(proj_robot(0,1)-(cell_proj_robot(1,0)-0.5* size_cell), proj_robot(0,0)-(cell_proj_robot(0,0)+0.5* size_cell))- (3.1415*0.5);
            break;
        case 2:
            angle = std::abs(std::atan2(proj_robot(0,1)-(cell_proj_robot(1,0)+0.5* size_cell), proj_robot(0,0)-(cell_proj_robot(0,0)+0.5* size_cell)))- (3.1415*0.5);
            sign = -1;
            break;
        case 3:
            angle = std::abs(std::atan2(proj_robot(0,1)-(cell_proj_robot(1,0)+0.5* size_cell), proj_robot(0,0)-(cell_proj_robot(0,0)-0.5* size_cell)));
            sign = -1;
            break;
        }
        switch (sign)
        {
        case 1:
            add_up_robot = (1.57075 - angle) * margin; // angle * radius for the length of an arc of circle
            add_down_robot = angle * margin;
            break;
        case -1:
            add_down_robot = (1.57075 - angle) * margin; // angle * radius for the length of an arc of circle
            add_up_robot = angle * margin;
            break;
        }
        if (false)
        {
            std::cout << "Robot Type 2" << std::endl;
            std::cout << "Angle of normal is: " << angle << std::endl;
            std::cout << "Add up robot is:   " << add_up_robot << std::endl;
            std::cout << "Add down robot is: " << add_down_robot << std::endl;
        }
        break;
    case 3: // Inward arc of circle
        switch (static_cast<int>(border(i_proj_robot,4)))
        {
        case 0:
            angle = std::atan2(proj_robot(0,1)-(cell_proj_robot(1,0)-0.5* size_cell), proj_robot(0,0)-(cell_proj_robot(0,0)-0.5* size_cell));
            break;
        case 1:
            angle = std::atan2(proj_robot(0,1)-(cell_proj_robot(1,0)-0.5* size_cell), proj_robot(0,0)-(cell_proj_robot(0,0)+0.5* size_cell))- (3.1415*0.5);
            break;
        case 2:
            angle = std::abs(std::atan2(proj_robot(0,1)-(cell_proj_robot(1,0)+0.5* size_cell), proj_robot(0,0)-(cell_proj_robot(0,0)+0.5* size_cell)))- (3.1415*0.5);
            sign = -1;
            break;
        case 3:
            angle = std::abs(std::atan2(proj_robot(0,1)-(cell_proj_robot(1,0)+0.5* size_cell), proj_robot(0,0)-(cell_proj_robot(0,0)-0.5* size_cell)));
            sign = -1;
            break;
        }
        switch (sign)
        {
        case 1:
            add_down_robot = (1.57075 - angle) * (size_cell - margin); // angle * radius for the length of an arc of circle
            add_up_robot = angle * (size_cell - margin);
            break;
        case -1:
            add_up_robot = (1.57075 - angle) * (size_cell - margin); // angle * radius for the length of an arc of circle
            add_down_robot = angle * (size_cell - margin);
            break;
        }
        break;
    }
    distance_up += add_up_robot;
    distance_down += add_down_robot;

    /*std::cout << "Robot add up " << add_up_robot << std::endl;
    std::cout << "Temp dist up " << distance_up << std::endl;*/

    // For cell_proj_attractor (same thing than for cell_proj_robot but up and down are switched)
    angle = 0;
    sign = 1;
    switch (static_cast<int>(border(i_proj_attractor,2)))
    {
    case 1:
        if (border(i_proj_attractor,3)==0)
        {
            float direction = border(i_proj_attractor,4);
            add_down_attractor =              (proj_attractor(0,0) - cell_proj_attractor(0,0))*direction + 0.5 * size_cell;
            add_up_attractor   = size_cell - ((proj_attractor(0,0) - cell_proj_attractor(0,0))*direction + 0.5 * size_cell);
        }
        else
        {
            float direction = border(i_proj_attractor,3);
            add_up_attractor   =              (proj_attractor(0,1) - cell_proj_attractor(1,0))*direction + 0.5 * size_cell;
            add_down_attractor = size_cell - ((proj_attractor(0,1) - cell_proj_attractor(1,0))*direction + 0.5 * size_cell);
        }
        break;

    case 2:
        switch (static_cast<int>(border(i_proj_attractor,4)))
        {
        case 0:
            angle = std::atan2(proj_attractor(0,1)-(cell_proj_attractor(1,0)-0.5* size_cell), proj_attractor(0,0)-(cell_proj_attractor(0,0)-0.5* size_cell));
            break;
        case 1:
            angle = std::atan2(proj_attractor(0,1)-(cell_proj_attractor(1,0)-0.5* size_cell), proj_attractor(0,0)-(cell_proj_attractor(0,0)+0.5* size_cell))- (3.1415*0.5);
            break;
        case 2:
            angle = std::abs(std::atan2(proj_attractor(0,1)-(cell_proj_attractor(1,0)+0.5* size_cell), proj_attractor(0,0)-(cell_proj_attractor(0,0)+0.5* size_cell)))- (3.1415*0.5);
            sign = -1;
            break;
        case 3:
            angle = std::abs(std::atan2(proj_attractor(0,1)-(cell_proj_attractor(1,0)+0.5* size_cell), proj_attractor(0,0)-(cell_proj_attractor(0,0)-0.5* size_cell)));
            sign = -1;
            break;
        }
        switch (sign)
        {
        case 1:
            add_down_attractor = (1.57075 - angle) * margin; // angle * radius for the length of an arc of circle
            add_up_attractor = angle * margin;
            break;
        case -1:
            add_up_attractor = (1.57075 - angle) * margin; // angle * radius for the length of an arc of circle
            add_down_attractor = angle * margin;
            break;
        }
        if (false)
        {
            std::cout << "Attractor Type 2" << std::endl;
            std::cout << "Angle of normal is: " << angle << std::endl;
            std::cout << "Add up attractor is:   " << add_up_attractor << std::endl;
            std::cout << "Add down attractor is: " << add_down_attractor << std::endl;
        }
        break;
    case 3:
        switch (static_cast<int>(border(i_proj_attractor,4)))
        {
        case 0:
            angle = std::atan2(proj_attractor(0,1)-(cell_proj_attractor(1,0)-0.5* size_cell), proj_attractor(0,0)-(cell_proj_attractor(0,0)-0.5* size_cell));
            break;
        case 1:
            angle = std::atan2(proj_attractor(0,1)-(cell_proj_attractor(1,0)-0.5* size_cell), proj_attractor(0,0)-(cell_proj_attractor(0,0)+0.5* size_cell))- (3.1415*0.5);
            break;
        case 2:
            angle = std::abs(std::atan2(proj_attractor(0,1)-(cell_proj_attractor(1,0)+0.5* size_cell), proj_attractor(0,0)-(cell_proj_attractor(0,0)+0.5* size_cell)))- (3.1415*0.5);
            sign = -1;
            break;
        case 3:
            angle = std::abs(std::atan2(proj_attractor(0,1)-(cell_proj_attractor(1,0)+0.5* size_cell), proj_attractor(0,0)-(cell_proj_attractor(0,0)-0.5* size_cell)));
            sign = -1;
            break;
        }
        switch (sign)
        {
        case 1:
            add_up_attractor = (1.57075 - angle) * (size_cell - margin); // angle * radius for the length of an arc of circle
            add_down_attractor = angle * (size_cell - margin);
            break;
        case -1:
            add_down_attractor = (1.57075 - angle) * (size_cell - margin); // angle * radius for the length of an arc of circle
            add_up_attractor = angle * (size_cell - margin);
            break;
        }
        if (false)
        {
            std::cout << "Attractor Type 3" << std::endl;
            std::cout << "Angle of normal is: " << angle << std::endl;
            std::cout << "Add up attractor is:   " << add_up_attractor << std::endl;
            std::cout << "Add down attractor is: " << add_down_attractor << std::endl;
        }
        break;
    }
    distance_up += add_up_attractor;
    distance_down += add_down_attractor;

    /*std::cout << "Attractor add up " << add_up_attractor << std::endl;
    std::cout << "Temp dist up " << distance_up << std::endl;

    std::cout << "i_rob & i_attrac " << i_proj_robot << " " << i_proj_attractor << std::endl;*/
    if (i_proj_robot==i_proj_attractor)
    {
        if (verbose) {
                std::cout << "Same cell for robot and attractor" << std::endl;
                std::cout << add_up_robot << " | " << add_down_robot << " | " << add_up_attractor << " | " << add_down_attractor << std::endl;
        }

        if (add_down_robot < add_up_attractor) // if the robot is on the "left" of the attractor
        {
            distance_up = add_up_robot - add_down_attractor;
            distance_down = distance_tot - distance_up;
        }
        else
        {
            distance_down = add_down_robot - add_up_attractor;
            distance_up = distance_tot - distance_down;
        }
    }

    if (false) {
    std::cout << "Distance up before output: " << distance_up << std::endl;
    std::cout << "Distance down before output: " << distance_down << std::endl;}

    distances_out(0,1) = std::min(distance_up, distance_down);
    if (distance_up < distance_down)
    {
        distances_out(0,2) = -1; // counter-clockwise direction (convention -1)
    }
    else
    {
        distances_out(0,2) = 1; //         clockwise direction  (convention  1)
    }
    return distances_out;
}

float get_max_gamma_distance(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float,1,2> const& normal, Border const& border) // maximum gamma distance that can be reached following the normal
{
    //std::cout << "Projected point: " << proj_robot << std::endl;
    //std::cout << "Normal vector: " << normal << std::endl;

    float current_step = 0.5 * size_cell;
    float min_step = 0.01; // minimum size of a step
    float max_step = 100;  // maximum size of a step

    Eigen::Matrix<float, 1, 2> current_point; current_point = proj_robot;
    Eigen::Matrix<float, 1, 2> next_point;
    Eigen::Matrix<int, 2, 1> starting_cell = get_cell( proj_robot(0,0), proj_robot(0,1), size_cell);
    Eigen::MatrixXf next_cell(1,6);

    while ((current_step>min_step) && (current_step<max_step))
    {
        //std::cout << "Current point: " << current_point << " | Current step: " << current_step << std::endl;
        next_point << current_point(0,0) + current_step * normal(0,0), current_point(0,1) + current_step * normal(0,1);
        next_cell = find_closest_point(next_point, border);
        if ((next_cell(0,0)==starting_cell(0,0))&&(next_cell(0,1)==starting_cell(1,0)))
        {
            current_point = next_point;
            current_step *= 2;
        }
        else
        {
            current_step *= 0.5;
        }
    }

    // . . . . . . . . .
    // . x . . . . x . .
    // . x 2 . . . x . .
    // . x . . . . x . .
    // 1 x . . . . x . .
    // . x . . . . x . .
    // . x x x x x x . .
    // . . . . . . . . .
    // we start from the projection of the robot on the surface and we follow the normal vector to the surface
    // we go further and further from the surface till we reached a upper bound or a lower bound for the step
    // if we reach the upper bound it means there is no limit in that direction, if we start from cell 1 we can go left to -infinity
    // if we reach the lower bound it means there is a limit in that direction, if we start from cell 2, going right, the point is assigned to another cell when
    // it crosses the axis of symmetry of the U

    // if (current_step > max_step) it means that the maximum gamma distance is likely infinity (approximation)
    // if (current_step < min_step) it means that the current point has reached a limit before switching to another closest cell
    // for instance if the current point is within a U-shaped object this limit is the vertical axis that cuts the U into two parts

    return (1 + std::pow(current_point(0,0)-proj_robot(0,0),2) + std::pow(current_point(0,1)-proj_robot(0,1),2));
    // TODO: Call a more general function that will compute gamma
}

Eigen::Matrix<float, 10, 1> get_point_circle_frame( float const& distance_proj, float const& distance_tot, float const& gamma_robot, float const& gamma_max_robot, float const& gamma_attractor, float const& gamma_max_attractor, int direction)
{
    // returns [gamma_circle_frame, robot_circle_frame, attractor_circle_frame, ref_vec_circle_frame] of dim [1, 3, 3, 3]

    // get lots of characteristic of the robot in the circle space depending of its position in the initial space

   // 1 is clockwise direction and -1 is counter-clockwise
   // so 1 is top half part of the circle and -1 is the bottom half part of the circle
   float ratio_distance = distance_proj / (0.5 * distance_tot);

   //float gamma_circle_space_robot = (gamma_max_robot - 1)/(gamma_max_robot - gamma_robot) ;// = 1 when gamma_robot = 1, = infinity when gamma_robot = gamma_max_robot
   // it should guarantee the continuity of the transform between shape-space and circle-space.
   //float gamma_circle_space_attractor = (gamma_max_attractor - 1)/(gamma_max_attractor - gamma_attractor);


   // In theory I should use the two formulas above to ensure continuity but in practice they do not lead to good results
   float gamma_circle_space_robot = gamma_robot; // /!\ TEST
   float gamma_circle_space_attractor = gamma_attractor; // /!\ TEST
   //std::cout << gamma_attractor << std::endl;
   float angle = 3.1415 * ratio_distance * direction;

   State reference_vector; reference_vector << std::cos(angle), std::sin(angle), 0;

   // we want to find the coefficient K such as K * ref_vector has a gamma distance of gamma_circle_space
   // considering that the gamma distance here is 1 + distance_from_surface^2
   // gamma = 1 + dist^2 so dist = sqrt(gamma - 1) and the link between the distance and K is dist = K - 1 since the
   // reference vector has a norm equal to 1

   float coefficient_robot     = std::sqrt(gamma_circle_space_robot     - 1) + 1;
   float coefficient_attractor = std::sqrt(gamma_circle_space_attractor - 1) + 1;

   Eigen::Matrix<float, 10, 1> output;
   output(0,0) = gamma_circle_space_robot;
   output.block(1,0,3,1) = (coefficient_robot * reference_vector);

   output.block(7,0,3,1) = reference_vector;

   reference_vector << 1, 0, 0;
   output.block(4,0,3,1) = (coefficient_attractor * reference_vector);

   return output;
}

void fillGrid(PointFill orig, Grid & occupancy_grid) // fill the holes in the occupied blobs of an occupancy grid
{
    int width  = occupancy_grid.cols();
    int height = occupancy_grid.rows();

    Grid temp_grid = Grid::Ones(height,width) * 100; // occupancy grid which is completely occupied

    /*std::cout << " PASS 1 " << std::endl;
    std::cout << " height " << height << std::endl;
    std::cout << " width "  << width  << std::endl;*/

    std::queue<PointFill> q;
    q.push(orig);

    // The main flood loop
    while(!q.empty())
    {
        PointFill pnt = q.front();
        q.pop();

        // grab adjacent points
        PointFill adj[4];
        adj[0].x = pnt.x;   adj[0].y = pnt.y-1;   // up
        adj[1].x = pnt.x+1; adj[1].y = pnt.y;     // right
        adj[2].x = pnt.x;   adj[2].y = pnt.y+1;   // down
        adj[3].x = pnt.x-1; adj[3].y = pnt.y;     // left

        for(int i = 0; i < 4; i++)
        {
            //std::cout << "Processing (" << adj[i].x << " , " << adj[i].y << ")"; // <<  std::endl;

            // Check for boundaries
            if(adj[i].x < 0 || adj[i].x >= height ||
            adj[i].y < 0 || adj[i].y >= width)
            {
                continue;
            }

            // if adjacent point meets some criteria, then set
            // its value and include it in the queue
            // If the cell is empty in occupancy_grid it means the flood can reach it so it is not a free cell that is enclosed in an obstacle
            // If this cell is also occupied in temp_grid we set it to 0
            // As a result, temp_grid is 100 everywhere except for the free cells that are not enclosed in obstacles which are set at 0
            if((temp_grid(adj[i].x,adj[i].y)!=0) && (occupancy_grid(adj[i].x,adj[i].y)!=100))
            {
                temp_grid(adj[i].x,adj[i].y) = 0;
                //std::cout << "Pushing (" << adj[i].x << " , " << adj[i].y << ")";
                q.push(adj[i]);
            }
        }
    }
    //std::cout << std::endl;

    occupancy_grid = occupancy_grid.cwiseMax(temp_grid); // add temporary grid (occupied space)

}
