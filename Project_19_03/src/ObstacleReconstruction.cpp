#include "ObstacleReconstruction.h"
#include "ObstacleAvoidance.h"

float myRad = 0.1;
const float size_cell = 1.0;
const float margin = 0.5;

Point get_center(Blob const& blob)
{
    Point res = blob.colwise().mean();
    return res;
}

Point get_random(Blob const& blob, int const& limit)
{
    Point res;
    int i = 0;
    std::srand(42);
    if (limit==0)
    {
        i = rand() % blob.rows(); // between 0 and blob.rows - 1
    }
    else
    {
        i = rand() % limit; // between 0 and limit - 1
    }
    res(0,0) = blob(i,0);
    res(0,1) = blob(i,1);
    return res;
}

bool isPart(Blob const& blob, float const& x, float const& y)
{
    for (int i=0; i<blob.rows(); i++)
    {
        if ((x==blob(i,0)) && (y==blob(i,1)))
        {
            return true;
        }
    }
    return false;
}

bool isPartBorder(Border const& border, float const& x, float const& y)
{
    for (int i=0; i<border.rows(); i++)
    {
        if ((x==border(i,0)) && (y==border(i,1)))
        {
            return true;
        }
    }
    return false;
}

bool isPartBorderReverse(Border const& border, float const& x, float const& y)
{
    for (int i=border.rows()-1; i>0; i--)
    {
        if ((x==border(i,0)) && (y==border(i,1)))
        {
            return true;
        }
    }
    return false;
}


bool check_direction(Blob const& obstacle, Point const& point, int const& direction)
{
    // Directions are:  3 2 1
    //                  4   0
    //                  5 6 7
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
    default: throw std::invalid_argument("Direction must be an integer between 0 and 7." );
             break;
    }
    return false;
}


Blob fill_gaps(Blob const& obstacle)
{
    Eigen::Matrix<int, 1, 2> mini = obstacle.colwise().minCoeff();
    Eigen::Matrix<int, 1, 2> maxi = obstacle.colwise().maxCoeff();
    Eigen::MatrixXi grid = Eigen::MatrixXi::Zero(maxi(0,0)-mini(0,0)+1+2, maxi(0,1)-mini(0,1)+1+2);

    // Fill a grid with occupied cells
    //int counter = 0;
    Blob filled_gaps;
    for (int i=0; i<obstacle.rows(); i++)
    {
        grid(obstacle(i,0)-mini(0,0)+1,obstacle(i,1)-mini(0,1)+1) = 1;
    }

    for (int i=1; i<(grid.rows()-1); i++)
    {
        for (int j=1; j<(grid.cols()-1); j++)
        {
            if ((grid(i,j)==0) && ((grid(i-1,j)+grid(i+1,j)+grid(i,j-1)+grid(i,j+1))>=3))
            {
                grid(i,j) = 1;
                filled_gaps.conservativeResize(filled_gaps.rows()+1, Eigen::NoChange);
                filled_gaps.row(filled_gaps.rows()-1) << i-1+mini(0,0),j-1+mini(0,1);
                i -= 1; // go 1 row up
                if (i<1) {i=1;}
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

Blob fill_gaps_with_grid(Blob const& obstacle, Grid & occupancy_grid)
{
    Eigen::Matrix<int, 1, 2> mini = obstacle.colwise().minCoeff();
    Eigen::Matrix<int, 1, 2> maxi = obstacle.colwise().maxCoeff();
    Eigen::MatrixXi grid = Eigen::MatrixXi::Zero(maxi(0,0)-mini(0,0)+1+2, maxi(0,1)-mini(0,1)+1+2);

    // Fill a grid with occupied cells
    //int counter = 0;
    Blob filled_gaps;
    for (int i=0; i<obstacle.rows(); i++)
    {
        grid(obstacle(i,0)-mini(0,0)+1,obstacle(i,1)-mini(0,1)+1) = 1;
    }

    for (int i=1; i<(grid.rows()-1); i++)
    {
        for (int j=1; j<(grid.cols()-1); j++)
        {
            if ((grid(i,j)==0) && ((grid(i-1,j)+grid(i+1,j)+grid(i,j-1)+grid(i,j+1))>=3))
            {
                grid(i,j) = 1;
                occupancy_grid(i+mini(0,0)-1,j+mini(0,1)-1) = 100;
                filled_gaps.conservativeResize(filled_gaps.rows()+1, Eigen::NoChange);
                filled_gaps.row(filled_gaps.rows()-1) << i-1+mini(0,0),j-1+mini(0,1);
                i -= 1; // go 1 row up
                if (i<1) {i=1;}
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

void update_border(Border & border, Point const& position, int const& previous_direction, int const& next_direction)
{
    const int x = position(0,0);
    const int y = position(0,1);
    switch(previous_direction) {
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
    default: throw std::invalid_argument("Direction must be an integer between 0 and 7." );
             break;
    }

}

void add_to_border(Border & border, int const& x, int const& y, int const& type, float const& charac_1, float const& charac_2)
{
    if (!isPartBorderReverse( border, x, y)) // if not already part of the border
    {
        border.conservativeResize(border.rows()+1, Eigen::NoChange);
        border.row(border.rows()-1) << x, y, type, charac_1, charac_2;
    }
}

void update_position(Point & current_position, int const& direction)
{
        // Directions are:  3 2 1
    //                  4   0
    //                  5 6 7
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
    default: throw std::invalid_argument("Direction must be an integer between 0 and 7." );
             break;
    }
}

Border compute_border_and_fill(Blob const& obstacle, Point const& center, Grid & occupancy_grid)
{
    Border border;
    Blob filled_obstacle = fill_gaps_with_grid(obstacle, occupancy_grid); // Fill the gaps in the obstacle

    return compute_border( filled_obstacle, center);
}
Border compute_border(Blob const& obstacle, Point const& center)
{

    Border border;
    // Fill the gaps in the obstacle
    Blob filled_obstacle = fill_gaps(obstacle);
    // Reach the right border starting from the center
    Point target = center;
    while (isPart(filled_obstacle, target(0,0), target(0,1)))
    {
        target(0,0) += 1;
    }
    target(0,0) -= 1;
    Point start = target;
    Point current_pos = target;
    int previous_dir = 2;
    int next_dir = 7;
    // Retrieve previous direction from starting position to ensure that the border is correctly created for the first step
    bool flag_start = true;
    while (flag_start)
    {
        if (check_direction(filled_obstacle, current_pos, next_dir))
        {
            previous_dir = next_dir;
            flag_start = false;
        }
        else
        {
            next_dir -= 1;
            if (next_dir==0) {flag_start = false;}
        }
    }
    previous_dir += 4;
    if (previous_dir >= 8) {previous_dir-=8;} // I am not using std::remainder because it returns a float (could cast it to int)
    int initial_previous_dir = previous_dir; // Used later
    next_dir = 1;
    flag_start = true;
    //check_direction(obstacle, current, next_dir);
    // Follow border till we come back to the initial position
    //std::cout << "(" << current_pos(0,0) << "," << current_pos(0,1) << ")" << std::endl;

    do
    {
        if (check_direction(filled_obstacle, current_pos, next_dir))
        {
            //std::cout << "(" << current_pos(0,0) << "," << current_pos(0,1) << ") and going from " << previous_dir << " to " << next_dir << std::endl;

            // go to next
            update_border(border, current_pos, previous_dir, next_dir);
            update_position(current_pos, next_dir);
            previous_dir = next_dir;
            next_dir = previous_dir - 1;
            if (next_dir<0) {next_dir = 7;}
            flag_start = false;
        }
        else
        {
            next_dir += 1;
            if (next_dir==8) {next_dir = 0;}
        }

    } while((flag_start)|| !((current_pos(0,0)==start(0,0))&&(current_pos(0,1)==start(0,1)&&(previous_dir==initial_previous_dir))));
    flag_start = true;
    while (flag_start)
    {
        if (check_direction(filled_obstacle, current_pos, next_dir))
        {
            // go to next
            update_border(border, current_pos, previous_dir, next_dir);
            update_position(current_pos, next_dir);
            previous_dir = next_dir;
            next_dir = previous_dir - 1;
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

    remove_end_duplicate(border);

    return border;
}

void display_border(Blob const& obstacle, Border const& border)
{
    Eigen::Matrix<int, 1, 2> mini = obstacle.colwise().minCoeff();
    Eigen::Matrix<int, 1, 2> maxi = obstacle.colwise().maxCoeff();
    Eigen::MatrixXi grid = Eigen::MatrixXi::Zero(maxi(0,0)-mini(0,0)+1+2, maxi(0,1)-mini(0,1)+1+2);

    std::string display_storage[maxi(0,0)-mini(0,0)+1+2][maxi(0,1)-mini(0,1)+1+2];
    for (int i=0; i<grid.rows();i++)
    {
        for (int j=0; j<grid.cols();j++)
        {
            display_storage[i][j] = ".";

        }
    }


    for (int i=0; i<obstacle.rows(); i++)
    {
        grid(obstacle(i,0)-mini(0,0)+1,obstacle(i,1)-mini(0,1)+1) = 1;
        display_storage[obstacle(i,0)-mini(0,0)+1][obstacle(i,1)-mini(0,1)+1] = "\u25A1";
    }
    for (int i=0; i<border.rows(); i++)
    {
        grid(border(i,0)-mini(0,0)+1,border(i,1)-mini(0,1)+1) = 2;
        switch((int)(border(i,2))) {
            case 1: if (border(i,3)==0) {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "\u007C";}
                    else {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "\u2015";}
                    break;
            case 2: if (border(i,4)==0) {display_storage[(int)(border(i,0))-mini(0,0)+1][(int)(border(i,1))-mini(0,1)+1] = "/";}
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

void remove_end_duplicate(Border & border)
{
    bool flag_end = false;
    while (!flag_end)
    {
        int x = border(border.rows()-1,0); // load data of last row
        int y = border(border.rows()-1,1);
        for (int i=0;i<=border.rows()-2; i++)
        {
            if ((x==border(i,0))&&(y==border(i,1))) // duplicate has been detected
            {
                border.conservativeResize(border.rows()-1,border.cols()); // remove last row because it is a duplicate
                break;
            }
            if (i==border.rows()-2) {flag_end = true;} // if not duplicate has been detected we stop
        }
    }
}


Blob expand_obstacle(Blob const& obstacle, int const& n_cells)
{
    Blob expanded_obs = fill_gaps(obstacle);

    for (int i_expand=0; i_expand<n_cells; i_expand++)
    {
        Point center_blob = get_random(expanded_obs);
        Border expanded_border = compute_border( expanded_obs, center_blob);

        int N = expanded_obs.rows();
        expanded_obs.conservativeResize(N+expanded_border.rows(), Eigen::NoChange);
        for (int i_cell=0; i_cell<expanded_border.rows(); i_cell++)
        {
            expanded_obs.row(N+i_cell) << expanded_border(i_cell, 0), expanded_border(i_cell, 1); // [x,y] of the border cell
        }
    }

    Border placeholder_border;
    //display_border(expanded_obs, placeholder_border);
    return expanded_obs;
}

Grid expand_occupancy_grid(Grid const& grid, int const& n_cells)
{
    Grid occupancy_res = grid;

    for (int n=0; n<n_cells; n++)
    {
        Grid occupancy_temp = occupancy_res;
        for (int i=1; i<(grid.rows()-1); i++)
        {
           for (int j=1; j<(grid.cols()-1); j++)
           {
               if (occupancy_temp(i,j) == 100)
               {
                   occupancy_res(i+1,j) = 100;
                   occupancy_res(i-1,j) = 100;
                   occupancy_res(i,j+1) = 100;
                   occupancy_res(i,j-1) = 100;
               }
           }
        }
    }
    return occupancy_res;
}
////obstacle = expand_obstacle( obstacle, 2); // security margin of n cells

Eigen::Matrix<float, 1, 6> find_closest_point(Eigen::Matrix<float,1,2> const& robot, Border const& border)
{
    float min_distance = std::pow(robot(0,0)-border(0,0),2) + std::pow(robot(0,1)-border(0,1),2);
    int i_closest = 0;
    for (int i=1; i<border.rows(); i++)
    {
        float distance = std::pow(robot(0,0)-border(i,0),2) + std::pow(robot(0,1)-border(i,1),2);
        if (distance < min_distance)
        {
            min_distance = distance;
            i_closest = i;
        }
    }
    Eigen::Matrix<float, 1, 6> res; res << border.row(i_closest), min_distance;
    return res;
}

std::ofstream mypoints;

Eigen::Matrix<float, 4, 1> gamma_and_ref_vector(Eigen::Matrix<float,1,2>  robot, Eigen::Matrix<float, 1, 6> data_closest)
{
   Eigen::Matrix<float, 4, 1> output; // output is transpose of [gamma, x, y, phi]

   float angle = 0;
   if (data_closest(0,2)==1)
   {
       output.block(1,0,2,1) = data_closest.block(0,3,1,2).transpose(); // assign x,y
   }
   else if ((data_closest(0,2)==2)||(data_closest(0,2)==3))
   {
       switch (static_cast<int>(data_closest(0,4))) {
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
   output(3,0) = 0; // assign phi

   Eigen::Matrix<float, 1, 2> projected;
   projected = get_projection_on_border( robot, data_closest, angle);

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
   output(0,0) = 1 + std::pow(robot(0,0)-projected(0,0),2) + std::pow(robot(0,1)-projected(0,1),2); // gamma function that is used is 1 + distance^2

   return output;
}



void compute_quiver_border(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, Blob const& obstacle)
{
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
            //for (int i=0; i < number_obstacles; i++)
            //{


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

void compute_stream_border(Eigen::Matrix<float, 5, 1> const& limits, State const& state_attractor, std::vector<Blob> obstacles)
{
    // Compute the velocity command at the initial time for all points of a [x,y] grid
    // Save the result in a txt file and then use a Python script to plot the vector field (quiver) with matplotlib
    //const int number_obstacles = mat_obs.cols();
    bool flag = false;

    std::vector<Border> borders;
    std::vector<Blob> filled_obstacles;

    std::ofstream myfile;
    myfile.open("stream_data_border.txt");

    mypoints.open("stream_points_border.txt");
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
                State next_eps = next_step_special_weighted( state_robot, state_attractor, borders); // compute special velocity with transform in circle space
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

Eigen::Matrix<int, 2, 1> get_cell(float const& x, float const& y, float const& size_side_cell)
{
    float x_cell = std::floor((x+0.5) / size_side_cell);
    float y_cell = std::floor((y+0.5) / size_side_cell);
    int x_int = static_cast<int>(x_cell);
    int y_int = static_cast<int>(y_cell);
    Eigen::Matrix<int, 2, 1> out; out << x_int, y_int;
    return out;
}

State next_step_single_obstacle_border(State const& state_robot, State const& state_attractor, Border const& border)
{
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
{
    Eigen::Matrix<float, 1, 2> projected_point;
    projected_point << 0,0;

    /*if (true)
    {
        std::cout << " -- PROJECTION --" << std::endl;
        std::cout << "Robot:   " << robot << std::endl;
        std::cout << "Closest: " << data_closest << std::endl;
    }*/


    switch (static_cast<int>(data_closest(0,2)))
    {
    case 1:
        if (data_closest(0,3)!=0) // normal along X axis
        {
            projected_point(0,1) = robot(0,1);
            projected_point(0,0) = data_closest(0,0) + data_closest(0,3) * (margin - (size_cell*0.5)); // center_cell +- half_side (depending on normal direction) + margin
        }
        else // normal along Y axis
        {
            projected_point(0,0) = robot(0,0);
            projected_point(0,1) = data_closest(0,1) + data_closest(0,4) * (margin - (size_cell*0.5));  // center_cell +- half_side (depending on normal direction) + margin
        }
        break;
    case 2:
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
    case 3:
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
// In gamma_and_ref -> get the projection on the border,easy with the info of closest

void growing_obstacle()
{
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

void growing_several_obstacle()
{

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
        storage = detect_borders( occupancy_grid );

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


std::vector<Border> detect_borders( Grid & occupancy_grid )
{
    std::vector<Border> detected_borders;
    //int cursor_x = 0;
    //int cursor_y = 0;
    Point start_obstacle;
    Border border_obstacle;
    //std::cout << "== Entry==" << std::endl;

    for (int cursor_row=0; cursor_row < occupancy_grid.rows(); cursor_row++)
    {
        for (int cursor_col=0; cursor_col < occupancy_grid.cols(); cursor_col++)
        {
            //std::cout << "Cursor at point (" << cursor_row << "," << cursor_col << ")" << std::endl;
            //if ( (cursor_row==1)&&(cursor_col==1) ) { std::cout << occupancy_grid << std::endl;}
            if (occupancy_grid(cursor_row, cursor_col)==100) // if the cell is occupied -> it may be new obstacle
            {
                //if ( (cursor_row==16)&&(cursor_col==8) ) { std::cout << " PASS - " << std::endl;}

                bool flag_not_in_border = true;

                for (int i_border=0; ((i_border<detected_borders.size())&&(flag_not_in_border == true)); i_border++)
                {
                    //if ( (cursor_row==16)&&(cursor_col==8) ) { std::cout << " PASS 2 " << std::endl;}

                    // cursor_col - 1 because we scan from left to right so we hit the left side of obstacle
                    // so the cell which may be part of a border is the cell on the left of the detected cell
                    if (isPartBorder(detected_borders[i_border],cursor_row,cursor_col-1))
                    {
                        flag_not_in_border = false;
                        // set col to the other side of the obstacle by going around it
                        cursor_col = other_side_obstacle( detected_borders[i_border], cursor_row, cursor_col-1);
                    }
                }
                if (flag_not_in_border)
                {

                    //if ( (cursor_row==16)&&(cursor_col==8) ) { std::cout << " PASS 3 " << std::endl;}
                    start_obstacle << cursor_row, cursor_col;
                    Grid temp_grid = occupancy_grid;
                    Blob obstacle;
                    explore_obstacle( obstacle, temp_grid, cursor_row, cursor_col);
                    std::cout << "New obstacle!" << std::endl;
                    //if ( (cursor_row==16)&&(cursor_col==8) ) { std::cout << obstacle << std::endl;}
                    if (obstacle.rows()>1)
                    {
                        border_obstacle = compute_border_and_fill( obstacle, start_obstacle, occupancy_grid);
                        detected_borders.push_back(border_obstacle);
                        // set col to the other side of the obstacle by going around it
                        cursor_col = other_side_obstacle( border_obstacle, cursor_row, cursor_col-1);
                    }
                    /*td::cout << border_obstacle << std::endl;
                    std::cout << cursor_row<< std::endl;
                    std::cout << cursor_col << std::endl;
                    return detected_borders;*/
                }
            }
        }
    }


    return detected_borders;
    /*while ((cursor_x!=(occupancy_grid.rows()-1))&&(cursor_y!=(occupancy_grid.cols()-1)))
    {
        if (occupancy_grid(cursor_x, cursor_y)==1) // if the cell is occupied and not in a border-> new obstacle
        {
            bool flag_not_in_border = true;
            for (int i_border=0; ((i_border<detected_borders.size())&&(flag_not_in_border == true)); i_border++)
            {
                // cursor_x - 1 because we scan from left to right so we hit the left side of obstacle
                // so the cell which may be part of a border is the cell on the left of the detected cell
                if (isPartBorder(detected_borders[i_border],cursor_x-1,cursor_y))
                {
                    flag_not_in_border = false;
                    // set x to the other side of the obstacle by going around it
                    cursor_x = other_side_obstacle( detected_borders[i_border]);
                }
            }
            if (flag_not_in_border)
            {
                start_obstacle << cursor_x, cursor_y;
                border_obstacle = compute_border( obstacle, start_obstacle);
                detected_borders.push_back(border_obstacle);
                // set x to the other side of the obstacle by going around it
                cursor_x = other_side_obstacle( border_obstacle);
            }



        }

        cursor_x += 0; // could be done with two for loops instead of a while
        cursor_y += 1;
    }*/
}

std::vector<Blob> detect_blobs( Grid & occupancy_grid )
{
    std::vector<Border> detected_borders;
    std::vector<Blob> detected_blobs;
    Point start_obstacle;
    Border border_obstacle;

    for (int cursor_row=0; cursor_row < occupancy_grid.rows(); cursor_row++)
    {
        for (int cursor_col=0; cursor_col < occupancy_grid.cols(); cursor_col++)
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
                if (flag_not_in_border)
                {

                    //if ( (cursor_row==16)&&(cursor_col==8) ) { std::cout << " PASS 3 " << std::endl;}
                    start_obstacle << cursor_row, cursor_col;
                    Grid temp_grid = occupancy_grid;
                    Blob obstacle;
                    explore_obstacle( obstacle, temp_grid, cursor_row, cursor_col);
                    std::cout << "New obstacle!" << std::endl;
                    //if ( (cursor_row==16)&&(cursor_col==8) ) { std::cout << obstacle << std::endl;}
                    if (obstacle.rows()>1)
                    {
                        border_obstacle = compute_border_and_fill( obstacle, start_obstacle, occupancy_grid);
                        detected_borders.push_back(border_obstacle);
                        detected_blobs.push_back(obstacle);
                        // set col to the other side of the obstacle by going around it
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
    // in case there are several non convex obstacles that are kind of interlocked we need to get out as left as possible
    // for instance if there is a I-shaped obstacle in a U-shaped obstacle, we have to get out in the middle or the U and not on the other side
    /*if ( (row==16)&&(col==7) ) {
            std::cout << " PASS 4 " << std::endl;
            std::cout <<  border  << std::endl;
            std::cout << " Row: " << row << " Col: " << col << std::endl;
    }*/

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

void explore_obstacle( Blob & obstacle, Grid & occupancy_grid, int const& row, int const& col)
{
    obstacle.conservativeResize(obstacle.rows()+1, Eigen::NoChange);
    obstacle.row(obstacle.rows()-1) << row, col;
    occupancy_grid(row,col) = 0;

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

}

bool verbose = false;

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
{
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

State next_step_special_weighted(State const& state_robot, State const& state_attractor, std::vector<Border> const& borders)
{
    // Compute all the steps to get the velocity command considering several obstacles
    const int number_obstacles = borders.size();

    // Velocity command for each obstacle
    //Eigen::MatrixXf mat_velocities(number_states, number_obstacles); // "3 x number_of_obstacles"
    //mat_velocities = velocities(state_robot, state_attractor, mat_obs);

    // mat_velocities is a matrix with size "number_states x number_obstacles"
    Eigen::MatrixXf mat_velocities(number_states, number_obstacles); // "3 x number_of_obstacles"
    Eigen::MatrixXf mat_gamma(1, number_obstacles); // "3 x number_of_obstacles"
    for (int i=0; i < borders.size(); i++)
    {
        Eigen::Matrix<float, 4, 1> output = next_step_special(state_robot, state_attractor, borders[i]); // compute velocity command for each obstacle
        mat_velocities.col(i) = output.block(0,0,3,1);
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

    // Special case if all obstacles are too far away from the robot so none of them is considered
    if (mat_weights.maxCoeff() == 0)
    {
        // No obstacle in range so the robot just goes toward the attractor
        State cmd_velocity = state_attractor - state_robot;
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

    if (verbose)
    {
        std::cout << "mat_velocities:      " << std::endl << mat_velocities << std::endl;
        std::cout << "mat_norm_velocities: " << std::endl << mat_norm_velocities << std::endl;
        std::cout << "mat_weights:    " << mat_weights << std::endl;
        std::cout << "weighted_mag:   " << weighted_mag << std::endl;
        std::cout << "weighted_direction:   " << weighted_direction << std::endl;
    }


    return speed_limiter(cmd_velocity);
}

Eigen::Matrix<float, 4, 1> next_step_special(State const& state_robot, State const& state_attractor, Border const& border)
{

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

    /*// Compute all the steps for a single step and a single obstacle
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

        if (((closest(0,2)==1)||(closest(0,2)==2))&&(normal_vec.dot(robot_vec.colwise().normalized()) < 0))
        {
            // If inside an obstacle, avoid display by returning NaN
            //Eigen::Matrix<float, 4, 1>  output; output << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), 0, std::numeric_limits<float>::quiet_NaN();
            Eigen::Matrix<float, 4, 1>  output; output << -robot_vec(0,0), -robot_vec(1,0), 0, 1;
            return output;
        }
        else if ((closest(0,2)==3)&&(normal_vec.dot(robot_vec.colwise().normalized()) > 0)) // if true it means the robot is inside obstacle
        {
            // If inside an obstacle, avoid display by returning NaN
            //Eigen::Matrix<float, 4, 1>  output; output << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), 0, std::numeric_limits<float>::quiet_NaN();
            Eigen::Matrix<float, 4, 1>  output; output << -robot_vec(0,0), -robot_vec(1,0), 0, 1;
            return output;
        }
        ////

        // Get minimal distance along surface (either by following the surface clockwise or counter-clockwise)
        // Distance between the projected point of the robot and the projected point of the attractor
        Eigen::Matrix<float, 1, 2> proj_robot;     proj_robot     << gamma_norm_proj(4,0), gamma_norm_proj(5,0);
        Eigen::Matrix<float, 1, 2> proj_attractor; proj_attractor << gamma_norm_proj_attractor(4,0), gamma_norm_proj_attractor(5,0);

        //std::cout << "PASS" << std::endl;

        // Get information about the position of the project point on the boundary of the obstacle
        Eigen::Matrix<float, 1, 3> distances_surface = get_distances_surface(proj_robot, proj_attractor, border);


        // Get the maximum gamma distance in the initial space for the robot
        float max_gamma_robot = get_max_gamma_distance( proj_robot, gamma_norm_proj.block(1,0,2,1).transpose() , border);


        // Get the maximum gamma distance in the initial space for the attractor
        float max_gamma_attractor = get_max_gamma_distance( proj_attractor, gamma_norm_proj_attractor.block(1,0,2,1).transpose() , border);


        // Get the position of the robot in the circle space
        Eigen::Matrix<float, 10, 1> point_circle_space = get_point_circle_frame( distances_surface(0,1), distances_surface(0,0), gamma_norm_proj(0,0), max_gamma_robot, gamma_norm_proj_attractor(0,0), max_gamma_attractor, distances_surface(0,2));
        float gamma_circle_frame = point_circle_space(0,0);
        State robot_circle_frame = point_circle_space.block(1,0,3,1);
        State attractor_circle_frame = point_circle_space.block(4,0,3,1);
        State ref_vec_circle_frame = point_circle_space.block(7,0,3,1);

        if (closest(0,2)==3)
        {
            ref_vec_circle_frame = (-1) * ref_vec_circle_frame;
        }

        // Compute attractor function
        f_eps = attractor_circle_frame - robot_circle_frame;

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
    velocity_shape_space = velocity_shape_space / std::sqrt(std::pow(velocity_shape_space(0,0),2)+std::pow(velocity_shape_space(1,0),2));

    Eigen::Matrix<float, 4, 1> output;
    output.block(0,0,3,1) = velocity_shape_space;
    output(3,0) = gamma_norm_proj(0,0);
    return output;
}

Eigen::Matrix<float, 6, 1> gamma_normal_projection(Eigen::Matrix<float,1,2> const& robot, Eigen::Matrix<float, 1, 6> const& data_closest)
{
   Eigen::Matrix<float, 6, 1> output; // output is transpose of [gamma, x, y, phi]

   float angle = 0;
   if (data_closest(0,2)==1)
   {
       output.block(1,0,2,1) = data_closest.block(0,3,1,2).transpose(); // assign x,y
   }
   else if ((data_closest(0,2)==2)||(data_closest(0,2)==3))
   {
       switch (static_cast<int>(data_closest(0,4))) {
            case 0: angle = std::atan2(robot(0,1)-(data_closest(0,1)-0.5* size_cell), robot(0,0)-(data_closest(0,0)-0.5* size_cell)); break;
            case 1: angle = std::atan2(robot(0,1)-(data_closest(0,1)-0.5* size_cell), robot(0,0)-(data_closest(0,0)+0.5* size_cell)); break;
            case 2: angle = std::atan2(robot(0,1)-(data_closest(0,1)+0.5* size_cell), robot(0,0)-(data_closest(0,0)+0.5* size_cell)); break;
            case 3: angle = std::atan2(robot(0,1)-(data_closest(0,1)+0.5* size_cell), robot(0,0)-(data_closest(0,0)-0.5* size_cell)); break;
      }
        output.block(1,0,2,1) << std::cos(angle), std::sin(angle); // assign x,y
        if (verbose) {std::cout << "Angle of normal is: " << angle << std::endl;}
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
    case 1:
        normal_vec << data_closest(0,3), data_closest(0,4), 0; break;
    case 2:
        normal_vec << output(1,0), output(2,0), 0; break;
    case 3:
        normal_vec << output(1,0), output(2,0), 0; break;
    }

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

   output(0,0) = 1 + std::pow(robot(0,0)-projected(0,0),2) + std::pow(robot(0,1)-projected(0,1),2); // gamma function that is used is 1 + distance^2

   return output;
}

Eigen::Matrix<float, 1, 3> get_distances_surface(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float, 1, 2> const& proj_attractor, Border const& border)
{


    Eigen::Matrix<float, 1, 3> distances_out;

    float distance_tot = 0;

    Eigen::Matrix<int, 2, 1> cell_proj_robot     = get_cell( proj_robot(0,0), proj_robot(0,1), size_cell);
    Eigen::Matrix<int, 2, 1> cell_proj_attractor = get_cell( proj_attractor(0,0), proj_attractor(0,1), size_cell);

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
            i_proj_robot = i;
        }

        if ((cell_proj_attractor(0,0)==border(i,0)) && (cell_proj_attractor(1,0)==border(i,1)))
        {
            i_proj_attractor = i;
        }

    }

    distances_out(0,0) = distance_tot;

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
    if (verbose) {
    std::cout << "Passed position of robot:     " << proj_robot << std::endl;
    std::cout << "Passed position of attractor: " << proj_attractor << std::endl;
    std::cout << "Distance up before start/stop cells: " << distance_up << std::endl;
    std::cout << "Distance down before start/stop cells: " << distance_down << std::endl; }

    // Now we need to add to distance_up and distance_down the small distance that exists from the projected points to the border of their cell
    // for both directions
    // For instance i the projected point is something like that "----|----|-*--|----|----|" with "|----|" a cell and "*" the projected point,
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
    case 1:

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
    case 2:
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
        if (verbose)
        {
            std::cout << "Angle of normal is: " << angle << std::endl;
        }
        break;
    case 3:
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

    // For cell_proj_attractor
    sign = 1;
    switch (static_cast<int>(border(i_proj_attractor,2)))
    {
    case 1:
        if (border(i_proj_robot,3)==0)
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
        break;
    }
    distance_up += add_up_attractor;
    distance_down += add_down_attractor;

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

    if (verbose) {
    std::cout << "Distance up before output: " << distance_up << std::endl;
    std::cout << "Distance down before output: " << distance_down << std::endl;}

    distances_out(0,1) = std::min(distance_up, distance_down);
    if (distance_up < distance_down)
    {
        distances_out(0,2) = -1; // counter-clockwise direction
    }
    else
    {
        distances_out(0,2) = 1; //         clockwise direction
    }
    return distances_out;
}

float get_max_gamma_distance(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float,1,2> const& normal, Border const& border) // maximum gamma distance that can be reached following the normal
{
    //std::cout << "Projected point: " << proj_robot << std::endl;
    //std::cout << "Normal vector: " << normal << std::endl;

    float current_step = 0.5 * size_cell;
    float min_step = 0.01;
    float max_step = 100;

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

    // if (current_step > max_step) it means that the maximum gamma distance is likely infinity (approximation)
    // if (current_step < min_step) it means that the current point has reached a limit before switching to another closest cell
    // for instance if the current point is within a U-shaped object this limit is the vertical axis that cuts the U into two parts

    return (1 + std::pow(current_point(0,0)-proj_robot(0,0),2) + std::pow(current_point(0,1)-proj_robot(0,1),2));
}

Eigen::Matrix<float, 10, 1> get_point_circle_frame( float const& distance_proj, float const& distance_tot, float const& gamma_robot, float const& gamma_max_robot, float const& gamma_attractor, float const& gamma_max_attractor, int direction)
{
   // 1 is clockwise direction and -1 is counter-clockwise
   // so 1 is top half part of the circle and -1 is the bottom half part of the circle
   float ratio_distance = distance_proj / (0.5 * distance_tot);

   //float gamma_circle_space_robot = (gamma_max_robot - 1)/(gamma_max_robot - gamma_robot); // = 1 when gamma_robot = 1, = infinity when gamma_robot = gamma_max_robot
   // it should guarantee the continuity of the transform between shape-space and circle-space.
   //float gamma_circle_space_attractor = (gamma_max_attractor - 1)/(gamma_max_attractor - gamma_attractor);

   float gamma_circle_space_robot = gamma_robot; // /!\ TEST
   float gamma_circle_space_attractor = gamma_attractor; // /!\ TEST

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

