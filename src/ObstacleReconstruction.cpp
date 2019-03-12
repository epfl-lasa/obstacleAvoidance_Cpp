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
        Point center_blob = get_center(expanded_obs);
        Border expanded_border = compute_border( expanded_obs, center_blob);

        int N = expanded_obs.rows();
        expanded_obs.conservativeResize(N+expanded_border.rows(), Eigen::NoChange);
        for (int i_cell=0; i_cell<expanded_border.rows(); i_cell++)
        {
            expanded_obs.row(N+i_cell) << expanded_border(i_cell, 0), expanded_border(i_cell, 1); // [x,y] of the border cell
        }
    }

    Border placeholder_border;
    display_border(expanded_obs, placeholder_border);
    return expanded_obs;
}

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
       output.block(1,0,2,1) = data_closest.block(0,3,1,2).transpose();
   }
   else if ((data_closest(0,2)==2)||(data_closest(0,2)==3))
   {
       switch (static_cast<int>(data_closest(0,4))) {
            case 0: angle = std::atan2(robot(0,1)-(data_closest(0,1)-0.5), robot(0,0)-(data_closest(0,0)-0.5)); break;
            case 1: angle = std::atan2(robot(0,1)-(data_closest(0,1)-0.5), robot(0,0)-(data_closest(0,0)+0.5)); break;
            case 2: angle = std::atan2(robot(0,1)-(data_closest(0,1)+0.5), robot(0,0)-(data_closest(0,0)+0.5)); break;
            case 3: angle = std::atan2(robot(0,1)-(data_closest(0,1)+0.5), robot(0,0)-(data_closest(0,0)-0.5)); break;
      }
        output.block(1,0,2,1) << std::cos(angle), std::sin(angle);
   }
   else
   {
        throw std::invalid_argument("Should not happen. Closest border cell has no type." );
   }
   output(3,0) = 0;

   Eigen::Matrix<float, 1, 2> projected;
   projected = get_projection_on_border( robot, data_closest, angle);
   //std::cout << "Projected: " << projected << std::endl;
   mypoints << projected(0,0)  << "," << projected(0,1) << "\n";
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

    Point center_blob = get_center(obstacle);
    Border border_out;
    border_out = compute_border( obstacle, center_blob);

    Blob filled_obstacle = fill_gaps(obstacle);

    for (float x=limits(0,0); x <= limits(1,0); x += limits(4,0)) // x direction of the grid
    {
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
            //}
            if (!flag) // if not inside an obstacle
            {
                State next_eps = next_step_single_obstacle_border( state_robot, state_attractor, border_out); // compute velocity command
                myfile << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file
            }
        }
    }
    myfile.close();
    std::cout << "-- Quiver file closed --" << std::endl;
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

Eigen::Matrix<float, 1, 2> get_projection_on_border(Eigen::Matrix<float,1,2>  robot, Eigen::Matrix<float, 1, 6> data_closest, float const& angle)
{
    Eigen::Matrix<float, 1, 2> projected_point; projected_point << 0,0;

    switch (static_cast<int>(data_closest(0,2)))
    {
        case 1: if (data_closest(0,3)!=0) // normal along X axis
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
        } while (occupancy_grid(r_x, r_y)==1);
        occupancy_grid(r_x, r_y) = 1;
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


std::vector<Border> detect_borders( Grid const& occupancy_grid )
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
            if (occupancy_grid(cursor_row, cursor_col)==1) // if the cell is occupied -> it may be new obstacle
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

                    //if ( (cursor_row==16)&&(cursor_col==8) ) { std::cout << obstacle << std::endl;}
                    if (obstacle.rows()>1)
                    {
                        border_obstacle = compute_border( obstacle, start_obstacle);
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
    if (((row+1) < occupancy_grid.rows()) && (occupancy_grid(row+1, col)==1)) {explore_obstacle( obstacle, occupancy_grid, row+1, col  );}
    if (((row-1) >= 0                   ) && (occupancy_grid(row-1, col)==1)) {explore_obstacle( obstacle, occupancy_grid, row-1, col  );}
    if (((col+1) < occupancy_grid.cols()) && (occupancy_grid(row, col+1)==1)) {explore_obstacle( obstacle, occupancy_grid, row  , col+1);}
    if (((col-1) >= 0                   ) && (occupancy_grid(row, col-1)==1)) {explore_obstacle( obstacle, occupancy_grid, row  , col-1);}

    // 4 corners
    if (((row+1) < occupancy_grid.rows()) && ((col+1) < occupancy_grid.cols()) && (occupancy_grid(row+1, col+1)==1)) {explore_obstacle( obstacle, occupancy_grid, row+1, col+1);}
    if (((row+1) < occupancy_grid.rows()) && ((col-1) >= 0                   ) && (occupancy_grid(row+1, col-1)==1)) {explore_obstacle( obstacle, occupancy_grid, row+1, col-1);}
    if (((row-1) >= 0                   ) && ((col+1) < occupancy_grid.cols()) && (occupancy_grid(row-1, col+1)==1)) {explore_obstacle( obstacle, occupancy_grid, row-1, col+1);}
    if (((row-1) >= 0                   ) && ((col-1) >= 0                   ) && (occupancy_grid(row-1, col-1)==1)) {explore_obstacle( obstacle, occupancy_grid, row-1, col-1);}

}
