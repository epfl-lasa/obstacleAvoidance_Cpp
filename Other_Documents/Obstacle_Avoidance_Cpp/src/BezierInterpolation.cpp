#include "BezierInterpolation.h"

#include <eigen3/Eigen/LU>

extern bool logging_enabled; // Get from ObstacleRecontruction.cpp whether logging is enabled
extern Eigen::MatrixXf log_matrix; // Get from ObstacleRecontruction.cpp the logging matrix in which information is stored
extern float current_obstacle; // Get from ObstacleRecontruction.cpp the number/ID of the current obstacle


std::vector<Eigen::MatrixXf> compute_bezier(Eigen::MatrixXf const& XY)
{
    /* XY is a matrix with the following format
    [x1,y1,
     x2,y2,
     ..,..
     xn,yn]*/

    int nb_points = XY.rows(); // Number of points defining the surface

    Eigen::MatrixXf A = XY;

    // Building matrix linking P and A families of points (see report)
    // Shape of this matrix is
    // [ 4 1       1
    //   1 4 1
    //       ...
    //         1 4 1
    //   1       1 4 ] * (1/6)
    Eigen::MatrixXf mat = Eigen::MatrixXf::Zero(nb_points,nb_points);
    for (int i=0; i<nb_points; i++)
    {
        mat(i,i) = 4.0;
    }
    for (int i=0; i<(nb_points-1); i++)
    {
        mat(i,i+1) = 1.0;
        mat(i+1,i) = 1.0;
    }
    mat(0,nb_points-1) = 1.0;
    mat(nb_points-1,0) = 1.0;
    mat /= 6;

    // Get P families of points by inversing the matrix
    Eigen::MatrixXf P = mat.inverse() * A;

    // For interpolation, we have N points and interpolation is done for each pair of points [k, k+1]
    // It loops so points N+1 is actually point 0
    P.conservativeResize(P.rows()+1, Eigen::NoChange);
    P.row(P.rows()-1) = P.row(0);
    A.conservativeResize(A.rows()+1, Eigen::NoChange);
    A.row(A.rows()-1) = A.row(0);

    // Definition of B and C families of points
    Eigen::MatrixXf B = (2 * P.block(0,0,nb_points,2) + P.block(1,0,nb_points,2))/3;
    Eigen::MatrixXf C = (2 * P.block(1,0,nb_points,2) + P.block(0,0,nb_points,2))/3;

    // CONSTANT VALUE, number of points for the discretization of each [k, k+1] section of the surface
    const int nb_step = 101;
    Eigen::MatrixXf t = Eigen::VectorXf::LinSpaced(nb_step,0.0,1.0); // evenly spaced values between 0 and 1
    t.conservativeResize(nb_step-1,Eigen::NoChange);
    Eigen::Matrix<float, nb_step-1, 1> s = Eigen::MatrixXf::Ones(nb_step-1,1) - t;

    // Definition of matrices to get the interpolated value (see equations in the report)
    Eigen::Matrix<float, nb_step-1, 1> s3 = s.cwiseProduct(s.cwiseProduct(s));
    Eigen::Matrix<float, nb_step-1, 1> s2t = 3.0*s.cwiseProduct(s.cwiseProduct(t));
    Eigen::Matrix<float, nb_step-1, 1> t2s = 3.0*t.cwiseProduct(t.cwiseProduct(s));
    Eigen::Matrix<float, nb_step-1, 1> t3 = t.cwiseProduct(t.cwiseProduct(t));

    // Two matrices to store information about x and y coordinates respectively
    Eigen::MatrixXf bezier_pts_X(nb_step-1, nb_points);
    Eigen::MatrixXf bezier_pts_Y(nb_step-1, nb_points);

    // Computation of discretization points (see equations in the report)
    for (int i=0; i<nb_points; i++)
    {
        bezier_pts_X.col(i) = s3*A(i,0)+s2t*B(i,0)+t2s*C(i,0)+t3*A(i+1,0);
        bezier_pts_Y.col(i) = s3*A(i,1)+s2t*B(i,1)+t2s*C(i,1)+t3*A(i+1,1);
    }

    /*std::cout << "A:" << std::endl << A << std::endl;
    std::cout << "mat:" << std::endl << mat << std::endl;
    std::cout << "P:" << std::endl << P << std::endl;
    std::cout << "B:" << std::endl << B << std::endl;
    std::cout << "C:" << std::endl << C << std::endl;
    std::cout << "t:" << std::endl << t << std::endl;

    std::cout << "s3:" << std::endl << s3 << std::endl;
    std::cout << "s2t:" << std::endl << s2t << std::endl;

    std::cout << "bezier_pts_X:" << std::endl << bezier_pts_X << std::endl;
    std::cout << "bezier_pts_Y:" << std::endl << bezier_pts_Y << std::endl;*/

    bool log_surface = false;
    if (log_surface)
    {
        int num = 0;
        const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
        std::ofstream my_surface;
        my_surface.open("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+std::to_string(num)+"_surface_X.txt");
        my_surface << bezier_pts_X.format(CSVFormat) << "\n";
        my_surface.close();
        my_surface.open("/home/leziart/Documents/Project_25_06/StreamData/stream_data_"+std::to_string(num)+"_surface_Y.txt");
        my_surface << bezier_pts_Y.format(CSVFormat) << "\n";
        my_surface.close();
    }

    std::vector<Eigen::MatrixXf> result;
    result.push_back(bezier_pts_X);
    result.push_back(bezier_pts_Y);
    return result;
}


Eigen::MatrixXf border_to_vertices(Border const& obs)
{
    Eigen::MatrixXf result = Eigen::MatrixXf(obs.rows(),2);
    int pt_row = 0; // value that indicates which row of the result matrix has to be filled

    /** The surface loops (last cell is adjacent to first cell).
     *  When adding the point associated with cell k, cells k-1 and k+1 are also considered so it's easier to process
     *  if we can directely access k-1 and k+1 without bothering about the looping effect.
     *  The surface of the obstacle is processed by following it in the counter-clockwise direction
     */
    Eigen::MatrixXf obstacle(obs.rows()+2,5);
    obstacle << obs.row(obs.rows()-1), obs, obs.row(0);
    bool remove_last = false;

    for (int k=1; k<obstacle.rows()-1; k++) // First and last row are duplicate so they are not processed
    {
        if (obstacle(k,2)==1) // Straight line type of cell
        {
            if (obstacle(k,3)==1) // Normal is (1,0)
            {
                result.row(pt_row) << obstacle(k,0),  obstacle(k,1)+(1*0.5);
            }
            else if (obstacle(k,3)==(-1)) // Normal is (-1,0)
            {
                result.row(pt_row) << obstacle(k,0),  obstacle(k,1)-(1*0.5);
            }
            else if (obstacle(k,4)==(+1)) // Normal is (0,1)
            {
                result.row(pt_row) << obstacle(k,0)-(1*0.5),  obstacle(k,1);
            }
            else if (obstacle(k,4)==(-1)) // Normal is (0,-1)
            {
                result.row(pt_row) << obstacle(k,0)+(1*0.5),  obstacle(k,1);
            }
            else
            {
                throw std::invalid_argument("Should not happen. Invalid line cell");
            }
            pt_row += 1; // Increment row pointer

        }
        else if (obstacle(k,2)==2) // Outer corner type of cell
        {
            if (obstacle(k,4)==0) // Outer corner number 0 (see report)
            {
                result.row(pt_row) << obstacle(k,0)-(1*0.5),  obstacle(k,1);
            }
            else if (obstacle(k,4)==1) // Outer corner number 1 (see report)
            {
                result.row(pt_row) << obstacle(k,0),  obstacle(k,1)-(1*0.5);
            }
            else if (obstacle(k,4)==2) // Outer corner number 2 (see report)
            {
                result.row(pt_row) << obstacle(k,0)+(1*0.5),  obstacle(k,1);
            }
            else if (obstacle(k,4)==3) // Outer corner number 3 (see report)
            {
                result.row(pt_row) << obstacle(k,0),  obstacle(k,1)+(1*0.5);
            }
            else
            {
                throw std::invalid_argument("Should not happen. Invalid circle cell");
            }
            pt_row += 1;
        }
        else if (obstacle(k,2)==3) // Inner corner type of cell
        {
            if ((obstacle(k-1,2) == 2) && (obstacle(k+1,2) != 2)) // Heuristic rule to make the surface smoother for a specific pattern
            {
                if (pt_row>0) {pt_row -= 1;}
                else {remove_last = true;}
            }

            if ((obstacle(k-1,2) == 1) && (obstacle(k+1,2) != 3)) // Heuristic rule to make the surface smoother for a specific pattern
            {
                if (pt_row>0) {pt_row -= 1;}
                else {remove_last = true;}
            }

            if ((obstacle(k-1,2) == 3) && (obstacle(k+1,2) == 1))
            {
                if (obstacle(k,4)==0) // Inner corner number 0 (see report)
                {
                    result.row(pt_row) << obstacle(k,0),  obstacle(k,1)-(1*0.5);
                }
                else if (obstacle(k,4)==1) // Inner corner number 1 (see report)
                {
                    result.row(pt_row) << obstacle(k,0)+(1*0.5),  obstacle(k,1);
                }
                else if (obstacle(k,4)==2) // Inner corner number 2 (see report)
                {
                    result.row(pt_row) << obstacle(k,0),  obstacle(k,1)+(1*0.5);
                }
                else if (obstacle(k,4)==3) // Inner corner number 3 (see report)
                {
                    result.row(pt_row) << obstacle(k,0)-(1*0.5),  obstacle(k,1);
                }
                else
                {
                    throw std::invalid_argument("Should not happen.");
                }
                pt_row += 1;
            }
        }
        else if ((obstacle(k-1,2) == 3) || (obstacle(k+1,2) == 1) || ((obstacle(k-1,2) == 1) && (obstacle(k+1,2) != 3)))
        {
            // Put c||ner cell
            if (obstacle(k,4)==0)
            {
                result.row(pt_row) << obstacle(k,0)-(1*0.5),  obstacle(k,1)-(1*0.5);
            }
            else if (obstacle(k,4)==1)
            {
                result.row(pt_row) << obstacle(k,0)+(1*0.5),  obstacle(k,1)-(1*0.5);
            }
            else if (obstacle(k,4)==2)
            {
                result.row(pt_row) << obstacle(k,0)+(1*0.5),  obstacle(k,1)+(1*0.5);
            }
            else if (obstacle(k,4)==3)
            {
                result.row(pt_row) << obstacle(k,0)-(1*0.5),  obstacle(k,1)+(1*0.5);
            }
            else
            {
                throw std::invalid_argument("Should not happen.");
            }
            pt_row += 1;

        }
        else if ((obstacle(k+1,2) == 2) || (obstacle(k-1,2) == 3))
        {
            if (obstacle(k,4)==0)
            {
                result.row(pt_row) << obstacle(k,0),  obstacle(k,1)-(1*0.5);
            }
            else if (obstacle(k,4)==1)
            {
                result.row(pt_row) << obstacle(k,0)+(1*0.5),  obstacle(k,1);
            }
            else if (obstacle(k,4)==2)
            {
                result.row(pt_row) << obstacle(k,0),  obstacle(k,1)+(1*0.5);
            }
            else if (obstacle(k,4)==3)
            {
                result.row(pt_row) << obstacle(k,0)-(1*0.5),  obstacle(k,1);
            }
            else
            {
                throw std::invalid_argument("Should not happen.");
            }
            pt_row += 1;
        }
        else
        {
            throw std::invalid_argument("Should not happen.");
        }

    }
    if (remove_last) {pt_row -= 1;}
    result.conservativeResize(pt_row, Eigen::NoChange);
    return result;
}


State get_projection_on_bezier(State const& state_robot, std::vector<Eigen::MatrixXf> const& pts_bezier)
{

    Eigen::Matrix<float, 1, 2> robot; robot << state_robot(0,0), state_robot(1,0);

    Eigen::MatrixXf robot_X = state_robot(0,0) * Eigen::MatrixXf::Ones(1,(pts_bezier[0]).cols());
    Eigen::MatrixXf robot_Y = state_robot(1,0) * Eigen::MatrixXf::Ones(1,(pts_bezier[0]).cols());

    Eigen::MatrixXf dist_X = (robot_X-(pts_bezier[0]).row(0)).cwiseProduct(robot_X-(pts_bezier[0]).row(0));
    Eigen::MatrixXf dist_Y = (robot_Y-(pts_bezier[1]).row(0)).cwiseProduct(robot_Y-(pts_bezier[1]).row(0));
    Eigen::MatrixXf dist = (dist_X+dist_Y).array().sqrt().matrix();

    /** The surface of the obstacles is defined by N points
     *  We want to get the closest point to the robot among these N points. Let's say it's the i-th point.
     *  That way we know that the projection of the robot will either be on the [i-1, i] section or the [i,i+1] one.
     */
    float value_min = dist(0,0);
    int index_min = 0;
    for (int i=1; i<dist.cols(); i++)
    {
        if (dist(0,i)<value_min)
        {
            index_min = i;
            value_min = dist(0,i);
        }
    }

    int index_prev_min = index_min - 1;
    int col_min = index_min;
    if (index_prev_min < 0) {index_prev_min = dist.cols()-1;} // Loop effect

    // I use a column matrix for faster computation by working directly on a whole section
    robot_X = state_robot(0,0) * Eigen::MatrixXf::Ones((pts_bezier[0]).rows(),1);
    robot_Y = state_robot(1,0) * Eigen::MatrixXf::Ones((pts_bezier[0]).rows(),1);

    // Get distances between the robot and all discretization points of section [i-1, i]
    dist_X = (robot_X-(pts_bezier[0]).col(index_prev_min)).cwiseProduct(robot_X-(pts_bezier[0]).col(index_prev_min));
    dist_Y = (robot_Y-(pts_bezier[1]).col(index_prev_min)).cwiseProduct(robot_Y-(pts_bezier[1]).col(index_prev_min));
    Eigen::MatrixXf dist_1 = (dist_X+dist_Y).array().sqrt().matrix();

    // Get distances between the robot and all discretization points of section [i, i+1]
    dist_X = (robot_X-(pts_bezier[0]).col(index_min)).cwiseProduct(robot_X-(pts_bezier[0]).col(index_min));
    dist_Y = (robot_Y-(pts_bezier[1]).col(index_min)).cwiseProduct(robot_Y-(pts_bezier[1]).col(index_min));
    Eigen::MatrixXf dist_2 = (dist_X+dist_Y).array().sqrt().matrix();

    // Find the closest discretization point by scanning both section to find the minimum distance
    int i_dist = 1;
    value_min = dist_1(0,0);
    index_min = 0;
    for (int i=1; i<dist_1.rows(); i++)
    {
        if (dist_1(i,0)<value_min)
        {
            index_min = i;
            value_min = dist_1(i,0);
        }
    }
    for (int i=0; i<dist_2.rows(); i++)
    {
        if (dist_2(i,0)<value_min)
        {
            i_dist = 2;
            index_min = i;
            value_min = dist_2(i,0);
        }
    }

    // Store information about the closest discretization point
    float closest_X, closest_Y;
    if (i_dist==1)
    {
        closest_X = (pts_bezier[0])(index_min,index_prev_min);
        closest_Y = (pts_bezier[1])(index_min,index_prev_min);
    }
    else
    {
        closest_X = (pts_bezier[0])(index_min,col_min);
        closest_Y = (pts_bezier[1])(index_min,col_min);
    }

    State closest; closest << closest_X, closest_Y, 0;

    return closest;
}

Eigen::Matrix<float, 1, 3> get_distances_surface_bezier(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float, 1, 2> const& proj_attractor, std::vector<Eigen::MatrixXf> const& pts_bezier)
{
    Eigen::Matrix<float, 1, 3> distances_out;

    int n_rows = (pts_bezier[0]).rows();
    int n_cols = (pts_bezier[0]).cols();

    // Definition of the "shifted" matrix
    // [ 1  4  7  10    becomes [ 12  3  6   9
    //   2  5  8  11               1  4  7  10
    //   3  6  9  12 ]             2  5  8  11 ]

    // Shifted matrices along X and Y dimensions
    Eigen::MatrixXf shifted_X = Eigen::MatrixXf( n_rows, n_cols);
    Eigen::MatrixXf shifted_Y = Eigen::MatrixXf( n_rows, n_cols);

    shifted_X(0,0) = (pts_bezier[0])(n_rows-1, n_cols-1);
    shifted_Y(0,0) = (pts_bezier[1])(n_rows-1, n_cols-1);

    shifted_X.block(0,1,1,n_cols-1) = (pts_bezier[0]).block(n_rows-1,0,1,n_cols-1);
    shifted_Y.block(0,1,1,n_cols-1) = (pts_bezier[1]).block(n_rows-1,0,1,n_cols-1);

    shifted_X.block(1,0,n_rows-1,n_cols) = (pts_bezier[0]).block(0,0,n_rows-1,n_cols);
    shifted_Y.block(1,0,n_rows-1,n_cols) = (pts_bezier[1]).block(0,0,n_rows-1,n_cols);

    // Matrix of the distance from one point of the surface to the next one close to it
    // dist = sqrt(diff_X^2 + diff_Y^2)
    Eigen::MatrixXf dist_2_points = Eigen::MatrixXf( n_rows, n_cols);
    dist_2_points = (((pts_bezier[0]-shifted_X).cwiseProduct(pts_bezier[0]-shifted_X))+((pts_bezier[1]-shifted_Y).cwiseProduct(pts_bezier[1]-shifted_Y))).array().sqrt().matrix();

    // Perimeter of the obstacle
    distances_out(0,0) = dist_2_points.sum(); // sum of all coefficients

    // Getting indexes of the projection of the robot and the projection of the attractor in the matrix
    int i_x_robot = 0;
    int i_y_robot = 0;
    int i_x_attractor = 0;
    int i_y_attractor = 0;

    for (int i=0; i<n_rows; i++)
    {
        for (int j=0; j<n_cols; j++)
        {
            if (((pts_bezier[0])(i,j) == proj_robot(0,0)) && ((pts_bezier[1])(i,j) == proj_robot(0,1)))
            {
                i_x_robot = i;
                i_y_robot = j;
            }

            if (((pts_bezier[0])(i,j) == proj_attractor(0,0)) && ((pts_bezier[1])(i,j) == proj_attractor(0,1)))
            {
                i_x_attractor = i;
                i_y_attractor = j;
            }
        }
    }

    float distance_up = 0;

    // Following the border till point associated to the attractor is reached
    // When jumping to the next discretization point, the distance between this point and the previous one is added to the total distance
    while ((i_x_robot!=i_x_attractor) || (i_y_robot!=i_y_attractor))
    {
        distance_up += dist_2_points(i_x_robot,i_y_robot);
        i_x_robot += 1;
        if (i_x_robot == n_rows)
        {
            i_x_robot = 0;
            i_y_robot++;
            if (i_y_robot == n_cols)
            {
                i_y_robot = 0;
            }
        }
    }

    // Saving the minimal distance
    float distance_down = distances_out(0,0) - distance_up; // Perimeter - distance traveled is the distance by following the surface in the other direction
    distances_out(0,1) = std::min(distance_up, distance_down);

    // Saving the direction for which the distance is minimal
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


float get_max_gamma_distance_bezier(Eigen::Matrix<float, 1, 2> const& proj_robot, Eigen::Matrix<float,1,2> const& normal, std::vector<Eigen::MatrixXf> const& pts_bezier) // maximum gamma distance that can be reached following the normal
{
    //std::cout << "Projected point: " << proj_robot << std::endl;
    //std::cout << "Normal vector: " << normal << std::endl;

    float current_step = 0.5 * 1;
    float min_step = 0.01; // minimum size of a step
    float max_step = 100;  // maximum size of a step
    float interval = 0.1;

    Eigen::Matrix<float, 1, 2> current_point; current_point = proj_robot;
    Eigen::Matrix<float, 1, 2> next_point;

    while ((current_step>min_step) && (current_step<max_step))
    {
        //std::cout << "Current point: " << current_point << " | Current step: " << current_step << std::endl;
        next_point << current_point(0,0) + current_step * normal(0,0), current_point(0,1) + current_step * normal(0,1);

        State curr; curr << next_point(0,0), next_point(0,1), 0;
        State next_proj = get_projection_on_bezier(curr, pts_bezier);

        if ((std::abs(next_proj(0,0)-proj_robot(0,0))<interval)&&(std::abs(next_proj(1,0)-proj_robot(0,1))<interval))
        {
            current_point = next_point;
            current_step *= 2;
        }
        else
        {
            //std::cout << next_proj.transpose() << std::endl;
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
    // it crosses the axis of symmetry of the U shape

    // if (current_step > max_step) it means that the maximum gamma distance is likely infinity (approximation)
    // if (current_step < min_step) it means that the current point has reached a limit before switching to another closest cell
    // for instance if the current point is within a U-shaped object this limit is the vertical axis that cuts the U into two parts

    return (1 + std::sqrt(std::pow(current_point(0,0)-proj_robot(0,0),2) + std::pow(current_point(0,1)-proj_robot(0,1),2)));
    // TODO: Call a more general function that will compute gamma
}

Eigen::Matrix<float, 4, 1> test_next_step_special(State const& state_robot, State const& state_attractor, Border const& border)
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
    if (false)
    {
        std::cout << "Closest         " << closest << std::endl;
        std::cout << "Projected robot " << gamma_norm_proj(4,0) << " " << gamma_norm_proj(5,0) << std::endl;
        std::cout << "Robot_vec       " << robot_vec.transpose() << std::endl;
        std::cout << "Normal_vec      " << normal_vec.transpose()  << std::endl;
        std::cout << "Dot product     " << normal_vec.dot(robot_vec.colwise().normalized()) << std::endl;
        std::cout << "Projected attractor " << gamma_norm_proj_attractor(4,0) << " " << gamma_norm_proj_attractor(5,0) << std::endl;
    }



    float speed_reverse = 1000; // speed to get out of an obstacle (will be limited by speed_limiter, basically the robot will go at max speed)

    if (((closest(0,2)==1)||(closest(0,2)==2))&&(normal_vec.dot(robot_vec.colwise().normalized()) < 0)) // if true it means the robot is inside obstacle
    {
        // If inside an obstacle, avoid display by returning NaN
        //Eigen::Matrix<float, 4, 1>  output; output << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), 0, std::numeric_limits<float>::quiet_NaN();

        // If inside an obstacle, velocity command is normal to the surface
        Eigen::Matrix<float, 4, 1>  output;
        float norm_vec = std::sqrt(std::pow(robot_vec(0,0),2) + std::pow(robot_vec(1,0),2));
        output << (- speed_reverse * robot_vec(0,0) / norm_vec), (- speed_reverse * robot_vec(1,0) / norm_vec), 0, 1; // [v_along_x, v_along_y, v_rotation, gamma_distance]
        //my_circle_space << 1000 << "," << 1000 << "\n";   // write position of the point in the circle space (for matplotlib)
        output << 0.0, 0.0, 0, 1;
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
        //my_circle_space << 1000 << "," << 1000 << "\n";   // write position of the point in the circle space (for matplotlib)
        output << 0.0, 0.0, 0, 1;
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

    /*std::cout << "Robot " << state_robot(0,0) << " " << state_robot(1,0) << std::endl;
    std::cout << "Projected robot " << gamma_norm_proj(4,0) << " " << gamma_norm_proj(5,0) << std::endl;
    std::cout << "Gamma dist robot:         " << gamma_norm_proj(0,0) << std::endl;*/
    // TEST BEZIER POINTS
    Eigen::MatrixXf res_bez = border_to_vertices(border);
    //std::cout << res_bez << std::endl;
    std::vector<Eigen::MatrixXf> pts_bezier = compute_bezier(res_bez);
    State proj_robot_state = get_projection_on_bezier(state_robot, pts_bezier);
    //std::cout << proj_robot_state.transpose() << std::endl;
    State proj_attractor_state = get_projection_on_bezier(state_attractor, pts_bezier);

    proj_robot.row(0) << proj_robot_state(0,0), proj_robot_state(1,0);
    proj_attractor.row(0) << proj_attractor_state(0,0), proj_attractor_state(1,0);
    Eigen::Matrix<float, 1, 2> normal_robot; normal_robot.row(0) << state_robot(0,0) - proj_robot(0,0), state_robot(1,0) - proj_robot(0,1);
    normal_robot = normal_robot / std::sqrt(std::pow(normal_robot(0,0),2) + std::pow(normal_robot(0,1),2));



    // Test if the robot is on the wrong side of the boundary for some reason (like time discretization)
    State temp_state; temp_state << normal_robot(0,0), normal_robot(0,1), 0.0;
    if ((((closest(0,2)==1)||(closest(0,2)==2))&&(normal_vec.dot(temp_state) < 0))||((closest(0,2)==3)&&(normal_vec.dot(temp_state) > 0)))
    {
        // std::cout << "IS IN OBSTACLE - REVERSE SPEED" << std::endl;
        normal_robot *= (-1);
        Eigen::Matrix<float, 4, 1> output;
        output << speed_reverse * normal_robot(0,0), speed_reverse * normal_robot(0,1), 0, 1;
        output << 0.0, 0.0, 0, 1;
        return output;
    }

    Eigen::Matrix<float, 1, 2> normal_attractor; normal_attractor << state_attractor(0,0) - proj_attractor(0,0), state_attractor(1,0) - proj_attractor(0,1);
    normal_attractor = normal_attractor / std::sqrt(std::pow(normal_attractor(0,0),2) + std::pow(normal_attractor(0,1),2));


    distances_surface = get_distances_surface_bezier( proj_robot, proj_attractor, pts_bezier);

    max_gamma_robot = get_max_gamma_distance_bezier( proj_robot, normal_robot, pts_bezier);

    max_gamma_attractor = get_max_gamma_distance_bezier( proj_attractor, normal_attractor, pts_bezier);

    float gamma_robot = (1 + std::sqrt(std::pow(state_robot(0,0)-proj_robot(0,0),2) + std::pow(state_robot(1,0)-proj_robot(0,1),2)));
    float gamma_attractor = (1 + std::sqrt(std::pow(state_attractor(0,0)-proj_attractor(0,0),2) + std::pow(state_attractor(1,0)-proj_attractor(0,1),2)));

    gamma_norm_proj << gamma_robot, normal_robot(0,0), normal_robot(0,1), 0.0, proj_robot(0,0), proj_robot(0,1);
    gamma_norm_proj_attractor << gamma_attractor, normal_attractor(0,0), normal_attractor(0,1), 0.0, proj_attractor(0,0), proj_attractor(0,1);

    /*std::cout << "Normal robot bezier " << normal_robot << std::endl;
    std::cout << "Projected robot bezier " << proj_robot(0,0) << " " << proj_robot(0,1) << std::endl;
    std::cout << "Gamma bezier " << gamma_robot << std::endl;*/
    // END TEST BEZIER POINTS

    // Get the position of the robot in the circle space
    Eigen::Matrix<float, 10, 1> point_circle_space = get_point_circle_frame( distances_surface(0,1), distances_surface(0,0), gamma_norm_proj(0,0), max_gamma_robot, gamma_norm_proj_attractor(0,0), max_gamma_attractor, distances_surface(0,2));
    float gamma_circle_frame = point_circle_space(0,0);
    State robot_circle_frame = point_circle_space.block(1,0,3,1);
    State attractor_circle_frame = point_circle_space.block(4,0,3,1);
    State ref_vec_circle_frame = point_circle_space.block(7,0,3,1);

//    my_circle_space << robot_circle_frame(0,0) << "," << robot_circle_frame(1,0) << "\n"; // write position of the point in the circle space (for matplotlib)


    if (closest(0,2)==3)
    {
        ref_vec_circle_frame = (-1) * ref_vec_circle_frame;
    }

    // Compute attractor function
    f_eps = (attractor_circle_frame - robot_circle_frame);

    // Normalization of the speed to a default desired speed
    f_eps = f_eps / (std::sqrt(std::pow(f_eps(0,0),2) + std::pow(f_eps(1,0),2)));

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

    if (false)
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
    //std::cout << "Vel circle frame:         " << velocity_circle_space.block(0,0,2,1).transpose() << std::endl;
    //std::cout << "Vel shape frame:          "  << velocity_shape_space.block(0,0,2,1).transpose() << std::endl;
    /*if (logging_enabled)
    {
        // std::cout << "Logging inside next_step_special" << std::endl;
        int nb_features = 6;
        log_matrix.conservativeResize(log_matrix.rows()+nb_features, Eigen::NoChange); // Add 6 rows at the end for features 3 to 8
        log_matrix.block(log_matrix.rows()-nb_features, 0, nb_features, 1) = current_obstacle * Eigen::MatrixXf::Ones(6, 1); // Numero of obstacle for the 6 features

        // Feature 3: projection of the robot on the boundary in the initial and circle spaces
        log_matrix(log_matrix.rows()-nb_features, 1) = 3; // Numero of feature
        log_matrix.block(log_matrix.rows()-nb_features, 2, 1, 5) << proj_robot(0,0),proj_robot(0,1),r_eps_vector(0,0),r_eps_vector(1,0),0.0; // Add data

        // Feature 4: projection of the attractor on the boundary in the initial and circle spaces
        log_matrix(log_matrix.rows()-nb_features+1, 1) = 4; // Numero of feature
        log_matrix.block(log_matrix.rows()-nb_features+1, 2, 1, 5) << proj_attractor(0,0),proj_attractor(0,1), 1.0, 0.0, 0.0; // Add data

        // Feature 5: position of the robot in the initial and circle spaces
        log_matrix(log_matrix.rows()-nb_features+2, 1) = 5; // Numero of feature
        log_matrix.block(log_matrix.rows()-nb_features+2, 2, 1, 5) << state_robot(0,0),state_robot(1,0),robot_circle_frame(0,0),robot_circle_frame(1,0),0.0; // Add data

        // Feature 6: position of the attractor in the initial and circle spaces
        log_matrix(log_matrix.rows()-nb_features+3, 1) = 6; // Numero of feature
        log_matrix.block(log_matrix.rows()-nb_features+3, 2, 1, 5) << state_attractor(0,0),state_attractor(1,0),attractor_circle_frame(0,0),attractor_circle_frame(1,0),0.0; // Add data

        // Feature 7: velocity command in the initial and circle spaces
        log_matrix(log_matrix.rows()-nb_features+4, 1) = 7; // Numero of feature
        log_matrix.block(log_matrix.rows()-nb_features+4, 2, 1, 5) << velocity_shape_space(0,0),velocity_shape_space(1,0),velocity_circle_space(0,0),velocity_circle_space(1,0),0.0; // Add data

        // Feature 8: velocity command in the initial and circle spaces
        log_matrix(log_matrix.rows()-nb_features+5, 1) = 8; // Numero of feature
        log_matrix.block(log_matrix.rows()-nb_features+5, 2, 1, 5) << gamma_norm_proj(0,0),gamma_circle_frame,0.0,0.0,0.0; // Add data
    }*/

    //Eigen::Matrix<float, 1, 1> norm = velocity_shape_space.colwise().norm();
    //velocity_shape_space = velocity_shape_space / std::sqrt(std::pow(velocity_shape_space(0,0),2)+std::pow(velocity_shape_space(1,0),2));

    Eigen::Matrix<float, 4, 1> output;
    output.block(0,0,3,1) = velocity_shape_space;
    output(3,0) = gamma_norm_proj(0,0);
    return output;
}

State test_next_step_special_weighted(State const& state_robot, State const& state_attractor, std::vector<Border> const& borders, float const& size_of_cells)
{   // compute the velocity command for a given position of the robot/attractor/obstacles

    if (logging_enabled)
    {
        current_obstacle = 1; // Init obstacle counter


        //log_matrix.conservativeResize(log_matrix.rows()+1, Eigen::NoChange); // Add a new row at the end
        //log_matrix.row(log_matrix.rows()-1) << 1,1,1,2,3,4,5;// Fill the new row;
    }

    // Compute all the steps to get the velocity command considering several obstacles
    const int number_obstacles = borders.size();

    // Velocity command for each obstacle
    // mat_velocities is a matrix with size "number_states x number_obstacles"
    Eigen::MatrixXf mat_velocities(number_states, number_obstacles);  // "3 x number_of_obstacles" (x,y,phi)
    Eigen::MatrixXf mat_gamma(1, number_obstacles);                   // "1 x number_of_obstacles" (gamma distance)



    for (int i=0; i < borders.size(); i++)
    {
        /*if (false)
        {
            log_matrix.conservativeResize(log_matrix.rows()+(borders[i]).rows(), Eigen::NoChange); // Add rows at the end
            log_matrix.block(log_matrix.rows()-(borders[i]).rows(), 0, (borders[i]).rows(), 1) = current_obstacle * Eigen::MatrixXf::Ones((borders[i]).rows(), 1); // Numero of obstacle
            log_matrix.block(log_matrix.rows()-(borders[i]).rows(), 1, (borders[i]).rows(), 1) = 2 * Eigen::MatrixXf::Ones((borders[i]).rows(), 1); // Numero of feature
            log_matrix.block(log_matrix.rows()-(borders[i]).rows(), 2, (borders[i]).rows(), 5) = borders[i]; // Add border information
        }*/
        if (logging_enabled)
        {
            log_matrix.conservativeResize(log_matrix.rows()+(borders[i]).rows(), Eigen::NoChange); // Add rows at the end
            log_matrix.block(log_matrix.rows()-(borders[i]).rows(), 0, (borders[i]).rows(), 1) = current_obstacle * Eigen::MatrixXf::Ones((borders[i]).rows(), 1); // Numero of obstacle
            log_matrix.block(log_matrix.rows()-(borders[i]).rows(), 1, (borders[i]).rows(), 1) = 2 * Eigen::MatrixXf::Ones((borders[i]).rows(), 1); // Numero of feature
            log_matrix.block(log_matrix.rows()-(borders[i]).rows(), 2, (borders[i]).rows(), 5) = borders[i]; // Add border information
        }

        Eigen::Matrix<float, 4, 1> output = test_next_step_special(state_robot, state_attractor, borders[i]); // compute velocity command for each obstacle
        mat_velocities(0,i) = output(0,0);
        mat_velocities(1,i) = output(1,0);
        mat_velocities(2,i) = output(2,0);
        // mat_velocities.col(i) = output.block(0,0,3,1); // TODO: Suddenly started to cause a Segmentation fault for no reason :O
        // Assigning the block does not work anymore but doing it coefficient by coefficient works... It makes no sense
        mat_velocities(2,i) = 0;
        mat_gamma(0,i) = output(3,0);

        //current_obstacle += 1;
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

        // Normalization of the speed to a default desired speed
        cmd_velocity = cmd_velocity / (std::sqrt(std::pow(cmd_velocity(0,0),2) + std::pow(cmd_velocity(1,0),2)));

        //cmd_velocity(2,0) = std::atan2(cmd_velocity(1,0),cmd_velocity(0,0)) - state_robot(2,0); // angle difference used for angular speed control (gain of 1)
        float diff_angle_1 = std::abs(std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0) + 2*3.1415);
        float diff_angle_2 = std::abs(std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0)         );
        float diff_angle_3 = std::abs(std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0) - 2*3.1415);
        if ((diff_angle_1<diff_angle_2)&&(diff_angle_1<diff_angle_3))
           {cmd_velocity(2,0) = std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0) + 2*3.1415;}
        else if (diff_angle_2 < diff_angle_3)
           {cmd_velocity(2,0) = std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0);}
        else
           {cmd_velocity(2,0) = std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0) - 2*3.1415;}

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

    // Transition in the [0, end_of_transition_weight] interval
    float end_of_transition_weight = 0.1;

    // Smooth transition with the "no obstacle in range" area of the workspace
    if ((mat_weights.size()==1))
    {
        Eigen::MatrixXf mat_weights_not_normalized(1, number_obstacles); // "1 x number_of_obstacles"
        mat_weights_not_normalized = weights_special( state_robot, mat_gamma, method_weights, limit_dist, true);

        if (mat_weights_not_normalized(0,0)<end_of_transition_weight)
        {
            // Velocity when there is no obstacle in range
            State cmd_no_obstacle = state_attractor - state_robot;

            // Normalization of the speed to a default desired speed
            cmd_no_obstacle = cmd_no_obstacle / (std::sqrt(std::pow(cmd_no_obstacle(0,0),2) + std::pow(cmd_no_obstacle(1,0),2)));

            //cmd_no_obstacle(2,0) = std::atan2(cmd_no_obstacle(1,0),cmd_no_obstacle(0,0)) - state_robot(2,0); // angle difference used for angular speed control (gain of 1)
            float diff_angle_1 = std::abs(std::atan2(-cmd_no_obstacle(1,0),-cmd_no_obstacle(0,0)) - state_robot(2,0) + 2*3.1415);
        float diff_angle_2 = std::abs(std::atan2(-cmd_no_obstacle(1,0),-cmd_no_obstacle(0,0)) - state_robot(2,0)         );
        float diff_angle_3 = std::abs(std::atan2(-cmd_no_obstacle(1,0),-cmd_no_obstacle(0,0)) - state_robot(2,0) - 2*3.1415);
        if ((diff_angle_1<diff_angle_2)&&(diff_angle_1<diff_angle_3))
           {cmd_no_obstacle(2,0) = std::atan2(-cmd_no_obstacle(1,0),-cmd_no_obstacle(0,0)) - state_robot(2,0) + 2*3.1415;}
        else if (diff_angle_2 < diff_angle_3)
           {cmd_no_obstacle(2,0) = std::atan2(-cmd_no_obstacle(1,0),-cmd_no_obstacle(0,0)) - state_robot(2,0);}
        else
           {cmd_no_obstacle(2,0) = std::atan2(-cmd_no_obstacle(1,0),-cmd_no_obstacle(0,0)) - state_robot(2,0) - 2*3.1415;}

            // Decrease speed when the robot get close to the attractor
            float distance_stop = 0.25;
            float distance_start_decreasing = 1 / size_of_cells; // numerical value is in meters, converted into the cell world
            float distance_to_attractor = std::sqrt(std::pow(state_robot(0,0)-state_attractor(0,0),2)+std::pow(state_robot(1,0)-state_attractor(1,0),2)); // Euclidian distance
            //std::cout << "Distance to attractor:   " << distance_to_attractor << std::endl;
            cmd_no_obstacle.block(0,0,3,1) *= std::min(std::max(static_cast<float>(0.0),distance_to_attractor-distance_stop)/(distance_start_decreasing-distance_stop), static_cast<float>(1.0));

            cmd_no_obstacle = speed_limiter(cmd_no_obstacle);

            // Transition equation
            State cmd_tempo = ((end_of_transition_weight - mat_weights_not_normalized(0,0))/end_of_transition_weight) * cmd_no_obstacle + (mat_weights_not_normalized(0,0)/end_of_transition_weight) * (speed_limiter(weighted_mag * weighted_direction));

            // Normalization
            //cmd_velocity = cmd_velocity / (std::sqrt(std::pow(cmd_velocity(0,0),2) + std::pow(cmd_velocity(1,0),2)));

             cmd_velocity = cmd_tempo;
        }

    }

    // HAVE SET VELOCIT FOR PLOTTING
    cmd_velocity = cmd_velocity / (std::sqrt(std::pow(cmd_velocity(0,0),2) + std::pow(cmd_velocity(1,0),2)));

    // Set the angular velocity to align the robot with the direction it moves (post process for better movement, thinner profile to go between obstacles)
    //cmd_velocity(2,0) = std::atan2(cmd_velocity(1,0),cmd_velocity(0,0)) - state_robot(2,0); // angle difference used for angular speed control (gain of 1)
    float diff_angle_1 = std::abs(std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0) + 2*3.1415);
        float diff_angle_2 = std::abs(std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0)         );
        float diff_angle_3 = std::abs(std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0) - 2*3.1415);
        if ((diff_angle_1<diff_angle_2)&&(diff_angle_1<diff_angle_3))
           {cmd_velocity(2,0) = std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0) + 2*3.1415;}
        else if (diff_angle_2 < diff_angle_3)
           {cmd_velocity(2,0) = std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0);}
        else
           {cmd_velocity(2,0) = std::atan2(-cmd_velocity(1,0),-cmd_velocity(0,0)) - state_robot(2,0) - 2*3.1415;}

    /*std::cout << state_robot << std::endl;
    std::cout << state_attractor << std::endl;
    std::cout << mat_obs << std::endl;
    std::cout << mat_velocities << std::endl;
    std::cout << mat_norm_velocities << std::endl;
    std::cout << mat_weights << std::endl;
    std::cout << weighted_mag << std::endl;*/
    //std::cout << "mat_weights:    " << mat_weights << std::endl;

    if (false)
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

Eigen::Matrix<float, 4, 1> next_step_classic(State const& state_robot, State const& state_attractor, State const& state_reference, Border const& border)
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
    Eigen::Matrix<float, 1, 2> reference_point = state_reference.block(0,0,2,1).transpose();

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

    if (false)
    {
        std::cout << "Closest         " << closest << std::endl;
        std::cout << "Projected robot " << gamma_norm_proj(4,0) << " " << gamma_norm_proj(5,0) << std::endl;
        std::cout << "Robot_vec       " << robot_vec.transpose() << std::endl;
        std::cout << "Normal_vec      " << normal_vec.transpose()  << std::endl;
        std::cout << "Dot product     " << normal_vec.dot(robot_vec.colwise().normalized()) << std::endl;
        std::cout << "Projected attractor " << gamma_norm_proj_attractor(4,0) << " " << gamma_norm_proj_attractor(5,0) << std::endl;
    }


    float speed_reverse = 1000; // speed to get out of an obstacle (will be limited by speed_limiter, basically the robot will go at max speed)

    if (((closest(0,2)==1)||(closest(0,2)==2))&&(normal_vec.dot(robot_vec.colwise().normalized()) < 0)) // if true it means the robot is inside obstacle
    {
        // If inside an obstacle, avoid display by returning NaN
        //Eigen::Matrix<float, 4, 1>  output; output << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), 0, std::numeric_limits<float>::quiet_NaN();

        // If inside an obstacle, velocity command is normal to the surface
        Eigen::Matrix<float, 4, 1>  output;
        float norm_vec = std::sqrt(std::pow(robot_vec(0,0),2) + std::pow(robot_vec(1,0),2));
        output << (- speed_reverse * robot_vec(0,0) / norm_vec), (- speed_reverse * robot_vec(1,0) / norm_vec), 0, 1; // [v_along_x, v_along_y, v_rotation, gamma_distance]
        //my_circle_space << 1000 << "," << 1000 << "\n";   // write position of the point in the circle space (for matplotlib)
        output << 0.0, 0.0, 0, 1;
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
        //my_circle_space << 1000 << "," << 1000 << "\n";   // write position of the point in the circle space (for matplotlib)
        output << 0.0, 0.0, 0, 1;
        return output;
    }

    // Get minimal distance along surface (either by following the surface clockwise or counter-clockwise)
    // Distance between the projected point of the robot and the projected point of the attractor
    Eigen::Matrix<float, 1, 2> proj_robot;     proj_robot     << gamma_norm_proj(4,0), gamma_norm_proj(5,0);
    Eigen::Matrix<float, 1, 2> proj_attractor; proj_attractor << gamma_norm_proj_attractor(4,0), gamma_norm_proj_attractor(5,0);


    // Get information about the position of the project point on the boundary of the obstacle
    //Eigen::Matrix<float, 1, 3> distances_surface = get_distances_surface(proj_robot, proj_attractor, border);


    // Get the maximum gamma distance in the initial space for the robot
    //float max_gamma_robot = get_max_gamma_distance( proj_robot, gamma_norm_proj.block(1,0,2,1).transpose(), border);


    // Get the maximum gamma distance in the initial space for the attractor
    //float max_gamma_attractor = get_max_gamma_distance( proj_attractor, gamma_norm_proj_attractor.block(1,0,2,1).transpose(), border);


    // Get the position of the robot in the circle space
    /*Eigen::Matrix<float, 10, 1> point_circle_space = get_point_circle_frame( distances_surface(0,1), distances_surface(0,0), gamma_norm_proj(0,0), max_gamma_robot, gamma_norm_proj_attractor(0,0), max_gamma_attractor, distances_surface(0,2));
    float gamma_circle_frame = point_circle_space(0,0);
    State robot_circle_frame = point_circle_space.block(1,0,3,1);
    State attractor_circle_frame = point_circle_space.block(4,0,3,1);
    State ref_vec_circle_frame = point_circle_space.block(7,0,3,1);*/

    float gamma_circle_frame = gamma_norm_proj(0,0);
    State robot_circle_frame; robot_circle_frame << state_robot(0,0),state_robot(1,0), 0;
    State attractor_circle_frame; attractor_circle_frame << state_attractor(0,0),state_attractor(1,0),0;
    State ref_vec_circle_frame; ref_vec_circle_frame << (state_robot(0,0)-state_reference(0,0)), (state_robot(1,0)-state_reference(1,0)), 0;
    ref_vec_circle_frame = ref_vec_circle_frame / (std::sqrt(std::pow(ref_vec_circle_frame(0,0),2) + std::pow(ref_vec_circle_frame(1,0),2)));

    if (closest(0,2)==3) // -1 applied to r_eps_vector instead
    {
        ref_vec_circle_frame = (-1) * ref_vec_circle_frame;
    }

    // Compute attractor function
    f_eps = (attractor_circle_frame - robot_circle_frame);

    // Normalization of the speed to a default desired speed
    f_eps = f_eps / (std::sqrt(std::pow(f_eps(0,0),2) + std::pow(f_eps(1,0),2)));

    // Several steps to get D(epsilon) matrix
    lamb_r = 1 - (1/gamma_circle_frame);
    lamb_e = 1 + (1/gamma_circle_frame);
    D_eps = D_epsilon( lamb_r, lamb_e);

    // Several steps to get E(epsilon) matrix
    Obstacle obs;
    obs << 0, 0, 0, 1, 1, 1, 1, 0, 0, 0; // unit circle [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]
    r_eps_vector = ref_vec_circle_frame;

    /*if (closest(0,2)==3)
    {
        r_eps_vector = (-1) * r_eps_vector;
    }*/

    //gradient_vector = gradient( robot_circle_frame, obs);
    gradient_vector << gamma_norm_proj(1,0),gamma_norm_proj(2,0),0;
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
    //velocity_shape_space.block(0,0,2,1) = shape_tranform * circle_tranform * velocity_circle_space.block(0,0,2,1);
    velocity_shape_space.block(0,0,2,1) = velocity_circle_space.block(0,0,2,1);

    if (closest(0,2)==3)
    {
        r_eps_vector = (-1) * r_eps_vector;
    }

    Eigen::Matrix<float, 4, 1> output;
    output.block(0,0,3,1) = velocity_shape_space;
    output(3,0) = gamma_norm_proj(0,0);
    return output;
}
