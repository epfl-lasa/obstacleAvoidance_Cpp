#include "BezierInterpolation.h"

#include <eigen3/Eigen/LU>

std::vector<Eigen::MatrixXf> compute_bezier(Eigen::MatrixXf const& XY)
{
    /* XY is a matrix with the following format
    [x1,y1,
     x2,y2,
     ..,..
     xn,yn]*/

    int nb_points = XY.rows();

    Eigen::MatrixXf A = XY;

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

    Eigen::MatrixXf P = mat.inverse() * A;

    P.conservativeResize(P.rows()+1, Eigen::NoChange);
    P.row(P.rows()-1) = P.row(0);
    A.conservativeResize(A.rows()+1, Eigen::NoChange);
    A.row(A.rows()-1) = A.row(0);

    Eigen::MatrixXf B = (2 * P.block(0,0,nb_points,2) + P.block(1,0,nb_points,2))/3;
    Eigen::MatrixXf C = (2 * P.block(1,0,nb_points,2) + P.block(0,0,nb_points,2))/3;

    const int nb_step = 101;
    Eigen::MatrixXf t = Eigen::VectorXf::LinSpaced(nb_step,0.0,1.0);
    t.conservativeResize(nb_step-1,Eigen::NoChange);
    Eigen::Matrix<float, nb_step-1, 1> s = Eigen::MatrixXf::Ones(nb_step-1,1) - t;

    Eigen::Matrix<float, nb_step-1, 1> s3 = s.cwiseProduct(s.cwiseProduct(s));
    Eigen::Matrix<float, nb_step-1, 1> s2t = 3.0*s.cwiseProduct(s.cwiseProduct(t));
    Eigen::Matrix<float, nb_step-1, 1> t2s = 3.0*t.cwiseProduct(t.cwiseProduct(s));
    Eigen::Matrix<float, nb_step-1, 1> t3 = t.cwiseProduct(t.cwiseProduct(t));

    Eigen::MatrixXf bezier_pts_X(nb_step-1, nb_points);
    Eigen::MatrixXf bezier_pts_Y(nb_step-1, nb_points);

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

    std::vector<Eigen::MatrixXf> result;
    result.push_back(bezier_pts_X);
    result.push_back(bezier_pts_Y);
    return result;
}


Eigen::MatrixXf border_to_vertices(Border const& obs)
{
    Eigen::MatrixXf result = Eigen::MatrixXf(obs.rows(),2);
    int pt_row = 0;

    std::cout << "PASS" << std::endl;
    std::cout << obs.row(obs.rows()-1) << std::endl;
    std::cout << obs.row(0) << std::endl;

    Eigen::MatrixXf obstacle(obs.rows()+2,5);
    obstacle << obs.row(obs.rows()-1), obs, obs.row(0);

    std::cout << "PASS" << std::endl;

    for (int k=1; k<obstacle.rows()-1; k++)
    {
        if (obstacle(k,2)==1)
        {
            if (obstacle(k,3)==1)
            {
                result.row(pt_row) << obstacle(k,0),  obstacle(k,1)+(1*0.5);
            }
            else if (obstacle(k,3)==(-1))
            {
                result.row(pt_row) << obstacle(k,0),  obstacle(k,1)-(1*0.5);
            }
            else if (obstacle(k,4)==(+1))
            {
                result.row(pt_row) << obstacle(k,0)-(1*0.5),  obstacle(k,1);
            }
            else if (obstacle(k,4)==(-1))
            {
                result.row(pt_row) << obstacle(k,0)+(1*0.5),  obstacle(k,1);
            }
            else
            {
                throw std::invalid_argument("Should not happen. Invalid line cell");
            }
            pt_row += 1;

        }
        else if (obstacle(k,2)==2)
        {
            if (obstacle(k,4)==0)
            {
                result.row(pt_row) << obstacle(k,0)-(1*0.5),  obstacle(k,1);
            }
            else if (obstacle(k,4)==1)
            {
                result.row(pt_row) << obstacle(k,0),  obstacle(k,1)-(1*0.5);
            }
            else if (obstacle(k,4)==2)
            {
                result.row(pt_row) << obstacle(k,0)+(1*0.5),  obstacle(k,1);
            }
            else if (obstacle(k,4)==3)
            {
                result.row(pt_row) << obstacle(k,0),  obstacle(k,1)+(1*0.5);
            }
            else
            {
                throw std::invalid_argument("Should not happen. Invalid circle cell");
            }
            pt_row += 1;
        }
        else if (obstacle(k,2)==3)
        {
            if ((obstacle(k-1,2) == 2) && (obstacle(k+1,2) != 2))
            {
                if (pt_row>0) {pt_row -= 1;}
            }

            if ((obstacle(k-1,2) == 1) && (obstacle(k+1,2) != 3))
            {
                if (pt_row>0) {pt_row -= 1;}
            }

            if ((obstacle(k-1,2) == 3) && (obstacle(k+1,2) == 1))
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
    if (index_prev_min < 0) {index_prev_min = dist.cols()-1;}

    robot_X = state_robot(0,0) * Eigen::MatrixXf::Ones((pts_bezier[0]).rows(),1);
    robot_Y = state_robot(1,0) * Eigen::MatrixXf::Ones((pts_bezier[0]).rows(),1);

    dist_X = (robot_X-(pts_bezier[0]).col(index_prev_min)).cwiseProduct(robot_X-(pts_bezier[0]).col(index_prev_min));
    dist_Y = (robot_Y-(pts_bezier[1]).col(index_prev_min)).cwiseProduct(robot_Y-(pts_bezier[1]).col(index_prev_min));
    Eigen::MatrixXf dist_1 = (dist_X+dist_Y).array().sqrt().matrix();

    dist_X = (robot_X-(pts_bezier[0]).col(index_min)).cwiseProduct(robot_X-(pts_bezier[0]).col(index_min));
    dist_Y = (robot_Y-(pts_bezier[1]).col(index_min)).cwiseProduct(robot_Y-(pts_bezier[1]).col(index_min));
    Eigen::MatrixXf dist_2 = (dist_X+dist_Y).array().sqrt().matrix();

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

    float closest_X, closest_Y;
    if (i_dist==1)
    {
        closest_X = (pts_bezier[0])(index_min,index_prev_min);
        closest_Y = (pts_bezier[1])(index_min,index_prev_min);
    }
    else
    {
        closest_X = (pts_bezier[0])(index_min,index_min);
        closest_Y = (pts_bezier[1])(index_min,index_min);
    }

    State closest; closest << closest_X, closest_Y, 0;

    return closest;
}


