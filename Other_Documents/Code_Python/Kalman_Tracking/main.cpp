
/*
*   Kalmand Filter Monodimensional Example
*
*   Implementation code of the example showed here:
*
*     http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies
*
*   It simulate a noisy voltage reading from a constant source.
*
*   by Fabio Carbone, 23/12/2016
*   www.fabiocarbone.org
*/



#include <iostream>
#include "KalmanFilter.h"
#include <fstream>  // To write data into files

using namespace std;

int main_example(int argc, char const *argv[])
{

  /* Set Matrix and Vector for Kalman Filter: */
  MatrixXf A(1, 1); A << 1;
  MatrixXf H(1, 1); H << 1;
  MatrixXf Q(1, 1); Q << 0;
  MatrixXf R(1, 1); R << 0.1;
  VectorXf X0(1); X0 << 0;
  MatrixXf P0(1, 1); P0 << 1;

  /* Create The Filter */
  KalmanFilter filter1(1, 0);

  /* Initialize the Filter*/
  filter1.setFixed(A, H, Q, R);
  filter1.setInitial(X0, P0);

  /* Create measure vector, and store measure value */
  VectorXf Z(1);
  float mesaure[10] = {0.39, 0.50, 0.48, 0.29, 0.25, 0.32, 0.34, 0.48, 0.41, 0.45};

  /* This loop simulate the measure/prediction process */
  for (int i = 0; i < 10; ++i)
  {

    filter1.predict(); //Predict phase
    Z << mesaure[i];
    filter1.correct( Z ); //Correction phase

    cout << "X" << i << ": " << filter1.X << endl;

  }

	return 0;
}

int main(int argc, char const *argv[])
{
  /* Set the parameters (standard deviations) */
  float sigma_1 = 5;
  float sigma_2 = 5;
  float sigma_vx =  5;
  float sigma_vy =  5;
  float sigma_0 = 50;
  float h = 0.125f;

  /* Set Matrix and Vector for Kalman Filter: */
  const int n = 4;
  MatrixXf A(n, n); A << 1, 0, h, 0,
                         0, 1, 0, h,
                         0, 0, 1, 0,
                         0, 0, 0, 1;

  MatrixXf H(n, n); H << 1, 0, 0, 0,
                         0, 1, 0, 0,
                         0, 0, 1, 0,
                         0, 0, 0, 1;

  MatrixXf Q(n, n); Q << std::pow(h*sigma_vx,2),          0,        0,        0,
                                  0, std::pow(h*sigma_vy,2),        0,        0,
                                  0,          0, std::pow(sigma_vx,2),        0,
                                  0,          0,        0, std::pow(sigma_vy,2);

  MatrixXf R(n, n); R << std::pow(sigma_1,2),       0,         0,         0,
                                           0, std::pow(sigma_2,2),        0,         0,
                                           0,       0, std::pow(sigma_1/h,2),        0,
                                           0,       0,         0,  std::pow(sigma_2/h,2);
  VectorXf X0(n); X0 << 100,100,0,0;
  MatrixXf P0(n, n); P0 << 0, 0,       0,       0,
                           0, 0,       0,       0,
                           0, 0, std::pow(sigma_0,2),       0,
                           0, 0,       0, std::pow(sigma_0,2);

  /* Create The Filter */
  KalmanFilter filter1(n, 0);

  /* Initialize the Filter*/
  filter1.setFixed(A, H, Q, R);
  filter1.setInitial(X0, P0);

  /* Create measure vector, and store measure value */
    VectorXf Z(4);
    Eigen::MatrixXf measures(64,4);

    std::ifstream infile("data_from_python.txt");
    float a, b, c, d;
    int counter = 0;
    while (infile >> a >> b >> c >> d)
    {
        measures.row(counter) << a, b, c, d;
        counter += 1;
    }

    //std::cout << measures << endl;

    /* For saving data */
    std::ofstream my_corrected;
    std::ofstream my_predicted;
    my_corrected.open("Data_Kalman_Corrected.txt");
    my_predicted.open("Data_Kalman_Predicted.txt");
    //myfile << X0[0] << "," << X0[1] << "," << X0[2] << "," << X0[3] << "\n";

    std::srand(42);

    /* This loop simulate the measure/prediction process */
    for (int i = 0; i < measures.rows(); ++i)
    {

        filter1.predict(); //Predict phase
        my_predicted << (filter1.X)[0] << "," << (filter1.X)[1] << "," << (filter1.X)[2] << "," << (filter1.X)[3] << "\n";

        Z = (measures.row(i)).transpose();
        //cout << "Z" << i << ": " << Z << endl;

        int random_variable = std::rand();
        if (true)//((random_variable%4) != 0)//((i<40)||((i>=43)&&(i<45))||(i>=48))
        {
            filter1.correct( Z ); //Correction phase
            my_corrected << (filter1.X)[0] << "," << (filter1.X)[1] << "," << (filter1.X)[2] << "," << (filter1.X)[3] << "\n";
        }
        else
        {
            filter1.correct(); //Correction phase
            my_corrected << (filter1.X)[0] << "," << (filter1.X)[1] << "," << (filter1.X)[2] << "," << (filter1.X)[3] << "\n";
            cout << "Predicted: " << (filter1.X).transpose() << endl;
        }

        //filter1.correct( Z ); //Correction phase
        //cout << "X" << i << ": " << (filter1.X).transpose() << endl;
        //my_corrected << (filter1.X)[0] << "," << (filter1.X)[1] << "," << (filter1.X)[2] << "," << (filter1.X)[3] << "\n";

    }
    my_corrected.close();
    my_predicted.close();
    return 0;
}
