#include <iostream>
#include "KalmanFilter.h"
using namespace std;

/* Constructor: */
KalmanFilter::KalmanFilter(int _n,  int _m) {
	n = _n;
	m = _m;
}

/* Set Fixed Matrix */
void KalmanFilter::setFixed( MatrixXf _A, MatrixXf _H, MatrixXf _Q, MatrixXf _R ){
	A = _A;
	H = _H;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

/* Set Fixed Matrix */
void KalmanFilter::setFixed( MatrixXf _A, MatrixXf _H, MatrixXf _Q, MatrixXf _R, MatrixXf _B ){
	A = _A;
	B = _B;
	H = _H;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

/* Set Initial Matrix */
void KalmanFilter::setInitial( VectorXf _X0, MatrixXf _P0 ){
	X0 = _X0;
	P0 = _P0;
}

/* Do prediction based of physical system (No external input)
*/
void KalmanFilter::predict(void){
  X = (A * X0);
  P = (A * P0 * A.transpose()) + Q;
}

/* Do prediction based of physical system (with external input)
* U: Control vector
*/
void KalmanFilter::predict( VectorXf U ){
  X = (A * X0) + (B * U);
  P = (A * P0 * A.transpose()) + Q;
}

/* Correct the prediction, using mesaurement
*  Z: mesaure vector
*/
void KalmanFilter::correct ( VectorXf Z ) {
  K = ( P * H.transpose() ) * ( H * P * H.transpose() + R).inverse();

  X = X + K*(Z - H * X);

  P = (I - K * H) * P;

  X0 = X;
  P0 = P;
}

void KalmanFilter::correct (void) {
  //K = ( P * H.transpose() ) * ( H * P * H.transpose() + R).inverse();

  //X = X + K*(Z - H * X);

  //P = (I - K * H) * P;

  X0 = X;
  P0 = P;
}


