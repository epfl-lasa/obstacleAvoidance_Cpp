#include "SolverFunctions.h"

using namespace Eigen;
using namespace std;

// Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  const int m_inputs, m_values;

  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

  // you should define that in the subclass :
//  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};

struct hybrd_functor : Functor<double>
{
    Matrix<float,11,1> m_params;
    //hybrd_functor(void) : Functor<double>(9,9) {}
    //hybrd_functor(void) : Functor<double>(1,1) {}
    hybrd_functor(Matrix<float,11,1> params) : Functor<double>(1,1) { this->m_params = params;}
    int operator()(const VectorXd &x, VectorXd &fvec) const
    {
        //cout << "Params = " << (this->m_params).transpose() << endl;
        //double temp, temp1, temp2;
        const VectorXd::Index n = x.size();

        assert(fvec.size()==n);
        for (VectorXd::Index k=0; k < n; k++)
        {
            /*temp = (3. - 2.*x[k])*x[k];
            temp1 = 0.;
            if (k) temp1 = x[k-1];
            temp2 = 0.;
            if (k != n-1) temp2 = x[k+1];
            fvec[k] = temp - temp1 - 2.*temp2 + 1.;*/

            //fvec[k] = 1 * x[k] - 2;

            // fvec[k] = std::pow(x[k]/1,2*1) + std::pow((1*x[k]+0)/1,2*1) - 1;

            double y = m_params(7,0) * x[k] + m_params(8,0);
            fvec[k] = (std::pow( (x[k]-m_params(0,0))*std::cos(m_params(2,0)) + (y-m_params(1,0))*std::sin(m_params(2,0)) / pow(m_params(3,0),2), 2 * m_params(5,0))) + (std::pow( (x[k]-m_params(0,0))*std::sin(m_params(2,0)) - (y-m_params(1,0))*std::cos(m_params(2,0)) / pow(m_params(4,0),2), 2 * m_params(6,0))) - 1;
        }
        return 0;
    }
};

Eigen::Matrix<float, 3, 1> findSurfacePointEllipse(Matrix<float,11,1> params)
{
  //float param_test = 4.65;

  //int n=9, info;
  int n=1, info;
  VectorXd x(n);

  /* the following starting values provide a rough solution. */
  x.setConstant(n, params(9,0)); // start search from robot position

  // do the computation
  hybrd_functor functor(params);
  HybridNonLinearSolver<hybrd_functor> solver(functor);
  info = solver.hybrd1(x);

  if (info == 1)
  {
      // cout << "Solver success! X=" << x << endl;
  }
  else
  {
      cout << "Solver failure." << endl;
      throw 42;
  }

  Eigen::Matrix<float, 3, 1> state_on_surface;
   // (x,y,phi) but phi don't matter for the distance so it can be set to 0
   // y = a * x + b
  state_on_surface(0,0) = x(0);
  state_on_surface(1,0) = params(7,0) * x(0) + params(8,0);
  state_on_surface(2,0) = 0;
  return state_on_surface;

  /*// check return value
  VERIFY_IS_EQUAL(info, 1);
  VERIFY_IS_EQUAL(solver.nfev, 20);

  // check norm
  VERIFY_IS_APPROX(solver.fvec.blueNorm(), 1.192636e-08);

  // check x
  VectorXd x_ref(n);
  x_ref << -0.5706545, -0.6816283, -0.7017325, -0.7042129, -0.701369, -0.6918656, -0.665792, -0.5960342, -0.4164121;
  VERIFY_IS_APPROX(x, x_ref);*/
}
