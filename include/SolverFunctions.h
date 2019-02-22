#ifndef SOLVERFUNCTIONS_H_INCLUDED
#define SOLVERFUNCTIONS_H_INCLUDED

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <Eigen> // For Linear Algebra

// To solve Non Linear Optimization problems
#include <unsupported/Eigen/NonLinearOptimization>

// tolerance for checking number of iterations
#define LM_EVAL_COUNT_TOL 4/3

Eigen::Matrix<float, 3, 1> findSurfacePointEllipse(Eigen::Matrix<float,9,1> params_ellipse);

#endif // SOLVERFUNCTIONS_H_INCLUDED
