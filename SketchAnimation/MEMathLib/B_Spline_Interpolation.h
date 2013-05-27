#pragma once

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <cmath>
#include <assert.h>
#include <vector>
#include <iostream>

#include "Point3d.h"

using namespace std;

namespace MathLib
{

const int B_SPLINE_ORDER = 3;	// 三次B样条插值
const int MAX_POINT_NUM = 50;	// 两个关键帧之间数据帧的数量，此处规定最大为50。可根据需要修改
const double DOUBLE_ZERO = 0.000;

class B_Spline_Interpolation
{
public:
	B_Spline_Interpolation(void);
	virtual ~B_Spline_Interpolation(void);

	int n,p;						// n数据个数 p阶数
	vector<Point3d> data_point;		// 数据点集合
	vector<Point3d> ctrl_point;		// 控制点集合
	gsl_matrix *x;
	double *knot;					// Knot vector
	double *parameter;				// parameter
	double **N;						// N(n+1)(n+1), basis function
	double **D;						// D(n+1)(3)
	double **P;						// P(n+1)(3)

	void Set_datapoint(vector<Point3d> &point_d);
	void Set_order(int n_p);
	void Interpolation();
	void Uniform_method();
	void Chord_method();
	void Centrip_method(double alpha);
	void Univer_method();
	double Calculate_distance(Point3d & p3d1, Point3d & p3d2, double alpha);
	void Calculate_N();
	void basis(double t,int row);
	//solve a linear equation
	void l_solve(gsl_matrix* A, gsl_vector* x, gsl_vector * b);
};
}