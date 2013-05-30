/*	This file is used to compute the invariant curve feature by curve 
    matching algorithm presented in 'Curve matching for open 2D curves'
	(Pattern Recognition letter)

	Author: lvp
	Date: 2013-5-26
*/

#ifndef _FEATURE_CURVE_H_
#define _FEATURE_CURVE_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_integration.h>
#include <fstream>

typedef struct _gsl_spline_pointer_{
	int param_num;
	double* params;
	gsl_spline* spline_x;
	gsl_spline* spline_y;
	gsl_interp_accel* acc;
	_gsl_spline_pointer_()
	{
		param_num = 0;
		params = NULL;
		spline_x = NULL;
		spline_y = NULL;
		acc = NULL;
	}
	~_gsl_spline_pointer_()
	{
		clear();
	}
	void clear()
	{
		param_num = 0;
		delete params;
		params = NULL;

		if(spline_x)
		{
			gsl_spline_free (spline_x);
			spline_x = NULL;
		}
		if(spline_y)
		{
			gsl_spline_free (spline_y);
			spline_y = NULL;
		}
		gsl_interp_accel_free (acc);
		acc = NULL;
	}
}gsl_spline_pointer;

// compute the parameters for basis spline according to those data points
void computeParameters(const double* data_x, const double* data_y, double* parameters, int param_num);

// construct the B-Spline curve
void constructBSpline(gsl_spline_pointer* gsp, double* x, double* y, int param_num);

// integrate function
double fx(double t, void* param);
double fx2(double t, void* param);

// bisection search to find appropriate point
void bisectionSearch(const gsl_function& gf, double low_length, double length, double start, double end, double* value);

// compute data points according to new parameters
void computeNewPointsByArcParameters(const gsl_spline_pointer* gsp, int param_num, const double* params, double* data_x, double* data_y);

// extract fixed number of feature points from curve
double extractCurveFeature(double* x, double* y, int point_num, int feature_num, double* feature, double base_length = 1.0);

// compare two feature
double compareCurveFeature(int feature_num, double* feature1, double* feature2);

#endif