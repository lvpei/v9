#include "curve_feature.h"

using namespace std;

const int MAX_POINT_NUM = 200;

// compute the parameters for B-spline
void computeParameters(const double* data_x, const double* data_y, double* parameters, int param_num)
{
	double L = 0;
	double L_k[MAX_POINT_NUM];
	L_k[0] = 0;
	int i;
	for (i = 1; i < param_num; i++)
	{
		L_k[i] = L_k[i-1] + sqrt((data_x[i] - data_x[i-1]) * (data_x[i] - data_x[i-1]) + (data_y[i] - data_y[i-1]) * (data_y[i] - data_y[i-1]));
	}
	L = L_k[param_num - 1];
	
	parameters[0] = 0;
	for (i = 1; i < param_num - 1; i++)
	{
		parameters[i] = L_k[i]/L;
	}
	parameters[param_num - 1] = 1.0;
}

// construct the B-Spline curve
void constructBSpline(gsl_spline_pointer* gsp, double* x, double* y, int param_num)
{	
	gsp->acc = gsl_interp_accel_alloc ();

	// for x & y variable
	if(x)
	{
		gsp->spline_x = gsl_spline_alloc (gsl_interp_cspline, param_num);
		gsl_spline_init (gsp->spline_x, gsp->params, x, param_num);
	}
	if(y)
	{
		gsp->spline_y = gsl_spline_alloc (gsl_interp_cspline, param_num);
		gsl_spline_init (gsp->spline_y, gsp->params, y, param_num);
	}
}

double fx(double t, void* param)
{
	double xd = 0.0, yd = 0.0;

	gsl_spline_pointer* gsp = (gsl_spline_pointer*)param;

	xd = gsl_spline_eval_deriv(gsp->spline_x, t, gsp->acc);
	yd = gsl_spline_eval_deriv(gsp->spline_y, t, gsp->acc);

	return sqrt(xd * xd + yd * yd);
}

// bisection search to find appropriate point
void bisectionSearch(const gsl_function& gf, double low_length, double length, double start, double end, double* value)
{
	double r, er;
	unsigned int n;
	double acc_length = low_length;

	double mid = (start + end) / 2;

	gsl_integration_qng(&gf,start,mid,0.1,0.1,&r,&er,&n);

	if(abs(low_length + r - length) < 0.001)
		(*value) = mid;
	else if(low_length + r - length < 0.001)
	{
		bisectionSearch(gf,low_length + r, length, mid, end, value);
	}
	else if(low_length + r - length > 0.001)
	{
		bisectionSearch(gf,low_length, length, start, mid, value);
	}
}

// compute data points according to new parameters
void computeNewPointsByArcParameters(const gsl_spline_pointer* gsp, int param_num, const double* params, double* data_x, double* data_y)
{
	for(int i = 0; i < param_num; i++)
	{
		data_x[i] = gsl_spline_eval(gsp->spline_x,params[i],gsp->acc);
		data_y[i] = gsl_spline_eval(gsp->spline_y,params[i],gsp->acc);
	}
}

// extract fixed number of feature points from curve
void extractCurveFeature(double* x, double* y, int point_num, int feature_num, double*& feature)
{
	/************************************************************************/
	/* Construct the traditional cubic spline curve                         */
	/************************************************************************/
	gsl_spline_pointer* gsp = new gsl_spline_pointer();
	int param_num = point_num;
	gsp->param_num = param_num;
	gsp->params = new double[param_num];

	// compute the parameters
	computeParameters(x,y,gsp->params,param_num);
	
	printf("original params:\n");
	for(int i = 0; i < param_num; i++)
		printf("%g ",gsp->params[i]);
	printf("\n");

	// construct the traditional spline curve
	constructBSpline(gsp,x,y,gsp->param_num);

	/************************************************************************/
	/* Construct the cubic spline curve based on arc-length parameter       */
	/************************************************************************/
	gsl_function gf;
	gf.function = fx;
	gf.params = (void*)gsp;

	// compute curve length
	double r, er;
	unsigned int n;
	double start, end;
	double length = 0.0;
	for(int i = 1; i < gsp->param_num; i++)
	{
		start = gsp->params[i-1];
		end = gsp->params[i];

		gsl_integration_qng(&gf,start,end,0.1,0.1,&r,&er,&n);

		length += r;
	}
	printf("the total arc length = %g\n",length);
	
	// compute m parameters which can generate equally arc-length segment
	int m = 40;
	double sub_length = length / (m - 1);
	double acc_length = 0.0;
	double* new_params = new double[m];
	new_params[0] = 0.0;

	for(int j = 1; j < m; j++)
	{
		double find_length = j * sub_length;
		
		acc_length = 0.0;
		
		for(int i = 1; i < gsp->param_num; i++)
		{
			start = gsp->params[i-1];
			end = gsp->params[i];

			gsl_integration_qng(&gf,start,end,0.1,0.1,&r,&er,&n);
			
			if(find_length - acc_length >= 0.0001 && acc_length + r - find_length > 0.0001)
			{
				acc_length += r;
				break;
			}
			else
				acc_length += r;
		} 

		bisectionSearch(gf, acc_length - r, find_length, start, end, new_params + j);
	}
	
	printf("new params:\n");
	for(int i = 0; i < m; i++)
	{
		printf("%g ",new_params[i]);
	}
	printf("\n");

	// compute m equally spaced point
	double* new_data_x = new double[m];
	double* new_data_y = new double[m];

	computeNewPointsByArcParameters(gsp,m,new_params,new_data_x,new_data_y);

	printf("new data points:\n");
	ofstream out_x("x.txt");
	ofstream out_y("y.txt");
	for(int i = 0; i < m; i++)
	{
		out_x<<new_data_x[i]<<" ";
		out_y<<new_data_y[i]<<" ";

		printf("%g %g\n",new_data_x[i],new_data_y[i]);
	}
	printf("\n");

	// re-parameterizing the traditional spline by arc-length parameter
	gsl_spline_pointer* gsp2 = new gsl_spline_pointer();
	gsp2->param_num = m;
	gsp2->params = new double[gsp2->param_num];

	for(int i = 0; i < gsp2->param_num; i++)
		gsp2->params[i] = i * sub_length;

	// spline parameterized by arc-length
	constructBSpline(gsp2,new_data_x,new_data_y,gsp2->param_num);
		
	feature = new double[feature_num * 3];

	printf("the first derivative for x & y and curvature:\n");
	for(int i = 0; i < feature_num - 1; i++)
	{
		double t =  i * length / (feature_num - 1);

		double new_dx = gsl_spline_eval_deriv(gsp2->spline_x,t,gsp->acc);
		double new_dx2 = gsl_spline_eval_deriv2(gsp2->spline_x,t,gsp->acc);

		double new_dy = gsl_spline_eval_deriv(gsp2->spline_y,t,gsp->acc);
		double new_dy2 = gsl_spline_eval_deriv2(gsp2->spline_y,t,gsp->acc);

		double kappa = (new_dx * new_dy2 - new_dx2 * new_dy)/sqrt(pow(new_dx * new_dx + new_dy * new_dy,3.0));

		feature[i*3]   = new_dx;
		feature[i*3+1] = new_dy;
		feature[i*3+2] = kappa;

		printf("%g %g %g\n",new_dx,new_dy,kappa);
	}
	printf("\n");

	delete new_data_x;
	delete new_data_y;

	delete gsp;
	delete gsp2;
	delete new_params;
}


// compare two feature
double compareCurveFeature(int feature_num, double* feature1, double* feature2)
{
	// using cross correlation to compare the feature between user's sketching motion curve and that from database
	double dist = 0.0;
	float mean_sketch = 0.0, mean_db = 0.0;
	for(int j = 0; j < feature_num; j++)
	{
		mean_sketch += feature1[j];
		mean_db += feature2[j];
	}
	mean_sketch = mean_sketch / feature_num;
	mean_db = mean_db / feature_num;

	double value1 = 0.0, value2 = 0.0, value3 = 0.0;
	for(int j = 0; j < feature_num; j++)
	{
		value1 += (feature1[j] - mean_sketch) * (feature2[j] - mean_db);

		value2 += (feature1[j] - mean_sketch) * (feature1[j] - mean_sketch);

		value3 += (feature2[j] - mean_db) * (feature2[j] - mean_db);
	}

	dist = value1 / sqrt(value2 * value3);

	return dist;
}
