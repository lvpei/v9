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

double fx2(double t, void* param)
{
	double kappa = 0.0;

	gsl_spline_pointer* gsp = (gsl_spline_pointer*)param;

	kappa = gsl_spline_eval(gsp->spline_x, t, gsp->acc);

	return abs(kappa);
}

double fx3(double t, void* param)
{
	gsl_spline_pointer* gsp = (gsl_spline_pointer*)param;

	double new_dx = gsl_spline_eval_deriv(gsp->spline_x,t,gsp->acc);
	double new_dx2 = gsl_spline_eval_deriv2(gsp->spline_x,t,gsp->acc);

	double new_dy = gsl_spline_eval_deriv(gsp->spline_y,t,gsp->acc);
	double new_dy2 = gsl_spline_eval_deriv2(gsp->spline_y,t,gsp->acc);

	double kappa = (new_dx * new_dy2 - new_dx2 * new_dy)/sqrt(pow(new_dx * new_dx + new_dy * new_dy,3.0));

	return abs(kappa);
}

// bisection search to find appropriate point
void bisectionSearch(const gsl_function& gf, double low_length, double length, double start, double end, double* value)
{
	double r, er;
	unsigned int n;
	double acc_length = low_length;

	double mid = (start + end) / 2;

	gsl_integration_qng(&gf,start,mid,1e-10,1e-10,&r,&er,&n);

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
double extractCurveFeature(double* x, double* y, int point_num, int feature_num, double*& feature,int* actual_feature_num, double base_length)
{
	gsl_integration_workspace* w = gsl_integration_workspace_alloc(20);

	/************************************************************************/
	/* Construct the traditional cubic spline curve                         */
	/************************************************************************/
	gsl_spline_pointer* gsp = new gsl_spline_pointer();
	int param_num = point_num;
	gsp->param_num = param_num;
	gsp->params = new double[param_num];

	// compute the parameters
	computeParameters(x,y,gsp->params,param_num);
	
	/*
	printf("original params:\n");
	for(int i = 0; i < param_num; i++)
		printf("%g ",gsp->params[i]);
	printf("\n");
	//*/

	// construct the traditional spline curve
	constructBSpline(gsp,x,y,param_num);

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

		//gsl_integration_qng(&gf,start,end,0.1,0.1,&r,&er,&n);
		
		gsl_integration_qag(&gf,start,end,1e-10,1e-10,20,GSL_INTEG_GAUSS41,w,&r,&er);			

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

			//gsl_integration_qng(&gf,start,end,0.1,0.1,&r,&er,&n);
			
			gsl_integration_qag(&gf,start,end,1e-10,1e-10,20,GSL_INTEG_GAUSS41,w,&r,&er);			

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
	
	/*
	printf("new params:\n");
	for(int i = 0; i < m; i++)
	{
		printf("%g ",new_params[i]);
	}
	printf("\n");
	//*/

	// compute m equally spaced point
	double* new_data_x = new double[m];
	double* new_data_y = new double[m];

	computeNewPointsByArcParameters(gsp,m,new_params,new_data_x,new_data_y);

	/*
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
	//*/

	// re-parameterizing the traditional spline by arc-length parameter
	gsl_spline_pointer* gsp2 = new gsl_spline_pointer();
	gsp2->param_num = m ;
	gsp2->params = new double[gsp2->param_num];

	for(int i = 0; i < m; i++)
		gsp2->params[i] = i * sub_length;

	// spline parameterized by arc-length
	constructBSpline(gsp2,new_data_x,new_data_y,m);
	
	/*
	printf("new sampling data points:\n");
	for(int i = 0; i < m - 1; i++)
	{
		double param = i * length / (m - 1);
		double new_x = gsl_spline_eval(gsp2->spline_x,param,gsp2->acc);
		double new_y = gsl_spline_eval(gsp2->spline_y,param,gsp2->acc);

		printf("%g %g\n",new_x,new_y);
	}
	printf("\n");
	//*/


	/************************************************************************/
	/* compute the curvature of the spline based on arc-length parameter 
	/************************************************************************/
	
	// for test
	/*
	m = 40;

	double* curvature = new double[m];	
	
	printf("compute curvature of arc-length based spline curve:\n");
	for(int i = 0; i < m - 1; i++)
	{
		double param = i * length / (m - 1);
		
		double new_dx = gsl_spline_eval_deriv(gsp2->spline_x,param,gsp2->acc);
		double new_dx2 = gsl_spline_eval_deriv2(gsp2->spline_x,param,gsp2->acc);

		double new_dy = gsl_spline_eval_deriv(gsp2->spline_y,param,gsp2->acc);
		double new_dy2 = gsl_spline_eval_deriv2(gsp2->spline_y,param,gsp2->acc);

		double kappa = (new_dx * new_dy2 - new_dx2 * new_dy)/sqrt(pow(new_dx * new_dx + new_dy * new_dy,3.0));
	
		curvature[i] = kappa;

		printf("%g ",kappa);
	}
	printf("\n");
	//*/

	/************************************************************************/
	/* Integrate the unsigned curvatures along the curve by summing the 
	   absolute values of curvatures discretely sampled along the curve
	/************************************************************************/
	gsl_function gf2;
	gf2.function = fx3;
	gf2.params = (void*)gsp2;

	double* integ = new double[gsp2->param_num];
	double length2 = 0.0;
	integ[0] = length2;
	for(int i = 1; i < gsp2->param_num; i++)
	{
		start = gsp2->params[i-1];
		end = gsp2->params[i];

		//gsl_integration_qng(&gf2,start,end,.1,.1,&r,&er,&n);

		gsl_integration_qag(&gf2,start,end,1e-10,1e-10,20,GSL_INTEG_GAUSS41,w,&r,&er);			

		length2 += r;
		integ[i] = length2;
	}
	
	/*
	printf("integral of curvature:\n");
	ofstream out_integral("integral.txt");
	ofstream out_params("params.txt");
	for(int i = 0; i < gsp2->param_num; i++)
	{
		printf("%g\n",integ[i]);
		out_integral<<integ[i]<<" ";
		out_params<<gsp2->params[i]<<" ";
	}
	printf("\n");
	//*/

	double curvature_curve_length = length2;

	printf("integral value = %g\n",curvature_curve_length);

	/************************************************************************/
	/* compute curvature of the curve at equal interval-sampled points along
	   the integral of unsigned curvature axis. (signed curvature w.r.t the 
	   integral of unsigned curvature)	
	/************************************************************************/
	gsl_spline_pointer* gsp3 = new gsl_spline_pointer();
	gsp3->param_num = gsp2->param_num;
	gsp3->params = new double[gsp3->param_num];

	double* arc_length_data = new double[gsp3->param_num];
	for(int i = 0; i < gsp3->param_num; i++)
	{	
		arc_length_data[i] = gsp2->params[i];
		gsp3->params[i] = integ[i];
	}

	// integral-arc spline parameterized by integral value
	constructBSpline(gsp3,arc_length_data,NULL,gsp3->param_num);

	/*
	printf("new sampling data points from integral curve wrt integral value:\n");
	ofstream out_arclength("new_arclength.txt");
	for(int i = 0; i < gsp3->param_num; i++)
	{
		double arclength = gsl_spline_eval(gsp3->spline_x,integ[i],gsp3->acc);

		printf("%g\n",arclength);
		out_arclength<<arclength<<" ";
	}
	printf("\n");
	//*/
	
	/*
	printf("curvature from integral curve wrt integral value:\n");
	ofstream out_curvature2("curvature2.txt");
	ofstream out_params2("params2.txt");

	for(int i = 0; i < gsp3->param_num; i++)
	{
		double t = i * integ[gsp3->param_num-1]/gsp3->param_num;

		double new_dx = gsl_spline_eval_deriv(gsp3->spline_x,t,gsp3->acc);
		double new_dx2 = gsl_spline_eval_deriv2(gsp3->spline_x,t,gsp3->acc);

		double kappa = new_dx2 / sqrt(pow(1.0 + new_dx * new_dx,3.0));

		printf("%g ",kappa);
		out_curvature2<<kappa<<",";
		out_params2<<t<<",";
	}
	printf("\n");
	//*/


	// extract the curve feature
	/*
	double sub_curve_length = 0.0;
	int seg_num = 0;
	if(abs(base_length - 1.0) > 0.00001)
	{
		sub_curve_length = base_length / (feature_num - 1);
		seg_num = int(curvature_curve_length / sub_curve_length);
	}
	else
	{	
		sub_curve_length = curvature_curve_length / (feature_num - 1);
		seg_num = feature_num;
	}

	printf("invariant curve feature:\n");
	
	if(seg_num >= feature_num) 	// the target curve is longer than template curve
	{
		for(int i = 0; i < feature_num - 1; i++)
		{
			double t = i * sub_curve_length;

			double new_dx = gsl_spline_eval_deriv(gsp3->spline_x,t,gsp3->acc);
			double new_dx2 = gsl_spline_eval_deriv2(gsp3->spline_x,t,gsp3->acc);

			double kappa = new_dx2 / sqrt(pow(1.0 + new_dx * new_dx,3.0));

			feature[i] = kappa;
			printf("%g ",feature[i]);
		}
		printf("\n");

		(*actual_feature_num) = feature_num;
	}
	else						// the target curve is shorter than template curve
	{
		int i = 0;
		for(i = 0; i < seg_num; i++)
		{
			double t = i * sub_curve_length;

			double new_dx = gsl_spline_eval_deriv(gsp3->spline_x,t,gsp3->acc);
			double new_dx2 = gsl_spline_eval_deriv2(gsp3->spline_x,t,gsp3->acc);

			double kappa = new_dx2 / sqrt(pow(1.0 + new_dx * new_dx,3.0));

			feature[i] = kappa;
			printf("%g ",feature[i]);
		}
		printf("\n");

		(*actual_feature_num) = seg_num;
	}
	//*/
	
	double sub_curve_length = 0.5;
	int seg_num = 0;
	seg_num = (int)(curvature_curve_length / sub_curve_length);
	feature = new double[seg_num];

	for(int i = 0; i < seg_num; i++)
	{
		double t = i * sub_curve_length;

		double new_dx = gsl_spline_eval_deriv(gsp3->spline_x,t,gsp3->acc);
		double new_dx2 = gsl_spline_eval_deriv2(gsp3->spline_x,t,gsp3->acc);
		
		double kappa = new_dx2 / sqrt(pow(1.0 + new_dx * new_dx,3.0));

		feature[i] = kappa;
		printf("%g ",feature[i]);
	}
	printf("\n");

	(*actual_feature_num) = seg_num;

	delete arc_length_data;
	delete integ;
	//delete curvature;
	delete new_data_x;
	delete new_data_y;

	delete gsp;
	delete gsp2;
	delete gsp3;
	delete new_params;

	gsl_integration_workspace_free(w);

	return curvature_curve_length;
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
