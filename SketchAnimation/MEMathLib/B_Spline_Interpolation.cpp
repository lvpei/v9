#include "B_Spline_Interpolation.h"

namespace MathLib
{

B_Spline_Interpolation::B_Spline_Interpolation(void)
{
	int i = 0;
	x = NULL;
	data_point.clear();
	ctrl_point.clear();
	knot = new double[MAX_POINT_NUM];
	parameter = new double[MAX_POINT_NUM];
	D = new double*[MAX_POINT_NUM];
	P = new double*[MAX_POINT_NUM];
	N = new double*[MAX_POINT_NUM];
	for (i = 0; i < MAX_POINT_NUM; i++)
	{
		D[i] = new double[3];
		P[i] = new double[3];
		N[i] = new double[MAX_POINT_NUM];
	}
}

B_Spline_Interpolation::~B_Spline_Interpolation(void)
{
	int i = 0;
	if (x != NULL)
	{
		gsl_matrix_free(x);
	}
	delete []knot;
	knot = NULL;
	delete []parameter;
	parameter = NULL;
	for (i = 0; i < MAX_POINT_NUM; i++)
	{
		delete [] D[i];
		delete [] P[i];
		delete [] N[i];
	}
	delete [] D;
	delete [] P;
	delete [] N;
	D = NULL;
	P = NULL;
	N = NULL;
}

void B_Spline_Interpolation::Set_datapoint(vector<Point3d> &point_d)
{
	assert (!point_d.empty());
	n = (int)point_d.size()-1;

	for (int i = 0; i <= n; i++)
	{
		data_point.push_back(point_d[i]);
	}
}

void B_Spline_Interpolation::Set_order(int n_p)
{
	p = n_p;
}

void B_Spline_Interpolation::l_solve(gsl_matrix* A, gsl_vector* x, gsl_vector * b)
{
	if (A->size1 != A->size2||A->size1 != x->size||x->size != b->size)
	{
		cout<<"can not solve the equation."<<endl;
		return;
	}
	int s;
	gsl_permutation *p = gsl_permutation_alloc(x->size);
	gsl_linalg_LU_decomp(A,p,&s);
	gsl_linalg_LU_solve(A,p,b,x);
	gsl_permutation_free(p);
}

void B_Spline_Interpolation::Interpolation()
{
	/*Select a method for computing a set of n+1 parameters t0, ..., tn; 
	As a by-product, we also receive a knot vector U; */

	//Uniform_method();
	//Chord_method();
	Centrip_method(0.0000000000001);
	Calculate_N();
	/* Construct matrix D*/
	for (int i = 0; i <= n; i++)
	{
		D[i][0] = data_point[i].x;
		D[i][1] = data_point[i].y;
		D[i][2] = data_point[i].z;
	}

	/*Use a linear system solver to solve for P from D = N.P*/
	gsl_matrix* A = gsl_matrix_calloc(n+1,n+1);
	gsl_matrix* tmp_A = gsl_matrix_calloc(n+1,n+1);
	x = gsl_matrix_calloc(n+1,3);
	gsl_matrix* b = gsl_matrix_calloc(n+1,3);
	gsl_vector* tmp_x = gsl_vector_calloc(n+1);
	gsl_vector* tmp_b = gsl_vector_calloc(n+1);

	int i,j;
	for (i = 0; i < n+1; i++)
	{
		for(j = 0; j<n+1; j++)
		{
			gsl_matrix_set(A,i,j,N[i][j]);
		}
	}
	for (i = 0; i < n+1; i++)
	{
		for(j = 0; j<3; j++)
		{
			gsl_matrix_set(b,i,j,D[i][j]);
		}
	}
	for (i = 0; i < 3; i++)
	{
		gsl_matrix_memcpy(tmp_A,A);
		gsl_matrix_get_col(tmp_b,b,i);
		l_solve(tmp_A,tmp_x,tmp_b);
		gsl_matrix_set_col(x,i,tmp_x);
	}
	gsl_vector_free(tmp_x);
	gsl_vector_free(tmp_b);
	gsl_matrix_free(A);
	gsl_matrix_free(b);

	for (i = 0; i < n+1; i++)
	{
		for(j = 0; j<3; j++)
		{
			P[i][j] = gsl_matrix_get(x,i,j);
		}
		Point3d tmp_p;
		tmp_p.x = P[i][0];
		tmp_p.y = P[i][1];
		tmp_p.z = P[i][2];
		ctrl_point.push_back(tmp_p);
	}
}

void B_Spline_Interpolation::Uniform_method()
{
	int i;
	for (i = 0; i <= n; i++)
	{
		parameter[i] = 1.0f/n*i;
	}
	for (i = 0; i <= p; i++)
	{
		knot[i] = 0;
	}
	for (i = p+1; i < n+1; i++)
	{
		knot[i] = 1.0f/(n-p+1)*(i-p);
	}
	for (i = n+1; i <= n+p+1; i++)
	{
		knot[i] = 1;
	}
}

void B_Spline_Interpolation::Chord_method()
{
	double L = 0;
	double L_k[MAX_POINT_NUM];
	L_k[0] = 0;
	int i,j;
	for (i = 1; i <= n; i++)
	{
		L_k[i] = L_k[i-1]+Calculate_distance(data_point[i], data_point[i-1], 1.0);
	}
	L = L_k[n];
	parameter[0] = 0;
	////////////////////liling////////////////////////
	if (fabs(L)<DOUBLE_ZERO)
	{
		for (i = 1; i < n; i++)
		{
			parameter[i] = 1.0;	// 先测试为1，如果不行此处改0
		}
	}else{
		for (i = 1; i < n; i++)
		{
			parameter[i] = L_k[i]/L;
		}
	}
	////////////////////liling////////////////////////
	parameter[n] = 1.0;

	for (i = 0; i <= p; i++)
	{
		knot[i] = 0;
	}
	for (i = p+1; i < n+1; i++)
	{
		double tj = 0;
		for (j = i-p; j <= i-1; j++)
		{
			tj+=parameter[j];
		}
		knot[i] = 1.0f/p*tj;
	}
	for (i = n+1; i <= n+p+1; i++)
	{
		knot[i] = 1;
	}
}

void B_Spline_Interpolation::Centrip_method(double alpha)
{
	double L = 0;
	double L_k[MAX_POINT_NUM];
	L_k[0] = 0;
	int i,j;
	for (i = 1; i <= n; i++)
	{
		L_k[i] = L_k[i-1]+Calculate_distance(data_point[i], data_point[i-1], alpha);
	}
	L = L_k[n];
	parameter[0] = 0;
	////////////////////liling////////////////////////
	if (fabs(L)<DOUBLE_ZERO)
	{
		for (i = 1; i < n; i++)
		{
			parameter[i] = 1.0;	// 先测试为1，如果不行此处改0
		}
	}else{
		for (i = 1; i < n; i++)
		{
			parameter[i] = L_k[i]/L;
		}
	}
	////////////////////liling////////////////////////
	parameter[n] = 1.0;

	for (i = 0; i <= p; i++)
	{
		knot[i] = 0;
	}
	for (i = p+1; i < n+1; i++)
	{
		double tj = 0;
		for (j = i-p; j <= i-1; j++)
		{
			tj+=parameter[j];
		}
		knot[i] = 1.0f/p*tj;
	}
	for (i = n+1; i <= n+p+1; i++)
	{
		knot[i] = 1;
	}
}

void B_Spline_Interpolation::Univer_method()
{
	// Useless to achieve it.
}

double B_Spline_Interpolation::Calculate_distance(Point3d & p3d1, Point3d & p3d2, double alpha)
{
	double dis = sqrt((p3d1.x-p3d2.x)*(p3d1.x-p3d2.x)+(p3d1.y-p3d2.y)*(p3d1.y-p3d2.y)+(p3d1.z-p3d2.z)*(p3d1.z-p3d2.z));
	dis = pow(dis, alpha);
	return dis;
}

void B_Spline_Interpolation::Calculate_N()
{ 
	double t;
	int i;
	for (i = 0; i <= n; i++)
	{
		t = parameter[i];
		basis(t,i);
	}
}

void B_Spline_Interpolation::basis(double t,int row)
{
	int m;
	int i,k;
	double d,e;
	m = n + p+1;
	double *temp = new double[m+1];

	/* calculate the first order basis functions n[i][1]	*/

	for (i = 0; i <= m; i++)
	{
		if ((t >= knot[i]) && (t < knot[i+1]))
			temp[i] = 1;
		else
			temp[i] = 0;
	}

	/* calculate the higher order basis functions */

	for (k = 2; k <= p+1; k++)
	{
		for (i = 0; i <= m-k; i++)
		{
			////////////////////liling////////////////////////
			if (temp[i] != 0)    /* if the lower order basis function is zero skip the calculation */
			{
				if (fabs(knot[i+k-1]-knot[i])<DOUBLE_ZERO)
				{
					d = 0;	// 先测试为0，如果不行此处改1
				}else{
					d = ((t-knot[i])*temp[i])/(knot[i+k-1]-knot[i]);
				}
			}else{
				d = 0;
			}

			if (temp[i+1] != 0)     /* if the lower order basis function is zero skip the calculation */
				if (fabs(knot[i+k]-knot[i+1])<DOUBLE_ZERO)
				{
					d = 0;	// 先测试为0，如果不行此处改1
				}else{
					e = ((knot[i+k]-t)*temp[i+1])/(knot[i+k]-knot[i+1]);
				}
			else{
				e = 0;
			}
			temp[i] = d + e;
			////////////////////liling////////////////////////
		}
	}

	if (t == knot[m])
	{	
		temp[n] = 1.0;
	}

	/* put in n array	*/

	for (i = 0; i < n+1; i++) 
	{
		N[row][i] = temp[i];
	}

	delete []temp;
	temp = NULL;
}
}