#include "Calculate_NUBRS.h"

namespace MathLib
{

double CalculateNt(const double t, const int i, const int k, double *knot)
{
	double Nt = 0.0f;
	if (k <= 1)	// for the first value of the recursion.
	{
		// t between [t[i], t[i+1])
		if (t < knot[i+1] && t >= knot[i])
		{
			Nt = 1;
		}else{
			Nt = 0;
		}
	}
	// for k > 1, start the recursion.
	else if (fabs(knot[i+k-1] - knot[i]) >= DOUBLE_ZERO && fabs(knot[i+k] - knot[i+1]) >= DOUBLE_ZERO)
	{
		Nt = (t - knot[i])/(knot[i+k-1] - knot[i]) * CalculateNt(t, i, k-1, knot)
			+ (knot[i+k] - t)/(knot[i+k] - knot[i+1]) * CalculateNt(t, i+1, k-1, knot);
	}
	// for zero exsists, the zero segment should be cut off.
	else if (fabs(knot[i+k-1] - knot[i]) < DOUBLE_ZERO && fabs(knot[i+k] - knot[i+1]) >= DOUBLE_ZERO)
	{
		Nt = (knot[i+k] - t)/(knot[i+k] - knot[i+1]) * CalculateNt(t, i+1, k-1, knot);
	}
	// for zero exsists, the zero segment should be cut off.
	else if (fabs(knot[i+k-1] - knot[i]) >= DOUBLE_ZERO && fabs(knot[i+k] - knot[i+1]) < DOUBLE_ZERO)
	{
		Nt = (t - knot[i])/(knot[i+k-1] - knot[i]) * CalculateNt(t, i, k-1, knot);
	}else{
		Nt = 0;
	}
	return Nt;
}

Point3d CalculatePt(const double t, const int n, const int k, double **ctrlpoint, double *knot)
{
	assert (ctrlpoint != NULL && knot != NULL && t >= 0 && t <= 1);

	Point3d Pt;

	int i = 0, j = 0;
	for (i = 0; i <= n; i++)	// n+1 values, so <=.
	{
		Point3d P(ctrlpoint[i][0], ctrlpoint[i][1], ctrlpoint[i][2]);
		double tmp = CalculateNt(t, i, k, knot);
		Pt = Pt + P * tmp;
	}
	return Pt;
}

bool VectorAvailable(vector<Point3d> & vecp3d)
{
	vector<Point3d>::iterator iter = vecp3d.begin();
	for (; iter != vecp3d.end(); ++iter)
	{
		// 当向量的Point3d结点不全部相等时，返回可使用
		if (iter != vecp3d.end()-1 && (*iter) != *(iter+1))
		{
			return true;
		}
	}
	return false;
}
}