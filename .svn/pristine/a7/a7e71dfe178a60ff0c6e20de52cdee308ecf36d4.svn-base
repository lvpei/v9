#include "AxisAngle.h"
#include "Quaternion.h"
#include "Vector3d.h"

namespace MathLib
{

const double PRECISION = 1.0e-4;
const double PRECISION4 = 1.0e-16;
const double m_pi = acos(-1.0);

// if x < lower, x = lower
// if x > upper, x = upper
inline double clamp(double x, double lower, double upper)
{
	return (x < lower) ? lower : ((x > upper) ? upper : x);
}

CAxisAngle::CAxisAngle(const Quaternion &qt)
{
	SetValue(qt);
}

void CAxisAngle::SetValue(const Quaternion &qt)
{
	double theta = acos(clamp(qt.s, -1.0, 1.0)) * 2;        //theta of rotation
	double vn = Vector3d(qt.v.x,qt.v.y,qt.v.z).length();
	
	if (vn < PRECISION4)
	{
		m_vtVector.x = 0;
		m_vtVector.y = 0;
		m_vtVector.z = 0;
	}else{
		m_vtVector.x = qt.v.x / vn * theta;
		m_vtVector.y = qt.v.y / vn * theta;
		m_vtVector.z = qt.v.z / vn * theta;
	}
}
//------------------------------------------------------------------------------
//     SetValue, convert a quaternion to an axis angle referring to another axis angle
//------------------------------------------------------------------------------
void CAxisAngle::SetValue(const Quaternion &qt, const CAxisAngle &aa)
{
	SetValue(qt);
	// ray direction,start point is m_vtVector
	Vector3d vtDir = GetAxis();    
	double len = vtDir.length();
	if (len < PRECISION4)
	{
		vtDir = aa.GetValue();
		vtDir.toUnit();
	}
	Vector3d vtDist(aa.GetValue() - GetValue());    
	double dn2pi = vtDir.dotProductWith(vtDist) / m_pi / 2.0;
	// round it
	int n2pi = (int)((dn2pi > 0) ? floor(dn2pi + 0.5) : ceil(dn2pi - 0.5));
	m_vtVector += vtDir * (n2pi * 2.0 * m_pi);
}

void CAxisAngle::SetValue(const CAxisAngle &aa, const CAxisAngle &aaref)
{
	SetValue(aa);
	Vector3d vtDir = GetAxis();   
	double len = m_vtVector.length();
	if (len < PRECISION4)
	{
		vtDir = aaref.GetValue();
		vtDir.toUnit();
	}
	Vector3d vtDist(aaref.GetValue() - GetValue());    
	double dn2pi = vtDir.dotProductWith(vtDist) / m_pi / 2.0;
	// round it
	int n2pi = (int)((dn2pi > 0) ? floor(dn2pi + 0.5) : ceil(dn2pi - 0.5));
	m_vtVector += vtDir * (n2pi * 2.0 * m_pi);
}

//------------------------------------------------------------------------------
//     GetQuaternion, translate an axis angle to a quaternion
//------------------------------------------------------------------------------
Quaternion CAxisAngle::GetQuaternion() const
{
	double theta = m_vtVector.length();
	Quaternion qt;
	qt.s = cos(theta / 2.0);
	double vn;
	// deal with small theta
	if (abs(theta) < PRECISION)
	{
		vn = 0.5 - theta * theta / 48.0;
	}
	else
	{
		vn = sin(theta / 2.0) / theta;
	}

	qt.v.x = m_vtVector.x * vn;
	qt.v.y = m_vtVector.y * vn;
	qt.v.z = m_vtVector.z * vn;
	double qn = sqrt(qt.v.x * qt.v.x + qt.v.y * qt.v.y + qt.v.z * qt.v.z + qt.s * qt.s);
	qt.v.x /= qn;
	qt.v.y /= qn;
	qt.v.z /= qn;
	qt.s /= qn;

	return qt;
}

//------------------------------------------------------------------------------
//     GetAxis, Get normalized axis
//------------------------------------------------------------------------------
Vector3d CAxisAngle::GetAxis(void) const
{
	Vector3d vt(m_vtVector); 
	return vt.toUnit();
}
}