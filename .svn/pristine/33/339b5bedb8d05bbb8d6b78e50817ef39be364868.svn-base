#ifndef _AXISANGLE
#define _AXISANGLE

#include "Vector3d.h"
#include "Quaternion.h"

namespace MathLib
{

#define M_PI_2 1.5707963f

class CAxisAngle
{
public:
	//constructors
	CAxisAngle(const CAxisAngle &aa):m_vtVector(aa.m_vtVector){}
	CAxisAngle(const Quaternion &qt);
	CAxisAngle(double x = 0.0, double y = M_PI_2, double z = 0.0) : m_vtVector(x,y,z){}
	CAxisAngle(const Vector3d &vt):m_vtVector(vt){}
	CAxisAngle(double *vt) : m_vtVector(vt[0], vt[1], vt[2]) { }

private:
	Vector3d m_vtVector;

public:
	inline void SetValue(const CAxisAngle &aa) { m_vtVector = aa.m_vtVector; }
	inline void SetValue(double x, double y, double z) { m_vtVector.x = x; m_vtVector.y = y; m_vtVector.z = z; }
	void SetValue(const Quaternion &qt);
	void SetValue(const Quaternion &qt, const CAxisAngle &aa);
	void SetValue(const CAxisAngle &aa, const CAxisAngle &aaref);

	// get value
	inline const Vector3d &GetValue(void) const { return m_vtVector; }

	// get angle
	inline double GetAngle(void) const { return m_vtVector.length(); }
	Vector3d GetAxis(void) const;

	// get quaternion
	Quaternion GetQuaternion(void) const;
};
}

#endif