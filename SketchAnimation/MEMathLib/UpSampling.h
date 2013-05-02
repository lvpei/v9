#ifndef _UPSAMPLING
#define _UPSAMPLING

#include "AxisAngle.h"
#include "Vector3d.h"
#include "Quaternion.h"

#include <vector>
namespace MathLib
{

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif


enum USBC   // boundary conditions
{
	US_BC_I = 1,  // boundary condition I
	US_BC_NATURE,    // natural boundary condition with p''(0) = 0;
	US_BC_PERIODIC  // periodic boundary condition
};


class CUpSampling
{
	// input
public:
	// input all data
	void AddAllDataQ(
		IN Quaternion *quaternions,   // quaternions of every point
		IN double tStart,    // start time, may 0
		IN double fps,        // fps 
		IN int num,            // number of point
		IN bool fRefLast = true   // refer to last point when adding
		);
	void AddAllDataA(
		IN CAxisAngle *axisAngles,   // axis angles of every point
		IN double tStart,    // start time, may 0
		IN double fps,        // fps 
		IN int num,            // number of point
		IN bool fRefLast = true   // refer to last point when adding
		);

	// input one data
	void AddOneDataQ(
		IN Quaternion &qt,   // quaternions of every point
		IN double tStart,    // start time, may 0
		IN double fps,         // fps, new one will replace old one
		IN bool fRefLast = true   // refer to last point when adding
		);
	void AddOneDataA(
		IN CAxisAngle &aa,   // axis angles of every point
		IN double tStart,    // start time, may 0
		IN double fps,         // fps, new one will replace old one
		IN bool fRefLast = true   // refer to last point when adding
		);

	// output
public:
	// get all data
	// return array is created by NEW operator, and should DELETE by the caller
	void GetAllDataQ(
		OUT Quaternion **ppQuaternions,
		OUT int &iNumOfQts,
		IN double fps         // desired fps
		) const;
	void GetAllDataA(
		OUT CAxisAngle **ppAxisAngles,
		OUT int &iNumOfAAs,
		IN double fps         // desired fps
		) const;

	// get data in a range
	// return values in INTERSECTION(range, [m_dStartTime, m_dStopTime])
	// if input range is out of [m_dStartTime, m_dStopTime], return NULL.
	// return array is created by NEW operator, and should DELETE by the caller
	void GetDataByRangeQ(
		OUT Quaternion **ppQuaternions,
		OUT int &iNumOfQts,
		IN double fps,         // desired fps
		IN double tStart,     // start time
		IN double tStop     // stop time
		) const;
	void GetDataByRangeA(
		OUT CAxisAngle **ppAxisAngles,
		OUT int &iNumOfAAs,
		IN double fps,         // desired fps
		IN double tStart,     // start time
		IN double tStop     // stop time
		) const;

	// get data by time
	// if t is out of [m_dStartTime, m_dStopTime], return the value of nearest boundary
	Quaternion GetDataByTimeQ(IN double t) const;    
	CAxisAngle GetDataByTimeA(IN double t) const;

	// get angular 
	Quaternion GetOmigaByTime(IN double t) const;
	void GetOmigaByRange(
		OUT Quaternion **ppOmigas,
		OUT int &iNumOfOmigas,
		IN double fps,         // desired fps
		IN double tStart,     // start time
		IN double tStop     // stop time
		) const;
	void GetAllOmiga(
		OUT Quaternion **ppOmigas,
		OUT int &iNumOfOmigas,
		IN double fps         // desired fps
		) const;

	// get velocity
	CAxisAngle GetVelocityByTime(IN double t) const;

	// class control
public:
	// set control variables
	inline void SetControler(IN double *pdFinished, IN bool *pfStopNow) { m_pdFinished = pdFinished; m_pfStopNow = pfStopNow; }

	// generate coefficients
	void GenerateIt(void);

	// free memary
	void Release(void);    // release all memary form heap
	void ReleaseData(void);   // release data only, keep coefficients

	// set boundary condition
	inline void SetBoundaryCondition(IN USBC bc) { m_boundary = bc; }

	// set blend boundary. Default is +/-10 frames from Fram 0/n
	inline void SetBlendBoundary(int boundary) { m_blendbound = boundary; }

public:
	// constructor
	CUpSampling(void);
	~CUpSampling(void);

private:
	// function used by others
	// add a point
	void AddPoint(IN const Quaternion &qt, IN bool fRefLast = true);
	void AddPoint(IN const CAxisAngle &aa, IN bool fRefLast = true);

	// get a value
	CAxisAngle GetPointA(IN double t, IN int index) const;
	Quaternion GetPointQ(IN double t, IN int index) const;

	// get angular velocity by time
	Quaternion GetOmiga(IN double t, IN int index) const;
	CAxisAngle GetVelocity(IN double t, IN int index) const;

	// calculate some useful num for computing omiga
	void CalOmiga(IN const CAxisAngle &aa, IN const CAxisAngle &aap, OUT Quaternion &omiga) const;

	// get the index of lower bound of range contains t
	inline int GetIndex(IN double t) const;

	// Cubic spline using period boundary
	void GenerateIt3();
	// Cubic spline using nature boundary
	void GenerateIt2();
	// Cubic spline using boundary condition I
	void GenerateIt1();


private:
	// members
	// used for external control
	// Note: when these variable set to NULL or left undefined, the process will start with no control
	double *m_pdFinished;    // percentage of completion of current process, range : [0.0, 100.0]
	bool *m_pfStopNow;        // force generating stop and quit

	USBC m_boundary;
	int m_blendbound;

private:
	// input data
	double m_dStartTime;        // start time
	double m_dStopTime;        // stop time
	double m_dFps;                 // frame per second
	double m_dTimeSpace;      // time space between tow point
	std::vector<CAxisAngle *> m_aaData;    // all data

	int m_iNumOfDatas;

	// coefficient structure
	struct Coefficient
	{  // f(t) = c3 * t^3 + c2 * t^2 + c1 * t + c0
		Vector3d c3;
		Vector3d c2;
		Vector3d c1;
		Vector3d c0;
		Coefficient():c3(0.0,0.0,0.0),c2(0.0,0.0,0.0),c1(0.0,0.0,0.0),c0(0.0,0.0,0.0){}
	};

	std::vector<Coefficient *> m_Coefs;   // coefficients for every segment

	// parameters used for solve linear equation


};
}

#endif