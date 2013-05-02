
#include "UpSampling.h"
#include "Quaternion.h"
#include "AxisAngle.h"
#include "Vector3d.h"
#include <math.h>
#include <fstream>

namespace MathLib
{

const double PRECISION = 1.0e-4;
const double PRECISION4 = 1.0e-16;
const double m_pi = acos(-1.0);

inline int iclamp(int val, int min, int max)
{
	return (val > max) ? max : ((val < min) ? min : val);
}

//------------------------------------------------------------------------------
// constructor and destructor
//------------------------------------------------------------------------------
CUpSampling::CUpSampling()
: m_dStartTime(0.0)
, m_dStopTime(0.0)
, m_dFps(1.0)
, m_dTimeSpace(1.0)
, m_pdFinished(NULL)
, m_pfStopNow(NULL)
, m_boundary(US_BC_PERIODIC)
, m_blendbound(10)
, m_iNumOfDatas(0)
{
}
CUpSampling::~CUpSampling()
{
	Release();
}
// release memary of data and coefficients
// reset data-relate member
void CUpSampling::Release()
{
	int num = (int)m_aaData.size();
	for (int i = 0; i < num; i++)
	{
		delete m_aaData[i];
	}
	m_aaData.clear();
	num = (int)m_Coefs.size();
	for (int i = 0; i < num; i++)
	{
		delete m_Coefs[i];
	}
	m_Coefs.clear();

	m_dStartTime = 0.0;
	m_dStopTime = 0.0;
	m_dFps = 1.0;
	m_dTimeSpace = 1.0;
}
void CUpSampling::ReleaseData()
{
	int num = (int)m_aaData.size();
	for (int i = 0; i < num; i++)
	{
		delete m_aaData[i];
	}
	m_aaData.clear();
}

//------------------------------------------------------------------------------
// functions to insert data
//------------------------------------------------------------------------------
// private:
// add one data 
void CUpSampling::AddPoint(const Quaternion &qt, IN bool fRefLast)
{
	CAxisAngle *newAA = new CAxisAngle;
	if (!fRefLast || m_aaData.empty())
	{
		newAA->SetValue(qt);
	}
	else
	{
		newAA->SetValue(qt, *m_aaData[m_aaData.size() - 1]);
		//       newAA->SetValue(qt);
	}
	m_aaData.push_back(newAA);
	Coefficient *coef = new Coefficient;
	m_Coefs.push_back(coef);
	m_iNumOfDatas++;
}
void CUpSampling::AddPoint(const CAxisAngle &aa, IN bool fRefLast)
{
	CAxisAngle *newAA = new CAxisAngle;
	if (!fRefLast || m_aaData.empty())
	{
		newAA->SetValue(aa);
	}
	else
	{
		newAA->SetValue(aa, *m_aaData[m_aaData.size() - 1]);
		//        newAA->SetValue(aa);
	}
	m_aaData.push_back(newAA);
	Coefficient *coef = new Coefficient;
	m_Coefs.push_back(coef);
	m_iNumOfDatas++;
}

// public:
// add one data
void CUpSampling::AddOneDataQ(
							  IN Quaternion &qt, 
							  IN double tStart, 
							  IN double fps,
							  IN bool fRefLast
							  )
{
	if (fps <= 0) 
	{
		return;
	}
	m_dStartTime = tStart;
	m_dFps = fps;
	m_dTimeSpace = 1.0 / fps;
	AddPoint(qt, fRefLast);
	m_dStopTime = tStart + m_dTimeSpace * m_aaData.size();
}
void CUpSampling::AddOneDataA(
							  IN CAxisAngle &aa, 
							  IN double tStart, 
							  IN double fps,
							  IN bool fRefLast
							  )
{
	if (fps <= 0) 
	{
		return;
	}
	m_dStartTime = tStart;
	m_dFps = fps;
	m_dTimeSpace = 1.0 / fps;
	AddPoint(aa, fRefLast);
	m_dStopTime = tStart + m_dTimeSpace * m_aaData.size();
}
// add all Data
void CUpSampling::AddAllDataA(
							  IN CAxisAngle *axisAngles,
							  IN double tStart,
							  IN double fps,
							  IN int num,
							  IN bool fRefLast
							  )
{
	if (fps <= 0) 
	{
		return;
	}
	for (int i = 0; i < num; i++)
	{
		// if have to stop now
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{
			Release();
			return;
		}

		AddPoint(axisAngles[i], fRefLast);

		// 
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = (double)i / (double)(num - 1) * 100.0;
		}    
	}
	m_dStartTime = tStart;
	m_dFps = fps;
	m_dTimeSpace = 1.0 / fps;
	m_dStopTime = tStart + (num - 1) / fps;
}
void CUpSampling::AddAllDataQ(
							  IN Quaternion *quaternions,
							  IN double tStart,
							  IN double fps,
							  IN int num,
							  IN bool fRefLast
							  )
{
	if (fps <= 0) 
	{
		return;
	}
	for (int i = 0; i < num; i++)
	{
		// if have to stop now
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{
			Release();
			return;
		}

		AddPoint(quaternions[i], fRefLast);

		// 
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = (double)i / (double)(num - 1) * 100.0;
		}    
	}
	m_dStartTime = tStart;
	m_dFps = fps;
	m_dTimeSpace = 1.0 / fps;
	m_dStopTime = tStart + (num - 1) / fps;
}

//------------------------------------------------------------------------------
// functions to get data
//------------------------------------------------------------------------------
// private:
// Get AA(t)
CAxisAngle CUpSampling::GetPointA(IN double t, IN int index) const
{   
	if (index < 0)
	{
		index = 0;
	}
	if (index >= m_iNumOfDatas - 1)
	{
		index = m_iNumOfDatas - 2;
	}
	Coefficient *coef = m_Coefs[index];
	t -= m_dStartTime + index * m_dTimeSpace;
	t /= m_dTimeSpace;
	return CAxisAngle(((coef->c3 * t + coef->c2) * t + coef->c1) * t + coef->c0);
}
Quaternion CUpSampling::GetPointQ(IN double t, IN int index) const
{
	return GetPointA(t, index).GetQuaternion();
}

// public:
// get data by time
CAxisAngle CUpSampling::GetDataByTimeA(IN double t) const
{
	int index = GetIndex(t);
	return GetPointA(t, index);
}
Quaternion CUpSampling::GetDataByTimeQ(IN double t) const
{
	int index = GetIndex(t);
	return GetPointA(t, index).GetQuaternion();
}

// get data by range
void CUpSampling::GetDataByRangeA(
								  OUT CAxisAngle **ppAxisAngles,
								  OUT int &iNumOfAAs,
								  IN double fps,
								  IN double tStart,
								  IN double tStop
								  ) const
{
	if (NULL == ppAxisAngles)
	{
		iNumOfAAs = 0;       

		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}

		return;
	}
	if (tStart > tStop)
	{
		iNumOfAAs = 0;
		*ppAxisAngles = NULL;
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = 0.0;
	}

	tStart = (tStart < m_dStartTime) ? m_dStartTime : tStart;
	tStop = (tStop > m_dStopTime) ? m_dStopTime : tStop;
	int startIdx = GetIndex(tStart);
	int stopIdx = GetIndex(tStop);
	double dTimeSpace = 1.0 / fps;
	iNumOfAAs = (int)((tStop - tStart) * fps) + 1;

	*ppAxisAngles = new CAxisAngle[iNumOfAAs];

	for (int i = 0; i < iNumOfAAs; i++)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}
			delete []*ppAxisAngles;
			*ppAxisAngles = NULL;
			return;
		}
		double t = tStart + i * dTimeSpace;
		(*ppAxisAngles)[i].SetValue(GetPointA(t, GetIndex(t)));

		if (NULL != m_pdFinished)
		{
			*m_pdFinished = (double)(i + 1) / double(iNumOfAAs) * 100.0;
		}
	}    
}

void CUpSampling::GetDataByRangeQ(
								  OUT Quaternion **ppQuaternions, 
								  OUT int &iNumOfQts, 
								  IN double fps,
								  IN double tStart,
								  IN double tStop
								  ) const
{
	if (NULL == ppQuaternions)
	{
		iNumOfQts = 0;
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}
	if (tStart > tStop)
	{
		iNumOfQts = 0;
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		*ppQuaternions = NULL;
		return;
	}

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = 0.0;
	}
	tStart = (tStart < m_dStartTime) ? m_dStartTime : tStart;
	tStop = (tStop > m_dStopTime) ? m_dStopTime : tStop;
	int startIdx = GetIndex(tStart);
	int stopIdx = GetIndex(tStop);
	double dTimeSpace = 1.0 / fps;
	iNumOfQts = (int)((tStop - tStart) * fps) + 1;

	*ppQuaternions = new Quaternion[iNumOfQts];

	for (int i = 0; i < iNumOfQts; i++)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}
			delete [] *ppQuaternions;
			*ppQuaternions = NULL;
			return;
		}
		double t = tStart + i * dTimeSpace;
		(*ppQuaternions)[i] = GetPointA(t, GetIndex(t)).GetQuaternion();
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = (double)(i + 1) / double(iNumOfQts) * 100.0;
		}

	}    
}

void CUpSampling::GetAllDataA(
							  OUT CAxisAngle **ppAxisAngles, 
							  OUT int &iNumOfAAs, 
							  IN double fps
							  ) const
{
	GetDataByRangeA(ppAxisAngles, iNumOfAAs, fps, m_dStartTime, m_dStopTime);
}

void CUpSampling::GetAllDataQ(
							  OUT Quaternion **ppQuaternions, 
							  OUT int &iNumOfQts, 
							  IN double fps
							  ) const
{
	GetDataByRangeQ(ppQuaternions, iNumOfQts, fps, m_dStartTime, m_dStopTime);
}
//------------------------------------------------------------------------------
// functions used for calculating angular velocity
//------------------------------------------------------------------------------
void CUpSampling::CalOmiga(const CAxisAngle &aa, const CAxisAngle &aap, OUT Quaternion &omiga) const
{
	double theta = aa.GetAngle();
	double x = aa.GetValue().x;
	double y = aa.GetValue().y;
	double z = aa.GetValue().z;
	double xp = aap.GetValue().x;
	double yp = aap.GetValue().y;
	double zp = aap.GetValue().z;
	static const double n_1_24 = -1.0 / 24.0;
	static const double n19_3840 = 19.0 / 3840.0;
	double A,B,C,M;
	C = cos(theta / 2.0);
	if (abs(theta) < PRECISION)
	{
		B = 0.5 - theta * theta / 48.0;
		A = n_1_24 + n19_3840 * theta * theta;
	}
	else
	{
		B = sin(theta / 2.0) / theta;
		A = (C / 2.0 - B) / theta / theta;
	}
	M = x * xp + y * yp + z * zp;
	A *= M;

	Quaternion qt;
	qt.s = - 0.5 * B * M;
	qt.v.x = A * x + B * xp;
	qt.v.y = A * y + B * yp;
	qt.v.z = A * z + B * zp;

	omiga = qt * aa.GetQuaternion().getComplexConjugate();
	omiga.s *= 2;
	omiga.v.x *= 2;
	omiga.v.y *= 2;
	omiga.v.z *= 2;

}

Quaternion CUpSampling::GetOmiga(double t, int index) const
{
	Quaternion omiga;
	if (index < 0)
	{
		index = 0;
	}
	if (index >= m_iNumOfDatas - 1)
	{
		index = m_iNumOfDatas - 2;
	}
	CAxisAngle aa,aap;
	Coefficient *coef = m_Coefs[index];
	t -= m_dStartTime + index * m_dTimeSpace;
	t /= m_dTimeSpace;
	aa.SetValue(((coef->c3 * t + coef->c2) * t + coef->c1) * t + coef->c0);
	aap.SetValue((coef->c3 * (3.0 * t * t) + coef->c2 * (2.0 * t) + coef->c1) / m_dTimeSpace);
	CalOmiga(aa,aap, omiga);
	return omiga;
}
CAxisAngle CUpSampling::GetVelocity(double t, int index) const
{
	if (index < 0)
	{
		index = 0;
	}
	if (index >= m_iNumOfDatas - 1)
	{
		index = m_iNumOfDatas - 2;
	}
	CAxisAngle aap;
	Coefficient *coef = m_Coefs[index];
	t -= m_dStartTime + index * m_dTimeSpace;
	t /= m_dTimeSpace;
	aap.SetValue((coef->c3 * (3.0 * t * t) + coef->c2 * (2.0 * t) + coef->c1) / m_dTimeSpace);
	return aap;
}

Quaternion CUpSampling::GetOmigaByTime(double t) const
{
	int index = GetIndex(t);
	return GetOmiga(t, index);
}
CAxisAngle CUpSampling::GetVelocityByTime(double t) const
{
	return GetVelocity(t, GetIndex(t));
}

void CUpSampling::GetOmigaByRange(
								  OUT Quaternion **ppOmigas,
								  OUT int &iNumOfOmigas,
								  IN double fps,
								  IN double tStart,
								  IN double tStop
								  ) const
{
	if (NULL == ppOmigas)
	{
		iNumOfOmigas = 0;
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}
	if (tStart > tStop)
	{
		iNumOfOmigas = 0;
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		*ppOmigas = NULL;
		return;
	}

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = 0.0;
	}
	tStart = (tStart < m_dStartTime) ? m_dStartTime : tStart;
	tStop = (tStop > m_dStopTime) ? m_dStopTime : tStop;
	int startIdx = GetIndex(tStart);
	int stopIdx = GetIndex(tStop);
	double dTimeSpace = 1.0 / fps;
	iNumOfOmigas = (int)((tStop - tStart) * fps) + 1;

	*ppOmigas = new Quaternion[iNumOfOmigas];

	for (int i = 0; i < iNumOfOmigas; i++)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}
			delete [] *ppOmigas;
			*ppOmigas = NULL;
			return;
		}
		double t = tStart + i * dTimeSpace;
		(*ppOmigas)[i] = GetOmiga(t, GetIndex(t));
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = (double)(i + 1) / double(iNumOfOmigas) * 100.0;
		}
	}    
}
void CUpSampling::GetAllOmiga(
							  OUT Quaternion **ppOmigas,
							  OUT int &iNumOfOmigas,
							  OUT double fps
							  ) const
{
	GetOmigaByRange(ppOmigas, iNumOfOmigas, fps, m_dStartTime, m_dStopTime);
}

//------------------------------------------------------------------------------
// functions used for interpolating
//------------------------------------------------------------------------------
void CUpSampling::GenerateIt3()
{
	int nofpts = (int)m_aaData.size();
	if (nofpts == 0)
	{
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}
	if (nofpts == 1)
	{
		m_Coefs[0]->c0 = m_aaData[0]->GetValue();
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}

	if (m_aaData[0]->GetValue() != m_aaData[nofpts - 1]->GetValue())
	{
		CAxisAngle aaDatan;
		aaDatan.SetValue(*m_aaData[0], *m_aaData[nofpts - 1]);
		Vector3d vtd = aaDatan.GetValue() - m_aaData[nofpts - 1]->GetValue();
		int blend = nofpts / 2;
		blend = (blend > m_blendbound) ? m_blendbound : blend;
		double blstep = 0.5 / (double)blend;
		double bl = 0.5;
		CAxisAngle *temp;
		for (int i = 0; i < blend; i++)
		{
			temp = m_aaData[i];
			temp->SetValue(temp->GetValue() - vtd * bl);
			temp = m_aaData[nofpts - i - 1];
			temp->SetValue(temp->GetValue() + vtd * bl);
			bl -= blstep;
		}
	}
	if (nofpts == 2)
	{
		m_Coefs[0]->c0 = m_aaData[0]->GetValue();
		m_Coefs[0]->c1 = m_aaData[1]->GetValue() - m_aaData[0]->GetValue();        
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}


	if (NULL != m_pdFinished)
	{
		*m_pdFinished = 0.0;
	}
	double base = 0.0;
	double rate = 0.0;
	// parameters used for solve linear equation
	// use Thomas algorithm
	double *l = NULL;
	double *u = NULL;
	double *p = NULL;
	double *s = NULL;

	// initial
	nofpts--;
	l = new double[nofpts];
	u = new double[nofpts];
	p = new double[nofpts];
	s = new double[nofpts];

	// initial L.U.
	rate = 10.0;

	l[0] = 0.0;
	u[0] = 2.0;
	s[0] = 0.25;
	p[0] = 0.5;
	int nn = nofpts - 1;
	double sp = 0.125;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base + rate * 1.0 / nofpts;
	}

	for (int i = 1; i < nn; i++)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}

			delete []l;
			delete []p;
			delete []u;
			delete []s;
			return;
		}
		l[i] = 0.5 / u[i - 1];
		u[i] = 2.0 - l[i] * 0.5;
		p[i] = -l[i] * p[i - 1];
		s[i] = -s[i - 1] * 0.5 / u[i];
		sp += p[i] * s[i];
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (i + 1) / nofpts;
		}
	}
	l[nn] = 0.5 / u[nn - 1];
	u[nn] = 2.0 - (l[nn] + s[nn - 1]) * (0.5 + p[nn - 1]) - sp + s[nn - 1] * p[nn - 1];
	s[nn] = 0.0;
	p[nn] = 0.0;

	base += rate;
	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base;
	}


	// calculate ds,ys
	rate = 20.0;

	Vector3d d;
	Vector3d *y = NULL;
	y = new Vector3d[nofpts];
	CAxisAngle *p0,*p1,*p2;
	p0 = m_aaData[0];
	p1 = m_aaData[1];
	p2 = m_aaData[2];
	d = (p2->GetValue() - p0->GetValue()) * (1.5 / m_dTimeSpace);
	y[0] = d;
	Vector3d sy(y[0] * s[0]);

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base + rate * 1.0 / nofpts;
	}

	for (int i = 1; i < nn; i++) // nn = m_aaData.size() - 2
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}                    
			delete []l;
			delete []p;
			delete []u;
			delete []s;
			delete []y;
			return;
		}
		p0 = p1;
		p1 = p2;
		p2 = m_aaData[i + 2];
		d = (p2->GetValue() - p0->GetValue()) * (1.5 / m_dTimeSpace);
		y[i] = d - y[i - 1] * l[i];
		sy += y[i] * s[i];

		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (i + 1) / nofpts;
		}

	}
	p0 = p1;
	CAxisAngle aaDatan;
	aaDatan.SetValue(*m_aaData[1], *m_aaData[nofpts - 1]);

	p2 = &aaDatan;
	d = (p2->GetValue() - p0->GetValue()) * (1.5 / m_dTimeSpace);
	y[nn] = d - sy - y[nn - 1] * l[nn];

	base += rate;
	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base;
	}


	// calculate ms
	rate = 20.0;

	Vector3d *m = new Vector3d[nofpts + 1];
	m[nn + 1] = y[nn] / u[nn];
	m[0] = m[nn + 1];

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base + rate * 1.0 / nofpts;
	}

	for (int i = nn - 1; i >= 0; i--)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}
			delete []y;
			// free memary
			delete []l;
			delete []p;
			delete []u;
			delete []s;
			delete []m;
			return;
		}
		m[i + 1] = (y[i] - m[i + 2] * 0.5 - m[nn + 1] * s[i]) / u[i];

		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (nn - i + 1) / nofpts;
		}
	}

	delete []y;
	// free memary
	delete []l;
	delete []p;
	delete []u;
	delete []s;

	base += rate;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base;
	}

	nofpts++;
	// calculate coefficients
	rate = 50.0;

	Vector3d pvt1,pvt2;
	pvt2 = m_aaData[0]->GetValue();
	for (int i = 0; i < nofpts - 1; i++)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}
			delete []m;
			return;
		}
		pvt1 = pvt2;
		pvt2 = m_aaData[i + 1]->GetValue();
		Vector3d &m0 = m[i];
		Vector3d &m1 = m[i + 1];
		Coefficient *coef = m_Coefs[i];
		coef->c0 = pvt1;
		coef->c1 = m0 * m_dTimeSpace;
		coef->c2 = (pvt2 - pvt1) * 3.0 - (m0 * 2.0 + m1) * m_dTimeSpace; 
		coef->c3 = (pvt1 - pvt2) * 2.0 + (m0 + m1) * m_dTimeSpace;


		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (i + 1) / nofpts;
		}

	}
	delete []m;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = 100.0;
	}

}

void CUpSampling::GenerateIt2()
{
	int nofpts = (int)m_aaData.size();
	if (nofpts == 0)
	{
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}
	if (nofpts == 1)
	{
		m_Coefs[0]->c0 = m_aaData[0]->GetValue();
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}
	if (nofpts == 2)
	{
		m_Coefs[0]->c0 = m_aaData[0]->GetValue();
		m_Coefs[0]->c1 = m_aaData[1]->GetValue() - m_aaData[0]->GetValue();        
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}


	if (NULL != m_pdFinished)
	{
		*m_pdFinished = 0.0;
	}
	double base = 0.0;
	double rate = 0.0;
	// parameters used for solve linear equation
	// use Thomas algorithm
	double *l = NULL;
	double *u = NULL;
	double *p = NULL;
	double *s = NULL;

	// initial
	l = new double[nofpts];
	u = new double[nofpts];

	// initial L.U.
	rate = 10.0;

	l[0] = 0.0;
	u[0] = 2.0;
	int nn = nofpts - 1;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base + rate * 1.0 / nofpts;
	}

	for (int i = 1; i <= nn; i++)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}

			delete []l;
			delete []u;
			return;
		}
		l[i] = 0.5 / u[i - 1];
		u[i] = 2.0 - l[i] * 0.5;
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (i + 1) / nofpts;
		}
	}

	base += rate;
	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base;
	}

	// calculate ds,ys
	rate = 20.0;

	Vector3d d;
	Vector3d *y = NULL;
	y = new Vector3d[nofpts];
	CAxisAngle *p0,*p1,*p2;
	p0 = m_aaData[0];
	p1 = m_aaData[1];
	p2 = m_aaData[2];
	d = (p1->GetValue() - p0->GetValue()) * (3.0 / m_dTimeSpace);

	y[0] = d;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base + rate * 1.0 / nofpts;
	}

	for (int i = 1; i < nn; i++) // nn = m_aaData.size() - 2
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}                    
			delete []l;
			delete []u;
			delete []y;
			return;
		}
		d = (p2->GetValue() - p0->GetValue()) * (1.5 / m_dTimeSpace);


		y[i] = d - y[i - 1] * l[i];
		p0 = p1;
		p1 = p2;
		if (i < nn - 1)
		{
			p2 = m_aaData[i + 2];
		}

		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (i + 1) / nofpts;
		}

	}
	d = (p1->GetValue() - p0->GetValue()) * (3.0 / m_dTimeSpace);


	y[nn] = d - y[nn - 1] * l[nn];

	base += rate;
	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base;
	}


	// calculate ms
	rate = 20.0;

	Vector3d *m = new Vector3d[nofpts];
	m[nn] = y[nn] / u[nn];

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base + rate * 1.0 / nofpts;
	}

	for (int i = nn - 1; i >= 0; i--)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}
			delete []y;
			// free memary
			delete []l;
			delete []u;
			delete []m;
			return;
		}
		m[i] = (y[i] - m[i + 1] * 0.5) / u[i];

		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (nn - i) / nofpts;
		}
	}

	delete []y;
	// free memary
	delete []l;
	delete []u;

	base += rate;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base;
	}

	// calculate coefficients
	rate = 50.0;

	Vector3d pvt1,pvt2;
	pvt2 = m_aaData[0]->GetValue();
	for (int i = 0; i < nofpts - 1; i++)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}
			delete []m;
			return;
		}
		pvt1 = pvt2;
		pvt2 = m_aaData[i + 1]->GetValue();
		Vector3d &m0 = m[i];
		Vector3d &m1 = m[i + 1];
		Coefficient *coef = m_Coefs[i];
		coef->c0 = pvt1;
		coef->c1 = m0 * m_dTimeSpace;
		coef->c2 = (pvt2 - pvt1) * 3.0 - (m0 * 2.0 + m1) * m_dTimeSpace; 
		coef->c3 = (pvt1 - pvt2) * 2.0 + (m0 + m1) * m_dTimeSpace;


		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (i + 1) / nofpts;
		}

	}


	delete []m;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = 100.0;
	}

}
/////////////////////////
void CUpSampling::GenerateIt1()
{
	int nofpts = (int)m_aaData.size();
	if (nofpts == 0)
	{
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}
	if (nofpts == 1)
	{
		m_Coefs[0]->c0 = m_aaData[0]->GetValue();
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}
	if (nofpts == 2)
	{
		m_Coefs[0]->c0 = m_aaData[0]->GetValue();
		m_Coefs[0]->c1 = m_aaData[1]->GetValue() - m_aaData[0]->GetValue();        
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = 100.0;
		}
		return;
	}

	nofpts -= 2;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = 0.0;
	}
	double base = 0.0;
	double rate = 0.0;
	// parameters used for solve linear equation
	// use Thomas algorithm
	double *l = NULL;
	double *u = NULL;
	double *p = NULL;
	double *s = NULL;

	// initial
	l = new double[nofpts];
	u = new double[nofpts];

	// initial L.U.
	rate = 10.0;

	l[0] = 0.0;
	u[0] = 2.0;
	int nn = nofpts - 1;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base + rate * 1.0 / nofpts;
	}

	for (int i = 1; i <= nn; i++)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}

			delete []l;
			delete []u;
			return;
		}
		l[i] = 0.5 / u[i - 1];
		u[i] = 2.0 - l[i] * 0.5;
		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (i + 1) / nofpts;
		}
	}

	base += rate;
	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base;
	}

	// calculate ds,ys
	rate = 20.0;

	Vector3d d;
	Vector3d *y = NULL;
	y = new Vector3d[nofpts];

	nofpts += 2;
	Vector3d *m = new Vector3d[nofpts];
	m[0] = (m_aaData[1]->GetValue() - m_aaData[0]->GetValue()) / m_dTimeSpace;
	m[nofpts - 1] = (m_aaData[nofpts - 1]->GetValue() - m_aaData[nofpts - 2]->GetValue()) / m_dTimeSpace;

	CAxisAngle *p0,*p1,*p2;
	p0 = m_aaData[0];
	p1 = m_aaData[1];
	p2 = m_aaData[2];

	d = (p2->GetValue() - p0->GetValue()) * (1.5 / m_dTimeSpace) - m[0] * 0.5;

	y[0] = d;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base + rate * 1.0 / nofpts;
	}


	for (int i = 1; i < nn; i++) // nn = m_aaData.size() - 3
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}                    
			delete []l;
			delete []u;
			delete []y;
			return;
		}
		d = (p2->GetValue() - p0->GetValue()) * (1.5 / m_dTimeSpace);


		y[i] = d - y[i - 1] * l[i];
		p0 = p1;
		p1 = p2;
		p2 = m_aaData[i + 2];

		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (i + 1) / nofpts;
		}

	}

	d = (p2->GetValue() - p0->GetValue()) * (1.5 / m_dTimeSpace) - m[nofpts - 1] * 0.5;
	y[nn] = d - y[nn - 1] * l[nn];

	base += rate;
	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base;
	}

	// calculate ms
	rate = 20.0;

	m[nn + 1] = y[nn] / u[nn];    // nn = nofpts - 2 - 1 = nofpts - 3,, nn + 1 = nofpts - 2;
	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base + rate * 1.0 / nofpts;
	}

	for (int i = nn - 1; i >= 0; i--)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}
			delete []y;
			// free memary
			delete []l;
			delete []u;
			delete []m;
			return;
		}
		m[i + 1] = (y[i] - m[i + 2] * 0.5) / u[i];

		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (nn - i) / nofpts;
		}
	}

	delete []y;
	// free memary
	delete []l;
	delete []u;

	base += rate;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = base;
	}

	// calculate coefficients
	rate = 50.0;

	Vector3d pvt1,pvt2;
	pvt2 = m_aaData[0]->GetValue();
	for (int i = 0; i < nofpts - 1; i++)
	{
		if (NULL != m_pfStopNow && (*m_pfStopNow))
		{    
			if (NULL != m_pdFinished)
			{
				*m_pdFinished = 100.0;
			}
			delete []m;
			return;
		}
		pvt1 = pvt2;
		pvt2 = m_aaData[i + 1]->GetValue();
		Vector3d &m0 = m[i];
		Vector3d &m1 = m[i + 1];
		Coefficient *coef = m_Coefs[i];
		coef->c0 = pvt1;
		coef->c1 = m0 * m_dTimeSpace;
		coef->c2 = (pvt2 - pvt1) * 3.0 - (m0 * 2.0 + m1) * m_dTimeSpace; 
		coef->c3 = (pvt1 - pvt2) * 2.0 + (m0 + m1) * m_dTimeSpace;


		if (NULL != m_pdFinished)
		{
			*m_pdFinished = base + rate * (i + 1) / nofpts;
		}

	}


	delete []m;

	if (NULL != m_pdFinished)
	{
		*m_pdFinished = 100.0;
	}

}

void CUpSampling::GenerateIt()
{
	switch (m_boundary)
	{
	case US_BC_I:
		GenerateIt1();
		break;
	case US_BC_NATURE:
		GenerateIt2();
		break;
	default:
		GenerateIt3();        
	}
}

int CUpSampling::GetIndex(double t) const
{
	return iclamp((int)floor(((t - m_dStartTime) / m_dTimeSpace)), 0, m_iNumOfDatas - 2);
}

}