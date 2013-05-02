/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#include "quaternion.h"
#include <Utils/Utils.h>
namespace MathLib
{

/**
	Returns the complex conjugate of the current quaternion.
*/
Quaternion Quaternion::getComplexConjugate() const{
	return Quaternion(s, v * (-1));
}

/**
	Returns the inverse of the current quaternion: q * q^-1 = identity quaternion: s = 1, v = (0,0,0)
*/
Quaternion Quaternion::getInverse() const {
	double length = this->getLength();
	return (this->getComplexConjugate() * (1/(length*length)));
}

/**
	Returns the length of a quaternion.
*/
double Quaternion::getLength() const{
	return sqrt(s*s + v.dotProductWith(v));
}

/**
	Computes the dot product between the current quaternion and the one given as parameter.
*/
double Quaternion::dotProductWith(const Quaternion &other) const{
	return (this->s * other.s + this->v.dotProductWith(other.v));
}

/**
	This method returns a quaternion that is the result of linearly interpolating between the current quaternion and the one provided as a parameter.
	The value of the parameter t indicates the progress: if t = 0, the result will be *this. If it is 1, it will be other. If it is inbetween, then
	the result will be a combination of the two initial quaternions.
	Both quaternions that are used for the interpolation are assumed to have unit length!!!
*/
Quaternion Quaternion::linearlyInterpolateWith(const Quaternion &other, double t) const{
	if (t<0) t = 0;
	if (t>1) t = 1;
	Quaternion result = (*this)*(1-t) + other*t;
	return result * (1/result.getLength());
}


/**
	This method is used to rotate the vector that is passed in as a parameter by the current quaternion (which is assumed to be a 
	unit quaternion).
*/
Vector3d Quaternion::rotate(const Vector3d& u) const{
	//uRot = q * (0, u) * q' = (s, v) * (0, u) * (s, -v)
	//working it out manually, we get:
	Vector3d t = u * s + v.crossProductWith(u);
	return v*u.dotProductWith(v) + t * s + v.crossProductWith(t);
}


/**
	This method is used to rotate the vector that is passed in as a parameter by the current quaternion (which is assumed to be a 
	unit quaternion).
*/
Vector3d Quaternion::inverseRotate(const Vector3d& u) const{
	//uRot = q * (0, u) * q' = (s, -v) * (0, u) * (s, v)
	//working it out manually, we get:
	Vector3d t = u * s + u.crossProductWith(v);
	return v*u.dotProductWith(v) + t * s + t.crossProductWith(v);
}

/**
	This method returns a quaternion that is the result of spherically interpolating between the current quaternion and the one provided as a parameter.
	The value of the parameter t indicates the progress: if t = 0, the result will be *this. If it is 1, it will be other. If it is inbetween, then
	the result will be a combination of the two initial quaternions.
	Both quaternions that are used for the interpolation are assumed to have unit length!!!
*/
Quaternion Quaternion::sphericallyInterpolateWith(const Quaternion &other, double t) const{

	//make sure that we return the same value if either of the quaternions involved is q or -q 
	if (this->dotProductWith(other) < 0){
		Quaternion temp;
		temp.s = -other.s;
		temp.v = other.v * (-1);
		return this->sphericallyInterpolateWith(temp, t);
	}

	if (t<0) t = 0;
	if (t>1) t = 1;
	double dotProduct = this->dotProductWith(other);
	double sinTheta = sqrt(MAX(0,1-dotProduct*dotProduct));
	double theta = safeACOS(dotProduct);
	if (sinTheta == 0)
		return (*this);
	return ((*this) * sin(theta * (1-t)) + other * sin(theta * t)) * (1/sin(theta));
}


/**
	This method will return a quaternion that represents a rotation of angle radians around the axis provided as a parameter.
	IT IS ASSUMED THAT THE VECTOR PASSED IN IS A UNIT VECTOR!!!
*/
Quaternion Quaternion::getRotationQuaternion(double angle, Vector3d &axis){
	Quaternion result(cos(angle/2), axis * sin(angle/2));
	result.toUnit();
	return result;
}

/**
	This method will return a 4x4 matrix that represents an equivalent rotation as the given quaternion.
*/
void Quaternion::getRotationMatrix(TransformationMatrix* m) const{
	double w = s, x = v.getX(), y = v.getY(), z = v.getZ();
	double values[16] = {1-2*y*y-2*z*z,	2*x*y - 2*w*z,	2*x*z + 2*w*y,	0, 
						 2*x*y + 2*w*z,	1-2*x*x-2*z*z,	2*y*z - 2*w*x,	0,
						 2*x*z - 2*w*y,	2*y*z + 2*w*x,	1-2*x*x-2*y*y,	0,
								0,			0,				0,			1};
	m->setValues(values);
}

/**
	This method fills the presumably 3x3 matrix so that it represents an equivalent rotation as the given quaternion.
*/
void Quaternion::getRotationMatrix(Matrix* m) const{
	//it is assumed that the matrix m is 3x3 - no checking to get the code to be fast
	double w = s, x = v.x, y = v.y, z = v.z;
	double values[9] = {1-2*y*y-2*z*z,	2*x*y - 2*w*z,	2*x*z + 2*w*y,
						 2*x*y + 2*w*z,	1-2*x*x-2*z*z,	2*y*z - 2*w*x,
						 2*x*z - 2*w*y,	2*y*z + 2*w*x,	1-2*x*x-2*y*y };
	m->setValues(values);
}

/**
	Returns the result of multiplying the current quaternion by rhs. NOTE: the product of two quaternions represents a rotation as well: q1*q2 represents
	a rotation by q2 followed by a rotation by q1!!!!
*/
Quaternion Quaternion::operator * (const Quaternion &other) const{
	return Quaternion(this->s * other.s - this->v.dotProductWith(other.v), other.v * this->s + this->v * other.s + this->v.crossProductWith(other.v));
}

/**
	this method is used to set the current quaternion to the product of the two quaternions passed in as parameters.
	the bool parameters invA and invB indicate wether or not, the quaternion a or b should be inverted (well, complex conjugate really)
	for the multiplication
*/
void Quaternion::setToProductOf(const Quaternion& a, const Quaternion& b, bool invA, bool invB){
	double multA = (invA==false)?(1):(-1);
	double multB = (invB==false)?(1):(-1);
	this->s = a.s*b.s - a.v.dotProductWith(b.v) * multA * multB;
	this->v.setToCrossProduct(a.v, b.v);
	this->v.multiplyBy(multA * multB);

	this->v.addScaledVector(a.v, b.s*multA);
	this->v.addScaledVector(b.v, a.s*multB);
}


/**
	This operator multiplies the current quaternion by the rhs one. Keep in mind the note RE quaternion multiplication.
*/
Quaternion& Quaternion::operator *= (const Quaternion &other){
	double newS = this->s * other.s - this->v.dotProductWith(other.v);
	Vector3d newV = other.v * this->s + this->v * other.s + this->v.crossProductWith(other.v);
	this->s = newS;
	this->v = newV;
	return *this;
}

/**
	This method multiplies the current quaternion by a scalar.
*/
Quaternion& Quaternion::operator *= (double scalar){
	this->s *= scalar;
	this->v *= scalar;
	return *this;
}


/**
	This method returns a copy of the current quaternion multiplied by a scalar.
*/
Quaternion Quaternion::operator * (double scalar) const{
	return Quaternion(s * scalar, v * scalar);
}


/**
	This method returns a quaternion that was the result of adding the quaternion rhs to the current quaternion.
*/
Quaternion Quaternion::operator + (const Quaternion &other) const{
	return Quaternion(s + other.s, v + other.v);
}


/**
	This method adds the rhs quaternion to the current one.
*/
Quaternion& Quaternion::operator += (const Quaternion &other){
	this->s += other.s;
	this->v += other.v;
	return *this;
}

/**
	This method returns a quaternion that was the result of adding the quaternion rhs to the current quaternion.
*/
Quaternion Quaternion::operator - (const Quaternion &other) const{
	return Quaternion(s - other.s, v - other.v);
}


/**
	This method adds the rhs quaternion to the current one.
*/
Quaternion& Quaternion::operator -= (const Quaternion &other){
	this->s -= other.s;
	this->v -= other.v;
	return *this;
}


/**
	This method transforms the current quaternion to a unit quaternion.
*/
Quaternion& Quaternion::toUnit(){
	*this *= (1/this->getLength());
	return *this;
}


/**
	Assume that the current quaternion represents the relative orientation between two coordinate frames A and B.
	This method decomposes the current relative rotation into a twist of frame B around the axis v passed in as a
	parameter, and another more arbitrary rotation.

	AqB = AqT * TqB, where T is a frame that is obtained by rotating frame B around the axis v by the angle
	that is returned by this function.

	In the T coordinate frame, v is the same as in B, and AqT is a rotation that aligns v from A to that
	from T.

	It is assumed that vB is a unit vector!! rotAngle is the rotation angle around rotAxis (this gives the
	AqT transformation. To go from frame B to A, we then twist around the axis v by the amount returned
	by this function, and we then rotate around rotAxis by rotAngle.
*/
/*
double Quaternion::decomposeRotation(const Vector3d vB, double *rotAngle, Vector3d* rotAxis) const{
	//we need to compute v in A's coordinates
	Vector3d vA = this->rotate(vB);
	vA.toUnit();

	double temp = 0;

	//compute the rotation that alligns the vector v in the two coordinate frames (A and T)
	*rotAxis = vA.crossProductWith(vB);
	rotAxis->toUnit();
	*rotAngle = -safeACOS(vA.dotProductWith(vB));

	Quaternion TqA = Quaternion::getRotationQuaternion((*rotAngle), *rotAxis*(-1));
	Quaternion TqB = TqA * (*this);

	//note: q and -q represent the same rotation!
	if (TqB.s<0){
		TqB.s = -TqB.s;
		TqB.v = -TqB.v;
	}

	return 2 * safeACOS(TqB.getS()) * SGN(TqB.getV().dotProductWith(vB));
}*/

/**
	Assume that the current quaternion represents the relative orientation between two coordinate frames A and B.
	This method decomposes the current relative rotation into a twist of frame B around the axis v passed in as a
	parameter, and another more arbitrary rotation.

	AqB = AqT * TqB, where T is a frame that is obtained by rotating frame B around the axis v by the angle
	that is returned by this function.

	In the T coordinate frame, v is the same as in B, and AqT is a rotation that aligns v from A to that
	from T.

	It is assumed that vB is a unit vector!! This method returns TqB, which represents a twist about
	the axis vB.
*/
Quaternion Quaternion::decomposeRotation(const Vector3d vB) const{
	//we need to compute v in A's coordinates
	Vector3d vA = this->rotate(vB);
	vA.toUnit();

	double temp = 0;

	//compute the rotation that alligns the vector v in the two coordinate frames (A and T)
	Vector3d rotAxis = vA.crossProductWith(vB);
	rotAxis.toUnit();
	double rotAngle = -safeACOS(vA.dotProductWith(vB));

	Quaternion TqA = Quaternion::getRotationQuaternion(rotAngle, rotAxis*(-1));
	return TqA * (*this);
}

/**
	Assume that the current quaternion represents the relative orientation between two coordinate frames P and C (i.e. q
	rotates vectors from the child/local frame C into the parent/global frame P).

	With v specified in frame C's coordinates, this method decomposes the current relative rotation, such that:

	PqC = qA * qB, where qB represents a rotation about axis v.
	
	This can be thought of us as a twist about axis v - qB - and a more general rotation, and swing
	- qA - decomposition. Note that qB can be thought of as a rotation from the C frame into a tmp trame T,
	and qA a rotation from T into P.

	In the T coordinate frame, v is the same as in C, and qA is a rotation that aligns v from P to that
	from T.
*/
void Quaternion::decomposeRotation(Quaternion* qA, Quaternion* qB, const Vector3d& vC) const{
	//we need to compute v in P's coordinates
	Vector3d vP;
	this->fastRotate(vC, &vP);

	//compute the rotation that alligns the vector v in the two coordinate frames (P and T - remember that v has the same coordinates in C and in T)
	Vector3d rotAxis;
	rotAxis.setToCrossProduct(vP, vC);
	rotAxis.toUnit();
	double rotAngle = -safeACOS(vP.dotProductWith(vC));

	qA->setToRotationQuaternion(rotAngle, rotAxis);
	//now qB = qAinv * PqC
	qB->setToProductOf(*qA, *this, true, false);

	*qB = (*qA).getComplexConjugate() * (*this);
}

//add by Huansen//////////////////////////////////////////////////////////////////////////

void Quaternion::normalize()
{
	double distance = sqrt(s*s + v.x*v.x + v.y*v.y + v.z*v.z);
	if(fabs(distance) < 1e-6)
	{
		s = 1.0;
		v.x = v.y = v.z = 0.0;
	}else{
		s /= distance;
		v.x /= distance;
		v.y /= distance;
		v.z /= distance;
	}
}

// [@, x, y, z]-->[cos@/2, x*sin@/2, y*sin@/2, z*sin@/2]
void Quaternion::fromAxisAngle(const double angle, const double x, const double y, const double z)
{
	double sin_a = sin(angle / 2);
	double cos_a = cos(angle / 2);

	s = cos_a;
	v.x = x * sin_a;
	v.y = y * sin_a;
	v.z = z * sin_a;

	normalize();
}

void Quaternion::fromEulerAnglesXYZ(const double angle[3])
{
	Quaternion qresult, tmp;

	tmp.fromAxisAngle(angle[0], 1, 0, 0);
	qresult = tmp;
	tmp.fromAxisAngle(angle[1], 0, 1, 0);
	qresult = qresult * tmp;
	tmp.fromAxisAngle(angle[2], 0, 0, 1);
	qresult = qresult * tmp;

	qresult.normalize();
	s = qresult.s;
	v.x = qresult.v.x;
	v.y = qresult.v.y;
	v.z = qresult.v.z;
}


void Quaternion::fromEulerAnglesXZY(const double angle[3])
{
	Quaternion qresult, tmp;

	tmp.fromAxisAngle(angle[0], 1, 0, 0);
	qresult = tmp;
	tmp.fromAxisAngle(angle[1], 0, 0, 1);
	qresult = qresult * tmp;
	tmp.fromAxisAngle(angle[2], 0, 1, 0);
	qresult = qresult * tmp;

	qresult.normalize();
	s = qresult.s;
	v.x = qresult.v.x;
	v.y = qresult.v.y;
	v.z = qresult.v.z;
}

void Quaternion::fromEulerAnglesYXZ(const double angle[3])
{
	Quaternion qresult, tmp;

	tmp.fromAxisAngle(angle[0], 0, 1, 0);
	qresult = tmp;
	tmp.fromAxisAngle(angle[1], 1, 0, 0);
	qresult = qresult * tmp;
	tmp.fromAxisAngle(angle[2], 0, 0, 1);
	qresult = qresult * tmp;

	qresult.normalize();
	s = qresult.s;
	v.x = qresult.v.x;
	v.y = qresult.v.y;
	v.z = qresult.v.z;
}

void Quaternion::fromEulerAnglesYZX(const double angle[3])
{
	Quaternion qresult, tmp;

	tmp.fromAxisAngle(angle[0], 0, 1, 0);
	qresult = tmp;
	tmp.fromAxisAngle(angle[1], 0, 0, 1);
	qresult = qresult * tmp;
	tmp.fromAxisAngle(angle[2], 1, 0, 0);
	qresult = qresult * tmp;

	qresult.normalize();
	s = qresult.s;
	v.x = qresult.v.x;
	v.y = qresult.v.y;
	v.z = qresult.v.z;
}

void Quaternion::fromEulerAnglesZXY(const double angle[3])
{
	Quaternion qresult, tmp;

	tmp.fromAxisAngle(angle[0], 0, 0, 1);
	qresult = tmp;
	tmp.fromAxisAngle(angle[1], 1, 0, 0);
	qresult = qresult * tmp;
	tmp.fromAxisAngle(angle[2], 0, 1, 0);
	qresult = qresult * tmp;

	qresult.normalize();
	s = qresult.s;
	v.x = qresult.v.x;
	v.y = qresult.v.y;
	v.z = qresult.v.z;
}

void Quaternion::fromEulerAnglesZYX(const double angle[3])
{
	Quaternion qresult, tmp;

	tmp.fromAxisAngle(angle[0], 0, 0, 1);
	qresult = tmp;
	tmp.fromAxisAngle(angle[1], 0, 1, 0);
	qresult = qresult * tmp;
	tmp.fromAxisAngle(angle[2], 1, 0, 0);
	qresult = qresult * tmp;

	qresult.normalize();
	s = qresult.s;
	v.x = qresult.v.x;
	v.y = qresult.v.y;
	v.z = qresult.v.z;
}


// Matrix you get looks like this
//      -----------
//     |0  4  8  12|
// M = |1  5  9  13|
//     |2  6  10 14|
//     |3  7  11 15|
//      ----------- 
void Quaternion::getRotationMatrix(double* M) const
{	
	double xx, xy, xz, wx, yy, yz, wy, zz, wz;
	//normalize();

	xx = v.x * v.x;
	xy = v.x * v.y;
	xz = v.x * v.z;
	wx = s * v.x;

	yy = v.y * v.y;
	yz = v.y * v.z;
	wy = s * v.y;

	zz = v.z * v.z;
	wz = s * v.z;

	M[0]  = 1 - 2 * (yy + zz);
	M[1]  =     2 * (xy + wz);
	M[2]  =     2 * (xz - wy);

	M[4]  =     2 * (xy - wz);
	M[5]  = 1 - 2 * (xx + zz);
	M[6]  =     2 * (yz + wx);

	M[8]  =     2 * (xz + wy);
	M[9]  =     2 * (yz - wx);
	M[10] = 1 - 2 * (xx + yy);

	M[3] = M[7] = M[11] = M[12] = M[13] = M[14] = 0;
	M[15] = 1;
}

Vector3d Quaternion::getEulerAngle()
{
	// 获取旋转矩阵
	// Matrix you get looks like this
	//      -----------
	//     |0  4  8  12|
	// M = |1  5  9  13|
	//     |2  6  10 14|
	//     |3  7  11 15|
	//      ----------- 

	//Vector3d vtmp;

	//double rotmat[16];
	//getRotationMatrix(rotmat);

	//// 旋转矩阵转换成欧拉角

	//// 计算绕x轴旋转的弧度 rad[0]
	//double sp = rotmat[6];
	//if (sp <= -1.0f)
	//{
	//	vtmp[0] = -1.570796f;	// -pi/2
	//} 
	//else if (sp >= 1.0f)
	//{
	//	vtmp[0] = 1.570796f;		// pi/2
	//} 
	//else
	//{
	//	vtmp[0] = asin(sp);
	//}

	//// 检查万象锁的情况，允许一些误差
	//if (sp > 0.9999f)
	//{
	//	// 向正上或正下看
	//	vtmp[2] = 0.0f;
	//	vtmp[1] = atan2(-rotmat[2],rotmat[10]);
	//}
	//else
	//{
	//	vtmp[1] = atan2(-rotmat[2],rotmat[10]);
	//	vtmp[2] = atan2(-rotmat[4],rotmat[5]);
	//}

	/** assumes q1 is a normalized quaternion */

	normalize();

	Vector3d vtmp;

	double heading, attitude, bank;

	double test = v.x*v.y + v.z*s;
	if (test > 0.499) { // singularity at north pole
		heading = 2 * atan2(v.x, s);
		attitude = PI/2;
		bank = 0;

		vtmp.x = attitude;
		vtmp.y = heading;
		vtmp.z = bank;

		return vtmp;
	}
	if (test < -0.499) { // singularity at south pole
		heading = -2 * atan2(v.x, s);
		attitude = -PI/2;
		bank = 0;

		vtmp.x = attitude;
		vtmp.y = heading;
		vtmp.z = bank;

		return vtmp;
	}
	double sqx = v.x*v.x;
	double sqy = v.y*v.y;
	double sqz = v.z*v.z;
	heading = atan2(2*v.y*s - 2*v.x*v.z , 1 - 2*sqy - 2*sqz);
	attitude = asin(2*test);
	bank = atan2(2*v.x*s - 2*v.y*v.z , 1 - 2*sqx - 2*sqz);

	vtmp.x = attitude;
	vtmp.y = heading;
	vtmp.z = bank;
	
	return vtmp;
}

Vector3d Quaternion::Logmap()
{
	Vector3d ret;
	normalize();
	double fAngle = acos(s);
	double fSin = sqrt(1.0 - s * s);

	if(fabs(fSin) < 1e-6)
	{
		ret.x = 0;
		ret.y = 0;
		ret.z = 0;
	}else{
		double fCoeff = (fAngle) / fSin;
		ret.x = fCoeff * v.x;
		ret.y = fCoeff * v.y;
		ret.z = fCoeff * v.z;
	}
	return ret;
}

const void Quaternion::Expmap(Vector3d exp)
{
	double theta,x,y,z;
	theta = sqrt(exp[0]*exp[0]+exp[1]*exp[1]+exp[2]*exp[2]);
	if (fabs(theta) < 1e-6)
	{
		s = 1.0f;
		v.x = 0.0f;
		v.y = 0.0f;
		v.z = 0.0f;
	}else{
		x = exp[0] / theta;
		y = exp[1] / theta;
		z = exp[2] / theta;
		fromAxisAngle(theta*2, x, y, z);
	}
}
//end


// added by lvp   11-9-15/////////////////////////////////////////////////////////////////
// Matrix you get looks like this
//      -----------
//     |0  4  8  12|
// M = |1  5  9  13|
//     |2  6  10 14|
//     |3  7  11 15|
//      ----------- 
void Quaternion::fromTransformationMatrix(const Matrix& transmat)
{
	double		trace;	// The trace for the rotation matrix
	double		S;

	trace = transmat.get(0,0) + transmat.get(1,1) + transmat.get(2,2);

	if (trace > 0) { 
		double S = sqrt(trace+1.0) * 2; // S=4*qw 
		s = 0.25 * S;
		v.x = (transmat.get(2,1) - transmat.get(1,2)) / S;
		v.y = (transmat.get(0,2) - transmat.get(2,0)) / S; 
		v.z = (transmat.get(1,0) - transmat.get(0,1)) / S; 
	}else if(transmat.get(0,0) > transmat.get(1,1) && transmat.get(0,0) > transmat.get(2,2))
	{
		double S = sqrt(1.0 + transmat.get(0,0) - transmat.get(1,1) - transmat.get(2,2));	// S=4*qx
		s = (transmat.get(2,1) - transmat.get(1,2)) / S;
		v.x = 0.25 * S;
		v.y = (transmat.get(0,1) + transmat.get(1,0)) / S;
		v.z = (transmat.get(0,2) + transmat.get(2,0)) / S;
	}else if(transmat.get(1,1) > transmat.get(2,2))
	{
		double S = sqrt(1.0 + transmat.get(1,1) - transmat.get(0,0) - transmat.get(2,2)) * 2; // S=4*qy
		s = (transmat.get(0,2) - transmat.get(2,0)) / S;
		v.x = (transmat.get(0,1) + transmat.get(1,0)) / S;
		v.y = 0.25 * S;
		v.z = (transmat.get(1,2) + transmat.get(2,1)) / S;
	}else
	{
		double S = sqrt(1.0 + transmat.get(2,2) - transmat.get(0,0) - transmat.get(1,1)) * 2; //S=4*qz
		s = (transmat.get(1,0) - transmat.get(0,1)) / S; 
		v.x = (transmat.get(0,2) + transmat.get(2,0)) / S;
		v.y = (transmat.get(1,2) + transmat.get(2,1)) / S;
		v.z = 0.25 * S;
	}
}
void Quaternion::fromTransformationMatrix(const TransformationMatrix& transmat)
{
	double		trace;	// The trace for the rotation matrix
	double		S;

	trace = transmat.get(0,0) + transmat.get(1,1) + transmat.get(2,2);

	if (trace > 0) { 
		double S = sqrt(trace+1.0) * 2; // S=4*qw 
		s = 0.25 * S;
		v.x = (transmat.get(2,1) - transmat.get(1,2)) / S;
		v.y = (transmat.get(0,2) - transmat.get(2,0)) / S; 
		v.z = (transmat.get(1,0) - transmat.get(0,1)) / S; 
	}else if(transmat.get(0,0) > transmat.get(1,1) && transmat.get(0,0) > transmat.get(2,2))
	{
		double S = sqrt(1.0 + transmat.get(0,0) - transmat.get(1,1) - transmat.get(2,2));	// S=4*qx
		s = (transmat.get(2,1) - transmat.get(1,2)) / S;
		v.x = 0.25 * S;
		v.y = (transmat.get(0,1) + transmat.get(1,0)) / S;
		v.z = (transmat.get(0,2) + transmat.get(2,0)) / S;
	}else if(transmat.get(1,1) > transmat.get(2,2))
	{
		double S = sqrt(1.0 + transmat.get(1,1) - transmat.get(0,0) - transmat.get(2,2)) * 2; // S=4*qy
		s = (transmat.get(0,2) - transmat.get(2,0)) / S;
		v.x = (transmat.get(0,1) + transmat.get(1,0)) / S;
		v.y = 0.25 * S;
		v.z = (transmat.get(1,2) + transmat.get(2,1)) / S;
	}else
	{
		double S = sqrt(1.0 + transmat.get(2,2) - transmat.get(0,0) - transmat.get(1,1)) * 2; //S=4*qz
		s = (transmat.get(1,0) - transmat.get(0,1)) / S; 
		v.x = (transmat.get(0,2) + transmat.get(2,0)) / S;
		v.y = (transmat.get(1,2) + transmat.get(2,1)) / S;
		v.z = 0.25 * S;
	}
}
void Quaternion::fromTransformationMatrix(const double* M)
{
	double		trace;	// The trace for the rotation matrix
	double		S;

	trace = M[0] + M[5] + M[10];

	if (trace > 0) { 
		double S = sqrt(trace+1.0) * 2; // S=4*qw 
		s = 0.25 * S;
		v.x = (M[6] - M[9]) / S;
		v.y = (M[8] - M[2]) / S; 
		v.z = (M[1] - M[4]) / S; 
	}else if(M[0] > M[5] && M[0 > M[10]])
	{
		double S = sqrt(1.0 + M[0] - M[5] - M[10]);	// S=4*qx
		s = (M[6] - M[9]) / S;
		v.x = 0.25 * S;
		v.y = (M[4] + M[1]) / S;
		v.z = (M[8] + M[2]) / S;
	}else if(M[5] > M[10])
	{
		double S = sqrt(1.0 + M[5] - M[0] - M[10]) * 2; // S=4*qy
		s = (M[8] - M[2]) / S;
		v.x = (M[4] + M[1]) / S;
		v.y = 0.25 * S;
		v.z = (M[9] + M[6]) / S;
	}else
	{
		double S = sqrt(1.0 + M[10] - M[0] - M[5]) * 2; //S=4*qz
		s = (M[1] - M[4]) / S; 
		v.x = (M[8] + M[2]) / S;
		v.y = (M[9] + M[6]) / S;
		v.z = 0.25 * S;
	}

}
// end
}