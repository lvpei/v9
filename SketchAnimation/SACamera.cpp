#include "SACamera.h"
#include "MEMathLib/Vector3d.h"
#include "MEMathLib/TransformationMatrix.h"
#include <Windows.h>
#include "gl/GL.h"

using namespace MathLib;

CSACamera::CSACamera(void)
{
}


CSACamera::~CSACamera(void)
{
}

CSACamera& CSACamera::operator=(const CSACamera& camera)
{
	// rotations
	for(int i = 0; i < 3; i++)
		m_rotations[i] = camera.m_rotations[i];

	// camera distance
	m_camDistance =  camera.m_camDistance;

	// target
	for(int i = 0; i < 3; i++)
		m_target[i] = camera.m_target[i];

	// pan
	for(int i = 0; i < 2; i++)
		m_pan[i] = camera.m_pan[i];

	// center
	for(int i = 0; i < 3; i++)
		m_center[i] = camera.m_center[i];

	// up direction
	for(int i = 0; i < 3; i++)
		m_up[i] = camera.m_up[i];

	// camera position
	for(int i = 0; i < 3; i++)
		m_camPos[i] = camera.m_camPos[i];

	// camera fov
	m_fov = camera.m_fov;

	return *this;
}

std::ostream& operator<<(std::ostream& os, const CSACamera& camera)
{
	// rotations
	for(int i = 0; i < 3; i++)
		os<<camera.m_rotations[i]<<" ";

	// camera distance
	os<< camera.m_camDistance<<" ";

	// target
	for(int i = 0; i < 3; i++)
		os<<camera.m_target[i]<<" ";

	// pan
	for(int i = 0; i < 2; i++)
		os<<camera.m_pan[i]<<" ";

	// center
	for(int i = 0; i < 3; i++)
		os<<camera.m_center[i]<<" ";

	// up direction
	for(int i = 0; i < 3; i++)
		os<<camera.m_up[i]<<" ";

	// camera position
	for(int i = 0; i < 3; i++)
		os<<camera.m_camPos[i]<<" ";

	// camera fov
	os<<camera.m_fov;

	os<<"\n";

	return os;
}
std::istream& operator>>(std::istream& is, CSACamera& camera)
{
	// rotations
	for(int i = 0; i < 3; i++)
		is>>camera.m_rotations[i];

	// camera distance
	is>> camera.m_camDistance;

	// target
	for(int i = 0; i < 3; i++)
		is>>camera.m_target[i];

	// pan
	for(int i = 0; i < 2; i++)
		is>>camera.m_pan[i];

	// center
	for(int i = 0; i < 3; i++)
		is>>camera.m_center[i];

	// up direction
	for(int i = 0; i < 3; i++)
		is>>camera.m_up[i];

	// camera position
	for(int i = 0; i < 3; i++)
		is>>camera.m_camPos[i];

	is>>camera.m_fov;

	return is;
}

// compute the global position of the camera
void CSACamera::computePositionInModelSpace(float* pos)
{
	/*
	TransformationMatrix translationmat, rotationmat, rotX,rotY,rotZ;
	
	// rotation part
	rotX.setToRotationMatrix(-m_rotations[0],Vector3d(1.0,0.0,0.0));
	rotY.setToRotationMatrix(-m_rotations[1],Vector3d(0.0,1.0,0.0));
	rotZ.setToRotationMatrix(-m_rotations[2],Vector3d(0.0,0.0,1.0));

	rotationmat.setToProductOf(rotZ,rotY);
	rotationmat.setToProductOf(rotationmat,rotX);

	translationmat.setToTranslationMatrix(ThreeTuple(m_pan[0] + m_target[0],m_pan[1] + m_target[1],m_target[2]));
	
	rotationmat.setToProductOf(rotationmat,translationmat);
	// transform the camera
	Vector3d newpos;
	rotationmat.postMultiplyVector(Vector3d(m_camPos[0],m_camPos[1],m_camPos[2]),newpos);

	pos[0] = newpos.getX();
	pos[1] = newpos.getY();
	pos[2] = newpos.getZ();
	
	//*/
	
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW_MATRIX);
	glLoadIdentity();

	glRotatef(-m_rotations[2],0.0,0.0,1.0);
	glRotatef(-m_rotations[1],0.0,1.0,0.0);
	glRotatef(-m_rotations[0],1.0,0.0,0.0);

	// Points camera to desired target.
	glTranslated(m_target[0], m_target[1], m_target[2]);

	// Panning the translation.
	glTranslated(m_pan[0], m_pan[1], 0.0);

	GLdouble modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelview);

	TransformationMatrix transmat;
	transmat.setOGLValues(modelview);
	Vector3d newpos;
	transmat.postMultiplyVector(Vector3d(m_camPos[0],m_camPos[1],m_camPos[2]),newpos);
	pos[0] = newpos.getX();
	pos[1] = newpos.getY();
	pos[2] = newpos.getZ();

	glPopMatrix();
	//*/
}
