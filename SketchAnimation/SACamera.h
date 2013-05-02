#pragma once

#include <fstream>
using namespace std;

class CSACamera
{
public:
	CSACamera(void);
	~CSACamera(void);

	//to simplify things, we will keep track of the rotation using this vector. This way, we won't need to mix rotations about the x, y and z axis
	double m_rotations[3];
	//and also how far it is along its z axis (assuming that it is looking down the  -z axis of its local frame)
	double m_camDistance;
	//we will also keep track of the point we are looking at, in world coordinates
	double m_target[3];

	double m_pan[2];

	double m_center[3];

	double m_up[3];

	double m_camPos[3];

	double m_fov;

	// print operator
	friend std::ostream& operator<<(std::ostream& os, const CSACamera& camera);
	friend std::istream& operator>>(std::istream& is, CSACamera& camera);

	// assignment operation
	CSACamera& operator= (const CSACamera& camera);

	// compute the global position of the camera
	void computePositionInModelSpace(float* pos);
};

