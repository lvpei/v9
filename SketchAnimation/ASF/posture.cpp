#include "posture.h"

/************************ Posture class functions **********************************/

//Output Posture  = (1-t)*a + t*b
//Linear interpolation the two root positions and two corresponding bones rotations between two human postures, commented by LGD
using namespace MathLib;

Posture::Posture()
{

}

Posture::~Posture()
{

}

Posture 
LinearInterpolate(float t, Posture const& a, Posture const& b )
{
	Posture InterpPosture;

	//Iterpolate root position
	InterpPosture.root_pos = interpolate(t, a.root_pos, b.root_pos);

	//Interpolate bones rotations
	for (int i = 0; i < MAX_BONES_IN_ASF_FILE; i++)
		InterpPosture.bone_rotation[i] = interpolate(t, a.bone_rotation[i], b.bone_rotation[i]);

	return InterpPosture;
}

Posture& Posture::operator = (const Posture& posture)
{
	root_pos = posture.root_pos;
	for(int i = 0; i < MAX_BONES_IN_ASF_FILE; i++)
	{
		bone_rotation[i] = posture.bone_rotation[i];
		bone_translation[i] = posture.bone_translation[i];
		bone_length[i] = posture.bone_length[i];
		joint_pos[i] = posture.joint_pos[i];
	}
	return *this;
}

/*
	obtain the distance between two poses, which are measured in joint angle space using degrees
**/
double Posture::getDistanceWithInDegree(const Posture& posture)
{
	double dist = 0.0; 
	
	// for all the joints
	for(int i = 0; i < MAX_BONES_IN_ASF_FILE; i++)
	{
		Vector3d diff = bone_rotation[i] - posture.bone_rotation[i];
		dist +=  diff.length();
	}

	return dist;
}