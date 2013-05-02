#ifndef _POSTURE_H
#define _POSTURE_H

#include "..\MEMathLib\Vector3d.h"

#define MAX_BONES_IN_ASF_FILE 256
#define MAX_CHAR 1024
#define MAX_SKELS 16

using namespace MathLib;

//Root position and all bone rotation angles (including root) 
class Posture
{
	//member functions
	public:
		Posture();
		~Posture();

		friend Posture LinearInterpolate(float, Posture const&, Posture const& );

	//member variables
	public:

		//Root position (x, y, z)		
		Vector3d root_pos;								
		
		//Rotation (x, y, z) of all bones at a particular time frame in their local coordinate system.
		//If a particular bone does not have a certain degree of freedom, 
		//the corresponding rotation is set to 0.
		//The order of the bones in the array corresponds to their ids in .ASf file: root, lhipjoint, lfemur, ...
		Vector3d bone_rotation[MAX_BONES_IN_ASF_FILE];
		Vector3d bone_translation[MAX_BONES_IN_ASF_FILE];
		float bone_length[MAX_BONES_IN_ASF_FILE];
		Vector3d joint_pos[MAX_BONES_IN_ASF_FILE];

	Posture& operator = (const Posture& posture);

public:
	/*
		obtain the distance between two poses, which are measured in joint angle space using degree
	**/
	double getDistanceWithInDegree(const Posture& posture);
};

#endif