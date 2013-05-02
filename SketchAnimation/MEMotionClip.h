/*
	Define the motion trajectory of clips
*/
#pragma once

#include <string>
#include <vector>
#include "../MEMathLib/Vector3d.h"

using namespace std;
using namespace MathLib;

class CMEMotionClip
{
public:
	CMEMotionClip(void);
	~CMEMotionClip(void);

	string getMotionClipName(){return m_sClipName;}
	void setMotionClipName(char* name){m_sClipName = name;}
	
public:
	vector< vector<Vector3d> > m_vJointTrajByPos;  

private:
	string m_sClipName;
};

