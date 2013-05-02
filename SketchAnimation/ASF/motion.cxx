#include <stdio.h>
#include <string.h>
#include <fstream>
#include <math.h>

#include "skeleton.h"
#include "motion.h"

// a default skeleton that defines each bone's degree of freedom and the order of the data stored in the AMC file
//static Skeleton actor("Skeleton.ASF", MOCAP_SCALE);
typedef float * floatptr;

using namespace std;

/************************ Motion class functions **********************************/

Motion::Motion(int nNumFrames)
{
//	m_NumDOFs = pActor.m_NumDOFs;
	
	m_NumFrames = nNumFrames;
	offset = 0;

	//allocate postures array
	m_pPostures = new Posture [m_NumFrames];

	//Set all postures to default posture
	SetPosturesToDefault();
}

Motion::Motion(const char *amc_filename, float scale,Skeleton * pActor2)
{
	pActor = pActor2;

//	m_NumDOFs = actor.m_NumDOFs;
	offset = 0;
	m_NumFrames = 0;
	m_pPostures = NULL;
	readAMCfile(amc_filename, scale);	
}

Motion::Motion(const char *amc_filename, float scale)
{
//	m_NumDOFs = actor.m_NumDOFs;
	offset = 0;
	m_NumFrames = 0;
	m_pPostures = NULL;
	readAMCfile(amc_filename, scale);
}


Motion::~Motion()
{
	if (m_pPostures != NULL)
		delete [] m_pPostures;
}


//Set all postures to default posture
void Motion::SetPosturesToDefault()
{
	//for each frame
	//int numbones = numBonesInSkel(bone[0]);
	//modified by lgd, 2007-3-23
	//for (int i = 0; i<MAX_BONES_IN_ASF_FILE; i++) //??
	for (int i = 0; i<m_NumFrames-1; i++) 
	{
		//set root position to (0,0,0)
		m_pPostures[i].root_pos = Vector3d(0.0,0.0,0.0);
		//set each bone orientation to (0,0,0)
		for (int j = 0; j < MAX_BONES_IN_ASF_FILE; j++)
			m_pPostures[i].bone_rotation[j] = Vector3d(0.0,0.0,0.0);
	}
}

//Set posture at spesified frame
void Motion::SetPosture(int nFrameNum, Posture InPosture)
{
	m_pPostures[nFrameNum] = InPosture; 	
}

int Motion::GetPostureNum(int nFrameNum)
{
	nFrameNum += offset;

	if (nFrameNum < 0)
		return 0;
	else if (nFrameNum >= m_NumFrames)
		return m_NumFrames-1;
	else
		return nFrameNum;
	return 0;
}

void Motion::SetTimeOffset(int n_offset)
{
	offset = n_offset;
}

void Motion::SetBoneRotation(int nFrameNum, Vector3d vRot, int nBone)
{
	m_pPostures[nFrameNum].bone_rotation[nBone] = vRot;
}

void Motion::SetRootPos(int nFrameNum, Vector3d vPos)
{
	m_pPostures[nFrameNum].root_pos = vPos;
}


Posture* Motion::GetPosture(int nFrameNum)
{
	if (m_pPostures != NULL) 
		return &m_pPostures[nFrameNum]; 
	else 
		return NULL;
}


int Motion::readAMCfile(const char* name, float scale)
{
	Bone *hroot, *bone;
	bone = hroot= (*pActor).getRoot();

	// -- modified by lzy
	// ifstream file( name, ios::in | ios::nocreate );
	ifstream file(name, ios::in | ifstream::_Nocreate);	
	if( file.fail() ) return -1;

	int n=0;
	char str[2048];

	// count the number of lines
	while(!file.eof())  
	{
		file.getline(str, 2048);
		if(file.eof()) break;
		//We do not want to count empty lines
		if (strcmp(str, "") != 0)
			n++;
	}

	file.close();

	//Compute number of frames. 
	//Subtract 3 to  ignore the header
	//There are (NUM_BONES_IN_ASF_FILE - 2) moving bones and 2 dummy bones (lhipjoint and rhipjoint)
	int numbones = numBonesInSkel(bone[0]);
	int movbones = movBonesInSkel(bone[0]);
	n = (n-3)/((movbones) + 1);   

	m_NumFrames = n;

	//Allocate memory for state vector
	m_pPostures = new Posture [m_NumFrames]; 

	// -- modified by lzy, add
	file.clear();
	
	file.open( name );	

	// skip the header
	while (1) 
	{
		file >> str;
		if(strcmp(str, ":DEGREES") == 0) break;
	}

	int frame_num;
	float x, y, z;
	// -- modified by lzy
	//int i, bone_idx, state_idx;
	int i, bone_idx;	

	//对每一帧
	for(i=0; i<m_NumFrames; i++)
	{
		//read frame number
		file >> frame_num;
		x=y=z=0;

		//There are (NUM_BONES_IN_ASF_FILE - 2) moving bones and 2 dummy bones (lhipjoint and rhipjoint)
		//对每一根活动的骨骼
		for( int j=0; j<movbones; j++ )
		{
			//read bone name
			file >> str;
			
			//Convert to corresponding integer
			//找到AMC文件中每一帧中某一个骨骼所对应的索引号
			for( bone_idx = 0; bone_idx < numbones; bone_idx++ )
			//if( strcmp( str, AsfPartName[bone_idx] ) == 0 ) 
				if( strcmp( str, pActor->idx2name(bone_idx) ) == 0 ) 
					break;

			//init rotation angles for this bone to (0, 0, 0)
			m_pPostures[i].bone_rotation[bone_idx] = Vector3d(0.0,0.0,0.0);

            //对每一个可能的自由度
			for(int x = 0; x < bone[bone_idx].dof; x++)
			{
				float tmp;
				file >> tmp;
			//	printf("%d %f\n",bone[bone_idx].dofo[x],tmp);
				switch (bone[bone_idx].dofo[x]) 
				{
					case 0:
						printf("FATAL ERROR in bone %d not found %d\n",bone_idx,x);
						x = bone[bone_idx].dof;
						break;
					case 1:
						m_pPostures[i].bone_rotation[bone_idx][0] = tmp;//rx
						break;
					case 2:
						m_pPostures[i].bone_rotation[bone_idx][1] = tmp;//ry
						break;
					case 3:
						m_pPostures[i].bone_rotation[bone_idx][2] = tmp;//rz
						break;
					case 4:
						m_pPostures[i].bone_translation[bone_idx][0] = tmp * scale;//tx
						break;
					case 5:
						m_pPostures[i].bone_translation[bone_idx][1] = tmp * scale;//ty
						break;
					case 6:
						m_pPostures[i].bone_translation[bone_idx][2] = tmp * scale;//tz
						break;
					case 7:
						m_pPostures[i].bone_length[bone_idx] = tmp;// * scale; l
						break;
				}
			}
			//设置root节点的位置
			if( strcmp( str, "root" ) == 0 ) 
			{
				m_pPostures[i].root_pos[0] = m_pPostures[i].bone_translation[0][0];// * scale;
				m_pPostures[i].root_pos[1] = m_pPostures[i].bone_translation[0][1];// * scale;
				m_pPostures[i].root_pos[2] = m_pPostures[i].bone_translation[0][2];// * scale;
			}
			// read joint angles, including root orientation
			
		}
	}

	file.close();
	printf("%d samples in '%s' are read.\n", n, name);
	return n;
}

int Motion::writeAMCfile(const char *filename, float scale)
{
	//modified by LGD
	//int f, n, j, d;
	int f,n,j;
	Bone *bone;
	bone=(*pActor).getRoot();

	ofstream os(filename);
	if(os.fail()) return -1;


	// header lines
	os << "#Unknow ASF file" << endl;
	os << ":FULLY-SPECIFIED" << endl;
	os << ":DEGREES" << endl;
	int numbones = numBonesInSkel(bone[0]);

	for(f=0; f < m_NumFrames; f++)
	{
		os << f+1 <<endl;
		os << "root " << m_pPostures[f].root_pos[0]/scale << " " 
			          << m_pPostures[f].root_pos[1]/scale << " " 
					  << m_pPostures[f].root_pos[2]/scale << " " 
					  << m_pPostures[f].bone_rotation[root][0] << " " 
					  << m_pPostures[f].bone_rotation[root][1] << " " 
					  << m_pPostures[f].bone_rotation[root][2] ;
		n=6;
		
		for(j = 2; j < numbones; j++) 
		{

			//output bone name
			if(bone[j].dof != 0)
//				os << endl << AsfPartName[j];
				os << endl << pActor->idx2name(j);

			//output bone rotation angles
			if(bone[j].dofx == 1) 
				os << " " << m_pPostures[f].bone_rotation[j][0];

			if(bone[j].dofy == 1) 
				os << " " << m_pPostures[f].bone_rotation[j][1];

			if(bone[j].dofz == 1) 
				os << " " << m_pPostures[f].bone_rotation[j][2];
		}
		os << endl;
	}

	os.close();
	printf("Write %d samples to '%s' \n", m_NumFrames, filename);
	return 0;
}



