/*
	skeleton.h

	Definition of the skeleton. 

    Revision 1 - Steve Lin, Jan. 14, 2002
 	Revision 2 - Alla and Kiran, Jan 18, 2002
*/

#ifndef _SKELETON_H
#define _SKELETON_H

#include "../Entity.h"

#include "posture.h"
#include <windows.h>
#include <gl\GL.h>
#include <gl\GLU.H>

#include <vector>
#include <string>

// Bone segment names used in ASF file
static int root = 0;

// the starting point of name stack for joints
extern int JOINT_NAME_STACK;

// this structure defines the property of each bone segment, including its connection to other bones,
// DOF (degrees of freedom), relative orientation and distance to the outboard bone			
struct Bone {

	// -- modified by lzy, add
	Bone();
   
	struct Bone *sibling;		// Pointer to the sibling (branch bone) in the hierarchy tree 
	struct Bone *child;			// Pointer to the child (outboard bone) in the hierarchy tree 
   
	int idx;					// Bone index
	
	//骨骼伸展的方向，在局部坐标系内
	float dir[3];				// Unit vector describes the direction from local origin to 
								// the origin of the child bone 
								// Notice: stored in local coordinate system of the bone

	float length;				// Bone length  

	//骨骼所在局部坐标系的旋转角
	float axis_x, axis_y, axis_z;// orientation of each bone's local coordinate 
								 //system as specified in ASF file (axis field)

	float aspx, aspy;			// aspect ratio of bone shape

	int dof;					// number of bone's degrees of freedom 
	int dofx, dofy, dofz;		// degree of freedom mask in x, y, z axis (local)
	int doftx, dofty, doftz;
	int doftl;
								// dofx=1 if this bone has x degree of freedom, otherwise dofx=0.
	
	char name[256];
	
	// rotation matrix from the local coordinate of this bone to the local coordinate system of it's parent
	double rot_parent_current[4][4];			
	
	//Rotation angles for this bone at a particular time frame (as read from AMC file) in local coordinate system, 
	//they are set in the setPosture function before display function is called
	float drx, dry, drz;
	float tx,ty,tz;
	float tl;
	int dofo[8];

	// bone position, added by lvp 12-10-22
	float m_GlobalPosition[3];
	bool m_bSelected;			// whether this bone is selected
	bool m_bDrawn;				// whether this bone is drawn
	char parent_name[256];
};


class Skeleton {

  //Member functions
  public: 

	// The scale parameter adjusts the size of the skeleton. The default value is 0.06 (MOCAP_SCALE).
    // This creates a human skeleton of 1.7 m in height (approximately)
    Skeleton(const char *asf_filename, float scale);  
    ~Skeleton();                                

    //Get root node's address; for accessing bone data
    Bone* getRoot();

	//Set the skeleton's pose based on the given posture    
	void setPosture(Posture posture);        

	//Initial posture Root at (0,0,0)
	//All bone rotations are set to 0
    void setBasePosture();

	//Rendering method: override the cEntity Base Class's function
	bool Render();

	// compute the global position for all the bones, added by lvp 12-10-22
	void updateGlobalPosition();
	void computeChildGlobalPosition(Bone* pBone);

	//This recursive function traverses skeleton hierarchy 
	//and returns a pointer to the bone with index - bIndex
	//ptr should be a pointer to the root node 
	//when this function first called
	Bone* getBone(Bone *ptr, int bIndex);

private:

	//parse the skeleton (.ASF) file	
    void readASFfile(const char* asf_filename, float scale);
 
	//This function sets sibling or child for parent bone
	//If parent bone does not have a child, 
	//then pChild is set as parent's child
	//else pChild is set as a sibling of parents already existing child
	int setChildrenAndSibling(int parent, Bone *pChild);

	//Rotate all bone's direction vector (dir) from global to local coordinate system
	void RotateBoneDirToLocalCoordSystem();

//Member Variables
public:
	// root position in world coordinate system
    float m_RootPos[3];
	int tx,ty,tz;
	int rx,ry,rz;
    // number of DOF of the skeleton
	// DEBUG: remove this variable???
	// int m_NumDOFs;
	int name2idx(char *);
	char * idx2name(int);
	int NUM_BONES_IN_ASF_FILE;
	int MOV_BONES_IN_ASF_FILE;

public:
	Bone *m_pRootBone;							// Pointer to the root bone, m_RootBone = &bone[0]
	Bone  m_pBoneList[MAX_BONES_IN_ASF_FILE];   // Array with all skeleton bones

  // added by lvp 2012-2-7
private:
	GLuint m_BoneList[MAX_SKELS];
	GLuint m_JointList[MAX_SKELS];

	double m_scale;
	bool m_bLoaded;		// indicate whether this asf file is loaded successfully

public:
	bool isLoaded(){return m_bLoaded;}

	void RenderFigure(float* bone_color, float* joint_color);
	void RenderFigureByJointPosition(Bone*,float* joint_color);
	void RenderFigureWithLines(float *bone_color);	//added by zpy
	void RenderLimbWithLines(const std::vector<std::string> & vSketchedBones, float* bone_color);	//
	
	void RenderSketchedFigure(const std::vector<std::string> &vSketchedBones, float aSkeBoneColor[4], float aSkeJointColor[4],
		                      float aBoneColor[4], float aJointColor[4]);
	
	/*
		render certain joint by its position through joint name or joint idx
	*/
	void RenderJoint(char* name);
	void RenderJoint(int idx);

	Bone* findBoneByName(Bone*, const char* name);
	Bone* findBoneByIdx(Bone*,int idx);

	// draw a particular bone
	void RenderBone(Bone* pBone, int skelNum);
	void RenderSketchedBone(const std::vector<std::string> &vSketchedBones,Bone *pBone, int skelNum);
	void RenderBoneWithLine(Bone *pBone, int skelNum);//added by zpy
	void RenderBoneWithLine(const std::vector<std::string> & vSketchedBones, Bone *pBone, int skelNum);

	// draw the skeleton hierarchy
	void traverse(Bone* ptr, int skelNum);
	void traverseSketched(const std::vector<std::string> &vSketchedBones,Bone *ptr, int skelNum);
	void traverseWithLines(Bone *ptr, int skelNum);
	void traverseLimbWithLines(const std::vector<std::string> &vSketchedBones, Bone *ptr, int skelNum);

	//Pre-draw the bones using quadratic object drawing function
	//and store them in the display list
	void set_display_list();
	void draw_bone_axis();

	void setScale(double scale){m_scale = scale;}
	double getScale(){return m_scale;}

// added by lvp 2012-9-19
private:
	float m_vBone_Color[4];
	float m_vJoint_Color[4];
public:
	void setBoneColor(float*);
	void setJointColor(float*);

// added by zpy 2012-11-8
private:
	float m_vSketchedBone_Color[4];
	float m_vSketchedJoint_Color[4];

public:
	void setSketchedBoneColor(float* aColor){memcpy(m_vSketchedBone_Color, aColor, sizeof(float)*4);}
	void setSketchedJointColor(float* aColor){memcpy(m_vSketchedJoint_Color, aColor, sizeof(float)*4);}
	//...
};

int numBonesInSkel(Bone item);
int movBonesInSkel(Bone item);

#endif
