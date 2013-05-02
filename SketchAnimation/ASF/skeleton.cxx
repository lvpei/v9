#include <fstream>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "skeleton.h"
#include "..\MEMathLib\Vector3d.h"
#include "..\MEMathLib\TransformationMatrix.h"

using namespace MathLib;
using namespace std;

/***********************************************************************************************************

   Read skeleton file

***********************************************************************************************************/
//计算骨架中骨骼的数量
int numBonesInSkel(Bone item)
{
	Bone * tmp = item.sibling;
	int i = 0;
	while (tmp != NULL) 
	{
		if (tmp->child != NULL)
			i+= numBonesInSkel(*(tmp->child));
		i++; tmp = tmp->sibling; 
	}
	if (item.child != NULL)
		return i+1+numBonesInSkel(*item.child);
	else
		return i+1;
}

//计算骨架中活动骨骼的数量
int movBonesInSkel(Bone item)
{
	Bone * tmp = item.sibling;
	int i = 0;

	if (item.dof > 0) i++;

	while (tmp != NULL) 
	{
		if (tmp->child != NULL)
			i+= movBonesInSkel(*(tmp->child));
		if (tmp->dof > 0) i++; tmp = tmp->sibling; 
	}

	if (item.child != NULL)
		return i+movBonesInSkel(*item.child);
	else
		return i;
}

// helper function to convert ASF part name into bone index
int Skeleton::name2idx(char *name)
{
	int i=0;
	while(strcmp(m_pBoneList[i].name, name) != 0 && i++ < NUM_BONES_IN_ASF_FILE);
		return m_pBoneList[i].idx;
}

char * Skeleton::idx2name(int idx)
{
	int i=0;
	while(m_pBoneList[i].idx != idx && i++ < NUM_BONES_IN_ASF_FILE);
		return m_pBoneList[i].name;
}

void Skeleton::readASFfile(const char* asf_filename, float scale)
{
	//open file
	// -- modified by lzy
	//ifstream is(asf_filename, ios::in | ios::nocreate);
	//打开文件
	ifstream is(asf_filename, ios::in | ios::_Nocreate);
	if (is.fail()) 
		return;
	//
	// ignore header information
	//
	// -- modified by lzy, commented
	//int n;
	char	str[2048], keyword[256];
	//忽略文件头并跳过
	while (1)
	{
		is.getline(str, 2048);	
		sscanf(str, "%s", keyword);
		if (strcmp(keyword, ":bonedata") == 0)	break;
	}
	
	//
	// read bone information: global orientation and translation, DOF.
	//
	is.getline(str, 2048);
	// -- modified by lzy
	//char	part[256], *token;
	char	*token;
	float length;

	bool done = false; //控制ASF中骨骼数据何时读取完成
	for(int i = 1; !done && i < MAX_BONES_IN_ASF_FILE ; i++)
	{		
		//对自由度及其掩码进行初始化
		m_pBoneList[i].dof=0;
		//m_pBoneList[i].dofty=0;
		m_pBoneList[i].dofx=m_pBoneList[i].dofy=m_pBoneList[i].dofz=0;
		m_pBoneList[i].doftx=m_pBoneList[i].dofty=m_pBoneList[i].doftz=0;
		// -- modified by lzy, commented
		NUM_BONES_IN_ASF_FILE++;//对骨骼进行计数
		MOV_BONES_IN_ASF_FILE++;//对活动骨骼进行计数
		while(1)
		{
			is.getline(str, 2048);	sscanf(str, "%s", keyword);

			if(strcmp(keyword, "end") == 0) { break; }

			//当读取到骨架分层处，跳出该while循环，后续的while循环将对骨架结构进行解析和构建
			if(strcmp(keyword, ":hierarchy") == 0) { MOV_BONES_IN_ASF_FILE-=1; NUM_BONES_IN_ASF_FILE -= 1; done=true; break; }			

			//id of bone
			if(strcmp(keyword, "id") == 0)
			//	sscanf(str, "%s %d", keyword, &m_pBoneList[i].idx);
			{
				m_pBoneList[i].idx=NUM_BONES_IN_ASF_FILE-1;
			}
			//name of the bone
			if(strcmp(keyword, "name") == 0) {
				// -- modified by lzy, commented
				//sscanf(str, "%s %s", keyword, part);
				sscanf(str, "%s %s", keyword, m_pBoneList[i].name);
			}
			
			//this line describes the bone's direction vector in global coordinate
			//it will later be converted to local coorinate system
			if(strcmp(keyword, "direction") == 0)  
				sscanf(str, "%s %f %f %f", keyword, &m_pBoneList[i].dir[0], &m_pBoneList[i].dir[1], &m_pBoneList[i].dir[2]);
			
			//length of the bone
			if(strcmp(keyword, "length") == 0)  
				sscanf(str, "%s %f", keyword, &length);

			//this line describes the orientation of bone's local coordinate 
			//system relative to the world coordinate system
			if(strcmp(keyword, "axis") == 0)      
				sscanf(str, "%s %f %f %f", keyword, &m_pBoneList[i].axis_x, &m_pBoneList[i].axis_y, &m_pBoneList[i].axis_z);

			// this line describes the bone's dof 
			if(strcmp(keyword, "dof") == 0)       
			{
				token=strtok(str, " "); 
				// -- modified by lzy, commented
				//m_pBoneList[i].dof=0;
				while(token != NULL)      
				{
					int tdof = m_pBoneList[i].dof;

					if(strcmp(token, "rx") == 0) { m_pBoneList[i].dofx = 1; m_pBoneList[i].dofo[tdof] = 1; }
					else if(strcmp(token, "ry") == 0) { m_pBoneList[i].dofy = 1; m_pBoneList[i].dofo[tdof] = 2; }
					else if(strcmp(token, "rz") == 0) { m_pBoneList[i].dofz = 1; m_pBoneList[i].dofo[tdof] = 3; }
					else if(strcmp(token, "tx") == 0) { m_pBoneList[i].doftx = 1; m_pBoneList[i].dofo[tdof] = 4; }
					else if(strcmp(token, "ty") == 0) { m_pBoneList[i].dofty = 1; m_pBoneList[i].dofo[tdof] = 5; }
					else if(strcmp(token, "tz") == 0) { m_pBoneList[i].doftz = 1; m_pBoneList[i].dofo[tdof] = 6; }
					else if(strcmp(token, "l") == 0)  { m_pBoneList[i].doftl = 1; m_pBoneList[i].dofo[tdof] = 7; }
					else if(strcmp(token, "dof") == 0) { goto end; }
					else { printf("UNKNOWN %s\n",token); }

					m_pBoneList[i].dof++;
					m_pBoneList[i].dofo[m_pBoneList[i].dof] = 0;
					
					end:
						token=strtok(NULL, " ");
				}
				// m_NumDOFs += m_pBoneList[i].dof;
				printf("Bone %d DOF: ",i);
				for (int x = 0; (x < 7) && (m_pBoneList[i].dofo[x] != 0); x++) printf("%d ",m_pBoneList[i].dofo[x]);
				printf("\n");
			}


		}
		//store all the info we read from the file into the data structure
		//m_pBoneList[i].idx = name2idx(part);
		// -- modified by lzy
		//if (!m_pBoneList[i].dofx && !m_pBoneList[i].dofx && !m_pBoneList[i].dofx)

		//如果该骨骼所有自由度都不存在，则该骨骼为不活动骨骼，计数器减1
		if (!m_pBoneList[i].dofx && !m_pBoneList[i].dofy && !m_pBoneList[i].dofz &&
			!m_pBoneList[i].doftx && !m_pBoneList[i].dofty && !m_pBoneList[i].doftz) 
			MOV_BONES_IN_ASF_FILE-=1;
		
		m_pBoneList[i].length = length * scale;
		//init child/sibling to NULL, it will be assigned next (when hierarchy read)
		m_pBoneList[i].sibling = NULL; 
		m_pBoneList[i].child = NULL;
	}
		printf("READ %d\n",NUM_BONES_IN_ASF_FILE);
		// -- modified by lzy, add
		printf("MOV %d\n",MOV_BONES_IN_ASF_FILE);
		
	//
	//read and build the hierarchy of the skeleton
	//
	char *part_name;
	int j, parent;
 
	//find "hierarchy" string in the ASF file
	/*	while(1)
	{
		is.getline(str, 2048);	sscanf(str, "%s", keyword);
		if(strcmp(keyword, ":hierarchy") == 0)	
			break;
	} */
	
	//skip "begin" line
	is.getline(str, 2048);

	//Assign parent/child relationship to the bones
	//读取并建立骨架分层结构
	while(1)
	{
		//read next line
		is.getline(str, 2048);	sscanf(str, "%s", keyword);

		//check if we are done
		if(strcmp(keyword, "end") == 0)   
			break;
		else
		{
			//parse this line, it contains parent followed by children
			part_name=strtok(str, " ");
			j=0;
			while(part_name != NULL)
			{
				if(j==0) 
					parent=name2idx(part_name);//每一行第一个单词对应的骨骼都是父结点
				else 
					setChildrenAndSibling(parent, &m_pBoneList[name2idx(part_name)]);//从第二个单词开始往后的骨骼都作为child或sibling
				part_name=strtok(NULL, " ");
				j++;
			}
		}
	}

	is.close();

	m_bLoaded = true;
}


/*
   This recursive function traverses skeleton hierarchy 
   and returns a pointer to the bone with index - bIndex
   ptr should be a pointer to the root node 
   when this function first called
*/
Bone* Skeleton::getBone(Bone *ptr, int bIndex)
{
   static Bone *theptr;
   if(ptr==NULL) 
      return(NULL);
   else if(ptr->idx == bIndex)
   {
      theptr=ptr;
      return(theptr);
   }
   else
   { 
      getBone(ptr->child, bIndex);
      getBone(ptr->sibling, bIndex);
      return(theptr);
   }
}

/*
  This function sets sibling or child for parent bone
  If parent bone does not have a child, 
  then pChild is set as parent's child
  else pChild is set as a sibling of parents already existing child
*/
//将pChild指向的骨骼建立到以parent为索引的骨骼的下一层（child或者sibling）
//每一个父节点只有一个child，它的child节点可以有一个sibling节点
int Skeleton::setChildrenAndSibling(int parent, Bone *pChild)
{
	Bone *pParent;  
   
	//Get pointer to root bone
	pParent = getBone(m_pRootBone, parent);

	if(pParent==NULL)
	{
		printf("inbord bone is undefined\n"); 
		return(0);
	}
	else
	{
		//if pParent bone does not have a child
		//set pChild as parent bone child
		if(pParent->child == NULL)   
		{
			pParent->child = pChild;

			strcpy(pChild->parent_name,pParent->name);
		}
		else
		{ 
			//if pParent bone already has a child 
			//set pChils as pParent bone's child sibling
			string parent_name = pParent->name;

			pParent=pParent->child;			  //找到parent索引的骨骼的child节点并赋予pParent 

			while(pParent->sibling != NULL) 
				pParent = pParent->sibling;   // 找到链表中最后一个sibling节点并将其赋予pParent        

			pParent->sibling = pChild;

			strcpy(pChild->parent_name,parent_name.c_str());
		}
		return(1);
	}
}

/* 
	Return the pointer to the root bone
*/	
Bone* Skeleton::getRoot()
{
   return(m_pRootBone);
}


/***************************************************************************************
  Compute relative orientation and translation between the 
  parent and child bones. That is, represent the orientation 
  matrix and translation vector in the local coordinate of parent body 
*****************************************************************************************/


/*
	This function sets rot_parent_current data member.
	Rotation from this bone local coordinate system 
	to the coordinate system of its parent
*/
//计算子节点在其父节点坐标系中的旋转
void compute_rotation_parent_child(Bone *parent, Bone *child)
{
   if(child != NULL)
   { 
     
     // The following openGL rotations are precalculated and saved in the orientation matrix. 
     //
     // glRotatef(-inboard->axis_x, 1., 0., 0.);
     // glRotatef(-inboard->axis_y, 0., 1,  0.);
     // glRotatef(-inboard->axis_z, 0., 0., 1.);
     // glRotatef(outboard->axis_z, 0., 0., 1.);
     // glRotatef(outboard->axis_y, 0., 1,  0.);
     // glRotatef(outboard->axis_x, 1., 0., 0.);
	
     TransformationMatrix rotz,roty,rotx,tmp,tmp1,tmp2;
	 rotz.setToRotationMatrix(-parent->axis_z * PI / 180,Vector3d(0.0,0.0,1.0));
	 roty.setToRotationMatrix(-parent->axis_y * PI / 180,Vector3d(0.0,1.0,0.0));
	 rotx.setToRotationMatrix(-parent->axis_x * PI / 180,Vector3d(1.0,0.0,0.0));
	 tmp.setToProductOf(rotx,roty);
	 tmp1.setToProductOf(tmp,rotz);

	 rotz.setToRotationMatrix(child->axis_z * PI / 180,Vector3d(0.0,0.0,1.0));
	 roty.setToRotationMatrix(child->axis_y * PI / 180,Vector3d(0.0,1.0,0.0));
	 rotx.setToRotationMatrix(child->axis_x * PI / 180,Vector3d(1.0,0.0,0.0));
     tmp.setToProductOf(rotz,roty);
	 tmp2.setToProductOf(tmp,rotx);

	 tmp.setToProductOf(tmp1,tmp2);
	 tmp.setToTranspose();
	 double values[16];
	 tmp.getValues(values);
	
	 for(int i = 0; i < 4; i++)
		 for(int j = 0; j < 4; j++)
		 {
			 child->rot_parent_current[i][j] = values[i*4+j];
		 } 
   }
}


// loop through all bones to calculate local coordinates direction vector and relative orientation  
void ComputeRotationToParentCoordSystem(Bone *bone)
{
	//Compute rot_parent_current for the root 

	//Compute tmp2, a matrix containing root 
	//joint local coordinate system orientation
	TransformationMatrix rotz,roty,rotx,tmp,tmp1;
	rotz.setToRotationMatrix(bone[root].axis_z * PI / 180,Vector3d(0.0,0.0,1.0));
	roty.setToRotationMatrix(bone[root].axis_y * PI / 180,Vector3d(0.0,1.0,0.0));
	rotx.setToRotationMatrix(bone[root].axis_x * PI / 180,Vector3d(1.0,0.0,0.0));
	tmp.setToProductOf(rotz,roty);
	tmp1.setToProductOf(tmp,rotx);
	tmp1.setToTranspose();
	double values[16];
	tmp1.getValues(values);
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
		{
			bone[root].rot_parent_current[i][j] = values[i*4+j];
		} 
	
	bone[root].m_bSelected = false;

	//Compute rot_parent_current for all other bones
	int numbones = numBonesInSkel(bone[0]);
	for(int i=0; i<numbones; i++) 
	{
		if(bone[i].child != NULL)
		{
			bone[i].child->m_bSelected = false;

			compute_rotation_parent_child(&bone[i], bone[i].child);
		
			// compute parent child siblings...
			Bone * tmpbone = NULL;
			// -- modified by lzy, redundant
			//if (bone[i].child != NULL) tmp = (bone[i].child)->sibling;
			tmpbone = (bone[i].child)->sibling;
			while (tmpbone != NULL)
			{
				tmpbone->m_bSelected = false;
				compute_rotation_parent_child(&bone[i], tmpbone);
				tmpbone = tmpbone->sibling;
			}
		}
	}
}

/*
	Transform the direction vector (dir), 
	which is defined in character's global coordinate system in the ASF file, 
	to local coordinate
*/
void Skeleton::RotateBoneDirToLocalCoordSystem()
{
	int i;

	for(i=1; i<NUM_BONES_IN_ASF_FILE; i++) 
	{
		//Transform dir vector into local coordinate system
		TransformationMatrix rotz,roty,rotx,tmp,tmp1;
		rotz.setToRotationMatrix(m_pBoneList[i].axis_z * PI / 180,Vector3d(0.0,0.0,1.0));
		roty.setToRotationMatrix(m_pBoneList[i].axis_y * PI / 180,Vector3d(0.0,1.0,0.0));
		rotx.setToRotationMatrix(m_pBoneList[i].axis_x * PI / 180,Vector3d(1.0,0.0,0.0));
		tmp.setToProductOf(rotz,roty);
		tmp1.setToProductOf(tmp,rotx);
		tmp1.setToTranspose();
		Vector3d newdir;
		tmp1.postMultiplyVector(Vector3d(m_pBoneList[i].dir[0],m_pBoneList[i].dir[1],m_pBoneList[i].dir[2]),newdir);
		m_pBoneList[i].dir[0] = newdir.getX();
		m_pBoneList[i].dir[1] = newdir.getY();
		m_pBoneList[i].dir[2] = newdir.getZ();
	}
}


/******************************************************************************
	Interface functions to set the pose of the skeleton 
******************************************************************************/

//Initial posture Root at (0,0,0)
//All bone rotations are set to 0
void Skeleton::setBasePosture()
{
   int i;
   m_RootPos[0] = m_RootPos[1] = m_RootPos[2] = 0.0;
   m_pBoneList[0].tx = m_pBoneList[0].ty = m_pBoneList[0].tz = 0.0;
 
   for(i=0;i<NUM_BONES_IN_ASF_FILE;i++)
   {   
	   m_pBoneList[i].drx = m_pBoneList[i].dry = m_pBoneList[i].drz = 0.0;
	   m_pBoneList[i].m_bDrawn = true;
   }

   // update all the joints position
	updateGlobalPosition();
}


// set the skeleton's pose based on the given posture
void Skeleton::setPosture(Posture posture) 
{
    m_RootPos[0] = posture.root_pos[0];
    m_RootPos[1] = posture.root_pos[1];
    m_RootPos[2] = posture.root_pos[2];

    for(int j=0;j<NUM_BONES_IN_ASF_FILE;j++)
    {
		// if the bone has rotational degree of freedom in x direction
		if(m_pBoneList[j].dofx) 
		   m_pBoneList[j].drx = posture.bone_rotation[j][0];  

		if(m_pBoneList[j].doftx)
			m_pBoneList[j].tx = posture.bone_translation[j][0];

		// if the bone has rotational degree of freedom in y direction
		if(m_pBoneList[j].dofy) 
		   m_pBoneList[j].dry = posture.bone_rotation[j][1];    

		if(m_pBoneList[j].dofty)
			m_pBoneList[j].ty = posture.bone_translation[j][1];

		// if the bone has rotational degree of freedom in z direction
		if(m_pBoneList[j].dofz) 
		   m_pBoneList[j].drz = posture.bone_rotation[j][2];  

		if(m_pBoneList[j].doftz)
			m_pBoneList[j].tz= posture.bone_translation[j][2];
		
		if(m_pBoneList[j].doftl)
			m_pBoneList[j].tl = posture.bone_length[j];
    }

	// update all the joints position
	updateGlobalPosition();
}


//Set the aspect ratio of each bone 
void set_bone_shape(Bone *bone)
{
	// -- modified by lzy
   //bone[root].aspx=1;          bone[root].aspy=1;
   	//printf("READ %d\n",numBonesInSkel(bone[0]));
	//printf("MOV %d\n",movBonesInSkel(bone[0]));
	//int numbones = numBonesInSkel(bone[0]);
	bone[0].aspx=1.0;          bone[0].aspy=1.0;   	
	int numbones = numBonesInSkel(bone[0]);
   
	for(int j=1;j<numbones;j++)
    {
		bone[j].aspx=0.25;   bone[j].aspy=0.25;
    }

}

// Constructor 
Skeleton::Skeleton(const char *asf_filename, float scale)
{
	//对根节点进行初始化
	sscanf("root","%s",m_pBoneList[0].name);
	NUM_BONES_IN_ASF_FILE = 1;
	MOV_BONES_IN_ASF_FILE = 1;
   	m_pBoneList[0].dofo[0] = 4;
	m_pBoneList[0].dofo[1] = 5;
	m_pBoneList[0].dofo[2] = 6;
    m_pBoneList[0].dofo[3] = 1;
	m_pBoneList[0].dofo[4] = 2;
	m_pBoneList[0].dofo[5] = 3;
	m_pBoneList[0].dofo[6] = 0;
	//Initializaton
	// -- modified by lzy
	//m_pBoneList[0].idx = root;   // root of hierarchy
	m_pBoneList[0].idx = 0;
	m_pRootBone = &m_pBoneList[0];
	m_pBoneList[0].sibling = NULL;
	m_pBoneList[0].child = NULL; 
	m_pBoneList[0].dir[0] = 0; m_pBoneList[0].dir[1] = 0.; m_pBoneList[0].dir[2] = 0.;
	m_pBoneList[0].axis_x = 0; m_pBoneList[0].axis_y = 0.; m_pBoneList[0].axis_z = 0.;
	m_pBoneList[0].length = 0.05;
	m_pBoneList[0].dof = 6;
	m_pBoneList[0].dofx = m_pBoneList[0].dofy = m_pBoneList[0].dofz=1;
	m_RootPos[0] = m_RootPos[1]=m_RootPos[2]=0;
	//m_NumDOFs=6;
	tx = ty = tz = rx = ry = rz = 0;

	m_bLoaded = false;
	
	// build hierarchy and read in each bone's DOF information
	readASFfile(asf_filename, scale);  

	//transform the direction vector for each bone from the world coordinate system 
	//to it's local coordinate system
	RotateBoneDirToLocalCoordSystem();

	//Calculate rotation from each bone local coordinate system to the coordinate system of its parent
	//store it in rot_parent_current variable for each bone
	ComputeRotationToParentCoordSystem(m_pRootBone);

	//Set the aspect ratio of each bone 
	set_bone_shape(m_pRootBone);

	// added by lvp 2012-2-9
	m_scale = 1.0;	

	// set the joint & bone color
	float joint_color[4] = {0.65,0.65,0.0,1.0};
	float bone_color[4] = {0.133,0.545,0.133,1.0};
	setBoneColor(bone_color);
	setJointColor(joint_color);
}

Skeleton::~Skeleton()
{
	// delete the number of display list,for present, we only have one skeleton in the scene
	int numbones = numBonesInSkel(*getRoot());
	glDeleteLists(m_BoneList[0],numbones);
}

// -- modified by lzy, add
Bone::Bone()
{
	doftx = dofty = doftz = 1;
	doftl = 0;
	tx = 0; ty = 0; tz = 0; tl = 0;

	m_bDrawn = true;	// drawing the bone default
}

void Skeleton::RenderBone(Bone* pBone, int skelNum)
{
	static float z_dir[3] = {0., 0., 1.};
	// -- modified by lzy
	//float r_axis[3], mag, theta;
	float r_axis[3], theta;

	//Tranform (rotate) from the local coordinate system of this bone to it's parent
	//This step corresponds to doing: ModelviewMatrix = M_k * (rot_parent_current)
	glMultMatrixd((double*)&pBone->rot_parent_current);     

	//rotate AMC 
	//This step corresponds to doing: ModelviewMatrix *= R_k+1
	if(pBone->doftz) glTranslatef(0.,0.,pBone->tz);
	if(pBone->dofty) glTranslatef(0.,pBone->ty,0.);
	if(pBone->doftx) glTranslatef(pBone->tx,0.,0.);

	if(pBone->dofz) glRotatef(pBone->drz, 0., 0., 1.);
	if(pBone->dofy) glRotatef(pBone->dry, 0., 1,  0.);
	if(pBone->dofx) glRotatef(pBone->drx, 1., 0., 0.);

	//Store the current ModelviewMatrix (before adding the translation part)
	glPushMatrix();

	//Compute tx, ty, tz - translation from pBone to it's child (in local coordinate system of pBone)
	float tx = pBone->dir[0]*pBone->length * m_scale;
	float ty = pBone->dir[1]*pBone->length * m_scale;
	float tz = pBone->dir[2]*pBone->length * m_scale;

	// Use the current ModelviewMatrix to display the current bone
	// Rotate the bone from its canonical position (elongated sphere 
	// with its major axis parallel to X axis) to its correct orientation
	if(pBone->idx == root)
		//modified by lzy
		//glCallList(m_BoneList[skelNum] + pBone->idx);
		;
	else
	{ 
		//translate to the center of the bone
		//glTranslatef(tx/2.0, ty/2.0, tz/2.0);

		//Compute the angle between the canonical pose and the correct orientation 
		//(specified in pBone->dir) using cross product.
		//Using the formula: r_axis = z_dir x pBone->dir
		//由于初始骨胳是在z轴方向伸展的
		Vector3d tmp1,tmp2;
		tmp1 = Vector3d(z_dir[0],z_dir[1],z_dir[2]);
		tmp2 = tmp1.crossProductWith(Vector3d(pBone->dir[0],pBone->dir[1],pBone->dir[2]));
		r_axis[0] = tmp2.getX();
		r_axis[1] = tmp2.getY();
		r_axis[2] = tmp2.getZ();

		theta =  tmp1.angleWithAxis(Vector3d(pBone->dir[0],pBone->dir[1],pBone->dir[2]),tmp2);

		glRotatef(theta*180.0/M_PI, r_axis[0], r_axis[1], r_axis[2]);
		//glScalef(0.0,0.0,pBone->length);
		
		// draw the bone
		glColor4fv(m_vBone_Color);
		glCallList(m_BoneList[skelNum] + pBone->idx);

		// draw the joint
		glColor4fv(m_vJoint_Color);
		glLoadName(JOINT_NAME_STACK + pBone->idx);
		glCallList(m_JointList[skelNum] + pBone->idx);
	}

	glPopMatrix(); 

	// Finally, add the translation component to the ModelviewMatrix
	// This step corresponds to doing: M_k+1 = ModelviewMatrix += T_k+1
	glTranslatef(tx, ty, tz);
}

void Skeleton::RenderBoneWithLine(Bone *pBone, int skelNum)
{
	static float z_dir[3] = {0., 0., 1.};
	// -- modified by lzy
	//float r_axis[3], mag, theta;
	float r_axis[3], theta;

	//Tranform (rotate) from the local coordinate system of this bone to it's parent
	//This step corresponds to doing: ModelviewMatrix = M_k * (rot_parent_current)
	glMultMatrixd((double*)&pBone->rot_parent_current);     

	//rotate AMC 
	//This step corresponds to doing: ModelviewMatrix *= R_k+1
	if(pBone->doftz) glTranslatef(0.,0.,pBone->tz);
	if(pBone->dofty) glTranslatef(0.,pBone->ty,0.);
	if(pBone->doftx) glTranslatef(pBone->tx,0.,0.);

	if(pBone->dofz) glRotatef(pBone->drz, 0., 0., 1.);
	if(pBone->dofy) glRotatef(pBone->dry, 0., 1,  0.);
	if(pBone->dofx) glRotatef(pBone->drx, 1., 0., 0.);

	//Store the current ModelviewMatrix (before adding the translation part)
	glPushMatrix();

	//Compute tx, ty, tz - translation from pBone to it's child (in local coordinate system of pBone)
	float tx = pBone->dir[0]*pBone->length * m_scale;
	float ty = pBone->dir[1]*pBone->length * m_scale;
	float tz = pBone->dir[2]*pBone->length * m_scale;

	// Use the current ModelviewMatrix to display the current bone
	// Rotate the bone from its canonical position (elongated sphere 
	// with its major axis parallel to X axis) to its correct orientation
	if(pBone->idx == root)
		//modified by lzy
		//glCallList(m_BoneList[skelNum] + pBone->idx);
		;
	else
	{ 
		//translate to the center of the bone
		//glTranslatef(tx/2.0, ty/2.0, tz/2.0);

		//Compute the angle between the canonical pose and the correct orientation 
		//(specified in pBone->dir) using cross product.
		//Using the formula: r_axis = z_dir x pBone->dir
		//由于初始骨胳是在z轴方向伸展的
		Vector3d tmp1,tmp2;
		tmp1 = Vector3d(z_dir[0],z_dir[1],z_dir[2]);
		tmp2 = tmp1.crossProductWith(Vector3d(pBone->dir[0],pBone->dir[1],pBone->dir[2]));
		r_axis[0] = tmp2.getX();
		r_axis[1] = tmp2.getY();
		r_axis[2] = tmp2.getZ();

		theta =  tmp1.angleWithAxis(Vector3d(pBone->dir[0],pBone->dir[1],pBone->dir[2]),tmp2);

		glRotatef(theta*180.0/M_PI, r_axis[0], r_axis[1], r_axis[2]);
		//glScalef(0.0,0.0,pBone->length);

		if(pBone->m_bDrawn)
		{
			// draw the bone
			glColor4fv(m_vBone_Color);
			//glTranslated(0.0,0.0,0.22);
			//gluCylinder( qobj, radius, radius, bone[j].length - 0.44, slices, stack );
			glBegin(GL_LINES);
			{
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(0.0f, 0.0f, pBone->length);
			}
			glEnd();
		}
	}

	glPopMatrix(); 

	// Finally, add the translation component to the ModelviewMatrix
	// This step corresponds to doing: M_k+1 = ModelviewMatrix += T_k+1
	glTranslatef(tx, ty, tz);
}

void Skeleton::RenderBoneWithLine(const std::vector<std::string> & vSketchedBones, Bone *pBone, int skelNum)
{
	static float z_dir[3] = {0., 0., 1.};
	// -- modified by lzy
	//float r_axis[3], mag, theta;
	float r_axis[3], theta;

	//Tranform (rotate) from the local coordinate system of this bone to it's parent
	//This step corresponds to doing: ModelviewMatrix = M_k * (rot_parent_current)
	glMultMatrixd((double*)&pBone->rot_parent_current);     

	//rotate AMC 
	//This step corresponds to doing: ModelviewMatrix *= R_k+1
	if(pBone->doftz) glTranslatef(0.,0.,pBone->tz);
	if(pBone->dofty) glTranslatef(0.,pBone->ty,0.);
	if(pBone->doftx) glTranslatef(pBone->tx,0.,0.);

	if(pBone->dofz) glRotatef(pBone->drz, 0., 0., 1.);
	if(pBone->dofy) glRotatef(pBone->dry, 0., 1,  0.);
	if(pBone->dofx) glRotatef(pBone->drx, 1., 0., 0.);

	//Store the current ModelviewMatrix (before adding the translation part)
	glPushMatrix();

	//Compute tx, ty, tz - translation from pBone to it's child (in local coordinate system of pBone)
	float tx = pBone->dir[0]*pBone->length * m_scale;
	float ty = pBone->dir[1]*pBone->length * m_scale;
	float tz = pBone->dir[2]*pBone->length * m_scale;

	// Use the current ModelviewMatrix to display the current bone
	// Rotate the bone from its canonical position (elongated sphere 
	// with its major axis parallel to X axis) to its correct orientation
	if(pBone->idx == root)
		//modified by lzy
		//glCallList(m_BoneList[skelNum] + pBone->idx);
		;
	else
	{ 
		//translate to the center of the bone
		//glTranslatef(tx/2.0, ty/2.0, tz/2.0);

		//Compute the angle between the canonical pose and the correct orientation 
		//(specified in pBone->dir) using cross product.
		//Using the formula: r_axis = z_dir x pBone->dir
		//由于初始骨胳是在z轴方向伸展的
		Vector3d tmp1,tmp2;
		tmp1 = Vector3d(z_dir[0],z_dir[1],z_dir[2]);
		tmp2 = tmp1.crossProductWith(Vector3d(pBone->dir[0],pBone->dir[1],pBone->dir[2]));
		r_axis[0] = tmp2.getX();
		r_axis[1] = tmp2.getY();
		r_axis[2] = tmp2.getZ();

		theta =  tmp1.angleWithAxis(Vector3d(pBone->dir[0],pBone->dir[1],pBone->dir[2]),tmp2);

		glRotatef(theta*180.0/M_PI, r_axis[0], r_axis[1], r_axis[2]);
		//glScalef(0.0,0.0,pBone->length);

		bool bSketched = false;
		for(int i = 0 ; i < vSketchedBones.size(); ++i)
		{
			if(string(pBone->name) == vSketchedBones[i])
			{
				bSketched = true;
				break;
			}
		}

		// draw the bone
		if(bSketched){
			// draw the bone
			glColor4fv(m_vBone_Color);
			//glTranslated(0.0,0.0,0.22);
			//gluCylinder( qobj, radius, radius, bone[j].length - 0.44, slices, stack );
			glBegin(GL_LINES);
			{
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(0.0f, 0.0f, pBone->length);
			}
			glEnd();
		}
		else{
			glColor4fv(m_vBone_Color);
		}
	}

	glPopMatrix(); 

	// Finally, add the translation component to the ModelviewMatrix
	// This step corresponds to doing: M_k+1 = ModelviewMatrix += T_k+1
	glTranslatef(tx, ty, tz);
}

void Skeleton::RenderFigure(float* bone_color,float* joint_color)
{
	setBoneColor(bone_color);
	setJointColor(joint_color);
	
	//设置整个整个角色的位置，然后再画出来		
	glPushMatrix();
	glTranslatef(m_scale*tx,m_scale*ty,m_scale*tz);
	glRotatef(rx,1,0,0);
	glRotatef(ry,0,1,0);
	glRotatef(rz,0,0,1);
	traverse(getRoot(),0);
	glPopMatrix();
}

void Skeleton::RenderFigureWithLines(float *bone_color)
{
	setBoneColor(bone_color);

	//设置整个整个角色的位置，然后再画出来		
	glPushMatrix();
	glTranslatef(m_scale*tx,m_scale*ty,m_scale*tz);
	glRotatef(rx,1,0,0);
	glRotatef(ry,0,1,0);
	glRotatef(rz,0,0,1);
	traverseWithLines(getRoot(),0);
	glPopMatrix();
}

void Skeleton::RenderLimbWithLines(const std::vector<std::string> & vSketchedBones, float* bone_color)
{
	setBoneColor(bone_color);

	//设置整个整个角色的位置，然后再画出来		
	glPushMatrix();
	glTranslatef(m_scale*tx,m_scale*ty,m_scale*tz);
	glRotatef(rx,1,0,0);
	glRotatef(ry,0,1,0);
	glRotatef(rz,0,0,1);
	traverseLimbWithLines(vSketchedBones,getRoot(),0);
	glPopMatrix();
}

//Traverse the hierarchy starting from the root 
//Every node in the data structure has just one child pointer. 
//If there are more than one children for any node, they are stored as sibling pointers
//The algorithm draws the current node (bone), visits its child and then visits siblings
void Skeleton::traverse(Bone *ptr,int skelNum)
{
	// jumping these joint for sketching
	/*
	if( strcmp(ptr->name,"lwrist") == 0 || strcmp(bone[j].name,"lhand")==0 || 
		strcmp(bone[j].name,"lfingers") == 0 || strcmp(bone[j].name,"lfingers_End")==0 ||
		strcmp(bone[j].name,"lthumb") == 0 || strcmp(bone[j].name,"lthumb_End")==0 ||
		strcmp(bone[j].name,"rwrist") == 0 || strcmp(bone[j].name,"rhand")==0 || 
		strcmp(bone[j].name,"rfingers") == 0 || strcmp(bone[j].name,"rfingers_End")==0 ||
		strcmp(bone[j].name,"rthumb") == 0 || strcmp(bone[j].name,"rthumb_End")==0 ||
		strcmp(bone[j].name,"lfoot") == 0 || strcmp(bone[j].name,"ltoes")==0 || 
		strcmp(bone[j].name,"ltoes_End") == 0 || 
		strcmp(bone[j].name,"rfoot") == 0 || strcmp(bone[j].name,"rtoes")==0 || 
		strcmp(bone[j].name,"rtoes_End") == 0 || 
		strcmp(bone[j].name,"lowerneck") == 0 || strcmp(bone[j].name,"upperneck")==0 || 
		strcmp(bone[j].name,"head") == 0 || strcmp(bone[j].name,"head_End")==0
		)
		continue;
	//*/

	if(ptr != NULL && 
		strcmp(ptr->name,"lwrist") != 0 && strcmp(ptr->name,"rwrist")!=0 &&
		strcmp(ptr->name,"lfoot") != 0 && strcmp(ptr->name,"rfoot") != 0 &&
		strcmp(ptr->name,"upperneck") != 0 )
	{
		glPushMatrix();
		RenderBone(ptr,skelNum);
		traverse(ptr->child,skelNum);
		glPopMatrix();
		traverse(ptr->sibling,skelNum);
	}
}

void Skeleton::traverseWithLines(Bone *ptr, int skelNum)
{
	if(ptr != NULL && 
		strcmp(ptr->name,"lwrist") != 0 && strcmp(ptr->name,"rwrist")!=0 &&
		strcmp(ptr->name,"lfoot") != 0 && strcmp(ptr->name,"rfoot") != 0 &&
		strcmp(ptr->name,"upperneck") != 0 )
	{
		glPushMatrix();
		RenderBoneWithLine(ptr,skelNum);
		traverseWithLines(ptr->child,skelNum);
		glPopMatrix();
		traverseWithLines(ptr->sibling,skelNum);
	}
}

//Pre-draw the bones using quadratic object drawing function
//and store them in the display list
void Skeleton::set_display_list()
{
	Bone* bone = getRoot();

	int numbones = numBonesInSkel(bone[0]);
	
	// if the display list is occupied, delete it.
	if(glIsList(*m_BoneList))
		glDeleteLists(*m_BoneList,numbones);

	*m_BoneList = glGenLists(numbones);
	
	// draw a single line
	/*
	for (int j = 0; j < numbones; j++)
	{
		glNewList(*m_BoneList+j,GL_COMPILE);
		
		if(strcmp(bone[j].name,"lowerneck") == 0)
		{
			glBegin(GL_LINES);
			glVertex3f(0.0, 0.0, -0.5);
			glVertex3f(0.0, 0.0, 0.5);
			glEnd();
		}
		else
		{
			glBegin(GL_LINES);
			glVertex3f(0.0, 0.0, -0.5);
			glVertex3f(0.0, 0.0, 0.5);
			glEnd();
		}
		glEndList();
	}
	//*/
	
	// draw quadric object
	//*
	GLUquadricObj *qobj;
	qobj= gluNewQuadric();

	gluQuadricDrawStyle(qobj, GLU_FILL);
	gluQuadricNormals(qobj, GLU_SMOOTH);
	gluQuadricOrientation(qobj, GLU_OUTSIDE);

	GLdouble radius = 0.2; 
	GLdouble slices = 32.0; 
	GLdouble stack  = 32.0; 
	
	GLfloat mat_ambient[] = {0.3,0.3,0.3,1.0};
	GLfloat mat_specular[] = {1.0,1.0,1.0,1.0};
	GLfloat mat_shininess[] = {100.0};
	GLfloat mat_diffuse[] = {0.6,0.6,0.6,1.0};
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mat_ambient);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS,mat_shininess);
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diffuse);

	for(int j=0; j<numbones; j++)
	{
		// the bone display list
		glNewList(*m_BoneList + j, GL_COMPILE);
		//glScalef(bone[j].aspx, bone[j].aspy, 1.);
		//gluSphere(qobj, radius + 0.02, slices, stack
		glPushMatrix();
		//glTranslated(0.0,0.0,0.22);
		gluCylinder( qobj, radius, radius, bone[j].length - 0.44, slices, stack );
		glPopMatrix();
		glEndList();
		
		// the joint display list
		glNewList(*m_JointList + j, GL_COMPILE);
		glPushMatrix();
		glTranslated(0, 0, bone[j].length - 0.22);
		gluSphere(qobj, radius + 0.05, slices, stack);
		glPopMatrix();
		glEndList();
	}
	//*/
}

void Skeleton::RenderFigureByJointPosition(Bone* pBone,float* joint_color)
{
	if(pBone != NULL && 
		strcmp(pBone->name,"lwrist") != 0 && strcmp(pBone->name,"rwrist")!=0 &&
		strcmp(pBone->name,"lfoot") != 0 && strcmp(pBone->name,"rfoot") != 0 &&
		strcmp(pBone->name,"upperneck") != 0 )
	{
		glPushMatrix();
		
		glTranslatef(pBone->m_GlobalPosition[0],pBone->m_GlobalPosition[1],pBone->m_GlobalPosition[2]);
		
		float edgelength = 0.5;
		// front face
		glBegin(GL_QUADS);
		glVertex3f(-edgelength/2,edgelength/2,edgelength/2);
		glVertex3f(edgelength/2,edgelength/2,edgelength/2);
		glVertex3f(edgelength/2,-edgelength/2,edgelength/2);
		glVertex3f(-edgelength/2,-edgelength/2,edgelength/2);
		glEnd();

		// back face
		glBegin(GL_QUADS);
		glVertex3f(edgelength/2,edgelength/2,-edgelength/2);
		glVertex3f(-edgelength/2,edgelength/2,-edgelength/2);
		glVertex3f(-edgelength/2,-edgelength/2,-edgelength/2);
		glVertex3f(edgelength/2,-edgelength/2,-edgelength/2);
		glEnd();

		// left face
		glBegin(GL_QUADS);
		glVertex3f(-edgelength/2,edgelength/2,-edgelength/2);
		glVertex3f(-edgelength/2,edgelength/2,edgelength/2);
		glVertex3f(-edgelength/2,-edgelength/2,edgelength/2);
		glVertex3f(-edgelength/2,-edgelength/2,-edgelength/2);
		glEnd();

		// right face
		glBegin(GL_QUADS);
		glVertex3f(edgelength/2,edgelength/2,edgelength/2);
		glVertex3f(edgelength/2,edgelength/2,-edgelength/2);
		glVertex3f(edgelength/2,-edgelength/2,-edgelength/2);
		glVertex3f(edgelength/2,-edgelength/2,edgelength/2);
		glEnd();

		// top face 
		glBegin(GL_QUADS);
		glVertex3f(-edgelength/2,edgelength/2,-edgelength/2);
		glVertex3f(edgelength/2,edgelength/2,-edgelength/2);
		glVertex3f(edgelength/2,edgelength/2,edgelength/2);
		glVertex3f(-edgelength/2,edgelength/2,edgelength/2);
		glEnd();

		// bottom face
		glBegin(GL_QUADS);
		glVertex3f(-edgelength/2,-edgelength/2,-edgelength/2);
		glVertex3f(edgelength/2,-edgelength/2,-edgelength/2);
		glVertex3f(edgelength/2,-edgelength/2,edgelength/2);
		glVertex3f(-edgelength/2,-edgelength/2,edgelength/2);
		glEnd();

		glPopMatrix();

		RenderFigureByJointPosition(pBone->child,joint_color);
		RenderFigureByJointPosition(pBone->sibling,joint_color);
	}
}

void Skeleton::draw_bone_axis()
{
	glBegin(GL_LINES);
		// draw x axis in red, y axis in green, z axis in blue 
		glColor3f(1., .2, .2);
		glVertex3f(0., 0., 0.);
		glVertex3f(.5, 0., 0.);

		glColor3f(.2, 1., .2);
		glVertex3f(0., 0., 0.);
		glVertex3f(0., .5, 0.);

		glColor3f(.2, .2, 1.);
		glVertex3f(0., 0., 0.);
		glVertex3f(0., 0., .5);
	glEnd();
}

void Skeleton::setBoneColor(float* color)
{
	m_vBone_Color[0] = color[0];
	m_vBone_Color[1] = color[1];
	m_vBone_Color[2] = color[2];
	m_vBone_Color[3] = color[3];
}

void Skeleton::setJointColor(float* color)
{
	m_vJoint_Color[0] = color[0];
	m_vJoint_Color[1] = color[1];
	m_vJoint_Color[2] = color[2];
	m_vJoint_Color[3] = color[3];
}

//Rendering method: override the cEntity Base Class's function
bool Skeleton::Render()
{
	RenderFigure(m_vBone_Color, m_vJoint_Color);

	return true;
}

// compute the global position for all the joints
void Skeleton::updateGlobalPosition()
{
	glMatrixMode(GL_MODELVIEW_MATRIX);
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(m_scale*tx,m_scale*ty,m_scale*tz);
	glRotatef(rx,1,0,0);
	glRotatef(ry,0,1,0);
	glRotatef(rz,0,0,1);
	computeChildGlobalPosition(getRoot());
	glPopMatrix();
}

// compute the global position for all the child bones, added by lvp 12-10-22
void Skeleton::computeChildGlobalPosition(Bone* pBone)
{
	if(pBone == NULL)
		return;
	
	static float z_dir[3] = {0., 0., 1.};
	float r_axis[3], theta;

	glPushMatrix();
	
	//transform (rotate) from the local coordinate system of this bone to it's parent
	//This step corresponds to doing: ModelviewMatrix = M_k * (rot_parent_current)
	glMultMatrixd((double*)&pBone->rot_parent_current);     

	//rotate AMC 
	//This step corresponds to doing: ModelviewMatrix *= R_k+1
	if(pBone->doftz) glTranslatef(0.,0.,pBone->tz);
	if(pBone->dofty) glTranslatef(0.,pBone->ty,0.);
	if(pBone->doftx) glTranslatef(pBone->tx,0.,0.);

	if(pBone->dofz) glRotatef(pBone->drz, 0., 0., 1.);
	if(pBone->dofy) glRotatef(pBone->dry, 0., 1,  0.);
	if(pBone->dofx) glRotatef(pBone->drx, 1., 0., 0.);

	glPushMatrix();
	// Use the current ModelviewMatrix to display the current bone
	// Rotate the bone from its canonical position (elongated sphere 
	// with its major axis parallel to X axis) to its correct orientation
	if(pBone->idx == root)
	{
		pBone->m_GlobalPosition[0] = m_RootPos[0];
		pBone->m_GlobalPosition[1] = m_RootPos[1];
		pBone->m_GlobalPosition[2] = m_RootPos[2];
	}
	else
	{ 
		//Compute the angle between the canonical pose and the correct orientation 
		//(specified in pBone->dir) using cross product.
		//Using the formula: r_axis = z_dir x pBone->dir

		//由于初始骨胳是在z轴方向伸展的
		Vector3d tmp1,tmp2;
		tmp1 = Vector3d(z_dir[0],z_dir[1],z_dir[2]);
		tmp2 = tmp1.crossProductWith(Vector3d(pBone->dir[0],pBone->dir[1],pBone->dir[2]));
		r_axis[0] = tmp2.getX();
		r_axis[1] = tmp2.getY();
		r_axis[2] = tmp2.getZ();

		theta =  tmp1.angleWithAxis(Vector3d(pBone->dir[0],pBone->dir[1],pBone->dir[2]),tmp2);

		glRotatef(theta*180.0/M_PI, r_axis[0], r_axis[1], r_axis[2]);

		// compute the joint position
		double modelview[16];
		glGetDoublev(GL_MODELVIEW_MATRIX,modelview);

		TransformationMatrix transform;
		transform.setOGLValues(modelview);

		Vector3d pos(0.0,0.0,pBone->length), newpos;

		transform.postMultiplyVector(pos,newpos);

		pBone->m_GlobalPosition[0] = newpos.getX();
		pBone->m_GlobalPosition[1] = newpos.getY();
		pBone->m_GlobalPosition[2] = newpos.getZ();
	}
	glPopMatrix();

	//Compute tx, ty, tz - translation from pBone to it's child (in local coordinate system of pBone)
	float tx = pBone->dir[0] * pBone->length * m_scale;
	float ty = pBone->dir[1] * pBone->length * m_scale;
	float tz = pBone->dir[2] * pBone->length * m_scale;

	// Finally, add the translation component to the ModelviewMatrix
	// This step corresponds to doing: M_k+1 = ModelviewMatrix += T_k+1
	glTranslatef(tx, ty, tz);

	// compute the child node
	computeChildGlobalPosition(pBone->child);
	glPopMatrix();

	// compute the sibling node
	computeChildGlobalPosition(pBone->sibling);
}

/*
	render certain joint by its position through joint name or joint idx
*/
void Skeleton::RenderJoint(char* name)
{
	Bone* pBone = findBoneByName(getRoot(),name);

	if(pBone == NULL)
		return;

	glPushMatrix();
	glTranslatef(pBone->m_GlobalPosition[0],pBone->m_GlobalPosition[1],pBone->m_GlobalPosition[2]);

	float edgelength = 0.5;
	// front face
	glBegin(GL_QUADS);
	glVertex3f(-edgelength/2,edgelength/2,edgelength/2);
	glVertex3f(edgelength/2,edgelength/2,edgelength/2);
	glVertex3f(edgelength/2,-edgelength/2,edgelength/2);
	glVertex3f(-edgelength/2,-edgelength/2,edgelength/2);
	glEnd();

	// back face
	glBegin(GL_QUADS);
	glVertex3f(edgelength/2,edgelength/2,-edgelength/2);
	glVertex3f(-edgelength/2,edgelength/2,-edgelength/2);
	glVertex3f(-edgelength/2,-edgelength/2,-edgelength/2);
	glVertex3f(edgelength/2,-edgelength/2,-edgelength/2);
	glEnd();

	// left face
	glBegin(GL_QUADS);
	glVertex3f(-edgelength/2,edgelength/2,-edgelength/2);
	glVertex3f(-edgelength/2,edgelength/2,edgelength/2);
	glVertex3f(-edgelength/2,-edgelength/2,edgelength/2);
	glVertex3f(-edgelength/2,-edgelength/2,-edgelength/2);
	glEnd();

	// right face
	glBegin(GL_QUADS);
	glVertex3f(edgelength/2,edgelength/2,edgelength/2);
	glVertex3f(edgelength/2,edgelength/2,-edgelength/2);
	glVertex3f(edgelength/2,-edgelength/2,-edgelength/2);
	glVertex3f(edgelength/2,-edgelength/2,edgelength/2);
	glEnd();

	// top face 
	glBegin(GL_QUADS);
	glVertex3f(-edgelength/2,edgelength/2,-edgelength/2);
	glVertex3f(edgelength/2,edgelength/2,-edgelength/2);
	glVertex3f(edgelength/2,edgelength/2,edgelength/2);
	glVertex3f(-edgelength/2,edgelength/2,edgelength/2);
	glEnd();

	// bottom face
	glBegin(GL_QUADS);
	glVertex3f(-edgelength/2,-edgelength/2,-edgelength/2);
	glVertex3f(edgelength/2,-edgelength/2,-edgelength/2);
	glVertex3f(edgelength/2,-edgelength/2,edgelength/2);
	glVertex3f(-edgelength/2,-edgelength/2,edgelength/2);
	glEnd();

	glPopMatrix();
}
void Skeleton::RenderJoint(int idx)
{

}


Bone* Skeleton::findBoneByName(Bone* root, const char* name)
{
	if(root == NULL)
		return NULL;
	else
	{
		if(strcmp(root->name,name) == 0)
			return root;
		else if(findBoneByName(root->child,name))
		{
			return findBoneByName(root->child,name);
		}
		else if(findBoneByName(root->sibling,name))
		{
			return findBoneByName(root->sibling,name);
		}
		
		return NULL;
	}
}
Bone* Skeleton::findBoneByIdx(Bone* root, int idx)
{
	if(root == NULL)
		return NULL;
	else
	{
		if(root->idx == idx)
			return root;
		else if(findBoneByIdx(root->child,idx))
		{
			return findBoneByIdx(root->child,idx);
		}
		else if(findBoneByIdx(root->sibling,idx))
		{
			return findBoneByIdx(root->sibling,idx);
		}

		return NULL;
	}
}


void Skeleton::RenderSketchedFigure(const vector<string> &vSketchedBones, float aSkeBoneColor[4], float aSkeJointColor[4], float aBoneColor[4], float aJointColor[4])
{
	setSketchedBoneColor(aSkeBoneColor);
	setSketchedJointColor(aSkeJointColor);
	setBoneColor(aBoneColor);
	setJointColor(aJointColor);

	//设置整个整个角色的位置，然后再画出来		
	glPushMatrix();
	glTranslatef(m_scale*tx,m_scale*ty,m_scale*tz);
	glRotatef(rx,1,0,0);
	glRotatef(ry,0,1,0);
	glRotatef(rz,0,0,1);
	traverseSketched(vSketchedBones, getRoot(),0);
	glPopMatrix();
}

void Skeleton::traverseSketched(const vector<string> &vSketchedBones, Bone *ptr, int skelNum)
{
	if(ptr != NULL && 
		strcmp(ptr->name,"lwrist") != 0 && strcmp(ptr->name,"rwrist")!=0 &&
		strcmp(ptr->name,"lfoot") != 0 && strcmp(ptr->name,"rfoot") != 0 &&
		strcmp(ptr->name,"upperneck") != 0 )
	{
		glPushMatrix();
		RenderSketchedBone(vSketchedBones, ptr,skelNum);
		traverseSketched(vSketchedBones, ptr->child,skelNum);
		glPopMatrix();
		traverseSketched(vSketchedBones, ptr->sibling,skelNum);
	}
}

void Skeleton::traverseLimbWithLines(const std::vector<std::string> &vSketchedBones, Bone *ptr, int skelNum)
{
	if(ptr != NULL && 
		strcmp(ptr->name,"lwrist") != 0 && strcmp(ptr->name,"rwrist")!=0 &&
		strcmp(ptr->name,"lfoot") != 0 && strcmp(ptr->name,"rfoot") != 0 &&
		strcmp(ptr->name,"upperneck") != 0 )
	{
		glPushMatrix();
		RenderBoneWithLine(vSketchedBones,ptr,skelNum);
		traverseLimbWithLines(vSketchedBones, ptr->child,skelNum);
		glPopMatrix();
		traverseLimbWithLines(vSketchedBones, ptr->sibling,skelNum);
	}
}

void Skeleton::RenderSketchedBone(const vector<string> &vSketchedBones, Bone *pBone, int skelNum)
{
	static float z_dir[3] = {0., 0., 1.};
	// -- modified by lzy
	//float r_axis[3], mag, theta;
	float r_axis[3], theta;

	//Tranform (rotate) from the local coordinate system of this bone to it's parent
	//This step corresponds to doing: ModelviewMatrix = M_k * (rot_parent_current)
	glMultMatrixd((double*)&pBone->rot_parent_current);     

	//rotate AMC 
	//This step corresponds to doing: ModelviewMatrix *= R_k+1
	if(pBone->doftz) glTranslatef(0.,0.,pBone->tz);
	if(pBone->dofty) glTranslatef(0.,pBone->ty,0.);
	if(pBone->doftx) glTranslatef(pBone->tx,0.,0.);

	if(pBone->dofz) glRotatef(pBone->drz, 0., 0., 1.);
	if(pBone->dofy) glRotatef(pBone->dry, 0., 1,  0.);
	if(pBone->dofx) glRotatef(pBone->drx, 1., 0., 0.);

	//Store the current ModelviewMatrix (before adding the translation part)
	glPushMatrix();

	//Compute tx, ty, tz - translation from pBone to it's child (in local coordinate system of pBone)
	float tx = pBone->dir[0]*pBone->length * m_scale;
	float ty = pBone->dir[1]*pBone->length * m_scale;
	float tz = pBone->dir[2]*pBone->length * m_scale;

	// Use the current ModelviewMatrix to display the current bone
	// Rotate the bone from its canonical position (elongated sphere 
	// with its major axis parallel to X axis) to its correct orientation
	if(pBone->idx == root)
		//modified by lzy
		//glCallList(m_BoneList[skelNum] + pBone->idx);
		;
	else
	{ 
		//translate to the center of the bone
		//glTranslatef(tx/2.0, ty/2.0, tz/2.0);

		//Compute the angle between the canonical pose and the correct orientation 
		//(specified in pBone->dir) using cross product.
		//Using the formula: r_axis = z_dir x pBone->dir
		//由于初始骨胳是在z轴方向伸展的
		Vector3d tmp1,tmp2;
		tmp1 = Vector3d(z_dir[0],z_dir[1],z_dir[2]);
		tmp2 = tmp1.crossProductWith(Vector3d(pBone->dir[0],pBone->dir[1],pBone->dir[2]));
		r_axis[0] = tmp2.getX();
		r_axis[1] = tmp2.getY();
		r_axis[2] = tmp2.getZ();

		theta =  tmp1.angleWithAxis(Vector3d(pBone->dir[0],pBone->dir[1],pBone->dir[2]),tmp2);

		glRotatef(theta*180.0/M_PI, r_axis[0], r_axis[1], r_axis[2]);
		//glScalef(0.0,0.0,pBone->length);

		bool bSketched = false;
		for(int i = 0 ; i < vSketchedBones.size(); ++i)
		{
			if(string(pBone->name) == vSketchedBones[i])
			{
				bSketched = true;
				break;
			}
		}

		// draw the bone
		if(bSketched){
			glColor4fv(m_vSketchedBone_Color);
			glCallList(m_BoneList[skelNum] + pBone->idx);
		}
		else{
			glColor4fv(m_vBone_Color);
		}
		//glCallList(m_BoneList[skelNum] + pBone->idx);

		// draw the joint
		if(bSketched){
			glColor4fv(m_vSketchedJoint_Color);
			glLoadName(JOINT_NAME_STACK + pBone->idx);
			glCallList(m_JointList[skelNum] + pBone->idx);
		}
		else{
			glColor4fv(m_vJoint_Color);
		}
		//glLoadName(JOINT_NAME_STACK + pBone->idx);
		//glCallList(m_JointList[skelNum] + pBone->idx);
	}

	glPopMatrix(); 

	// Finally, add the translation component to the ModelviewMatrix
	// This step corresponds to doing: M_k+1 = ModelviewMatrix += T_k+1
	glTranslatef(tx, ty, tz);
}

