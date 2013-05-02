/**
***  BVH文件定义
***  Copyright (c) 2004-2007, Masaki OSHITA (www.oshita-lab.org)
**/

#ifndef  _BVH_H_
#define  _BVH_H_

#include "common.h"


/*  变量定义  */

// Channel标识
enum  ChannelEnum
{
	X_ROTATION, Y_ROTATION, Z_ROTATION,
	X_POSITION, Y_POSITION, Z_POSITION
};
struct  Joint;

// Channel定义
struct  Channel
{
	// 关节
	Joint *              joint;

	// Channel类型
	ChannelEnum          type;

	// 索引
	int                  index;
};

// 关节定义
struct  Joint
{
	// 关节名称
	string               name;
	// 关节索引
	int                  index;

	// 父关节
	Joint *              parent;
	// 子关节
	vector< Joint * >    children;

	// 偏移量
	double               offset[3];

	// 是否为结束片段
	bool                 has_site;
	// 片段长度
	double               site[3];

	// 关节所需的变换
	vector< Channel * >  channels;
};

//
//  BVH类定义
//
class  BVH
{
  public:


  private:
	// 是否载入成功
	bool                     is_load_success;

	/*  字符串变量  */
	string                   file_name;   // 文件名
	string                   motion_name; // 运动名

	
	int                      num_channel; // channel的数量
	vector< Channel * >      channels;    // 
	vector< Joint * >        joints;      // 
	map< string, Joint * >   joint_index; // 


	int                      num_frame;   // 数据文件的帧数
	double                   interval;    // 每帧之间的间隔
	double *                 motion;      // 


  public:
	// 构造函数
	BVH();
	BVH( const char * bvh_file_name );
	~BVH();

	void  Clear();

	// 载入BVH文件
	void  Load( const char * bvh_file_name );

  public:
	/*  实用函数  */

	// 文件是否载入成功
	bool  IsLoadSuccess() const { return is_load_success; }

	// 获取相应内容的名称
	const string &  GetFileName() const { return file_name; }
	const string &  GetMotionName() const { return motion_name; }

	// 获取BVH文件信息
	const int       GetNumJoint() const { return  joints.size(); }
	const Joint *   GetJoint( int no ) const { return  joints[no]; }
	const int       GetNumChannel() const { return  channels.size(); }
	const Channel * GetChannel( int no ) const { return  channels[no]; }

	const Joint *   GetJoint( const string & j ) const  {
		map< string, Joint * >::const_iterator  i = joint_index.find( j );
		return  ( i != joint_index.end() ) ? (*i).second : NULL; }
	const Joint *   GetJoint( const char * j ) const  {
		map< string, Joint * >::const_iterator  i = joint_index.find( j );
		return  ( i != joint_index.end() ) ? (*i).second : NULL; }

	int     GetNumFrame() const { return  num_frame; }
	double  GetInterval() const { return  interval; }
	double  GetMotion( int f, int c ) const { return  motion[ f*num_channel + c ]; }

	void  SetMotion( int f, int c, double v ) { motion[ f*num_channel + c ] = v; }

  public:
	
	void  RenderFigure( int frame_no, float scale = 1.0f );

	static void  RenderFigure( const Joint * root, const double * data, float scale = 1.0f );

	static void  RenderBone( float x0, float y0, float z0, float x1, float y1, float z1 );
	// 返回motion地址
	double* GetMotion(void);
};



#endif // _BVH_H_
