//this class is created by zpy

#ifndef CYLINDER_H
#define CYLINDER_H

#include "Point3d.h"
/************************************************************************/
/* this class is used to represent a cylinder,it contains the top radius,
	base radius and length information.									*/
/************************************************************************/
namespace MathLib
{


class Cylinder
{
public:

	//the base disk's radius
	double baseRadius;
	
	//the base disk's center
	Point3d baseCenter;

	//the top disk's radius
	double topRadius;

	//the top disk's center
	Point3d topCenter;

public:

	/**
		default constructor
	*/
	Cylinder();
	
	/**
		constructor from the cylinder's geometry info
	*/
	Cylinder(double baseR_,Point3d &basePt,double topR_,Point3d &topPt);
	
	/**
		destructor
	*/
	~Cylinder();
	
};
}
#endif