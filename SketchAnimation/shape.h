/*
 *  shape.h
 *  
 *
 */
#pragma once

#include <vector>
#include <QGLShaderProgram>

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

class Shape
{
protected:

public:
    typedef unsigned int Index;
    
	Shape();
    Shape(const std::string& filname);
	void draw(QSharedPointer< QGLShaderProgram > shader);
	void drawFirstPass();
    void generateNormals();
	std::vector<unsigned int> indexBuffer;
	std::vector<QVector3D> vertexBuffer;
	std::vector<QVector3D> normalBuffer;
	std::vector<QVector2D> texcoordBuffer;
private:
	//void render(QSharedPointer< QGLShaderProgram > shader);
    void LoadOBJ(const std::string& filname);

};

namespace Shapes
{
    Shape* CreateCylinder(
							float radius, float height,
							unsigned int numSlices = 20,
							unsigned int numStacks = 2);

    Shape* CreateClosedCylinder(
                          float radius, float height,
                          unsigned int numSlices = 20,
                          unsigned int numStacks = 2);
    
    Shape* CreateSphere(
						  float radius, const QVector3D center = QVector3D(0,0,0),
						  unsigned int numSlices = 15,
						  unsigned int numStacks = 15);
	
	Shape* CreateFloor( float size, int subdiv );
	
	Shape* CreateArrow(float tailSize, float thickness);
}
