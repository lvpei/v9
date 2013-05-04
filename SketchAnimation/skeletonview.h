#pragma once

#include "depthrendertarget.h"
#include "view3d.h"
#include <QGLShaderProgram>
#include <QTimer>
#include <vector>
#include <map>
#include <string>

#include <QGLFramebufferObject>
#include "MEMathLib/Vector3d.h"
#include "shape.h"
#include "ASF/skeleton.h"
#include "ASF/posture.h"

using namespace std;
using namespace MathLib;

class Skeleton;

class SkeletonView: public View3D
{
	Q_OBJECT

public:
	SkeletonView(QWidget* parent);
	~SkeletonView();
	virtual void mousePressEvent ( QMouseEvent * event );
	virtual void mouseReleaseEvent(QMouseEvent *event);
	virtual void mouseMoveEvent(QMouseEvent *event);
	static std::map<string,Shape*>* shapes;

	void startTimer();

protected:

	Vector3d floor_pos_;
	double total_height_;

	virtual void initializeGL();
	virtual void resizeGL( int w, int h );
	virtual void paintGL();

	bool highlightJoint(int joint);
	void drawShadowMap();
	void draw();
	void setLightPosition();

	void render(QSharedPointer< QGLShaderProgram > shader);

	// Rendering data
	float light_angle_x_,light_angle_y_;
	DepthRenderTarget* shadow_map_;
	QGLFramebufferObject* reflection_fbo_;
	QGLFramebufferObject* shadow_fbo_;
	unsigned int shadow_texture_id_;
	QSharedPointer< QGLShaderProgram > shader_;
	QSharedPointer< QGLShaderProgram > simple_shader_;
	QSharedPointer< QGLShaderProgram > shadow_shader_;
	QSharedPointer< QGLShaderProgram > final_shader_;
	GLfloat light_model_view_[16];
	GLfloat light_projection_[16];
	bool pipeRendering;

	// Joint focus
	Vector3d curColor;
	bool highlight_;
	bool focus_camera_;
	int focus_joint_id_;
	QVector3D newCenter_;

	virtual void keyPressEvent( QKeyEvent* event );

public slots:
	/*
    void setReward(double r) { lastReward = r; }
	void setPose( const MatrixXd & Y );
	//*/
	void setFocusJoint( const int joint_id );
    void setFocus(int state);
    void setHighlight(int state);
    void setHeight(double height);

	// receive new motion
	void receiveNewMotion(vector<Posture>&);
	
private slots:
	void updatePose();

private:
	
	// the character skeleton
	Skeleton* m_pSkeleton;
	// the pose sequence
	vector<Posture> m_vPostureSeq;

	// emit signal to update the character pose
	QTimer* m_pTimer;
	bool m_bDrawSkeleton;
};
