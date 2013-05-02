#ifndef VIEW3D_H
#define VIEW3D_H

#include <QGLWidget>
#include "ui_view3d.h"
#include <QVector3D>
#include <QPoint>
#include <QGLShaderProgram>
#include <QTime>
#include <QCloseEvent>
#include <gl\GLU.h>

class View3D : public QGLWidget
{
	Q_OBJECT

protected:
	enum ViewType{
		PERSPECTIVE,
		ORTHO,
		TOP
	};

public:
	View3D(ViewType view_type,QWidget *parent = 0);
	virtual ~View3D();

private:
	Ui::View3D ui;

	ViewType view_type_;
	double zoom_, rotation_x_, rotation_y_;
	QVector3D center_;

protected: // OpenGL stuff
	unsigned int texture_;
	QMatrix4x4 currentViewMatrix();
	QMatrix4x4 currentReflectedViewMatrix();
	QMatrix4x4 currentProjectionMatrix();

protected: // Drawing and more

	virtual void initializeGL();
	virtual void resizeGL( int w, int h );
	virtual void paintGL();
	virtual void draw() = 0;

protected: // Navigation
	bool rotating_;
	QPoint last_pos_;
	virtual void mouseMoveEvent ( QMouseEvent * event );
	virtual void mousePressEvent ( QMouseEvent * event );
	virtual void wheelEvent ( QWheelEvent* event );
	virtual void mouseReleaseEvent ( QMouseEvent * event );
	virtual void keyPressEvent( QKeyEvent* event );
};

#endif // VIEW3D_H
