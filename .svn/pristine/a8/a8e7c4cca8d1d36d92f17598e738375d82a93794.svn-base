#include "view3d.h"
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QKeyEvent>
#include <cmath>
#include <QApplication>

View3D::View3D(ViewType view_type,QWidget *parent)
	: QGLWidget(QGLFormat(QGL::AlphaChannel | QGL::SampleBuffers),parent),
	view_type_(view_type),
	zoom_(5.0),
	rotation_x_(0.5),
	rotation_y_(0.6),
	center_(0,0,0),
	texture_(0),
	last_pos_(0,0)
{
	ui.setupUi(this);

	setAutoBufferSwap(true);
}

static void loadMatrix(const QMatrix4x4& m) {
	// static to prevent glLoadMatrixf to fail on certain drivers
	static GLfloat mat[16];
	const qreal *data = m.constData();
	for (int index = 0; index < 16; ++index)
		mat[index] = data[index];
	glLoadMatrixf(mat);
}

void View3D::initializeGL() {
	qglClearColor( Qt::gray );
	glDepthMask(TRUE);
	glEnable( GL_DEPTH_TEST );

}


void View3D::paintGL() {

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// Rotate appropriately
	QMatrix4x4 viewmatrix = currentViewMatrix();
	loadMatrix( viewmatrix );

	if (view_type_ == PERSPECTIVE || view_type_ == TOP){
		draw();
	}
	else{

		qglClearColor( Qt::gray );
		glDisable(GL_LIGHTING);

		glBegin( GL_LINES );
		glColor3f( 1., 1., 1. );
		glVertex3d( -100, 0, 0 );
		glVertex3d( 100, 0, 0 );

		glVertex3d( 0, -100, 0 );
		glVertex3d( 0, 100, 0 );

		glVertex3d( 0, 0, -100 );
		glVertex3d( 0, 0, 100 );
		glEnd();
		draw();
	}
}


void View3D::resizeGL(int w, int h) {
	glViewport(0, 0, (GLint)w, (GLint)h);
	QMatrix4x4 pm = currentProjectionMatrix();

	glMatrixMode(GL_PROJECTION);
	loadMatrix( pm );
	glMatrixMode(GL_MODELVIEW);
}

View3D::~View3D() {
	if (texture_)
		deleteTexture(texture_);
}
void View3D::mouseMoveEvent(QMouseEvent* event) {
	if (rotating_){
		QPoint delta = event->pos() - last_pos_;
		rotation_x_ += delta.y() / 60.0;
		if (rotation_x_ > 1.57) rotation_x_ = 1.57;
		if (rotation_x_ <-1.57) rotation_x_ =-1.57;
		rotation_y_ -= delta.x() / 30.0;
		updateGL();
		last_pos_ = event->pos();
	}
}
void View3D::mousePressEvent(QMouseEvent* event) {
	if (event->button() == Qt::LeftButton)
	{
		rotating_ = true;
		last_pos_ = event->pos();
	}
}
void View3D::wheelEvent ( QWheelEvent * event ){
	zoom_ *= exp(-event->delta()/400.0);
	updateGL();
}
void View3D::mouseReleaseEvent(QMouseEvent* event) {
	rotating_ = false;
	updateGL();
}

void View3D::keyPressEvent( QKeyEvent* event ) {
}

QMatrix4x4 View3D::currentViewMatrix() {
	QMatrix4x4 viewmatrix;
	viewmatrix.setToIdentity();
	QVector3D viewdir( sin(rotation_y_)*cos(rotation_x_), sin(rotation_x_), cos(rotation_y_)*cos(rotation_x_) );
	viewmatrix.lookAt( center_ + viewdir*zoom_, center_, QVector3D(0,1,0));
	if (view_type_ == ORTHO || view_type_ == TOP )
		viewmatrix.scale(zoom_);
	return viewmatrix;
}
QMatrix4x4 View3D::currentProjectionMatrix() {
	QMatrix4x4 pm;
	pm.setToIdentity();
	if (view_type_ == ORTHO)
		pm.ortho( -1, 1, -1, 1, -50, 50 );
	else if (view_type_ == TOP)
	{
		double ratio = (double)width() / (double)height();
		pm.ortho( -1, 1, -1./ratio, 1./ratio, 0.01, 1000.0 );  
	}
	else
		pm.perspective( 45.0, (double)width() / (double)height(), 0.1, 1000.0 );
	return pm;
}
QMatrix4x4 View3D::currentReflectedViewMatrix() {
	QMatrix4x4 viewmatrix;
	viewmatrix.setToIdentity();
	QVector3D viewdir( sin(rotation_y_)*cos(rotation_x_), -sin(rotation_x_), cos(rotation_y_)*cos(rotation_x_) );
	center_.setY( -center_.y() );
	viewmatrix.lookAt( center_ + viewdir*zoom_, center_, QVector3D(0,1,0));
	if (view_type_ == ORTHO || view_type_ == TOP )
		viewmatrix.scale(zoom_);
	center_.setY( -center_.y() );
	return viewmatrix;
}