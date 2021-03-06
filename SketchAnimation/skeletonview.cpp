#include "skeletonview.h"
#include <QMouseEvent>

#define SIZE_SPHERE 0.0325
#define RADIUS_CYLINDER 0.0325
#define VOLUME_RENDERING 1
#define SHADOW_RES 2000
#define SHADOW_MAP_SIZE 4.0f


#define FLOOR_SIZE 1000

#define ENABLE_LIGHT 0

//#define WHITE_MODE

#ifdef _WIN32
#define random rand
#define srandom srand
#endif

const double skeleton_scale_factor = 0.09;

static void loadMatrix(const QMatrix4x4& m)
{
	// static to prevent glLoadMatrixf to fail on certain drivers
	static GLfloat mat[16];
	const qreal *data = m.constData();
	for (int index = 0; index < 16; ++index)
		mat[index] = data[index];
	glLoadMatrixf(mat);
}

static std::map<string,Shape*>* init_shapes()
{
	std::map<string,Shape*>* shapes = new std::map<string,Shape*>;

	Shape* cylinder = Shapes::CreateClosedCylinder(1,1,20,2);
	(*shapes)["cylinder"] = cylinder;

	Shape* closed_cylinder = Shapes::CreateClosedCylinder(1,1,20,2);
	(*shapes)["closed_cylinder"] = closed_cylinder;

	Shape* sphere = Shapes::CreateSphere(1, QVector3D(0,0,0),10,10);
	(*shapes)["sphere"] = sphere;

	Shape* floor = Shapes::CreateFloor( FLOOR_SIZE , 100);
	(*shapes)["floor"] = floor;

	Shape* arrow = Shapes::CreateArrow(3.,0.5);
	(*shapes)["arrow"] = arrow;

	return shapes;
}

std::map<string,Shape*>* SkeletonView::shapes = init_shapes();

void setLights() {
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	
	GLfloat ambient[] = {0.2,0.2,0.3, 1.0};
	GLfloat diffuse[] = {0.9,0.9, 0.7, 1.0};
	GLfloat specular[] = {0.5,0.5,0.5, 1.0};
	
	GLfloat position[] = {0, 0, 0, 0.};
	glLightfv(GL_LIGHT0,GL_POSITION,position);
	glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
	glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
	
}


SkeletonView::SkeletonView(QWidget* parent): View3D(PERSPECTIVE,parent)
{
	pipeRendering = true;

	highlight_ = false;
	focus_camera_ = false;
	focus_joint_id_ = -1;
	curColor = Vector3d(1.,1.,1.);
	
	m_pSkeleton = NULL;
	m_bDrawSkeleton = false;

	m_pTimer = NULL;
	m_pTimer = new QTimer();
	m_pTimer->setInterval(30);

	connect(m_pTimer,SIGNAL(timeout()),this,SLOT(updatePose()));
}

SkeletonView::~SkeletonView()
{
	if(m_pTimer)
		delete m_pTimer;

	if(m_pSkeleton)
		delete m_pSkeleton;
}
void SkeletonView::initializeGL() {
	qglClearColor( Qt::gray );
	glDepthMask(TRUE);
	glEnable( GL_DEPTH_TEST );
	//if (view_type_ == PERSPECTIVE)
    texture_ = bindTexture(QPixmap(QString("Resources/datagui/floor2.png")), GL_TEXTURE_2D);
	
	glDisable( GL_CULL_FACE );

	shadow_map_ = new DepthRenderTarget(SHADOW_RES,SHADOW_RES);
	reflection_fbo_ = NULL;
	shadow_fbo_ = NULL;
	
	if(ENABLE_LIGHT) { glEnable(GL_LIGHTING); }
	light_angle_x_ = 0.;
	light_angle_y_ = 1.;
	setLights();
    
    //floor_pos_ = Vector3d(0.,0.,0.);
	floor_pos_ = Vector3d(0.,-0.1,0.);

	// load the skeleton structure
	//m_pSkeleton = new Skeleton(".\\Resources\\boxing_13.asf",0.3);
	//m_pSkeleton = new Skeleton("Resources\\walk_from_jianyuan_5_5\\jianyuan.asf",0.3);
	m_pSkeleton = new Skeleton(".\\Resources\\reaching.asf",0.3);

	if(!m_pSkeleton->isLoaded())
		qDebug()<<"skeleton is not loaded";
	else
	{	
		qDebug()<<"skeleton is loaded";
		m_pSkeleton->set_display_list();
		m_pSkeleton->setBasePosture();
	}
}


void SkeletonView::resizeGL(int w, int h) {
	glViewport(0, 0, (GLint)w, (GLint)h);
	QMatrix4x4 pm = currentProjectionMatrix();
	
	glMatrixMode(GL_PROJECTION);
	loadMatrix( pm );
	glMatrixMode(GL_MODELVIEW);
	
	//if (view_type_ == PERSPECTIVE){
		
		QGLFramebufferObjectFormat format;
		format.setAttachment(QGLFramebufferObject::CombinedDepthStencil);
		if(w != 0 && h != 0) 
		{ 
			reflection_fbo_ = new QGLFramebufferObject(w,h,format); 
			shadow_fbo_ = new QGLFramebufferObject(w,h,format); 
			printf("Resize to : %dx%d\n",w,h);
		}
		
	//}
}


void SkeletonView::setLightPosition()
{
	float light0_x = cosf(light_angle_x_)*cosf(light_angle_y_);
	float light0_y = sinf(light_angle_y_);;
	float light0_z = sinf(light_angle_x_)*cosf(light_angle_y_);;
	
	GLfloat position[] = {light0_x, light0_y, light0_z, 0.};
	glLightfv(GL_LIGHT0,GL_POSITION,position);
	
}

void drawBone( const Vector3d & a, const Vector3d & b ){
	// TODO: Draw a cylinder
	glLineWidth( 5 );
	glBegin( GL_LINES );
		glVertex3f( a.x, a.y, a.z );
		glVertex3f( b.x, b.y, b.z );
	glEnd();
}


void drawCylinderBone( const Vector3d & a, const Vector3d & b, QSharedPointer< QGLShaderProgram > shader, bool highlight, Vector3d curColor = Vector3d(1.,1.,1.), float radius = 0){
	
	if (radius != 0)
    {radius *= skeleton_scale_factor;}
	else
    {
        radius = RADIUS_CYLINDER;
    }
    Vector3d diff = a-b;
	float height = diff.length();
	
	diff.toUnit();
	Vector3d tangent;
	
	if(diff.z != 0){
		 tangent = Vector3d(0,1.,-diff.y / diff.z); }
	else if(diff.y != 0) { tangent = Vector3d(0,-diff.z / diff.y,1.); }
	else {
		tangent = Vector3d(1.,-diff.z / diff.x,0.);
	}
	
	Vector3d bitangent = diff.crossProductWith(tangent);
	tangent.toUnit();
	bitangent.toUnit();
	
	float basisMat[16] = {tangent.x,tangent.y,tangent.z,0.,bitangent.x,bitangent.y,bitangent.z,0.,diff.x,diff.y,diff.z,0.,0.,0.,0.,1.};
	
	if (highlight) { glColor3f(0.,1.,0.); }
	else {
		glColor3f(curColor.x,curColor.y,curColor.z);	
	}
	
	glPushMatrix();
	glTranslatef(b.x,b.y,b.z);

	glMultMatrixf(basisMat);

	glScalef(radius,radius,height);
	(*SkeletonView::shapes)["cylinder"]->draw(shader);
	glPopMatrix();
}


void drawSphereJoint( const Vector3d & a , QSharedPointer< QGLShaderProgram > shader, bool highlight, Vector3d curColor = Vector3d(1.,1.,1.)){
	if (highlight) { glColor3f(0.,1.,0.); }
	else {
		glColor3f(curColor.x,curColor.y,curColor.z);	
	}
	glPushMatrix();
		glTranslatef(a.x,a.y,a.z);
		glScalef(SIZE_SPHERE,SIZE_SPHERE,SIZE_SPHERE);
		(*SkeletonView::shapes)["sphere"]->draw(shader);
	glPopMatrix();
}

void drawFloor( const Vector3d & a , QSharedPointer< QGLShaderProgram > shader )
{
	glPushMatrix();
	glTranslatef(a.x,a.y,a.z);
	(*SkeletonView::shapes)["floor"]->draw(shader);
	glPopMatrix();
}

void drawFloor2( const Vector3d & a , QSharedPointer< QGLShaderProgram > shader )
{
    (*SkeletonView::shapes)["floor"]->draw(shader);
}

void drawJoint( const Vector3d & a ){
	// TODO: Draw a sphere
	glPointSize( 7 );
	glBegin( GL_POINTS );
		glVertex3f(a.x, a.y, a.z);
	glEnd();
}

void randomColor(){
	const double rmin = 0.25;
	const double rmax = 1.0;
	const double rng = rmax - rmin;
	glColor3f( rmin + rng*random() / RAND_MAX, rmin + rng*random() / RAND_MAX, rmin + rng*random() / RAND_MAX );
}

void SkeletonView::render(QSharedPointer< QGLShaderProgram > shader)
{   
	if(!m_bDrawSkeleton)
		return;

	// obtain the root joint
	Bone root = m_pSkeleton->m_pBoneList[0];
    
	// draw the bone and joint
	for ( int i=1; i < m_pSkeleton->NUM_BONES_IN_ASF_FILE; i++ )
    {
		Bone bone = m_pSkeleton->m_pBoneList[i];

        Vector3d a(bone.m_GlobalPosition[0] * skeleton_scale_factor,bone.m_GlobalPosition[1] * skeleton_scale_factor,bone.m_GlobalPosition[2] * skeleton_scale_factor);
        
        if(VOLUME_RENDERING)
		{
			drawSphereJoint( a, shader, false, curColor);
		}
		else {
			drawJoint( a );
		}
        
        int parent_id = m_pSkeleton->findBoneByName(&root,bone.parent_name)->idx;
        
        if (parent_id != i && parent_id >= 0)
        {
			bone = m_pSkeleton->m_pBoneList[parent_id];
            Vector3d b(bone.m_GlobalPosition[0] * skeleton_scale_factor,bone.m_GlobalPosition[1] * skeleton_scale_factor,bone.m_GlobalPosition[2] * skeleton_scale_factor);
			if(VOLUME_RENDERING)
			{ drawCylinderBone( a, b, shader, false, curColor); }
			else { drawBone( a, b ); }
        }
    }
}

// Determine if a particular joint should be highlighted.
bool SkeletonView::highlightJoint(int joint )
{
	/*
    // Always highlight the selected joint.
    if (focus_joint_id_ == joint) return true;

    // Check if we're highlighting the joint because of foot skate cleanup.
    return controlSets[activeControlSet].optimizer->isHighlighted(joint);
	//*/

	return true;
}

void SkeletonView::drawShadowMap() {
	
	Vector3d pos(0.0,0.0,0.0);
    
	glViewport(0, 0,SHADOW_RES,SHADOW_RES);

	simple_shader_->bind();
	GLfloat nearClip = -10.f;
	GLfloat farClip = 10.f;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho( -SHADOW_MAP_SIZE,SHADOW_MAP_SIZE,-SHADOW_MAP_SIZE,SHADOW_MAP_SIZE,nearClip, farClip);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	float light0_x = cosf(light_angle_x_)*cosf(light_angle_y_);
	float light0_y = sinf(light_angle_y_);;
	float light0_z = sinf(light_angle_x_)*cosf(light_angle_y_);
	
	gluLookAt(pos.x,pos.y,pos.z,pos.x-light0_x, pos.y-light0_y, pos.z-light0_z, -light0_z, 0.0f, light0_x);
    //gluLookAt(floor_pos_.x(),floor_pos_.y(),floor_pos_.z(),floor_pos_.x()-light0_x, floor_pos_.y()-light0_y, floor_pos_.z()-light0_z, -light0_z, 0.0f, light0_x);
    
	glEnable(GL_TEXTURE_2D);

	shadow_map_->bind();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	render(simple_shader_);
	shadow_map_->unbind();
	
	
	simple_shader_->release();
	
	shadow_texture_id_ = shadow_map_->textureID();
	glGetFloatv(GL_MODELVIEW_MATRIX, light_model_view_);
	glGetFloatv(GL_PROJECTION_MATRIX, light_projection_);

	
	// Debug
	
/*	glActiveTexture(GL_TEXTURE6);
	
	glViewport(0, 0,width(),height());
	
	shadow_shader_->bind();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	GLint filtered = shadow_shader_->uniformLocation("filtered");
   	glUniform1i(filtered,6); 
	glActiveTexture(GL_TEXTURE6);
	glBindTexture(GL_TEXTURE_2D, shadow_texture_id_);
	
	glMatrixMode(GL_PROJECTION);
   	glLoadIdentity();
   	glMatrixMode(GL_MODELVIEW);
   	glLoadIdentity();
	glBegin(GL_QUADS);
	glVertex2f(-1,-1);
	glVertex2f(1,-1);
	glVertex2f(1,1);
	glVertex2f(-1,1);
	glEnd();
	
	glBindTexture(GL_TEXTURE_2D, 0);
	shadow_shader_->release();*/

}


void SkeletonView::draw() {
   	
	/*
	// Move the floor if character too far from its center
	Vector3d pos = Vector3d(m_pSkeleton->m_RootPos[0],m_pSkeleton->m_RootPos[1],m_pSkeleton->m_RootPos[2]);
    pos.multiplyBy(skeleton_scale_factor);
    
	floor_pos_.multiplyBy(skeleton_scale_factor);
	Vector3d tmp = pos - floor_pos_;
    if (tmp.length() > 1.)
     {
         floor_pos_.x = floor(pos.x);
         floor_pos_.z = floor(pos.z);
     }
    //*/

    // Link shaders
	if (simple_shader_.isNull())
		simple_shader_ = QSharedPointer< QGLShaderProgram >( new QGLShaderProgram );
	if (!simple_shader_->isLinked()){
		simple_shader_->addShaderFromSourceFile( QGLShader::Vertex  , "Resources/shaders/simple.vert" );
		simple_shader_->addShaderFromSourceFile( QGLShader::Fragment, "Resources/shaders/simple.frag" );
		if (!simple_shader_->link())
			qFatal("Shader linking failed!");
	}	
	
	if (shader_.isNull())
		shader_ = QSharedPointer< QGLShaderProgram >( new QGLShaderProgram );
	if (!shader_->isLinked()){
		shader_->addShaderFromSourceFile( QGLShader::Vertex  , "Resources/shaders/phong.vert" );
		shader_->addShaderFromSourceFile( QGLShader::Fragment, "Resources/shaders/phong.frag" );
		if (!shader_->link())
			qFatal("Shader linking failed!");
	}	
	
	if (shadow_shader_.isNull())
		shadow_shader_ = QSharedPointer< QGLShaderProgram >( new QGLShaderProgram );
	if (!shadow_shader_->isLinked()){
		shadow_shader_->addShaderFromSourceFile( QGLShader::Vertex  , "Resources/shaders/shadow.vert" );
		shadow_shader_->addShaderFromSourceFile( QGLShader::Fragment, "Resources/shaders/shadow.frag" );
		if (!shadow_shader_->link())
			qFatal("Shader linking failed!");
	}	
	
	if (final_shader_.isNull())
		final_shader_ = QSharedPointer< QGLShaderProgram >( new QGLShaderProgram );
	if (!final_shader_->isLinked()){
		final_shader_->addShaderFromSourceFile( QGLShader::Vertex  , "Resources/shaders/final.vert" );
		final_shader_->addShaderFromSourceFile( QGLShader::Fragment, "Resources/shaders/final.frag" );
		if (!final_shader_->link())
			qFatal("Shader linking failed!");
	}	
	
	
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	
	// Rotate appropriately
	QMatrix4x4 viewmatrix = currentViewMatrix();
	loadMatrix( viewmatrix );
	
	// Set the light position
	setLightPosition();
	
	// Draw the coordinate frame
	if (texture_){
		glPushAttrib( GL_TEXTURE_BIT );
		glEnable( GL_TEXTURE_2D );
		glBindTexture( GL_TEXTURE_2D, texture_ );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	}
	
	if (reflection_fbo_ != NULL){
		
#ifdef WHITE_MODE
		qglClearColor( Qt::white );
#else
		qglClearColor( Qt::black );
#endif
		
		if(!pipeRendering)
		{
			glDisable(GL_LIGHTING);
			
			glBegin( GL_TRIANGLE_STRIP );
			//glColor3f( 0.1, 0.75, 0.25 );
			glColor3f( .8, .8, .8 );
			glTexCoord2d(  100,  100 );
			glVertex3d(  100, 0,  100 );
			
			glTexCoord2d( -100,  100 );
			glVertex3d( -100, 0,  100 );
			
			glTexCoord2d(  100, -100 );
			glVertex3d(  100, 0, -100 );
			
			glTexCoord2d( -100, -100 );
			glVertex3d( -100, 0, -100 );
			glEnd();
			
			//glUseProgram(0);
			glDisable(GL_TEXTURE_2D);
			//drawWire();
			glEnable(GL_TEXTURE_2D);
		}
		else 
		{
			glEnable(GL_LIGHTING);
			
			QMatrix4x4 pm = currentProjectionMatrix();
			
			glViewport(0, 0,width(),height());
		
#ifndef WHITE_MODE
			// Reflection first pass
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			loadMatrix( pm );
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			loadMatrix( currentReflectedViewMatrix() );
			setLightPosition();
			
			reflection_fbo_->bind();
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			shader_->bind();
			render(shader_);
			shader_->release();
			reflection_fbo_->release();
			
			// Shadow Map first pass
			drawShadowMap();
			
			// Shadow Map second pass
			glViewport(0, 0,width(),height());
			
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			loadMatrix( pm );
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			loadMatrix( viewmatrix );
			setLightPosition();
			
			shadow_shader_->bind();
			
			GLint lightmv = shadow_shader_->uniformLocation("lightMv");
			glUniformMatrix4fv(lightmv, 1, false, light_model_view_);
			
			GLint lightproj = shadow_shader_->uniformLocation("lightProj");
			glUniformMatrix4fv(lightproj, 1, false, light_projection_);
			
			GLint depth = shadow_shader_->uniformLocation("depthMap");
			glUniform1i(depth,3); 
			glActiveTexture(GL_TEXTURE3);
			glBindTexture(GL_TEXTURE_2D,shadow_texture_id_);
			
			shadow_fbo_->bind();
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
			drawFloor2(Vector3d(0,0,0),shadow_shader_);
            
			shadow_fbo_->release();
			shadow_shader_->release();
			
			// Shadow Map third pass with reflection
			final_shader_->bind();
			
			GLint diffuse = final_shader_->uniformLocation("diffuseMap");
			glUniform1i(diffuse,1); 
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D,texture_);
			
			GLint reflection = final_shader_->uniformLocation("reflectionMap");
			glUniform1i(reflection,2); 
			glActiveTexture(GL_TEXTURE2);
			glBindTexture(GL_TEXTURE_2D,reflection_fbo_->texture());
			
			GLint shadow = final_shader_->uniformLocation("shadowMap");
			glUniform1i(shadow,4); 
			glActiveTexture(GL_TEXTURE4);
			glBindTexture(GL_TEXTURE_2D,shadow_fbo_->texture());
            
			drawFloor(floor_pos_,final_shader_);
			
			final_shader_->release();
#endif
			
			// Draw the final pass
			shader_->bind();
			render(shader_);
			shader_->release();
		}
		if (texture_){
			glBindTexture( GL_TEXTURE_2D, 0 );
			glPopAttrib();
		}
	}
	
}


void SkeletonView::mousePressEvent(QMouseEvent* event) {
		View3D::mousePressEvent( event );
}

void SkeletonView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
	{
		;
	}
    else
        View3D::mouseReleaseEvent(event);
}

void SkeletonView::mouseMoveEvent(QMouseEvent* event)
{
	View3D::mouseMoveEvent(event);
}


void SkeletonView::setFocusJoint( const int joint_id ) {
    focus_joint_id_ = joint_id-1;
}


void SkeletonView::keyPressEvent( QKeyEvent* event ) {
	
}

void SkeletonView::setHighlight(int state)
{
	if (state == 2) { highlight_ = true; }
	else highlight_ = false;
}

void SkeletonView::setFocus(int state)
{
	if (state == 2) { focus_camera_ = true; }
	else focus_camera_ = false;
}

void SkeletonView::setHeight(double height)
{
	total_height_ = height;
}

void SkeletonView::paintGL()
{
	draw();
}

void SkeletonView::updatePose()
{
	static int frame = 0;

	if(m_vPostureSeq.size() == 0)
		return;

	frame %= m_vPostureSeq.size();

	m_pSkeleton->setPosture(m_vPostureSeq[frame]);

	emit updateStep(frame);

	frame++;

	update();
}

// receive new motion
void SkeletonView::receiveNewMotion(vector<Posture>& posture_arr)
{
	m_vPostureSeq.assign(posture_arr.begin(),posture_arr.end());

	startTimer();

	m_bDrawSkeleton = true;

	emit updateTimelineRangeAndInterval(0,m_vPostureSeq.size()-1,5);
}

void SkeletonView::startTimer()
{
	m_pTimer->start();
}