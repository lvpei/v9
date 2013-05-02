#include "renderarea.h"
#include <QPainter>
#include <QMouseEvent>
#include <QPoint>
#include <QtDebug>
#include "mainwindow.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include "BVH.h"
#include "LSH/BasicDefinitions.h"
#include "LSH/Random.h"
#include "MEMathLib/TransformationMatrix.h"
#include "gsl-1.15/include/gsl/gsl_randist.h"
#include "PSO.h"
#include "..\MEMathLib\AxisAngle.h"
#include "..\MEMathLib\UpSampling.h"
#include <QFile>
#include <QtXml/QDomDocument>
#include <QTextStream>
#include <QString>
#include <iostream>
#include "MEMotionClip.h"

using namespace std;

int JOINT_NAME_STACK = 1000;

const int MAX_SKETCH_NUM = 7;

const int IMAGE_SIZE = 480;

const float SKELETON_SCALE = 0.3;

#define SAMPLING_WHOLE_BODY
//#define SAMPLING_JOINTS

QColor aColor[7] = {
	QColor(255,0,0),QColor(255,165,0),QColor(255,255,0),
	QColor(0,255,0),QColor(0,127,255),QColor(0,0,255),QColor(139,0,255)															
}; 

QString hintStr[7] = {
	"Torso", "Left Shoulder", "Left Arm","Right Shoulder","Right Arm","Left Leg","Right Leg"
};

char* motion_clip_filename[] = {
	"Resources\\walk\\walk_02_01_Take_001.amc",
	"Resources\\walk\\walk_05_01_Take_001.amc",
	"Resources\\walk\\walk_07_04_Take_001.amc",
	"Resources\\walk\\walk_07_12_Take_001.amc",
	"Resources\\walk\\walk_08_05_Take_001.amc",
	"Resources\\walk\\walk_08_08_Take_001.amc",
	"Resources\\walk\\walk_08_11_Take_001.amc",
	"Resources\\walk\\walk_15_03_Take_001.amc",
	"Resources\\walk\\walk_17_03_Take_001.amc",
	"Resources\\walk\\walk_17_05_Take_001.amc"
};
RenderArea::RenderArea(QWidget *parent)
	: QGLWidget(QGLFormat(QGL::AlphaChannel|QGL::SampleBuffers), parent)
{
	setBackgroundRole(QPalette::Base);
	setAutoFillBackground(true);

	image = QImage(IMAGE_SIZE,IMAGE_SIZE,QImage::Format_ARGB32);  //32bit color with size is 720,720
	backColor = qRgba(255,255,255,0);				//initial background color is white, transparent
	image.fill(backColor);
	resultImage = backImage = gridImage = styleUsrDrawingImage = image;

	shape = LINE;
	pen.setWidth(1);
	m_bShowGrid = true;
	m_bShowSketch = true;
	m_bShowShadow = true;
	bContentChanged = false;
	m_DrawMode = PEN_MODE;

	// create the cursor pixmap
	m_penPixmap.load(".\\Resources\\cursor\\pen.png","png");
	m_eraserPixmap.load(".\\Resources\\cursor\\eraser.png","png");

	m_cursor = QCursor(m_penPixmap,0,30);

	setCursor(m_cursor);

	stylePen.setColor(QColor(255,0,0));
	stylePen.setWidth(12.0);

	drawGridImage(gridImage);

	int w = gridImage.width();

	pen.setColor(QColor(0,0,0));
	// create the timer
	m_pTimer = NULL;

	m_pTimer = new QTimer(this);
	//connect(m_pTimer, SIGNAL(timeout()), this, SLOT(dynamicShadowHint()));
	//connect(m_pTimer, SIGNAL(timeout()), this, SLOT(repaint()));
	connect(m_pTimer, SIGNAL(timeout()), this, SLOT(screenCaptureAsPng()));

	// 30 FPS to draw the scene
	//m_pTimer->start(500);

	// mocap data
	m_pBvh = NULL;

	// camera
	constructDefaultCamera();

	m_Camera = m_DefaultCamera;

	// whether to show 3d pose
	m_b3DPose = false;

	// initialize the array to null
	for(int i = 0; i < 20; i++)
		m_pViewArray[i] = NULL;
	m_iCurrenView = m_iViewNum = 0;

	m_bInitMousePosition = true;

	behaviorMode = SKETCH;

	m_bLeftButtonDown = false;
	m_bRightButtonDown = false;
	m_bMidButtonDown = false;

	m_pSkeleton = NULL;
	m_pMotion = NULL;

	defaultColor = QColor(34,139,34,255);

	m_bStartUp = true;

	m_iAxisIdx = 0;			// no rotation circle is selected

	// create the default joint chain
	visualizeBodyParts(0);

	m_iSketchOrder = 0;

	m_bRefined = false;
	m_dDist = 0.0;

	// initialize the Model-View & Projection Matrix, undone

	m_bSketchPose = true;

	m_iShowTrajectoryIndex = 0;

	m_TrajectoryFeature.resize(5);
}

RenderArea::~RenderArea()
{
	// delete the timer
	if(m_pTimer)
		delete m_pTimer;

	// delete the BVH
	if(m_pBvh)
	{
		delete m_pBvh;
		m_pBvh = NULL;
	}

	// delete views
	for(int i = 0; i < m_iViewNum; i++)
	{	
		delete m_pViewArray[i];
		m_pViewArray[i] = NULL;
	}

	if(m_pSkeleton)
		delete m_pSkeleton;

	for(int i = 0; i < m_vMotionClip.size(); i++)
	{
		delete m_vMotionClip[i];
	}
}

void RenderArea::setShape(DrawShape shape)
{
	this->shape = shape;
	update();
}
void RenderArea::setPen(const QPen &pen)
{
	this->pen = pen;
	update();
}

void RenderArea::setBrush(const QBrush &brush)
{
	this->brush = brush;
	update();
}

void RenderArea::paintEvent(QPaintEvent * /* event */)
{
	makeCurrent();

	//glEnable(GL_TEXTURE_2D);	// Enable Texture Mapping

	glEnable(GL_MULTISAMPLE);

	// clear the background to dark
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0f,0.0f,0.0f,1.0f);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// set the viewport and projection matrix
	setupViewport(width(), height());

	// set the ModelView matrix
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// always use the default camera
	gluLookAt(m_Camera.m_camPos[0],m_Camera.m_camPos[1],m_Camera.m_camPos[2],m_Camera.m_center[0],m_Camera.m_center[1],m_Camera.m_center[2],m_Camera.m_up[0],m_Camera.m_up[1],m_Camera.m_up[2]);

	// Panning the translation.
	glTranslated(-1.0 * m_Camera.m_pan[0], -1.0 * m_Camera.m_pan[1], 0.0);

	// Points camera to desired target.
	glTranslated(-1.0 * m_Camera.m_target[0], -1.0 * m_Camera.m_target[1], -1.0 * m_Camera.m_target[2]);

	glRotatef(m_Camera.m_rotations[0],1.0,0.0,0.0);
	glRotatef(m_Camera.m_rotations[1],0.0,1.0,0.0);
	glRotatef(m_Camera.m_rotations[2],0.0,0.0,1.0);

	// obtain the Model-View matrix
	glGetDoublev(GL_MODELVIEW_MATRIX,m_vModelViewMatrix);

	float vPosition[4] = {0.0f, 5.0f, 10.0f, 1.0f};
	glLightfv(GL_LIGHT0, GL_POSITION, vPosition);

	// draw the ground
	//glPushMatrix();
	//glTranslatef(0.0,-10.0,0.0);
	//drawGround();
	//glPopMatrix();

	// draw the shadow
	//drawShadow();

	glEnable(GL_BLEND);
	glDepthMask(GL_FALSE);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	if(m_bSketchPose)
	{
		drawSketchingPoseInterface();
	}
	else
	{
		drawSketchingAnimationInterface();
	}

	if(behaviorMode == VIEW)
	{
		// draw navigation ball	
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE);
		drawNavigationBall();
		glDisable(GL_BLEND);
	}

	if(behaviorMode == VIEW && m_iAxisIdx == 0)
	{
		const int BUFFERSIZE = 512;
		GLuint selectBuf[BUFFERSIZE];
		GLint hits = -1;
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT,viewport);
		glSelectBuffer(BUFFERSIZE,selectBuf);
		glRenderMode(GL_SELECT);

		glInitNames();
		glPushName(0);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();

		// projection matrix
		gluPickMatrix(m_iPickX,viewport[3] - m_iPickY,4,4,viewport);
		glMultMatrixd(m_vProjectionMatrix);

		glMatrixMode(GL_MODELVIEW);

		// draw navigation ball
		drawNavigationBall();

		// draw the character
		//m_pSkeleton->RenderFigure(bone_color,joint_color);

		// restore the original project matrix
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glFlush();

		hits = glRenderMode(GL_RENDER);	
		if(hits > 0 && selectBuf[0])
		{
			processSelection(hits,selectBuf);
		}
		else
			m_iAxisIdx = 0;						// no rotation circle is selected
	}

	// 3d poses image
	threedpose = grabFrameBuffer(true);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	//glDisable(GL_TEXTURE_2D);		// Disable Texture Mapping

	QPainter painter(this);
	// the background
	painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
	if(m_b3DPose)	// 3D
	{
		if(m_bShowShadow)
		{
			// background
			painter.drawImage(0,0,threedpose);	// candidate 3D poses
		}

		if(m_bShowSketch)
		{
			// foreground
			painter.setCompositionMode(QPainter::CompositionMode_SourceOver);	
			//painter.drawImage(0,0,threedstyleline);	// user's sketch
			painter.drawImage(0,0,styleUsrDrawingImage);	// user's sketch

		}
	}
	else	// 2D
	{
		if(m_bShowShadow)
		{
			// background
			painter.drawImage(0,0,backImage);	// candidate images
		}

		if(m_bShowGrid)
		{
			painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
			painter.drawImage(0,0,gridImage);	// grid images
		}
		if(m_bShowSketch)
		{
			// foreground
			painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
			painter.drawImage(0,0,styleUsrDrawingImage);	// user's sketch
		}
	}
	
	if(m_iSketchOrder < MAX_SKETCH_NUM)
	{
		QFont serifFont("Times New Roman", 22, QFont::Bold);
		painter.setFont(serifFont);
		painter.setPen(QColor(aColor[m_iSketchOrder]));
		painter.drawText(10,30,hintStr[m_iSketchOrder]);
	}
	painter.end();
}

void RenderArea::mousePressEvent(QMouseEvent *event)
{
	if(SKETCH == behaviorMode)
	{
		if(event->button() == Qt::LeftButton)
		{
			if(m_iSketchOrder>-1)
			{
				stylePen.setColor(aColor[m_iSketchOrder]);
			
				MainWindow *pParent = dynamic_cast<MainWindow*>(parentWidget()->parentWidget());
				if(pParent)
				{
					pParent->SetSkectchOrderLabelBackground(aColor[m_iSketchOrder]);
				}
			}

			point = event->pos();
			m_vPoints.clear();
			m_vPoints.push_back(point);

			m_TimerSketch.start();
		}
	}
	else if(VIEW == behaviorMode)
	{
		if(event->button() & Qt::LeftButton)
		{	
			m_bLeftButtonDown = true;
			m_iPickX = event->x();
			m_iPickY = event->y();
			repaint();
		}
		else if(event->button() & Qt::RightButton)
			m_bRightButtonDown = true;
		else if(event->button() & Qt::MidButton)
			m_bMidButtonDown = true;
	}
}

/*
	compute the new candidate sets
*/
void RenderArea::updateCandidateSets()
{
	qDebug()<<"changed patch index";

	int left,up;			// left up corner coordinate for sub area
	QImage subarea;
	char varname[256];

	vector<QImage> subarea_vec;
	vector<int> patchindex_vec;
	vector<Mat> img_sketch;

	if(subarea_vec.size() == 0)
	{
		qDebug()<<"No patch changes";
		return;
	}

	vector<Mat> bin_sketch;
	vector<vector<double>> dist;
	vector<vector<int>> sortpatchindex;

	imgindex.clear();
	//vector<int> finaldist_sort;
	vector<double> dist_sorted;

	// compute the elapsed time for different parts of this function
	QTime t;
	t.start();
	// deal with all the patches user sketch
	imageprocess.batch_process_sketch(img_sketch,bin_sketch);
	qDebug("Time elapsed for batch process user sketch: %d ms", t.elapsed());

	//* continuous comparison
	//* commented by lvp 12-5-19
	t.restart();
	imageprocess.patchdistmeasure(bin_sketch,patchindex_vec,dist,sortpatchindex,imgindex,dist_sorted);
	qDebug("Time elapsed for patch distance measure: %d ms",t.elapsed());
	//*/

	/************************************************************************/
	/* using the min-hash and inverse structure index to fast find the most
	nearest poses in the database*/
	/************************************************************************/
	/*
	vector<vector<int>> sketches,nearestpatchidx;
	t.restart();
	imageprocess.minhash(bin_sketch,sketches,true,"E:\\TAMU Project\\SketchAnimation\\SketchAnimation\\tmp\\tmp.inv");
	qDebug("Time elapsed for min-hash: %d ms", t.elapsed());

	t.restart();
	vector<int> finaldist_sort;
	// two different versions for voting
	//imageprocess.vote(sketches,patchindex_vec,imgindex,nearestpatchidx,finaldist_sort);
	imageprocess.vote(bin_sketch,sketches,patchindex_vec,imgindex,nearestpatchidx,dist_sorted);

	qDebug("Time elapsed for voting: %d ms", t.elapsed());
	//*/	

	if(imgindex.size() > m_iDisplayNearestNum)
		imgindex.erase(imgindex.begin() + m_iDisplayNearestNum,imgindex.end());

	// assign different colors to selected images
	int* index = new int[imgindex.size()];
	double* color = new double[imgindex.size()];
	vector<double> color_v;
	if(imgindex.size() == 1)
	{
		index[0] = imgindex[0];
		color[0] = 0;
		color_v.push_back(0);
	}
	else
	{
		for(int i = 0; i < imgindex.size(); i++)
		{
			index[i] = imgindex[i];
			color[i] = (double)i/(imgindex.size()-1);
			color_v.push_back((double)i/(imgindex.size()-1));
		}
	}

	/************************************************************************/
	/* Final display                                                        */
	/************************************************************************/
	if(imgindex.size() > 0)
	{
		/************************************************************************/
		/*  Deal with the sketch image                                          */
		/************************************************************************/
		// convert qimage to mat
		Mat usr_sketch = qimage2mat(image);

		/*
		// change the red color to black, the same to database image
		MatIterator_<Vec3b> it,end;
		for(it = usr_sketch.begin<Vec3b>(),end = usr_sketch.end<Vec3b>(); it != end; ++it)
		{
		if((*it)[2] == 255 && (*it)[0] == 0 && (*it)[1] == 0)
		{
		(*it)[2] = 0;
		}
		}
		//*/

		/************************************************************************/
		/*  Image alignment                                                     */
		/************************************************************************/
		// compute the transformation and apply these transformation to images, for present only translation (x,y)
		//* commented 4-12
		vector<vector<double>> transform;
		// the transformed edge image
		vector<Mat> db_transformed_edge_response;
		vector<Mat> db_transformed_edge_orientation;

		// just keep several images to show the result roughly
		/*
		int num = 20;
		if(selectedImg.size() > num)
		{	
		selectedImg.erase(selectedImg.begin() + num,selectedImg.end());
		imgindex.erase(imgindex.begin() + num,imgindex.end());
		}
		//*/

		/* commented by lvp 12-5-10
		t.restart();
		imageprocess.imagealignment(usr_sketch,selectedImg,transform,db_transformed_edge_response,db_transformed_edge_orientation,true);
		//imageprocess.imagealignment(usr_sketch,selectedImg,transform);
		qDebug("Time elapsed for image alignment: %d ms",t.elapsed());
		//*/

		/************************************************************************/
		/* Image Weighting                                                      */
		/************************************************************************/
		//*
		// weight image
		int nWeightImage = imgindex.size();
		vector<Mat> WeightImage;
		WeightImage.resize(nWeightImage);

		// store the Vi,vi,hi
		vector<Mat> V;
		vector<double> v,h;

		// eight different orientation
		vector<double> orientation_v;
		for(int i = 0; i < 8; i++)
		{
			orientation_v.push_back(i * 22.5 * 3.1415 / 180);
		}

		// obtain the user sketch image
		//QImage tmp = image.copy(minLeft * 60 + 1,minUp * 60 + 1,(maxRight - minLeft + 1) * 60,(maxBottom - minUp + 1) * 60);
		Mat& usr_sketch_img = usr_sketch;

		// extract edge image from user sketch image
		Mat edgelength,edgeorientation,m,m_n,edgeresponse;
		imageprocess.edgeextraction(usr_sketch_img,edgelength,edgeorientation,m,m_n,edgeresponse);
		edgeresponse = m_n * _COEFFICIENT_;
		//edgeresponse = m_n;

		t.restart();
		// decompose the edge image from user sketch
		vector<Mat> decompose_usr_sketch;
		for(int j = 0; j < _REDUCE_RESOLUTION_; j++)
		{
			int rows = edgeresponse.rows;
			int cols = edgeresponse.cols;
			pyrDown(edgeresponse,edgeresponse,cv::Size(rows/2,cols/2));
			pyrDown(edgeorientation,edgeorientation,cv::Size(rows/2,cols/2));
		}
		imageprocess.decomposeimage(edgeresponse,edgeorientation,orientation_v,decompose_usr_sketch);
		qDebug("Time elapsed for user sketch image decomposing: %d ms",t.elapsed());

		// compute the value of V
		vector<Mat> edgeresponse_db_v;
		for(int i = 0; i < nWeightImage; i++)
		{
			/* commented by lvp 2012-5-10
			// obtain the edge image
			edgeresponse_db_v.push_back(imageprocess.getEdgeResponse(imgindex[i])*1000);

			// obtain the decomposed edge image
			vector<Mat> decompose_db_img = imageprocess.getDecEdgeResponse(imgindex[i]);
			//*/

			vector<Mat> decompose_db_img;
			edgeresponse_db_v.push_back(imageprocess.getEdgeResponse(imgindex[i]) * _COEFFICIENT_ );
			imageprocess.decomposeimage(imgindex[i],orientation_v,decompose_db_img);

			// compute the positive and negative correlation image
			Mat positive_corr_img, negative_corr_img;
			imageprocess.computeedgecorrimg(decompose_db_img,decompose_usr_sketch,positive_corr_img,negative_corr_img);

			Scalar sca = sum(positive_corr_img - negative_corr_img);
			h.push_back(sca.val[0]);

			// here should be applied a Gaussian blur
			GaussianBlur(positive_corr_img,positive_corr_img,Size(11,11),0);

			// a small offset to ensure non-zero values
			Mat mat(positive_corr_img.rows,positive_corr_img.cols,positive_corr_img.type(),Scalar::all(0.1));
			positive_corr_img = positive_corr_img + mat;

			V.push_back(positive_corr_img);
		}
		qDebug("Time elapsed for database image decomposing: %d ms",t.elapsed());

		t.restart();
		// compute the value v
		vector<double> h_sort = h;
		sort(h_sort.begin(),h_sort.end(),greater<double>());
		double h_star = 0.0;
		int j = 0;
		for(; j < 5 && j < h_sort.size(); j++)
		{
			h_star += h_sort[j];
		}
		h_star /= j;
		double gamma = 0.5, kappa = 2;
		for(int i = 0; i < nWeightImage; i++)
		{
			double tmp = (h[i] - gamma * h_star)/(h_star - gamma * h_star);
			double maxVal = max(0.0,pow(tmp,2));
			v.push_back(maxVal);
		}

		double sum_v = 0.0;
		for(int i = 0; i < v.size(); i++)
			sum_v += v[i];

		// compute alpha
		double epsilon = 5.0;
		//double alpha = sum_v/(epsilon+sum_v);
		double alpha = 10.0;

		// compute the weight image
		Mat sum_vV(V[0].rows,V[0].cols,V[0].type(),Scalar::all(0.0));
		for(int i = 0; i < nWeightImage; i++)
		{
			sum_vV = sum_vV + V[i] * v[i];
		}

		//sprintf(varname,"sum_vV.png");
		//imwrite(varname,sum_vV);

		// compute the blended result
		Mat finalBlend(edgeresponse_db_v[0].rows,edgeresponse_db_v[0].cols,edgeresponse_db_v[0].type(),Scalar::all(0.0));

		for(int i = 0; i < nWeightImage; i++)
		{
			WeightImage[i] = alpha * v[i] * V[i] / sum_vV;

			finalBlend = finalBlend + WeightImage[i].mul(edgeresponse_db_v[i]);
			/*
			char filename[256];
			sprintf(filename,"WeightImage_%d.png",i);
			imwrite(filename,WeightImage[i] * 100);
			//*/

			// commented by lvp 2012-5-16
			//finalBlend = finalBlend + WeightImage[i].mul(edgeresponse_db_v[i]);
		}

		int rows, cols;
		for(int i = 0; i < _REDUCE_RESOLUTION_; i++)
		{
			rows = finalBlend.rows;
			cols = finalBlend.cols;

			pyrUp(finalBlend,finalBlend,cv::Size(rows*2,cols*2));
		}

		// show the final result
		double minVal,maxVal,range;
		minMaxLoc(finalBlend,&minVal,&maxVal);
		range = maxVal - minVal;
		for(int i = 0; i < finalBlend.rows; i++)
		{
			for(int j = 0; j < finalBlend.cols; j++)
			{
				int gray = (maxVal - finalBlend.at<double>(i,j))/range * 255; 
				QColor color(gray,gray,gray);
				backImage.setPixel(j,i,color.rgba());
				//finalBlend.at<double>(i,j) = (maxVal - finalBlend.at<double>(i,j))/range * 255; 
			}
		}
		//*/

		/* commented by lvp 2012-5-8
		// store the result into the XML file
		FileStorage fs("blend_result.xml",FileStorage::WRITE);
		if(!fs.isOpened())
		{
		return;
		}
		fs<<"blend_result"<<finalBlend;
		//*/

		// commented by lvp 12-5-23
		//imwrite("blend_result.png",finalBlend);		
		//backImage = QImage("blend_result.png","png");
		//backImage.invertPixels();

		emit loadImages(m_sDBDir.toAscii(),index,color,imgindex.size(),false);

		//emit loadImages(m_sDBDir.toAscii(),imgindex,color_v,transform);
		qDebug("Time elapsed for image blend: %d ms",t.elapsed());
	}
	//*
	for(int i = 0; i < imgindex.size() && i < 10; i++)
		qDebug("The %d th nearest neighbor is %d, the dist is %lf",i,imgindex[i],dist_sorted[i]);
	//*/
	delete []index;
	delete []color;

	resetPatchChangedFlag();
}

/*
	compute the new candidate sets
*/
void RenderArea::updateCandidateSets2(const QImage& queryImage, const QImage& currentSketchImage)
{
	/**************************************************************************/
	/*	put the descriptor of each limb into m_vSketchHist in order to compute 
	/*  the image index which is more close to the current sketch line															
	/**************************************************************************/
	imageprocess.process_sketch_image(qimage2mat(currentSketchImage),m_vSketchHist,ARP);

	/**************************************************************************/
	/*	compute the image index which is more close to all the sketch has been			
	/*	drawn														
	/**************************************************************************/
	imgindex.clear();

	// the distance between query image and database image
	vector<double> dist_sorted;

	// extract the feature from the whole sketch image
	vector<Mat> bin_sketch;

	// compute the elapsed time for different parts of this function
	QTime t;
	t.start();

	bin_sketch.clear();
	imageprocess.process_sketch_image(qimage2mat(queryImage),bin_sketch,ARP);
	//vector<vector<Vec4i>> sketch_lines;
	//imageprocess.process_sketch_image(qimage2mat(image),bin_sketch,sketch_lines,HOUGH);
	qDebug("Time elapsed for batch process user sketch: %d ms", t.elapsed());

	//* continuous comparison
	float weight[7] = {2.0,0.3,1.5,0.3,1.5,2.0,2.0};
	t.restart();
	imageprocess.patchdistmeasure(bin_sketch,imgindex,dist_sorted,m_iSketchOrder,weight[m_iSketchOrder]);	// only one resolution
	//imageprocess.patchdistmeasure(bin_sketch,imgindex,dist_sorted);	// only one resolution
	//imageprocess.patchdistmeasure(sketch_lines,imgindex,dist_sorted);		// multi-resolution
	//imageprocess.patchdistmeasureByANN(bin_sketch,imgindex,dist_sorted);	// ANN
	//imageprocess.patchdistmeasureByLSH(bin_sketch,imgindex,dist_sorted);	// LSH
	qDebug("Time elapsed for patch distance measure: %d ms",t.elapsed());

	// filter the data according to the database selected
	QWidget* direct_parent = parentWidget();
	static_cast<MainWindow*>(direct_parent->parentWidget())->FilterImageIndex(imgindex);

	m_dDist = dist_sorted[0];

	if(imgindex.size() > m_iDisplayNearestNum)
		imgindex.erase(imgindex.begin() + m_iDisplayNearestNum,imgindex.end());

	// assign different colors to selected images
	int* index = new int[imgindex.size()];
	double* color = new double[imgindex.size()];
	vector<double> color_v;
	if(imgindex.size() == 1)
	{
		index[0] = imgindex[0];
		color[0] = 0;
		color_v.push_back(0);
	}
	else
	{
		for(int i = 0; i < imgindex.size(); i++)
		{
			index[i] = imgindex[i];
			color[i] = (double)i/(imgindex.size()-1);
			color_v.push_back((double)i/(imgindex.size()-1));
		}
	}

	/************************************************************************/
	/* right nearest neighbor display                                       */
	/************************************************************************/
	if(imgindex.size() > 0)
	{
		emit loadImages(m_sDBDir.toAscii(),index,color,imgindex.size(),false);
	}

	for(int i = 0; i < imgindex.size() && i < 10; i++)
		qDebug("The %d th nearest neighbor is %d, the dist is %lf",i,imgindex[i],dist_sorted[i]);

	delete []index;
	delete []color;
}

void RenderArea::mouseReleaseEvent(QMouseEvent * event)
{
	m_bInitMousePosition = true;
	
	if(SKETCH == behaviorMode)
	{
		if(event->button() == Qt::LeftButton)
		{
			point = event->pos();
			m_vPoints.push_back(point);
			
			// record elapsed time for each sketch
			m_vElapsedTimeForEachSketch.push_back(m_TimerSketch.elapsed());

			if(m_vPoints.size() < 5)
				return;

			paint(image,pen);
			paint(styleUsrDrawingImage,stylePen);

			bContentChanged = true;

			//return;

			// The user is sketching a character pose
			if(m_bSketchPose)
			{
				// indicate the sketch has began
				m_bStartUp = false;

				// The database is not loaded
				if(m_iViewNum == 0)
				{
					qDebug("no database is loaded!");
					return;
				}

				// only contain the current sketch line
				QImage current_sketch_image = QImage(IMAGE_SIZE,IMAGE_SIZE,QImage::Format_ARGB32);  //32bit color 
				backColor = qRgba(255,255,255,0);													//initial background color is white, transparent
				current_sketch_image.fill(backColor);
				paint(current_sketch_image,pen);

				// update the candidate image index
				//updateCandidateSets2(image, current_sketch_image);
				updateCandidateSets2(current_sketch_image, current_sketch_image);

				m_iSketchOrder++;

				if(m_iSketchOrder > MAX_SKETCH_NUM)
					m_iSketchOrder = -1;

				// record this sketch
				m_vHistoryPoints.push_back(m_vPoints);
			}
			else // The user is sketching animation
			{
				vector<QPoint> feature;
				extractMotionFeature(m_vPoints,feature);

				// compare the extracted feature with database
				m_TrajectoryFeature[m_iShowTrajectoryIndex].assign(feature.begin(),feature.end());

				// update candidate motion clips
				updateCandidateAnimationSets(m_TrajectoryFeature[m_iShowTrajectoryIndex]);
			}
		}
	}
	else if(VIEW == behaviorMode)
	{
		if(event->button() & Qt::LeftButton)
			m_bLeftButtonDown = false;
		else if(event->button() & Qt::RightButton)
			m_bRightButtonDown = false;
		else if(event->button() & Qt::MidButton)
			m_bMidButtonDown = false;

		m_iAxisIdx = 0;
	}
}

void RenderArea::mouseMoveEvent(QMouseEvent * event)
{
	int x,y;
	x = event->x();
	y = event->y();

	// Calculate differentials of mouse position.
	static int oldx, oldy;
	if (m_bInitMousePosition) {
		oldx = x; oldy = y;
		m_bInitMousePosition = false;
	}
	int diffx = x -	oldx;
	int diffy = y -	oldy;
	oldx = x; oldy = y;

	if(VIEW == behaviorMode) // deal with the 3d space event, such as rotation, pan and zoom
	{
		if(m_bLeftButtonDown)
		{
			if(m_iAxisIdx == 3)			// rotate by the x axis
			{
				m_Camera.m_rotations[0] -= -360.0 * diffy/ height();

				if(m_Camera.m_rotations[0] < 0.0)
					m_Camera.m_rotations[0] += 360.0;
				else if(m_Camera.m_rotations[0] > 360.0)
					m_Camera.m_rotations[0] -= 360.0;

			}
			else if(m_iAxisIdx == 2)	// rotate by the z axis
			{
				m_Camera.m_rotations[2] -= 360.0 * diffy/ height();
				if(m_Camera.m_rotations[2] < 0.0)
					m_Camera.m_rotations[2] += 360.0;
				else if(m_Camera.m_rotations[2] > 360.0)
					m_Camera.m_rotations[2] -= 360.0;
			}
			else if(m_iAxisIdx == 1)	// rotate by the y axis
			{
				m_Camera.m_rotations[1] -= -360.0 * diffx/ width();
				if(m_Camera.m_rotations[1] < 0.0)
					m_Camera.m_rotations[1] += 360.0;
				else if(m_Camera.m_rotations[1] > 360.0)
					m_Camera.m_rotations[1] -= 360.0;
			}
		}

		update();
	}
	else if(SKETCH == behaviorMode)
	{
		if(event->buttons() & Qt::LeftButton)
		{
			point = event->pos();
			m_vPoints.push_back(point);
			if(shape == LINE)   //if draw line, direct draw it on image
			{
				paint(image,pen);
				paint(styleUsrDrawingImage,stylePen);
			}
			else	//if draw other shapes, draw it on temp image
			{
				tempImage = image;    //use last image to fill the temp image
				paint(tempImage,pen);
			}

			int x = point.x();
			int y = point.y();
		}
	}
}

void RenderArea::paint(QImage &theImage,const QPen& pen)
{
	QPainter pp(&theImage);
	pp.setPen(pen);

	int x,y;
	switch(shape)
	{
	case LINE:
		if(m_DrawMode == PEN_MODE)
		{
			for(int i = 1; i < m_vPoints.size(); i++)
				pp.drawLine(m_vPoints[i-1],m_vPoints[i]);
		}
		else if(m_DrawMode == ERASER_MODE)
		{
			QImage bkImg,subImg;
			if(m_b3DPose)
			{
				bkImg = threedpose;
			}
			else
			{
				bkImg = gridImage;
			}

			for(int i = 0; i < m_vPoints.size(); i++)
			{
				int x = m_vPoints[i].x() - 5 > 0 ? m_vPoints[i].x() - 5 : 0;
				int y = m_vPoints[i].y() - 5 > 0 ? m_vPoints[i].y() - 5 : 0;
				int w = x + 10 > threedpose.size().width() ? threedpose.size().width() - x : 10;
				int h = y + 10 > threedpose.size().height() ? threedpose.size().height() - y : 10;
				subImg = bkImg.copy(x,y,w,h);
				pp.eraseRect(x,y,w,h);
				pp.drawImage(x,y,subImg);
			}
		}
		break;
	}

	update();
}

bool RenderArea::isContentChanged()
{
	return bContentChanged;
}

void RenderArea::setContentChanged(bool changed)
{
	bContentChanged = changed;
}

void RenderArea::resetContent()
{
	image = QImage(IMAGE_SIZE,IMAGE_SIZE,QImage::Format_ARGB32);
	image.fill(backColor);
	backImage = styleUsrDrawingImage = image;

	// set the first vote flag to true
	imageprocess.setFirstVoteStatus(true);

	// clear the multi 3d lines
	m_vMulti3DLines.clear();

	m_vPoints.clear();

	// clear 3d poses and set to default
	imgindex.clear();
	m_pSkeleton->setBasePosture();

	if(!m_sDBDir.isEmpty())
		setDefaultStatus(true);

	m_iSketchOrder = 0;
	vSketchedBones.clear();

	m_bRefined = false;

	repaint();
}

void RenderArea::resetPatchChangedFlag()
{

}

QImage& RenderArea::getCurrentCanvas()
{
	// return the front image
	return image;
}

QImage& RenderArea::getFinalCanvas()
{
	// return the final composition image
	QPainter painter(&resultImage);
	painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
	painter.drawImage(0, 0, backImage);
	painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
	painter.drawImage(0, 0, image);
	painter.end(); 

	return resultImage;
}
void RenderArea::setCurrentCanvas(QImage img)
{
	//image = img;
	backImage =  img.scaled(backImage.size());
	//image.fill(backColor);

	//updateCandidateSets2(backImage);
	repaint();

	/*
	Mat gray_image;
	Mat detected_edges;

	// convert color image to gray image
	cvtColor(qimage2mat(backImage),gray_image,CV_RGB2GRAY);

	// reduce the noise with a kernel 3x3
	blur(gray_image,detected_edges,Size(3,3));

	// edge detection
	Canny(detected_edges,detected_edges,50,200,3);

	// hough transform
	vector<Vec4i> lines;
	HoughLinesP( detected_edges, lines, 1, CV_PI/180, 15,10,5);

	cvtColor(detected_edges,detected_edges,CV_GRAY2RGB);

	imwrite("detected_edges.png",detected_edges);
	QImage tmp;
	tmp.load("detected_edges.png","png");
	QPainter painter(&tmp);
	for(int i = 0; i < lines.size(); i++)
	{
	Vec4i l = lines[i];
	painter.setPen(QColor(0,0,255));
	painter.drawLine(l[0],l[1],l[2],l[3]);
	}
	painter.end();

	tmp.save("tmp.png","png");

	//*/
}

void RenderArea::patchChanged2(int x, int y)
{
	x = int(x/60.0);
	y = int(y/60.0);

	m_isPatchChanged[y][x] = true;
}

// ARGB--->BGR
Mat RenderArea::qimage2mat(const QImage& qimage)
{
	cv::Mat mat = cv::Mat(qimage.height(), qimage.width(), CV_8UC4, (uchar*)qimage.bits(), qimage.bytesPerLine()); 
	cv::Mat mat2 = cv::Mat(mat.rows, mat.cols, CV_8UC3 ); 
	int from_to[] = { 0,0,  1,1, 2,2 };
	cv::mixChannels( &mat, 1, &mat2, 1, from_to, 3 ); 
	return mat2; 
}

//BGR---->ARGB
QImage RenderArea::mat2qimage(const Mat& mat)
{
	vector<Mat> mv;
	mv.push_back(mat);
	mv.push_back(mat);
	mv.push_back(mat);
	Mat rgb;
	merge(mv,rgb);

	return QImage((const unsigned char*)(rgb.data), rgb.cols, rgb.rows, QImage::Format_RGB888); 
}

// set the database dir
void RenderArea::setDBDir(QString dir)
{
	m_sDBDir = dir;

	m_iCurrenView = m_iViewNum;

	if(m_pViewArray[m_iCurrenView])
	{	
		delete m_pViewArray[m_iCurrenView];
		m_pViewArray[m_iCurrenView] = NULL;
	}
	m_pViewArray[m_iCurrenView] = new CSAView();

	// load the inverted file
	QString invfilename = m_sDBDir.section('\\',-1);
	invfilename = m_sDBDir + "\\" + invfilename + ".inv";
	//m_pViewArray[m_iCurrenView]->loadInvFile(invfilename.toAscii());

	// load the histogram file
	QString histfilename = m_sDBDir.section('\\',-1);
	histfilename = m_sDBDir + "\\" + histfilename + "_hist.xml";
	m_pViewArray[m_iCurrenView]->load_hist_from(histfilename.toAscii());

	// load the magnitude file
	QString magnitudefilename = m_sDBDir.section('\\',-1);
	magnitudefilename = m_sDBDir + "\\" + magnitudefilename + "_m.xml";
	//m_pViewArray[m_iCurrenView]->load_edgemag_from(magnitudefilename.toAscii());

	// load the orientation file
	QString orienfilename = m_sDBDir.section('\\',-1);
	orienfilename = m_sDBDir + "\\" + orienfilename + "_o.xml";
	//m_pViewArray[m_iCurrenView]->load_edgeorien_from(orienfilename.toAscii());

	// load the lines' info from file
	QString linesfilename = m_sDBDir.section('\\',-1);
	linesfilename = m_sDBDir + "\\" + linesfilename + ".txt";
	//m_pViewArray[m_iCurrenView]->load_lines_from(linesfilename.toAscii());

	/*
	// load the decomposed magnitude file
	QString decmagnitudefilename = m_sDBDir.section('\\',-1);
	decmagnitudefilename = m_sDBDir + "\\" + decmagnitudefilename + "_dec.xml";
	imageprocess.load_dec_edgemag_from(decmagnitudefilename.toAscii());
	//*/

	// load the mocap data
	QString mocapfileName = m_sDBDir;
	//mocapfileName += "\\"+mocapfileName.section('\\',-1) + ".bvh";
	//loadMocapData(mocapfileName.toAscii(),"bvh");

	mocapfileName += "\\"+mocapfileName.section('\\',-1) + ".amc";
	int start,end;
	loadMocapData(mocapfileName.toAscii(),"amc",&start,&end);

	// load the camera parameters
	CSACamera* pCamera = new CSACamera();
	QString cameraConfigFileName = m_sDBDir;
	cameraConfigFileName += "\\"+cameraConfigFileName.section('\\',-1) + ".cfg";
	string file = cameraConfigFileName.toAscii();
	try
	{
		ConfigFile cfg(file);

		cfg.readInto(*pCamera,"Camera");
		m_pViewArray[m_iCurrenView]->m_pCamera = pCamera;

		// set the current camera
		m_Camera = *pCamera;
	}
	catch (ConfigFile::file_not_found e)
	{
		m_Camera = m_DefaultCamera;
	}

	// set this view the view processed by imageprocess
	imageprocess.setView(m_pViewArray[m_iCurrenView]);

	// obtain the camera position in model space
	float pos[3];
	m_pViewArray[m_iCurrenView]->m_pCamera->computePositionInModelSpace(pos);
	Vector3d vec3d(pos[0],pos[1],pos[2]);
	vec3d.toUnit();
	vec3d.multiplyBy(5.5);
	m_vVec3d[m_iCurrenView] = vec3d;

	m_iViewNum = m_iCurrenView + 1;

	// immediately repainting the widget
	repaint();

	setDefaultStatus(true);
}

// set the database dir and update the database hierarchy using the actual data
void RenderArea::setDBDir(const vector<QString>& baseDir, vector<DatabaseUnit>& databaseHierarchy)
{
	// load the trajectory of the joint movement
	loadMotionClipsFromFile("test.xml");

	for(int i = 0; i < m_vMotionClip.size(); i++)
		motion_clip_index.push_back(i);

	int start,end;
	//loadMocapData("D:\\Siggraph2013\\v9\\SketchAnimation\\Resources\\lvpei.standingstraightforward_a.amc","amc",&start,&end);
	//loadMocapData("Resources\\boxing\\boxing_13_17_Take_001.amc","amc",&start,&end);
	loadMocapData(motion_clip_filename[0],"amc",&start,&end);

	//return;

	m_iCurrenView = m_iViewNum;
	if(m_pViewArray[m_iCurrenView])
	{	
		delete m_pViewArray[m_iCurrenView];
		m_pViewArray[m_iCurrenView] = NULL;
	}
	m_pViewArray[m_iCurrenView] = new CSAView();

	// read the histogram & mocap data
	for(int i = 0; i < baseDir.size(); i++)
	{
		m_sDBDir = baseDir[i];

		// load the histogram file
		QString histfilename = m_sDBDir.section('/',-1);
		histfilename = m_sDBDir + "/" + histfilename + "_hist.xml";
		m_pViewArray[m_iCurrenView]->load_hist_from(histfilename.toAscii());

		// first limb
		histfilename = m_sDBDir + "/";
		histfilename += "lowerback_upperback_thorax_lowerneck_hist.xml";
		m_pViewArray[m_iCurrenView]->load_limb_hist_from(histfilename.toAscii(),0);

		// second limb
		histfilename = m_sDBDir + "/";
		histfilename += "lclavicle_hist.xml";
		m_pViewArray[m_iCurrenView]->load_limb_hist_from(histfilename.toAscii(),1);

		// third limb
		histfilename = m_sDBDir + "/";
		histfilename += "lhumerus_lradius_hist.xml";
		m_pViewArray[m_iCurrenView]->load_limb_hist_from(histfilename.toAscii(),2);

		// fourth limb
		histfilename = m_sDBDir + "/";
		histfilename += "rclavicle_hist.xml";
		m_pViewArray[m_iCurrenView]->load_limb_hist_from(histfilename.toAscii(),3);

		// fifth limb
		histfilename = m_sDBDir + "/";
		histfilename += "rhumerus_rradius_hist.xml";
		m_pViewArray[m_iCurrenView]->load_limb_hist_from(histfilename.toAscii(),4);

		// sixth limb
		histfilename = m_sDBDir + "/";
		histfilename += "lhipjoint_lfemur_ltibia_hist.xml";
		m_pViewArray[m_iCurrenView]->load_limb_hist_from(histfilename.toAscii(),5);

		// seventh limb
		histfilename = m_sDBDir + "/";
		histfilename += "rhipjoint_rfemur_rtibia_hist.xml";
		m_pViewArray[m_iCurrenView]->load_limb_hist_from(histfilename.toAscii(),6);

		// load the mocap data
		QString mocapfileName = m_sDBDir;
		mocapfileName += "/"+m_sDBDir.section('/',-1) + ".amc";
		
		int start,end;
		//loadMocapData(mocapfileName.toAscii(),"amc",&start,&end);

		// load the camera parameters, at present, the camera info only need to be read once
		static int read_only_once = 0;
		if(read_only_once == 0)
		{
			CSACamera* pCamera = new CSACamera();
			QString cameraConfigFileName = m_sDBDir;
			cameraConfigFileName += "/"+cameraConfigFileName.section('/',-1) + ".cfg";
			string file = cameraConfigFileName.toAscii();
			try
			{
				ConfigFile cfg(file);

				cfg.readInto(*pCamera,"Camera");
				m_pViewArray[m_iCurrenView]->m_pCamera = pCamera;

				// set the current camera
				m_Camera = *pCamera;
			}
			catch (ConfigFile::file_not_found e)
			{
				m_Camera = m_DefaultCamera;
			}
		}
		else
			read_only_once = 1;

		// update the database hierarchy
		for(int j = 0; j < databaseHierarchy.size(); j++)
		{
			DatabaseUnit& dbu = databaseHierarchy[j];

			if(dbu.type == m_sDBDir.toStdString())
			{
				dbu.start = start;
				dbu.end	  = end;
			}

			updateDabaseHierarchy(dbu,databaseHierarchy);
		}
	}

	// set this view the view processed by imageprocess
	imageprocess.setView(m_pViewArray[m_iCurrenView]);

	// obtain the camera position in model space
	float pos[3];
	m_pViewArray[m_iCurrenView]->m_pCamera->computePositionInModelSpace(pos);
	Vector3d vec3d(pos[0],pos[1],pos[2]);
	vec3d.toUnit();
	vec3d.multiplyBy(5.5);
	m_vVec3d[m_iCurrenView] = vec3d;

	m_iViewNum = m_iCurrenView + 1;

	// immediately repainting the widget
	repaint();

	setDefaultStatus(true);
}

/*
dynamically show the shadow
*/
void RenderArea::dynamicShadowHint()
{
	updateCandidateSets();
}

/*
start the timer
*/
void RenderArea::startTimer()
{
	m_pTimer->start(66.67);
}

/*
stop the timer
*/
void RenderArea::stopTimer()
{
	m_pTimer->stop();
}

/*
set the drawing mode
*/
void RenderArea::setDrawingMode(DrawMode mode)
{
	m_DrawMode = mode;
	if(m_DrawMode == PEN_MODE)
	{	
		// set the pen cursor
		m_cursor = QCursor(m_penPixmap,0,30);

		// set the pen color to draw
		pen.setColor(QColor(0,0,0));
		pen.setWidth(1);
		stylePen.setWidth(5);
		stylePen.setColor(QColor(255,0,0));
	}
	else
	{	// set the eraser cursor
		m_cursor = QCursor(m_eraserPixmap,0,30);

		// set pen color to erase previous drawing, larger width to erase more
		pen.setWidth(10);
		pen.setColor(QColor(255,255,255));	// use the white color to cover the black line
		stylePen.setWidth(10);
		stylePen.setColor(QColor(255,255,255,255));
		//stylePen.setColor(backColor);
	}

	setCursor(m_cursor);
}

void RenderArea::initializeGL()
{
	qDebug("OpenGL State:\nBuffer size is %i x %i\nThe color depth is %i",width(),height(),context()->device()->depth());

	QGLFormat fmt = format();

	if(fmt.rgba())
	{
		qDebug("RGBA Mode is set.");
	}
	else
	{
		qDebug("RGBA Mode is not set.");
	}
	if(fmt.alpha())
	{
		qDebug("Alpha Channel on.");
	}
	else
	{
		qDebug("Alpha Channel off.");
	}
	if(fmt.doubleBuffer())
	{
		qDebug("double buffer on.");
	}
	format().setAlpha(true);
	glEnable(GL_ALPHA);

	// enable multi-sample to anti-aliasing
	GLint smBuf=0;
	GLint sm=0;
	glGetIntegerv(GL_SAMPLE_BUFFERS,&smBuf);
	glGetIntegerv(GL_SAMPLES,&sm);
	if(smBuf == 1 && sm > 1){
		glEnable(GL_MULTISAMPLE);
	}
	else{
		qDebug("current GPU device not support for multisamples\n");
	}

	// load the ground texture image
	bool loadedsucess = groundImage.load(".\\Resources\\texture\\plane.bmp","bmp");
	if(loadedsucess)
	{
		glGenTextures(1,&ground_texture[0]);

		// Typical Texture Generation Using Data From The Bitmap
		glBindTexture(GL_TEXTURE_2D, ground_texture[0]);

		QImage tmp;
		tmp = QGLWidget::convertToGLFormat(groundImage);

		// Generate The Texture
		glTexImage2D(GL_TEXTURE_2D, 0, 3, tmp.width(), tmp.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tmp.bits());

		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);// Linear Filtering
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);// Linear Filtering
	}

	glShadeModel(GL_SMOOTH);                                       // Enable Smooth Shading
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);

	// enable the light & material
	GLfloat ambient[] = {0.5,0.5,0.5,1.0};
	GLfloat diffuse[] = {0.8,0.8, 0.8,1.0};
	GLfloat specular[] = {1.0,1.0,1.0,1.0};
	GLfloat emission[] = {.0,.0,.0,1.0};

	glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
	glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
	glLightfv(GL_LIGHT0,GL_EMISSION,emission);

	glEnable(GL_DEPTH_TEST);                                       // Enables Depth Testing
	glDepthFunc(GL_LEQUAL);                                        // The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);             // Really Nice Perspective Calculations

	// load the skeleton structure
	//m_pSkeleton = new Skeleton(".\\Resources\\skeleton.asf",SKELETON_SCALE);
	m_pSkeleton = new Skeleton(".\\Resources\\boxing_13.asf",SKELETON_SCALE);
	//m_pSkeleton = new Skeleton(".\\Resources\\walk_02.asf",SKELETON_SCALE);
	if(!m_pSkeleton->isLoaded())
		qDebug()<<"skeleton is not loaded";
	else
		qDebug()<<"skeleton is loaded";

	// set the display list for skeleton
	m_pSkeleton->set_display_list();

	m_pSkeleton->setBasePosture();
}

void RenderArea::resizeGL(int width, int height)
{
	setupViewport(width,height);
}

void RenderArea::setupViewport(int width, int height)
{
	glViewport(0, 0, width, height);

	glGetIntegerv(GL_VIEWPORT,m_vViewPort);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(m_Camera.m_fov, 1, 0.1, 1000);

	glGetDoublev(GL_PROJECTION_MATRIX,m_vProjectionMatrix);
}

/*
	load mocap data
*/
void RenderArea::loadMocapData(const char* filename,const char* type, int* p_start, int* p_end)
{
	if(strcmp(type,"bvh") == 0)
	{
		// delete the existing BVH object
		if(m_pBvh)
		{
			delete m_pBvh;
			m_pBvh = NULL;
		}
		qDebug("loading mocap data file %s",filename);
		m_pBvh = new BVH(filename);
		if(m_pBvh->IsLoadSuccess())
			qDebug("mocap data loaded");
		else
		{	
			qDebug("mocap data not loaded");
			delete m_pBvh;
			m_pBvh = NULL;
		}
	}

	if(strcmp(type,"amc") == 0)
	{
		qDebug("loading mocap data file %s",filename);
		Motion* m = new Motion(filename,SKELETON_SCALE,m_pSkeleton);
		// error process should be involved here
		qDebug("mocap data loaded");

		if(!m_pMotion)
		{
			(*p_start) = 0;
			(*p_end) = m->m_NumFrames - 1;

			m_pMotion = m;
		}
		else
		{	
			delete m_pMotion;
			m_pMotion = NULL;

			if(p_start)
				(*p_start) = 0;
			if(p_end)
				(*p_end) = m->m_NumFrames - 1;

			m_pMotion = m;

			/*
			// concatenate all the motion together
			(*p_start) = m_pMotion->m_NumFrames;

			// new posture number
			int newposturenum = m_pMotion->m_NumFrames + m->m_NumFrames;

			// allocate new posture space
			Posture* posture = new Posture[newposturenum];

			if(posture == NULL)
				return;

			// copy the original posture data into new array
			for(int i = 0; i < m_pMotion->m_NumFrames; i++)
				posture[i] = m_pMotion->m_pPostures[i];

			// delete the original posture space
			delete []m_pMotion->m_pPostures;
			m_pMotion->m_pPostures = NULL;

			// copy the new posture data into new array
			for(int i = 0; i < m->m_NumFrames; i++)
				posture[m_pMotion->m_NumFrames + i] = m->m_pPostures[i];

			m_pMotion->m_pPostures = posture;
			m_pMotion->m_NumFrames = newposturenum;

			(*p_end) = newposturenum - 1;
			//*/
		}

		vector<Posture> vPosture;

		for(int i = 0; i < m_pMotion->m_NumFrames; i++)
			vPosture.push_back(*m_pMotion->GetPosture(i));

		emit sendNewMotion(vPosture);
	}
}

/*
	set the flag whether to show 3d pose
*/
void RenderArea::setShow3DPose(bool show)
{
	m_b3DPose = show;
	update();
}

/*
reconstruct 3d line
*/
void RenderArea::from2Dto3DLine(float* depthdata)
{
	GLdouble pos3D_x, pos3D_y, pos3D_z;
	GLfloat win_x, win_y, win_z;

	// arrays to hold matrix information
	GLdouble model_view[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, model_view);

	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	// get 3D coordinates based on window coordinates
	m_v3DPoints.clear();

	for(int i = 0; i < m_vPoints.size(); i++)
	{
		win_x = (GLfloat)m_vPoints[i].x();
		win_y = (GLfloat)viewport[3] - (GLfloat)m_vPoints[i].y();
		//glReadPixels(int(win_x), int(win_y), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &win_z);  
		win_z = depthdata[(int)win_y * 480 + (int)win_x];
		if(1.0 - win_z > 0.0001)
		{
			gluUnProject(win_x, win_y, win_z,model_view, projection, viewport, &pos3D_x, &pos3D_y, &pos3D_z);
			vector<double> pos3D;
			pos3D.push_back(pos3D_x);
			pos3D.push_back(pos3D_y);
			pos3D.push_back(pos3D_z);
			m_v3DPoints.push_back(pos3D);
		}
	}

	m_vMulti3DLines.push_back(m_v3DPoints);
}

/*
	visualize the 3d histogram
*/
void RenderArea::histVisualize(bool sketch,int imgidx, int patchidx)
{
	// compute the patch index
	int i = patchidx / _PATCH_NUM;
	int j = patchidx % _PATCH_NUM;

	// obtain the patch area
	int up = i * 60;
	int left = j * 60;

	QImage subarea;

	if(sketch)	// user's sketch
	{
		// convert to mat
		subarea = image.copy(left,up,60,60);
		for(int i = 0; i < 60; i++)
			for(int j = 0; j < 60; j++)
			{
				if(subarea.pixel(i,j) == backColor)
					subarea.setPixel(i,j,qRgb(255,255,255));
			}
			subarea.save("patch.png","png");

			return; 

			Mat mat = qimage2mat(subarea);
			// compute the patch descriptor(3d histogram)
			int sz[3] = {BIN_X,BIN_Y,BIN_THETA};
			Mat hist(3,sz,CV_32F,Scalar::all(0));
			Mat m_n,orientation;
			imageprocess.process_one_patch(mat,hist,m_n,orientation);

			double minVal = 0, maxVal = 20;
			// for each theta bin, allocate a qimage object
			QImage patch = QImage(BIN_X * BIN_THETA + 2 * BIN_THETA,BIN_Y,QImage::Format_ARGB32); 
			int ix = 0;
			backColor = qRgb(255,255,255);	
			patch.fill(backColor);

			// assign different color to the patch visualize image
			for(int k = 0; k < BIN_THETA; k++)
			{
				for(int i = 0; i < BIN_X; i++)
				{
					for(int j = 0; j < BIN_Y; j++)
					{
						ix = (hist.at<float>(i,j,k) - minVal) / (maxVal - minVal) * (m_vColormap.size() - 1);
						if(ix < 0)
							continue;
						QColor color(m_vColormap[ix][0] * 255,m_vColormap[ix][1] * 255,m_vColormap[ix][2] * 255);
						patch.setPixel(i + k * (BIN_Y + 2),j ,color.rgb());
					}
				}
			}
			patch.save("3DHist.png","png");
	}
	else		// database image
	{
		// convert to mat
		subarea = backImage.copy(left,up,60,60);

		subarea.save("patch.png","png");
	}
}

/*
	change the view
*/
void RenderArea::changeView(int index)
{
	assert(index < m_iViewNum);

	imageprocess.setView(m_pViewArray[index]);
	m_Camera = *(m_pViewArray[index]->m_pCamera);

	imageprocess.setViewChanged(true);

	repaint();
	//update();

	// whether user have drawn something on the canvas
	//if(m_vPoints.size() > 2)
	{
		// change the view with image blending
		/*
		memset((bool*)m_isPatchChanged,true,_PATCH_NUM * _PATCH_NUM);

		// obtain the new 3d line image and new 3d style line image
		repaint();

		image = threedline;
		image.save("threedline.png","png");
		styleUsrDrawingImage = threedstyleline;
		imageprocess.setFirstVoteStatus(true);
		// update new candidate images
		updateCandidateSets();
		update();
		//*/

		// change the view without image blending
		image.fill(backColor);
		styleUsrDrawingImage.fill(backColor);

		int* index = new int[imgindex.size()];
		double* color = new double[imgindex.size()];
		if(imgindex.size() == 1)
		{
			index[0] = imgindex[0];
			color[0] = 0;
		}
		else
		{
			for(int i = 0; i < imgindex.size(); i++)
			{
				index[i] = imgindex[i];
				color[i] = (double)i/(imgindex.size()-1);
			}
		}

		// show items in right graphics view 
		emit loadImages(m_sDBDir.toAscii(),index,color,imgindex.size(),true);

		/*
		// compute the value of V
		Mat finalBlend = imageprocess.getEdgeResponse(imgindex[0]) * _COEFFICIENT_;
		for(int i = 1; i < imgindex.size(); i++)
		{

		finalBlend = finalBlend + imageprocess.getEdgeResponse(imgindex[i]) * _COEFFICIENT_;
		}
		int rows, cols;
		for(int i = 0; i < _REDUCE_RESOLUTION_; i++)
		{
		rows = finalBlend.rows;
		cols = finalBlend.cols;

		pyrUp(finalBlend,finalBlend,cv::Size(rows*2,cols*2));
		}

		// show the final result
		double minVal,maxVal,range;
		minMaxLoc(finalBlend,&minVal,&maxVal);
		range = maxVal - minVal;
		for(int i = 0; i < finalBlend.rows; i++)
		{
		for(int j = 0; j < finalBlend.cols; j++)
		{
		int gray = (maxVal - finalBlend.at<double>(i,j))/range * 255; 
		QColor color(gray,gray,gray);
		backImage.setPixel(j,i,color.rgba());
		}
		}
		//*/
		delete []index;
		delete []color;
		//*/
	}
}

void RenderArea::drawGround()
{
	glBindTexture(GL_TEXTURE_2D, ground_texture[0]);                      // Select Our Texture
	float pos = 100;
	glBegin(GL_QUADS);
	glNormal3f(0.0,1.0,0.0);
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-pos, 0.0f,-pos);      // Bottom Left Of The Texture and Quad
	glTexCoord2f(10.0f, 0.0f); glVertex3f( pos, 0.0f,-pos);      // Bottom Right Of The Texture and Quad
	glTexCoord2f(10.0f, 10.0f); glVertex3f( pos, 0.0f, pos);      // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 10.0f); glVertex3f(-pos, 0.0f, pos);     // Top Left Of The Texture and Quad
	glEnd();
}

/*
draw the navigation ball
*/
void RenderArea::drawNavigationBall()
{
	// draw a transparent ball
	glColor4f(0.2,0.1,0.5,0.8);
	GLUquadricObj *qobj;
	qobj= gluNewQuadric();

	gluQuadricDrawStyle(qobj, GLU_FILL);
	gluQuadricNormals(qobj, GLU_SMOOTH);
	gluQuadricOrientation(qobj, GLU_OUTSIDE);

	GLdouble radius = 4.8; 
	GLdouble slices = 32.0; 
	GLdouble stack  = 32.0; 

	GLfloat no_mat[] = {0.0,0.0,0.0,1.0};
	GLfloat mat_diffuse[] = {0.3,0.3,0.3,1.0};
	GLfloat mat_specular[] = {1.0,1.0,1.0,1.0};
	GLfloat no_shininess[] = {0.0};
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,no_mat);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,no_mat);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS,no_shininess);
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diffuse);

	gluSphere(qobj, radius, slices, stack);


	// draw the circle in x-z plane, rotate by y-axis
	if(m_iAxisIdx == 1)
		qglColor(QColor(255,255,0));
	else
		qglColor(QColor(255,0,0));
	glPushMatrix();
	glRotatef(90,1.0,0.0,0.0);
	glLoadName(1);
	drawCircle(0,0,5,50);
	glPopMatrix();

	// draw the circle in x-y plane, rotate by z-axis
	if(m_iAxisIdx == 2)
		qglColor(QColor(255,255,0));
	else
		qglColor(QColor(0,255,0));
	glPushMatrix();
	glLoadName(2);
	drawCircle(0,0,5,50);
	glPopMatrix();

	// draw the circle in y-z plane, rotate by x-axis
	if(m_iAxisIdx == 3)
		qglColor(QColor(255,255,0));
	else
		qglColor(QColor(0,0,255));
	glPushMatrix();
	glRotatef(90,0.0,1.0,0.0);
	glLoadName(3);
	drawCircle(0,0,5,50);
	glPopMatrix();

	glColor3f(0.0,1.0,1.0);
	// draw the view tag
	for(int i = 0; i < m_iViewNum; i++)
	{
		glPushMatrix();
		glLoadName(4+i);
		glTranslated(m_vVec3d[i].getX(),m_vVec3d[i].getY(),m_vVec3d[i].getZ());
		drawBox(0.5);
		glPopMatrix();
	}
}
/*
draw circle using an efficient way
*/
void RenderArea::drawCircle(float cx, float cy, float r, int num_segments)
{
	float theta = 2 * 3.1415926 / float(num_segments); 
	float tangetial_factor = tanf(theta);				//calculate the tangential factor 

	float radial_factor = cosf(theta);					//calculate the radial factor 

	float x = r;										//we start at angle = 0 

	float y = 0; 

	glBegin(GL_LINE_LOOP); 
	for(int ii = 0; ii < num_segments; ii++) 
	{ 
		glVertex3f(x + cx, y + cy,0);					//output vertex 

		//calculate the tangential vector 
		//remember, the radial vector is (x, y) 
		//to get the tangential vector we flip those coordinates and negate one of them 

		float tx = -y; 
		float ty = x; 

		//add the tangential vector 
		x += tx * tangetial_factor; 
		y += ty * tangetial_factor; 

		//correct using the radial factor 

		x *= radial_factor; 
		y *= radial_factor; 
	} 
	glEnd(); 
}

/*
draw the box
*/
void RenderArea::drawBox( float edgelength)
{
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
}


void RenderArea::drawShadow()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glDisable( GL_LIGHTING );
	glDisable(GL_TEXTURE_2D);		
	glDepthFunc( GL_LEQUAL );				
	glDepthMask( GL_FALSE );	
	glColorMask(false,false,false,false);
	glClear(GL_STENCIL_BUFFER_BIT );	
	glEnable( GL_STENCIL_TEST );				
	glStencilFunc(GL_ALWAYS, 1, 0xffffffff);
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

	// draw the ground
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslatef(0.0,-10.0,0.0);
	drawGround();
	glPopMatrix();

	//now we'll render the scene, using the projection matrix, and we'll only be
	//drawing where there is a 1 in the stencil buffer. This makes sure we're only drawing
	//on top of the ground, where it is visible
	glStencilFunc(GL_LESS, 0, 0xffffffff);
	glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);

	//// Draw the shadow
	glColorMask(true,true,true,true);
	glPolygonOffset(-2.0f, -2.0f);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	// the position for light
	float d[4] = {0.0f, 5.0f, 10.0f, 1.0f};

	// the normal for ground
	float n[4] ={0.0,1.0,0.0};

	glPushMatrix();
	double dot = n[0]*d[0]+n[1]*d[1]+n[2]*d[2];

	////this is the projection matrix
	//double mat[16] = {
	//	dot - n[0]*d[0] + 10.0,		-n[0] * d[1],			-n[0] * d[2],			-n[0],
	//	-n[1] * d[0],			dot - n[1] * d[1] + 10.0,	-n[1] * d[2],			-n[1],
	//	-n[2] * d[0],			- n[2] * d[1],			dot - n[2] * d[2] + 10.0,	-n[2],
	//	-10.0 * d[0],			-10.0 * d[1],				-10.0 * d[2],			 dot
	//};

	double mat[16] = {
		dot - n[0]*d[0],		-n[0] * d[1],			-n[0] * d[2],			0,
		-n[1] * d[0],			dot - n[1] * d[1],		-n[1] * d[2],			0,
		-n[2] * d[0],			- n[2] * d[1],			dot - n[2] * d[2],		0,
		-10.0 * d[0],			-10.0 * d[1],				-10.0 * d[2],	  dot
	};

	glMultMatrixd(mat);

	static GLfloat bone_color[4] =	{0.0,0.0,0.0,0.8};
	static GLfloat joint_color[4] = {0.0,0.0,0.0,0.8};
	static GLfloat bone_color2[4] = {0,0,0,0.8};

	if(!m_bStartUp && m_pMotion)
	{
		Posture posture = *m_pMotion->GetPosture(imgindex[0]);

		// align the root to the origin
		posture.root_pos[0] = posture.bone_translation[0][0] = 0.0;
		posture.root_pos[1] = posture.bone_translation[0][1] = 0.0;
		posture.root_pos[2] = posture.bone_translation[0][2] = 0.0;

		m_pSkeleton->setPosture(posture);
		m_pSkeleton->RenderFigure(bone_color2,joint_color);
	}
	else
	{
		// draw the default character
		//qglColor(defaultColor);
		m_pSkeleton->RenderFigure(bone_color,joint_color);
	}

	glPopMatrix();

	glPopAttrib();
}

/*
construct default camera
*/
void RenderArea::constructDefaultCamera()
{
	m_DefaultCamera.m_camDistance = 0.0;

	m_DefaultCamera.m_camPos[0] =  0.0;
	m_DefaultCamera.m_camPos[1] =  10.0;
	m_DefaultCamera.m_camPos[2] =  10.0;

	m_DefaultCamera.m_center[0] = 0.0;
	m_DefaultCamera.m_center[1] = 0.0;
	m_DefaultCamera.m_center[2] = -10.0;

	m_DefaultCamera.m_up[0] = 0.0;
	m_DefaultCamera.m_up[1] = 1.0;
	m_DefaultCamera.m_up[2] = 0.0;

	m_DefaultCamera.m_fov = 45.0;

	m_DefaultCamera.m_pan[0] = 0.0;
	m_DefaultCamera.m_pan[1] = -8.0;

	m_DefaultCamera.m_rotations[0] = 0.0;
	m_DefaultCamera.m_rotations[1] = 0.0;
	m_DefaultCamera.m_rotations[2] = 0.0;

	m_DefaultCamera.m_target[0] = 0.0;
	m_DefaultCamera.m_target[1] = 10.0;
	m_DefaultCamera.m_target[2] = 15.0;
}

/*
	set the flag to indicate whether the program is in default status
*/
void RenderArea::setDefaultStatus(bool status)
{
	m_bStartUp = status;

	if(m_bStartUp)
	{
		int numposes = m_pViewArray[m_iCurrenView]->m_nDBNum;
		
		// recommend different poses for user selection, with filter		
		//*
		MainWindow* pMainWindow = NULL;
		QWidget* direct_parent = parentWidget();
		pMainWindow = static_cast<MainWindow*>(direct_parent->parentWidget());

		if(pMainWindow->GetSelectedUnits().size() != 0)
		{
			while (imgindex.size() < m_iDisplayNearestNum)
			{
				vector<int> idx;
				idx.push_back(genRandomInt(0,numposes-1));

				// filter the data according to the database selected
				QWidget* direct_parent = parentWidget();
				static_cast<MainWindow*>(direct_parent->parentWidget())->FilterImageIndex(idx);

				if(!idx.empty())
					imgindex.push_back(idx[0]);
			}
		}
		//*/
		
		// recommend different poses for user selection, without filter
		/*
		for(int i = 0; i < m_iDisplayNearestNum; i++)
		{
			imgindex.push_back(genRandomInt(0,numposes-1));
		}
		//*/
		// assign different colors to selected images
		int* index = new int[imgindex.size()];
		double* color = new double[imgindex.size()];
		for(int i = 0; i < imgindex.size(); i++)
		{
			index[i] = imgindex[i];
			color[i] = 0;
		}

		emit loadImages(m_sDBDir.toAscii(),index,color,imgindex.size(),false);

		delete []index;
		delete []color;
	}

	// clear each sketch line feature
	m_vSketchHist.clear();

	repaint();
}


/*
process the selection event
*/
void RenderArea::processSelection(int hits,unsigned int* buffer)
{
	int stacknum = buffer[0];								//The stack number
	int choose = -1;										//The joint index
	GLuint depth = buffer[1];								// Store How Far Away It is
	GLuint* p = buffer;
	for(int loop = 0; loop < hits; loop++)
	{
		stacknum = *p;
		if(stacknum == 0)									// ==0,jump two variable
			p += 3;
		else
		{
			if(p[1] <= depth)
			{
				depth = p[1];								// Store How Far Away It is
				p += stacknum + 2;
				choose = *p;
				p++;
			}
		}
	}

	// the rotation axis is selected
	if(choose > 0 && choose < 4)
	{
		m_iAxisIdx = choose;

		switch(choose)
		{
		case 1:
			qDebug("rotate by y axis");		
			break;
		case 2:
			qDebug("rotate by z axis");
			break;
		case 3:
			qDebug("rotate by x axis");
			break;
		}

	}
	else if(choose >= 4 && choose < JOINT_NAME_STACK)
	{
		if(choose - 4 != m_iCurrenView)
		{
			m_iCurrenView = choose - 4;
			emit switchViewTo(m_iCurrenView);

			qDebug()<<"change to the "<< m_iCurrenView<<"th view";
		}
	}
	else if(choose >= JOINT_NAME_STACK)
	{
		int idx = choose - JOINT_NAME_STACK;

		if(m_pSkeleton)
		{
			Bone* pBone = m_pSkeleton->findBoneByIdx(m_pSkeleton->getRoot(),idx);
			pBone->m_bSelected = !pBone->m_bSelected;
			if(pBone->m_bSelected)
				qDebug()<<pBone->name<<" is selected";
		}
	}
}


/*
visualize the joint position using probability, 	
*/
void RenderArea::visualizeJointsInBoundingBox(float* boundingbox,int num, float* data, float* joint_color, float* interp_color)
{
	// There is no any data
	if(num == 0)
		return;
	boundingbox[0] = data[0] - 0.5;
	boundingbox[3] = data[0] + 0.5;

	boundingbox[1] = data[1] - 0.5;
	boundingbox[4] = data[1] + 0.5;

	boundingbox[2] = data[2] - 0.5;
	boundingbox[5] = data[2] + 0.5;

	// compute & display the probability volume
	float range_x = boundingbox[3] - boundingbox[0];
	float range_y = boundingbox[4] - boundingbox[1];
	float range_z = boundingbox[5] - boundingbox[2];

	vector<Vector3d> dataPoints;
	for(int i = 0; i < num; i++)
	{
		Vector3d tmp(data[i*3],data[i*3+1],data[i*3+2]);
		dataPoints.push_back(tmp);
	}

	int step = 10;
	float step_x = range_x / step;
	float step_y = range_y / step;
	float step_z = range_z / step;

	vector<Vector3d> Points;
	vector<float> Probability;
	Probability.assign(step*step*step,0.0);
	float total = 0.0;
	float min = 0.0;
	float max = 1.0;
	for(int i = 0; i < step; i++)
	{
		for(int j = 0; j < step; j++)
		{
			for(int k = 0; k < step; k++)
			{
				Vector3d p(boundingbox[0] + i * step_x,boundingbox[1] + j * step_y, boundingbox[2] + k * step_z);
				Points.push_back(p);

				Vector3d diff;
				for(int l = 0; l < num; l++)
				{
					diff = dataPoints[l] - p;
					double len = diff.length();

					Probability[i * step * step + j * step + k] += gsl_ran_gaussian_pdf(len,0.5);
				}

				total += Probability[i * step * step + j * step + k];

				if(Probability[i * step * step + j * step + k] < min)
					min = Probability[i * step * step + j * step + k];

				if(Probability[i * step * step + j * step + k] > max)
					max = Probability[i * step * step + j * step + k];
			}
		}
	}

	// normalize the probability
	for(int i = 0; i < Probability.size(); i++)
		Probability[i] /= total;
	min /= total;
	max /= total;

	// draw data samples
	//glPointSize(3.5);
	////glColor3f(joint_color[0],joint_color[1],joint_color[2]);
	//float dist = 0.0;
	//glBegin(GL_POINTS);
	//for(int i = 0; i < num; i++)
	//{
	//	//dist = sqrt((data[i*3]-data[0])*(data[i*3]-data[0])+(data[i*3+1]-data[1])*(data[i*3+1]-data[1])+(data[i*3+2]-data[2])*(data[i*3+2]-data[2]));
	//	//if(dist > 1.2)
	//	//	continue;

	//	glColor3f(joint_color[0]*(1-(float)i/num),joint_color[1]*(1-(float)i/num),joint_color[2]*(1-(float)i/num));
	//	glVertex3f(data[i*3],data[i*3+1],data[i*3+2]);
	//}
	//glEnd();

	// draw other interpolation values
	glPointSize(3.5);
	glBegin(GL_POINTS);
	for(int i = 0; i < Points.size(); i++)
	{
		//int idx = (max - Probability[i])/(max-min) * (m_vColormap.size()-1);
		//glColor3f(m_vColormap[idx][0],m_vColormap[idx][1],m_vColormap[idx][2]);
		float percent = (max - Probability[i])/(max-min);
		glColor3f(joint_color[0]*percent,joint_color[1]*percent,joint_color[2]*percent);
		glVertex3f(Points[i].getX(),Points[i].getY(),Points[i].getZ());
	}
	glEnd();
}

/*
	draw grid image
*/
void RenderArea::drawGridImage(QImage& image)
{
	// draw the grid image
	QPainter painter(&image);
	pen.setColor(QColor(202,202,202));
	painter.setPen(pen);

	/*
	// draw grid image(rectangle)
	for(int i = 0; i <= _PATCH_NUM; i++)
	{
	painter.drawLine(0,i*60,480,i*60);
	painter.drawLine(i*60,0,i*60,480);
	}
	//*/

	int center_x = image.width()/2;
	int center_y = image.height()/2;

	// resolution is set to 18 angular &  30 radius 
	int ANGULER_RES  = 18;
	int RADIUS_RES   = 30;  

	// draw circle grid image
	int N = ANGULER_RES;
	int M = RADIUS_RES;
	QPoint center(center_x,center_y);

	//int r_step = sqrt((float)(center_x*center_y+center_x*center_y))/M;
	int r_step = center_x / M;
	for(int i = 1; i <= M; i++)		// draw circle
	{
		int radius = i * r_step;
		painter.drawEllipse(center,radius,radius);
	}

	float a_step = 2 * CV_PI/ N;
	//int radius = sqrt((center_x+0.4)*(center_y+0.4)*2);
	int radius = center_x;
	for(int i = 0; i < N; i++)		// draw line
	{
		float theta = i * a_step;
		int x = radius * sin(theta);
		if(x < -center_x)
			x = -center_x;
		else if(x > center_x)
			x = center_x;
		int y = radius * cos(theta);
		if(y < -center_y)
			y = -center_y;
		else if(y > center_y)
			y = center_y;
		painter.drawLine(center.x(),center.y(),center.x() + y,center.y() + x);
	}

	painter.end();
}

/*
	visualize different body parts
*/
void RenderArea::visualizeBodyParts(int chainIdx)
{
	/*
	switch(chainIdx)
	{
	case 0:
	{
	m_vChainName.clear();
	m_vChainName.push_back("root");
	m_vChainName.push_back("lowerback");
	m_vChainName.push_back("upperback");
	m_vChainName.push_back("thorax");
	m_vChainName.push_back("lowerneck");
	}
	break;
	case 1:
	{
	m_vChainName.clear();
	m_vChainName.push_back("thorax");
	m_vChainName.push_back("lclavicle");
	m_vChainName.push_back("lhumerus");
	m_vChainName.push_back("lradius");
	}
	break;
	case 2:
	{
	m_vChainName.clear();
	m_vChainName.push_back("thorax");
	m_vChainName.push_back("rclavicle");
	m_vChainName.push_back("rhumerus");
	m_vChainName.push_back("rradius");
	}
	break;
	case 3:
	{
	m_vChainName.clear();
	m_vChainName.push_back("root");
	m_vChainName.push_back("lhipjoint");
	m_vChainName.push_back("lfemur");
	m_vChainName.push_back("ltibia");
	}
	break;
	case 4:
	{
	m_vChainName.clear();
	m_vChainName.push_back("root");
	m_vChainName.push_back("rhipjoint");
	m_vChainName.push_back("rfemur");
	m_vChainName.push_back("rtibia");
	}
	break;

	default:
	break;
	}
	//*/

	/************************************************************************/
	/* Seven sketches                                                       */
	/************************************************************************/
	/*
	m_vChainName.clear();
	vector<string> chain;

	chain.push_back("lowerback");
	chain.push_back("upperback");
	chain.push_back("thorax");
	chain.push_back("lowerneck");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("lclavicle");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("lhumerus");
	chain.push_back("lradius");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("rclavicle");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("rhumerus");
	chain.push_back("rradius");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("lhipjoint");
	chain.push_back("lfemur");
	chain.push_back("ltibia");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("rhipjoint");
	chain.push_back("rfemur");
	chain.push_back("rtibia");
	m_vChainName.push_back(chain);
	//*/
	m_vChainName.clear();
	vector<string> chain;

	chain.push_back("lowerback");
	chain.push_back("upperback");
	chain.push_back("thorax");
	chain.push_back("lowerneck");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("lclavicle");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("lhumerus");
	chain.push_back("lradius");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("rclavicle");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("rhumerus");
	chain.push_back("rradius");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("lfemurDummy");
	chain.push_back("lfemur");
	chain.push_back("ltibia");
	m_vChainName.push_back(chain);

	chain.clear();
	chain.push_back("rfemurDummy");
	chain.push_back("rfemur");
	chain.push_back("rtibia");
	m_vChainName.push_back(chain);
	
	update();
}

/*
	visualize sample points
*/
void RenderArea::visualizeSamplePoints(float* boundingbox, vector<Vector3d>& dataPoints)
{
	int num = dataPoints.size();

	// There is no any data
	if(num == 0)
		return;

	// compute & display the probability volume
	float range_x = boundingbox[3] - boundingbox[0];
	float range_y = boundingbox[4] - boundingbox[1];
	float range_z = boundingbox[5] - boundingbox[2];

	int step = 20;
	float step_x = range_x / step;
	float step_y = range_y / step;
	float step_z = range_z / step;

	vector<Vector3d> Points;
	vector<float> Probability;
	Probability.assign(step*step*step,0.0);
	float total = 0.0;
	float min = 0.0;
	float max = 1.0;
	for(int i = 0; i < step; i++)
	{
		for(int j = 0; j < step; j++)
		{
			for(int k = 0; k < step; k++)
			{
				Vector3d p(boundingbox[0] + i * step_x,boundingbox[1] + j * step_y, boundingbox[2] + k * step_z);
				Points.push_back(p);

				Vector3d diff;
				for(int l = 0; l < num; l++)
				{
					diff = dataPoints[l] - p;
					double len = diff.length();

					//Probability[i * step * step + j * step + k] += gsl_ran_gaussian_pdf(len,0.5);
					Probability[i * step * step + j * step + k] += gsl_ran_gaussian_pdf(len,0.8);
				}

				total += Probability[i * step * step + j * step + k];

				if(Probability[i * step * step + j * step + k] < min)
					min = Probability[i * step * step + j * step + k];

				if(Probability[i * step * step + j * step + k] > max)
					max = Probability[i * step * step + j * step + k];
			}
		}
	}

	// normalize the probability
	for(int i = 0; i < Probability.size(); i++)
		Probability[i] /= total;
	min /= total;
	max /= total;

	// draw data samples
	//glPointSize(2.5);
	//glColor3f(m_vColormap[m_vColormap.size()-1][0],m_vColormap[m_vColormap.size()-1][1],m_vColormap[m_vColormap.size()-1][2]);
	//glBegin(GL_POINTS);
	//for(int i = 0; i < num; i++)
	//	glVertex3f(dataPoints[i].getX(),dataPoints[i].getY(),dataPoints[i].getZ());
	//glEnd();

	// draw other interpolation values
	glPointSize(2.5);
	glBegin(GL_POINTS);
	for(int i = 0; i < Points.size(); i++)
	{
		//int idx = (max - Probability[i])/(max-min) * (m_vColormap.size()-1);
		int idx = (Probability[i] - min)/(max-min) * (m_vColormap.size()-1);
		glColor3f(m_vColormap[idx][0],m_vColormap[idx][1],m_vColormap[idx][2]);
		glVertex3f(Points[i].getX(),Points[i].getY(),Points[i].getZ());
	}
	glEnd();
}


/*
	using the child value to update the parent value
*/
void RenderArea::updateDabaseHierarchy(const DatabaseUnit& dbu,vector<DatabaseUnit>& databaseHierarchy)
{
	string parent_type = dbu.parent_type;

	for(int i = 0; i < databaseHierarchy.size(); i++)
	{
		DatabaseUnit& databaseunit = databaseHierarchy[i];
		if(parent_type == databaseunit.type)
		{
			if(databaseunit.start > dbu.start)
				databaseunit.start = dbu.start;
			if(databaseunit.end < dbu.end)
				databaseunit.end = dbu.end;

			updateDabaseHierarchy(databaseunit,databaseHierarchy);
		}
	}
}

/*
	refine the final result by sampling
*/
void RenderArea::refineResult()
{
	// only when the last sketch has been done, the result will be refined
	if(m_iSketchOrder != MAX_SKETCH_NUM && m_vSketchHist.size() != MAX_SKETCH_NUM)
		return;

	vector<int> whole_candidate_imageindex;

	int nn_1 = 0;
	int nn_2 = 50;
	
	static double max_sample_dist = 0.0;

	// select nn_1 nearest neighbors by each sketch
	for(int i = 0; i < m_vSketchHist.size(); i++)
	{
		vector<int>		current_candidate_imageindex;
		vector<double>	dist_sorted;
		vector<Mat>		current_hist;

		current_hist.push_back(m_vSketchHist[i]);
		
		imageprocess.setFirstVoteStatus(true);
		imageprocess.patchdistmeasure(current_hist,current_candidate_imageindex,dist_sorted);			// only one resolution

		if(i == 0)
			whole_candidate_imageindex.insert(whole_candidate_imageindex.end(),current_candidate_imageindex.begin(),current_candidate_imageindex.begin()+nn_1);
		else
		{
			int j = 0;
			while(j < nn_1)
			{
				vector<int>::const_iterator iter = std::find(whole_candidate_imageindex.begin(),whole_candidate_imageindex.end(),current_candidate_imageindex[j]);
				if(iter == whole_candidate_imageindex.end())
				{	
					whole_candidate_imageindex.push_back(current_candidate_imageindex[j]);
				}
				j++;
			}
		}
	}

	// select nn_2 nearest neighbors by all the sketches
	for(int i = 0; i < nn_2; i++)
	{
		vector<int>::const_iterator iter = std::find(whole_candidate_imageindex.begin(),whole_candidate_imageindex.end(),imgindex[i]);
		if(iter == whole_candidate_imageindex.end())
		{	
			whole_candidate_imageindex.push_back(imgindex[i]);
		}
	}

	//double max_sample_dist = m_dDist;
	qDebug("The nearest neighbor distance is %f",max_sample_dist);

	// organize the data
	//int local_data_num = whole_candidate_imageindex.size() > 20 ? 20: whole_candidate_imageindex.size();whole_candidate_imageindex.size();
	int local_data_num = whole_candidate_imageindex.size();

	// original data
	Mat data(local_data_num,m_pSkeleton->NUM_BONES_IN_ASF_FILE * 3,CV_64FC1);

	for(int i = 0; i < local_data_num; i++)
	{
		Posture posture;
		posture = *m_pMotion->GetPosture(whole_candidate_imageindex[i]);

		// align the rotation to one same orientation
		TransformationMatrix rot,rotx,roty,rotz,tmp;
		rotx.setToRotationMatrix(posture.bone_rotation[0].x * PI / 180,Vector3d(1.0,0.0,0.0));
		roty.setToRotationMatrix(posture.bone_rotation[0].y * PI / 180,Vector3d(0.0,1.0,0.0));
		rotz.setToRotationMatrix(posture.bone_rotation[0].z * PI / 180,Vector3d(0.0,0.0,1.0));
		tmp.setToProductOf(rotx,roty);
		rot.setToProductOf(tmp,rotz);
		double mTemp[16];
		rot.getOGLValues(mTemp);

		Vector3d vZAxis(mTemp[8], 0.0f, mTemp[10]);
		vZAxis.toUnit();
		float fAngle = acosf(vZAxis.dotProductWith(Vector3d(0.0f, 0.0f, 1.0f)));
		if(vZAxis.x > 0.0f)
		{
			posture.bone_rotation[0].y = -fAngle * 180.0f / PI;
		}
		else{
			posture.bone_rotation[0].y = fAngle * 180.0f / PI;
		}
		
		posture.bone_rotation[0].x = posture.bone_rotation[0].z = 0.0;

		for(int j = 0; j < m_pSkeleton->NUM_BONES_IN_ASF_FILE; j++)
		{
			data.at<double>(i,j*3)   = posture.bone_rotation[j].x;
			data.at<double>(i,j*3+1) = posture.bone_rotation[j].y;
			data.at<double>(i,j*3+2) = posture.bone_rotation[j].z;
		}
	} 

#ifdef SAMPLING_JOINTS

	/************************************************************************/
	/* Method 1                                                             */
	/************************************************************************/
	// sampling certain joints, according to the current limbs drawn
	vector<int> bone_idx;

	if(m_iSketchOrder > -1 && m_iSketchOrder < MAX_SKETCH_NUM)
	{
		Bone* pBone = NULL;

		for(int bone_in_chain = 0; bone_in_chain < m_vChainName[m_iSketchOrder].size(); bone_in_chain++)
		{
			pBone = m_pSkeleton->findBoneByName(m_pSkeleton->getRoot(),m_vChainName[m_iSketchOrder][bone_in_chain].c_str());

			bone_idx.push_back(pBone->idx);

			pBone->m_bDrawn = true;
		}
	}

	// find the Min & Max value for each channel
	Mat sample_range(2,bone_idx.size() * 3,CV_64FC1);
	for(int c = 0; c < bone_idx.size(); c++)
	{
		double minVal,maxVal;

		Mat column = data.col(bone_idx[c] * 3);
		cv::minMaxIdx(column,&minVal,&maxVal);
		sample_range.at<double>(0,c * 3) = minVal;
		sample_range.at<double>(1,c * 3) = maxVal;

		column = data.col(bone_idx[c] * 3 + 1);
		cv::minMaxIdx(column,&minVal,&maxVal);
		sample_range.at<double>(0,c * 3 + 1) = minVal;
		sample_range.at<double>(1,c * 3 + 1) = maxVal;

		column = data.col(bone_idx[c] * 3 + 2);
		cv::minMaxIdx(column,&minVal,&maxVal);
		sample_range.at<double>(0,c * 3 + 2) = minVal;
		sample_range.at<double>(1,c * 3 + 2) = maxVal;
	}

	// randomized sampling, this method can be changed to parallel
	int rand_num = 1000;
	Mat sample_point(1,bone_idx.size() * 3,CV_64FC1);

	// initialize the random function
	initRandom();

	/*************************************************************************/
	/* find the closest sample point                                         */
	/*************************************************************************/
	makeCurrent();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixd(m_vProjectionMatrix);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixd(m_vModelViewMatrix);

	double max_sample_dist = 0.0;

	Posture posture = *m_pMotion->GetPosture(imgindex[0]);

	if(m_iSketchOrder == 0)
		m_SampleClosestPosture = posture;

	for(int j = 0; j < rand_num; j++)
	{
		// clear the background to dark
		glClearColor(1.0f,1.0f,1.0f,1.0f);
		glClearDepth(1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		for(int i = 0; i < bone_idx.size(); i++)
		{
			double range = sample_range.at<double>(1,i * 3) - sample_range.at<double>(0,i * 3);
			sample_point.at<double>(0,i * 3) = sample_range.at<double>(0,i * 3) + range / rand_num * genRandomInt(0,rand_num);

			range = sample_range.at<double>(1,i * 3 + 1) - sample_range.at<double>(0,i * 3 + 1);
			sample_point.at<double>(0,i * 3 + 1) = sample_range.at<double>(0,i * 3 + 1) + range / rand_num * genRandomInt(0,rand_num);

			range = sample_range.at<double>(1,i * 3 + 2) - sample_range.at<double>(0,i * 3 + 2);
			sample_point.at<double>(0,i * 3 + 2) = sample_range.at<double>(0,i * 3 + 2) + range / rand_num * genRandomInt(0,rand_num);
		}

		for(int j = 0; j < bone_idx.size(); j++)
		{
			posture.bone_rotation[bone_idx[j]] = Vector3d(sample_point.at<double>(0,j * 3),sample_point.at<double>(0,j * 3 + 1),sample_point.at<double>(0,j * 3 + 2));
		}

		for(int j = 0; j < m_pSkeleton->NUM_BONES_IN_ASF_FILE; j++)
		{
			vector<int>::const_iterator iter = std::find(bone_idx.begin(),bone_idx.end(),j);
			if(iter != bone_idx.end()) // not update the drawn limbs
			{
				posture.bone_rotation[j] = m_SampleClosestPosture.bone_rotation[j];
			}
		}

		// align the root to the origin and orientation to positive z-axis
		posture.root_pos[0] = posture.bone_translation[0][0] = 0.0;
		posture.root_pos[1] = posture.bone_translation[0][1] = 0.0;
		posture.root_pos[2] = posture.bone_translation[0][2] = 0.0;

		float mTemp[16];
		glPushMatrix();
		glLoadIdentity();
		glRotatef(posture.bone_rotation[0].x,1,0,0);
		glRotatef(posture.bone_rotation[0].y,0,1,0);
		glRotatef(posture.bone_rotation[0].z,0,0,1);
		glPopMatrix();
		Vector3d vZAxis(mTemp[8], 0.0f, mTemp[10]);
		vZAxis.toUnit();
		float fAngle = acosf(vZAxis.dotProductWith(Vector3d(0.0f, 0.0f, 1.0f)));
		if(vZAxis.x > 0.0f)
		{
			m_pSkeleton->ry = -fAngle * 180.0f / PI;
		}
		else{
			m_pSkeleton->ry = fAngle * 180.0f / PI;
		}
		m_pSkeleton->setPosture(posture);

		/************************************************************************/
		/* draw the sampled figure                                              */
		/************************************************************************/ 
		GLfloat bone_color[4] = {0.0,0.0,0.0,1.0};
		m_pSkeleton->RenderFigureWithLines(bone_color);

		// obtain the sample image
		vector<Mat> bin_sample;
		QImage sampleImg = grabFrameBuffer(true);

		imageprocess.process_sketch_image(qimage2mat(sampleImg),bin_sample,ARP);

		Mat sketchpatch32F,samplepatch32F;
		bin_sketch[0].assignTo(sketchpatch32F,CV_32F);
		bin_sample[0].assignTo(samplepatch32F,CV_32F);

		// compute the distance
		double dist = compareHist(sketchpatch32F,samplepatch32F,CV_COMP_CORREL);

		if(max_sample_dist < dist)
		{
			max_sample_dist = dist;

			for(int k = 0; k < bone_idx.size(); k++)
				m_SampleClosestPosture.bone_rotation[bone_idx[k]] = posture.bone_rotation[bone_idx[k]];

			//sampleImg.save("sample.png","png");
		}
	}

	if(m_iSketchOrder > -1 && m_iSketchOrder < MAX_SKETCH_NUM)
	{
		Bone* pBone = NULL;

		for(int bone_in_chain = 0; bone_in_chain < m_vChainName[m_iSketchOrder].size(); bone_in_chain++)
		{
			pBone = m_pSkeleton->findBoneByName(m_pSkeleton->getRoot(),m_vChainName[m_iSketchOrder][bone_in_chain].c_str());
			pBone->m_bDrawn = false;
		}
	}

#endif

#ifdef SAMPLING_WHOLE_BODY
	/************************************************************************/
	/*  Method 2: construct the local PCA model                             */ 
	/************* **********************************************************/ 
	QTime t;
	t.start();

	// PCA dimension reduction without the root rotation
	int low_dimensional = local_data_num > (m_pSkeleton->NUM_BONES_IN_ASF_FILE*3 - 3)? (m_pSkeleton->NUM_BONES_IN_ASF_FILE*3 - 3) : local_data_num;
	//int low_dimensional = 10;

	//cv::PCA pca(data(Range::all(),Range(3,data.cols)),Mat(),CV_PCA_DATA_AS_ROW,low_dimensional);
	//Mat low_dimensional_data = pca.project(data(Range::all(),Range(3,data.cols)));

	cv::PCA pca(data,Mat(),CV_PCA_DATA_AS_ROW,low_dimensional);
	Mat low_dimensional_data = pca.project(data);

	// find the Min & Max value for each channel
	Mat sample_range(2,low_dimensional_data.cols,CV_64FC1);
	for(int c = 0; c < low_dimensional_data.cols; c++)
	{
		double minVal,maxVal;
		Mat column = low_dimensional_data.col(c);
		cv::minMaxIdx(column,&minVal,&maxVal);

		sample_range.at<double>(0,c) = minVal;
		sample_range.at<double>(1,c) = maxVal;;
	}

	// randomized sampling, this method can be changed to parallel
	int rand_num = 1000;
	Mat sample_point(1,low_dimensional,CV_64FC1);
	Mat high_dimensioanl_point;

	// initialize the random function
	initRandom();

	/************************************************************************/
	/* find the closest sample point                                        */
	/************************************************************************/
	makeCurrent();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixd(m_vProjectionMatrix);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixd(m_vModelViewMatrix);

	// obtain the current sketch histogram
	vector<Mat> current_sketch_bin;
	imageprocess.process_sketch_image(qimage2mat(image),current_sketch_bin,ARP);
	Mat sketchpatch32F;
	current_sketch_bin[0].assignTo(sketchpatch32F,CV_32F);

	image.save("current_sketch_image.png","png");

	// initialize the sampling posture
	Posture posture = *m_pMotion->GetPosture(imgindex[0]);
	posture.root_pos[0] = posture.bone_translation[0][0] = 0.0;
	posture.root_pos[1] = posture.bone_translation[0][1] = 0.0;
	posture.root_pos[2] = posture.bone_translation[0][2] = 0.0;

	// align the rotation to one same orientation
	TransformationMatrix rot,rotx,roty,rotz,tmp;
	rotx.setToRotationMatrix(posture.bone_rotation[0].x * PI / 180,Vector3d(1.0,0.0,0.0));
	roty.setToRotationMatrix(posture.bone_rotation[0].y * PI / 180,Vector3d(0.0,1.0,0.0));
	rotz.setToRotationMatrix(posture.bone_rotation[0].z * PI / 180,Vector3d(0.0,0.0,1.0));
	tmp.setToProductOf(rotx,roty);
	rot.setToProductOf(tmp,rotz);
	double mTemp[16];
	rot.getOGLValues(mTemp);

	Vector3d vZAxis(mTemp[8], 0.0f, mTemp[10]);
	vZAxis.toUnit();
	float fAngle = acosf(vZAxis.dotProductWith(Vector3d(0.0f, 0.0f, 1.0f)));
	if(vZAxis.x > 0.0f)
	{
		posture.bone_rotation[0].y = -fAngle * 180.0f / PI;
	}
	else{
		posture.bone_rotation[0].y = fAngle * 180.0f / PI;
	}
	posture.bone_rotation[0].x = posture.bone_rotation[0].z = 0.0;

	m_SampleClosestPosture = posture;

	for(int k = 0; k < rand_num; k++)
	{
		// clear the background to dark
		glClearColor(1.0f,1.0f,1.0f,1.0f);
		glClearDepth(1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		for(int i = 0; i < low_dimensional; i++)
		{
			double range = sample_range.at<double>(1,i) - sample_range.at<double>(0,i);
			sample_point.at<double>(0,i) = sample_range.at<double>(0,i) + range / rand_num * genRandomInt(0,rand_num);
		}

		// measure the distance
		high_dimensioanl_point = pca.backProject(sample_point);
		
		// assume the root orientation is the nearest neighbor
		for(int j = 0; j < m_pSkeleton->NUM_BONES_IN_ASF_FILE ; j++)
		{
			//vector<int>::const_iterator iter = std::find(bone_idx.begin(),bone_idx.end(),j);
			//if(iter != bone_idx.end()) // not update the drawn limbs
			//{
			//	posture.bone_rotation[j] = m_SampleClosestPosture.bone_rotation[j];
			//}
			//else
			posture.bone_rotation[j] = Vector3d(high_dimensioanl_point.at<double>(0,j*3),high_dimensioanl_point.at<double>(0,j*3+1),high_dimensioanl_point.at<double>(0,j*3+2));
		}

		//if(bone_idx.size() != 0)
		//	posture.bone_rotation[0] = m_SampleClosestPosture.bone_rotation[0];
		m_pSkeleton->rx = 0.0;
		m_pSkeleton->rz = 0.0;
		m_pSkeleton->ry = posture.bone_rotation[0].y;

		m_pSkeleton->setPosture(posture);

		/************************************************************************/
		/* draw the sampled figure                                              */
		/************************************************************************/ 
		GLfloat bone_color[4] = {0.0,0.0,0.0,1.0};
		
		// 需要注意的是Bone中m_bDrawn要设置为true
		m_pSkeleton->RenderFigureWithLines(bone_color);

		// obtain the sample image
		vector<Mat> bin_sample;
		QImage sampleImg = grabFrameBuffer(true);
		imageprocess.process_sketch_image(qimage2mat(sampleImg),bin_sample,ARP);
		
		//sampleImg.save("sample.png","png");

		Mat samplepatch32F;
		bin_sample[0].assignTo(samplepatch32F,CV_32F);

		// compute the distance
		double dist = compareHist(sketchpatch32F,samplepatch32F,CV_COMP_CORREL);

		qDebug("The %d sample: distance is %f",k,dist);

		if(max_sample_dist < dist)
		{
			max_sample_dist = dist;

			m_SampleClosestPosture = posture;

			sampleImg.save("sample.png","png");

			qDebug("The current maximum distance is %f",max_sample_dist);
		}
	}
	qDebug("Time elapsed for %d times sampling: %d ms",rand_num, t.elapsed());

#endif
	m_bRefined = true;

	repaint();
}

/*
	refine the final result by sampling
*/
void RenderArea::refineResult2()
{
	m_vLocalNNPosture.clear();

	// only when the last sketch has been done, the result will be refined
	if(m_iSketchOrder != MAX_SKETCH_NUM && m_vSketchHist.size() != MAX_SKETCH_NUM)
		return;

	// select the first nearest neighbor by each sketch
	for(int i = 0; i < m_vSketchHist.size(); i++)
	{
		vector<int>		current_candidate_imageindex;
		vector<double>	dist_sorted;
		vector<Mat>		current_hist;

		current_hist.push_back(m_vSketchHist[i]);

		imageprocess.setFirstVoteStatus(true);
		imageprocess.patchdistmeasure(current_hist,current_candidate_imageindex,dist_sorted,i);

		Posture posture = *m_pMotion->GetPosture(current_candidate_imageindex[0]);

		m_vLocalNNPosture.push_back(posture);
		
		/*
		// update the joints related with first sketch
		if(i == 0)
		{
			posture.root_pos[0] = posture.bone_translation[0][0] = 0.0;
			posture.root_pos[1] = posture.bone_translation[0][1] = 0.0;
			posture.root_pos[2] = posture.bone_translation[0][2] = 0.0;

			// align the rotation to one same orientation
			TransformationMatrix rot,rotx,roty,rotz,tmp;
			rotx.setToRotationMatrix(posture.bone_rotation[0].x * PI / 180,Vector3d(1.0,0.0,0.0));
			roty.setToRotationMatrix(posture.bone_rotation[0].y * PI / 180,Vector3d(0.0,1.0,0.0));
			rotz.setToRotationMatrix(posture.bone_rotation[0].z * PI / 180,Vector3d(0.0,0.0,1.0));
			tmp.setToProductOf(rotx,roty);
			rot.setToProductOf(tmp,rotz);
			double mTemp[16];
			rot.getOGLValues(mTemp);

			Vector3d vZAxis(mTemp[8], 0.0f, mTemp[10]);
			vZAxis.toUnit();
			float fAngle = acosf(vZAxis.dotProductWith(Vector3d(0.0f, 0.0f, 1.0f)));
			if(vZAxis.x > 0.0f)
			{
				posture.bone_rotation[0].y = -fAngle * 180.0f / PI;
			}
			else{
				posture.bone_rotation[0].y = fAngle * 180.0f / PI;
			}
			posture.bone_rotation[0].x = posture.bone_rotation[0].z = 0.0;

			m_SampleClosestPosture = posture;
		}

		// update the joints related with other sketches
		if(m_iSketchOrder > -1 && m_iSketchOrder < MAX_SKETCH_NUM)
		{
			Bone* pBone = NULL;
			Bone* pParentBone = NULL;
			for(int bone_in_chain = 0; bone_in_chain < m_vChainName[m_iSketchOrder].size(); bone_in_chain++)
			{
				pBone = m_pSkeleton->findBoneByName(m_pSkeleton->getRoot(),m_vChainName[m_iSketchOrder][bone_in_chain].c_str());
				pParentBone = m_pSkeleton->findBoneByName(m_pSkeleton->getRoot(),pBone->parent_name);

				m_SampleClosestPosture.bone_rotation[pBone->idx] = Vector3d(pBone->drx,pBone->dry,pBone->drz);
				m_SampleClosestPosture.bone_rotation[pParentBone->idx] = Vector3d(pParentBone->drx,pParentBone->dry,pParentBone->drz);
			}
		}
		//*/
	}

	m_bRefined = true;

	repaint();
}

/*
	refine the final result by sampling
*/
void RenderArea::refineResult3()
{
	// assign the nearest neighbor to the initial sample posture
	Posture posture = *m_pMotion->GetPosture(imgindex[0]);
	m_SampleClosestPosture = posture;

	// only when the last sketch has been done, the result will be refined
	if(m_iSketchOrder != MAX_SKETCH_NUM && m_vSketchHist.size() != MAX_SKETCH_NUM)
		return;

	int nn_1 = 50;	
	// extract the feature from the whole sketch image
	/*vector<Mat> bin_sketch;
	imageprocess.process_sketch_image(qimage2mat(image),bin_sketch,ARP);
	Mat sketchpatch32F;
	bin_sketch[0].assignTo(sketchpatch32F,CV_32F);*/

	// select nn_1 nearest neighbors by each sketch
	for(int i = 0; i < m_vSketchHist.size(); i++)
	{
		if(i == 1 || i == 3)
			continue;

		vector<int>		current_candidate_imageindex;
		vector<double>	dist_sorted;
		vector<Mat>		current_hist;

		current_hist.push_back(m_vSketchHist[i]);

		// compute the NN according to limb
		imageprocess.setFirstVoteStatus(true);
		imageprocess.patchdistmeasure(current_hist,current_candidate_imageindex,dist_sorted,i);	// only one resolution

		/************************************************************************/
		/* obtain the original data                                             */
		/************************************************************************/
		Mat data(nn_1,m_vChainName[i].size() * 3,CV_64FC1);

		for(int j = 0; j < nn_1; j++)
		{
			Posture tmp_posture;
			tmp_posture = *m_pMotion->GetPosture(current_candidate_imageindex[j]);

			for(int k = 0; k < m_vChainName[i].size(); k++)
			{
				Bone* pBone = m_pSkeleton->findBoneByName(m_pSkeleton->getRoot(),m_vChainName[i][k].c_str());

				data.at<double>(j,k*3)   = tmp_posture.bone_rotation[pBone->idx].x;
				data.at<double>(j,k*3+1) = tmp_posture.bone_rotation[pBone->idx].y;
				data.at<double>(j,k*3+2) = tmp_posture.bone_rotation[pBone->idx].z;
			}
		} 

		/************************************************************************/
		/* sampling limb                                                        */
		/************************************************************************/
		// find the Min & Max value for each channel
		Mat sample_range(2,data.cols,CV_64FC1);
		for(int c = 0; c < data.cols; c++)
		{
			double minVal,maxVal;

			Mat column = data.col(c);
			cv::minMaxIdx(column,&minVal,&maxVal);
			sample_range.at<double>(0,c) = minVal;
			sample_range.at<double>(1,c) = maxVal;
		}

		// randomized sampling, this method can be changed to parallel
		int rand_num = 5000;
		Mat sample_point(1,data.cols,CV_64FC1);

		// initialize the random function
		initRandom();

		/*************************************************************************/
		/* find the closest sample point                                         */
		/*************************************************************************/
		makeCurrent();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixd(m_vProjectionMatrix);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixd(m_vModelViewMatrix);

		// assign the initial maximum distance computed by distance measure
		//double max_sample_dist = dist_sorted[0];
		double max_sample_dist = 0.0;
		qDebug("The nearest neighbor distance is %f",max_sample_dist);

		Mat sketchpatch32F;
		m_vSketchHist[i].assignTo(sketchpatch32F,CV_32F);

		// to measure the time
		QTime t;
		t.start();

		for(int sampleIter = 0; sampleIter < rand_num; sampleIter++)
		{
			// clear the background to dark
			glClearColor(1.0f,1.0f,1.0f,1.0f);
			glClearDepth(1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			for(int sampleDimension = 0; sampleDimension < data.cols; sampleDimension++)
			{
				double range = sample_range.at<double>(1,sampleDimension) - sample_range.at<double>(0,sampleDimension);
				sample_point.at<double>(0,sampleDimension) = sample_range.at<double>(0,sampleDimension) + range / rand_num * genRandomInt(0,rand_num);
			}

			// update certain joints
			for(int k = 0; k < m_vChainName[i].size(); k++)
			{
				Bone* pBone = m_pSkeleton->findBoneByName(m_pSkeleton->getRoot(),m_vChainName[i][k].c_str());

				posture.bone_rotation[pBone->idx].x = sample_point.at<double>(0,k * 3);
				posture.bone_rotation[pBone->idx].y = sample_point.at<double>(0,k * 3 + 1);
				posture.bone_rotation[pBone->idx].z = sample_point.at<double>(0,k * 3 + 2);
			}

			// align the root to the origin and orientation to positive z-axis
			posture.root_pos[0] = posture.bone_translation[0][0] = 0.0;
			posture.root_pos[1] = posture.bone_translation[0][1] = 0.0;
			posture.root_pos[2] = posture.bone_translation[0][2] = 0.0;

			float mTemp[16];
			glPushMatrix();
			glLoadIdentity();
			glRotatef(posture.bone_rotation[0].x,1,0,0);
			glRotatef(posture.bone_rotation[0].y,0,1,0);
			glRotatef(posture.bone_rotation[0].z,0,0,1);
			glGetFloatv(GL_MODELVIEW_MATRIX, mTemp);
			glPopMatrix();
			Vector3d vZAxis(mTemp[8], 0.0f, mTemp[10]);
			vZAxis.toUnit();
			float fAngle = acosf(vZAxis.dotProductWith(Vector3d(0.0f, 0.0f, 1.0f)));
			if(vZAxis.x > 0.0f)
			{
				m_pSkeleton->ry = -fAngle * 180.0f / PI;
			}
			else{
				m_pSkeleton->ry = fAngle * 180.0f / PI;
			}
			m_pSkeleton->setPosture(posture);

			/************************************************************************/
			/* draw the sampled figure                                              */
			/************************************************************************/ 
			GLfloat bone_color[4] = {0.0,0.0,0.0,1.0};

			//vector<string> chainName;
			//for(int k = 0; k <= i; k++)
			//{
			//	chainName.insert(chainName.end(),m_vChainName[i].begin(),m_vChainName[i].end());
			//}
			//m_pSkeleton->RenderLimbWithLines(chainName,bone_color);

			m_pSkeleton->RenderLimbWithLines(m_vChainName[i],bone_color);

			// obtain the sample image
			vector<Mat> bin_sample;
			QImage sampleImg = grabFrameBuffer(true);
			//sampleImg.save("sample.png","png");

			imageprocess.process_sketch_image(qimage2mat(sampleImg),bin_sample,ARP);

			Mat samplepatch32F;
			bin_sample[0].assignTo(samplepatch32F,CV_32F);

			// compute the distance
			double dist = compareHist(sketchpatch32F,samplepatch32F,CV_COMP_CORREL);
			qDebug("The %d sample: distance is %f",sampleIter,dist);

			if(max_sample_dist < dist)
			{
				max_sample_dist = dist;
				qDebug("The current maximum distance is %f",max_sample_dist);

				// update certain joints
				for(int k = 0; k < m_vChainName[i].size(); k++)
				{
					Bone* pBone = m_pSkeleton->findBoneByName(m_pSkeleton->getRoot(),m_vChainName[i][k].c_str());

					m_SampleClosestPosture.bone_rotation[pBone->idx] = posture.bone_rotation[pBone->idx];
				}
				//sampleImg.save("sample.png","png");
			}
		}
		qDebug("Time elapsed for %d times sampling: %d ms by %d-th limb",rand_num, t.elapsed(),i);

		posture = m_SampleClosestPosture;
	}

	m_bRefined = true;

	repaint();
}


/*
	compute the recall ratio
*/
void RenderArea::computeRecallRatioAndMQR()
{
	// select 500 examples randomly
	int randNum = 500;
	
	// initialize the random function
	initRandom();

	// the result range
	vector<int> range;
	range.push_back(1);
	range.push_back(5);
	range.push_back(10);
	range.push_back(20);
	range.push_back(30);

	// count the number 
	vector<int> count;
	count.assign(5,0);

	int rank = 0;
	for(int i = 0; i < randNum; i++)
	{
		int r = genRandomInt(0,m_pViewArray[m_iCurrenView]->m_nDBNum-1);

		// query result
		vector<int> queryResult;

		// the distance between query image and database image
		vector<double> dist_sorted;

		// extract the feature from the whole sketch image
		vector<Mat> query_sketch;
		query_sketch.push_back(m_pViewArray[m_iCurrenView]->m_vHistDb[r]);
 
		// query
		imageprocess.patchdistmeasure(query_sketch,queryResult,dist_sorted);

		/************************************************************************/
		/* compute the recall ratio                                             */
		/************************************************************************/
		if(queryResult[0] == r)
			count[0]++;
		
		vector<int> tmp;
		tmp.insert(tmp.end(),queryResult.begin(),queryResult.begin()+range[1]);
		if(std::find(tmp.begin(),tmp.end(),r) != tmp.end())
			count[1]++;

		tmp.clear();
		tmp.insert(tmp.end(),queryResult.begin(),queryResult.begin()+range[2]);
		if(std::find(tmp.begin(),tmp.end(),r) != tmp.end())
			count[2]++;

		tmp.clear();
		tmp.insert(tmp.end(),queryResult.begin(),queryResult.begin()+range[3]);
		if(std::find(tmp.begin(),tmp.end(),r) != tmp.end())
			count[3]++;

		tmp.clear();
		tmp.insert(tmp.end(),queryResult.begin(),queryResult.begin()+range[4]);
		if(std::find(tmp.begin(),tmp.end(),r) != tmp.end())
			count[4]++;

		/************************************************************************/
		/* compute the rank                                                     */
		/************************************************************************/
		tmp.clear();
		tmp.insert(tmp.end(),queryResult.begin(),queryResult.begin()+30);
		rank += std::find(tmp.begin(),tmp.begin()+30,r) - tmp.begin();
	}

	for(int i = 0; i < count.size(); i++)
	{
		qDebug("The ratio is %f from %d%% best candidates",(float)count[i]/randNum * 100,range[i]);
	}

	qDebug("The mean of query rank is %d\n",rank);
}

/*
	capture the canvas
*/
void RenderArea::screenCaptureAsPng()
{
	/************************************************************************/
	/* Draw the canvas context into pixmap object                           */
	/************************************************************************/
    QImage	canvas = QImage(IMAGE_SIZE,IMAGE_SIZE,QImage::Format_ARGB32);

	QPainter pp(&canvas);
	
	// the background
	pp.setCompositionMode(QPainter::CompositionMode_SourceOver);
	
	if(m_b3DPose)	// 3D
	{
		if(m_bShowShadow)
		{
			// background
			pp.drawImage(0,0,threedpose);	// candidate 3D poses
		}

		if(m_bShowSketch)
		{
			// foreground
			pp.setCompositionMode(QPainter::CompositionMode_SourceOver);	
			//painter.drawImage(0,0,threedstyleline);	// user's sketch
			pp.drawImage(0,0,styleUsrDrawingImage);	// user's sketch

		}
	}
	else	// 2D
	{
		if(m_bShowShadow)
		{
			// background
			pp.drawImage(0,0,backImage);	// candidate images
		}

		if(m_bShowGrid)
		{
			pp.setCompositionMode(QPainter::CompositionMode_SourceOver);
			pp.drawImage(0,0,gridImage);	// grid images
		}
		if(m_bShowSketch)
		{
			// foreground
			pp.setCompositionMode(QPainter::CompositionMode_SourceOver);
			pp.drawImage(0,0,styleUsrDrawingImage);	// user's sketch
		}
	}

	if(m_iSketchOrder < MAX_SKETCH_NUM)
	{
		QFont serifFont("Times New Roman", 22, QFont::Bold);
		pp.setFont(serifFont);
		pp.setPen(QColor(aColor[m_iSketchOrder]));
		pp.drawText(10,30,hintStr[m_iSketchOrder]);
	}
	
	pp.end();

	/************************************************************************/
	/* Save the pixmap object into file                                     */
	/************************************************************************/
	static int nImg = 0;

	string fPath = ".\\ScreenCapture";
	string capturedName = "image";

	capturedName += '0' + nImg / 10000;
	capturedName += '0' + (nImg / 1000) % 10;
	capturedName += '0' + (nImg / 100) % 10;
	capturedName += '0' + (nImg / 10) % 10;
	capturedName += '0' + nImg % 10;
	capturedName += ".png";

	string capturedPath = fPath + "\\" +capturedName;

	canvas.save(capturedPath.c_str(), "png");

	nImg++;
}


// refine the final result by particle swarm optimization
void RenderArea::refineResultbyPSO(void)
{
	// only when the last sketch has been done, the result will be refined
	if(m_iSketchOrder != MAX_SKETCH_NUM && m_vSketchHist.size() != MAX_SKETCH_NUM)
		return;
	
	// the number of particles: nn_1
	int nn_each_limb = 30;
	int nn_1 = nn_each_limb * 7;	

	// the dimension of particle
	int dim = m_pSkeleton->NUM_BONES_IN_ASF_FILE * 3;

	// calculate the sampling time
	QTime t;
	t.start();

	qDebug("PSO is beginning...");
	
	// particle swarm optimization
	PSO pso;
	pso.createParticles(nn_1,dim);

	// initialize the best global position using the nearest neighbor
	Posture posture = *m_pMotion->GetPosture(imgindex[0]);

	float position[500];
	memset(position,0.0,sizeof(float) * 500);
	
	/*
	for(int i = 0; i < m_pSkeleton->NUM_BONES_IN_ASF_FILE; i++)
	{
		position[i*3] = posture.bone_rotation[i].getX();
		position[i*3+1] = posture.bone_rotation[i].getY();
		position[i*3+2] = posture.bone_rotation[i].getZ();
	}
	//*/
	pso.initGlobalBestPosition(position,dim);

	// select nn_1 nearest neighbors by each sketch
	for(int i = 0; i < m_vSketchHist.size(); i++)
	{
		vector<int>		current_candidate_imageindex;
		vector<double>	dist_sorted;
		vector<Mat>		current_hist;

		current_hist.push_back(m_vSketchHist[i]);

		// compute the NN according to limb
		imageprocess.setFirstVoteStatus(true);
		imageprocess.patchdistmeasure(current_hist,current_candidate_imageindex,dist_sorted,i);	

		/************************************************************************/
		/* obtain the original data                                             */
		/************************************************************************/
		for(int j = 0; j < nn_each_limb; j++)
		{
			Posture tmp_posture;
			tmp_posture = *m_pMotion->GetPosture(current_candidate_imageindex[j]);

			for(int k = 0; k < m_pSkeleton->NUM_BONES_IN_ASF_FILE; k++)
			{
				pso.setPosition(j + i * nn_each_limb,k * 3,tmp_posture.bone_rotation[k].x);
				pso.setPosition(j + i * nn_each_limb,k * 3 + 1,tmp_posture.bone_rotation[k].y);
				pso.setPosition(j + i * nn_each_limb,k * 3 + 2,tmp_posture.bone_rotation[k].z);

				pso.setVelocity(j + i * nn_each_limb,k * 3,0.0);
				pso.setVelocity(j + i * nn_each_limb,k * 3 + 1,0.0);
				pso.setVelocity(j + i * nn_each_limb,k * 3 + 2,0.0);

				pso.setPersonalBestPosition(j + i * nn_each_limb,k * 3,tmp_posture.bone_rotation[k].x);
				pso.setPersonalBestPosition(j + i * nn_each_limb,k * 3 + 1,tmp_posture.bone_rotation[k].y);
				pso.setPersonalBestPosition(j + i * nn_each_limb,k * 3 + 2,tmp_posture.bone_rotation[k].z);
			}
		}
	}

	// obtain the users' whole sketches
	vector<Mat> current_sketch_bin;
	imageprocess.process_sketch_image(qimage2mat(image),current_sketch_bin,ARP);
	Mat sketchpatch32F;
	current_sketch_bin[0].assignTo(sketchpatch32F,CV_32F);

	// the distance between the global best position with the user's sketches
	double max_sample_dist = 0.0;
	
	// the distance array for each particle's personal best position
	double dist[1000];
	memset(dist,0.0,sizeof(double) * 1000);

	/************************************************************************/
	/* find the closest sample point using PSO method                       */
	/************************************************************************/
	
	// set the ModelView & Projection matrix
	makeCurrent();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixd(m_vProjectionMatrix);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixd(m_vModelViewMatrix);

	// sampling iterations
	for(int iteration = 0; iteration < 50; iteration++)
	{
		qDebug("The %d-th iteration \n",iteration);

		// update all those particles
		pso.updateParticles();

		for(int particle_idx = 0; particle_idx < nn_1; particle_idx++)
		{
			// clear the background to dark
			glClearColor(1.0f,1.0f,1.0f,1.0f);
			glClearDepth(1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			for(int i = 0; i < m_pSkeleton->NUM_BONES_IN_ASF_FILE; i++)
			{
				posture.bone_rotation[i].x = pso.getPosition(particle_idx,i * 3);
				posture.bone_rotation[i].y = pso.getPosition(particle_idx,i * 3 + 1);
				posture.bone_rotation[i].z = pso.getPosition(particle_idx,i * 3 + 2);
			}

			// align the root to the origin and orientation to positive z-axis
			posture.root_pos[0] = posture.bone_translation[0][0] = 0.0;
			posture.root_pos[1] = posture.bone_translation[0][1] = 0.0;
			posture.root_pos[2] = posture.bone_translation[0][2] = 0.0;

			float mTemp[16];
			glPushMatrix();
			glLoadIdentity();
			glRotatef(posture.bone_rotation[0].x,1,0,0);
			glRotatef(posture.bone_rotation[0].y,0,1,0);
			glRotatef(posture.bone_rotation[0].z,0,0,1);
			glGetFloatv(GL_MODELVIEW_MATRIX, mTemp);
			glPopMatrix();
			Vector3d vZAxis(mTemp[8], 0.0f, mTemp[10]);
			vZAxis.toUnit();
			float fAngle = acosf(vZAxis.dotProductWith(Vector3d(0.0f, 0.0f, 1.0f)));
			if(vZAxis.x > 0.0f)
			{
				m_pSkeleton->ry = -fAngle * 180.0f / PI;
			}
			else{
				m_pSkeleton->ry = fAngle * 180.0f / PI;
			}
			m_pSkeleton->setPosture(posture);

			/************************************************************************/
			/* draw the sampled figure                                              */
			/************************************************************************/ 
			GLfloat bone_color[4] = {0.0,0.0,0.0,1.0};
			m_pSkeleton->RenderFigureWithLines(bone_color);

			// obtain the sample image
			vector<Mat> bin_sample;
			QImage sampleImg = grabFrameBuffer(true);
			//sampleImg.save("sample.png","png");

			imageprocess.process_sketch_image(qimage2mat(sampleImg),bin_sample,ARP);

			Mat samplepatch32F;
			bin_sample[0].assignTo(samplepatch32F,CV_32F);

			// compute the distance
			double d = compareHist(sketchpatch32F,samplepatch32F,CV_COMP_CORREL);

			if(dist[particle_idx] < d)
			{
				dist[particle_idx] = d;
				qDebug("The distance between personal best position of the %d-th particle with user's sketches is %f",particle_idx, dist[particle_idx]);

				// update the personal best of particles
				for(int i = 0; i < m_pSkeleton->NUM_BONES_IN_ASF_FILE; i++)
				{
					pso.setPersonalBestPosition(particle_idx,i * 3,posture.bone_rotation[i].getX());
					pso.setPersonalBestPosition(particle_idx,i * 3 + 1,posture.bone_rotation[i].getY());
					pso.setPersonalBestPosition(particle_idx,i * 3 + 2,posture.bone_rotation[i].getZ());
				}

				if(max_sample_dist < dist[particle_idx])
				{
					max_sample_dist = dist[particle_idx];

					// update the global best position
					for(int i = 0; i < m_pSkeleton->NUM_BONES_IN_ASF_FILE; i++)
					{
						pso.setGlobalBestPosition(i * 3,posture.bone_rotation[i].getX());
						pso.setGlobalBestPosition(i * 3 + 1,posture.bone_rotation[i].getY());
						pso.setGlobalBestPosition(i * 3 + 2,posture.bone_rotation[i].getZ());
					}

					sampleImg.save("sample.png","png");
				}
			}
		}

		qDebug("\nThe distance between global best sampling position and user's sketches is %f at %d iteration\n",max_sample_dist, iteration);
	}

	// obtain the final sampling result
	for(int i = 0; i < m_pSkeleton->NUM_BONES_IN_ASF_FILE; i++)
	{
		m_SampleClosestPosture.bone_rotation[i].x = pso.getGlobalBestPosition(i * 3);
		m_SampleClosestPosture.bone_rotation[i].y = pso.getGlobalBestPosition(i * 3 + 1);
		m_SampleClosestPosture.bone_rotation[i].z = pso.getGlobalBestPosition(i * 3 + 2);
	}
	//*/
	qDebug("The final distance between global best sampling position and user's sketches is %f\n", max_sample_dist);
	qDebug("The elapsed time for all these iterations is %d ms",t.elapsed());


	m_bRefined = true;
	
	repaint();
}


/*
	replay the process of user's sketching
*/
void RenderArea::replay()
{
	if(m_vHistoryPoints.size() != 0)
	{
		m_iSketchOrder = 0;

		for(int i = 0; i < m_vHistoryPoints.size(); i++)
		{
			stylePen.setColor(aColor[i]);

			double t = m_vElapsedTimeForEachSketch[i];
			int num_of_points = m_vHistoryPoints[i].size();

			for(int j = 0; j < num_of_points; j++)
			{
				m_vPoints.assign(m_vHistoryPoints[i].begin(),m_vHistoryPoints[i].begin() + j);
				
				paint(styleUsrDrawingImage,stylePen);
				repaint();

				Sleep(t/num_of_points);
			}

			paint(image,pen);

			// indicate the sketch has began
			m_bStartUp = false;

			// only contain the current sketch line
			QImage current_sketch_image = QImage(IMAGE_SIZE,IMAGE_SIZE,QImage::Format_ARGB32);  //32bit color with size is 720,720
			backColor = qRgba(255,255,255,0);									  //initial background color is white, transparent
			current_sketch_image.fill(backColor);
			paint(current_sketch_image,pen);

			// update the candidate image index
			updateCandidateSets2(current_sketch_image, current_sketch_image);

			m_iSketchOrder++;
		}
	}
}


void RenderArea::test()
{
	int sz = m_TrajectoryFeature.size();

	int idx = genRandomInt(0,sz);

	vector<QPoint> feature1;
	feature1.assign(m_TrajectoryFeature[idx].begin(),m_TrajectoryFeature[idx].end());

	int dim_of_feature = feature1.size();

	for(int i = 0; i < sz; i++)
	{
		double dist = 0;

		vector<QPoint> feature2;
		if(i == idx)
			continue;

		feature2.assign(m_TrajectoryFeature[i].begin(),m_TrajectoryFeature[i].end());

		QPoint diff_p;
		for(int j = 0; j < dim_of_feature; j++)
		{
			diff_p = feature2[j] - feature1[j];
			dist += sqrt((double)(diff_p.x() * diff_p.x() + diff_p.y() * diff_p.y()));
		}

		qDebug()<<"The distance between "<<idx<<" trajectory and "<< i << " trajectory is "<<dist<<endl;
	}
}

void RenderArea::loadMotionClipsFromFile(char* filename)
{
	QFile file(filename);

	if(!file.open(QFile::ReadOnly | QFile::Text))
	{
		std::cerr<<"Error: Cannot read file "<< qPrintable(filename)<<" : "<<qPrintable(file.errorString());

	}

	QString errostr;
	int errorline;
	int errocolumn;

	QDomDocument doc;

	if(!doc.setContent(&file,false,&errostr,&errorline,&errocolumn))
	{
		std::cerr<<"Error: Parse error at line "<<errorline<<", "<<"column "<<errocolumn<<": "<<qPrintable(errostr)<<std::endl;
	}

	QDomElement root = doc.documentElement();

	// get the list of motion clips 
	QDomNodeList nodelist = root.childNodes();

	for(int i = 0; i < nodelist.size(); i++)
	{
		CMEMotionClip* pMotionClip = new CMEMotionClip();

		QDomNode node = nodelist.at(i);

		pMotionClip->setMotionClipName((char*)(node.nodeName().toStdString().c_str()));

		QDomNodeList nodelist2 = node.childNodes();

		for(int j = 0; j < nodelist2.size(); j++)
		{
			vector<Vector3d> vPos;

			QDomNode node2 = nodelist2.at(j);

			QString node_str = node2.nodeName();

			static int data_num = 0;
			if(node_str == "ctrl_num")
			{
				QDomNode leapnode = node2.firstChild();
				QDomText str = leapnode.toText();
				QString s = str.data();
				data_num = s.toInt();
			}
			else
			{
				QDomNode node3 = node2.firstChild();
				QDomNode leapnode = node3.firstChild();
				QDomText str = leapnode.toText();
				QString s = str.data();

				stringstream ss;
				ss<<s.toStdString().c_str();

				Vector3d v3d;
				for(int k = 0; k < data_num; k++)
				{
					ss>>v3d.x>>v3d.y>>v3d.z;

					vPos.push_back(v3d);
				}

				pMotionClip->m_vJointTrajByPos.push_back(vPos);
			}
		}

		qDebug()<<"The "<<(pMotionClip->getMotionClipName()).c_str()<<"motion clip file is loaded\n";

		m_vMotionClip.push_back(pMotionClip);
	}
	qDebug()<<"Totally there are "<<nodelist.size()<<"motion clips loaded\n";
}

/*
	draw sketching pose interface
*/
void RenderArea::drawSketchingPoseInterface()
{
	// draw the character
	int displaynum = m_iDisplayNearestNum >= imgindex.size() ? imgindex.size() : m_iDisplayNearestNum;

	GLfloat bone_color[4] = {m_vColormap[0][0],m_vColormap[0][1],m_vColormap[0][2],1.0};
	GLfloat joint_color[4] = {0.65,0.65,0.0,1.0};

	if(m_pMotion && m_pSkeleton && imgindex.size() != 0)
	{
		int i = 0;

		//displaynum = displaynum > 10 ? 10: displaynum;
		displaynum = displaynum > 30 ? 30: displaynum;

		// line width
		float linwidth = 5.0 / displaynum;
		float color = 1.0 / displaynum;
		for(i = 0; i < displaynum; i++)
		{
			Posture posture;
			posture = *m_pMotion->GetPosture(imgindex[i]);

			// align the root to the origin and orientation to positive z-axis
			posture.root_pos[0] = posture.bone_translation[0][0] = 0.0;
			posture.root_pos[1] = posture.bone_translation[0][1] = 0.0;
			posture.root_pos[2] = posture.bone_translation[0][2] = 0.0;

			float mTemp[16];
			glPushMatrix();
			glLoadIdentity();
			glRotatef(posture.bone_rotation[0].x,1,0,0);
			glRotatef(posture.bone_rotation[0].y,0,1,0);
			glRotatef(posture.bone_rotation[0].z,0,0,1);
			glGetFloatv(GL_MODELVIEW_MATRIX, mTemp);
			glPopMatrix();
			Vector3d vZAxis(mTemp[8], 0.0f, mTemp[10]);
			vZAxis.toUnit();
			float fAngle = acosf(vZAxis.dotProductWith(Vector3d(0.0f, 0.0f, 1.0f)));
			if(vZAxis.x > 0.0f)
			{
				m_pSkeleton->ry = -fAngle * 180.0f / PI;
			}
			else{
				m_pSkeleton->ry = fAngle * 180.0f / PI;
			}
			m_pSkeleton->setPosture(posture);

			glLineWidth((displaynum - i) * linwidth);
			//glLineWidth(2.0);

			// draw the limb according to the fixed order
			if(m_iSketchOrder > -1 && m_iSketchOrder < MAX_SKETCH_NUM)
			{
				Bone* pBone = NULL;
				Bone* pParentBone = NULL;

				for(int bone_in_chain = 0; bone_in_chain < m_vChainName[m_iSketchOrder].size(); bone_in_chain++)
				{
					pBone = m_pSkeleton->findBoneByName(m_pSkeleton->getRoot(),m_vChainName[m_iSketchOrder][bone_in_chain].c_str());
					pParentBone = m_pSkeleton->findBoneByName(m_pSkeleton->getRoot(),pBone->parent_name);

					glBegin(GL_LINES);
					//glColor3f(m_vColormap[m_vColormap.size()-1-i*3][0],m_vColormap[m_vColormap.size()-1-i*3][1],m_vColormap[m_vColormap.size()-1-i*3][2]);
					//glColor3f(1.0 * (1 - i * 0.05),1.0 * (1 - i * 0.05),1.0 * (1 - i * 0.05));
					glColor3f((displaynum - i) * color,(displaynum - i) * color,(displaynum - i) * color);
					glVertex3f(pBone->m_GlobalPosition[0],pBone->m_GlobalPosition[1],pBone->m_GlobalPosition[2]);
					glVertex3f(pParentBone->m_GlobalPosition[0],pParentBone->m_GlobalPosition[1],pParentBone->m_GlobalPosition[2]);
					glEnd();
				}

				bone_color[3] = m_iSketchOrder * 0.0667;
				joint_color[3] = m_iSketchOrder * 0.0667;
			}

			// draw the character
			if(i == 0)
			{	
				/************************************************************************/
				/*  Draw the nearest neighbor                                           */
				/************************************************************************/
				if(!m_bStartUp)
				{
					glEnable(GL_BLEND);
					glDepthMask(GL_FALSE);
					glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

					/************************************************************************/
					/* Render the sampling result                                           */
					/************************************************************************/
					if(m_iSketchOrder < 0)
					{
						m_pSkeleton->RenderFigure(bone_color,joint_color);
					}
					else{
						static float cs_aSketchedBoneColor[4] = {0.9f, 0.7f, 0.2f, 1.0f};
						static float cs_aSketchedJointColor[4]= {0.9f, 0.2f, 0.2f, 1.0f};

						if(m_iSketchOrder - 1 >= 0)
						{
							for(int bone_in_chain = 0; bone_in_chain < m_vChainName[m_iSketchOrder-1].size(); bone_in_chain++)
							{
								vSketchedBones.push_back(m_vChainName[m_iSketchOrder-1][bone_in_chain]);
							}
						}

						m_pSkeleton->RenderSketchedFigure(vSketchedBones, cs_aSketchedBoneColor, cs_aSketchedJointColor,bone_color, joint_color);
					}

					glDepthMask(GL_TRUE);
					glDisable(GL_BLEND);
				}
			}
		}

		/************************************************************************/
		/* Draw the sampling result                                             */
		/************************************************************************/
		if(m_bRefined)
		{
			Posture posture = m_SampleClosestPosture;

			// align the root to the origin and orientation to positive z-axis
			posture.root_pos[0] = posture.bone_translation[0][0] = 0.0;
			posture.root_pos[1] = posture.bone_translation[0][1] = 0.0;
			posture.root_pos[2] = posture.bone_translation[0][2] = 0.0;

			float mTemp[16];
			glPushMatrix();
			glLoadIdentity();
			glRotatef(posture.bone_rotation[0].x,1,0,0);
			glRotatef(posture.bone_rotation[0].y,0,1,0);
			glRotatef(posture.bone_rotation[0].z,0,0,1);
			glGetFloatv(GL_MODELVIEW_MATRIX, mTemp);
			glPopMatrix();
			Vector3d vZAxis(mTemp[8], 0.0f, mTemp[10]);
			vZAxis.toUnit();
			float fAngle = acosf(vZAxis.dotProductWith(Vector3d(0.0f, 0.0f, 1.0f)));
			if(vZAxis.x > 0.0f)
			{
				m_pSkeleton->ry = -fAngle * 180.0f / PI;
			}
			else{
				m_pSkeleton->ry = fAngle * 180.0f / PI;
			}
			m_pSkeleton->setPosture(posture);

			float cs_aSketchedBoneColor[4] = {0.2f, 0.7f, 0.9f, 1.0f};
			float cs_aSketchedJointColor[4]= {0.2f, 0.2f, 0.9f, 1.0f};

			vector<string> sampledSketchedBones;
			for(int k = 0; k < MAX_SKETCH_NUM; k++)
			{
				for(int bone_in_chain = 0; bone_in_chain < m_vChainName[k].size(); bone_in_chain++)
				{
					sampledSketchedBones.push_back(m_vChainName[k][bone_in_chain]);
				}
			}

			m_pSkeleton->RenderSketchedFigure(sampledSketchedBones, cs_aSketchedBoneColor, cs_aSketchedJointColor,bone_color, joint_color);
		}
	}
}

/*
	draw sketching animation interface
*/
void RenderArea::drawSketchingAnimationInterface()
{
	if(m_pMotion)
	{
		Posture posture;

		posture = *m_pMotion->GetPosture(0);

		// align the root to the origin
		posture.root_pos[0] = posture.bone_translation[0][0] = 0.0;
		posture.root_pos[1] = posture.bone_translation[0][1] = 0.0;
		posture.root_pos[2] = posture.bone_translation[0][2] = 0.0;

		m_pSkeleton->setPosture(posture);
	}

	GLfloat bone_color[4] = {m_vColormap[0][0],m_vColormap[0][1],m_vColormap[0][2],1.0};
	GLfloat joint_color[4] = {0.65,0.65,0.0,1.0};
	GLfloat cs_aSketchedBoneColor[4] = {0.9f, 0.7f, 0.2f, 1.0f};
	GLfloat cs_aSketchedJointColor[4]= {0.9f, 0.2f, 0.2f, 1.0f};

	// draw the character
	glEnable(GL_BLEND);
	glDepthMask(GL_FALSE);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	for(int i = 0; i < m_vChainName.size(); i++)
	{
		for(int bone_in_chain = 0; bone_in_chain < m_vChainName[i].size(); bone_in_chain++)
		{
			vSketchedBones.push_back(m_vChainName[i][bone_in_chain]);
		}
	}
	
	m_pSkeleton->RenderSketchedFigure(vSketchedBones, cs_aSketchedBoneColor, cs_aSketchedJointColor,bone_color, joint_color);

	char* joint_name[] = {"root","lradius","rradius","ltibia","rtibia"};

	int idx = m_pSkeleton->name2idx(joint_name[m_iShowTrajectoryIndex]);
	
	Bone* bone = m_pSkeleton->getBone(m_pSkeleton->getRoot(),idx);
	
	Vector3d global_pos;
	global_pos.x = bone->m_GlobalPosition[0];
	global_pos.y = bone->m_GlobalPosition[1];
	global_pos.z = bone->m_GlobalPosition[2];

	// line width
	int motion_clip_num = m_vMotionClip.size();

	float linwidth = 5.0 / motion_clip_num;
	float color = 1.0 / motion_clip_num;

	// show the joint trajectory
	for(int clip_idx = 0; clip_idx < motion_clip_num; clip_idx++)
	{
		int idx = motion_clip_index[clip_idx];
		vector<Vector3d>& vPos = m_vMotionClip[idx]->m_vJointTrajByPos[m_iShowTrajectoryIndex];

		int num_frame = vPos.size();

		Vector3d vtmp = vPos[0] * .6;

		glPushMatrix();
		// for translation
		glTranslatef(global_pos.x - vtmp.x,global_pos.y - vtmp.y,global_pos.z - vtmp.z);
		// for rotation

		glLineWidth((motion_clip_num - clip_idx) * linwidth);
		glColor3f((motion_clip_num - clip_idx) * color,(motion_clip_num - clip_idx) * color,(motion_clip_num - clip_idx) * color);

		glBegin(GL_LINE_STRIP);
		for(int j = 0; j < num_frame; j++)
		{
			vtmp = vPos[j] * .6;
			glVertex3f(vtmp.getX(),vtmp.getY(),vtmp.getZ());
		}
		glEnd();
		glPopMatrix();
	}

	glDepthMask(GL_TRUE);
	glDisable(GL_BLEND);
}

 /*
	compute the new candidate animation sets
 */
void RenderArea::updateCandidateAnimationSets(const vector<QPoint>& sketch_curve_feature)
{
	// The global position of initial character pose
	char* joint_name[] = {"root","lradius","rradius","ltibia","rtibia"};

	int idx = m_pSkeleton->name2idx(joint_name[m_iShowTrajectoryIndex]);

	Bone* bone = m_pSkeleton->getBone(m_pSkeleton->getRoot(),idx);

	Vector3d global_pos;
	global_pos.x = bone->m_GlobalPosition[0];
	global_pos.y = bone->m_GlobalPosition[1];
	global_pos.z = bone->m_GlobalPosition[2];

	// compute the distance between features
	int dim_of_feature = sketch_curve_feature.size();
	double min_dist = DBL_MAX;
	int cloest_motion_clip = 0;
	vector<double> dist_sort_arr;
	dist_sort_arr.assign(m_vMotionClip.size(),DBL_MAX);

	motion_clip_index.clear();
	motion_clip_index.resize(m_vMotionClip.size());

	for(int i = 0; i < m_vMotionClip.size(); i++)
	{
		// store the curve feature of motion from database
		vector<QPoint> db_curve_feature;

		vector<Vector3d>& traj3D = m_vMotionClip[i]->m_vJointTrajByPos[m_iShowTrajectoryIndex];
		
		// The first point
		Vector3d vtmp = traj3D[0] * .6;

		// The new model-view matrix
		GLdouble model_view_matrix[16];
		memcpy(model_view_matrix,m_vModelViewMatrix,sizeof(double) * 16);

		GLdouble win[3];
		QPoint point2D;

		vector<QPoint> prj2DArr;
		// project these 3D points to 2D
		for(int j = 0; j < traj3D.size(); j++)
		{
			vtmp = traj3D[j] * .6 + (global_pos - traj3D[0] * .6);
			gluProject(vtmp.x,vtmp.y,vtmp.z,model_view_matrix,m_vProjectionMatrix,m_vViewPort,&win[0],&win[1],&win[2]);

			point2D.setX(win[0]);
			point2D.setY(m_vViewPort[3] -  win[1]);
			
			prj2DArr.push_back(point2D);
		}

		// for test
		/*
		QImage tmp_image = QImage(IMAGE_SIZE,IMAGE_SIZE,QImage::Format_ARGB32);  //32bit color with size is 720,720
		backColor = qRgba(0,0,0,255);				//initial background color is white, transparent
		tmp_image.fill(backColor);

		QPainter pt(&tmp_image);
		pt.setPen(QColor(255,255,0));
		for(int i = 1; i < prj2DArr.size(); i++)
			pt.drawLine(prj2DArr[i-1],prj2DArr[i]);
		pt.end();

		tmp_image.save("prj.png","png");
		//*/

		extractMotionFeature(prj2DArr,db_curve_feature);

		// compare the feature between user's sketching motion curve and that from database
		QPoint diff_p;
		double dist = 0.0;
		for(int j = 0; j < dim_of_feature; j++)
		{
			diff_p = sketch_curve_feature[j] - db_curve_feature[j];
			dist += sqrt((double)(diff_p.x() * diff_p.x() + diff_p.y() * diff_p.y()));
		}

		if(dist < min_dist)
		{
			min_dist = dist;
			cloest_motion_clip = i;
		}
		qDebug()<<"The distance between "<<idx<<" trajectory and "<< i << " trajectory is "<<dist<<endl;

		// sort motion clip index array according to the distance
		// insert sorting
		bool insert = false;
		for(int k = 0; k <= i; k++)
		{
			if(dist - dist_sort_arr[k] < 0.0)
			{
				for(int l = i; l > k; l--)
				{
					dist_sort_arr[l] = dist_sort_arr[l-1];
					motion_clip_index[l] = motion_clip_index[l-1];
				}
				dist_sort_arr[k] = dist;
				motion_clip_index[k] = i;
				insert = true;
				break;
			}
		}
		if(!insert)
		{	
			dist_sort_arr[i] = dist;
			motion_clip_index[i] = i;
		}
	}

	qDebug()<<"The closest motion clip is "<< cloest_motion_clip<<endl;
	loadMocapData(motion_clip_filename[cloest_motion_clip],"amc");
}

 /*
	extract the feature of motion curves presented by 2D image points
 */
void RenderArea::extractMotionFeature(const vector<QPoint>& m_vPoints, vector<QPoint>& feature)
{
	double x, y, z;
	std::vector<CAxisAngle> vPos;

	// fit the sketched trajectory into a smooth curve
	for(int i = 0; i < m_vPoints.size(); i++)
	{
		x = m_vPoints[i].x();
		y = m_vPoints[i].y();
		z = 0.0;
		CAxisAngle aa = CAxisAngle(x, y, z);
		vPos.push_back(aa);
	}

	CUpSampling m_SketchTraj;
	m_SketchTraj.AddAllDataA(&vPos[0], 0.0,30.0, vPos.size(), false);
	m_SketchTraj.SetBoundaryCondition(US_BC_I);
	m_SketchTraj.GenerateIt();

	// resample points
	vector<QPoint> resamplePoints;
	int resampleNum = 100;
	for(int i = 0; i < resampleNum; i++)
	{
		CAxisAngle aa = m_SketchTraj.GetDataByTimeA(i * 0.033 * m_vPoints.size() / resampleNum);
		Vector3d vtmp = aa.GetValue();

		resamplePoints.push_back(QPoint((int)vtmp.getX(),(int)vtmp.getY()));
	}

	// compute the feature of motion curve
	for(int i = 0; i < resampleNum; i++)
	{
		QPoint f = resamplePoints[i] - resamplePoints[0];

		feature.push_back(f);
	}
}
