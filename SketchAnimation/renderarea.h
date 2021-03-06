#ifndef RENDERAREA_H
#define RENDERAREA_H

#include "gl/glew.h"
#include <QGLWidget>
#include <QBrush>
#include <QPen>
#include <QPixmap>
#include <QImage>
#include <QTimer>
#include <QTime>
#include <QCursor>
#include <vector>
#include "SAImageProcess.h"
#include "ConfigFile.h"
#include "SACamera.h"
#include "SAView.h"
#include "MEMathLib/Vector3d.h"
#include "DatabaseUnit.h"
#include "ASF/skeleton.h"
#include "ASF/motion.h"
#include "ASF/posture.h"

enum DrawShape{LINE,RECTANGLE,ELLIPSE};
enum DrawMode{PEN_MODE,ERASER_MODE};
enum BehaviorMode{SKETCH,VIEW};			// the behavior: sketching or viewing

#define _PATCH_NUM	8					// the patch number included in the canvas

class BVH;
class Skeleton;
class Motion;
class Posture;
class QTime;
class CMEMotionClip;

class RenderArea : public QGLWidget
{
    Q_OBJECT

public:
    RenderArea(QWidget*parent = 0);
    ~RenderArea();

public slots:
     void setPen(const QPen &pen);
     void setBrush(const QBrush &brush);
     void mousePressEvent(QMouseEvent *);
     void mouseReleaseEvent(QMouseEvent *);
     void mouseMoveEvent(QMouseEvent *);
     void setShape(DrawShape);
	 
	 /*
		dynamically show the shadow
	 */
	 void dynamicShadowHint();
 	
	/*
		capture the canvas
    */
	 void screenCaptureAsPng();

protected:
     void paintEvent(QPaintEvent *event);	 
	 void initializeGL();
	 void resizeGL(int width, int height);

private:
     BehaviorMode behaviorMode;
	 DrawShape shape;
     QPen pen;						// used for drawing query image
	 QPen stylePen;					// used for drawing display image
     QBrush brush;
     QImage image;					// vectorized multi-segment strokes with a width of one pixel, black color
	 QImage styleUsrDrawingImage;	// user's stylish drawing
	 QImage tempImage;				//temp draw area
	 QImage backImage,resultImage;	// background image used for hint,resultImage store the final composition result
	 QImage gridImage;				// the image containing only the grid lines
	 QImage threedpose;				// the 3d character pose image
	 QImage threedline;				// the 3d line used for image matching
	 QImage threedstyleline;		// the 3d line used for displaying
	 QImage blackBackgroundImage;	// the black background image 
     QRgb backColor;				// QRgb color, store the color of canvas background
	 QPoint point;					// the mouse position
	 vector<QPoint> m_vPoints;		// the 2d points which mouse moves along
	 vector< vector<QPoint> > m_vHistoryPoints;	// record each user's sketch
	 vector<vector<double>> m_v3DPoints;	// one 3d line drawn by users
	 vector<vector<vector<double>>> m_vMulti3DLines;	// multi 3d lines
	 bool m_bShowGrid;				// whether drawing the grid
	 bool m_bShowSketch;			// whether drawing the sketch
	 bool m_bShowShadow;			// whether drawing the shadow
	 bool bContentChanged;			// whether the content has been changed
	 bool m_isPatchChanged[_PATCH_NUM][_PATCH_NUM];
	 CSAImageProcess imageprocess;	// used for image processing
	 QString m_sDBDir;				// The path for current selected database
	 QTimer* m_pTimer;				// Timer which is used for dynamically updating shadow
	 DrawMode m_DrawMode;			// The current drawing mode
	 QPixmap m_eraserPixmap;		// the eraser pixmap
	 QPixmap m_penPixmap;			// the pen pixmap
	 QCursor m_cursor;				// the cursor
	
	 // view associated
	 CSAView* m_pViewArray[20];		// the view array
	 MathLib::Vector3d m_vVec3d[20];// indicate the camera position in 3d model space
	 int m_iCurrenView;				// the current view index
	 int m_iViewNum;				// the total view number
	 
	 // associated with ground render
	 QImage groundImage;			// the ground texture image
	 GLuint ground_texture[1];		// texture object

	 // whether key in the keyboard is pressed
	bool m_bInitMousePosition;		// whether the position of the mouse is initialized

	// indicate which mouse button is pressed down, used for viewing 3d scene
	bool m_bLeftButtonDown;
	bool m_bRightButtonDown;
	bool m_bMidButtonDown;

	int m_iPickX;					// the mouse position x for selection
	int m_iPickY;					// the mouse position y for selection
	int m_iAxisIdx;					// indicate which rotation circle is selected, 1 red, 2 green, 3 blue

	// draw the default character
	QColor defaultColor; 

	// indicate whether this is the first start up for this program
	bool m_bStartUp;

	// projection matrix
	double m_vProjectionMatrix[16];
	// model-view matrix
	double m_vModelViewMatrix[16];
	// viewport
	GLint m_vViewPort[4];

	// index of joint chains to be visualized
	vector<vector<string>> m_vChainName;
	
	// sketched limbs
	vector<string> vSketchedBones;

	// indicate whether the result has been refined
	bool m_bRefined;
	
	// the distance between the nearest neighbor and sketch lines
	double m_dDist;

	// the image feature computed by current sketch line
	vector<Mat> m_vSketchHist;
	vector<Posture> m_vLocalNNPosture;

	// record the time used for each sketch
	QTime m_TimerSketch;
	vector<double> m_vElapsedTimeForEachSketch;

	// indicate sketch animation or pose
	bool m_bSketchPose;		// true is pose, false is animation

	// the feature for trajectories
	vector< vector<CvPoint2D32f> > m_TrajectoryFeature;
	int m_iShowTrajectoryIndex;

	// alignment the user's sketching with character's pose
	QPoint m_Translation;

public:
	 BVH*	 m_pBvh;						// BVH file
	 CSACamera m_Camera;					// the camera used for OpenGL transformation from database
	 CSACamera m_DefaultCamera;				// default camera
	 bool	 m_b3DPose;						// show 3d pose
	 vector< vector<float> > m_vColormap;	// the color map
	 int m_iDisplayNearestNum;				// the display number of nearest neighbor
	 vector<int> imgindex;					// the candidate image index and also the frame index in mocap data
	 vector<int> motion_clip_index;			// the candidate motion clip index

private:
     void paint(QImage &theImage, const QPen& pen);
	 
	 /*
		judge whether which patch is changed
	 */
	 void patchChanged2(int x, int y);
	 
	 /*
		compute the new candidate sets
	 */
	 void updateCandidateSets();

 	 /*
		compute the new candidate sets
	 */
	 void updateCandidateSets2(const QImage& queryImage, const QImage& currentSketchImage);

	 /*
		compute the new candidate animation sets
	 */
	 void updateCandidateAnimationSets(const vector<CvPoint2D32f>& sketch_curve_feature);

	 /*
		set up viewport
	 */
	 void setupViewport(int width, int height);
 	
	 /*
		load mocap data
	 */
	 void loadMocapData(const char* filename,const char* type,int* p_start = NULL, int* p_end = NULL);
	
	 /*
	 	reconstruct 3d line
	 */
	 void from2Dto3DLine(float* depthdata);

	 /*
		draw the ground
	 */
	 void drawGround();

	 /*
		construct default camera
	 */
	 void constructDefaultCamera();

	 /*
		draw shadow
	 */
	 void drawShadow();

	 /*
		draw grid image
	 */
	 void drawGridImage(QImage& image);

	 /*
		draw sketching pose interface
	 */
	void drawSketchingPoseInterface();

	/*
		draw sketching animation interface
	*/
	void drawSketchingAnimationInterface();
	 /*
		draw the navigation ball
	 */
	 void drawNavigationBall();

	 /*
		draw circle using an efficient way
	 */
	 void drawCircle(float cx, float cy, float r, int num_segments);

	 /*
		draw the box
	 */
	 void drawBox(float edgelength);

	 /*
		visualize the joint position using probability, 	
	 */
	 void visualizeJointsInBoundingBox(float* boundingbox,int num, float* data, float* joint_color, float* interp_color);

	 /*
		process the selection event
	 */
	 void processSelection(int hits,unsigned int* buffer);

	 /*
		extract the feature of motion curves presented by 2D image points
	 */
	 void extractMotionFeature(const vector<QPoint>&, vector<CvPoint2D32f>&);

	 /*
		compute the translation between certain joint and user's sketching
	 */
	 QPoint computeTranslationBetweenJointAndSketching();

 public:
	 bool isContentChanged();
	 void setContentChanged(bool);
	 void resetContent();
	 void resetPatchChangedFlag();
	 QImage& getCurrentCanvas();
	 QImage& getFinalCanvas();
	 void setCurrentCanvas(QImage);
	 // whether show the grid
	 void setShowGrid(bool show){m_bShowGrid = show;update();}
	 // whether show the sketch
	 void setShowSketch(bool show){m_bShowSketch = show;update();}
	 // whether show the shadow
	 void setShowShadow(bool show){m_bShowShadow = show;update();}
	 // QImage(ARGB) converted to OpenCV mat (BGR)
	 Mat qimage2mat(const QImage&);
	 // OpenCV mat (BGR) converted to QImage (ARGB)
	 QImage mat2qimage(const Mat&);
	 
	 // set the database dir
	 void setDBDir(QString);
	 
	 // set the database dir and update the database hierarchy using the actual data
	 void setDBDir(const vector<QString>&, vector<DatabaseUnit>&);

	 /*
		start the timer
	 */
	 void startTimer();

	 /*
		stop the timer
	 */
	 void stopTimer();
	 
	 /*
		set the drawing mode
	 */
	 void setDrawingMode(DrawMode mode);
	 
	 /*
		set the flag whether to show 3d pose
	 */
	 void setShow3DPose(bool show);
	 
	 /*
		set the flag to indicate whether the program is in default status
	 */
	 void setDefaultStatus(bool status);
	 
	 /*
		visualize the 3d histogram
	 */
	 void histVisualize(bool sketch,int imgidx = 0, int patchidx = 0); 

	 /*
		change the view
	 */
	 void changeView(int index);

	 /*
		set behavior mode
	 */
	 void setBehaviorMode(BehaviorMode bm){behaviorMode = bm; repaint();}
	 
	 /*
		visualize different body parts
	 */
	 void visualizeBodyParts(int chainIdx);

	 /*
		visualize sample points
	 */
	 void visualizeSamplePoints(float*, vector<MathLib::Vector3d>&);

	 /*
		refine the final result by sampling
	 */
	 void refineResult();

 	 /*
		refine the final result by sampling
	 */
	 void refineResult2();

  	 /*
		refine the final result by sampling
	 */
	 void refineResult3();

	 /*
		compute the recall ratio
	 */
	 void computeRecallRatioAndMQR();

	 /*
		replay the process of user's sketching
	 */
	 void replay();

	 /*
		set to sketch pose or animation
	 */
	 void setToSketchPoseOrAnimation(bool t_){m_bSketchPose = t_;update();}

	 /*
		load motion clips from file
	 */
	 void loadMotionClipsFromFile(char* filename);

	 /*
		select certain joint to show its trajectory
	 */
	 void showJointTrajectory(int index);

signals:
	void loadImages(const char* folder,int* ,double*,int size);
	void loadImages(const char* folder,int* ,double*,int size,bool bkg);
	void loadImages(const char* folder,const vector<int>& file,const vector<vector<int>>& nearestpatchidx,const vector<double>& color);
	void loadImages(const char* folder, const vector<int>& imageidx, vector<Mat>& images);
	void loadImages(const char* folder,const vector<int>& file,const vector<double>& color,const vector<vector<double>>& transform);

	/*
		Inform MainWindow that the view changes
	*/
	void switchViewTo(int index);

/************************************************************************/
/*  asf/amc associated file                                             */
/************************************************************************/

public:
	Skeleton* m_pSkeleton;

private:
	Motion* m_pMotion;

	int m_iSketchOrder;

	/*
		the closest sampling posture
	*/
	Posture m_SampleClosestPosture;

	/*
		using the child value to update the parent value
	*/
	void updateDabaseHierarchy(const DatabaseUnit& dbu,vector<DatabaseUnit>& databaseHierarchy);

	/*
		the motion clips
	*/
	vector<CMEMotionClip*> m_vMotionClip;

public:
	// refine the final result by particle swarm optimization
	void refineResultbyPSO(void);

	// test new function
	void test();

signals:
	void sendNewMotion(vector<Posture>&);
};

#endif // RENDERAREA_H
