#include "mainwindow_m.h"
#include "ui_mainwindow.h"
#include <Qpen>
#include <QImage>
#include <QImageReader>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <iostream>
#include <QPainter>
#include <fstream>
#include <vector>
#include <QDebug>
#include <QContextMenuEvent>
#include <QSettings>
#include "csanearestnum.h"
#include "SAImageProcess.h"
#include "csacustomgraphicspixmapitem.h"
#include "asf/skeleton.h"
#include <opencv2/opencv.hpp>

#include "TreeModel.h"
#include <QAbstractItemView>

using namespace std;
using namespace cv;

const float SKELETON_SCALE = 0.3;

MainWindow_M::MainWindow_M(QWidget* parent /* = 0 */)
	:QMainWindow(parent)
	,strCurrentFileName("")
	,m_pSAEncodeImg(NULL)
	,m_iDisplayNearestNum(50)
	,m_bShowOriginalImg(false)
	,m_pTreeModel(0x0)
{
	//setMaximumSize(1440, 900);
	QPalette current_palette = palette();
	current_palette.setColor(QPalette::Window, Qt::lightGray);
	current_palette.setColor(QPalette::WindowText, Qt::white);
	current_palette.setColor(QPalette::Button, Qt::darkGray);
	setPalette(current_palette);

	createActions();
	createMenu();
	createWidgets();
	createToolBar();
	createStatusBar();
	createDockWidget();
	createConnects();

	Qt::WindowFlags flags = 0;
	flags |= Qt::WindowMinimizeButtonHint;
	flags |= Qt::WindowMaximizeButtonHint;
	setWindowFlags(flags);

	connect(renderarea, SIGNAL(loadImages(const char*, int*, double*, int, bool)), this, SLOT(importFile(const char*, int*, double*, int, bool)));
	connect(renderarea, SIGNAL(switchViewTo(int)), this, SLOT(changeView(int)));

	clearTempFiles();

	renderarea->setDrawingMode(PEN_MODE);

	loadColormap();

	//initialize the variables in RenderArea class
	renderarea->m_vColormap = colormap;
	renderarea->m_iDisplayNearestNum = m_iDisplayNearestNum;
}

MainWindow_M::~MainWindow_M()
{
	writeSettings();
	for (int i = 0; i < m_pSAEncodeImgArr.size(); ++i)
	{
		delete m_pSAEncodeImgArr[i];
		m_pSAEncodeImgArr[i] = NULL;
	}
	m_pSAEncodeImg = NULL;
}
	/*----------Public Fun----------*/
void MainWindow_M::createMenu()
{
	file_menu = menuBar()->addMenu("&File");
	file_menu->addAction(new_action);
	file_menu->addAction(open_action);
	file_menu->addAction(import_action);
	file_menu->addAction(save_action);
	file_menu->addAction(saveAs_action);
	file_menu->addAction(exit_action);

	view_menu = menuBar()->addMenu("&View");
	view_menu->addAction(showGrid_action);
	view_menu->addAction(showSketch_action);
	view_menu->addAction(showShadow_action);
	view_menu->addAction(showOriginalImage_action);

	tools_menu = menuBar()->addMenu("&Tools");
	tools_menu->addAction(encodeImageFiles_action);
	tools_menu->addAction(extractEdgeImage_action);
	tools_menu->addAction(computeContour_action);
	tools_menu->addAction(test_action);

	setting_menu = menuBar()->addMenu("&Settings");
	setting_menu->addAction(setDisplayNearestNum_action);
	setting_menu->addAction(setWorkingDirectionary_action);

	mode_menu = menuBar()->addMenu("&Mode");
	mode_menu->addAction(sketching_action);
	mode_menu->addAction(viewing_action);

	window_menu = menuBar()->addMenu("&Window");
	window_menu->addAction(timeLine_action);
}

void MainWindow_M::createWidgets()
{
	renderarea = new RenderArea;
	animationWindow = new SkeletonView(this);
	mdi = new QMdiArea;

	QMdiSubWindow* animation_subwindow = new QMdiSubWindow;
	QMdiSubWindow* renderarea_subwindow = new QMdiSubWindow;
	renderarea_subwindow->setWidget(renderarea);
	animation_subwindow->setWidget(animationWindow);
	mdi->addSubWindow(animation_subwindow);
	mdi->addSubWindow(renderarea_subwindow);

	renderarea_subwindow->setParent(animation_subwindow);
	renderarea_subwindow->setWindowOpacity(1);

	renderarea_subwindow->setGeometry(0, 0, 495, 520);
	renderarea_subwindow->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
	animation_subwindow->setGeometry(0, 0, 935, 635);
	animation_subwindow->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

	Qt::WindowFlags flags = 0;
	//flags |= Qt::WindowMinimizeButtonHint;
	//flags |= Qt::WindowMaximizeButtonHint;
	renderarea_subwindow->setWindowFlags(flags);
	animation_subwindow->setWindowFlags(flags);

	animation_subwindow->showMaximized();

	setCentralWidget(mdi);

	QPixmap pen_pixmap;
	QPixmap eraser_pixmap;
	QPixmap analysisImage_pixmap;
	QPixmap dynamichint_pixmap;
	QPixmap show3D_pixmap;
	pen_pixmap.load(".\\Resources\\cursor\\pen.png", "png");
	eraser_pixmap.load(".\\Resources\\cursor\\eraser.png", "png");
	analysisImage_pixmap.load(".\\Resources\\cursor\\analysis.png", "png");
	dynamichint_pixmap.load(".\\Resources\\cursor\\dynamichint.png", "png");
	show3D_pixmap.load(".\\Resources\\cursor\\3d.png", "png");
	pen_act = new QAction(pen_pixmap, "Pen", this);
	eraser_act = new QAction(eraser_pixmap, "Eraser", this);
	analysisImage_act = new QAction(analysisImage_pixmap, "Analysis", this);
	dynamichint_act = new QAction(dynamichint_pixmap, "DynamicHint", this);
	show3D_act = new QAction(show3D_pixmap, "Show3D", this);
	pen_act->setCheckable(true);
	eraser_act->setCheckable(true);
	analysisImage_act->setCheckable(true);
	dynamichint_act->setCheckable(true);
	show3D_act->setCheckable(true);

	dataTreeView = new DataBaseTreeView;
	pose_button = new QRadioButton("Pose");
	animation_button = new QRadioButton("Animation");
	view_slider = new QSlider(Qt::Horizontal);
	refine_button = new QPushButton("Refine");
	playAnimation_button = new QPushButton("Play");
	selectTraj_combo = new QComboBox;
	selectTraj_combo->addItem("torso");
	selectTraj_combo->addItem("l_hand");
	selectTraj_combo->addItem("r_hand");
	selectTraj_combo->addItem("l_foot");
	selectTraj_combo->addItem("r_foot");
	replay_button = new QPushButton("Replay");
	timeline_slider = new Slider(70);
}

void MainWindow_M::createStatusBar()
{
	statusBar()->showMessage("Ready!");
}

void MainWindow_M::createToolBar()
{
	QToolBar* toolBar = new QToolBar;
	addToolBar(Qt::LeftToolBarArea, toolBar);
	toolBar->addAction(pen_act);
	toolBar->addAction(eraser_act);
	toolBar->addAction(analysisImage_act);
	toolBar->addAction(dynamichint_act);
	toolBar->addAction(show3D_act);
}

void MainWindow_M::createActions()
{
	new_action = new QAction("New", this);
	new_action->setShortcut(QKeySequence::New);

	open_action = new QAction("Open", this);
	open_action->setShortcut(QKeySequence::Open);

	import_action = new QAction("Import", this);
	import_action->setShortcut(Qt::CTRL + Qt::Key_I);

	save_action = new QAction("Save", this);
	save_action->setShortcut(QKeySequence::Save);

	saveAs_action = new QAction("Save as", this);
	saveAs_action->setShortcut(QKeySequence::SaveAs);

	exit_action = new QAction("Exit", this);
	exit_action->setShortcut(QKeySequence::Quit);

	showGrid_action = new QAction("Show grid", this);
	showSketch_action = new QAction("Show sketch", this);
	showShadow_action = new QAction("Show shadow", this);
	showOriginalImage_action = new QAction("Show original image", this);

	encodeImageFiles_action = new QAction("Encode image files", this);
	extractEdgeImage_action = new QAction("Extract edge image", this);
	computeContour_action = new QAction("Compute contour", this);
	test_action = new QAction("Test", this);

	setDisplayNearestNum_action = new QAction("Set dispay nearest num", this);
	setWorkingDirectionary_action = new QAction("Set working directionary", this);

	sketching_action = new QAction("Sketching", this);
	viewing_action = new QAction("Viewing", this);

	timeLine_action = new QAction("Timeline", this);
}

void MainWindow_M::createDockWidget()
{
	QDockWidget* bottomDock = new QDockWidget;
	QWidget* widget = new QWidget;
	QHBoxLayout* mlayout = new QHBoxLayout;
	QGridLayout* glayout = new QGridLayout;

	glayout->addWidget(pose_button, 0, 0, 1, 1);
	glayout->addWidget(refine_button, 0, 1, 1, 1);
	glayout->addWidget(replay_button, 0, 2, 1, 1);

	glayout->addWidget(animation_button, 1, 0, 1, 1);
	glayout->addWidget(playAnimation_button, 1, 1, 1, 1);
	glayout->addWidget(selectTraj_combo, 1, 2, 1, 1);

	mlayout->addLayout(glayout);
	mlayout->addWidget(timeline_slider);

	widget->setLayout(mlayout);
	bottomDock->setWidget(widget);
	addDockWidget(Qt::BottomDockWidgetArea, bottomDock);
}

void MainWindow_M::createConnects()
{
	/*----------Menu Connect----------*/
	connect(new_action, SIGNAL(triggered()), this, SLOT(newCanvas()));
	connect(open_action, SIGNAL(triggered()), this, SLOT(openFile()));
	connect(import_action, SIGNAL(triggered()), this, SLOT(importFile()));
	connect(save_action, SIGNAL(triggered()), this, SLOT(saveCanvas()));
	connect(saveAs_action, SIGNAL(triggered()), this, SLOT(saveAsCanvas()));
	connect(exit_action, SIGNAL(triggered()), this, SLOT(exitProgram()));

	connect(showGrid_action, SIGNAL(triggered()), this, SLOT(showGrid()));
	connect(showSketch_action, SIGNAL(toggled(bool)), this, SLOT(showSketch(bool)));
	connect(showShadow_action, SIGNAL(toggled(bool)), this, SLOT(showShadow(bool)));
	connect(showOriginalImage_action, SIGNAL(triggered()), this, SLOT(showOriginalImage()));

	connect(encodeImageFiles_action, SIGNAL(triggered()), this, SLOT(encodeImageFiles()));
	connect(extractEdgeImage_action, SIGNAL(triggered()), this, SLOT(extractEdgeImage()));
	connect(computeContour_action, SIGNAL(triggered()), this, SLOT(computeContour()));
	connect(test_action, SIGNAL(triggered()), this, SLOT(test()));

	connect(setDisplayNearestNum_action, SIGNAL(triggered()), this, SLOT(setDisplayNearestNum()));
	connect(setWorkingDirectionary_action, SIGNAL(triggered()), this, SLOT(setWorkingDirectionary()));

	connect(sketching_action, SIGNAL(triggered()), this, SLOT(sketching()));
	connect(viewing_action, SIGNAL(triggered()), this, SLOT(viewing()));

	connect(timeLine_action, SIGNAL(triggered()), this, SLOT(timeLine()));

	/*----------Widget Connect----------*/
	connect(pose_button, SIGNAL(toggled(bool)), this, SLOT(setToSketchPose(bool)));
	connect(animation_button, SIGNAL(toggled(bool)), this, SLOT(setToSketchAnimation(bool)));
	connect(analysisImage_act, SIGNAL(toggled(bool)), this, SLOT(analysisImage()));
	connect(dynamichint_act, SIGNAL(triggered(bool)), this, SLOT(dynamicHint(bool)));
	connect(pen_act, SIGNAL(triggered(bool)), this, SLOT(changeToPenMode(bool)));
	connect(eraser_act, SIGNAL(triggered(bool)), this, SLOT(changeToEraserMode(bool)));
	connect(view_slider, SIGNAL(valueChanged(int)), this, SLOT(change3DView(int)));
	connect(show3D_act, SIGNAL(triggered(bool)), this, SLOT(show3DPose(bool)));
	connect(refine_button, SIGNAL(triggered()), this, SLOT(refineFinalResultBySampling()));
	connect(playAnimation_button, SIGNAL(triggered()), this, SLOT(playAnimation()));
	connect(selectTraj_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(selectTrajectory(int)));
	connect(replay_button, SIGNAL(triggered()), this, SLOT(replay()));
	connect(animationWindow, SIGNAL(updateStep(int)), timeline_slider, SLOT(setValue(int)));
}

void MainWindow_M::clearTempFiles()
{
	QDir dir(".\\tmp");
	QFileInfoList list = dir.entryInfoList();
	for(int i = 0; i < list.size(); i++)
	{
		dir.remove(list[i].fileName());
	}
}

	/*----------End public Fun----------*/


	/*----------Action slots----------*/

void MainWindow_M::newCanvas()
{
	renderarea->resetContent();
	clearTempFiles();
}

void MainWindow_M::openFile()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"./",tr("Images (*.png)"));
	if(fileName.isEmpty())
		return;
	QFile file(fileName);
	file.read(QIODevice::ReadOnly);
	QImageReader imageReader;
	imageReader.setDevice(&file);
	renderarea->setCurrentCanvas(imageReader.read());
	renderarea->setContentChanged(false);

	statusBar()->showMessage(fileName.section('/', -1));
	clearTempFiles();

	strCurrentFileName = fileName;
}

void MainWindow_M::importFile()
{

}

void MainWindow_M::importFile(const char* folder,int* file,double* color,int size,bool bkg)
{
	// if CSAEncodeImages object not exist, create it
	if(m_pSAEncodeImg == NULL)
	{
		qDebug()<<"loading the encoded file";

		QString fileName = folder;
		//fileName += "\\"+fileName.section('\\',-1) + ".sa";
		fileName += "\\"+fileName.section('\\',-1) + ".data";

		m_pSAEncodeImg = new CSAEncodeVectorImages();
		m_pSAEncodeImg->loadFromFile(fileName.toAscii());

		qDebug()<<"encoded file loaded";
	}

	// clear the graphics scene
	m_graphicsScene.clear();

	// draw the shadow
	QImage resultImage,tmp;
	resultImage = QImage(480,480,QImage::Format_ARGB32);
	resultImage.fill(qRgba(255,255,255,0));
	tmp = QImage(120,120,QImage::Format_ARGB32);

	int displaynum = min(m_iDisplayNearestNum,size);
	
	if(renderarea->m_pSkeleton)
	{
		Skeleton* pSkeleton = renderarea->m_pSkeleton;

		// Nearest neighbor in graphics view
		for(int i = 0; i < displaynum /*&& i < 20*/; i++)
		{
			tmp.fill(qRgba(255,255,255,0));
			int ix = (displaynum - i) * (colormap.size() - 1) / displaynum;

			// use the colormap to describe the probability
			QColor color2(colormap[ix][0] * 255,colormap[ix][1] * 255,colormap[ix][2] * 255);
			//QColor color2(colormap[colormap.size()-1-i*3][0] * 255,colormap[colormap.size()-1-i*3][1] * 255,colormap[colormap.size()-1-i*3][2] * 255);
			
			QPainter painter(&tmp);

			painter.setPen(color2);
			
			for(int j = 1; j < m_pSAEncodeImg->m_sJointName.size(); j++)
			{
				string child_name = m_pSAEncodeImg->m_sJointName[j];
				Bone* pBone = NULL;
				pBone = pSkeleton->findBoneByName(pSkeleton->getRoot(),child_name.c_str());
				string parent_name = pBone->parent_name;

				int k = 0;
				for(; k < m_pSAEncodeImg->m_sJointName.size(); k++)
				{
					if(parent_name == m_pSAEncodeImg->m_sJointName[k])
						break;
				}

				painter.drawLine(m_pSAEncodeImg->m_iJointPosition[file[i]][k*2]/4,m_pSAEncodeImg->m_iJointPosition[file[i]][k*2+1]/4,m_pSAEncodeImg->m_iJointPosition[file[i]][j*2]/4,m_pSAEncodeImg->m_iJointPosition[file[i]][j*2+1]/4);
			}
			painter.end();
			
			QPixmap pixmap;
			pixmap.convertFromImage(tmp);
			CSACustomGraphicsPixmapItem* graphicspixmapitem = new CSACustomGraphicsPixmapItem(&m_graphicsScene);
			graphicspixmapitem->setPixmap(pixmap);
			graphicspixmapitem->setPos(4 + i%2*124,4 + i/2*124);
			graphicspixmapitem->setImageIndex(file[i]);
		}
	}

	// background images
	//for(int i = 0; i < displaynum; i++)
	//{
	//	QColor color(250-i*100.0/(displaynum-1),250-i*100.0/(displaynum-1),250-i*100.0/(displaynum-1));
	//	for(int j = 0; j < m_pSAEncodeImg->m_sJointName.size(); j += 2)
	//	{
	//		int m = m_pSAEncodeImg->m_iJointPosition[file[displaynum-1-i]][j]; 
	//		int n = m_pSAEncodeImg->m_iJointPosition[file[displaynum-1-i]][j+1];

	//		resultImage.setPixel(m,n,color.rgb());
	//	}
	//}
	//*/
	if(bkg)
		renderarea->setCurrentCanvas(resultImage);
}

void MainWindow_M::saveCanvas()
{
	QImage image = renderarea->getFinalCanvas();
	if (renderarea->isContentChanged())
	{
		if (strCurrentFileName.isEmpty())
		{
			QMessageBox msgBox;
			msgBox.setText("The content has been modified.");
			msgBox.setInformativeText("Do you want to save your change?");
			msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
			msgBox.setDefaultButton(QMessageBox::Save);
			int ret = msgBox.exec();
			switch (ret)
			{
			case QMessageBox::Save:
				{
					QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),"./",tr("Images (*.png)"));
					if(!fileName.isEmpty())
						strCurrentFileName = fileName;
					break;
				}
			case QMessageBox::Discard:
				{
					renderarea->setContentChanged(false);
					break;
				}
			case QMessageBox::Cancel:
				{
					break;
				}
			default:
				break;
			}
		}
		QFile file(strCurrentFileName);
		file.open(QIODevice::WriteOnly);
		image.save(&file);
		renderarea->setContentChanged(false);
	}
}

void MainWindow_M::saveAsCanvas()
{
	//pass
}

void MainWindow_M::exitProgram()
{
	close();
}

void MainWindow_M::showGrid()
{
	if (showGrid_action->isChecked())
	{
		renderarea->setShowGrid(true);
	}
	else
	{
		renderarea->setShowGrid(false);
	}
	
}

void MainWindow_M::showSketch(bool _on)
{
	renderarea->setShowSketch(_on);
}

void MainWindow_M::showShadow(bool _on)
{
	renderarea->setShowShadow(_on);
}

void MainWindow_M::showOriginalImage()
{
	if (showOriginalImage_action->isChecked())
	{
		m_bShowOriginalImg = true;
	}
	else
	{
		m_bShowOriginalImg = false;
	}
	
}

void MainWindow_M::encodeImageFiles()
{
	QString path = QFileDialog::getExistingDirectory(this, tr("Open Directory"),".\\database", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	if(path == "")
		return;
	QDir dir(path);
	if(!dir.exists())
		return;

	// Save the patch descriptor, edge magnitude & edge orientation
	vector<Mat> bin_db;
	QString filename = path.section('\\',-1);
	QString histfile,mfile,ofile,decfile;
	histfile = path + "\\" + filename + "_hist.xml";
	mfile = path + "\\" + filename + "_m.xml";
	ofile = path + "\\" + filename + "_o.xml";	
	decfile = path + "\\" + filename + "_dec.xml";

	//QString vecfile;
	//vecfile = path + "\\" + filename + ".txt";

	CSAImageProcess imgprocess;
	CSAView* pView = new CSAView();
	imgprocess.setView(pView);
	imgprocess.batch_process_db(path.toAscii(),bin_db,histfile.toAscii(),mfile.toAscii(),ofile.toAscii(),decfile.toAscii(),ARP);
	//imgprocess.batch_process_db(path.toAscii(),bin_db,histfile.toAscii(),mfile.toAscii(),ofile.toAscii(),decfile.toAscii(),vecfile.toAscii(),HOUGH);
	delete pView;
	qDebug()<<"database images processing completed";
}

void MainWindow_M::extractEdgeImage()
{
	QString path = QFileDialog::getExistingDirectory(this, tr("Open Directory"),".\\database", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	if(path == "")
		return;
	QDir dir(path);
	if(!dir.exists())
		return;

	QString eifilename = path.section('\\',-1);
	eifilename = path + "\\" + eifilename + ".xml";

	// convert the fileName from QString to char*
	char file[256];
	int len = 0;
	for(;len < eifilename.length(); len++)
	{
		file[len] = (eifilename.at(len)).toAscii();
	}
	file[len] = '\0';

	// store this edge image
	FileStorage fs(file,FileStorage::WRITE);
	if(!fs.isOpened())
	{
		return;
	}

	CSAImageProcess imageProcess;
	QImage srcImage;
	QString fileName;
	dir.setFilter(QDir::Files | QDir::NoSymLinks);

	QFileInfoList list = dir.entryInfoList();
	int i = 0;
	while(i < list.size())
	{
		QFileInfo fileInfo = list.at(i++);
		fileName = fileInfo.fileName();
		if(fileName.section('.',-1) != "png" && fileName.section('.',-1) != "PNG")
			continue;
		fileName = fileInfo.path() + "/"+fileName;

		// convert the fileName from QString to char*
		char file[256];
		int len = 0;
		for(;len < fileName.length(); len++)
		{
			file[len] = (fileName.at(len)).toAscii();
		}
		file[len] = '\0';

		// read the image
		Mat mat = imread(file,CV_LOAD_IMAGE_COLOR);

		Mat edgelength,edgeorientation,m,m_n,edgeresponse;
		imageProcess.edgeextraction(mat,edgelength,edgeorientation,m,m_n,edgeresponse);

		// save this edge image
		char name[256];
		sprintf(name,"edgeimage_%d",i);
		fs<<name<<edgelength;
	}
	fs.release();
}

void MainWindow_M::computeContour()
{
		// load the final blended edge image
	Mat edgeimg,edgeimg_singlechannel;
	edgeimg = renderarea->qimage2mat(renderarea->getFinalCanvas());

	// convert color image to gray image
	cvtColor(edgeimg,edgeimg_singlechannel,CV_RGB2GRAY);

	Mat edgeimg_float;
	edgeimg_singlechannel.convertTo(edgeimg_float,CV_32F,1.0/255);

	// store this edge image
	FileStorage fs("blend_image_continuous.xml",FileStorage::WRITE);
	if(!fs.isOpened())
	{
		return;
	}

	vector<vector<cv::Point>> contours;
	vector<Vec4i> hierarchy;

	// find the contours
	findContours(edgeimg_singlechannel,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,cv::Point(0,0));

	// approximate contours to polygons & get bounding rects
	vector<vector<cv::Point>> contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());

	for(int i = 0; i < contours.size(); i++)
	{
		boundRect[i] = boundingRect(Mat(contours[i]));
	}

	cv::Point tl,br;
	tl.x = edgeimg.rows - 1;
	tl.y = edgeimg.cols - 1;
	br.x = 0;
	br.y = 0;
	for(int i = 0; i < contours.size(); i++)
	{
		cv::Point tl_tmp = boundRect[i].tl();
		cv::Point br_tmp = boundRect[i].br();

		if(tl_tmp.x < tl.x)
			tl.x = tl_tmp.x;
		if(tl_tmp.y < tl.y)
			tl.y = tl_tmp.y;
		if(br_tmp.x > br.x)
			br.x = br_tmp.x;
		if(br_tmp.y > br.y)
			br.y = br_tmp.x;
	}

	// the interested area
	Mat interpArea = edgeimg_float(Range(tl.y,br.y+1),Range(tl.x,br.x+1));

	fs<<"interArea_original"<<interpArea;

	double minVal,maxVal;
	minMaxLoc(interpArea,&minVal,&maxVal);

	float thresh = 0.1;
	// find where the array has a value above threshold
	vector<vector<bool>> hasWeight;
	hasWeight.resize(interpArea.rows);
	for(int i = 0; i < hasWeight.size(); i++)
		hasWeight[i].assign(interpArea.cols,false);

	for(int m = 0; m < interpArea.rows; m++)
	{
		for(int n = 0; n < interpArea.cols; n++)
		{
			if(interpArea.at<float>(m,n) > thresh)
				hasWeight[m][n] = true;
		}
	}

	float sigma = 5.0;
	for(int m = 0; m < interpArea.rows; m++)
	{
		for(int n = 0; n < interpArea.cols; n++)
		{
			if(hasWeight[m][n])
			{
				float intensity = interpArea.at<float>(m,n); 

				for(int r = -20; r <= 20; r++)
				{
					for(int c = -20; c <= 20; c++)
					{
						if(m + r >= 0 && m + r < interpArea.rows && n + c >= 0 && n + c < interpArea.cols)
						{
							if(!hasWeight[m+r][n+c])
								interpArea.at<float>(m + r,n + c) += intensity  * exp(-(r * r + c * c)/(2 * sigma * sigma));
						}
					}
				}
			}
		}
	}
	fs<<"interArea_interp"<<interpArea;

	double minValNew,maxValNew;
	minMaxLoc(interpArea,&minValNew,&maxValNew);

	double rangeVal[2];
	if(minVal > minValNew)
		rangeVal[0] = minValNew;
	else
		rangeVal[0] = minVal;
	if(maxVal < maxValNew)
		rangeVal[1] = maxValNew;
	else
		rangeVal[1] = maxVal;

	for(int m = 0; m < interpArea.rows; m++)
	{
		for(int n = 0; n < interpArea.cols; n++)
		{
			if(!hasWeight[m][n])
			{
				//interpArea.at<float>(m,n) = (interpArea.at<float>(m,n) - rangeVal[0])/(rangeVal[1] - rangeVal[0]) * 0.1;
				interpArea.at<float>(m,n) = (interpArea.at<float>(m,n) - rangeVal[0])/(rangeVal[1] - rangeVal[0]);
			}
		}
	}
	fs<<"interArea_norm"<<interpArea;
	//*/
	interpArea = interpArea * 1000;
	fs<<"interpArea"<<interpArea;

	imwrite("blend_result_continuous.png",interpArea);
	//*/
}

void MainWindow_M::test()
{
	renderarea->test();
}

void MainWindow_M::setDisplayNearestNum()
{
	CSANearestNum nndlg;
	nndlg.setNearestNum(m_iDisplayNearestNum);
	nndlg.exec();
	m_iDisplayNearestNum = nndlg.getNearestNum();

	// update the display number of nearest neighbor in RenderArea
	renderarea->m_iDisplayNearestNum = m_iDisplayNearestNum;
}

void MainWindow_M::setWorkingDirectionary()
{
	QString path = QFileDialog::getExistingDirectory(this, tr("Open Directory"),".\\database", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	if(path == "")
		return;
	strCurrentDBDir = path;
	strCurrentDBDir.replace('\\',"/");
	qDebug()<<"The default database directory is set to "<<path;

	// store the root dir
	DatabaseUnit dbu;
	dbu.parent_type = "";
	dbu.type = strCurrentDBDir.toStdString();
	databaseHierarchy.push_back(dbu);

	//added by zpy
	QString strConfigFile;
	TraversalDirectory(strConfigFile,baseDir,databaseHierarchy,strCurrentDBDir, 0);
	m_pTreeModel = new TreeModel(strConfigFile);

	dataTreeView->setModel(m_pTreeModel);
	dataTreeView->setSelectionMode(QAbstractItemView::MultiSelection);

	// load the encoded file
	for(int i = 0; i < baseDir.size(); i++)
	{
		//QString fileName = strCurrentDBDir;
		QString fileName = baseDir[i];
		//fileName += "/"+fileName.section('/',-1) + ".data";
		fileName +="/capture.data";

		CSAEncodeVectorImages* pSAEncodeVectorImages = new CSAEncodeVectorImages();
		loadEncodedFile(fileName.toAscii(),pSAEncodeVectorImages);

		if(i == 0)
		{
			// assign the first CSAEncodeVectorImages to m_pSAEncodeImg
			m_pSAEncodeImg = pSAEncodeVectorImages;

			m_pSAEncodeImgArr.push_back(m_pSAEncodeImg);
		}
		else
		{
			// copy the data into the m_pSAEncodeImg
			m_pSAEncodeImg->m_iImageNum += pSAEncodeVectorImages->m_iImageNum;

			m_pSAEncodeImg->m_iJointPosition.insert(m_pSAEncodeImg->m_iJointPosition.end(),pSAEncodeVectorImages->m_iJointPosition.begin(),pSAEncodeVectorImages->m_iJointPosition.end());

			delete pSAEncodeVectorImages;
		}
	}

	// set the db dir for RenderArea
	renderarea->setDBDir(baseDir,databaseHierarchy);
}

void MainWindow_M::sketching()
{
	sketching_action->setChecked(true);
	renderarea->setBehaviorMode(SKETCH);

	viewing_action->setChecked(false);
}

void MainWindow_M::viewing()
{
	viewing_action->setChecked(true);
	renderarea->setBehaviorMode(VIEW);

	sketching_action->setChecked(false);
}

void MainWindow_M::timeLine()
{
	//pass
}

	/*----------End Action Slots----------*/

	/*----------Widget Slots----------*/

void MainWindow_M::dynamicHint(bool _on)
{
	dynamichint_act->setCheckable(true);
	if(_on)
	{
		renderarea->startTimer();
	}
	else
	{
		renderarea->stopTimer();
	}
}

void MainWindow_M::setToSketchPose(bool _on)
{
	if (_on == true)
	{
		renderarea->setToSketchPoseOrAnimation(true);
	}
}

void MainWindow_M::setToSketchAnimation(bool _on)
{
	if (_on)
	{
		renderarea->setToSketchPoseOrAnimation(false);
	}
}

void MainWindow_M::changeToPenMode(bool _on)
{
	eraser_act->setChecked(false);
	pen_act->setCheckable(true);
	renderarea->setDrawingMode(PEN_MODE);
}

void MainWindow_M::changeToEraserMode(bool _on)
{
	pen_act->setChecked(false);
	eraser_act->setCheckable(true);
	renderarea->setDrawingMode(ERASER_MODE);
}

void MainWindow_M::change3DView(int angle)
{
	static double lastchange = 0.0;

	double rad = angle;
	{
		renderarea->m_Camera.m_rotations[0] += (rad - lastchange);

		lastchange = rad;

		renderarea->update();
	}
}

void MainWindow_M::show3DPose(bool _on)
{
	show3D_act->setCheckable(true);
	renderarea->setShow3DPose(_on);
}

void MainWindow_M::refineFinalResultBySampling()
{
	renderarea->refineResultbyPSO();
}

void MainWindow_M::playAnimation()
{
	animationWindow->startTimer();
}

void MainWindow_M::selectTrajectory(int index)
{
	renderarea->showJointTrajectory(index);
}

void MainWindow_M::replay()
{
	renderarea->replay();
}

void MainWindow_M::changeTimelineRangeAndInterval(int s, int e, int interval)
{
	timeline_slider->setRange(s, e);
	//timeline_slider->setTickInterval(interval);
}

	/*----------End Widget slots----------*/

void MainWindow_M::loadEncodedFile(const char* filename)
{

}

void MainWindow_M::loadEncodedFile(const char* filename, CSAEncodeVectorImages* pSAEncodeVectorImages)
{

}

void MainWindow_M::changeView(int index)
{
	if (index >= m_pSAEncodeImgArr.size())
		return;

	m_pSAEncodeImg = m_pSAEncodeImgArr[index];

	renderarea->changeView(index);
}

void MainWindow_M::loadColormap()
{
	// read the colormap file
	ifstream in;
	in.open(".\\Resources\\colormap.txt");
	if(in.is_open())
	{
		float r,g,b;
		in>>r;
		in>>g;
		in>>b;
		while(!in.eof())
		{
			vector<float> color_rgb;
			color_rgb.push_back(r);
			color_rgb.push_back(g);
			color_rgb.push_back(b);
			colormap.push_back(color_rgb);
			in>>r;
			in>>g;
			in>>b;
		}
	}
	else{
		qDebug()<<"colormap open failed";
	}
	in.close();
}

void MainWindow_M::writeSettings()
{
	QSettings settings("CAD&CG ZJU", "SketchAnimation");

	settings.setValue("db_dir", strCurrentDBDir);
}

void MainWindow_M::readSettings()
{
	QSettings settings("CAD&CG ZJU", "SketchAnimatin");
}

void MainWindow_M::TraversalDirectory(QString &strRes, vector<QString> &baseDir, vector<DatabaseUnit>& databaseHierachy ,const QString &strDir, int nDepth)
{
	QDir dir(strDir);
	if (!dir.exists())
	{
		return;
	}

	int nTemp = nDepth;
	while (nTemp--)
	{
		strRes.push_back(' ');
	}
	
	int i;
	for (i = strDir.size() - 1; i >= 0; --i)
	{
		if (strDir[i] == '\\' || strDir[i] == '/')break;		
	}

	strRes.push_back(strDir.right(strDir.size() - i - 1));
	strRes.push_back('\n');

	QStringList ChildList = QDir(strDir).entryList(QStringList(), QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);

	QString fullPath(QDir(strDir).absolutePath());

	if(ChildList.empty())
	{
		baseDir.push_back(fullPath);
	}
	else
	{
		for(i = 0 ; i < ChildList.size(); ++i)
		{
			DatabaseUnit dbu;
			dbu.parent_type = fullPath.toStdString();
			QString tmp = fullPath +"/" + ChildList[i];
			dbu.type = tmp.toStdString();
			databaseHierarchy.push_back(dbu);
			TraversalDirectory(strRes, baseDir,databaseHierarchy,fullPath +"/" + ChildList[i], nDepth+1);
		}
	}
}

vector<DatabaseUnit> MainWindow_M::GetSelectedUnits() const
{
	vector<QString> vSelectedItemPaths;

	QModelIndex qCurrentIndex = dataTreeView->model()->index(0, 0, dataTreeView->rootIndex());

	GetSelectedItemsPath(dataTreeView->GetSelectedIndexes(), strCurrentDBDir, qCurrentIndex, &vSelectedItemPaths);

	vector<DatabaseUnit> vResUnits;
	for(vector<QString>::iterator ItrPath = vSelectedItemPaths.begin(); ItrPath != vSelectedItemPaths.end(); ++ItrPath){
		for(vector<DatabaseUnit>::const_iterator ItrUnit = databaseHierarchy.begin(); ItrUnit != databaseHierarchy.end(); ++ItrUnit){
			if(*ItrPath == QString(ItrUnit->type.c_str())){
				vResUnits.push_back(*ItrUnit);
				break;
			}
		}
	}

	return vResUnits;
}

void MainWindow_M::FilterImageIndex(std::vector<int> &vImageIndex, std::vector<double>& dist_sorted)
{
	vector<DatabaseUnit> vSelectedUnits = GetSelectedUnits();
	vector<int> vFiltered;
	vector<double> vDistFiltered;
	for(vector<int>::iterator ItrImage = vImageIndex.begin(); ItrImage != vImageIndex.end(); ++ItrImage)
	{
		for(vector<DatabaseUnit>::iterator ItrUnit = vSelectedUnits.begin(); ItrUnit != vSelectedUnits.end(); ++ItrUnit)
		{
			if(*ItrImage>=ItrUnit->start && *ItrImage<=ItrUnit->end){
				vFiltered.push_back(*ItrImage);
				vDistFiltered.push_back(*(dist_sorted.begin() + (ItrImage - vImageIndex.begin())));
				break;
			}
			if(*ItrImage < ItrUnit->start){
				break;
			}
		}
	}
	vImageIndex.swap(vFiltered);
	dist_sorted.swap(vDistFiltered);
}

void MainWindow_M::FilterImageIndex(std::vector<int> &vImageIndex)
{
	vector<DatabaseUnit> vSelectedUnits = GetSelectedUnits();
	vector<int> vFiltered;
	for(vector<int>::iterator ItrImage = vImageIndex.begin(); ItrImage != vImageIndex.end(); ++ItrImage)
	{
		for(vector<DatabaseUnit>::iterator ItrUnit = vSelectedUnits.begin(); ItrUnit != vSelectedUnits.end(); ++ItrUnit)
		{
			if(*ItrImage>=ItrUnit->start && *ItrImage<=ItrUnit->end){
				vFiltered.push_back(*ItrImage);
				break;
			}
			if(*ItrImage < ItrUnit->start){
				break;
			}
		}
	}
	vImageIndex.swap(vFiltered);
}

void MainWindow_M::GetSelectedItemsPath(const QModelIndexList& qIndexList, const QString &strParent, const QModelIndex &qCurrentIndex, std::vector<QString> *pResPaths) const
{
	if(qIndexList.contains(qCurrentIndex)){
		pResPaths->push_back(strParent);
		return;
	}

	int nChildNum = dataTreeView->model()->rowCount(qCurrentIndex);

	for(int i = 0 ; i < nChildNum ; ++i){

		QModelIndex qChildIndex = qCurrentIndex.child(i,0);
		QString strName = dataTreeView->model()->data(qChildIndex).toString();
		if(qIndexList.contains(qChildIndex)){
			pResPaths->push_back(strParent + "/" + strName);
		}
		else{
			GetSelectedItemsPath(qIndexList, strParent + "/" + strName, qChildIndex, pResPaths);
		}

	}
}

void MainWindow_M::SetSkectchOrderLabelBackground(const QColor& color)
{
	//pass
}

/*----------Unused----------*/

void MainWindow_M::analysisImage()
{
	analysisImage_act->setChecked(false);
}

void MainWindow_M::setBackgroundImg(int index)
{

}

/*----------End unused----------*/