#include "mainwindow.h"
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

MainWindow::MainWindow(QWidget *parent) :
QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	strCurrentFileName = "";
	m_pSAEncodeImg = NULL;
	m_iDisplayNearestNum = 50;
	m_bShowOriginalImg = false;

	// status bar
	msgLabel = new QLabel; 
	msgLabel->setMinimumSize(msgLabel->sizeHint()); 
	msgLabel->setAlignment(Qt::AlignHCenter); 

	statusBar()->addWidget(msgLabel);
	statusBar()->setStyleSheet(QString("QStatusBar::item{border: 0px}"));

	// disable the maximize button & fixed window size
	Qt::WindowFlags flags = 0;
	flags |= Qt::WindowMinimizeButtonHint;
	flags |= Qt::MSWindowsFixedSizeDialogHint;
	setWindowFlags(flags); 
	//setFixedSize(1290,921);

	QObject::connect(ui->renderarea,SIGNAL(loadImages(const char*,int*,double*,int,bool)),this,SLOT(importFile(const char*,int*,double*,int,bool)));
	//QObject::connect(ui->renderarea,SIGNAL(loadImages(const char*, const vector<int>&, const vector<vector<int>>&,const vector<double>&)),this,SLOT(importFile(const char*,const vector<int>&,const vector<vector<int>>&,const vector<double>&)));
	//QObject::connect(ui->renderarea,SIGNAL(loadImages(const char*, const vector<int>&, vector<Mat>&)),this,SLOT(importFile(const char*, const vector<int>&, vector<Mat>&)));
	//QObject::connect(ui->renderarea,SIGNAL(loadImages(const char*,const vector<int>&,const vector<double>&,const vector<vector<double>>&)),this,SLOT(importFile(const char*,const vector<int>&,const vector<double>&,const vector<vector<double>>&)));
	QObject::connect(ui->renderarea,SIGNAL(switchViewTo(int)),this,SLOT(changeView(int)));
	
	//QObject::connect(ui->graphicsView,SIGNAL(setBackgroundImg(int)),this,SLOT(setBackgroundImg(int)));
	
	// send & receive new motion
	QObject::connect(ui->renderarea,SIGNAL(sendNewMotion(vector<Posture>&)),ui-> animationWindow,SLOT(receiveNewMotion(vector<Posture>&)));

	clearTempFiles();

	//ui->graphicsView->setScene(&m_graphicsScene);

	ui->penmodeRadioButton->setChecked(true);

	// load the color map
	loadColormap();

	// initialize the variables in RenderArea class
	ui->renderarea->m_vColormap = colormap;
	ui->renderarea->m_iDisplayNearestNum = m_iDisplayNearestNum;

	//added by zpy
	m_pTreeModel = 0x0;
}

MainWindow::~MainWindow()
{
	writeSettings();
	delete ui;
	if(msgLabel)
		delete msgLabel;

	for(int i = 0; i < m_pSAEncodeImgArr.size(); i++)
	{
		delete m_pSAEncodeImgArr[i];
		m_pSAEncodeImgArr[i] = NULL;
	}

	m_pSAEncodeImg = NULL;
}

void MainWindow::exitProgram()
{
	close();
}

// open existing file
void MainWindow::openFile()
{
	// commented by lvp 12-6-1
	//saveCanvas();

	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"./",tr("Images (*.png)"));
	if(fileName.isEmpty())
		return;
	QFile file(fileName);
	file.read(QIODevice::ReadOnly);
	QImageReader imageReader;
	imageReader.setDevice(&file);
	ui->renderarea->setCurrentCanvas(imageReader.read());
	ui->renderarea->setContentChanged(false);

	// set the reference pose image
	//QPixmap pixmap(fileName);
	//pixmap = pixmap.scaled(QSize(240,240));
	//ui->refPoseImage->setPixmap(pixmap);

	msgLabel->setText(fileName.section('/',-1));

	clearTempFiles();

	// commented by lvp. This is for testing.
	strCurrentFileName = fileName;
}
// create new canvas
void MainWindow::newCanvas()
{
	// commented by lvp 12-6-1
	//saveCanvas();
	ui->renderarea->resetContent();
	clearTempFiles();
}

// save current canvas
void MainWindow::saveCanvas()
{
	QImage image = ui->renderarea->getFinalCanvas();
	if(ui->renderarea->isContentChanged())
	{
		if(strCurrentFileName.isEmpty())
		{
			QMessageBox msgBox;
			msgBox.setText("The content has been modified.");
			msgBox.setInformativeText("Do you want to save your changes?");
			msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
			msgBox.setDefaultButton(QMessageBox::Save);
			int ret = msgBox.exec();
			switch(ret)
			{
			case QMessageBox::Save:
				{
					// Save was clicked
					QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),"./",tr("Images (*.png)"));
					if(!fileName.isEmpty())
						strCurrentFileName = fileName;
					break;
				}				
			case QMessageBox::Discard:
				{
					// Don't Save was clicked
					ui->renderarea->setContentChanged(false);
					break;
				}				
			case QMessageBox::Cancel:
				{
					// Cancel was clicked
					break;
				}		
			default:
				// should never be reached
				break;
			}
		}
		QFile file(strCurrentFileName);
		file.open(QIODevice::WriteOnly);
		image.save(&file);
		ui->renderarea->setContentChanged(false);
	}	
}

// dynamic hint while user is drawing
void MainWindow::dynamicHint(bool hint)
{
	if(hint)
		ui->renderarea->startTimer();
	else
		ui->renderarea->stopTimer();
}

// analysis the current front image on the canvas
void MainWindow::analysisImage()
{
}

void MainWindow::importFile()
{
	/*
	saveCanvas();

	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"./",tr("EncodeImages (*.sa)"));
	if(fileName.isEmpty())
		return;
	if(m_pSAEncodeImg)
	{
		delete m_pSAEncodeImg;
		m_pSAEncodeImg = NULL;
	}
	m_pSAEncodeImg = new CSAEncodeImages();
	m_pSAEncodeImg->loadFromFile(fileName.toAscii());

	// draw the shadow
	QImage srcImage,destImage,resultImage;
	resultImage = QImage(480,480,QImage::Format_ARGB32);
	resultImage.fill(qRgba(255,255,255,0));

	float colorpercent = 200.0 / m_pSAEncodeImg->m_iImageNum;
	QColor color2( 192,192,192);

	QPainter painter(&resultImage);
	painter.fillRect(resultImage.rect(), Qt::transparent);
	for(int i = 0; i < m_pSAEncodeImg->m_iImageNum; i++)
	{
		for(int j = 0; j < m_pSAEncodeImg->m_iPointNums[i].size(); j += 2)
		{
			int m = m_pSAEncodeImg->m_iPointNums[i][j]; 
			int n = m_pSAEncodeImg->m_iPointNums[i][j+1];
			resultImage.setPixel(m,n,color2.rgb());
		}
	}
	painter.end();

	ui->renderarea->setCurrentCanvas(resultImage);
	ui->renderarea->setContentChanged(true);
	//*/
}

//void MainWindow::importFile(const char* folder,int* file,double* color,int size)
//{
//	// if CSAEncodeImages object not exist, create it
//	if(m_pSAEncodeImg == NULL)
//	{
//		qDebug()<<"loading the encoded file";
//
//		QString fileName = folder;
//		fileName += "\\"+fileName.section('\\',-1) + ".sa";
//
//		m_pSAEncodeImg = new CSAEncodeImages();
//		m_pSAEncodeImg->loadFromFile(fileName.toAscii());
//
//		qDebug()<<"encoded file loaded";
//	}
//
//	// clear the graphics scene
//	m_graphicsScene.clear();
//
//	// draw the shadow
//	QImage resultImage,tmp,tmp_small;
//	resultImage = QImage(480,480,QImage::Format_ARGB32);
//	resultImage.fill(qRgba(255,255,255,0));
//	tmp = QImage(120,120,QImage::Format_ARGB32);;
//
//	int displaynum = min(m_iDisplayNearestNum,size);
//
//	for(int i = 0; i < displaynum; i++)
//	{
//		tmp.fill(qRgba(255,255,255,0));
//		int ix = (displaynum - i) * (colormap.size() - 1) / displaynum;
//		//int ix = (1-color[i]) * (colormap.size() - 1);
//		// use the colormap to describe the probability
//		QColor color2(colormap[ix][0] * 255,colormap[ix][1] * 255,colormap[ix][2] * 255);
//		for(int j = 0; j < m_pSAEncodeImg->m_iPointNums[file[i]].size(); j += 2)
//		{
//			int m = m_pSAEncodeImg->m_iPointNums[file[i]][j]; 
//			int n = m_pSAEncodeImg->m_iPointNums[file[i]][j+1];
//			resultImage.setPixel(m,n,color2.rgb());
//			tmp.setPixel(m/4,n/4,color2.rgb());
//		}
//		QPixmap pixmap;
//		pixmap.convertFromImage(tmp);
//		CSACustomGraphicsPixmapItem* graphicspixmapitem = new CSACustomGraphicsPixmapItem(&m_graphicsScene);
//		graphicspixmapitem->setPixmap(pixmap);
//		graphicspixmapitem->setPos(4 + i%2*124,4 + i/2*124);
//		graphicspixmapitem->setImageIndex(file[i]);
//	}
//
//	//ui->renderarea->setCurrentCanvas(resultImage);
//}

void MainWindow::importFile(const char* folder,int* file,double* color,int size,bool bkg)
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
	
	if(ui->renderarea->m_pSkeleton)
	{
		Skeleton* pSkeleton = ui->renderarea->m_pSkeleton;

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
		ui->renderarea->setCurrentCanvas(resultImage);
}

//void MainWindow::importFile(const char* folder,const vector<int>& file,const vector<vector<int>>& nearestpatchidx,const vector<double>& color)
//{
//	// if CSAEncodeImages object not exist, create it
//	if(m_pSAEncodeImg == NULL)
//	{
//		qDebug()<<"loading the encoded file";
//
//		QString fileName = folder;
//		fileName += "\\"+fileName.section('\\',-1) + ".sa";
//
//		m_pSAEncodeImg = new CSAEncodeImages();
//		m_pSAEncodeImg->loadFromFile(fileName.toAscii());
//
//		qDebug()<<"encoded file loaded";
//	}
//
//	// read the colormap file
//	vector< vector<float> > colormap;
//	ifstream in;
//	in.open("colormap.txt");
//	if(in.is_open())
//	{
//		float r,g,b;
//		in>>r;
//		in>>g;
//		in>>b;
//		while(!in.eof())
//		{
//			vector<float> color_rgb;
//			color_rgb.push_back(r);
//			color_rgb.push_back(g);
//			color_rgb.push_back(b);
//			colormap.push_back(color_rgb);
//			in>>r;
//			in>>g;
//			in>>b;
//		}
//	}
//	else{
//		qDebug()<<"colormap open failed";
//	}
//
//	// draw the shadow
//	QImage resultImage;
//	resultImage = QImage(480,480,QImage::Format_ARGB32);
//	resultImage.fill(qRgba(255,255,255,0));
//
//	QPainter painter(&resultImage);
//	int size = file.size();
//	int displaynum = min(m_iDisplayNearestNum,size);
//	for(int i = 0; i < displaynum; i++)
//	{
//		//int ix = (displaynum - i) * (colormap.size() - 1) / displaynum;
//		int ix = (1-color[i]) * (colormap.size() - 1);
//		// use the colormap to describe the probability
//		QColor color2(colormap[ix][0] * 255,colormap[ix][1] * 255,colormap[ix][2] * 255);
//
//		for(int j = 0; j < m_pSAEncodeImg->m_iPointNums[file[i]].size(); j += 2)
//		{
//			int m = m_pSAEncodeImg->m_iPointNums[file[i]][j]; 
//			int n = m_pSAEncodeImg->m_iPointNums[file[i]][j+1];
//			resultImage.setPixel(m,n,color2.rgb());
//		}
//	}
//	painter.end();
//
//	ui->renderarea->setCurrentCanvas(resultImage);
//}

//void MainWindow::importFile(const char* folder, const vector<int>& imageidx, vector<Mat>& images)
//{
//	QString path = folder;
//	QDir dir(folder);
//	if(!dir.exists())
//		return;
//
//	// the name filter
//	QStringList nameFilter ; 
//	nameFilter << "*.png" << "*.PNG";
//	QFileInfoList list = dir.entryInfoList(nameFilter,QDir::Files |QDir::NoSymLinks,QDir::Name);  
//
//	QFileInfo fileInfo;
//	QString fileName;
//
//	for(int i = 0; i < imageidx.size(); i++)
//	{
//		fileInfo = list.at(imageidx[i]);
//
//		// compute patch descriptor
//		fileName = fileInfo.path() + "/" + fileInfo.fileName();
//		// convert the fileName from QString to char*
//		char file[256];
//		int len = 0;
//		for(;len < fileName.length(); len++)
//		{
//			file[len] = (fileName.at(len)).toAscii();
//		}
//		file[len] = '\0';
//
//		Mat mat = imread(file,CV_LOAD_IMAGE_COLOR);
//		images.push_back(mat);
//	}
//}

// load the images and also transform it
//void MainWindow::importFile(const char* folder,const vector<int>& file,const vector<double>& color,const vector<vector<double>>& transform)
//{
//	// if CSAEncodeImages object not exist, create it
//	if(m_pSAEncodeImg == NULL)
//	{
//		qDebug()<<"loading the encoded file";
//
//		QString fileName = folder;
//		fileName += "\\"+fileName.section('\\',-1) + ".sa";
//
//		m_pSAEncodeImg = new CSAEncodeImages();
//		m_pSAEncodeImg->loadFromFile(fileName.toAscii());
//
//		qDebug()<<"encoded file loaded";
//	}
//
//	// read the colormap file
//	vector< vector<float> > colormap;
//	ifstream in;
//	in.open("colormap.txt");
//	if(in.is_open())
//	{
//		float r,g,b;
//		in>>r;
//		in>>g;
//		in>>b;
//		while(!in.eof())
//		{
//			vector<float> color_rgb;
//			color_rgb.push_back(r);
//			color_rgb.push_back(g);
//			color_rgb.push_back(b);
//			colormap.push_back(color_rgb);
//			in>>r;
//			in>>g;
//			in>>b;
//		}
//	}
//	else{
//		qDebug()<<"colormap open failed";
//	}
//
//	// draw the shadow
//	QImage resultImage,resultImage_Original;
//	resultImage = QImage(480,480,QImage::Format_ARGB32);
//	resultImage.fill(qRgba(255,255,255,0));
//	resultImage_Original = resultImage;
//
//	QPainter painter(&resultImage);
//	int size = file.size();
//	int displaynum = min(m_iDisplayNearestNum,size);
//	for(int i = 0; i < displaynum; i++)
//	{
//		//int ix = (displaynum - i) * (colormap.size() - 1) / displaynum;
//		int ix = (1-color[i]) * (colormap.size() - 1);
//		// use the colormap to describe the probability
//		QColor color2(colormap[ix][0] * 255,colormap[ix][1] * 255,colormap[ix][2] * 255);
//
//		int offsetx = transform[i][0];
//		int offsety = transform[i][1];
//
//		qDebug()<<"File index: "<<file[i]<<"\t\ttranslation:"<<" "<<offsetx<<" "<<offsety;
//
//		for(int j = 0; j < m_pSAEncodeImg->m_iPointNums[file[i]].size(); j += 2)
//		{
//			int m = m_pSAEncodeImg->m_iPointNums[file[i]][j]; 
//			int n = m_pSAEncodeImg->m_iPointNums[file[i]][j+1];
//
//			// the original image
//			if(m_bShowOriginalImg)
//				resultImage.setPixel(m,n,color2.rgb());
//
//			resultImage_Original.setPixel(m,n,color2.rgb());
//
//			m += offsetx;
//			n += offsety;
//			if( m >= 0 && m <= 479 && n >=0 && n <= 479)
//				resultImage.setPixel(m,n,color2.rgb());
//		}
//	}
//	painter.end();
//
//	ui->renderarea->setCurrentCanvas(resultImage);
//
//	//resultImage_Original.save("image_original.png","png");
//	//resultImage.save("image_align.png","png");
//}

// delete all the sketch images in tmp folder
void MainWindow::clearTempFiles()
{
	QDir dir(".\\tmp");
	QFileInfoList list = dir.entryInfoList();
	for(int i = 0; i < list.size(); i++)
	{
		dir.remove(list[i].fileName());
	}
}

void MainWindow::encodeImageFiles()
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
void MainWindow::extractEdgeImage()
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

void MainWindow::setDisplayNearestNum()
{
	CSANearestNum nndlg;
	nndlg.setNearestNum(m_iDisplayNearestNum);
	nndlg.exec();
	m_iDisplayNearestNum = nndlg.getNearestNum();

	// update the display number of nearest neighbor in RenderArea
	ui->renderarea->m_iDisplayNearestNum = m_iDisplayNearestNum;
}

void MainWindow::showGrid()
{
	if(ui->actionShow_Grid->isChecked())
	{
		ui->renderarea->setShowGrid(true);
	}
	else
		ui->renderarea->setShowGrid(false);
}


/*
	show the sketch
*/
void MainWindow::showSketch(bool show)
{
	ui->renderarea->setShowSketch(show);
}
/*
	show the shadow
*/
void MainWindow::showShadow(bool show)
{
	ui->renderarea->setShowShadow(show);
}

void MainWindow::showOriginalImage()
{
	if(ui->actionShow_Original_Image->isChecked())
		m_bShowOriginalImg = true;
	else
		m_bShowOriginalImg = false;
}

/*
compute the contour of the final blended edge image
*/
void MainWindow::computeContour()
{
	// load the final blended edge image
	Mat edgeimg,edgeimg_singlechannel;
	edgeimg = ui->renderarea->qimage2mat(ui->renderarea->getFinalCanvas());

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

/*
	set the default database directory
*/
void MainWindow::setWorkingDir()
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

	ui->DataTreeView->setModel(m_pTreeModel);
	ui->DataTreeView->setSelectionMode(QAbstractItemView::MultiSelection);
	
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
	ui->renderarea->setDBDir(baseDir,databaseHierarchy);
}


void MainWindow::TraversalDirectory(QString &strRes, vector<QString> &baseDir, vector<DatabaseUnit>& databaseHierarchy,const QString &strDir, int nDepth)
{
	QDir dir(strDir);
	if(!dir.exists()){
		return;
	}

	int nTemp = nDepth;
	while(nTemp--){
		strRes.push_back(' ');
	}

	int i;
	for(i = strDir.size() - 1;  i >= 0; --i)
	{
		if(strDir[i]=='\\'||strDir[i]=='/')break;
	}

	strRes.push_back(strDir.right(strDir.size() - i-1));
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

/*
	load the encoded file
*/
void MainWindow::loadEncodedFile(const char* filename)
{
	CSAEncodeVectorImages* p;
	qDebug("loading the encoded file %s",filename);
	p = new CSAEncodeVectorImages();
	p->loadFromFile(filename);
	qDebug()<<"encoded file loaded";

	m_pSAEncodeImgArr.push_back(p);
	m_pSAEncodeImg = p;
}

/*
	load the encoded file
*/
void MainWindow::loadEncodedFile(const char* filename,CSAEncodeVectorImages* pSAEncodeVectorImages)
{
	qDebug("loading the encoded file %s",filename);
	pSAEncodeVectorImages->loadFromFile(filename);
	qDebug()<<"encoded file loaded";
}

// set the background image
void MainWindow::setBackgroundImg(int index)
{
	QImage resultImage;
	resultImage = QImage(480,480,QImage::Format_ARGB32);
	resultImage.fill(qRgba(255,255,255,0));

	/*
	QColor color(0,0,0);
	for(int j = 0; j < m_pSAEncodeImg->m_iPointNums[index].size(); j += 2)
	{
		int m = m_pSAEncodeImg->m_iPointNums[index][j]; 
		int n = m_pSAEncodeImg->m_iPointNums[index][j+1];
		resultImage.setPixel(m,n,color.rgb());
	}
	//*/

	// update the frame index of 3d character 
	//ui->renderarea->imgindex.clear();
	//ui->renderarea->imgindex.push_back(index);
	ui->renderarea->setCurrentCanvas(resultImage.scaled(QSize(720,720)));

	ui->renderarea->setDefaultStatus(false);
}

/*
change the current drawing mode to pen mode
*/
void MainWindow::changeToPenMode(bool mode)
{
	ui->penmodeRadioButton->setChecked(mode);
	if(mode)
	{
		ui->renderarea->setDrawingMode(PEN_MODE);
	}
}

/*
change the current drawing mode to eraser mode
*/
void MainWindow::changeToEraserMode(bool mode)
{
	ui->erasermodeRadioButton->setChecked(mode);
	if(mode)
	{
		ui->renderarea->setDrawingMode(ERASER_MODE);
	}
}

/*
	change the sketch view
*/
void MainWindow::changeView(int index)
{
	// the index should not be larger than the total loaded view number 
	if(index >= m_pSAEncodeImgArr.size())
		return;

	m_pSAEncodeImg = m_pSAEncodeImgArr[index];

	ui->renderarea->changeView(index);
}

/*
	change the 3d pose view
*/
void MainWindow::change3DView(int angle)
{
	static double lastchange = 0.0;

	double rad = angle;
	{
		ui->renderarea->m_Camera.m_rotations[0] += (rad - lastchange);

		lastchange = rad;

		ui->renderarea->update();
	}
}

/*
show the 3d pose
*/
void MainWindow::show3DPose(bool show)
{
	ui->renderarea->setShow3DPose(show);
}

/**
load the color map file
*/
void MainWindow::loadColormap()
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

/*
	This function is used to test any new algorithm
*/
void MainWindow::test()
{
	/*
	CSAImageProcess imageprocess;
	CSAView *pView = new CSAView();
	imageprocess.setView(pView);

	vector<Mat> bin;
	imageprocess.process_one_image(strCurrentFileName.toAscii(),bin,ARP);
	QImage image = ui->renderarea->getCurrentCanvas();
	image.save("sketch.png","png");
	imageprocess.process_sketch_image(ui->renderarea->qimage2mat(image),bin,ARP);

	//imwrite("2DHistBackground.png",bin[0] * 500);
	//imwrite("2DHistForeground.png",bin[1] * 500);

	delete pView;
	//*/
	
	/*
	int rows = bin[0].rows;
	int cols = bin[0].cols;

	ofstream db("db.txt");
	ofstream sk("sk.txt");
	for(int i = 0; i < rows; i++)
	{
		for(int j = 0; j < cols; j++)
		{
			db<<bin[0].at<unsigned short>(i,j)<<" ";
			sk<<bin[1].at<unsigned short>(i,j)<<" ";
		}
		db<<endl;
		sk<<endl;
	}

	//ui->renderarea->computeRecallRatioAndMQR();
	//*/

	ui->renderarea->test();
}

/*
	refine the final result
*/
void MainWindow::refineFinalResultBySampling()
{
	//ui->renderarea->refineResult();
	ui->renderarea->refineResultbyPSO();
}

/*
	set the current behavior mode for main window area
*/
void MainWindow::setToSketchingMode()
{
	ui->actionSketching->setChecked(true);
	ui->renderarea->setBehaviorMode(SKETCH);

	ui->actionViewing->setChecked(false);
}
void MainWindow::setToViewingMode()
{
	ui->actionViewing->setChecked(true);
	ui->renderarea->setBehaviorMode(VIEW);

	ui->actionSketching->setChecked(false);
}


/**
	write persistent platform-independent application settings.	
*/
void MainWindow::writeSettings()
{
	QSettings settings("CAD&CG ZJU", "SketchAnimation");

	settings.setValue("db_dir",strCurrentDBDir);
}

/**
	read persistent platform-independent application settings.
*/
void MainWindow::readSettings()
{
	QSettings settings("CAD&CG ZJU", "SketchAnimation");
}

vector<DatabaseUnit> MainWindow::GetSelectedUnits()const
{
	vector<QString> vSelectedItemPaths;

	QModelIndex qCurrentIndex = ui->DataTreeView->model()->index(0, 0, ui->DataTreeView->rootIndex());

   	GetSelectedItemsPath(ui->DataTreeView->GetSelectedIndexes(), strCurrentDBDir, qCurrentIndex, &vSelectedItemPaths);

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

void MainWindow::GetSelectedItemsPath(const QModelIndexList& qIndexList, const QString &strParent, const QModelIndex &qCurrentIndex, std::vector<QString> *pResPaths)const
{

	if(qIndexList.contains(qCurrentIndex)){
		pResPaths->push_back(strParent);
		return;
	}

	int nChildNum = ui->DataTreeView->model()->rowCount(qCurrentIndex);
	
	for(int i = 0 ; i < nChildNum ; ++i){
		
		QModelIndex qChildIndex = qCurrentIndex.child(i,0);
		QString strName = ui->DataTreeView->model()->data(qChildIndex).toString();
		if(qIndexList.contains(qChildIndex)){
			pResPaths->push_back(strParent + "/" + strName);
		}
		else{
			GetSelectedItemsPath(qIndexList, strParent + "/" + strName, qChildIndex, pResPaths);
		}

	}
}

void MainWindow::FilterImageIndex(std::vector<int> &vImageIndex, std::vector<double>& dist_sorted)
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

void MainWindow::FilterImageIndex(std::vector<int> &vImageIndex)
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

void MainWindow::SetSkectchOrderLabelBackground(const QColor& color)
{
	/*QPalette palette;
	palette.setColor(ui->SketchOrderNotifyLabel->backgroundRole(), color);

	ui->SketchOrderNotifyLabel->setPalette(palette);

	ui->SketchOrderNotifyLabel->setAutoFillBackground(true);*/
}

// replay the process of drawing
void MainWindow::replay()
{
	ui->renderarea->replay();
}

/*
	set to sketch character pose
*/
void MainWindow::setToSketchPose(bool t_)
{
	if(t_ == true)
	{
		ui->renderarea->setToSketchPoseOrAnimation(true);
	}
}

/*
	set to sketch character animation
*/
void MainWindow::setToSketchAnimation(bool t_)
{
	if(t_ == true)
	{
		ui->renderarea->setToSketchPoseOrAnimation(false);
	}
}

/*
	play animation
*/
void MainWindow::playAnimation()
{
	ui->animationWindow->startTimer();
}

void MainWindow::selectTrajectory(int index)
{
	ui->renderarea->showJointTrajectory(index);
}
