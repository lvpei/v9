#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QLabel>
#include <QGraphicsScene>
#include <QWeakPointer>

#include "renderarea.h"
#include "SAEncodeVectorImages.h"

#include <vector>
#include <QFileSystemModel>
#include "DatabaseUnit.h"

using namespace std;

namespace Ui {
class MainWindow;
}

class Skeleton;

class TreeModel;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
private:
    Ui::MainWindow *ui;
	QLabel* msgLabel; 
	int m_iDisplayNearestNum;
	bool m_bShowOriginalImg;
	/*
		graphics scene container for candidate poses
	*/
	QGraphicsScene m_graphicsScene;

private slots:
    void exitProgram();
	void newCanvas();
	void saveCanvas();
	void openFile();
	void dynamicHint(bool hint);
	void analysisImage();
	
	// import .sa file
	void importFile();
	void showGrid();
	void encodeImageFiles();
	void extractEdgeImage();
	void setDisplayNearestNum();
	void showOriginalImage();

	/*
		set the current behavior mode for main window area
	*/
	void setToSketchingMode();
	void setToViewingMode();

	/*
		set to sketch character pose
	*/
	void setToSketchPose(bool t_);
	
	/*
		set to sketch character animation
	*/
	void setToSketchAnimation(bool t_);

	/*
		compute the contour of the final blended edge image
	*/
	void computeContour();
	/*
		set the default database directory
	*/
	void setWorkingDir();

	/*
		load the encoded file
	*/
	void loadEncodedFile(const char* filename);

	/*
		load the encoded file
	*/
	void loadEncodedFile(const char* filename,CSAEncodeVectorImages* pSAEncodeVectorImages);

	/*
		change the current drawing mode to pen mode
	*/
	void changeToPenMode(bool mode);

	/*
		change the current drawing mode to eraser mode
	*/
	void changeToEraserMode(bool mode);
	/*
		change the sketch view
	*/
	void changeView(int index);
	/*
		change the 3d pose view
	*/
	void change3DView(int angle);
	/*
		show the 3d pose
	*/
	void show3DPose(bool show);
	/*
		show the sketch
	*/
	void showSketch(bool show);
	/*
		show the sketch
	*/
	void showShadow(bool show);

	/*
		This function is used to test any new algorithm
	*/
	void test();

	/*
		refine the final result
	*/
	void refineFinalResultBySampling();

	/*
		play animation
	*/
	void playAnimation();

	/*
		select to show the trajectory of which joint
	*/
	void selectTrajectory(int index);

private:
	QString strCurrentFileName;
	QString strCurrentDBDir;

	vector<CSAEncodeVectorImages*> m_pSAEncodeImgArr;
	CSAEncodeVectorImages* m_pSAEncodeImg;

	// the color map
	vector< vector<float> > colormap;

	// read hierarchy
	vector<QString> baseDir;
	vector<DatabaseUnit> databaseHierarchy;

private:
	/**
		load the color map file
	*/
	void loadColormap();

	/**
		write persistent platform-independent application settings.	
	*/
	void writeSettings();

	/**
		read persistent platform-independent application settings.
	*/
	void readSettings();

public slots:
	// import certain files in batch
	//void importFile(const char* folder,int* file,double* color,int size);
	void importFile(const char* folder,int* file,double* color,int size,bool bkg);

	//void importFile(const char* folder,const vector<int>& file,const vector<vector<int>>& nearestpatchidx,const vector<double>& color);
	// load the images as OpenCV format
	//void importFile(const char* folder, const vector<int>& imageidx, vector<Mat>& images);
	// load the images and also transform it
	//void importFile(const char* folder,const vector<int>& file,const vector<double>& color,const vector<vector<double>>& transform);
	// set the background image
	void setBackgroundImg(int index);

	// replay the process of drawing
	void replay();

private:
	// delete all the sketch images in tmp folder
	void clearTempFiles();

//added by zpy
private:
	void TraversalDirectory(QString &strRes, vector<QString> &baseDir, vector<DatabaseUnit>& databaseHierachy ,const QString &strDir, int nDepth);

public:
	std::vector<DatabaseUnit> GetSelectedUnits()const;
	void FilterImageIndex(std::vector<int> &vImageIndex, std::vector<double>& dist_sorted);
	void FilterImageIndex(std::vector<int> &vImageIndex);

private:
	void GetSelectedItemsPath(const QModelIndexList& qIndexList, const QString &strParent, const QModelIndex &qCurrentIndex, std::vector<QString> *pResPaths)const;

private:
	TreeModel *m_pTreeModel;
//...

public:
	void SetSkectchOrderLabelBackground(const QColor& color);

};

#endif // MAINWINDOW_H
