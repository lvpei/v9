#ifndef MAINWINDOW_M_H
#define MAINWINDOW_M_H

#include <QMainWindow>
#include <QString>
#include <QLabel>
#include <QGraphicsScene>
#include <QWeakPointer>
#include <QMdiArea>
#include <QRadioButton>
#include <QComboBox>
#include <QPushButton>
#include <QMdiSubWindow>
#include <QToolBar>

#include "renderarea.h"
#include "SAEncodeVectorImages.h"

#include <vector>
#include <QFileSystemModel>
#include "DatabaseUnit.h"
#include "skeletonview.h"
#include "DataBaseTreeView.h "
#include "slider.h"

using namespace std;

namespace Ui {
	class MainWindow;
}

class Skeleton;

class TreeModel;

class MainWindow_M : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow_M(QWidget* parent = 0);
	~MainWindow_M();

public:
	/*----------Public Fun----------*/
	void createMenu();
	void createWidgets();
	void createToolBar();
	void createStatusBar();
	void createActions();
	void createDockWidget();
	void createConnects();

	void clearTempFiles();
	/*----------End public Fun----------*/

private slots:
	/*----------Action Slots----------*/
	void newCanvas();
	void openFile();
	void importFile();
	void importFile(const char* folder,int* file,double* color,int size,bool bkg);
	void saveCanvas();
	void saveAsCanvas();//added
	void exitProgram();

	void showGrid();
	void showSketch(bool _on);
	void showShadow(bool _on);
	void showOriginalImage();

	void encodeImageFiles();
	void extractEdgeImage();
	//compute the contour of the final blended edge image
	void computeContour();
	void test();

	void setDisplayNearestNum();
	//set the default database directory
	void setWorkingDirectionary();

	//set the current behavior mode for main window area
	void sketching();
	void viewing();

	void timeLine();
	/*----------End Action Slots----------*/

	/*----------Widget Slots----------*/
	void analysisImage();
	void dynamicHint(bool _on);
	void setToSketchPose(bool _on);
	void setToSketchAnimation(bool _on);
	//change the current drawing mode to pen mode
	void changeToPenMode(bool _on);
	//change to current drawing mode to eraser mode
	void changeToEraserMode(bool _on);
	//change the 3d pose view
	void change3DView(int angle);
	//show the 3d pose
	void show3DPose(bool _on);
	//refine the final result
	void refineFinalResultBySampling();
	//play animation
	void playAnimation();
	//select to show the trajectory of which joint
	void selectTrajectory(int index);
	//replay the process of drawing
	void replay();
	//update the range and interval for timeline slider
	void changeTimelineRangeAndInterval(int, int, int);
	/*----------End Widget Slots----------*/

private:
	//load the encoded file
	void loadEncodedFile(const char* filename);
	//load the encoded file
	void loadEncodedFile(const char* filename, CSAEncodeVectorImages* pSAEncodeVectorImages);
	//change the sketch	
	void changeView(int index);
	//load the color map file
	void loadColormap();
	//write persistent platform-independent application settings
	void writeSettings();
	//read persistent platform-independent application settings
	void readSettings();
	void TraversalDirectory(QString &strRes, vector<QString> &baseDir, vector<DatabaseUnit>& databaseHierachy ,const QString &strDir, int nDepth);
	vector<DatabaseUnit> GetSelectedUnits()const;
	void FilterImageIndex(std::vector<int> &vImageIndex, std::vector<double>& dist_sorted);
	void FilterImageIndex(std::vector<int> &vImageIndex);
	void GetSelectedItemsPath(const QModelIndexList& qIndexList, const QString &strParent, const QModelIndex &qCurrentIndex, std::vector<QString> *pResPaths)const;
	void SetSkectchOrderLabelBackground(const QColor& color);

	/*----------Unused----------*/
	void setBackgroundImg(int index);
	/*----------End unused----------*/

private:
	/*----------UI----------*/
	QMenu* file_menu;
	QMenu* view_menu;
	QMenu* tools_menu;
	QMenu* setting_menu;
	QMenu* mode_menu;
	QMenu* window_menu;

	QAction* new_action;
	QAction* open_action;
	QAction* import_action;
	QAction* save_action;
	QAction* saveAs_action;
	QAction* exit_action;

	QAction* showGrid_action;
	QAction* showSketch_action;
	QAction* showShadow_action;
	QAction* showOriginalImage_action;

	QAction* encodeImageFiles_action;
	QAction* extractEdgeImage_action;
	QAction* computeContour_action;
	QAction* test_action;

	QAction* setDisplayNearestNum_action;
	//set the default database directory
	QAction* setWorkingDirectionary_action;

	QAction* sketching_action;
	QAction* viewing_action;

	QAction* timeLine_action;
	/*----------End UI----------*/

	/*----------Widget----------*/
	QMdiArea* mdi;
	RenderArea* renderarea;
	SkeletonView* animationWindow;

	DataBaseTreeView* dataTreeView;
	QRadioButton* pose_button;
	QRadioButton* animation_button;
	QAction* analysisImage_act;
	QAction* dynamichint_act;
	QAction* pen_act;
	QAction* eraser_act;
	QSlider* view_slider;
	QAction* show3D_act;
	QPushButton* refine_button;
	QPushButton* playAnimation_button;
	QComboBox* selectTraj_combo;
	QPushButton* replay_button;
	Slider* timeline_slider;
	/*----------End Widget----------*/

	/*----------Content----------*/
	QString strCurrentFileName;
	QString strCurrentDBDir;

	bool m_bShowOriginalImg;
	int m_iDisplayNearestNum;

	//graphics scene container for candidate poses
	QGraphicsScene m_graphicsScene;

	vector<CSAEncodeVectorImages*> m_pSAEncodeImgArr;
	CSAEncodeVectorImages* m_pSAEncodeImg;

	vector< vector<float> > colormap;

	vector<QString> baseDir;
	vector<DatabaseUnit> databaseHierarchy;

	TreeModel *m_pTreeModel;
	/*----------End content----------*/
};

#endif