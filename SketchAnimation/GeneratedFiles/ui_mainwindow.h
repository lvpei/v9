/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Tue Apr 30 20:39:18 2013
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSlider>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "databasetreeview.h"
#include "renderarea.h"
#include "skeletonview.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionNew;
    QAction *actionSave;
    QAction *actionSave_2;
    QAction *actionExit;
    QAction *actionOpen;
    QAction *actionImport;
    QAction *actionShow_Grid;
    QAction *actionEncodeImageFiles;
    QAction *actionSetDisplayNearestNum;
    QAction *actionShow_Original_Image;
    QAction *actionExtractEdgeImage;
    QAction *actionComputeContour;
    QAction *actionSetWorkingDirectory;
    QAction *actionShow_Shadow;
    QAction *actionShow_Sketch;
    QAction *actionTest;
    QAction *actionSketching;
    QAction *actionViewing;
    QWidget *centralwidget;
    RenderArea *renderarea;
    QGroupBox *groupBox_3;
    QRadioButton *radioButton;
    QRadioButton *radioButton_2;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QPushButton *refineButton;
    QPushButton *replayButton;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *playAnimation;
    QComboBox *selectTraj;
    SkeletonView *animationWindow;
    QSlider *horizontalSlider;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuTools;
    QMenu *menuView;
    QMenu *menuSettings;
    QMenu *menuMode;
    QStatusBar *statusbar;
    QDockWidget *leftdockWidget;
    QWidget *toolsdockWidgetContents;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout_2;
    QRadioButton *penmodeRadioButton;
    QRadioButton *erasermodeRadioButton;
    QGroupBox *groupBox_2;
    QSlider *viewHorizontalSlider;
    QPushButton *analysisImageButton;
    QRadioButton *dynamichintradioButton;
    QRadioButton *show3DPoseRadioButton;
    DataBaseTreeView *DataTreeView;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1308, 679);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        actionNew = new QAction(MainWindow);
        actionNew->setObjectName(QString::fromUtf8("actionNew"));
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        actionSave_2 = new QAction(MainWindow);
        actionSave_2->setObjectName(QString::fromUtf8("actionSave_2"));
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        actionImport = new QAction(MainWindow);
        actionImport->setObjectName(QString::fromUtf8("actionImport"));
        actionShow_Grid = new QAction(MainWindow);
        actionShow_Grid->setObjectName(QString::fromUtf8("actionShow_Grid"));
        actionShow_Grid->setCheckable(true);
        actionShow_Grid->setChecked(true);
        actionShow_Grid->setEnabled(true);
        actionEncodeImageFiles = new QAction(MainWindow);
        actionEncodeImageFiles->setObjectName(QString::fromUtf8("actionEncodeImageFiles"));
        actionSetDisplayNearestNum = new QAction(MainWindow);
        actionSetDisplayNearestNum->setObjectName(QString::fromUtf8("actionSetDisplayNearestNum"));
        actionSetDisplayNearestNum->setCheckable(false);
        actionShow_Original_Image = new QAction(MainWindow);
        actionShow_Original_Image->setObjectName(QString::fromUtf8("actionShow_Original_Image"));
        actionShow_Original_Image->setCheckable(true);
        actionShow_Original_Image->setChecked(false);
        actionExtractEdgeImage = new QAction(MainWindow);
        actionExtractEdgeImage->setObjectName(QString::fromUtf8("actionExtractEdgeImage"));
        actionComputeContour = new QAction(MainWindow);
        actionComputeContour->setObjectName(QString::fromUtf8("actionComputeContour"));
        actionSetWorkingDirectory = new QAction(MainWindow);
        actionSetWorkingDirectory->setObjectName(QString::fromUtf8("actionSetWorkingDirectory"));
        actionShow_Shadow = new QAction(MainWindow);
        actionShow_Shadow->setObjectName(QString::fromUtf8("actionShow_Shadow"));
        actionShow_Shadow->setCheckable(true);
        actionShow_Shadow->setChecked(true);
        actionShow_Sketch = new QAction(MainWindow);
        actionShow_Sketch->setObjectName(QString::fromUtf8("actionShow_Sketch"));
        actionShow_Sketch->setCheckable(true);
        actionShow_Sketch->setChecked(true);
        actionTest = new QAction(MainWindow);
        actionTest->setObjectName(QString::fromUtf8("actionTest"));
        actionSketching = new QAction(MainWindow);
        actionSketching->setObjectName(QString::fromUtf8("actionSketching"));
        actionSketching->setCheckable(true);
        actionSketching->setChecked(true);
        actionViewing = new QAction(MainWindow);
        actionViewing->setObjectName(QString::fromUtf8("actionViewing"));
        actionViewing->setCheckable(true);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        sizePolicy.setHeightForWidth(centralwidget->sizePolicy().hasHeightForWidth());
        centralwidget->setSizePolicy(sizePolicy);
        renderarea = new RenderArea(centralwidget);
        renderarea->setObjectName(QString::fromUtf8("renderarea"));
        renderarea->setGeometry(QRect(0, 20, 480, 480));
        groupBox_3 = new QGroupBox(centralwidget);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(0, 510, 481, 111));
        radioButton = new QRadioButton(groupBox_3);
        radioButton->setObjectName(QString::fromUtf8("radioButton"));
        radioButton->setGeometry(QRect(10, 20, 47, 16));
        radioButton->setChecked(true);
        radioButton_2 = new QRadioButton(groupBox_3);
        radioButton_2->setObjectName(QString::fromUtf8("radioButton_2"));
        radioButton_2->setGeometry(QRect(10, 70, 77, 16));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(1);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(radioButton_2->sizePolicy().hasHeightForWidth());
        radioButton_2->setSizePolicy(sizePolicy1);
        radioButton_2->setAutoRepeatInterval(100);
        layoutWidget = new QWidget(groupBox_3);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(110, 20, 161, 25));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        refineButton = new QPushButton(layoutWidget);
        refineButton->setObjectName(QString::fromUtf8("refineButton"));

        horizontalLayout->addWidget(refineButton);

        replayButton = new QPushButton(layoutWidget);
        replayButton->setObjectName(QString::fromUtf8("replayButton"));

        horizontalLayout->addWidget(replayButton);

        layoutWidget1 = new QWidget(groupBox_3);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(110, 70, 161, 25));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        playAnimation = new QPushButton(layoutWidget1);
        playAnimation->setObjectName(QString::fromUtf8("playAnimation"));

        horizontalLayout_2->addWidget(playAnimation);

        selectTraj = new QComboBox(layoutWidget1);
        selectTraj->setObjectName(QString::fromUtf8("selectTraj"));

        horizontalLayout_2->addWidget(selectTraj);

        animationWindow = new SkeletonView(centralwidget);
        animationWindow->setObjectName(QString::fromUtf8("animationWindow"));
        animationWindow->setGeometry(QRect(490, 19, 571, 541));
        horizontalSlider = new QSlider(centralwidget);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setGeometry(QRect(490, 580, 571, 19));
        horizontalSlider->setOrientation(Qt::Horizontal);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1308, 23));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuTools = new QMenu(menubar);
        menuTools->setObjectName(QString::fromUtf8("menuTools"));
        menuView = new QMenu(menubar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuSettings = new QMenu(menubar);
        menuSettings->setObjectName(QString::fromUtf8("menuSettings"));
        menuMode = new QMenu(menubar);
        menuMode->setObjectName(QString::fromUtf8("menuMode"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);
        leftdockWidget = new QDockWidget(MainWindow);
        leftdockWidget->setObjectName(QString::fromUtf8("leftdockWidget"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(leftdockWidget->sizePolicy().hasHeightForWidth());
        leftdockWidget->setSizePolicy(sizePolicy2);
        leftdockWidget->setMinimumSize(QSize(104, 470));
        leftdockWidget->setMaximumSize(QSize(234, 629));
        toolsdockWidgetContents = new QWidget();
        toolsdockWidgetContents->setObjectName(QString::fromUtf8("toolsdockWidgetContents"));
        verticalLayout_4 = new QVBoxLayout(toolsdockWidgetContents);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        groupBox = new QGroupBox(toolsdockWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy3);
        groupBox->setMinimumSize(QSize(0, 100));
        groupBox->setFlat(false);
        groupBox->setCheckable(false);
        layoutWidget2 = new QWidget(groupBox);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(10, 20, 111, 51));
        verticalLayout_2 = new QVBoxLayout(layoutWidget2);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        penmodeRadioButton = new QRadioButton(layoutWidget2);
        penmodeRadioButton->setObjectName(QString::fromUtf8("penmodeRadioButton"));

        verticalLayout_2->addWidget(penmodeRadioButton);

        erasermodeRadioButton = new QRadioButton(layoutWidget2);
        erasermodeRadioButton->setObjectName(QString::fromUtf8("erasermodeRadioButton"));

        verticalLayout_2->addWidget(erasermodeRadioButton);


        verticalLayout_3->addWidget(groupBox);

        groupBox_2 = new QGroupBox(toolsdockWidgetContents);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        sizePolicy3.setHeightForWidth(groupBox_2->sizePolicy().hasHeightForWidth());
        groupBox_2->setSizePolicy(sizePolicy3);
        groupBox_2->setMinimumSize(QSize(0, 100));
        viewHorizontalSlider = new QSlider(groupBox_2);
        viewHorizontalSlider->setObjectName(QString::fromUtf8("viewHorizontalSlider"));
        viewHorizontalSlider->setGeometry(QRect(10, 20, 111, 20));
        viewHorizontalSlider->setMaximum(360);
        viewHorizontalSlider->setSingleStep(10);
        viewHorizontalSlider->setOrientation(Qt::Horizontal);
        analysisImageButton = new QPushButton(groupBox_2);
        analysisImageButton->setObjectName(QString::fromUtf8("analysisImageButton"));
        analysisImageButton->setGeometry(QRect(10, 50, 91, 23));
        dynamichintradioButton = new QRadioButton(groupBox_2);
        dynamichintradioButton->setObjectName(QString::fromUtf8("dynamichintradioButton"));
        dynamichintradioButton->setGeometry(QRect(10, 80, 91, 16));

        verticalLayout_3->addWidget(groupBox_2);


        verticalLayout->addLayout(verticalLayout_3);

        show3DPoseRadioButton = new QRadioButton(toolsdockWidgetContents);
        show3DPoseRadioButton->setObjectName(QString::fromUtf8("show3DPoseRadioButton"));

        verticalLayout->addWidget(show3DPoseRadioButton);

        DataTreeView = new DataBaseTreeView(toolsdockWidgetContents);
        DataTreeView->setObjectName(QString::fromUtf8("DataTreeView"));
        QSizePolicy sizePolicy4(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(DataTreeView->sizePolicy().hasHeightForWidth());
        DataTreeView->setSizePolicy(sizePolicy4);
        DataTreeView->setMinimumSize(QSize(0, 0));
        DataTreeView->setSizeIncrement(QSize(0, 0));

        verticalLayout->addWidget(DataTreeView);


        verticalLayout_4->addLayout(verticalLayout);

        leftdockWidget->setWidget(toolsdockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(1), leftdockWidget);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuView->menuAction());
        menubar->addAction(menuTools->menuAction());
        menubar->addAction(menuSettings->menuAction());
        menubar->addAction(menuMode->menuAction());
        menuFile->addAction(actionNew);
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionImport);
        menuFile->addAction(actionSave);
        menuFile->addAction(actionSave_2);
        menuFile->addAction(actionExit);
        menuTools->addAction(actionEncodeImageFiles);
        menuTools->addAction(actionExtractEdgeImage);
        menuTools->addAction(actionComputeContour);
        menuTools->addAction(actionTest);
        menuView->addAction(actionShow_Grid);
        menuView->addAction(actionShow_Sketch);
        menuView->addAction(actionShow_Shadow);
        menuView->addAction(actionShow_Original_Image);
        menuSettings->addAction(actionSetDisplayNearestNum);
        menuSettings->addAction(actionSetWorkingDirectory);
        menuMode->addAction(actionSketching);
        menuMode->addAction(actionViewing);

        retranslateUi(MainWindow);
        QObject::connect(actionExit, SIGNAL(triggered()), MainWindow, SLOT(exitProgram()));
        QObject::connect(actionNew, SIGNAL(triggered()), MainWindow, SLOT(newCanvas()));
        QObject::connect(actionSave, SIGNAL(triggered()), MainWindow, SLOT(saveCanvas()));
        QObject::connect(actionSave_2, SIGNAL(triggered()), MainWindow, SLOT(saveCanvas()));
        QObject::connect(actionOpen, SIGNAL(triggered()), MainWindow, SLOT(openFile()));
        QObject::connect(analysisImageButton, SIGNAL(clicked()), MainWindow, SLOT(analysisImage()));
        QObject::connect(actionImport, SIGNAL(triggered()), MainWindow, SLOT(importFile()));
        QObject::connect(actionShow_Grid, SIGNAL(triggered(bool)), MainWindow, SLOT(showGrid()));
        QObject::connect(actionSetDisplayNearestNum, SIGNAL(triggered()), MainWindow, SLOT(setDisplayNearestNum()));
        QObject::connect(actionEncodeImageFiles, SIGNAL(triggered()), MainWindow, SLOT(encodeImageFiles()));
        QObject::connect(actionShow_Original_Image, SIGNAL(toggled(bool)), MainWindow, SLOT(showOriginalImage()));
        QObject::connect(actionExtractEdgeImage, SIGNAL(triggered()), MainWindow, SLOT(extractEdgeImage()));
        QObject::connect(actionComputeContour, SIGNAL(triggered()), MainWindow, SLOT(computeContour()));
        QObject::connect(actionSetWorkingDirectory, SIGNAL(triggered()), MainWindow, SLOT(setWorkingDir()));
        QObject::connect(dynamichintradioButton, SIGNAL(toggled(bool)), MainWindow, SLOT(dynamicHint(bool)));
        QObject::connect(penmodeRadioButton, SIGNAL(toggled(bool)), MainWindow, SLOT(changeToPenMode(bool)));
        QObject::connect(erasermodeRadioButton, SIGNAL(toggled(bool)), MainWindow, SLOT(changeToEraserMode(bool)));
        QObject::connect(show3DPoseRadioButton, SIGNAL(toggled(bool)), MainWindow, SLOT(show3DPose(bool)));
        QObject::connect(viewHorizontalSlider, SIGNAL(valueChanged(int)), MainWindow, SLOT(change3DView(int)));
        QObject::connect(actionShow_Shadow, SIGNAL(triggered(bool)), MainWindow, SLOT(showShadow(bool)));
        QObject::connect(actionShow_Sketch, SIGNAL(triggered(bool)), MainWindow, SLOT(showSketch(bool)));
        QObject::connect(actionTest, SIGNAL(triggered()), MainWindow, SLOT(test()));
        QObject::connect(actionSketching, SIGNAL(triggered()), MainWindow, SLOT(setToSketchingMode()));
        QObject::connect(actionViewing, SIGNAL(triggered()), MainWindow, SLOT(setToViewingMode()));
        QObject::connect(refineButton, SIGNAL(pressed()), MainWindow, SLOT(refineFinalResultBySampling()));
        QObject::connect(replayButton, SIGNAL(pressed()), MainWindow, SLOT(replay()));
        QObject::connect(radioButton, SIGNAL(toggled(bool)), MainWindow, SLOT(setToSketchPose(bool)));
        QObject::connect(radioButton_2, SIGNAL(toggled(bool)), MainWindow, SLOT(setToSketchAnimation(bool)));
        QObject::connect(playAnimation, SIGNAL(pressed()), MainWindow, SLOT(playAnimation()));
        QObject::connect(selectTraj, SIGNAL(currentIndexChanged(int)), MainWindow, SLOT(selectTrajectory(int)));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionNew->setText(QApplication::translate("MainWindow", "New", 0, QApplication::UnicodeUTF8));
        actionSave->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
        actionSave_2->setText(QApplication::translate("MainWindow", "Save As", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0, QApplication::UnicodeUTF8));
        actionOpen->setText(QApplication::translate("MainWindow", "Open", 0, QApplication::UnicodeUTF8));
        actionImport->setText(QApplication::translate("MainWindow", "Import", 0, QApplication::UnicodeUTF8));
        actionShow_Grid->setText(QApplication::translate("MainWindow", "Show Grid", 0, QApplication::UnicodeUTF8));
        actionEncodeImageFiles->setText(QApplication::translate("MainWindow", "EncodeImageFiles", 0, QApplication::UnicodeUTF8));
        actionSetDisplayNearestNum->setText(QApplication::translate("MainWindow", "SetDisplayNearestNum", 0, QApplication::UnicodeUTF8));
        actionShow_Original_Image->setText(QApplication::translate("MainWindow", "Show Original Image", 0, QApplication::UnicodeUTF8));
        actionExtractEdgeImage->setText(QApplication::translate("MainWindow", "ExtractEdgeImage", 0, QApplication::UnicodeUTF8));
        actionComputeContour->setText(QApplication::translate("MainWindow", "ComputeContour", 0, QApplication::UnicodeUTF8));
        actionSetWorkingDirectory->setText(QApplication::translate("MainWindow", "SetWorkingDirectory", 0, QApplication::UnicodeUTF8));
        actionShow_Shadow->setText(QApplication::translate("MainWindow", "Show Shadow", 0, QApplication::UnicodeUTF8));
        actionShow_Sketch->setText(QApplication::translate("MainWindow", "Show Sketch", 0, QApplication::UnicodeUTF8));
        actionTest->setText(QApplication::translate("MainWindow", "Test", 0, QApplication::UnicodeUTF8));
        actionSketching->setText(QApplication::translate("MainWindow", "Sketching", 0, QApplication::UnicodeUTF8));
        actionViewing->setText(QApplication::translate("MainWindow", "Viewing", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QString());
        radioButton->setText(QApplication::translate("MainWindow", "Pose", 0, QApplication::UnicodeUTF8));
        radioButton_2->setText(QApplication::translate("MainWindow", "Animation", 0, QApplication::UnicodeUTF8));
        refineButton->setText(QApplication::translate("MainWindow", "Refine", 0, QApplication::UnicodeUTF8));
        replayButton->setText(QApplication::translate("MainWindow", "Replay", 0, QApplication::UnicodeUTF8));
        playAnimation->setText(QApplication::translate("MainWindow", "Play", 0, QApplication::UnicodeUTF8));
        selectTraj->clear();
        selectTraj->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "torso", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "l_hand", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "r_hand", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "l_foot", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "r_foot", 0, QApplication::UnicodeUTF8)
        );
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuTools->setTitle(QApplication::translate("MainWindow", "Tools", 0, QApplication::UnicodeUTF8));
        menuView->setTitle(QApplication::translate("MainWindow", "View", 0, QApplication::UnicodeUTF8));
        menuSettings->setTitle(QApplication::translate("MainWindow", "Settings", 0, QApplication::UnicodeUTF8));
        menuMode->setTitle(QApplication::translate("MainWindow", "Mode", 0, QApplication::UnicodeUTF8));
        leftdockWidget->setWindowTitle(QApplication::translate("MainWindow", "Tools", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "Op", 0, QApplication::UnicodeUTF8));
        penmodeRadioButton->setText(QApplication::translate("MainWindow", "Pen", 0, QApplication::UnicodeUTF8));
        erasermodeRadioButton->setText(QApplication::translate("MainWindow", "Eraser", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "ChangeView", 0, QApplication::UnicodeUTF8));
        analysisImageButton->setText(QApplication::translate("MainWindow", "AnalysisImage", 0, QApplication::UnicodeUTF8));
        dynamichintradioButton->setText(QApplication::translate("MainWindow", "DynamicHint", 0, QApplication::UnicodeUTF8));
        show3DPoseRadioButton->setText(QApplication::translate("MainWindow", "3D Pose", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
