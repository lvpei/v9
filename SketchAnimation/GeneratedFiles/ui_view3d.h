/********************************************************************************
** Form generated from reading UI file 'view3d.ui'
**
** Created: Mon May 27 10:59:35 2013
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VIEW3D_H
#define UI_VIEW3D_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtOpenGL/QGLWidget>

QT_BEGIN_NAMESPACE

class Ui_View3D
{
public:

    void setupUi(QGLWidget *View3D)
    {
        if (View3D->objectName().isEmpty())
            View3D->setObjectName(QString::fromUtf8("View3D"));
        View3D->resize(400, 300);

        retranslateUi(View3D);

        QMetaObject::connectSlotsByName(View3D);
    } // setupUi

    void retranslateUi(QGLWidget *View3D)
    {
        View3D->setWindowTitle(QApplication::translate("View3D", "View3D", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class View3D: public Ui_View3D {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VIEW3D_H
