/********************************************************************************
** Form generated from reading UI file 'skeletonview.ui'
**
** Created: Mon May 27 10:59:35 2013
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SKELETONVIEW_H
#define UI_SKELETONVIEW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <view3d.h>

QT_BEGIN_NAMESPACE

class Ui_SkeletonView
{
public:

    void setupUi(View3D *SkeletonView)
    {
        if (SkeletonView->objectName().isEmpty())
            SkeletonView->setObjectName(QString::fromUtf8("SkeletonView"));
        SkeletonView->resize(400, 300);

        retranslateUi(SkeletonView);

        QMetaObject::connectSlotsByName(SkeletonView);
    } // setupUi

    void retranslateUi(View3D *SkeletonView)
    {
        SkeletonView->setWindowTitle(QApplication::translate("SkeletonView", "SkeletonView", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SkeletonView: public Ui_SkeletonView {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SKELETONVIEW_H
