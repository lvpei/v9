/********************************************************************************
** Form generated from reading UI file 'csanearestnum.ui'
**
** Created: Mon May 27 10:59:35 2013
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CSANEARESTNUM_H
#define UI_CSANEARESTNUM_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QSpinBox>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CSANearestNum
{
public:
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpinBox *nearestNumSpinBox;

    void setupUi(QDialog *CSANearestNum)
    {
        if (CSANearestNum->objectName().isEmpty())
            CSANearestNum->setObjectName(QString::fromUtf8("CSANearestNum"));
        CSANearestNum->resize(247, 83);
        layoutWidget = new QWidget(CSANearestNum);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 30, 201, 22));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        nearestNumSpinBox = new QSpinBox(layoutWidget);
        nearestNumSpinBox->setObjectName(QString::fromUtf8("nearestNumSpinBox"));
        nearestNumSpinBox->setMinimum(0);
        nearestNumSpinBox->setMaximum(200);
        nearestNumSpinBox->setValue(50);

        horizontalLayout->addWidget(nearestNumSpinBox);


        retranslateUi(CSANearestNum);
        QObject::connect(nearestNumSpinBox, SIGNAL(valueChanged(int)), CSANearestNum, SLOT(setNearestNum(int)));

        QMetaObject::connectSlotsByName(CSANearestNum);
    } // setupUi

    void retranslateUi(QDialog *CSANearestNum)
    {
        CSANearestNum->setWindowTitle(QApplication::translate("CSANearestNum", "CSANearestNum", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("CSANearestNum", "NearestNum", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class CSANearestNum: public Ui_CSANearestNum {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CSANEARESTNUM_H
