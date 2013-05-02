#ifndef CSANEARESTNUM_H
#define CSANEARESTNUM_H

#include <QDialog>
#include "ui_csanearestnum.h"

class CSANearestNum : public QDialog
{
	Q_OBJECT

public:
	CSANearestNum(QWidget *parent = 0);
	~CSANearestNum();
	int getNearestNum(){return m_iNearestNum;}
	public slots:
		void setNearestNum(int num);
private:
	Ui::CSANearestNum ui;
	int m_iNearestNum;

};

#endif // CSANEARESTNUM_H
