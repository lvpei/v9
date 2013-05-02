#include "csanearestnum.h"

CSANearestNum::CSANearestNum(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	m_iNearestNum = 30;
}

CSANearestNum::~CSANearestNum()
{

}

void CSANearestNum::setNearestNum(int num)
{
	m_iNearestNum = num;
	ui.nearestNumSpinBox->setValue(m_iNearestNum);
}