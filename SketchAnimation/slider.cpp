#include "slider.h"
#include <Qt/qpainter.h>
#include <QtGui/QMouseEvent>
#include <QTimer>

Slider::Slider(int maximum, QWidget *parent)
	: _maximum(maximum),
	_minimum(0),
	QWidget(parent),
	_value(0),
	indexRect(122, 40, 36, 16),
	_markPosition(140, 3),
	_labelPosition_1(0, 0),
	_labelSize_1(60, 60),
	_labelPosition_2(60, 0),
	_labelSize_2(60, 60)
{
	setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}

Slider::~Slider()
{

}

QSize Slider::sizeHint() const
{
	return QSize(800, 60);
}

void Slider::paintEvent(QPaintEvent *event)
{
	_interval = (double)(width() - _markPosition.x() - 30) / _maximum;
	QPainter* painter = new QPainter();
	painter->setRenderHint(QPainter::Antialiasing);
	painter->begin(this);
	painter->setPen(Qt::white);

	painter->fillRect(QRect(_labelPosition_1, _labelSize_1), Qt::lightGray);
	painter->drawText(QRect(_labelPosition_1, _labelSize_1), Qt::AlignCenter, "Action");

	painter->fillRect(QRect(_labelPosition_2, _labelSize_2), Qt::lightGray);
	painter->drawText(QRect(_labelPosition_2, QSize(_labelSize_2.width() / 2, _labelSize_2.height() / 2)), Qt::AlignCenter, "Now");
	painter->drawText(QRect(_labelPosition_2.x(), _labelPosition_2.y() + _labelSize_2.height() / 2, _labelSize_2.width() / 2, _labelSize_2.height() / 2), Qt::AlignCenter, "End");
	painter->drawText(QRect(_labelPosition_2.x() + _labelSize_2.width() / 2, _labelPosition_2.y(), _labelSize_2.width() / 2, _labelSize_2.height() / 2), Qt::AlignCenter, QString("%1").arg(_value));
	painter->drawText(QRect(_labelPosition_2.x() + _labelSize_2.width() / 2, _labelPosition_2.y() + _labelSize_2.height() / 2, _labelSize_2.width() / 2, _labelSize_2.height() / 2), Qt::AlignCenter, QString("%1").arg(_maximum));

	markRect.setRect(_labelPosition_1.x() + _labelSize_1.width() + _labelSize_2.width(), _labelPosition_1.y(), width() - _labelPosition_1.x() - _labelSize_1.width() - _labelSize_2.width(), height());
	
	painter->fillRect(markRect, Qt::black);
	painter->fillRect(QRect(markRect.topLeft().x() + 1, markRect.topLeft().y() + 1, markRect.width() - 3, markRect.height() - 3), Qt::white);
	painter->fillRect(QRect(markRect.topLeft().x() + 2, markRect.topLeft().y() + 2, markRect.width() - 4, markRect.height() - 4), Qt::gray);
	painter->fillRect(QRect(markRect.topLeft().x() + 3, markRect.topLeft().y() + 3, markRect.width() - 5, markRect.height() - 5), Qt::lightGray);

	double x = _markPosition.x();
	for (int j = 0; j != _maximum + 1; ++j)
	{
		if (j % 12 == 0)
		{
			painter->drawLine(QPointF(x, _markPosition.y() + 3), QPointF(x, _markPosition.y() + 26));
			painter->setPen(Qt::black);
			painter->drawText(QRect(x + 3, _markPosition.y() + 20, 20, 20), QString("%1").arg(j));
			painter->setPen(Qt::white);
		}
		else if (j % 6 == 0)
			painter->drawLine(QPointF(x, _markPosition.y() + 3), QPointF(x, _markPosition.y() + 18));
		else
			painter->drawLine(QPointF(x, _markPosition.y() + 3), QPointF(x, _markPosition.y() + 10));
		x += _interval;
	}

	painter->setPen(Qt::lightGray);

	x = _markPosition.x();
	for (int j = 0; j != _maximum + 1; ++j)
	{
		if (j % 12 == 0)
		{
			painter->drawLine(QPointF(x - 1, _markPosition.y() + 3), QPointF(x - 1, _markPosition.y() + 26));
		}
		else if (j % 6 == 0)
			painter->drawLine(QPointF(x - 1, _markPosition.y() + 3), QPointF(x - 1, _markPosition.y() + 18));
		else
			painter->drawLine(QPointF(x - 1, _markPosition.y() + 3), QPointF(x - 1, _markPosition.y() + 10));
		x += _interval;
	}

	painter->fillRect(indexRect, QColor(255, 255, 255, 127));
	painter->fillRect(QRect(indexRect.topLeft().x() + 1, indexRect.topLeft().y() + 1, indexRect.width(), indexRect.height()), QColor(0, 0, 0, 127));
	painter->setPen(Qt::white);
	painter->drawLine(QPoint(indexRect.topLeft().x() + indexRect.width() / 2, indexRect.top()), QPoint(indexRect.topLeft().x() + indexRect.width() / 2, indexRect.bottom()));

	painter->end();
}

void Slider::mousePressEvent(QMouseEvent *event)
{
	if (markRect.contains(event->pos()))
	{
		_value = _ground(event->x());
		indexRect.moveTo(_markPosition.x() + _ground(event->x()) * _interval - indexRect.width() / 2, indexRect.y());
	}
	_MousePressed = true;
	update();
}

void Slider::mouseMoveEvent(QMouseEvent *event)
{
	if (_MousePressed)
	{
		if (markRect.contains(event->pos()))
		{
			_value = _ground(event->x());
			indexRect.moveTo(_markPosition.x() + _ground(event->x()) * _interval - indexRect.width() / 2, indexRect.y());
		}
	}
	update();
}

void Slider::keyPressEvent(QKeyEvent *event)
{
	if (event->key() == Qt::Key_Plus || event->key() == Qt::Key_Right)
	{
		setValue(_value + 1);
	}
	else if (event->key() == Qt::Key_Minus || event->key() == Qt::Key_Left)
	{
		setValue(_value - 1);
	}
	else if (event->key() == Qt::Key_Space)
	{
		setValue(0);
		QTimer* timer = new QTimer;
		timer->setInterval(1000 / 24);
		this->connect(this, SIGNAL(valueMaximum()), timer, SLOT(stop()));
		this->connect(timer, SIGNAL(timeout()), this, SLOT(addValue()));
		timer->start();
	}
}

int Slider::_ground(int x)
{
	int result = (x - _markPosition.x()) / _interval;
	if (result >= -1 && result <= _maximum)
		return result;
	else
		return result < -1 ? -1 : _maximum;
}

void Slider::setValue(int value)
{
	_value = value >= -1 && value <= _maximum ? value : value < -1 ? -1 : _maximum;
	indexRect.moveTo(_markPosition.x() + _value * _interval - indexRect.width() / 2, indexRect.y());
	emit valueChanged(_value);
	if (_value ==_maximum)
	{
		emit valueMaximum();
	}
	update();
}

void Slider::addValue()
{
	setValue(_value + 1);
}