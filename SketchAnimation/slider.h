#ifndef SLIDER_H
#define SLIDER_H

#include <QtGui/QWidget>
#include <QtGui/QLineEdit>
#include <QtGui/QLabel>

class Slider : public QWidget
{
	Q_OBJECT

public:
	Slider(int value = 80, QWidget *parent = 0);
	~Slider();

public:
	QSize sizeHint() const;
	void setMaximum(int maximum){_maximum = maximum;};
	int getMaximum(){return _maximum;};
	void setMinimum(int minimun){_minimum = minimun;};
	int getMinimum(){return _minimum;};
	int getValue(){return _value;};
	void setRange(int min, int max){setMinimum(min); setMaximum(max);};

protected:
	void paintEvent(QPaintEvent *event);
	void mousePressEvent(QMouseEvent *);
	void mouseMoveEvent(QMouseEvent *);
	void keyPressEvent(QKeyEvent *);

private:
	int _ground(int x);

signals:
	void valueChanged(int newValue);
	void valueMaximum();

public slots:
	void setValue(int value);
	void addValue();

private:
	int _maximum;
	int _minimum;
	int _value;
	double _interval;
	QRect markRect;
	QRect indexRect;
	
	QPoint _markPosition;
	QPoint _labelPosition_1;
	QSize _labelSize_1;
	QPoint _labelPosition_2;
	QSize _labelSize_2;

	bool _MousePressed;
};

#endif // SLIDER_H
