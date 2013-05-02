#ifndef CSACUSTOMGRAPHICSVIEW_H
#define CSACUSTOMGRAPHICSVIEW_H

#include <QGraphicsView>

class CSACustomGraphicsView : public QGraphicsView
{
	Q_OBJECT

public:
	CSACustomGraphicsView(QWidget *parent);
	~CSACustomGraphicsView();

	void mousePressEvent(QMouseEvent *event);

signals:
	void setBackgroundImg(int index);
};

#endif // CSACUSTOMGRAPHICSVIEW_H
