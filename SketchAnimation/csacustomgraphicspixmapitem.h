#ifndef CSACUSTOMGRAPHICSPIXMAPITEM_H
#define CSACUSTOMGRAPHICSPIXMAPITEM_H

#include <QGraphicsPixmapItem>

class QGraphicsScene;

class CSACustomGraphicsPixmapItem : public QGraphicsPixmapItem
{
public:
	CSACustomGraphicsPixmapItem(QGraphicsScene *parent);
	~CSACustomGraphicsPixmapItem();

	// set the image index of database images related to this graphics item 
	void setImageIndex(int index);
	// get the image index of database images related to this graphics item 
	int getImageIndex(void);

	// draw the item
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	// deal with mouse event
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
	int m_nImageIndex;
	bool m_bPressed;
};

#endif // CSACUSTOMGRAPHICSPIXMAPITEM_H
