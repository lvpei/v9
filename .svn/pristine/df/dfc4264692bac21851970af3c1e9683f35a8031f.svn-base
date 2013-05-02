#include "csacustomgraphicspixmapitem.h"
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOption>

CSACustomGraphicsPixmapItem::CSACustomGraphicsPixmapItem(QGraphicsScene *parent)
	: QGraphicsPixmapItem(0,parent)
{
	m_nImageIndex = 0;

	// more flexible selection than BouudingMaskShape
	setShapeMode(QGraphicsPixmapItem::BoundingRectShape);

	m_bPressed = false;
}

CSACustomGraphicsPixmapItem::~CSACustomGraphicsPixmapItem()
{

}


// set the image index of database images related to this graphics item 
void CSACustomGraphicsPixmapItem::setImageIndex(int index)
{
	m_nImageIndex = index;
}


// get the image index of database images related to this graphics item 
int CSACustomGraphicsPixmapItem::getImageIndex(void)
{
	return m_nImageIndex;
}

void CSACustomGraphicsPixmapItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	QPixmap px = pixmap();
	QSize sz = px.size();
	
	painter->setPen(QPen(Qt::black, 0));	
	painter->setBrush(Qt::darkGreen);

	if (m_bPressed) {
		painter->drawRect(0,0,sz.width(),sz.height());
		painter->drawPixmap(0,0,sz.width(),sz.height(),px);
	}
	else
		painter->drawPixmap(0,0,px);
}

void CSACustomGraphicsPixmapItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	m_bPressed = true;
	update();
}
void CSACustomGraphicsPixmapItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	m_bPressed = false;
	update();
}
