#include "csacustomgraphicsview.h"
#include <QMouseEvent>
#include "csacustomgraphicspixmapitem.h"

CSACustomGraphicsView::CSACustomGraphicsView(QWidget *parent)
	: QGraphicsView(parent)
{

}

CSACustomGraphicsView::~CSACustomGraphicsView()
{

}

void CSACustomGraphicsView::mousePressEvent(QMouseEvent *event)
{
	QPoint point = event->pos();
	
	QGraphicsItem* p_graphicsItem = itemAt(point.x(),point.y());

	if(p_graphicsItem)
	{
		QGraphicsPixmapItem* p_graphicspixmapitem = dynamic_cast<QGraphicsPixmapItem*>(p_graphicsItem);
		CSACustomGraphicsPixmapItem* p_customgraphicspixmapitem = dynamic_cast<CSACustomGraphicsPixmapItem*>(p_graphicspixmapitem);

		int index = p_customgraphicspixmapitem->getImageIndex();

		emit setBackgroundImg(index);
	}

	QGraphicsView::mousePressEvent(event);
}