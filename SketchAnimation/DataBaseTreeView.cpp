#include "DataBaseTreeView.h"

DataBaseTreeView::DataBaseTreeView(QWidget *parent /* = 0 */)
	: QTreeView(parent)
{

}

QModelIndexList DataBaseTreeView::GetSelectedIndexes()const
{
	return selectedIndexes();
}
