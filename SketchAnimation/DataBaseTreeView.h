#ifndef _DATA_BASE_TREE_VIEW_
#define _DATA_BASE_TREE_VIEW_

#include <QWidget>
#include <QTreeView>

class DataBaseTreeView : public QTreeView
{
public:
	explicit DataBaseTreeView(QWidget *parent = 0);
	~DataBaseTreeView(){}

public:
	QModelIndexList GetSelectedIndexes()const;
};

#endif