#include "SAEncodeImages.h"
#include <QDir>
#include <QImage>
#include <fstream>
using namespace std;

const int image_max_num = 10000;

CSAEncodeImages::CSAEncodeImages(void)
{
	m_iImageNum = 0;
	m_iPointNums.resize(image_max_num);
}


CSAEncodeImages::CSAEncodeImages(const char* folder)
{
	m_iImageNum = 0;
	m_iPointNums.resize(image_max_num);

	QDir dir(folder);
	if(!dir.exists())
		return;
	
	QImage srcImage;
	QString fileName;
	dir.setFilter(QDir::Files);
	QFileInfoList list = dir.entryInfoList();
	int i = 0,j = 0;
	while(i < list.size())
	{
		QFileInfo fileInfo = list.at(i++);
		fileName = fileInfo.fileName();
		if(fileName.section('.',-1) != "png" && fileName.section('.',-1) != "PNG")
			continue;
		fileName = fileInfo.path() + "/"+fileName;
		srcImage.load(fileName,"png");

		for(int m = 0; m < srcImage.height(); m++)
			for(int n = 0; n < srcImage.width(); n++)
			{
				QRgb color = srcImage.pixel(m,n);
				if( qRed(color) == 0)
				{
					m_iPointNums[j].push_back(m);
					m_iPointNums[j].push_back(n);
				}
			}
		j++;
	}
	m_iImageNum = j;
}
CSAEncodeImages::~CSAEncodeImages(void)
{
}

bool CSAEncodeImages::loadFromFile(const char* filename)
{
	ifstream in;
	in.open(filename);
	if(!in.is_open())
		return false;

	// read the file amount
	in>>m_iImageNum;	
	int pointnum,a,b;
	for(int i = 0; i < m_iImageNum; i++)
	{
		in>>pointnum;
		for(int j = 0; j < pointnum / 2; j++)
		{	
			in>>a>>b;
			m_iPointNums[i].push_back(a);
			m_iPointNums[i].push_back(b);
		}
	}
	
	in.close();

	return true;
}
bool CSAEncodeImages::saveToFile(const char* filename)
{
	ofstream out(filename);
	if(!out.is_open())
		return false;

	out<<m_iImageNum<<" ";
	for(int i = 0; i < m_iImageNum; i++)
	{
		out<<m_iPointNums[i].size()<<" ";
		for(int j = 0; j < m_iPointNums[i].size(); j = j+2)
		{
			out<<m_iPointNums[i][j]<<" "<<m_iPointNums[i][j+1]<<" ";
		}
	}

	return true;
}
