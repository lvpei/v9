#include "SAEncodeVectorImages.h"
#include <QDir>
#include <fstream>
using namespace std;

CSAEncodeVectorImages::CSAEncodeVectorImages(void)
{
	m_iImageNum = 0;
}


CSAEncodeVectorImages::~CSAEncodeVectorImages(void)
{
}

bool CSAEncodeVectorImages::loadFromFile(const char* filename)
{
	ifstream in;
	in.open(filename);
	if(!in.is_open())
		return false;
	
	char buffer[2048];
	memset(buffer,0,2048);

	// read the joint name
	while(in>>buffer)
	{
		if(isdigit(buffer[0]))
			break;
		m_sJointName.push_back(buffer);
	}

	m_iImageNum = atoi(buffer);
	m_iJointPosition.resize(m_iImageNum);

	// read the data
	for(int i = 0; i < m_iImageNum; i++)
	{
		for(int j = 0; j < m_sJointName.size() * 2; j++)
		{
			in>>buffer;
			int x = atoi(buffer);
			m_iJointPosition[i].push_back(x);
		}
	}

	in.close();

	return true;
}
