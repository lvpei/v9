/*
	This class is used to encode the database images, whose purpose is to avoid reading files from the disk frequently

	Author: lvp
	Date: 2012-3-6
*/
#pragma once

#include <vector>

class CSAEncodeImages
{
public:
	CSAEncodeImages(void);
	CSAEncodeImages(const char* folder);
	~CSAEncodeImages(void);
	bool loadFromFile(const char* filename);
	bool saveToFile(const char* filename);

	int m_iImageNum;
	std::vector< std::vector<int> > m_iPointNums;  
};

