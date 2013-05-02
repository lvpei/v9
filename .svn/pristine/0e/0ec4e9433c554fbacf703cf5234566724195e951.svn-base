/*
	This class is used to encode the database images by vectorization, whose purpose is to avoid reading files from the disk frequently

	Author: lvp
	Date: 2012-11-19
*/
#pragma once

#include <vector>
#include <string>

class CSAEncodeVectorImages
{
public:
	CSAEncodeVectorImages(void);
	~CSAEncodeVectorImages(void);

	bool loadFromFile(const char* filename);

	int m_iImageNum;
	std::vector< std::vector<int> >		m_iJointPosition; 
	std::vector< std::string >			m_sJointName;
};

