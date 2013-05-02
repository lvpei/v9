/*
	This class is used to keep the data for one view and used by CSAImageProcess class

	Author:		lvp
	Date:		2012-5-10
	Modified:	2012-5-19 add inverted file structure
*/
#pragma once

#include <opencv2/core/core.hpp>
#include <vector>
#include "ANN/ANN.h"

using namespace cv;
using namespace std;

/* 
const int _ARRAY_NUM_ = 400;
const int _MAX_UPDATED_PATCH_ = 5;
const int _MAX_DB_IMG_ = 800;
//*/

//* enable this when encoding files or extract edge image
const int _ARRAY_NUM_ = 1;
const int _MAX_UPDATED_PATCH_ = 1;
const int _MAX_DB_IMG_ = 1;
//*/

const int _PATCH_INDEX_RANGE_ = 64;

// The feature used by Image Process
enum FEATURE{BICE,HOUGH,ARP};

// CSACamera class declaration
class CSACamera;

class CSAView
{
public:
	CSAView(void);
	~CSAView(void);
	
	/*
		The histogram for database image
	*/
	vector<Mat> m_vHistDb;
	
	/*
		The histogram of each limb for the whole character pose
	*/
	vector< vector<Mat> > m_vHistDbLimb;

	/*
		store the database histogram as a high-dimensional vector for ANN search
	*/
	ANNpointArray m_vDataPts;
	
	/*
		near neighbor indices
	*/
	ANNidxArray m_vIdxArr;
	
	/*
		near neighbor distance
	*/
	ANNdistArray m_vDistArr;
	
	/*
		search structure
	*/
	ANNkd_tree* m_pKdtree;

	PCA m_PCA;
	int m_nEigenNum;
	/*
		The normalized magnitude for database image
	*/
	vector<Mat> m_vNormMagnitude;

	/*
		The orientation for each pixel in the database image
	*/
	vector<Mat> m_vOrientation;

	/*
		The decomposed normalized magnitude for database image
	*/
	vector<Mat> m_vDecNormMagnitude;

	/*
		The camera info
	*/
	CSACamera* m_pCamera;

	/*
		The image number in database
	*/
	int	m_nDBNum;	

	/*
		inverted file structure
	*/
	vector<vector<vector<int*>>> m_pInvArr;							
	
	/*
		the <img,patch> pair number in the m_pInvArr array 
	*/
	//int	invnum[_ARRAY_NUM_][_ARRAY_NUM_][_ARRAY_NUM_];
	
	// all the lines in database images
	vector<vector<Vec4i>>	m_vLines;

private:
	bool m_bInvFileLoaded;	// whether the inverted file loaded

public:
	/* 
		load the histogram data from file
	*/
	void load_hist_from(const char* filename);
	
	/* 
		load the histogram data from file for each limb
	*/
	void load_limb_hist_from(const char* filename, int limb_idx);

	/*
	    load the edge magnitude from file
	*/
	void load_edgemag_from(const char* filename);

	/*
	    load the edge magnitude from file
	*/
	void load_edgeorien_from(const char* filename);

	/*
	    load the decomposed edge magnitude from file
	*/
	void load_dec_edgemag_from(const char* filename);

	/*
		load the inverted structure file
	*/
	//void loadInvFile(const char* filename);

	/*
		load lines from file 
	*/
	void load_lines_from(const char* filename);
};

