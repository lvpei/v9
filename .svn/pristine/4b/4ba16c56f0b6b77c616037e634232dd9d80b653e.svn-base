#include "SAView.h"
#include <QDebug>
#include "SACamera.h"
#include "SAImageProcess.h"

CSAView::CSAView(void)
{
	m_nDBNum = 0;

	m_pCamera = NULL;

	m_bInvFileLoaded = false;

	m_vDataPts = NULL;
	m_vIdxArr = NULL;
	m_vDistArr = NULL;
	m_pKdtree = NULL;

	// for present, there is about 7 limbs
	vector<Mat> emptyMat;
	m_vHistDbLimb.assign(7,emptyMat);
}

CSAView::~CSAView(void)
{
	// delete the camera
	if(m_pCamera)
		delete m_pCamera;
	m_pCamera = NULL;

	//  delete the previous inverse file content
	if(m_bInvFileLoaded)
	{
		for(int i = 0; i < _ARRAY_NUM_; i++)
		{
			for(int j = 0; j < _ARRAY_NUM_; j++)
			{
				for(int k = 0; k < _ARRAY_NUM_; k++)
				{
					delete []m_pInvArr[i][j][k];
				}
			}
		}
		m_nDBNum = 0;
		// clear the array memory
		//memset((char*)invnum,0,sizeof(int) * _ARRAY_NUM_ * _ARRAY_NUM_ * _ARRAY_NUM_);
	}

	if(m_vDataPts)
	{
		annDeallocPts(m_vDataPts);
		m_vDataPts = NULL;
	}

	if(m_vIdxArr)
	{
		delete []m_vIdxArr;
		m_vIdxArr = NULL;
	}
	if(m_vDistArr)
	{
		delete []m_vDistArr;
		m_vDistArr = NULL;
	}

	delete m_pKdtree;
	m_pKdtree = NULL;

	// close ANN
	annClose();
}

/*
	load the edge magnitude from file
*/
void CSAView::load_edgemag_from(const char* filename)
{
	m_vNormMagnitude.clear();

	qDebug("loading the magnitude file %s",filename);
	int filenum = 0;
	FileStorage fs(filename,FileStorage::READ);
	fs["filenum"]>>filenum;
	//m_nDBNum = filenum;
	char name[20];
	for(int l = 0;l < filenum; l++)
	{
		Mat mat;
		sprintf(name,"image_%d_m_n",l);
		string str = name;
		fs[str]>>mat;
		m_vNormMagnitude.push_back(mat);
	}
	fs.release();
	qDebug("decomposed magnitude file loaded");
}

/*
load the edge orientation from file
*/
void CSAView::load_edgeorien_from(const char* filename)
{
	m_vOrientation.clear();

	qDebug("loading the orientation file %s",filename);
	int filenum = 0;
	FileStorage fs(filename,FileStorage::READ);
	fs["filenum"]>>filenum;
	//m_nDBNum = filenum;
	char name[20];
	for(int l = 0;l < filenum; l++)
	{
		Mat mat;
		sprintf(name,"image_%d_o",l);
		string str = name;
		fs[str]>>mat;
		m_vOrientation.push_back(mat);
	}
	fs.release();
	qDebug("orientation file loaded");
}

// load the histogram data from file
void CSAView::load_hist_from(const char* filename)
{
	// commented by lvp 12-11-29
	//m_vHistDb.clear();

	qDebug("loading the histogram file %s",filename);
	int filenum = 0;
	FileStorage fs(filename,FileStorage::READ);
	fs["filenum"]>>filenum;
	m_nDBNum = filenum + m_vHistDb.size();
	char bin_name[20];

	// which kind of feature is loaded
	FEATURE feature;
	string str;
	fs["feature_type"]>>str;
	if(str.compare("BICE") == 0)
		feature = BICE;
	else if(str.compare("HOUGH") == 0)
		feature = HOUGH;
	else if(str.compare("ARP") == 0)
		feature = ARP;

	if(feature == BICE)
	{
		for(int l = 0;l < filenum; l++)
		{
			for(int k = 0; k < 64;k++)
			{
				Mat mat;
				sprintf(bin_name,"r%d_c%d",l,k);
				string str = bin_name;
				fs[str]>>mat;
				m_vHistDb.push_back(mat);
			}
		}
	}
	else if(feature == HOUGH)
	{
		for(int l = 0;l < filenum; l++)
		{
			Mat mat;
			sprintf(bin_name,"r_%i",l);
			string str = bin_name;
			fs[str]>>mat;
			m_vHistDb.push_back(mat);
		}
	}
	else if(feature == ARP)
	{
		for(int l = 0;l < filenum; l++)
		{
			Mat mat;
			sprintf(bin_name,"r_%i",l);
			string str = bin_name;
			fs[str]>>mat;
			m_vHistDb.push_back(mat);
		}
	}

	fs.release();
	qDebug("histogram file loaded");

	/**
		The following code is in charge of generating data points for different database
	*/
	/*
	int rows,cols,nPts;
	rows = m_vHistDb[0].rows;
	cols = m_vHistDb[0].cols;
	int d = rows * cols;
	nPts = m_vHistDb.size();

	Mat rowVec(nPts,d,CV_32SC1);

	// change the original histogram data into row-vector matrix
	for(int m = 0; m < nPts; m++)
	{
		int col = 0;
		for(int i = 0; i < rows; i++)
		{
			for(int j = 0; j < cols; j++)
			{
				rowVec.at<int>(m,col++) = m_vHistDb[m].at<int>(i,j);
			}
		}
	}

	ofstream out("db_data.txt");
	for(int i = 0; i < rowVec.rows; i++)
	{
		for(int j = 0; j < rowVec.cols; j++)
		{	
			out<<rowVec.at<int>(i,j)<<" ";
		}
		out<<endl;
	}
	//*/

	//PCA pca(rowVec, Mat(),0);

	//float total = 0.0, sum = 0.0, percent = 0.95;
	//for(int i = 0; i < pca.eigenvalues.rows; i++)
	//{
	//	
	//	total += pca.eigenvalues.at<float>(i,0);
	//}
	//for(int j = 0; j < pca.eigenvalues.rows; j++)
	//{
	//	sum += pca.eigenvalues.at<float>(j,0);

	//	if(sum / total >= percent)
	//	{
	//		m_nEigenNum = j;
	//		break;
	//	}
	//}
	//cout<<"keep "<< percent<< " info need "<<m_nEigenNum<<" eigenvalues"<<endl;

	//// assign the PCA structure
	//m_PCA = pca;

	//m_nEigenNum = 2;
	///*
	//	construct the KD-tree for low-dimensional data
	//*/
	//m_vDataPts = annAllocPts(nPts,m_nEigenNum);
	//Mat subspace;
	//m_PCA.project(rowVec,subspace);
	//for(int m = 0; m < nPts; m++)
	//{
	//	for(int i = 0; i < m_nEigenNum; i++)
	//	{
	//			m_vDataPts[m][i] = subspace.at<float>(m,i);
	//	}
	//}
	//qDebug("descriptors converted to high-dimensional points");

	//// construct search structure
	//qDebug("construct KD-tree...");
	//m_vIdxArr = new ANNidx[nPts];
	//m_vDistArr = new ANNdist[nPts];
	//m_pKdtree = new ANNkd_tree(m_vDataPts,nPts,m_nEigenNum);

	//qDebug("KD-tree constructed");
	////*/
}

/* 
	load the histogram data from file for each limb
*/
void CSAView::load_limb_hist_from(const char* filename, int limb_idx)
{
	assert(m_vHistDbLimb.size() > limb_idx);

	qDebug("loading the histogram file %s",filename);
	int filenum = 0;
	FileStorage fs(filename,FileStorage::READ);
	if(!fs.isOpened())
	{
		qDebug("fail to load the histogram file");
		return;
	}

	fs["filenum"]>>filenum;
	char bin_name[20];

	// which kind of feature is loaded
	FEATURE feature;
	string str;
	fs["feature_type"]>>str;
	if(str.compare("BICE") == 0)
		feature = BICE;
	else if(str.compare("HOUGH") == 0)
		feature = HOUGH;
	else if(str.compare("ARP") == 0)
		feature = ARP;

	if(feature == BICE)
	{
		for(int l = 0;l < filenum; l++)
		{
			for(int k = 0; k < 64;k++)
			{
				Mat mat;
				sprintf(bin_name,"r%d_c%d",l,k);
				string str = bin_name;
				fs[str]>>mat;
				m_vHistDbLimb[limb_idx].push_back(mat);
			}
		}
	}
	else if(feature == HOUGH)
	{
		for(int l = 0;l < filenum; l++)
		{
			Mat mat;
			sprintf(bin_name,"r_%i",l);
			string str = bin_name;
			fs[str]>>mat;
			m_vHistDbLimb[limb_idx].push_back(mat);
		}
	}
	else if(feature == ARP)
	{
		for(int l = 0;l < filenum; l++)
		{
			Mat mat;
			sprintf(bin_name,"r_%i",l);
			string str = bin_name;
			fs[str]>>mat;
			m_vHistDbLimb[limb_idx].push_back(mat);
		}
	}

	fs.release();
	qDebug("histogram file loaded");
}

/*
load the inverted structure file
*/
/*
void CSAView::loadInvFile(const char* filename)
{
// allocate the memory
m_pInvArr.resize(_ARRAY_NUM_);
for(int i = 0; i < _ARRAY_NUM_;i++)
{
m_pInvArr[i].resize(_ARRAY_NUM_);
for(int j = 0; j < _ARRAY_NUM_; j++)
{
m_pInvArr[i][j].assign(_ARRAY_NUM_,NULL);
}
}

ifstream in;
in.open(filename);
if(!in.is_open())
{
qDebug()<<"inverse structure file load failed";
return;
}
qDebug("loading the inv file %s",filename);

in>>m_nDBNum;

// clear the array memory
memset((char*)invnum,0,sizeof(int) * _ARRAY_NUM_ * _ARRAY_NUM_ * _ARRAY_NUM_);

for(int i = 0; i < m_nDBNum; i++)
{
for(int j = 0; j < 64; j++)
{
int value[32];
for(int k = 0; k < 32; k++)
in>>value[k];

// fill the structure
for(int l = 0; l < 10; l++)
{
int index1 = value[l*3];
int index2 = value[l*3+1];
int index3 = value[l*3+2];

if(index1 == -1 || index2 == -1 || index3 == -1 )
continue;
else
{
// the element number stored in [value[l*3] value[l*3+1]value[l*3+1] ] coordinate
int curnum = invnum[index1][index2][index3];
bool exist = false;
int* imgpatchindex = m_pInvArr[index1][index2][index3];
for(int m = 0; m < curnum; m++)
{
if(imgpatchindex[m*2] == value[30] && imgpatchindex[m*2+1] == value[31])
{	
exist = true;
break;
}
}
if(!exist)
{
int* newimgpatchindex = new int[curnum*2 +2];
if(curnum != 0)
{	
memcpy((char*)newimgpatchindex,(char*)imgpatchindex,sizeof(int) * curnum*2);
delete []imgpatchindex;
imgpatchindex = NULL;
}
invnum[index1][index2][index3] = curnum + 1;
newimgpatchindex[curnum*2] = value[30];
newimgpatchindex[curnum*2+1] = value[31];

m_pInvArr[index1][index2][index3] = newimgpatchindex;
}
}
}
}
}
m_bInvFileLoaded = true;
qDebug()<<"inverse structure file loaded";
}
//*/

/*
load lines from file 
*/
void CSAView::load_lines_from(const char* filename)
{
	// read the database image
	ifstream in(filename);

	if(!in.is_open())
		return;

	// the image number in our database
	int numOfImages;
	// the line number in one image 
	int numOfLines;

	in>>numOfImages;

	for(int i =0; i < numOfImages; i++)
	{
		in>>numOfLines;
		vector<Vec4i> lines;
		Vec4i line;
		for(int j = 0; j < numOfLines; j++)
		{
			in>>line[0];
			in>>line[1];
			in>>line[2];
			in>>line[3];
			lines.push_back(line);
		}

		m_vLines.push_back(lines);
	}
	qDebug("lines file loaded");

	//ofstream out("screecapture1_db_data_multi.txt");

	// only 3 resolutions
	/*
	int rho_res[] = {1,5,10,15};
	int theta_res[] = {1,2,4,6};
	CSAImageProcess imageprocess;

	for(int j = 0; j < m_nDBNum; j++)
	{
	// the database patch at the same position
	const vector<Vec4i>& lines_db = m_vLines[j];

	// construct different resolution histograms
	Mat dbpatch32F;
	for(int m = 0; m < 3; m++)
	{
	imageprocess.lines2Histogram(lines_db,rho_res[m],theta_res[m],dbpatch32F);

	// change the original histogram data into row-vector matrix
	int col = 0;
	for(int i = 0; i < dbpatch32F.rows; i++)
	{
	for(int j = 0; j < dbpatch32F.cols; j++)
	{
	out<<dbpatch32F.at<float>(i,j)<<" ";
	}
	}
	}
	out<<endl;
	}
	//*/

	// multi-resolutions(25)
	//CSAImageProcess imageprocess;
	//for(int j = 0; j < m_nDBNum; j++)
	//{
	//	// the database patch at the same position
	//	const vector<Vec4i>& lines_db = m_vLines[j];

	//	// construct different resolution histograms
	//	Mat dbpatch32F;

	//	// more resolutions
	//	//*
	//	int rho_res= 1;
	//	for(int m = 0; m < 5; m++)
	//	{
	//		rho_res += m * 5;
	//		int theta_res = 1;
	//		for(int n = 0; n < 5; n++)
	//		{
	//			theta_res += n * 2;

	//			imageprocess.lines2Histogram(lines_db,rho_res,theta_res,dbpatch32F);

	//			// change the original histogram data into row-vector matrix
	//			int col = 0;
	//			for(int i = 0; i < dbpatch32F.rows; i++)
	//			{
	//				for(int j = 0; j < dbpatch32F.cols; j++)
	//				{
	//					out<<dbpatch32F.at<float>(i,j)<<" ";
	//				}
	//			}
	//			out<<endl;
	//		}
	//	}
	//}
}
