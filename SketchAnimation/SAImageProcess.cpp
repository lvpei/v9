#include "SAImageProcess.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <QDebug>
#include <QDir>
#include <QTime>
#include <time.h>
#include <fstream>
#include ".\LSH\SAE2LSH.h"

using namespace std;

#define  USE_LESS_RESOLUTION

// resolution for angular & radius
const int ANGULER_RES  = 18;
const int RADIUS_RES   = 30;  

CSAImageProcess::CSAImageProcess(void)
{
	//H = NULL;
	m_pView = NULL;

	m_iTheta = 6;
	m_iRes = 10;

	m_pE2LSH = NULL;

	// the first voting
	setFirstVoteStatus(true);
	// the view is not changed
	setViewChanged(false);
}

CSAImageProcess::~CSAImageProcess(void)
{

}

void CSAImageProcess::edgeextraction(const Mat& color_image, Mat& edgelength, Mat& edgeorientation,Mat& m,Mat& m_n,Mat& edgeresponse)
{
	Mat gray_image;
	// convert color image to gray image
	cvtColor(color_image,gray_image,CV_RGB2GRAY);

	// add gaussian noise to gray image
	Mat noise(gray_image.size(),gray_image.type()),noise_gray_image(gray_image.size(),gray_image.type());
	randn(noise,Scalar::all(0.0),Scalar::all(0.5));
	add(gray_image, noise, noise_gray_image);

	// Generate grad_x & grad_y
	int scale = 1, delta = 0,ddepth = CV_64F;
	Mat grad_x,grad_y;
	Sobel(noise_gray_image,grad_x,ddepth,1,0,3,scale,delta,BORDER_DEFAULT);
	Sobel(noise_gray_image,grad_y,ddepth,0,1,3,scale,delta,BORDER_DEFAULT);

	// compute the magnitude
	cartToPolar(grad_x,grad_y,m,edgeorientation);

	/*for(int i = 0; i < grad_x.rows; i++)
	for(int j = 0; j < grad_x.cols; j++)
	{
	assert(abs(grad_x.at<double>(i,j)) < 0.000001);
	edgeorientation.at<double>(i,j) = atan(grad_y.at<double>(i,j)/grad_x.at<double>(i,j));
	}*/

	//* commented by lvp 2012-5-8
	// normalize the magnitude
	normmat(m,5,m_n);
	/*
	// pixel projection
	Mat prj1,prj2;
	pixelproject(edgeorientation,1.4142,prj1,prj2);
	// edge length estimation
	lengthestimation(m_n,edgeorientation,prj1,prj2,5,edgelength);
	// compute the edge response
	computeedgeresponse(edgelength,edgeorientation,grad_x,grad_y,edgeresponse);
	//*/
}

void CSAImageProcess::normmat(const Mat& m,int winsize,Mat& m_n)
{
	Mat result(m.size(),m.type());

	int width = (winsize - 1) / 2;
	int row, col;
	row = m.rows;
	col = m.cols;
	double p_m = 0.0;
	int top,left,bottom,right;
	for(int i = 0; i < row; i++)
		for(int j = 0; j < col; j++)
		{
			p_m = m.at<double>(i,j);

			if(i - width < 0)
				top = 0;
			else
				top = i - width;
			if(i + width > row - 1)
				bottom = row - 1;
			else
				bottom = i + width;
			if(j - width < 0)
				left = 0;
			else
				left = j - width;
			if(j + width > col-1)
				right = col - 1;
			else
				right = j + width;

			double p_mean = 0.0, std_dev = 3.0, dist = 0.0;
			for(int l = top; l <= bottom; l++)
				for(int k = left; k <= right; k++)
				{
					dist = (l - i) * (l - i) + (k - j) * (k -j);
					p_mean += p_m * exp(-dist/(2*std_dev*std_dev));
				}
				result.at<double>(i,j) = m.at<double>(i,j) / (std::max(p_mean,4.0));
		}
		m_n = result;
}

void CSAImageProcess::findedgepos(const Mat& m,uchar color,vector<int>& edgepos)
{
	// accept only char type matrices
	CV_Assert(m.depth() != sizeof(uchar));

	MatConstIterator_<Vec3b> it,end;
	//MatIterator_<Vec3b> it,end;
	int i = 0, j = 0;
	for(it = m.begin<Vec3b>(),end = m.end<Vec3b>(); it != end; ++it)
	{
		if((*it)[2] == color && (*it)[0] == 0 && (*it)[1] == 0)
		{
			int step = it - m.begin<Vec3b>();
			j = step % m.cols;
			i = (step - j)/m.cols;
			edgepos.push_back(i);
			edgepos.push_back(j);
		}
	}
}

// compute the patch descriptor
void CSAImageProcess::computepatchdescriptor(const Mat& edgeorientation,const Mat& m_n, const vector<int>& edgepos,Mat& hist)
{
	// establish the number of bins
	int binx = BIN_X,biny = BIN_Y,bintheta = BIN_THETA;

	int pointnum = edgepos.size() / 2;
	Mat x_src(1,pointnum,CV_32F),y_src(1,pointnum,CV_32F),theta_src(1,pointnum,CV_32F);

	int x = 0, y = 0;
	float new_x = 0.0 ,new_y = 0.0 ,tmp_x = 0.0,tmp_y = 0.0;
	float theta = 0.0;

	// add gaussian noise
	Mat noise1(1,pointnum,CV_32F),noise2(1,pointnum,CV_32F);
	randn(noise1,Scalar::all(0.0),Scalar::all(1));
	randn(noise2,Scalar::all(0.0),Scalar::all(3));

	for(int i = 0; i < pointnum; i++)
	{
		x = edgepos[i*2];
		y = edgepos[i*2+1];
		theta = edgeorientation.at<double>(x,y);

		tmp_x = x - 30.0;
		tmp_y = y - 30.0;

		new_x = tmp_x * cos(theta) - tmp_y * sin(theta);
		new_y = tmp_x * sin(theta) + tmp_y * cos(theta);

		x_src.at<float>(0,i) = new_x + noise1.at<float>(0,i);
		y_src.at<float>(0,i) = new_y + noise2.at<float>(0,i);
		theta_src.at<float>(0,i) = theta + noise1.at<float>(0,i);
	}

	// set the ranges
	/*
	double min_value = 0.0,max_value = 0.0;
	minMaxLoc(x_src,&min_value,&max_value);
	float x_range[] = {min_value,max_value};
	//*/
	float x_range[] = {-50.0,50.0};

	/*
	min_value = max_value = 0.0;
	minMaxLoc(y_src,&min_value,&max_value);
	float y_range[] = {min_value,max_value};
	//*/
	float y_range[] = {-50.0,50.0};

	/*
	min_value = max_value = 0.0;
	minMaxLoc(theta_src,&min_value,&max_value);
	float theta_range[] = {min_value,max_value};
	//*/
	float theta_range[] = {0.0,6.283};

	const float* histRange[] = {x_range,y_range,theta_range};

	// compute the histograms
	Mat x_src1,y_src1,theta_src1;
	x_src1.create(1,1,CV_32F);
	y_src1.create(1,1,CV_32F);
	theta_src1.create(1,1,CV_32F);
	for(int i = 0; i < pointnum; i++)
	{
		Mat x_hist,y_hist,theta_hist;
		int index_x = 0,index_y = 0,index_theta = 0;
		Point maxLoc;
		x_src1.at<float>(0,0) = x_src.at<float>(0,i);
		y_src1.at<float>(0,0) = y_src.at<float>(0,i);
		theta_src1.at<float>(0,0) = theta_src.at<float>(0,i);

		calcHist(&x_src1,1,0,Mat(),x_hist,1,&binx,&histRange[0]);
		minMaxLoc(x_hist,0,0,0,&maxLoc);
		index_x = maxLoc.y;

		calcHist(&y_src1,1,0,Mat(),y_hist,1,&biny,&histRange[1]);
		minMaxLoc(y_hist,0,0,0,&maxLoc);
		index_y = maxLoc.y;

		calcHist(&theta_src1,1,0,Mat(),theta_hist,1,&bintheta,&histRange[2]);
		minMaxLoc(theta_hist,0,0,0,&maxLoc);
		index_theta = maxLoc.y;

		hist.at<float>(index_x,index_y,index_theta)  += 1;

		//hist.at<float>(index_x,index_y,index_theta)  += m_n.at<double>(edgepos[i*2],edgepos[i*2+1]);
		/*
		float std_dev = 2.0;
		for(int index1 = 0; index1 < binx; index1++)
		for(int index2 = 0; index2 < biny; index2++)
		for(int index3 = 0; index3 < bintheta; index3++)
		{
		float dist = (index_x - index1)*(index_x - index1) + (index_y-index2)*(index_y-index2) + (index_theta-index3)*(index_theta-index3);
		hist.at<float>(index1,index2,index3)  += m_n.at<double>(edgepos[i*2],edgepos[i*2+1]) * exp(-dist/(2*std_dev*std_dev));
		}
		//*/
	}
}
/*
compute the edge response according to the edge length, edge orientation and gradient
*/

void CSAImageProcess::computeedgeresponse(const Mat& edgelength,const Mat& orientation,const Mat& grad_x,const Mat& grad_y,Mat& edgeresponse)
{
	edgeresponse = Mat(edgelength.rows,edgelength.cols,edgelength.type());

	for(int i = 0; i < edgelength.rows; i++)
	{
		for(int j = 0; j < edgelength.cols; j++)
		{
			double sx = cos(orientation.at<double>(i,j)) * cos(orientation.at<double>(i,j)) * edgelength.at<double>(i,j) * grad_x.at<double>(i,j);
			double sy = sin(orientation.at<double>(i,j)) * sin(orientation.at<double>(i,j)) * edgelength.at<double>(i,j) * grad_y.at<double>(i,j);

			edgeresponse.at<double>(i,j) = sqrt(sx * sx + sy * sy);
		}
	}
}

// batch process the sketch image
void CSAImageProcess::batch_process_sketch(const char* folder,vector<Mat>& bin_sketch,vector<int>& patchnum)
{
	QDir dir(folder);
	if(!dir.exists())
		return;

	QString fileName;
	dir.setFilter(QDir::Files);
	QFileInfoList list = dir.entryInfoList();
	int i = 0,j=0;
	while(i < list.size())
	{
		QFileInfo fileInfo = list.at(i++);
		fileName = fileInfo.fileName();
		if(fileName.section('.',-1) != "png" && fileName.section('.',-1) != "PNG")
			continue;

		// compute patchnum
		QString str = fileName.left(3);
		QString str1 = str.left(1);
		QString str2 = str.right(1);
		patchnum.push_back((str1.toInt() - 1 ) * 8 + str2.toInt() - 1);

		// compute patch descriptor
		fileName = fileInfo.path() + "/" + fileName;
		// convert the fileName from QString to char*
		char file[256];
		int len = 0;
		for(;len < fileName.length(); len++)
		{
			file[len] = (fileName.at(len)).toAscii();
		}
		file[len] = '\0';

		vector<Mat> bin;
		process_one_image(file,bin,BICE);

		for(int l = 0; l < bin.size(); l++)
			bin_sketch.push_back(bin[l]);
		j++;
	}

	FileStorage fs("E:\\TAMU Project\\SketchAnimation\\SketchAnimation\\tmp\\tmp.xml",FileStorage::WRITE);
	char bin_name[20];
	for(int l = 0;l < j; l++)
	{
		int row = 0,col = 0;

		row = patchnum[l] / 8;
		col = patchnum[l] - row * 8;

		sprintf(bin_name,"r%i_c%i",row,col);
		string str = bin_name;
		fs<<str<<bin_sketch[l];
	}
	fs.release();
}

// batch process the sketch from memory
void CSAImageProcess::batch_process_sketch(const vector<Mat>& img,vector<Mat>& bin_sketch)
{
	for(int i = 0; i < img.size(); i++)
	{
		int sz[3] = {BIN_X,BIN_Y,BIN_THETA};
		Mat hist(3,sz,CV_32F,Scalar::all(0));

		// extract the edge info from patch
		Mat edgelength,m,m_n,orientation,edgeresponse;
		edgeextraction(img[i],edgelength,orientation,m,m_n,edgeresponse);

		process_one_patch(img[i],hist,m_n,orientation);
		bin_sketch.push_back(hist);
	}
}

// batch process the database image
void CSAImageProcess::batch_process_db(const char* folder,vector<Mat>& bin_db,const char* savefile, const char* mfile, const char* ofile, const char* decfile,FEATURE feature)
{
	QDir dir(folder);
	if(!dir.exists())
		return;

	QString fileName;
	dir.setFilter(QDir::Files);
	QFileInfoList list = dir.entryInfoList();
	int i = 0, j = 0;
	while(i < list.size())
	{
		QFileInfo fileInfo = list.at(i++);
		fileName = fileInfo.fileName();
		if(fileName.section('.',-1) != "png" && fileName.section('.',-1) != "PNG")
			continue;

		fileName = fileInfo.path() + "/" + fileName;

		vector<Mat> bin;
		process_one_image(fileName.toAscii(),bin,feature);
		qDebug()<< fileInfo.fileName()<<" processed";

		for(int l = 0; l < bin.size(); l++)
			bin_db.push_back(bin[l]);
		j++;
	}

	// save the histogram file
	FileStorage fs(savefile,FileStorage::WRITE);
	fs<<"filenum"<<j;				// write the file number 

	char name[20];
	if(feature == BICE)
	{
		sprintf(name,"BICE");
		fs<<"feature_type"<<name;	// write the feature type

		for(int l = 0;l < j; l++)
		{
			for(int k = 0; k < 64;k++)
			{
				sprintf(name,"r%i_c%i",l,k);
				string str = name;
				fs<<str<<bin_db[l * 64 + k];
			}
		}
	}
	else if(feature == HOUGH)
	{
		sprintf(name,"HOUGH");
		fs<<"feature_type"<<name;	// write the feature type

		for(int l = 0;l < j; l++)
		{
			sprintf(name,"r_%i",l);
			string str = name;
			fs<<str<<bin_db[l];
		}
	}
	else if(feature == ARP)			// write the ARP feature
	{
		sprintf(name,"ARP");
		fs<<"feature_type"<<name;	

		for(int l = 0;l < j; l++)
		{
			sprintf(name,"r_%i",l);
			string str = name;
			fs<<str<<bin_db[l];
		}
	}

	fs.release();

	/*
	// save the magnitude file
	FileStorage fs2(mfile,FileStorage::WRITE);
	fs2<<"filenum"<<j;
	for(int l = 0;l < j; l++)
	{
		sprintf(name,"image_%d_m_n",l);
		string str = name;
		fs2<<str<<m_pView->m_vNormMagnitude[l];
	}
	fs2.release();

	// save the orientation file
	FileStorage fs3(ofile,FileStorage::WRITE);
	fs3<<"filenum"<<j;
	for(int l = 0;l < j; l++)
	{
		sprintf(name,"image_%d_o",l);
		string str = name;
		fs3<<str<<m_pView->m_vOrientation[l];
	}
	fs3.release();

	// save the decomposing image
	FileStorage fs4(decfile,FileStorage::WRITE);
	fs4<<"filenum"<<j;
	// eight different orientation
	vector<double> orientation_v;
	for(int i = 0; i < 8; i++)
	{
	orientation_v.push_back(i * 22.5 * 3.1415 / 180);
	}

	vector<Mat> dec_m;
	for(int l = 0;l < j; l++)
	{
	decomposeimage(l,orientation_v,dec_m);
	for(int i = 0; i < dec_m.size(); i++)
	{
	sprintf(name,"image_%d_dec_%d",l,i);
	string str = name;
	fs4<<str<<dec_m[i];
	}
	}
	fs4.release();
	//*/
}

/*
process the database image in batch and store them into files using the Hough transform 12-7-7
*/
void CSAImageProcess::batch_process_db(const char* folder,vector<Mat>& bin_db,const char* savefile, const char* mfile, const char* ofile, const char* decfile,const char* vecfile,FEATURE feature)
{
	QDir dir(folder);
	if(!dir.exists())
		return;

	QString fileName;
	dir.setFilter(QDir::Files);
	QFileInfoList list = dir.entryInfoList();
	int i = 0, j = 0;
	vector<vector<Vec4i>> linesarr;
	while(i < list.size())
	{
		QFileInfo fileInfo = list.at(i++);
		fileName = fileInfo.fileName();
		if(fileName.section('.',-1) != "png" && fileName.section('.',-1) != "PNG")
			continue;

		fileName = fileInfo.path() + "/" + fileName;

		vector<Mat> bin;
		process_one_image(fileName.toAscii(),bin,linesarr,feature);
		qDebug()<< fileInfo.fileName()<<" processed";

		for(int l = 0; l < bin.size(); l++)
			bin_db.push_back(bin[l]);
		j++;
	}

	// save the histogram file
	FileStorage fs(savefile,FileStorage::WRITE);
	fs<<"filenum"<<j;				// write the file number 

	char name[20];
	if(feature == BICE)				// writhe the BICE feature
	{
		sprintf(name,"BICE");
		fs<<"feature_type"<<name;	

		for(int l = 0;l < j; l++)
		{
			for(int k = 0; k < 64;k++)
			{
				sprintf(name,"r%i_c%i",l,k);
				string str = name;
				fs<<str<<bin_db[l * 64 + k];
			}
		}
	}
	else if(feature == HOUGH)		// write the HOUGH feature
	{
		sprintf(name,"HOUGH");
		fs<<"feature_type"<<name;	

		for(int l = 0;l < j; l++)
		{
			sprintf(name,"r_%i",l);
			string str = name;
			fs<<str<<bin_db[l];
		}
	}

	fs.release();

	// save the magnitude file
	FileStorage fs2(mfile,FileStorage::WRITE);
	fs2<<"filenum"<<j;
	for(int l = 0;l < j; l++)
	{
		sprintf(name,"image_%d_m_n",l);
		string str = name;
		fs2<<str<<m_pView->m_vNormMagnitude[l];
	}
	fs2.release();

	// save the orientation file
	FileStorage fs3(ofile,FileStorage::WRITE);
	fs3<<"filenum"<<j;
	for(int l = 0;l < j; l++)
	{
		sprintf(name,"image_%d_o",l);
		string str = name;
		fs3<<str<<m_pView->m_vOrientation[l];
	}
	fs3.release();

	/*
	// save the decomposing image
	FileStorage fs4(decfile,FileStorage::WRITE);
	fs4<<"filenum"<<j;
	// eight different orientation
	vector<double> orientation_v;
	for(int i = 0; i < 8; i++)
	{
	orientation_v.push_back(i * 22.5 * 3.1415 / 180);
	}

	vector<Mat> dec_m;
	for(int l = 0;l < j; l++)
	{
	decomposeimage(l,orientation_v,dec_m);
	for(int i = 0; i < dec_m.size(); i++)
	{
	sprintf(name,"image_%d_dec_%d",l,i);
	string str = name;
	fs4<<str<<dec_m[i];
	}
	}
	fs4.release();
	//*/

	// save the lines extracted from image into file
	ofstream out(vecfile);
	out<<linesarr.size()<<endl;		// image number
	for(int i = 0; i < linesarr.size(); i++)
	{
		vector<Vec4i>& lines = linesarr[i];
		out<<lines.size()<<' ';
		for(int j = 0; j < lines.size(); j++)
		{
			Vec4i l = lines[j];
			out<<l[0]<<' '<<l[1]<<' '<<l[2]<<' '<<l[3]<<' ';
		}
		out<<endl;
	}
}

// find the surrounding patch given the patch number
void CSAImageProcess::findsurroundingpatches(int patchnum, vector<int>& patchindex)
{
	int row = 0,col = 0;

	row = patchnum / 8;
	col = patchnum - row * 8;
	patchindex.clear();
	if(row == 0)
	{
		if(col == 0)
		{
			patchindex.push_back(0);
			patchindex.push_back(1);
			patchindex.push_back(8);
			patchindex.push_back(9);
		}
		else if (col == 7)
		{
			patchindex.push_back(6);
			patchindex.push_back(7);
			patchindex.push_back(14);
			patchindex.push_back(15);
		} 
		else
		{
			patchindex.push_back(patchnum-1);
			patchindex.push_back(patchnum);
			patchindex.push_back(patchnum+1);
			patchindex.push_back(patchnum+7);
			patchindex.push_back(patchnum+8);
			patchindex.push_back(patchnum+9);
		}
	}
	else if(row == 7)
	{
		if(col == 0)
		{
			patchindex.push_back(48);
			patchindex.push_back(49);
			patchindex.push_back(56);
			patchindex.push_back(57);
		}
		else if (col == 7)
		{
			patchindex.push_back(54);
			patchindex.push_back(55);
			patchindex.push_back(62);
			patchindex.push_back(63);
		} 
		else
		{
			patchindex.push_back(patchnum-9);
			patchindex.push_back(patchnum-8);
			patchindex.push_back(patchnum-7);
			patchindex.push_back(patchnum-1);
			patchindex.push_back(patchnum);
			patchindex.push_back(patchnum+1);
		}
	}
	if(col == 0)
	{
		if(row != 0 && row != 7)
		{
			patchindex.push_back(patchnum-8);
			patchindex.push_back(patchnum-7);
			patchindex.push_back(patchnum);
			patchindex.push_back(patchnum+1);
			patchindex.push_back(patchnum+8);
			patchindex.push_back(patchnum+9);
		}
	}
	else if(col == 7)
	{
		if(row != 1 && row != 7)
		{
			patchindex.push_back(patchnum-9);
			patchindex.push_back(patchnum-8);
			patchindex.push_back(patchnum-1);
			patchindex.push_back(patchnum);
			patchindex.push_back(patchnum+7);
			patchindex.push_back(patchnum+8);
		}
	}

	if(row != 0 && row != 7 && col != 0 && col != 7)
	{
		patchindex.push_back(patchnum-9);
		patchindex.push_back(patchnum-8);
		patchindex.push_back(patchnum-7);
		patchindex.push_back(patchnum-1);
		patchindex.push_back(patchnum);
		patchindex.push_back(patchnum+1);
		patchindex.push_back(patchnum+7);
		patchindex.push_back(patchnum+8);
		patchindex.push_back(patchnum+9);
	}
}

// measure the patch distances between sketch patches and db patches
void CSAImageProcess::patchdistmeasure(const vector<Mat>& sketch,const vector<int>& patchnum,vector<vector<double>>& dist,vector<vector<int>>& sortpatchindex,vector<int>& imgindex,vector<double>& finaldist_sort)
{
	int nSketch = sketch.size();

	// allocate the dist map for each db image
	static vector<vector<double>> H;
	if(m_bFirstVote)
	{
		H.resize(m_nDBNum);
		for(int i = 0; i < m_nDBNum; i++)
			H[i].assign(_PATCH_INDEX_RANGE_,0.0);

		m_bFirstVote = false;
	}

	for(int i = 0; i < nSketch; i++)
	{
		// current sketch patch
		Mat sketchpatch = sketch[i];

		// for each image in the database, the distance between it and sketch patch is computed
		for(int j = 0; j < m_nDBNum; j++)
		{
			// the database patch at the same position
			Mat dbpatch = m_pView->m_vHistDb[j * _PATCH_INDEX_RANGE_ + patchnum[i]];

			// whether the patch is empty
			double distance;
			if(abs(dbpatch.at<float>(0,0,0)+1) < 0.00001)
				distance = 0;
			else
				distance = compareHist(sketchpatch,dbpatch,CV_COMP_CORREL);

			H[j][patchnum[i]] += distance;
		}
	}

	vector<double> finaldist;
	finaldist.assign(m_nDBNum,0);
	for(int i = 0; i < m_nDBNum; i++)
	{
		for(int j = 0; j < _PATCH_INDEX_RANGE_; j++)
			finaldist[i] += H[i][j];
	}
	finaldist_sort = finaldist;
	sort(finaldist_sort.begin(),finaldist_sort.end(),greater<double>());

	// find the index of the sorted element
	for(vector<double>::iterator it = finaldist_sort.begin(); it != finaldist_sort.end(); it++)
	{
		int index = 0;
		for(vector<double>::iterator its = finaldist.begin(); its != finaldist.end(); its++)
		{
			if(abs((*it) - (*its)) < 0.00001)
			{
				index = its - finaldist.begin();
				break;
			}
		}
		imgindex.push_back(index);
	}
}

/* 
	measure the descriptor distances between sketch image and db images using the difference between histograms(Hough)
*/
void CSAImageProcess::patchdistmeasure(const vector<Mat>& sketch,vector<int>& imgindex,vector<double>& finaldist_sort,int limb_idx, float weight)
{
	int nSketch = sketch.size();

	// the distance array for the whole database
	static vector<double> finaldist;
	if(m_bFirstVote)
	{
		finaldist.assign(m_nDBNum,0);
		m_bFirstVote = false;
	}

	/************************************************************************/
	/* Whether the view has been changed                                    */
	/************************************************************************/
	/*
	if(m_bViewChanged)
	{
		m_bViewChanged = false;
	}
	else
	{
		finaldist.assign(m_nDBNum,0);
	}
	//*/

	for(int i = 0; i < nSketch; i++)
	{
		// current sketch patch
		Mat sketchpatch = sketch[i];

		// two vector array
		finaldist_sort.assign(m_nDBNum,-1.0);
		imgindex.assign(m_nDBNum,0);

		// type conversion
		Mat sketchpatch32F,dbpatch32F;
		sketchpatch.assignTo(sketchpatch32F,CV_32F);

		// for each image in the database, the distance between it and sketch patch is computed
		for(int j = 0; j < m_nDBNum; j++)
		{
			Mat dbpatch;
			
			// The sketched strokes as global descriptor
			if(limb_idx == -1)
			{
				dbpatch = m_pView->m_vHistDb[j];
				dbpatch.assignTo(dbpatch32F,CV_32F);

				// histogram distance measure
				finaldist[j] = compareHist(sketchpatch32F,dbpatch32F,CV_COMP_CORREL) * weight;
			}
			// Different body parts as local descriptor
			else
			{
				dbpatch = m_pView->m_vHistDbLimb[limb_idx][j];
				dbpatch.assignTo(dbpatch32F,CV_32F);

				// histogram distance measure
				finaldist[j] += compareHist(sketchpatch32F,dbpatch32F,CV_COMP_CORREL) * weight;
			}


			// insert sorting
			bool insert = false;
			for(int k = 0; k < j; k++)
			{
				if(finaldist[j] - finaldist_sort[k] >= 0.0)
				{
					for(int l = j; l > k; l--)
					{
						finaldist_sort[l] = finaldist_sort[l-1];
						imgindex[l] = imgindex[l-1];
					}
					finaldist_sort[k] = finaldist[j];
					imgindex[k] = j;
					insert = true;
					break;
				}
			}
			if(!insert)
			{	
				finaldist_sort[j] = finaldist[j];
				imgindex[j] = j;
			}
		}
	}
}
/*
measure the patch distances between sketch image and db image using the difference between histograms(Hough) and search the result by ANN
*/
void CSAImageProcess::patchdistmeasureByANN(const vector<Mat>& sketch,vector<int>& imgindex,vector<double>& finaldist_sort)
{
	int nSketch = sketch.size();

	if(m_bFirstVote)
	{
		m_bFirstVote = false;
	}

	if(m_bViewChanged)
	{
		m_bViewChanged = false;
	}

	for(int i = 0; i < nSketch; i++)
	{
		// current sketch patch
		Mat sketchpatch = sketch[i];

		// obtain the low-dimensional vector
		Mat input,output;
		input.create(1,sketchpatch.rows * sketchpatch.cols,CV_32F);
		int col = 0;
		for(int i = 0; i < sketchpatch.rows; i++)
		{
			for(int j = 0; j < sketchpatch.cols; j++)
			{
				input.at<float>(0,col++) = sketchpatch.at<float>(i,j);
			}
		}
		m_pView->m_PCA.project(input,output);

		ANNpoint query_point = annAllocPt(m_pView->m_nEigenNum);
		for(int k = 0; k < m_pView->m_nEigenNum; k++)
			query_point[k] = output.at<float>(0,k);

		//Search based on KD-Tree
		m_pView->m_pKdtree->annkSearch(query_point,50,m_pView->m_vIdxArr,m_pView->m_vDistArr,0);

		for(int j = 0; j < 50; j++)
		{
			imgindex.push_back(m_pView->m_vIdxArr[j]);
			finaldist_sort.push_back(m_pView->m_vDistArr[j]);
		}
		// release allocated space
		annDeallocPt(query_point);
	}
}

/* 
measure the patch distances between sketch image and db images using the difference between histograms(Hough)
*/
void CSAImageProcess::patchdistmeasure(const vector<vector<Vec4i>>& sketch_lines,vector<int>& imgindex,vector<double>& finaldist_sort)
{
	int nSketch = sketch_lines.size();

	// allocate the dist array
	static vector<double> finaldist;
	if(m_bFirstVote)
	{
		finaldist.assign(m_nDBNum,0);
		m_bFirstVote = false;
	}

	if(m_bViewChanged)
	{
		m_bViewChanged = false;
	}
	else
	{
		finaldist.assign(m_nDBNum,0);
	}

	for(int i = 0; i < nSketch; i++)
	{
		// current sketch patch
		const vector<Vec4i>& lines_sketch = sketch_lines[i];

		// two vector array
		finaldist_sort.assign(m_nDBNum,-1.0);
		imgindex.assign(m_nDBNum,0);

		vector<Mat> sketchpatch32F_v;

#ifdef USE_LESS_RESOLUTION
		// three resolutions
		int rho_res[] = {5,5,5};
		int theta_res[] = {5,15,30};

		float weight[] = {0.5,0.3,0.2};

		// construct different resolution histograms for current sketch lines
		for(int m = 0; m < 3; m++)
		{
			Mat	sketchpatch32F;

			lines2Histogram(lines_sketch,rho_res[m],theta_res[m],sketchpatch32F);

			sketchpatch32F_v.push_back(sketchpatch32F);
		}
#endif

#ifndef USE_LESS_RESOLUTION
		int rho_res= 1;
		for(int m = 0; m < 5; m++)
		{
			rho_res += m * 5;
			int theta_res = 1;
			for(int n = 0; n < 5; n++)
			{
				theta_res += n * 2;

				Mat	sketchpatch32F;
				lines2Histogram(lines_sketch,rho_res,theta_res,sketchpatch32F);

				sketchpatch32F_v.push_back(sketchpatch32F);
			}
		}

#endif

		// for each image in the database, the distance between it and sketch patch is computed
		for(int j = 0; j < m_nDBNum; j++)
		{
			// the database patch at the same position
			const vector<Vec4i>& lines_db = m_pView->m_vLines[j];

			Mat dbpatch32F;

#ifdef USE_LESS_RESOLUTION
			// construct different resolution histograms
			for(int m = 1; m < 2; m++)
			{
				lines2Histogram(lines_db,rho_res[m],theta_res[m],dbpatch32F);

				// histogram distance measure
				finaldist[j] += weight[m] * compareHist(sketchpatch32F_v[m],dbpatch32F,CV_COMP_CORREL);
				//finaldist[j] += compareHist(sketchpatch32F,dbpatch32F,CV_COMP_CORREL);
			}
#endif

#ifndef USE_LESS_RESOLUTION

			// more resolutions
			int rho_res= 1;
			for(int m = 0; m < 5; m++)
			{
				rho_res += m * 5;
				int theta_res = 1;
				for(int n = 0; n < 5; n++)
				{
					theta_res += n * 2;

					lines2Histogram(lines_db,rho_res,theta_res,dbpatch32F);

					// histogram distance measure
					finaldist[j] += compareHist(sketchpatch32F_v[m*5+n],dbpatch32F,CV_COMP_CORREL);
				}
			}
#endif
			// insert sorting
			bool insert = false;
			for(int k = 0; k < j; k++)
			{
				if(finaldist[j] - finaldist_sort[k] >= 0.0)
				{
					for(int l = j; l > k; l--)
					{
						finaldist_sort[l] = finaldist_sort[l-1];
						imgindex[l] = imgindex[l-1];
					}
					finaldist_sort[k] = finaldist[j];
					imgindex[k] = j;
					insert = true;
					break;
				}
			}
			if(!insert)
			{	
				finaldist_sort[j] = finaldist[j];
				imgindex[j] = j;
			}
		}
	}
}

// process one patch
void CSAImageProcess::process_one_patch(const Mat& patch, Mat& bin,Mat& m_n, Mat& orientation)
{
	// find the edge position
	vector<int> edgepos;
	findedgepos(patch,0,edgepos);

	// patch descriptor
	if(edgepos.size() > 10)
		computepatchdescriptor(orientation,m_n,edgepos,bin);
	else
	{
		int sz[3] = {BIN_X,BIN_Y,BIN_THETA};
		Mat hist(3,sz,CV_32F,Scalar::all(-1));
		bin = hist;
	}
}

// process one image in the database
void CSAImageProcess::process_one_image(const char* filename,vector<Mat>& bin,FEATURE feature)
{
	// load the color image 
	Mat color_image,color_image_sub;
	color_image = imread(filename,CV_LOAD_IMAGE_COLOR);

	// when the feature is set to ARP, the following code can be annotated
	/*
	// extract edge info from the whole image
	Mat edgelength,edgeorientation,m,m_n,edgeresponse;
	edgeextraction(color_image,edgelength,edgeorientation,m,m_n,edgeresponse);

	// reduce the resolution to 60x60
	Mat m_n_small, edgeorientation_small;
	m_n_small = m_n;
	edgeorientation_small = edgeorientation;
	int rows,cols;
	for(int i = 0; i < _REDUCE_RESOLUTION_; i++)
	{
		rows = m_n_small.rows;
		cols = m_n_small.cols;

		pyrDown(m_n_small,m_n_small,cv::Size(rows/2,cols/2));
		pyrDown(edgeorientation_small,edgeorientation_small,cv::Size(rows/2,cols/2));
	}

	// save the magnitude & orientation
	m_pView->m_vNormMagnitude.push_back(m_n_small);
	m_pView->m_vOrientation.push_back(edgeorientation_small);
	//*/
	switch(feature)
	{
	/*
	case BICE:
		{
			// the sub area of magnitude and orientation for one patch
			Mat m_n_sub,orientation_sub;
			for(int i = 0; i < 480; i += 60)
			{
				for(int j = 0; j < 480; j += 60)
				{
					color_image_sub = color_image(Range(i,i+60),Range(j,j+60));
					m_n_sub = m_n(Range(i,i+60),Range(j,j+60));
					orientation_sub = edgeorientation(Range(i,i+60),Range(j,j+60));

					int sz[3] = {BIN_X,BIN_Y,BIN_THETA};
					Mat hist(3,sz,CV_32F,Scalar::all(0.0));
					process_one_patch(color_image_sub,hist,m_n_sub,orientation_sub);
					bin.push_back(hist);
				}
			}
		}
		break;
	//*/
	case HOUGH:
		{
			Mat gray_image;
			Mat detected_edges;

			// convert color image to gray image
			cvtColor(color_image,gray_image,CV_RGB2GRAY);

			// reduce the noise with a kernel 3x3
			blur(gray_image,detected_edges,Size(3,3));

			// edge detection
			Canny(detected_edges,detected_edges,50,200,3);

			// hough transform
			vector<Vec4i> lines;
			HoughLinesP( detected_edges, lines, 1, CV_PI/180, 15,10,5);

			// formulate 2D Histogram
			Mat hist;
			lines2Histogram(lines,m_iRes,m_iTheta,hist);

			bin.push_back(hist);
		}
		break;
	case ARP:
		{
			Mat gray_image;
			Mat detected_edges;

			// convert color image to gray image
			cvtColor(color_image,gray_image,CV_RGB2GRAY);

			// reduce the noise with a kernel 3x3(it is not so necessary)
			blur(gray_image,detected_edges,Size(3,3));

			// edge detection
			Canny(detected_edges,detected_edges,50,200,3);

			// resolution is set to ANGULER_RES * RADIUS_RES 
			Mat hist;
			edgeimg2HistogramByARP(detected_edges,ANGULER_RES,RADIUS_RES,hist);

			bin.push_back(hist);
		}
		break;
	}
}

/*
	process one image for database using the Hough transform 12-7-7
*/
void CSAImageProcess::process_one_image(const char* filename,vector<Mat>&bin,vector<vector<Vec4i>>& linesarr, FEATURE feature)
{
	// load the color image
	Mat color_image,color_image_sub;
	color_image = imread(filename,CV_LOAD_IMAGE_COLOR);

	// extract edge info from the whole image
	Mat edgelength,edgeorientation,m,m_n,edgeresponse;
	edgeextraction(color_image,edgelength,edgeorientation,m,m_n,edgeresponse);

	// reduce the resolution to 60x60
	Mat m_n_small, edgeorientation_small;
	m_n_small = m_n;
	edgeorientation_small = edgeorientation;
	int rows,cols;
	for(int i = 0; i < _REDUCE_RESOLUTION_; i++)
	{
		rows = m_n_small.rows;
		cols = m_n_small.cols;

		pyrDown(m_n_small,m_n_small,cv::Size(rows/2,cols/2));
		pyrDown(edgeorientation_small,edgeorientation_small,cv::Size(rows/2,cols/2));
	}

	// store the magnitude & orientation
	m_pView->m_vNormMagnitude.push_back(m_n_small);
	m_pView->m_vOrientation.push_back(edgeorientation_small);

	switch(feature)
	{
	case BICE:
		{
			// the sub area of magnitude and orientation for one patch
			Mat m_n_sub,orientation_sub;
			for(int i = 0; i < 480; i += 60)
			{
				for(int j = 0; j < 480; j += 60)
				{
					color_image_sub = color_image(Range(i,i+60),Range(j,j+60));
					m_n_sub = m_n(Range(i,i+60),Range(j,j+60));
					orientation_sub = edgeorientation(Range(i,i+60),Range(j,j+60));

					int sz[3] = {BIN_X,BIN_Y,BIN_THETA};
					Mat hist(3,sz,CV_32F,Scalar::all(0.0));
					process_one_patch(color_image_sub,hist,m_n_sub,orientation_sub);
					bin.push_back(hist);
				}
			}
		}
		break;
	case HOUGH:
		{
			Mat gray_image;
			Mat detected_edges;

			// convert color image to gray image
			cvtColor(color_image,gray_image,CV_RGB2GRAY);

			// reduce the noise with a kernel 3x3
			blur(gray_image,detected_edges,Size(3,3));

			// edge detection
			Canny(detected_edges,detected_edges,50,200,3);

			// hough transform
			vector<Vec4i> lines;
			HoughLinesP( detected_edges, lines, 1, CV_PI/180, 15,10,5);

			linesarr.push_back(lines);

			// formulate 2D Histogram
			Mat hist;
			lines2Histogram(lines,m_iRes,m_iTheta,hist);

			bin.push_back(hist);
		}
		break;
	}
}


/*
	process sketch image 
*/
void CSAImageProcess::process_sketch_image(const Mat& mat,vector<Mat>& bin,FEATURE feature)
{
	const Mat& color_image = mat;

	switch(feature)
	{
	case BICE:
		{
			Mat color_image_sub;

			// extract edge info from the whole image
			Mat edgelength,edgeorientation,m,m_n,edgeresponse;
			edgeextraction(color_image,edgelength,edgeorientation,m,m_n,edgeresponse);

			// the sub area of magnitude and orientation for one patch
			Mat m_n_sub,orientation_sub;
			for(int i = 0; i < 480; i += 60)
			{
				for(int j = 0; j < 480; j += 60)
				{
					color_image_sub = color_image(Range(i,i+60),Range(j,j+60));
					m_n_sub = m_n(Range(i,i+60),Range(j,j+60));
					orientation_sub = edgeorientation(Range(i,i+60),Range(j,j+60));

					int sz[3] = {BIN_X,BIN_Y,BIN_THETA};
					Mat hist(3,sz,CV_32F,Scalar::all(0.0));
					process_one_patch(color_image_sub,hist,m_n_sub,orientation_sub);
					bin.push_back(hist);
				}
			}
		}
		break;
	case HOUGH:
		{
			Mat gray_image;
			Mat detected_edges;

			// convert color image to gray image
			cvtColor(color_image,gray_image,CV_RGB2GRAY);

			// reduce the noise with a kernel 3x3
			blur(gray_image,detected_edges,Size(3,3));

			// edge detection
			Canny(detected_edges,detected_edges,50,200,3);

			// Hough transform
			vector<Vec4i> lines;
			HoughLinesP( detected_edges, lines, 1, CV_PI/180, 15,10,5);

			// formulate 2D Histogram
			Mat hist;
			lines2Histogram(lines,m_iRes,m_iTheta,hist);

			bin.push_back(hist);
		}
		break;
	case ARP:
		{
			Mat gray_image;
			Mat detected_edges;

			// convert color image to gray image
			cvtColor(color_image,gray_image,CV_RGB2GRAY);

			// reduce the noise with a kernel 3x3
			blur(gray_image,detected_edges,Size(3,3));

			// edge detection
			Canny(detected_edges,detected_edges,50,200,3);

			// resolution is set to 10 degrees & 5 pixels 
			Mat hist;
			edgeimg2HistogramByARP(detected_edges,ANGULER_RES,RADIUS_RES,hist);

			bin.push_back(hist);
		}
		break;
	}

}

/*
process sketch image using the Hough transform 12-7-7
*/
void CSAImageProcess::process_sketch_image(const Mat& mat,vector<Mat>&bin,vector<vector<Vec4i>>& linesarr,FEATURE feature)
{
	const Mat& color_image = mat;

	switch(feature)
	{
	case BICE:
		{
			Mat color_image_sub;

			// extract edge info from the whole image
			Mat edgelength,edgeorientation,m,m_n,edgeresponse;
			edgeextraction(color_image,edgelength,edgeorientation,m,m_n,edgeresponse);

			// the sub area of magnitude and orientation for one patch
			Mat m_n_sub,orientation_sub;
			for(int i = 0; i < 480; i += 60)
			{
				for(int j = 0; j < 480; j += 60)
				{
					color_image_sub = color_image(Range(i,i+60),Range(j,j+60));
					m_n_sub = m_n(Range(i,i+60),Range(j,j+60));
					orientation_sub = edgeorientation(Range(i,i+60),Range(j,j+60));

					int sz[3] = {BIN_X,BIN_Y,BIN_THETA};
					Mat hist(3,sz,CV_32F,Scalar::all(0.0));
					process_one_patch(color_image_sub,hist,m_n_sub,orientation_sub);
					bin.push_back(hist);
				}
			}
		}
		break;
	case HOUGH:
		{
			Mat gray_image;
			Mat detected_edges;

			// convert color image to gray image
			cvtColor(color_image,gray_image,CV_RGB2GRAY);

			// reduce the noise with a kernel 3x3
			blur(gray_image,detected_edges,Size(3,3));

			//int rows,cols;
			//for(int i = 0; i < _REDUCE_RESOLUTION_ - 1; i++)
			//{
			//	rows = detected_edges.rows;
			//	cols = detected_edges.cols;

			//	pyrDown(detected_edges,detected_edges,cv::Size(rows/2,cols/2));
			//}

			// edge detection
			Canny(detected_edges,detected_edges,50,200,3);

			// hough transform
			vector<Vec4i> lines;
			HoughLinesP( detected_edges, lines, 1, CV_PI/180, 15,10,5);

			linesarr.push_back(lines);

			// formulate 2D Histogram
			Mat hist;
			lines2Histogram(lines,m_iRes,m_iTheta,hist);

			bin.push_back(hist);
		}
		break;
	}
}

// min-hash correlated
void CSAImageProcess::minhash(const vector<Mat>& bin_db,vector<vector<int>>& sketches,bool usr_sketch,const char* filename)
{
	float percent = 0.2;

	// the number of files
	int dbnum;
	int patchrange;
	if(!usr_sketch)
	{	
		dbnum = bin_db.size() / _PATCH_INDEX_RANGE_;
		patchrange = _PATCH_INDEX_RANGE_;
	}
	else
	{	
		dbnum = bin_db.size();
		patchrange = 1;
	}

	sketches.resize(dbnum);
	Mat mat;
	ofstream out;
	if(filename)
	{
		out.open(filename);
		if(!out.is_open())
			return;
	}
	out<<dbnum<<"\n";
	for(int i =0 ; i < dbnum; i++)
	{
		for(int j = 0; j < patchrange; j++)
		{
			// get the one & zero array
			mat = bin_db[i * patchrange + j];
			double minvalue = 0.0,maxvalue = 0.0,thresh = 0.0;
			vector<int> onezeroarr;
			minMaxIdx(mat,&minvalue,&maxvalue);			// when all the value in the mat are same
			if(abs(maxvalue - minvalue) < 0.000001)
			{
				for(int k = 0; k < 30; k++)
				{	
					sketches[i].push_back(-1);
					out<<-1<<" ";						// write the min-hash value
				}
				out<<i<<" "<<j<<"\n";					// write the image index and patch offset
				continue;
			}
			//thresh = maxvalue - (maxvalue - minvalue) * percent;		// top 20%
			thresh = 0.0;
			for(int l = 0; l < BIN_X; l++)
				for(int m = 0; m < BIN_Y; m++)
					for(int n = 0; n < BIN_THETA; n++)
					{
						if(mat.at<float>(l,m,n) >= thresh)
							onezeroarr.push_back(1);
						else
							onezeroarr.push_back(0);
					}
					// using the min-hash method to get the sketches
					vector<int> oneindex;
					for(int p = 0; p < onezeroarr.size(); p++)
					{
						if(onezeroarr[p] == 1)
							oneindex.push_back(p);
					}
					//compute the hash value for this patch
					vector<int> hashvalue;
					permutesetindex(oneindex,hashvalue);

					for(vector<int>::const_iterator iter = hashvalue.begin(); iter != hashvalue.end(); iter++)
					{	
						sketches[i].push_back(*iter);
						out<<*iter<<" ";
					}
					out<<i<<" "<<j<<"\n";
		}
	}
}

// permute the array index and return the hash value
void CSAImageProcess::permutesetindex(const vector<int>& oneindex,vector<int>& hashvalue)
{
	static vector<vector<int>> randomarr1;
	static vector<vector<int>> randomarr2;
	static vector<vector<int>> randomarr3;

	if(randomarr1.empty() || randomarr2.empty() || randomarr3.empty())
	{
		randomarr1.resize(10);
		randomarr2.resize(10);
		randomarr3.resize(10);

		// read the file first, if fails, then generate the random number
		ifstream in;
		in.open("hash_func.txt");
		if(in.is_open())
		{
			int number;
			for(int i = 0; i < 10; i++)
				for(int j = 0; j < 400; j++)
				{
					in>>number;
					randomarr1[i].push_back(number);
				}
				for(int i = 0; i < 10; i++)
					for(int j = 0; j < 400; j++)
					{
						in>>number;
						randomarr2[i].push_back(number);
					}
					for(int i = 0; i < 10; i++)
						for(int j = 0; j < 400; j++)
						{
							in>>number;
							randomarr3[i].push_back(number);
						}
						in.close();
		}
		else
		{
			ofstream out("hash_func.txt");
			// generate the 10 random array 0-400
			//if(randomarr1.empty())
			{
				for(int i = 0; i < 10; i++)
				{
					//srand((int)time(0));
					for(int j = 0; j < 400; j++)
					{
						int u = (double)rand() / (RAND_MAX + 1) * (400 - 0) + 0;
						randomarr1[i].push_back(u);
						out<<u<<" ";
					}
				}
				out<<"\n";
			}

			// generate the 10 random array 400-800
			//if(randomarr2.empty())
			{
				for(int i = 0; i < 10; i++)
				{
					//srand((int)time(0));
					for(int j = 0; j < 400; j++)
					{
						//int u = (double)rand() / (RAND_MAX + 1) * (800 - 400) + 400
						int u = (double)rand() / (RAND_MAX + 1) * (800 - 400);
						randomarr2[i].push_back(u);
						out<<u<<" ";
					}
				}
				out<<"\n";
			}

			// generate the 10 random array 800-1200
			//if(randomarr3.empty())
			{
				for(int i = 0; i < 10; i++)
				{
					//srand((int)time(0));
					for(int j = 0; j < 400; j++)
					{
						//int u = (double)rand() / (RAND_MAX + 1) * (1200 - 800) + 800;
						int u = (double)rand() / (RAND_MAX + 1) * (1200 - 800);
						randomarr3[i].push_back(u);
						out<<u<<" ";
					}
				}
				out<<"\n";
			}
			out.close();
		}
	}

	vector<int> permuteindex1;
	vector<int> permuteindex2;
	vector<int> permuteindex3;

	for(int i = 0; i < 10; i++)
	{
		for(int j = 0; j < oneindex.size(); j++)
		{
			permuteindex1.push_back(randomarr1[i][oneindex[j]]);
			permuteindex2.push_back(randomarr2[i][oneindex[j]]);
			permuteindex3.push_back(randomarr3[i][oneindex[j]]);
		}
		hashvalue.push_back(findminvalueinarray(permuteindex1));
		hashvalue.push_back(findminvalueinarray(permuteindex2));
		hashvalue.push_back(findminvalueinarray(permuteindex3));
		permuteindex1.clear();
		permuteindex2.clear();
		permuteindex3.clear();
	}
}

int CSAImageProcess::findminvalueinarray(const vector<int>& arr)
{
	vector<int>::const_iterator iter = arr.begin();
	int min = *iter;
	for(;iter != arr.end(); iter++)
	{
		if(min > *iter)
			min = *iter;
	}

	return min;
}

// vote

//void CSAImageProcess::vote(const vector<vector<int>>& usr_hashvalue,const vector<int>& patchnum,vector<int>& imgindex,vector<vector<int>>& nearestpatchidx,vector<int>& finaldist_sort)
//{
//	int nPatch = usr_hashvalue.size();
//	nearestpatchidx.resize(nPatch);
//
//	memset((char*)H,0,_MAX_UPDATED_PATCH_ * _MAX_DB_IMG_ * _PATCH_INDEX_RANGE_ * sizeof(int));
//
//	// the total voting number for each image
//	static vector<int> lastimgindex;
//	static vector<int> finaldist;
//	static int votenum = 0;
//
//	// whether this is the first voting
//	if(m_bFirstVote)
//	{
//		lastimgindex.clear();
//		finaldist.assign(m_nDBNum,0);
//		votenum = 0;
//
//		m_bFirstVote = false;
//	}
//
//	vector<int> newimgindex;
//	// compute the vote matrix for voting
//	for(int i = 0; i < nPatch; i++)
//	{
//		for(int j = 0; j < 10; j++)
//		{
//			if(usr_hashvalue[i][j*3] == -1)		// no points in this patch 
//				continue;
//			int* imgpatchindex = m_pView->m_pInvArr[usr_hashvalue[i][j*3]][usr_hashvalue[i][j*3+1]][usr_hashvalue[i][j*3+2]];
//			int curnum = m_pView->invnum[usr_hashvalue[i][j*3]][usr_hashvalue[i][j*3+1]][usr_hashvalue[i][j*3+2]];
//			if(curnum == 0)
//				continue;
//
//			for(int k = 0; k < curnum; k++)
//			{
//				int imageindex	= imgpatchindex[k*2];
//				int patchindex	= imgpatchindex[k*2+1];
//
//				// ensure the patch scopes
//				vector<int> patchindexarr;
//				findsurroundingpatches(patchnum[i],patchindexarr);
//				if(find(patchindexarr.begin(),patchindexarr.end(),patchindex) != patchindexarr.end())
//					//if(patchindex == patchnum[i])
//				{
//					H[i][imageindex][patchindex] += 1;
//					newimgindex.push_back(imageindex);
//
//					// for each image in the database, compute the voted number
//					finaldist[imageindex] += 1;
//				}
//			}			
//		}
//		votenum++;
//	}
//
//	// delete the current selected image which do not appear in last selection
//	/*
//	for(int i = 0; i < newimgindex.size(); i++)
//	{
//	if(lastimgindex.size() == 0)
//	break;
//	if(find(lastimgindex.begin(),lastimgindex.end(),newimgindex[i]) == lastimgindex.end())
//	{
//	finaldist[newimgindex[i]] -= 1;
//	}
//	}
//	//*/
//	// remove the image index whose vote number equals zero
//	finaldist_sort = finaldist;
//	sort(finaldist_sort.begin(),finaldist_sort.end(),greater<int>());
//	vector<int>::const_iterator iter = find(finaldist_sort.begin(),finaldist_sort.end(),0);
//	if(iter != finaldist_sort.end())
//		finaldist_sort.erase(iter,finaldist_sort.end());
//
//	// find the index of the sorted element, some image indexes have same value
//	vector<int> imgageindex_duplicate;
//	for(vector<int>::iterator it = finaldist_sort.begin(); it != finaldist_sort.end(); it++)
//	{
//		int index = -1;
//		for(vector<int>::iterator its = finaldist.begin(); its != finaldist.end(); its++)
//		{
//			if((*it) - (*its) == 0)
//			{
//				index = its - finaldist.begin();
//				(*its) = 0;
//				break;
//			}
//		}
//		if(index != -1)
//			imgageindex_duplicate.push_back(index);
//	}
//
//	// remove the duplicate value
//	if(imgageindex_duplicate.size() > 0)
//	{
//		int idx = imgageindex_duplicate[0];
//		imgindex.push_back(idx);
//		for(vector<int>::const_iterator iter = imgageindex_duplicate.begin() + 1; iter != imgageindex_duplicate.end(); iter++)
//		{
//			if(*iter != idx)
//			{
//				idx = *iter;
//				imgindex.push_back(idx);
//			}
//		}
//	}
//
//	// save the selected image index
//	if(!imgindex.empty())
//		lastimgindex.assign(imgindex.begin(),imgindex.end());
//
//	/* for present, we don not need to compute this variables
//	// for each sketch patch, compute the nearest patch index in database image which are selected
//	for(int i = 0; i < nPatch; i++)
//	{
//	for(int j = 0; j < imgindex.size(); j++)
//	{
//	int patchidx = -1;
//	int maxvotenum = -1;
//	for(int k = 0; k < 64; k++)
//	{
//	if(H[i][imgindex[j]][k] > maxvotenum)
//	{
//	maxvotenum = H[i][imgindex[j]][k];
//	patchidx = k;
//	}
//	}
//	nearestpatchidx[i].push_back(patchidx);
//	}
//	}
//	//*/
//}

/*	
estimating the lengths of two sub edges that start at the pixel and
proceed in two opposite directions given by the local edge orientation(po 
and po+pi)
*/
void CSAImageProcess::pixelproject(const Mat& orientation,float length,Mat& prj1,Mat& prj2)
{
	int rows = orientation.rows;
	int cols = orientation.cols;

	int sz[3];
	sz[0] = rows;
	sz[1] = cols;
	sz[2] = 8;
	prj1.create(3,sz,CV_32S);
	prj2.create(3,sz,CV_32S);

	for(int i = 0; i < rows; i++)
	{
		for(int j = 0; j < cols; j++)
		{
			float positive_prj_cos = length * cos(orientation.at<double>(i,j));
			float positive_prj_sin = length * sin(orientation.at<double>(i,j));
			float negative_prj_cos = -positive_prj_cos;
			float negative_prj_sin = -positive_prj_sin;

			prj1.at<int>(i,j,0) = ceil(i + positive_prj_cos);
			prj1.at<int>(i,j,1) = ceil(j + positive_prj_sin);

			prj1.at<int>(i,j,2) = ceil(i + positive_prj_cos);
			prj1.at<int>(i,j,3) = floor(j + positive_prj_sin);

			prj1.at<int>(i,j,4) = floor(i + positive_prj_cos);
			prj1.at<int>(i,j,5) = ceil(j + positive_prj_sin);

			prj1.at<int>(i,j,6) = floor(i + positive_prj_cos);
			prj1.at<int>(i,j,7) = floor(j + positive_prj_sin);

			// when the index < 0
			for(int m = 0; m < 8; m++)
			{
				if(prj1.at<int>(i,j,m) < 0)
					prj1.at<int>(i,j,m) = 0;
			}
			// when the row index > rows-1 or the col index > cols-1
			for(int m = 0; m < 8; m += 2)
			{
				if(prj1.at<int>(i,j,m) > rows-1)
					prj1.at<int>(i,j,m) = rows-1;
				if(prj1.at<int>(i,j,m+1) > cols-1)
					prj1.at<int>(i,j,m+1) = cols-1;
			}

			prj2.at<int>(i,j,0) = ceil(i + negative_prj_cos);
			prj2.at<int>(i,j,1) = ceil(j + negative_prj_sin);

			prj2.at<int>(i,j,2) = ceil(i + negative_prj_cos);
			prj2.at<int>(i,j,3) = floor(j + negative_prj_sin);

			prj2.at<int>(i,j,4) = floor(i + negative_prj_cos);
			prj2.at<int>(i,j,5) = ceil(j + negative_prj_sin);

			prj2.at<int>(i,j,6) = floor(i + negative_prj_cos);
			prj2.at<int>(i,j,7) = floor(j + negative_prj_sin);

			// when the index < 0
			for(int m = 0; m < 8; m++)
			{
				if(prj2.at<int>(i,j,m) < 0)
					prj2.at<int>(i,j,m) = 0;
			}
			// when the row index > rows-1 or the col index > cols-1
			for(int m = 0; m < 8; m += 2)
			{
				if(prj2.at<int>(i,j,m) > rows-1)
					prj2.at<int>(i,j,m) = rows-1;
				if(prj2.at<int>(i,j,m+1) > cols-1)
					prj2.at<int>(i,j,m+1) = cols-1;
			}
		}
	}
}

/*	
length estimation using message passing as described in GradientShop2010.
*/
void CSAImageProcess::lengthestimation(const Mat& m_n,const Mat& orientation,const Mat& prj1,const Mat& prj2,int iter,Mat& edgelength)
{
	int rows = m_n.rows;
	int cols = m_n.cols;

	Mat m1(rows,cols,CV_64F,Scalar(0.0)),m2(rows,cols,CV_64F,Scalar(0.0));
	edgelength.create(rows,cols,CV_64F);

	float w_alpha = 0.25;
	float w_theta = 0,w_theta2 = 0;
	for(int i = 0; i < rows; i++)
	{
		for(int j = 0; j < cols; j++)
		{
			for(int k = 1; k < iter; k++)
			{
				for(int l = 0; l < 8; l +=2)
				{
					w_theta = exp(-(orientation.at<double>(i,j)-orientation.at<double>(prj1.at<int>(i,j,l),prj1.at<int>(i,j,l+1))) *(orientation.at<double>(i,j)-orientation.at<double>(prj1.at<int>(i,j,l),prj1.at<int>(i,j,l+1))) / (2 * 3.1415 / 5));
					m1.at<double>(i,j) = w_alpha * w_theta * (m_n.at<double>(i,j) + m1.at<double>(prj1.at<int>(i,j,1),prj1.at<int>(i,j,l+1)));

					w_theta2 = exp(-(orientation.at<double>(i,j)-orientation.at<double>(prj2.at<int>(i,j,l),prj2.at<int>(i,j,l+1))) *(orientation.at<double>(i,j)-orientation.at<double>(prj2.at<int>(i,j,l),prj2.at<int>(i,j,l+1))) / (2 * 3.1415 / 5));
					m2.at<double>(i,j) = w_alpha * w_theta2 * (m_n.at<double>(i,j) + m1.at<double>(prj2.at<int>(i,j,1),prj2.at<int>(i,j,l+1)));
				}
			}
			edgelength.at<double>(i,j) = m1.at<double>(i,j) + m2.at<double>(i,j) + m_n.at<double>(i,j);
		}
	}
}

// image alignment
void CSAImageProcess::imagealignment(const Mat& usr_sketch,const vector<Mat>& db_sketch,vector<vector<double>>& transform,vector<Mat>& db_sketch_new_response,vector<Mat>& db_sketch_new_orientation,bool useedgeimage)
{
	Mat usr_sketch_small,db_sketch_small;
	pyrDown(usr_sketch,usr_sketch_small,Size(usr_sketch.rows/2,usr_sketch.cols/2));			//	zoom out 2x
	pyrDown(usr_sketch_small,usr_sketch_small,Size(usr_sketch.rows/4,usr_sketch.cols/4));	//	zoom out 2x

	transform.resize(db_sketch.size());
	db_sketch_new_response.resize(db_sketch.size());
	db_sketch_new_orientation.resize(db_sketch.size());

	int rows = usr_sketch_small.rows;
	int cols = usr_sketch_small.cols;

	//int offsetrange = 10;
	int offsetrange = 0;

	if(useedgeimage)
	{
		// Use the method in ShadowDraw
		// extract edge image from user sketch image
		Mat edgelength, edgeorientation, m, m_n,edgeresponse;
		edgeextraction(usr_sketch_small,edgelength,edgeorientation,m,m_n,edgeresponse);						
		imwrite("E:\\TAMU Project\\SketchAnimation\\SketchAnimation\\tmp\\usr_sketch.png",edgeresponse);

		for(int i = 0; i < db_sketch.size(); i++)
		{
			pyrDown(db_sketch[i],db_sketch_small,Size(db_sketch[i].rows/2,db_sketch[i].cols/2));	//	zoom out 2x
			pyrDown(db_sketch_small,db_sketch_small,Size(db_sketch[i].rows/4,db_sketch[i].cols/4));	//	zoom out 2x

			// extract edge image from database image 
			Mat edgelength_db, edgeorientation_db, m_db, m_n_db,edgeresponse_db;
			edgeextraction(db_sketch_small,edgelength_db,edgeorientation_db,m_db,m_n_db,edgeresponse_db);	

			/*
			// compute the offset for x & y separately
			Point maxLocX,maxLocY;

			Mat Tx(1,2*offsetrange+1,CV_64F,Scalar(0)), Ty(1,2*offsetrange+1,CV_64F,Scalar(0));

			char name[256];
			sprintf(name,"E:\\TAMU Project\\SketchAnimation\\SketchAnimation\\tmp\\db_img_%d.png",i);
			imwrite(name,edgeresponse_db);

			// compute the x offset
			for(int offset = -offsetrange,x = 0; offset <= offsetrange; offset++,x++)
			{
			int r_up = 0, r_bottom = rows;
			if(offset <= 0)
			{
			r_up = 0;
			r_bottom = rows + offset;
			}
			else 
			{
			r_up = offset;
			r_bottom = rows;
			}
			for(int r = r_up; r < r_bottom; r++)
			{
			for(int c = 0; c < cols; c++)
			{
			//Tx.at<double>(0,x) += sin(edgeorientation.at<double>(r,c)) * edgelength.at<double>(r,c) * sin(edgeorientation_db.at<double>(r-offset,c)) * edgelength_db.at<double>(r-offset,c);
			Tx.at<double>(0,x) += sin(edgeorientation.at<double>(r,c)) * edgeresponse.at<double>(r,c) * sin(edgeorientation_db.at<double>(r-offset,c)) * edgeresponse_db.at<double>(r-offset,c);
			}
			}
			}
			minMaxLoc(Tx,0,0,0,&maxLocX);
			transform[i].push_back(maxLocX.x - offsetrange);

			// compute the y offset
			for(int offset = -offsetrange,x = 0; offset <= offsetrange; offset++,x++)
			{
			int c_left = 0, c_right = cols;
			if(offset <= 0)
			{
			c_left = 0;
			c_right = cols + offset;
			}
			else 
			{
			c_left = offset;
			c_right = cols;
			}
			for(int r = 0; r < rows; r++)
			{
			for(int c = c_left; c < c_right; c++)
			{
			//Ty.at<double>(0,x) += cos(edgeorientation.at<double>(r,c)) * edgelength.at<double>(r,c) * cos(edgeorientation_db.at<double>(r,c-offset)) * edgelength_db.at<double>(r,c-offset);
			Ty.at<double>(0,x) += cos(edgeorientation.at<double>(r,c)) * edgeresponse.at<double>(r,c) * cos(edgeorientation_db.at<double>(r,c-offset)) * edgeresponse_db.at<double>(r,c-offset);
			}
			}
			}
			minMaxLoc(Ty,0,0,0,&maxLocY);
			transform[i].push_back(maxLocY.x - offsetrange);
			//*/

			// compute the offset for x & y together
			Mat T(2*offsetrange+1,2*offsetrange+1,CV_64F,Scalar(0.0));
			Point maxLoc;

			for(int offsetx = -offsetrange, x = 0; offsetx <= offsetrange; offsetx++,x++)
			{
				for(int offsety = -offsetrange, y = 0; offsety <= offsetrange; offsety++,y++)
				{
					int r_up = 0, r_bottom = rows;
					int c_left = 0,c_right = cols;

					if(offsetx <= 0)
					{
						r_up = 0;
						r_bottom = rows + offsetx;
					}
					else
					{
						r_up = offsetx;
						r_bottom = rows;
					}
					if(offsety <= 0)
					{
						c_left = 0;
						c_right = cols + offsety;
					}
					else 
					{
						c_left = offsety;
						c_right = cols;
					}
					Mat mat1,mat2,corr;
					mat1 = edgeresponse(Range(r_up,r_bottom),Range(c_left,c_right));
					mat2 = edgeresponse_db(Range(r_up - offsetx,r_bottom - offsetx),Range(c_left - offsety ,c_right - offsety));
					Mat src,tml;
					mat1.convertTo(tml,CV_32F);
					mat2.convertTo(src,CV_32F);
					matchTemplate(src,tml,corr,CV_TM_CCORR);
					//multiply(mat1,mat2,corr);
					Scalar sca = sum(corr);
					T.at<double>(x,y) = sca.val[0];
				}
			}

			minMaxLoc(T,0,0,0,&maxLoc);
			int x_offset = (maxLoc.x - offsetrange) * 4;
			int y_offset = (maxLoc.y - offsetrange) * 4;

			transform[i].push_back(x_offset);
			transform[i].push_back(y_offset);

			int rows_full = db_sketch[i].rows;
			int cols_full = db_sketch[i].cols;
			edgeextraction(db_sketch[i],edgelength_db,edgeorientation_db,m_db,m_n_db,edgeresponse_db);	

			db_sketch_new_response[i] = Mat(rows_full,cols_full,edgeresponse_db.type(),Scalar::all(0.0));
			db_sketch_new_orientation[i] = Mat(rows_full,cols_full,edgeorientation_db.type(),Scalar::all(0.0));
			for(int m = 0; m < rows_full; m++)
			{
				for(int n = 0; n < cols_full; n++)
				{
					if(m - x_offset >= 0 && m - x_offset < rows_full && n - y_offset >= 0 && n - y_offset < cols_full)
					{
						db_sketch_new_response[i].at<double>(m - x_offset,n - y_offset) = edgeresponse_db.at<double>(m,n);
						db_sketch_new_orientation[i].at<double>(m - x_offset,n - y_offset) = edgeorientation_db.at<double>(m,n);
					}
				}
			}
			qDebug()<<"image "<<i<<" response transformed";
		}
	}
	else
	{
		for(int i = 0; i < db_sketch.size(); i++)
		{
			Mat T(2*offsetrange+1,2*offsetrange+1,CV_64F,Scalar(0.0));
			Point minLoc;

			for(int offsetx = -offsetrange, x = 0; offsetx <= offsetrange; offsetx++,x++)
			{
				for(int offsety = -offsetrange, y = 0; offsety <= offsetrange; offsety++,y++)
				{
					int r_up = 0, r_bottom = rows;
					int c_left = 0,c_right = cols;

					if(offsetx <= 0)
					{
						r_up = 0;
						r_bottom = rows + offsetx;
					}
					else
					{
						r_up = offsetx;
						r_bottom = rows;
					}
					if(offsety <= 0)
					{
						c_left = 0;
						c_right = cols + offsety;
					}
					else 
					{
						c_left = offsety;
						c_right = cols;
					}

					Mat usr_sketch_gray,db_sketch_gray;

					// convert color image to gray image
					cvtColor(usr_sketch(Range(r_up,r_bottom),Range(c_left,c_right)),usr_sketch_gray,CV_RGB2GRAY);
					cvtColor(db_sketch[i](Range(r_up - offsetx,r_bottom - offsetx),Range(c_left - offsety ,c_right - offsety)),db_sketch_gray,CV_RGB2GRAY);
					//out<<"db_sketch_gray=\n"<<db_sketch_gray<<"\n\n";
					Mat diff,diff_float,diff2;
					absdiff(usr_sketch_gray,db_sketch_gray,diff);
					diff.convertTo(diff_float,CV_32F);
					multiply(diff_float,diff_float,diff2);
					Scalar sca = sum(diff2);
					T.at<double>(x,y) = sca.val[0];
				}
			}
			minMaxLoc(T,0,0,&minLoc);
			transform[i].push_back(minLoc.x - offsetrange);
			transform[i].push_back(minLoc.y - offsetrange);

			/*
			// remap the original image
			db_sketch_new[i] = Mat(rows,cols,db_sketch[i].type(),Scalar::all(255));	// initialize to white
			for(int m = 0; m < rows; m++)
			{
			for(int n = 0; n < cols; n++)
			{
			if(m + minLoc.x - 30 >= 0 && m + minLoc.x - 30 < rows && n + minLoc.y - 30 >= 0 && n + minLoc.y - 30 < cols)
			{
			db_sketch_new[i].at<Vec3b>(m+minLoc.x-30,n+minLoc.y-30) = db_sketch[i].at<Vec3b>(m,n);
			}
			}
			}
			//*/
		}
	}
}

/* 
image alignment using Lucas-Kanade Registration
*/
void CSAImageProcess::imagealignment(const Mat& usr_sketch,const vector<Mat>& db_sketch,vector<vector<double>>& transform)
{
	transform.resize(db_sketch.size());

	/************************************************************************/
	/*  user sketch image                                                   */
	/************************************************************************/
	Mat usr_sketch_gray_image,H1,H2,H3;
	// convert color image to gray image
	cvtColor(usr_sketch,usr_sketch_gray_image,CV_RGB2GRAY);
	// reduce the resolution
	pyrDown(usr_sketch_gray_image,H1,Size(usr_sketch_gray_image.rows/2,usr_sketch_gray_image.cols/2));
	pyrDown(H1,H2,Size(H1.rows/2,H1.cols/2));
	pyrDown(H2,H3,Size(H2.rows/2,H2.cols/2));

	H3.convertTo(H3,CV_32F);
	/************************************************************************/
	/* Database Image                                                       */
	/************************************************************************/
	Mat gray_image,I1,I2,I3;
	// convert color image to gray image
	cvtColor(db_sketch[0],gray_image,CV_RGB2GRAY);
	pyrDown(gray_image,I1,Size(db_sketch[0].rows/2,db_sketch[0].cols/2));	
	pyrDown(I1,I2,Size(I1.rows/2,I1.cols/2));	
	pyrDown(I2,I3,Size(I2.rows/2,I2.cols/2));	

	I3.convertTo(I3,CV_32F);

	// Generate grad_x & grad_y
	int scale = 1, delta = 0,ddepth = CV_32F;
	Mat grad_x,grad_y;
	Sobel(I3,grad_x,ddepth,1,0,3,scale,delta,BORDER_DEFAULT);
	Sobel(I3,grad_y,ddepth,0,1,3,scale,delta,BORDER_DEFAULT);
	imwrite("grad_x.png",grad_x);
	imwrite("grad_y.png",grad_y);

	float x = 0.0, y = 0.0;
	Mat warpedI,warpedI_x,warpedI_y,err_image;
	// run iterative L-K 
	for(int j = 0; j < 10; j++)
	{
		// warp the input image(db image)
		warpImg(I3,warpedI,x,y);
		imwrite("warpedI.png",warpedI);
		imwrite("H3.png",H3);
		// compute the error image
		err_image = H3 - warpedI;

		// warp the gradient
		warpImg(grad_x,warpedI_x,x,y);
		warpImg(grad_y,warpedI_y,x,y);

		// compute delta_x & delta_y
		int rows = warpedI_x.rows * warpedI_x.cols;
		Mat new_warpedI_x = warpedI_x.reshape(1,rows);	// change it to column vector
		Mat new_warpedI_y = warpedI_y.reshape(1,rows);

		// construct matrix A which is 2-column matrix
		Mat A(rows,2,new_warpedI_x.type());
		Mat A0 = A.col(0),A1 = A.col(1);
		new_warpedI_x.copyTo(A0);
		new_warpedI_y.copyTo(A1);

		// construct matrix b which is 1-column matrix
		Mat b = err_image.reshape(1,rows);

		// solve the linear function
		Mat delata_xy;
		solve(A,b,delata_xy,DECOMP_NORMAL);

		// update x & y
		x += delata_xy.at<float>(0,0);
		y += delata_xy.at<float>(1,0);
	}
}
/*
warp image, currently only for translation
*/
void CSAImageProcess::warpImg(const Mat& original, Mat& warped,float x, float y)
{
	warped.create(original.size(),original.type());

	Mat warp_mat(2,3,CV_32FC1);
	// no rotation
	warp_mat.at<float>(0,0) = warp_mat.at<float>(1,1) = 1.0;
	warp_mat.at<float>(0,1) = warp_mat.at<float>(1,0) = 0.0;
	// translation
	warp_mat.at<float>(0,2) = x;
	warp_mat.at<float>(1,2) = y;

	warpAffine(original,warped,warp_mat,warped.size());
}

/*
decompose the image into different images according to given orientation(radian)
*/
void CSAImageProcess::decomposeimage(const Mat& m, const Mat& orientation,const vector<double>& orientation_v,vector<Mat>& dec_m)
{
	int rows = m.rows;
	int cols = m.cols;
	dec_m.resize(8);

	// initialize the oriented image
	for(int i = 0; i < 8; i++)
	{
		dec_m[i] = Mat(rows,cols,m.type(),Scalar::all(0.0));
	}

	// assign the original edge image into different oriented images
	for(int i = 0; i < rows; i++)
	{
		for(int j = 0; j < cols; j++)
		{
			double rad = orientation.at<double>(i,j);
			double response = m.at<double>(i,j);

			// the radian scope is from 0 to pi
			if(rad > 3.14159265)
				rad -= 3.14159265;

			// the last oriented image
			if(rad >= orientation_v[7])
			{
				dec_m[7].at<double>(i,j) = response;
				continue;
			}
			// the 1-7 oriented image
			for(int k = 0; k < 7; k++)
			{
				if(rad >= orientation_v[k] && rad < orientation_v[k+1])
				{
					double alpha = (rad - orientation_v[k])/(orientation_v[k+1]-orientation_v[k]);

					dec_m[k].at<double>(i,j) = (1-alpha) * response;
					dec_m[k+1].at<double>(i,j) = alpha * response;

					break;
				}
			}
		}
	}
	for(int i = 0; i < 8; i++)
	{
		GaussianBlur(dec_m[i],dec_m[i],Size(5,5),0);
	}
}

/*
decompose edge image into different oriented images according to given orientation(radian) through setting the image index in database
*/
void CSAImageProcess::decomposeimage(int index,const vector<double>& orientation_v,vector<Mat>& dec_m)
{
	// The edge image and edge orientation
	Mat m,orientation;

	assert(!m_pView->m_vNormMagnitude.empty() || !m_pView->m_vOrientation.empty());

	m = m_pView->m_vNormMagnitude[index] * _COEFFICIENT_;
	//m = m_pView->m_vNormMagnitude[index];

	orientation = m_pView->m_vOrientation[index];

	decomposeimage(m,orientation,orientation_v,dec_m);
}

/*
compute positive and negative edge correlation images
*/
void CSAImageProcess::computeedgecorrimg(const vector<Mat>& dec_db_align,const vector<Mat>& dec_usr_sketch,Mat& positive_corr_img,Mat& negative_corr_img)
{
	positive_corr_img = Mat(dec_db_align[0].rows,dec_db_align[0].cols,dec_db_align[0].type(),Scalar(0.0));
	negative_corr_img = Mat(dec_db_align[0].rows,dec_db_align[0].cols,dec_db_align[0].type(),Scalar(0.0));

	double minVal,maxVal;
	for(int i = 0; i < 8; i++)
	{
		// using the matchTemplate method
		/* commented 3-31
		//Mat cross_corr(dec_db_align[0].rows,dec_db_align[0].cols,CV_32F);
		Mat cross_corr;
		Mat src,tml;

		dec_db_align[i].convertTo(src,CV_32F);
		dec_usr_sketch[i].convertTo(tml,CV_32F);

		matchTemplate(src,tml,cross_corr,CV_TM_CCORR_NORMED);
		out<<"\nCross_Corr\n";
		out<<cross_corr;
		minMaxLoc(cross_corr,&minVal,&maxVal);
		positive_corr_img = positive_corr_img + cross_corr;

		int j = (i + 4) % 8;
		dec_usr_sketch[j].convertTo(tml,CV_32F);
		matchTemplate(src,tml,cross_corr,CV_TM_CCORR_NORMED);
		negative_corr_img = negative_corr_img + cross_corr;
		//*/

		Mat multi1,multi2;
		multi1 = dec_db_align[i].mul(dec_usr_sketch[i]);
		//multi1 = dec_db_align[i] * dec_usr_sketch[i];
		positive_corr_img = positive_corr_img + multi1;

		int j = (i+4)%8;
		multi2 = dec_db_align[i].mul(dec_usr_sketch[j]);
		//multi2 = dec_db_align[i] * dec_usr_sketch[j];
		negative_corr_img = negative_corr_img + multi2;
	}
}

/*
reset the vote matrix H
*/
void CSAImageProcess::resetVoteMatrix()
{

}

/* 
continuous comparison for voting
*/
//void CSAImageProcess::vote(const vector<Mat>& bin_sketch,const vector<vector<int>>& usr_hashvalue,const vector<int>& patchnum,vector<int>& imgindex,vector<vector<int>>& nearestpatchidx,vector<double>& finaldist_sort)
//{
//	int nPatch = usr_hashvalue.size();
//	nearestpatchidx.resize(nPatch);
//
//	memset((char*)H,0,_MAX_UPDATED_PATCH_ * _MAX_DB_IMG_ * _PATCH_INDEX_RANGE_ * sizeof(int));
//	memset(m_vImageVoted,0,sizeof(m_vImageVoted));
//
//	// the distance between the total sketch image and database image
//	static vector<double> finaldist;
//	if(m_bFirstVote)
//	{
//		finaldist.assign(m_nDBNum,0.0);
//		m_bFirstVote = false;
//	}
//
//	// compute the vote matrix for voting
//	for(int i = 0; i < nPatch; i++)
//	{
//		for(int j = 0; j < 10; j++)
//		{
//			if(usr_hashvalue[i][j*3] == -1)		// no points in this patch 
//				continue;
//			int* imgpatchindex = m_pView->m_pInvArr[usr_hashvalue[i][j*3]][usr_hashvalue[i][j*3+1]][usr_hashvalue[i][j*3+2]];
//			int curnum = m_pView->invnum[usr_hashvalue[i][j*3]][usr_hashvalue[i][j*3+1]][usr_hashvalue[i][j*3+2]];
//			if(curnum == 0)
//				continue;
//
//			for(int k = 0; k < curnum; k++)
//			{
//				int imageindex	= imgpatchindex[k*2];
//				int patchindex	= imgpatchindex[k*2+1];
//
//				// ensure the patch scopes
//				/*
//				vector<int> patchindexarr;
//				findsurroundingpatches(patchnum[i],patchindexarr);
//				if(find(patchindexarr.begin(),patchindexarr.end(),patchindex) != patchindexarr.end())
//				//*/
//				if(patchindex == patchnum[i])
//				{
//					H[i][imageindex][patchindex] += 1;
//					m_vImageVoted[imageindex] = true;
//				}
//			}			
//		}
//	}
//
//	// for each sketch patch, compute the nearest patch index in database image which are selected and
//	// its corresponding image index
//	vector<int> newimgindex;
//	vector<int> patchindex;
//	for(int i = 0; i < nPatch; i++)
//	{
//		for(int j = 0; j < m_nDBNum; j++)
//		{
//			if(!m_vImageVoted[j])
//				continue;
//
//			int patchidx = -1;
//			int maxvotenum = 0;
//
//			for(int k = 0; k < _PATCH_INDEX_RANGE_; k++)
//			{
//				if(H[i][j][k] > maxvotenum)
//				{
//					maxvotenum = H[i][j][k];
//					patchidx = k;
//				}
//			}
//
//			if(patchidx != -1)
//			{
//				newimgindex.push_back(j);
//				patchindex.push_back(patchidx);
//
//				Mat dbpatch = m_pView->m_vHistDb[j * _PATCH_INDEX_RANGE_ + patchidx];
//
//				// compute the distance between to histogram
//				double dist = compareHist(bin_sketch[i],dbpatch,CV_COMP_CORREL);
//				finaldist[j] += dist;
//			}
//		}
//	}
//
//	// remove the image index whose vote number equals zero
//	finaldist_sort = finaldist;
//	sort(finaldist_sort.begin(),finaldist_sort.end(),greater<double>());
//	vector<double>::const_iterator iter;
//	for(iter = finaldist_sort.begin(); iter != finaldist_sort.end(); iter++)
//	{
//		if((*iter) < 0.00001)
//			break;
//	}
//	if(iter != finaldist_sort.end())
//		finaldist_sort.erase(iter,finaldist_sort.end());
//
//	// find the index of the sorted element, some image indexes have same value
//	for(vector<double>::iterator it = finaldist_sort.begin(); it != finaldist_sort.end(); it++)
//	{
//		int index = -1;
//		for(vector<double>::iterator its = finaldist.begin(); its != finaldist.end(); its++)
//		{
//			if(abs((*it) - (*its)) < 0.00001)
//			{
//				index = its - finaldist.begin();
//				(*its) = 0;
//				break;
//			}
//		}
//		if(index != -1)
//			imgindex.push_back(index);
//	}
//}


/*
get the decomposed magnitude
*/
vector<Mat> CSAImageProcess::getDecEdgeResponse(int index)
{
	vector<Mat> result;

	assert(!m_pView->m_vDecNormMagnitude.empty() && index < m_pView->m_vDecNormMagnitude.size()/8);

	for(int i = 0; i < 8; i++)
	{
		result.push_back(m_pView->m_vDecNormMagnitude[index*8+i]);
	}

	return result;
}

/*
construct histogram by different resolutions
*/
void CSAImageProcess::lines2Histogram(const vector<Vec4i>& lines,float rho_resolution,float theta_resolution,Mat& hist)
{
	// formulate 2D Histogram
	float min_rho = -500, max_rho = 700;					// the range for rho
	float min_theta = - CV_PI/2, max_theta = CV_PI/2;		// the range for theta

	int rho_size = (max_rho - min_rho)/rho_resolution;
	int theta_size = (max_theta - min_theta)/(CV_PI / 180.0 * theta_resolution);
	int x = 0, y = 0;									// bin position
	float rho = 0.0 ,theta = 0.0;

	hist.create(theta_size,rho_size,CV_32F);
	hist = cv::Scalar(0);

	for(int i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		theta = atan(-(double)(l[2]-l[0])/(double)(l[3]-l[1]));

		/*
		if(abs(l[0] - l[2]) < 1)
		rho = l[0];
		else if(abs(l[1] - l[3]) < 1)
		rho = l[1];
		else
		{
		rho = l[0] * cos(theta) + l[1] * sin(theta);
		}
		//*/

		rho = l[0] * cos(theta) + l[1] * sin(theta);

		x = int((theta - min_theta)/(max_theta - min_theta) * theta_size);
		if(x < 0)
			x = 0;
		if(x > theta_size - 1)
			x = theta_size - 1;
		y = int((rho - min_rho)/(max_rho - min_rho) * rho_size);
		if(y < 0)
			y = 0;
		if(y > rho_size - 1)
			y = rho_size - 1;

		int votenum = (int)sqrt(double((l[2] - l[0]) * (l[2] - l[0]) + (l[3] - l[1]) * (l[3] - l[1])));

		hist.at<float>(x,y) += votenum;
	}
}

/*
convert Mat to ANNpoint
*/
void CSAImageProcess::mat2ANNpoint(const Mat& mat,ANNpoint& annpoint)
{
	// compute the dimensional for annpoint
	int d = mat.rows * mat.cols;

	annpoint = annAllocPt(d);

	for(int i = 0; i < mat.rows; i++)
	{
		for(int j = 0; j < mat.cols; j++)
		{
			annpoint[i * mat.rows + j] = mat.at<float>(i,j);
		}
	}
}

/*
	measure the patch distances between sketch image and db image using the difference between histograms(Hough) and search the result by LSH
*/
void CSAImageProcess::patchdistmeasureByLSH(const vector<Mat>& sketch,vector<int>& imgindex,vector<double>& finaldist_sort)
{
	IntT pointsDimension = sketch[0].rows * sketch[0].cols;

	if(m_pE2LSH == NULL)
	{
		m_pE2LSH = new CSAE2LSH();

		float thresholdR[] = {1000.0};
		m_pE2LSH->init(m_nDBNum,pointsDimension,0.9,1,thresholdR,"db_data.txt","db_data_parameter.txt");
	}

	// allocate the dist array
	if(m_bFirstVote)
	{
		m_bFirstVote = false;
	}

	if(m_bViewChanged)
	{
		m_bViewChanged = false;
	}

	PPointT queryPoint;
	RealT sqrLength = 0;
	FAILIF(NULL == (queryPoint = (PPointT)MALLOC(sizeof(PointT))));
	FAILIF(NULL == (queryPoint->coordinates = (RealT*)MALLOC(pointsDimension * sizeof(RealT))));

	queryPoint->index = -1;
	queryPoint->sqrLength = sqrLength;
	queryPoint->dimension = pointsDimension;

	IntT index = 0;
	for(int r = 0; r < sketch[0].rows; r++)
	{
		for(int c = 0; c < sketch[0].cols; c++)
		{
			queryPoint->coordinates[index] = sketch[0].at<int>(r,c);
			queryPoint->sqrLength += SQR(queryPoint->coordinates[index]);
			index++;
		}
	}

	if(m_pE2LSH)
		m_pE2LSH->query(queryPoint,imgindex,finaldist_sort);
}


/*
	construct histogram using the Angular Radial Partition method, N is the theta division, M is the radius division.
*/
void CSAImageProcess::edgeimg2HistogramByARP(const Mat& edgeimg,int N, int M, Mat& hist)
{
	int rows = edgeimg.rows;
	int cols = edgeimg.cols;

	// using the Angular Radial partition
	float R = sqrt((float)(rows * rows + cols * cols))/2.0;
	hist.create(N,M, CV_16UC1);
	hist = Scalar::all(0);

	int binx,biny;
	int centerx = rows/2;
	int centery = cols/2;

	for(int i = 0; i < rows; i++)
	{
		for(int j = 0; j < cols; j++)
		{
			if(edgeimg.at<uchar>(i,j) == 255)
			{
				// compute the binx, biny
				float coorx = i - centerx;
				float coory = j - centery;

				float length = sqrt(coorx*coorx + coory*coory);

				// the value range is in [0,2*PI]
				float theta = atan2(coory,coorx);
				if(theta < 0)
					theta += CV_PI * 2;

				binx = (int)(theta/(2 * CV_PI) * N);
				biny = (int)(length / R * M);

				// vote once
				hist.at<unsigned short>(binx,biny) += 1;
			}
		}
	}
}
