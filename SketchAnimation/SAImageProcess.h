/************************************************************************/
/* CSAImageProcess Class                                                */
/* used for image feature extraction and other operations               */
/*																		*/
/* Author: lvp															*/
/* Date: 2012-3-7														*/
/************************************************************************/
#pragma once

#include <opencv2/core/core.hpp>
#include <vector>
#include "SAView.h"
#include "ANN/ANN.h"

using namespace cv;
using namespace std;

/*
	The patch histogram used for BICE feature extraction
*/
const int BIN_X = 10;
const int BIN_Y = 10;
const int BIN_THETA = 4;

const float _COEFFICIENT_ = 1000.0;		// enlarge the pixel color value
const int _REDUCE_RESOLUTION_ = 3;		// reduce the resolution to 480/2^_REDUCE_RESOLUTION_

class CSAE2LSH;
class CSAImageProcess
{
public:
	CSAImageProcess(void);
	~CSAImageProcess(void);
	
	/* 
		extract the edge
	*/
	void edgeextraction(const Mat& color_image ,Mat& edgelength, Mat& edgeorientation,Mat& m,Mat& m_n,Mat& edgeresponse);
	
	/*
		normalize the values in the matrix according to the given window size
	*/
	void normmat(const Mat& m,int winsize,Mat& m_n);
	
	/*
		find the edge position through comparing the color, ensure that m is a color matrices
	*/
	void findedgepos(const Mat& m,uchar color,vector<int>& edgepos);
	
	/*
		compute the patch descriptor
	*/
	void computepatchdescriptor(const Mat& edgeorientation,const Mat& m_n, const vector<int>& edgepos,Mat& hist);
	
	/*
		process the sketch image in batch
	*/
	void batch_process_sketch(const char* folder,vector<Mat>& bin_sketch,vector<int>& patchnum);
	
	/* 
		process the sketch mat in batch
	*/
	void batch_process_sketch(const vector<Mat>& img,vector<Mat>& bin_sketch);
	
	/*
		process the database image in batch and store them into files
	*/
	void batch_process_db(const char* folder,vector<Mat>& bin_db,const char* savefile, const char* mfile, const char* ofile, const char* decfile,FEATURE feature);

	/*
		process the database image in batch and store them into files using the Hough transform 12-7-7
	*/
	void batch_process_db(const char* folder,vector<Mat>& bin_db,const char* savefile, const char* mfile, const char* ofile, const char* decfile,const char* vecfile,FEATURE feature);
	
	/* 
		measure the patch distances between sketch patches and db patches using the difference between histograms
	*/
	void patchdistmeasure(const vector<Mat>& sketch,const vector<int>& patchnum,vector<vector<double>>& dist,vector<vector<int>>& sortpatchindex,vector<int>& imgindex,vector<double>& finaldist_sort);

	/* 
		measure the descriptor distances between sketch image and db images using the difference between histograms(Hough),
		limb_idx = -1, indicate comparing with the whole image
		limb_idx = 0-6, indicate comparing with different limb images
	*/
	void patchdistmeasure(const vector<Mat>& sketch,vector<int>& imgindex,vector<double>& finaldist_sort, int limb_idx = -1, float weight = 1.0); 

	/*
		measure the patch distances between sketch image and db image using the difference between histograms(Hough) and search the result by ANN
	*/
	void patchdistmeasureByANN(const vector<Mat>& sketch,vector<int>& imgindex,vector<double>& finaldist_sort);

	/*
		measure the patch distances between sketch image and db image using the difference between histograms(Hough) and search the result by LSH
	*/
	void patchdistmeasureByLSH(const vector<Mat>& sketch,vector<int>& imgindex,vector<double>& finaldist_sort);

	/* 
		measure the patch distances between sketch image and db images using the difference between histograms(Hough)
	*/
	void patchdistmeasure(const vector<vector<Vec4i>>& sketch_lines,vector<int>& imgindex,vector<double>& finaldist_sort);

	/* 
		find the surrounding patch given the patch number
	*/
	void findsurroundingpatches(int patchnum, vector<int>& patchindex);
	
	/*
		process one patch and obtain the patch descriptor, normalized magnitude and orientation
	*/
	void process_one_patch(const Mat& patch, Mat& bin,Mat& m_n, Mat& orientation);
	
	/*
		process one image for database
	*/
	void process_one_image(const char* filename,vector<Mat>&bin,FEATURE feature);	

	/*
		process one image for database using the Hough transform 12-7-7
	*/
	void process_one_image(const char* filename,vector<Mat>&bin,vector<vector<Vec4i>>& linesarr, FEATURE feature);	

	/*
		process sketch image
	*/
	void process_sketch_image(const Mat& mat,vector<Mat>&bin,FEATURE feature);	

	/*
		process sketch image using the Hough transform 12-7-7
	*/
	void process_sketch_image(const Mat& mat,vector<Mat>&bin,vector<vector<Vec4i>>& linesarr,FEATURE feature);	

	/* 
		min-hash correlated
	*/
	void minhash(const vector<Mat>& bin_db,vector<vector<int>>& sketches,bool usr_sketch = false,const char* filename = NULL);
	
	/*
		permute the array index randomly
	*/
	void permutesetindex(const vector<int>& oneindex,vector<int>& hashvalue);
	
	/* 
		vote
	*/
	//void vote(const vector<vector<int>>& usr_hashvalue,const vector<int>& patchnum,vector<int>& imgindex,vector<vector<int>>& nearestpatchidx,vector<int>& finaldist_sort);
	
	/* 
		continuous comparison for voting
	*/
	//void vote(const vector<Mat>& bin_db,const vector<vector<int>>& usr_hashvalue,const vector<int>& patchnum,vector<int>& imgindex,vector<vector<int>>& nearestpatchidx,vector<double>& finaldist_sort);

	/* 
		image alignment
	*/
	void imagealignment(const Mat& usr_sketch,const vector<Mat>& db_sketch,vector<vector<double>>& transform,vector<Mat>& db_sketch_new,vector<Mat>& db_sketch_orientation,bool useedgeimage = false);
	
	/* 
		image alignment using Lucas-Kanade Registration
	*/
	void imagealignment(const Mat& usr_sketch,const vector<Mat>& db_sketch,vector<vector<double>>& transform);

	/*
		decompose edge image into different oriented images according to given orientation(radian)
	*/
	void decomposeimage(const Mat& m, const Mat& orientation,const vector<double>& orientation_v,vector<Mat>& dec_m);
	
	/*
		decompose edge image into different oriented images according to given orientation(radian) through setting the image index in database
	*/
	void decomposeimage(int index,const vector<double>& orientation_v,vector<Mat>& dec_m);

	/*
		compute positive and negative edge correlation images
	*/
	void computeedgecorrimg(const vector<Mat>& dec_db_align,const vector<Mat>& dec_usr_sketch,Mat& positive_corr_img,Mat& negative_corr_img);

	/*
		whether it is the first vote
	*/
	void setFirstVoteStatus(bool firstvote){m_bFirstVote = firstvote;}

	/*
		whether the view is changed
	*/
	void setViewChanged(bool changed){m_bViewChanged = changed;}

	/*
		get the edge response
	*/
	Mat getEdgeResponse(int index){assert(!m_pView->m_vNormMagnitude.empty() && index < m_pView->m_vNormMagnitude.size()); return m_pView->m_vNormMagnitude[index];}
	
	/*
		get the edge orientation
	*/
	Mat getEdgeOrientation(int index){assert(!m_pView->m_vOrientation.empty() && index < m_pView->m_vOrientation.size()); return m_pView->m_vOrientation[index];}
	
	/*
		get the decomposed magnitude
	*/
	vector<Mat> getDecEdgeResponse(int index);

	/*
		set the current view
	*/
	void setView(CSAView* view){m_pView = view; m_nDBNum = m_pView->m_nDBNum;}

private:
	int findminvalueinarray(const vector<int>& arr);

	/*	estimating the lengths of two sub-edges that start at the pixel and
		proceed in two opposite directions given by the local edge orientation(po 
		and po+pi)
	*/
	void pixelproject(const Mat& orientation,float length,Mat& prj1,Mat& prj2);
	
	/*	
		length estimation using message passing as described in GradientShop2010.
	*/
	void lengthestimation(const Mat& m_n,const Mat& orientation,const Mat& prj1,const Mat& prj2,int iter,Mat& edgelength);
	
	/*
		compute the edge response according to the edge length, edge orientation and gradient
	*/
	void computeedgeresponse(const Mat& edgelength,const Mat& orientation,const Mat& grad_x,const Mat& grad_y,Mat& edgeresponse);
	
	/*
		reset the vote matrix H
	*/
	void resetVoteMatrix();
	
	/*
		warp image, currently only for translation
	*/
	void warpImg(const Mat& original, Mat& warped,float x, float y);

	/*
		convert Mat to ANNpoint
	*/
	void mat2ANNpoint(const Mat& mat,ANNpoint& annpoint);

	/*
		construct histogram using the Angular Radial Partition method, N is the theta division, M is the radius division.
	*/
	void edgeimg2HistogramByARP(const Mat& edgeimg,int N, int M, Mat& hist);

public:
	/*
		construct histogram by different resolutions
	*/
	void lines2Histogram(const vector<Vec4i>& lines,float rho_resolution,float theta_resolution,Mat& hist);

private:
	int H[_MAX_UPDATED_PATCH_][_MAX_DB_IMG_][_PATCH_INDEX_RANGE_];	// the voted matrix
	bool m_bFirstVote;												// whether this is the first vote
	bool m_bViewChanged;											// whether the view is changed
	int	m_nDBNum;													// the loaded number of database image
	
	/*
		the current view processed by CSAImageProcess class
	*/
	CSAView* m_pView;												
	
	/*
		The flag showing whether this image is voted
	*/
	bool m_vImageVoted[_MAX_DB_IMG_];

	/*
		The fixed resolution for Histogram
	*/
	int m_iTheta;			// measured by degree
	int m_iRes;				// measured by pixel

	CSAE2LSH* m_pE2LSH;		
 };