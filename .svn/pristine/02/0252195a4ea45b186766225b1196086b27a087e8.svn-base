/*
	This class is used to encode the E2LSH library. 
*/
#pragma once

#include "headers.h"
#include <vector>

#define N_SAMPLE_QUERY_POINTS 1000

using namespace std;

class CSAE2LSH
{
public:
	CSAE2LSH(void);
	~CSAE2LSH(void);
	
	/*
		initialize the data structure using original data
	*/
	bool init(int nPoints,int nDimension,float successProbability, int nThresholdR, float* thresholdR,const char* dataSet, const char* parameterFile = NULL);
	
	/*
		initialize the data structure using precomputed parameters
	*/
	bool init(const char* precomputedpara);

	/*
		search a high-dimensional point
	*/
	void query(const PPointT query,vector<int>& imageidx,vector<double>& dist);

private:
	PPointT readPoint(FILE *fileHandle, int pointDimension);
	void readDataSetFromFile(const char *filename,int nPoints, int nDimension);
	void transformMemRatios();

private:
	// The data set containing all the points.
	PPointT *dataSetPoints;
	// Number of points in the data set.
	IntT nPoints;
	// The dimension of the points.
	IntT pointsDimension;
	// The value of parameter R (a near neighbor of a point <q> is any
	// point <p> from the data set that is the within distance
	// <thresholdR>).
	//RealT thresholdR = 1.0;

	// The success probability of each point (each near neighbor is
	// reported by the algorithm with probability <successProbability>).
	RealT successProbability;

	// Same as <thresholdR>, only an array of R's (for the case when
	// multiple R's are specified).
	RealT *listOfRadii;
	IntT nRadii;

	RealT *memRatiosForNNStructs;

	char sBuffer[600000];

	RNNParametersT *algParameters;
	PRNearNeighborStructT *nnStructs;
};

