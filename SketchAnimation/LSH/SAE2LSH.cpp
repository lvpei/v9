#include "SAE2LSH.h"


CSAE2LSH::CSAE2LSH(void)
{
	// variables initialized
	listOfRadii = NULL;
	nRadii = 1;
	memRatiosForNNStructs = NULL;

	algParameters = NULL;
	nnStructs = NULL;
}


CSAE2LSH::~CSAE2LSH(void)
{
	for(IntT i = 0; i < nRadii; i++){
		freePRNearNeighborStruct(nnStructs[i]);
	}
}

/*
	initialize the data structure using original data
*/
bool CSAE2LSH::init(int nPoints,int nDimension,float successProbability, int nThresholdR, float* thresholdR,const char* dataSet, const char* parameterFile)
{
	this->nPoints = nPoints;
	this->pointsDimension = nDimension;
	this->successProbability = successProbability;

	// currently, nThresholdR and thresholdR are not used
	nRadii = 1;
	FAILIF(NULL == (listOfRadii = (RealT*)MALLOC(nRadii * sizeof(RealT))));
	FAILIF(NULL == (memRatiosForNNStructs = (RealT*)MALLOC(nRadii * sizeof(RealT))));
	listOfRadii[0] = thresholdR[0];
	memRatiosForNNStructs[0] = 1;

	//DPRINTF("No. radii: %d\n", nRadii);
	printf("No. radii: %d\n",nRadii);

	// 1GB memory
	availableTotalMemory = 1000000000.0;

	if (nPoints > MAX_N_POINTS) {
		printf("Error: the structure supports at most %d points (%d were specified).\n", MAX_N_POINTS, nPoints);
		fprintf(ERROR_OUTPUT, "Error: the structure supports at most %d points (%d were specified).\n", MAX_N_POINTS, nPoints);
		exit(1);
	}

	// load the data from file
	readDataSetFromFile(dataSet,nPoints,nDimension);
	//DPRINTF("Allocated memory (after reading data set): %lld\n", totalAllocatedMemory);
	printf("Allocated memory (after reading data set): %lld\n", totalAllocatedMemory);

	Int32T nSampleQueries = N_SAMPLE_QUERY_POINTS;
	PPointT* sampleQueries = new PPointT[nSampleQueries];
	Int32T* sampleQBoundaryIndeces = new int[nSampleQueries];

	// Choose several data set points for the sample query points.
	for(IntT i = 0; i < nSampleQueries; i++)
	{
		sampleQueries[i] = dataSetPoints[genRandomInt(0, nPoints - 1)];
	}

	// for each query points from original data, we remove part of information from them
	for(IntT i = 0; i < nSampleQueries; i++)
	{
		PPointT ppoint = sampleQueries[i];

		// obtain the index whose element is not zero
		vector<int> nonzeroindex;
		for(IntT j = 0; j < nDimension; j++)
		{
			if(ppoint->coordinates[j] > 0.0)
			{	
				nonzeroindex.push_back(j);
			}
		}
		
		int k = i / 10;
		int nonzeroN = nonzeroindex.size();
		for(IntT j = 0; j < (int)(nonzeroN * k * 0.1);j++)	// each time remove k * 10% info
		{
			ppoint->coordinates[nonzeroindex[genRandomInt(0,nonzeroN-1)]] = 0;
		}
	}

	
	// Compute the array sampleQBoundaryIndeces that specifies how to
	// segregate the sample query points according to their distance
	// to NN.
	sortQueryPointsByRadii(pointsDimension,nSampleQueries,sampleQueries,nPoints,dataSetPoints,nRadii,listOfRadii,sampleQBoundaryIndeces);

	FAILIF(NULL == (nnStructs = (PRNearNeighborStructT*)MALLOC(nRadii * sizeof(PRNearNeighborStructT))));
	// Determine the R-NN DS parameters, construct the DS and run the queries.
	transformMemRatios();

	for(IntT i = 0; i < nRadii; i++)
	{
		// XXX: segregate the sample queries...
		nnStructs[i] = initSelfTunedRNearNeighborWithDataSet(listOfRadii[i], successProbability,nPoints,pointsDimension,dataSetPoints,nSampleQueries,sampleQueries, 
			(MemVarT)((availableTotalMemory - totalAllocatedMemory) * memRatiosForNNStructs[i]),parameterFile);
	}

	DPRINTF1("X\n");

	return true;
}

/*
initialize the data structure using precomputed parameters
*/
bool CSAE2LSH::init(const char* precomputedpara)
{
	// open the parameter files
	FILE *file = NULL;
	file = fopen(precomputedpara, "rt");
	if(file)
	{
		(*algParameters) = readRNNParameters(file);

		return true;
	}
	else
		return false;
}

/*
	search a high-dimensional point
*/
void CSAE2LSH::query(const PPointT queryPoint,vector<int>& imageidx,vector<double>& dist)
{
	IntT resultSize = nPoints;
	PPointT *result = (PPointT*)MALLOC(resultSize * sizeof(*result));

	TimeVarT meanQueryTime = 0;
	PPointAndRealTStructT *distToNN = NULL;

	RealT sqrLength = 0;
	// compute the square length of the query point.
	for(IntT d = 0; d < pointsDimension; d++)
	{
		sqrLength += SQR(queryPoint->coordinates[d]);
	}
	queryPoint->sqrLength = sqrLength;
	//printRealVector("Query: ", pointsDimension, queryPoint->coordinates);

	// get the near neighbors.
	IntT nNNs = 0;
	for(IntT r = 0; r < nRadii; r++)
	{
		nNNs = getRNearNeighbors(nnStructs[r], queryPoint, result, resultSize);
		printf("Total time for R-NN query at radius %0.6lf (radius no. %d):\t%0.6lf\n", (double)(listOfRadii[r]), r, timeRNNQuery);

		if (nNNs > 0){
			printf("Found %d NNs at distance %0.6lf (%dth radius). First %d NNs are:\n", nNNs, (double)(listOfRadii[r]), r, MIN(nNNs, MAX_REPORTED_POINTS));

			// compute the distances to the found NN, and sort according to the distance
			FAILIF(NULL == (distToNN = (PPointAndRealTStructT*)REALLOC(distToNN, nNNs * sizeof(*distToNN))));
			for(IntT p = 0; p < nNNs; p++){
				distToNN[p].ppoint = result[p];
				distToNN[p].real = distance(pointsDimension, queryPoint, result[p]);
			}
			qsort(distToNN, nNNs, sizeof(*distToNN), comparePPointAndRealTStructT);

			// Print the points
			for(IntT j = 0; j < MIN(nNNs, MAX_REPORTED_POINTS); j++){
				ASSERT(distToNN[j].ppoint != NULL);
				printf("%09d\tDistance:%0.6lf\n", distToNN[j].ppoint->index, distToNN[j].real);
				CR_ASSERT(distToNN[j].real <= listOfRadii[r]);
				//DPRINTF("Distance: %lf\n", distance(pointsDimension, queryPoint, result[j]));
				//printRealVector("NN: ", pointsDimension, result[j]->coordinates);
				imageidx.push_back(distToNN[j].ppoint->index);
				dist.push_back(distToNN[j].real);
			}
			break;
		}
		
		if (nNNs == 0)
		{
			printf("no NNs found.\n");
		}
	}
}

PPointT CSAE2LSH::readPoint(FILE *fileHandle,IntT pointsDimension)
{
	PPointT p;
	RealT sqrLength = 0;
	FAILIF(NULL == (p = (PPointT)MALLOC(sizeof(PointT))));
	FAILIF(NULL == (p->coordinates = (RealT*)MALLOC(pointsDimension * sizeof(RealT))));
	for(IntT d = 0; d < pointsDimension; d++){
		FSCANF_REAL(fileHandle, &(p->coordinates[d]));
		sqrLength += SQR(p->coordinates[d]);
	}
	fscanf(fileHandle, "%[^\n]", sBuffer);
	p->index = -1;
	p->sqrLength = sqrLength;
	p->dimension = pointsDimension;
	return p;
}

// Reads in the data set points from <filename> in the array
// <dataSetPoints>. Each point get a unique number in the field
// <index> to be easily identifiable.
void CSAE2LSH::readDataSetFromFile(const char *filename,IntT nPoints, IntT nDimension)
{
	FILE *f = fopen(filename, "rt");
	FAILIF(f == NULL);

	//fscanf(f, "%d %d ", &nPoints, &pointsDimension);
	//FSCANF_DOUBLE(f, &thresholdR);
	//FSCANF_DOUBLE(f, &successProbability);
	//fscanf(f, "\n");
	FAILIF(NULL == (dataSetPoints = (PPointT*)MALLOC(nPoints * sizeof(PPointT))));
	for(IntT i = 0; i < nPoints; i++){
		dataSetPoints[i] = readPoint(f,nDimension);
		dataSetPoints[i]->index = i;
	}
}

// transforming <memRatiosForNNStructs> from
// <memRatiosForNNStructs[i]=ratio of mem/total mem> to
// <memRatiosForNNStructs[i]=ratio of mem/mem left for structs i,i+1,...>.
void CSAE2LSH::transformMemRatios()
{
	RealT sum = 0;
	for(IntT i = nRadii - 1; i >= 0; i--){
		sum += memRatiosForNNStructs[i];
		memRatiosForNNStructs[i] = memRatiosForNNStructs[i] / sum;
		//DPRINTF("%0.6lf\n", memRatiosForNNStructs[i]);
	}
	ASSERT(sum <= 1.000001);
}
