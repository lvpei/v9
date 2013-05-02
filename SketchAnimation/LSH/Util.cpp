/*
 * Copyright (c) 2004-2005 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * MIT grants permission to use, copy, modify, and distribute this software and
 * its documentation for NON-COMMERCIAL purposes and without fee, provided that
 * this copyright notice appears in all copies.
 *
 * MIT provides this software "as is," without representations or warranties of
 * any kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability, fitness for a particular purpose, and
 * noninfringement.  MIT shall not be liable for any damages arising from any
 * use of this software.
 *
 * Author: Alexandr Andoni (andoni@mit.edu), Piotr Indyk (indyk@mit.edu)
 */

#include "headers.h"

// Verifies whether vector v1 and v2 are equal (component-wise). The
// size of the vectors is given by the parameter size.
BooleanT vectorsEqual(IntT size, IntT *v1, IntT *v2){
  for(IntT i = 0; i < size; i++){
    if (v1[i] != v2[i]){
      return FALSE;
    }
  }
  return TRUE;
}

// Copies the vector <from> to the vector <to>. The size of the
// vectors is given by <size>.
void copyVector(IntT size, IntT *from, IntT *to){
  for(IntT i = 0; i < size; i++){
    to[i] = from[i];
  }
}

// Creates a new vector of size <size> and copies the vector <from> to
// the new vector.
IntT *copyOfVector(IntT size, IntT *from){
  IntT *newVector;
  FAILIF(NULL == (newVector = (IntT*)MALLOC(size * sizeof(IntT))));
  for(IntT i = 0; i < size; i++){
    newVector[i] = from[i];
  }
  return newVector;
}

// Prints the vector <v> of size <size>. The string <s> appears
// in front.
void printRealVector(char *s, IntT size, RealT *v){
  ASSERT(v != NULL);
  
  printf("%s", s);
  for(IntT i = 0; i < size; i++){
    if (i > 0){
      printf(" ");
    }
    printf("%0.2lf", (double)v[i]);
  }

  printf("\n");
}

// Prints the vector <v> of size <size>. The string <s> appears
// in front.
void printIntVector(char *s, IntT size, IntT *v){
  ASSERT(v != NULL);
  
  printf("%s", s);
  for(IntT i = 0; i < size; i++){
    if (i > 0){
      printf(" ");
    }
    printf("%d", v[i]);
  }

  printf("\n");
}

// Returns the amount of available memory.
MemVarT getAvailableMemory(){
  // TODO
  //printf("mem=%lu\n", MEMORY_MAX_AVAILABLE - totalAllocatedMemory);
  FAILIFWR(availableTotalMemory < totalAllocatedMemory, "Not enough memory.\n");
  return availableTotalMemory - totalAllocatedMemory; 
}

double erf(double x)
{
	// constants
	double a1 =  0.254829592;
	double a2 = -0.284496736;
	double a3 =  1.421413741;
	double a4 = -1.453152027;
	double a5 =  1.061405429;
	double p  =  0.3275911;

	// Save the sign of x
	int sign = 1;
	if (x < 0)
		sign = -1;
	x = fabs(x);

	// A&S formula 7.1.26
	double t = 1.0/(1.0 + p*x);
	double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

	return sign*y;
}
