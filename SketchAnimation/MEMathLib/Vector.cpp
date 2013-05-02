/*
Simbicon 1.5 Controller Editor Framework, 
Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

This file is part of the Simbicon 1.5 Controller Editor Framework.

Simbicon 1.5 Controller Editor Framework is free software: you can 
redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Simbicon 1.5 Controller Editor Framework is distributed in the hope 
that it will be useful, but WITHOUT ANY WARRANTY; without even the 
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Simbicon 1.5 Controller Editor Framework. 
If not, see <http://www.gnu.org/licenses/>.
*/

#include ".\vector.h"
#include <assert.h>

#include <gsl/gsl_blas.h>
#include <Utils/Utils.h>

namespace MathLib
{


	/**
	constructor - creates an n row vector that is not initialized to any particular values
	*/
	Vector::Vector(int n) : Matrix(n,1){

	}

	/**
	default constructor
	*/
	Vector::Vector() : Matrix(4, 1){
		//4 for no good reason...

	}

	/**
	copy constructor - performs a deep copy of the matrix passed in as a parameter.
	*/
	Vector::Vector(const Vector& other) : Matrix(other){
	}

	/**
	destructor.
	*/
	Vector::~Vector(){
	}

	/**
	copy operator - performs a deep copy of the Vector passed in as a parameter.
	*/
	Vector& Vector::operator=(const Matrix &other){
		if (other.getColumnCount() != 1)
			throwError("Cannot copy a matrix into a vector.");
		Matrix::deepCopy(other);
		return *this;
	}

	/**
	this method performs a shallow copy of the Vector that is passed in as a parameter.
	*/
	void Vector::shallowCopy(const Matrix& other){
		if (other.getColumnCount() != 1)
			throwError("Cannot copy a matrix into a vector.");
		Matrix::shallowCopy(other);
	}

	/**
	copy operator - performs a deep copy of the Vector passed in as a parameter.
	*/
	Vector& Vector::operator=(const Vector &other){
		Matrix::deepCopy(other);
		return *this;
	}

	/**
	this method performs a deep copy of the vector that is passed in as a paramerer.
	*/
	void Vector::deepCopy(const Matrix& other){
		if (other.getColumnCount() != 1)
			throwError("Cannot copy a matrix into a vector.");
		Matrix::deepCopy(other);
	}

	/**
	This method sets the current vector to be equal to one of the products: A * b or A'*b.
	The value of transA indicates if A is transposed or not
	*/
	void Vector::setToProductOf(const Matrix& A, const Matrix& B, bool transA, bool transB){
		Matrix::setToProductOf(A, B, transA, transB);
		if (this->matrix->size2 != 1){
			resizeTo((int)this->matrix->size1, 1);
			throwError("Matrix product did not result in a vector!");
		}
	}

	/**
	This method prints the contents of the matrix - testing purpose only.
	*/
	void Vector::printVector() const{
		for (unsigned int i=0;i<this->matrix->size1;i++){
			logPrint("%+6.6lf ", this->get(i));
			logPrint("\n");
		}
	}

	/**
	This method returns a copy of the value of the matrix at (i,j)
	*/
	double Vector::get(int i) const{
		return VECTOR_AT(this->matrix, i);
	}

	/**
	This method sets the value of the matrix at (i,j) to newVal.
	*/
	void Vector::set(int i, double newVal){
		VECTOR_AT(this->matrix, i) = newVal;
	}


	/**
	This method sets the current vector to be equal to one of the rows of A - shallow column only!
	*/
	void Vector::setToRow(const Matrix& A, int row, int start, int howManyCols){
		if (this->matrix->owner){
			gsl_block_free(this->matrix->block);
		}

		//make sure that end, if unspecified is equal to the number of columns - i.e. a whole row
		if (howManyCols <= 0)
			howManyCols = A.matrix->size2 - start;

		//this is where it starts
		this->matrix->data = A.matrix->data + row * A.matrix->tda + start ;
		//this vector will have as many rows as the row we're copying has columns
		this->matrix->size1 = howManyCols;
		this->matrix->size2 = 1;
		//set this to 1 - as long as on a row of the matrix, elements are packed (i.e. MULTIPLICITY is 1), then this is fine. 
		this->matrix->tda = 1;
		this->matrix->block = A.matrix->block;
		this->matrix->owner = 0;
	}

	/**
	This method sets the current vector to be equal to one of the cols of A - shallow column only!
	*/
	void Vector::setToCol(const Matrix& A, int col, int start, int howManyRows){
		if (this->matrix->owner){
			gsl_block_free(this->matrix->block);
		}

		//make sure that end, if unspecified is equal to the number of columns - i.e. a whole row
		if (howManyRows <= 0)
			howManyRows = A.matrix->size1 - start;

		//this is where it starts
		this->matrix->data = A.matrix->data + start * A.matrix->tda + col ;
		//this vector will have as many rows as the column we're copying has rows
		this->matrix->size1 = howManyRows;
		this->matrix->size2 = 1;
		//set this to 1 - as long as on a row of the matrix, elements are packed (i.e. MULTIPLICITY is 1), then this is fine. 
		this->matrix->tda = A.matrix->tda;
		this->matrix->block = A.matrix->block;
		this->matrix->owner = 0;
	}


	void testVectorClass(){
		Matrix A(4,3);
		A.loadIdentity();
		A.set(3,2,5);
		A.printMatrix();
		printf("------\n");

		Vector v(3);
		v.set(0, 4);
		v.set(1, 3);
		v.set(2, 2);

		v.printVector();
		printf("------\n");

		Vector v1;
		v1.setToProductOf(A, v);

		v1.printVector();
		printf("------\n");

		v.setToProductOf(A, v1, true);
		v.printVector();
		printf("------\n");


	}

	double Vector::setDotProduct(const Vector& v1, bool row)
	{
		// compute the dot product
		double result;
		gsl_vector *mult1, *mult2;

		if(row) // row vector
		{
			// ensure the dimension of v1 is the same to the current vector
			assert(getRowCount() == 1 && v1.getRowCount() == 1 && getColumnCount() == v1.getColumnCount());

			mult1 = gsl_vector_alloc(getColumnCount());
			mult2 = gsl_vector_alloc(getColumnCount());

			gsl_matrix_get_row(mult1,this->matrix,0);
			gsl_matrix_get_row(mult2,v1.matrix,0);
		}
		else
		{
			assert(getColumnCount() == 1 && v1.getColumnCount() == 1 && getRowCount() == v1.getRowCount());

			mult1 = gsl_vector_alloc(getRowCount());
			mult2 = gsl_vector_alloc(getRowCount());

			gsl_matrix_get_col(mult1,this->matrix,0);
			gsl_matrix_get_col(mult2,v1.matrix,0);

		}

		gsl_blas_ddot (mult1, mult2, &result);

		gsl_vector_free(mult1);
		gsl_vector_free(mult2);

		return result;
	}

}