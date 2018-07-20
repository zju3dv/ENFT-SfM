// -*- c++ -*-

// Copyright (C) 2005,2009 Tom Drummond (twd20@cam.ac.uk)
//
// This file is part of the TooN Library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along
// with this library; see the file COPYING.  If not, write to the Free
// Software Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307,
// USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU General Public License.

#ifndef TOON_INCLUDE_LAPCK_H
#define TOON_INCLUDE_LAPCK_H

// LAPACK and BLAS routines
namespace TooN {

	extern "C" {
		// LU decomoposition of a general matrix
		void dgetrf_(int* M, int *N, double* A, int* lda, int* IPIV, int* INFO);
		void sgetrf_(int* M, int *N, float* A, int* lda, int* IPIV, int* INFO);

		// generate inverse of a matrix given its LU decomposition
		void dgetri_(int* N, double* A, int* lda, int* IPIV, double* WORK, int* lwork, int* INFO);
		void sgetri_(int* N, float* A, int* lda, int* IPIV, float* WORK, int* lwork, int* INFO);

		// inverse of a triangular matrix * a vector (BLAS level 2)
		void dtrsm_(char* SIDE, char* UPLO, char* TRANSA, char* DIAG, int* M, int* N, double* alpha, double* A, int* lda, double* B, int* ldb);
		void strsm_(char* SIDE, char* UPLO, char* TRANSA, char* DIAG, int* M, int* N, float* alpha, float* A, int* lda, float* B, int* ldb);
  

		// SVD of a general matrix
		void dgesvd_(const char* JOBU, const char* JOBVT, int* M, int *N, double* A, int* lda,
					 double* S, double *U, int* ldu, double* VT, int* ldvt,
					 double* WORK, int* lwork, int* INFO);

		void sgesvd_(const char* JOBU, const char* JOBVT, int* M, int *N, float* A, int* lda,
					 float* S, float *U, int* ldu, float* VT, int* ldvt,
					 float* WORK, int* lwork, int* INFO);

		// Eigen decomposition of a symmetric matrix
		void dsyev_(const char* JOBZ, const char* UPLO, int* N, double* A, int* lda, double* W, double* WORK, int* LWORK, int* INFO);
		void ssyev_(const char* JOBZ, const char* UPLO, int* N, float* A, int* lda, float* W, float* WORK, int* LWORK, int* INFO);

		// Cholesky decomposition
		void dpotrf_(const char* UPLO, const int* N, double* A, const int* LDA, int* INFO);
		void spotrf_(const char* UPLO, const int* N, float* A, const int* LDA, int* INFO);

		// Cholesky solve AX=B given decomposition
		void dpotrs_(const char* UPLO, const int* N, const int* NRHS, const double* A, const int* LDA, double* B, const int* LDB, int* INFO);
		void spotrs_(const char* UPLO, const int* N, const int* NRHS, const float* A, const int* LDA, float* B, const int* LDB, int* INFO);

		// Cholesky inverse given decomposition
		void dpotri_(const char* UPLO, const int* N, double* A, const int* LDA, int* INFO);
		void spotri_(const char* UPLO, const int* N, float* A, const int* LDA, int* INFO);
	}


	//////////////////////////////////////////////////////////////////////////////////
	// C++ overloaded functions to access single and double precision automatically //
	//////////////////////////////////////////////////////////////////////////////////

	inline void getrf_(int* M, int *N, float* A, int* lda, int* IPIV, int* INFO){
		sgetrf_(M, N, A, lda, IPIV, INFO);
	}

	inline void getrf_(int* M, int *N, double* A, int* lda, int* IPIV, int* INFO){
		dgetrf_(M, N, A, lda, IPIV, INFO);
	}

	inline void trsm_(const char* SIDE, const char* UPLO, const char* TRANSA, const char* DIAG, int* M, int* N, float* alpha, float* A, int* lda, float* B, int* ldb) { 
		strsm_(const_cast<char*>(SIDE), const_cast<char*>(UPLO), const_cast<char*>(TRANSA), const_cast<char*>(DIAG), M, N, alpha, A, lda, B, ldb);
	}

	inline void trsm_(const char* SIDE, const char* UPLO, const char* TRANSA, const char* DIAG, int* M, int* N, double* alpha, double* A, int* lda, double* B, int* ldb) {
		dtrsm_(const_cast<char*>(SIDE), const_cast<char*>(UPLO), const_cast<char*>(TRANSA), const_cast<char*>(DIAG), M, N, alpha, A, lda, B, ldb);
	}

	inline void getri_(int* N, double* A, int* lda, int* IPIV, double* WORK, int* lwork, int* INFO){
		dgetri_(N, A, lda, IPIV, WORK, lwork, INFO);
	}

	inline void getri_(int* N, float* A, int* lda, int* IPIV, float* WORK, int* lwork, int* INFO){
		sgetri_(N, A, lda, IPIV, WORK, lwork, INFO);
	}

	inline void potrf_(const char * UPLO, const int* N, double* A, const int* LDA, int* INFO){
		dpotrf_(UPLO, N, A, LDA, INFO);
	}

	inline void potrf_(const char * UPLO, const int* N, float* A, const int* LDA, int* INFO){
		spotrf_(UPLO, N, A, LDA, INFO);
	}

	// SVD
	inline void gesvd_(const char* JOBU, const char* JOBVT, int* M, int *N, double* A, int* lda,
				double* S, double *U, int* ldu, double* VT, int* ldvt,
				double* WORK, int* lwork, int* INFO){
		dgesvd_(JOBU, JOBVT, M, N, A, lda, S, U, ldu, VT, ldvt, WORK, lwork, INFO);
	}

	inline void gesvd_(const char* JOBU, const char* JOBVT, int* M, int *N, float* A, int* lda,
					 float* S, float *U, int* ldu, float* VT, int* ldvt,
					 float* WORK, int* lwork, int* INFO){
		sgesvd_(JOBU, JOBVT, M, N, A, lda, S, U, ldu, VT, ldvt, WORK, lwork, INFO);
	}

	// Cholesky solve AX=B given decomposition
	inline void potrs_(const char* UPLO, const int* N, const int* NRHS, const double* A, const int* LDA, double* B, const int* LDB, int* INFO){
		dpotrs_(UPLO, N, NRHS, A, LDA, B, LDB, INFO);
	}

	inline void potrs_(const char* UPLO, const int* N, const int* NRHS, const float* A, const int* LDA, float* B, const int* LDB, int* INFO){
		spotrs_(UPLO, N, NRHS, A, LDA, B, LDB, INFO);
	}

	// Cholesky inverse given decomposition
	inline void potri_(const char* UPLO, const int* N, double* A, const int* LDA, int* INFO){
		dpotri_(UPLO, N, A, LDA, INFO);
	}

	inline void potri_(const char* UPLO, const int* N, float* A, const int* LDA, int* INFO){
		spotri_(UPLO, N, A, LDA, INFO);
	}

	inline void syev_(const char* JOBZ, const char* UPLO, int* N, double* A, int* lda, double* W, double* WORK, int* LWORK, int* INFO){
		dsyev_(JOBZ, UPLO, N, A, lda, W, WORK, LWORK, INFO);
	}
	inline void syev_(const char* JOBZ, const char* UPLO, int* N, float* A, int* lda, float* W, float* WORK, int* LWORK, int* INFO){
		ssyev_(JOBZ, UPLO, N, A, lda, W, WORK, LWORK, INFO);
	}

}
#endif
