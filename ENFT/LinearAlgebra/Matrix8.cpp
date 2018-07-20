////////////////////////////////////////////////////////////////////////////
//  Copyright 2017-2018 Computer Vision Group of State Key Lab at CAD&CG, 
//  Zhejiang University. All Rights Reserved.
//
//  For more information see <https://github.com/ZJUCVG/ENFT-SfM>
//  If you use this code, please cite the corresponding publications as 
//  listed on the above website.
//
//  Permission to use, copy, modify and distribute this software and its
//  documentation for educational, research and non-profit purposes only.
//  Any modification based on this work must be open source and prohibited
//  for commercial use.
//  You must retain, in the source form of any derivative works that you 
//  distribute, all copyright, patent, trademark, and attribution notices 
//  from the source form of this work.
//   
//
////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Matrix8.h"
#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}
#include "LinearSystem.h"

bool LA::InvertSymmetricUpper(const LA::AlignedMatrix8f &A,
                              LA::AlignedMatrix8f &Ainv, float *work64) {
    char uplo = 'L';
    integer n = 8, lda = 8, info1, info2;
    Ainv = A;
    spotrf_(&uplo, &n, Ainv, &lda, &info1);
    spotri_(&uplo, &n, Ainv, &lda, &info2);
    Ainv.SetLowerFromUpper();
    //const float* _A[8] = {&A.M00(), &A.M10(), &A.M20(), &A.M30(), &A.M40(), &A.M50(), &A.M60(), &A.M70()};
    //const float* _Ainv[8] = {&Ainv.M00(), &Ainv.M10(), &Ainv.M20(), &Ainv.M30(), &Ainv.M40(), &Ainv.M50(), &Ainv.M60(), &Ainv.M70()};
    //for(int i = 0; i < 8; ++i)
    //for(int j = 0; j < 8; ++j)
    //{
    //  float sum = 0;
    //  for(int k = 0; k < 8; ++k)
    //      sum += _A[i][k] * _Ainv[k][j];
    //  printf("(%d, %d): %f\n", i, j, sum);
    //}
    return info1 == 0 && info2 == 0;
    //Ainv = A;
    //float* _Ainv[8] = {&Ainv.M00(), &Ainv.M10(), &Ainv.M20(), &Ainv.M30(), &Ainv.M40(), &Ainv.M50(), &Ainv.M60(), &Ainv.M70()};
    ////return InvertLDL<float, 8>(_Ainv, work64);
    //InvertLDL<float, 8>(_Ainv, work64);
    //const float* _A[8] = {&A.M00(), &A.M10(), &A.M20(), &A.M30(), &A.M40(), &A.M50(), &A.M60(), &A.M70()};
    //for(int i = 0; i < 8; ++i)
    //for(int j = 0; j < 8; ++j)
    //{
    //  float sum = 0;
    //  for(int k = 0; k < 8; ++k)
    //      sum += _A[i][k] * _Ainv[k][j];
    //  printf("(%d, %d): %f\n", i, j, sum);
    //}
    return true;
}

//typedef Eigen::Matrix<float, 8, 8, Eigen::AutoAlign | Eigen::RowMajor> EigenMatrix8f;
//static inline Eigen::Map<EigenMatrix8f> ToEigenMatrix(float *data)
//{
//  return Eigen::Map<EigenMatrix8f>(data);
//}
//typedef Eigen::Matrix<float, 8, 1> EigenVector8f;
//static inline Eigen::Map<EigenVector8f> ToEigenVector(float *data)
//{
//  return Eigen::Map<EigenVector8f>(data);
//}

bool LA::SolveLinearSystemSymmetricUpper(AlignedMatrix8f &A,
        AlignedVector8f &b) {
    float *_A[8] = {&A.M00(), &A.M10(), &A.M20(), &A.M30(), &A.M40(), &A.M50(), &A.M60(), &A.M70()};
    return SolveLinearSystemLDL<float, 8>(_A, b);
    //A.SetLowerFromUpper();
    //EigenMatrix8f _A = ToEigenMatrix(A);
    //EigenVector8f _b = ToEigenVector(b);
    //EigenVector8f x = _A.ldlt().solve(_b);
    //memcpy(b, x.data(), 32);
    //return true;
}

bool LA::SolveLinearSystem(AlignedMatrix8f &A, AlignedVector8f &b,
                           float *work24) {
    float *_A[8] = {&A.M00(), &A.M10(), &A.M20(), &A.M30(), &A.M40(), &A.M50(), &A.M60(), &A.M70()};
    return SolveLinearSystemLU<float, 8>(_A, b, work24, (int *) work24 + 16);
    //EigenMatrix8f _A = ToEigenMatrix(A);
    //EigenVector8f _b = ToEigenVector(b);
    //EigenVector8f x = _A.fullPivLu().solve(_b);
    //memcpy(b, x.data(), 32);
    //return true;
}