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
#include "Matrix6.h"
#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}
#include "LinearSystem.h"

bool LA::InvertSymmetricUpper(AlignedCompactMatrix6f &A, float *work36) {
    char uplo = 'L';
    integer n = 6, lda = 6, info1, info2;
    A.ConvertToConventionalStorage(work36);
    spotrf_(&uplo, &n, A, &lda, &info1);
    spotri_(&uplo, &n, A, &lda, &info2);
    A.ConvertToSpecialStorage(work36);
    A.SetLowerFromUpper();
    return info1 == 0 && info2 == 0;
    //A.ConvertToConventionalStorage(work36);
    //float *pA = A;
    //float* _A[6] = {pA, pA + 6, pA + 12, pA + 18, pA + 24, pA + 30};
    //if(!InvertLDL<float, 6>(_A, work36))
    //  return false;
    //A.ConvertToSpecialStorage(work36);
    //A.SetLowerFromUpper();
    //return true;
}

bool LA::InvertSymmetricUpper(const AlignedCompactMatrix6f &A,
                              AlignedCompactMatrix6f &Ainv, float *work36) {
    char uplo = 'L';
    integer n = 6, lda = 6, info1, info2;
    A.ConvertToConventionalStorage(Ainv);
    spotrf_(&uplo, &n, Ainv, &lda, &info1);
    spotri_(&uplo, &n, Ainv, &lda, &info2);
    Ainv.ConvertToSpecialStorage(work36);
    Ainv.SetLowerFromUpper();
    return info1 == 0 && info2 == 0;
    //A.ConvertToConventionalStorage(Ainv);
    //float *pA = Ainv;
    //float* _A[6] = {pA, pA + 6, pA + 12, pA + 18, pA + 24, pA + 30};
    //if(!InvertLDL<float, 6>(_A, work36))
    //  return false;
    //Ainv.ConvertToSpecialStorage(work36);
    //Ainv.SetLowerFromUpper();
    //return true;
}

bool LA::SolveLinearSystemSymmetricUpper(AlignedCompactMatrix6f &A,
        AlignedVector6f &b, float *work2) {
    A.ConvertToConventionalStorage(work2);
    //return SolveLinearSystemLDL<float, 6>((float(*)[6]) ((float *) A), b);
    float *pA = A;
    float *_A[6] = {pA, pA + 6, pA + 12, pA + 18, pA + 24, pA + 30};
    //printf("\n");
    //Print<float, 6>(_A);
    return SolveLinearSystemLDL<float, 6>(_A, b);
}