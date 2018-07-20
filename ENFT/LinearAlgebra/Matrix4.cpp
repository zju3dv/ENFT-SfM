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
#include "Matrix4.h"
#include "LinearSystem.h"
#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}

bool LA::InvertSymmetricUpper(AlignedMatrix4f &A) {
    char uplo = 'L';
    integer n = 4, lda = 4, info1, info2;
    spotrf_(&uplo, &n, A, &lda, &info1);
    spotri_(&uplo, &n, A, &lda, &info2);
    A.SetLowerFromUpper();
    return info1 == 0 && info2 == 0;
}

bool LA::InvertSymmetricUpper(const AlignedMatrix4f &A, AlignedMatrix4f &Ainv) {
    Ainv = A;
    return InvertSymmetricUpper(Ainv);
}

bool LA::SolveLinearSystemSymmetricUpper(AlignedMatrix4f &A,
        AlignedVector4f &b) {
    float *_A[4] = {&A.M00(), &A.M10(), &A.M20(), &A.M30()};
    return LA::SolveLinearSystemLDL<float, 4>(_A, b);
}