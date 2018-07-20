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
#include "Matrix11.h"
#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}
#include "LinearSystem.h"

bool LA::InvertSymmetricUpper(const AlignedMatrix11f &A,
                              AlignedMatrix11f &Ainv) {
    char uplo = 'L';
    integer n = 11, lda = 12, info1, info2;
    Ainv = A;
    spotrf_(&uplo, &n, Ainv, &lda, &info1);
    spotri_(&uplo, &n, Ainv, &lda, &info2);
    Ainv.SetLowerFromUpper();
    return info1 == 0 && info2 == 0;
}

bool LA::SolveLinearSystemSymmetricUpper(AlignedMatrix11f &A,
        AlignedVector11f &b) {
    float *_A[11] = {&A.M00(), &A.M10(), &A.M20(), &A.M30(), &A.M40(), &A.M50(), &A.M60(), &A.M70(), &A.M80(), &A.M90(), &A.M100()};
    return SolveLinearSystemLDL<float, 11>(_A, b);
}