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
#include "Matrix3.h"
#include "LinearSystem.h"

bool LA::InvertSymmetricUpper(AlignedMatrix3f &A) {
    //char uplo = 'L';
    //integer n = 3, lda = 4, info1, info2;
    //spotrf_(&uplo, &n, A, &lda, &info1);
    //spotri_(&uplo, &n, A, &lda, &info2);
    //A.SetLowerFromUpper();
    //return info1 == 0 && info2 == 0;
    //const float det = A.Determinant();
    const float det = A.M00() * (A.M11() * A.M22() - A.M12() * A.M12()) +
                      A.M01() * (A.M12() * A.M02() - A.M01() * A.M22()) +
                      A.M02() * (A.M01() * A.M12() - A.M11() * A.M02());
    if(fabs(det) < FLT_EPSILON) {
        A.SetZero();
        return false;
    }
    const float t = 1 / det, m00 = A.M00(), m01 = A.M01(), m02 = A.M02(), m11 = A.M11();
    A.M00() = (m11 * A.M22() - A.M12() * A.M12()) * t;
    A.M01() = (m02 * A.M12() - m01 * A.M22()) * t;
    A.M02() = (m01 * A.M12() - m02 * m11) * t;
    A.M11() = (m00 * A.M22() - m02 * m02) * t;
    A.M12() = (m02 * m01 - m00 * A.M12()) * t;
    A.M22() = (m00 * m11 - m01 * m01) * t;
    A.SetLowerFromUpper();
    return true;
}

bool LA::InvertSymmetricUpper(const AlignedMatrix3f &A, AlignedMatrix3f &Ainv) {
    //Ainv = A;
    //return InvertSymmetricUpper(Ainv);
    //const float det = A.Determinant();
    const float det = A.M00() * (A.M11() * A.M22() - A.M12() * A.M12()) +
                      A.M01() * (A.M12() * A.M02() - A.M01() * A.M22()) +
                      A.M02() * (A.M01() * A.M12() - A.M11() * A.M02());
    if(fabs(det) < FLT_EPSILON) {
        Ainv.SetZero();
        return false;
    }
    const float t = 1 / det;
    Ainv.M00() = (A.M11() * A.M22() - A.M12() * A.M12()) * t;
    Ainv.M01() = (A.M02() * A.M12() - A.M01() * A.M22()) * t;
    Ainv.M02() = (A.M01() * A.M12() - A.M02() * A.M11()) * t;
    Ainv.M11() = (A.M00() * A.M22() - A.M02() * A.M02()) * t;
    Ainv.M12() = (A.M02() * A.M01() - A.M00() * A.M12()) * t;
    Ainv.M22() = (A.M00() * A.M11() - A.M01() * A.M01()) * t;
    Ainv.SetLowerFromUpper();
    return true;
}

bool LA::SolveLinearSystemSymmetricUpper(AlignedMatrix3f &A, AlignedVector3f &b) {
    float *_A[3] = {&A.M00(), &A.M10(), &A.M20()};
    return SolveLinearSystemLDL<float, 3>(_A, b);
}