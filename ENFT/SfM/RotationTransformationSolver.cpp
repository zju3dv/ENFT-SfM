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
#include "RotationTransformationSolver.h"

#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}

#if _DEBUG
#include "assert.h"
#endif

bool RotationTransformationSolver::Run(const ThreeMatches3D &data,
                                       RotationTransformation3D &R, AlignedVector<ENFT_SSE::__m128> &work) {
    ComputeCovariance(data);
    return RecoverRotation(R, work);
}

bool RotationTransformationSolver::Run(const SixMatches3D &data,
                                       RotationTransformation3D &R, AlignedVector<ENFT_SSE::__m128> &work) {
    ComputeCovariance(data);
    return RecoverRotation(R, work);
}

bool RotationTransformationSolver::Run(const AlignedVector<Point3D> &X1s,
                                       const AlignedVector<Point3D> &X2s, RotationTransformation3D &R,
                                       AlignedVector<ENFT_SSE::__m128> &work) {
    ComputeCovariance(X1s, X2s);
    return RecoverRotation(R, work);
}

void RotationTransformationSolver::ComputeCovariance(const ThreeMatches3D
        &data) {
#if _DEBUG
    assert(data.X1(0).reserve() == 1 && data.X1(1).reserve() == 1 &&
           data.X1(2).reserve() == 1);
#endif

    m_C.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(data.X1(0).XYZx(),
                                    _mm_set1_ps(data.X2(0).X())),
                                    ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(data.X1(1).XYZx(), _mm_set1_ps(data.X2(1).X())),
                                            ENFT_SSE::_mm_mul_ps(data.X1(2).XYZx(), _mm_set1_ps(data.X2(2).X()))));
    m_C.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(data.X1(0).XYZx(),
                                    _mm_set1_ps(data.X2(0).Y())),
                                    ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(data.X1(1).XYZx(), _mm_set1_ps(data.X2(1).Y())),
                                            ENFT_SSE::_mm_mul_ps(data.X1(2).XYZx(), _mm_set1_ps(data.X2(2).Y()))));
    m_C.M_20_21_22_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(data.X1(0).XYZx(),
                                    _mm_set1_ps(data.X2(0).Z())),
                                    ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(data.X1(1).XYZx(), _mm_set1_ps(data.X2(1).Z())),
                                            ENFT_SSE::_mm_mul_ps(data.X1(2).XYZx(), _mm_set1_ps(data.X2(2).Z()))));
}

void RotationTransformationSolver::ComputeCovariance(const SixMatches3D &data) {
#if _DEBUG
    assert(data.X1(0).reserve() == 1 && data.X1(1).reserve() == 1 &&
           data.X1(2).reserve() == 1);
    assert(data.X1(3).reserve() == 1 && data.X1(4).reserve() == 1 &&
           data.X1(5).reserve() == 1);
#endif

    m_C.SetZero();

#ifndef ACCUMULATE_COVARIANCE_6
#define ACCUMULATE_COVARIANCE_6(i)\
    m_C.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(data.X1(i).XYZx(), _mm_set1_ps(data.X2(i).X())), m_C.M_00_01_02_x());\
    m_C.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(data.X1(i).XYZx(), _mm_set1_ps(data.X2(i).Y())), m_C.M_10_11_12_x());\
    m_C.M_20_21_22_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(data.X1(i).XYZx(), _mm_set1_ps(data.X2(i).Z())), m_C.M_20_21_22_x())
#endif

    ACCUMULATE_COVARIANCE_6(0);
    ACCUMULATE_COVARIANCE_6(1);
    ACCUMULATE_COVARIANCE_6(2);
    ACCUMULATE_COVARIANCE_6(3);
    ACCUMULATE_COVARIANCE_6(4);
    ACCUMULATE_COVARIANCE_6(5);
}

void RotationTransformationSolver::ComputeCovariance(const
        AlignedVector<Point3D> &X1s, const AlignedVector<Point3D> &X2s) {
    m_C.SetZero();
    const ushort N = ushort(X1s.Size());
    for(ushort i = 0; i < N; ++i) {
        const Point3D &X1 = X1s[i], &X2 = X2s[i];
        m_C.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(X1.XYZx(), _mm_set1_ps(X2.X())),
                                        m_C.M_00_01_02_x());
        m_C.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(X1.XYZx(), _mm_set1_ps(X2.Y())),
                                        m_C.M_10_11_12_x());
        m_C.M_20_21_22_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(X1.XYZx(), _mm_set1_ps(X2.Z())),
                                        m_C.M_20_21_22_x());
    }
}

bool RotationTransformationSolver::RecoverRotation(RotationTransformation3D &R,
        AlignedVector<ENFT_SSE::__m128> &work) {
    char jobu = 'A', jobvt = 'A';
    integer m = 3, n = 3, lda = 4, ldu = 4, ldvt = 4, lwork = 15, info;
    float s[3];

    // C = U * S * V^T
    // LAPACK treats matrix as column-major, i.e., C' = C^T.
    // C' = U' * S' * V'^T
    // Since C^T = V * S * U^T, LAPACK returned U' and V'^T correspond to V and U^T respectively.
    // So here just pass U and V^T to LAPACK reversely.
    work.Resize(4);
    sgesvd_(&jobu, &jobvt, &m, &n, m_C, &lda, s, m_VT, &ldvt, m_U, &ldu,
            (float *) work.Data(), &lwork, &info);

    if(info != 0)
        return false;

    LA::AlignedMatrix3f &V = m_VT;
    V.Transpose();
    R.r00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_U.M_00_01_02_x(), V.M_00_01_02_x()));
    R.r01() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_U.M_00_01_02_x(), V.M_10_11_12_x()));
    R.r02() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_U.M_00_01_02_x(), V.M_20_21_22_x()));
    R.r10() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_U.M_10_11_12_x(), V.M_00_01_02_x()));
    R.r11() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_U.M_10_11_12_x(), V.M_10_11_12_x()));
    R.r12() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_U.M_10_11_12_x(), V.M_20_21_22_x()));
    R.r20() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_U.M_20_21_22_x(), V.M_00_01_02_x()));
    R.r21() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_U.M_20_21_22_x(), V.M_10_11_12_x()));
    R.r22() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_U.M_20_21_22_x(), V.M_20_21_22_x()));
    const float det = R.Determinant();
    if(det < 0) {
        R.r20() = -R.r20();
        R.r21() = -R.r21();
        R.r22() = -R.r22();
    }
    return true;
}