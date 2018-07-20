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
#include "CameraEPnP.h"
#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}

bool CameraEPnP::Run(const SixMatches3DTo2D &data, Camera &C,
                     AlignedVector<ENFT_SSE::__m128> &work) {
    if(!ComputeCws(data, work) || !ComputeAlphas(data, work))
        return false;
    ConstructLinearSystem(data, work);
    if(!ComputeCcs(work) || !ComputeXcs())
        return false;
    m_data3D.Set(0, data.X(0), m_Xcs[0]);
    m_data3D.Set(1, data.X(1), m_Xcs[1]);
    m_data3D.Set(2, data.X(2), m_Xcs[2]);
    m_data3D.Set(3, data.X(3), m_Xcs[3]);
    m_data3D.Set(4, data.X(4), m_Xcs[4]);
    m_data3D.Set(5, data.X(5), m_Xcs[5]);
    return m_Tsolver.Run(m_data3D, C, work);
}

bool CameraEPnP::Run(const CameraEstimatorData &data, Camera &C,
                     AlignedVector<ENFT_SSE::__m128> &work) {
    if(!ComputeCws(data, work) || !ComputeAlphas(data, work))
        return false;
    ConstructLinearSystem(data, work);
    if(!ComputeCcs(work) || !ComputeXcs())
        return false;
    return m_Tsolver.Run(data.Xs(), m_Xcs, C, work);
}

bool CameraEPnP::ComputeCws(const SixMatches3DTo2D &data,
                            AlignedVector<ENFT_SSE::__m128> &work) {
    Point3D &Cw0 = m_Cws[0];
    Cw0.XYZx() = ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(data.X(0).XYZx(),
                                       data.X(1).XYZx()),
                                       ENFT_SSE::_mm_add_ps(_mm_add_ps(data.X(2).XYZx(), data.X(3).XYZx()),
                                               ENFT_SSE::_mm_add_ps(data.X(4).XYZx(), data.X(5).XYZx()))), _mm_set1_ps(1.0f / 6));

    Point3D &dXw = m_Cws[1];

#ifndef SET_AT_3x6
#define SET_AT_3x6(i)\
    dXw.XYZx() = ENFT_SSE::_mm_sub_ps(data.X(i).XYZx(), Cw0.XYZx());\
    m_AT3x6[0][i] = dXw.X();\
    m_AT3x6[1][i] = dXw.Y();\
    m_AT3x6[2][i] = dXw.Z()
#endif

    SET_AT_3x6(0);
    SET_AT_3x6(1);
    SET_AT_3x6(2);
    SET_AT_3x6(3);
    SET_AT_3x6(4);
    SET_AT_3x6(5);

    LA::AAT(m_AT3x6, m_ATA);

    char jobz = 'V', uplo = 'L';
    integer n = 3, lda = 4, m = 3, lwork = 9, info;
    work.Resize(3);
    ssyev_(&jobz, &uplo, &n, m_ATA, &lda, m_w.Data(), (float *) work.Data(), &lwork,
           &info);
    if(info != 0)
        return false;
    memcpy(m_pds[0], m_ATA, 48);
    //char jobz = 'V', range = 'A', uplo = 'L';
    //integer n = 3, lda = 4, m = 3, ldz = 4/*, isuppz[6]*/, lwork = 78, liwork = 30, info;
    //float abstol = 0;
    //float *z = m_pds[0];
    //work.Resize(29);
    //integer *isuppz = (integer *) work.Data();
    //float *work1 = (float *) (isuppz + 6);
    //integer *iwork = (integer *) (work1 + lwork);
    //ssyevr_(&jobz, &range, &uplo, &n, (float *) m_ATA, &lda, NULL, NULL, NULL, NULL, &abstol, &m, m_w, z, &ldz, isuppz, work1, &lwork, iwork, &liwork, &info);
    //if(info != 0)
    //  return false;

    ENFT_SSE::__m128 &ks = work[0], &k = work[1];
    ks = _mm_sqrt_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(m_w.Data()), _mm_set1_ps(1.0f / 6)));

    k = _mm_set1_ps(ks.m128_f32[0]);
    k.m128_f32[3] = 0;
    m_pds[0] *= k;
    m_Cws[1].XYZx() = ENFT_SSE::_mm_add_ps(Cw0.XYZx(), m_pds[0].XYZx());

    k = _mm_set1_ps(ks.m128_f32[1]);
    k.m128_f32[3] = 0;
    m_pds[1] *= k;
    m_Cws[2].XYZx() = ENFT_SSE::_mm_add_ps(Cw0.XYZx(), m_pds[1].XYZx());

    k = _mm_set1_ps(ks.m128_f32[2]);
    k.m128_f32[3] = 0;
    m_pds[2] *= k;
    m_Cws[3].XYZx() = ENFT_SSE::_mm_add_ps(Cw0.XYZx(), m_pds[2].XYZx());

    return true;
}

bool CameraEPnP::ComputeCws(const CameraEstimatorData &data,
                            AlignedVector<ENFT_SSE::__m128> &work) {
    work.Resize(3);
    Point3D &Cw0 = m_Cws[0];
    Cw0.SetZero();
    const ushort N = data.Size();
    for(ushort i = 0; i < N; ++i)
        Cw0.XYZx() = ENFT_SSE::_mm_add_ps(data.X(i).XYZx(), Cw0.XYZx());
    Cw0 *= _mm_set1_ps(1.0f / N);

    Point3D &dXw = m_Cws[1];
    m_AT3xX.Resize(N);
    for(ushort i = 0; i < N; ++i) {
        dXw.XYZx() = ENFT_SSE::_mm_sub_ps(data.X(i).XYZx(), Cw0.XYZx());
        m_AT3xX[0][i] = dXw.X();
        m_AT3xX[1][i] = dXw.Y();
        m_AT3xX[2][i] = dXw.Z();
    }

    LA::AAT(m_AT3xX, m_ATA);

    char jobz = 'V', uplo = 'L';
    integer n = 3, lda = 4, m = 3, lwork = 9, info;
    work.Resize(3);
    ssyev_(&jobz, &uplo, &n, m_ATA, &lda, m_w.Data(), (float *) work.Data(), &lwork,
           &info);
    if(info != 0)
        return false;

    memcpy(m_pds[0], m_ATA, 48);

    ENFT_SSE::__m128 &ks = work[0], &k = work[1];
    ks = _mm_sqrt_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(m_w.Data()), _mm_set1_ps(1.0f / N)));

    k = _mm_set1_ps(ks.m128_f32[0]);
    k.m128_f32[3] = 0;
    m_pds[0] *= k;
    m_Cws[1].XYZx() = ENFT_SSE::_mm_add_ps(Cw0.XYZx(), m_pds[0].XYZx());

    k = _mm_set1_ps(ks.m128_f32[1]);
    k.m128_f32[3] = 0;
    m_pds[1] *= k;
    m_Cws[2].XYZx() = ENFT_SSE::_mm_add_ps(Cw0.XYZx(), m_pds[1].XYZx());

    k = _mm_set1_ps(ks.m128_f32[2]);
    k.m128_f32[3] = 0;
    m_pds[2] *= k;
    m_Cws[3].XYZx() = ENFT_SSE::_mm_add_ps(Cw0.XYZx(), m_pds[2].XYZx());

    return true;
}

bool CameraEPnP::ComputeAlphas(const SixMatches3DTo2D &data,
                               AlignedVector<ENFT_SSE::__m128> &work) {
    // Inverse [ pds[1]^T, pds[2]^T, pds[3]^T ]
    work.Resize(4);
    ENFT_SSE::__m128 &T0 = work[0], &T1 = work[1], &T2 = work[2];
    T0.m128_f32[0] = m_pds[1].Y() * m_pds[2].Z() - m_pds[2].Y() * m_pds[1].Z();
    T0.m128_f32[1] = m_pds[2].X() * m_pds[1].Z() - m_pds[1].X() * m_pds[2].Z();
    T0.m128_f32[2] = m_pds[1].X() * m_pds[2].Y() - m_pds[2].X() * m_pds[1].Y();
    T1.m128_f32[0] = m_pds[2].Y() * m_pds[0].Z() - m_pds[0].Y() * m_pds[2].Z();
    T1.m128_f32[1] = m_pds[0].X() * m_pds[2].Z() - m_pds[2].X() * m_pds[0].Z();
    T1.m128_f32[2] = m_pds[2].X() * m_pds[0].Y() - m_pds[0].X() * m_pds[2].Y();
    T2.m128_f32[0] = m_pds[0].Y() * m_pds[1].Z() - m_pds[1].Y() * m_pds[0].Z();
    T2.m128_f32[1] = m_pds[1].X() * m_pds[0].Z() - m_pds[0].X() * m_pds[1].Z();
    T2.m128_f32[2] = m_pds[0].X() * m_pds[1].Y() - m_pds[1].X() * m_pds[0].Y();
    const float det = m_pds[0].X() * T0.m128_f32[0] + m_pds[1].X() * T1.m128_f32[0]+
                      m_pds[2].X() * T2.m128_f32[0];
    //if(fabs(det) < FLT_EPSILON)
    if(det == 0.0f)
        return false;

    ENFT_SSE::__m128 &one_over_det = work[3];
    one_over_det = _mm_set1_ps(1 / det);
    T0 = ENFT_SSE::_mm_mul_ps(T0, one_over_det);
    T1 = ENFT_SSE::_mm_mul_ps(T1, one_over_det);
    T2 = ENFT_SSE::_mm_mul_ps(T2, one_over_det);

    m_alphas.Resize(6);
    ENFT_SSE::__m128 &dXw = work[3], &Cw0 = m_Cws[0].XYZx();

#ifndef SET_ALPHA_6
#define SET_ALPHA_6(i)\
    dXw = ENFT_SSE::_mm_sub_ps(data.X(i).XYZx(), Cw0);\
    m_alphas[i].v1() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T0, dXw));\
    m_alphas[i].v2() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T1, dXw));\
    m_alphas[i].v3() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T2, dXw));\
    m_alphas[i].v0() = 1 - m_alphas[i].v1() - m_alphas[i].v2() - m_alphas[i].v3()
#endif

    SET_ALPHA_6(0);
    SET_ALPHA_6(1);
    SET_ALPHA_6(2);
    SET_ALPHA_6(3);
    SET_ALPHA_6(4);
    SET_ALPHA_6(5);

    return true;
}

bool CameraEPnP::ComputeAlphas(const CameraEstimatorData &data,
                               AlignedVector<ENFT_SSE::__m128> &work) {
    // Inverse [ pds[1]^T, pds[2]^T, pds[3]^T ]
    work.Resize(4);
    ENFT_SSE::__m128 &T0 = work[0], &T1 = work[1], &T2 = work[2];
    T0.m128_f32[0] = m_pds[1].Y() * m_pds[2].Z() - m_pds[2].Y() * m_pds[1].Z();
    T0.m128_f32[1] = m_pds[2].X() * m_pds[1].Z() - m_pds[1].X() * m_pds[2].Z();
    T0.m128_f32[2] = m_pds[1].X() * m_pds[2].Y() - m_pds[2].X() * m_pds[1].Y();
    T1.m128_f32[0] = m_pds[2].Y() * m_pds[0].Z() - m_pds[0].Y() * m_pds[2].Z();
    T1.m128_f32[1] = m_pds[0].X() * m_pds[2].Z() - m_pds[2].X() * m_pds[0].Z();
    T1.m128_f32[2] = m_pds[2].X() * m_pds[0].Y() - m_pds[0].X() * m_pds[2].Y();
    T2.m128_f32[0] = m_pds[0].Y() * m_pds[1].Z() - m_pds[1].Y() * m_pds[0].Z();
    T2.m128_f32[1] = m_pds[1].X() * m_pds[0].Z() - m_pds[0].X() * m_pds[1].Z();
    T2.m128_f32[2] = m_pds[0].X() * m_pds[1].Y() - m_pds[1].X() * m_pds[0].Y();
    const float det = m_pds[0].X() * T0.m128_f32[0] + m_pds[1].X() * T1.m128_f32[0]+
                      m_pds[2].X() * T2.m128_f32[0];
    //if(fabs(det) < FLT_EPSILON)
    if(det == 0.0f)
        return false;

    ENFT_SSE::__m128 &one_over_det = work[3];
    one_over_det = _mm_set1_ps(1 / det);
    T0 = ENFT_SSE::_mm_mul_ps(T0, one_over_det);
    T1 = ENFT_SSE::_mm_mul_ps(T1, one_over_det);
    T2 = ENFT_SSE::_mm_mul_ps(T2, one_over_det);

    const ushort N = data.Size();
    m_alphas.Resize(N);
    ENFT_SSE::__m128 &dXw = work[3], Cw0 = m_Cws[0].XYZx();
    for(ushort i = 0; i < N; ++i) {
        LA::AlignedVector4f &alpha = m_alphas[i];
        dXw = ENFT_SSE::_mm_sub_ps(data.X(i).XYZx(), Cw0);
        alpha.v1() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T0, dXw));
        alpha.v2() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T1, dXw));
        alpha.v3() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T2, dXw));
        alpha.v0() = 1 - alpha.v1() - alpha.v2() - alpha.v3();
    }

    return true;
}

inline static void DOT_AB_6(const LA::AlignedVectorXf &A,
                            const LA::AlignedVectorXf &B, float &dot, ENFT_SSE::__m128 &tmp) {
    tmp = ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A.Data()), PTR_TO_SSE(B.Data()));
    dot = tmp.m128_f32[0] + tmp.m128_f32[1] + tmp.m128_f32[2] + tmp.m128_f32[3] +
          A[4] * B[4] + A[5] * B[5];
}

inline static void DOT_AB_12(const LA::AlignedVectorXf &A,
                             const LA::AlignedVectorXf &B, float &dot, ENFT_SSE::__m128 &tmp) {
    tmp = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A.Data()  ), PTR_TO_SSE(B.Data()  )),
                     ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A.Data()+4), PTR_TO_SSE(B.Data()+4)),
                                ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A.Data()+8), PTR_TO_SSE(B.Data()+8))));
    dot = ENFT_SSE::SSE::Sum0123(tmp);
}

inline static void DOT_AB_MULTIPLE_4(const LA::AlignedVectorXf &A,
                                     const LA::AlignedVectorXf &B, const ushort N, float &dot, ENFT_SSE::__m128 &tmp) {
    const ENFT_SSE::__m128 *pA = (const ENFT_SSE::__m128 *) A.Data(), *pB = (const ENFT_SSE::__m128 *) B.Data();
    tmp = ENFT_SSE::_mm_setzero_ps();
    for(ushort k = 0; k < N; k += 4, ++pA, ++pB)
        tmp = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(*pA, *pB), tmp);
    dot = ENFT_SSE::SSE::Sum0123(tmp);
}

inline static void DOT_AB(const LA::AlignedVectorXf &A,
                          const LA::AlignedVectorXf &B, const ushort &_N, const ushort &N, float &dot,
                          ENFT_SSE::__m128 &tmp) {
    const ENFT_SSE::__m128 *pA = (const ENFT_SSE::__m128 *) A.Data(), *pB = (const ENFT_SSE::__m128 *) B.Data();
    tmp = ENFT_SSE::_mm_setzero_ps();
    for(ushort k = 0; k < _N; k += 4, ++pA, ++pB)
        tmp = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(*pA, *pB), tmp);
    dot = ENFT_SSE::SSE::Sum0123(tmp);
    for(uint k = _N; k < N; k++, pA++, pB++)
        dot += A[k] * B[k];
}

void CameraEPnP::ConstructLinearSystem(const SixMatches3DTo2D &data,
                                       AlignedVector<ENFT_SSE::__m128> &work) {
    // col        = [  0,  1,     2,  3,  4,     5,  6,  7,     8,  9, 10,    11 ]
    // M[row    ] = [ a0,  0, -a0*u, a1,  0, -a1*u, a2,  0, -a2*u, a3,  0, -a3*u ]
    // M[row + 1] = [  0, a0, -a0*v,  0, a1, -a1*v,  0, a2, -a2*v,  0, a3, -a3*v ]

    if(m_Mcol0.Size() != 6) {
        m_Mcol0.Resize(6);
        m_Mcol1.Resize(6);
        m_Mcol2.Resize(12);
        m_Mcol2_0.Resize(6);
        m_Mcol2_1.Resize(6);
        m_Mcol3.Resize(6);
        m_Mcol4.Resize(6);
        m_Mcol5.Resize(12);
        m_Mcol5_0.Resize(6);
        m_Mcol5_1.Resize(6);
        m_Mcol6.Resize(6);
        m_Mcol7.Resize(6);
        m_Mcol8.Resize(12);
        m_Mcol8_0.Resize(6);
        m_Mcol8_1.Resize(6);
        m_Mcol9.Resize(6);
        m_Mcol10.Resize(6);
        m_Mcol11.Resize(12);
        m_Mcol11_0.Resize(6);
        m_Mcol11_1.Resize(6);
    }

    //m_MTM.SetZero();

    work.Resize(2);
    ENFT_SSE::__m128 &nau = work[0], &nav = work[1];

#ifndef SET_M_12x12
#define SET_M_12x12(i, ix2, ix2p1)\
    ENFT_SSE::__m128 &a##i = m_alphas[i].v0123();\
    nau = ENFT_SSE::_mm_mul_ps(a##i, _mm_set1_ps(-data.x(i).x()));\
    nav = ENFT_SSE::_mm_mul_ps(a##i, _mm_set1_ps(-data.x(i).y()));\
    m_Mcol0[i]=a##i.m128_f32[0]; m_Mcol3[i]=a##i.m128_f32[1]; m_Mcol6[i]=a##i.m128_f32[2]; m_Mcol9[i]=a##i.m128_f32[3];\
    m_Mcol1[i]=a##i.m128_f32[0]; m_Mcol4[i]=a##i.m128_f32[1]; m_Mcol7[i]=a##i.m128_f32[2]; m_Mcol10[i]=a##i.m128_f32[3];\
    m_Mcol2[ix2]=m_Mcol2_0[i]=nau.m128_f32[0];  m_Mcol5[ix2]=m_Mcol5_0[i]=nau.m128_f32[1];\
    m_Mcol8[ix2]=m_Mcol8_0[i]=nau.m128_f32[2];  m_Mcol11[ix2]=m_Mcol11_0[i]=nau.m128_f32[3];\
    m_Mcol2[ix2p1]=m_Mcol2_1[i]=nav.m128_f32[0];    m_Mcol5[ix2p1]=m_Mcol5_1[i]=nav.m128_f32[1];\
    m_Mcol8[ix2p1]=m_Mcol8_1[i]=nav.m128_f32[2];    m_Mcol11[ix2p1]=m_Mcol11_1[i]=nav.m128_f32[3]
#endif

    SET_M_12x12(0, 0, 1);
    SET_M_12x12(1, 2, 3);
    SET_M_12x12(2, 4, 5);
    SET_M_12x12(3, 6, 7);
    SET_M_12x12(4, 8, 9);
    SET_M_12x12(5, 10, 11);

    ENFT_SSE::__m128 &tmp = work[0];
    //double *pMTM = m_MTM[0];
    float *pMTM = m_MTM[0];
    DOT_AB_6(m_Mcol0, m_Mcol0, *pMTM++, tmp);
    *pMTM++=0;
    DOT_AB_6(m_Mcol0, m_Mcol2_0, *pMTM++, tmp);
    DOT_AB_6(m_Mcol0, m_Mcol3, *pMTM++, tmp);
    *pMTM++=0;
    DOT_AB_6(m_Mcol0, m_Mcol5_0, *pMTM++, tmp);
    DOT_AB_6(m_Mcol0, m_Mcol6, *pMTM++, tmp);
    *pMTM++=0;
    DOT_AB_6(m_Mcol0, m_Mcol8_0, *pMTM++, tmp);
    DOT_AB_6(m_Mcol0, m_Mcol9, *pMTM++, tmp);
    *pMTM++=0;
    DOT_AB_6(m_Mcol0, m_Mcol11_0, *pMTM++, tmp);
    pMTM++;
    *pMTM++=m_MTM[0][0];
    DOT_AB_6(m_Mcol1, m_Mcol2_1, *pMTM++, tmp);
    *pMTM++=0;
    *pMTM++=m_MTM[0][3];
    DOT_AB_6(m_Mcol1, m_Mcol5_1, *pMTM++, tmp);
    *pMTM++=0;
    *pMTM++=m_MTM[0][6];
    DOT_AB_6(m_Mcol1, m_Mcol8_1, *pMTM++, tmp);
    *pMTM++=0;
    *pMTM++=m_MTM[0][9];
    DOT_AB_6(m_Mcol1, m_Mcol11_1, *pMTM++, tmp);
    pMTM+=2;
    DOT_AB_12(m_Mcol2, m_Mcol2, *pMTM++, tmp);
    DOT_AB_6(m_Mcol2_0, m_Mcol3, *pMTM++, tmp);
    DOT_AB_6(m_Mcol2_1, m_Mcol4, *pMTM++, tmp);
    DOT_AB_12(m_Mcol2, m_Mcol5, *pMTM++, tmp);
    DOT_AB_6(m_Mcol2_0, m_Mcol6, *pMTM++, tmp);
    DOT_AB_6(m_Mcol2_1, m_Mcol7, *pMTM++, tmp);
    DOT_AB_12(m_Mcol2, m_Mcol8, *pMTM++, tmp);
    DOT_AB_6(m_Mcol2_0, m_Mcol9, *pMTM++, tmp);
    DOT_AB_6(m_Mcol2_1, m_Mcol10, *pMTM++, tmp);
    DOT_AB_12(m_Mcol2, m_Mcol11, *pMTM++, tmp);

    pMTM+=3;
    DOT_AB_6(m_Mcol3, m_Mcol3, *pMTM++, tmp);
    *pMTM++=0;
    DOT_AB_6(m_Mcol3, m_Mcol5_0, *pMTM++, tmp);
    DOT_AB_6(m_Mcol3, m_Mcol6, *pMTM++, tmp);
    *pMTM++=0;
    DOT_AB_6(m_Mcol3, m_Mcol8_0, *pMTM++, tmp);
    DOT_AB_6(m_Mcol3, m_Mcol9, *pMTM++, tmp);
    *pMTM++=0;
    DOT_AB_6(m_Mcol3, m_Mcol11_0, *pMTM++, tmp);
    pMTM+=4;
    *pMTM++=m_MTM[3][3];
    DOT_AB_6(m_Mcol4, m_Mcol5_1, *pMTM++, tmp);
    *pMTM++=0;
    *pMTM++=m_MTM[3][6];
    DOT_AB_6(m_Mcol4, m_Mcol8_1, *pMTM++, tmp);
    *pMTM++=0;
    *pMTM++=m_MTM[3][9];
    DOT_AB_6(m_Mcol4, m_Mcol11_1, *pMTM++, tmp);
    pMTM+=5;
    DOT_AB_12(m_Mcol5, m_Mcol5, *pMTM++, tmp);
    DOT_AB_6(m_Mcol5_0, m_Mcol6, *pMTM++, tmp);
    DOT_AB_6(m_Mcol5_1, m_Mcol7, *pMTM++, tmp);
    DOT_AB_12(m_Mcol5, m_Mcol8, *pMTM++, tmp);
    DOT_AB_6(m_Mcol5_0, m_Mcol9, *pMTM++, tmp);
    DOT_AB_6(m_Mcol5_1, m_Mcol10, *pMTM++, tmp);
    DOT_AB_12(m_Mcol5, m_Mcol11, *pMTM++, tmp);

    pMTM+=6;
    DOT_AB_6(m_Mcol6, m_Mcol6, *pMTM++, tmp);
    *pMTM++=0;
    DOT_AB_6(m_Mcol6, m_Mcol8_0, *pMTM++, tmp);
    DOT_AB_6(m_Mcol6, m_Mcol9, *pMTM++, tmp);
    *pMTM++=0;
    DOT_AB_6(m_Mcol6, m_Mcol11_0, *pMTM++, tmp);
    pMTM+=7;
    *pMTM++=m_MTM[6][6];
    DOT_AB_6(m_Mcol7, m_Mcol8_1, *pMTM++, tmp);
    *pMTM++=0;
    *pMTM++=m_MTM[6][9];
    DOT_AB_6(m_Mcol7, m_Mcol11_1, *pMTM++, tmp);
    pMTM+=8;
    DOT_AB_12(m_Mcol8, m_Mcol8, *pMTM++, tmp);
    DOT_AB_6(m_Mcol8_0, m_Mcol9, *pMTM++, tmp);
    DOT_AB_6(m_Mcol8_1, m_Mcol10, *pMTM++, tmp);
    DOT_AB_12(m_Mcol8, m_Mcol11, *pMTM++, tmp);

    pMTM+=9;
    DOT_AB_6(m_Mcol9, m_Mcol9, *pMTM++, tmp);
    *pMTM++=0;
    DOT_AB_6(m_Mcol9, m_Mcol11_0, *pMTM++, tmp);
    pMTM+=10;
    *pMTM++=m_MTM[9][9];
    DOT_AB_6(m_Mcol10, m_Mcol11_1, *pMTM++, tmp);
    pMTM+=11;
    DOT_AB_12(m_Mcol11, m_Mcol11, *pMTM++, tmp);

    //m_MTM.Print();
}

void CameraEPnP::ConstructLinearSystem(const CameraEstimatorData &data,
                                       AlignedVector<ENFT_SSE::__m128> &work) {
    // col        = [  0,  1,     2,  3,  4,     5,  6,  7,     8,  9, 10,    11 ]
    // M[row    ] = [ a0,  0, -a0*u, a1,  0, -a1*u, a2,  0, -a2*u, a3,  0, -a3*u ]
    // M[row + 1] = [  0, a0, -a0*v,  0, a1, -a1*v,  0, a2, -a2*v,  0, a3, -a3*v ]

    const ushort N = data.Size(), Nx2 = (N << 1);
    if(m_Mcol0.Size() != N) {
        m_Mcol0.Resize(N);
        m_Mcol1.Resize(N);
        m_Mcol2.Resize(Nx2);
        m_Mcol2_0.Resize(N);
        m_Mcol2_1.Resize(N);
        m_Mcol3.Resize(N);
        m_Mcol4.Resize(N);
        m_Mcol5.Resize(Nx2);
        m_Mcol5_0.Resize(N);
        m_Mcol5_1.Resize(N);
        m_Mcol6.Resize(N);
        m_Mcol7.Resize(N);
        m_Mcol8.Resize(Nx2);
        m_Mcol8_0.Resize(N);
        m_Mcol8_1.Resize(N);
        m_Mcol9.Resize(N);
        m_Mcol10.Resize(N);
        m_Mcol11.Resize(Nx2);
        m_Mcol11_0.Resize(N);
        m_Mcol11_1.Resize(N);
    }

    work.Resize(2);
    ENFT_SSE::__m128 &nau = work[0], &nav = work[1];
    for(ushort i = 0, ix2 = 0, ix2p1 = 1; i < N;
            ++i, ix2 = (i << 1), ix2p1 = ix2 + 1) {
        const ENFT_SSE::__m128 &a = m_alphas[i].v0123();
        nau = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(-data.x(i).x()));
        nav = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(-data.x(i).y()));
        m_Mcol0[i]=a.m128_f32[0];
        m_Mcol3[i]=a.m128_f32[1];
        m_Mcol6[i]=a.m128_f32[2];
        m_Mcol9[i]=a.m128_f32[3];
        m_Mcol1[i]=a.m128_f32[0];
        m_Mcol4[i]=a.m128_f32[1];
        m_Mcol7[i]=a.m128_f32[2];
        m_Mcol10[i]=a.m128_f32[3];
        m_Mcol2[ix2]=m_Mcol2_0[i]=nau.m128_f32[0];
        m_Mcol5[ix2]=m_Mcol5_0[i]=nau.m128_f32[1];
        m_Mcol8[ix2]=m_Mcol8_0[i]=nau.m128_f32[2];
        m_Mcol11[ix2]=m_Mcol11_0[i]=nau.m128_f32[3];
        m_Mcol2[ix2p1]=m_Mcol2_1[i]=nav.m128_f32[0];
        m_Mcol5[ix2p1]=m_Mcol5_1[i]=nav.m128_f32[1];
        m_Mcol8[ix2p1]=m_Mcol8_1[i]=nav.m128_f32[2];
        m_Mcol11[ix2p1]=m_Mcol11_1[i]=nav.m128_f32[3];
    }

    ENFT_SSE::__m128 &tmp = work[0];
    //double *pMTM;
    float *pMTM;
    ushort rem1 = N & 3, _N = N - rem1, _Nx2 = Nx2 - (Nx2 & 3);
    switch(rem1) {
        case 0:
            pMTM = m_MTM[0];
            DOT_AB_MULTIPLE_4(m_Mcol0, m_Mcol0, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB_MULTIPLE_4(m_Mcol0, m_Mcol2_0, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol0, m_Mcol3, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB_MULTIPLE_4(m_Mcol0, m_Mcol5_0, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol0, m_Mcol6, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB_MULTIPLE_4(m_Mcol0, m_Mcol8_0, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol0, m_Mcol9, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB_MULTIPLE_4(m_Mcol0, m_Mcol11_0, N, *pMTM++, tmp);
            pMTM++;
            *pMTM++=m_MTM[0][0];
            DOT_AB_MULTIPLE_4(m_Mcol1, m_Mcol2_1, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[0][3];
            DOT_AB_MULTIPLE_4(m_Mcol1, m_Mcol5_1, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[0][6];
            DOT_AB_MULTIPLE_4(m_Mcol1, m_Mcol8_1, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[0][9];
            DOT_AB_MULTIPLE_4(m_Mcol1, m_Mcol11_1, N, *pMTM++, tmp);
            pMTM+=2;
            DOT_AB_MULTIPLE_4(m_Mcol2, m_Mcol2, Nx2, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2_0, m_Mcol3, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2_1, m_Mcol4, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2, m_Mcol5, Nx2, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2_0, m_Mcol6, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2_1, m_Mcol7, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2, m_Mcol8, Nx2, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2_0, m_Mcol9, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2_1, m_Mcol10, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2, m_Mcol11, Nx2, *pMTM++, tmp);

            pMTM+=3;
            DOT_AB_MULTIPLE_4(m_Mcol3, m_Mcol3, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB_MULTIPLE_4(m_Mcol3, m_Mcol5_0, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol3, m_Mcol6, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB_MULTIPLE_4(m_Mcol3, m_Mcol8_0, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol3, m_Mcol9, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB_MULTIPLE_4(m_Mcol3, m_Mcol11_0, N, *pMTM++, tmp);
            pMTM+=4;
            *pMTM++=m_MTM[3][3];
            DOT_AB_MULTIPLE_4(m_Mcol4, m_Mcol5_1, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[3][6];
            DOT_AB_MULTIPLE_4(m_Mcol4, m_Mcol8_1, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[3][9];
            DOT_AB_MULTIPLE_4(m_Mcol4, m_Mcol11_1, N, *pMTM++, tmp);
            pMTM+=5;
            DOT_AB_MULTIPLE_4(m_Mcol5, m_Mcol5, Nx2, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol5_0, m_Mcol6, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol5_1, m_Mcol7, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol5, m_Mcol8, Nx2, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol5_0, m_Mcol9, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol5_1, m_Mcol10, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol5, m_Mcol11, Nx2, *pMTM++, tmp);

            pMTM+=6;
            DOT_AB_MULTIPLE_4(m_Mcol6, m_Mcol6, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB_MULTIPLE_4(m_Mcol6, m_Mcol8_0, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol6, m_Mcol9, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB_MULTIPLE_4(m_Mcol6, m_Mcol11_0, N, *pMTM++, tmp);
            pMTM+=7;
            *pMTM++=m_MTM[6][6];
            DOT_AB_MULTIPLE_4(m_Mcol7, m_Mcol8_1, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[6][9];
            DOT_AB_MULTIPLE_4(m_Mcol7, m_Mcol11_1, N, *pMTM++, tmp);
            pMTM+=8;
            DOT_AB_MULTIPLE_4(m_Mcol8, m_Mcol8, Nx2, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol8_0, m_Mcol9, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol8_1, m_Mcol10, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol8, m_Mcol11, Nx2, *pMTM++, tmp);

            pMTM+=9;
            DOT_AB_MULTIPLE_4(m_Mcol9, m_Mcol9, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB_MULTIPLE_4(m_Mcol9, m_Mcol11_0, N, *pMTM++, tmp);
            pMTM+=10;
            *pMTM++=m_MTM[9][9];
            DOT_AB_MULTIPLE_4(m_Mcol10, m_Mcol11_1, N, *pMTM++, tmp);
            pMTM+=11;
            DOT_AB_MULTIPLE_4(m_Mcol11, m_Mcol11, Nx2, *pMTM++, tmp);
            break;
        case 1:
        case 3:
            pMTM = m_MTM[0];
            DOT_AB(m_Mcol0, m_Mcol0, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol0, m_Mcol2_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol0, m_Mcol3, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol0, m_Mcol5_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol0, m_Mcol6, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol0, m_Mcol8_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol0, m_Mcol9, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol0, m_Mcol11_0, _N, N, *pMTM++, tmp);
            pMTM++;
            *pMTM++=m_MTM[0][0];
            DOT_AB(m_Mcol1, m_Mcol2_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[0][3];
            DOT_AB(m_Mcol1, m_Mcol5_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[0][6];
            DOT_AB(m_Mcol1, m_Mcol8_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[0][9];
            DOT_AB(m_Mcol1, m_Mcol11_1, _N, N, *pMTM++, tmp);
            pMTM+=2;
            DOT_AB(m_Mcol2, m_Mcol2, _Nx2, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol2_0, m_Mcol3, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol2_1, m_Mcol4, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol2, m_Mcol5, _Nx2, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol2_0, m_Mcol6, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol2_1, m_Mcol7, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol2, m_Mcol8, _Nx2, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol2_0, m_Mcol9, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol2_1, m_Mcol10, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol2, m_Mcol11, _Nx2, Nx2, *pMTM++, tmp);

            pMTM+=3;
            DOT_AB(m_Mcol3, m_Mcol3, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol3, m_Mcol5_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol3, m_Mcol6, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol3, m_Mcol8_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol3, m_Mcol9, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol3, m_Mcol11_0, _N, N, *pMTM++, tmp);
            pMTM+=4;
            *pMTM++=m_MTM[3][3];
            DOT_AB(m_Mcol4, m_Mcol5_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[3][6];
            DOT_AB(m_Mcol4, m_Mcol8_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[3][9];
            DOT_AB(m_Mcol4, m_Mcol11_1, _N, N, *pMTM++, tmp);
            pMTM+=5;
            DOT_AB(m_Mcol5, m_Mcol5, _Nx2, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol5_0, m_Mcol6, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol5_1, m_Mcol7, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol5, m_Mcol8, _Nx2, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol5_0, m_Mcol9, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol5_1, m_Mcol10, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol5, m_Mcol11, _Nx2, Nx2, *pMTM++, tmp);

            pMTM+=6;
            DOT_AB(m_Mcol6, m_Mcol6, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol6, m_Mcol8_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol6, m_Mcol9, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol6, m_Mcol11_0, _N, N, *pMTM++, tmp);
            pMTM+=7;
            *pMTM++=m_MTM[6][6];
            DOT_AB(m_Mcol7, m_Mcol8_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[6][9];
            DOT_AB(m_Mcol7, m_Mcol11_1, _N, N, *pMTM++, tmp);
            pMTM+=8;
            DOT_AB(m_Mcol8, m_Mcol8, _Nx2, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol8_0, m_Mcol9, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol8_1, m_Mcol10, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol8, m_Mcol11, _Nx2, Nx2, *pMTM++, tmp);

            pMTM+=9;
            DOT_AB(m_Mcol9, m_Mcol9, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol9, m_Mcol11_0, _N, N, *pMTM++, tmp);
            pMTM+=10;
            *pMTM++=m_MTM[9][9];
            DOT_AB(m_Mcol10, m_Mcol11_1, _N, N, *pMTM++, tmp);
            pMTM+=11;
            DOT_AB(m_Mcol11, m_Mcol11, _Nx2, Nx2, *pMTM++, tmp);
            break;
        case 2:
            pMTM = m_MTM[0];
            DOT_AB(m_Mcol0, m_Mcol0, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol0, m_Mcol2_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol0, m_Mcol3, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol0, m_Mcol5_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol0, m_Mcol6, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol0, m_Mcol8_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol0, m_Mcol9, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol0, m_Mcol11_0, _N, N, *pMTM++, tmp);
            pMTM++;
            *pMTM++=m_MTM[0][0];
            DOT_AB(m_Mcol1, m_Mcol2_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[0][3];
            DOT_AB(m_Mcol1, m_Mcol5_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[0][6];
            DOT_AB(m_Mcol1, m_Mcol8_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[0][9];
            DOT_AB(m_Mcol1, m_Mcol11_1, _N, N, *pMTM++, tmp);
            pMTM+=2;
            DOT_AB_MULTIPLE_4(m_Mcol2, m_Mcol2, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol2_0, m_Mcol3, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol2_1, m_Mcol4, _N, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2, m_Mcol5, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol2_0, m_Mcol6, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol2_1, m_Mcol7, _N, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2, m_Mcol8, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol2_0, m_Mcol9, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol2_1, m_Mcol10, _N, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol2, m_Mcol11, Nx2, *pMTM++, tmp);

            pMTM+=3;
            DOT_AB(m_Mcol3, m_Mcol3, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol3, m_Mcol5_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol3, m_Mcol6, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol3, m_Mcol8_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol3, m_Mcol9, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol3, m_Mcol11_0, _N, N, *pMTM++, tmp);
            pMTM+=4;
            *pMTM++=m_MTM[3][3];
            DOT_AB(m_Mcol4, m_Mcol5_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[3][6];
            DOT_AB(m_Mcol4, m_Mcol8_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[3][9];
            DOT_AB(m_Mcol4, m_Mcol11_1, _N, N, *pMTM++, tmp);
            pMTM+=5;
            DOT_AB_MULTIPLE_4(m_Mcol5, m_Mcol5, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol5_0, m_Mcol6, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol5_1, m_Mcol7, _N, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol5, m_Mcol8, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol5_0, m_Mcol9, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol5_1, m_Mcol10, _N, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol5, m_Mcol11, Nx2, *pMTM++, tmp);

            pMTM+=6;
            DOT_AB(m_Mcol6, m_Mcol6, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol6, m_Mcol8_0, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol6, m_Mcol9, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol6, m_Mcol11_0, _N, N, *pMTM++, tmp);
            pMTM+=7;
            *pMTM++=m_MTM[6][6];
            DOT_AB(m_Mcol7, m_Mcol8_1, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            *pMTM++=m_MTM[6][9];
            DOT_AB(m_Mcol7, m_Mcol11_1, _N, N, *pMTM++, tmp);
            pMTM+=8;
            DOT_AB_MULTIPLE_4(m_Mcol8, m_Mcol8, Nx2, *pMTM++, tmp);
            DOT_AB(m_Mcol8_0, m_Mcol9, _N, N, *pMTM++, tmp);
            DOT_AB(m_Mcol8_1, m_Mcol10, _N, N, *pMTM++, tmp);
            DOT_AB_MULTIPLE_4(m_Mcol8, m_Mcol11, Nx2, *pMTM++, tmp);

            pMTM+=9;
            DOT_AB(m_Mcol9, m_Mcol9, _N, N, *pMTM++, tmp);
            *pMTM++=0;
            DOT_AB(m_Mcol9, m_Mcol11_0, _N, N, *pMTM++, tmp);
            pMTM+=10;
            *pMTM++=m_MTM[9][9];
            DOT_AB(m_Mcol10, m_Mcol11_1, _N, N, *pMTM++, tmp);
            pMTM+=11;
            DOT_AB_MULTIPLE_4(m_Mcol11, m_Mcol11, Nx2, *pMTM++, tmp);
            break;
    }
}

inline static void Convert3DTo4D(const float *src,
                                 AlignedVectorN<Point3D, 4> &dst) {
    dst[0].Set(src);
    dst[1].Set(src + 3);
    dst[2].Set(src + 6);
    dst[3].Set(src + 9);
}

inline static void PairwiseSubtract(const AlignedVectorN<Point3D, 4> &Xs,
                                    AlignedVectorN<Point3D, 6> &dXs) {
    dXs[0].XYZx() = ENFT_SSE::_mm_sub_ps(Xs[0].XYZx(), Xs[1].XYZx());
    dXs[1].XYZx() = ENFT_SSE::_mm_sub_ps(Xs[0].XYZx(), Xs[2].XYZx());
    dXs[2].XYZx() = ENFT_SSE::_mm_sub_ps(Xs[0].XYZx(), Xs[3].XYZx());
    dXs[3].XYZx() = ENFT_SSE::_mm_sub_ps(Xs[1].XYZx(), Xs[2].XYZx());
    dXs[4].XYZx() = ENFT_SSE::_mm_sub_ps(Xs[1].XYZx(), Xs[3].XYZx());
    dXs[5].XYZx() = ENFT_SSE::_mm_sub_ps(Xs[2].XYZx(), Xs[3].XYZx());
}

inline static void ElementwiseNormL2(const AlignedVectorN<Point3D, 6> &Xs,
                                     AlignedVectorN<float, 6> &normsL2_2, AlignedVectorN<float, 6> &normsL2) {
    normsL2_2[0] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Xs[0].XYZx(), Xs[0].XYZx()));
    normsL2_2[1] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Xs[1].XYZx(), Xs[1].XYZx()));
    normsL2_2[2] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Xs[2].XYZx(), Xs[2].XYZx()));
    normsL2_2[3] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Xs[3].XYZx(), Xs[3].XYZx()));
    normsL2_2[4] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Xs[4].XYZx(), Xs[4].XYZx()));
    normsL2_2[5] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Xs[5].XYZx(), Xs[5].XYZx()));
    PTR_TO_SSE(normsL2.Data()) = _mm_sqrt_ps(PTR_TO_SSE(normsL2_2.Data()));
    normsL2[4] = sqrt(normsL2_2[4]);
    normsL2[5] = sqrt(normsL2_2[5]);
}

bool CameraEPnP::ComputeCcs(AlignedVector<ENFT_SSE::__m128> &work) {
    char jobz = 'V', uplo = 'L';
    integer n = 12, lda = 12, lwork = 36, info;
    work.Resize(24);
    //double *w = (double *) work.Data(), *work1 = w + 12;
    //dsyev_(&jobz, &uplo, &n, m_MTM[0], &lda, w, work1, &lwork, &info);
    float *w = (float *) work.Data(), *work1 = w + 12;
    //ssyev_(&jobz, &uplo, &n, m_MTM[0], &lda, w, work1, &lwork, &info);
    ssyev_(&jobz, &uplo, &n, m_MTM[0], &lda, w, work1, &lwork, &info);
    if(info != 0)
        return false;
    Convert3DTo4D(m_MTM[0], m_v0);

    //EigenMatrix12f MTM = ToEigenMatrix(m_MTM[0]);
    //Eigen::SelfAdjointEigenSolver<EigenMatrix12f> eig(MTM, Eigen::ComputeEigenvectors);
    //if(eig.info() == Eigen::NoConvergence)
    //  return false;
    //ToEigenMatrix(m_MTM[0]) = eig.eigenvectors();
    //Convert3DTo4D(m_MTM[0], m_v0);
    //m_v0[0].Print();
    //m_v0[1].Print();
    //m_v0[2].Print();
    //m_v0[3].Print();

    //work.Resize(204);
    //char jobz = 'V', range = 'I', uplo = 'L';
    //integer n = 12, lda = 12, m = 12, ldz = 12, il = 1, iu = 1/*, isuppz[24]*/, lwork = 312, liwork = 120, info;
    //double abstol = 0, vl, vu/*, w[12], z[12]*/;
    //double *w = (double *) work.Data();
    //double *z = w + 12;
    //integer *isuppz = (integer *) (z + 12);
    //double *dwork = (double *) (isuppz + 24);
    //integer *iwork = (integer *) (dwork + lwork);
    //dsyevr_(&jobz, &range, &uplo, &n, m_MTM[0], &lda, &vl, &vu, &il, &iu, &abstol, &m, w, z, &ldz, isuppz, dwork, &lwork, iwork, &liwork, &info);
    //if(info != 0)
    //  return false;
    //Convert3DTo4D(z, m_v0);

    PairwiseSubtract(m_Cws, m_dCws);
    ElementwiseNormL2(m_dCws, m_dCwsNormL2_2, m_dCwsNormL2);

    PairwiseSubtract(m_v0, m_dv0);
    ElementwiseNormL2(m_dv0, m_dv0NormL2_2, m_dv0NormL2);

    float num = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(m_dCwsNormL2.Data()),
                                        PTR_TO_SSE(m_dv0NormL2.Data())))
                + m_dCwsNormL2[4] * m_dv0NormL2[4] + m_dCwsNormL2[5] * m_dv0NormL2[5];
    float den = m_dv0NormL2_2[0] + m_dv0NormL2_2[1] + m_dv0NormL2_2[2] +
                m_dv0NormL2_2[3] + m_dv0NormL2_2[4] + m_dv0NormL2_2[5];
    ENFT_SSE::__m128 &beta = work[0];
    beta = _mm_set1_ps(num / den);
    m_Ccs[0].XYZx() = ENFT_SSE::_mm_mul_ps(m_v0[0].XYZx(), beta);
    m_Ccs[1].XYZx() = ENFT_SSE::_mm_mul_ps(m_v0[1].XYZx(), beta);
    m_Ccs[2].XYZx() = ENFT_SSE::_mm_mul_ps(m_v0[2].XYZx(), beta);
    m_Ccs[3].XYZx() = ENFT_SSE::_mm_mul_ps(m_v0[3].XYZx(), beta);

    return true;
}

bool CameraEPnP::ComputeXcs() {
    const ushort N = ushort(m_alphas.Size());
    m_Xcs.Resize(N);
    bool allPos = true, allNeg = true, pos;
    for(ushort i = 0; i < N; ++i) {
        const LA::AlignedVector4f &alpha = m_alphas[i];
        m_Xcs[i].XYZx() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_Ccs[0].XYZx(),
                                                _mm_set1_ps(alpha.v0())), ENFT_SSE::_mm_mul_ps(m_Ccs[1].XYZx(), _mm_set1_ps(alpha.v1()))),
                                     ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_Ccs[2].XYZx(), _mm_set1_ps(alpha.v2())),
                                                ENFT_SSE::_mm_mul_ps(m_Ccs[3].XYZx(), _mm_set1_ps(alpha.v3()))));
        pos = (m_Xcs[i].Z() > 0);
        allPos = allPos && pos;
        allNeg = allNeg && !pos;
    }
    if(allPos)
        return true;
    else if(allNeg) {
        for(ushort i = 0; i < N; ++i) {
            Point3D &Xc = m_Xcs[i];
            Xc.X() = -Xc.X();
            Xc.Y() = -Xc.Y();
            Xc.Z() = -Xc.Z();
        }
        return true;
    } else
        return false;
}