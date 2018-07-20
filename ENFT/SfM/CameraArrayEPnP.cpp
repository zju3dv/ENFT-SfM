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
#include "CameraArrayEPnP.h"
#undef small

extern "C" {
#include <f2c.h>
#include <clapack.h>
}

bool CameraArrayEPnP::Run(const CameraArrayEstimatorMinimalSample &data,
                          CameraArray &CA, AlignedVector<ENFT_SSE::__m128> &work) {
    //CameraArrayEstimatorData _data;
    //_data.Resize(8);
    //for(ushort i = 0; i < 8; ++i)
    //  _data.Set(i, data.X(i), data.x(i), data.GetCameraIndex(i));
    //CameraArray _C;
    //Run(_data, _C, work);
    ////_C.MakeIdentity();
    ////Debug(_data, _C, work);
    //_C[0].Print();

    if(!ComputeCws(data, work) || !ComputeAlphas(data, work))
        return false;
    ConstructLinearSystem(data, work);
    if(!ComputeCcs(work) ||
            !ComputeXcs(data.GetCalibrationParameter(), data.GetCameraIndexes()))
        return false;
    EightMatches3D data3D;
    data3D.Set(0, data.X(0), m_Xcs[0]);
    data3D.Set(1, data.X(1), m_Xcs[1]);
    data3D.Set(2, data.X(2), m_Xcs[2]);
    data3D.Set(3, data.X(3), m_Xcs[3]);
    data3D.Set(4, data.X(4), m_Xcs[4]);
    data3D.Set(5, data.X(5), m_Xcs[5]);
    data3D.Set(6, data.X(6), m_Xcs[6]);
    data3D.Set(7, data.X(7), m_Xcs[7]);
    if(!m_RtSolver.Run(data3D, CA[0], work))
        return false;
    work.Resize(1);
    CA.FromFirstCamera(data.GetCalibrationParameter(), work[0]);

    //CA[0].Print();
    //CA = _CA;

    return true;
}

bool CameraArrayEPnP::Run(const CameraArrayEstimatorData &data, CameraArray &CA,
                          AlignedVector<ENFT_SSE::__m128> &work) {
    if(!ComputeCws(data, work) || !ComputeAlphas(data, work))
        return false;
    ConstructLinearSystem(data, work);
    if(!ComputeCcs(work) ||
            !ComputeXcs(data.GetCalibrationParameter(), data.GetCameraIndexes().data()))
        return false;
    if(!m_RtSolver.Run(data.Xs(), m_Xcs, CA[0], work))
        return false;
    work.Resize(1);
    CA.FromFirstCamera(data.GetCalibrationParameter(), work[0]);
    return true;
}

bool CameraArrayEPnP::ComputeCws(const CameraArrayEstimatorMinimalSample &data,
                                 AlignedVector<ENFT_SSE::__m128> &work) {
    Point3D &Cw0 = m_Cws[0];
    Cw0.XYZx() = ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(_mm_add_ps(data.X(0).XYZx(),
                                       data.X(1).XYZx()), ENFT_SSE::_mm_add_ps(data.X(2).XYZx(), data.X(3).XYZx())),
                                       ENFT_SSE::_mm_add_ps(_mm_add_ps(data.X(4).XYZx(), data.X(5).XYZx()),
                                               ENFT_SSE::_mm_add_ps(data.X(6).XYZx(), data.X(7).XYZx()))), _mm_set1_ps(0.125f));

    Point3D &dXw = m_Cws[1];

#ifndef SET_AT_3x8
#define SET_AT_3x8(i)\
    dXw.XYZx() = ENFT_SSE::_mm_sub_ps(data.X(i).XYZx(), Cw0.XYZx());\
    m_AT3x8.M0##i() = dXw.X();\
    m_AT3x8.M1##i() = dXw.Y();\
    m_AT3x8.M2##i() = dXw.Z()
#endif

    SET_AT_3x8(0);
    SET_AT_3x8(1);
    SET_AT_3x8(2);
    SET_AT_3x8(3);
    SET_AT_3x8(4);
    SET_AT_3x8(5);
    SET_AT_3x8(6);
    SET_AT_3x8(7);

    //LA::AAT(m_AT3x8, m_ATA);
    LA::AATUpper(m_AT3x8, m_ATA);

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
    ks = _mm_sqrt_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(m_w.Data()), _mm_set1_ps(0.125f)));

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

    //for(ushort i = 0; i < 4; ++i)
    //{
    //  printf("Cw %d: ", i);
    //  m_Cws[i].Print();
    //}

    return true;
}

bool CameraArrayEPnP::ComputeCws(const CameraArrayEstimatorData &data,
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

    //for(ushort i = 0; i < 4; ++i)
    //{
    //  printf("Cw %d: ", i);
    //  m_Cws[i].Print();
    //}

    return true;
}

bool CameraArrayEPnP::ComputeAlphas(const CameraArrayEstimatorMinimalSample
                                    &data, AlignedVector<ENFT_SSE::__m128> &work) {
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

    m_alphas.Resize(8);
    ENFT_SSE::__m128 &dXw = work[3], &Cw0 = m_Cws[0].XYZx();

#ifndef SET_ALPHA_8
#define SET_ALPHA_8(i)\
    dXw = ENFT_SSE::_mm_sub_ps(data.X(i).XYZx(), Cw0);\
    m_alphas[i].v1() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T0, dXw));\
    m_alphas[i].v2() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T1, dXw));\
    m_alphas[i].v3() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T2, dXw));\
    m_alphas[i].v0() = 1 - m_alphas[i].v1() - m_alphas[i].v2() - m_alphas[i].v3()
#endif

    SET_ALPHA_8(0);
    SET_ALPHA_8(1);
    SET_ALPHA_8(2);
    SET_ALPHA_8(3);
    SET_ALPHA_8(4);
    SET_ALPHA_8(5);
    SET_ALPHA_8(6);
    SET_ALPHA_8(7);

    //for(ushort i = 0; i < 8; ++i)
    //  m_alphas[i].Print();

    return true;
}

bool CameraArrayEPnP::ComputeAlphas(const CameraArrayEstimatorData &data,
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

    //for(ushort i = 0; i < N; ++i)
    //  m_alphas[i].Print();

    return true;
}

#ifndef SET_MT_ENTRIES
#define SET_MT_ENTRIES(r0, r1, r2, r3, c, v)\
    m_MT[r0][c] = v.m128_f32[0];    m_MT[r1][c] = v.m128_f32[1];    m_MT[r2][c] = v.m128_f32[2];    m_MT[r3][c] = v.m128_f32[3]
#endif

void CameraArrayEPnP::ConstructLinearSystem(const
        CameraArrayEstimatorMinimalSample &data, AlignedVector<ENFT_SSE::__m128> &work) {
    m_MT.Resize(16);
    m_b.Resize(16);

    work.Resize(3);
    ENFT_SSE::__m128 &zero = work[0], &na = work[1], &ax = work[2], &ay = work[2];
    ENFT_SSE::__m128 &cx = work[1], &cy = work[1], &acx = work[2], &acy = work[2];
    zero = ENFT_SSE::_mm_setzero_ps();
    CameraIndex iCam;

#ifndef SET_MT_TWO_ROWS
#define SET_MT_TOW_ROWS(i, r0, r1)\
    if((iCam = data.GetCameraIndex(i)) == 0)\
    {\
        const Point2D &x = data.x(i);\
        const ENFT_SSE::__m128 &a = m_alphas[i].v0123();\
        na = ENFT_SSE::_mm_sub_ps(zero, a);                   SET_MT_ENTRIES(0, 3, 6,  9, r0, na);\
                                                    SET_MT_ENTRIES(1, 4, 7, 10, r0, zero);\
        ax = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(x.x()));     SET_MT_ENTRIES(2, 5, 8, 11, r0, ax);\
                                                    SET_MT_ENTRIES(0, 3, 6,  9, r1, zero);\
                                                    SET_MT_ENTRIES(1, 4, 7, 10, r1, na);\
        ay = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(x.y()));     SET_MT_ENTRIES(2, 5, 8, 11, r1, ay);\
        m_b[r0] = m_b[r1] = 0;\
    }\
    else\
    {\
        const Point2D &x = data.x(i);\
        const ENFT_SSE::__m128 &a = m_alphas[i].v0123();\
        const RigidTransformation3D &T = data.GetCalibrationParameter().GetRelativePose(iCam);\
        cx = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.x()), T.r_20_21_22_x()), T.r_00_01_02_x());\
        acx = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cx.m128_f32[0]));   SET_MT_ENTRIES(0, 3, 6,  9, r0, acx);\
        acx = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cx.m128_f32[1]));   SET_MT_ENTRIES(1, 4, 7, 10, r0, acx);\
        acx = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cx.m128_f32[2]));   SET_MT_ENTRIES(2, 5, 8, 11, r0, acx);\
        m_b[r0] = T.tX() - x.x() * T.tZ();\
        cy = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.y()), T.r_20_21_22_x()), T.r_10_11_12_x());\
        acy = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cy.m128_f32[0]));   SET_MT_ENTRIES(0, 3, 6,  9, r1, acy);\
        acy = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cy.m128_f32[1]));   SET_MT_ENTRIES(1, 4, 7, 10, r1, acy);\
        acy = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cy.m128_f32[2]));   SET_MT_ENTRIES(2, 5, 8, 11, r1, acy);\
        m_b[r1] = T.tY() - x.y() * T.tZ();\
    }
#endif
    SET_MT_TOW_ROWS(0, 0, 1);
    SET_MT_TOW_ROWS(1, 2, 3);
    SET_MT_TOW_ROWS(2, 4, 5);
    SET_MT_TOW_ROWS(3, 6, 7);
    SET_MT_TOW_ROWS(4, 8, 9);
    SET_MT_TOW_ROWS(5, 10, 11);
    SET_MT_TOW_ROWS(6, 12, 13);
    SET_MT_TOW_ROWS(7, 14, 15);

    //m_MT.Print();
}

void CameraArrayEPnP::ConstructLinearSystem(const CameraArrayEstimatorData
        &data, AlignedVector<ENFT_SSE::__m128> &work) {
    const ushort N = data.Size(), Nx2 = (N << 1);
    m_MT.Resize(Nx2);
    m_b.Resize(Nx2);

    work.Resize(3);
    ENFT_SSE::__m128 &zero = work[0], &na = work[1], &ax = work[2], &ay = work[2];
    ENFT_SSE::__m128 &cx = work[1], &cy = work[1], &acx = work[2], &acy = work[2];
    zero = ENFT_SSE::_mm_setzero_ps();

    CameraIndex iCam;
    for(ushort i = 0, ix2 = 0, ix2p1 = 1; i < N;
            ++i, ix2 = (i << 1), ix2p1 = ix2 + 1) {
        const Point2D &x = data.x(i);
        const ENFT_SSE::__m128 &a = m_alphas[i].v0123();
        if((iCam = data.GetCameraIndex(i)) == 0) {
            na = ENFT_SSE::_mm_sub_ps(zero, a);
            SET_MT_ENTRIES(0, 3, 6,  9, ix2, na);
            SET_MT_ENTRIES(1, 4, 7, 10, ix2, zero);
            ax = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(x.x()));
            SET_MT_ENTRIES(2, 5, 8, 11, ix2, ax);
            SET_MT_ENTRIES(0, 3, 6,  9, ix2p1, zero);
            SET_MT_ENTRIES(1, 4, 7, 10, ix2p1, na);
            ay = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(x.y()));
            SET_MT_ENTRIES(2, 5, 8, 11, ix2p1, ay);
            m_b[ix2] = m_b[ix2p1] = 0;
        } else
            //iCam = data.GetCameraIndex(i);
        {
            const RigidTransformation3D &T = data.GetCalibrationParameter().GetRelativePose(
                                                 iCam);
            cx = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.x()), T.r_20_21_22_x()),
                            T.r_00_01_02_x());
            acx = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cx.m128_f32[0]));
            SET_MT_ENTRIES(0, 3, 6,  9, ix2, acx);
            acx = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cx.m128_f32[1]));
            SET_MT_ENTRIES(1, 4, 7, 10, ix2, acx);
            acx = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cx.m128_f32[2]));
            SET_MT_ENTRIES(2, 5, 8, 11, ix2, acx);
            m_b[ix2] = T.tX() - x.x() * T.tZ();
            cy = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.y()), T.r_20_21_22_x()),
                            T.r_10_11_12_x());
            acy = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cy.m128_f32[0]));
            SET_MT_ENTRIES(0, 3, 6,  9, ix2p1, acy);
            acy = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cy.m128_f32[1]));
            SET_MT_ENTRIES(1, 4, 7, 10, ix2p1, acy);
            acy = ENFT_SSE::_mm_mul_ps(a, _mm_set1_ps(cy.m128_f32[2]));
            SET_MT_ENTRIES(2, 5, 8, 11, ix2p1, acy);
            m_b[ix2p1] = T.tY() - x.y() * T.tZ();
        }
    }

    //m_MT.Print();
}

bool CameraArrayEPnP::ComputeCcs(AlignedVector<ENFT_SSE::__m128> &work) {
//#if _DEBUG
//  LA::AlignedMatrixXx12d _M;
//  const ushort N = ushort(m_MT.GetColumnsNumber());
//  _M.Resize(N);
//  for(ushort i = 0; i < N; ++i)
//  for(ushort j = 0; j < 12; ++j)
//      _M[i][j] = m_MT[j][i];
//#endif

    double *z;
    if(m_MT.GetColumnsNumber() == 12) {
        work.Resize(9);
        z = (double *) work.Data();

        // Call LAPACK
        integer n = 12, nrhs = 1, lda = m_MT.Stride(), ldb = n, info,
                *ipiv = (integer *) (work.Data() + 6);
        memcpy(z, m_b.Data(), 96);
        dgesv_(&n, &nrhs, m_MT[0], &lda, ipiv, z, &ldb, &info);
        if(info != 0)
            return false;

    } else {
        // Call LAPACK
        integer m = m_MT.GetColumnsNumber(), n = 12, nrhs = 1, lda = m_MT.Stride(),
                ldb = m, rank, lwork = -1, info;
        double rcond = -1;
        //m_dwork.Resize(1);
        //dgelsd_(&m, &n, &nrhs, m_MT[0], &lda, m_z, &ldb, m_s, &rcond, &rank, m_dwork, &lwork, m_iwork, &info);
        //lwork = integer(m_dwork[0]);
        //m_dwork.Resize(uint(lwork));
        //dgelsd_(&m, &n, &nrhs, m_MT[0], &lda, m_z, &ldb, m_s, &rcond, &rank, m_dwork, &lwork, m_iwork, &info);
        work.Resize(1);
        double *workd = (double *) work.Data();
        dgelsy_(&m, &n, &nrhs, m_MT[0], &lda, NULL, &ldb, NULL, &rcond, &rank, workd,
                &lwork, &info);
        lwork = integer(workd[0]);
        work.Resize((m_b.Size() + 6 + lwork) >> 1);
        z = (double *) work.Data();
        integer *jpvt = (integer *) (z + m_b.Size());
        workd = (double *) (jpvt + 12);
        memcpy(z, m_b.Data(), m_b.Size() << 3);
        dgelsy_(&m, &n, &nrhs, m_MT[0], &lda, z, &ldb, jpvt, &rcond, &rank, workd,
                &lwork, &info);
        if(info != 0)
            return false;
    }
    m_Ccs[0].Set(z);
    m_Ccs[1].Set(z + 3);
    m_Ccs[2].Set(z + 6);
    m_Ccs[3].Set(z + 9);

//#if _DEBUG
//  for(ushort i = 0; i < N; ++i)
//  {
//      double dot = 0;
//      for(ushort j = 0; j < 12; ++j)
//          dot += _M[i][j] * z[j];
//      printf("%f\n", dot - m_b[i]);
//  }
//  for(ushort i = 0; i < N; ++i)
//  for(ushort j = 0; j < 12; ++j)
//      m_MT[j][i] = _M[i][j];
//#endif

    //for(ushort i = 0; i < 4; ++i)
    //{
    //  printf("Cc %d: ", i);
    //  m_Ccs[i].Print();
    //}

    return true;
}

bool CameraArrayEPnP::ComputeXcs(const CameraArrayCalibrationParameter
                                 &calibParam, const CameraIndex *iCams) {
    const ushort N = ushort(m_alphas.Size());
    m_Xcs.Resize(N);
    bool allPos = true, allNeg = true, pos;
    for(ushort i = 0; i < N; ++i) {
        const LA::AlignedVector4f &alpha = m_alphas[i];
        m_Xcs[i].XYZx() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_Ccs[0].XYZx(),
                                                _mm_set1_ps(alpha.v0())), ENFT_SSE::_mm_mul_ps(m_Ccs[1].XYZx(), _mm_set1_ps(alpha.v1()))),
                                     ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_Ccs[2].XYZx(), _mm_set1_ps(alpha.v2())),
                                                ENFT_SSE::_mm_mul_ps(m_Ccs[3].XYZx(), _mm_set1_ps(alpha.v3()))));
        m_Xcs[i].reserve() = 1;
        if(iCams[0] == 0)
            pos = (m_Xcs[i].Z() > 0);
        else
            pos = (calibParam.GetRelativePose(iCams[i]).ApplyZ(m_Xcs[i]) > 0);
        allPos = allPos && pos;
        allNeg = allNeg && !pos;
    }
    //for(ushort i = 0; i < N; ++i)
    //{
    //  printf("Xc: ", i);
    //  m_Xcs[i].Print();
    //}
//#if _DEBUG
//  return true;
//#endif
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

bool CameraArrayEPnP::Debug(const CameraArrayEstimatorData &data,
                            const CameraArray &CA, AlignedVector<ENFT_SSE::__m128> &work) {
    if(!ComputeCws(data, work) || !ComputeAlphas(data, work))
        return false;
    AlignedVectorN<Point3D, 4> Ccs;
    for(ushort i = 0; i < 4; ++i) {
        m_Cws[i].reserve() = 1;
        CA[0].Apply(m_Cws[i], Ccs[i]);
    }
    const ushort N = data.Size();
    Point3D Xf, Xc;
    CameraIndex iCam;
    Point2D x;
    float sqErr;
    for(ushort i = 0; i < N; ++i) {
        const LA::AlignedVector4f &a = m_alphas[i];
        Xf.XYZx() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(a.v0()),
                                          Ccs[0].XYZx()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(a.v1()), Ccs[1].XYZx())),
                               ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(a.v2()), Ccs[2].XYZx()),
                                          ENFT_SSE::_mm_mul_ps(_mm_set1_ps(a.v3()), Ccs[3].XYZx())));
        Xf.reserve() = 1;
        iCam = data.GetCameraIndex(i);
        if(iCam == 0) {
            x.x() = Xf.X() / Xf.Z();
            x.y() = Xf.Y() / Xf.Z();
        } else {
            data.GetCalibrationParameter().GetRelativePose(iCam).Apply(Xf, Xc);
            x.x() = Xc.X() / Xc.Z();
            x.y() = Xc.Y() / Xc.Z();
        }
        x.Print();
        data.x(i).Print();
        sqErr = x.SquaredDistance(data.x(i));
        printf("%f\n", sqErr);
    }
    ConstructLinearSystem(data, work);
    //LA::AlignedVector12d col;
    //double bx, by, chkx, chky;
    //for(ushort i = 0, ix2 = 0, ix2p1 = 1; i < N; ++i, ix2 = (i << 1), ix2p1 = ix2 + 1)
    //{
    //  m_MT.GetColumn(ix2, col);
    //  //if(i == 5)
    //  //col.Print();
    //  bx = col[ 0] * Ccs[0].X() + col[ 1] * Ccs[0].Y() + col[ 2] * Ccs[0].Z()
    //     + col[ 3] * Ccs[1].X() + col[ 4] * Ccs[1].Y() + col[ 5] * Ccs[1].Z()
    //     + col[ 6] * Ccs[2].X() + col[ 7] * Ccs[2].Y() + col[ 8] * Ccs[2].Z()
    //     + col[ 9] * Ccs[3].X() + col[10] * Ccs[3].Y() + col[11] * Ccs[3].Z();
    //  m_MT.GetColumn(ix2p1, col);
    //  //if(i == 5)
    //  //col.Print();
    //  by = col[ 0] * Ccs[0].X() + col[ 1] * Ccs[0].Y() + col[ 2] * Ccs[0].Z()
    //     + col[ 3] * Ccs[1].X() + col[ 4] * Ccs[1].Y() + col[ 5] * Ccs[1].Z()
    //     + col[ 6] * Ccs[2].X() + col[ 7] * Ccs[2].Y() + col[ 8] * Ccs[2].Z()
    //     + col[ 9] * Ccs[3].X() + col[10] * Ccs[3].Y() + col[11] * Ccs[3].Z();
    //  chkx = bx - m_b[ix2];
    //  chky = by - m_b[ix2p1];
    //  //printf("(%f, %f), (%f, %f): %f, %f\n", bx, by, m_b[ix2], m_b[ix2p1], chkx, chky);
    //  printf("%f, %f\n", chkx, chky);
    //}
    if(!ComputeCcs(work))
        return false;
    //for(ushort i = 0, ix2 = 0, ix2p1 = 1; i < N; ++i, ix2 = (i << 1), ix2p1 = ix2 + 1)
    //{
    //  m_MT.GetColumn(ix2, col);
    //  //if(i == 5)
    //  //col.Print();
    //  bx = col[ 0] * m_Ccs[0].X() + col[ 1] * m_Ccs[0].Y() + col[ 2] * m_Ccs[0].Z()
    //     + col[ 3] * m_Ccs[1].X() + col[ 4] * m_Ccs[1].Y() + col[ 5] * m_Ccs[1].Z()
    //     + col[ 6] * m_Ccs[2].X() + col[ 7] * m_Ccs[2].Y() + col[ 8] * m_Ccs[2].Z()
    //     + col[ 9] * m_Ccs[3].X() + col[10] * m_Ccs[3].Y() + col[11] * m_Ccs[3].Z();
    //  m_MT.GetColumn(ix2p1, col);
    //  //if(i == 5)
    //  //col.Print();
    //  by = col[ 0] * m_Ccs[0].X() + col[ 1] * m_Ccs[0].Y() + col[ 2] * m_Ccs[0].Z()
    //     + col[ 3] * m_Ccs[1].X() + col[ 4] * m_Ccs[1].Y() + col[ 5] * m_Ccs[1].Z()
    //     + col[ 6] * m_Ccs[2].X() + col[ 7] * m_Ccs[2].Y() + col[ 8] * m_Ccs[2].Z()
    //     + col[ 9] * m_Ccs[3].X() + col[10] * m_Ccs[3].Y() + col[11] * m_Ccs[3].Z();
    //  chkx = bx - m_b[ix2];
    //  chky = by - m_b[ix2p1];
    //  //printf("(%f, %f), (%f, %f): %f, %f\n", bx, by, m_b[ix2], m_b[ix2p1], chkx, chky);
    //  printf("%f, %f\n", chkx, chky);
    //}
    //for(ushort i = 0, ix2 = 0, ix2p1 = 1; i < N; ++i, ix2 = (i << 1), ix2p1 = ix2 + 1)
    //{
    //  m_MT.GetColumn(ix2, col);
    //  //if(i == 5)
    //  //col.Print();
    //  bx = col[ 0] * Ccs[0].X() + col[ 1] * Ccs[0].Y() + col[ 2] * Ccs[0].Z()
    //     + col[ 3] * Ccs[1].X() + col[ 4] * Ccs[1].Y() + col[ 5] * Ccs[1].Z()
    //     + col[ 6] * Ccs[2].X() + col[ 7] * Ccs[2].Y() + col[ 8] * Ccs[2].Z()
    //     + col[ 9] * Ccs[3].X() + col[10] * Ccs[3].Y() + col[11] * Ccs[3].Z();
    //  m_MT.GetColumn(ix2p1, col);
    //  //if(i == 5)
    //  //col.Print();
    //  by = col[ 0] * Ccs[0].X() + col[ 1] * Ccs[0].Y() + col[ 2] * Ccs[0].Z()
    //     + col[ 3] * Ccs[1].X() + col[ 4] * Ccs[1].Y() + col[ 5] * Ccs[1].Z()
    //     + col[ 6] * Ccs[2].X() + col[ 7] * Ccs[2].Y() + col[ 8] * Ccs[2].Z()
    //     + col[ 9] * Ccs[3].X() + col[10] * Ccs[3].Y() + col[11] * Ccs[3].Z();
    //  chkx = bx - m_b[ix2];
    //  chky = by - m_b[ix2p1];
    //  //printf("(%f, %f), (%f, %f): %f, %f\n", bx, by, m_b[ix2], m_b[ix2p1], chkx, chky);
    //  printf("%f, %f\n", chkx, chky);
    //}
    float chk;
    for(ushort i = 0; i < 4; ++i) {
        //m_Ccs[i].Print();
        //Ccs[i].Print();
        chk = m_Ccs[i].SquaredDistance(Ccs[i]);
        printf("%f\n", chk);
    }
    //if(!ComputeCcs(work) || !ComputeXcs(data.GetCameraIndexes()))
    //  return false;
    //if(!m_RtSolver.Run(data.Xs(), m_Xcs, CA[0], work))
    //  return false;
    //work.Resize(1);
    //CA.FromFirstCamera(work[0]);
    return true;
}