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
#include "SegmentSetBundleAdjustorData2DSimilarity.h"
#if _DEBUG
//#define _DEBUG_WITH_EIGEN
#ifdef _DEBUG_WITH_EIGEN
//#define _DEBUG_WITH_EIGEN_JTJ_JTE
#define _DEBUG_WITH_EIGEN_AX
#include <Eigen>
#endif
#endif

void SegmentSetBundleAdjustorData2DSimilarity::Resize(const SegmentIndex &nSegs,
        const TrackIndex &nTrks, const MeasurementIndex &nMeas) {
    BundleAdjustorDataTemplate<SEGMENT_SET_BA_ARGUMENT_2D_SIMILARITY>::Resize(nSegs,
            nTrks, nMeas);
    m_mapMeaToSeqFrm.resize(nMeas);
    m_mapSegToSeqFrm.resize(nSegs);
}

void SegmentSetBundleAdjustorData2DSimilarity::ReorderMeasurements(
    const MeasurementIndexList &iMeasOriToNew) {
    BundleAdjustorDataTemplate<SEGMENT_SET_BA_ARGUMENT_2D_SIMILARITY>::ReorderMeasurements(
        iMeasOriToNew);
    const MeasurementIndex nMeas = GetMeasurementsNumber();
    m_mapMeaToSeqFrm.swap(m_mapMeaToSeqFrmOri);
    m_mapMeaToSeqFrm.resize(nMeas);
    for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea)
        m_mapMeaToSeqFrm[iMeasOriToNew[iMea]] = m_mapMeaToSeqFrmOri[iMea];
}

void SegmentSetBundleAdjustorData2DSimilarity::ValidateGlobal() {
    m_r2s.Resize(m_xs.Size());
    m_r2xs.Resize(0);
}

void SegmentSetBundleAdjustorData2DSimilarity::InvalidateGlobal() {
    m_r2s.Resize(0);
    m_r2xs.Resize(0);
}

bool SegmentSetBundleAdjustorData2DSimilarity::IsGlobalValid() const {
    return !m_r2s.Empty();
}

void SegmentSetBundleAdjustorData2DSimilarity::UndistortMeasurements(
    const Camera::IntrinsicParameter &Kr) {
#if _DEBUG
    assert(IsGlobalValid());
#endif
    const uint N = m_xs.Size(), _N2 = N - (N & 1), _N4 = N - (N & 3);

    uint i;
    if(m_r2xs.Empty()) {
        m_r2xs.Resize(N);

        ENFT_SSE::__m128 r2;
        const ENFT_SSE::__m128 *x = (ENFT_SSE::__m128 *) m_xs.Data();
        ENFT_SSE::__m128 *r2x = (ENFT_SSE::__m128 *) m_r2xs.Data();
        for(i = 0; i < _N2; i += 2, ++x, ++r2x) {
            r2 = ENFT_SSE::_mm_mul_ps(*x, *x);
            m_r2s[i  ] = r2.m128_f32[0] = r2.m128_f32[1] = r2.m128_f32[0] + r2.m128_f32[1];
            m_r2s[i+1] = r2.m128_f32[2] = r2.m128_f32[3] = r2.m128_f32[2] + r2.m128_f32[3];
            *r2x = ENFT_SSE::_mm_mul_ps(r2, *x);
        }
        if(_N2 != N) {
            const Point2D &x = m_xs[_N2];
            float &r2 = m_r2s[_N2];
            r2 = x.SquaredLength();
            m_r2xs[_N2].Set(r2 * x.x(), r2 * x.y());
        }
    }

    m_cs.Resize(N);
    m_xus.Resize(N);

    const ENFT_SSE::__m128 d = _mm_set1_ps(Kr.d()), one = _mm_set1_ps(1.0f);
    const ENFT_SSE::__m128 *r2 = (ENFT_SSE::__m128 *) m_r2s.Data();
    ENFT_SSE::__m128 *ps = (ENFT_SSE::__m128 *) m_cs.Data();
    for(i = 0; i < _N4; i += 4, ++r2, ++ps)
        *ps = ENFT_SSE::_mm_add_ps(one, ENFT_SSE::_mm_mul_ps(d, *r2));
    for(; i < N; ++i)
        m_cs[i] = 1 + Kr.d() * m_r2s[i];

    ENFT_SSE::__m128 c;
    const ENFT_SSE::__m128 *x = (ENFT_SSE::__m128 *) m_xs.Data();
    ENFT_SSE::__m128 *xu = (ENFT_SSE::__m128 *) m_xus.Data();
    for(i = 0; i < _N2; i += 2, ++x, ++xu) {
        c.m128_f32[0] = c.m128_f32[1] = m_cs[i];
        c.m128_f32[2] = c.m128_f32[3] = m_cs[i + 1];
        *xu = ENFT_SSE::_mm_mul_ps(c, *x);
    }
    if(_N2 != N) {
        const float &c = m_cs[_N2];
        const Point2D &x = m_xs[_N2];
        m_xus[_N2].Set(c * x.x(), c * x.y());
    }
}

void SegmentSetBundleAdjustorData2DSimilarity::ScaleMeasurements(
    const float scale, AlignedVector<Point2D> &xs) {
    if(scale == 1.0f)
        return;
    const ENFT_SSE::__m128 s = _mm_set1_ps(scale);
    const uint N = xs.Size(), _N = N - (N & 1);
    ENFT_SSE::__m128 *x = (ENFT_SSE::__m128 *) xs.Data();
    for(uint i = 0; i < _N; i += 2, ++x)
        *x = ENFT_SSE::_mm_mul_ps(s, *x);
    if(_N != N)
        xs[_N] *= scale;
}

void SegmentSetBundleAdjustorData2DSimilarity::NormalizeData(
    const float dataNormalizeMedian) {
    if(dataNormalizeMedian == 0) {
        m_scaleScene = m_scaleFocal = 1.0f;
        m_translation.SetZero();
        return;
    }
    //Point3D sum;
    //sum.SetZero();
    //const TrackIndex nTrks = TrackIndex(m_Xs.Size());
    //for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
    //  LA::ApB(m_Xs[iTrk], sum, sum);
    //Point3D mean;
    //mean.XYZx() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1.0f / nTrks), sum.XYZx());
    TrackIndex nTrks = TrackIndex(m_Xs.Size());
    if (m_Xs.Size() == 0) {
        printf("m_Xs.Size == 0 bug");//bug
        return;
    }
    std::vector<float> Xs(nTrks), Ys(nTrks), Zs(nTrks);
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        const Point3D &X = m_Xs[iTrk];
        Xs[iTrk] = X.X();
        Ys[iTrk] = X.Y();
        Zs[iTrk] = X.Z();
    }
    const TrackIndex ith = (nTrks >> 1);
    std::nth_element(Xs.begin(), Xs.begin() + ith, Xs.end());
    std::nth_element(Ys.begin(), Ys.begin() + ith, Ys.end());
    std::nth_element(Zs.begin(), Zs.begin() + ith, Zs.end());
    const Point3D mean(Xs[ith], Ys[ith], Zs[ith]);
    m_translation.Set(-mean.X(), -mean.Y(), -mean.Z());

    Point3D dX;
    m_distSqs.resize(nTrks);
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        LA::AmB(m_Xs[iTrk], mean, dX);
        m_distSqs[iTrk] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(dX.XYZx(), dX.XYZx()));
    }
    //const TrackIndex ith = (nTrks >> 1);
    std::nth_element(m_distSqs.begin(), m_distSqs.begin() + ith, m_distSqs.end());
    const float distSqMed = m_distSqs[ith];
    m_scaleScene = (1 / sqrt(distSqMed)) / dataNormalizeMedian;

    ENFT_SSE::__m128 dt;
    const ENFT_SSE::__m128 translation = _mm_setr_ps(m_translation.v0(), m_translation.v1(),
                                           m_translation.v2(), 0);
    const SegmentIndex nSegs = SegmentIndex(m_Cs.Size());
    for(SegmentIndex iSeg = 0; iSeg < nSegs; ++iSeg) {
        SimilarityTransformation3D &S = m_Cs[iSeg];
        S.ApplyRotation(translation, dt);
        dt = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / S.s()), translation), dt);
        S.tX() = m_scaleScene * (S.tX() + dt.m128_f32[0]);
        S.tY() = m_scaleScene * (S.tY() + dt.m128_f32[1]);
        S.tZ() = m_scaleScene * (S.tZ() + dt.m128_f32[2]);
    }

    const SequenceIndex nSeqs = SequenceIndex(m_pSeqs.size());
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        m_pSeqs[iSeq]->TranslateScene(m_translation);
        m_pSeqs[iSeq]->ScaleScene(m_scaleScene);
    }

    const ENFT_SSE::__m128 scale = _mm_setr_ps(m_scaleScene, m_scaleScene, m_scaleScene,
                                     1.0f);
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        m_Xs[iTrk] += translation;
        m_Xs[iTrk] *= scale;
    }

    if(IsGlobalValid()) {
        m_scaleFocal = dataNormalizeMedian / m_G.f();
        m_G.Scale(m_scaleFocal);
        m_xsOri = m_xs;
        ScaleMeasurements(m_scaleFocal, m_xs);
    } else
        m_scaleFocal = 1.0f;
}

void SegmentSetBundleAdjustorData2DSimilarity::DenormalizeData() {
    if(m_scaleScene != 1 || m_translation.SquaredLength() != 0) {
        m_scaleScene = 1 / m_scaleScene;
        ENFT_SSE::__m128 dt;
        m_translation.X() = -m_translation.X();
        m_translation.Y() = -m_translation.Y();
        m_translation.Z() = -m_translation.Z();
        const ENFT_SSE::__m128 translation = _mm_setr_ps(m_translation.X(), m_translation.Y(),
                                               m_translation.Z(), 0);
        const SegmentIndex nSegs = SegmentIndex(m_Cs.Size());
        for(SegmentIndex iSeg = 0; iSeg < nSegs; ++iSeg) {
            SimilarityTransformation3D &S = m_Cs[iSeg];
            S.ApplyRotation(translation, dt);
            //dt =_mm_sub_ps(dt, ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / S.s()), translation));
            dt = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / S.s()), translation), dt);
            S.tX() = m_scaleScene * S.tX() + dt.m128_f32[0];
            S.tY() = m_scaleScene * S.tY() + dt.m128_f32[1];
            S.tZ() = m_scaleScene * S.tZ() + dt.m128_f32[2];
        }

        const SequenceIndex nSeqs = SequenceIndex(m_pSeqs.size());
        for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
            m_pSeqs[iSeq]->ScaleScene(m_scaleScene);
            m_pSeqs[iSeq]->TranslateScene(m_translation);
        }

        const ENFT_SSE::__m128 scale = _mm_setr_ps(m_scaleScene, m_scaleScene, m_scaleScene,
                                         1.0f);
        const TrackIndex nTrks = TrackIndex(m_Xs.Size());
        for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
            m_Xs[iTrk] *= scale;
            //m_Xs[iTrk] -= translation;
            m_Xs[iTrk] += translation;
        }
    }

    if(IsGlobalValid()) {
        m_scaleFocal = 1 / m_scaleFocal;
        m_G.Scale(m_scaleFocal);
        m_xs.Swap(m_xsOri);
    }
}

double SegmentSetBundleAdjustorData2DSimilarity::ComputeSSE(
    std::vector<float> &ptSSEs) {
    const TrackIndex nTrks = GetPointsNumber();
    ptSSEs.assign(nTrks, 0.0f);

    Point3D SX;
    SequenceIndex iSeq;
    FrameIndex iFrm;
    TrackIndex iTrk;
    MeasurementIndex iMea;
    const MeasurementIndex nMeas = GetMeasurementsNumber();
    if(IsGlobalValid()) {
        Point2D xp;
        UndistortMeasurements(m_G);
        const float f = m_G.f();
        for(iMea = 0; iMea < nMeas; ++iMea) {
            iTrk = m_mapMeaToTrk[iMea];
            m_Cs[m_mapMeaToSeg[iMea]].Apply(m_Xs[iTrk], SX);
            m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
            m_pSeqs[iSeq]->GetCamera(iFrm).ProjectToNormalizedPlane(SX, xp);
            xp *= f;
            ptSSEs[iTrk] += m_xus[iMea].SquaredDistance(xp);
        }
    } else {
        Point2D e;
        for(iMea = 0; iMea < nMeas; ++iMea) {
            iTrk = m_mapMeaToTrk[iMea];
            m_Cs[m_mapMeaToSeg[iMea]].Apply(m_Xs[iTrk], SX);
            m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
            ptSSEs[iTrk] += m_pSeqs[iSeq]->GetCamera(iFrm).ComputeProjectionSquaredError(SX,
                            m_xs[iMea], e);
        }

        //float errSq, errSqMax = 0;
        //MeasurementIndex iMeaMax = 0;
        //for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea)
        //{
        //  m_Cs[m_mapMeaToSeg[iMea]].Apply(m_Xs[m_mapMeaToTrk[iMea]], SX);
        //  m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
        //  if((errSq = m_pSeqs[iSeq]->GetCamera(iFrm).ComputeProjectionSquaredError(SX, m_xs[iMea], e)) < errSqMax)
        //      continue;
        //  errSqMax = errSq;
        //  iMeaMax = iMea;
        //}
        ////iMeaMax = 160165;
        //printf("S:\n");       m_Cs[m_mapMeaToSeg[iMeaMax]].Print();
        //printf("X:\n");       m_Xs[m_mapMeaToTrk[iMeaMax]].Print();
        //printf("C:\n");       m_mapMeaToSeqFrm[iMeaMax].Get(iSeq, iFrm);  m_pSeqs[iSeq]->GetCamera(iFrm).Print();
        //printf("x:\n");       m_xs[iMeaMax].Print();
        //printf("SX:\n");  m_Cs[m_mapMeaToSeg[iMeaMax]].Apply(m_Xs[m_mapMeaToTrk[iMeaMax]], SX);   SX.Print();
        //printf("xp:\n");  Point2D xp; m_pSeqs[iSeq]->GetCamera(iFrm).ProjectToNormalizedPlane(SX, xp); xp.Print();
        //printf("errSq = %f\n", errSq = m_pSeqs[iSeq]->GetCamera(iFrm).ComputeProjectionSquaredError(SX, m_xs[iMeaMax], e));
        //printf("iSeq = %d, iFrm = %d, iTrk = %d, iMea = %d\n", iSeq, iFrm, m_mapMeaToTrk[iMeaMax], iMeaMax);
    }

    double SSE = 0;
    for(iTrk = 0; iTrk < nTrks; ++iTrk)
        SSE += ptSSEs[iTrk];
    return SSE;
}

void SegmentSetBundleAdjustorData2DSimilarity::ComputeMSE(
    std::vector<float> &seqMSEs) {
    const SequenceIndex nSeqs = SequenceIndex(m_pSeqs.size());
    seqMSEs.assign(nSeqs, 0.0f);

    Point3D SX;
    SequenceIndex iSeq;
    FrameIndex iFrm;
    TrackIndex iTrk;
    MeasurementIndex iMea;
    const MeasurementIndex nMeas = GetMeasurementsNumber();
    if(IsGlobalValid()) {
        Point2D xp;
        UndistortMeasurements(m_G);
        const float f = m_G.f();
        for(iMea = 0; iMea < nMeas; ++iMea) {
            iTrk = m_mapMeaToTrk[iMea];
            m_Cs[m_mapMeaToSeg[iMea]].Apply(m_Xs[iTrk], SX);
            m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
            m_pSeqs[iSeq]->GetCamera(iFrm).ProjectToNormalizedPlane(SX, xp);
            xp *= f;
            seqMSEs[iSeq] += m_xus[iMea].SquaredDistance(xp);
        }
    } else {
        Point2D e;
        for(iMea = 0; iMea < nMeas; ++iMea) {
            iTrk = m_mapMeaToTrk[iMea];
            m_Cs[m_mapMeaToSeg[iMea]].Apply(m_Xs[iTrk], SX);
            m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
            seqMSEs[iSeq] += m_pSeqs[iSeq]->GetCamera(iFrm).ComputeProjectionSquaredError(
                                 SX, m_xs[iMea], e);
        }

        //float errSq, errSqMax = 0;
        //MeasurementIndex iMeaMax = 0;
        //for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea)
        //{
        //  m_Cs[m_mapMeaToSeg[iMea]].Apply(m_Xs[m_mapMeaToTrk[iMea]], SX);
        //  m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
        //  if((errSq = m_pSeqs[iSeq]->GetCamera(iFrm).ComputeProjectionSquaredError(SX, m_xs[iMea], e)) < errSqMax)
        //      continue;
        //  errSqMax = errSq;
        //  iMeaMax = iMea;
        //}
        ////iMeaMax = 160165;
        //printf("S:\n");       m_Cs[m_mapMeaToSeg[iMeaMax]].Print();
        //printf("X:\n");       m_Xs[m_mapMeaToTrk[iMeaMax]].Print();
        //printf("C:\n");       m_mapMeaToSeqFrm[iMeaMax].Get(iSeq, iFrm);  m_pSeqs[iSeq]->GetCamera(iFrm).Print();
        //printf("x:\n");       m_xs[iMeaMax].Print();
        //printf("SX:\n");  m_Cs[m_mapMeaToSeg[iMeaMax]].Apply(m_Xs[m_mapMeaToTrk[iMeaMax]], SX);   SX.Print();
        //printf("xp:\n");  Point2D xp; m_pSeqs[iSeq]->GetCamera(iFrm).ProjectToNormalizedPlane(SX, xp); xp.Print();
        //printf("errSq = %f\n", errSq = m_pSeqs[iSeq]->GetCamera(iFrm).ComputeProjectionSquaredError(SX, m_xs[iMeaMax], e));
        //printf("iSeq = %d, iFrm = %d, iTrk = %d, iMea = %d\n", iSeq, iFrm, m_mapMeaToTrk[iMeaMax], iMeaMax);
    }

    SegmentIndex iSeg1, iSeg2 = 0;
    const SegmentIndex nSegs = SegmentIndex(m_mapSegToSeqFrm.size());
    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        iSeg1 = iSeg2;
#if _DEBUG
        assert(m_mapSegToSeqFrm[iSeg2].GetSequenceIndex() == iSeq);
#endif
        while(iSeg2 < nSegs && m_mapSegToSeqFrm[iSeg2].GetSequenceIndex() == iSeq)
            ++iSeg2;
        seqMSEs[iSeq] *= m_pSeqs[iSeq]->GetIntrinsicMatrix().fxy() /
                         (m_mapSegToMea[iSeg2] - m_mapSegToMea[iSeg1]);
    }
}

double SegmentSetBundleAdjustorData2DSimilarity::ComputeMSE(
    const Camera::IntrinsicParameter &Kr) {
    ValidateGlobal();

    Point3D SX;
    SequenceIndex iSeq;
    FrameIndex iFrm;
    TrackIndex iTrk;
    MeasurementIndex iMea;
    const MeasurementIndex nMeas = GetMeasurementsNumber();

    Point2D e;
    double SSE = 0.0;
    UndistortMeasurements(m_G);
    m_xrs.Set(m_xus.Data(), m_xus.Size());
    ScaleMeasurements(1 / Kr.f(), m_xrs);
    for(iMea = 0; iMea < nMeas; ++iMea) {
        iTrk = m_mapMeaToTrk[iMea];
        m_Cs[m_mapMeaToSeg[iMea]].Apply(m_Xs[iTrk], SX);
        m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
        SSE += m_pSeqs[iSeq]->GetCamera(iFrm).ComputeProjectionSquaredError(SX,
                m_xrs[iMea], e);
    }
    return SSE * GetFactorSSEToMSE();
}

static inline void Project(const SimilarityTransformation3D &S,
                           const Point3D &X, Point3D &SX) {
    S.Apply(X, SX);
    SX.reserve() = 1.0f;
}
static inline void Project(const SimilarityTransformation3D &S,
                           const Point3D &X, Point3D &sRX, Point3D &TX, Point3D &SX) {
    S.Apply(X, sRX, TX, SX);
    sRX.Scale(S.sss1());
    SX.reserve() = 1.0f;
}
static inline void Project(const ENFT_SSE::__m128 &s, const LA::AlignedMatrix3f &sR,
                           const Point3D &sRX, const Point3D &TX, const Point3D &SX, const Camera &C,
                           Point2D &x,
                           LA::AlignedMatrix2x7f &Jc, LA::AlignedMatrix2x3f &Jx, ENFT_SSE::__m128 *work3) {
    ENFT_SSE::__m128 &ZcI = work3[0];
    C.ProjectToNormalizedPlane(SX, ZcI.m128_f32[0], x.x(), x.y());
    ZcI = _mm_set1_ps(ZcI.m128_f32[0]);
    ENFT_SSE::__m128 &J_x_SX = work3[1], &J_y_SX = work3[2];
    J_x_SX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ZcI, C.r_00_01_02_x()),
                        ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI.m128_f32[0] * x.x()), C.r_20_21_22_x()));
    J_y_SX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ZcI, C.r_10_11_12_x()),
                        ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI.m128_f32[0] * x.y()), C.r_20_21_22_x()));

    // S = {t, w, s}
    Jc.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(J_x_SX, s);
    Jc.M03() = J_x_SX.m128_f32[2] * sRX.Y() - J_x_SX.m128_f32[1] * sRX.Z();
    Jc.M04() = J_x_SX.m128_f32[0] * sRX.Z() - J_x_SX.m128_f32[2] * sRX.X();
    Jc.M05() = J_x_SX.m128_f32[1] * sRX.X() - J_x_SX.m128_f32[0] * sRX.Y();
    Jc.M06() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(J_x_SX, TX.XYZx()));

    Jc.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(J_y_SX, s);
    Jc.M13() = J_y_SX.m128_f32[2] * sRX.Y() - J_y_SX.m128_f32[1] * sRX.Z();
    Jc.M14() = J_y_SX.m128_f32[0] * sRX.Z() - J_y_SX.m128_f32[2] * sRX.X();
    Jc.M15() = J_y_SX.m128_f32[1] * sRX.X() - J_y_SX.m128_f32[0] * sRX.Y();
    Jc.M16() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(J_y_SX, TX.XYZx()));

    Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                       J_x_SX.m128_f32[0]), sR.M_00_01_02_x()),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_x_SX.m128_f32[1]), sR.M_10_11_12_x())),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_x_SX.m128_f32[2]), sR.M_20_21_22_x()));
    Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                       J_y_SX.m128_f32[0]), sR.M_00_01_02_x()),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_y_SX.m128_f32[1]), sR.M_10_11_12_x())),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_y_SX.m128_f32[2]), sR.M_20_21_22_x()));
}
static inline void Project(const ENFT_SSE::__m128 &s, const LA::AlignedMatrix3f &sR,
                           const Point3D &SX, const Camera &C, Point2D &x, LA::AlignedMatrix2x3f &Jx,
                           ENFT_SSE::__m128 *work3) {
    ENFT_SSE::__m128 &ZcI = work3[0];
    C.ProjectToNormalizedPlane(SX, ZcI.m128_f32[0], x.x(), x.y());
    ZcI = _mm_set1_ps(ZcI.m128_f32[0]);
    ENFT_SSE::__m128 &J_x_SX = work3[1], &J_y_SX = work3[2];
    J_x_SX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ZcI, C.r_00_01_02_x()),
                        ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI.m128_f32[0] * x.x()), C.r_20_21_22_x()));
    J_y_SX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ZcI, C.r_10_11_12_x()),
                        ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI.m128_f32[0] * x.y()), C.r_20_21_22_x()));
    Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                       J_x_SX.m128_f32[0]), sR.M_00_01_02_x()),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_x_SX.m128_f32[1]), sR.M_10_11_12_x())),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_x_SX.m128_f32[2]), sR.M_20_21_22_x()));
    Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                       J_y_SX.m128_f32[0]), sR.M_00_01_02_x()),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_y_SX.m128_f32[1]), sR.M_10_11_12_x())),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_y_SX.m128_f32[2]), sR.M_20_21_22_x()));
}
static inline void Project(const ENFT_SSE::__m128 &s, const LA::AlignedMatrix3f &sR,
                           const Point3D &sRX, const Point3D &TX, const Point3D &SX, const Camera &C,
                           const float &f,
                           const Point2D &r2x, float &ZcI, Point2D &x, LA::AlignedMatrix2x7f &Jc,
                           LA::AlignedMatrix2x3f &Jx, LA::AlignedMatrix2f &Jg, ENFT_SSE::__m128 *work3,
                           const bool &distortionInvalid) {
    C.ProjectToNormalizedPlane(SX, ZcI, x.x(), x.y());
    Jg.M00() = x.x();
    Jg.M10() = x.y();
    if(distortionInvalid) {
        Jg.M01() = 0.0f;
        Jg.M11() = 0.0f;
    } else {
        Jg.M01() = -r2x.x();
        Jg.M11() = -r2x.y();
    }
    x *= f;
    work3[0] = _mm_set1_ps(f * ZcI);
    ENFT_SSE::__m128 &J_x_SX = work3[1], &J_y_SX = work3[2];
    J_x_SX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_00_01_02_x()),
                        ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI * x.x()), C.r_20_21_22_x()));
    J_y_SX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_10_11_12_x()),
                        ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI * x.y()), C.r_20_21_22_x()));

    // S = {t, w, s}
    Jc.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(J_x_SX, s);
    Jc.M03() = J_x_SX.m128_f32[2] * sRX.Y() - J_x_SX.m128_f32[1] * sRX.Z();
    Jc.M04() = J_x_SX.m128_f32[0] * sRX.Z() - J_x_SX.m128_f32[2] * sRX.X();
    Jc.M05() = J_x_SX.m128_f32[1] * sRX.X() - J_x_SX.m128_f32[0] * sRX.Y();
    Jc.M06() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(J_x_SX, TX.XYZx()));

    Jc.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(J_y_SX, s);
    Jc.M13() = J_y_SX.m128_f32[2] * sRX.Y() - J_y_SX.m128_f32[1] * sRX.Z();
    Jc.M14() = J_y_SX.m128_f32[0] * sRX.Z() - J_y_SX.m128_f32[2] * sRX.X();
    Jc.M15() = J_y_SX.m128_f32[1] * sRX.X() - J_y_SX.m128_f32[0] * sRX.Y();
    Jc.M16() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(J_y_SX, TX.XYZx()));

    Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                       J_x_SX.m128_f32[0]), sR.M_00_01_02_x()),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_x_SX.m128_f32[1]), sR.M_10_11_12_x())),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_x_SX.m128_f32[2]), sR.M_20_21_22_x()));
    Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                       J_y_SX.m128_f32[0]), sR.M_00_01_02_x()),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_y_SX.m128_f32[1]), sR.M_10_11_12_x())),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_y_SX.m128_f32[2]), sR.M_20_21_22_x()));
}
static inline void Project(const ENFT_SSE::__m128 &s, const LA::AlignedMatrix3f &sR,
                           const Point3D &SX, const Camera &C, const float &f, const Point2D &r2x,
                           float &ZcI,
                           Point2D &x, LA::AlignedMatrix2x3f &Jx, LA::AlignedMatrix2f &Jg, ENFT_SSE::__m128 *work3,
                           const bool &distortionInvalid) {
    C.ProjectToNormalizedPlane(SX, ZcI, x.x(), x.y());
    Jg.M00() = x.x();
    Jg.M10() = x.y();
    if(distortionInvalid) {
        Jg.M01() = 0.0f;
        Jg.M11() = 0.0f;
    } else {
        Jg.M01() = -r2x.x();
        Jg.M11() = -r2x.y();
    }
    x *= f;
    work3[0] = _mm_set1_ps(f * ZcI);
    ENFT_SSE::__m128 &J_x_SX = work3[1], &J_y_SX = work3[2];
    J_x_SX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_00_01_02_x()),
                        ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI * x.x()), C.r_20_21_22_x()));
    J_y_SX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_10_11_12_x()),
                        ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI * x.y()), C.r_20_21_22_x()));
    Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                       J_x_SX.m128_f32[0]), sR.M_00_01_02_x()),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_x_SX.m128_f32[1]), sR.M_10_11_12_x())),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_x_SX.m128_f32[2]), sR.M_20_21_22_x()));
    Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                       J_y_SX.m128_f32[0]), sR.M_00_01_02_x()),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_y_SX.m128_f32[1]), sR.M_10_11_12_x())),
                                   ENFT_SSE::_mm_mul_ps(_mm_set1_ps(J_y_SX.m128_f32[2]), sR.M_20_21_22_x()));
}

#ifdef _DEBUG_WITH_EIGEN
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
EigenMatrix;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> EigenVector;
EigenMatrix g_JTJ, g_A;
EigenVector g_JTE, g_b, g_x;
static inline void Print(const EigenMatrix &M) {
    int r, c;
    const int nRows = int(M.rows()), nCols = int(M.cols());
    for(r = 0; r < nRows; ++r) {
        for(c = 0; c < nCols; ++c)
            printf("%f ", M(r, c));
        printf("\n");
    }
}
static inline void SetMc(const int iEq, const int iC,
                         const LA::AlignedMatrix2x7f &Msrc, EigenMatrix &Mdst) {
    int r, c;
    r = iEq;
    c = iC;
    Mdst(r, c++) = Msrc.M00();
    Mdst(r, c++) = Msrc.M01();
    Mdst(r, c++) = Msrc.M02();
    Mdst(r, c++) = Msrc.M03();
    Mdst(r, c++) = Msrc.M04();
    Mdst(r, c++) = Msrc.M05();
    Mdst(r, c++) = Msrc.M06();
    ++r;
    c = iC;
    Mdst(r, c++) = Msrc.M10();
    Mdst(r, c++) = Msrc.M11();
    Mdst(r, c++) = Msrc.M12();
    Mdst(r, c++) = Msrc.M13();
    Mdst(r, c++) = Msrc.M14();
    Mdst(r, c++) = Msrc.M15();
    Mdst(r, c++) = Msrc.M16();
}
static inline void SetMx(const int iEq, const int iX,
                         const LA::AlignedMatrix2x3f &Msrc, EigenMatrix &Mdst) {
    int r, c;
    r = iEq;
    c = iX;
    Mdst(r, c++) = Msrc.M00();
    Mdst(r, c++) = Msrc.M01();
    Mdst(r, c++) = Msrc.M02();
    ++r;
    c = iX;
    Mdst(r, c++) = Msrc.M10();
    Mdst(r, c++) = Msrc.M11();
    Mdst(r, c++) = Msrc.M12();
}
static inline void SetMg(const int iEq, const int iG,
                         const LA::AlignedMatrix2f &Msrc, EigenMatrix &Mdst) {
    int r, c;
    r = iEq;
    c = iG;
    Mdst(r, c++) = Msrc.M00();
    Mdst(r, c++) = Msrc.M01();
    ++r;
    c = iG;
    Mdst(r, c++) = Msrc.M10();
    Mdst(r, c++) = Msrc.M11();
}
static inline void SetVE(const int iEq, const Point2D &Vsrc,
                         EigenVector &Vdst) {
    int i = iEq;
    Vdst(i++) = Vsrc.x();
    Vdst(i++) = Vsrc.y();
}
static inline void SetMcc(const int iCr, const int iCc,
                          const LA::AlignedMatrix7f &Msrc, EigenMatrix &Mdst) {
    int r, c;
    r = iCr;
    c = iCc;
    Mdst(r, c++) = Msrc.M00();
    Mdst(r, c++) = Msrc.M01();
    Mdst(r, c++) = Msrc.M02();
    Mdst(r, c++) = Msrc.M03();
    Mdst(r, c++) = Msrc.M04();
    Mdst(r, c++) = Msrc.M05();
    Mdst(r, c++) = Msrc.M06();
    ++r;
    c = iCc;
    Mdst(r, c++) = Msrc.M10();
    Mdst(r, c++) = Msrc.M11();
    Mdst(r, c++) = Msrc.M12();
    Mdst(r, c++) = Msrc.M13();
    Mdst(r, c++) = Msrc.M14();
    Mdst(r, c++) = Msrc.M15();
    Mdst(r, c++) = Msrc.M16();
    ++r;
    c = iCc;
    Mdst(r, c++) = Msrc.M20();
    Mdst(r, c++) = Msrc.M21();
    Mdst(r, c++) = Msrc.M22();
    Mdst(r, c++) = Msrc.M23();
    Mdst(r, c++) = Msrc.M24();
    Mdst(r, c++) = Msrc.M25();
    Mdst(r, c++) = Msrc.M26();
    ++r;
    c = iCc;
    Mdst(r, c++) = Msrc.M30();
    Mdst(r, c++) = Msrc.M31();
    Mdst(r, c++) = Msrc.M32();
    Mdst(r, c++) = Msrc.M33();
    Mdst(r, c++) = Msrc.M34();
    Mdst(r, c++) = Msrc.M35();
    Mdst(r, c++) = Msrc.M36();
    ++r;
    c = iCc;
    Mdst(r, c++) = Msrc.M40();
    Mdst(r, c++) = Msrc.M41();
    Mdst(r, c++) = Msrc.M42();
    Mdst(r, c++) = Msrc.M43();
    Mdst(r, c++) = Msrc.M44();
    Mdst(r, c++) = Msrc.M45();
    Mdst(r, c++) = Msrc.M46();
    ++r;
    c = iCc;
    Mdst(r, c++) = Msrc.M50();
    Mdst(r, c++) = Msrc.M51();
    Mdst(r, c++) = Msrc.M52();
    Mdst(r, c++) = Msrc.M53();
    Mdst(r, c++) = Msrc.M54();
    Mdst(r, c++) = Msrc.M55();
    Mdst(r, c++) = Msrc.M56();
    ++r;
    c = iCc;
    Mdst(r, c++) = Msrc.M60();
    Mdst(r, c++) = Msrc.M61();
    Mdst(r, c++) = Msrc.M62();
    Mdst(r, c++) = Msrc.M63();
    Mdst(r, c++) = Msrc.M64();
    Mdst(r, c++) = Msrc.M65();
    Mdst(r, c++) = Msrc.M66();
    if(iCr == iCc)
        return;
    Mdst.block<7, 7>(iCc, iCr) = Mdst.block<7, 7>(iCr, iCc).transpose();
}
static inline void SetMxx(const int iXr, const int iXc,
                          const LA::SymmetricMatrix3f &Msrc, EigenMatrix &Mdst) {
    int r, c;
    r = iXr;
    c = iXc;
    Mdst(r, c++) = Msrc.M00();
    Mdst(r, c++) = Msrc.M01();
    Mdst(r, c++) = Msrc.M02();
    ++r;
    c = iXc;
    Mdst(r, c++) = Msrc.M10();
    Mdst(r, c++) = Msrc.M11();
    Mdst(r, c++) = Msrc.M12();
    ++r;
    c = iXc;
    Mdst(r, c++) = Msrc.M20();
    Mdst(r, c++) = Msrc.M21();
    Mdst(r, c++) = Msrc.M22();
    if(iXr == iXc)
        return;
    Mdst.block<3, 3>(iXc, iXr) = Mdst.block<3, 3>(iXr, iXc).transpose();
}
static inline void SetMgg(const int iGr, const int iGc,
                          const LA::AlignedMatrix2f &Msrc, EigenMatrix &Mdst) {
    int r, c;
    r = iGr;
    c = iGc;
    Mdst(r, c++) = Msrc.M00();
    Mdst(r, c++) = Msrc.M01();
    ++r;
    c = iGc;
    Mdst(r, c++) = Msrc.M10();
    Mdst(r, c++) = Msrc.M11();
    if(iGr == iGc)
        return;
    Mdst.block<2, 2>(iGc, iGr) = Mdst.block<2, 2>(iGr, iGc).transpose();
}
static inline void SetMcx(const int iC, const int iX,
                          const LA::AlignedMatrix3x7f Msrc, EigenMatrix &Mdst) {
    int r, c;
    r = iX;
    c = iC;
    Mdst(r, c++) = Msrc.M00();
    Mdst(r, c++) = Msrc.M01();
    Mdst(r, c++) = Msrc.M02();
    Mdst(r, c++) = Msrc.M03();
    Mdst(r, c++) = Msrc.M04();
    Mdst(r, c++) = Msrc.M05();
    Mdst(r, c++) = Msrc.M06();
    ++r;
    c = iC;
    Mdst(r, c++) = Msrc.M10();
    Mdst(r, c++) = Msrc.M11();
    Mdst(r, c++) = Msrc.M12();
    Mdst(r, c++) = Msrc.M13();
    Mdst(r, c++) = Msrc.M14();
    Mdst(r, c++) = Msrc.M15();
    Mdst(r, c++) = Msrc.M16();
    ++r;
    c = iC;
    Mdst(r, c++) = Msrc.M20();
    Mdst(r, c++) = Msrc.M21();
    Mdst(r, c++) = Msrc.M22();
    Mdst(r, c++) = Msrc.M23();
    Mdst(r, c++) = Msrc.M24();
    Mdst(r, c++) = Msrc.M25();
    Mdst(r, c++) = Msrc.M26();
    Mdst.block<7, 3>(iC, iX) = Mdst.block<3, 7>(iX, iC).transpose();
    //printf("----------------------------------------------------------------\n");
    //Msrc.Print();
    //printf("----------------------------------------------------------------\n");
    //Print(Mdst.block<7, 3>(iC, iX));
    //printf("----------------------------------------------------------------\n");
    //Print(Mdst.block<3, 7>(iX, iC));
}
static inline void SetMcg(const int iC, const int iG,
                          const LA::AlignedMatrix2x7f &Msrc, EigenMatrix &Mdst) {
    int r, c;
    r = iG;
    c = iC;
    Mdst(r, c++) = Msrc.M00();
    Mdst(r, c++) = Msrc.M01();
    Mdst(r, c++) = Msrc.M02();
    Mdst(r, c++) = Msrc.M03();
    Mdst(r, c++) = Msrc.M04();
    Mdst(r, c++) = Msrc.M05();
    Mdst(r, c++) = Msrc.M06();
    ++r;
    c = iC;
    Mdst(r, c++) = Msrc.M10();
    Mdst(r, c++) = Msrc.M11();
    Mdst(r, c++) = Msrc.M12();
    Mdst(r, c++) = Msrc.M13();
    Mdst(r, c++) = Msrc.M14();
    Mdst(r, c++) = Msrc.M15();
    Mdst(r, c++) = Msrc.M16();
    Mdst.block<7, 2>(iC, iG) = Mdst.block<2, 7>(iG, iC).transpose();
}
static inline void SetMxg(const int iX, const int iG,
                          const LA::AlignedMatrix2x3f &Msrc, EigenMatrix &Mdst) {
    int r, c;
    r = iG;
    c = iX;
    Mdst(r, c++) = Msrc.M00();
    Mdst(r, c++) = Msrc.M01();
    Mdst(r, c++) = Msrc.M02();
    ++r;
    c = iX;
    Mdst(r, c++) = Msrc.M10();
    Mdst(r, c++) = Msrc.M11();
    Mdst(r, c++) = Msrc.M12();
    Mdst.block<3, 2>(iX, iG) = Mdst.block<2, 3>(iG, iX).transpose();
}
static inline void SetVc(const int iC, const LA::AlignedVector7f &Vsrc,
                         EigenVector &Vdst) {
    int i = iC;
    Vdst(i++) = Vsrc.v0();
    Vdst(i++) = Vsrc.v1();
    Vdst(i++) = Vsrc.v2();
    Vdst(i++) = Vsrc.v3();
    Vdst(i++) = Vsrc.v4();
    Vdst(i++) = Vsrc.v5();
    Vdst(i++) = Vsrc.v6();
}
static inline void SetVx(const int iX, const LA::Vector3f &Vsrc,
                         EigenVector &Vdst) {
    int i = iX;
    Vdst(i++) = Vsrc.v0();
    Vdst(i++) = Vsrc.v1();
    Vdst(i++) = Vsrc.v2();
}
static inline void SetVg(const int iG, const LA::Vector2f &Vsrc,
                         EigenVector &Vdst) {
    int i = iG;
    Vdst(i++) = Vsrc.v0();
    Vdst(i++) = Vsrc.v1();
}
static inline void GetIndex(const int iC, const int iX, const int iG,
                            const int p, int &i, int &j, char &s) {
    if(p < iX) {
        s = 'C';
        i = p / 7;
        j = p % 7;
    } else if(p < iG) {
        s = 'X';
        i = (p - iX) / 3;
        j = (p - iX) % 3;
    } else {
        s = 'G';
        i = 0;
        j = p - iG;
    }
}
static inline void CheckMcxg(const int iC, const int iX, const int iG,
                             const EigenMatrix &M1, const EigenMatrix &M2) {
    int r, c, ir, jr, ic, jc;
    char sr, sc;
    float v1, v2;
    const int nRows = int(M1.rows()), nCols = int(M1.cols());
    for(r = 0; r < nRows; ++r)
        for(c = 0; c < nCols; ++c) {
            v1 = M1(r, c);
            v2 = M2(r, c);
            if(EQUAL(v1, v2))
                continue;
            GetIndex(iC, iX, iG, r, ir, jr, sr);
            GetIndex(iC, iX, iG, c, ic, jc, sc);
            printf("(%c%d, %c%d)(%d, %d): %f - %f = %f\n", sr, ir, sc, ic, jr, jc, v1, v2,
                   v1 - v2);
        }
}
static inline void CheckVcxg(const int iC, const int iX, const int iG,
                             const EigenVector &V1, const EigenVector &V2) {
    int r, i, j;
    char s;
    float v1, v2;
    const int nRows = int(V1.rows());
    for(r = 0; r < nRows; ++r) {
        v1 = V1(r);
        v2 = V2(r);
        if(EQUAL(v1, v2))
            continue;
        GetIndex(iC, iX, iG, r, i, j, s);
        printf("(%c%d)(%d): %f - %f = %f\n", s, i, j, v1, v2, v1 - v2);
    }
}
#endif

void SegmentSetBundleAdjustorData2DSimilarity::ConstructNormalEquation(
    AlignedVector<LA::AlignedMatrix7f> &Dcs,
    AlignedVector<LA::SymmetricMatrix3f> &Dxs,
    LA::AlignedMatrix2f &Dg, AlignedVector<LA::AlignedMatrix3x7f> &Wxcs,
    AlignedVector<LA::AlignedMatrix2x7f> &Wgcs,
    AlignedVector<LA::AlignedMatrix2x3f> &Wgxs,
    AlignedVector<LA::AlignedVector7f> &bcs, AlignedVector<LA::Vector3f> &bxs,
    LA::Vector2f &bg,
    AlignedVector<LA::AlignedVector7f> &scs, AlignedVector<LA::Vector3f> &sxs,
    LA::Vector2f &sg) {
#ifdef _DEBUG_WITH_EIGEN
    const int nEqs = int(m_xs.Size() * 2),
              nVars = int(scs.Size()) * 7 + int(sxs.Size()) * 3 + 2;
    const int iC = 0, iX = int(scs.Size()) * 7,
              iG = int(scs.Size()) * 7 + int(sxs.Size()) * 3;
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
    EigenMatrix J;
    EigenVector E;
    int iEq = 0;
    J.resize(nEqs, nVars);
    J.setZero();
    E.resize(nEqs);
    E.setZero();
    Wxcs.SetZero();
#endif
#endif

    const bool global = IsGlobalValid();
    if(global)
        UndistortMeasurements(m_G);

    Dcs.SetZero();
    Dxs.SetZero();
    Dg.SetZero();
    Wgcs.SetZero();
    Wgxs.SetZero();
    Wxcs.SetZero();
    bcs.SetZero();
    bxs.SetZero();
    bg.SetZero();
    scs.SetZero();
    sxs.SetZero();
    sg.SetZero();

    SegmentIndex iSeg;
    SequenceIndex iSeq;
    FrameIndex iFrm;
    TrackIndex iTrk;
    MeasurementIndex iMea;
    float ZcI;
    LA::AlignedMatrix3f sR;
    Point3D sRX, TX, SX;
    Point2D xp, e;
    LA::AlignedMatrix2x7f Jc;
    LA::AlignedMatrix2x3f Jx;
    LA::AlignedMatrix2f Jg;
    ENFT_SSE::__m128 work[3];
    const float f = m_G.f();
    const SegmentIndex nSegs = SegmentIndex(m_Cs.Size()),
                       nSegsFix = nSegs - SegmentIndex(scs.Size());
    for(iSeg = 0; iSeg < nSegsFix; ++iSeg) {
        const SimilarityTransformation3D &S = m_Cs[iSeg];
        const ENFT_SSE::__m128 &s = S.sss1();
        sR.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(s, S.r_00_01_02_x());
        sR.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(s, S.r_10_11_12_x());
        sR.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(s, S.r_20_21_22_x());

        const MeasurementIndex iMea1 = m_mapSegToMea[iSeg],
                               iMea2 = m_mapSegToMea[iSeg + 1];
        for(iMea = iMea1, iTrk = INVALID_TRACK_INDEX; iMea < iMea2; ++iMea) {
            if(m_mapMeaToTrk[iMea] != iTrk) {
                iTrk = m_mapMeaToTrk[iMea];
                Project(S, m_Xs[iTrk], SX);
            }
            m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
            if(global) {
                Project(s, sR, SX, m_pSeqs[iSeq]->GetCamera(iFrm), f, m_r2xs[iMea], ZcI, xp, Jx,
                        Jg, work, m_distortionInvalid);
                LA::AmB(m_xus[iMea], xp, e);
                LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
                LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
                LA::AddAij2To(Jx, sxs[iTrk], work[0]);
                LA::AddATAToUpper(Jg, Dg, work[0]);
                LA::AddATBTo(Jg, Jx, Wgxs[iTrk]);
                LA::AddATBTo(Jg, e, bg, work[0]);
                LA::AddAij2To(Jg, sg, work[0]);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
                SetMx(iEq, iX + iTrk * 3, Jx, J);
                SetMg(iEq, iG, Jg, J);
                SetVE(iEq, e, E);
                iEq += 2;
#endif
            } else {
                Project(s, sR, SX, m_pSeqs[iSeq]->GetCamera(iFrm), xp, Jx, work);
                LA::AmB(m_xs[iMea], xp, e);
                LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
                LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
                LA::AddAij2To(Jx, sxs[iTrk], work[0]);
            }
        }
    }
    SegmentIndex i;
    MeasurementIndex j;
    for(i = 0, j = INVALID_MEASUREMENT_INDEX; iSeg < nSegs; ++iSeg, ++i) {
        const SimilarityTransformation3D &S = m_Cs[iSeg];
        LA::AlignedMatrix7f &Dc = Dcs[i];
        LA::AlignedVector7f &bc = bcs[i], &sc = scs[i];
        const ENFT_SSE::__m128 &s = S.sss1();
        sR.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(s, S.r_00_01_02_x());
        sR.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(s, S.r_10_11_12_x());
        sR.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(s, S.r_20_21_22_x());

        const MeasurementIndex iMea1 = m_mapSegToMea[iSeg],
                               iMea2 = m_mapSegToMea[iSeg + 1];
        for(iMea = iMea1, iTrk = INVALID_TRACK_INDEX; iMea < iMea2; ++iMea) {
            if(m_mapMeaToTrk[iMea] != iTrk) {
                iTrk = m_mapMeaToTrk[iMea];
                Project(S, m_Xs[iTrk], sRX, TX, SX);
                ++j;
            }
            m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
            if(global) {
                Project(s, sR, sRX, TX, SX, m_pSeqs[iSeq]->GetCamera(iFrm), f, m_r2xs[iMea],
                        ZcI, xp, Jc, Jx, Jg, work, m_distortionInvalid);
                LA::AmB(m_xus[iMea], xp, e);
                LA::AddATAToUpper(Jc, Dc, work);
                LA::AddATBTo(Jc, e, bc, work);
                LA::AddAij2To(Jc, sc);
                LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
                LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
                LA::AddAij2To(Jx, sxs[iTrk], work[0]);
                LA::AddATBTo(Jx, Jc, Wxcs[j], work);
                LA::AddATAToUpper(Jg, Dg, work[0]);
                LA::AddATBTo(Jg, Jc, Wgcs[i], work);
                LA::AddATBTo(Jg, Jx, Wgxs[iTrk]);
                LA::AddATBTo(Jg, e, bg, work[0]);
                LA::AddAij2To(Jg, sg, work[0]);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
                SetMc(iEq, iC + i * 7, Jc, J);
                SetMx(iEq, iX + iTrk * 3, Jx, J);
                SetMg(iEq, iG, Jg, J);
                SetVE(iEq, e, E);
                iEq += 2;
#endif
            } else {
                Project(s, sR, sRX, TX, SX, m_pSeqs[iSeq]->GetCamera(iFrm), xp, Jc, Jx, work);
                LA::AmB(m_xs[iMea], xp, e);
                LA::AddATAToUpper(Jc, Dc, work);
                LA::AddATBTo(Jc, e, bc, work);
                LA::AddAij2To(Jc, sc);
                LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
                LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
                LA::AddAij2To(Jx, sxs[iTrk], work[0]);
                LA::AddATBTo(Jx, Jc, Wxcs[j], work);
            }
        }
        LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x7f>(Dc);
        LA::SetLowerFromUpper(Dc);
        LA::FinishAdditionATBTo<LA::AlignedMatrix2x7f, LA::Vector2f>(bc);
        LA::SetReserve<BA_STAGE_B>(bc);
        LA::FinishAdditionAij2To<LA::AlignedMatrix2x7f>(sc);
        LA::MakeReciprocal(sc);
        LA::MakeSquareRoot(sc);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
        sc.v0123() = sc.v456x() = _mm_set1_ps(1.0f);
#endif
        LA::SetReserve<BA_STAGE_S>(sc);
        LA::ssTA(sc, Dc, work[0]);
        LA::sA(sc, bc);
        //printf("----------------------------------------------------------------\n");
        //printf("%d\n", i);
        //Dc.Print();
    }
    const TrackIndex nTrks = TrackIndex(m_Xs.Size());
    for(iTrk = 0; iTrk < nTrks; ++iTrk) {
        LA::SymmetricMatrix3f &Dx = Dxs[iTrk];
        LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x3f>(Dx);
        LA::SetLowerFromUpper(Dx);
        LA::Vector3f &bx = bxs[iTrk];
        LA::FinishAdditionATBTo<LA::AlignedMatrix2x3f, LA::Vector2f>(bx);
        LA::Vector3f &sx = sxs[iTrk];
        LA::FinishAdditionAij2To<LA::AlignedMatrix2x3f>(sx);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
        sx.v0() = sx.v1() = sx.v2() = 1.0f;
#endif
        LA::MakeReciprocal(sx);
        LA::MakeSquareRoot(sx);
        LA::SetReserve<BA_STAGE_S>(sx);
        LA::ssTA(sx, Dx);
        LA::sA(sx, bx);
        LA::SetReserve<BA_STAGE_B>(bx);
    }
    for(iSeg = nSegsFix, i = 0, j = INVALID_MEASUREMENT_INDEX; iSeg < nSegs;
            ++iSeg, ++i) {
        const LA::AlignedVector7f &sc = scs[i];
        const MeasurementIndex iMea1 = m_mapSegToMea[iSeg],
                               iMea2 = m_mapSegToMea[iSeg + 1];
        for(iMea = iMea1, iTrk = INVALID_TRACK_INDEX; iMea < iMea2; ++iMea) {
            if(m_mapMeaToTrk[iMea] == iTrk)
                continue;
            iTrk = m_mapMeaToTrk[iMea];
            LA::s1s2TA(sxs[iTrk], sc, Wxcs[++j], work[0]);
        }
    }
    if(global) {
        LA::FinishAdditionATAToUpper<LA::AlignedMatrix2f>(Dg);
        LA::SetLowerFromUpper(Dg);
        LA::FinishAdditionATBTo<LA::AlignedMatrix2f, LA::Vector2f>(bg);
        LA::SetReserve<BA_STAGE_B>(bg);
        LA::FinishAdditionAij2To<LA::AlignedMatrix2f>(sg);
        LA::MakeReciprocal(sg);
        LA::MakeSquareRoot(sg);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
        sg.v0() = sg.v1() = 1.0f;
#endif
        LA::SetReserve<BA_STAGE_S>(sg);
        LA::ssTA(sg, Dg);
        LA::sA(sg, bg);
        for(iSeg = nSegsFix, i = 0; iSeg < nSegs; ++iSeg, ++i)
            LA::s1s2TA(sg, scs[i], Wgcs[i], work[0]);
        for(iTrk = 0; iTrk < nTrks; ++iTrk)
            LA::s1s2TA(sg, sxs[iTrk], Wgxs[iTrk], work[0]);
    }

#ifdef _DEBUG_WITH_EIGEN
    g_JTJ.resize(nVars, nVars);
    g_JTJ.setZero();
    g_JTE.resize(nVars);
    g_JTE.setZero();
    for(iSeg = nSegsFix, i = 0, j = INVALID_MEASUREMENT_INDEX; iSeg < nSegs;
            ++iSeg, ++i) {
        SetMcc(iC + i * 7, iC + i * 7, Dcs[i], g_JTJ);
        SetVc(iC + i * 7, bcs[i], g_JTE);
        const MeasurementIndex iMea1 = m_mapSegToMea[iSeg],
                               iMea2 = m_mapSegToMea[iSeg + 1];
        for(iMea = iMea1, iTrk = INVALID_TRACK_INDEX; iMea < iMea2; ++iMea) {
            if(m_mapMeaToTrk[iMea] == iTrk)
                continue;
            iTrk = m_mapMeaToTrk[iMea];
            SetMcx(iC + i * 7, iX + iTrk * 3, Wxcs[++j], g_JTJ);
        }
        SetMcg(iC + i * 7, iG, Wgcs[i], g_JTJ);
    }
    for(iTrk = 0; iTrk < nTrks; ++iTrk) {
        SetMxx(iX + iTrk * 3, iX + iTrk * 3, Dxs[iTrk], g_JTJ);
        SetVx(iX + iTrk * 3, bxs[iTrk], g_JTE);
        SetMxg(iX + iTrk * 3, iG, Wgxs[iTrk], g_JTJ);
    }
    SetMgg(iG, iG, Dg, g_JTJ);
    SetVg(iG, bg, g_JTE);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
    //PrintA(iC, iX, iG, J);
    const EigenMatrix JT = J.transpose();
    const EigenMatrix JTJ1 = JT * J, &JTJ2 = g_JTJ;
    const EigenMatrix JTE1 = JT * E, &JTE2 = g_JTE;
    CheckMcxg(iC, iX, iG, JTJ1, JTJ2);
    CheckVcxg(iC, iX, iG, JTE1, JTE2);
#endif
#endif
}

void SegmentSetBundleAdjustorData2DSimilarity::ConstructNormalEquation(
    const AlignedVector<LA::AlignedVector7f> &scs,
    const AlignedVector<LA::Vector3f> &sxs,
    const LA::Vector2f &sg, AlignedVector<LA::AlignedMatrix7f> &Dcs,
    AlignedVector<LA::SymmetricMatrix3f> &Dxs, LA::AlignedMatrix2f &Dg,
    AlignedVector<LA::AlignedMatrix3x7f> &Wxcs,
    AlignedVector<LA::AlignedMatrix2x7f> &Wgcs,
    AlignedVector<LA::AlignedMatrix2x3f> &Wgxs,
    AlignedVector<LA::AlignedVector7f> &bcs,
    AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg) {
#ifdef _DEBUG_WITH_EIGEN
    const int nEqs = int(m_xs.Size() * 2),
              nVars = int(scs.Size()) * 7 + int(sxs.Size()) * 3 + 2;
    const int iC = 0, iX = int(scs.Size()) * 7,
              iG = int(scs.Size()) * 7 + int(sxs.Size()) * 3;
#endif

    const bool global = IsGlobalValid();
    if(global)
        UndistortMeasurements(m_G);

    Dcs.SetZero();
    Dxs.SetZero();
    Dg.SetZero();
    Wgcs.SetZero();
    Wgxs.SetZero();
    Wxcs.SetZero();
    bcs.SetZero();
    bxs.SetZero();
    bg.SetZero();

    SegmentIndex iSeg;
    SequenceIndex iSeq;
    FrameIndex iFrm;
    TrackIndex iTrk;
    MeasurementIndex iMea;
    float ZcI;
    LA::AlignedMatrix3f sR;
    Point3D sRX, TX, SX;
    Point2D xp, e;
    LA::AlignedMatrix2x7f Jc;
    LA::AlignedMatrix2x3f Jx;
    LA::AlignedMatrix2f Jg;
    ENFT_SSE::__m128 work[3];
    const float f = m_G.f();
    const SegmentIndex nSegs = SegmentIndex(m_Cs.Size()),
                       nSegsFix = nSegs - SegmentIndex(scs.Size());
    for(iSeg = 0; iSeg < nSegsFix; ++iSeg) {
        const SimilarityTransformation3D &S = m_Cs[iSeg];
        const ENFT_SSE::__m128 &s = S.sss1();
        sR.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(s, S.r_00_01_02_x());
        sR.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(s, S.r_10_11_12_x());
        sR.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(s, S.r_20_21_22_x());

        const MeasurementIndex iMea1 = m_mapSegToMea[iSeg],
                               iMea2 = m_mapSegToMea[iSeg + 1];
        for(iMea = iMea1, iTrk = INVALID_TRACK_INDEX; iMea < iMea2; ++iMea) {
            if(m_mapMeaToTrk[iMea] != iTrk) {
                iTrk = m_mapMeaToTrk[iMea];
                Project(S, m_Xs[iTrk], SX);
            }
            m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
            if(global) {
                Project(s, sR, SX, m_pSeqs[iSeq]->GetCamera(iFrm), f, m_r2xs[iMea], ZcI, xp, Jx,
                        Jg, work, m_distortionInvalid);
                LA::AmB(m_xus[iMea], xp, e);
                LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
                LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
                LA::AddATAToUpper(Jg, Dg, work[0]);
                LA::AddATBTo(Jg, Jx, Wgxs[iTrk]);
                LA::AddATBTo(Jg, e, bg, work[0]);
            } else {
                Project(s, sR, SX, m_pSeqs[iSeq]->GetCamera(iFrm), xp, Jx, work);
                LA::AmB(m_xs[iMea], xp, e);
                LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
                LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
            }
        }
    }
    SegmentIndex i;
    MeasurementIndex j;
    for(i = 0, j = INVALID_MEASUREMENT_INDEX; iSeg < nSegs; ++iSeg, ++i) {
        const SimilarityTransformation3D &S = m_Cs[iSeg];
        LA::AlignedMatrix7f &Dc = Dcs[i];
        LA::AlignedVector7f &bc = bcs[i];
        const ENFT_SSE::__m128 &s = S.sss1();
        sR.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(s, S.r_00_01_02_x());
        sR.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(s, S.r_10_11_12_x());
        sR.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(s, S.r_20_21_22_x());

        const MeasurementIndex iMea1 = m_mapSegToMea[iSeg],
                               iMea2 = m_mapSegToMea[iSeg + 1];
        for(iMea = iMea1, iTrk = INVALID_TRACK_INDEX; iMea < iMea2; ++iMea) {
            if(m_mapMeaToTrk[iMea] != iTrk) {
                iTrk = m_mapMeaToTrk[iMea];
                Project(S, m_Xs[iTrk], sRX, TX, SX);
                ++j;
            }
            m_mapMeaToSeqFrm[iMea].Get(iSeq, iFrm);
            if(global) {
                Project(s, sR, sRX, TX, SX, m_pSeqs[iSeq]->GetCamera(iFrm), f, m_r2xs[iMea],
                        ZcI, xp, Jc, Jx, Jg, work, m_distortionInvalid);
                LA::AmB(m_xus[iMea], xp, e);
                LA::AddATAToUpper(Jc, Dc, work);
                LA::AddATBTo(Jc, e, bc, work);
                LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
                LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
                LA::AddATBTo(Jx, Jc, Wxcs[j], work);
                LA::AddATAToUpper(Jg, Dg, work[0]);
                LA::AddATBTo(Jg, Jc, Wgcs[i], work);
                LA::AddATBTo(Jg, Jx, Wgxs[iTrk]);
                LA::AddATBTo(Jg, e, bg, work[0]);
            } else {
                Project(s, sR, sRX, TX, SX, m_pSeqs[iSeq]->GetCamera(iFrm), xp, Jc, Jx, work);
                LA::AmB(m_xs[iMea], xp, e);
                LA::AddATAToUpper(Jc, Dc, work);
                LA::AddATBTo(Jc, e, bc, work);
                LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
                LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
                LA::AddATBTo(Jx, Jc, Wxcs[j], work);
            }
        }
        LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x7f>(Dc);
        LA::SetLowerFromUpper(Dc);
        LA::FinishAdditionATBTo<LA::AlignedMatrix2x7f, LA::Vector2f>(bc);
        LA::SetReserve<BA_STAGE_B>(bc);
        const LA::AlignedVector7f &sc = scs[i];
        LA::ssTA(sc, Dc, work[0]);
        LA::sA(sc, bc);
        //printf("----------------------------------------------------------------\n");
        //printf("%d\n", i);
        //Dc.Print();
    }
    const TrackIndex nTrks = TrackIndex(m_Xs.Size());
    for(iTrk = 0; iTrk < nTrks; ++iTrk) {
        LA::SymmetricMatrix3f &Dx = Dxs[iTrk];
        LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x3f>(Dx);
        LA::SetLowerFromUpper(Dx);
        LA::Vector3f &bx = bxs[iTrk];
        LA::FinishAdditionATBTo<LA::AlignedMatrix2x3f, LA::Vector2f>(bx);
        const LA::Vector3f &sx = sxs[iTrk];
        LA::ssTA(sx, Dx);
        LA::sA(sx, bx);
        LA::SetReserve<BA_STAGE_B>(bx);
    }
    for(iSeg = nSegsFix, i = 0, j = INVALID_MEASUREMENT_INDEX; iSeg < nSegs;
            ++iSeg, ++i) {
        const LA::AlignedVector7f &sc = scs[i];
        const MeasurementIndex iMea1 = m_mapSegToMea[iSeg],
                               iMea2 = m_mapSegToMea[iSeg + 1];
        for(iMea = iMea1, iTrk = INVALID_TRACK_INDEX; iMea < iMea2; ++iMea) {
            if(m_mapMeaToTrk[iMea] == iTrk)
                continue;
            iTrk = m_mapMeaToTrk[iMea];
            LA::s1s2TA(sxs[iTrk], sc, Wxcs[++j], work[0]);
        }
    }
    if(global) {
        LA::FinishAdditionATAToUpper<LA::AlignedMatrix2f>(Dg);
        LA::SetLowerFromUpper(Dg);
        LA::FinishAdditionATBTo<LA::AlignedMatrix2f, LA::Vector2f>(bg);
        LA::SetReserve<BA_STAGE_B>(bg);
        LA::ssTA(sg, Dg);
        LA::sA(sg, bg);
        for(iSeg = nSegsFix, i = 0; iSeg < nSegs; ++iSeg, ++i)
            LA::s1s2TA(sg, scs[i], Wgcs[i], work[0]);
        for(iTrk = 0; iTrk < nTrks; ++iTrk)
            LA::s1s2TA(sg, sxs[iTrk], Wgxs[iTrk], work[0]);
    }

#ifdef _DEBUG_WITH_EIGEN
    g_JTJ.resize(nVars, nVars);
    g_JTJ.setZero();
    g_JTE.resize(nVars);
    g_JTE.setZero();
    for(iSeg = nSegsFix, i = 0, j = INVALID_MEASUREMENT_INDEX; iSeg < nSegs;
            ++iSeg, ++i) {
        SetMcc(iC + i * 7, iC + i * 7, Dcs[i], g_JTJ);
        SetVc(iC + i * 7, bcs[i], g_JTE);
        const MeasurementIndex iMea1 = m_mapSegToMea[iSeg],
                               iMea2 = m_mapSegToMea[iSeg + 1];
        for(iMea = iMea1, iTrk = INVALID_TRACK_INDEX; iMea < iMea2; ++iMea) {
            if(m_mapMeaToTrk[iMea] == iTrk)
                continue;
            iTrk = m_mapMeaToTrk[iMea];
            SetMcx(iC + i * 7, iX + iTrk * 3, Wxcs[++j], g_JTJ);
        }
        SetMcg(iC + i * 7, iG, Wgcs[i], g_JTJ);
    }
    for(iTrk = 0; iTrk < nTrks; ++iTrk) {
        SetMxx(iX + iTrk * 3, iX + iTrk * 3, Dxs[iTrk], g_JTJ);
        SetVx(iX + iTrk * 3, bxs[iTrk], g_JTE);
        SetMxg(iX + iTrk * 3, iG, Wgxs[iTrk], g_JTJ);
    }
    SetMgg(iG, iG, Dg, g_JTJ);
    SetVg(iG, bg, g_JTE);
#endif
}

void SegmentSetBundleAdjustorData2DSimilarity::UpdateCameras(
    const AlignedVector<LA::AlignedVector7f> &scs,
    const AlignedVector<LA::AlignedVector7f> &xcs,
    const AlignedVector<SimilarityTransformation3D> &SsOld) {
    RotationTransformation3D dR;
    ENFT_SSE::__m128 workm[3];
    float workf[24];
    const SequenceIndex nSeqs = SequenceIndex(m_Cs.Size()),
                        nSeqsFix = nSeqs - FrameIndex(scs.Size());
    for(SequenceIndex iSeq = nSeqsFix, i = 0; iSeq < nSeqs; ++iSeq, ++i) {
        const LA::AlignedVector7f &sc = scs[i], &xc = xcs[i];
        const SimilarityTransformation3D &Sold = SsOld[iSeq];
        SimilarityTransformation3D &Snew = m_Cs[iSeq];
        workm[0] = ENFT_SSE::_mm_mul_ps(sc.v0123(), xc.v0123());
        workm[1] = ENFT_SSE::_mm_mul_ps(sc.v456x(), xc.v456x());
        dR.FromRodrigues(workm[0].m128_f32[3], workm[1].m128_f32[0],
                         workm[1].m128_f32[1], workf);
        Sold.LeftMultiplyRotation(dR, Snew, workm[2]);
        Snew.tX() = workm[0].m128_f32[0] + Sold.tX();
        Snew.tY() = workm[0].m128_f32[1] + Sold.tY();
        Snew.tZ() = workm[0].m128_f32[2] + Sold.tZ();
        Snew.SetScale(Sold.s() + workm[1].m128_f32[2]);
    }
}

void SegmentSetBundleAdjustorData2DSimilarity::UpdatePoints(
    const AlignedVector<LA::Vector3f> &sxs, const AlignedVector<LA::Vector3f> &xxs,
    const AlignedVector<Point3D> &XsOld) {
    const TrackIndex nTrks = TrackIndex(m_Xs.Size());
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        const LA::Vector3f &sx = sxs[iTrk], &xx = xxs[iTrk];
        const Point3D &Xold = XsOld[iTrk];
        Point3D &Xnew = m_Xs[iTrk];
        Xnew.X() = sx.v0() * xx.v0() + Xold.X();
        Xnew.Y() = sx.v1() * xx.v1() + Xold.Y();
        Xnew.Z() = sx.v2() * xx.v2() + Xold.Z();
    }
}

void SegmentSetBundleAdjustorData2DSimilarity::UpdateGlobal(
    const LA::Vector2f &sg, const LA::Vector2f &xg,
    const Camera::IntrinsicParameter &Gold) {
#if _DEBUG
    assert(IsGlobalValid());
#endif
    m_G.f() = Gold.f() + sg.v0() * xg.v0();
    m_G.d() = Gold.d() + sg.v1() * xg.v1();
}

#if _DEBUG
void SegmentSetBundleAdjustorData2DSimilarity::DebugSchurComplement(
    const float damping, const std::vector<std::pair<SegmentIndex, SegmentIndex> >
    &iPairs,
    const AlignedVector<LA::AlignedMatrix7f> &Accs, const LA::AlignedMatrix2f &Agg,
    const AlignedVector<LA::AlignedMatrix2x7f> &Agcs,
    const AlignedVector<LA::AlignedVector7f> &bcs,
    const LA::Vector2f &bg) {
#ifdef _DEBUG_WITH_EIGEN
    const int nC = int(bcs.Size()) * 7, nX = int(m_Xs.Size()) * 3, nG = 2,
              nVars = nC + nX + nG;
    const int iC = 0, iX = nC, iG = nC + nX;

    int i;
    g_A.resize(nVars, nVars);
    g_A.setZero();
    g_b.resize(nVars);
    g_b.setZero();
    const int nPairs = int(iPairs.size());
    for(i = 0; i < nPairs; ++i)
        SetMcc(iC + iPairs[i].first * 7, iC + iPairs[i].second * 7, Accs[i], g_A);
    SetMgg(iG, iG, Agg, g_A);
    const int nSegsAdj = int(bcs.Size());
    for(i = 0; i < nSegsAdj; ++i) {
        SetMcg(iC + i * 7, iG, Agcs[i], g_A);
        SetVc(iC + i * 7, bcs[i], g_b);
    }
    SetVg(iG, bg, g_b);

    const float lambda = damping + 1;
    EigenMatrix Dc = g_JTJ.block(iC, iC, nC, nC), Dx = g_JTJ.block(iX, iX, nX, nX),
                Dg = g_JTJ.block(iG, iG, nG, nG);
    for(i = 0; i < nC; ++i)
        Dc(i, i) *= lambda;
    for(i = 0; i < nX; ++i)
        Dx(i, i) *= lambda;
    for(i = 0; i < nG; ++i)
        Dg(i, i) *= lambda;
    const EigenMatrix DxI = Dx.inverse(), Wxc = g_JTJ.block(iX, iC, nX, nC),
                      Wgc = g_JTJ.block(iG, iC, nG, nC), Wgx = g_JTJ.block(iG, iX, nG, nX);
    const EigenMatrix Ycx = Wxc.transpose() * DxI, Ygx = Wgx * DxI;
    const EigenVector JTEc = g_JTE.block(iC, 0, nC, 1), JTEx = g_JTE.block(iX, 0,
                             nX, 1), JTEg = g_JTE.block(iG, 0, nG, 1);

    EigenMatrix A1;
    EigenVector b1;
    A1.resize(nVars, nVars);
    A1.setZero();
    b1.resize(nVars);
    b1.setZero();
    A1.block(iC, iC, nC, nC) = Dc - Ycx * Wxc;
    A1.block(iG, iC, nG, nC) = Wgc - Wgx * DxI * Wxc;
    A1.block(iC, iG, nC, nG) = A1.block(iG, iC, nG, nC).transpose();
    A1.block(iG, iG, nG, nG) = Dg - Ygx * Wgx.transpose();
    b1.block(iC, 0, nC, 1) = JTEc - Ycx * JTEx;
    b1.block(iG, 0, nG, 1) = JTEg - Ygx * JTEx;
    const EigenMatrix &A2 = g_A;
    const EigenVector &b2 = g_b;
    CheckMcxg(iC, iX, iG, A1, A2);
    CheckVcxg(iC, iX, iG, b1, b2);
#endif
}

void SegmentSetBundleAdjustorData2DSimilarity::DebugSolution(
    const float damping, const AlignedVector<LA::AlignedVector7f> &xcs,
    const AlignedVector<LA::Vector3f> &xxs,
    const LA::Vector2f &xg) {
#ifdef _DEBUG_WITH_EIGEN
    const int nC = int(xcs.Size()) * 7, nX = int(xxs.Size()) * 3, nG = 2,
              nVars = nC + nX + nG;
    const int iC = 0, iX = nC, iG = nC + nX;
    g_x.resize(nVars);
    g_x.setZero();

    int i;
    const int nSegsAdj = int(xcs.Size());
    for(i = 0; i < nSegsAdj; ++i)
        SetVc(iC + i * 7, xcs[i], g_x);
    const int nTrks = int(xxs.Size());
    for(i = 0; i < nTrks; ++i)
        SetVx(iX + i * 3, xxs[i], g_x);
    SetVg(iG, xg, g_x);

    EigenMatrix A = g_JTJ;
    EigenVector b = g_JTE;
    const float lambda = damping + 1;
    for(i = 0; i < nVars; ++i)
        A(i, i) *= lambda;
    const EigenVector x1 = A.ldlt().solve(b), &x2 = g_x;
    //CheckVcxg(iC, iX, iG, x1, x2);
    const EigenVector Ax1 = A * x1, Ax2 = A * x2;
    CheckVcxg(iC, iX, iG, Ax1, b);
    CheckVcxg(iC, iX, iG, Ax2, b);
#endif
}

void SegmentSetBundleAdjustorData2DSimilarity::DebugAx(const
        AlignedVector<LA::AlignedVector7f> &xcs, const LA::Vector2f &xg,
        const AlignedVector<LA::AlignedVector7f> &Axcs,
        const LA::Vector2f &Axg) {
#ifdef _DEBUG_WITH_EIGEN_AX
    const int nC = int(xcs.Size()) * 7, nX = int(m_Xs.Size()) * 3, nG = 2;
    const int iC = 0, iX = nC, iG = nC + nX;

    EigenMatrix A;
    A.resize(nC + nG, nC + nG);
    A.setZero();
    A.block(iC, iC, nC, nC) = g_A.block(iC, iC, nC, nC);
    A.block(nC, nC, nG, nG) = g_A.block(iG, iG, nG, nG);
    A.block(iC, nC, nC, nG) = g_A.block(iC, iG, nC, nG);
    A.block(nC, iC, nG, nC) = g_A.block(iG, iC, nG, nC);

    EigenVector x;
    x.resize(nC + nG);
    x.setZero();
    const int nSegsAdj = int(xcs.Size());
    for(int i = 0; i < nSegsAdj; ++i)
        SetVc(iC + i * 7, xcs[i], x);
    SetVg(nC, xg, x);

    EigenVector Ax;
    Ax.resize(nC + nG);
    Ax.setZero();
    for(int i = 0; i < nSegsAdj; ++i)
        SetVc(iC + i * 7, Axcs[i], Ax);
    SetVg(nC, Axg, Ax);

    const EigenVector Ax1 = A * x, &Ax2 = Ax;
    CheckVcxg(iC, nC, nC, Ax1, Ax2);
#endif
}
#endif