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
#include "SequenceRegisteror.h"

using namespace ENFT_SfM;

static inline float SumInconsistencies(const std::vector<float> &inconsistencies) {
    float sum = 0;
    const ushort N = ushort(inconsistencies.size());
    for(ushort j = 0; j < N; ++j)
        sum += inconsistencies[j];
    return sum;
}

void SequenceRegisteror::SegmentSequence(const AlignedVector<RigidTransformation3D> &Ts, const SequenceIndex &iSeq, SequenceSet &seqs,
        const SegmentIndex &nSegs) {
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Sequence %d", iSeq);
#endif
    Sequence &seq = seqs[iSeq];
    const FrameIndex nFrms = seq.GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
        seq.UnmarkFrameSplitPoint(iFrm);
    if(nSegs == 1) {
#if VERBOSE_SEQUENCE_REGISTRATION
        printf(", Segments 1\n");
#endif
        //for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
        //  seq.UnmarkFrameSplitPoint(iFrm);
    } else if(nSegs >= nFrms) {
#if VERBOSE_SEQUENCE_REGISTRATION
        printf(", Segments %d\n", nFrms);
#endif
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
            seq.MarkFrameSplitPoint(iFrm);
    } else {
        if(m_spsfmInconsistencyWeightGradient != 0)
            EstimateConsecutiveFrameInconsistencies_Gradient(seqs, iSeq, m_inconsistenciesGradient);
        if(m_spsfmInconsistencyWeightGradient != 1)
            EstimateConsecutiveFrameInconsistencies_Structure(seqs, iSeq, Ts, m_inconsistenciesStructure);

        const FrameIndex nFrms_m_1 = nFrms - 1;
        m_candidateSplitPts.resize(0);
        FrameIndex iFrm;
        if(m_spsfmInconsistencyWeightGradient == 1) {
            for(iFrm = 0; iFrm < nFrms_m_1; ++iFrm)
                m_candidateSplitPts.push_back(CandidateSplitPoint(iFrm, m_inconsistenciesGradient[iFrm]));
        } else if(m_spsfmInconsistencyWeightGradient == 0) {
            for(iFrm = 0; iFrm < nFrms_m_1; ++iFrm)
                m_candidateSplitPts.push_back(CandidateSplitPoint(iFrm, m_inconsistenciesStructure[iFrm]));
        } else {
            const float weightGradient = m_spsfmInconsistencyWeightGradient / SumInconsistencies(m_inconsistenciesGradient);
            const float weightStructure = (1 - m_spsfmInconsistencyWeightGradient) / SumInconsistencies(m_inconsistenciesStructure);
            for(iFrm = 0; iFrm < nFrms_m_1; ++iFrm)
                m_candidateSplitPts.push_back(CandidateSplitPoint(iFrm, weightGradient * m_inconsistenciesGradient[iFrm] + weightStructure * m_inconsistenciesStructure[iFrm]));
        }
        std::sort(m_candidateSplitPts.begin(), m_candidateSplitPts.end());
        const FrameIndex width = nFrms / nSegs, widthHalf = (width >> 1);
        m_frmMarks.assign(nFrms, false);
        FrameIndex iFrmMarked;
        for(iFrmMarked = 0; iFrmMarked < widthHalf; ++iFrmMarked)
            m_frmMarks[iFrmMarked] = true;
        for(iFrmMarked = nFrms - widthHalf + 1; iFrmMarked < nFrms; ++iFrmMarked)
            m_frmMarks[iFrmMarked] = true;
        FrameIndex iFrm1, iFrm2;
        for(iFrm = 0; iFrm < nFrms_m_1; ++iFrm) {
            if(!(seq.GetFrameState(iFrm) & FLAG_FRAME_STATE_SPLIT_POINT) || !(seq.GetFrameState(iFrm + 1) & FLAG_FRAME_STATE_SPLIT_POINT))
                continue;
            iFrm1 = iFrm - widthHalf;
            iFrm2 = iFrm + 1 + widthHalf;
            for(iFrmMarked = iFrm1; iFrmMarked < iFrm2; ++iFrmMarked)
                m_frmMarks[iFrmMarked] = true;
        }
        SegmentIndex i, j, k;
        const SegmentIndex nSplitPts = nSegs - 1;
        for(i = j = k = 0; i < nFrms_m_1; ++i) {
            const CandidateSplitPoint &splitPt = m_candidateSplitPts[i];
            iFrm1 = splitPt.GetFrameIndex();
            iFrm2 = iFrm1 + 1;
            if(m_frmMarks[iFrm2]) {
                m_candidateSplitPts[j++] = splitPt;
                continue;
            }
            seq.MarkFrameSplitPoint(iFrm1);
            seq.MarkFrameSplitPoint(iFrm2);
            if(++k == nSplitPts)
                break;

            iFrm1 = iFrm2 - widthHalf + 1;
            iFrm2 += widthHalf;
            for(iFrmMarked = iFrm1; iFrmMarked < iFrm2; ++iFrmMarked)
                m_frmMarks[iFrmMarked] = true;
        }
        m_candidateSplitPts.resize(j);
        for(i = 0; k < nSplitPts; ++i, ++k) {
            const CandidateSplitPoint &splitPt = m_candidateSplitPts[i];
            iFrm1 = splitPt.GetFrameIndex();
            iFrm2 = iFrm1 + 1;
            seq.MarkFrameSplitPoint(iFrm1);
            seq.MarkFrameSplitPoint(iFrm2);
        }

#if VERBOSE_SEQUENCE_REGISTRATION
        printf(", Segments %d --> %d {", nFrms, nSegs);
        iFrm2 = 0;
        //int idx = 0;
        //printf("\n");
        while(iFrm2 < nFrms) {
            iFrm1 = iFrm2;
            for(iFrm2 = iFrm1 + 1; iFrm2 < nFrms && !(seq.GetFrameState(iFrm2) & FLAG_FRAME_STATE_SPLIT_POINT); ++iFrm2);
            if(iFrm2 != nFrms && iFrm2 != iFrm1 + 1)
                ++iFrm2;
            printf(" %d", iFrm2 - iFrm1);
            //printf("  %d (%d, %d)\n", idx++, iFrm1, iFrm2);
        }
        printf(" }\n");
#endif
    }
}

static inline bool ComputeGradient(const SequenceSet &seqs, const SequenceIndex &iSeq, FrameIndex &iFrm, LA::AlignedVector7f &g) {
    TrackIndex iTrkIdv, iTrkCmn;
    Point2D e;
    LA::AlignedCompactMatrix2x7f Jc;
    ENFT_SSE::__m128 work[2];
    ENFT_SSE::__m128 &invZc = work[0];

    g.SetZero();

    const Sequence &seq = seqs[iSeq];
    const Camera &C = seq.GetCamera(iFrm);
    const TrackIndex *iTrksIdv = seq.GetFrameTrackIndexes(iFrm);
    const TrackIndexList &iTrksCmn = seqs.GetSequenceCommonTrackIndexList(iSeq);
    const Point2D *xs = seq.GetFrameFeatures(iFrm);
    const MeasurementState *meaStates = seq.GetFrameMeasurementStates(iFrm);
    const FeatureIndex nFtrs = seq.GetFrameFeaturesNumber(iFrm);
    for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr) {
        if((iTrkIdv = iTrksIdv[iFtr]) == INVALID_TRACK_INDEX || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) ||
                (meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
            continue;
        iTrkCmn = (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER) ? INVALID_TRACK_INDEX : iTrksCmn[iTrkIdv];
        const Point3D &X = iTrkCmn == INVALID_TRACK_INDEX ? seq.GetPoint(iTrkIdv) : seqs.GetCommonPoint(iTrkCmn);
        if(!C.ProjectToNormalizedPlane(X, invZc.m128_f32[0], e.x(), e.y()))
            continue;
        invZc = ENFT_SSE::_mm_set1_ps(invZc.m128_f32[0]);
        // S = {t, w, s}
        ENFT_SSE::__m128 &J_x_Xseg = Jc.M_00_01_02_03(), &J_y_Xseg = Jc.M_10_11_12_13();
        J_x_Xseg = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(invZc, C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(invZc, ENFT_SSE::_mm_set1_ps(e.x())),
                                        C.r_20_21_22_x()));
        J_y_Xseg = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(invZc, C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(invZc, ENFT_SSE::_mm_set1_ps(e.y())),
                                        C.r_20_21_22_x()));
        Jc.M03() = J_x_Xseg.m128_f32[2] * X.Y() - J_x_Xseg.m128_f32[1] * X.Z();
        Jc.M04() = J_x_Xseg.m128_f32[0] * X.Z() - J_x_Xseg.m128_f32[2] * X.X();
        Jc.M05() = J_x_Xseg.m128_f32[1] * X.X() - J_x_Xseg.m128_f32[0] * X.Y();
        Jc.M06() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(J_x_Xseg, X.XYZx()));
        Jc.M13() = J_y_Xseg.m128_f32[2] * X.Y() - J_y_Xseg.m128_f32[1] * X.Z();
        Jc.M14() = J_y_Xseg.m128_f32[0] * X.Z() - J_y_Xseg.m128_f32[2] * X.X();
        Jc.M15() = J_y_Xseg.m128_f32[1] * X.X() - J_y_Xseg.m128_f32[0] * X.Y();
        Jc.M16() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(J_y_Xseg, X.XYZx()));

        e.x() = xs[iFtr].x() - e.x();
        e.y() = xs[iFtr].y() - e.y();

        LA::AddATBTo(Jc, e, g, work);
    }
    LA::FinishAdditionATBTo<LA::AlignedCompactMatrix2x7f>(g);
    g.reserve() = 0;
    ENFT_SSE::__m128 &s = work[0];
    if((s.m128_f32[0] = g.SquaredLength()) < FLT_EPSILON)
        return false;
    s = ENFT_SSE::_mm_set1_ps(1 / sqrt(s.m128_f32[0]));
    g.Scale(s);
    return true;
}

void SequenceRegisteror::EstimateConsecutiveFrameInconsistencies_Gradient(const SequenceSet &seqs, const SequenceIndex &iSeq,
        std::vector<float> &inconsistencies) {
    FrameIndex iFrm1, iFrm2;
    bool b1, b2;
    LA::AlignedVector7f g1, g2;

    const FrameIndex nFrms = seqs[iSeq].GetFramesNumber();
    inconsistencies.resize(nFrms - 1);

    iFrm1 = 0;
    b1 = ComputeGradient(seqs, iSeq, iFrm1, g1);
    for(iFrm2 = 1; iFrm2 < nFrms; iFrm1 = iFrm2, ++iFrm2, b1 = b2, g1 = g2) {
        b2 = ComputeGradient(seqs, iSeq, iFrm2, g2);
        if(b1 && b2)
            inconsistencies[iFrm1] = 1 - g1.Dot(g2);
        else
            inconsistencies[iFrm1] = 0.0f;
    }
}

void SequenceRegisteror::EstimateConsecutiveFrameInconsistencies_Structure(const SequenceSet &seqs, const SequenceIndex &iSeq,
        const AlignedVector<RigidTransformation3D> &Ts, std::vector<float> &inconsistencies) {
    FrameIndex iFrm1, iFrm2;
    TrackIndexList &iTrks = m_iTrksIdv;
    Point3D X1, RX1;
    LA::Vector2f scale;
    double fitErr;
    std::vector<ushort> inliers;

    const Sequence &seq = seqs[iSeq];
    const FrameIndex nFrms = seq.GetFramesNumber();
    inconsistencies.resize(nFrms - 1);
    for(iFrm1 = 0, iFrm2 = 1; iFrm2 < nFrms; iFrm1 = iFrm2, ++iFrm2) {
        seq.SearchForFrameFeatureMatchesInlierTrackAndMeasurement(iFrm1, iFrm2, m_matches, iTrks);
        const RigidTransformation3D &T = Ts[iFrm1];
        if(T.r00() == FLT_MAX) {
            inconsistencies[iFrm1] = 0;
            continue;
        }
        m_TSdata.SetTranslation(T);
        const LA::AlignedVector3f &t = m_TSdata.GetTranslation();
        const ushort N = ushort(iTrks.size());
        const Camera &C1 = seq.GetCamera(iFrm1);
        const Point2D *xs2 = seq.GetFrameFeatures(iFrm2);
        m_TSdata.Resize(N);
        for(ushort i = 0; i < N; ++i) {
            C1.Apply(seq.GetPoint(iTrks[i]), X1);
            T.ApplyRotation(X1, RX1);
            const FeatureIndex iFtr2 = m_matches[i].GetIndex2();
            TranslationScaleSolver::Run(RX1, xs2[iFtr2], t, scale);
            m_TSdata.Set(i, RX1, xs2[iFtr2], scale);
        }
        m_TSestor.RunLosac(m_TSdata, scale.v0(), inliers);
        m_TSestor.VerifyModel(m_TSdata, scale.v0(), inliers, fitErr);
        inconsistencies[iFrm1] = float(fitErr / inliers.size());
    }
}