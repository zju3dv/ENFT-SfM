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
#include "Sequence/SequenceBundleAdjustorData.h"
#include "SfM/EssentialMatrixEstimator.h"

using namespace ENFT_SfM;

void SequenceRegisteror::SelectInitialSequence(const SequenceSet &seqs,
        SequenceIndex &iSeqInit) {
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    if(nSeqs == 1) {
        iSeqInit = 0;
        return;
    }
    TrackIndex nTrksCmn, nTrksCmnMax = 0;
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        if((nTrksCmn = seqs[iSeq].CountTracks(FLAG_TRACK_STATE_COMMON,
                                              FLAG_TRACK_STATE_INLIER)) >= nTrksCmnMax) {//bug >
            nTrksCmnMax = nTrksCmn;
            iSeqInit = iSeq;
        }
    }
    if (nTrksCmnMax == 0) {//new add
        printf("No common Track in Sequences");
    }
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Initial sequence: Sequence %d, %d common tracks\n", iSeqInit,
           nTrksCmnMax);
#endif
}

void SequenceRegisteror::GetIncrementalSequence(const SequenceSet &seqs,
        SequenceIndex &iSeqIncr, TrackIndex &nTrksCmnIncr) {
    iSeqIncr = INVALID_SEQUENCE_INDEX;
    nTrksCmnIncr = 0;
    TrackIndex nTrksCmn;
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        if(!(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED) &&
                (nTrksCmn = seqs.CountSequenceRegisteredCommonTracks(iSeq)) > nTrksCmnIncr) {
            iSeqIncr = iSeq;
            nTrksCmnIncr = nTrksCmn;
        }
    }
}

void SequenceRegisteror::RegisterSequence_MarkCommonOutliers(
    const SequenceIndex &iSeq, SequenceSet &seqs, SequenceErrorLevel &seqErrLevel) {
    Sequence &seq = seqs[iSeq];

    seqs.GetSimilarityEstimatorData(iSeq, m_Sdata, m_iTrksIdv);
    seqs.MarkSequenceConnectedComponent(iSeq, m_seqMarks);
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    //ushort maxNumTransformations = 0;
    //for(SequenceIndex iSeqCC = 0; iSeqCC < nSeqs; ++iSeqCC)
    //{
    //  if(iSeqCC != iSeq && m_seqMarks[iSeq] && (seqs.GetSequenceState(iSeqCC) & FLAG_SEQUENCE_STATE_REGISTRED))
    //      ++maxNumTransformations;
    //}
    //if(maxNumTransformations > m_sfmSeqMaxNumTransformations)
    //  maxNumTransformations = m_sfmSeqMaxNumTransformations;

    const IntrinsicMatrix &K = seq.GetIntrinsicMatrix();
    const Camera::IntrinsicParameter &Kr = seq.GetIntrinsicRectification();
    const ushort N = ushort(m_Sdata.Size());
    ushort nInliers, nTransformations;
    seqErrLevel.Initialize();
    while(1) {
        m_Sestor.m_ransacErrorThreshold =
            m_sfmReprojErrSqThs[seqErrLevel.GetReprojectionErrorLevel()] * K.one_over_fxy();
        if(seq.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT)
            m_Sestor.m_ransacErrorThreshold /= Kr.f() * Kr.f();
        if(m_sfmSeqMaxNumTransformations > 0) {
            //m_Sestor.RunLosac(m_Sdata, m_S, m_inliers);
            //if((nInliers = ushort(m_inliers.size())) >= m_sfmSeqInliersMinNum && nInliers >= ushort(N * m_sfmSeqInliersRatioThs[seqErrLevel.GetOutlierLevel()] + 0.5f)
            //|| !seqErrLevel.Increase())
            //  break;
            //m_Sestor.RunLosacMultiple(m_Sdata, m_SList, m_inliersList, maxNumTransformations, m_sfmSeqInliersMinNum);
            m_Sestor.RunLosacMultiple(m_Sdata, m_SList, m_inliersList,
                                      m_sfmSeqMaxNumTransformations, m_sfmSeqInliersMinNum);
            nTransformations = ushort(m_inliersList.size());
        } else {
            double fitErr;
            m_SList.Resize(1);
            m_SList[0].MakeIdentity();
            m_inliersList.resize(1);
            m_Sestor.VerifyModel(m_Sdata, m_SList[0], m_inliersList[0], fitErr);
            nTransformations = 1;
        }
        if(nTransformations == 0 && !seqErrLevel.Increase())
            break;
        nInliers = ushort(m_inliersList[0].size());//bug
        for(ushort i = 1; i < nTransformations; ++i)
            nInliers += ushort(m_inliersList[i].size());
#if VERBOSE_SEQUENCE_REGISTRATION
        if(nTransformations == 1) {
            printf("Sequence %d: inliers = %d/%d = %d%%, error threshold = %.2f\n", iSeq,
                   nInliers, N, nInliers * 100 / N,
                   sqrt(m_sfmReprojErrSqThs[seqErrLevel.GetReprojectionErrorLevel()]));
        } else {
            printf("Sequence %d: inliers = (%d", iSeq, m_inliersList[0].size());
            for(ushort i = 1; i < nTransformations; ++i)
                printf("+%d", m_inliersList[i].size());
            printf(")/%d = %d%%, error threshold = %.2f\n", N, nInliers * 100 / N,
                   sqrt(m_sfmReprojErrSqThs[seqErrLevel.GetReprojectionErrorLevel()]));
        }
#endif
        if(nInliers >= m_sfmSeqInliersMinNum &&
                nInliers >= ushort(N * m_sfmSeqInliersRatioThs[seqErrLevel.GetOutlierLevel()] +
                                   0.5f) || !seqErrLevel.Increase())
            break;
    }
    if(nTransformations == 0)
        return;
    m_S = m_SList[0];
    m_inliers = m_inliersList[0];
    for(ushort i = 1; i < nTransformations; ++i)
        m_inliers.insert(m_inliers.end(), m_inliersList[i].begin(),
                         m_inliersList[i].end());
    if(nTransformations > 1)
        std::sort(m_inliers.begin(), m_inliers.end());

    ENFT_SSE::__m128 work[2];
    m_S.Invert(work);
    seq.TransformScene(m_S);

    m_Sestor.FromInliersToInlierMarks(m_inliers, N, m_inlierMarks);
    for(ushort i = 0; i < N; ++i) {
        if(m_inlierMarks[i])
            seq.MarkTrackCommonInlier(m_iTrksIdv[i]);
        else
            seq.MarkTrackCommonOutlier(m_iTrksIdv[i]);
    }

    seqs.MarkSequenceRegistered(iSeq);
    ViewSequences(seqs, iSeq);
}

void SequenceRegisteror::EstimateRelativePoses(const SequenceSet &seqs,
        std::vector<AlignedVector<RigidTransformation3D> > &TsList) {
    EssentialMatrixEstimator Eestor(FLT_MAX);
    EssentialMatrix E;
    EssentialMatrixEstimatorData Edata;
    ENFT_SSE::__m128 work[38];
    std::vector<ushort> inliers;

    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
#if VERBOSE_SEQUENCE_REGISTRATION
    SegmentIndex N = 0;
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
        N += seqs[iSeq].GetFramesNumber() - 1;
    SegmentIndex i = 0;
#endif
    TsList.resize(nSeqs);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        const Sequence &seq = seqs[iSeq];
        AlignedVector<RigidTransformation3D> &Ts = TsList[iSeq];
        const FrameIndex nFrms = seq.GetFramesNumber();
        Ts.Resize(nFrms - 1);
        for(FrameIndex iFrm1 = 0, iFrm2 = 1; iFrm2 < nFrms; iFrm1 = iFrm2, ++iFrm2) {
            seq.SearchForFrameFeatureMatchesInlierTrackAndMeasurement(iFrm1, iFrm2,
                    m_matches, Edata, true);
            Eestor.RunLosac(Edata, E, inliers);
            if(!E.ToRelativePose(Edata, Ts[iFrm1], 1.0f, work))
                Ts[iFrm1].r00() = FLT_MAX;
#if VERBOSE_SEQUENCE_REGISTRATION
            printf("\rEstimating relative poses...%d%%", i++ * 100 / N);
#endif
        }
    }
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("\rEstimating relative poses...%d%%\n", i++ * 100 / N);
#endif
}

void SequenceRegisteror::BundleAdjust3D(const SequenceIndexList &iSeqsAdj,
                                        SequenceIndexList &iSeqsBA, TrackIndexList &iTrksCmnBA, SequenceSet &seqs) {
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("----------------------------------------------------------------\n");
    printf("Bundle adjustment 3D\n");
#endif
    seqs.GetSequenceTransformationOptimizerData(m_gtoData, iSeqsBA);
    RemoveOutlierData(m_gtoData);
    m_gto.Run(m_gtoData, VERBOSE_SEQUENCE_REGISTRATION ? 2 : 0);
    seqs.SetSequenceTransformationOptimizationResults(m_gtoData, iSeqsBA);
    //seqs.AverageIndividualPointsToCommonPoints();
    seqs.CopyIndividualPointsToCommonPoints();
    //seqs.CopyCommonPointsToIndividualPoints();
    ViewSequences(seqs, iSeqsAdj.back());
}

static inline void ComputeTransformationError(const SimilarityTransformation3D
        &S, const Point3D &X, const Point3D &x, Point3D &e) {
    S.ComputeTransformationError(X, x, e);
}

static inline void ComputeTransformationError(const float &s, const Point3D &X,
        const Point3D &x, Point3D &e) {
    e.XYZx() = ENFT_SSE::_mm_sub_ps(x.XYZx(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), X.XYZx()));
}

template<BA_TEMPLATE_PARAMETER>
void SequenceRegisteror::RemoveOutlierData(
    BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> &data,
    TrackIndexList &iTrksCmn) {
    //if(m_sfm3DErrSqThFactor < 0)
    if(m_sfm3DErrThRatio <= 0 || m_sfm3DErrThRatio == 1.0f)
        return;

    TrackIndex iPt, iMea;
    SequenceIndex i, iCam;
    float errSq;
    Point3D e;

    const std::vector<TrackIndexList> &mapPtToMea = data.GetPointMeasurementMap();
    const SequenceIndexList &mapMeaToCam = data.GetMeasurementCameraMap();

    const TrackIndex nPts = data.GetPointsNumber();
    m_errSqs.resize(nPts);
    for(iPt = 0; iPt < nPts; ++iPt) {
        const Point3D &X = data.GetPoint(iPt);
        const TrackIndexList &iMeas = mapPtToMea[iPt];
        const SequenceIndex nCrsps = SequenceIndex(iMeas.size());
        for(i = 0, errSq = 0; i < nCrsps; ++i) {
            iMea = iMeas[i];
            iCam = mapMeaToCam[iMea];
            //data.GetCamera(iCam).ComputeTransformationError(X, data.GetMeasurement(iMea), e);
            ComputeTransformationError(data.GetCamera(iCam), X, data.GetMeasurement(iMea),
                                       e);
            errSq += e.SquaredLength();
        }
        errSq /= nCrsps;
        m_errSqs[iPt] = errSq;
    }

    m_errSqsSort = m_errSqs;
    //const TrackIndex ith = (nPts >> 1);
    //std::nth_element(m_errSqsSort.begin(), m_errSqsSort.begin() + ith, m_errSqsSort.end());
    //const float errSqTh = m_errSqsSort[ith] * m_sfm3DErrSqThFactor;
    const TrackIndex ith = TrackIndex(nPts * m_sfm3DErrThRatio);
    std::nth_element(m_errSqsSort.begin(), m_errSqsSort.begin() + ith,
                     m_errSqsSort.end());
    const float errSqTh = m_errSqsSort[ith];

    TrackIndex iPtOri, iPtNew;
    for(iPtOri = iPtNew = 0; iPtOri < nPts; ++iPtOri) {
        if(m_errSqs[iPtOri] > errSqTh)
            data.BreakPoint(iPtOri);
        else
            iTrksCmn[iPtNew++] = iTrksCmn[iPtOri];
    }
    iTrksCmn.resize(iPtNew);

    data.RemoveBrokenPoints(m_iPtsOriToNew, m_iMeasOriToNew);

#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Removed %d / %d points, error threshold = %f\n",
           nPts - data.GetPointsNumber(), nPts, sqrt(errSqTh));
#endif
}

static inline void ComputeTransformationError(const SimilarityTransformation3D
        &S1, const Point3D &X1, const SimilarityTransformation3D &S2, const Point3D &X2,
        Point3D &S1X1, Point3D &S2X2, Point3D &e) {
    S1.Apply(X1, S1X1);
    S2.Apply(X2, S2X2);
    LA::AmB(S1X1, S2X2, e);
}

template<GTO_TEMPLATE_PARAMETER>
void SequenceRegisteror::RemoveOutlierData(
    GlobalTransformationOptimizerDataTemplate<GTO_TEMPLATE_ARGUMENT> &data) {
    //if(m_sfm3DErrSqThFactor < 0)
    if(m_sfm3DErrThRatio <= 0 || m_sfm3DErrThRatio == 1.0f)
        return;

    uint i;
    TrackIndex j, k, cnt = 0;
    SequenceIndex iSeq1, iSeq2;
    Point3D S1X1, S2X2, e;
    const uint nMapPairs = data.GetMapPairsNumber();
    for(i = 0; i < nMapPairs; ++i) {
        data.GetMapIndexes(i, iSeq1, iSeq2);
        const SimilarityTransformation3D &S1 = data.GetTransformation(iSeq1);
        const SimilarityTransformation3D &S2 = data.GetTransformation(iSeq2);
        AlignedVector<SequenceTransformationOptimizerDataSimilarity::PointPair> &Xs =
            data.Xs(i);
        const TrackIndex nPtPairs = TrackIndex(Xs.Size());
        m_errSqs.resize(nPtPairs);
        for(j = 0; j < nPtPairs; ++j) {
            ComputeTransformationError(S1, Xs[j].X1(), S2, Xs[j].X2(), S1X1, S2X2, e);
            m_errSqs[j] = e.SquaredLength();
        }
        m_errSqsSort = m_errSqs;
        const TrackIndex ith = TrackIndex(nPtPairs * m_sfm3DErrThRatio);
        std::nth_element(m_errSqsSort.begin(), m_errSqsSort.begin() + ith,
                         m_errSqsSort.end());
        const float errSqTh = m_errSqsSort[ith];
        for(j = k = 0; j < nPtPairs; ++j) {
            if(m_errSqs[j] <= errSqTh)
                Xs[k++] = Xs[j];
        }
        Xs.Resize(k);
        cnt += k;

#if VERBOSE_SEQUENCE_REGISTRATION
        printf("Sequence (%d, %d): Removed %d / %d point pairs, error threshold = %f\n",
               iSeq1, iSeq2, nPtPairs - k, nPtPairs, sqrt(errSqTh));
#endif
    }
    const TrackIndex nPts = (cnt << 1);
    data.SetPointsNumber(nPts);
}

void SequenceRegisteror::BundleAdjust2DLocal(const
        std::vector<AlignedVector<RigidTransformation3D> > &TsList,
        const SequenceIndex &iSeqFix, const SequenceIndexList &iSeqsSeed,
        SequenceIndexList &iSeqsAdj, SequenceIndexList &iSeqsBA,
        TrackIndexList &iTrksCmnBA, SequenceTrackIndexList &iSeqTrksIdvBA,
        SequenceSet &seqs) {
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("----------------------------------------------------------------\n");
    printf("Local bundle adjustment 2D...\n");
#endif
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    iSeqsAdj.resize(0);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        if(/*iSeq != iSeqFix && */seqs.GetSequenceState(iSeq) &
                                  FLAG_SEQUENCE_STATE_REGISTRED)
            iSeqsAdj.push_back(iSeq);
    }
    const SequenceIndex nSeqsSeed = SequenceIndex(iSeqsSeed.size()),
                        nSeqsLocalCandidate = SequenceIndex(iSeqsAdj.size()) - nSeqsSeed;
    if(nSeqsLocalCandidate > m_baLocalMaxNumAdditionalAdjustedSeqs) {
        m_seqMarks.assign(nSeqs, false);
        m_cmnTrkMarks.assign(seqs.GetCommonTracksNumber(), false);
        for(SequenceIndex i = 0; i < nSeqsSeed; ++i) {
            const SequenceIndex iSeq = iSeqsSeed[i];
            m_seqMarks[iSeq] = true;
            const Sequence &seq = seqs[iSeq];
            const TrackIndexList &iTrksIdvToCmn = seqs.GetSequenceCommonTrackIndexList(
                    iSeq);
            const TrackIndex nTrksIdv = TrackIndex(iTrksIdvToCmn.size());
            for(TrackIndex iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
                const TrackIndex iTrkCmn = iTrksIdvToCmn[iTrkIdv];
                if(iTrkCmn != INVALID_TRACK_INDEX &&
                        (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) &&
                        !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
                    m_cmnTrkMarks[iTrkCmn] = true;
            }
        }
        m_candidateSeqs.resize(0);
        for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
            if(iSeq != iSeqFix && !m_seqMarks[iSeq] &&
                    (seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED))
                m_candidateSeqs.push_back(CandidateSequence(iSeq,
                                          seqs.CountSequenceMarkedCommonTracks(iSeq, m_cmnTrkMarks)));
        }
        std::sort(m_candidateSeqs.begin(), m_candidateSeqs.end());
        for(SequenceIndex i = 0; i < m_baLocalMaxNumAdditionalAdjustedSeqs; ++i)
            m_seqMarks[m_candidateSeqs[i].GetSequenceIndex()] = true;
        iSeqsAdj.resize(0);
        for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
            if(m_seqMarks[iSeq])
                iSeqsAdj.push_back(iSeq);
        }
    }

    FrameIndex nFrmsMax = 0;
    const SequenceIndex nSeqsAdj = SequenceIndex(iSeqsAdj.size());
    for(SequenceIndex i = 0; i < nSeqsAdj; ++i) {
        const FrameIndex nFrms = seqs[iSeqsAdj[i]].GetFramesNumber();
        if(nFrms > nFrmsMax)
            nFrmsMax = nFrms;
    }

    SegmentIndex nSegsPerSeq = m_spsfmSegsNumPerSeqInit;
    //for(ushort iter = 0; iter < m_spsfmMaxNumIters; ++iter, nSegsPerSeq *= m_spsfmSegsNumPerSeqFactor)
    //for(ushort iter = 0; iter < m_spsfmMaxNumIters; ++iter, nSegsPerSeq += m_spsfmSegsNumPerSeqIncr)
    for(ushort iter = 0; iter < m_spsfmMaxNumIters; ++iter) {
#if VERBOSE_SEQUENCE_REGISTRATION
        printf("----------------------------------------------------------------\n");
#endif
        for(SequenceIndex i = 0; i < nSeqsAdj; ++i) {
            const SequenceIndex iSeq = iSeqsAdj[i];
            SegmentSequence(TsList[iSeq], iSeq, seqs, nSegsPerSeq);
        }

        BAResult res;
        bool toInternal;
        RunBundleAdjustment2D(iSeqsAdj, iSeqsBA, iTrksCmnBA, iSeqTrksIdvBA, seqs, res,
                              toInternal, m_baLocalMaxNumIters, m_baLocalStopMSE,
                              m_baLocalStopRelativeReduction);
        ViewSequences(seqs, iSeqsSeed.back());

        if(nSegsPerSeq >= nFrmsMax || res == BA_SMALL_MSE)
            break;
        if(m_spsfmSegsNumPerSeqFactor != 0)
            nSegsPerSeq *= m_spsfmSegsNumPerSeqFactor;
        else
            nSegsPerSeq += m_spsfmSegsNumPerSeqIncr;
    }
}

static inline void FindFarthestPoint(const SequenceSet &seqs) {
    float val, valMax = 0;
    TrackIndex iTrkCmnMax = INVALID_TRACK_INDEX;
    const TrackIndex nTrksCmn = seqs.GetCommonTracksNumber();
    for(TrackIndex iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn) {
        const Point3D &X = seqs.GetCommonPoint(iTrkCmn);
        if((val = fabs(X.X())) > valMax || (val = fabs(X.Y())) > valMax ||
                (val = fabs(X.Z())) > valMax) {
            valMax = val;
            iTrkCmnMax = iTrkCmn;
        }
    }
    SequenceIndex iSeqMax = INVALID_SEQUENCE_INDEX;
    TrackIndex iTrkIdvMax = INVALID_TRACK_INDEX;
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        const Sequence &seq = seqs[iSeq];
        const TrackIndex nTrksIdv = seq.GetTracksNumber();
        for(TrackIndex iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
            const Point3D &X = seq.GetPoint(iTrkIdv);
            if((val = fabs(X.X())) > valMax || (val = fabs(X.Y())) > valMax ||
                    (val = fabs(X.Z())) > valMax) {
                valMax = val;
                iSeqMax = iSeq;
                iTrkIdvMax = iTrkIdv;
            }
        }
    }
    if(iSeqMax == INVALID_SEQUENCE_INDEX) {
        printf("iTrkCmn = %d\n", iTrkCmnMax);
        seqs.GetCommonPoint(iTrkCmnMax).Print();
    } else {
        printf("iSeq = %d, iTrkIdv = %d\n", iSeqMax, iTrkIdvMax);
        seqs[iSeqMax].GetPoint(iTrkIdvMax).Print();
    }
}

void SequenceRegisteror::BundleAdjust2DGlobal(const
        std::vector<AlignedVector<RigidTransformation3D> > &TsList,
        const SequenceIndex &iSeqFix, SequenceIndexList &iSeqsAdj,
        SequenceIndexList &iSeqsBA, TrackIndexList &iTrksCmnBA,
        SequenceTrackIndexList &iSeqTrksIdvBA, SequenceSet &seqs) {
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("----------------------------------------------------------------\n");
    printf("Global bundle adjustment 2D...\n");
#endif

    //BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT_SEQUENCE> ba;
    //SequenceBundleAdjustorData data;
    //ba.m_alg = ALG_SCHUR_EXPLICIT;
    //ba.m_duplicateReorderedPointJacobians = true;
    //ba.m_duplicateReorderedPointCameraBlocks = true;

    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    iSeqsAdj.resize(0);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        if(/*iSeq != iSeqFix && */seqs.GetSequenceState(iSeq) &
                                  FLAG_SEQUENCE_STATE_REGISTRED)
            iSeqsAdj.push_back(iSeq);
    }

    FrameIndex nFrmsMax = 0;
    const SequenceIndex nSeqsAdj = SequenceIndex(iSeqsAdj.size());
    for(SequenceIndex i = 0; i < nSeqsAdj; ++i) {
        const FrameIndex nFrms = seqs[iSeqsAdj[i]].GetFramesNumber();
        if(nFrms > nFrmsMax)
            nFrmsMax = nFrms;
    }

    SegmentIndex nSegsPerSeq = m_spsfmSegsNumPerSeqInit;
    //char fileName[MAX_LINE_LENGTH];
    //sprintf(fileName, "E:/tmp/test_%02d.txt", 5);
    //LoadB(fileName, seqs, nSegsPerSeq);
    //FindFarthestPoint(seqs);
    //m_stop = true;
    //ViewSequences(seqs, 33);
    //for(SequenceIndex i = 0; i < nSeqsAdj; ++i)
    //{
    //  const SequenceIndex iSeq = iSeqsAdj[i];
    //  SegmentSequence(RtsList[iSeq], iSeq, seqs, nSegsPerSeq);
    //}
    //BAResult res;
    //bool toInternal;
    //RunBundleAdjustment2D(iSeqsAdj, iSeqsBA, iTrksCmnBA, iSeqTrksIdvBA, seqs, res, toInternal, m_baGlobalMaxNumIters, m_baGlobalStopMSE, m_baGlobalStopRelativeReduction);
    //m_stop = true;
    //ViewSequences(seqs, 33);
    for(ushort iter = 0; iter < m_spsfmMaxNumIters; ++iter) {
        //seqs.AverageIndividualPointsToCommonPoints();
        //BundleAdjust3D(iSeqsAdj, m_iSeqsBA, m_iTrksCmnBA, seqs);

#if VERBOSE_SEQUENCE_REGISTRATION
        printf("----------------------------------------------------------------\n");
#endif
        for(SequenceIndex i = 0; i < nSeqsAdj; ++i) {
            const SequenceIndex iSeq = iSeqsAdj[i];
            SegmentSequence(TsList[iSeq], iSeq, seqs, nSegsPerSeq);
        }

        BAResult res;
        bool toInternal;
        RunBundleAdjustment2D(iSeqsAdj, iSeqsBA, iTrksCmnBA, iSeqTrksIdvBA, seqs, res,
                              toInternal, m_baGlobalMaxNumIters, m_baGlobalStopMSE,
                              m_baGlobalStopRelativeReduction);
        ViewSequences(seqs, iSeqFix);

        if(toInternal)
            seqs.CopyIndividualPointsToCommonPoints();
        else
            BundleAdjust3D(iSeqsAdj, iSeqsBA, iTrksCmnBA, seqs);

//      for(SequenceIndex i = 0; i < nSeqsAdj; ++i)
//      {
//          const SequenceIndex iSeq = iSeqsAdj[i];
//          if(!m_seqMarks[iSeq])
//              continue;
//#if VERBOSE_SEQUENCE_REGISTRATION
//          printf("----------------------------------------------------------------\n");
//          printf("Sequence %d\n", iSeq);
//#endif
//          seqs[iSeq].GetBundleAdjustorData(data);
//          ba.Run(data, 1, VERBOSE_SEQUENCE_REGISTRATION ? 2 : 0);
//          seqs[iSeq].SetCameras(data.GetCameras());
//          seqs[iSeq].SetPoints(data.GetPoints());
//          ViewSequences(seqs, iSeq);
//      }

        if(nSegsPerSeq >= nFrmsMax || res == BA_SMALL_MSE)
            break;
        //if(m_spsfmSegsNumPerSeqFactor != 0)
        //  nSegsPerSeq *= m_spsfmSegsNumPerSeqFactor;
        //else
        //  nSegsPerSeq += m_spsfmSegsNumPerSeqIncr;
        if(toInternal)
            nSegsPerSeq *= m_spsfmSegsNumPerSeqFactor;
        else
            nSegsPerSeq += m_spsfmSegsNumPerSeqIncr;

        //m_stop = true;
        //ViewSequences(seqs, 12);
        //SequenceSet seqsChk = seqs;
        //seqsChk.CopyCommonPointsToIndividualPoints();
        //ViewSequences(seqsChk, iSeqFix);
        //ViewSequences(seqs, iSeqFix);

        //sprintf(fileName, "E:/tmp/test_%02d.txt", iter);
        //SaveB(fileName, seqs, nSegsPerSeq);
    }
}

void SequenceRegisteror::RunBundleAdjustment2D(const SequenceIndexList
        &iSeqsAdj, SequenceIndexList &iSeqsBA, TrackIndexList &iTrksCmnBA,
        SequenceTrackIndexList &iSeqTrksIdvBA, SequenceSet &seqs, BAResult &res,
        bool &toInternal,
        const uint &maxNumIters, const float &stopMSE,
        const float &stopRelativeReduction) {
    //m_stop = true;
    //m_view = true;
    //ViewSequences(seqs, 31);
    SequenceIndex nSeqsFix;
    seqs.GetBundleAdjustorData(iSeqsAdj, m_baData2D, nSeqsFix, iSeqsBA, iTrksCmnBA,
                               iSeqTrksIdvBA);
    //data.ComputeProjectionMSE();
    m_ba2D.m_lmMaxNumIters = maxNumIters;
    m_ba2D.m_lmStopMSE = stopMSE;
    m_ba2D.m_lmStopRelativeReduction = stopRelativeReduction;
    const SegmentIndex nSegsFix = nSeqsFix == std::min(SequenceIndex(1), nSeqsFix);
    res = m_ba2D.Run(m_baData2D, nSegsFix,
                     seqs.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT,
                     VERBOSE_SEQUENCE_REGISTRATION ? 2 : 0);
    //res = BA_ENOUGH_ITERATIONS;
    toInternal = true;
    m_baData2D.ComputeMSE(m_errSqs);
    const SequenceIndex nSeqsBA = SequenceIndex(m_errSqs.size());
    for(SequenceIndex i = 0; i < nSeqsBA; ++i) {
        if(m_errSqs[i] > m_spsfmReprojErrSqThInternal)
            toInternal = false;
    }
#if VERBOSE_SEQUENCE_REGISTRATION
    if(!toInternal) {
        for(SequenceIndex i = 0; i < nSeqsBA; ++i)
            printf("Sequence %d: error = %f\n", iSeqsBA[i], m_errSqs[i]);
    }
#endif
    seqs.SetBundleAdjustmentResults(m_baData2D, nSegsFix, iSeqsBA, iTrksCmnBA,
                                    iSeqTrksIdvBA, toInternal);
}

void SequenceRegisteror::BundleAdjust2DAdaptiveErrorLevelLocal(
    const std::vector<AlignedVector<RigidTransformation3D> > &TsList,
    const SequenceIndex &iSeqFix,
    const SequenceIndexList &iSeqsSeed, SequenceSet &seqs,
    std::vector<SequenceErrorLevel> &seqErrLevels) {
    BundleAdjust2DLocal(TsList, iSeqFix, iSeqsSeed, m_iSeqsAdj, m_iSeqsBA,
                        m_iTrksCmnBA, m_iSeqTrksIdvBA, seqs);
    //ViewSequences(seqs, iSeqsSeed.back());
    seqs.MarkSequencesCommonTracks(m_iSeqsAdj, m_cmnTrkMarks);
    UpdateStructureAndInlierStates(m_cmnTrkMarks, seqs, seqErrLevels);
    ViewSequences(seqs, iSeqsSeed.back());

    for(uint iter = 1; iter < m_baaelLocalMaxNumIters; iter += 2) {
        DecreaseOutlierLevels_UpdateReprojectionErrorLevels(seqs, iSeqsSeed, m_iSeqsUpd,
                m_iSeqsErr, seqErrLevels);
        seqs.MarkSequencesCommonTracks(m_iSeqsUpd, m_cmnTrkMarks);
        if(!UpdateStructureAndInlierStates(m_cmnTrkMarks, seqs, seqErrLevels))
            break;
        //ViewSequences(seqs, iSeqsSeed.back());
        if(m_iSeqsErr.empty())
            BundleAdjust2DLocal(TsList, iSeqFix, iSeqsSeed, m_iSeqsAdj, m_iSeqsBA,
                                m_iTrksCmnBA, m_iSeqTrksIdvBA, seqs);
        else
            BundleAdjust2DLocal(TsList, iSeqFix, m_iSeqsErr, m_iSeqsAdj, m_iSeqsBA,
                                m_iTrksCmnBA, m_iSeqTrksIdvBA, seqs);
        ViewSequences(seqs, iSeqsSeed.back());

        DecreaseReprojectionErrorLevels_UpdateOutlierLevels(seqs, iSeqsSeed, m_iSeqsUpd,
                m_iSeqsErr, seqErrLevels);
        seqs.MarkSequencesCommonTracks(m_iSeqsAdj, m_cmnTrkMarks);
        if(!UpdateStructureAndInlierStates(m_cmnTrkMarks, seqs, seqErrLevels))
            break;
        //ViewSequences(seqs, iSeqsSeed.back());
        if(m_iSeqsErr.empty())
            BundleAdjust2DLocal(TsList, iSeqFix, iSeqsSeed, m_iSeqsAdj, m_iSeqsBA,
                                m_iTrksCmnBA, m_iSeqTrksIdvBA, seqs);
        else
            BundleAdjust2DLocal(TsList, iSeqFix, m_iSeqsErr, m_iSeqsAdj, m_iSeqsBA,
                                m_iTrksCmnBA, m_iSeqTrksIdvBA, seqs);
        //ViewSequences(seqs, iSeqsSeed.back());
        seqs.MarkSequencesCommonTracks(m_iSeqsAdj, m_cmnTrkMarks);
        if(!UpdateStructureAndInlierStates(m_cmnTrkMarks, seqs, seqErrLevels))
            break;
        ViewSequences(seqs, iSeqsSeed.back());
    }

    //FitCamerasToPlane(seqs);
    //OptimizePoints(seqs);
    //FitPointsToPlanes(seqs);
    //ViewSequences(seqs, iSeqsSeed.back());

    //BundleAdjust2DLocal(RtsList, iSeqFix, iSeqsSeed, m_iSeqsAdj, m_iSeqsBA, m_iTrksCmnBA, m_iSeqTrksIdvBA, seqs);
    //ViewSequences(seqs, iSeqsSeed.back());
}

void SequenceRegisteror::BundleAdjust2DAdaptiveErrorLevelGlobal(
    const std::vector<AlignedVector<RigidTransformation3D> > &TsList,
    const SequenceIndex &iSeqFix,
    SequenceSet &seqs, std::vector<SequenceErrorLevel> &seqErrLevels) {
    BundleAdjust2DGlobal(TsList, iSeqFix, m_iSeqsAdj, m_iSeqsBA, m_iTrksCmnBA,
                         m_iSeqTrksIdvBA, seqs);
    //seqs.SaveB("E:/tmp/test.txt");
    //exit(0);
    //seqs.LoadB("E:/tmp/test.txt");
    //seqs.GetSequenceIndexList(FLAG_SEQUENCE_STATE_REGISTRED, m_iSeqsAdj);
    //m_stop = true;

    seqs.MarkSequencesCommonTracks(m_iSeqsAdj, m_cmnTrkMarks);
    UpdateStructureAndInlierStates(m_cmnTrkMarks, seqs, seqErrLevels);
    ViewSequences(seqs, iSeqFix);

    for(uint iter = 2; iter < m_baaelGlobalMaxNumIters; iter += 2) {
        seqs.GetSequenceIndexList(FLAG_SEQUENCE_STATE_REGISTRED, m_iSeqsBA);
        DecreaseOutlierLevels_UpdateReprojectionErrorLevels(seqs, m_iSeqsBA, m_iSeqsUpd,
                m_iSeqsErr, seqErrLevels);
        seqs.MarkSequencesCommonTracks(m_iSeqsUpd, m_cmnTrkMarks);
        if(!UpdateStructureAndInlierStates(m_cmnTrkMarks, seqs, seqErrLevels))
            break;
        //ViewSequences(seqs, iSeqFix);
        if(m_iSeqsErr.empty())
            BundleAdjust2DGlobal(TsList, iSeqFix, m_iSeqsAdj, m_iSeqsBA, m_iTrksCmnBA,
                                 m_iSeqTrksIdvBA, seqs);
        else
            BundleAdjust2DLocal(TsList, iSeqFix, m_iSeqsErr, m_iSeqsAdj, m_iSeqsBA,
                                m_iTrksCmnBA, m_iSeqTrksIdvBA, seqs);
        ViewSequences(seqs, iSeqFix);

        DecreaseReprojectionErrorLevels_UpdateOutlierLevels(seqs, m_iSeqsAdj,
                m_iSeqsUpd, m_iSeqsErr, seqErrLevels);
        seqs.MarkSequencesCommonTracks(m_iSeqsAdj, m_cmnTrkMarks);
        if(!UpdateStructureAndInlierStates(m_cmnTrkMarks, seqs, seqErrLevels))
            break;
        //ViewSequences(seqs, iSeqFix);
        if(m_iSeqsErr.empty())
            BundleAdjust2DGlobal(TsList, iSeqFix, m_iSeqsAdj, m_iSeqsBA, m_iTrksCmnBA,
                                 m_iSeqTrksIdvBA, seqs);
        else
            BundleAdjust2DLocal(TsList, iSeqFix, m_iSeqsErr, m_iSeqsAdj, m_iSeqsBA,
                                m_iTrksCmnBA, m_iSeqTrksIdvBA, seqs);
        //ViewSequences(seqs, iSeqFix);
        seqs.MarkSequencesCommonTracks(m_iSeqsAdj, m_cmnTrkMarks);
        if(!UpdateStructureAndInlierStates(m_cmnTrkMarks, seqs, seqErrLevels))
            break;
        ViewSequences(seqs, iSeqFix);
    }

    //FitCamerasToPlane(seqs);
    //OptimizePoints(seqs);
    //FitPointsToPlanes(seqs);
    //ViewSequences(seqs, iSeqFix);

    if(m_baaelGlobalMaxNumIters >= 2) {
        BundleAdjust2DGlobal(TsList, iSeqFix, m_iSeqsAdj, m_iSeqsBA, m_iTrksCmnBA,
                             m_iSeqTrksIdvBA, seqs);
        ViewSequences(seqs, iSeqFix);
    }
}

bool SequenceRegisteror::UpdateStructureAndInlierStates(
    const std::vector<bool> &cmnTrkMarks, SequenceSet &seqs,
    const std::vector<SequenceErrorLevel> &seqErrLevels) {
    TrackIndex cntTrkIdvIn = 0, cntTrkIdvOut = 0, cntTrkIdvInToOut = 0,
               cntTrkIdvOutToIn = 0;
    SequenceIndex iSeq, i, j, iMin;
    TrackIndex iTrkCmn, iTrkIdv;
    float errSq, errSqSum, errSqSumMin;
    ubyte outlierOri;
    bool inlierNew, update;

    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    m_errSqThs.resize(nSeqs);
    for(iSeq = 0; iSeq < nSeqs; ++iSeq)
        m_errSqThs[iSeq] =
            m_sfmReprojErrSqThs[seqErrLevels[iSeq].GetReprojectionErrorLevel()];

    const TrackIndex nTrksCmn = seqs.GetCommonTracksNumber();
    for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn) {
        if(!cmnTrkMarks[iTrkCmn] ||
                seqs.CountCommonTrackRegisteredSequenceIndividualTracks(iTrkCmn) == 1)
            continue;
        update = false;
        if(!(seqs.GetCommonTrackState(iTrkCmn) & FLAG_COMMON_TRACK_STATE_REGISTERED)) {
            seqs.GetCommonPoint3DEstimatorDataInlier(iTrkCmn, m_Xdata, m_XdataTmp);
            m_Xestor.Triangulate(m_Xdata, m_Xcmn);
            seqs.SetCommonPoint(iTrkCmn, m_Xcmn);
            seqs.MarkCommonTrackRegistered(iTrkCmn);
            update = true;
        }
        m_Xcmn = seqs.GetCommonPoint(iTrkCmn);

        const SequenceTrackIndexList &iSeqTrksIdv =
            seqs.GetCommonTrackIndividualTrackIndexList(iTrkCmn);
        const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
        for(i = 0; i < nCrsps; ++i) {
            iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
            Sequence &seq = seqs[iSeq];
            if(!(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED) ||
                    !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER))
                continue;
            outlierOri = (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER);
            inlierNew = seq.ComputeTrackMSE(iTrkIdv, m_Xcmn, errSq) &&
                        errSq < m_errSqThs[iSeq];
            if(!outlierOri && !inlierNew) {
                seq.GetPoint3DEstimatorDataInlier(iTrkIdv, m_Xdata, true);
                //m_Xestor.m_ransacErrorThreshold = m_errSqThs[iSeq];
                if(m_Xestor.Triangulate(m_Xdata, m_Xidv))
                    seq.SetPoint(iTrkIdv, m_Xidv);
                //else
                //  seq.SetPoint(iTrkIdv, m_Xcmn);
                seq.MarkTrackCommonOutlier(iTrkIdv);
                ++cntTrkIdvInToOut;
                update = true;
            } else if(outlierOri && inlierNew) {
                seq.MarkTrackCommonInlier(iTrkIdv);
                ++cntTrkIdvOutToIn;
                update = true;
            }
            if(outlierOri)
                ++cntTrkIdvOut;
            else
                ++cntTrkIdvIn;
        }
        if(!update)
            continue;
        if(seqs.CountCommonTrackInlierIndividualTracks(iTrkCmn) == 0) {
            for(i = 0, iMin = INVALID_SEQUENCE_INDEX, errSqSumMin = FLT_MAX; i < nCrsps;
                    ++i) {
                iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
                if(!(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED) ||
                        !(seqs[iSeq].GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER))
                    continue;
                m_Xcmn = seqs[iSeq].GetPoint(iTrkIdv);
                for(j = 0, errSqSum = 0; j < nCrsps; ++j) {
                    iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
                    if(!(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED) ||
                            !(seqs[iSeq].GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER))
                        continue;
                    seqs.GetSequence(iSeq).ComputeTrackMSE(iTrkIdv, m_Xcmn, errSq);
                    errSqSum += errSq;
                }
                if(errSqSum < errSqSumMin) {
                    errSqSumMin = errSqSum;
                    iMin = i;
                    seqs.SetCommonPoint(iTrkCmn, m_Xcmn);
                }
            }
            if(iMin != INVALID_SEQUENCE_INDEX) {
                iSeqTrksIdv[iMin].Get(iSeq, iTrkIdv);
                seqs[iSeq].MarkTrackCommonInlier(iTrkIdv);
                --cntTrkIdvInToOut;
            }
        } else {
            seqs.GetCommonPoint3DEstimatorDataInlier(iTrkCmn, m_Xdata, m_XdataTmp);
            m_Xestor.OptimizeModel(m_Xdata, m_Xcmn);
            seqs.SetCommonPoint(iTrkCmn, m_Xcmn);
            for(i = 0; i < nCrsps; ++i) {
                iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
                if((seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED) &&
                        !(seqs[iSeq].GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
                    seqs[iSeq].SetPoint(iTrkIdv, m_Xcmn);
            }
        }
    }
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("----------------------------------------------------------------\n");
    printf("INLIER --> OUTLIER: %d / %d individual tracks\n", cntTrkIdvInToOut,
           cntTrkIdvIn);
    printf("INLIER <-- OUTLIER: %d / %d individual tracks\n", cntTrkIdvOutToIn,
           cntTrkIdvOut);
#endif

    return cntTrkIdvInToOut != 0 || cntTrkIdvOutToIn;
}

void SequenceRegisteror::DecreaseOutlierLevels_UpdateReprojectionErrorLevels(
    const SequenceSet &seqs, const SequenceIndexList &iSeqs,
    SequenceIndexList &iSeqsUpd,
    SequenceIndexList &iSeqsErr, std::vector<SequenceErrorLevel> &seqErrLevels) {
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("----------------------------------------------------------------\n");
    printf("Decreasing outlier levels & updating reprojection error levels...\n");
#endif

    SequenceIndex iSeq;
    TrackIndex iTrkIdv, iTrkCmn, cntTrkCmn, cntTrkIdvInlier, cntTrkIdvInlierMin;
    float errSq, errSqTh;

    iSeqsUpd.resize(0);
    iSeqsErr.resize(0);

    const SequenceIndex nSeqs = SequenceIndex(iSeqs.size());
    for(SequenceIndex i = 0; i < nSeqs; ++i) {
        iSeq = iSeqs[i];
        const Sequence &seq = seqs[iSeq];

        m_errSqs.resize(0);
        const TrackIndexList &iTrksIdvToCmn = seqs.GetSequenceCommonTrackIndexList(
                iSeq);
        const TrackIndex nTrksIdv = TrackIndex(iTrksIdvToCmn.size());
        for(iTrkIdv = cntTrkCmn = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
            if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) == INVALID_TRACK_INDEX ||
                    !(seqs.GetCommonTrackState(iTrkCmn) & FLAG_COMMON_TRACK_STATE_REGISTERED))
                continue;
            ++cntTrkCmn;
            m_Xcmn = seqs.GetCommonPoint(iTrkCmn);
            if(seq.ComputeTrackMSE(iTrkIdv, m_Xcmn, errSq))
                m_errSqs.push_back(errSq);
        }


        SequenceErrorLevel &seqErrLevel = seqErrLevels[iSeq];
        const SequenceErrorLevel seqErrLevelBkp = seqErrLevel;
        seqErrLevel.DecreaseOutlierErrorLevel();
        seqErrLevel.SetReprojectionErrorLevelLowest();
        cntTrkIdvInlierMin = TrackIndex(cntTrkCmn *
                                        m_sfmSeqInliersRatioThs[seqErrLevel.GetOutlierLevel()] + 0.5f);
        while(1) {
            errSqTh = m_sfmReprojErrSqThs[seqErrLevel.GetReprojectionErrorLevel()];
            cntTrkIdvInlier = 0;
            const TrackIndex N = TrackIndex(m_errSqs.size());
            for(TrackIndex i = 0; i < N; ++i) {
                if(m_errSqs[i] < errSqTh)
                    ++cntTrkIdvInlier;
            }
            if(cntTrkIdvInlier >= cntTrkIdvInlierMin ||
                    !seqErrLevel.IncreaseReprojectionErrorLevel())
                break;
        }
        if(seqErrLevel != seqErrLevelBkp)
            iSeqsUpd.push_back(iSeq);
        if(!seqErrLevel.IsLowest())
            iSeqsErr.push_back(iSeq);
#if VERBOSE_SEQUENCE_REGISTRATION
        if(seqErrLevel != seqErrLevelBkp || !seqErrLevel.IsLowest()) {
            printf("  Sequence %d: inliers = %d/%d = %.2f%%, error threshold = %.2f\n",
                   iSeq, cntTrkIdvInlier, cntTrkCmn, float(cntTrkIdvInlier) / cntTrkCmn * 100,
                   sqrt(errSqTh));
        }
#endif
    }
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Updated %d / %d sequences, %d / %d error sequences\n", iSeqsUpd.size(),
           nSeqs, iSeqsErr.size(), nSeqs);
#endif
}

void SequenceRegisteror::DecreaseReprojectionErrorLevels_UpdateOutlierLevels(
    const SequenceSet &seqs, const SequenceIndexList &iSeqs,
    SequenceIndexList &iSeqsUpd,
    SequenceIndexList &iSeqsErr, std::vector<SequenceErrorLevel> &seqErrLevels) {
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("----------------------------------------------------------------\n");
    printf("Decreasing reprojection error levels & updating outlier levels...\n");
#endif

    SequenceIndex iSeq;
    TrackIndex iTrkIdv, iTrkCmn, cntTrkCmn, cntTrkIdvInlier;
    float errSq, errSqTh, inlierRatio;

    iSeqsUpd.resize(0);
    iSeqsErr.resize(0);

    const SequenceIndex nSeqs = SequenceIndex(iSeqs.size());
    for(SequenceIndex i = 0; i < nSeqs; ++i) {
        iSeq = iSeqs[i];
        const Sequence &seq = seqs[iSeq];

        SequenceErrorLevel &seqErrLevel = seqErrLevels[iSeq];
        const SequenceErrorLevel seqErrLevelBkp = seqErrLevel;
        //seqErrLevel.SetReprojectionErrorLevelLowest();
        seqErrLevel.DecreaseReprojectionErrorLevel();
        errSqTh = m_sfmReprojErrSqThs[seqErrLevel.GetReprojectionErrorLevel()];

        const TrackIndexList &iTrksIdvToCmn = seqs.GetSequenceCommonTrackIndexList(
                iSeq);
        const TrackIndex nTrksIdv = TrackIndex(iTrksIdvToCmn.size());
        for(iTrkIdv = cntTrkCmn = cntTrkIdvInlier = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
            if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) == INVALID_TRACK_INDEX ||
                    !(seqs.GetCommonTrackState(iTrkCmn) & FLAG_COMMON_TRACK_STATE_REGISTERED))
                continue;
            ++cntTrkCmn;
            m_Xcmn = seqs.GetCommonPoint(iTrkCmn);
            if(seq.ComputeTrackMSE(iTrkIdv, m_Xcmn, errSq) && errSq < errSqTh)
                ++cntTrkIdvInlier;
        }
        inlierRatio = float(cntTrkIdvInlier) / cntTrkCmn;

        seqErrLevel.SetOutlierLevelLowest();
        while(1) {
            if(inlierRatio > m_sfmSeqInliersRatioThs[seqErrLevel.GetOutlierLevel()] ||
                    !seqErrLevel.IncreaseOutlierLevel())
                break;
        }
        if(seqErrLevel != seqErrLevelBkp)
            iSeqsUpd.push_back(iSeq);
        if(!seqErrLevel.IsLowest())
            iSeqsErr.push_back(iSeq);
#if VERBOSE_SEQUENCE_REGISTRATION
        if(seqErrLevel != seqErrLevelBkp || !seqErrLevel.IsLowest()) {
            printf("  Sequence %d: inliers = %d/%d = %.2f%%, error threshold = %.2f\n",
                   iSeq, cntTrkIdvInlier, cntTrkCmn, inlierRatio * 100,
                   sqrt(errSqTh));
        }
#endif
    }
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Updated %d / %d sequences, %d / %d error sequences\n", iSeqsUpd.size(),
           nSeqs, iSeqsErr.size(), nSeqs);
#endif
}

//void SequenceRegisteror::FitCamerasToPlane(SequenceSet &seqs)
//{
//  const SequenceIndex nSeqs = seqs.GetSequencesNumber();
//  SequenceIndex iSeq;
//  ushort idx;
//  for(iSeq = idx = 0; iSeq < nSeqs; ++iSeq)
//  {
//      if(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED)
//          idx += seqs[iSeq].GetFramesNumber();
//  }
//  const ushort N = idx;
//  m_Pdata.Resize(N);
//
//  m_iSeqFrms.resize(0);
//  FrameIndex iFrm;
//  for(iSeq = idx = 0; iSeq < nSeqs; ++iSeq)
//  {
//      if(!(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED))
//          continue;
//      const Sequence &seq = seqs[iSeq];
//      const FrameIndex nFrms = seq.GetFramesNumber();
//      for(iFrm = 0; iFrm < nFrms; ++iFrm, ++idx)
//      {
//          seq.GetCamera(iFrm).GetCenter(m_Pdata.X(idx));
//          m_iSeqFrms.push_back(SequenceFrameIndex(iSeq, iFrm));
//      }
//  }
//  m_Pestor.RunLosac(m_Pdata, m_P, m_inliers);
//
//  Point3D center;
//  const uint nInliers = uint(m_inliers.size());
//  for(uint i = 0; i < nInliers; ++i)
//  {
//      idx = m_inliers[i];
//      m_P.ComputeProjection(m_Pdata.X(idx), center);
//      m_iSeqFrms[idx].Get(iSeq, iFrm);
//      seqs[iSeq].SetCameraCenter(iFrm, center);
//  }
//}
//
//void SequenceRegisteror::OptimizePoints(SequenceSet &seqs)
//{
//  TrackIndex iTrkCmn;
//  const TrackIndex nTrksCmn = seqs.GetCommonTracksNumber();
//  for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
//  {
//      if(!(seqs.GetCommonTrackState(iTrkCmn) & FLAG_COMMON_TRACK_STATE_REGISTERED))
//          continue;
//      seqs.GetCommonPoint3DEstimatorDataInlier(iTrkCmn, m_Xdata);
//      m_Xcmn = seqs.GetCommonPoint(iTrkCmn);
//      m_Xestor.OptimizeModel(m_Xdata, m_Xcmn);
//      seqs.SetCommonPoint(iTrkCmn, m_Xcmn);
//  }
//
//  TrackIndex iTrkIdv;
//  const SequenceIndex nSeqs = seqs.GetSequencesNumber();
//  for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
//  {
//      if(!(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED))
//          continue;
//      const TrackIndexList &iTrksIdvToCmn = seqs.GetSequenceCommonTrackIndexList(iSeq);
//      Sequence &seq = seqs[iSeq];
//      const TrackIndex nTrksIdv = seq.GetTracksNumber();
//      for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
//      {
//          if(!(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) || (iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX
//          && !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
//              continue;
//          seq.GetPoint3DEstimatorDataInlier(iTrkIdv, m_Xdata);
//          m_Xidv = seq.GetPoint(iTrkIdv);
//          m_Xestor.OptimizeModel(m_Xdata, m_Xidv);
//          seq.SetPoint(iTrkIdv, m_Xidv);
//      }
//  }
//}
//
//void SequenceRegisteror::FitPointsToPlanes(SequenceSet &seqs)
//{
//  TrackIndex iTrkCmn;
//  m_iTrksCmn.resize(0);
//  const TrackIndex nTrksCmn = seqs.GetCommonTracksNumber();
//  for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
//  {
//      if(!(seqs.GetCommonTrackState(iTrkCmn) & FLAG_COMMON_TRACK_STATE_REGISTERED))
//          continue;
//      m_iTrksCmn.push_back(iTrkCmn);
//      seqs.SetCommonTrackPlaneIndex(iTrkCmn, 0);
//  }
//
//  m_iSeqTrksIdv.resize(0);
//  SequenceIndex iSeq;
//  TrackIndex iTrkIdv;
//  const SequenceIndex nSeqs = seqs.GetSequencesNumber();
//  for(iSeq = 0; iSeq < nSeqs; ++iSeq)
//  {
//      if(!(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED))
//          continue;
//      const TrackIndexList &iTrksIdvToCmn = seqs.GetSequenceCommonTrackIndexList(iSeq);
//      Sequence &seq = seqs[iSeq];
//      const TrackIndex nTrksIdv = seq.GetTracksNumber();
//      for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
//      {
//          if(!(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) || (iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX
//          && !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
//              continue;
//          m_iSeqTrksIdv.push_back(SequenceTrackIndex(iSeq, iTrkIdv));
//          seq.SetTrackPlaneIndex(iTrkIdv, 0);
//      }
//  }
//
//  uint i, i2, j;
//  const uint N1 = uint(m_iTrksCmn.size()), N2 = uint(m_iSeqTrksIdv.size()), N = N1 + N2;
//  m_Pdata.Resize(N);
//  for(i = 0; i < N1; ++i)
//      m_Pdata.X(i) = seqs.GetCommonPoint(m_iTrksCmn[i]);
//  for(i2 = 0; i2 < N2; ++i, ++i2)
//  {
//      m_iSeqTrksIdv[i2].Get(iSeq, iTrkIdv);
//      m_Pdata.X(i) = seqs[iSeq].GetPoint(iTrkIdv);
//  }
//#if VERBOSE_SEQUENCE_REGISTRATION
//  printf("----------------------------------------------------------------\n");
//#endif
//  m_Pestor.RunLosac(m_Pdata, m_PList, m_inliersList, m_sfmMaxNumPlanes, m_sfmMinNumPtsPerPlane, VERBOSE_SEQUENCE_REGISTRATION ? 1 : 0);
//
//  const uint nPlanes = uint(m_inliersList.size());
//  for(i = 0; i < nPlanes; ++i)
//  {
//      const Plane &P = m_PList[i];
//      const ubyte iPlane = i + 1;
//      const std::vector<uint> &inliers = m_inliersList[i];
//      const uint nInliers = uint(inliers.size());
//      for(j = 0; j < nInliers && inliers[j] < N1; ++j)
//      {
//          iTrkCmn = m_iTrksCmn[inliers[j]];
//          m_Xcmn = seqs.GetCommonPoint(iTrkCmn);
//          P.ComputeProjection(m_Xcmn, m_Xcmn);
//          seqs.SetCommonPoint(iTrkCmn, m_Xcmn);
//          seqs.SetCommonTrackPlaneIndex(iTrkCmn, iPlane);
//      }
//      for(; j < nInliers; ++j)
//      {
//          m_iSeqTrksIdv[inliers[j] - N1].Get(iSeq, iTrkIdv);
//          P.ComputeProjection(seqs[iSeq].GetPoint(iTrkIdv), m_Xidv);
//          seqs[iSeq].SetPoint(iTrkIdv, m_Xidv);
//          seqs[iSeq].SetTrackPlaneIndex(iTrkIdv, iPlane);
//      }
//  }
//  SetPlanesNumber(nPlanes);
//}