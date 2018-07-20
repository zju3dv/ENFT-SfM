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
#include "CameraTrackerDepth.h"

using namespace ENFT_SfM;

bool CameraTrackerDepth::EstimateInitialStructureAndMotion(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3, SequenceDepth &seq) {
    Camera C1, C3;
    seq.InitializeCameras();
    seq.InitializePoints();
    seq.InitializeMeasurements();
    seq.SearchForFrameFeatureMatches(iFrm1, iFrm3, m_matches, m_Tdata);
    if(m_Tdata.GetMatches3D().Size() < m_sfmTwoViewInliersMinNum)
        return false;
    m_Testor.RunLosac(m_Tdata, C3, m_inliers);
    m_Xestor.m_ransacErrorThreshold = m_Eestor.m_ransacErrorThreshold;
    m_Xdata.Resize(2);
    C1.MakeIdentity();
    m_Xdata.SetCamera(0, C1);
    m_Xdata.SetCamera(1, C3);
    m_Xdata.SetFocal(seq.GetIntrinsicMatrix().fxy());
    CameraTracker::m_rayDirs.Resize(2);

    FeatureIndex iFtr1, iFtr3;
    m_inliers.resize(0);
    const Point2D *x1s = seq.GetFrameFeatures(iFrm1), *x3s = seq.GetFrameFeatures(iFrm3);
    const float *d1s = seq.GetFrameDepths(iFrm1), *d3s = seq.GetFrameDepths(iFrm3);
    const ushort nMatches = ushort(m_matches.size());
    m_Xs.Resize(nMatches);
    for(ushort i = 0; i < nMatches; ++i) {
        m_matches[i].Get(iFtr1, iFtr3);
        m_Xdata.x(0) = x1s[iFtr1];
        m_Xdata.d(0) = d1s[iFtr1];
        m_Xdata.x(1) = x3s[iFtr3];
        m_Xdata.d(1) = d3s[iFtr3];
        m_Xdata.GetCamera(0).ComputeRayDirection(m_Xdata.x(0), CameraTracker::m_rayDirs[0]);
        m_Xdata.GetCamera(1).ComputeRayDirection(m_Xdata.x(1), CameraTracker::m_rayDirs[1]);
        m_Xdata.FinishSettingMatches();
        if(CameraTracker::m_rayDirs[0].Dot(CameraTracker::m_rayDirs[1]) < m_sfmTwoViewRayAngleDotTh && m_Xestor.Triangulate(m_Xdata, m_Xs[i]))
            m_inliers.push_back(i);
//#if _DEBUG
//      printf("%e\n", m_Xdata.ComputeSSE(m_Xs[i]) * m_Xdata.GetFactorSSEToMSE());
//#endif
    }
    const ushort nInliers = ushort(m_inliers.size()), nInliersTh = std::max(m_sfmTwoViewInliersMinNum, ushort(nMatches * m_sfmTwoViewInliersMinRatio + 0.5f));
#if VERBOSE_CAMERA_TRACKING
    printf("----------------------------------------------------------------\n");
    printf("  Frame %d --> %d: inliers = %d/%d = %.2f%%", iFrm1, iFrm3, nInliers, m_matches.size(), nInliers * 100.0f / m_matches.size());
#endif
    if(nInliers < nInliersTh) {
#if VERBOSE_CAMERA_TRACKING
        printf(" < %d, FAILED!\n", nInliersTh);
#endif
        return false;
    }
#if VERBOSE_CAMERA_TRACKING
    printf("\n");
#endif

    seq.SetCamera(iFrm1, C1);
    seq.MarkFrameSolved(iFrm1);
    seq.MarkFrameInitial(iFrm1);
    seq.SetCamera(iFrm3, C3);
    seq.MarkFrameSolved(iFrm3);
    seq.MarkFrameInitial(iFrm3);

    TrackIndex iTrk;
    MeasurementIndex iMea1, iMea3;
    m_Testor.FromInliersToInlierMarks(m_inliers, nMatches, m_inlierMarks);
    const MeasurementIndex iMea1Start = seq.GetFrameFirstMeasurementIndex(iFrm1), iMea3Start = seq.GetFrameFirstMeasurementIndex(iFrm3);
    for(ushort i = 0; i < nMatches; ++i) {
        iMea1 = iMea1Start + m_matches[i].GetIndex1();
        iMea3 = iMea3Start + m_matches[i].GetIndex2();
        iTrk = seq.GetMeasurementTrackIndex(iMea1);
        if(m_inlierMarks[i]) {
            seq.SetPoint(iTrk, m_Xs[i]);
            seq.MarkTrackSolvedAndInlier(iTrk);
            seq.MarkMeasurementInlier(iMea1);
            seq.MarkMeasurementInlier(iMea3);
            if(seq.GetDepth(iMea1) != 0.0f) {
                if(fabs(C1.ComputeDepth(m_Xs[i]) - seq.GetDepth(iMea1)) < m_depthErrTh)
                    seq.MarkMeasurementDepthInlier(iMea1);
                else
                    seq.MarkMeasurementDepthOutlier(iMea1);
            }
            if(seq.GetDepth(iMea3) != 0.0f) {
                if(fabs(C3.ComputeDepth(m_Xs[i]) - seq.GetDepth(iMea3)) < m_depthErrTh)
                    seq.MarkMeasurementDepthInlier(iMea3);
                else
                    seq.MarkMeasurementDepthOutlier(iMea3);
            }
        } else {
            seq.MarkMeasurementOutlier(iMea1);
            seq.MarkMeasurementOutlier(iMea3);
            seq.MarkMeasurementDepthOutlier(iMea1);
            seq.MarkMeasurementDepthOutlier(iMea3);
        }
        seq.MarkTrackInitial(iTrk);
    }

    FrameErrorLevel frmErrLevel;
    frmErrLevel.SetLowest();
    if(!EstimateIncrementalMotion_PnP(iFrm2, seq, frmErrLevel, true))
        return false;
#if VERBOSE_CAMERA_TRACKING
    printf("\n");
#endif
    seq.MarkFrameInitial(iFrm2);

    RunBundleAdjustmentGlobal(iFrm1, iFrm3, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, true);

    return true;
}

float CameraTrackerDepth::ComputeFrameDepthConfidence(const SequenceDepth &seq, const FrameIndex &iFrm) {
    Point2D mean;
    LA::Vector3f cov;
    seq.GetFrameInlierDepthMeasurementIndexList(iFrm, m_iMeasInlier);
    seq.ComputeMeasurementsDistribution(m_iMeasInlier, mean, cov);
    return m_iMeasInlier.size() * sqrt(cov.v0() * cov.v2() - cov.v1() * cov.v1());
}

float CameraTrackerDepth::ComputeFrameMSE(const SequenceDepth &seq, const FrameIndex &iFrm) {
    FeatureIndex iFtr;
    TrackIndex iTrk;
    FeatureIndex cnt;
    Point2D e1;
    float e2, d;
    float SSE1, SSE2;
    const Camera &C = seq.GetCamera(iFrm);
    const FeatureIndex nFtrs = seq.GetFrameFeaturesNumber(iFrm);
    const TrackIndex *iTrks = seq.GetFrameTrackIndexes(iFrm);
    const Point2D *xs = seq.GetFrameFeatures(iFrm);
    const MeasurementState *meaStates = seq.GetFrameMeasurementStates(iFrm);
    const float *ds = seq.GetFrameDepths(iFrm);

    cnt = 0;
    SSE1 = SSE2 = 0.0f;
    for(iFtr = 0; iFtr < nFtrs; ++iFtr) {
        if((iTrk = iTrks[iFtr]) == INVALID_TRACK_INDEX || !(seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER)
                || (meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
            continue;
        C.ComputeProjectionError(seq.GetPoint(iTrk), xs[iFtr], d, e1);
        ++cnt;
        SSE1 += e1.SquaredLength();
        if(ds[iFtr] == 0.0f)
            continue;
        e2 = ds[iFtr] - d;
        SSE2 += e2 * e2;
    }
    return (SSE1 + m_depthErrTh * SSE2) / cnt;
}

bool CameraTrackerDepth::EstimateIncrementalMotion_PnP(const FrameIndex &iFrm, Sequence &seq, FrameErrorLevel &frmErrLevel, const bool metric) {
#if _DEBUG
    assert(metric);
#endif
    SequenceDepth *pSeq = (SequenceDepth *) &seq;
    pSeq->GetCameraEstimatorData(iFrm, m_Cdata, m_iMeas);
    const ushort N = ushort(m_iMeas.size());
    if(N < m_sfmCamInliersMinNum)
        return false;

    Point2D mean;
    LA::Vector3f cov;
    seq.ComputeMeasurementsDistribution(m_iMeas, mean, cov);
    const float areaInlierTrk = sqrt(cov.v0() * cov.v2() - cov.v1() * cov.v1());

    ushort i, nInliers, nInliersTh;
    float errSqTh, areaInlierFtr, areaInlierFtrMin;
    frmErrLevel.SetLowest();
    while(1) {
        errSqTh = m_sfmReprojErrSqThs[frmErrLevel.GetReprojectionErrorLevel()];
        m_Cestor.m_ransacErrorThreshold = errSqTh;
        //m_Cestor.RunLoArsac(m_Cdata, m_C, m_inliers);
        m_Cestor.RunLosac(m_Cdata, m_C, m_inliers);
        nInliersTh = std::max(m_sfmCamInliersMinNum, ushort(N * m_sfmCamInlierRatioThs[frmErrLevel.GetOutlierLevel()] + 0.5f));
        //if((nInliers = ushort(m_inliers.size())) >= nInliersTh || !frmErrLevel.Increase())
        //  break;
        nInliers = ushort(m_inliers.size());
        m_iMeasInlier.resize(nInliers);
        for(i = 0; i < nInliers; ++i)
            m_iMeasInlier[i] = m_iMeas[m_inliers[i]];
        areaInlierFtrMin = areaInlierTrk * m_sfmCamInlierRatioThs[frmErrLevel.GetOutlierLevel()];
        seq.ComputeMeasurementsDistribution(m_iMeasInlier, mean, cov);
        //if((areaInlierFtr = sqrt(cov.v0() * cov.v2() - cov.v1() * cov.v1())) >= areaInlierFtrMin && nInliers >= m_sfmCamInliersMinNum || !frmErrLevel.Increase())
        //  break;
        areaInlierFtr = sqrt(cov.v0() * cov.v2() - cov.v1() * cov.v1());
#if VERBOSE_CAMERA_TRACKING
        printf("\r  Frame %d: inliers = %d/%d = %.2f%%, error <= %.2f", iFrm, nInliers, N, nInliers == 0 ? 0 : nInliers * 100.0f / N,
               sqrt(errSqTh * seq.GetIntrinsicMatrix().fxy()));
        //printf("\r  Frame %d: inliers = %d/%d = %.2f%%, error <= %.2f", iFrm, m_inliers.size(), N, areaInlierFtr * 100.0f / areaInlierTrk,
        //  sqrt(errSqTh * seq.GetIntrinsicMatrix().fxy()));
#endif
        if(areaInlierFtr >= areaInlierFtrMin && nInliers >= nInliersTh || !frmErrLevel.Increase())
            break;
    }

    //if(nInliers < nInliersTh)
    if(frmErrLevel.IsHighest()) {
#if VERBOSE_CAMERA_TRACKING
        printf(", FAILED!\n");
#endif
        return false;
    }
    seq.SetCamera(iFrm, m_C);
    seq.MarkFrameSolved(iFrm);

    MeasurementIndex iMea;
    m_Cestor.FromInliersToInlierMarks(m_inliers, N, m_inlierMarks);
    for(i = 0; i < N; ++i) {
        iMea = m_iMeas[i];
        if(m_inlierMarks[i]) {
            seq.MarkMeasurementInlier(iMea);
            if(pSeq->GetDepth(iMea) != 0.0f) {
                if(fabs(m_C.ComputeDepth(seq.GetPoint(seq.GetMeasurementTrackIndex(iMea))) - pSeq->GetDepth(iMea)) < m_depthErrTh)
                    pSeq->MarkMeasurementDepthInlier(iMea);
                else
                    pSeq->MarkMeasurementDepthOutlier(iMea);
            }
        } else {
            seq.MarkMeasurementOutlier(iMea);
            pSeq->MarkMeasurementDepthOutlier(iMea);
        }
    }
    return true;
}

void CameraTrackerDepth::TransformScene(const FrameIndex &iFrm1, const FrameIndex &iFrm2, Sequence &seq) {
    if(iFrm1 < seq.GetFramesNumber())
        seq.SetReferenceFrame(iFrm1);
}

void CameraTrackerDepth::BundleAdjust(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, FrameIndexList &iFrmsBA, Sequence &seq,
                                      const uint &maxNumIters, const float &stopMSE, const float &stopRelativeReduction, const bool metric) {
    SequenceDepth *pSeq = (SequenceDepth *) &seq;
    pSeq->GetBundleAdjustorData(iFrmsAdj, iTrksAdj, m_baData, iFrmsBA);
    m_ba.m_lmMaxNumIters = maxNumIters;
    m_ba.m_lmStopMSE = stopMSE;
    m_ba.m_lmStopRelativeReduction = stopRelativeReduction;
//#if _DEBUG
//  printf("%f\n", m_baData.ComputeSSE() * seq.GetIntrinsicMatrix().fxy() / m_baData.GetMeasurementsNumber());
//#endif
    const FrameIndex nFrmsFix = FrameIndex(iFrmsBA.size() - iFrmsAdj.size());
    m_ba.Run(m_baData, nFrmsFix, seq.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT, VERBOSE_CAMERA_TRACKING ? 2 : 0);
    pSeq->SetBunbleAdjustmentResults(m_baData, nFrmsFix, iFrmsBA, iTrksAdj);
//  return m_baData.ComputeSSE();
}

void CameraTrackerDepth::RunPointsOptimization(const TrackIndexList &iTrks, Sequence &seq, const bool metric) {
#if _DEBUG
    assert(metric);
#endif
    SequenceDepth *pSeq = (SequenceDepth *) &seq;
    const TrackIndex nTrks = TrackIndex(iTrks.size());
    for(TrackIndex i = 0; i < nTrks; ++i) {
        const TrackIndex iTrk = iTrks[i];
        pSeq->GetPoint3DEstimatorDataInlier(iTrk, m_Xdata);
        m_X = seq.GetPoint(iTrk);
        m_Xestor.OptimizeModel(m_Xdata, m_X);
        //m_Xestor.Triangulate(m_Xdata, m_X);
        seq.SetPoint(iTrk, m_X);
        //SSE += m_Xdata.ComputeSSE(m_X);
    }
}

bool CameraTrackerDepth::UpdateStructureAndInlierStates(const std::vector<bool> &trkMarks, Sequence &seq, const std::vector<FrameErrorLevel> &frmErrLevels,
        const bool metric, const bool triangulateUnstablePts) {
#if _DEBUG
    assert(metric);
#endif
    TrackIndex cntTrkIn = 0, cntTrkOut = 0, cntTrkInToOut = 0, cntTrkOutToIn = 0, cntTrkUpd = 0;
    MeasurementIndex cntMeaInToOut = 0, cntMeaOutToIn = 0, cntDepInToOut = 0, cntDepOutToIn = 0;
    FrameIndex iFrm;
    TrackIndex iTrk;
    MeasurementIndex iMea;
    ushort i, nInliers;
    float errSqTh;
    double fitErr;
    bool update;

    m_Xestor.m_ransacErrorThreshold = 1.0f;

    SequenceDepth *pSeq = (SequenceDepth *) &seq;
    const TrackIndex nTrks = TrackIndex(seq.GetTracksNumber());
    for(iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(!trkMarks[iTrk])
            continue;
        pSeq->GetPoint3DEstimatorData(iTrk, m_Xdata, m_iMeas);
        const ushort N = ushort(m_iMeas.size());
        if(N < m_sfmPtInliersMinNum)
            continue;
        const ushort nInliersTh = std::max(m_sfmPtInliersMinNum, ushort(N * m_sfmPtInliersMinRatio + 0.5f));
        m_Xdata.ValidateWeights();
        for(i = 0; i < N; ++i) {
            iFrm = seq.GetMeasurementFrameIndex(m_iMeas[i]);
            errSqTh = m_sfmReprojErrSqThs[frmErrLevels[iFrm].GetReprojectionErrorLevel()];
            m_Xdata.SetSquaredWeight(i, 1 / errSqTh);
        }
        if(seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER) {
            m_Xestor.VerifyModel(m_Xdata, seq.GetPoint(iTrk), m_inliers, fitErr);
            ++cntTrkIn;
        } else {
            m_inliers.resize(0);
            ++cntTrkOut;
        }
        if((nInliers = ushort(m_inliers.size())) < nInliersTh) {
            //if(metric && (triangulateUnstablePts || ComputeMinimalRayAngleDot_RankTriangulationPairs(m_Xdata, m_triPairs) < m_sfmMultiViewRayAngleDotTh)
            //&& m_Xestor.Triangulate(m_Xdata, m_X, m_inliers, m_triPairs, nInliersTh)
            //|| !metric && m_Xestor.Triangulate(m_Xdata, m_X, m_inliers, nInliersTh))
            if((!metric || triangulateUnstablePts || ComputeMinimalRayAngleDot(m_Xdata) < m_sfmMultiViewRayAngleDotTh)
                    && m_Xestor.Triangulate(m_Xdata, m_X, m_inliers, nInliersTh)) {
                seq.SetPoint(iTrk, m_X);
                seq.MarkTrackSolved(iTrk);
            }
//#if _DEBUG
//          else
//          {
//              printf("...\n");
//              seq.GetPoint3DEstimatorData(iTrk, m_Xdata, m_iMeas, metric);
//              m_X = m_XsBkp[m_iTrksKF[iTrk]];
//              m_Xestor.Triangulate(m_Xdata, m_X, m_inliers, nInliersTh);
//          }
//#endif
        }
        //else if(triangulateUnstablePts)
        //{
        //  m_Xestor.FromInliersToInlierMarks(m_inliers, N, m_inlierMarks);
        //  if(CheckMaximalRayAngleSmall_RankTriangulationPairs(m_Xdata, m_inlierMarks, m_triPairs))
        //  {
        //      if(m_Xestor.Triangulate(m_Xdata, m_X, m_inliers, m_triPairs, nInliersTh) && ushort(m_inliers.size()) >= nInliers)
        //      {
        //          seq.SetPoint(iTrk, m_X);
        //          seq.MarkTrackSolved(iTrk);
        //      }
        //      else
        //          m_Xestor.FromInlierMarksToInliers(m_inlierMarks, m_inliers);
        //  }
        //}
        update = false;
        if((nInliers = ushort(m_inliers.size())) < nInliersTh && (seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER)) {
            seq.MarkTrackOutlier(iTrk);
            ++cntTrkInToOut;
            update = true;
        } else if(nInliers >= nInliersTh && !(seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER)) {
            seq.MarkTrackInlier(iTrk);
            ++cntTrkOutToIn;
            update = true;
        }
        if(nInliers >= nInliersTh) {
            const Point3D &X = seq.GetPoint(iTrk);
            CameraTracker::m_Xestor.FromInliersToInlierMarks(m_inliers, N, m_inlierMarks);
            for(i = 0; i < N; ++i) {
                iMea = m_iMeas[i];
                if(m_inlierMarks[i] && (seq.GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER)) {
                    seq.MarkMeasurementInlier(iMea);
                    ++cntMeaOutToIn;
                    update = true;
                } else if(!m_inlierMarks[i] && !(seq.GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER)) {
                    seq.MarkMeasurementOutlier(iMea);
                    ++cntMeaInToOut;
                    update = true;
                }
                if(pSeq->GetDepth(iMea) == 0.0f)
                    continue;
                if(m_inlierMarks[i]) {
                    const bool depIn = fabs(pSeq->GetDepth(iMea) - m_Xdata.GetCamera(i).ComputeDepth(X)) < m_depthErrTh;
                    if(depIn && (seq.GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH)) {
                        pSeq->MarkMeasurementDepthInlier(iMea);
                        ++cntDepOutToIn;
                        update = true;
                    } else if(!depIn && !(seq.GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH)) {
                        pSeq->MarkMeasurementDepthOutlier(iMea);
                        ++cntDepInToOut;
                        update = true;
                    }
                }
            }
            //if(update)
            //{
            //  m_Xdata.SetSubset(m_inliers);
            //  m_X = seq.GetPoint(iTrk);
            //  m_Xestor.OptimizeModel(m_Xdata, m_X);
            //  seq.SetPoint(iTrk, m_X);
            //  ++cntTrkUpd;
            //}
        }
        if(update)
            ++cntTrkUpd;
    }
#if VERBOSE_CAMERA_TRACKING
    printf("----------------------------------------------------------------\n");
    printf("INLIER --> OUTLIER: %d / %d tracks, %d measurements, %d depths\n", cntTrkInToOut, cntTrkIn, cntMeaInToOut, cntDepInToOut);
    printf("INLIER <-- OUTLIER: %d / %d tracks, %d measurements, %d depths\n", cntTrkOutToIn, cntTrkOut, cntMeaOutToIn, cntDepOutToIn);
#endif

//#if _DEBUG
//  m_stop = true;
//  ViewSequence(seq, 19);
//#endif

    return cntTrkUpd != 0;
}