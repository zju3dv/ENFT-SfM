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
#include "CameraTracker.h"
//#if _DEBUG
//#include "ViewerFeatureMatching.h"
//#endif
//#include <cvd/image_io.h>

using namespace ENFT_SfM;

bool CameraTracker::EstimateInitialStructureAndMotion(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3, Sequence &seq,
        const bool metric) {
    Camera C1, C3;
    ProjectiveMatrix P1, P3;
    seq.InitializeCameras();
    seq.InitializePoints();
    seq.InitializeMeasurements();
    srand(5);
    if(metric) {
        seq.SearchForFrameFeatureMatches(iFrm1, iFrm3, m_matches, m_Edata, false);
        if(m_Edata.Size() < m_sfmTwoViewInliersMinNum)
            return false;
        m_Eestor.RunLosac(m_Edata, m_E, m_inliers);
        if(!m_E.ToRelativePose(m_Edata, C3, m_sfmTwoViewInliersMinRatio, m_work))
            return false;
    } else {
        seq.SearchForFrameFeatureMatches(iFrm1, iFrm3, m_matches, m_Fdata, false);
        if(m_Fdata.Size() < m_sfmTwoViewInliersMinNum)
            return false;
        m_Festor.RunLosac(m_Fdata, m_F, m_inliers/*, 4*/);
        m_F.EnforceUnitFrobeniusNorm();
        if(!m_F.ToRelativeProjectiveMatrix(P3, m_work))
            return false;
        C3 = P3;
    }

    m_Xestor.m_ransacErrorThreshold = m_Eestor.m_ransacErrorThreshold;
    if(metric)
        m_Xestor.SetMetric();
    else
        m_Xestor.SetProjective();
    m_Xdata.Resize(2);
    C1.MakeIdentity();
    //m_C2.Scale(m_sfmTwoViewScale);
    m_Xdata.SetCamera(0, C1);
    m_Xdata.SetCamera(1, C3);
    m_Xdata.SetFocal(seq.GetIntrinsicMatrix().fxy());
    m_rayDirs.Resize(2);

    m_inliers.resize(0);
    const Point2D *x1s = seq.GetFrameFeatures(iFrm1), *x3s = seq.GetFrameFeatures(iFrm3);
    const ushort nMatches = ushort(m_matches.size());
    m_Xs.Resize(nMatches);
    for(ushort i = 0; i < nMatches; ++i) {
        m_Xdata.x(0) = x1s[m_matches[i].GetIndex1()];
        m_Xdata.x(1) = x3s[m_matches[i].GetIndex2()];
        m_Xdata.GetCamera(0).ComputeRayDirection(m_Xdata.x(0), m_rayDirs[0]);
        m_Xdata.GetCamera(1).ComputeRayDirection(m_Xdata.x(1), m_rayDirs[1]);
        if((!metric || m_rayDirs[0].Dot(m_rayDirs[1]) < m_sfmTwoViewRayAngleDotTh) && m_Xestor.Triangulate(m_Xdata, m_Xs[i]))
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

    if(metric) {
        seq.SetCamera(iFrm1, C1);
        seq.SetCamera(iFrm3, C3);
    } else {
        P1.Set(C1);
        seq.SetCamera(iFrm1, P1);
        seq.SetCamera(iFrm3, P3);
    }
    seq.MarkFrameSolved(iFrm1);
    seq.MarkFrameInitial(iFrm1);
    seq.MarkFrameSolved(iFrm3);
    seq.MarkFrameInitial(iFrm3);

    TrackIndex iTrk;
    MeasurementIndex iMea1, iMea3;
    m_Eestor.FromInliersToInlierMarks(m_inliers, nMatches, m_inlierMarks);
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
        } else {
            seq.MarkMeasurementOutlier(iMea1);
            seq.MarkMeasurementOutlier(iMea3);
        }
        seq.MarkTrackInitial(iTrk);
    }

    FrameErrorLevel frmErrLevel;
    frmErrLevel.SetLowest();
    if(!EstimateIncrementalMotion_PnP(iFrm2, seq, frmErrLevel, metric))
        return false;
#if VERBOSE_CAMERA_TRACKING
    printf("\n");
#endif
    seq.MarkFrameInitial(iFrm2);

    //printf("----------------------------------------------------------------\n");
    //seq.GetProjectiveMatrix(iFrm1).Print();
    //printf("----------------------------------------------------------------\n");
    //seq.GetProjectiveMatrix(iFrm2).Print();
    //printf("----------------------------------------------------------------\n");
    //seq.GetProjectiveMatrix(iFrm3).Print();
    RunBundleAdjustmentGlobal(iFrm1, iFrm3, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
    //printf("----------------------------------------------------------------\n");
    //seq.GetProjectiveMatrix(iFrm1).Print();
    //printf("----------------------------------------------------------------\n");
    //seq.GetProjectiveMatrix(iFrm2).Print();
    //printf("----------------------------------------------------------------\n");
    //seq.GetProjectiveMatrix(iFrm3).Print();

    return true;
}

float CameraTracker::ComputeImageBasedDistance(const Sequence &seq, const FrameIndex &iFrm1, const FrameIndex &iFrm2) {
    //seq.SearchForFrameFeatureMatches(iFrm1, iFrm2, m_matches, m_Hdata, false);
    seq.SearchForFrameFeatureMatchesInlierMeasurement(iFrm1, iFrm2, m_matches, m_Hdata, false);
    m_Hestor.RunLosac(m_Hdata, m_H, m_inliers);
    if(m_Hestor.m_ransacErrorThreshold == FLT_MAX) {
        //if(int(m_inliers.size()) < int(m_Hdata.Size() * m_sfmTwoViewHomoRatioTh + 0.5f))
        //{
        //  const float w = float(seq.GetImageWidth()), h = float(seq.GetImageHeight());
        //  return sqrt(w * w + h * h);
        //}
        //const float thBkp = m_Hestor.m_ransacErrorThreshold;
        //m_Hestor.m_ransacErrorThreshold = FLT_MAX;
        m_Hestor.VerifyModel(m_Hdata, m_H, m_errSqs);
        //m_Hestor.m_ransacErrorThreshold = thBkp;

        const int ith = int(m_errSqs.size() * m_sfmInitHomoRatio);
        std::nth_element(m_errSqs.begin(), m_errSqs.begin() + ith, m_errSqs.end());
        return sqrt(m_errSqs[ith] * seq.GetIntrinsicMatrix().fxy()) * m_inliers.size();
        //float errSqSum = 0.0f;
        //const int N = int(m_errSqs.size());
        //for(int i = 0; i < N; ++i)
        //  errSqSum = m_errSqs[i] + errSqSum;
        //return sqrt(errSqSum / N * seq.GetIntrinsicMatrix().fxy());
    } else
        return float(m_inliers.size());
}

void CameraTracker::EstimateIncrementalMotion(const FrameIndexList &iFrmsIncr, FrameIndexList &iFrmsSolve, Sequence &seq,
        std::vector<FrameErrorLevel> &frmErrLevels, const bool metric) {
#if VERBOSE_CAMERA_TRACKING
    printf("Estimating motion...\n");
#endif

    iFrmsSolve.resize(0);
    const FrameIndex nFrms = seq.GetFramesNumber(), nFrmsIncr = FrameIndex(iFrmsIncr.size());
    for(FrameIndex i = 0; i < nFrmsIncr; ++i) {
        const FrameIndex iFrm = iFrmsIncr[i];
        FrameErrorLevel &frmErrLevel = frmErrLevels[iFrm];
        if(EstimateIncrementalMotion_PnP(iFrm, seq, frmErrLevel, metric)) {
            iFrmsSolve.push_back(iFrm);
            ViewSequence(seq, iFrm);
        }
    }

#if VERBOSE_CAMERA_TRACKING
    printf("\nTotal: %d / %d cameras succeed\n", iFrmsSolve.size(), nFrmsIncr);
#endif
}

bool CameraTracker::EstimateIncrementalMotion_PnP(const FrameIndex &iFrm, Sequence &seq, FrameErrorLevel &frmErrLevel, const bool metric) {
    const Sequence::IntrinsicType intrinsicType = seq.GetIntrinsicType();
    const bool projective = intrinsicType == Sequence::INTRINSIC_VARIABLE || !metric;
    if(projective) {
#if _DEBUG
        assert(intrinsicType != Sequence::INTRINSIC_USER_FIXED);
#endif
        seq.GetProjectiveMatrixEstimatorData(iFrm, m_Pdata, m_iMeas, false);
        if(metric) {
            m_Pestor.SetMetric(m_sfmFocalPriorWeight);

            float fMin, fMax;
            seq.GetIntrinsicRectificationFocalRange(fMin, fMax);
            fMin /= m_sfmFocalRangeFactor;
            fMax *= m_sfmFocalRangeFactor;
            m_Pestor.SetFocalRange(fMin, fMax);
            //printf("(%f, %f)\n", fMin, fMax);
        } else
            m_Pestor.SetProjective();
    } else
        seq.GetCameraEstimatorData(iFrm, m_Cdata, m_iMeas, true);
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
        if(projective) {
            m_Pestor.m_ransacErrorThreshold = errSqTh;
            m_Pestor.RunLoArsac(m_Pdata, m_P, m_inliers/*, 4*/);
        } else {
            m_Cestor.m_ransacErrorThreshold = errSqTh;
            if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
                m_Cestor.m_ransacErrorThreshold /= seq.GetIntrinsicRectification().f() * seq.GetIntrinsicRectification().f();
            m_Cestor.RunLoArsac(m_Cdata, m_C, m_inliers);
            //m_Cestor.RunLosac(m_Cdata, m_C, m_inliers, 4);
        }
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
    if(projective) {
        if(metric) {
#if _DEBUG
            assert(intrinsicType == Sequence::INTRINSIC_VARIABLE);
#endif
            float f;
            if(!m_P.ToIdealIntrinsicExtrinsic(f, m_C, m_work))
                return false;
            seq.SetCamera(iFrm, m_C, f);
        } else
            seq.SetCamera(iFrm, m_P);
    } else
        seq.SetCamera(iFrm, m_C);
    seq.MarkFrameSolved(iFrm);
    m_Cestor.FromInliersToInlierMarks(m_inliers, N, m_inlierMarks);
    for(i = 0; i < N; ++i) {
        if(m_inlierMarks[i])
            seq.MarkMeasurementInlier(m_iMeas[i]);
        else
            seq.MarkMeasurementOutlier(m_iMeas[i]);
    }
    return true;
}

void CameraTracker::UpdateProjectiveToMetric(const FrameIndex &iFrm1, const FrameIndex &iFrm2, Sequence &seq, std::vector<FrameErrorLevel> &frmErrLevels) {
    seq.GetFrameIndexList(FLAG_FRAME_STATE_SOLVED, m_iFrmsSolve);
    const FrameIndex nFrmsSolve = FrameIndex(m_iFrmsSolve.size());
    m_Ps.Resize(nFrmsSolve);
    for(FrameIndex i = 0; i < nFrmsSolve; ++i)
        m_Ps[i] = seq.GetProjectiveMatrix(m_iFrmsSolve[i]);

//#if _DEBUG
//  Sequence::SaveProjectiveMatrixes("F:/tmp/test.txt", m_Ps);
//#endif

    float Ecalib;
    m_Qestor.Run(m_Ps, m_Q, Ecalib);
    seq.UpdateToMetric(m_Q);

//#if _DEBUG
//  m_stop = true;
//  ViewSequence(seq, iFrm1);
//#endif

    FrameIndex iFrm;
    m_Pestor.SetMetric(m_sfmFocalPriorWeight);
    m_Pestor.SetFocalRange(1 / m_sfmFocalRangeFactor, m_sfmFocalRangeFactor);
    const Sequence::IntrinsicType intrinsicType = seq.GetIntrinsicType();
    if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
        m_fs.resize(nFrmsSolve);
    for(FrameIndex i = 0; i < nFrmsSolve; ++i) {
        iFrm = m_iFrmsSolve[i];
        m_PdataMetric.SetPriorWeight(m_sfmFocalPriorWeight);
        seq.GetProjectiveMatrixEstimatorDataInlier(iFrm, m_PdataMetric, true);
        m_PMetric.Set(1.0f, seq.GetCamera(iFrm));
//#if _DEBUG
//#if VERBOSE_CAMERA_TRACKING
//      printf("----------------------------------------------------------------\n");
//      printf("Frame %d\n", iFrm);
//      m_Pestor.OptimizeModel(m_PdataMetric, m_PMetric, 4);
//#endif
//#else
        m_Pestor.OptimizeModel(m_PdataMetric, m_PMetric);
//#endif
        if(intrinsicType == Sequence::INTRINSIC_CONSTANT) {
            seq.SetCamera(iFrm, m_PMetric.C());
            m_fs[i] = seq.GetIntrinsicRectification().f() * m_PMetric.f();
        } else if(intrinsicType == Sequence::INTRINSIC_VARIABLE)
            seq.SetCamera(iFrm, m_PMetric.C(), seq.GetIntrinsicRectification(iFrm).f() * m_PMetric.f());
    }
    if(intrinsicType == Sequence::INTRINSIC_CONSTANT) {
        const FrameIndex ith = FrameIndex(nFrmsSolve >> 1);
        std::nth_element(m_fs.begin(), m_fs.begin() + ith, m_fs.end());
        seq.SetIntrinsicRectification(m_fs[ith], 0.0f);
    }

    //seq.GetTrackIndexList(FLAG_TRACK_STATE_INLIER, m_iTrksAdj);
    //RunPointsOptimization(m_iTrksAdj, seq, true);

//#if _DEBUG
//  m_stop = true;
//  ViewSequence(seq, iFrm1);
//#endif

    TransformScene(iFrm1, iFrm2, seq);
    //RunBundleAdjustmentGlobal(iFrm1, iFrm2, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, true);
    BundleAdjustAdaptiveErrorLevelGlobal(iFrm1, iFrm2, seq, frmErrLevels, true, false);
}

void CameraTracker::TransformScene(const FrameIndex &iFrm1, const FrameIndex &iFrm2, Sequence &seq) {
    if(iFrm1 < seq.GetFramesNumber())
        seq.SetReferenceFrame(iFrm1);
    if(iFrm2 < seq.GetFramesNumber()) {
        Point3D center;
        seq.GetCamera(iFrm2).GetCenter(center);
        const float dist = sqrt(center.SquaredLength());
        seq.ScaleScene(m_sfmTwoViewScale / dist);
    }
}

void CameraTracker::RunBundleAdjustmentLocal(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndexList &iFrmsSeed, FrameIndexList &iFrmsAdj,
        FrameIndexList &iFrmsBA, TrackIndexList &iTrksAdj, Sequence &seq, const bool metric) {
#if VERBOSE_CAMERA_TRACKING
    printf("----------------------------------------------------------------\n");
    printf("Local bundle adjustment...\n");
#endif
    const FrameIndex nFrms = seq.GetFramesNumber();
    iFrmsAdj.resize(0);
    const Sequence::IntrinsicType intrinsicType = seq.GetIntrinsicType();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        if((seq.GetFrameState(iFrm) & FLAG_FRAME_STATE_SOLVED) && (iFrm != iFrm1 || metric && intrinsicType == Sequence::INTRINSIC_VARIABLE))
            iFrmsAdj.push_back(iFrm);
    }
    const FrameIndex nFrmsSeed = FrameIndex(iFrmsSeed.size()), nFrmsAdj = FrameIndex(iFrmsAdj.size());
    const FrameIndex nFrmsLocalCandidate = nFrmsAdj >= nFrmsSeed ? nFrmsAdj - nFrmsSeed : 0;
    if(nFrmsLocalCandidate > m_baLocalMaxNumAdditionalAdjustedFrms) {
        m_frmMarks.assign(nFrms, false);
        m_trkMarks.assign(seq.GetTracksNumber(), false);
        for(FrameIndex i = 0; i < nFrmsSeed; ++i) {
            const FrameIndex iFrm = iFrmsSeed[i];
            m_frmMarks[iFrm] = true;
            const TrackIndex *iTrks = seq.GetFrameTrackIndexes(iFrm);
            const MeasurementState *meaStates = seq.GetFrameMeasurementStates(iFrm);
            const FeatureIndex nFtrs = seq.GetFrameFeaturesNumber(iFrm);
            for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr) {
                const TrackIndex iTrk = iTrks[iFtr];
                if(iTrk != INVALID_TRACK_INDEX && !m_trkMarks[iTrk] && (seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER)
                        && !(meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
                    m_trkMarks[iTrk] = true;
            }
        }
        m_candidateFrms.resize(0);
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            if(!m_frmMarks[iFrm] && (seq.GetFrameState(iFrm) & FLAG_FRAME_STATE_SOLVED)
                    && (iFrm != iFrm1 || metric && intrinsicType == Sequence::INTRINSIC_VARIABLE))
                m_candidateFrms.push_back(CandidateFrame(iFrm, seq.CountFrameMarkedTracks(iFrm, m_trkMarks)));
        }
        std::sort(m_candidateFrms.begin(), m_candidateFrms.end());
        for(FrameIndex i = 0; i < m_baLocalMaxNumAdditionalAdjustedFrms; ++i)
            m_frmMarks[m_candidateFrms[i].GetFrameIndex()] = true;
        iFrmsAdj.resize(0);
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            if(m_frmMarks[iFrm])
                iFrmsAdj.push_back(iFrm);
        }
    }
    SelectAdjustedTracks(seq, iFrmsAdj, iTrksAdj, m_iTrksUnadj, m_baLocalMinNumTrksPerFrm, metric);
    /*const double SSE1 = */BundleAdjust(iFrmsAdj, iTrksAdj, iFrmsBA, seq, m_baLocalMaxNumIters, m_baLocalStopMSE, m_baLocalStopRelativeReduction, metric);
    TransformScene(iFrm1, iFrm2, seq);
    /*const double SSE2 = */RunPointsOptimization(m_iTrksUnadj, seq, metric);
    iTrksAdj.insert(iTrksAdj.end(), m_iTrksUnadj.begin(), m_iTrksUnadj.end());
    //return SSE1 + SSE2;
}

void CameraTracker::RunBundleAdjustmentGlobal(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FrameIndexList &iFrmsAdj, FrameIndexList &iFrmsBA,
        TrackIndexList &iTrksAdj, Sequence &seq, const bool metric) {
#if VERBOSE_CAMERA_TRACKING
    printf("----------------------------------------------------------------\n");
    printf("Global bundle adjustment...\n");
#endif
    iFrmsAdj.resize(0);
    const Sequence::IntrinsicType intrinsicType = seq.GetIntrinsicType();
    const FrameIndex nFrms = seq.GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        if((seq.GetFrameState(iFrm) & FLAG_FRAME_STATE_SOLVED) && (iFrm != iFrm1 || metric && intrinsicType == Sequence::INTRINSIC_VARIABLE))
            iFrmsAdj.push_back(iFrm);
    }

    SelectAdjustedTracks(seq, iFrmsAdj, iTrksAdj, m_iTrksUnadj, m_baGlobalMinNumTrksPerFrm, metric);
    /*const double SSE1 = */BundleAdjust(iFrmsAdj, iTrksAdj, iFrmsBA, seq, m_baGlobalMaxNumIters, m_baGlobalStopMSE, m_baGlobalStopRelativeReduction, metric);
    TransformScene(iFrm1, iFrm2, seq);
    /*const double SSE2 = */RunPointsOptimization(m_iTrksUnadj, seq, metric);
    iTrksAdj.insert(iTrksAdj.end(), m_iTrksUnadj.begin(), m_iTrksUnadj.end());
    //return SSE1 + SSE2;
}

void CameraTracker::BundleAdjust(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, FrameIndexList &iFrmsBA, Sequence &seq,
                                 const uint &maxNumIters, const float &stopMSE, const float &stopRelativeReduction, const bool metric) {
    if(!metric) {
        seq.GetBundleAdjustorData(iFrmsAdj, iTrksAdj, m_baDataProjective, iFrmsBA);
        m_baProjective.m_lmMaxNumIters = maxNumIters;
        m_baProjective.m_lmStopMSE = stopMSE;
        m_baProjective.m_lmStopRelativeReduction = stopRelativeReduction;
//#if _DEBUG
//      if(iFrmsBA.size() == 7)
//      printf("%f\n", m_baDataProjective.ComputeSSE() * seq.GetIntrinsicMatrix().fxy() / m_baDataProjective.GetMeasurementsNumber());
//#endif
        const FrameIndex nFrmsFix = FrameIndex(iFrmsBA.size() - iFrmsAdj.size());
        m_baProjective.Run(m_baDataProjective, nFrmsFix, false, VERBOSE_CAMERA_TRACKING ? 2 : 0);
        seq.SetBunbleAdjustmentResults(m_baDataProjective, nFrmsFix, iFrmsBA, iTrksAdj);
        //return m_baDataProjective.ComputeSSE();
    } else if(seq.GetIntrinsicType() == Sequence::INTRINSIC_VARIABLE) {
        seq.GetBundleAdjustorData(iFrmsAdj, iTrksAdj, m_baDataMetricIntrinsicVariable, iFrmsBA);
        m_baMetricIntrinsicVariable.m_lmMaxNumIters = maxNumIters;
        m_baMetricIntrinsicVariable.m_lmStopMSE = stopMSE;
        m_baMetricIntrinsicVariable.m_lmStopRelativeReduction = stopRelativeReduction;
//#if _DEBUG
//      printf("%f\n", m_baDataMetricIntrinsicVariable.ComputeSSE() * seq.GetIntrinsicMatrix().fxy() / m_baDataMetricIntrinsicVariable.GetMeasurementsNumber());
//#endif
        const FrameIndex nFrmsFix = FrameIndex(iFrmsBA.size() - iFrmsAdj.size());
        m_baMetricIntrinsicVariable.Run(m_baDataMetricIntrinsicVariable, nFrmsFix, false, VERBOSE_CAMERA_TRACKING ? 2 : 0);
        seq.SetBunbleAdjustmentResults(m_baDataMetricIntrinsicVariable, nFrmsFix, iFrmsBA, iTrksAdj);
        //return m_baDataMetricIntrinsicVariable.ComputeSSE();
    } else {
        seq.GetBundleAdjustorData(iFrmsAdj, iTrksAdj, m_baDataMetric, iFrmsBA);
        m_baMetric.m_lmMaxNumIters = maxNumIters;
        m_baMetric.m_lmStopMSE = stopMSE;
        m_baMetric.m_lmStopRelativeReduction = stopRelativeReduction;
//#if _DEBUG
//      if(seq.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT)
//          m_baDataMetric.ValidateGlobal();
//      printf("%f\n", m_baDataMetric.ComputeSSE() * seq.GetIntrinsicMatrix().fxy() / m_baDataMetric.GetMeasurementsNumber());
//#endif
        const FrameIndex nFrmsFix = FrameIndex(iFrmsBA.size() - iFrmsAdj.size());
        m_baMetric.Run(m_baDataMetric, nFrmsFix, seq.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT, VERBOSE_CAMERA_TRACKING ? 2 : 0);
        seq.SetBunbleAdjustmentResults(m_baDataMetric, nFrmsFix, iFrmsBA, iTrksAdj);
        //return m_baDataMetric.ComputeSSE();
    }
}

void CameraTracker::SelectAdjustedTracks(const Sequence &seq, const FrameIndexList &iFrmsAdj, TrackIndexList &iTrksAdj, TrackIndexList &iTrksUnadj,
        const FeatureIndex minNumTrksPerFrm, const bool metric) {
    FrameIndex i, j, iFrm;
    TrackIndex k, iTrk;
    MeasurementIndex iMea;
    FeatureIndex iFtr;
    //FrameIndex nCrspsSolved, nCrspsInlier;

    const FrameIndex nFrms = seq.GetFramesNumber();
    m_frmMarks.assign(nFrms, false);
    m_trkMarks.assign(seq.GetTracksNumber(), false);
    m_candidateTrks.resize(0);

    const FrameIndex nFrmsAdj = FrameIndex(iFrmsAdj.size());
    for(i = 0; i < nFrmsAdj; ++i) {
        iFrm = iFrmsAdj[i];
        m_frmMarks[iFrm] = true;

        const TrackIndex *iTrks = seq.GetFrameTrackIndexes(iFrm);
        const MeasurementState *meaStates = seq.GetFrameMeasurementStates(iFrm);
        const FeatureIndex nFtrs = seq.GetFrameFeaturesNumber(iFrm);
        for(iFtr = 0; iFtr < nFtrs; ++iFtr) {
            if((iTrk = iTrks[iFtr]) == INVALID_TRACK_INDEX || m_trkMarks[iTrk] || !(seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER)
                    || (meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
                continue;
            m_trkMarks[iTrk] = true;
            //seq.CountTrackSolvedFrameInlierMeasurements(iTrk, nCrspsSolved, nCrspsInlier);
            //m_candidateTrks.push_back(CandidateTrack(iTrk, nCrspsSolved, nCrspsInlier));
            if(metric)
                m_candidateTrks.push_back(CandidateTrack(iTrk, seq.ComputePointMinimalRayAngleDot(iTrk, m_rayDirs, true)));
            else
                m_candidateTrks.push_back(CandidateTrack(iTrk, -float(seq.CountTrackInlierMeasurements(iTrk))));
        }
    }
    std::sort(m_candidateTrks.begin(), m_candidateTrks.end());

    FrameIndex cntAdjFrmsEnoughTrks = 0;
    m_frmTrkCnts.assign(nFrms, 0);
    iTrksAdj.resize(0);
    iTrksUnadj.resize(0);

    const TrackIndex nTrksCandidate = TrackIndex(m_candidateTrks.size());
    for(k = 0; k < nTrksCandidate && cntAdjFrmsEnoughTrks < nFrmsAdj; ++k) {
        iTrk = m_candidateTrks[k].GetTrackIndex();
        const MeasurementIndexList &iMeas = seq.GetTrackMeasurementIndexList(iTrk);
        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        for(j = 0; j < nCrsps; ++j) {
            iMea = iMeas[j];
            iFrm = seq.GetMeasurementFrameIndex(iMea);
            if(m_frmMarks[iFrm] && !(seq.GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER) && m_frmTrkCnts[iFrm] < minNumTrksPerFrm)
                break;
        }
        if(j == nCrsps) {
            iTrksUnadj.push_back(iTrk);
            continue;
        }
        iTrksAdj.push_back(iTrk);
        for(j = 0; j < nCrsps; ++j) {
            iMea = iMeas[j];
            iFrm = seq.GetMeasurementFrameIndex(iMea);
            if(m_frmMarks[iFrm] && !(seq.GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER) && ++m_frmTrkCnts[iFrm] == minNumTrksPerFrm)
                ++cntAdjFrmsEnoughTrks;
        }
    }
    const float scoreMin = m_candidateTrks.empty() ? 1.0f : m_candidateTrks.front().GetRayAngleDot();
    const float scoreMax = m_candidateTrks.empty() ? 1.0f : k == nTrksCandidate ? std::min(m_candidateTrks.back().GetRayAngleDot(), 1.0f) :
                           m_candidateTrks[k].GetRayAngleDot();
    for(; k < nTrksCandidate; ++k)
        iTrksUnadj.push_back(m_candidateTrks[k].GetTrackIndex());

#if VERBOSE_CAMERA_TRACKING
    printf("  Selected %d / %d adjusted tracks\n", iTrksAdj.size(), iTrksAdj.size() + iTrksUnadj.size());
    if(metric)
        printf("  Ray angle: [%f, %f]\n", acos(scoreMax) * FACTOR_RAD_TO_DEG, acos(scoreMin) * FACTOR_RAD_TO_DEG);
    else
        printf("  Track length: [%d, %d]\n", FrameIndex(-scoreMin), FrameIndex(-scoreMin));
#endif
}

void CameraTracker::RunPointsOptimization(const TrackIndexList &iTrks, Sequence &seq, const bool metric) {
    //m_Xestor.m_ransacErrorThreshold = 0.0f;
    if(metric)
        m_Xestor.SetMetric();
    else
        m_Xestor.SetProjective();
    //double SSE = 0;
    const TrackIndex nTrks = TrackIndex(iTrks.size());
    for(TrackIndex i = 0; i < nTrks; ++i) {
        const TrackIndex iTrk = iTrks[i];
        seq.GetPoint3DEstimatorDataInlier(iTrk, m_Xdata, metric);
        m_X = seq.GetPoint(iTrk);
        m_Xestor.OptimizeModel(m_Xdata, m_X);
        //m_Xestor.Triangulate(m_Xdata, m_X);
        seq.SetPoint(iTrk, m_X);
        //SSE += m_Xdata.ComputeSSE(m_X);
    }
    //return SSE;
}

void CameraTracker::BundleAdjustAdaptiveErrorLevelLocal(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndexList &iFrmsSeed, Sequence &seq,
        std::vector<FrameErrorLevel> &frmErrLevels, const bool metric, const bool triangulateUnstablePts) {
    RunBundleAdjustmentLocal(iFrm1, iFrm2, iFrmsSeed, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
    //ViewSequence(seq, iFrmsSeed.back());
    seq.MarkFramesTracks(m_iFrmsAdj, m_trkMarks);
    UpdateStructureAndInlierStates(m_trkMarks, seq, frmErrLevels, metric, triangulateUnstablePts);
    ViewSequence(seq, iFrmsSeed.back());

    for(uint iter = 1; iter < m_baaelLocalMaxNumIters; iter += 2) {
        DecreaseOutlierLevels_UpdateReprojectionErrorLevels(seq, m_iFrmsAdj, m_iFrmsUpd, m_iFrmsErr, frmErrLevels, metric);
        seq.MarkFramesTracks(m_iFrmsUpd, m_trkMarks);
        if(!UpdateStructureAndInlierStates(m_trkMarks, seq, frmErrLevels, metric, triangulateUnstablePts))
            break;
        //ViewSequence(seq, iFrmsSeed.back());
        if(m_iFrmsErr.empty())
            RunBundleAdjustmentLocal(iFrm1, iFrm2, iFrmsSeed, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
        else
            RunBundleAdjustmentLocal(iFrm1, iFrm2, m_iFrmsErr, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
        ViewSequence(seq, iFrmsSeed.back());

        DecreaseReprojectionErrorLevels_UpdateOutlierLevels(seq, m_iFrmsAdj, m_iFrmsUpd, m_iFrmsErr, frmErrLevels, metric);
        seq.MarkFramesTracks(m_iFrmsAdj, m_trkMarks);
        if(!UpdateStructureAndInlierStates(m_trkMarks, seq, frmErrLevels, metric, triangulateUnstablePts))
            break;
        //ViewSequence(seq, iFrmsSeed.back());
        if(m_iFrmsErr.empty())
            RunBundleAdjustmentLocal(iFrm1, iFrm2, iFrmsSeed, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
        else
            RunBundleAdjustmentLocal(iFrm1, iFrm2, m_iFrmsErr, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
        //ViewSequence(seq, iFrmsSeed.back());
        seq.MarkFramesTracks(m_iFrmsAdj, m_trkMarks);
        if(!UpdateStructureAndInlierStates(m_trkMarks, seq, frmErrLevels, metric, triangulateUnstablePts))
            break;
        ViewSequence(seq, iFrmsSeed.back());
    }
}

void CameraTracker::BundleAdjustAdaptiveErrorLevelGlobal(const FrameIndex &iFrm1, const FrameIndex &iFrm2, Sequence &seq,
        std::vector<FrameErrorLevel> &frmErrLevels, const bool metric,
        const bool triangulateUnstablePts) {
    RunBundleAdjustmentGlobal(iFrm1, iFrm2, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
    //ViewSequence(seq, iFrmFix);
    seq.MarkFramesTracks(m_iFrmsAdj, m_trkMarks);
    UpdateStructureAndInlierStates(m_trkMarks, seq, frmErrLevels, metric, triangulateUnstablePts);
    ViewSequence(seq, iFrm1);

    for(uint iter = 1; iter < m_baaelGlobalMaxNumIters; iter += 2) {
        seq.GetFrameIndexList(FLAG_FRAME_STATE_SOLVED, m_iFrmsBA);
        DecreaseOutlierLevels_UpdateReprojectionErrorLevels(seq, m_iFrmsBA, m_iFrmsUpd, m_iFrmsErr, frmErrLevels, metric);
        seq.MarkFramesTracks(m_iFrmsUpd, m_trkMarks);
        if(!UpdateStructureAndInlierStates(m_trkMarks, seq, frmErrLevels, metric, triangulateUnstablePts))
            break;
        //ViewSequence(seq, iFrmFix);
        if(m_iFrmsErr.empty())
            RunBundleAdjustmentGlobal(iFrm1, iFrm2, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
        else
            RunBundleAdjustmentLocal(iFrm1, iFrm2, m_iFrmsErr, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
        ViewSequence(seq, iFrm1);

        DecreaseReprojectionErrorLevels_UpdateOutlierLevels(seq, m_iFrmsAdj, m_iFrmsUpd, m_iFrmsErr, frmErrLevels, metric);
        seq.MarkFramesTracks(m_iFrmsAdj, m_trkMarks);
        if(!UpdateStructureAndInlierStates(m_trkMarks, seq, frmErrLevels, metric, triangulateUnstablePts))
            break;
        //ViewSequence(seq, iFrmFix);
        if(m_iFrmsErr.empty() || iter + 2 >= m_baaelGlobalMaxNumIters)
            RunBundleAdjustmentGlobal(iFrm1, iFrm2, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
        else
            RunBundleAdjustmentLocal(iFrm1, iFrm2, m_iFrmsErr, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
        //ViewSequence(seq, iFrmFix);
        seq.MarkFramesTracks(m_iFrmsAdj, m_trkMarks);
        if(!UpdateStructureAndInlierStates(m_trkMarks, seq, frmErrLevels, metric, triangulateUnstablePts))
            break;
        ViewSequence(seq, iFrm1);
    }

    //if(m_baaelGlobalMaxNumIters >= 2)
    //{
    //  BundleAdjustGlobal(iFrm1, iFrm2, m_iFrmsAdj, m_iFrmsBA, m_iTrksAdj, seq, metric);
    //  ViewSequence(seq, iFrmFix);
    //}
}

bool CameraTracker::UpdateStructureAndInlierStates(const std::vector<bool> &trkMarks, Sequence &seq, const std::vector<FrameErrorLevel> &frmErrLevels,
        const bool metric, const bool triangulateUnstablePts) {
    TrackIndex cntTrkIn = 0, cntTrkOut = 0, cntTrkInToOut = 0, cntTrkOutToIn = 0, cntTrkUpd = 0;
    MeasurementIndex cntMeaInToOut = 0, cntMeaOutToIn = 0;
    FrameIndex iFrm;
    TrackIndex iTrk;
    MeasurementIndex iMea;
    ushort i, nInliers;
    float errSqTh;
    double fitErr;
    bool update;

    m_Xestor.m_ransacErrorThreshold = 1.0f;
    if(metric)
        m_Xestor.SetMetric();
    else
        m_Xestor.SetProjective();

    const Sequence::IntrinsicType intrinsicType = seq.GetIntrinsicType();
    const float f2 = seq.GetIntrinsicRectification().f() * seq.GetIntrinsicRectification().f();
    const TrackIndex nTrks = TrackIndex(seq.GetTracksNumber());
    for(iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(!trkMarks[iTrk])
            continue;
        seq.GetPoint3DEstimatorData(iTrk, m_Xdata, m_iMeas, metric);
        const ushort N = ushort(m_iMeas.size());
        if(N < m_sfmPtInliersMinNum)
            continue;
        const ushort nInliersTh = std::max(m_sfmPtInliersMinNum, ushort(N * m_sfmPtInliersMinRatio + 0.5f));
        m_Xdata.ValidateWeights();
        for(i = 0; i < N; ++i) {
            iFrm = seq.GetMeasurementFrameIndex(m_iMeas[i]);
            errSqTh = m_sfmReprojErrSqThs[frmErrLevels[iFrm].GetReprojectionErrorLevel()];
            if(!metric || intrinsicType == Sequence::INTRINSIC_USER_FIXED)
                m_Xdata.SetSquaredWeight(i, 1 / errSqTh);
            else if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
                m_Xdata.SetSquaredWeight(i, f2 / errSqTh);
            else
                m_Xdata.SetSquaredWeight(i, m_Xdata.GetSquaredWeight(i) / errSqTh);
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
            m_Xestor.FromInliersToInlierMarks(m_inliers, N, m_inlierMarks);
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
    printf("INLIER --> OUTLIER: %d / %d tracks, %d measurements\n", cntTrkInToOut, cntTrkIn, cntMeaInToOut);
    printf("INLIER <-- OUTLIER: %d / %d tracks, %d measurements\n", cntTrkOutToIn, cntTrkOut, cntMeaOutToIn);
#endif

//#if _DEBUG
//  m_stop = true;
//  ViewSequence(seq, 19);
//#endif

    return cntTrkUpd != 0;
}

float CameraTracker::ComputeMinimalRayAngleDot(const Point3DEstimatorData &data) {
    const ushort N = data.Size();
    m_rayDirs.Resize(N);
    for(ushort i = 0; i < N; ++i)
        data.GetCamera(i).ComputeRayDirection(data.x(i), m_rayDirs[i]);

    float dot, dotMin = FLT_MAX;
    m_candidateTriPairs.resize(0);
    for(ushort i1 = 0; i1 < N; ++i1)
        for(ushort i2 = i1 + 1; i2 < N; ++i2) {
            if((dot = m_rayDirs[i1].Dot(m_rayDirs[i2])) < dotMin)
                dotMin = dot;
        }

    return dotMin;
}

float CameraTracker::ComputeMinimalRayAngleDot_RankTriangulationPairs(const Point3DEstimatorData &data, std::vector<Match<ushort> > &triPairs) {
    const ushort N = data.Size();
    m_rayDirs.Resize(N);
    for(ushort i = 0; i < N; ++i)
        data.GetCamera(i).ComputeRayDirection(data.x(i), m_rayDirs[i]);

    float dot;
    m_candidateTriPairs.resize(0);
    for(ushort i1 = 0; i1 < N; ++i1)
        for(ushort i2 = i1 + 1; i2 < N; ++i2) {
            dot = m_rayDirs[i1].Dot(m_rayDirs[i2]);
            m_candidateTriPairs.push_back(CandidateTriangulationPair(i1, i2, dot));
        }
    std::sort(m_candidateTriPairs.begin(), m_candidateTriPairs.end());

    //const uint nOutliersTh = N - max(m_sfmPtInliersMinNum, uint(N * m_sfmPtInliersMinRatio + 0.5f)), nPairs = ((nOutliersTh * (nOutliersTh - 1)) >> 1) + 1;
    const uint nPairs = uint(m_candidateTriPairs.size());
    triPairs.resize(nPairs);
    for(uint i = 0; i < nPairs; ++i)
        triPairs[i] = m_candidateTriPairs[i];

    return m_candidateTriPairs.front().GetRayAngleDot();
}

//bool CameraTracker::CheckMaximalRayAngleSmall_RankTriangulationPairs(const Point3DEstimatorData &data, const std::vector<bool> &chkMarks,
//                                                                   std::vector<Match<ushort> > &triPairs)
//{
//  const ushort N = data.Size();
//  m_rayDirs.Resize(N);
//  for(ushort i = 0; i < N; ++i)
//      data.GetCamera(i).ComputeRayDirection(data.x(i), m_rayDirs[i]);
//
//  float dot;
//  m_candidateTriPairs.resize(0);
//  for(ushort i1 = 0; i1 < N; ++i1)
//  for(ushort i2 = i1 + 1; i2 < N; ++i2)
//  {
//      dot = m_rayDirs[i1].Dot(m_rayDirs[i2]);
//      if(chkMarks[i1] && chkMarks[i2] && dot < m_sfmMultiViewRayAngleDotTh)       // Large enough
//          return false;
//      m_candidateTriPairs.push_back(CandidateTriangulationPair(i1, i2, dot));
//  }
//  std::sort(m_candidateTriPairs.begin(), m_candidateTriPairs.end());
//
//  const uint nOutliersTh = N - max(m_sfmPtInliersMinNum, uint(N * m_sfmPtInliersMinRatio + 0.5f)), nPairs = ((nOutliersTh * (nOutliersTh - 1)) >> 1) + 1;
//  triPairs.resize(nPairs);
//  for(uint i = 0; i < nPairs; ++i)
//      triPairs[i] = m_candidateTriPairs[i];
//  return true;
//}

void CameraTracker::DecreaseOutlierLevels_UpdateReprojectionErrorLevels(const Sequence &seq, const FrameIndexList &iFrms, FrameIndexList &iFrmsUpd,
        FrameIndexList &iFrmsErr, std::vector<FrameErrorLevel> &frmErrLevels, const bool metric) {
#if VERBOSE_CAMERA_TRACKING
    printf("----------------------------------------------------------------\n");
    printf("Decreasing outlier levels & updating reprojection error levels...\n");
#endif
    FrameIndex iFrm;
    TrackIndex iTrk;
    FeatureIndex iFtr;
    FeatureIndex nInliers, nInliersTh;
    Point2D e, mean;
    LA::Vector3f cov;
    float areaFtrTrked, areaFtrInlier, areaFtrInlierTh, errSqTh;

    iFrmsUpd.resize(0);
    iFrmsErr.resize(0);

//#if _DEBUG
//  if(seq.CountFrames(FLAG_FRAME_STATE_SOLVED) == 7)
//      printf("...\n");
//#endif

    const Point2D *xs;
    const Sequence::IntrinsicType intrinsicType = seq.GetIntrinsicType();
    const FrameIndex nFrms = FrameIndex(iFrms.size());
    for(FrameIndex i = 0; i < nFrms; ++i) {
        iFrm = iFrms[i];
        const Camera &C = seq.GetCamera(iFrm);
        const TrackIndex *iTrks = seq.GetFrameTrackIndexes(iFrm);
        //const Point2D *xs = seq.GetFrameFeatures(iFrm);
        const FeatureIndex nFtrs = seq.GetFrameFeaturesNumber(iFrm);
        if(!metric || intrinsicType == Sequence::INTRINSIC_USER_FIXED)
            xs = seq.GetFrameFeatures(iFrm);
        else {
            m_xrs.Resize(nFtrs);
            m_xrs.CopyFrom(seq.GetFrameFeatures(iFrm), nFtrs);
            if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
                Sequence::RectifyMeasurements(seq.GetIntrinsicRectification(), m_xrs);
            else
                Sequence::RectifyMeasurements(seq.GetIntrinsicRectification(iFrm), m_xrs);
            xs = m_xrs.Data();
        }
        m_errSqs.resize(nFtrs);
        m_iFtrs.resize(0);
        for(iFtr = 0; iFtr < nFtrs; ++iFtr) {
            if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && seq.CountTrackSolvedFrameMeasurements(iTrk) >= m_sfmPtInliersMinNum)
                //if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && (seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER))
                m_iFtrs.push_back(iFtr);
            if(iTrk == INVALID_TRACK_INDEX || !(seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER))
                m_errSqs[iFtr] = FLT_MAX;
            else if(!metric)
                m_errSqs[iFtr] = C.ComputeProjectionSquaredError(seq.GetPoint(iTrk), xs[iFtr], e);
            else if(C.ComputeProjectionError_CheckCheirality(seq.GetPoint(iTrk), xs[iFtr], e))
                m_errSqs[iFtr] = e.SquaredLength();
            else
                m_errSqs[iFtr] = FLT_MAX;
        }
        const FeatureIndex nFtrsTrked = FeatureIndex(m_iFtrs.size());
        seq.ComputeFrameFeaturesDistribution(iFrm, m_iFtrs, mean, cov);
        areaFtrTrked = sqrt(cov.v0() * cov.v2() - cov.v1() * cov.v1());

        FrameErrorLevel &frmErrLevel = frmErrLevels[iFrm];
        const FrameErrorLevel frmErrLevelBkp = frmErrLevel;
        frmErrLevel.DecreaseOutlierErrorLevel();
        frmErrLevel.SetReprojectionErrorLevelLowest();
        nInliersTh = std::max(m_sfmCamInliersMinNum, FeatureIndex(nFtrsTrked * m_sfmCamInlierRatioThs[frmErrLevel.GetOutlierLevel()] + 0.5f));
        areaFtrInlierTh = areaFtrTrked * m_sfmCamInlierRatioThs[frmErrLevel.GetOutlierLevel()];
        while(1) {
            errSqTh = m_sfmReprojErrSqThs[frmErrLevel.GetReprojectionErrorLevel()];
            if(metric) {
                if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
                    errSqTh /= seq.GetIntrinsicRectification().f() * seq.GetIntrinsicRectification().f();
                else if(intrinsicType == Sequence::INTRINSIC_VARIABLE)
                    errSqTh /= seq.GetIntrinsicRectification(iFrm).f() * seq.GetIntrinsicRectification(iFrm).f();
            }
            m_iFtrs.resize(0);
            for(iFtr = 0; iFtr < nFtrs; ++iFtr) {
                if(m_errSqs[iFtr] < errSqTh)
                    m_iFtrs.push_back(iFtr);
            }
            //if((nInliers = FeatureIndex(m_iFtrs.size())) >= nInliersTh || !frmErrLevel.IncreaseReprojectionErrorLevel())
            //  break;
            seq.ComputeFrameFeaturesDistribution(iFrm, m_iFtrs, mean, cov);
            //if((areaFtrInlier = sqrt(cov.v0() * cov.v2() - cov.v1() * cov.v1())) >= areaFtrInlierTh && ushort(m_iFtrs.size()) >= m_sfmCamInliersMinNum
            //|| !frmErrLevel.IncreaseReprojectionErrorLevel())
            //  break;
            areaFtrInlier = sqrt(cov.v0() * cov.v2() - cov.v1() * cov.v1());
            nInliers = ushort(m_iFtrs.size());
            if(areaFtrInlier >= areaFtrInlierTh && nInliers >= nInliersTh || !frmErrLevel.IncreaseReprojectionErrorLevel())
                break;
        }
        //if(nInliers < nInliersTh)
        //if(areaFtrInlier < areaFtrInlierTh)
        //if(areaFtrInlier < areaFtrInlierTh || nInliers < nInliersTh)
        //  frmErrLevel = frmErrLevelBkp;
        if(frmErrLevel != frmErrLevelBkp)
            iFrmsUpd.push_back(iFrm);
        if(!frmErrLevel.IsLowest())
            iFrmsErr.push_back(iFrm);
#if VERBOSE_CAMERA_TRACKING
        if(/*frmErrLevel != frmErrLevelBkp || */!frmErrLevel.IsLowest()) {
            printf("  Frame %d: inliers = %d/%d = %.2f%%, error threshold = %.2f\n", iFrm, nInliers, nFtrsTrked, nInliers * 100.0f / nFtrsTrked,
                   sqrt(m_sfmReprojErrSqThs[frmErrLevel.GetReprojectionErrorLevel()] * seq.GetIntrinsicMatrix().fxy()));
            //printf("  Frame %d: inliers = %d/%d = %.2f%%, error threshold = %.2f\n", iFrm, m_iFtrs.size(), nFtrsTrked, areaFtrInlier * 100.0f / areaFtrTrked,
            //  sqrt(seq, iFrm, m_sfmReprojErrSqThs[frmErrLevel.GetReprojectionErrorLevel()] * seq.GetIntrinsicMatrix().fxy()));
        }
#endif
    }
#if VERBOSE_CAMERA_TRACKING
    printf("Updated %d / %d frames, %d / %d error frame\n", iFrmsUpd.size(), nFrms, iFrmsErr.size(), nFrms);
#endif
}

void CameraTracker::DecreaseReprojectionErrorLevels_UpdateOutlierLevels(const Sequence &seq, const FrameIndexList &iFrms, FrameIndexList &iFrmsUpd,
        FrameIndexList &iFrmsErr, std::vector<FrameErrorLevel> &frmErrLevels, const bool metric) {
#if VERBOSE_CAMERA_TRACKING
    printf("----------------------------------------------------------------\n");
    printf("Decreasing reprojection error levels & updating outlier levels...\n");
#endif
    FrameIndex iFrm;
    TrackIndex iTrk;
    FeatureIndex iFtr;
    FeatureIndex nInliers, nInliersTh;
    Point2D e, meanFtrTrked, meanFtrInlier;
    LA::Vector3f covFtrTrked, covFtrInlier;
    float inlierAreaRatio, errSqTh;

    iFrmsUpd.resize(0);
    iFrmsErr.resize(0);

    const Point2D *xs;
    const Sequence::IntrinsicType intrinsicType = seq.GetIntrinsicType();
    const FrameIndex nFrms = FrameIndex(iFrms.size());
    for(FrameIndex i = 0; i < nFrms; ++i) {
        iFrm = iFrms[i];

        const Camera &C = seq.GetCamera(iFrm);
        const TrackIndex *iTrks = seq.GetFrameTrackIndexes(iFrm);
        //const Point2D *xs = seq.GetFrameFeatures(iFrm);
        const FeatureIndex nFtrs = seq.GetFrameFeaturesNumber(iFrm);
        if(!metric || intrinsicType == Sequence::INTRINSIC_USER_FIXED)
            xs = seq.GetFrameFeatures(iFrm);
        else {
            m_xrs.Resize(nFtrs);
            m_xrs.CopyFrom(seq.GetFrameFeatures(iFrm), nFtrs);
            if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
                Sequence::RectifyMeasurements(seq.GetIntrinsicRectification(), m_xrs);
            else
                Sequence::RectifyMeasurements(seq.GetIntrinsicRectification(iFrm), m_xrs);
            xs = m_xrs.Data();
        }
        m_iFtrs.resize(0);
        m_errSqs.resize(nFtrs);
        for(iFtr = 0; iFtr < nFtrs; ++iFtr) {
            if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && seq.CountTrackSolvedFrameMeasurements(iTrk) >= m_sfmPtInliersMinNum)
                //if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && (seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER))
                m_iFtrs.push_back(iFtr);
            if(iTrk == INVALID_TRACK_INDEX || !(seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER))
                m_errSqs[iFtr] = FLT_MAX;
            else if(!metric)
                m_errSqs[iFtr] = C.ComputeProjectionSquaredError(seq.GetPoint(iTrk), xs[iFtr], e);
            else if(C.ComputeProjectionError_CheckCheirality(seq.GetPoint(iTrk), xs[iFtr], e))
                m_errSqs[iFtr] = e.SquaredLength();
            else
                m_errSqs[iFtr] = FLT_MAX;
        }
        const FeatureIndex nFtrsTrked = FeatureIndex(m_iFtrs.size());
        seq.ComputeFrameFeaturesDistribution(iFrm, m_iFtrs, meanFtrTrked, covFtrTrked);

        FrameErrorLevel &frmErrLevel = frmErrLevels[iFrm];
        const FrameErrorLevel frmErrLevelBkp = frmErrLevel;
        frmErrLevel.SetReprojectionErrorLevelLowest();
        //frmErrLevel.DecreaseReprojectionErrorLevel();
        errSqTh = m_sfmReprojErrSqThs[frmErrLevel.GetReprojectionErrorLevel()];
        if(metric) {
            if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
                errSqTh /= seq.GetIntrinsicRectification().f() * seq.GetIntrinsicRectification().f();
            else if(intrinsicType == Sequence::INTRINSIC_VARIABLE)
                errSqTh /= seq.GetIntrinsicRectification(iFrm).f() * seq.GetIntrinsicRectification(iFrm).f();
        }
        m_iFtrs.resize(0);
        for(iFtr = 0; iFtr < nFtrs; ++iFtr) {
            if(m_errSqs[iFtr] < errSqTh)
                m_iFtrs.push_back(iFtr);
        }
        nInliers = FeatureIndex(m_iFtrs.size());
        seq.ComputeFrameFeaturesDistribution(iFrm, m_iFtrs, meanFtrInlier, covFtrInlier);
        inlierAreaRatio = sqrt((covFtrInlier.v0() * covFtrInlier.v2() - covFtrInlier.v1() * covFtrInlier.v1())
                               / (covFtrTrked.v0() * covFtrTrked.v2() - covFtrTrked.v1() * covFtrTrked.v1()));

        frmErrLevel.SetOutlierLevelLowest();
        while(1) {
            nInliersTh = std::max(m_sfmCamInliersMinNum, FeatureIndex(nFtrsTrked * m_sfmCamInlierRatioThs[frmErrLevel.GetOutlierLevel()] + 0.5f));
            //if(nInliers >= nInliersTh || !frmErrLevel.IncreaseOutlierLevel())
            //  break;
            //if(inlierAreaRatio >= m_sfmCamInlierAreaRatioThs[frmErrLevel.GetOutlierLevel()] || !frmErrLevel.IncreaseOutlierLevel())
            //  break;
            if(inlierAreaRatio >= m_sfmCamInlierRatioThs[frmErrLevel.GetOutlierLevel()] && nInliers >= nInliersTh || !frmErrLevel.IncreaseOutlierLevel())
                break;
        }
        if(frmErrLevel != frmErrLevelBkp)
            iFrmsUpd.push_back(iFrm);
        if(!frmErrLevel.IsLowest())
            iFrmsErr.push_back(iFrm);
#if VERBOSE_CAMERA_TRACKING
        if(/*frmErrLevel != frmErrLevelBkp || */!frmErrLevel.IsLowest()) {
            printf("  Frame %d: inliers = %d/%d = %.2f%%, error threshold = %.2f\n", iFrm, nInliers, nFtrsTrked, nInliers * 100.0f / nFtrsTrked,
                   sqrt(m_sfmReprojErrSqThs[frmErrLevel.GetReprojectionErrorLevel()] * seq.GetIntrinsicMatrix().fxy()));
            //printf("  Frame %d: inliers = %d/%d = %.2f%%, error threshold = %.2f\n", iFrm, m_iFtrs.size(), nFtrsTrked, inlierAreaRatio * 100,
            //  sqrt(m_sfmReprojErrSqThs[frmErrLevel.GetReprojectionErrorLevel()] * seq.GetIntrinsicMatrix().fxy()));
        }
#endif
    }
#if VERBOSE_CAMERA_TRACKING
    printf("Updated %d / %d frames, %d / %d error frames\n", iFrmsUpd.size(), nFrms, iFrmsErr.size(), nFrms);
#endif
}