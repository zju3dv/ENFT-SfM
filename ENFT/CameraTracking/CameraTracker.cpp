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
#include "Utility/Utility.h"
#include "Utility/Timer.h"

using namespace ENFT_SfM;

ubyte CameraTracker::FrameErrorLevel::g_outlierLevelHighest = 0;
ubyte CameraTracker::FrameErrorLevel::g_reprojErrLevelHighest = 0;

void CameraTracker::Initialize(const Sequence &seq,
                               const std::string paramFileName, const bool distortionInvalid) {
    Configurator param;
    param.Load(paramFileName.c_str());
    Initialize(seq, param, distortionInvalid);
}

void CameraTracker::Initialize(const Sequence &seq, const Configurator &param,
                               const bool distortionInvalid) {
    //ProgramGL::Initialize(seq.GetImageWidth(), seq.GetImageHeight());
    //ViewerSequence::Initialize(seq);
    //m_imgView = false;

    m_kfMinNumTrksPerKeyFrm = FeatureIndex(
                                  param.GetArgument("kf_min_tracks_number_per_key_frame", 300));
    m_kfMinNumCmnTrksTwoKeyFrms = FeatureIndex(
                                      param.GetArgument("kf_min_common_tracks_number_two_key_frames", 200));
    m_kfMinNumCmnTrksThreeKeyFrms = FeatureIndex(
                                        param.GetArgument("kf_min_common_tracks_number_three_key_frames", 100));
    m_kfMinTrkLenKeyFrm = FrameIndex(
                              param.GetArgument("kf_min_track_length_key_frame", 2));
    m_kfNumBinsX = ubyte(param.GetArgument("kf_bins_number_x", 8));
    m_kfNumBinsY = ubyte(param.GetArgument("kf_bins_number_y", 8));
    m_kfMinNumFtrsPerBin = FeatureIndex(
                               param.GetArgument("kf_min_features_number_per_bin", 2));

    m_sfmInitScoreFilterSigma = FrameIndex(
                                    param.GetArgument("sfm_initial_score_filter_sigma", 3));
    m_sfmInitHomoRatio = param.GetArgument("sfm_initial_homography_ratio", 0.5f);

    m_sfmTwoViewInliersMinNum = ushort(
                                    param.GetArgument("sfm_two_view_min_inliers_number", 100));
    m_sfmTwoViewInliersMinRatio =
        param.GetArgument("sfm_two_view_min_inliers_ratio", 0.8f);
    m_sfmTwoViewHomoRatioTh =
        param.GetArgument("sfm_two_view_homography_ratio_threshold", 0.8f);
    m_sfmTwoViewScale = param.GetArgument("sfm_two_view_scale", 0.01f);
    m_sfmTwoViewRayAngleDotTh = cos(
                                    param.GetArgument("sfm_two_view_ray_angle_threshold",
                                            1.0f) * FACTOR_DEG_TO_RAD);
    m_sfmTwoViewMotionInitialization =
        param.GetArgument("sfm_two_view_motion_initialization", 1) != 0;
    m_sfmMultiViewRayAngleDotTh = cos(
                                      param.GetArgument("sfm_multi_view_ray_angle_threshold",
                                              5.0f) * FACTOR_DEG_TO_RAD);
    m_sfmFocalPriorWeight = param.GetArgument("sfm_focal_prior_weight", 1.0f);

    m_sfmIncrFrmTrksMinNum = FeatureIndex(
                                 param.GetArgument("sfm_min_incremental_frame_tracks_number", 20));
    //m_sfmIncrFrmTrksMinNumInitialRatio = param.GetArgument("sfm_min_incremental_frame_tracks_number_initial_ratio", 0.5f);
    m_sfmIncrFrmTrksMinNumInitialRatio =
        param.GetArgument("sfm_min_incremental_frame_tracks_number_initial_ratio",
                          1.0f);
    m_sfmIncrFrmTrksMinToMaxRatio =
        param.GetArgument("sfm_min_incremental_frame_tracks_number_to_max_ratio", 0.5f);
    m_sfmCamInliersMinNum = FeatureIndex(
                                param.GetArgument("sfm_min_camera_inliers_number", 20));
    const float sfmCamInlierMinRatioIncrStart =
        param.GetArgument("sfm_min_camera_inlier_ratio_start", 0.95f);
    const float sfmCamInlierMinRatioIncrStep =
        param.GetArgument("sfm_min_camera_inlier_ratio_step", -0.01f);
    const float sfmCamInlierMinRatioIncrEnd =
        param.GetArgument("sfm_min_camera_inlier_ratio_end", 0.8f);
    m_sfmCamInlierRatioThs.resize(0);
    for (float ratio = sfmCamInlierMinRatioIncrStart;
            ratio >= sfmCamInlierMinRatioIncrEnd; ratio += sfmCamInlierMinRatioIncrStep)
        m_sfmCamInlierRatioThs.push_back(ratio);
    FrameErrorLevel::SetHighestOutlierLevel(ubyte(m_sfmCamInlierRatioThs.size() -
                                            1));
    m_sfmPtInliersMinNum = FrameIndex(
                               param.GetArgument("sfm_min_point_inliers_number", 3));
    m_sfmPtInliersMinRatio = param.GetArgument("sfm_min_point_inliers_ratio", 0.5f);
    m_sfmDensePts = param.GetArgument("sfm_dense_points", 0) != 0;
    m_sfmFocalRangeFactor = param.GetArgument("sfm_focal_range_factor", 2.0f);

    const float errThHomo = param.GetArgument("error_threshold_homography",
                            FLT_MAX);
    const float errThEp = param.GetArgument("error_threshold_epipolar", 2.0f);
    const float errThReprojStart =
        param.GetArgument("error_threshold_reprojection_start", 4.0f);
    const float errThReprojStep =
        param.GetArgument("error_threshold_reprojection_step", 2.0f);
    const float errThReprojEnd =
        param.GetArgument("error_threshold_reprojection_end", 10.0f);
    m_Festor.m_ransacErrorThreshold = m_Eestor.m_ransacErrorThreshold = errThEp *
                                      errThEp * seq.GetIntrinsicMatrix().one_over_fxy();
    if (errThHomo == FLT_MAX)
        m_Hestor.m_ransacErrorThreshold = FLT_MAX;
    else
        m_Hestor.m_ransacErrorThreshold = errThHomo * errThHomo *
                                          seq.GetIntrinsicMatrix().one_over_fxy();
    m_sfmReprojErrSqThs.resize(0);
    for (float errTh = errThReprojStart; errTh <= errThReprojEnd;
            errTh += errThReprojStep)
        m_sfmReprojErrSqThs.push_back(errTh * errTh *
                                      seq.GetIntrinsicMatrix().one_over_fxy());
    FrameErrorLevel::SetHighestReprojectionErrorLevel(ubyte(
                m_sfmReprojErrSqThs.size() - 1));

    m_Cestor.SetBinSize(ushort(param.GetArgument("arsac_bins_number_x", 10)),
                        ushort(param.GetArgument("arsac_bins_number_y", 10)));
    m_Pestor.SetBinSize(m_Cestor.GetBinSizeX(), m_Cestor.GetBinSizeY());
    m_Cestor.m_ransacMinNumIters = uint(
                                       param.GetArgument("ransac_min_iterations_number_camera", 300));
    m_Cestor.m_ransacMaxNumIters = uint(
                                       param.GetArgument("ransac_max_iterations_number_camera", 500));
    m_Pestor.m_ransacMinNumIters = uint(
                                       param.GetArgument("ransac_min_iterations_number_projective", 300));
    m_Pestor.m_ransacMaxNumIters = uint(
                                       param.GetArgument("ransac_max_iterations_number_projective", 500));
    m_Xestor.m_ransacMinNumIters = uint(
                                       param.GetArgument("ransac_min_iterations_number_point", 1));
    m_Xestor.m_ransacMaxNumIters = uint(
                                       param.GetArgument("ransac_max_iterations_number_point", 20));
    m_Xestor.m_ransacMinInlierProportion = m_sfmPtInliersMinRatio;

    m_baaelLocalMaxNumIters = uint(
                                  param.GetArgument("baael_max_iterations_number_local", 5));
    m_baaelGlobalMaxNumIters = uint(
                                   param.GetArgument("baael_max_iterations_number_global", 3));

    m_baLocalMaxNumAdditionalAdjustedFrms = FrameIndex(
            param.GetArgument("ba_local_max_additional_adjusted_frames_number", 20));
    m_baLocalMinNumTrksPerFrm = FeatureIndex(
                                    param.GetArgument("ba_local_min_tracks_number_per_frame", 500));
    m_baLocalMaxNumIters = uint(param.GetArgument("ba_local_max_iterations_number",
                                20));
    m_baLocalStopMSE = param.GetArgument("ba_local_stop_mse", 0.2f);
    m_baLocalStopRelativeReduction =
        param.GetArgument("ba_local_stop_relative_reduction", 0.001f);
    m_baGlobalMinNumTrksPerFrm = FeatureIndex(
                                     param.GetArgument("ba_global_min_tracks_number_per_frame", 200));
    m_baGlobalMaxNumIters = uint(
                                param.GetArgument("ba_global_max_iterations_number", 50));
    m_baGlobalStopMSE = param.GetArgument("ba_global_stop_mse", 0.1f);
    m_baGlobalStopRelativeReduction =
        param.GetArgument("ba_global_stop_relative_reduction", 0.0001f);

    if (distortionInvalid) {
        m_baDataMetric.InvalidateDistortion();
        m_baDataMetricIntrinsicVariable.InvalidateDistortion();
    }

    m_stop = param.GetArgument("stop", 0) != 0;
    m_view = param.GetArgument("view", 1) != 0;
    if (m_view)
        ProgramGL::Initialize(seq.GetImageWidth(), seq.GetImageHeight());
}

void CameraTracker::Run(Sequence &seq, const std::string outputFileName,
                        FILE *fpTiming) {
    if(outputFileName != "" && seq.LoadBwithDir(outputFileName.c_str()))
        return;

//#if _DEBUG
//  TransformScene(2, 6, seq);
//  m_KrBkp = seq.GetIntrinsicRectification();
//  m_KrsBkp = seq.GetIntrinsicRectifications();
//  m_CsBkp = seq.GetCameras();
//  m_PsBkp = seq.GetProjectiveMatrixes();
//  m_XsBkp = seq.GetPoints();
//#endif

    Timer timer;
    timer.Start();

    const bool stopBkp = m_stop;

#if VERBOSE_CAMERA_TRACKING
    printf("****************************************************************\n");
#endif

#if 1
//#if _DEBUG
//  std::vector<float> fs(seq.GetFramesNumberTotal());
//  for(FrameIndex iFrm = 0; iFrm < seq.GetFramesNumberTotal(); ++iFrm)
//      fs[iFrm] = seq.GetIntrinsicRectification(iFrm).f();
//#endif
    seq.InitializeCameras();
    seq.InitializePoints();
    seq.InitializeMeasurements();
    seq.DenormalizeMeasurements();
    //seq.PrintStates();
    ExtractKeyFrameSequence(seq, m_seqKF, m_iFrmsKF, m_iTrksKF, m_iMeasKF);
    m_seqKF.NormalizeMeasurements();
    const FrameIndex nFrmsKF = m_seqKF.GetFramesNumber();
    m_frmErrLevelsKF.resize(nFrmsKF);
    for(FrameIndex iFrm = 0; iFrm < nFrmsKF; ++iFrm)
        m_frmErrLevelsKF[iFrm].SetLowest();
//#if _DEBUG
//  for(FrameIndex iFrm = 0; iFrm < nFrmsKF; ++iFrm)
//      printf("Frame %d: %f\n", iFrm, fs[m_iFrmsKF[iFrm]]);
//#endif
    //SaveB("F:/tmp/tmpKF.txt", seq, m_seqKF, m_iFrmsKF, m_iTrksKF, m_iMeasKF, m_frmErrLevelsKF);
    //exit(0);
#else
    LoadB("F:/tmp/tmpKF.txt", seq, m_seqKF, m_iFrmsKF, m_iTrksKF, m_iMeasKF,
          m_frmErrLevelsKF);
#endif
    if (nFrmsKF < 3) {
        printf("bug : mFrmsKF < 3");
        return;
    }
    FrameIndex iFrm1, iFrm2, iFrm3;
    //const bool metric = seq.GetIntrinsicType() == Sequence::INTRINSIC_USER_FIXED;
    //const bool metric = seq.GetIntrinsicType() != Sequence::INTRINSIC_VARIABLE;
    const bool metric = true;
#if 1
    SelectInitialFrames(m_seqKF, iFrm1, iFrm2, iFrm3, metric);
    //SaveB("F:/tmp/tmpInit.txt", m_seqKF, iFrm1, iFrm2, iFrm3);
    //exit(0);
#else
    LoadB("F:/tmp/tmpInit.txt", m_seqKF, iFrm1, iFrm2, iFrm3);
#endif

    EstimateInitialStructureAndMotion(iFrm1, iFrm2, iFrm3, m_seqKF, metric);
    m_stop = stopBkp;
    ViewSequence(m_seqKF, iFrm1);
    if(!metric) {
#if 1
        RegisterKeyFrameSequence(m_seqKF, m_frmErrLevelsKF, iFrm1, iFrm3, false);
        //SaveB("F:/tmp/tmpProjective.txt", m_seqKF, iFrm1, iFrm2, iFrm3, m_frmErrLevelsKF);
        //exit(0);
#else
        LoadB("F:/tmp/tmpProjective.txt", m_seqKF, iFrm1, iFrm2, iFrm3,
              m_frmErrLevelsKF);
#endif
        UpdateProjectiveToMetric(iFrm1, iFrm3, m_seqKF, m_frmErrLevelsKF);
//#if _DEBUG
//      m_seqKF.ComputeProjectiveMatrixes();
//      m_seqKF.GetFrameIndexList(FLAG_FRAME_STATE_SOLVED, m_iFrmsAdj);
//      m_seqKF.GetBundleAdjustorData(m_iFrmsAdj, m_baDataProjective, m_iFrmsBA, m_iTrksAdj);
//      printf("%f\n", m_baDataProjective.ComputeSSE() * m_baDataProjective.GetFactorSSEToMSE());
//
//      TransformScene(iFrm1, iFrm3, m_seqKF);
//      //printf("****************************************************************\n");
//      //printf("Frame %d\n", iFrm1);
//      //printf("----------------------------------------------------------------\n");
//      //m_seqKF.GetCamera(iFrm1).Print();
//      //printf("----------------------------------------------------------------\n");
//      //CsBkp[m_iFrmsKF[iFrm1]].Print();
//      //printf("----------------------------------------------------------------\n");
//      //m_seqKF.GetProjectiveMatrix(iFrm1).Print();
//      //printf("----------------------------------------------------------------\n");
//      //PsBkp[m_iFrmsKF[iFrm1]].Print();
//
//      printf("****************************************************************\n");
//      printf("Frame %d\n", iFrm2);
//      printf("----------------------------------------------------------------\n");
//      m_seqKF.GetCamera(iFrm2).Print();
//      printf("----------------------------------------------------------------\n");
//      CsBkp[m_iFrmsKF[iFrm2]].Print();
//      printf("----------------------------------------------------------------\n");
//      m_seqKF.GetProjectiveMatrix(iFrm2).Print();
//      printf("----------------------------------------------------------------\n");
//      PsBkp[m_iFrmsKF[iFrm2]].Print();
//
//      printf("****************************************************************\n");
//      printf("Frame %d\n", iFrm3);
//      printf("----------------------------------------------------------------\n");
//      m_seqKF.GetCamera(iFrm3).Print();
//      printf("----------------------------------------------------------------\n");
//      CsBkp[m_iFrmsKF[iFrm3]].Print();
//      printf("----------------------------------------------------------------\n");
//      m_seqKF.GetProjectiveMatrix(iFrm3).Print();
//      printf("----------------------------------------------------------------\n");
//      PsBkp[m_iFrmsKF[iFrm3]].Print();
//
//      printf("****************************************************************\n");
//      TrackIndexList iTrksInlier;
//      m_seqKF.GetTrackIndexList(FLAG_TRACK_STATE_INLIER, iTrksInlier);
//      const TrackIndex nTrksInlier = TrackIndex(iTrksInlier.size());
//      for(TrackIndex i = 0; i < nTrksInlier; ++i)
//      {
//          const TrackIndex iTrkKF = iTrksInlier[i], iTrk = m_iTrksKF[iTrkKF];
//          const Point3D &X1 = m_seqKF.GetPoint(iTrkKF), &X2 = XsBkp[iTrk];
//          printf("Track %d: (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f) = %f\n", iTrkKF, X1.X(), X1.Y(), X1.Z(), X2.X(), X2.Y(), X2.Z(), X1.SquaredDistance(X2));
//      }
//      exit(0);
//#endif
        m_seqKF.PrintStates();
        m_stop = stopBkp;
        ViewSequence(m_seqKF, iFrm1);
    }

    RegisterKeyFrameSequence(m_seqKF, m_frmErrLevelsKF, iFrm1, iFrm3, true);
#if VERBOSE_CAMERA_TRACKING
    printf("****************************************************************\n");
#endif
    m_seqKF.PrintStates();
    m_stop = stopBkp;
    ViewSequence(m_seqKF, iFrm1);

    RegisterOriginalSequence(m_seqKF, m_frmErrLevelsKF, iFrm1, iFrm3, m_iFrmsKF,
                             m_iTrksKF, m_iMeasKF, seq);

#if VERBOSE_CAMERA_TRACKING
    printf("****************************************************************\n");
#endif
    seq.PrintStates();
    m_stop = stopBkp;
    ViewSequence(seq, m_iFrmsKF[iFrm1]);
    m_stop = stopBkp;

    timer.Stop();
    timer.PrintTotalTiming(0, fpTiming);

    if(outputFileName != "")
        seq.SaveBwithDir(outputFileName.c_str());
}

static inline void PrintKeyFrameCommonTracksNumber(const Sequence &seq,
        const FrameIndexList &iFrmsKey) {
    const FrameIndex nFrmsKey = FrameIndex(iFrmsKey.size());
    for(FrameIndex i = 0; i < nFrmsKey; ++i) {
        const FrameIndex iFrm = iFrmsKey[i];
        const FeatureIndex nTrks = seq.CountFrameTrackedFeatures(iFrm);
        printf("Frame %d: %d\n", iFrm, nTrks);
    }
    for(FrameIndex i1 = 0, i2 = 1; i2 < nFrmsKey; i1 = i2, ++i2) {
        const FrameIndex iFrm1 = iFrmsKey[i1], iFrm2 = iFrmsKey[i2];
        const FeatureIndex nTrks = seq.SearchForFrameCommonTracksNumber(iFrm1, iFrm2);
        printf("Frame (%d, %d): %d\n", iFrm1, iFrm2, nTrks);
    }
    for(FeatureIndex i1 = 0, i2 = 1, i3 = 2; i3 < nFrmsKey;
            i1 = i2, i2 = i3, ++i3) {
        const FrameIndex iFrm1 = iFrmsKey[i1], iFrm2 = iFrmsKey[i2],
                         iFrm3 = iFrmsKey[i3];
        const FeatureIndex nTrks = seq.SearchForFrameCommonTracksNumber(iFrm1, iFrm2,
                                   iFrm3);
        printf("Frame (%d, %d, %d): %d\n", iFrm1, iFrm2, iFrm3, nTrks);
    }
}

void CameraTracker::ExtractKeyFrameSequence(Sequence &seq, Sequence &seqKF,
        FrameIndexList &iFrmsKF, TrackIndexList &iTrksKF,
        MeasurementIndexList &iMeasKF) {
    seq.DenormalizeMeasurements();
    const FrameIndex nFrms = seq.GetFramesNumber();
    FeatureIndex minNumCmnTrksTwoKeyFrms = m_kfMinNumCmnTrksTwoKeyFrms,
                 minNumCmnTrksThreeKeyFrms = m_kfMinNumCmnTrksThreeKeyFrms;
    while(1) {
        seq.UnmarkFramesKeyFrame();
        seq.MarkKeyFrames(minNumCmnTrksTwoKeyFrms, minNumCmnTrksThreeKeyFrms,
                          m_frmMarks);
        iFrmsKF.resize(0);
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            if(!m_frmMarks[iFrm])
                continue;
            seq.MarkFrameKeyFrame(iFrm);
            iFrmsKF.push_back(iFrm);
        }
        if(iFrmsKF.size() >= 3||nFrms<3)//bug nFrms<3
            break;
        ++minNumCmnTrksTwoKeyFrms;
        ++minNumCmnTrksThreeKeyFrms;
    }
#if VERBOSE_CAMERA_TRACKING
    printf("Key frames: %d --> %d\n", nFrms, iFrmsKF.size());
#endif

    iTrksKF.resize(0);
    const TrackIndex nTrks = seq.GetTracksNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(seq.CountTrackMarkedFrameInlierMeasurements(iTrk,
                m_frmMarks) >= m_kfMinTrkLenKeyFrm)
            iTrksKF.push_back(iTrk);
    }
    seq.MarkKeyFrameTracks(iFrmsKF, m_frmMarks, iTrksKF, m_kfNumBinsX, m_kfNumBinsY,
                           m_kfMinNumFtrsPerBin, m_kfMinNumTrksPerKeyFrm, m_kfMinNumCmnTrksTwoKeyFrms,
                           m_kfMinNumCmnTrksThreeKeyFrms, m_trkMarks);
    iTrksKF.resize(0);
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(m_trkMarks[iTrk])
            iTrksKF.push_back(iTrk);
    }
#if VERBOSE_CAMERA_TRACKING
    printf("Key frame tracks: %d --> %d\n", nTrks, iTrksKF.size());
#endif

    seq.GetSubSequence(iFrmsKF, iTrksKF, seqKF, iMeasKF, false, true);
}

void CameraTracker::SelectInitialFrames(Sequence &seq, FrameIndex &iFrm1,
                                        FrameIndex &iFrm2, FrameIndex &iFrm3, const bool metric) {
    const float epsilon = 0.1f;
    const float sigma = 0.2f, one_over_2sigma2 = 0.5f / (sigma * sigma);
    const float beta = 0.04f;

    FrameIndex i;
    float b12, b13, b23, B12, B13, B23, Ecalib, CEcalib, f1, f2, f3, df12, df13,
          df23;
    ENFT_SSE::__m128 work[4];
    const Sequence::IntrinsicType intrinsicType = seq.GetIntrinsicType();
    const FrameIndex nTriples = seq.GetFramesNumber() - 2;
    m_frmTripleScores.assign(nTriples, 0.0f);
    m_Ps.Resize(3);
    for(i = 0; i < nTriples; ++i) {
        iFrm1 = i;
        iFrm2 = i + 1;
        iFrm3 = i + 2;
        if(!EstimateInitialStructureAndMotion(iFrm1, iFrm2, iFrm3, seq, metric))
            continue;
        //if(metric)
        //{
        //  m_frmTripleScores[i] = float(seq.CountFrameInlierFeatures(iFrm1) + seq.CountFrameInlierFeatures(iFrm2) + seq.CountFrameInlierFeatures(iFrm3));
        //  ViewSequence(seq, iFrm1);
        //  continue;
        //}
        b12 = ComputeImageBasedDistance(seq, iFrm1, iFrm2);
        b13 = ComputeImageBasedDistance(seq, iFrm1, iFrm3);
        b23 = ComputeImageBasedDistance(seq, iFrm2, iFrm3);
        if(metric) {
            m_frmTripleScores[i] = b12 + b13 + b23;
            ViewSequence(seq, iFrm1);
            continue;
        }
        m_Ps[0] = seq.GetProjectiveMatrix(iFrm1);
        m_Ps[1] = seq.GetProjectiveMatrix(iFrm2);
        m_Ps[2] = seq.GetProjectiveMatrix(iFrm3);
        if(!m_Qestor.Run(m_Ps, m_Q, Ecalib))
            continue;
        //CEcalib = epsilon * exp(-Ecalib / (2 * 2 * sigma2)) / (epsilon + sqrt(Ecalib / 2));
        CEcalib = epsilon * exp(-one_over_2sigma2 * Ecalib) / (epsilon + sqrt(Ecalib));
        Sequence::UpdateToMetric(m_Q, m_Ps);
        m_Ps[0].ToIdealIntrinsicExtrinsic(f1, m_C, work);
        m_Ps[1].ToIdealIntrinsicExtrinsic(f2, m_C, work);
        m_Ps[2].ToIdealIntrinsicExtrinsic(f3, m_C, work);
        df12 = 0.5f * (fabs(f1/f2 - 1) + fabs(f2/f1 - 1)) / b12;
        df13 = 0.5f * (fabs(f1/f3 - 1) + fabs(f3/f1 - 1)) / b13;
        df23 = 0.5f * (fabs(f2/f3 - 1) + fabs(f3/f2 - 1)) / b13;
        B12 = b12 / (beta+df12);
        B13 = b13 / (beta+df13);
        B23 = b23 / (beta+df23);
        m_frmTripleScores[i] = CEcalib * (B12+B13+B23);
#if VERBOSE_CAMERA_TRACKING
        printf("----------------------------------------------------------------\n");
        printf("Frame (%d, %d, %d)\n  f = (%.2f, %.2f, %.2f), Ecalib = %f, S = %f\n",
               iFrm1, iFrm2, iFrm3, f1, f2, f3, Ecalib, m_frmTripleScores[i]);
#endif
        ViewSequence(seq, iFrm1);
    }
    SelectInitialFrames(m_frmTripleScores, iFrm1, iFrm2, iFrm3);
}

void CameraTracker::SelectInitialFrames(const std::vector<float>
                                        &frmTripleScores, FrameIndex &iFrm1, FrameIndex &iFrm2, FrameIndex &iFrm3) {
//#if _DEBUG
//  IO::SaveValues("F:/tmp/scores.txt", frmTripleScores);
//#endif
    FrameIndex i, i1, i2, i3;
    int dist;
    float weight, weightSum, weightedScoreSum, scoreMax;
    const FrameIndex nTriples = FrameIndex(frmTripleScores.size());
    if(m_sfmInitScoreFilterSigma == 0)
        m_frmTripleScoresFilter = frmTripleScores;
    else {
        m_frmTripleScoresFilter.resize(nTriples);
        const FrameIndex winSize = m_sfmInitScoreFilterSigma * 3;
        const float one_over_2sigma2 = 0.5f / (float(m_sfmInitScoreFilterSigma) * float(
                m_sfmInitScoreFilterSigma));
        for(i = 0; i < nTriples; ++i) {
            if(frmTripleScores[i] == 0.0f) {
                m_frmTripleScoresFilter[i] = 0.0f;
                continue;
            }
            i1 = i < winSize ? 0 : i - winSize;
            i3 = i + winSize > nTriples ? nTriples : i + winSize;
            for(i2 = i1, weightSum = weightedScoreSum = 0.0f; i2 < i3; ++i2) {
                dist = int(i2) - i;
                weight = exp(-one_over_2sigma2 * dist * dist);
                weightSum += weight;
                weightedScoreSum += weight * frmTripleScores[i2];
            }
            m_frmTripleScoresFilter[i] = weightedScoreSum / weightSum;
        }
    }
//#if _DEBUG
#if 0
    IO::SaveValues("E:/tmp/scores_filter.txt", m_frmTripleScoresFilter);
#endif
    for(i = 0, iFrm1 = iFrm2 = iFrm3 = INVALID_FRAME_INDEX, scoreMax = -FLT_MAX;
            i < nTriples; ++i) {
        //if(m_frmTripleScoresFilter[i] < scoreMax || i + 1 < nTriples && m_frmTripleScoresFilter[i + 1] == 0.0f || i + 2 < nTriples && m_frmTripleScoresFilter[i + 2] == 0.0f)
        if(m_frmTripleScoresFilter[i] < scoreMax)
            continue;
        iFrm1 = i;
        iFrm2 = i + 1;
        iFrm3 = i + 2;
        scoreMax = m_frmTripleScoresFilter[i];
    }
}

void CameraTracker::SelectIncrementalFrames(const Sequence &seq,
        FrameIndexList &iFrmsIncr, const bool metric) {
    m_candidateFrms.resize(0);
    FeatureIndex nTrksInlier, nTrksInlierMax = 0;
    const FrameIndex nFrms = seq.GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        if(seq.GetFrameState(iFrm) & FLAG_FRAME_STATE_SOLVED)
            continue;
        nTrksInlier = metric ? seq.CountFrameInlierTracks(iFrm) :
                      seq.CountFrameInlierInitialTracks(iFrm);
        m_candidateFrms.push_back(CandidateFrame(iFrm, nTrksInlier));
        if(nTrksInlier > nTrksInlierMax)
            nTrksInlierMax = nTrksInlier;
    }
    const FeatureIndex nTrksInlierMin = std::max(m_sfmIncrFrmTrksMinNum,
                                        FeatureIndex(nTrksInlierMax * m_sfmIncrFrmTrksMinToMaxRatio + 0.5f));
    const TrackIndex nTrksInlierInitial = seq.CountTracks(FLAG_TRACK_STATE_INITIAL,
                                          FLAG_TRACK_STATE_INLIER);
    const FeatureIndex nTrksInlierMinProjective = FeatureIndex(
                nTrksInlierInitial * m_sfmIncrFrmTrksMinNumInitialRatio + 0.5f);

    iFrmsIncr.resize(0);
    const FrameIndex nCandidateFrms = FrameIndex(m_candidateFrms.size());
    for(FrameIndex i = 0; i < nCandidateFrms; ++i) {
        if(m_candidateFrms[i].GetInlierTracksNumber() >= nTrksInlierMin && (metric ||
                m_candidateFrms[i].GetInlierTracksNumber() >= nTrksInlierMinProjective))
            iFrmsIncr.push_back(m_candidateFrms[i].GetFrameIndex());
    }

#if VERBOSE_CAMERA_TRACKING
    printf("Select %d / %d incremental frames\n", iFrmsIncr.size(), nCandidateFrms);
    printf("Inlier tracks number: max = %d, min = %d\n", nTrksInlierMax,
           nTrksInlierMin);
#endif
}

void CameraTracker::RegisterKeyFrameSequence(Sequence &seq,
        std::vector<FrameErrorLevel> &frmErrLevels, const FrameIndex &iFrm1,
        const FrameIndex &iFrm2,
        const bool metric) {
    //SaveB("F:/tmp/KF0003.txt", seq, frmErrLevels);
#if 1
    while(1) {
#if VERBOSE_CAMERA_TRACKING
        printf("****************************************************************\n");
#endif
        //char fileName[MAX_LINE_LENGTH];
        //sprintf(fileName, "F:/tmp/KF%04d.txt", seq.CountFrames(FLAG_FRAME_STATE_SOLVED));
        //SaveB(fileName, seq, frmErrLevels);

        SelectIncrementalFrames(seq, m_iFrmsIncr, metric);
        const FrameIndex nFrmsIncr = FrameIndex(m_iFrmsIncr.size());
        if(nFrmsIncr == 0)
            break;
        EstimateIncrementalMotion(m_iFrmsIncr, m_iFrmsSolve, seq, frmErrLevels, metric);
        if(m_iFrmsSolve.empty())
            break;
        //BundleAdjustAdaptiveErrorLevelLocal(iFrm1, m_iFrmsSolve, seq, frmErrLevels, metric, true);
        BundleAdjustAdaptiveErrorLevelLocal(iFrm1, iFrm2, m_iFrmsSolve, seq,
                                            frmErrLevels, metric, false);
    }
    //BundleAdjustAdaptiveErrorLevelGlobal(iFrm1, iFrm2, seq, frmErrLevels, metric, true);
    BundleAdjustAdaptiveErrorLevelGlobal(iFrm1, iFrm2, seq, frmErrLevels, metric,
                                         false);
    //SaveB("F:/tmp/KF.txt", seq, m_iFrmsKF, m_iTrksKF, m_iMeasKF, frmErrLevels);
    //exit(0);
#else
    LoadB("F:/tmp/KF.txt", seq, m_iFrmsKF, m_iTrksKF, m_iMeasKF, frmErrLevels);
    BundleAdjustAdaptiveErrorLevelGlobal(iFrmFixKF, seq, frmErrLevels, metric,
                                         false);
    ViewSequence(seq, iFrmFixKF);
#endif
}

void CameraTracker::RegisterOriginalSequence(const Sequence &seqKF,
        const std::vector<FrameErrorLevel> &frmErrLevelsKF, const FrameIndex &iFrm1KF,
        const FrameIndex &iFrm2KF, const FrameIndexList &iFrmsKF,
        const TrackIndexList &iTrksKF,
        const MeasurementIndexList &iMeasKF, Sequence &seq) {
    seq.SetIntrinsicRectification(seqKF.GetIntrinsicRectification());

    const FrameIndex nFrms = seq.GetFramesNumber();
    m_frmErrLevels.resize(nFrms);
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
        m_frmErrLevels[iFrm].SetLowest();

    const FrameIndex nFrmsKF = FrameIndex(iFrmsKF.size());
    for(FrameIndex i = 0; i < nFrmsKF; ++i) {
        const FrameIndex iFrm = iFrmsKF[i];
        if(seq.GetIntrinsicType() == Sequence::INTRINSIC_VARIABLE)
            seq.SetCamera(iFrm, seqKF.GetCamera(i), seqKF.GetIntrinsicRectification(i));
        else
            seq.SetCamera(iFrm, seqKF.GetCamera(i));
        seq.SetFrameState(iFrm, seqKF.GetFrameState(i));
        m_frmErrLevels[iFrm] = frmErrLevelsKF[i];
    }

    const TrackIndex nTrksKF = TrackIndex(iTrksKF.size());
    for(TrackIndex i = 0; i < nTrksKF; ++i) {
        const TrackIndex iTrk = iTrksKF[i];
        seq.SetPoint(iTrk, seqKF.GetPoint(i));
        seq.SetTrackState(iTrk, seqKF.GetTrackState(i));
    }

    const MeasurementIndex nMeasKF = MeasurementIndex(iMeasKF.size());
    for(MeasurementIndex i = 0; i < nMeasKF; ++i)
        seq.SetMeasurementState(iMeasKF[i], seqKF.GetMeasurementState(i));

    if(!m_sfmDensePts) {
        const TrackIndex nTrks = seq.GetTracksNumber();
        m_trkMarks.assign(nTrks, false);
        for(TrackIndex i = 0; i < nTrksKF; ++i)
            m_trkMarks[iTrksKF[i]] = true;
        for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
            if(!m_trkMarks[iTrk])
                seq.BreakTrack(iTrk);
        }
        seq.RemoveBrokenTracks();
        seq.RemoveNullMeasurements();
    }

    seq.NormalizeMeasurements();

    const FrameIndex iFrm1 = iFrmsKF[iFrm1KF], iFrm2 = iFrmsKF[iFrm2KF];
    m_pSeq = &seq;
    ViewSequence(seq, iFrm1);

    while(1) {
#if VERBOSE_CAMERA_TRACKING
        printf("****************************************************************\n");
#endif
        m_iFrmsIncr.resize(0);
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            if(!(seq.GetFrameState(iFrm) & FLAG_FRAME_STATE_SOLVED))
                m_iFrmsIncr.push_back(iFrm);
        }
        const FrameIndex nFrmsIncr = FrameIndex(m_iFrmsIncr.size());
        if(nFrmsIncr == 0)
            break;
        EstimateIncrementalMotion(m_iFrmsIncr, m_iFrmsSolve, seq, m_frmErrLevels, true);
        if(m_iFrmsSolve.empty())
            break;
        //SaveB("E:/tmp/ORI.txt", seq, iFrm1Init, iFrmsIncrScc, frmErrLevels);
        //exit(0);
        BundleAdjustAdaptiveErrorLevelLocal(iFrm1, iFrm2, m_iFrmsSolve, seq,
                                            m_frmErrLevels, true, false);
    }
    BundleAdjustAdaptiveErrorLevelGlobal(iFrm1, iFrm2, seq, m_frmErrLevels, true,
                                         false);
}