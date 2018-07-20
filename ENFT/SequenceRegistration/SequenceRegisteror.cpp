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
#include "Utility/Timer.h"
#include "SfM/CameraEstimator.h"
#include "ProgramGL/ProgramGL.h"

using namespace ENFT_SfM;

ubyte SequenceRegisteror::SequenceErrorLevel::g_outlierLevelHighest = 0;
ubyte SequenceRegisteror::SequenceErrorLevel::g_reprojErrLevelHighest = 0;

void SequenceRegisteror::Initialize(const SequenceSet &seqs,
                                    const std::string paramFileName, const bool distortionInvalid) {
    Configurator param;
    param.Load(paramFileName.c_str());
    Initialize(seqs, param, distortionInvalid);
}

void SequenceRegisteror::Run(SequenceSet &seqs,
                             const std::string outputFileName, const std::string tmpFileNameKF,
                             const std::string tmpFileNameRelativePoses,
                             const std::string tmpFileNameLocal, const std::string tmpFileNameGlobal,
                             const std::string tmpFileNameBA, FILE *fpTiming) {
//  if(outputFileName != "" && seqs.LoadB(outputFileName.c_str()))
//      return;

    const bool stopBkp = m_stop;

#if VERBOSE_SEQUENCE_REGISTRATION
    printf("****************************************************************\n");
#endif

    const Sequence::IntrinsicType intrinsicType = seqs.GetIntrinsicType();
    m_seqsKF.SetDirectory(seqs.GetDirectory());
    m_seqsKF.SetIntrinsicType(intrinsicType);
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    m_seqsKF.CreateSequences(nSeqs);
    std::string tmpFileName = tmpFileNameKF == "" ? "" : seqs.GetDirectory() +
                              tmpFileNameKF;
    if(tmpFileName == "" ||
            !LoadB(tmpFileName.c_str(), seqs, m_seqsKF, m_iFrmsListKF, m_iTrksListIdvKF,
                   m_iTrksCmnKF)) {
        ExtractKeyFrameSequences(seqs, m_seqsKF, m_iFrmsListKF, m_iTrksListIdvKF,
                                 m_iTrksCmnKF);
        //std::vector<FrameIndexList> iFrmsListFix(nSeqs);
        //for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
        //  iFrmsListFix[iSeq].push_back(0);
        //BundleAdjustmentIndividual(m_seqsKF, iFrmsListFix, m_baLocalMaxNumIters, m_baLocalStopMSE, m_baLocalStopRelativeReduction);
        if(tmpFileName != "")
            SaveB(tmpFileName.c_str(), seqs, m_seqsKF, m_iFrmsListKF, m_iTrksListIdvKF,
                  m_iTrksCmnKF);
    }
    //exit(0);

    Timer timer(5);

    std::vector<AlignedVector<RigidTransformation3D> > TsList(
        seqs.GetSequencesNumber());
    tmpFileName = tmpFileNameRelativePoses == "" ? "" : seqs.GetDirectory() +
                  tmpFileNameRelativePoses;
    if(m_spsfmInconsistencyWeightGradient != 1.0f && (tmpFileName == "" ||
            !LoadRelativePoses(tmpFileName.c_str(), TsList))) {
#if VERBOSE_SEQUENCE_REGISTRATION
        printf("****************************************************************\n");
#endif
        timer.Start(0);
        EstimateRelativePoses(m_seqsKF, TsList);
        timer.Stop(0);
        if(tmpFileName != "")
            SaveRelativePoses(tmpFileName.c_str(), TsList);
    }

    SequenceIndex iSeqInit;
    m_seqsKF.SetCommonPointsNumber(m_seqsKF.GetCommonTracksNumber());
    m_seqErrLevels.resize(nSeqs);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
        m_seqErrLevels[iSeq].Initialize();

#if VERBOSE_SEQUENCE_REGISTRATION
    printf("****************************************************************\n");
#endif

    //for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
    //  m_seqsKF[iSeq].NormalizeMeasurements();
    //m_seqsKF.SetSequenceStates(FLAG_SEQUENCE_STATE_DEFAULT);
    //m_seqsKF.SetCommonTrackStates(FLAG_COMMON_TRACK_STATE_DEFAULT);

    SelectInitialSequence(m_seqsKF, iSeqInit);
    m_seqsKF.MarkSequenceRegistered(iSeqInit);
    ViewSequences(m_seqsKF, iSeqInit);

    if(nSeqs == 1)
        return;

    if(tmpFileNameLocal == "" || !m_seqsKF.LoadB(tmpFileNameLocal.c_str())) {
        timer.Start(1);
        SequenceIndex iSeqIncr;
        TrackIndex nTrksCmnIncr;
        while(1) {
            GetIncrementalSequence(m_seqsKF, iSeqIncr, nTrksCmnIncr);
            if(iSeqIncr == INVALID_SEQUENCE_INDEX)
                break;
            RegisterSequence_MarkCommonOutliers(iSeqIncr, m_seqsKF,
                                                m_seqErrLevels[iSeqIncr]);
//#if _DEBUG
#if 0
            m_stop = stopBkp;
            ViewSequences(m_seqsKF, iSeqInit);
#endif
        }
        timer.Stop(1);
        if(tmpFileNameLocal != "")
            m_seqsKF.SaveB(tmpFileNameLocal.c_str());
    }
    m_stop = stopBkp;
    ViewSequences(m_seqsKF, iSeqInit);

    if(tmpFileNameGlobal == "" || !m_seqsKF.LoadB(tmpFileNameGlobal.c_str())) {
        timer.Start(2);
        SequenceIndexList iSeqsAdj3D;
        //m_seqsKF.GetSequenceIndexList(FLAG_SEQUENCE_STATE_REGISTRED, iSeqsAdj3D);
        for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
            if(iSeq != iSeqInit &&
                    (m_seqsKF.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED))
                iSeqsAdj3D.push_back(iSeq);
        }
        BundleAdjust3D(iSeqsAdj3D, m_iSeqsBA, m_iTrksCmnBA, m_seqsKF);
        timer.Stop(2);
        if(tmpFileNameGlobal != "")
            m_seqsKF.SaveB(tmpFileNameGlobal.c_str());
    }
    m_stop = stopBkp;
    ViewSequences(m_seqsKF, iSeqInit);

    if(tmpFileNameBA == "" || !m_seqsKF.LoadB(tmpFileNameBA.c_str())) {
        timer.Start(3);
        BundleAdjust2DAdaptiveErrorLevelGlobal(TsList, iSeqInit, m_seqsKF,
                                               m_seqErrLevels);
        timer.Stop(3);
        if(tmpFileNameBA != "")
            m_seqsKF.SaveB(tmpFileNameBA.c_str());
    }
    m_stop = stopBkp;
    ViewSequences(m_seqsKF, iSeqInit);

    //m_iSeqsAdj.resize(0);
    //for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
    //{
    //  if(iSeq != iSeqInit)
    //      m_iSeqsAdj.push_back(iSeq);
    //}
    //seqs.MarkSequencesCommonTracks(m_iSeqsAdj, m_cmnTrkMarks);
    //UpdateStructureAndInlierStates(m_cmnTrkMarks, m_seqsKF, m_seqErrLevels);
    //ViewSequences(m_seqsKF, iSeqInit);
    //BundleAdjustmentIndividual(m_seqsKF, m_baLocalMaxNumIters, m_baLocalStopMSE, m_baLocalStopRelativeReduction);

    timer.Start(4);
    RegisterOriginalSequences(m_seqsKF, m_iFrmsListKF, m_iTrksListIdvKF,
                              m_iTrksCmnKF, seqs);
    //SaveB("E:/tmp/test.txt", seqs, m_seqsKF, m_iFrmsListKF, m_iTrksListIdvKF, m_iTrksCmnKF);
    //exit(0);
    //LoadB("E:/tmp/test.txt", seqs, m_seqsKF, m_iFrmsListKF, m_iTrksListIdvKF, m_iTrksCmnKF);
    BundleAdjustmentIndividual(seqs, m_iFrmsListKF, m_baLocalMaxNumIters,
                               m_baLocalStopMSE, m_baLocalStopRelativeReduction);
    timer.Stop(4);

    m_stop = stopBkp;
    ViewSequences(seqs, iSeqInit);

    if(fpTiming) {
        fprintf(fpTiming, "Relative pose:     ");
        timer.PrintTotalTiming(0, fpTiming);
        fprintf(fpTiming, "Local:             ");
        timer.PrintTotalTiming(1, fpTiming);
        fprintf(fpTiming, "Global:            ");
        timer.PrintTotalTiming(2, fpTiming);
        fprintf(fpTiming, "Segment based SfM: ");
        timer.PrintTotalTiming(3, fpTiming);
        fprintf(fpTiming, "Interpolation:     ");
        timer.PrintTotalTiming(4, fpTiming);
    } else {
        printf("Relative pose:     ");
        timer.PrintTotalTiming(0);
        printf("Local:             ");
        timer.PrintTotalTiming(1);
        printf("Global:            ");
        timer.PrintTotalTiming(2);
        printf("Segment based SfM: ");
        timer.PrintTotalTiming(3);
        printf("Interpolation:     ");
        timer.PrintTotalTiming(4);
    }

    if(outputFileName != "")
        seqs.SaveB(outputFileName.c_str());
}

void SequenceRegisteror::Initialize(const SequenceSet &seqs,
                                    const Configurator &param, const bool distortionInvalid) {
    ProgramGL::Initialize(seqs[0].GetImageWidth(), seqs[0].GetImageHeight());

    m_kfMinNumCmnTrksTwoSeqs = TrackIndex(
                                   param.GetArgument("kf_min_common_tracks_number_two_sequences", 300));
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

    m_sfmSeqMaxNumTransformations = ushort(
                                        param.GetArgument("sfm_max_sequence_transformations_number", 2));
    m_sfmSeqInliersMinNum = TrackIndex(
                                param.GetArgument("sfm_min_sequence_inliers_number", 20));
    const float sfmSeqInliersMinRatioStart =
        param.GetArgument("sfm_min_sequence_inliers_ratio_start", 0.9f);
    const float sfmSeqInliersMinRatioStep =
        param.GetArgument("sfm_min_sequence_inliers_ratio_step", -0.1f);
    const float sfmSeqInliersMinRatioEnd =
        param.GetArgument("sfm_min_sequence_inliers_ratio_end", 0.5f);
    m_sfmSeqInliersRatioThs.resize(0);
    for(float sfmSeqInliersMinRatio = sfmSeqInliersMinRatioStart;
            sfmSeqInliersMinRatio >= sfmSeqInliersMinRatioEnd;
            sfmSeqInliersMinRatio += sfmSeqInliersMinRatioStep)
        m_sfmSeqInliersRatioThs.push_back(sfmSeqInliersMinRatio);
    SequenceErrorLevel::SetHighestOutlierLevel(ubyte(m_sfmSeqInliersRatioThs.size()
            - 1));
    //m_sfm3DErrSqThFactor = param.GetArgument("sfm_3d_squared_error_threshold_factor", 5.0f);
    m_sfm3DErrThRatio = param.GetArgument("sfm_3d_error_threshold_ratio", 0.9f);
    m_sfmDensePts = param.GetArgument("sfm_dense_points", 0) != 0;

    const float errThReprojStart =
        param.GetArgument("error_threshold_reprojection_start", 4.0f);
    const float errThReprojStep =
        param.GetArgument("error_threshold_reprojection_step", 2.0f);
    const float errThReprojEnd =
        param.GetArgument("error_threshold_reprojection_end", 10.0f);
    m_sfmReprojErrSqThs.resize(0);
    for(float errTh = errThReprojStart; errTh <= errThReprojEnd;
            errTh += errThReprojStep)
        m_sfmReprojErrSqThs.push_back(errTh * errTh);
    SequenceErrorLevel::SetHighestReprojectionErrorLevel(ubyte(
                m_sfmReprojErrSqThs.size() - 1));

    const float errThReprojInternal =
        param.GetArgument("error_threshold_reprojection_internal", 1.0f);
    m_spsfmReprojErrSqThInternal = errThReprojInternal * errThReprojInternal;

    m_sfmMaxNumPlanes = param.GetArgument("sfm_max_planes_number", 3);
    m_sfmMinNumPtsPerPlane = param.GetArgument("sfm_min_points_number_per_plane",
                             100);
    //m_Pestor.m_ransacErrorThreshold = param.GetArgument("sfm_point_to_plane_distance_threshold", 0.01f);

    m_TSestor.m_ransacErrorThreshold = FLT_MAX;
    m_Sestor.m_ransacMinNumIters = uint(
                                       param.GetArgument("ransac_min_iterations_number_similarity", 100));
    m_Sestor.m_ransacMaxNumIters = uint(
                                       param.GetArgument("ransac_max_iterations_number_similarity", 1000));
    m_Cestor.m_ransacMinNumIters = uint(
                                       param.GetArgument("ransac_min_iterations_number_camera", 100));
    m_Cestor.m_ransacMaxNumIters = uint(
                                       param.GetArgument("ransac_max_iterations_number_camera", 300));
    //m_Pestor.m_ransacMaxNumIters = uint(param.GetArgument("ransac_max_iterations_number_plane", 3000));

    m_baaelLocalMaxNumIters = uint(
                                  param.GetArgument("baael_max_iterations_number_local", 5));
    m_baaelGlobalMaxNumIters = uint(
                                   param.GetArgument("baael_max_iterations_number_global", 3));

    m_ba3D.m_lmStopMSE = m_gto.m_lmStopMSE = 0;
    m_ba3D.m_lmMaxNumIters = m_gto.m_lmMaxNumIters = uint(
                                 param.GetArgument("ba_3d_max_iterations_number", 100));
    m_baIdvMinNumTrksPerFrm = FeatureIndex(
                                  param.GetArgument("ba_individual_min_tracks_number_per_frame", 300));
    m_baLocalMaxNumAdditionalAdjustedSeqs = SequenceIndex(
            param.GetArgument("ba_local_max_additional_adjusted_sequences_number", 100));
    m_baLocalMaxNumIters = uint(param.GetArgument("ba_local_max_iterations_number",
                                20));
    m_baLocalStopMSE = param.GetArgument("ba_local_stop_mse", 0.2f);
    m_baLocalStopRelativeReduction =
        param.GetArgument("ba_local_stop_relative_reduction", 0.001f);
    m_baGlobalMaxNumIters = uint(
                                param.GetArgument("ba_global_max_iterations_number", 50));
    m_baGlobalStopMSE = param.GetArgument("ba_global_stop_mse", 0.1f);
    m_baGlobalStopRelativeReduction =
        param.GetArgument("ba_global_stop_relative_reduction", 0.0001f);
    if(distortionInvalid) {
        m_baData2D.InvalidateDistortion();
        m_baData.InvalidateDistortion();
        m_baDataIntrinsicVariable.InvalidateDistortion();
    }

    m_spsfmMaxNumIters = ushort(param.GetArgument("spsfm_max_iterations_number",
                                10));
    m_spsfmSegsNumPerSeqInit = SegmentIndex(
                                   param.GetArgument("spsfm_segments_number_per_sequence_initial", 1));
    m_spsfmSegsNumPerSeqFactor = SegmentIndex(
                                     param.GetArgument("spsfm_segments_number_per_sequence_factor", 2));
    m_spsfmSegsNumPerSeqIncr = SegmentIndex(
                                   param.GetArgument("spsfm_segments_number_per_sequence_increment", 2));
    m_spsfmInconsistencyWeightGradient =
        param.GetArgument("spsfm_inconsistency_weight_gradient", 0.5f);

    m_stop = param.GetArgument("stop", 0) != 0;
    m_view = param.GetArgument("view", 1) != 0;
}

void SequenceRegisteror::ExtractKeyFrameSequences(SequenceSet &seqs,
        SequenceSet &seqsKF, std::vector<FrameIndexList> &iFrmsListKF,
        std::vector<TrackIndexList> &iTrksListIdvKF, TrackIndexList &iTrksCmnKF) {
    SequenceIndex iSeq;
    FeatureIndex iFrm, nFrmsTotal = 0, nFrmsTotalKF = 0;
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    std::vector<std::vector<bool> > frmMarksListKF(nSeqs);
    iFrmsListKF.resize(nSeqs);
    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        Sequence &seq = seqs[iSeq];
        std::vector<bool> &frmMarksKF = frmMarksListKF[iSeq];
        seq.UnmarkFramesKeyFrame();
        seq.MarkKeyFrames(m_kfMinNumCmnTrksTwoKeyFrms, m_kfMinNumCmnTrksThreeKeyFrms,
                          frmMarksKF);
        const FrameIndex nFrms = FrameIndex(frmMarksKF.size());
        frmMarksKF[nFrms - 1] = true;
        FrameIndexList &iFrmsKF = iFrmsListKF[iSeq];
        iFrmsKF.resize(0);
        for(iFrm = 0; iFrm < nFrms; ++iFrm) {
            if(!frmMarksKF[iFrm] || !(seq.GetFrameState(iFrm) & FLAG_FRAME_STATE_SOLVED))
                continue;
            iFrmsKF.push_back(iFrm);
            seq.MarkFrameKeyFrame(iFrm);
        }
        nFrmsTotal += nFrms;
        nFrmsTotalKF += FrameIndex(iFrmsKF.size());
#if VERBOSE_SEQUENCE_REGISTRATION
        printf("Sequence %d: %d --> %d frames\n", iSeq, nFrms, iFrmsKF.size());
#endif
    }
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Total: %d --> %d frames\n", nFrmsTotal, nFrmsTotalKF);
    printf("----------------------------------------------------------------\n");
#endif
#if 1
    TrackIndex iTrkCmn;
    std::vector<CandidateCommonTrack> candidateCmnTrks;
    const TrackIndex nTrksCmn = seqs.GetCommonTracksNumber();
    candidateCmnTrks.resize(nTrksCmn);
    for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn) {
        candidateCmnTrks[iTrkCmn].Set(iTrkCmn,
                                      SequenceIndex(seqs.GetCommonTrackIndividualTrackIndexList(iTrkCmn).size()),
                                      seqs.CountCommonTrackInlierIndividualTracks(iTrkCmn),
                                      seqs.CountCommonTrackInlierMeasurements(iTrkCmn));
    }
    std::sort(candidateCmnTrks.begin(), candidateCmnTrks.end());

    SequenceIndex iSeq1, iSeq2;
    m_cmnTrkMarks.assign(nTrksCmn, false);
    Table<TrackIndex, SequenceIndex> cmnTrkCntTable(nSeqs, nSeqs);
    cmnTrkCntTable.SetZero();
    for(TrackIndex i = 0; i < nTrksCmn; ++i) {
        iTrkCmn = candidateCmnTrks[i].GetCommonTrackIndex();
        const SequenceTrackIndexList &iSeqTrksIdv =
            seqs.GetCommonTrackIndividualTrackIndexList(iTrkCmn);
        const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
        bool notEnough = true;
        for(SequenceIndex j1 = 0; j1 < nCrsps && notEnough; ++j1) {
            iSeq1 = iSeqTrksIdv[j1].GetSequenceIndex();
            for(SequenceIndex j2 = j1 + 1; j2 < nCrsps && notEnough; ++j2) {
                iSeq2 = iSeqTrksIdv[j2].GetSequenceIndex();
                notEnough = cmnTrkCntTable[iSeq1][iSeq2] < m_kfMinNumCmnTrksTwoSeqs;
            }
        }
        if(!notEnough)
            continue;
        m_cmnTrkMarks[iTrkCmn] = true;
#if _DEBUG
        seqs.AssertCommonTrackSorted(iTrkCmn);
#endif
        for(SequenceIndex j1 = 0; j1 < nCrsps; ++j1) {
            iSeq1 = iSeqTrksIdv[j1].GetSequenceIndex();
            for(SequenceIndex j2 = j1 + 1; j2 < nCrsps; ++j2) {
                iSeq2 = iSeqTrksIdv[j2].GetSequenceIndex();
                ++cmnTrkCntTable[iSeq1][iSeq2];
            }
        }
    }

    TrackIndex iTrkIdv, nTrksIdvTotal = 0, nTrksIdvTotalKF = 0;
    FrameIndex trkLen;
    std::vector<bool> idvTrkMarks;
    iTrksListIdvKF.resize(nSeqs);
    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        /*const */Sequence &seq = seqs[iSeq];
        const std::vector<bool> &frmMarksKF = frmMarksListKF[iSeq];
        TrackIndexList &iTrksIdvKF = iTrksListIdvKF[iSeq];
        iTrksIdvKF.resize(0);
        const TrackIndex nTrksIdv = seq.GetTracksNumber();
        for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
            //if((iTrkCmn = seqs.GetIndividualTrackCommonTrack(iSeq, iTrkIdv)) != INVALID_TRACK_INDEX && (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
            //&& m_cmnTrkMarks[iTrkCmn] && seq.CountTrackMarkedFrameInlierMeasurements(iTrkIdv, frmMarks) >= 2)
            //  iTrksIdv.push_back(iTrkIdv);
            if((seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) &&
                    ((trkLen = seq.CountTrackMarkedFrameInlierMeasurements(iTrkIdv,
                               frmMarksKF)) >= m_kfMinTrkLenKeyFrm
                     || (iTrkCmn = seqs.GetIndividualTrackCommonTrack(iSeq,
                                   iTrkIdv)) != INVALID_TRACK_INDEX && m_cmnTrkMarks[iTrkCmn] && trkLen >= 2))
                iTrksIdvKF.push_back(iTrkIdv);
        }
        seq.DenormalizeMeasurements();
        seq.MarkKeyFrameTracks(iFrmsListKF[iSeq], frmMarksKF, iTrksIdvKF, m_kfNumBinsX,
                               m_kfNumBinsY, m_kfMinNumFtrsPerBin, m_kfMinNumTrksPerKeyFrm,
                               m_kfMinNumCmnTrksTwoKeyFrms, m_kfMinNumCmnTrksThreeKeyFrms, idvTrkMarks);
        iTrksIdvKF.resize(0);
        for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
            if(idvTrkMarks[iTrkIdv])
                iTrksIdvKF.push_back(iTrkIdv);
        }
        nTrksIdvTotal += nTrksIdv;
        nTrksIdvTotalKF += TrackIndex(iTrksIdvKF.size());
#if VERBOSE_SEQUENCE_REGISTRATION
        printf("Sequence %d: %d --> %d individual tracks\n", iSeq, nTrksIdv,
               iTrksIdvKF.size());
#endif
    }
#else
    iTrksListIdvKF.resize(nSeqs);
    std::vector<bool> trkMarks;
    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        const Sequence &seq = seqs[iSeq];
        const std::vector<bool> &frmMarks = frmMarksList[iSeq];
        TrackIndexList &iTrksIdvKF = iTrksListIdvKF[iSeq];
        iTrksIdvKF.resize(0);
        const TrackIndex nTrksIdv = seq.GetTracksNumber();
        for(TrackIndex iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
            if((seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) &&
                    seq.CountTrackMarkedFrameInlierMeasurements(iTrkIdv,
                            frmMarks) >= m_sfmMinTrkLenKeyFrm)
                iTrksIdvKF.push_back(iTrkIdv);
        }
        seq.MarkKeyFrameTracks(iFrmsListKF[iSeq], iTrksIdvKF, m_sfmMinNumTrksPerKeyFrm,
                               m_sfmMinNumCmnTrksTwoKeyFrms, m_sfmMinNumCmnTrksThreeKeyFrms, trkMarks);
        iTrksIdvKF.resize(0);
        for(TrackIndex iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
            if(trkMarks[iTrkIdv])
                iTrksIdvKF.push_back(iTrkIdv);
        }
    }
#endif

    seqs.GetSubSequences(iFrmsListKF, iTrksListIdvKF, seqsKF, iTrksCmnKF, false,
                         true);
    for(iSeq = 0; iSeq < nSeqs; ++iSeq)
        seqsKF[iSeq].NormalizeMeasurements();
//#if VERBOSE_SEQUENCE_REGISTRATION
//  printf("Total: %d / %d --> %d / %d common tracks\n", nTrksCmn, nTrksIdvTotal, iTrksCmnKF.size(), nTrksIdvTotalKF);
//#endif

    //seqsKF.AssertConsistency();
    //ViewSequences(seqsKF, 0);
}

void SequenceRegisteror::RegisterOriginalSequences(const SequenceSet &seqsKF,
        const std::vector<FrameIndexList> &iFrmsListKF,
        const std::vector<TrackIndexList> &iTrksListIdvKF, TrackIndexList &iTrksCmnKF,
        SequenceSet &seqs) {
    const TrackIndex nTrksCmnKF = seqsKF.GetCommonTracksNumber();
    seqs.SetCommonPointsNumber(seqs.GetCommonTracksNumber());
    for(TrackIndex i = 0; i < nTrksCmnKF; ++i) {
        seqs.SetCommonPoint(iTrksCmnKF[i], seqsKF.GetCommonPoint(i));
        seqs.SetCommonTrackState(iTrksCmnKF[i], seqsKF.GetCommonTrackState(i));
    }

    //Timer timer;

    SequenceIndex iSeq;
    FrameIndex iFrm;
    TrackIndex iTrk;
    std::vector<bool> frmMarks, trkMarks;
    std::vector<ushort> inliers;

    //Cestor.m_ransacErrorThreshold = FLT_MAX;
    //m_Xestor.m_ransacErrorThreshold = FLT_MAX;

    const SequenceIndex nSeqs = seqsKF.GetSequencesNumber();
    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
#if VERBOSE_SEQUENCE_REGISTRATION
        printf("----------------------------------------------------------------\n");
        printf("Sequence %d\n", iSeq);
#endif
        seqs.SetSequenceState(iSeq, seqsKF.GetSequenceState(iSeq));
        const Sequence &seqKF = seqsKF[iSeq];
        Sequence &seq = seqs[iSeq];
        seq.NormalizeMeasurements();

        if(seq.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT)
            seq.SetIntrinsicRectification(seqKF.GetIntrinsicRectification());

        const FrameIndex nFrms = seq.GetFramesNumber();
        frmMarks.assign(nFrms, false);

        const FrameIndexList &iFrmsKF = iFrmsListKF[iSeq];
        const FrameIndex nFrmsKF = FrameIndex(iFrmsKF.size());
        for(FrameIndex i = 0; i < nFrmsKF; ++i) {
            iFrm = iFrmsKF[i];
            seq.SetCamera(iFrm, seqKF.GetCamera(i));
            //seq.SetFrameState(iFrm, seqKF.GetFrameState(i));
            frmMarks[iFrm] = true;
        }

        const TrackIndex nTrks = seq.GetTracksNumber();
        trkMarks.assign(nTrks, false);

        const TrackIndexList &iTrksKF = iTrksListIdvKF[iSeq];
        const TrackIndex nTrksKF = TrackIndex(iTrksKF.size());
        for(TrackIndex i = 0; i < nTrksKF; ++i) {
            iTrk = iTrksKF[i];
            //if(iSeq == 0 && iTrk == 7929)
            //{
            //  printf("%d\n", i);
            //  seqKF.GetPoint(i).Print();
            //}
            seq.SetPoint(iTrk, seqKF.GetPoint(i));
            //seq.SetTrackState(iTrk, seqKF.GetTrackState(i));
            trkMarks[iTrk] = true;
        }

        //for(iFrm = 0; iFrm < nFrms; ++iFrm)
        //{
        //  if(!frmMarks[iFrm])
        //      seq.MarkFrameUnsolved(iFrm, 2, 0);
        //}
        //ViewSequences(seqs, iSeq);

        if(!m_sfmDensePts) {
            for(iTrk = 0; iTrk < nTrks; ++iTrk) {
                if(!trkMarks[iTrk])
                    seq.BreakTrack(iTrk);
            }
            seq.BreakOutlierTracksAndMeasurements();
            seqs.RemoveBrokenTracks(iSeq);
        }

        m_Cestor.m_ransacErrorThreshold = m_Xestor.m_ransacErrorThreshold =
                                              m_sfmReprojErrSqThs.front() * seq.GetIntrinsicMatrix().one_over_fxy();
        if(seq.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT)
            m_Cestor.m_ransacErrorThreshold /= seq.GetIntrinsicRectification().f() *
                                               seq.GetIntrinsicRectification().f();
        for(iFrm = 0; iFrm < nFrms; ++iFrm) {
#if VERBOSE_SEQUENCE_REGISTRATION
            printf("\r  Estimating cameras...%d%%", iFrm * 100 / nFrms);
#endif
            if(frmMarks[iFrm])
                continue;
            if(m_sfmDensePts)
                seq.GetCameraEstimatorDataMarkedTrack(iFrm, m_Cdata, trkMarks, true);
            else
                seq.GetCameraEstimatorData(iFrm, m_Cdata, true);
            m_Cestor.RunLosac(m_Cdata, m_C, inliers);
            seq.SetCamera(iFrm, m_C);
            //if(iSeq == 0 && iFrm == 93)
            //{
            //  const ushort j = 344;
            //  m_Cdata.X(j).Print();
            //  Point2D e;
            //  m_C.ComputeProjectionSquaredError(m_Cdata.X(j), m_Cdata.x(j), e);
            //  printf("%f\n", e.SquaredLength() * seq.GetIntrinsicMatrix().fxy() * seq.GetIntrinsicRectification().f() * seq.GetIntrinsicRectification().f());
            //}
//#if VERBOSE_SEQUENCE_REGISTRATION
//          printf("\r  Frame %d, inliers ratio = %d/%d = %d%%\n", iFrm, inliers.size(), m_Cdata.Size(), inliers.size() * 100 / m_Cdata.Size());
//#endif
        }
#if VERBOSE_SEQUENCE_REGISTRATION
        printf("\r  Estimating cameras...%d%%\n", iFrm * 100 / nFrms);
#endif

        if(m_sfmDensePts) {
            //timer.Start();
            for(iTrk = 0; iTrk < nTrks; ++iTrk) {
#if VERBOSE_SEQUENCE_REGISTRATION
                printf("\r  Estimating points...%d%%", iTrk * 100 / nTrks);
#endif
                if(trkMarks[iTrk])
                    continue;
                seq.GetPoint3DEstimatorData(iTrk, m_Xdata, true);
                if(m_Xestor.Triangulate(m_Xdata, m_Xidv))
                    seq.MarkTrackInlier(iTrk);
                else
                    seq.MarkTrackOutlier(iTrk);
                seq.SetPoint(iTrk, m_Xidv);
            }
#if VERBOSE_SEQUENCE_REGISTRATION
            printf("\r  Estimating points...%d%%\n", iTrk * 100 / nTrks);
#endif
            //timer.Stop();
        }
        ViewSequences(seqs, iSeq);

//      for(iTrk = 0; iTrk < nTrks; ++iTrk)
//      {
//#if VERBOSE_SEQUENCE_REGISTRATION
//          printf("\r  Estimating points...%d%%", iTrk * 100 / nTrks);
//#endif
//          if(trkMarks[iTrk])
//              continue;
//          seq.GetPoint3DEstimatorDataInlier(iTrk, m_Xdata);
//          m_Xestor.RunLosac(m_Xdata, m_Xidv, inliers);
//          seq.SetPoint(iTrk, m_Xidv);
//      }
//#if VERBOSE_SEQUENCE_REGISTRATION
//      printf("\r  Estimating points...%d%%\n", iTrk * 100 / nTrks);
//#endif
//      ViewSequences(seqs, iSeq);
    }

    //timer.PrintTotalTiming();
    //exit(0);
}

void SequenceRegisteror::BundleAdjustmentIndividual(SequenceSet &seqs,
        const std::vector<FrameIndexList> &iFrmsListFix, const uint &maxNumIters,
        const float &stopMSE,
        const float &stopRelativeReduction) {
    const Sequence::IntrinsicType intrinsicType = seqs.GetIntrinsicType();
    if(intrinsicType == Sequence::INTRINSIC_VARIABLE) {
        m_baIntrinsicVariable.m_lmMaxNumIters = maxNumIters;
        m_baIntrinsicVariable.m_lmStopMSE = stopMSE;
        m_baIntrinsicVariable.m_lmStopRelativeReduction = stopRelativeReduction;
    } else {
        m_ba.m_lmMaxNumIters = maxNumIters;
        m_ba.m_lmStopMSE = stopMSE;
        m_ba.m_lmStopRelativeReduction = stopRelativeReduction;
    }

    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
#if VERBOSE_SEQUENCE_REGISTRATION
        printf("----------------------------------------------------------------\n");
        printf("Sequence %d\n", iSeq);
#endif
        Sequence &seq = seqs[iSeq];
        seq.NormalizeMeasurements();
        const FrameIndex nFrms = seq.GetFramesNumber();
        m_frmMarks.assign(nFrms, false);
        const FrameIndexList &iFrmsFix = iFrmsListFix[iSeq];
        const FrameIndex nFrmsFix = FrameIndex(iFrmsFix.size());
        for(FrameIndex i = 0; i < nFrmsFix; ++i)
            m_frmMarks[iFrmsFix[i]] = true;
        m_iFrmsAdj.resize(0);
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            if(!m_frmMarks[iFrm])
                m_iFrmsAdj.push_back(iFrm);
        }
        if(!m_iFrmsAdj.empty()) {
            SelectAdjustedTracks(seq, m_iFrmsAdj, m_iTrksBA, m_iTrksIdv,
                                 m_baIdvMinNumTrksPerFrm);
            if(intrinsicType == Sequence::INTRINSIC_VARIABLE) {
                seq.GetBundleAdjustorData(m_iFrmsAdj, m_iTrksBA, m_baDataIntrinsicVariable,
                                          m_iFrmsBA);
                const FrameIndex nFrmsFixBA = FrameIndex(m_iFrmsBA.size() - m_iFrmsAdj.size());
                m_baIntrinsicVariable.Run(m_baDataIntrinsicVariable, nFrmsFixBA, false,
                                          VERBOSE_SEQUENCE_REGISTRATION ? 2 : 0);
                seq.SetBunbleAdjustmentResults(m_baDataIntrinsicVariable, nFrmsFixBA, m_iFrmsBA,
                                               m_iTrksBA);
            } else {
                seq.GetBundleAdjustorData(m_iFrmsAdj, m_iTrksBA, m_baData, m_iFrmsBA);
                if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
                    m_baData.RectifyMeasurements();
                const FrameIndex nFrmsFixBA = FrameIndex(m_iFrmsBA.size() - m_iFrmsAdj.size());
                m_ba.Run(m_baData, nFrmsFixBA, false, VERBOSE_SEQUENCE_REGISTRATION ? 2 : 0);
                seq.SetBunbleAdjustmentResults(m_baData, nFrmsFixBA, m_iFrmsBA, m_iTrksBA);
            }
            AdjustPoints(m_iTrksIdv, seq);
            m_iTrksBA.insert(m_iTrksBA.end(), m_iTrksIdv.begin(), m_iTrksIdv.end());
        }

        ViewSequences(seqs, iSeq);
    }

//#if _DEBUG
//  m_stop = true;
//  ViewSequences(seqs, 0);
//#endif

    m_cmnTrkMarks.assign(seqs.GetCommonTracksNumber(), true);
    UpdateStructureAndInlierStates(m_cmnTrkMarks, seqs, m_seqErrLevels);
}

void SequenceRegisteror::SelectAdjustedTracks(const Sequence &seq,
        const FrameIndexList &iFrmsAdj, TrackIndexList &iTrksAdj,
        TrackIndexList &iTrksUnadj,
        const FeatureIndex minNumTrksPerFrm) {
    FrameIndex iFrm;
    TrackIndex iTrk;
    MeasurementIndex iMea;
    FeatureIndex iFtr;
    //FrameIndex nCrspsSolved, nCrspsInlier;

    const FrameIndex nFrms = seq.GetFramesNumber();
    m_frmMarks.assign(nFrms, false);
    m_trkMarks.assign(seq.GetTracksNumber(), false);
    m_candidateTrks.resize(0);

    const FrameIndex nFrmsAdj = FrameIndex(iFrmsAdj.size());
    for(FrameIndex i = 0; i < nFrmsAdj; ++i) {
        iFrm = iFrmsAdj[i];
        m_frmMarks[iFrm] = true;

        const TrackIndex *iTrks = seq.GetFrameTrackIndexes(iFrm);
        const MeasurementState *meaStates = seq.GetFrameMeasurementStates(iFrm);
        const FeatureIndex nFtrs = seq.GetFrameFeaturesNumber(iFrm);
        for(iFtr = 0; iFtr < nFtrs; ++iFtr) {
            if((iTrk = iTrks[iFtr]) == INVALID_TRACK_INDEX || m_trkMarks[iTrk] ||
                    !(seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER) ||
                    (meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
                continue;
            m_trkMarks[iTrk] = true;
            //seq.CountTrackSolvedFrameInlierMeasurements(iTrk, nCrspsSolved, nCrspsInlier);
            //m_candidateTrks.push_back(CandidateTrack(iTrk, nCrspsSolved, nCrspsInlier));
            m_candidateTrks.push_back(CandidateTrack(iTrk,
                                      seq.ComputePointMinimalRayAngleDot(iTrk, m_rayDirs, true)));
        }
    }
    std::sort(m_candidateTrks.begin(), m_candidateTrks.end());

    FrameIndex cntAdjFrmsEnoughTrks = 0;
    m_frmTrkCnts.assign(nFrms, 0);
    iTrksAdj.resize(0);
    iTrksUnadj.resize(0);

    const TrackIndex nTrksCandidate = TrackIndex(m_candidateTrks.size());
    TrackIndex i;
    for(i = 0; i < nTrksCandidate && cntAdjFrmsEnoughTrks < nFrmsAdj; ++i) {
        iTrk = m_candidateTrks[i].GetTrackIndex();
        const MeasurementIndexList &iMeas = seq.GetTrackMeasurementIndexList(iTrk);
        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        FrameIndex j;
        for(j = 0; j < nCrsps; ++j) {
            iMea = iMeas[j];
            iFrm = seq.GetMeasurementFrameIndex(iMea);
            if(m_frmMarks[iFrm] &&
                    !(seq.GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER) &&
                    m_frmTrkCnts[iFrm] < minNumTrksPerFrm)
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
            if(m_frmMarks[iFrm] &&
                    !(seq.GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER) &&
                    ++m_frmTrkCnts[iFrm] == minNumTrksPerFrm)
                ++cntAdjFrmsEnoughTrks;
        }
    }
    const float dotMin = m_candidateTrks.empty() ? 1.0f :
                         m_candidateTrks.front().GetRayAngleDot();
    const float dotMax = m_candidateTrks.empty() ? 1.0f : i == nTrksCandidate ?
                         std::min(m_candidateTrks.back().GetRayAngleDot(), 1.0f) :
                         m_candidateTrks[i].GetRayAngleDot();
    for(; i < nTrksCandidate; ++i)
        iTrksUnadj.push_back(m_candidateTrks[i].GetTrackIndex());

#if VERBOSE_SEQUENCE_REGISTRATION
    printf("  Selected %d / %d adjusted tracks\n", iTrksAdj.size(),
           iTrksAdj.size() + iTrksUnadj.size());
    printf("  Ray angle: max = %f, min = %f\n", acos(dotMin) * FACTOR_RAD_TO_DEG,
           acos(dotMax) * FACTOR_RAD_TO_DEG);
#endif
}

void SequenceRegisteror::AdjustPoints(const TrackIndexList &iTrks,
                                      Sequence &seq) {
    Point3D X;
    const TrackIndex nTrks = TrackIndex(iTrks.size());
    for(TrackIndex i = 0; i < nTrks; ++i) {
        const TrackIndex iTrk = iTrks[i];
        seq.GetPoint3DEstimatorDataInlier(iTrk, m_Xdata, true);
        if(m_Xdata.Size() < 2)
            continue;
        X = seq.GetPoint(iTrk);
        m_Xestor.OptimizeModel(m_Xdata, X);
        seq.SetPoint(iTrk, X);
    }
}