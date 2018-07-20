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
#include "FeatureTracker.h"
#include "Utility/Utility.h"
#include "Utility/SparseMatrix.h"

using namespace ENFT_SfM;

FeatureTracker::FeatureTracker() {
}

FeatureTracker::~FeatureTracker() {
}

void FeatureTracker::Initialize(const Sequence &seq,
                                const std::string paramFileName) {
    Configurator param;
    param.Load(paramFileName.c_str());
    Initialize(seq, param);
}

void FeatureTracker::Initialize(const Sequence &seq, const Configurator &param) {
    m_imgWidth = seq.GetImageWidth(), m_imgHeight = seq.GetImageHeight();
    ProgramGL::Initialize(m_imgWidth, m_imgHeight);

    m_bruteForceMatching = param.GetArgument("brute_force_matching", 0) != 0;
    m_repeated = param.GetArgument("repeated", 0) != 0;
    m_minNumMatchesKeyToLast = ushort( param.GetArgument("min_matches_number_key_to_last_frame", 300));

    const ushort ftrTexWidth = ushort(param.GetArgument("feature_texture_width", 64));
    m_ftrMinDist = ushort(param.GetArgument("feature_min_distance", 10));
    m_bufferSize = ushort(param.GetArgument("feature_buffer_size", 50));
    m_ftrsBuffer.resize(m_bufferSize);
    m_descsBuffer.resize(m_bufferSize);
    m_FBuffer.Resize(m_bufferSize);
    m_matchesBuffer.resize(m_bufferSize);
    m_matchesBufferEnft.resize(m_bufferSize);
    m_iBufferTmp1 = m_bufferSize;
    m_iBufferTmp2 = m_bufferSize + 1;
    const ushort bufferSizeExtended = m_bufferSize + 2;

    errThEp = param.GetArgument("error_threshold_epipolar", 2.0f);
    errThHomo = param.GetArgument("error_threshold_homography", 2.0f);
    m_Festor.m_ransacErrorThreshold = errThEp * errThEp;
    m_Hestor.m_ransacErrorThreshold = errThHomo * errThHomo;

    m_epInliersMinNum = ushort(param.GetArgument("epipolar_min_inliers_number",
                               20));
    const ushort nBinsX = ushort(param.GetArgument("arsac_bin_size_x", 10));
    const ushort nBinsY = ushort(param.GetArgument("arsac_bin_size_y", 10));
    m_Festor.SetBinSize(nBinsX, nBinsY);
    m_Festor.SetImageSize(m_imgWidth, m_imgHeight);
    m_Fdata.SetImageSize(m_imgWidth, m_imgHeight);
    m_Festor.m_ransacMinNumIters = uint(
                                       param.GetArgument("ransac_min_iterations_number_epipolar", 100));
    m_Festor.m_ransacMaxNumIters = uint(
                                       param.GetArgument("ransac_max_iterations_number_epipolar", 300));
    m_Hestor.m_ransacMinNumIters = uint(
                                       param.GetArgument("ransac_min_iterations_number_homography", 100));
    m_Hestor.m_ransacMaxNumIters = uint(
                                       param.GetArgument("ransac_max_iterations_number_homography", 300));

    m_texRGB.Generate(m_imgWidth, m_imgHeight);
    m_texGray.Generate(m_imgWidth, m_imgHeight);
    m_programConvertRGB2Gray.Initialize();
    m_siftMaxNumFtrs = ushort(param.GetArgument("sift_max_features_number", 2048));
    const ushort nOctaves = ushort(param.GetArgument("sift_octaves_number", 3));
    const ushort nLevelsDoG = ushort(param.GetArgument("sift_dog_levels_number",
                                     3));
    const bool upSample = param.GetArgument("sift_upsample_image", 0) != 0;
    const float dogTh = param.GetArgument("sift_dog_threshold", 0.02f);
    const float edgeTh = param.GetArgument("sift_edge_threshold", 10.0f);
    const float hessianTh = param.GetArgument("surf_hessian_threshold", 100.0f);
    const ushort searchRangeKeyToCurrent = m_bruteForceMatching ? USHRT_MAX :
                                           ushort(param.GetArgument("sift_matching_search_range_key_to_current_frame",
                                                   USHRT_MAX));
    const ushort searchRangeLastToCurrent = m_bruteForceMatching ? USHRT_MAX :
                                            ushort(param.GetArgument("sift_matching_search_range_last_to_current_frame",
                                                    USHRT_MAX));
    const float descDotTh =
        param.GetArgument("sift_matching_descriptor_dot_product_threshold", 0.95f);
    const float nearest1To2RatioTh =
        param.GetArgument("sift_matching_descriptor_nearest_first_to_second_ratio_threshold", 0.7f);
    // (key frame --> current frame) + (last frame --> current frame)
    m_enftMaxNumFtrs = m_siftMaxNumFtrs * 2;
    m_ftrExtractorSift.Initialize(m_imgWidth, m_imgHeight, bufferSizeExtended,
                                  m_siftMaxNumFtrs, m_enftMaxNumFtrs, ftrTexWidth, nOctaves, nLevelsDoG, upSample,
                                  dogTh, edgeTh, hessianTh, m_ftrMinDist);

    const ushort winSz = ushort(param.GetArgument("enft_patch_window_size", 9));
    const ushort nIters = ushort(param.GetArgument("enft_max_iterations_number",
                                 20));
    const float lambdaEp = param.GetArgument("enft_weight_epipolar", 50.0f);
    const float lambdaHomo = param.GetArgument("enft_weight_homography", 50.0f);
    const float deltaTh = param.GetArgument("enft_offset_threshold", 0.01f);
    const float SADTh = param.GetArgument("enft_intensity_error_threshold", 0.01f) * winSz * winSz;
    m_maxNumFtrs = m_siftMaxNumFtrs + m_enftMaxNumFtrs;
    m_enftSmoothLevel = ushort(param.GetArgument("enft_smooth_level", 1));
    m_enftMaxNumPlanes = ushort(param.GetArgument("enft_max_planes_number", 3));
    m_enftMinNumPtsPerPlane = ushort(
                                  param.GetArgument("enft_min_points_number_per_plane", 20));
    m_ftrMatcherSift.Initialize(m_maxNumFtrs, m_maxNumFtrs, ftrTexWidth, errThEp,
                                nearest1To2RatioTh, descDotTh,
                                searchRangeKeyToCurrent, searchRangeLastToCurrent);
    m_ftrTrackerEnft.Initialize(m_imgWidth, m_imgHeight, m_bufferSize, m_maxNumFtrs,
                                ftrTexWidth, winSz, nIters, lambdaEp, lambdaHomo,
                                deltaTh, SADTh, errThEp, errThHomo);

    m_descDistTh = 2 - 2 * descDotTh;
    m_nearest1To2RatioTh = nearest1To2RatioTh * nearest1To2RatioTh;

    m_flowDifferenceTh = param.GetArgument("flow_difference_threshold", 50.0f);
    m_flowPositiveVotingTh = ushort(
                                 param.GetArgument("flow_positive_voting_threshold", 10));
    m_flowPositiveVotingRatioTh =
        param.GetArgument("flow_positive_voting_ratio_threshold", 0.5f);
    m_bucket.Initialize(m_imgWidth, m_imgHeight,
                        ushort(param.GetArgument("flow_voting_bin_size_x", 10)),
                        ushort(param.GetArgument("flow_voting_bin_size_y", 10)));

    m_bfPositiveVotingTh = ushort(param.GetArgument("bf_positive_voting_threshold", 5));
    m_bfPositiveVotingRatioTh =
        param.GetArgument("bf_positive_voting_ratio_threshold", 0.8f);

    m_removeSingleTrk = param.GetArgument("remove_single_track", 1) != 0;

    m_stop = param.GetArgument("stop", 0) != 0;
    m_view = param.GetArgument("view", 1) != 0;
}

void FeatureTracker::Run(Sequence &seq, const std::string outputFileName,
                         FILE *fpTiming) {
    if(outputFileName != "" && seq.LoadBwithDir(outputFileName.c_str()))
        return;

    const MeasurementIndex nMeasurementMax = MeasurementIndex(
                m_siftMaxNumFtrs*seq.GetFramesNumberTotal());
    seq.ReserveMeasurements(nMeasurementMax);
#if DESCRIPTOR_TRACK == 0
    seq.ReserveDescriptors(nMeasurementMax);
#endif

    if(m_bruteForceMatching)
        RunBruteForceMatching(seq, fpTiming);
    else
        RunSequentialTracking(seq, fpTiming);

    if(m_removeSingleTrk) {
        const TrackIndex nTrks = seq.GetTracksNumber();
        for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
            if(seq.GetTrackLength(iTrk) == 1)
                seq.BreakTrack(iTrk);
        }
    }
    seq.RemoveBrokenTracks();
    seq.RemoveNullMeasurements();

    if(outputFileName != "")
        seq.SaveBwithDir(outputFileName.c_str());
}

template<typename TYPE>
static inline void LoadImage(CVD::Image<TYPE> &img1, CVD::Image<TYPE> &img2,
                             const std::string &fileName) {
    CVD::img_load(img1, fileName);
    if(img1.size().x % 4 == 0)
        img2 = img1;
    else {
        const int width = (img1.size().x + 3) & (~3), height = img1.size().y;
        img2.resize(CVD::ImageRef(width, height));
        img2.zero();
        for(int y = 0; y < height; ++y)
            memcpy(img2[y], img1[y], sizeof(TYPE) * img1.row_stride());
    }
}

void FeatureTracker::RunSequentialTracking(Sequence &seq, FILE *fpTiming) {
    Timer timer;
    //timer.Start();

    const bool stopBkp = m_stop;

    const FrameIndex nFrms = seq.GetFramesNumberTotal();
    FrameIndex iFrmKey = 0, iFrmLast = 0, iFrmCurrent = 1;

    if (m_minNumMatchesKeyToLast == 0)
        m_bufferManager.Initialize(nFrms, 2);
    else if (m_bufferSize >= 3)
        m_bufferManager.Initialize(nFrms, m_bufferSize);
    else
        m_bufferManager.Initialize(nFrms, 3);

    ::LoadImage(m_imgRGBTmp, m_imgRGB, seq.GetImageFileName(iFrmKey));
    timer.Start();
    ExtractFeatures(iFrmKey);
    PushBackFrame(iFrmKey, seq);//push back the information of iFrmKey into seq
    //include m_xs m_frmStates m_meaState m_mapFrmToMea m_mapMeaToFrm m_mapTrkToMea m_mapMeaToTrk m_trkStates
    seq.MarkFrameKeyFrame(iFrmKey);
    timer.Stop();
    ViewSequence(seq, iFrmKey);//display image

    ::LoadImage(m_imgRGBTmp, m_imgRGB, seq.GetImageFileName(iFrmCurrent));
    timer.Start();
    ExtractFeatures(iFrmCurrent);
    TrackFeatures2(iFrmKey, iFrmCurrent);//match 2 frames
    PushBackFrameAndFeatureMatches2(iFrmKey, iFrmCurrent, seq);
    iFrmLast = iFrmCurrent;
    ++iFrmCurrent;
    timer.Stop();
    ViewSequence(seq, iFrmLast);

    FeatureIndexList iFtrsKeyToLast, iFtrsLastToKey;
    while (iFrmCurrent < nFrms) {
        ::LoadImage(m_imgRGBTmp, m_imgRGB, seq.GetImageFileName(iFrmCurrent));
        timer.Start();
        if (!m_bufferManager.IsDataBuffered(iFrmCurrent))
            m_bufferManager.BufferData(iFrmCurrent);
        ExtractFeatures(iFrmCurrent);
        if (m_minNumMatchesKeyToLast == 0) {
            TrackFeatures2(iFrmLast, iFrmCurrent);
            PushBackFrameAndFeatureMatches2(iFrmLast, iFrmCurrent, seq);
        } else {
            //remark KeyFrame
            seq.UnmarkFrameKeyFrame(iFrmKey);
            for (iFrmKey = iFrmLast - 1; iFrmKey>0 &&
                    CountFeatureMatchesKeyFrameToLastFrame(seq, iFrmKey - 1, iFrmLast) >
                    m_minNumMatchesKeyToLast; --iFrmKey);
            seq.MarkFrameKeyFrame(iFrmKey);
            if (!m_bufferManager.IsDataBuffered(iFrmKey)) {
                m_bufferManager.BufferData(iFrmKey);
                SetFeatureBuffer(seq, iFrmKey);
            }
            GetFeatureMatchesKeyFrameToLastFrame(seq, iFrmKey, iFrmLast, m_iFtrsKeyToLast, m_iFtrsLastToKey);
            TrackFeatures3(iFrmKey, iFrmLast, iFrmCurrent, m_iFtrsKeyToLast, m_iFtrsLastToKey);
            PushBackFrameAndFeatureMatches3(iFrmKey, iFrmLast, iFrmCurrent, seq);
        }
        timer.Stop();
        ViewSequence(seq, iFrmCurrent);
        iFrmLast = iFrmCurrent++;
    }
    seq.MarkFrameKeyFrame(iFrmLast);

#if VERBOSE_FEATURE_TRACKING
    printf("****************************************************************\n");
#endif
    seq.PrintStates();

    //timer.Stop();
    timer.PrintTotalAndStableMeanTiming(0, fpTiming);
    extractTimer.PrintTotalTiming();
    matchTimer.PrintTotalTiming();

    m_stop = stopBkp;
    ViewSequence(seq, iFrmLast);
}

void FeatureTracker::RunBruteForceMatching(Sequence &seq, FILE *fpTiming) {
    Timer timer(2);

    const bool stopBkp = m_stop;

    AlignedVector<Point2D> &ftrs = m_ftrsBuffer[0];
    AlignedVector<Descriptor> &descs = m_descsBuffer[0];
    const FrameIndex nFrms = seq.GetFramesNumberTotal();

    m_bufferManager.Initialize(nFrms, 1);

    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        ::LoadImage(m_imgRGBTmp, m_imgRGB, seq.GetImageFileName(iFrm));

        timer.Start(0);
        if(!m_bufferManager.IsDataBuffered(iFrm))
            m_bufferManager.BufferData(iFrm);
        const ushort iBuffer = m_bufferManager.GetDataBufferIndex(iFrm);
        m_texRGB.Bind();
        m_texRGB.UploadFromCPU((ubyte *) m_imgRGB.data());
        ProgramGL::FitViewportGL(m_texGray);
        m_programConvertRGB2Gray.Run(m_texRGB, m_texGray);
        m_ftrExtractorSift.ExtractFeatures(m_texGray, 0);
        m_ftrExtractorSift.DownloadFeaturesToCPU(0, ftrs);
        m_ftrExtractorSift.DownloadDescriptorsToCPU(0, descs);
        const FeatureIndex nFtrs = FeatureIndex(ftrs.Size());
        seq.PushBackFrame(ftrs.Data(), descs.Data(), nFtrs, nFtrs, false);
        PushBackTrackColors(iFrm, seq);
        timer.Stop(0);
#if VERBOSE_FEATURE_TRACKING
        printf("\rFrame %d: %d features", iFrm, nFtrs);
#endif
        ViewSequence(seq, iFrm);
    }
#if VERBOSE_FEATURE_TRACKING
    printf("\n");
#endif

    seq.MarkFrameKeyFrame(0);
    seq.MarkFrameKeyFrame(nFrms - 1);

    m_bufferManager.Initialize(nFrms, 2);
    m_pSeq = &seq;

    timer.Start(1);
    FrameIndex iFrm1, iFrm2, iFrm1F, iFrm2F;
    FeatureIndex iFtr1, iFtr2;
    TrackIndex iTrk1, iTrk2;
    MeasurementIndex iMea1F, iMea2F;
    ushort cntPositive, cntNegative, cntInliers;
    FeatureMatchList matches;
    std::vector<FeatureEnftMatch> matchesEnft;
    std::vector<bool> marks;
    FundamentalMatrix F, *pF, **ppF;
    SparseMatrix<FundamentalMatrix *> FTable(nFrms, nFrms);
    std::vector<FundamentalMatrix *> FPool;
    float Ferr, work[6];
    for(iFrm2 = 0; iFrm2 < nFrms; ++iFrm2) {
        if(!m_bufferManager.IsDataBuffered(iFrm2))
            m_bufferManager.BufferData(iFrm2);
        SetFeatureBuffer(seq, iFrm2);
        const TrackIndex *iTrks2 = seq.GetFrameTrackIndexes(iFrm2);
        for(iFrm1 = 0; iFrm1 < iFrm2; ++iFrm1) {
            if(!m_bufferManager.IsDataBuffered(iFrm1))
                m_bufferManager.BufferData(iFrm1);
            SetFeatureBuffer(seq, iFrm1);

            MatchFeaturesSift(iFrm1, iFrm2, F, matches, true);
            if(ushort(matches.size()) < m_epInliersMinNum) {
#if VERBOSE_FEATURE_TRACKING
                printf("\rFrame (%d, %d): NO matches!                              ", iFrm1,
                       iFrm2);
#endif
                continue;
            }
            const TrackIndex *iTrks1 = seq.GetFrameTrackIndexes(iFrm1);
            const ushort nMatches = ushort(matches.size());
            pF = (FundamentalMatrix *) _aligned_malloc(sizeof(FundamentalMatrix),
                    SSE_ALIGNMENT);
            *pF = F;
            FPool.push_back(pF);
            FTable.Insert(iFrm1, iFrm2, FPool.back());
            cntInliers = 0;
            for(ushort i = 0; i < nMatches; ++i) {
                iFtr1 = matches[i].GetIndex1();
                iTrk1 = iTrks1[iFtr1];
                iFtr2 = matches[i].GetIndex2();
                iTrk2 = iTrks2[iFtr2];
                if(iTrk1 == iTrk2 || seq.AreTracksOverlappingInFrames(iTrk1, iTrk2, marks))
                    continue;
                cntPositive = cntNegative = 0;
                const MeasurementIndexList &iMeas1 = seq.GetTrackMeasurementIndexList(iTrk1),
                                            &iMeas2 = seq.GetTrackMeasurementIndexList(iTrk2);
                const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()),
                                 nCrsps2 = FrameIndex(iMeas2.size());
                for(FrameIndex i1 = 0; i1 < nCrsps1; ++i1) {
                    iMea1F = iMeas1[i1];
                    iFrm1F = seq.GetMeasurementFrameIndex(iMea1F);
                    const Point2D &x1F = seq.GetMeasurement(iMea1F);
                    for(FrameIndex i2 = 0; i2 < nCrsps2; ++i2) {
                        iMea2F = iMeas2[i2];
                        iFrm2F = seq.GetMeasurementFrameIndex(iMea2F);
                        const Point2D &x2F = seq.GetMeasurement(iMea2F);
                        if(iFrm1F < iFrm2F && (ppF = FTable.Get(iFrm1F, iFrm2F)) != NULL)
                            Ferr = (*ppF)->ComputeSymmetricSquaredError(x1F, x2F, work);
                        else if(iFrm2F < iFrm1F && (ppF = FTable.Get(iFrm2F, iFrm1F)) != NULL)
                            Ferr = (*ppF)->ComputeSymmetricSquaredError(x2F, x1F, work);
                        else
                            continue;
                        if(Ferr < m_Festor.m_ransacErrorThreshold)
                            ++cntPositive;
                        else
                            ++cntNegative;
                    }
                }
                if(cntPositive < m_bfPositiveVotingTh &&cntPositive < ushort(
                            (cntPositive + cntNegative) * m_bfPositiveVotingRatioTh +0.5f))
                    continue;
                seq.MatchTracks(iTrk1, iTrk2);
                ++cntInliers;
            }

#if VERBOSE_FEATURE_TRACKING
            printf("\rFrame (%d, %d): %d - %d = %d matches      ", iFrm1, iFrm2,
                   matches.size(), matches.size() - cntInliers, cntInliers);
#endif
            seq.MarkFrameKeyFrame(iFrm1);
            ViewSequence(seq, iFrm2);
            seq.UnmarkFrameKeyFrame(iFrm1);
        }
    }
#if VERBOSE_FEATURE_TRACKING
    printf("\n");
#endif
    const uint nFs = uint(FPool.size());
    for(uint i = 0; i < nFs; ++i)
        _aligned_free(FPool[i]);
    timer.Stop(1);

    seq.PrintStates();

    if(fpTiming) {
        fprintf(fpTiming, "Feature extraction: ");
        timer.PrintTotalAndStableMeanTiming(0, fpTiming);
        fprintf(fpTiming, "Feature matching:   ");
        timer.PrintTotalAndStableMeanTiming(1, fpTiming);
    } else {
        printf("Feature extraction: ");
        timer.PrintTotalAndStableMeanTiming(0);
        printf("Feature matching:   ");
        timer.PrintTotalAndStableMeanTiming(1);
    }

    m_stop = stopBkp;
    ViewSequence(seq, nFrms - 1);
}

void FeatureTracker::PushBackFrame(const FrameIndex &iFrm, Sequence &seq) {
    const ushort iBuffer = m_bufferManager.GetDataBufferIndex(iFrm);
    const ushort nFtrsSift = m_ftrExtractorSift.GetFeaturesNumber(iBuffer),
                 nFtrsTotal = ushort(m_ftrsBuffer[iBuffer].Size());
    seq.PushBackFrame(m_ftrsBuffer[iBuffer].Data(), m_descsBuffer[iBuffer].Data(),
                      nFtrsTotal, nFtrsSift, false);
    PushBackTrackColors(iFrm, seq);
}

void FeatureTracker::PushBackFrameAndFeatureMatches2(const FrameIndex &iFrm1,
        const FrameIndex &iFrm2, Sequence &seq) {
    const ushort iBuffer1 = m_bufferManager.GetDataBufferIndex(iFrm1),
                 iBuffer2 = m_bufferManager.GetDataBufferIndex(iFrm2);
    const std::vector<FeatureEnftMatch> &matchesEnft =
        m_matchesBufferEnft[iBuffer1];
    const AlignedVector<Descriptor> &descs1 = m_descsBuffer[iBuffer1];

    FeatureMatchList &matches = m_matchesBuffer[iBuffer1];
    AlignedVector<Point2D> &ftrs2 = m_ftrsBuffer[iBuffer2];
    AlignedVector<Descriptor> &descs2 = m_descsBuffer[iBuffer2];
    const ushort nFtrsSift = ushort(ftrs2.Size()),
                 nFtrsEnft = ushort(matchesEnft.size()), nFtrsTotal = nFtrsSift + nFtrsEnft;
    ftrs2.EnlargeCapacity(nFtrsTotal);
    descs2.EnlargeCapacity(nFtrsTotal);

    FeatureIndex iFtr1, iFtr2 = nFtrsSift;
    const ushort nMatchesEnft = ushort(matchesEnft.size());
    for(ushort i = 0; i < nMatchesEnft; ++i, ++iFtr2) {
        iFtr1 = matchesEnft[i].GetFeatureIndex1();
        matches.push_back(FeatureMatch(iFtr1, iFtr2));
        ftrs2.PushBack(matchesEnft[i].GetFeature2());
        descs2.PushBack(descs1[iFtr1]);
    }
    m_ftrExtractorSift.PushBackFeaturesAndDescriptorsFromCPU(iBuffer2, nFtrsEnft,
            ftrs2.Data() + nFtrsSift, descs2.Data() + nFtrsSift);
    seq.PushBackFrame(ftrs2.Data(), descs2.Data(), nFtrsTotal, nFtrsSift, false);
    seq.MatchFrameFeatures(iFrm1, iFrm2, matches, m_marks1);
    PushBackTrackColors(iFrm2, seq);
}

void FeatureTracker::PushBackFrameAndFeatureMatches3(const FrameIndex &iFrmKey,
        const FrameIndex &iFrmLast, const FrameIndex &iFrmCurrent, Sequence &seq) {
    const ushort iBufferKey = m_bufferManager.GetDataBufferIndex(iFrmKey);
    const ushort iBufferLast = m_bufferManager.GetDataBufferIndex(iFrmLast);
    const ushort iBufferCurrent = m_bufferManager.GetDataBufferIndex(iFrmCurrent);
    const std::vector<FeatureEnftMatch> &matchesKeyToCurrentEnft = m_matchesBufferEnft[iBufferKey],
                                         &matchesLastToCurrentEnft = m_matchesBufferEnft[iBufferLast];
    const AlignedVector<Descriptor> &descsKey = m_descsBuffer[iBufferKey],
                                     &descsLast = m_descsBuffer[iBufferLast];

    FeatureMatchList &matchesKeyToCurrent = m_matchesBuffer[iBufferKey],
                      &matchesLastToCurrent = m_matchesBuffer[iBufferLast];
    AlignedVector<Point2D> &ftrsCurrent = m_ftrsBuffer[iBufferCurrent];
    AlignedVector<Descriptor> &descsCurrent = m_descsBuffer[iBufferCurrent];
    const ushort nFtrsSift = ushort(ftrsCurrent.Size()),
                 nFtrsEnft = ushort(matchesKeyToCurrentEnft.size() +matchesLastToCurrentEnft.size()),
                 nFtrsTotal = nFtrsSift + nFtrsEnft;
    ftrsCurrent.EnlargeCapacity(nFtrsTotal);
    descsCurrent.EnlargeCapacity(nFtrsTotal);

    FeatureIndex iFtrKey, iFtrLast, iFtrCurrent = nFtrsSift;
    const ushort nMatchesKeyToCurrentEnft = ushort(matchesKeyToCurrentEnft.size());
    for(ushort i = 0; i < nMatchesKeyToCurrentEnft; ++i, ++iFtrCurrent) {
        iFtrKey = matchesKeyToCurrentEnft[i].GetFeatureIndex1();
        matchesKeyToCurrent.push_back(FeatureMatch(iFtrKey, iFtrCurrent));
        ftrsCurrent.PushBack(matchesKeyToCurrentEnft[i].GetFeature2());
        descsCurrent.PushBack(descsKey[iFtrKey]);
    }
    const ushort nMatchesLastToCurrentEnft = ushort(matchesLastToCurrentEnft.size());
    for(ushort i = 0; i < nMatchesLastToCurrentEnft; ++i, ++iFtrCurrent) {
        iFtrLast = matchesLastToCurrentEnft[i].GetFeatureIndex1();
        matchesLastToCurrent.push_back(FeatureMatch(iFtrLast, iFtrCurrent));
        ftrsCurrent.PushBack(matchesLastToCurrentEnft[i].GetFeature2());
        descsCurrent.PushBack(descsLast[iFtrLast]);
    }
    m_ftrExtractorSift.PushBackFeaturesAndDescriptorsFromCPU(iBufferCurrent,
            nFtrsEnft, ftrsCurrent.Data() + nFtrsSift, descsCurrent.Data() + nFtrsSift);

    seq.PushBackFrame(ftrsCurrent.Data(), descsCurrent.Data(), nFtrsTotal, nFtrsSift, false);
    seq.MatchFrameFeatures(iFrmKey, iFrmCurrent, matchesKeyToCurrent, m_marks1);
    seq.MatchFrameFeatures(iFrmLast, iFrmCurrent, matchesLastToCurrent, m_marks1);
    PushBackTrackColors(iFrmCurrent, seq);
}

void FeatureTracker::SetFeatureBuffer(const Sequence &seq,
                                      const FrameIndex &iFrm) {
    const ushort iBuffer = m_bufferManager.GetDataBufferIndex(iFrm);
    seq.GetFrameFeaturesAndDescriptors(iFrm, m_ftrsBuffer[iBuffer], m_descsBuffer[iBuffer]);
    m_ftrExtractorSift.UploadFeaturesAndDescriptorsFromCPU(iBuffer, m_ftrsBuffer[iBuffer], m_descsBuffer[iBuffer]);
}

ushort FeatureTracker::CountFeatureMatchesKeyFrameToLastFrame(
    const Sequence &seq, const FrameIndex &iFrmKey, const FrameIndex &iFrmLast) {
#if _DEBUG
    assert(iFrmLast == seq.GetFramesNumber() - 1);
#endif
    const MeasurementIndex iMeaStartKey = seq.GetFrameFirstMeasurementIndex(iFrmKey);
    const TrackIndex *iTrksKey = seq.GetMeasurementTrackMap().data() + iMeaStartKey;
    const FeatureIndex nFtrsKey = seq.GetFrameFirstMeasurementIndex(
                                      iFrmKey + 1) - iMeaStartKey;

    FrameIndex iFrm;
    TrackIndex iTrk;
    ushort cnt = 0;
    for(FeatureIndex iFtrKey = 0; iFtrKey < nFtrsKey; ++iFtrKey) {
        if((iTrk = iTrksKey[iFtrKey]) != INVALID_TRACK_INDEX &&
                (iFrm = seq.GetMeasurementFrameIndex(seq.GetTrackMeasurementIndexList(
                            iTrk).back())) == iFrmLast)
            ++cnt;
    }
    return cnt;
}

void FeatureTracker::GetFeatureMatchesKeyFrameToLastFrame(const Sequence &seq,
        const FrameIndex &iFrmKey, const FrameIndex &iFrmLast,
        FeatureIndexList &iFtrsKeyToLast, FeatureIndexList &iFtrsLastToKey) {
#if _DEBUG
    assert(iFrmLast == seq.GetFramesNumber() - 1);
#endif
    const MeasurementIndex iMeaStartKey = seq.GetFrameFirstMeasurementIndex(
            iFrmKey), iMeaStartLast = seq.GetFrameFirstMeasurementIndex(iFrmLast);
    const TrackIndex *iTrksKey = seq.GetMeasurementTrackMap().data() + iMeaStartKey;
    const FeatureIndex nFtrsKey = seq.GetFrameFirstMeasurementIndex(
                                      iFrmKey + 1) - iMeaStartKey;
    const FeatureIndex nFtrsLast = seq.GetFrameFirstMeasurementIndex(
                                       iFrmLast + 1) - iMeaStartLast;

    FrameIndex iFrm;
    TrackIndex iTrk;
    MeasurementIndex iMea;
    FeatureIndex iFtrKey, iFtrLast;
    iFtrsKeyToLast.assign(nFtrsKey, INVALID_FEATURE_INDEX);
    iFtrsLastToKey.assign(nFtrsLast, INVALID_FEATURE_INDEX);
    for(iFtrKey = 0; iFtrKey < nFtrsKey; ++iFtrKey) {
        if((iTrk = iTrksKey[iFtrKey]) == INVALID_TRACK_INDEX)
            continue;
        iMea = seq.GetTrackMeasurementIndexList(iTrk).back();
        if((iFrm = seq.GetMeasurementFrameIndex(iMea)) == iFrmLast) {
            iFtrLast = FeatureIndex(iMea - iMeaStartLast);
            iFtrsKeyToLast[iFtrKey] = iFtrLast;
            iFtrsLastToKey[iFtrLast] = iFtrKey;
        }
    }
}

void FeatureTracker::ExtractFeatures(const FrameIndex &iFrm) {

    const ushort iBuffer = m_bufferManager.GetDataBufferIndex(iFrm);
    m_texRGB.Bind();
    m_texRGB.UploadFromCPU((ubyte *)m_imgRGB.data());
    ProgramGL::FitViewportGL(m_texGray);
    m_programConvertRGB2Gray.Run(m_texRGB, m_texGray);
    m_ftrExtractorSift.ExtractFeatures(m_texGray, iBuffer);
    m_ftrExtractorSift.DownloadFeaturesToCPU(iBuffer, m_ftrsBuffer[iBuffer]);
    m_ftrExtractorSift.DownloadDescriptorsToCPU(iBuffer, m_descsBuffer[iBuffer]);
    m_ftrTrackerEnft.SetImage(iBuffer, m_ftrExtractorSift.GetGaussianTexture(0, m_enftSmoothLevel));

#if VERBOSE_FEATURE_TRACKING
    printf("****************************************************************\n");
    printf("Frame %d\n", iFrm);
    printf("----------------------------------------------------------------\n");
    printf("  SIFT features: %d\n", m_ftrsBuffer[iBuffer].Size());
#endif
}

void FeatureTracker::TrackFeatures2(const FrameIndex &iFrm1, const FrameIndex &iFrm2) {
#if VERBOSE_FEATURE_TRACKING >= 2
    printf("----------------------------------------------------------------\n");
    printf("  Frame (%d, %d)\n", iFrm1, iFrm2);
#endif
    const ushort iBuffer1 = m_bufferManager.GetDataBufferIndex(iFrm1),
                 iBuffer2 = m_bufferManager.GetDataBufferIndex(iFrm2);
    FundamentalMatrix &F = m_FBuffer[iBuffer1];
    FeatureMatchList &matches = m_matchesBuffer[iBuffer1];
    std::vector<FeatureEnftMatch> &matchesEnft = m_matchesBufferEnft[iBuffer1];

    MatchFeaturesSift(iFrm1, iFrm2, F, matches, false);//get match & F
    m_matchesEnft1.resize(0);
    TrackFeaturesEnft(iFrm1, iFrm2, F, matches, m_matchesEnft1, matchesEnft);//ENFT
    RemoveOutlierMatchesFlowVoting(iFrm1, iFrm2, matches, matchesEnft);
    NonMaximalSuppressEnft(iFrm1, iFrm2, matchesEnft);

#if VERBOSE_FEATURE_TRACKING
    printf("----------------------------------------------------------------\n");
    printf("  SIFT matches: %d\n", matches.size());
    printf("  ENFT matches: %d\n", matchesEnft.size());
#endif
}

template<class Match>
static inline void PushBackMatches(const std::vector<Match> &matchesSrc,
                                   std::vector<Match> &matchesDst) {
    matchesDst.insert(matchesDst.end(), matchesSrc.begin(), matchesSrc.end());
}

void FeatureTracker::TrackFeatures3(const FrameIndex &iFrmKey,
                                    const FrameIndex &iFrmLast, const FrameIndex &iFrmCurrent,
                                    const FeatureIndexList &iFtrsKeyToLast,
                                    const FeatureIndexList
                                    &iFtrsLastToKey/*, const FundamentalMatrix &FKeyToLast*/) {
    const ushort iBufferKey = m_bufferManager.GetDataBufferIndex(iFrmKey);
    const ushort iBufferLast = m_bufferManager.GetDataBufferIndex(iFrmLast);
    const ushort iBufferCurrent = m_bufferManager.GetDataBufferIndex(iFrmCurrent);

#if VERBOSE_FEATURE_TRACKING >= 2
    printf("----------------------------------------------------------------\n");
    printf("  Frame (%d, %d)\n", iFrmLast, iFrmCurrent);
#endif
    FundamentalMatrix &FLastToCurrent = m_FBuffer[iBufferLast];
    FeatureMatchList &matchesLastToCurrent = m_matchesBuffer[iBufferLast];
    MatchFeaturesSift(iFrmLast, iFrmCurrent, FLastToCurrent, matchesLastToCurrent, false);

#if VERBOSE_FEATURE_TRACKING >= 2
    printf("  Frame (%d, %d)\n", iFrmKey, iFrmCurrent);
#endif
    m_ftrMatcherSift.FromMatches13To23(iFtrsLastToKey, matchesLastToCurrent, m_matches1); //LastToKey + LastToCurrent = KeyToCurrent

    FundamentalMatrix &FKeyToCurrent = m_FBuffer[iBufferKey];
    FeatureMatchList &matchesKeyToCurrent = m_matchesBuffer[iBufferKey];
    RemoveOutlierMatchesSift_MatchNewFeaturesSift(iFrmKey, iFrmCurrent, FKeyToCurrent, m_matches1, matchesKeyToCurrent, true);

#if VERBOSE_FEATURE_TRACKING >= 2
    printf("  Frame (%d, %d)\n", iFrmLast, iFrmCurrent);
#endif
    m_ftrMatcherSift.FromMatches13To23(iFtrsKeyToLast, matchesKeyToCurrent, m_matches2); // m_matches2 is LastToCurrent
    RemoveOutlierMatchesSift(iFrmLast, iFrmCurrent, FLastToCurrent, m_matches2);
    RemoveConflictedMatchesSift(iFrmLast, iFrmCurrent, matchesLastToCurrent, m_matches2);
    RemoveConflictedMatchesSift(iFrmKey, iFrmCurrent, matchesKeyToCurrent, m_matches1);

    const ushort nMatchesSiftExclusiveKeyToCurrent = ushort(
                matchesKeyToCurrent.size()),
            nMatchesSiftExclusiveLastToCurrent = ushort(matchesLastToCurrent.size());
    m_ftrMatcherSift.FromMatches13To23(iFtrsKeyToLast, m_matches1);
    PushBackMatches(m_matches1, m_matches2);
    PushBackMatches(m_matches2, matchesLastToCurrent);
    const ushort nMatchesSiftCommon = ushort(m_matches2.size());

    std::vector<FeatureEnftMatch> &matchesLastToCurrentEnft =
        m_matchesBufferEnft[iBufferLast];
    m_matchesEnft1.resize(0);
    TrackFeaturesEnft(iFrmLast, iFrmCurrent, FLastToCurrent, matchesLastToCurrent,
                      m_matchesEnft1, matchesLastToCurrentEnft);//unknow

#if VERBOSE_FEATURE_TRACKING >= 2
    printf("  Frame (%d, %d)\n", iFrmKey, iFrmCurrent);
#endif
    m_ftrTrackerEnft.FromMatches13To23(iFtrsLastToKey, matchesLastToCurrentEnft,
                                       m_matchesEnft1);

    std::vector<FeatureEnftMatch> &matchesKeyToCurrentEnft =
        m_matchesBufferEnft[iBufferKey];
    RemoveOutlierMatchesEnft(iFrmKey, iFrmCurrent, FKeyToCurrent, m_matchesEnft1);
    m_ftrMatcherSift.FromMatches13To23(iFtrsLastToKey, m_matches2);
    PushBackMatches(m_matches2, matchesKeyToCurrent);
    TrackFeaturesEnft(iFrmKey, iFrmCurrent, FKeyToCurrent, matchesKeyToCurrent,
                      m_matchesEnft1, matchesKeyToCurrentEnft);
    matchesKeyToCurrent.resize(nMatchesSiftExclusiveKeyToCurrent);

#if VERBOSE_FEATURE_TRACKING >= 2
    printf("  Frame (%d, %d)\n", iFrmLast, iFrmCurrent);
#endif
    m_ftrTrackerEnft.FromMatches13To23(iFtrsKeyToLast, matchesKeyToCurrentEnft,
                                       m_matchesEnft2);
    const ushort nMatchesEnftExclusiveKeyToCurrent = ushort(
                matchesKeyToCurrentEnft.size());
    RemoveOutlierMatchesEnft(iFrmLast, iFrmCurrent, FLastToCurrent, m_matchesEnft2);
    const ushort nMatchesEnftExclusiveLastToCurrent = ushort(
                matchesLastToCurrentEnft.size());
    m_ftrTrackerEnft.FromMatches13To23(iFtrsKeyToLast, m_matchesEnft1);
    PushBackMatches(m_matchesEnft1, m_matchesEnft2);
    PushBackMatches(m_matchesEnft2, matchesLastToCurrentEnft);
    const ushort nMatchesEnftCommon = ushort(m_matchesEnft2.size());

#if VERBOSE_FEATURE_TRACKING >= 2
    printf("----------------------------------------------------------------\n");
    printf("  SIFT matches: (%d + (%d) + %d) = %d\n",
           nMatchesSiftExclusiveKeyToCurrent, nMatchesSiftCommon,
           nMatchesSiftExclusiveLastToCurrent,
           nMatchesSiftExclusiveKeyToCurrent + nMatchesSiftCommon +
           nMatchesSiftExclusiveLastToCurrent);
    printf("  ENFT features: (%d + (%d) + %d) = %d\n",
           nMatchesEnftExclusiveKeyToCurrent, nMatchesEnftCommon,
           nMatchesEnftExclusiveLastToCurrent,
           nMatchesEnftExclusiveKeyToCurrent + nMatchesEnftCommon +
           nMatchesEnftExclusiveLastToCurrent);
#endif

    RemoveOutlierMatchesFlowVoting(iFrmLast, iFrmCurrent, matchesLastToCurrent, matchesLastToCurrentEnft); //unknow

    m_ftrMatcherSift.FromMatches13To23(iFtrsLastToKey, matchesLastToCurrent, m_matches1);
    PushBackMatches(m_matches1, matchesKeyToCurrent);
    m_ftrTrackerEnft.FromMatches13To23(iFtrsLastToKey, matchesLastToCurrentEnft, m_matchesEnft1);
    PushBackMatches(m_matchesEnft1, matchesKeyToCurrentEnft);
    NonMaximalSuppressEnft(iFrmKey, iFrmLast, iFrmCurrent, matchesKeyToCurrentEnft, matchesLastToCurrentEnft);

#if VERBOSE_FEATURE_TRACKING
    printf("----------------------------------------------------------------\n");
    printf("  SIFT matches: %d\n", matchesKeyToCurrent.size() + matchesLastToCurrent.size());
    printf("  ENFT matches: %d\n", matchesKeyToCurrentEnft.size() + matchesLastToCurrentEnft.size());
    printf("  Frame (%d, %d) matches: %d\n", iFrmKey, iFrmCurrent,
           matchesKeyToCurrent.size() + matchesKeyToCurrentEnft.size());
    printf("  Frame (%d, %d) matches: %d\n", iFrmLast, iFrmCurrent,
           matchesLastToCurrent.size() + matchesLastToCurrentEnft.size() +
           m_matches1.size() + m_matchesEnft1.size());
#endif
}