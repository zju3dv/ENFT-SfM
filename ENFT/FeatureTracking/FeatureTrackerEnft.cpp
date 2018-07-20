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

using namespace ENFT_SfM;

void FeatureTracker::TrackFeaturesEnft(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FundamentalMatrix &F, const FeatureMatchList &matchesSift,
                                       const std::vector<FeatureEnftMatch> &matchesEnftExist, std::vector<FeatureEnftMatch> &matchesEnftNew) {
    matchesEnftNew.resize(0);
    if(m_enftMaxNumPlanes == 0)
        return;

#if VERBOSE_FEATURE_TRACKING >= 2
    printf("    ENFT tracking:");
#endif

    // Step1: get unmatched features (BOTH SIFT and ENFT features)
    const ushort iBuffer1 = m_bufferManager.GetDataBufferIndex(iFrm1), iBuffer2 = m_bufferManager.GetDataBufferIndex(iFrm2);
    const AlignedVector<Point2D> &ftrs1 = m_ftrsBuffer[iBuffer1], &ftrs2 = m_ftrsBuffer[iBuffer2];
    const ushort nFtrs1 = ushort(ftrs1.Size());
    std::vector<ushort> &iFtrs1Unmatched = m_idxs1;
    m_ftrTrackerEnft.FromMatches12ToUnmatchedFeatureIndexes1(nFtrs1, matchesSift, matchesEnftExist, iFtrs1Unmatched, m_marks1);
    const ushort nFtrs1Unmatched = ushort(iFtrs1Unmatched.size());
    if(nFtrs1Unmatched == 0)
        return;

    // Step2: plane segmentation (computer Homography Matrix)
    m_Hdata.SetMatches(ftrs1.Data(), ftrs2.Data(), matchesSift/*, matchesEnftExist*/);
    m_Hestor.RunLosacMultiple(m_Hdata, m_HList, m_inliersList, m_enftMaxNumPlanes, m_enftMinNumPtsPerPlane);
    const ushort nPlanes = ushort(m_HList.Size());
    if(nPlanes == 0)
        return;
#if VERBOSE_FEATURE_TRACKING >= 2
    printf(" %d planes,", nPlanes);
#endif

    // Step3: ENFT tracking //gainRatios
#if _DEBUG
    assert(m_ftrExtractorSift.GetFeaturesNumber(iBuffer1) == nFtrs1);
#endif
    FeatureMatchList &matchesInlier = m_matches3;
    m_gainRatios.resize(0);
    for(ushort iPlane = 0; iPlane < nPlanes; ++iPlane) {
        const std::vector<ushort> &inliers = m_inliersList[iPlane];
        const ushort nInliers = ushort(inliers.size());
        matchesInlier.resize(nInliers);
        for(ushort i = 0; i < nInliers; ++i)
            matchesInlier[i] = matchesSift[inliers[i]];
        m_ftrExtractorSift.SelectFeatures(iBuffer1, iBuffer2, m_iBufferTmp1, matchesInlier);
        m_ftrTrackerEnft.PushBackGainRatios(iBuffer1, iBuffer2, m_ftrExtractorSift.GetFeatureTexture(m_iBufferTmp1), nInliers, m_HList[iPlane], m_gainRatios);
    }
    const ushort N = ushort(m_gainRatios.size()), ith = (N >> 1);
    std::nth_element(m_gainRatios.begin(), m_gainRatios.begin() + ith, m_gainRatios.end());
    const float gainRatio = m_gainRatios[ith];

    m_ftrExtractorSift.SelectFeatures(iBuffer1, m_iBufferTmp1, iFtrs1Unmatched);
    m_ftrTrackerEnft.TrackFeatures(iBuffer1, iBuffer2, m_ftrExtractorSift.GetFeatureTexture(m_iBufferTmp1), nFtrs1Unmatched, F, m_HList, gainRatio, matchesEnftNew);
    const ushort nMatchesNew = ushort(matchesEnftNew.size());
    for(ushort i = 0; i < nMatchesNew; ++i)
        matchesEnftNew[i].SetFeatureIndex1(iFtrs1Unmatched[matchesEnftNew[i].GetFeatureIndex1()]);

#if VERBOSE_FEATURE_TRACKING >= 2
    printf(" %d features\n", matchesEnftNew.size());
#endif
}

void FeatureTracker::RemoveOutlierMatchesEnft(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FundamentalMatrix &F,
        std::vector<FeatureEnftMatch> &matchesEnft) {
#if VERBOSE_FEATURE_TRACKING >= 2
    printf("    ENFT inliers: %d", matchesEnft.size());
#endif
    ushort i, j;
    float work[6];
    const AlignedVector<Point2D> &ftrs1 = m_ftrsBuffer[m_bufferManager.GetDataBufferIndex(iFrm1)];
    const ushort nMatches = ushort(matchesEnft.size());
    for(i = j = 0; i < nMatches; ++i) {
        if(F.ComputeSymmetricSquaredError(ftrs1[matchesEnft[i].GetFeatureIndex1()], matchesEnft[i].GetFeature2(), work) < m_Festor.m_ransacErrorThreshold)
            matchesEnft[j++] = matchesEnft[i];
    }
    matchesEnft.resize(j);
#if VERBOSE_FEATURE_TRACKING >= 2
    printf(" - %d = %d matches\n", nMatches - matchesEnft.size(), matchesEnft.size());
#endif
}

void FeatureTracker::NonMaximalSuppressEnft(const FrameIndex &iFrm1, const FrameIndex &iFrm2, std::vector<FeatureEnftMatch> &matches) {
    const ushort nMatches = ushort(matches.size());
    //if(nMatches <= m_enftMaxNumFtrs)
    //  return;
    if(nMatches == 0)
        return;

    ushort i, j;
    m_iFtrsSort.resize(nMatches);
    for(i = 0; i < nMatches; ++i)
        m_iFtrsSort[i] = std::make_pair(matches[i].GetSAD(), i);
    std::sort(m_iFtrsSort.begin(), m_iFtrsSort.end());
    std::vector<bool> &ftrMarks = m_marks1;
    //m_ftrExtractorSift.MarkMaximalFeatures(matches.data(), m_iFtrsSort, USHRT_MAX, ftrMarks);
    m_ftrExtractorSift.MarkMaximalFeatures(matches.data(), m_iFtrsSort, m_enftMaxNumFtrs, ftrMarks);
    for(i = j = 0; i < nMatches; ++i) {
        if(ftrMarks[i])
            matches[j++] = matches[i];
    }
    matches.resize(j);

#if VERBOSE_FEATURE_TRACKING/* >= 2*/
    printf("----------------------------------------------------------------\n");
    printf("  Frame (%d, %d): %d - %d = %d matches\n", iFrm1, iFrm2, nMatches, nMatches - matches.size(), matches.size());
#endif
}

void FeatureTracker::NonMaximalSuppressEnft(const FrameIndex &iFrmKey, const FrameIndex &iFrmLast, const FrameIndex &iFrmCurrent,
        std::vector<FeatureEnftMatch> &matchesKeyToCurrent, std::vector<FeatureEnftMatch> &matchesLastToCurrent) {
    const ushort nMatchesKeyToCurrent = ushort(matchesKeyToCurrent.size()), nMatchesLastToCurrent = ushort(matchesLastToCurrent.size());
    const ushort nMatches = nMatchesKeyToCurrent + nMatchesLastToCurrent;
    //if(nMatches <= m_enftMaxNumFtrs)
    //  return;
    //if(nMatches == 0)
    //  return;

    std::vector<FeatureEnftMatch> &matches = matchesKeyToCurrent;
    matches.insert(matches.end(), matchesLastToCurrent.begin(), matchesLastToCurrent.end());

    ushort i, j;
    m_iFtrsSort.resize(nMatches);
    for(i = 0; i < nMatches; ++i)
        m_iFtrsSort[i] = std::make_pair(matches[i].GetSAD(), i);
    std::sort(m_iFtrsSort.begin(), m_iFtrsSort.end());
    std::vector<bool> &ftrMarks = m_marks1;
    //m_ftrExtractorSift.MarkMaximalFeatures(matches.data(), m_iFtrsSort, USHRT_MAX, ftrMarks);
    m_ftrExtractorSift.MarkMaximalFeatures(matches.data(), m_iFtrsSort, m_enftMaxNumFtrs, ftrMarks);
    for(i = j = 0; i < nMatchesKeyToCurrent; ++i) {
        if(ftrMarks[i])
            matchesKeyToCurrent[j++] = matchesKeyToCurrent[i];
    }
    matchesKeyToCurrent.resize(j);
    for(i = j = 0; i < nMatchesLastToCurrent; ++i) {
        if(ftrMarks[nMatchesKeyToCurrent + i])
            matchesLastToCurrent[j++] = matchesLastToCurrent[i];
    }
    matchesLastToCurrent.resize(j);

#if VERBOSE_FEATURE_TRACKING/* >= 2*/
    printf("----------------------------------------------------------------\n");
    printf("  Frame (%d, %d): %d - %d = %d matches\n", iFrmKey, iFrmCurrent,
           nMatchesKeyToCurrent, nMatchesKeyToCurrent - matchesKeyToCurrent.size(), matchesKeyToCurrent.size());
    printf("  Frame (%d, %d): %d - %d = %d matches\n", iFrmLast, iFrmCurrent,
           nMatchesLastToCurrent, nMatchesLastToCurrent - matchesLastToCurrent.size(), matchesLastToCurrent.size());
#endif
}