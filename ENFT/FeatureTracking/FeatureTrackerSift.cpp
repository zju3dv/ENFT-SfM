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
#include <cvd/image_io.h>

using namespace ENFT_SfM;

static inline void SaveSiftFeatures(const char *fileName,
                                    const AlignedVector<Point2D> &ftrs,
                                    const AlignedVector<Descriptor> &descs) {

    FILE *fp = fopen( fileName, "w");
    const ushort nFtrs = ushort(descs.Size());
    for (ushort i = 0; i < nFtrs; ++i) {
        const Point2D &ftr = ftrs[i];
        fprintf(fp, "%d: (%f %f)\n", i, ftr.x(), ftr.y());

        const Descriptor &desc = descs[i];
        for (ushort j = 0; j < 64; ++j) {
            fprintf(fp, " %f", desc[j]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    printf("%s\n", fileName);
}

void FeatureTracker::MatchFeaturesSift(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
                                       FundamentalMatrix &F, FeatureMatchList &matches, const bool larger) {
    // Step1: SIFT matching
    const ushort iBuffer1 = m_bufferManager.GetDataBufferIndex(iFrm1),
                 iBuffer2 = m_bufferManager.GetDataBufferIndex(iFrm2);
    const AlignedVector<Point2D> &ftrs1 = m_ftrsBuffer[iBuffer1],
                                  &ftrs2 = m_ftrsBuffer[iBuffer2];
    const ushort nFtrs1 = ushort(ftrs1.Size()), nFtrs2 = ushort(ftrs2.Size());
    m_ftrMatcherSift.MatchFeatures(m_ftrExtractorSift.GetFeatureTexture(iBuffer1),
                                   m_ftrExtractorSift.GetFeatureTexture(iBuffer2),
                                   m_ftrExtractorSift.GetDescriptorTexture(iBuffer1),
                                   m_ftrExtractorSift.GetDescriptorTexture(iBuffer2), nFtrs1, nFtrs2, larger);
    m_ftrMatcherSift.DownloadMatches12ToCPU(nFtrs1, nFtrs2, m_matchesScored);
    const ushort nMatches = ushort(m_matchesScored.size());
    if (nMatches < m_epInliersMinNum) {
        F.Invalidate();
        matches.resize(0);
        return;
    }

    // Step2: filter outliers by epipolar geometry
    std::vector<ushort> &orders = m_idxs1, &inliers = m_idxs2;
    m_Fdata.SetMatches(ftrs1.Data(), ftrs2.Data(), m_matchesScored, orders);
    m_Festor.RunLoProsac(m_Fdata, orders, F, inliers);
    const ushort nInliers = ushort(inliers.size());
    if (nInliers < m_epInliersMinNum) {
        F.Invalidate();
        matches.resize(0);
        return;
    }

    m_ftrMatcherSift.MatchFeatures(m_ftrExtractorSift.GetFeatureTexture(iBuffer1),
                                   m_ftrExtractorSift.GetFeatureTexture(iBuffer2),
                                   m_ftrExtractorSift.GetDescriptorTexture(iBuffer1),
                                   m_ftrExtractorSift.GetDescriptorTexture(iBuffer2),
                                   nFtrs1, nFtrs2, F, false, larger);
    m_ftrMatcherSift.DownloadMatches12ToCPU(nFtrs1, nFtrs2, matches);

#if VERBOSE_FEATURE_TRACKING >= 2
    printf("    SIFT matching : %d - %d = %d matches\n",
           nMatches, nMatches - nInliers, nInliers);
#endif

}

void FeatureTracker::RemoveOutlierMatchesSift_MatchNewFeaturesSift(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
        FundamentalMatrix &F, FeatureMatchList &matchesExist, FeatureMatchList &matchesNew, const bool larger) {
    const ushort nMatchesExist = ushort(matchesExist.size());
#if VERBOSE_FEATURE_TRACKING >= 2
    printf("    SIFT matching: %d", nMatchesExist);
#endif
    // Step1: estimate fundamental matrix and remove outliers
    const ushort iBuffer1 = m_bufferManager.GetDataBufferIndex(iFrm1),
                 iBuffer2 = m_bufferManager.GetDataBufferIndex(iFrm2);
    const AlignedVector<Point2D> &ftrs1 = m_ftrsBuffer[iBuffer1],
                                  &ftrs2 = m_ftrsBuffer[iBuffer2];
    std::vector<ushort> &inliers = m_idxs1;
    if (nMatchesExist >= m_epInliersMinNum) {
        m_Fdata.SetMatches(ftrs1.Data(), ftrs2.Data(), matchesExist);
        m_Festor.RunLosac(m_Fdata, F, inliers);

        const ushort nInliersExist = ushort(inliers.size());
        if (nInliersExist >= m_epInliersMinNum) {
            for (ushort i = 0; i < nInliersExist; ++i) {
                matchesExist[i] = matchesExist[inliers[i]];
            }
            matchesExist.resize(nInliersExist);
        }
    }
    const ushort nInliersExist = ushort(matchesExist.size());
    if (nInliersExist < m_epInliersMinNum) {
        matchesExist.resize(0);
        MatchFeaturesSift(iFrm1, iFrm2, F, matchesNew, larger);
        return;
    }

#if VERBOSE_FEATURE_TRACKING >= 2
    printf(" - %d", nMatchesExist - matchesExist.size());
#endif

    // Step2: find unmatched features
    const ushort nFtrs1 = ushort(ftrs1.Size()), nFtrs2 = ushort(ftrs2.Size());
#if _DEBUG
    assert(nFtrs1 == m_ftrExtractorSift.GetFeaturesNumber(iBuffer1) &&
           nFtrs2 == m_ftrExtractorSift.GetFeaturesNumber(iBuffer2));
#endif
    std::vector<ushort> &iFtrs1Unmatched = m_idxs1, &iFtrs2Unmatched = m_idxs2;
    m_ftrMatcherSift.FromMatches12ToUnmatchedFeatureIndexes(nFtrs1, nFtrs2,
            matchesExist, iFtrs1Unmatched, iFtrs2Unmatched, m_marks1, m_marks2);
    m_ftrExtractorSift.SelectFeaturesAndDescriptors(iBuffer1, m_iBufferTmp1,
            iFtrs1Unmatched);
    m_ftrExtractorSift.SelectFeaturesAndDescriptors(iBuffer2, m_iBufferTmp2,
            iFtrs2Unmatched);

    // Step3: find new matchesExist among unmatched SIFT features by EPIPOLAR search
    const ushort nFtrs1Unmatched = ushort(iFtrs1Unmatched.size()),
                 nFtrs2Unmatched = ushort(iFtrs2Unmatched.size());
    m_ftrMatcherSift.MatchFeatures(m_ftrExtractorSift.GetFeatureTexture(
                                       m_iBufferTmp1), m_ftrExtractorSift.GetFeatureTexture(m_iBufferTmp2),
                                   m_ftrExtractorSift.GetDescriptorTexture(m_iBufferTmp1),
                                   m_ftrExtractorSift.GetDescriptorTexture(m_iBufferTmp2), nFtrs1Unmatched,
                                   nFtrs2Unmatched,
                                   F, true, larger);
    m_ftrMatcherSift.DownloadMatches12ToCPU(iFtrs1Unmatched, iFtrs2Unmatched,
                                            matchesNew);



#if VERBOSE_FEATURE_TRACKING >= 2
    printf(" + %d = %d matches\n", matchesNew.size(),
           matchesExist.size() + matchesNew.size());
#endif


}

void FeatureTracker::RemoveOutlierMatchesSift(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
        const FundamentalMatrix &F, FeatureMatchList &matches) {
    if (F.IsInvalid()) {
        return;
    }
#if VERBOSE_FEATURE_TRACKING >= 2
    printf("    SIFT inliers: %d", matches.size());
#endif
    ushort i, j;
    float work[6];
    const ushort iBuffer1 = m_bufferManager.GetDataBufferIndex(iFrm1),
                 iBuffer2 = m_bufferManager.GetDataBufferIndex(iFrm2);
    const AlignedVector<Point2D> &ftrs1 = m_ftrsBuffer[iBuffer1],
                                  &ftrs2 = m_ftrsBuffer[iBuffer2];
    const ushort nMatches = ushort(matches.size());
    for (i = j = 0; i < nMatches; ++i) {
        if (F.ComputeSymmetricSquaredError(ftrs1[matches[i].GetIndex1()],
                                           ftrs2[matches[i].GetIndex2()], work) < m_Festor.m_ransacErrorThreshold) {
            matches[j++] = matches[i];
        }
    }
    matches.resize(j);
#if VERBOSE_FEATURE_TRACKING >= 2
    printf(" - %d = %d matches\n", nMatches - matches.size(), matches.size());
#endif
}

void FeatureTracker::RemoveConflictedMatchesSift(const FrameIndex &iFrm1,
        const FrameIndex &iFrm2, FeatureMatchList &matches1,
        FeatureMatchList &matches2) {
    const ushort iBuffer1 = m_bufferManager.GetDataBufferIndex(iFrm1),
                 iBuffer2 = m_bufferManager.GetDataBufferIndex(iFrm2);
    const AlignedVector<Descriptor> &descs1 = m_descsBuffer[iBuffer1],
                                     &descs2 = m_descsBuffer[iBuffer2];
    const ushort nFtrs2 = ushort(descs2.Size());
    m_iFtrs2To1.assign(nFtrs2, INVALID_FEATURE_INDEX);

    ushort i;
    const ushort nMatches1 = ushort(matches1.size());
    for (i = 0; i < nMatches1; ++i) {
        m_iFtrs2To1[matches1[i].GetIndex2()] = matches1[i].GetIndex1();
    }

    FeatureIndex iFtr1, iFtr2;
    float dis1, dis2;
    const ushort nMatches2 = ushort(matches2.size());
    for (i = 0; i < nMatches2; ++i) {
        matches2[i].Get(iFtr1, iFtr2);
        if (m_iFtrs2To1[iFtr2] == INVALID_FEATURE_INDEX
                || m_iFtrs2To1[iFtr2] == iFtr1) {
            continue;
        }

        dis1 = 1 - Descriptor::Dot(descs1[m_iFtrs2To1[iFtr2]], descs2[iFtr2]);
        dis2 = 1 - Descriptor::Dot(descs1[iFtr1], descs2[iFtr2]);
        if (dis1 < dis2) {
            matches2[i].SetIndex1(INVALID_FEATURE_INDEX);
        } else {
            m_iFtrs2To1[iFtr2] = INVALID_FEATURE_INDEX;
        }
    }

    ushort j;
    for (i = j = 0; i < nMatches1; ++i) {
        if (m_iFtrs2To1[matches1[i].GetIndex2()] != INVALID_FEATURE_INDEX) {
            matches1[j++] = matches1[i];
        }
    }
    matches1.resize(j);
    for (i = j = 0; i < nMatches2; ++i) {
        if (matches2[i].GetIndex1() != INVALID_FEATURE_INDEX) {
            matches2[j++] = matches2[i];
        }
    }
    matches2.resize(j);
}

void FeatureTracker::RemoveConflictedMatchesSift(const FrameIndex &iFrm1,
        const FrameIndex &iFrm2, const FrameIndex &iFrm3, const FundamentalMatrix &F12,
        FeatureMatchList &matches13, FeatureMatchList &matches23) {
    const ushort iBuffer1 = m_bufferManager.GetDataBufferIndex(iFrm1);
    const ushort iBuffer2 = m_bufferManager.GetDataBufferIndex(iFrm2);
    const ushort iBuffer3 = m_bufferManager.GetDataBufferIndex(iFrm3);
    const AlignedVector<Point2D> &ftrs1 = m_ftrsBuffer[iBuffer1],
                                  &ftrs2 = m_ftrsBuffer[iBuffer2], &ftrs3 = m_ftrsBuffer[iBuffer3];
    const ushort nFtrs3 = ushort(ftrs3.Size());
    m_iFtrs3To1.assign(nFtrs3, INVALID_FEATURE_INDEX);

    ushort i;
    const ushort nMatches13 = ushort(matches13.size());
    for (i = 0; i < nMatches13; ++i) {
        m_iFtrs3To1[matches13[i].GetIndex2()] = matches13[i].GetIndex1();
    }

    FeatureIndex iFtr1, iFtr2, iFtr3;
    float dis13, dis23;
    float work[6];
    const AlignedVector<Descriptor> &descs1 = m_descsBuffer[iBuffer1],
                                     &descs2 = m_descsBuffer[iBuffer2], &descs3 = m_descsBuffer[iBuffer3];
    const ushort nMatches23 = ushort(matches23.size());
    for (i = 0; i < nMatches23; ++i) {
        matches23[i].Get(iFtr2, iFtr3);
        if ((iFtr1 = m_iFtrs3To1[iFtr3]) == INVALID_FEATURE_INDEX
                || F12.ComputeSymmetricSquaredError(ftrs1[iFtr1], ftrs2[iFtr2],
                        work) < m_Festor.m_ransacErrorThreshold) {
            continue;
        }

        dis13 = 1 - Descriptor::Dot(descs1[iFtr1], descs3[iFtr3]);
        dis23 = 1 - Descriptor::Dot(descs2[iFtr2], descs3[iFtr3]);

        if (dis13 < dis23) {
            matches23[i].SetIndex1(INVALID_FEATURE_INDEX);
        } else {
            m_iFtrs3To1[iFtr3] = INVALID_FEATURE_INDEX;
        }
    }

    ushort j;
    for (i = j = 0; i < nMatches13; ++i) {
        if (m_iFtrs3To1[matches13[i].GetIndex2()] != INVALID_FEATURE_INDEX) {
            matches13[j++] = matches13[i];
        }
    }
    matches13.resize(j);
    for (i = j = 0; i < nMatches23; ++i) {
        if (matches23[i].GetIndex1() != INVALID_FEATURE_INDEX) {
            matches23[j++] = matches23[i];
        }
    }
    matches23.resize(j);
}