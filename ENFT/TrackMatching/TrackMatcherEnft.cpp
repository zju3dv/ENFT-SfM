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
#include "TrackMatcher.h"
#include "Utility/Random.h"
#include <cvd/image_io.h>
#include <cvd/vision.h>

using namespace ENFT_SfM;

void TrackMatcher::MatchTracksEnft(/*const */Sequence &seq, const std::vector<TrackIndexList> &iTrkClusters, TrackMatchList &trkMatches) {
#if VERBOSE_TRACK_MATCHING
    printf("Matching tracks...\n");
#endif

    m_matchingMatrix.Initialize(seq, iTrkClusters);

    const bool stopBkp = m_stop;
    PrepareViewing(seq, iTrkClusters);

    ushort iterInit, iterUpd, iRegion, confidenceUpd;
    FrameIndex iFrm1Init, iFrm2Init, iFrm1Upd, iFrm2Upd;
    float confidenceInit, confidenceInitMin;

    trkMatches.resize(0);
    for(iterInit = iRegion = 0; iterInit < m_enftMaxNumItersInitMatchingMatrix; ++iterInit) {
        if(!m_matchingMatrix.ExtractInitialSeed(iFrm1Init, iFrm2Init, m_matchesExist, m_idxs, confidenceInit))
            break;
        if(iterInit == 0)
            confidenceInitMin = confidenceInit * m_enftMinConfidenceInitMatchingMatrixRatio;
        else if(confidenceInit < confidenceInitMin)
            break;
        MatchFeatures_VerifyMatches(seq, iFrm1Init, iFrm2Init, m_matchesExist, m_F, m_inliers, m_outliers, m_matchesNew);
        confidenceUpd = ushort(m_matchesNew.size());
#if VERBOSE_TRACK_MATCHING
        printf("\r  [%d] Frame (%d, %d): %d matches", iRegion, iFrm1Init, iFrm2Init, confidenceUpd);
#endif
        if(confidenceUpd < m_enftMinConfidenceUpdMatchingMatrix) {
            m_seedClrs.push_back(CVD::Rgb<ubyte>(Random::GenerateUbyte(), Random::GenerateUbyte(), Random::GenerateUbyte()));
            m_crossesList.push_back(FrameIndexPairList(1, FrameIndexPair(iFrm1Init, iFrm2Init)));
            //DrawFeawtureMatches(seq, iFrm1Init, iFrm2Init, m_matchesExist, m_inliers, m_outliers, m_matchesNew);
            ViewMatchingMatrix();
            continue;
        }
        ++iRegion;
#if VERBOSE_TRACK_MATCHING
        printf("\n");
#endif
        m_matchingMatrix.VoteCandidateTrackMatches(m_idxs, m_inliers, m_outliers);
        m_matchingMatrix.AddCandidateTrackMatches_UpdateConfidences(iFrm1Init, iFrm2Init, m_matchesNew);

        m_seedClrs.push_back(CVD::Rgb<ubyte>(Random::GenerateUbyte(), Random::GenerateUbyte(), Random::GenerateUbyte()));
        m_crossesList.push_back(FrameIndexPairList(1, FrameIndexPair(iFrm1Init, iFrm2Init)));
        PrepareUpdatingMatchingMatrixImageAndTexture(seq, iFrm1Init, iFrm2Init, m_matchesNew);
        //DrawFeawtureMatches(seq, iFrm1Init, iFrm2Init, m_matchesExist, m_inliers, m_outliers, m_matchesNew);
        ViewMatchingMatrix();

        for(iterUpd = 0; iterUpd < m_enftMaxNumItersUpdMatchingMatrix; ++iterUpd) {
            if(!m_matchingMatrix.ExtractUpdatedSeed(iFrm1Upd, iFrm2Upd, m_matchesExist, m_idxs))
                break;
            VerifyMatches_MatchNewFeatures(seq, iFrm1Upd, iFrm2Upd, m_matchesExist, m_F, m_inliers, m_outliers, m_matchesNew);
            confidenceUpd = ushort(m_inliers.size() + m_matchesNew.size());
#if VERBOSE_TRACK_MATCHING
            printf("\r      Frame (%d, %d): %d", iFrm1Upd, iFrm2Upd, confidenceUpd);
#endif
            if(confidenceUpd < m_enftMinConfidenceUpdMatchingMatrix) {
#if VERBOSE_TRACK_MATCHING
                printf("                                              ");
#endif
                break;
            }
#if VERBOSE_TRACK_MATCHING
            printf(" (%d inliers, %d outliers, %d new matches)     ", m_inliers.size(), m_outliers.size(), m_matchesNew.size());
#endif
            m_matchingMatrix.VoteCandidateTrackMatches(m_idxs, m_inliers, m_outliers);
            m_matchingMatrix.AddCandidateTrackMatches_UpdateConfidences(iFrm1Upd, iFrm2Upd, m_matchesNew);

            m_crossesList.back().push_back(FrameIndexPair(iFrm1Upd, iFrm2Upd));
            PrepareUpdatingMatchingMatrixImageAndTexture(seq, iFrm1Upd, iFrm2Upd, m_matchesNew);
            //DrawFeawtureMatches(seq, iFrm1Upd, iFrm2Upd, m_matchesExist, m_inliers, m_outliers, m_matchesNew);
            ViewMatchingMatrix();
        }
#if VERBOSE_TRACK_MATCHING
        if(iterUpd > 0)
            printf("\n");
#endif
    }

    const TrackIndex nCandidateTrkMatches = m_matchingMatrix.GetCandidateTrackMatchesNumber();
    m_matchingMatrix.ExtractInlierMatches(trkMatches, m_enftPositiveVotingTh, m_enftPositiveVotingRatioTh);
    const TrackIndex nTrkMatches = TrackIndex(trkMatches.size());
#if VERBOSE_TRACK_MATCHING
    printf("\nTrack matches: %d - %d = %d\n", nCandidateTrkMatches, nCandidateTrkMatches - nTrkMatches, nTrkMatches);
#endif

    m_stop = stopBkp;
    ViewMatchingMatrix();
    m_stop = stopBkp;
}

void TrackMatcher::MatchTracksEnft(/*const */Sequence &seq, const SparseMatrix<float> &matchingMatrixInit, TrackMatchList &trkMatches) {
#if VERBOSE_TRACK_MATCHING
    printf("Matching tracks...\n");
#endif

    m_matchingMatrix.Initialize(seq, matchingMatrixInit);

    const bool stopBkp = m_stop;
    PrepareViewing(seq, matchingMatrixInit);

    ushort iterInit, iterUpd, iRegion, confidenceUpd;
    FrameIndex iFrm1Init, iFrm2Init, iFrm1Upd, iFrm2Upd;
    float confidenceInit, confidenceInitMin;

    trkMatches.resize(0);
    for(iterInit = iRegion = 0; iterInit < m_enftMaxNumItersInitMatchingMatrix; ++iterInit) {
        if(!m_matchingMatrix.ExtractInitialSeed(iFrm1Init, iFrm2Init, m_matchesExist, m_idxs, confidenceInit))
            break;
        if(iterInit == 0)
            confidenceInitMin = confidenceInit * m_enftMinConfidenceInitMatchingMatrixRatio;
        else if(confidenceInit < confidenceInitMin)
            break;
        MatchFeatures_VerifyMatches(seq, iFrm1Init, iFrm2Init, m_matchesExist, m_F, m_inliers, m_outliers, m_matchesNew);
        confidenceUpd = ushort(m_matchesNew.size());
#if VERBOSE_TRACK_MATCHING
        printf("\r  [%d] Frame (%d, %d): %d matches", iRegion, iFrm1Init, iFrm2Init, confidenceUpd);
#endif
        if(confidenceUpd < m_enftMinConfidenceUpdMatchingMatrix) {
            m_seedClrs.push_back(CVD::Rgb<ubyte>(Random::GenerateUbyte(), Random::GenerateUbyte(), Random::GenerateUbyte()));
            m_crossesList.push_back(FrameIndexPairList(1, FrameIndexPair(iFrm1Init, iFrm2Init)));
            ViewMatchingMatrix();
            continue;
        }
        ++iRegion;
#if VERBOSE_TRACK_MATCHING
        printf("\n");
#endif
        m_matchingMatrix.VoteCandidateTrackMatches(m_idxs, m_inliers, m_outliers);
        m_matchingMatrix.AddCandidateTrackMatches_UpdateConfidences(iFrm1Init, iFrm2Init, m_matchesNew);

        m_seedClrs.push_back(CVD::Rgb<ubyte>(Random::GenerateUbyte(), Random::GenerateUbyte(), Random::GenerateUbyte()));
        m_crossesList.push_back(FrameIndexPairList(1, FrameIndexPair(iFrm1Init, iFrm2Init)));
        PrepareUpdatingMatchingMatrixImageAndTexture(seq, iFrm1Init, iFrm2Init, m_matchesNew);
        ViewMatchingMatrix();

        for(iterUpd = 0; iterUpd < m_enftMaxNumItersUpdMatchingMatrix; ++iterUpd) {
            if(!m_matchingMatrix.ExtractUpdatedSeed(iFrm1Upd, iFrm2Upd, m_matchesExist, m_idxs))
                break;
            VerifyMatches_MatchNewFeatures(seq, iFrm1Upd, iFrm2Upd, m_matchesExist, m_F, m_inliers, m_outliers, m_matchesNew);
            confidenceUpd = ushort(m_inliers.size() + m_matchesNew.size());
#if VERBOSE_TRACK_MATCHING
            printf("\r      Frame (%d, %d): %d", iFrm1Upd, iFrm2Upd, confidenceUpd);
#endif
            if(confidenceUpd < m_enftMinConfidenceUpdMatchingMatrix) {
#if VERBOSE_TRACK_MATCHING
                printf("                                              ");
#endif
                break;
            }
#if VERBOSE_TRACK_MATCHING
            printf(" (%d inliers, %d outliers, %d new matches)     ", m_inliers.size(), m_outliers.size(), m_matchesNew.size());
#endif
            m_matchingMatrix.VoteCandidateTrackMatches(m_idxs, m_inliers, m_outliers);
            m_matchingMatrix.AddCandidateTrackMatches_UpdateConfidences(iFrm1Upd, iFrm2Upd, m_matchesNew);

            m_crossesList.back().push_back(FrameIndexPair(iFrm1Upd, iFrm2Upd));
            PrepareUpdatingMatchingMatrixImageAndTexture(seq, iFrm1Upd, iFrm2Upd, m_matchesNew);
            ViewMatchingMatrix();
        }
#if VERBOSE_TRACK_MATCHING
        if(iterUpd > 0)
            printf("\n");
#endif
    }

    const TrackIndex nCandidateTrkMatches = m_matchingMatrix.GetCandidateTrackMatchesNumber();
    m_matchingMatrix.ExtractInlierMatches(trkMatches, m_enftPositiveVotingTh, m_enftPositiveVotingRatioTh);
    const TrackIndex nTrkMatches = TrackIndex(trkMatches.size());
#if VERBOSE_TRACK_MATCHING
    printf("\nTrack matches: %d - %d = %d\n", nCandidateTrkMatches, nCandidateTrkMatches - nTrkMatches, nTrkMatches);
#endif

    m_stop = stopBkp;
    ViewMatchingMatrix();
    m_stop = stopBkp;
}

void TrackMatcher::MatchTracksEnft(/*const */SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2,
        const std::vector<TrackIndexListPair> &iTrkClusters,
        TrackMatchList &trkMatches) {
#if VERBOSE_TRACK_MATCHING
    printf("Matching tracks\n");
#endif

    const Sequence &seq1 = seqs[iSeq1], &seq2 = seqs[iSeq2];
    m_matchingMatrix.Initialize(seq1, seq2, iTrkClusters, iSeq2 == iSeq1 + 1);

    PrepareViewing(seqs, iSeq1, iSeq2, iTrkClusters);

    ushort iterInit, iterUpd, iRegion, confidenceUpd;
    FrameIndex iFrm1Init, iFrm2Init, iFrm1Upd, iFrm2Upd;
    float confidenceInit, confidenceInitMin;

    trkMatches.resize(0);
    for(iterInit = iRegion = 0; iterInit < m_enftMaxNumItersInitMatchingMatrix; ++iterInit) {
        if(!m_matchingMatrix.ExtractInitialSeed(iFrm1Init, iFrm2Init, m_matchesExist, m_idxs, confidenceInit))
            break;
        if(iterInit == 0)
            confidenceInitMin = confidenceInit * m_enftMinConfidenceInitMatchingMatrixRatio;
        else if(confidenceInit < confidenceInitMin)
            break;
        MatchFeatures_VerifyMatches(seqs, iSeq1, iSeq2, iFrm1Init, iFrm2Init, m_matchesExist, m_F, m_inliers, m_outliers, m_matchesNew);
        confidenceUpd = ushort(m_matchesNew.size());
#if VERBOSE_TRACK_MATCHING
        printf("\r  [%d] Frame (%d, %d): %d matches", iRegion, iFrm1Init, iFrm2Init, confidenceUpd);
#endif
        if(confidenceUpd < m_enftMinConfidenceUpdMatchingMatrix) {
            m_seedClrs.push_back(CVD::Rgb<ubyte>(Random::GenerateUbyte(), Random::GenerateUbyte(), Random::GenerateUbyte()));
            m_crossesList.push_back(FrameIndexPairList(1, FrameIndexPair(iFrm1Init, iFrm2Init)));
            ViewMatchingMatrix();
            continue;
        }
        ++iRegion;
#if VERBOSE_TRACK_MATCHING
        printf("\n");
#endif
        m_matchingMatrix.VoteCandidateTrackMatches(m_idxs, m_inliers, m_outliers);
        m_matchingMatrix.AddCandidateTrackMatches_UpdateConfidences(iFrm1Init, iFrm2Init, m_matchesNew);
        m_seedClrs.push_back(CVD::Rgb<ubyte>(Random::GenerateUbyte(), Random::GenerateUbyte(), Random::GenerateUbyte()));
        m_crossesList.push_back(FrameIndexPairList(1, FrameIndexPair(iFrm1Init, iFrm2Init)));
        PrepareUpdatingMatchingMatrixImageAndTexture(seq1, seq2);
        ViewMatchingMatrix();

        for(iterUpd = 0; iterUpd < m_enftMaxNumItersUpdMatchingMatrix; ++iterUpd) {
            if(!m_matchingMatrix.ExtractUpdatedSeed(iFrm1Upd, iFrm2Upd, m_matchesExist, m_idxs))
                break;
            VerifyMatches_MatchNewFeatures(seqs, iSeq1, iSeq2, iFrm1Upd, iFrm2Upd, m_matchesExist, m_F, m_inliers, m_outliers, m_matchesNew);
            confidenceUpd = ushort(m_inliers.size() + m_matchesNew.size());
#if VERBOSE_TRACK_MATCHING
            printf("\r      Frame (%d, %d): %d", iFrm1Upd, iFrm2Upd, confidenceUpd);
#endif
            if(confidenceUpd < m_enftMinConfidenceUpdMatchingMatrix) {
#if VERBOSE_TRACK_MATCHING
                printf("                                              ");
#endif
                break;
            }
#if VERBOSE_TRACK_MATCHING
            printf(" (%d inliers, %d outliers, %d new matches)     ", m_inliers.size(), m_outliers.size(), m_matchesNew.size());
#endif
            m_matchingMatrix.VoteCandidateTrackMatches(m_idxs, m_inliers, m_outliers);
            m_matchingMatrix.AddCandidateTrackMatches_UpdateConfidences(iFrm1Upd, iFrm2Upd, m_matchesNew);

            m_crossesList.back().push_back(FrameIndexPair(iFrm1Upd, iFrm2Upd));
            PrepareUpdatingMatchingMatrixImageAndTexture(seq1, seq2);
            ViewMatchingMatrix();
        }
#if VERBOSE_TRACK_MATCHING
        if(iterUpd > 0)
            printf("\n");
#endif
    }

    const TrackIndex nCandidateTrkMatches = m_matchingMatrix.GetCandidateTrackMatchesNumber();
    m_matchingMatrix.ExtractInlierMatches(trkMatches, m_enftPositiveVotingTh, m_enftPositiveVotingRatioTh);
    const TrackIndex nTrkMatches = TrackIndex(trkMatches.size());
#if VERBOSE_TRACK_MATCHING
    printf("\nTrack matches: %d - %d = %d\n", nCandidateTrkMatches, nCandidateTrkMatches - nTrkMatches, nTrkMatches);
#endif

    //m_stop = stopBkp;
    ViewMatchingMatrix();
    //m_stop = stopBkp;
}


void TrackMatcher::MatchTracksEnft(/*const */SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2,
        const SparseMatrix<float> &matchingMatrixInit,
        TrackMatchList &trkMatches) {
#if VERBOSE_TRACK_MATCHING
    printf("Matching tracks\n");
#endif

    const Sequence &seq1 = seqs[iSeq1], &seq2 = seqs[iSeq2];
    m_matchingMatrix.Initialize(seq1, seq2, matchingMatrixInit);

    PrepareViewing(seqs, iSeq1, iSeq2, matchingMatrixInit);

    ushort iterInit, iterUpd, iRegion, confidenceUpd;
    FrameIndex iFrm1Init, iFrm2Init, iFrm1Upd, iFrm2Upd;
    float confidenceInit, confidenceInitMin;

    trkMatches.resize(0);
    for(iterInit = iRegion = 0; iterInit < m_enftMaxNumItersInitMatchingMatrix; ++iterInit) {
        if(!m_matchingMatrix.ExtractInitialSeed(iFrm1Init, iFrm2Init, m_matchesExist, m_idxs, confidenceInit))
            break;
        if(iterInit == 0)
            confidenceInitMin = confidenceInit * m_enftMinConfidenceInitMatchingMatrixRatio;
        else if(confidenceInit < confidenceInitMin)
            break;
        MatchFeatures_VerifyMatches(seqs, iSeq1, iSeq2, iFrm1Init, iFrm2Init, m_matchesExist, m_F, m_inliers, m_outliers, m_matchesNew);
        confidenceUpd = ushort(m_matchesNew.size());
#if VERBOSE_TRACK_MATCHING
        printf("\r  [%d] Frame (%d, %d): %d matches", iRegion, iFrm1Init, iFrm2Init, confidenceUpd);
#endif
        if(confidenceUpd < m_enftMinConfidenceUpdMatchingMatrix) {
            m_seedClrs.push_back(CVD::Rgb<ubyte>(Random::GenerateUbyte(), Random::GenerateUbyte(), Random::GenerateUbyte()));
            m_crossesList.push_back(FrameIndexPairList(1, FrameIndexPair(iFrm1Init, iFrm2Init)));
            ViewMatchingMatrix();
            continue;
        }
        ++iRegion;
#if VERBOSE_TRACK_MATCHING
        printf("\n");
#endif
        m_matchingMatrix.VoteCandidateTrackMatches(m_idxs, m_inliers, m_outliers);
        m_matchingMatrix.AddCandidateTrackMatches_UpdateConfidences(iFrm1Init, iFrm2Init, m_matchesNew);
        m_seedClrs.push_back(CVD::Rgb<ubyte>(Random::GenerateUbyte(), Random::GenerateUbyte(), Random::GenerateUbyte()));
        m_crossesList.push_back(FrameIndexPairList(1, FrameIndexPair(iFrm1Init, iFrm2Init)));
        PrepareUpdatingMatchingMatrixImageAndTexture(seq1, seq2);
        ViewMatchingMatrix();

        for(iterUpd = 0; iterUpd < m_enftMaxNumItersUpdMatchingMatrix; ++iterUpd) {
            if(!m_matchingMatrix.ExtractUpdatedSeed(iFrm1Upd, iFrm2Upd, m_matchesExist, m_idxs))
                break;
            VerifyMatches_MatchNewFeatures(seqs, iSeq1, iSeq2, iFrm1Upd, iFrm2Upd, m_matchesExist, m_F, m_inliers, m_outliers, m_matchesNew);
            confidenceUpd = ushort(m_inliers.size() + m_matchesNew.size());
#if VERBOSE_TRACK_MATCHING
            printf("\r      Frame (%d, %d): %d", iFrm1Upd, iFrm2Upd, confidenceUpd);
#endif
            if(confidenceUpd < m_enftMinConfidenceUpdMatchingMatrix) {
#if VERBOSE_TRACK_MATCHING
                printf("                                              ");
#endif
                break;
            }
#if VERBOSE_TRACK_MATCHING
            printf(" (%d inliers, %d outliers, %d new matches)     ", m_inliers.size(), m_outliers.size(), m_matchesNew.size());
#endif
            m_matchingMatrix.VoteCandidateTrackMatches(m_idxs, m_inliers, m_outliers);
            m_matchingMatrix.AddCandidateTrackMatches_UpdateConfidences(iFrm1Upd, iFrm2Upd, m_matchesNew);

            m_crossesList.back().push_back(FrameIndexPair(iFrm1Upd, iFrm2Upd));
            PrepareUpdatingMatchingMatrixImageAndTexture(seq1, seq2);
            ViewMatchingMatrix();
        }
#if VERBOSE_TRACK_MATCHING
        if(iterUpd > 0)
            printf("\n");
#endif
    }

    const TrackIndex nCandidateTrkMatches = m_matchingMatrix.GetCandidateTrackMatchesNumber();
    m_matchingMatrix.ExtractInlierMatches(trkMatches, m_enftPositiveVotingTh, m_enftPositiveVotingRatioTh);
    const TrackIndex nTrkMatches = TrackIndex(trkMatches.size());
#if VERBOSE_TRACK_MATCHING
    printf("\nTrack matches: %d - %d = %d\n", nCandidateTrkMatches, nCandidateTrkMatches - nTrkMatches, nTrkMatches);
#endif

    //m_stop = stopBkp;
    ViewMatchingMatrix();
    //m_stop = stopBkp;
}