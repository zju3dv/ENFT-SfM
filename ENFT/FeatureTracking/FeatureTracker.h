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

#ifndef _FEATURE_TRACKER_H_
#define _FEATURE_TRACKER_H_

#include "Viewing/ViewerSequence.h"
#include "Utility/Configurator.h"
#include "FeatureExtractorSift.h"
#include "FeatureMatcherSift.h"
#include "FeatureTrackerEnft.h"
#include "SfM/FundamentalMatrixEstimator.h"
#include "SfM/HomographyEstimator.h"
#include "ProgramGL/ProgramGLUtility.h"
#include "Utility/BufferManager.h"
#include "Utility/Bucket.h"
#include "Utility/Timer.h"

#define VERBOSE_FEATURE_TRACKING    1
#define KEY_STOP_FEATURE_TRACKING   19  // Ctrl + s

namespace ENFT_SfM {

class FeatureTracker : private ViewerSequence {

  public:

    FeatureTracker();
    ~FeatureTracker();
    void Initialize(const Sequence &seq, const std::string paramFileName);
    void Initialize(const Sequence &seq, const Configurator &param);
    void Run(Sequence &seq, const std::string outputFileName = "",
             FILE *fpTiming = NULL);

  public:

    // Parameters
    ushort m_imgWidth, m_imgHeight;
    Timer extractTimer, matchTimer, tmpTimer;

    bool m_bruteForceMatching, m_removeSingleTrk, m_repeated, m_siftUseGPU;
    ushort m_ftrMinDist, m_siftMaxNumFtrs, m_epInliersMinNum, m_maxNumFtrs,
           m_enftMaxNumFtrs, m_enftSmoothLevel, m_enftMaxNumPlanes,
           m_enftMinNumPtsPerPlane;
    float m_descDistTh, m_nearest1To2RatioTh;
    ushort m_flowPositiveVotingTh;
    float m_flowPositiveVotingRatioTh, m_flowDifferenceTh;
    ushort m_minNumMatchesKeyToLast, m_bfPositiveVotingTh;
    float m_bfPositiveVotingRatioTh;
    bool m_stop, m_view;

    float errThEp, errThHomo;

  protected:

    //////////////////////////////////////////////////////////////////////////
    // Basic
    //////////////////////////////////////////////////////////////////////////
    void RunSequentialTracking(Sequence &seq, FILE *fpTiming = NULL);
    void RunBruteForceMatching(Sequence &seq, FILE *fpTiming = NULL);
    void PushBackFrame(const FrameIndex &iFrm, Sequence &seq);
    void PushBackFrameAndFeatureMatches2(const FrameIndex &iFrm1,
                                         const FrameIndex &iFrm2, Sequence &seq);
    void PushBackFrameAndFeatureMatches3(const FrameIndex &iFrmKey,
                                         const FrameIndex &iFrmLast, const FrameIndex &iFrmCurrent, Sequence &seq);
    void SetFeatureBuffer(const Sequence &seq, const FrameIndex &iFrm);
    ushort CountFeatureMatchesKeyFrameToLastFrame(const Sequence &seq,
            const FrameIndex &iFrmKey, const FrameIndex &iFrmLast);
    void GetFeatureMatchesKeyFrameToLastFrame(const Sequence &seq,
            const FrameIndex &iFrmKey, const FrameIndex &iFrmLast,
            FeatureIndexList &iFtrsKeyToLast,
            FeatureIndexList &iFtrsLastToKey);

    //////////////////////////////////////////////////////////////////////////
    // Feature tracking
    //////////////////////////////////////////////////////////////////////////
    void ExtractFeatures(const FrameIndex &iFrm);
    void TrackFeatures2(const FrameIndex &iFrm1, const FrameIndex &iFrm2);
    void TrackFeatures3(const FrameIndex &iFrmKey, const FrameIndex &iFrmLast,
                        const FrameIndex &iFrmCurrent, const FeatureIndexList &iFtrsKeyToLast,
                        const FeatureIndexList
                        &iFtrsLastToKey/*, const FundamentalMatrix &FKeyToLast*/);

    //////////////////////////////////////////////////////////////////////////
    // Flow voting
    //////////////////////////////////////////////////////////////////////////
    void RemoveOutlierMatchesFlowVoting(const FrameIndex &iFrm1,
                                        const FrameIndex &iFrm2, FeatureMatchList &matchesSift,
                                        std::vector<FeatureEnftMatch> &matchesEnft);
    void VoteNeighboringFlows(const uint &iBin1, const uint &iBin2,
                              const std::vector<Point2D> &flows);

    //////////////////////////////////////////////////////////////////////////
    // SIFT
    //////////////////////////////////////////////////////////////////////////
    void MatchFeaturesSift(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
                           FundamentalMatrix &F, FeatureMatchList &matches, const bool larger);
    void RemoveOutlierMatchesSift_MatchNewFeaturesSift(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
            FundamentalMatrix &F, FeatureMatchList &matchesExist,
            FeatureMatchList &matchesNew, const bool larger);
    void RemoveOutlierMatchesSift(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
                                  const FundamentalMatrix &F, FeatureMatchList &matches);
    void RemoveConflictedMatchesSift(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
                                     FeatureMatchList &matches1, FeatureMatchList &matches2);
    void RemoveConflictedMatchesSift(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
                                     const FrameIndex &iFrm3, const FundamentalMatrix &F12,
                                     FeatureMatchList &matches13, FeatureMatchList &matches23);

    //////////////////////////////////////////////////////////////////////////
    // ENFT
    //////////////////////////////////////////////////////////////////////////
    void TrackFeaturesEnft(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
                           const FundamentalMatrix &F, const FeatureMatchList &matchesSift,
                           const std::vector<FeatureEnftMatch> &matchesEnftExist,
                           std::vector<FeatureEnftMatch> &matchesEnftNew);
    void RemoveOutlierMatchesEnft(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
                                  const FundamentalMatrix &F, std::vector<FeatureEnftMatch> &matchesEnft);
    void NonMaximalSuppressEnft(const FrameIndex &iFrm1, const FrameIndex &iFrm2,
                                std::vector<FeatureEnftMatch> &matches);
    void NonMaximalSuppressEnft(const FrameIndex &iFrmKey,
                                const FrameIndex &iFrmLast, const FrameIndex &iFrmCurrent,
                                std::vector<FeatureEnftMatch> &matchesKeyToCurrent,
                                std::vector<FeatureEnftMatch> &matchesLastToCurrent);

    //////////////////////////////////////////////////////////////////////////
    // IO
    //////////////////////////////////////////////////////////////////////////
    void ViewSequence(const Sequence &seq, const FrameIndex &iFrm);
    virtual bool OnKeyDown(const int key);
    void PushBackTrackColors(const FrameIndex &iFrm, Sequence &seq);
    void SaveB(const char *fileName, const Sequence &seq, const FrameIndex &iFrmKey,
               const FrameIndex &iFrmLast, const FrameIndex &iFrmCurrent) const;
    void LoadB(const char *fileName, Sequence &seq, FrameIndex &iFrmKey,
               FrameIndex &iFrmLast, FrameIndex &iFrmCurrent);

  protected:

    CVD::Image<CVD::Rgb<ubyte> >    m_imgRGB, m_imgRGBTmp;
    CVD::Image<float>               m_imgGray;
    TextureGL3                      m_texRGB;
    TextureGL1                      m_texGray;
    ProgramGLConvertRGB2Gray        m_programConvertRGB2Gray;

    FeatureExtractorSift    m_ftrExtractorSift;
    FeatureMatcherSift      m_ftrMatcherSift;
    FeatureTrackerEnft      m_ftrTrackerEnft;

    FundamentalMatrixEstimator      m_Festor;
    FundamentalMatrixEstimatorData  m_Fdata;
    HomographyEstimator             m_Hestor;
    HomographyEstimatorData         m_Hdata;
    AlignedVector<Homography>       m_HList;

    ushort m_bufferSize, m_iBufferTmp1, m_iBufferTmp2;
    std::vector<AlignedVector<Point2D> > m_ftrsBuffer;
    std::vector<AlignedVector<Descriptor> > m_descsBuffer;
    AlignedVector<FundamentalMatrix> m_FBuffer;
    std::vector<FeatureMatchList> m_matchesBuffer;
    std::vector<std::vector<FeatureEnftMatch> > m_matchesBufferEnft;
    BufferManager<FrameIndex, ushort> m_bufferManager;

    std::vector<ushort> m_idxs1, m_idxs2;
    std::vector<ScoredMatch> m_matchesScored;
    std::vector<bool> m_marks1, m_marks2, m_marks3;
    FrameIndexList m_iFrms;
    FeatureIndexList m_iFtrsKeyToLast, m_iFtrsLastToKey, m_iFtrs2To1, m_iFtrs3To1;
    std::vector<std::vector<ushort> > m_inliersList;
    AlignedVector<Point2D> m_ftrs;
    FeatureMatchList m_matches1, m_matches2, m_matches3;
    std::vector<FeatureEnftMatch> m_matchesEnft1, m_matchesEnft2;
    std::vector<std::pair<float, uint> >    m_iFtrsSort;
    std::vector<float> m_gainRatios;

    std::vector<FeatureIndexList>   m_mapFtrToBufferFtr;
    std::vector<TrackIndexList>     m_mapFtrToTrk;
    FeatureIndexList                m_mapTrkToFtr;

    class CandidateFrame {
      public:
        CandidateFrame() {}
        CandidateFrame(const FrameIndex &iFrm, const FeatureIndex &nMatches,
                       const FeatureIndex &nFtrs) : m_iFrm(iFrm), m_nMatches(nMatches),
            m_nFtrs(nFtrs) {}
        inline const FrameIndex &GetFrameIndex() const {
            return m_iFrm;
        }
        inline bool operator < (const CandidateFrame &frm) const {
            return m_nMatches > frm.m_nMatches || m_nMatches == frm.m_nMatches &&
                   m_nFtrs < frm.m_nFtrs;
        }
      private:
        FrameIndex m_iFrm;
        FeatureIndex m_nMatches, m_nFtrs;
    };
    std::vector<CandidateFrame> m_candidateFrms;

    class CandidateTrack {
      public:
        CandidateTrack() {}
        CandidateTrack(const TrackIndex &iTrk, const ushort &nCrsps,
                       const ushort &nMatches) : m_iTrk(iTrk), m_nCrsps(nCrsps),
            m_nMatches(nMatches) {}
        inline const TrackIndex &GetTrackIndex() const {
            return m_iTrk;
        }
        inline bool operator < (const CandidateTrack &trk) const {
            return m_nCrsps > trk.m_nCrsps || m_nCrsps == trk.m_nCrsps &&
                   m_nMatches >= trk.m_nMatches;
        }
      private:
        TrackIndex m_iTrk;
        ushort m_nCrsps, m_nMatches;
    };
    std::vector<CandidateTrack> m_candidateTrks;

    Bucket<ushort> m_bucket;
    std::vector<Point2D> m_flows;
    class FlowVote {
      public:
        inline void VotePositive() {
            ++m_cntTotal;
            ++m_cntPositive;
        }
        inline void VoteNegative() {
            ++m_cntTotal;
        }
        inline const ushort &GetTotalCount() const {
            return m_cntTotal;
        }
        inline const ushort &GetPositiveCount() const {
            return m_cntPositive;
        }
        inline void Print() const {
            printf("%d/%d\n", m_cntPositive, m_cntTotal);
        }
      private:
        ushort m_cntTotal, m_cntPositive;
    };
    std::vector<FlowVote> m_flowVotes;

};

}//namespace ENFT_SfM

#endif
