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

#ifndef _TRACK_MATCHER_H_
#define _TRACK_MATCHER_H_

#include "Viewing/Viewer.h"
#include "SequenceSet/SequenceSet.h"
#include "Utility/Configurator.h"
#include "HierarchicalKMeans.h"
#include "FeatureTracking/FeatureMatcherSift.h"
#include "SfM/FundamentalMatrixEstimator.h"
#include "SfM/CameraPairEstimator.h"
#include "MatchingMatrix.h"
#include "ProgramGL/ProgramGLUtility.h"
#include "ProgramGL/ProgramGLDescriptorNormalize.h"
#include <cvd/image.h>

#define VERBOSE_TRACK_MATCHING 1

namespace ENFT_SfM {

class TrackMatcher : public Viewer {

  public:

    void Initialize(const Sequence &seq, const std::string paramFileName);
    void Initialize(/*const */SequenceSet &seqs, const std::string paramFileName);
    void Run(Sequence &seq, const std::string outputFileName = "", const std::string tmpFileNameTrkClusters = "", FILE *fpTiming = NULL);
    void Run(Sequence &seq, const FrameIndexPairList &iFrmPairs, const std::string outputFileName = "", FILE *fpTiming = NULL);
    void Run(Sequence &seq, const SparseMatrix<float> &matchingMatrixInit, const std::string outputFileName = "", FILE *fpTiming = NULL);
    void Run(SequenceSet &seqs, const SequenceIndexPairList &iSeqPairs, const std::string outputFileName = "", const std::string tmpFileNameTrkClusters = "",
             const std::string tmpFileNameTrkMatches = "", FILE *fpTiming = NULL, const bool clearDescs = true);
    void Run(SequenceSet &seqs, const SequenceIndexPairList &iSeqPairs, const std::vector<FrameIndexPairList> &iFrmPairsList, const std::string outputFileName = "",
             const std::string tmpFileNameTrkMatches = "", FILE *fpTiming = NULL);

  public:

    // Parameters
    float m_descDotTh, m_descDistTh, m_nearest1To2RatioTh;
    FeatureIndex m_kfMinNumCmnTrksTwoKeyFrms, m_kfMinNumCmnTrksThreeKeyFrms;
    FrameIndex m_hkmClusteredTrkLenKF;
    uint m_ftrTexWidth, m_ftrTexWidthLog, m_descTexWidth, m_descTexWidthLog, m_maxNumFtrsPerImg;
    float m_errSqThReprojCam, m_errSqThReprojPt;
    ushort m_ransacMinNumInliersCam;
    FrameIndex m_enftFrmPairWinSize;
    ushort m_enftMaxNumItersInitMatchingMatrix, m_enftMaxNumItersUpdMatchingMatrix, m_enftMinConfidenceUpdMatchingMatrix, m_enftPositiveVotingTh;
    float m_enftMinConfidenceInitMatchingMatrixRatio, m_enftPositiveVotingRatioTh, m_enftMergeDistanceSqTh;
    bool m_removeSingleTrk, m_stop, m_view;

  protected:

    //////////////////////////////////////////////////////////////////////////
    // Basic
    //////////////////////////////////////////////////////////////////////////
    virtual void Initialize(/*const ushort &width, const ushort &height, */const ushort &maxNumFtrsPerImg, const FrameIndexList &seqFrmCnts,
            const Configurator &param);
    virtual void Initialize(const Sequence &seq, const Configurator &param);
    virtual void Initialize(/*const */SequenceSet &seqs, const Configurator &param);
    void ExtractKeyFrameSequence(Sequence &seq, Sequence &seqKF, FrameIndexList &iFrmsKF, TrackIndexList &iTrksKF);
    void ExtractKeyFrameSequence(Sequence &seq, Sequence &seqKF, FrameIndexList &iFrmsKF, TrackIndexList &iTrksKF, const std::vector<bool> &frmMarks);
    void ConvertTrackMatchesKeyFrameSequenceToOriginalSequence(const TrackIndexList &iTrksKF, TrackMatchList &trkMatches);
    void UploadDescriptorsFromCPU(const Sequence &seq, const FrameIndex &iFrm, const TextureGL4 &descTex);
    void UploadDescriptorsFromCPU(const Sequence &seq, const FrameIndex &iFrm, const FeatureIndexList &iFtrs, const TextureGL4 &descTex);
    void UploadFeaturesAndDescriptorsFromCPU(const Sequence &seq, const FrameIndex &iFrm, const TextureGL2 &ftrTex, const TextureGL4 &descTex);
    void UploadFeaturesAndDescriptorsFromCPU(const Sequence &seq, const FrameIndex &iFrm, const FeatureIndexList &iFtrs, const TextureGL2 &ftrTex,
            const TextureGL4 &descTex);
    void ExpandFramePairs(const FrameIndex nFrms1, const FrameIndex nFrms2, const FrameIndexPairList &iFrmPairs, FrameIndexPairList &iFrmPairsExp);
    bool MergeMatchedTracks(Sequence &seq, const TrackIndex &iTrk1, const TrackIndex &iTrk2);
    void MergeMatchedTracks_RemoveUnmergeableTrackMatches(Sequence &seq, TrackMatchList &trkMatches);

    //////////////////////////////////////////////////////////////////////////
    // ENFT
    //////////////////////////////////////////////////////////////////////////
    virtual void ClusterTracks(const Sequence &seq, std::vector<TrackIndexList> &iTrkClusters);
    virtual void ClusterTracks(const SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, std::vector<TrackIndexListPair> &iTrkClusters);
    virtual void ClusterTracks(const SequenceSet &seqs, const SequenceIndexPairList &iSeqPairs, std::vector<std::vector<TrackIndexListPair> > &iTrkClustersList);
    void MatchTracksEnft(/*const */Sequence &seq, const std::vector<TrackIndexList> &iTrkClusters, TrackMatchList &trkMatches);
    void MatchTracksEnft(/*const */Sequence &seq, const SparseMatrix<float> &matchingMatrixInit, TrackMatchList &trkMatches);
    void MatchTracksEnft(/*const */SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const std::vector<TrackIndexListPair> &iTrkClusters,
                                   TrackMatchList &trkMatches);
    void MatchTracksEnft(/*const */SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const SparseMatrix<float> &matchingMatrixInit,
                                   TrackMatchList &trkMatches);

    //////////////////////////////////////////////////////////////////////////
    // Feature matching
    //////////////////////////////////////////////////////////////////////////
    void MatchFeatures(/*const */Sequence &seq1, /*const */Sequence &seq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2, std::vector<ScoredMatch> &matches);
    void MatchFeatures(/*const */Sequence &seq1, /*const */Sequence &seq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureIndexList &iFtrs1,
                                 const FeatureIndexList &iFtrs2, std::vector<ScoredMatch> &matches);
    void MatchFeatures(/*const */Sequence &seq1, /*const */Sequence &seq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureIndexList &iFtrs1,
                                 const FeatureIndexList &iFtrs2, const FundamentalMatrix &F, FeatureMatchList &matches);
    void MatchFeatures_VerifyMatches(/*const */Sequence &seq, const FrameIndex &iFrm1, const FrameIndex &iFrm2, /*const */FeatureMatchList &matchesExist,
            FundamentalMatrix &F, std::vector<ushort> &inliers, std::vector<ushort> &outliers, FeatureMatchList &matchesNew);
    void MatchFeatures_VerifyMatches(/*const */SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const FrameIndex &iFrm1,
            const FrameIndex &iFrm2, const FeatureMatchList &matchesExist, FundamentalMatrix &F, std::vector<ushort> &inliers, std::vector<ushort> &outliers,
            FeatureMatchList &matchesNew);
    void VerifyMatches_MatchNewFeatures(/*const */Sequence &seq, const FrameIndex &iFrm1, const FrameIndex &iFrm2, /*const */FeatureMatchList &matchesExist,
            FundamentalMatrix &F, std::vector<ushort> &inliers, std::vector<ushort> &outliers, FeatureMatchList &matchesNew);
    void VerifyMatches_MatchNewFeatures(/*const */SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const FrameIndex &iFrm1,
            const FrameIndex &iFrm2, const FeatureMatchList &matchesExist, FundamentalMatrix &F, std::vector<ushort> &inliers, std::vector<ushort> &outliers,
            FeatureMatchList &matchesNew);

    //////////////////////////////////////////////////////////////////////////
    // IO
    //////////////////////////////////////////////////////////////////////////
    void PrepareViewing(const Sequence &seq, const std::vector<TrackIndexList> &iTrkClusters);
    void PrepareViewing(const Sequence &seq, const SparseMatrix<float> &matchingMatrixInit);
    void PrepareViewing(const SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const std::vector<TrackIndexListPair> &iTrkClusters);
    void PrepareViewing(const SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const SparseMatrix<float> &matchingMatrixInit);
    void ViewMatchingMatrix();
    void PrepareInitialMatchingMatrixImageAndTexture(const Sequence &seq, const std::vector<TrackIndexList> &iTrkClusters);
    void PrepareInitialMatchingMatrixImageAndTexture(const Sequence &seq, const SparseMatrix<float> &matchingMatrixInit);
    void PrepareInitialMatchingMatrixImageAndTexture(const Sequence &seq1, const Sequence &seq2, const std::vector<TrackIndexListPair> &iTrkClusters);
    void PrepareInitialMatchingMatrixImageAndTexture(const Sequence &seq1, const Sequence &seq2, const SparseMatrix<float> &matchingMatrixInit);
    void PrepareUpdatingMatchingMatrixImageAndTexture(const Sequence &seq, const FrameIndex &iFrm1Upd, const FrameIndex &iFrm2Upd, const FeatureMatchList &matches);
    void PrepareUpdatingMatchingMatrixImageAndTexture(const Sequence &seq, const TrackMatchList &trkMatches);
    void PrepareUpdatingMatchingMatrixImageAndTexture(const Sequence &seq1, const Sequence &seq2);
    void ScaleMatchingMatrixTexture(const TextureGL1 &srcTex, const TextureGL1 &dstTex);
    bool SaveTrackClusters(const char *fileName, const std::vector<TrackIndexList> &iTrkClusters);
    bool LoadTrackClusters(const char *fileName, std::vector<TrackIndexList> &iTrkClusters);
    bool SaveTrackClusters(const char *fileName, const std::vector<TrackIndexListPair> &iTrkClusters);
    bool LoadTrackClusters(const char *fileName, std::vector<TrackIndexListPair> &iTrkClusters);
    bool SaveTrackClusters(const char *fileName, const std::vector<std::vector<TrackIndexListPair> > &iTrkClustersList);
    bool LoadTrackClusters(const char *fileName, std::vector<std::vector<TrackIndexListPair> > &iTrkClustersList);
    bool SaveTrackMatches(const char *fileName, const TrackMatchList &trkMatches);
    bool LoadTrackMatches(const char *fileName, TrackMatchList &trkMatches);
    void DrawFeawtureMatches(const Sequence &seq, const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureMatchList &matchesExist,
                             const std::vector<ushort> &inliers, const std::vector<ushort> &outliers, const FeatureMatchList &matchesNew);
    virtual void OnDraw();
    virtual void OnDrawString();
    virtual bool OnKeyDown(const int key);
    virtual void OnMouseMove(const CVD::ImageRef &where);
    virtual void OnResize(const CVD::ImageRef &size);

  protected:

    SequenceSet m_seqsKF;
    std::vector<FrameIndexList> m_iFrmsListKF;
    std::vector<TrackIndexList> m_iTrksListKF;

    std::vector<TrackIndexListPair> m_iTrkClusters;
    std::vector<TrackMatchList> m_trkMatchesList;

    std::vector<std::vector<bool> > m_frmMarksList;
    std::vector<FrameIndexPairList> m_iFrmPairsListExp;
    SparseMatrix<float> m_matchingMatrixInit;

    HierarchicalKMeans<Descriptor, DESCRIPTOR_DIMENSION, float, ubyte> m_hkm;
    std::vector<KMeansCluster> m_clusters;

    FeatureMatcherSift m_ftrMatcherSift;

    FundamentalMatrixEstimator      m_Festor;
    FundamentalMatrix               m_F;
    FundamentalMatrixEstimatorData  m_Fdata;
    //HomographyEstimator       m_Hestor;
    //AlignedVector<Homography> m_HList;

    CameraPairEstimator         m_CPestor;
    CameraPair                  m_CP;
    CameraPairEstimatorData     m_CPdata;

    TextureGL2 m_ftrTex1, m_ftrTex2;
    TextureGL4 m_descTex1, m_descTex2;
    AlignedVector<Point2D> m_ftrs;
    AlignedVector<Descriptor> m_descs;

    std::vector<ScoredMatch> m_scoredMatches;
    std::vector<ushort> m_orders, m_inliers, m_outliers, m_inliers3D;
    TrackIndexList m_idxs;
    FeatureMatchList m_matchesExist, m_matchesNew, m_matchesPrior;
    FeatureIndexList m_iFtrs1Unmatched, m_iFtrs2Unmatched;
    MeasurementIndexList m_iMeas;
    std::vector<bool> m_marks1, m_marks2;
    std::vector<Match<MeasurementIndex> > m_iMeasOverlapping;
    std::vector<float *> m_pts;

    MatchingMatrix m_matchingMatrix;

    bool m_initView, m_hideSeed;
    std::vector<CVD::Rgb<ubyte> > m_seedClrs;
    std::vector<FrameIndexPairList> m_crossesList;
    CVD::Image<float> m_matchingMatrixInitImg;
    CVD::Image<ushort> m_matchingMatrixUpdImg;
    CVD::Image<float> m_img;
    TextureGL1 m_matchingMatrixInitTex/*, m_matchingMatrixInitTexScaled*/, m_matchingMatrixUpdTex, m_matchingMatrixUpdTexScaled;
    //FeatureIndexList m_frm1ClusteredTrkCnts, m_frm2ClusteredTrkCnts;
    Point2D m_factorFrmToWin, m_factorWinToFrm;
    FrameIndex m_iFrm1, m_iFrm2;
    float m_scale;

    ProgramGLScale m_programScale;

};

}//namespace ENFT_SfM

#endif
