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

#ifndef _SEQUENCE_REGISTEROR_H_
#define _SEQUENCE_REGISTEROR_H_

#define VERBOSE_SEQUENCE_REGISTRATION   1
#define KEY_STOP_SEQUENCE_REGISTRATION  19  // Ctrl + s

#include "Viewing/ViewerSequenceSet.h"
#include "Utility/Configurator.h"
#include "SfM/SimilarityTransformationEstimator.h"
#include "SfM/TranslationScaleEstimator.h"
#include "SfM/CameraEstimator.h"
#include "SfM/Point3DEstimator.h"
#include "SequenceSet/SequenceTransformationOptimizerDataSimilarity.h"
#include "SequenceSet/SequenceSetBundleAdjustorData3DSimilarity.h"
#include "SequenceSet/SegmentSetBundleAdjustorData2DSimilarity.h"
#include "Sequence/SequenceBundleAdjustorData.h"
#include "Sequence/SequenceBundleAdjustorDataIntrinsicVariable.h"
#include "Optimization/BundleAdjustor.h"
#include "Optimization/GlobalTransformationOptimizer.h"

namespace ENFT_SfM {

class SequenceRegisteror : protected ViewerSequenceSet {

  public:

    void Initialize(const SequenceSet &seqs, const std::string paramFileName, const bool distortionInvalid);
    void Run(SequenceSet &seqs, const std::string outputFileName = "", const std::string tmpFileNameKF = "", const std::string tmpFileNameRelativePoses = "",
             const std::string tmpFileNameLocal = "", const std::string tmpFileNameGlobal = "", const std::string tmpFileNameBA = "", FILE *fpTiming = NULL);

  public:

    TrackIndex m_kfMinNumCmnTrksTwoSeqs;
    FeatureIndex m_kfMinNumTrksPerKeyFrm, m_kfMinNumCmnTrksTwoKeyFrms, m_kfMinNumCmnTrksThreeKeyFrms, m_kfMinNumFtrsPerBin;
    FrameIndex m_kfMinTrkLenKeyFrm;
    ubyte m_kfNumBinsX, m_kfNumBinsY;
    ushort m_sfmSeqMaxNumTransformations;
    TrackIndex m_sfmSeqInliersMinNum;
    float m_sfm3DErrThRatio;
    uint m_sfmMaxNumPlanes, m_sfmMinNumPtsPerPlane;
    bool m_sfmDensePts;
    uint m_baaelLocalMaxNumIters, m_baaelGlobalMaxNumIters;
    FeatureIndex m_baIdvMinNumTrksPerFrm;
    SequenceIndex m_baLocalMaxNumAdditionalAdjustedSeqs;
    uint m_baLocalMaxNumIters, m_baGlobalMaxNumIters;
    float m_baLocalStopMSE, m_baLocalStopRelativeReduction, m_baGlobalStopMSE, m_baGlobalStopRelativeReduction;
    ushort m_spsfmMaxNumIters;
    SegmentIndex m_spsfmSegsNumPerSeqInit, m_spsfmSegsNumPerSeqFactor, m_spsfmSegsNumPerSeqIncr;
    float m_spsfmInconsistencyWeightGradient, m_spsfmReprojErrSqThInternal;
    bool m_stop, m_view;

  protected:

    std::vector<float> m_sfmSeqInliersRatioThs, m_sfmReprojErrSqThs;

  protected:

    //////////////////////////////////////////////////////////////////////////
    // Basic
    //////////////////////////////////////////////////////////////////////////
    virtual void Initialize(const SequenceSet &seqs, const Configurator &param, const bool distortionInvalid);
    virtual void ExtractKeyFrameSequences(SequenceSet &seqs, SequenceSet &seqsKF, std::vector<FrameIndexList> &iFrmsListKF,
                                          std::vector<TrackIndexList> &iTrksListIdvKF,
                                          TrackIndexList &iTrksCmnKF);
    virtual void RegisterOriginalSequences(const SequenceSet &seqsKF, const std::vector<FrameIndexList> &iFrmsListKF,
                                           const std::vector<TrackIndexList> &iTrksListIdvKF,
                                           TrackIndexList &iTrksCmnKF, SequenceSet &seqs);
    virtual void BundleAdjustmentIndividual(SequenceSet &seqs, const std::vector<FrameIndexList> &iFrmsListFix, const uint &maxNumIters, const float &stopMSE,
                                            const float &stopRelativeReduction);
    virtual void SelectAdjustedTracks(const Sequence &seq, const FrameIndexList &iFrmsAdj, TrackIndexList &iTrksAdj, TrackIndexList &iTrksUnadj,
                                      const FeatureIndex minNumTrksPerFrm);
    virtual void AdjustPoints(const TrackIndexList &iTrks, Sequence &seq);

    //////////////////////////////////////////////////////////////////////////
    // SfM
    //////////////////////////////////////////////////////////////////////////
    class SequenceErrorLevel {
      public:
        SequenceErrorLevel() : m_outlierLevel(UCHAR_MAX), m_reprojErrLevel(UCHAR_MAX) {}
        inline void Initialize() {
            m_outlierLevel = m_reprojErrLevel = 0;
        }
        inline bool Increase() {
            if(IsHighest())
                return false;
            else if(GetOutlierLevelRatio() < GetReprojectionErrorLevelRatio())
                ++m_outlierLevel;
            else
                ++m_reprojErrLevel;
            return true;
        }
        inline bool Decrease() {
            if(IsLowest())
                return false;
            else if(GetOutlierLevelRatio() > GetReprojectionErrorLevelRatio())
                --m_outlierLevel;
            else
                --m_reprojErrLevel;
            return true;
        }
        inline bool IncreaseOutlierLevel() {
            if(IsOutlierLevelHighest())
                return false;
            ++m_outlierLevel;
            return true;
        }
        inline bool IncreaseReprojectionErrorLevel() {
            if(IsReprojectionErrorLevelHighest())
                return false;
            ++m_reprojErrLevel;
            return true;
        }
        inline bool DecreaseOutlierErrorLevel() {
            if(IsOutlierLevelLowest())
                return false;
            --m_outlierLevel;
            return true;
        }
        inline bool DecreaseReprojectionErrorLevel() {
            if(IsReprojectionErrorLevelLowest())
                return false;
            --m_reprojErrLevel;
            return true;
        }
        inline void SetOutlierLevelLowest() {
            m_outlierLevel = 0;
        }
        inline void SetReprojectionErrorLevelLowest() {
            m_reprojErrLevel = 0;
        }
        inline const ubyte &GetOutlierLevel() const {
            return m_outlierLevel;
        }
        inline const ubyte &GetReprojectionErrorLevel() const {
            return m_reprojErrLevel;
        }
        inline float GetOutlierLevelRatio() const {
            return float(m_outlierLevel) / g_outlierLevelHighest;
        }
        inline float GetReprojectionErrorLevelRatio() const {
            return float(m_reprojErrLevel) / g_reprojErrLevelHighest;
        }
        inline bool IsLowest() const {
            return m_outlierLevel == 0 && m_reprojErrLevel == 0;
        }
        inline bool IsHighest() const {
            return m_outlierLevel == g_outlierLevelHighest && m_reprojErrLevel == g_reprojErrLevelHighest;
        }
        inline bool IsOutlierLevelLowest() const {
            return m_outlierLevel == 0;
        }
        inline bool IsOutlierLevelHighest() const {
            return m_outlierLevel == g_outlierLevelHighest;
        }
        inline bool IsReprojectionErrorLevelLowest() const {
            return m_reprojErrLevel == 0;
        }
        inline bool IsReprojectionErrorLevelHighest() const {
            return m_reprojErrLevel == g_reprojErrLevelHighest;
        }
        inline bool operator == (const SequenceErrorLevel &seqErrLevel) const {
            return m_outlierLevel == seqErrLevel.m_outlierLevel && m_reprojErrLevel == seqErrLevel.m_reprojErrLevel;
        }
        inline bool operator != (const SequenceErrorLevel &seqErrLevel) const {
            return m_outlierLevel != seqErrLevel.m_outlierLevel || m_reprojErrLevel != seqErrLevel.m_reprojErrLevel;
        }
      private:
        ubyte m_outlierLevel, m_reprojErrLevel;
      public:
        inline static void SetHighestOutlierLevel(const ubyte &outlierLevelHighest) {
            g_outlierLevelHighest = outlierLevelHighest;
        }
        inline static void SetHighestReprojectionErrorLevel(const ubyte &reprojErrLevelHighest) {
            g_reprojErrLevelHighest = reprojErrLevelHighest;
        }
      private:
        static ubyte g_outlierLevelHighest, g_reprojErrLevelHighest;
    };
    virtual void SelectInitialSequence(const SequenceSet &seqs, SequenceIndex &iSeqInit);
    void GetIncrementalSequence(const SequenceSet &seqs, SequenceIndex &iSeqIncr, TrackIndex &nTrksCmnIncr);
    virtual void RegisterSequence_MarkCommonOutliers(const SequenceIndex &iSeq, SequenceSet &seqs, SequenceErrorLevel &seqErrLevel);
    template<BA_TEMPLATE_PARAMETER>
    void RemoveOutlierData(BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> &data, TrackIndexList &iTrksCmn);
    template<GTO_TEMPLATE_PARAMETER>
    void RemoveOutlierData(GlobalTransformationOptimizerDataTemplate<GTO_TEMPLATE_ARGUMENT> &data);
    void EstimateRelativePoses(const SequenceSet &seqs, std::vector<AlignedVector<RigidTransformation3D> > &TsList);
    void BundleAdjust3D(const SequenceIndexList &iSeqsAdj, SequenceIndexList &iSeqsBA, TrackIndexList &iTrksCmnBA, SequenceSet &seqs);
    void BundleAdjust2DLocal(const std::vector<AlignedVector<RigidTransformation3D> > &TsList, const SequenceIndex &iSeqFix, const SequenceIndexList &iSeqsSeed,
                             SequenceIndexList &iSeqsAdj, SequenceIndexList &iSeqsBA, TrackIndexList &iTrksCmnBA, SequenceTrackIndexList &iSeqTrksIdvBA, SequenceSet &seqs);
    void BundleAdjust2DGlobal(const std::vector<AlignedVector<RigidTransformation3D> > &TsList, const SequenceIndex &iSeqFix, SequenceIndexList &iSeqsAdj,
                              SequenceIndexList &iSeqsBA, TrackIndexList &iTrksCmnBA, SequenceTrackIndexList &iSeqTrksIdvBA, SequenceSet &seqs);
    virtual void RunBundleAdjustment2D(const SequenceIndexList &iSeqsAdj, SequenceIndexList &iSeqsBA, TrackIndexList &iTrksCmnBA,
                                       SequenceTrackIndexList &iSeqTrksIdvBA, SequenceSet &seqs, BAResult &res, bool &toInternal, const uint &maxNumIters, const float &stopMSE,
                                       const float &stopRelativeReduction);
    void BundleAdjust2DAdaptiveErrorLevelLocal(const std::vector<AlignedVector<RigidTransformation3D> > &TsList, const SequenceIndex &iSeqFix,
            const SequenceIndexList &iSeqsSeed, SequenceSet &seqs, std::vector<SequenceErrorLevel> &seqErrLevels);
    void BundleAdjust2DAdaptiveErrorLevelGlobal(const std::vector<AlignedVector<RigidTransformation3D> > &TsList, const SequenceIndex &iSeqFix,
            SequenceSet &seqs, std::vector<SequenceErrorLevel> &seqErrLevels);
    virtual bool UpdateStructureAndInlierStates(const std::vector<bool> &cmnTrkMarks, SequenceSet &seqs, const std::vector<SequenceErrorLevel> &seqErrLevels);
    void DecreaseOutlierLevels_UpdateReprojectionErrorLevels(const SequenceSet &seqs, const SequenceIndexList &iSeqs, SequenceIndexList &iSeqsUpd,
            SequenceIndexList &iSeqsErr, std::vector<SequenceErrorLevel> &seqErrLevels);
    void DecreaseReprojectionErrorLevels_UpdateOutlierLevels(const SequenceSet &seqs, const SequenceIndexList &iSeqs, SequenceIndexList &iSeqsUpd,
            SequenceIndexList &iSeqsErr, std::vector<SequenceErrorLevel> &seqErrLevels);

    //////////////////////////////////////////////////////////////////////////
    // Segmentation
    //////////////////////////////////////////////////////////////////////////
    void SegmentSequence(const AlignedVector<RigidTransformation3D> &Ts, const SequenceIndex &iSeq, SequenceSet &seqs, const SegmentIndex &nSegs);
    class CandidateSplitPoint;
    void EstimateConsecutiveFrameInconsistencies_Gradient(const SequenceSet &seqs, const SequenceIndex &iSeq, std::vector<float> &inconsistencies);
    void EstimateConsecutiveFrameInconsistencies_Structure(const SequenceSet &seqs, const SequenceIndex &iSeq, const AlignedVector<RigidTransformation3D> &Ts,
            std::vector<float> &inconsistencies);

    //////////////////////////////////////////////////////////////////////////
    // IO
    //////////////////////////////////////////////////////////////////////////
    virtual void ViewSequences(const SequenceSet &seqs, const SequenceIndex &iSeq);
    virtual bool OnKeyDown(const int key);
    bool SaveRelativePoses(const char *fileName, const std::vector<AlignedVector<RigidTransformation3D> > &TsList);
    bool LoadRelativePoses(const char *fileName, std::vector<AlignedVector<RigidTransformation3D> > &TsList);
    virtual bool SaveB(const char *fileName, const SequenceSet &seqs, const SequenceSet &seqsKF, const std::vector<FrameIndexList> &iFrmsListKF,
                       const std::vector<TrackIndexList> &iTrksListIdvKF, const TrackIndexList &iTrksCmnKF);
    virtual bool LoadB(const char *fileName, SequenceSet &seqs, SequenceSet &seqsKF, std::vector<FrameIndexList> &iFrmsListKF,
                       std::vector<TrackIndexList> &iTrksListIdvKF, TrackIndexList &iTrksCmnKF);
    virtual bool SaveB(const char *fileName, const SequenceSet &seqs, const SegmentIndex &nSegsPerSeq);
    virtual bool LoadB(const char *fileName, SequenceSet &seqs, SequenceIndex &nSegsPerSeq);

  protected:

    SequenceSet m_seqsKF;
    std::vector<FrameIndexList> m_iFrmsListKF;
    std::vector<TrackIndexList> m_iTrksListIdvKF;
    TrackIndexList m_iTrksCmnKF;
    std::vector<SequenceErrorLevel> m_seqErrLevels;

    SimilarityTransformationEstimator2D         m_Sestor;
    SimilarityTransformationEstimatorData2D     m_Sdata;
    SimilarityTransformation3D                  m_S;
    AlignedVector<SimilarityTransformation3D>   m_SList;

    TranslationScaleEstimator       m_TSestor;
    TranslationScaleEstimatorData   m_TSdata;
    FeatureMatchList m_matches;

    CameraEstimator         m_Cestor;
    CameraEstimatorData     m_Cdata;
    Camera                  m_C;

    Point3DEstimator        m_Xestor;
    Point3DEstimatorData    m_Xdata, m_XdataTmp;
    Point3D                 m_Xcmn, m_Xidv;

    BundleAdjustorTemplate<SEQUENCE_SET_BA_ARGUMENT_3D_SIMILARITY> m_ba3D;
    SequenceSetBundleAdjustorData3DSimilarity m_baData3D;
    BundleAdjustorTemplate<SEGMENT_SET_BA_ARGUMENT_2D_SIMILARITY> m_ba2D;
    SegmentSetBundleAdjustorData2DSimilarity m_baData2D;
    GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT_SIMILARITY> m_gto;
    SequenceTransformationOptimizerDataSimilarity m_gtoData;
    BundleAdjustorTemplate<SEQUENCE_BA_ARGUMENT> m_ba;
    SequenceBundleAdjustorData m_baData;
    BundleAdjustorTemplate<SEQUENCE_BA_ARGUMENT_INTRINSIC_VARIABLE> m_baIntrinsicVariable;
    SequenceBundleAdjustorDataIntrinsicVariable m_baDataIntrinsicVariable;

    class CandidateSequence {
      public:
        CandidateSequence() {}
        CandidateSequence(const SequenceIndex &iSeq, const TrackIndex &nTrksCmn) : m_iSeq(iSeq), m_nTrksCmn(nTrksCmn) {}
        inline void Set(const SequenceIndex &iSeq, const TrackIndex &nTrksCmn) {
            m_iSeq = iSeq;
            m_nTrksCmn = nTrksCmn;
        }
        inline const SequenceIndex &GetSequenceIndex() const {
            return m_iSeq;
        }
        inline const TrackIndex &GetCommonTracksNumber() const {
            return m_nTrksCmn;
        }
        inline bool operator < (const CandidateSequence &seq) const {
            return m_nTrksCmn > seq.m_nTrksCmn;
        }
      private:
        SequenceIndex m_iSeq;
        TrackIndex m_nTrksCmn;
    };
    std::vector<CandidateSequence> m_candidateSeqs;

    class CandidateCommonTrack {
      public:
        CandidateCommonTrack() {}
        CandidateCommonTrack(const TrackIndex &iTrkCmn, const SequenceIndex &nCrspsTrk, const SequenceIndex &nCrspsInlierTrk, const FrameIndex &nCrspsInlierMea)
            : m_iTrkCmn(iTrkCmn), m_nCrspsTrk(nCrspsTrk), m_nCrspsInlierTrk(nCrspsInlierTrk), m_nCrspsInlierMea(nCrspsInlierMea) {}
        inline const TrackIndex &GetCommonTrackIndex() const {
            return m_iTrkCmn;
        }
        inline void Set(const TrackIndex &iTrkCmn, const SequenceIndex &nCrspsTrk, const SequenceIndex &nCrspsInlierTrk, const FrameIndex &nCrspsInlierMea) {
            m_iTrkCmn = iTrkCmn;
            m_nCrspsTrk = nCrspsTrk;
            m_nCrspsInlierTrk = nCrspsInlierTrk;
            m_nCrspsInlierMea = nCrspsInlierMea;
        }
        inline bool operator < (const CandidateCommonTrack &trk) const {
            return m_nCrspsInlierTrk > trk.m_nCrspsInlierTrk || m_nCrspsInlierTrk == trk.m_nCrspsInlierTrk && m_nCrspsTrk < trk.m_nCrspsTrk
                   || m_nCrspsInlierTrk == trk.m_nCrspsInlierTrk && m_nCrspsTrk == trk.m_nCrspsTrk && m_nCrspsInlierMea > trk.m_nCrspsInlierMea;
        }
      private:
        TrackIndex m_iTrkCmn;
        FrameIndex m_nCrspsTrk, m_nCrspsInlierTrk, m_nCrspsInlierMea;
    };

    class CandidateIndividualTrack {
      public:
        CandidateIndividualTrack() {}
        CandidateIndividualTrack(const TrackIndex &iTrkIdv, const FrameIndex &nCrsps, const FrameIndex &nCrspsInlier, const SequenceIndex &nCrspsInlierCmnTrk)
            : m_iTrkIdv(iTrkIdv), m_nCrsps(nCrsps), m_nCrspsInlier(nCrspsInlier), m_nCrspsInlierCmnTrk(nCrspsInlierCmnTrk) {}
        inline const TrackIndex &GetIndividualTrackIndex() const {
            return m_iTrkIdv;
        }
        inline void Set(const TrackIndex &iTrkIdv, const FrameIndex &nCrsps, const FrameIndex &nCrspsInlier, const FrameIndex &nCrspsInlierCmnTrk) {
            m_iTrkIdv = iTrkIdv;
            m_nCrsps = nCrsps;
            m_nCrspsInlier = nCrspsInlier;
            m_nCrspsInlierCmnTrk = nCrspsInlierCmnTrk;
        }
        inline bool operator < (const CandidateIndividualTrack &trk) const {
            return m_nCrspsInlier > trk.m_nCrspsInlier || m_nCrspsInlier == trk.m_nCrspsInlier && m_nCrsps < trk.m_nCrsps
                   || m_nCrspsInlier == trk.m_nCrspsInlier && m_nCrsps == trk.m_nCrsps && m_nCrspsInlierCmnTrk > trk.m_nCrspsInlierCmnTrk;
        }
      private:
        TrackIndex m_iTrkIdv;
        FrameIndex m_nCrsps, m_nCrspsInlier;
        SequenceIndex m_nCrspsInlierCmnTrk;
    };

    class CandidateSplitPoint {
      public:
        CandidateSplitPoint() {}
        CandidateSplitPoint(const FrameIndex &iFrm, const float &inconsistency) : m_iFrm(iFrm), m_inconsistency(inconsistency) {}
        inline void Set(const FrameIndex &iFrm, const float &inconsistency) {
            m_iFrm = iFrm;
            m_inconsistency = inconsistency;
        }
        inline const FrameIndex &GetFrameIndex() const {
            return m_iFrm;
        }
        inline const float &GetInconsistency() const {
            return m_inconsistency;
        }
        inline bool operator < (const CandidateSplitPoint &splitPt) const {
            return m_inconsistency > splitPt.m_inconsistency;
        }
      private:
        FrameIndex m_iFrm;
        float m_inconsistency;
    };
    std::vector<CandidateSplitPoint> m_candidateSplitPts;

    class CandidateTrack {
      public:
        CandidateTrack() {}
        CandidateTrack(const TrackIndex &iTrk, const float &rayAngleDot) : m_iTrk(iTrk), m_rayAngleDot(rayAngleDot) {}
        inline const TrackIndex &GetTrackIndex() const {
            return m_iTrk;
        }
        inline const float &GetRayAngleDot() const {
            return m_rayAngleDot;
        }
        inline void Set(const TrackIndex &iTrk, const float &rayAngleDot) {
            m_iTrk = iTrk;
            m_rayAngleDot = rayAngleDot;
        }
        inline bool operator < (const CandidateTrack &trk) const {
            return m_rayAngleDot < trk.m_rayAngleDot;
        }
      private:
        TrackIndex m_iTrk;
        float m_rayAngleDot;
    };
    std::vector<CandidateTrack> m_candidateTrks;

    SequenceIndexList m_iSeqsAdj, m_iSeqsBA, m_iSeqsUpd, m_iSeqsErr;
    FrameIndexList m_iFrmsAdj, m_iFrmsBA;
    TrackIndexList m_iTrksCmnBA, m_iTrksCmn, m_iTrksIdv, m_iTrksBA, m_iPtsOriToNew, m_iMeasOriToNew;
    SequenceTrackIndexList m_iSeqTrksIdvBA, m_iSeqTrksIdv;
    SequenceFrameIndexList m_iSeqFrms;
    FeatureIndexList m_frmTrkCnts;
    std::vector<uint> m_inliers;
    std::vector<std::vector<uint> > m_inliersList;
    std::vector<bool> m_inlierMarks, m_seqMarks, m_frmMarks, m_trkMarks, m_cmnTrkMarks;
    std::vector<float> m_inconsistenciesGradient, m_inconsistenciesStructure, m_errSqs, m_errSqsSort, m_errSqThs;
};

}//namespace ENFT_SfM

#endif