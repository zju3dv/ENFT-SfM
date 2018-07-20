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

#ifndef _CAMERA_TRACKER_H_
#define _CAMERA_TRACKER_H_

#include "Viewing/ViewerSequence.h"
#include "Utility/Configurator.h"
#include "SfM/AbsoluteQuadricEstimator.h"
#include "SfM/CameraEstimator.h"
#include "SfM/ProjectiveMatrixEstimator.h"
#include "SfM/FundamentalMatrixEstimator.h"
#include "SfM/EssentialMatrixEstimator.h"
#include "SfM/HomographyEstimator.h"
#include "SfM/Point3DEstimator.h"
#include "Optimization/BundleAdjustor.h"
#include "Sequence/SequenceBundleAdjustorData.h"
#include "Sequence/SequenceBundleAdjustorDataProjective.h"
#include "Sequence/SequenceBundleAdjustorDataIntrinsicVariable.h"

//#define VERBOSE_CAMERA_TRACKING   0
#define VERBOSE_CAMERA_TRACKING     1
#define KEY_STOP_CAMERA_TRACKING    19  // Ctrl + s

namespace ENFT_SfM {

class CameraTracker : protected ViewerSequence {

  public:

    void Initialize(const Sequence &seq, const std::string paramFileName, const bool distortionInvalid);
    void Run(Sequence &seq, const std::string outputFileName = "", FILE *fpTiming = NULL);

    void RunBundleAdjustmentLocal(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndexList &iFrmsSeed, FrameIndexList &iFrmsAdj,
                                  FrameIndexList &iFrmsBA, TrackIndexList &iTrksAdj, Sequence &seq, const bool metric);
    void RunBundleAdjustmentGlobal(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FrameIndexList &iFrmsAdj, FrameIndexList &iFrmsBA,
                                   TrackIndexList &iTrksAdj, Sequence &seq, const bool metric);
    virtual void RunPointsOptimization(const TrackIndexList &iTrks, Sequence &seq, const bool metric);

  public:

    // Parameters
    FeatureIndex m_kfMinNumTrksPerKeyFrm, m_kfMinNumCmnTrksTwoKeyFrms, m_kfMinNumCmnTrksThreeKeyFrms, m_kfMinNumFtrsPerBin;
    FrameIndex m_kfMinTrkLenKeyFrm;
    ubyte m_kfNumBinsX, m_kfNumBinsY;
    FrameIndex m_sfmInitScoreFilterSigma;
    float m_sfmInitHomoRatio;
    FeatureIndex m_sfmIncrFrmTrksMinNum, m_sfmCamInliersMinNum;
    FrameIndex m_sfmPtInliersMinNum;
    ushort m_sfmTwoViewInliersMinNum;
    float m_sfmTwoViewInliersMinRatio, m_sfmTwoViewHomoRatioTh, m_sfmTwoViewScale, m_sfmTwoViewRayAngleDotTh, m_sfmMultiViewRayAngleDotTh, m_sfmFocalPriorWeight;
    float m_sfmPtInliersMinRatio, m_sfmIncrFrmTrksMinNumInitialRatio, m_sfmIncrFrmTrksMinToMaxRatio, m_sfmFocalRangeFactor;
    bool m_sfmTwoViewMotionInitialization, m_sfmDensePts;
    uint m_baaelLocalMaxNumIters, m_baaelGlobalMaxNumIters;
    FrameIndex m_baLocalMaxNumAdditionalAdjustedFrms;
    FeatureIndex m_baLocalMinNumTrksPerFrm, m_baGlobalMinNumTrksPerFrm;
    uint m_baLocalMaxNumIters, m_baGlobalMaxNumIters;
    float m_baLocalStopMSE, m_baLocalStopRelativeReduction, m_baGlobalStopMSE, m_baGlobalStopRelativeReduction;
    bool m_stop, m_view;
    //FrameIndex m_idx;

  protected:

    std::vector<float> m_sfmCamInlierRatioThs/*, m_sfmCamInlierAreaRatioThs*/, m_sfmReprojErrSqThs;

  protected:

    //////////////////////////////////////////////////////////////////////////
    // Basic
    //////////////////////////////////////////////////////////////////////////
    void Initialize(const Sequence &seq, const Configurator &param, const bool distortionInvalid);
    virtual void ExtractKeyFrameSequence(Sequence &seq, Sequence &seqKF, FrameIndexList &iFrmsKF, TrackIndexList &iTrksKF, MeasurementIndexList &iMeasKF);
    void SelectInitialFrames(Sequence &seq, FrameIndex &iFrm1, FrameIndex &iFrm2, FrameIndex &iFrm3, const bool metric);
    void SelectInitialFrames(const std::vector<float> &frmTripleScores, FrameIndex &iFrm1, FrameIndex &iFrm2, FrameIndex &iFrm3);
    void SelectIncrementalFrames(const Sequence &seq, FrameIndexList &iFrmsIncr, const bool metric);
    class FrameErrorLevel {
      public:
        FrameErrorLevel() : m_outlierLevel(UCHAR_MAX), m_reprojErrLevel(UCHAR_MAX) {}
        inline void SetLowest() {
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
        inline void SetReprojectionErrorLevelHighest() {
            m_reprojErrLevel = g_reprojErrLevelHighest;
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
        inline bool operator == (const FrameErrorLevel &frmErrLevel) const {
            return m_outlierLevel == frmErrLevel.m_outlierLevel && m_reprojErrLevel == frmErrLevel.m_reprojErrLevel;
        }
        inline bool operator != (const FrameErrorLevel &frmErrLevel) const {
            return m_outlierLevel != frmErrLevel.m_outlierLevel || m_reprojErrLevel != frmErrLevel.m_reprojErrLevel;
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
    void RegisterKeyFrameSequence(Sequence &seq, std::vector<FrameErrorLevel> &frmErrLevels, const FrameIndex &iFrm1, const FrameIndex &iFrm2, const bool metric);
    void RegisterOriginalSequence(const Sequence &seqKF, const std::vector<FrameErrorLevel> &frmErrLevelsKF, const FrameIndex &iFrm1KF,
                                  const FrameIndex &iFrm2KF, const FrameIndexList &iFrmsKF, const TrackIndexList &iTrksKF, const MeasurementIndexList &iMeasKF, Sequence &seq);

    //////////////////////////////////////////////////////////////////////////
    // SfM
    //////////////////////////////////////////////////////////////////////////
    bool EstimateInitialStructureAndMotion(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3, Sequence &seq, const bool metric);
    float ComputeImageBasedDistance(const Sequence &seq, const FrameIndex &iFrm1, const FrameIndex &iFrm2);
    void EstimateIncrementalMotion(const FrameIndexList &iFrmsIncr, FrameIndexList &iFrmsSolve, Sequence &seq, std::vector<FrameErrorLevel> &frmErrLevels,
                                   const bool metric);
    virtual bool EstimateIncrementalMotion_PnP(const FrameIndex &iFrm, Sequence &seq, FrameErrorLevel &frmErrLevel, const bool metric);
    void UpdateProjectiveToMetric(const FrameIndex &iFrm1, const FrameIndex &iFrm2, Sequence &seq, std::vector<FrameErrorLevel> &frmErrLevels);
    virtual void TransformScene(const FrameIndex &iFrm1, const FrameIndex &iFrm2, Sequence &seq);
    void SelectAdjustedTracks(const Sequence &seq, const FrameIndexList &iFrmsAdj, TrackIndexList &iTrksAdj, TrackIndexList &iTrksUnadj,
                              const FeatureIndex minNumTrksPerFrm,    const bool metric);
    virtual void BundleAdjust(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, FrameIndexList &iFrmsBA, Sequence &seq, const uint &maxNumIters,
                              const float &stopMSE, const float &stopRelativeReduction, const bool metric);
    void BundleAdjustAdaptiveErrorLevelLocal(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndexList &iFrmsSeed, Sequence &seq,
            std::vector<FrameErrorLevel> &frmErrLevels, const bool metric, const bool triangulateUnstablePts);
    void BundleAdjustAdaptiveErrorLevelGlobal(const FrameIndex &iFrm1, const FrameIndex &iFrm2, Sequence &seq, std::vector<FrameErrorLevel> &frmErrLevels,
            const bool metric, const bool triangulateUnstablePts);
    virtual bool UpdateStructureAndInlierStates(const std::vector<bool> &trkMarks, Sequence &seq, const std::vector<FrameErrorLevel> &frmErrLevels,
            const bool metric, const bool triangulateUnstablePts);
    float ComputeMinimalRayAngleDot(const Point3DEstimatorData &data);
    float ComputeMinimalRayAngleDot_RankTriangulationPairs(const Point3DEstimatorData &data, std::vector<Match<ushort> > &triPairs);
    void DecreaseOutlierLevels_UpdateReprojectionErrorLevels(const Sequence &seq, const FrameIndexList &iFrms, FrameIndexList &iFrmsUpd, FrameIndexList &iFrmsErr,
            std::vector<FrameErrorLevel> &frmErrLevels, const bool metric);
    void DecreaseReprojectionErrorLevels_UpdateOutlierLevels(const Sequence &seq, const FrameIndexList &iFrms, FrameIndexList &iFrmsUpd, FrameIndexList &iFrmsErr,
            std::vector<FrameErrorLevel> &frmErrLevels, const bool metric);

    //////////////////////////////////////////////////////////////////////////
    // IO
    //////////////////////////////////////////////////////////////////////////
    virtual void ViewSequence(const Sequence &seq, const FrameIndex &iFrm);
    virtual bool OnKeyDown(const int key);
    void SaveB(const char *fileName, const Sequence &seq, const Sequence &seqKF, const FrameIndexList &iFrmsKF, const TrackIndexList &iTrksKF,
               const MeasurementIndexList &iMeasKF, const std::vector<FrameErrorLevel> &frmErrLevels);
    void SaveB(const char *fileName, const Sequence &seq, const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3);
    void SaveB(const char *fileName, const Sequence &seq/*, const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3*/,
               const std::vector<FrameErrorLevel> &frmErrLevels);
    void LoadB(const char *fileName, Sequence &seq, Sequence &seqKF, FrameIndexList &iFrmsKF, TrackIndexList &iTrksKF, MeasurementIndexList &iMeasKF,
               std::vector<FrameErrorLevel> &frmErrLevels);
    void LoadB(const char *fileName, Sequence &seq, FrameIndex &iFrm1, FrameIndex &iFrm2, FrameIndex &iFrm3);
    void LoadB(const char *fileName, Sequence &seq/*, FrameIndex &iFrm1, FrameIndex &iFrm2, FrameIndex &iFrm3*/, std::vector<FrameErrorLevel> &frmErrLevels);

  protected:

#if _DEBUG
    Camera::IntrinsicParameter m_KrBkp;
    AlignedVector<Camera::IntrinsicParameter> m_KrsBkp;
    AlignedVector<Camera> m_CsBkp;
    AlignedVector<ProjectiveMatrix> m_PsBkp;
    AlignedVector<Point3D> m_XsBkp;
#endif

    AbsoluteQuadric                     m_Q;
    AbsoluteQuadricEstimator            m_Qestor;
    FundamentalMatrix                   m_F;
    FundamentalMatrixEstimator          m_Festor;
    FundamentalMatrixEstimatorData      m_Fdata;
    EssentialMatrix                     m_E;
    EssentialMatrixEstimator            m_Eestor;
    EssentialMatrixEstimatorData        m_Edata;
    Homography                          m_H;
    HomographyEstimator                 m_Hestor;
    HomographyEstimatorData             m_Hdata;
    RigidTransformation3D               m_T;
    Camera                              m_C;
    CameraEstimator                     m_Cestor;
    CameraEstimatorData                 m_Cdata;
    ProjectiveMatrix                    m_P;
    ProjectiveMatrixMetric              m_PMetric;
    AlignedVector<ProjectiveMatrix>     m_Ps;
    ProjectiveMatrixEstimator           m_Pestor;
    ProjectiveMatrixEstimatorData       m_Pdata;
    ProjectiveMatrixEstimatorDataMetric m_PdataMetric;

    Point3D                 m_X;
    Point3DEstimator        m_Xestor;
    Point3DEstimatorData    m_Xdata;
    AlignedVector<Point3D>  m_Xs;

    Sequence m_seqKF;
    BundleAdjustorTemplate<SEQUENCE_BA_ARGUMENT_PROJECTIVE> m_baProjective;
    SequenceBundleAdjustorDataProjective m_baDataProjective;
    BundleAdjustorTemplate<SEQUENCE_BA_ARGUMENT> m_baMetric;
    SequenceBundleAdjustorData m_baDataMetric;
    BundleAdjustorTemplate<SEQUENCE_BA_ARGUMENT_INTRINSIC_VARIABLE> m_baMetricIntrinsicVariable;
    SequenceBundleAdjustorDataIntrinsicVariable m_baDataMetricIntrinsicVariable;

    class CandidateFrame {
      public:
        CandidateFrame() {}
        CandidateFrame(const FrameIndex &iFrm, const FeatureIndex &nTrksInlier) : m_iFrm(iFrm), m_nTrksInlier(nTrksInlier) {}
        inline void Set(const FrameIndex &iFrm, const FeatureIndex &nTrksInlier) {
            m_iFrm = iFrm;
            m_nTrksInlier = nTrksInlier;
        }
        inline const FrameIndex &GetFrameIndex() const {
            return m_iFrm;
        }
        inline const FeatureIndex &GetInlierTracksNumber() const {
            return m_nTrksInlier;
        }
        inline bool operator < (const CandidateFrame &frm) const {
            return m_nTrksInlier > frm.m_nTrksInlier;
        }
      private:
        FrameIndex m_iFrm;
        FeatureIndex m_nTrksInlier;
    };
    std::vector<CandidateFrame> m_candidateFrms;

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

    class CandidateTriangulationPair : public Match<ushort> {
      public:
        CandidateTriangulationPair() {}
        CandidateTriangulationPair(const ushort &idx1, const ushort &idx2, const float &rayAngleDot) : Match<ushort>(idx1, idx2), m_rayAngleDot(rayAngleDot) {}
        inline const float &GetRayAngleDot() const {
            return m_rayAngleDot;
        }
        inline bool operator < (const CandidateTriangulationPair &triPair) const {
            return m_rayAngleDot < triPair.m_rayAngleDot;
        }
      private:
        float m_rayAngleDot;
    };
    std::vector<CandidateTriangulationPair> m_candidateTriPairs;

    FeatureMatchList m_matches;
    FrameIndexList m_iFrmsKF, m_iFrmsIncr, m_iFrmsSolve, m_iFrmsAdj, m_iFrmsBA, m_iFrmsUpd, m_iFrmsErr;
    TrackIndexList m_iTrksKF/*, m_iTrks*/, m_iTrksAdj, m_iTrksUnadj;
    MeasurementIndexList m_iMeasKF, m_iMeas, m_iMeasInlier;
    std::vector<FrameErrorLevel> m_frmErrLevelsKF, m_frmErrLevels;
    std::vector<float> m_frmTripleScores, m_frmTripleScoresFilter, m_fs;
    std::vector<ushort> m_inliers;
    FeatureIndexList m_iFtrs;
    std::vector<bool> m_inlierMarks, m_frmMarks, m_trkMarks;
    //std::vector<float> m_frmErrSqThs, m_errSqThs;
    FeatureIndexList m_frmTrkCnts;
    std::vector<float> m_errSqs;
    AlignedVector<Point3D> m_rayDirs;
    //std::vector<Match<ushort> > m_triPairs;
    ENFT_SSE::__m128 m_work[38];

    AlignedVector<Point2D> m_xrs;

};

}//namespace ENFT_SfM

#endif
