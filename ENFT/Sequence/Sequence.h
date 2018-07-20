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

#ifndef _SEQUENCE_H_
#define _SEQUENCE_H_

#include "SequenceTag.h"
#include "SfM/IntrinsicMatrix.h"
#include "SfM/Camera.h"
#include "SfM/Point.h"
#include "SfM/SimilarityTransformation.h"
#include "SfM/AbsoluteQuadric.h"
#include "SfM/Match.h"
#include "Sequence/Descriptor.h"
#include "SfM/CameraEstimatorData.h"
#include "SfM/ProjectiveMatrixEstimatorData.h"
#include "SfM/Point3DEstimatorData.h"
#include <cvd/rgb.h>

#define DESCRIPTOR_TRACK    0

typedef ushort  FrameIndex;
typedef uint    TrackIndex;
typedef uint    MeasurementIndex;
typedef ushort  FeatureIndex;

typedef std::vector<FrameIndex>         FrameIndexList;
typedef std::vector<TrackIndex>         TrackIndexList;
typedef std::vector<MeasurementIndex>   MeasurementIndexList;
typedef std::vector<FeatureIndex>       FeatureIndexList;

#define INVALID_FRAME_INDEX         USHRT_MAX
#define INVALID_TRACK_INDEX         UINT_MAX
#define INVALID_MEASUREMENT_INDEX   UINT_MAX
#define INVALID_FEATURE_INDEX       USHRT_MAX
#define INVALID_DESCRIPTOR_INDEX    UINT_MAX

typedef Match<FrameIndex>           FrameIndexPair;
typedef Match<FeatureIndex>         FeatureMatch;
typedef Match<TrackIndex>           TrackMatch;
typedef std::vector<FrameIndexPair> FrameIndexPairList;
typedef std::vector<FeatureMatch>   FeatureMatchList;
typedef std::vector<TrackMatch>     TrackMatchList;

typedef std::pair<TrackIndexList, TrackIndexList> TrackIndexListPair;

#define FLAG_FRAME_STATE_DEFAULT        0
#define FLAG_FRAME_STATE_KEY_FRAME      1
#define FLAG_FRAME_STATE_INITIAL        2
#define FLAG_FRAME_STATE_SOLVED         4
#define FLAG_FRAME_STATE_FIXED          8
#define FLAG_FRAME_STATE_ADJUSTED       16
#define FLAG_FRAME_STATE_INTERPOLATED   32
#define FLAG_FRAME_STATE_SPLIT_POINT    64

#define FLAG_TRACK_STATE_DEFAULT        0
#define FLAG_TRACK_STATE_INITIAL        1
#define FLAG_TRACK_STATE_SOLVED         2
#define FLAG_TRACK_STATE_INLIER         4
#define FLAG_TRACK_STATE_MERGED         8
#define FLAG_TRACK_STATE_COMMON         16
#define FLAG_TRACK_STATE_COMMON_OUTLIER 32

#define FLAG_MEASUREMENT_STATE_DEFAULT  0
#define FLAG_MEASUREMENT_STATE_ENFT     1
#define FLAG_MEASUREMENT_STATE_OUTLIER  2

typedef ubyte FrameState;
typedef ubyte TrackState;
typedef ubyte MeasurementState;

typedef std::vector<FrameState>         FrameStateList;
typedef std::vector<TrackState>         TrackStateList;
typedef std::vector<MeasurementState>   MeasurementStateList;

class FrameFeatureIndex {
  public:
    FrameFeatureIndex() {
        Invalidate();
    }
    FrameFeatureIndex(const FrameIndex &iFrm, const FeatureIndex &iFtr) : m_iFrm(iFrm), m_iFtr(iFtr) {}
    inline const FrameIndex    &GetFrameIndex   () const {
        return m_iFrm;
    }
    inline const FeatureIndex  &GetFeatureIndex () const {
        return m_iFtr;
    }
    inline void Get(FrameIndex &iFrm, FeatureIndex &iFtr) const {
        iFrm = m_iFrm;
        iFtr = m_iFtr;
    }
    inline void SetFrameIndex   (const FrameIndex   &iFrm) {
        m_iFrm = iFrm;
    }
    inline void SetFeatureIndex (const FeatureIndex &iFtr) {
        m_iFtr = iFtr;
    }
    inline void Set(const FrameIndex &iFrm, const FeatureIndex &iFtr) {
        m_iFrm = iFrm;
        m_iFtr = iFtr;
    }
    inline void Invalidate() {
        m_iFrm = INVALID_FRAME_INDEX;
    }
    inline bool IsValid()   const {
        return m_iFrm != INVALID_FRAME_INDEX;
    }
    inline bool IsInvalid() const {
        return m_iFrm == INVALID_FRAME_INDEX;
    }
  private:
    FrameIndex      m_iFrm;
    FeatureIndex    m_iFtr;
};
typedef std::vector<FrameFeatureIndex> FrameFeatureIndexList;

class TrackFeatureIndex {
  public:
    TrackFeatureIndex() {
        Invalidate();
    }
    TrackFeatureIndex(const TrackIndex &iTrk, const FeatureIndex &iFtr) : m_iTrk(iTrk), m_iFtr(iFtr) {}
    inline const TrackIndex &GetTrackIndex() const {
        return m_iTrk;
    }
    inline const FeatureIndex  &GetFeatureIndex () const {
        return m_iFtr;
    }
    inline void Get(TrackIndex &iTrk, FeatureIndex &iFtr) const {
        iTrk = m_iTrk;
        iFtr = m_iFtr;
    }
    inline void SetTrackIndex(const TrackIndex  &iTrk) {
        m_iTrk = iTrk;
    }
    inline void SetFeatureIndex (const FeatureIndex     &iFtr) {
        m_iFtr = iFtr;
    }
    inline void Set(const TrackIndex &iTrk, const FeatureIndex &iFtr) {
        m_iTrk = iTrk;
        m_iFtr = iFtr;
    }
    inline void Invalidate() {
        m_iTrk = INVALID_TRACK_INDEX;
    }
    inline bool IsValid()   const {
        return m_iTrk != INVALID_TRACK_INDEX;
    }
    inline bool IsInvalid() const {
        return m_iTrk == INVALID_TRACK_INDEX;
    }
    inline bool operator < (const TrackFeatureIndex &iTrkFtr) const {
        return m_iTrk < iTrkFtr.m_iTrk || m_iTrk == iTrkFtr.m_iTrk && m_iFtr < iTrkFtr.m_iFtr;
    }
  private:
    TrackIndex      m_iTrk;
    FeatureIndex    m_iFtr;
};
typedef std::vector<TrackFeatureIndex> TrackFeatureIndexList;

class SequenceBundleAdjustorData;
class SequenceBundleAdjustorDataIntrinsicVariable;
class SequenceBundleAdjustorDataProjective;
class SequenceSet;

class Sequence {

  public:

    typedef MeasurementIndexList                FrameMeasurementMap;
    typedef std::vector<MeasurementIndexList>   TrackMeasurementMap;
    typedef FrameIndexList                      MeasurementFrameMap;
    typedef TrackIndexList                      MeasurementTrackMap;

    enum IntrinsicType { INTRINSIC_USER_FIXED, INTRINSIC_CONSTANT, INTRINSIC_VARIABLE };

    //////////////////////////////////////////////////////////////////////////
    // Basic
    //////////////////////////////////////////////////////////////////////////
    Sequence() : m_K(*(IntrinsicMatrix *) _aligned_malloc(sizeof(IntrinsicMatrix), SSE_ALIGNMENT)), m_intrinsicType(INTRINSIC_USER_FIXED),
        m_measNormalized(false) {}
    ~Sequence() {
        Clear();
        _aligned_free(&m_K);
    }
    inline const SequenceTag &GetTag() const {
        return m_tag;
    }
    inline void SetDirectory(const std::string &seqDir) {
        m_tag.SetDirectory(seqDir);
    }
    inline const std::string &GetDirectory() const {
        return m_tag.GetDirectory();
    }
    inline void SetImageSize(const ushort &width, const ushort &height) {
        m_tag.SetImageSize(width, height);
    }
    inline const ushort &GetImageWidth () const {
        return m_tag.GetImageWidth ();
    }
    inline const ushort &GetImageHeight() const {
        return m_tag.GetImageHeight();
    }
    inline const std::string &GetImageFileName(const FrameIndex &iFrm) const {
        return m_tag.GetImageFileName(iFrm);
    }
    inline const std::string &GetSequenceName() const {
        return m_tag.GetSequenceName();
    }
    inline const int &GetStartFrame() const {
        return m_tag.GetStartFrame();
    }
    inline const int &GetStepFrame() const {
        return m_tag.GetStepFrame();
    }
    inline const int &GetEndFrame() const {
        return m_tag.GetEndFrame();
    }
    inline void SetImageFileName(const FrameIndex &iFrm, const std::string &imgFileName) {
        m_tag.SetImageFileName(iFrm, imgFileName);
    }
    inline void PushBackImageFileName(const std::string &imgFileName) {
        m_tag.PushBackImageFileName(imgFileName);
    }
    inline void SetIntrinsicMatrix(const float &fx, const float &fy, const float &cx, const float &cy) {
        m_K.Set(fx, fy, cx, cy);
    }
    inline void SetIntrinsicMatrix(const IntrinsicMatrix &K) {
        m_K = K;
    }
    inline const IntrinsicMatrix &GetIntrinsicMatrix() const {
        return m_K;
    }
    inline void SetIntrinsicRectification(const Camera::IntrinsicParameter &Kr) {
        m_Kr = Kr;
        ComputeProjectiveMatrixes();
    }
    inline const Camera::IntrinsicParameter &GetIntrinsicRectification() const {
        return m_Kr;
    }
    inline const AlignedVector<Camera::IntrinsicParameter> &GetIntrinsicRectifications() const {
        return m_Krs;
    }
    inline const Camera::IntrinsicParameter &GetIntrinsicRectification(const FrameIndex &iFrm) const {
        return m_Krs[iFrm];
    }
    inline void SetIntrinsicType(const IntrinsicType &intrinsicType) {
        if((m_intrinsicType = intrinsicType) == INTRINSIC_VARIABLE) m_Krs.Resize(m_Cs.Size());
    }
    inline const IntrinsicType &GetIntrinsicType() const {
        return m_intrinsicType;
    }
    void operator = (const Sequence &seq);
    void Swap(Sequence &seq);
    void SetTag(const SequenceTag &tag);
    void SetTag(const std::string &seqDir, const std::string &seqName, const int &iStart, const int &iStep, const int &iEnd);
    void SetTag(const std::vector<std::string> &imgFileNames);
    void ChangeTag(const std::string &seqDir, const std::string &seqName, const int iStart, const int iStep, const int iEnd, const bool copyImgs = true);
    void SetCalib(const char *calibFileName, bool focalConst);
    void Initialize();
    void SetIntrinsicRectification(const float &f, const float &d);
    void GetIntrinsicRectificationFocalRange(float &fMin, float &fMax) const;
    void Resize(const FrameIndex &nFrms, const TrackIndex &nTrks, const MeasurementIndex &nMeas, const bool resizeCam = true, const bool resizePt = true,
                const bool resizeDesc = false);
    //void Reserve(const FrameIndex &nFrms, const TrackIndex &nTrks, const MeasurementIndex &nMeas);
    void Clear(const bool clearStates = true);

    //////////////////////////////////////////////////////////////////////////
    // Frame
    //////////////////////////////////////////////////////////////////////////
    inline FrameIndex GetFramesNumber() const {
        return FrameIndex(m_frmStates.size());
    }
    inline FrameIndex GetFramesNumberTotal() const {
        return m_tag.GetFramesNumber();
    }
    inline FrameIndex GetCamerasNumber() const {
        return FrameIndex(m_Cs.Size());
    }
    inline const AlignedVector<Camera> &GetCameras() const {
        return m_Cs;
    }
    inline const Camera &GetCamera(const FrameIndex &iFrm) const {
        return m_Cs[iFrm];
    }
    inline void SwapCameras(AlignedVector<Camera> &Cs) {
        m_Cs.Swap(Cs);
    }
    inline const AlignedVector<ProjectiveMatrix> &GetProjectiveMatrixes() const {
        return m_Ps;
    }
    inline const ProjectiveMatrix &GetProjectiveMatrix(const FrameIndex &iFrm) const {
        return m_Ps[iFrm];
    }
    inline const FrameState &GetFrameState(const FrameIndex &iFrm) const {
        return m_frmStates[iFrm];
    }
    inline ubyte IsFrameKeyFrame(const FrameIndex &iFrm) const {
        return (m_frmStates[iFrm] & FLAG_FRAME_STATE_KEY_FRAME);
    }
    inline ubyte IsFrameSolved(const FrameIndex &iFrm) const {
        return (m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED);
    }
    inline const MeasurementIndex &GetFrameFirstMeasurementIndex(const FrameIndex &iFrm) const {
        return m_mapFrmToMea[iFrm];
    }
    inline FeatureIndex GetFrameFeaturesNumber(const FrameIndex &iFrm) const {
        return m_mapFrmToMea[iFrm + 1] - m_mapFrmToMea[iFrm];
    }
    inline const Point2D *GetFrameFeatures(const FrameIndex &iFrm) const {
        return &m_xs[m_mapFrmToMea[iFrm]];
    }
    inline const Point2D &GetFrameFeature(const FrameIndex &iFrm, const FeatureIndex &iFtr) const {
        return m_xs[m_mapFrmToMea[iFrm] + iFtr];
    }
    inline const MeasurementState *GetFrameMeasurementStates(const FrameIndex &iFrm) const {
        return &m_meaStates[m_mapFrmToMea[iFrm]];
    }
    inline const TrackIndex *GetFrameTrackIndexes(const FrameIndex &iFrm) const {
        return &m_mapMeaToTrk[m_mapFrmToMea[iFrm]];
    }
    inline const TrackIndex &GetFrameFeatureTrackIndex(const FrameIndex &iFrm, const FeatureIndex &iFtr) const {
        return m_mapMeaToTrk[m_mapFrmToMea[iFrm] + iFtr];
    }
    inline MeasurementIndex GetFrameFeatureMeasurementIndex(const FrameIndex &iFrm, const FeatureIndex &iFtr) const {
        return m_mapFrmToMea[iFrm] + iFtr;
    }
#if DESCRIPTOR_TRACK == 0
    inline const Descriptor *GetFrameDescriptors(const FrameIndex &iFrm) const {
        return &m_descs[m_mapFrmToMea[iFrm]];
    }
    inline       Descriptor *GetFrameDescriptors(const FrameIndex &iFrm)       {
        return &m_descs[m_mapFrmToMea[iFrm]];
    }
#endif
    inline void SetFramesNumber() {
        const FrameIndex nFrms = m_tag.GetFramesNumber();
        m_frmStates.resize(nFrms, FLAG_FRAME_STATE_DEFAULT);
        m_mapFrmToMea.resize(nFrms + 1, 0);
    }
    //inline void SetCameraCenter(const FrameIndex &iFrm, const Point3D &center) { m_Cs[iFrm].SetCenter(center); }
    //inline void SetCameras(const AlignedVector<Camera> &Cs) { m_Cs = Cs; }
    //inline void SetCameras(const FrameIndex nFrms, const FrameIndex *iFrms, const Camera *Cs) { for(FrameIndex i = 0; i < nFrms; ++i) m_Cs[iFrms[i]] = Cs[i]; }
    inline void SetFrameState(const FrameIndex &iFrm, const FrameState &frmState) {
        m_frmStates[iFrm] = frmState;
    }
    inline void SetFrameStates(const FrameState &frmState) {
        m_frmStates.assign(GetFramesNumber(), frmState);
    }
    inline void MarkFrameKeyFrame(const FrameIndex &iFrm) {
        m_frmStates[iFrm] |= FLAG_FRAME_STATE_KEY_FRAME;
    }
    inline void UnmarkFrameKeyFrame(const FrameIndex &iFrm) {
        m_frmStates[iFrm] &= ~FLAG_FRAME_STATE_KEY_FRAME;
    }
    inline void MarkFrameInitial(const FrameIndex &iFrm) {
        m_frmStates[iFrm] |= FLAG_FRAME_STATE_INITIAL;
    }
    inline void MarkFrameSolved(const FrameIndex &iFrm) {
        m_frmStates[iFrm] |= FLAG_FRAME_STATE_SOLVED;
    }
    inline void MarkFrameSplitPoint(const FrameIndex &iFrm) {
        m_frmStates[iFrm] |= FLAG_FRAME_STATE_SPLIT_POINT;
    }
    inline void UnmarkFrameSplitPoint(const FrameIndex &iFrm) {
        m_frmStates[iFrm] &= ~FLAG_FRAME_STATE_SPLIT_POINT;
    }
    inline void MarkFrameInterpolated(const FrameIndex &iFrm) {
        m_frmStates[iFrm] |= FLAG_FRAME_STATE_INTERPOLATED;
    }
    //inline void PushBackCamera(const Camera &C) { m_Cs.PushBack(C); }
    void InitializeCameras();
    void SetCamera(const FrameIndex &iFrm, const Camera &C);
    void SetCamera(const FrameIndex &iFrm, const Camera &C, const Camera::IntrinsicParameter &Kr);
    void SetCamera(const FrameIndex &iFrm, const Camera &C, const float &f);
    void SetCamera(const FrameIndex &iFrm, const ProjectiveMatrix &P);
    void GetFrameIndexList(const FrameState frmState, FrameIndexList &iFrms) const;
    void GetFrameDescriptors(const FrameIndex &iFrm, const FeatureIndexList &iFtrs, AlignedVector<Descriptor> &descs) const;
    void GetFrameFeaturesAndDescriptors(const FrameIndex &iFrm, AlignedVector<Point2D> &ftrs, AlignedVector<Descriptor> &descs) const;
    void GetFrameFeaturesAndDescriptors(const FrameIndex &iFrm, const FeatureIndexList &iFtrs, AlignedVector<Point2D> &ftrs,
                                        AlignedVector<Descriptor> &descs) const;
    void GetCameraEstimatorData(const FrameIndex &iFrm, CameraEstimatorData &data, const bool rectData) const;
    void GetCameraEstimatorData(const FrameIndex &iFrm, CameraEstimatorData &data, MeasurementIndexList &iMeas, const bool rectData) const;
    void GetCameraEstimatorDataMarkedTrack(const FrameIndex &iFrm, CameraEstimatorData &data, const std::vector<bool> &trkMarks, const bool rectData) const;
    void GetProjectiveMatrixEstimatorData(const FrameIndex &iFrm, ProjectiveMatrixEstimatorData &data, const bool rectData) const;
    void GetProjectiveMatrixEstimatorData(const FrameIndex &iFrm, ProjectiveMatrixEstimatorData &data, MeasurementIndexList &iMeas, const bool rectData) const;
    void GetProjectiveMatrixEstimatorDataInlier(const FrameIndex &iFrm, ProjectiveMatrixEstimatorDataMetric &data, const bool rectData) const;
    void GetProjectiveMatrixEstimatorDataMarkedTrack(const FrameIndex &iFrm, ProjectiveMatrixEstimatorData &data, const std::vector<bool> &trkMarks,
            const bool rectData) const;
    FrameIndex CountFrames(const FrameState frmState) const;
    FeatureIndex CountFrameInlierTracks(const FrameIndex &iFrm) const;
    FeatureIndex CountFrameInlierInitialTracks(const FrameIndex &iFrm) const;
    FeatureIndex CountFrameMarkedTracks(const FrameIndex &iFrm, const std::vector<bool> &trkMarks) const;
    FeatureIndex CountFrameMarkedInlierTracks(const FrameIndex &iFrm, const std::vector<bool> &trkMarks) const;
    FeatureIndex CountFrameTrackedFeatures(const FrameIndex &iFrm) const;
    FeatureIndex CountFrameTrackedSiftFeatures(const FrameIndex &iFrm) const;
    FeatureIndex CountFrameSolvedFeatures(const FrameIndex &iFrm) const;
    FeatureIndex CountFrameInlierFeatures(const FrameIndex &iFrm) const;
    void GetFrameTrackedFeatureIndexList(const FrameIndex &iFrm, FeatureIndexList &iFtrs) const;
    void GetFrameSolvedFeatureIndexList(const FrameIndex &iFrm, FeatureIndexList &iFtrs) const;
    void GetFrameInlierFeatureIndexList(const FrameIndex &iFrm, FeatureIndexList &iFtrs) const;
    void GetFrameOutlierFeatureIndexList(const FrameIndex &iFrm, FeatureIndexList &iFtrs) const;
    void UnmarkFramesKeyFrame();
    void MarkFrameUnsolved(const FrameIndex &iFrm, const FrameIndex minTrkInliersNum, const float minTrkInliersRatio);
    void MarkFrameTracks(const FrameIndex &iFrm, std::vector<bool> &trkMarks) const;
    void MarkFramesTracks(const FrameIndexList &iFrms, std::vector<bool> &trkMarks) const;
    void MarkFrameInlierTracks(const FrameIndex &iFrm, std::vector<bool> &trkMarks) const;
    void MarkKeyFrames(const FeatureIndex minNumCmnTrksTwoKeyFrms, const FeatureIndex minNumCmnTrksThreeKeyFrms, std::vector<bool> &frmMarks) const;
    void MarkKeyFrameTracks(const FrameIndexList &iFrmsKey, const std::vector<bool> &frmMarksKey, const TrackIndexList &iTrksKey, const ubyte nBinsX,
                            const ubyte nBinsY, const FeatureIndex minNumFtrsPerBin, const FeatureIndex minNumTrksPerKeyFrm, const FeatureIndex minNumCmnTrksTwoKeyFrms,
                            const FeatureIndex minNumCmnTrksThreeKeyFrms, std::vector<bool> &trkMarks) const;
    FrameIndex ComputeFrameAverageTrackLength(const FrameIndex &iFrm) const;
    void PushBackFrame();
#if DESCRIPTOR_TRACK == 0
    void PushBackFrame(const Point2D *xs, const Descriptor *descs, const FeatureIndex nFtrsTotal, const FeatureIndex nFtrsSift, const bool measNormalized);
#else
    void PushBackFrame(const Point2D *xs, const Descriptor *descs, const FeatureIndex nFtrsTotal, const FeatureIndex nFtrsSift, const bool measNormalized,
                       TrackIndexList &iTrks);
    void PushBackFrame(const Point2D *xs, const FeatureIndex nFtrsTotal, const FeatureIndex nFtrsSift, const bool measNormalized);
#endif
    void PushBackFrame(const Camera &C, const FrameState &frmState, const AlignedVector<Point2D> &xs, const TrackIndexList &iTrks,
                       const MeasurementStateList &meaStates,
                       const bool measNormalized);
    void InterpolateFrames(const FrameIndex &iFrmStart, const FrameIndex &iFrmEnd, const FrameIndex &nFrmsInterpolate, FrameIndexList &iFrmsInterpolate) const;
    void RemoveMarkedFrames(const std::vector<bool> &frmMarks);
    void ComputeProjectiveMatrixes();
    bool ComputeFrameDepthRange(const FrameIndex &iFrm, const float &ratio, float &depthAvg, float &depthMin, float &depthMax) const;
    float ComputeFrameMSE(const FrameIndex &iFrm) const;
    void ComputeFrameInlierRatioAndMSE(const FrameIndex &iFrm, FeatureIndex &nInliers, float &inlierRatio, float &SSE, float &MSE) const;
    void ComputeFrameFeaturesDistribution(const FrameIndex &iFrm, const FeatureIndexList &iFtrs, Point2D &mean, LA::Vector3f &cov) const;

    //////////////////////////////////////////////////////////////////////////
    // Track
    //////////////////////////////////////////////////////////////////////////
    inline TrackIndex GetTracksNumber() const {
        return TrackIndex(m_trkStates.size());
    }
    inline TrackIndex GetPointsNumber() const {
        return TrackIndex(m_Xs.Size());
    }
    inline const AlignedVector<Point3D> &GetPoints() const {
        return m_Xs;
    }
    inline const Point3D &GetPoint(const TrackIndex &iTrk) const {
        return m_Xs[iTrk];
    }
    inline const TrackState &GetTrackState(const TrackIndex &iTrk) const {
        return m_trkStates[iTrk];
    }
    inline const TrackStateList &GetTrackStates() const {
        return m_trkStates;
    }
#if DESCRIPTOR_TRACK
    inline TrackIndex GetDescriptorsNumber() const {
        return TrackIndex(m_descs.Size());
    }
    inline const Descriptor &GetDescriptor(const TrackIndex &iTrk) const {
        return m_descs[iTrk];
    }
    inline const AlignedVector<Descriptor> &GetDescriptors() const {
        return m_descs;
    }
#endif
    inline TrackIndex GetTrackColorsNumber() const {
        return TrackIndex(m_trkClrs.size());
    }
    inline const CVD::Rgb<ubyte> &GetTrackColor(const TrackIndex &iTrk) const {
        return m_trkClrs[iTrk];
    }
    inline const std::vector<CVD::Rgb<ubyte> > &GetTrackColors() const {
        return m_trkClrs;
    }
    inline FrameIndex GetTrackLength(const TrackIndex &iTrk) const {
        return FrameIndex(m_mapTrkToMea[iTrk].size());
    }
    inline const MeasurementIndexList &GetTrackMeasurementIndexList(const TrackIndex &iTrk) const {
        return m_mapTrkToMea[iTrk];
    }
    inline const FrameIndex GetTrackFirstFrameIndex(const TrackIndex &iTrk) const {
        return m_mapTrkToMea[iTrk].empty() ? INVALID_FRAME_INDEX : m_mapMeaToFrm[m_mapTrkToMea[iTrk].front()];
    }
    inline const FrameIndex GetTrackLastFrameIndex(const TrackIndex &iTrk) const {
        return m_mapTrkToMea[iTrk].empty() ? INVALID_FRAME_INDEX : m_mapMeaToFrm[m_mapTrkToMea[iTrk].back()];
    }
    inline const MeasurementIndex GetTrackFirstMeasurementIndex(const TrackIndex &iTrk) const {
        return m_mapTrkToMea[iTrk].empty() ? INVALID_MEASUREMENT_INDEX : m_mapTrkToMea[iTrk].front();
    }
    inline const MeasurementIndex GetTrackLastMeasurementIndex(const TrackIndex &iTrk) const {
        return m_mapTrkToMea[iTrk].empty() ? INVALID_MEASUREMENT_INDEX : m_mapTrkToMea[iTrk].back();
    }
    inline void MarkTrackInitial        (const TrackIndex &iTrk) {
        m_trkStates[iTrk] |= FLAG_TRACK_STATE_INITIAL;
    }
    inline void MarkTrackSolved         (const TrackIndex &iTrk) {
        m_trkStates[iTrk] |= FLAG_TRACK_STATE_SOLVED;
    }
    inline void MarkTrackInlier         (const TrackIndex &iTrk) {
        m_trkStates[iTrk] |= FLAG_TRACK_STATE_INLIER;
    }
    inline void MarkTrackSolvedAndInlier(const TrackIndex &iTrk) {
        m_trkStates[iTrk] |= (FLAG_TRACK_STATE_SOLVED | FLAG_TRACK_STATE_INLIER);
    }
    inline void MarkTrackMerged         (const TrackIndex &iTrk) {
        m_trkStates[iTrk] |= FLAG_TRACK_STATE_MERGED;
    }
    inline void MarkTrackCommon         (const TrackIndex &iTrk) {
        m_trkStates[iTrk] |= FLAG_TRACK_STATE_COMMON;
    }
    inline void MarkTrackIndividual     (const TrackIndex &iTrk) {
        m_trkStates[iTrk] &= ~FLAG_TRACK_STATE_COMMON;
    }
    inline void MarkTrackCommonInlier   (const TrackIndex &iTrk) {
        m_trkStates[iTrk] &= ~FLAG_TRACK_STATE_COMMON_OUTLIER;
    }
    inline void MarkTrackCommonOutlier  (const TrackIndex &iTrk) {
        m_trkStates[iTrk] |= FLAG_TRACK_STATE_COMMON_OUTLIER;
    }
    inline void SetTracksNumber(const TrackIndex &nTrks) {
        m_mapTrkToMea.resize(nTrks, MeasurementIndexList());
        m_trkStates.resize(nTrks);
    }
    inline void SetPoint(const TrackIndex &iTrk, const Point3D &X) {
        m_Xs[iTrk] = X;
    }
    inline void SetPoints(const AlignedVector<Point3D> &Xs) {
        m_Xs.Resize(Xs.Size());
        m_Xs.CopyFrom(Xs.Data());
    }
    inline void SetTrackState(const TrackIndex &iTrk, const TrackState &trkState) {
        m_trkStates[iTrk] = trkState;
    }
    inline void SetTrackStates(const TrackStateList &trkStates) {
        m_trkStates = trkStates;
    }
#if DESCRIPTOR_TRACK
    inline void SetDescriptorsNumber(const TrackIndex &nTrks) {
        m_descs.Resize(nTrks);
    }
    inline void SetDescriptor(const TrackIndex &iTrk, const Descriptor &desc) {
        m_descs[iTrk] = desc;
    }
    inline void ClearDescriptors() {
        m_descs.Clear();
    }
#endif
    inline void SetTrackColorsNumber(const TrackIndex &nTrks) {
        m_trkClrs.resize(nTrks);
    }
    inline void SetTrackColor(const TrackIndex &iTrk, const CVD::Rgb<ubyte> &clr) {
        m_trkClrs[iTrk] = clr;
    }
    inline void PushBackTrackColor(const CVD::Rgb<ubyte> &clr) {
        m_trkClrs.push_back(clr);
    }
    inline void SetTrackColors(const std::vector<CVD::Rgb<ubyte> > &trkClrs) {
        m_trkClrs = trkClrs;
    }
    void InitializePoints();
    void GetTrackIndexList(const TrackState trkState, TrackIndexList &iTrks) const;
    void SetPoints(const TrackIndexList &iTrks, const AlignedVector<Point3D> &Xs);
    void SetTrackStates(const TrackState &trkState);
    void SetTrackStates(const TrackIndexList &iTrks, const TrackState &trkState);
    void MarkTrackOutlier(const TrackIndex &iTrk);
    void UnmarkTracksMerged();
    FrameIndex GetPoint3DEstimatorData(const TrackIndex &iTrk, Point3DEstimatorData &data, const bool rectData) const;
    FrameIndex GetPoint3DEstimatorData(const TrackIndex &iTrk, Point3DEstimatorData &data, MeasurementIndexList &iMeas, const bool rectData) const;
    FrameIndex GetPoint3DEstimatorDataInlier(const TrackIndex &iTrk, Point3DEstimatorData &data, const bool rectData) const;
    TrackIndex CountTracks(const TrackState trkState) const;
    TrackIndex CountTracks(const TrackState trkState1, const TrackState trkState2) const;
    FrameIndex CountTrackKeyFrameMeasurements(const TrackIndex &iTrk) const;
    FrameIndex CountTrackSolvedFrameMeasurements(const TrackIndex &iTrk) const;
    FrameIndex CountTrackSiftMeasurements(const TrackIndex &iTrk) const;
    FrameIndex CountTrackInlierMeasurements(const TrackIndex &iTrk) const;
    FrameIndex CountTrackMarkedFrameMeasurements(const TrackIndex &iTrk, const std::vector<bool> &frmMarks) const;
    FrameIndex CountTrackMarkedFrameInlierMeasurements(const TrackIndex &iTrk, const std::vector<bool> &frmMarks) const;
    void CountTrackSolvedFrameInlierMeasurements(const TrackIndex &iTrk, FrameIndex &cntSolved, FrameIndex &cntInlier) const;
    MeasurementIndex SearchTrackForFrameMeasurementIndex(const TrackIndex &iTrk, const FrameIndex &iFrm) const;
    FeatureIndex SearchTrackForFrameFeatureIndex(const TrackIndex &iTrk, const FrameIndex &iFrm) const;
    bool AreTracksOverlappingInFrames(const TrackIndex &iTrk1, const TrackIndex &iTrk2, std::vector<bool> &marks) const;
    FrameIndex CountTracksOverlappingFrames(const TrackIndex &iTrk1, const TrackIndex &iTrk2) const;
#if DESCRIPTOR_TRACK == 0
    void PushBackTrack(const FrameIndex &iFrm1, const FeatureIndex &iFtr1, const FrameIndex &iFrm2, const FeatureIndex &iFtr2);
#else
    void PushBackTrack(const FrameIndex &iFrm1, const FeatureIndex &iFtr1, const FrameIndex &iFrm2, const FeatureIndex &iFtr2, const Descriptor &desc1,
                       const Descriptor &desc2);
#endif
    void PushBackTrack(const FrameFeatureIndexList &iFrmFtrs);
    void BreakTrack(const TrackIndex &iTrk);
    void RemoveBrokenTracks();
    void RemoveBrokenTracks(TrackIndexList &iTrksOriToNew);
#if DESCRIPTOR_TRACK == 0
    void ComputeTrackDescriptor(const TrackIndex &iTrk, Descriptor &desc) const;
#endif
    bool ComputeTrackSSE(const TrackIndex &iTrk, const Point3D &X, float &SSE, FrameIndex &nCrspsInlier) const;
    bool ComputeTrackMSE(const TrackIndex &iTrk, const Point3D &X, float &MSE) const;
    void ComputePointRayDirections(const TrackIndex &iTrk, AlignedVector<Point3D> &rayDirs, const bool rectData) const;
    float ComputePointMinimalRayAngleDot(const TrackIndex &iTrk, AlignedVector<Point3D> &rayDirs, const bool rectData) const;
    float ComputePointMinimalRayAngleDot(const TrackIndex &iTrk, AlignedVector<Point3D> &rayDirs, FrameIndex &iFrm1, FrameIndex &iFrm2, const bool rectData) const;
    float ComputePointMaximalRayAngle(const TrackIndex &iTrk, AlignedVector<Point3D> &rayDirs, const bool rectData) const;
    float ComputePointMaximalRayAngle(const TrackIndex &iTrk, AlignedVector<Point3D> &rayDirs, FrameIndex &iFrm1, FrameIndex &iFrm2, const bool rectData) const;

    //////////////////////////////////////////////////////////////////////////
    // Measurement
    //////////////////////////////////////////////////////////////////////////
    //inline MeasurementIndex GetMeasurementsNumber() const { return MeasurementIndex(m_xs.Size()); }
    inline MeasurementIndex GetMeasurementsNumber() const {
        return MeasurementIndex(m_meaStates.size());
    }
    inline const Point2D &GetMeasurement(const MeasurementIndex &iMea) const {
        return m_xs[iMea];
    }
    inline const AlignedVector<Point2D> &GetMeasurements() const {
        return m_xs;
    }
    inline const MeasurementState &GetMeasurementState(const MeasurementIndex &iMea) const {
        return m_meaStates[iMea];
    }
    inline const MeasurementStateList &GetMeasurementStates() const {
        return m_meaStates;
    }
    inline FrameIndex GetMeasurementFrameIndex(const MeasurementIndex &iMea) const {
        return m_mapMeaToFrm[iMea];
    }
    inline TrackIndex GetMeasurementTrackIndex(const MeasurementIndex &iMea) const {
        return m_mapMeaToTrk[iMea];
    }
    inline FeatureIndex GetMeasurementFrameFeatureIndex(const MeasurementIndex &iMea) const {
        return iMea - m_mapFrmToMea[m_mapMeaToFrm[iMea]];
    }
    inline void MarkMeasurementInlier(const MeasurementIndex &iMea) {
        m_meaStates[iMea] &= ~FLAG_MEASUREMENT_STATE_OUTLIER;
    }
    inline void MarkMeasurementOutlier(const MeasurementIndex &iMea) {
        m_meaStates[iMea] |= FLAG_MEASUREMENT_STATE_OUTLIER;
    }
    inline const bool &AreMeasurementsNormalized() const {
        return m_measNormalized;
    }
    inline void SetMeasurement(const MeasurementIndex &iMea, const Point2D &x) {
        m_xs[iMea] = x;
    }
    inline void SetMeasurementState(const MeasurementIndex &iMea, const MeasurementState &meaState) {
        m_meaStates[iMea] = meaState;
    }
    inline void SwapMeasurements(AlignedVector<Point2D> &xs) {
        m_xs.Swap(xs);
    }
    inline void ReserveMeasurements(const MeasurementIndex &nMeas) {
        m_xs.Reserve(nMeas);
    }
#if DESCRIPTOR_TRACK == 0
    inline TrackIndex GetDescriptorsNumber() const {
        return TrackIndex(m_descs.Size());
    }
    inline const Descriptor &GetDescriptor(const MeasurementIndex &iMea) const {
        return m_descs[iMea];
    }
    inline const AlignedVector<Descriptor> &GetDescriptors() const {
        return m_descs;
    }
    inline void SetDescriptor(const MeasurementIndex &iMea, const Descriptor &desc) {
        m_descs[iMea] = desc;
    }
    inline void SetDescriptorsNumber(const MeasurementIndex &nMeas) {
        m_descs.Resize(nMeas);
    }
    inline void ReserveDescriptors(const MeasurementIndex &nMeas) {
        m_descs.Reserve(nMeas);
    }
    inline void ClearDescriptors() {
        m_descs.Clear();
    }
#endif
    virtual void InitializeMeasurements();
    void SetMeasurementStates(const MeasurementIndexList &iMeas, const MeasurementStateList &meaStates);
    MeasurementIndex CountMeasurements(const MeasurementState meaState) const;
    void NullifyMeasurement(const MeasurementIndex &iMea);
    virtual void RemoveNullMeasurements();
    void ComputeMeasurementsDistribution(const MeasurementIndexList &iMeas, Point2D &mean, LA::Vector3f &cov) const;
    void BucketMeasurements(const ubyte nBinsX, const ubyte nBinsY, std::vector<ubyte> &mapMeaToBin) const;
    void NormalizeMeasurements();
    void DenormalizeMeasurements();
    static void ScaleMeasurements(const float scale, AlignedVector<Point2D> &xs);
    static void DistortMeasurements(const float fxy, const float distortion, AlignedVector<Point2D> &xs);
    static void UndistortMeasurements(const float distortion, AlignedVector<Point2D> &xs);
    static void RectifyMeasurements(const Camera::IntrinsicParameter &Kr, AlignedVector<Point2D> &xs);

    //////////////////////////////////////////////////////////////////////////
    // Data association
    //////////////////////////////////////////////////////////////////////////
    inline const FrameMeasurementMap &GetFrameMeasurementMap() const {
        return m_mapFrmToMea;
    }
    inline const TrackMeasurementMap &GetTrackMeasurementMap() const {
        return m_mapTrkToMea;
    }
    inline const MeasurementFrameMap &GetMeasurementFrameMap() const {
        return m_mapMeaToFrm;
    }
    inline const MeasurementTrackMap &GetMeasurementTrackMap() const {
        return m_mapMeaToTrk;
    }
    virtual void GetSubSequence(const FrameIndexList &iFrms, Sequence &seqSub, TrackIndexList &iTrks, const bool copyDesc, const bool copyClr) const;
    virtual void GetSubSequence(const FrameIndexList &iFrms, Sequence &seqSub, TrackIndexList &iTrks, MeasurementIndexList &iMeas, const bool copyDesc,
                                const bool copyClr) const;
    virtual void GetSubSequence(const FrameIndexList &iFrms, const TrackIndexList &iTrks, Sequence &seqSub, const bool copyDesc, const bool copyClr) const;
    virtual void GetSubSequence(const FrameIndexList &iFrms, const TrackIndexList &iTrks, Sequence &seqSub, MeasurementIndexList &iMeas, const bool copyDesc,
                                const bool copyClr) const;
    FeatureIndex SearchForFrameCommonTracksNumber(const FrameIndex &iFrm1, const FrameIndex &iFrm2) const;
    FeatureIndex SearchForFrameCommonTracksNumber(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3) const;
    void SearchForFrameCommonTracks(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const std::vector<bool> &trkMarks, TrackIndexList &iTrksMarked,
                                    TrackIndexList &iTrksUnmarked) const;
    void SearchForFrameCommonTracks(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3, const std::vector<bool> &trkMarks,
                                    TrackIndexList &iTrksMarked, TrackIndexList &iTrksUnmarked) const;
    void SearchForFrameFeatureMatches(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches) const;
    void SearchForFrameFeatureMatches(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, TrackIndexList &iTrks) const;
    void SearchForFrameFeatureMatches(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, TrackIndexList &iTrks, MatchSet2D &data,
                                      const bool rectData) const;
    void SearchForFrameFeatureMatches(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, MatchSet2D &data, const bool rectData) const;
    void SearchForFrameFeatureMatchesMarkedTrack(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const std::vector<bool> &trkMarks, FeatureMatchList &matches,
            MatchSet2D &data, const bool rectData) const;
    void SearchForFrameFeatureMatchesUnmarkedTrack(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const std::vector<bool> &trkMarks, FeatureMatchList &matches,
            TrackIndexList &iTrks, MatchSet2D &data, const bool rectData) const;
    void SearchForFrameFeatureMatchesInlierTrackAndMeasurement(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, MatchSet2D &data,
            const bool rectData) const;
    void SearchForFrameFeatureMatchesInlierTrackAndMeasurement(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches,
            TrackIndexList &iTrks) const;
    void SearchForFrameFeatureMatchesInlierMeasurement(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, MatchSet2D &data,
            const bool rectData) const;
    void SearchForSubsequenceCommonTracks(const FrameIndex iFrmStart, const FrameIndex iFrmEnd, TrackIndexList &iTrks,
                                          std::vector<MeasurementIndexList> &iMeasList) const;
    void ComputeFrameFeatureAverageDisparityPass1(const FrameIndex &iFrm1, FeatureIndexList &mapTrkToFtr1) const;
    float ComputeFrameFeatureAverageDisparityPass2(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureIndexList &mapTrkToFtr1,
            FeatureIndex &nCmnPts/*, std::vector<bool> &trkMarks*/) const;
#if DESCRIPTOR_TRACK == 0
    void MatchFrameFeatures(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureMatchList &matches, std::vector<bool> &marks);
    void MatchTrackAndFrameFeature(const TrackIndex &iTrk, const FrameIndex &iFrm, const FeatureIndex &iFtr);
#else
    void MatchFrameFeatures(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureMatchList &matches, std::vector<bool> &marks,
                            const AlignedVector<Descriptor> &descs1, const AlignedVector<Descriptor> &descs2);
    void MatchTrackAndFrameFeature(const TrackIndex &iTrk, const FrameIndex &iFrm, const FeatureIndex &iFtr, const Descriptor &desc);
#endif
    void MatchTracks(const TrackIndex &iTrk1, const TrackIndex &iTrk2);
    void MatchTracks(const TrackMatchList &trkMatches);
    void BreakOutlierTracksAndMeasurements();
    void BreakSingleTracksAndMeasurements();
    void RemoveOutlierTracksAndMeasurements();
    void RemoveSingleTracksAndMeasurements();

    //////////////////////////////////////////////////////////////////////////
    // Transformation
    //////////////////////////////////////////////////////////////////////////
    inline void TransformCamera(const FrameIndex &iFrm, const RigidTransformation3D &T, ENFT_SSE::__m128 *const &work3) {
        T.Apply(m_Cs[iFrm], m_Cs[iFrm], work3);
    }
    inline void TransformCamera(const FrameIndex &iFrm, const SimilarityTransformation3D &S, ENFT_SSE::__m128 *const &work3) {
        S.Apply(m_Cs[iFrm], m_Cs[iFrm], work3);
    }
    inline void ScaleCamera(const FrameIndex &iFrm, const float &scale) {
        m_Cs[iFrm].Scale(scale);
    }
    inline void TransformPoint(const TrackIndex &iTrk, const RigidTransformation3D &T, ENFT_SSE::__m128 *const &work1) {
        T.Apply(m_Xs[iTrk], work1);
    }
    inline void TransformPoint(const TrackIndex &iTrk, const SimilarityTransformation3D &S, ENFT_SSE::__m128 *const &work1) {
        S.Apply(m_Xs[iTrk], work1);
    }
    inline void ScalePoint(const TrackIndex &iTrk, const ENFT_SSE::__m128 &scale) {
        m_Xs[iTrk] *= scale;
    }
    void SetReferenceFrame(const FrameIndex &iFrmRef);
    void ComputeSceneSize(Point3D &size) const;
    void ComputeMeasurementDepths(std::vector<float> &depths) const;
    void TranslateScene(const Point3D &translation);
    void ScaleScene(const float &scale);
    void TransformScene(const RigidTransformation3D &T);
    void TransformScene(const SimilarityTransformation3D &S);
    void UpdateToMetric(const AbsoluteQuadric &Q);
    static void UpdateToMetric(const AbsoluteQuadric &Q, AlignedVector<ProjectiveMatrix> &Ps);

    //////////////////////////////////////////////////////////////////////////
    // Bundle adjustment
    //////////////////////////////////////////////////////////////////////////
    void GetBundleAdjustorData(SequenceBundleAdjustorData &data) const;
    void GetBundleAdjustorData(SequenceBundleAdjustorDataIntrinsicVariable &data) const;
    void GetBundleAdjustorData(SequenceBundleAdjustorDataProjective &data) const;
    void SetBunbleAdjustmentResults(const SequenceBundleAdjustorData &data);
    void SetBunbleAdjustmentResults(const SequenceBundleAdjustorDataIntrinsicVariable &data);
    void SetBunbleAdjustmentResults(const SequenceBundleAdjustorDataProjective &data);
    void GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, SequenceBundleAdjustorData &data, FrameIndexList &iFrmsBA, TrackIndexList &iTrksBA) const;
    void GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, SequenceBundleAdjustorDataIntrinsicVariable &data, FrameIndexList &iFrmsBA,
                               TrackIndexList &iTrksBA) const;
    void GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, SequenceBundleAdjustorDataProjective &data, FrameIndexList &iFrmsBA, TrackIndexList &iTrksBA) const;
    void GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, SequenceBundleAdjustorData &data, FrameIndexList &iFrmsBA) const;
    void GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, SequenceBundleAdjustorDataIntrinsicVariable &data,
                               FrameIndexList &iFrmsBA) const;
    void GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, SequenceBundleAdjustorDataProjective &data,
                               FrameIndexList &iFrmsBA) const;
    void SetBunbleAdjustmentResults(const SequenceBundleAdjustorData &data, const FrameIndex &nFrmsFix, const FrameIndexList &iFrmsBA,
                                    const TrackIndexList &iTrksBA);
    void SetBunbleAdjustmentResults(const SequenceBundleAdjustorDataIntrinsicVariable &data, const FrameIndex &nFrmsFix, const FrameIndexList &iFrmsBA,
                                    const TrackIndexList &iTrksBA);
    void SetBunbleAdjustmentResults(const SequenceBundleAdjustorDataProjective &data, const FrameIndex &nFrmsFix, const FrameIndexList &iFrmsBA,
                                    const TrackIndexList &iTrksBA);

    //////////////////////////////////////////////////////////////////////////
    // IO
    //////////////////////////////////////////////////////////////////////////
    void LoadCalibrationFile(const char *fileName);
    void LoadTrackColors();
    bool HasSwappedOut() const;
    void SwapOut();
    void SwapIn();
    bool SaveActc(const char *fileName) const;
    bool SaveActb(const char *fileName) const;
    bool LoadActb(const char *fileName, const bool normalizeMeas = true, const bool loadClr = false);
    bool SaveAct(const char *fileName) const;
    bool LoadAct(const char *fileName, const bool normalizeMeas = true, const bool loadClr = false, const bool skipCam = false);
    bool SaveNvm(const char *fileName) const;
    void SaveNvm(FILE *fp) const;
    bool LoadNvm(const char *fileName, const bool normalizeMeas = true);
    void LoadNvm(FILE *fp);
    bool SaveVsfm(const char *fileName, const bool saveNvm = true, const bool saveSift = true) const;
    bool LoadVsfm(const char *fileName);
    bool LoadVsfmF(const char *fileName);
    bool SaveOut(const char *fileName) const;
    bool SaveImageLists(const char *fileName) const;
    bool SaveCameras(const char *fileName) const;
    bool SavePoints(const char *fileName) const;
    bool SaveB(const char *fileName) const;
    bool SaveBwithDir(const char *fileName) const;
    virtual void SaveB(FILE *fp) const;
    bool LoadB(const char *fileName);
    bool LoadBwithDir(const char *fileName);
    virtual void LoadB(FILE *fp);
    void PrintCamera(const FrameIndex &iFrm) const;
    virtual void PrintFrameFeature(const FrameIndex &iFrm, const FeatureIndex &iFtr) const;
    void PrintTrack(const TrackIndex &iTrk) const;
    virtual void PrintStates() const;
    void AssertTrackOrdered(const TrackIndex &iTrk) const;
    void AssertConsistency() const;
    static void SaveProjectiveMatrixes(const char *fileName, const AlignedVector<ProjectiveMatrix> &Ps);

    //////////////////////////////////////////////////////////////////////////
    // Synthesis
    //////////////////////////////////////////////////////////////////////////
    void AddNoiseToCamerasOrPoints(const float percent, const bool &toCams, const bool &toPts);
    void AddNoiseToMeasurements(const float sigma = 1.0f, const float errTh = FLT_MAX);
    void AddNoiseToFrameSegments(const float percent);
    void SynthesizeLoop(const ushort &width, const ushort &height, const float &fx, const float &fy, const float &cx, const float &cy, const FrameIndex nFrms,
                        const TrackIndex nTrks, const FeatureIndex nFtrsPerFrm, const float dCenterPercentMax = 0, const float dAngleMax = 0);
    void SynthesizeLoopPlanarScene(const ushort &width, const ushort &height, const float &fx, const float &fy, const float &cx, const float &cy,
                                   const FrameIndex nFrms, const TrackIndex nTrks, const FeatureIndex nFtrsPerFrm, AlignedVector<LA::AlignedVector4f> &Pcs,
                                   const float dCenterPercentMax = 0, const float dAngleMax = 0);
    void SynthesizeTransformation(const float percentRigid, const float maxScale, SimilarityTransformation3D &S) const;
    void SynthesizeIntrinsicRectification(const float maxFocal, const float maxDistortion);
    void SynthesizeProjectiveTransformation(const float maxVal, AbsoluteQuadric &Q);
    static void AnalyzeError(const std::vector<float> &errs, float &avg, float &std, float &inf, uint &iMax, const float &errMedScaleFactor = 2.0f);
    static void AnalyzeAndPrintError(const std::vector<float> &errs, const float &errMedScaleFactor = 2.0f);
    static void PushBackCameraError(const Camera &C1, const Camera &C2, std::vector<float> &qAbsErrs, std::vector<float> &qRelErrs, std::vector<float> &wAbsErrs,
                                    std::vector<float> &wRelErrs, std::vector<float> &tAbsErrs, std::vector<float> &tRelErrs, std::vector<float> &cAbsErrs, std::vector<float> &cRelErrs);
    static void PrintCameraErrors(const std::vector<float> &qAbsErrs, const std::vector<float> &qRelErrs, const std::vector<float> &wAbsErrs,
                                  const std::vector<float> &wRelErrs, const std::vector<float> &tAbsErrs, const std::vector<float> &tRelErrs, const std::vector<float> &cAbsErrs,
                                  const std::vector<float> &cRelErrs, const float errMedScaleFactor = 2.0f);
    static void PushBackPointError(const Point3D &X1, const Point3D &X2, std::vector<float> &absErrs, std::vector<float> &relErrs);
    static void PrintPointErrors(const std::vector<float> &absErrs, const std::vector<float> &relErrs, const float errMedScaleFactor = 2.0f);
    static void PushBackIntrinsicRectificationError(const Camera::IntrinsicParameter &Kr1, const Camera::IntrinsicParameter &Kr2, std::vector<float> &absErrs,
            std::vector<float> &relErrs);
    static void PushBackIntrinsicRectificationError(const float &f1, const float &f2, std::vector<float> &absErrs, std::vector<float> &relErrs);
    static void PrintIntrinsicRectificationErrors(const std::vector<float> &absErrs, const std::vector<float> &relErrs, const float errMedScaleFactor = 2.0f);
    static void PushBackProjectiveMatrixError(const ProjectiveMatrix &P1, const ProjectiveMatrix &P2, std::vector<float> &absErrs, std::vector<float> &relErrs);
    static void PrintProjectiveMatrixErrors(const std::vector<float> &absErrs, const std::vector<float> &relErrs, const float errMedScaleFactor = 2.0f);

  protected:

    SequenceTag m_tag;
    IntrinsicMatrix &m_K;
    Camera::IntrinsicParameter m_Kr;
    IntrinsicType m_intrinsicType;

    AlignedVector<Camera> m_Cs;
    AlignedVector<Camera::IntrinsicParameter> m_Krs;
    AlignedVector<ProjectiveMatrix> m_Ps;
    AlignedVector<Point3D> m_Xs;
    AlignedVector<Point2D> m_xs;
    AlignedVector<Descriptor> m_descs;

    FrameMeasurementMap m_mapFrmToMea;
    TrackMeasurementMap m_mapTrkToMea;
    MeasurementFrameMap m_mapMeaToFrm;
    MeasurementTrackMap m_mapMeaToTrk;

    FrameStateList m_frmStates;
    TrackStateList m_trkStates;
    MeasurementStateList m_meaStates;

    std::vector<CVD::Rgb<ubyte> > m_trkClrs;

    bool m_measNormalized;

    friend SequenceSet;

};

#endif