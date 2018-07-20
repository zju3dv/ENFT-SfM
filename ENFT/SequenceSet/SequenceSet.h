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

#ifndef _SEQUENCE_SET_H_
#define _SEQUENCE_SET_H_

#include "Sequence/Sequence.h"
#include "SfM/SimilarityTransformationEstimatorData.h"
#include "SfM/CameraPairEstimatorData.h"

typedef ushort						SequenceIndex;
typedef ushort						SegmentIndex;
#define INVALID_SEQUENCE_INDEX	USHRT_MAX
#define INVALID_SEGMENT_INDEX	USHRT_MAX
typedef std::vector<SequenceIndex>	SequenceIndexList;
typedef std::vector<SegmentIndex>	SegmentIndexList;

typedef Match<SequenceIndex>			SequenceIndexPair;
typedef std::vector<SequenceIndexPair>	SequenceIndexPairList;

class SequenceFrameIndex
{
public:
	SequenceFrameIndex() { Invalidate(); }
	SequenceFrameIndex(const SequenceIndex &iSeq, const FrameIndex &iFrm) : m_iSeq(iSeq), m_iFrm(iFrm) {}
	inline const SequenceIndex& GetSequenceIndex() const { return m_iSeq; }
	inline const FrameIndex&	GetFrameIndex	() const { return m_iFrm; }
	inline void Get(SequenceIndex &iSeq, FrameIndex &iFrm) const { iSeq = m_iSeq; iFrm = m_iFrm; }
	inline void SetSequenceIndex(const SequenceIndex	&iSeq) { m_iSeq = iSeq; }
	inline void SetFrameIndex	(const FrameIndex		&iFrm) { m_iFrm = iFrm; }
	inline void Set(const SequenceIndex &iSeq, const FrameIndex &iFrm) { m_iSeq = iSeq; m_iFrm = iFrm; }
	inline void Invalidate() { m_iSeq = INVALID_SEQUENCE_INDEX; }
	inline bool IsValid()	const { return m_iSeq != INVALID_SEQUENCE_INDEX; }
	inline bool IsInvalid() const { return m_iSeq == INVALID_SEQUENCE_INDEX; }
private:
	SequenceIndex	m_iSeq;
	FrameIndex		m_iFrm;
};
typedef std::vector<SequenceFrameIndex> SequenceFrameIndexList;

class SequenceFramePairIndex
{
public:
	SequenceFramePairIndex() { Invalidate(); }
	SequenceFramePairIndex(const SequenceIndex &iSeq, const FrameIndex &iFrm1, const FrameIndex &iFrm2) : m_iSeq(iSeq), m_iFrm1(iFrm1), m_iFrm2(iFrm2) {}
	inline const SequenceIndex& GetSequenceIndex() const { return m_iSeq; }
	inline const FrameIndex&	GetFrameIndex1	() const { return m_iFrm1; }
	inline const FrameIndex&	GetFrameIndex2	() const { return m_iFrm2; }
	inline void Get(SequenceIndex &iSeq, FrameIndex &iFrm1, FrameIndex &iFrm2) const { iSeq = m_iSeq; iFrm1 = m_iFrm1; iFrm2 = m_iFrm2; }
	inline void SetSequenceIndex(const SequenceIndex	&iSeq)  { m_iSeq = iSeq; }
	inline void SetFrameIndex1	(const FrameIndex		&iFrm1) { m_iFrm1 = iFrm1; }
	inline void SetFrameIndex2	(const FrameIndex		&iFrm2) { m_iFrm2 = iFrm2; }
	inline void Set(const SequenceIndex &iSeq, const FrameIndex &iFrm1, const FrameIndex &iFrm2) { m_iSeq = iSeq; m_iFrm1 = iFrm1; m_iFrm2 = iFrm2; }
	inline void Invalidate() { m_iSeq = INVALID_SEQUENCE_INDEX; }
	inline bool IsValid()	const { return m_iSeq != INVALID_SEQUENCE_INDEX; }
	inline bool IsInvalid() const { return m_iSeq == INVALID_SEQUENCE_INDEX; }
	inline bool operator < (const SequenceFramePairIndex &iSeqFrmPair) const { return m_iSeq < iSeqFrmPair.m_iSeq; }
private:
	SequenceIndex	m_iSeq;
	FrameIndex		m_iFrm1, m_iFrm2;
};
typedef std::vector<SequenceFramePairIndex> SequenceFramePairIndexList;

class SequenceTrackIndex
{
public:
	SequenceTrackIndex() { Invalidate(); }
	SequenceTrackIndex(const SequenceIndex &iSeq, const TrackIndex &iTrk) : m_iSeq(iSeq), m_iTrk(iTrk) {}
	inline const SequenceIndex& GetSequenceIndex() const { return m_iSeq; }
	inline const TrackIndex&	GetTrackIndex	() const { return m_iTrk; }
	inline void Get(SequenceIndex &iSeq, TrackIndex &iTrk) const { iSeq = m_iSeq; iTrk = m_iTrk; }
	inline void SetSequenceIndex(const SequenceIndex	&iSeq) { m_iSeq = iSeq; }
	inline void SetTrackIndex	(const TrackIndex		&iTrk) { m_iTrk = iTrk; }
	inline void Set(const SequenceIndex &iSeq, const TrackIndex &iTrk) { m_iSeq = iSeq; m_iTrk = iTrk; }
	inline void Invalidate() { m_iSeq = INVALID_SEQUENCE_INDEX; }
	inline bool IsValid()	const { return m_iSeq != INVALID_SEQUENCE_INDEX; }
	inline bool IsInvalid() const { return m_iSeq == INVALID_SEQUENCE_INDEX; }
	inline bool operator < (const SequenceTrackIndex &iSeqTrk) const { return m_iSeq < iSeqTrk.m_iSeq || m_iSeq == iSeqTrk.m_iSeq && m_iTrk < iSeqTrk.m_iTrk; }
private:
	SequenceIndex	m_iSeq;
	TrackIndex		m_iTrk;
};
typedef std::vector<SequenceTrackIndex> SequenceTrackIndexList;

class SequenceMeasurementIndex
{
public:
	SequenceMeasurementIndex() { Invalidate(); }
	SequenceMeasurementIndex(const SequenceIndex &iSeq, const MeasurementIndex &iMea) : m_iSeq(iSeq), m_iMea(iMea) {}
	inline const SequenceIndex&		GetSequenceIndex	() const { return m_iSeq; }
	inline const MeasurementIndex&	GetMeasurementIndex	() const { return m_iMea; }
	inline void Get(SequenceIndex &iSeq, MeasurementIndex &iMea) const { iSeq = m_iSeq; iMea = m_iMea; }
	inline void SetSequenceIndex	(const SequenceIndex	&iSeq) { m_iSeq = iSeq; }
	inline void SetMeasurementIndex	(const MeasurementIndex	&iMea) { m_iMea = iMea; }
	inline void Set(const SequenceIndex &iSeq, const MeasurementIndex &iMea) { m_iSeq = iSeq; m_iMea = iMea; }
	inline void Invalidate() { m_iSeq = INVALID_SEQUENCE_INDEX; }
	inline bool IsValid()	const { return m_iSeq != INVALID_SEQUENCE_INDEX; }
	inline bool IsInvalid() const { return m_iSeq == INVALID_SEQUENCE_INDEX; }
private:
	SequenceIndex		m_iSeq;
	MeasurementIndex	m_iMea;
};
typedef std::vector<SequenceMeasurementIndex> SequenceMeasurementIndexList;

#define FLAG_SEQUENCE_STATE_DEFAULT			0	// NOT_REGISTERED
#define FLAG_SEQUENCE_STATE_REGISTRED		1
#define FLAG_SEQUENCE_STATE_FIXED			2
#define FLAG_SEQUENCE_STATE_ADJUSTED		4
#define FLAG_SEQUENCE_STATE_SWAPED_OUT		8

#define FLAG_COMMON_TRACK_STATE_DEFAULT		0	// NOT_REGISTERED
#define FLAG_COMMON_TRACK_STATE_REGISTERED	64

typedef ubyte SequenceState;
typedef std::vector<SequenceState> SequenceStateList;

class SequenceSetBundleAdjustorData3DSimilarity;
class SequenceSetBundleAdjustorData3DScale;
class SegmentSetBundleAdjustorData2DSimilarity;
class SequenceTransformationOptimizerDataSimilarity;
class SequenceTransformationOptimizerDataScale;

class SequenceSet
{

public:

	typedef std::vector<SequenceTrackIndexList>	CommonTrackIndividualTrackMap;
	typedef std::vector<TrackIndexList>			IndividualTrackCommonTrackMap;

	//////////////////////////////////////////////////////////////////////////
	// Sequence
	//////////////////////////////////////////////////////////////////////////
	inline SequenceSet() { m_intrinsicType = Sequence::INTRINSIC_USER_FIXED; }
	inline ~SequenceSet() { ReleaseSequences(); }
	inline void SetDirectory(const std::string &dir) { m_dir = dir; }
	inline const std::string& GetDirectory() const { return m_dir; }
	inline void SetIntrinsicType(const Sequence::IntrinsicType &intrinsicType) { m_intrinsicType = intrinsicType; }
	inline const Sequence::IntrinsicType& GetIntrinsicType() const { return m_intrinsicType; }
	inline const Sequence& GetSequence(const SequenceIndex &iSeq) const { return *m_pSeqs[iSeq]; }
	inline const Sequence& operator[](const SequenceIndex &iSeq) const { return *m_pSeqs[iSeq]; }
	inline		 Sequence& operator[](const SequenceIndex &iSeq)	   { return *m_pSeqs[iSeq]; }
	inline SequenceIndex GetSequencesNumber() const { return SequenceIndex(m_pSeqs.size()); }
	inline const SequenceState& GetSequenceState(const SequenceIndex &iSeq) const { return m_seqStates[iSeq]; }
	inline void SetSequenceState(const SequenceIndex &iSeq, const SequenceState &seqState) { m_seqStates[iSeq] = seqState; }
	inline void SetSequenceStates(const SequenceState &seqState) { m_seqStates.assign(GetSequencesNumber(), seqState); }
	inline void MarkSequenceSwappedOut(const SequenceIndex &iSeq) { m_seqStates[iSeq] |= FLAG_SEQUENCE_STATE_SWAPED_OUT; }
	inline void MarkSequenceSwappedIn(const SequenceIndex &iSeq) { m_seqStates[iSeq] &= ~FLAG_SEQUENCE_STATE_SWAPED_OUT; }
	void SetCalib(const char *calibFileName, bool focalConst);
	void GetSequenceIndexList(const SequenceState &seqState, SequenceIndexList &iSeqs) const;
	void GetSimilarityEstimatorData(const SequenceIndex &iSeq, SimilarityTransformationEstimatorData2D &data, TrackIndexList &iTrksIdv) const;
	void operator = (const SequenceSet &seqs);
	void CreateSequences(const SequenceIndex &nSeqs);
	void ReleaseSequences();
	void InitializeCommonPoints();
	SequenceIndex CountSequences(const SequenceState seqState) const;
	TrackIndex CountSequenceRegisteredCommonTracks(const SequenceIndex &iSeq) const;
	TrackIndex CountSequenceMarkedCommonTracks(const SequenceIndex &iSeq, const std::vector<bool> &cmnTrkMarks) const;
	void GetSequenceMarkedCommonTrackIndexList(const SequenceIndex &iSeq, const std::vector<bool> &cmnTrkMarks, TrackIndexList &iTrksCmn) const;
	void MarkSequenceRegistered(const SequenceIndex &iSeq);
	void MarkSequencesSwappedOut();
	void MarkSequenceCommonTracks(const SequenceIndex &iSeq, std::vector<bool> &cmnTrkMarks) const;
	void MarkSequencesCommonTracks(const SequenceIndexList &iSeqs, std::vector<bool> &cmnTrkMarks) const;

	//////////////////////////////////////////////////////////////////////////
	// Frame
	//////////////////////////////////////////////////////////////////////////
	FrameIndex CountFramesNumber() const;
	FeatureIndex CountFrameRegisteredCommonPoints(const SequenceIndex &iSeq, const FrameIndex &iFrm) const;
	void GetCameraEstimatorDataByRegisteredCommonPoints(const SequenceIndex &iSeq, const FrameIndex &iFrm, CameraEstimatorData &data, TrackIndexList &iTrksIdv) const;
	void GetCameraPairEstimatorData(const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureMatchList &matches, 
		CameraPairEstimatorData &data) const;

	//////////////////////////////////////////////////////////////////////////
	// Track
	//////////////////////////////////////////////////////////////////////////
	inline TrackIndex GetCommonTracksNumber() const { return TrackIndex(m_mapCmnTrkToIdvTrk.size()); }
	inline const Point3D& GetCommonPoint(const TrackIndex &iTrkCmn) const { return m_XsCmn[iTrkCmn]; }
	inline const AlignedVector<Point3D>& GetCommonPoints() const { return m_XsCmn; }
	inline TrackIndex GetIndividualTrackCommonTrack(const SequenceIndex &iSeq, const TrackIndex &iTrkIdv) const { return m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv]; }
	inline const TrackIndexList& GetSequenceCommonTrackIndexList(const SequenceIndex &iSeq) const { return m_mapIdvTrkToCmnTrk[iSeq]; }
	inline const SequenceTrackIndexList& GetCommonTrackIndividualTrackIndexList(const TrackIndex &iTrkCmn) const { return m_mapCmnTrkToIdvTrk[iTrkCmn]; }
	inline const TrackState& GetCommonTrackState(const TrackIndex &iTrkCmn) const { return m_cmnTrkStates[iTrkCmn]; }
	inline void SetCommonPointsNumber(const TrackIndex &nTrksCmn) { m_XsCmn.Resize(nTrksCmn); }
	inline void SetCommonPoint(const TrackIndex &iTrkCmn, const Point3D &Xcmn) { m_XsCmn[iTrkCmn] = Xcmn; }
	inline void SetCommonTrackState(const TrackIndex &iTrkCmn, const TrackState &trkState) { m_cmnTrkStates[iTrkCmn] = trkState; }
	inline void SetCommonTrackStates(const TrackState &trkState) { m_cmnTrkStates.assign(GetCommonTracksNumber(), trkState); }
	inline void MarkCommonTrackRegistered(const TrackIndex &iTrkCmn) { m_cmnTrkStates[iTrkCmn] |= FLAG_COMMON_TRACK_STATE_REGISTERED; }
	void GetCommonPoint3DEstimatorDataInlier(const TrackIndex &iTrkCmn, Point3DEstimatorData &data, Point3DEstimatorData &dataTmp) const;
	SequenceIndex CountCommonTrackRegisteredSequenceIndividualTracks(const TrackIndex &iTrkCmn) const;
	SequenceIndex CountCommonTrackInlierIndividualTracks(const TrackIndex &iTrkCmn) const;
	FrameIndex CountCommonTrackInlierMeasurements(const TrackIndex &iTrkCmn) const;
	TrackIndex SearchCommonTrackForIndividualTrack(const TrackIndex &iTrkCmn, const SequenceIndex &iSeq) const;
	void SearchCommonTrackForIndividualTracks(const TrackIndex &iTrkCmn, const SequenceIndex &iSeq, TrackIndexList &iTrksIdv) const;
	bool AreCommonTrackAndIndividualTrackOverlappingInFrames(const TrackIndex &iTrkCmn, const SequenceIndex &iSeq, const TrackIndex &iTrkIdv, std::vector<bool> &marks) const;
	bool AreCommonTracksOverlappingInFrames(const TrackIndex &iTrkCmn1, const TrackIndex &iTrkCmn2, std::vector<bool> &marks) const;
	//void SetCommonTrackPlaneIndex(const TrackIndex &iTrkCmn, const ubyte &iPlane);
	void UnmarkCommonTrackRegistered(const TrackIndex &iTrkCmn);
	void PushBackCommonTrack(const SequenceIndex &iSeq, const TrackIndex &iTrkIdv);
	void PushBackCommonTrack(const SequenceIndex &iSeq1, const TrackIndex &iTrkIdv1, const SequenceIndex &iSeq2, const TrackIndex &iTrkIdv2);
	void PushBackCommonTrack(const SequenceTrackIndexList &iSeqTrksIdv);
	void RemoveIndividualTracks(const SequenceIndex &iSeq);
	bool ComputeCommonTrackMSE(const TrackIndex &iTrkCmn, const Point3D &Xcmn, float &MSE) const;
	void CopyCommonPointsToIndividualPoints();
	void CopyCommonPointsToIndividualSinglePoints();
	void CopyIndividualPointsToCommonPoints();
	void AverageIndividualPointsToCommonPoints();
	void RemoveBrokenTracks(const SequenceIndex &iSeq);

	//////////////////////////////////////////////////////////////////////////
	// Measurement
	//////////////////////////////////////////////////////////////////////////
	void SetMeasurementStates(const SequenceMeasurementIndexList &iSeqMeas, const MeasurementStateList &meaStates);

	//////////////////////////////////////////////////////////////////////////
	// Data association
	//////////////////////////////////////////////////////////////////////////
	void GetSubSequences(const std::vector<FrameIndexList> &iFrmsList, const std::vector<TrackIndexList> &iTrksListIdv, SequenceSet &seqsSub, TrackIndexList &iTrksCmn, 
		const bool copyDesc, const bool copyClr) const;
	void MarkSequenceConnectedComponent(const SequenceIndex &iSeq, std::vector<bool> &seqMarks) const;
	void SearchForSequenceConnectedComponent(const SequenceIndex &iSeq, SequenceIndexList &iSeqsCC) const;
	void SearchForFrameFeatureMatches(const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches) const;
	void MatchIndividualTracks(const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const TrackMatchList &trkMatches);
	void FinishMatchingIndividualTracks(const float errSqTh);
	void MatchCommonTrackAndIndividualTrack(const TrackIndex &iTrkCmn, const SequenceIndex &iSeq, const TrackIndex &iTrkIdv);
	void MatchCommonTracks(const TrackIndex &iTrkCmn1, const TrackIndex &iTrkCmn2);
	void MatchFrameFeatures(const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches);
	void ConcatenateSequences(Sequence &seqCat, SequenceFrameIndexList &iSeqFrms, TrackIndexList &iTrksCmn, TrackIndexList &iTrksCmnToCat, 
		SequenceTrackIndexList &iSeqTrksIdv, std::vector<TrackIndexList> &iTrksIdvToCatList, SequenceMeasurementIndexList &iSeqMeas, const bool copyTag = true, 
		const bool copyDesc = false, const bool copyClr = false) const;
	void SplitSequences(const Sequence &seqCat, const FrameIndex &nFrmsPerSeq);

	//////////////////////////////////////////////////////////////////////////
	// Bundle adjustment
	//////////////////////////////////////////////////////////////////////////
	void GetBundleAdjustorData(const SequenceIndexList &iSeqsAdj, SequenceSetBundleAdjustorData3DSimilarity &data, SequenceIndex &nSeqsFix, 
		SequenceIndexList &iSeqsBA, TrackIndexList &iTrksCmnBA) const;
	void SetBundleAdjustmentResults(const SequenceSetBundleAdjustorData3DSimilarity &data, const SequenceIndex &nSeqsFix, const SequenceIndexList &iSeqsBA, 
		const TrackIndexList &iTrksCmnBA);
	void GetBundleAdjustorData(const SequenceIndexList &iSeqsAdj, SegmentSetBundleAdjustorData2DSimilarity &data, SequenceIndex &nSeqsFix, 
		SequenceIndexList &iSeqsBA, TrackIndexList &iTrksCmnBA, SequenceTrackIndexList &iSeqTrksIdvBA) const;
	void SetBundleAdjustmentResults(const SegmentSetBundleAdjustorData2DSimilarity &data, const SegmentIndex &nSegsFix, const SequenceIndexList &iSeqsBA, 
		const TrackIndexList &iTrksCmnBA, const SequenceTrackIndexList &iSeqTrksIdvBA, const bool toInternal);
	void GetSequenceTransformationOptimizerData(SequenceTransformationOptimizerDataSimilarity &data, SequenceIndexList &iSeqsAdj) const;
	void SetSequenceTransformationOptimizationResults(const SequenceTransformationOptimizerDataSimilarity &data, const SequenceIndexList &iSeqsAdj);
	void GetSequenceTransformationOptimizerData(SequenceTransformationOptimizerDataScale &data, SequenceIndexList &iSeqsAdj) const;
	void SetSequenceTransformationOptimizationResults(const SequenceTransformationOptimizerDataScale &data, const SequenceIndexList &iSeqsAdj);


	//////////////////////////////////////////////////////////////////////////
	// IO
	//////////////////////////////////////////////////////////////////////////
	void SwapOut(const SequenceIndex &iSeq);
	void SwapIn(const SequenceIndex &iSeq);
	void SaveB(FILE *fp) const;
	bool SaveB(const char *fileName) const;
	void LoadB(FILE *fp);
	bool LoadB(const char *fileName);
	void PrintCommonTrack(const TrackIndex &iTrkCmn) const;
	void PrintStates() const;
	void AssertCommonTrackSorted(const TrackIndex &iTrkCmn) const;
	void AssertConsistency() const;

	//////////////////////////////////////////////////////////////////////////
	// Synthesis
	//////////////////////////////////////////////////////////////////////////
	void AddNoiseToTransformations(const SequenceIndex nSeqsFix, const float noisePercentRigid, const float maxScale, AlignedVector<SimilarityTransformation3D> &Ss);
	void SynthesizeLoop(const ushort &width, const ushort &height, const float &fx, const float &fy, const float &cx, const float &cy, const SequenceIndex nSeqs, 
		const SegmentIndex nSegsPerSeq, const FrameIndex nFrmsPerSeq, const TrackIndex nTrks, const FeatureIndex nFtrsPerFrm, const float dCenterPercentMax = 0, 
		const float dAngleMax = 0);
	void SynthesizeIntrinsicRectification(const float maxFocal, const float maxDistortion, Camera::IntrinsicParameter &G);
	static void PushBackSimilarityError(const SimilarityTransformation3D &S1, const SimilarityTransformation3D &S2, std::vector<float> &sAbsErrs, 
		std::vector<float> &sRelErrs, std::vector<float> &qAbsErrs, std::vector<float> &qRelErrs, std::vector<float> &wAbsErrs, std::vector<float> &wRelErrs, 
		std::vector<float> &tAbsErrs, std::vector<float> &tRelErrs, std::vector<float> &centerAbsErrs, std::vector<float> &centerRelErrs);
	static void PrintSimilarityErrors(const std::vector<float> &sAbsErrs, const std::vector<float> &sRelErrs, const std::vector<float> &qAbsErrs, 
		const std::vector<float> &qRelErrs, const std::vector<float> &wAbsErrs, const std::vector<float> &wRelErrs, const std::vector<float> &tAbsErrs, 
		const std::vector<float> &tRelErrs, const std::vector<float> &centerAbsErrs, const std::vector<float> &centerRelErrs, const float errMedScaleFactor = 2.0f);
	static void PushBackRigidError(const RigidTransformation3D &T1, const RigidTransformation3D &T2, std::vector<float> &qAbsErrs, std::vector<float> &qRelErrs, 
		std::vector<float> &wAbsErrs, std::vector<float> &wRelErrs, std::vector<float> &tAbsErrs, std::vector<float> &tRelErrs, std::vector<float> &centerAbsErrs, 
		std::vector<float> &centerRelErrs);
	static void PrintRigidErrors(const std::vector<float> &qAbsErrs, const std::vector<float> &qRelErrs, const std::vector<float> &wAbsErrs, 
		const std::vector<float> &wRelErrs, const std::vector<float> &tAbsErrs, const std::vector<float> &tRelErrs, const std::vector<float> &centerAbsErrs, 
		const std::vector<float> &centerRelErrs, const float errMedScaleFactor = 2.0f);
	static void PushBackScaleError(const float &scale1, const float &scale2, std::vector<float> &sAbsErrs, std::vector<float> &sRelErrs);
	static void PrintScaleErrors(const std::vector<float> &sAbsErrs, const std::vector<float> &sRelErrs, const float errMedScaleFactor = 2.0f);
	
protected:

	std::string m_dir;
	Sequence::IntrinsicType m_intrinsicType;

	std::vector<Sequence *> m_pSeqs;
	CommonTrackIndividualTrackMap	m_mapCmnTrkToIdvTrk;
	IndividualTrackCommonTrackMap	m_mapIdvTrkToCmnTrk;

	SequenceStateList m_seqStates;
	TrackStateList m_cmnTrkStates;

	AlignedVector<Point3D> m_XsCmn;

};

#endif