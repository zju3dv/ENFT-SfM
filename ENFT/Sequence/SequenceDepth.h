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

#ifndef _SEQUENCE_DEPTH_H_
#define _SEQUENCE_DEPTH_H_

#include "Sequence/Sequence.h"
#include "SfM/CameraEstimatorDataDepth.h"
#include "SfM/Point3DEstimatorDataDepth.h"
#include "SfM/RelativePoseEstimatorDataDepth.h"

#define FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH	8

class SequenceBundleAdjustorDataDepth;

class SequenceDepth : public Sequence
{

public:

	SequenceDepth() : Sequence() {}
	void operator = (const SequenceDepth &seq);
	void Swap(SequenceDepth &seq);

	void SetTag(const std::string &seqDir, const std::string &seqNameRGB, const std::string &seqNameDep, const std::string &seqNameNormal, 
		const int &iStart, const int &iStep, const int &iEnd);

	inline const std::string& GetDepthFileName(const FrameIndex &iFrm) const { return m_tagDep.GetImageFileName(iFrm); }
	inline const std::string& GetNormalFileName(const FrameIndex &iFrm) const { return m_tagNormal.GetImageFileName(iFrm); }

	//inline const MeasurementIndex& GetMeasurementDepthIndex(const MeasurementIndex &iMea) const { return m_mapMeaToDep[iMea]; }
	inline void MarkMeasurementDepthInlier(const MeasurementIndex &iMea) { m_meaStates[iMea] &= ~FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH; }
	inline void MarkMeasurementDepthOutlier(const MeasurementIndex &iMea) { m_meaStates[iMea] |= FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH; }
	inline const float& GetDepth(const MeasurementIndex &iMea) const { return m_ds[iMea]; }
	inline const float* GetFrameDepths(const FrameIndex &iFrm) const { return &m_ds[m_mapFrmToMea[iFrm]]; }

	void GetFrameInlierDepthMeasurementIndexList(const FrameIndex &iFrm, MeasurementIndexList &iMeas) const;
	FrameIndex CountTrackInlierDepths(const TrackIndex &iTrk) const;

	void GetCameraEstimatorData(const FrameIndex &iFrm, CameraEstimatorDataDepth &data, MeasurementIndexList &iMeas) const;
	void GetPoint3DEstimatorData(const TrackIndex &iTrk, Point3DEstimatorDataDepth &data, MeasurementIndexList &iMeas) const;
	void GetPoint3DEstimatorDataInlier(const TrackIndex &iTrk, Point3DEstimatorDataDepth &data) const;
	//void GetScaleEstimatorData(ScaleEstimatorData &data, MeasurementIndexList &iMeas) const;
	void SearchForFrameFeatureMatches(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, RelativePoseEstimatorDataDepth &data) const;

	virtual void InitializeMeasurements();
	virtual void RemoveNullMeasurements();

	virtual void GetSubSequence(const FrameIndexList &iFrms, Sequence &seqSub, TrackIndexList &iTrks, MeasurementIndexList &iMeas, const bool copyDesc, 
		const bool copyClr) const;
	virtual void GetSubSequence(const FrameIndexList &iFrms, const TrackIndexList &iTrks, Sequence &seqSub, MeasurementIndexList &iMeas, const bool copyDesc, 
		const bool copyClr) const;

	void GetBundleAdjustorData(SequenceBundleAdjustorDataDepth &data) const;
	void SetBunbleAdjustmentResults(const SequenceBundleAdjustorDataDepth &data);
	void GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, SequenceBundleAdjustorDataDepth &data, FrameIndexList &iFrmsBA, TrackIndexList &iTrksBA) const;
	void GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, SequenceBundleAdjustorDataDepth &data, 
		FrameIndexList &iFrmsBA) const;
	void SetBunbleAdjustmentResults(const SequenceBundleAdjustorDataDepth &data, const FrameIndex &nFrmsFix, const FrameIndexList &iFrmsBA, 
		const TrackIndexList &iTrksBA);

	void LoadDepths();
	void LoadDepths(const float errTh);
	void SynthesizeDepths(const float ratio);
	virtual void SaveB(FILE *fp) const;
	virtual void LoadB(FILE *fp);
	virtual void PrintFrameFeature(const FrameIndex &iFrm, const FeatureIndex &iFtr) const;
	virtual void PrintStates() const;
	bool SavePly(const char *fileName) const;
	bool SaveObj(const char *fileName) const;

private:

	SequenceTag m_tagDep, m_tagNormal;
	AlignedVector<float> m_ds;

};

#endif