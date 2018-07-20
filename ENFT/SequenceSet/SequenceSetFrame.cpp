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
#include "SequenceSet.h"

FrameIndex SequenceSet::CountFramesNumber() const
{
	FrameIndex cnt = 0;
	const SequenceIndex nSeqs = GetSequencesNumber();
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
		cnt += GetSequence(iSeq).GetFramesNumber();
	return cnt;
}

FeatureIndex SequenceSet::CountFrameRegisteredCommonPoints(const SequenceIndex &iSeq, const FrameIndex &iFrm) const
{
	const Sequence &seq = GetSequence(iSeq);
	const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
	const MeasurementIndex iMeaFirst = seq.m_mapFrmToMea[iFrm];
	const TrackIndex *iTrksIdvSrc = seq.m_mapMeaToTrk.data() + iMeaFirst;
	const MeasurementState *meaStates = seq.m_meaStates.data() + iMeaFirst;
	FeatureIndex i, j;
	TrackIndex iTrkIdv, iTrkCmn;
	const FeatureIndex N = FeatureIndex(seq.m_mapFrmToMea[iFrm + 1] - iMeaFirst);
	FeatureIndex cnt = 0;
	for(i = 0, j = 0; i < N; ++i)
	{
		if((iTrkIdv = iTrksIdvSrc[i]) != INVALID_TRACK_INDEX && (iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX && (m_cmnTrkStates[iTrkCmn] & FLAG_COMMON_TRACK_STATE_REGISTERED)
		&& (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) && !(meaStates[i] & FLAG_MEASUREMENT_STATE_OUTLIER))
			++cnt;
	}
	return cnt;
}

void SequenceSet::GetCameraEstimatorDataByRegisteredCommonPoints(const SequenceIndex &iSeq, const FrameIndex &iFrm, CameraEstimatorData &data, TrackIndexList &iTrksIdv) const
{
	const Sequence &seq = GetSequence(iSeq);
	const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
	const MeasurementIndex iMeaFirst = seq.m_mapFrmToMea[iFrm];
	const TrackIndex *iTrksIdvSrc = seq.m_mapMeaToTrk.data() + iMeaFirst;
	const MeasurementState *meaStates = seq.m_meaStates.data() + iMeaFirst;
	const Point2D *xs = seq.m_xs.Data() + iMeaFirst;
	FeatureIndex i, j;
	TrackIndex iTrkIdv, iTrkCmn;
	const FeatureIndex N = FeatureIndex(seq.m_mapFrmToMea[iFrm + 1] - iMeaFirst);
	data.Resize(N);
	iTrksIdv.resize(N);
	for(i = 0, j = 0; i < N; ++i)
	{
		if((iTrkIdv = iTrksIdvSrc[i]) != INVALID_TRACK_INDEX && (iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX && (m_cmnTrkStates[iTrkCmn] & FLAG_COMMON_TRACK_STATE_REGISTERED)
		&& (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) && !(meaStates[i] & FLAG_MEASUREMENT_STATE_OUTLIER))
		{
			data.Set(j, GetCommonPoint(iTrkCmn), xs[i]);
			iTrksIdv[j] = iTrkIdv;
			++j;
		}
	}
	data.Resize(j);
	iTrksIdv.resize(j);
	if(seq.m_measNormalized)
		data.SetImageLocations(data.xs(), seq.m_K);
	else
	{
		data.SwapImageLocations(data.xs());
		seq.GetIntrinsicMatrix().ImageToNormalizedPlaneN(data.GetImageLocations(), data.xs());
	}
	data.SetImageSize(seq.GetImageWidth(), seq.GetImageHeight());
	if(seq.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT)
		Sequence::RectifyMeasurements(seq.GetIntrinsicRectification(), data.xs());
	else if(seq.GetIntrinsicType() == Sequence::INTRINSIC_VARIABLE)
		Sequence::RectifyMeasurements(seq.GetIntrinsicRectification(iFrm), data.xs());
}

void SequenceSet::GetCameraPairEstimatorData(const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2, 
											 const FeatureMatchList &matches, CameraPairEstimatorData &data) const
{
	const Sequence &seq1 = GetSequence(iSeq1), &seq2 = GetSequence(iSeq2);
	const ushort N = ushort(matches.size());
	data.Resize(N);
	data.SetCameraPair(seq1.GetCamera(iFrm1), seq2.GetCamera(iFrm2));

#if _DEBUG
	assert((seq1.GetFrameState(iFrm1) & FLAG_FRAME_STATE_SOLVED) && (seq2.GetFrameState(iFrm2) & FLAG_FRAME_STATE_SOLVED));
	assert(!seq1.m_measNormalized && !seq2.m_measNormalized);
	const MeasurementState *meaStates1 = seq1.GetFrameMeasurementStates(iFrm1), *meaStates2 = seq2.GetFrameMeasurementStates(iFrm2);
#endif

	FeatureIndex iFtr1, iFtr2;
	const TrackIndex *iTrks1 = seq1.GetFrameTrackIndexes(iFrm1), *iTrks2 = seq2.GetFrameTrackIndexes(iFrm2);
	const Point2D *xs1 = seq1.GetFrameFeatures(iFrm1), *xs2 = seq2.GetFrameFeatures(iFrm2);
	Point2D mean1, mean2;
	mean1.SetZero();
	mean2.SetZero();
	for(ushort i = 0; i < N; ++i)
	{
		matches[i].Get(iFtr1, iFtr2);
#if _DEBUG
		assert((seq1.GetTrackState(iTrks1[iFtr1]) & FLAG_TRACK_STATE_INLIER) && (seq2.GetTrackState(iTrks2[iFtr2])));
		assert(!(meaStates1[iFtr1] & FLAG_MEASUREMENT_STATE_OUTLIER) && !(meaStates2[iFtr2] & FLAG_MEASUREMENT_STATE_OUTLIER));
#endif
		data.X1(i) = seq1.GetPoint(iTrks1[iFtr1]);
		data.X2(i) = seq2.GetPoint(iTrks2[iFtr2]);
		data.x1(i) = xs1[iFtr1];
		data.x2(i) = xs2[iFtr2];
		mean1 += data.x1(i);
		mean2 += data.x2(i);
	}
	const float norm = 1.0f / N;
	mean1 *= norm;
	mean2 *= norm;

	float var1 = 0, var2 = 0;
	for(ushort i = 0; i < N; ++i)
	{
		var1 += data.x1(i).SquaredDistance(mean1);
		var2 += data.x2(i).SquaredDistance(mean2);
	}
	if(var1 > var2)
	{
		data.SetImageSize(seq1.GetImageWidth(), seq1.GetImageHeight());
		data.SwapImageLocations(data.x1s());
		seq1.GetIntrinsicMatrix().ImageToNormalizedPlaneN(data.GetImageLocations(), data.x1s());
		seq2.GetIntrinsicMatrix().ImageToNormalizedPlaneN(data.x2s());
	}
	else
	{
		data.SetImageSize(seq2.GetImageWidth(), seq2.GetImageHeight());
		data.SwapImageLocations(data.x2s());
		seq2.GetIntrinsicMatrix().ImageToNormalizedPlaneN(data.GetImageLocations(), data.x2s());
		seq1.GetIntrinsicMatrix().ImageToNormalizedPlaneN(data.x1s());
	}
	if(seq1.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT)
		Sequence::RectifyMeasurements(seq1.GetIntrinsicRectification(), data.x1s());
	else if(seq1.GetIntrinsicType() == Sequence::INTRINSIC_VARIABLE)
		Sequence::RectifyMeasurements(seq1.GetIntrinsicRectification(iFrm1), data.x1s());
	if(seq2.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT)
		Sequence::RectifyMeasurements(seq2.GetIntrinsicRectification(), data.x2s());
	else if(seq2.GetIntrinsicType() == Sequence::INTRINSIC_VARIABLE)
		Sequence::RectifyMeasurements(seq2.GetIntrinsicRectification(iFrm2), data.x2s());
}