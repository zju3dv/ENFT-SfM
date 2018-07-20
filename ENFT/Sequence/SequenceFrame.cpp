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
#include "Sequence/Sequence.h"
#include "SfM/TranslationScaleSolver.h"
#include "Utility/Utility.h"

void Sequence::InitializeCameras()
{
	const FrameIndex nFrms = GetFramesNumber();
	m_Cs.Resize(nFrms);
	if(m_intrinsicType == INTRINSIC_VARIABLE)
	{
		m_Krs.Resize(nFrms);
		for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
			m_Krs[iFrm].Set(1.0f, 0.0f);
	}
	else
	{
		m_Kr.Set(1.0f, 0.0f);
		m_Krs.Resize(0);
	}
	m_Ps.Resize(nFrms);
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		//m_frmStates[iFrm] &= ~FLAG_FRAME_STATE_KEY_FRAME;
		m_frmStates[iFrm] &= ~FLAG_FRAME_STATE_INITIAL;
		m_frmStates[iFrm] &= ~FLAG_FRAME_STATE_SOLVED;
	}
}

void Sequence::SetCamera(const FrameIndex &iFrm, const Camera &C)
{
	m_Cs[iFrm] = C;
	switch(m_intrinsicType)
	{
	case INTRINSIC_USER_FIXED:	m_Ps[iFrm].Set(C);										break;
	case INTRINSIC_CONSTANT:	m_Ps[iFrm].FromIntrinsicExtrinsic(m_Kr.f(), C);			break;
	case INTRINSIC_VARIABLE:	m_Ps[iFrm].FromIntrinsicExtrinsic(m_Krs[iFrm].f(), C);	break;
	}
}

void Sequence::SetCamera(const FrameIndex &iFrm, const Camera &C, const Camera::IntrinsicParameter &Kr)
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_VARIABLE);
#endif
	m_Cs[iFrm] = C;
	m_Krs[iFrm] = Kr;
	m_Ps[iFrm].FromIntrinsicExtrinsic(Kr.f(), C);
}

void Sequence::SetCamera(const FrameIndex &iFrm, const Camera &C, const float &f)
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_VARIABLE);
#endif
	m_Cs[iFrm] = C;
	m_Krs[iFrm].Set(f, 0.0f);
	m_Ps[iFrm].FromIntrinsicExtrinsic(f, C);
}

void Sequence::SetCamera(const FrameIndex &iFrm, const ProjectiveMatrix &P)
{
	m_Cs[iFrm] = P;
	if(m_intrinsicType == INTRINSIC_VARIABLE)
		m_Krs[iFrm].Set(1.0f, 0.0f);
	m_Ps[iFrm] = P;
}

void Sequence::GetFrameIndexList(const FrameState frmState, FrameIndexList &iFrms) const
{
	iFrms.resize(0);
	const FrameIndex nFrms = GetFramesNumber();
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		if(m_frmStates[iFrm] & frmState)
			iFrms.push_back(iFrm);
	}
}

void Sequence::GetFrameDescriptors(const FrameIndex &iFrm, const FeatureIndexList &iFtrs, AlignedVector<Descriptor> &descs) const
{
#if DESCRIPTOR_TRACK
	FeatureIndex iFtr;
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(iFtrs.size());
	descs.Resize(nFtrs);
	for(FeatureIndex i = 0; i < nFtrs; ++i)
	{
		iFtr = iFtrs[i];
		descs[i] = m_descs[iTrks[iFtr]];
	}
#else
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const Descriptor *ds = m_descs.Data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(iFtrs.size());
	descs.Resize(nFtrs);
	for(FeatureIndex i = 0; i < nFtrs; ++i)
		descs[i] = ds[iFtrs[i]];
#endif
}

void Sequence::GetFrameFeaturesAndDescriptors(const FrameIndex &iFrm, AlignedVector<Point2D> &ftrs, AlignedVector<Descriptor> &descs) const
{
#if DESCRIPTOR_TRACK
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	ftrs.Resize(nFtrs);
	ftrs.CopyFrom((Feature *) m_xs.Data() + iMeaStart);
	descs.Resize(nFtrs);
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
		descs[iFtr] = m_descs[iTrks[iFtr]];
#else
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	ftrs.Resize(nFtrs);
	ftrs.CopyFrom(m_xs.Data() + iMeaStart);
	descs.Resize(nFtrs);
	descs.CopyFrom(m_descs.Data() + iMeaStart);
#endif
}

void Sequence::GetFrameFeaturesAndDescriptors(const FrameIndex &iFrm, const FeatureIndexList &iFtrs, AlignedVector<Point2D> &ftrs, AlignedVector<Descriptor> &descs) const
{
#if DESCRIPTOR_TRACK
	FeatureIndex iFtr;
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const Point2D *xs = m_xs.Data() + iMeaStart;
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(iFtrs.size());
	ftrs.Resize(nFtrs);
	descs.Resize(nFtrs);
	for(FeatureIndex i = 0; i < nFtrs; ++i)
	{
		iFtr = iFtrs[i];
		ftrs[i] = xs[iFtr];
		descs[i] = m_descs[iTrks[iFtr]];
	}
#else
	FeatureIndex iFtr;
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const Point2D *xs = m_xs.Data() + iMeaStart;
	const Descriptor *ds = m_descs.Data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(iFtrs.size());
	ftrs.Resize(nFtrs);
	descs.Resize(nFtrs);
	for(FeatureIndex i = 0; i < nFtrs; ++i)
	{
		iFtr = iFtrs[i];
		ftrs[i] = xs[iFtr];
		descs[i] = ds[iFtr];
	}
#endif
}

void Sequence::GetCameraEstimatorData(const FrameIndex &iFrm, CameraEstimatorData &data, const bool rectData) const
{
	const MeasurementIndex iMeaFirst = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaFirst;
	const Point2D *xs = m_xs.Data() + iMeaFirst;
	FeatureIndex i, j;
	TrackIndex iTrk;
	const FeatureIndex N = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaFirst);
	data.Resize(N);
	for(i = 0, j = 0; i < N; ++i)
	{
		if((iTrk = iTrks[i]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
			data.Set(j++, m_Xs[iTrk], xs[i]);
	}
	data.Resize(j);
	data.SetFocal(m_K.fxy());
	data.SetArsacData(GetImageWidth(), GetImageHeight(), m_K, m_measNormalized);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		RectifyMeasurements(m_Kr, data.xs());
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		RectifyMeasurements(m_Krs[iFrm], data.xs());
}

void Sequence::GetCameraEstimatorData(const FrameIndex &iFrm, CameraEstimatorData &data, MeasurementIndexList &iMeas, const bool rectData) const
{
	const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
	const FeatureIndex N = iMea2 - iMea1;
	data.Resize(N);
	iMeas.resize(N);

	TrackIndex iTrk;
	FeatureIndex i = 0;
	for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea)
	{
		if((iTrk = m_mapMeaToTrk[iMea]) == INVALID_TRACK_INDEX || !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
			continue;
		data.Set(i, m_Xs[iTrk], m_xs[iMea]);
		iMeas[i] = iMea;
		++i;
	}
	data.Resize(i);
	iMeas.resize(i);
	data.SetFocal(m_K.fxy());
	data.SetArsacData(GetImageWidth(), GetImageHeight(), m_K, m_measNormalized);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		RectifyMeasurements(m_Kr, data.xs());
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		RectifyMeasurements(m_Krs[iFrm], data.xs());
}

void Sequence::GetCameraEstimatorDataMarkedTrack(const FrameIndex &iFrm, CameraEstimatorData &data, const std::vector<bool> &trkMarks, const bool rectData) const
{
	const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
	const FeatureIndex N = iMea2 - iMea1;
	data.Resize(N);

	TrackIndex iTrk;
	FeatureIndex i = 0;
	for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea)
	{
		if((iTrk = m_mapMeaToTrk[iMea]) != INVALID_TRACK_INDEX && trkMarks[iTrk] && !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
			data.Set(i++, m_Xs[iTrk], m_xs[iMea]);
	}
	data.Resize(i);
	data.SetFocal(m_K.fxy());
	data.SetArsacData(GetImageWidth(), GetImageHeight(), m_K, m_measNormalized);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		RectifyMeasurements(m_Kr, data.xs());
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		RectifyMeasurements(m_Krs[iFrm], data.xs());
}

void Sequence::GetProjectiveMatrixEstimatorData(const FrameIndex &iFrm, ProjectiveMatrixEstimatorData &data, const bool rectData) const
{
	const MeasurementIndex iMeaFirst = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaFirst;
	const Point2D *xs = m_xs.Data() + iMeaFirst;
	FeatureIndex i, j;
	TrackIndex iTrk;
	const FeatureIndex N = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaFirst);
	data.Resize(N);
	for(i = 0, j = 0; i < N; ++i)
	{
		if((iTrk = iTrks[i]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
			data.Set(j++, m_Xs[iTrk], xs[i]);
	}
	data.Resize(j);
	//iFtrs.resize(j);
	data.SetArsacData(GetImageWidth(), GetImageHeight(), m_K, m_measNormalized);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		RectifyMeasurements(m_Kr, data.xs());
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		RectifyMeasurements(m_Krs[iFrm], data.xs());
}

void Sequence::GetProjectiveMatrixEstimatorData(const FrameIndex &iFrm, ProjectiveMatrixEstimatorData &data, MeasurementIndexList &iMeas, const bool rectData) const
{
	const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
	const FeatureIndex N = iMea2 - iMea1;
	data.Resize(N);
	iMeas.resize(N);

	TrackIndex iTrk;
	FeatureIndex i = 0;
	for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea)
	{
		if((iTrk = m_mapMeaToTrk[iMea]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
		{
			data.Set(i, m_Xs[iTrk], m_xs[iMea]);
			iMeas[i] = iMea;
			++i;
		}
	}
	data.Resize(i);
	iMeas.resize(i);
	data.SetArsacData(GetImageWidth(), GetImageHeight(), m_K, m_measNormalized);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		RectifyMeasurements(m_Kr, data.xs());
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		RectifyMeasurements(m_Krs[iFrm], data.xs());
}

void Sequence::GetProjectiveMatrixEstimatorDataInlier(const FrameIndex &iFrm, ProjectiveMatrixEstimatorDataMetric &data, const bool rectData) const
{
	const MeasurementIndex iMeaFirst = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaFirst;
	const Point2D *xs = m_xs.Data() + iMeaFirst;
	const MeasurementState *meaStates = m_meaStates.data() + iMeaFirst;
	FeatureIndex i, j;
	TrackIndex iTrk;
	const FeatureIndex N = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaFirst);
	data.Resize(N);
	for(i = 0, j = 0; i < N; ++i)
	{
		if((iTrk = iTrks[i]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) && !(meaStates[i] & FLAG_MEASUREMENT_STATE_OUTLIER))
			data.Set(j++, m_Xs[iTrk], xs[i]);
	}
	data.Resize(j);
	//iFtrs.resize(j);
	data.SetArsacData(GetImageWidth(), GetImageHeight(), m_K, m_measNormalized);
	if(!rectData)
	{
		data.SetFocal(m_K.fxy());
		return;
	}
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
	{
		data.SetFocal(m_K.fxy() * m_Kr.f() * m_Kr.f());
		RectifyMeasurements(m_Kr, data.xs());
	}
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
	{
		data.SetFocal(m_K.fxy() * m_Krs[iFrm].f() * m_Krs[iFrm].f());
		RectifyMeasurements(m_Krs[iFrm], data.xs());
	}
}

void Sequence::GetProjectiveMatrixEstimatorDataMarkedTrack(const FrameIndex &iFrm, ProjectiveMatrixEstimatorData &data, const std::vector<bool> &trkMarks, 
														   const bool rectData) const
{
	const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
	const FeatureIndex N = iMea2 - iMea1;
	data.Resize(N);

	TrackIndex iTrk;
	FeatureIndex i = 0;
	for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea)
	{
		if((iTrk = m_mapMeaToTrk[iMea]) != INVALID_TRACK_INDEX && trkMarks[iTrk] && !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
			data.Set(i++, m_Xs[iTrk], m_xs[iMea]);
	}
	data.Resize(i);
	data.SetArsacData(GetImageWidth(), GetImageHeight(), m_K, m_measNormalized);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		RectifyMeasurements(m_Kr, data.xs());
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		RectifyMeasurements(m_Krs[iFrm], data.xs());
}

FrameIndex Sequence::CountFrames(const FrameState frmState) const
{
	FrameIndex cnt = 0;
	const FrameIndex nFrms = GetFramesNumber();
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		if(m_frmStates[iFrm] & frmState)
			++cnt;
	}
	return cnt;
}

FeatureIndex Sequence::CountFrameInlierTracks(const FrameIndex &iFrm) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	FeatureIndex cnt = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
			++cnt;
	}
	return cnt;
}

FeatureIndex Sequence::CountFrameInlierInitialTracks(const FrameIndex &iFrm) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	FeatureIndex cnt = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INITIAL))
			++cnt;
	}
	return cnt;
}

FeatureIndex Sequence::CountFrameMarkedTracks(const FrameIndex &iFrm, const std::vector<bool> &trkMarks) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	FeatureIndex cnt = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && trkMarks[iTrk])
			++cnt;
	}
	return cnt;
}

FeatureIndex Sequence::CountFrameMarkedInlierTracks(const FrameIndex &iFrm, const std::vector<bool> &trkMarks) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const MeasurementState *meaStates = m_meaStates.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	FeatureIndex cnt = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && trkMarks[iTrk] && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) && !(meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
			++cnt;
	}
	return cnt;
}

FeatureIndex Sequence::CountFrameTrackedFeatures(const FrameIndex &iFrm) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	FeatureIndex cnt = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if(iTrks[iFtr] != INVALID_TRACK_INDEX)
			++cnt;
	}
	return cnt;
}

FeatureIndex Sequence::CountFrameTrackedSiftFeatures(const FrameIndex &iFrm) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const MeasurementState *meaStates = m_meaStates.data() + iMeaStart;
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	FeatureIndex cnt = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if(iTrks[iFtr] != INVALID_TRACK_INDEX && !(meaStates[iFtr] & FLAG_MEASUREMENT_STATE_ENFT))
			++cnt;
	}
	return cnt;
}

FeatureIndex Sequence::CountFrameSolvedFeatures(const FrameIndex &iFrm) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	FeatureIndex cnt = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED))
			++cnt;
	}
	return cnt;
}

FeatureIndex Sequence::CountFrameInlierFeatures(const FrameIndex &iFrm) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const MeasurementState *meaStates = m_meaStates.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	FeatureIndex cnt = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) && !(meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
			++cnt;
	}
	return cnt;
}

void Sequence::GetFrameTrackedFeatureIndexList(const FrameIndex &iFrm, FeatureIndexList &iFtrs) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	iFtrs.resize(0);
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if(iTrks[iFtr] != INVALID_TRACK_INDEX)
			iFtrs.push_back(iFtr);
	}
}

void Sequence::GetFrameSolvedFeatureIndexList(const FrameIndex &iFrm, FeatureIndexList &iFtrs) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	iFtrs.resize(0);
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED))
			iFtrs.push_back(iFtr);
	}
}

void Sequence::GetFrameInlierFeatureIndexList(const FrameIndex &iFrm, FeatureIndexList &iFtrs) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const MeasurementState *meaStates = m_meaStates.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	iFtrs.resize(0);
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) && !(meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
			iFtrs.push_back(iFtr);
	}
}

void Sequence::GetFrameOutlierFeatureIndexList(const FrameIndex &iFrm, FeatureIndexList &iFtrs) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const MeasurementState *meaStates = m_meaStates.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	iFtrs.resize(0);
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && (!(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) || (meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER)))
			iFtrs.push_back(iFtr);
	}
}

void Sequence::UnmarkFramesKeyFrame()
{
	const FrameIndex nFrms = GetFramesNumber();
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
		m_frmStates[iFrm] &= ~FLAG_FRAME_STATE_KEY_FRAME;
}

void Sequence::MarkFrameUnsolved(const FrameIndex &iFrm, const FrameIndex minTrkInliersNum, const float minTrkInliersRatio)
{
	m_frmStates[iFrm] &= ~FLAG_FRAME_STATE_SOLVED;
	const MeasurementIndex iMeaFirst = m_mapFrmToMea[iFrm];
	MeasurementState *meaStates = m_meaStates.data() + iMeaFirst;
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaFirst;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaFirst);
	FrameIndex nCrspsSolved, nCrspsInlier;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		meaStates[iFtr] &= ~FLAG_MEASUREMENT_STATE_OUTLIER;
		CountTrackSolvedFrameInlierMeasurements(iTrks[iFtr], nCrspsSolved, nCrspsInlier);
		if(nCrspsInlier < minTrkInliersNum || nCrspsInlier < FrameIndex(nCrspsSolved * minTrkInliersRatio + 0.5f))
			MarkTrackOutlier(iTrks[iFtr]);
	}
}

void Sequence::MarkFrameTracks(const FrameIndex &iFrm, std::vector<bool> &trkMarks) const
{
	trkMarks.assign(GetTracksNumber(), false);
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX)
			trkMarks[iTrk] = true;
	}
}

void Sequence::MarkFramesTracks(const FrameIndexList &iFrms, std::vector<bool> &trkMarks) const
{
	trkMarks.assign(GetTracksNumber(), false);

	TrackIndex iTrk;
	const FrameIndex nFrms = FrameIndex(iFrms.size());
	for(FrameIndex i = 0; i < nFrms; ++i)
	{
		//MarkFrameTracks(iFrms[i], trkMarks);
		const FrameIndex iFrm = iFrms[i];
		const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
		const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
		const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
		for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
		{
			if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX)
				trkMarks[iTrk] = true;
		}
	}
}

void Sequence::MarkFrameInlierTracks(const FrameIndex &iFrm, std::vector<bool> &trkMarks) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const MeasurementState *meaStates = m_meaStates.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) && !(meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
			trkMarks[iTrk] = true;
	}
}

void Sequence::MarkKeyFrames(const FeatureIndex minNumCmnTrksTwoKeyFrms, const FeatureIndex minNumCmnTrksThreeKeyFrms, std::vector<bool> &frmMarks) const
{
	const FrameIndex nFrms = GetFramesNumber();
	frmMarks.assign(nFrms, false);

	FrameIndex iFrm1, iFrm2, iFrm3;
	iFrm1 = 0;
	frmMarks[iFrm1] = true;
	//for(iFrm2 = iFrm1 + 1; iFrm2 < nFrms && SearchForFrameCommonTracksNumber(iFrm1, iFrm2) >= minNumCmnTrksTwoKeyFrms; ++iFrm2);
	for(iFrm2 = iFrm1 + 1; iFrm2 < nFrms && SearchForFrameCommonTracksNumber(iFrm1, iFrm2) >= minNumCmnTrksThreeKeyFrms; ++iFrm2);
	--iFrm2;
	if(iFrm2 == iFrm1&&iFrm2 != nFrms-1)//if(iFrm2 == iFrm1)bug
		++iFrm2;
	frmMarks[iFrm2] = true;
	if(iFrm2 == nFrms - 1)
		return;
	FrameIndex iFrm2Max, iFrm2Tmp, iFrm2Mid, iFrm2Dif, iFrm2DifMin, iFrm3Min, iFrm3Max;
	FeatureIndex nTrksCmn12, nTrksCmn23, nTrksCmnDif, nTrksCmnDifMin;
	while(1)//bug too slow
	{
		frmMarks[iFrm2] = false;
		iFrm2Max = iFrm2;
		for(iFrm3Max = iFrm2 + 1; iFrm3Max < nFrms && SearchForFrameCommonTracksNumber(iFrm1, iFrm3Max) >= minNumCmnTrksThreeKeyFrms; ++iFrm3Max);
		if(--iFrm3Max < (iFrm3Min = iFrm1 + 2))
			iFrm3Max = iFrm3Min;
		if(iFrm3Min == nFrms)
			--iFrm3Min;
		for(iFrm3 = iFrm3Max; iFrm3 > iFrm3Min; --iFrm3)
		{
			iFrm2Mid = ((iFrm1 + iFrm3) >> 1);
			if(iFrm2Max > iFrm3 - 1)
				iFrm2Max = iFrm3 - 1;
			nTrksCmnDifMin = INVALID_FEATURE_INDEX;
			for(iFrm2Tmp = iFrm1 + 1, iFrm2 = INVALID_FRAME_INDEX, iFrm2DifMin = INVALID_FRAME_INDEX; iFrm2Tmp <= iFrm2Max; ++iFrm2Tmp)
			{
				if((nTrksCmn23 = SearchForFrameCommonTracksNumber(iFrm2Tmp, iFrm3)) < minNumCmnTrksTwoKeyFrms
				|| (nTrksCmn12 = SearchForFrameCommonTracksNumber(iFrm1, iFrm2Tmp)) < minNumCmnTrksTwoKeyFrms
				|| (nTrksCmnDif = ABS_DIF(nTrksCmn12, nTrksCmn23)) > nTrksCmnDifMin
				|| (iFrm2Dif = ABS_DIF(iFrm2Tmp, iFrm2Mid)) > iFrm2DifMin && nTrksCmnDif == nTrksCmnDifMin)
					continue;
				nTrksCmnDifMin = nTrksCmnDif;
				iFrm2DifMin = iFrm2Dif;
				iFrm2 = iFrm2Tmp;
			}
			if(iFrm2 != INVALID_FRAME_INDEX && SearchForFrameCommonTracksNumber(iFrm1, iFrm2, iFrm3) >= minNumCmnTrksThreeKeyFrms)
				break;
		}
		if(iFrm3 == iFrm3Min)
			iFrm2 = iFrm1 + 1;
		frmMarks[iFrm2] = frmMarks[iFrm3] = true;
		if(iFrm3 == nFrms - 1)
			break;
		iFrm1 = iFrm2;
		iFrm2 = iFrm3;
	}
}

//void Sequence::MarkKeyFrameTracks(const FrameIndexList &iFrmsKey, const TrackIndexList &iTrksKey, const FeatureIndex minNumTrksPerKeyFrm, 
//								  const FeatureIndex minNumCmnTrksTwoKeyFrms, const FeatureIndex minNumCmnTrksThreeKeyFrms, std::vector<bool> &trkMarks) const
//{
//	const TrackIndex nTrks = GetTracksNumber();
//	trkMarks.assign(nTrks, false);
//	const TrackIndex nTrksKey = TrackIndex(iTrksKey.size());
//	for(TrackIndex i = 0; i < nTrksKey; ++i)
//		trkMarks[iTrksKey[i]] = true;
//
//	TrackIndex iTrk;
//	FeatureIndex iFtr;
//	TrackIndexList iTrksMarked, iTrksUnmarked;
//	std::vector<std::pair<FrameIndex, TrackIndex> > iTrksSort;
//	const FrameIndex nFrmsKey = FrameIndex(iFrmsKey.size());
//	for(FrameIndex i = 0; i < nFrmsKey; ++i)
//	{
//		const FrameIndex iFrm = iFrmsKey[i];
//		const FeatureIndex nTrksMarked = CountFrameMarkedTracks(iFrm, trkMarks);
//		if(nTrksMarked >= minNumTrksPerKeyFrm)
//			continue;
//		iTrksSort.resize(0);
//		const TrackIndex *iTrks = GetFrameTrackIndexes(iFrm);
//		const FeatureIndex nFtrs = GetFrameFeaturesNumber(iFrm);
//		for(iFtr = 0; iFtr < nFtrs; ++iFtr)
//		{
//			iTrk = iTrks[iFtr];
//			if(!trkMarks[iTrk])
//				iTrksSort.push_back(std::make_pair(GetTrackLength(iTrk), iTrk));
//		}
//		std::sort(iTrksSort.begin(), iTrksSort.end(), std::greater<std::pair<FrameIndex, TrackIndex> >());
//		const FeatureIndex nTrksRecover = min(minNumTrksPerKeyFrm - nTrksMarked, FeatureIndex(iTrksSort.size()));
//		for(FeatureIndex j = 0; j < nTrksRecover; ++j)
//			trkMarks[iTrksSort[j].second] = true;
//	}
//
//	for(FrameIndex i1 = 0, i2 = 1; i2 < nFrmsKey; i1 = i2, ++i2)
//	{
//		const FrameIndex iFrm1 = iFrmsKey[i1], iFrm2 = iFrmsKey[i2];
//		SearchForFrameCommonTracks(iFrm1, iFrm2, trkMarks, iTrksMarked, iTrksUnmarked);
//		const FeatureIndex nTrksMarked = FeatureIndex(iTrksMarked.size());
//		if(nTrksMarked >= minNumCmnTrksTwoKeyFrms)
//			continue;
//		const FeatureIndex nTrksUnmarked = FeatureIndex(iTrksUnmarked.size());
//		iTrksSort.resize(nTrksUnmarked);
//		for(FeatureIndex j = 0; j < nTrksUnmarked; ++j)
//		{
//			iTrk = iTrksUnmarked[j];
//			iTrksSort[j] = std::make_pair(GetTrackLength(iTrk), iTrk);
//		}
//		std::sort(iTrksSort.begin(), iTrksSort.end(), std::greater<std::pair<FrameIndex, TrackIndex> >());
//		const FeatureIndex nTrksRecover = FeatureIndex(min(minNumCmnTrksTwoKeyFrms - nTrksMarked, nTrksUnmarked));
//		for(FeatureIndex j = 0; j < nTrksRecover; ++j)
//			trkMarks[iTrksSort[j].second] = true;
//	}
//	for(FeatureIndex i1 = 0, i2 = 1, i3 = 2; i3 < nFrmsKey; i1 = i2, i2 = i3, ++i3)
//	{
//		const FrameIndex iFrm1 = iFrmsKey[i1], iFrm2 = iFrmsKey[i2], iFrm3 = iFrmsKey[i3];
//		SearchForFrameCommonTracks(iFrm1, iFrm2, iFrm3, trkMarks, iTrksMarked, iTrksUnmarked);
//		const FeatureIndex nTrksMarked = FeatureIndex(iTrksMarked.size());
//		if(nTrksMarked >= minNumCmnTrksThreeKeyFrms)
//			continue;
//		const FeatureIndex nTrksUnmarked = FeatureIndex(iTrksUnmarked.size());
//		iTrksSort.resize(nTrksUnmarked);
//		for(FeatureIndex j = 0; j < nTrksUnmarked; ++j)
//		{
//			iTrk = iTrksUnmarked[j];
//			iTrksSort[j] = std::make_pair(GetTrackLength(iTrk), iTrk);
//		}
//		std::sort(iTrksSort.begin(), iTrksSort.end(), std::greater<std::pair<FrameIndex, TrackIndex> >());
//		const FeatureIndex nTrksRecover = FeatureIndex(min(minNumCmnTrksThreeKeyFrms - nTrksMarked, nTrksUnmarked));
//		for(FeatureIndex j = 0; j < nTrksRecover; ++j)
//			trkMarks[iTrksSort[j].second] = true;
//	}
//}

class BuekctFeatureCounter
{
public:
	inline void Initialize(const ushort width, const ushort height, const ubyte nBinsX, const ubyte nBinsY, const FeatureIndex maxNumFtrsPerBin)
	{
		IO::Assert(uint(nBinsX) * uint(nBinsY) <= UCHAR_MAX, "nBinsX = %d, nBinsY = %d\n", nBinsX, nBinsY);
		m_nBins = nBinsX * nBinsY;
		m_maxNumFtrsPerBin = maxNumFtrsPerBin;

		m_iBinsX.resize(width + 1);
		const float binWidth = float(width) / nBinsX;
		float x1 = 0, x2 = binWidth;
		ubyte x1i = 0, x2i = ubyte(x2 + 0.5f);
		for(ubyte i = 0; i < nBinsX; ++i, x1 = x2, x2 += binWidth, x1i = x2i, x2i = ubyte(x2 + 0.5f))
		for(ubyte x = x1i; x < x2i; ++x)
			m_iBinsX[x] = i;
		m_iBinsY.resize(height + 1);
		const float binHeight = float(height) / nBinsY;
		float y1 = 0, y2 = binHeight;
		ubyte y1i = 0, y2i = ubyte(y2 + 0.5f);
		int iBin = 0;
		for(uint i = 0; i < nBinsY; ++i, iBin += nBinsX, y1 = y2, y2 += binHeight, y1i = y2i, y2i = ubyte(y2 + 0.5f))
		for(ubyte y = y1i; y < y2i; ++y)
			m_iBinsY[y] = iBin;
	}
	inline void Reset() { m_cntFullBins = 0; m_cntsBinFtr.assign(m_nBins, 0); }
	inline bool IsBinFull(const ubyte &iBin) const { return m_cntsBinFtr[iBin] == INVALID_FEATURE_INDEX; }
	inline void MarkBinFull(const ubyte &iBin) { m_cntsBinFtr[iBin] = INVALID_FEATURE_INDEX; ++m_cntFullBins; }
	inline bool AreAllBinsFull() const { return m_cntFullBins == m_nBins; }
	inline bool AddFeature(const Point2D &x)
	{
		const ubyte iBin = m_iBinsY[size_t(x.y() + 0.5f)] + m_iBinsX[size_t(x.x() + 0.5f)];
		if(IsBinFull(iBin))
			return false;
		else if(++m_cntsBinFtr[iBin] == m_maxNumFtrsPerBin)
			MarkBinFull(iBin);
		return true;
	}
private:

	ubyte m_nBins;
	FeatureIndex m_maxNumFtrsPerBin;
	std::vector<ubyte> m_iBinsX, m_iBinsY;

	ubyte m_cntFullBins;
	std::vector<FeatureIndex> m_cntsBinFtr; 
};

void Sequence::MarkKeyFrameTracks(const FrameIndexList &iFrmsKey, const std::vector<bool> &frmMarksKey, const TrackIndexList &iTrksKey, const ubyte nBinsX, 
								  const ubyte nBinsY, const FeatureIndex minNumFtrsPerBin, const FeatureIndex minNumTrksPerKeyFrm, 
								  const FeatureIndex minNumCmnTrksTwoKeyFrms, const FeatureIndex minNumCmnTrksThreeKeyFrms, 
								  std::vector<bool> &trkMarks) const
{
	IO::Assert(!m_measNormalized, "m_meaNormalized = %d\n", m_measNormalized);
	BuekctFeatureCounter counter;
	counter.Initialize(GetImageWidth(), GetImageHeight(), nBinsX, nBinsY, minNumFtrsPerBin);

	const TrackIndex nTrks = GetTracksNumber();
	trkMarks.assign(nTrks, false);
	const TrackIndex nTrksKey = TrackIndex(iTrksKey.size());
	for(TrackIndex i = 0; i < nTrksKey; ++i)
		trkMarks[iTrksKey[i]] = true;

	FrameIndex iFrm, trkLen;
	TrackIndex iTrk;
	FeatureIndex iFtr, cntTrksMarked;
	std::vector<std::pair<FrameIndex, FeatureIndex> > iFtrsSort;
	const FrameIndex nFrmsKey = FrameIndex(iFrmsKey.size());
	for(FrameIndex i = 0; i < nFrmsKey; ++i)
	{
		counter.Reset();

		iFrm = iFrmsKey[i];
		const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
		const Point2D *xs = m_xs.Data() + iMeaStart;
		const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
		const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
		for(iFtr = cntTrksMarked = 0; iFtr < nFtrs; ++iFtr)
		{
			if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && trkMarks[iTrk])
			{
				counter.AddFeature(xs[iFtr]);
				++cntTrksMarked;
			}
		}
		if(counter.AreAllBinsFull() && cntTrksMarked >= minNumTrksPerKeyFrm)
			continue;

		iFtrsSort.resize(0);
		for(iFtr = 0; iFtr < nFtrs; ++iFtr)
		{
			//if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && !trkMarks[iTrk] && GetTrackLength(iTrk) >= 2)
			//	iFtrsSort.push_back(std::make_pair(GetTrackLength(iTrk), iFtr));
			if((iTrk = iTrks[iFtr]) != INVALID_TRACK_INDEX && !trkMarks[iTrk] && (trkLen = CountTrackMarkedFrameInlierMeasurements(iTrk, frmMarksKey)) >= 2)
				iFtrsSort.push_back(std::make_pair(trkLen, iFtr));
		}
		std::sort(iFtrsSort.begin(), iFtrsSort.end(), std::greater<std::pair<FrameIndex, FeatureIndex> >());

		const FeatureIndex nFtrsCandidate = FeatureIndex(iFtrsSort.size());
		for(FeatureIndex j = 0; j < nFtrsCandidate; ++j)
		{
			iFtr = iFtrsSort[j].second;
			if(!counter.AddFeature(xs[iFtr]))
				continue;
			trkMarks[iTrks[iFtr]] = true;
			++cntTrksMarked;
			if(counter.AreAllBinsFull() && cntTrksMarked >= minNumTrksPerKeyFrm)
				break;
		}
		for(FeatureIndex j = 0; j < nFtrsCandidate && cntTrksMarked < minNumTrksPerKeyFrm; ++j)
		{
			iTrk = iTrks[iFtrsSort[j].second];
			if(!trkMarks[iTrk])
				continue;
			trkMarks[iTrk] = true;
			++cntTrksMarked;
		}
	}

	TrackIndexList iTrksMarked, iTrksUnmarked;
	std::vector<std::pair<FrameIndex, TrackIndex> > iTrksSort;
	for(FrameIndex i1 = 0, i2 = 1; i2 < nFrmsKey; i1 = i2, ++i2)
	{
		const FrameIndex iFrm1 = iFrmsKey[i1], iFrm2 = iFrmsKey[i2];
		SearchForFrameCommonTracks(iFrm1, iFrm2, trkMarks, iTrksMarked, iTrksUnmarked);
		const FeatureIndex nTrksMarked = FeatureIndex(iTrksMarked.size());
		if(nTrksMarked >= minNumCmnTrksTwoKeyFrms)
			continue;
		const FeatureIndex nTrksUnmarked = FeatureIndex(iTrksUnmarked.size());
		iTrksSort.resize(nTrksUnmarked);
		for(FeatureIndex j = 0; j < nTrksUnmarked; ++j)
		{
			iTrk = iTrksUnmarked[j];
			//iTrksSort[j] = std::make_pair(GetTrackLength(iTrk), iTrk);
			iTrksSort[j] = std::make_pair(CountTrackMarkedFrameInlierMeasurements(iTrk, frmMarksKey), iTrk);
		}
		std::sort(iTrksSort.begin(), iTrksSort.end(), std::greater<std::pair<FrameIndex, TrackIndex> >());
		const FeatureIndex nTrksRecover = std::min(FeatureIndex(minNumCmnTrksTwoKeyFrms - nTrksMarked), nTrksUnmarked);
		for(FeatureIndex j = 0; j < nTrksRecover; ++j)
			trkMarks[iTrksSort[j].second] = true;
	}
	for(FeatureIndex i1 = 0, i2 = 1, i3 = 2; i3 < nFrmsKey; i1 = i2, i2 = i3, ++i3)
	{
		const FrameIndex iFrm1 = iFrmsKey[i1], iFrm2 = iFrmsKey[i2], iFrm3 = iFrmsKey[i3];
		SearchForFrameCommonTracks(iFrm1, iFrm2, iFrm3, trkMarks, iTrksMarked, iTrksUnmarked);
		const FeatureIndex nTrksMarked = FeatureIndex(iTrksMarked.size());
		if(nTrksMarked >= minNumCmnTrksThreeKeyFrms)
			continue;
		const FeatureIndex nTrksUnmarked = FeatureIndex(iTrksUnmarked.size());
		iTrksSort.resize(nTrksUnmarked);
		for(FeatureIndex j = 0; j < nTrksUnmarked; ++j)
		{
			iTrk = iTrksUnmarked[j];
			//iTrksSort[j] = std::make_pair(GetTrackLength(iTrk), iTrk);
			iTrksSort[j] = std::make_pair(CountTrackMarkedFrameInlierMeasurements(iTrk, frmMarksKey), iTrk);
		}
		std::sort(iTrksSort.begin(), iTrksSort.end(), std::greater<std::pair<FrameIndex, TrackIndex> >());
		const FeatureIndex nTrksRecover = std::min(FeatureIndex(minNumCmnTrksThreeKeyFrms - nTrksMarked), nTrksUnmarked);
		for(FeatureIndex j = 0; j < nTrksRecover; ++j)
			trkMarks[iTrksSort[j].second] = true;
	}
}

FrameIndex Sequence::ComputeFrameAverageTrackLength(const FrameIndex &iFrm) const
{
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	TrackIndex iTrk;
	uint trkLenSum = 0;
	FeatureIndex trkCnt = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		iTrk = iTrks[iFtr];
		if(iTrk == INVALID_TRACK_INDEX)
			continue;
		trkLenSum += GetTrackLength(iTrk);
		++trkCnt;
	}
	const FrameIndex trkLenAvg = FrameIndex(trkLenSum / trkCnt);
	return trkLenAvg;
}

void Sequence::PushBackFrame()
{
	m_mapFrmToMea.push_back(m_mapFrmToMea.back());
	m_frmStates.push_back(FLAG_FRAME_STATE_DEFAULT);
}

#if DESCRIPTOR_TRACK == 0
void Sequence::PushBackFrame(const Point2D *xs, const Descriptor *descs, const FeatureIndex nFtrsTotal, const FeatureIndex nFtrsSift, const bool measNormalized)
{
	if(measNormalized && !m_measNormalized)
		m_K.ImageToNormalizedPlaneN(m_xs);
	else if(!measNormalized && m_measNormalized)
		m_K.NormalizedPlaneToImageN(m_xs);
	m_measNormalized = measNormalized;

	const FrameIndex iFrm = GetFramesNumber();
	m_frmStates.push_back(FLAG_FRAME_STATE_DEFAULT);

	m_xs.PushBack(xs, nFtrsTotal);
	if(descs)
		m_descs.PushBack(descs, nFtrsTotal);
	const MeasurementState meaStateDefault = FLAG_MEASUREMENT_STATE_DEFAULT;
	const MeasurementState meaStateEnft = FLAG_MEASUREMENT_STATE_ENFT;
	m_meaStates.insert(m_meaStates.end(), nFtrsSift, meaStateDefault);
	m_meaStates.insert(m_meaStates.end(), nFtrsTotal - nFtrsSift, meaStateEnft);
	const MeasurementIndex nMeas = GetMeasurementsNumber();
	m_mapFrmToMea.push_back(nMeas);
	m_mapMeaToFrm.insert(m_mapMeaToFrm.end(), nFtrsTotal, iFrm);

	MeasurementIndex iMea = m_mapFrmToMea[iFrm];
	TrackIndex iTrk = GetTracksNumber();
	const TrackIndex nTrks = iTrk + TrackIndex(nMeas - iMea);
	m_mapTrkToMea.resize(nTrks);
	m_mapMeaToTrk.resize(nMeas);
	for(; iMea < nMeas; ++iMea, ++iTrk)
	{
		m_mapTrkToMea[iTrk].push_back(iMea);
		m_mapMeaToTrk[iMea] = iTrk;
	}
	const TrackIndex nTrksInsert = nTrks - GetTracksNumber();
	//m_mapTrkToPlane.insert(m_mapTrkToPlane.end(), nTrksInsert, INVALID_PLANE_INDEX);
	m_trkStates.insert(m_trkStates.end(), nTrksInsert, FLAG_TRACK_STATE_DEFAULT);
}
#else
void Sequence::PushBackFrame(const Point2D *xs, const Descriptor *descs, const FeatureIndex nFtrsTotal, const FeatureIndex nFtrsSift, const bool measNormalized, 
							 TrackIndexList &iTrks)
{
	if(measNormalized && !m_measNormalized)
		m_K.ImageToNormalizedPlaneN(m_xs);
	else if(!measNormalized && m_measNormalized)
		m_K.NormalizedPlaneToImageN(m_xs);
	m_measNormalized = measNormalized;

	const FrameIndex iFrm = GetFramesNumber();
	m_frmStates.push_back(FLAG_FRAME_STATE_DEFAULT);

	m_xs.PushBack(xs, nFtrsTotal);
	const MeasurementState meaStateSift = FLAG_MEASUREMENT_STATE_DEFAULT;
	const MeasurementState meaStateEnft = FLAG_MEASUREMENT_STATE_ENFT;
	m_meaStates.insert(m_meaStates.end(), nFtrsSift, meaStateSift);
	m_meaStates.insert(m_meaStates.end(), nFtrsTotal - nFtrsSift, meaStateEnft);
	const MeasurementIndex nMeas = GetMeasurementsNumber();
	m_mapFrmToMea.push_back(nMeas);
	m_mapMeaToFrm.insert(m_mapMeaToFrm.end(), nFtrsTotal, iFrm);

	FeatureIndex iFtr;
	MeasurementIndex iMea = m_mapFrmToMea[iFrm];
	TrackIndex iTrk, iTrkNew = GetTracksNumber();
	m_mapMeaToTrk.resize(nMeas);
	for(iFtr = 0; iFtr < nFtrsTotal; ++iFtr, ++iMea)
	{
		if((iTrk = iTrks[iFtr]) == INVALID_TRACK_INDEX)
			iTrk = iTrkNew++;
		else
		{
			m_mapTrkToMea[iTrk].push_back(iMea);
			Descriptor::ApB(m_descs[iTrk], descs[iFtr], m_descs[iTrk]);
		}
		m_mapMeaToTrk[iMea] = iTrk;
	}
	const TrackIndex nTrks = iTrkNew;
	m_mapTrkToMea.resize(nTrks);
	const TrackIndex nTrksInsert = nTrks - GetTracksNumber();
	//m_mapTrkToPlane.insert(m_mapTrkToPlane.end(), nTrksInsert, INVALID_PLANE_INDEX);
	m_trkStates.insert(m_trkStates.end(), nTrksInsert, FLAG_TRACK_STATE_DEFAULT);
	m_descs.EnlargeCapacity(nTrks);
	for(iFtr = 0, iMea = m_mapFrmToMea[iFrm], iTrkNew -= nTrksInsert; iFtr < nFtrsTotal; ++iFtr, ++iMea)
	{
		if(iTrks[iFtr] != INVALID_TRACK_INDEX)
			continue;
		iTrks[iFtr] = iTrkNew;
		m_mapTrkToMea[iTrkNew].push_back(iMea);
		m_descs.PushBack(descs[iFtr]);
		++iTrkNew;
	}
}

void Sequence::PushBackFrame(const Point2D *xs, const FeatureIndex nFtrsTotal, const FeatureIndex nFtrsSift, const bool measNormalized)
{
	if(measNormalized && !m_measNormalized)
		m_K.ImageToNormalizedPlaneN(m_xs);
	else if(!measNormalized && m_measNormalized)
		m_K.NormalizedPlaneToImageN(m_xs);
	m_measNormalized = measNormalized;

	const FrameIndex iFrm = GetFramesNumber();
	m_frmStates.push_back(FLAG_FRAME_STATE_DEFAULT);

	m_xs.PushBack(xs, nFtrsTotal);
	const MeasurementState meaStateSift = FLAG_MEASUREMENT_STATE_DEFAULT;
	const MeasurementState meaStateEnft = FLAG_MEASUREMENT_STATE_ENFT;
	m_meaStates.insert(m_meaStates.end(), nFtrsSift, meaStateSift);
	m_meaStates.insert(m_meaStates.end(), nFtrsTotal - nFtrsSift, meaStateEnft);
	const MeasurementIndex nMeas = GetMeasurementsNumber();
	m_mapFrmToMea.push_back(nMeas);
	m_mapMeaToFrm.insert(m_mapMeaToFrm.end(), nFtrsTotal, iFrm);

	MeasurementIndex iMea = m_mapFrmToMea[iFrm];
	TrackIndex iTrk = GetTracksNumber();
	const TrackIndex nTrks = iTrk + TrackIndex(nMeas - iMea);
	m_mapTrkToMea.resize(nTrks);
	m_mapMeaToTrk.resize(nMeas);
	for(; iMea < nMeas; ++iMea, ++iTrk)
	{
		m_mapTrkToMea[iTrk].push_back(iMea);
		m_mapMeaToTrk[iMea] = iTrk;
	}
	const TrackIndex nTrksInsert = nTrks - GetTracksNumber();
	//m_mapTrkToPlane.insert(m_mapTrkToPlane.end(), nTrksInsert, INVALID_PLANE_INDEX);
	m_trkStates.insert(m_trkStates.end(), nTrksInsert, FLAG_TRACK_STATE_DEFAULT);
}
#endif

void Sequence::PushBackFrame(const Camera &C, const FrameState &frmState, const AlignedVector<Point2D> &xs, const TrackIndexList &iTrks, 
							 const MeasurementStateList &meaStates, const bool measNormalized)
{
	if(measNormalized && !m_measNormalized)
		m_K.ImageToNormalizedPlaneN(m_xs);
	else if(!measNormalized && m_measNormalized)
		m_K.NormalizedPlaneToImageN(m_xs);
	m_measNormalized = measNormalized;

	const FeatureIndex iFrm = GetFramesNumber();
	m_Cs.EnlargeCapacity(iFrm + 1);
	m_Cs.Resize(iFrm + 1);
	if(m_intrinsicType == INTRINSIC_VARIABLE)
	{
		m_Krs.EnlargeCapacity(iFrm + 1);
		m_Krs.Resize(iFrm + 1);
	}
	m_Ps.EnlargeCapacity(m_Cs.Size());
	m_Ps.Resize(m_Cs.Size());
	SetCamera(iFrm, C);
	m_frmStates.push_back(frmState);

	const FeatureIndex nFtrs = FeatureIndex(xs.Size());
	m_mapFrmToMea.push_back(m_mapFrmToMea.back() + nFtrs);
	m_xs.PushBack(xs.Data(), nFtrs);

	MeasurementIndex iMea = m_mapFrmToMea[iFrm];
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr, ++iMea)
		m_mapTrkToMea[iTrks[iFtr]].push_back(iMea);

	m_mapMeaToFrm.insert(m_mapMeaToFrm.end(), nFtrs, iFrm);
	m_mapMeaToTrk.insert(m_mapMeaToTrk.end(), iTrks.begin(), iTrks.end());
	m_meaStates.insert(m_meaStates.end(), meaStates.begin(), meaStates.end());
}

void Sequence::InterpolateFrames(const FrameIndex &iFrmStart, const FrameIndex &iFrmEnd, const FrameIndex &nFrmsInterpolate, FrameIndexList &iFrmsInterpolate) const
{
	iFrmsInterpolate.resize(0);
	const FrameIndex nFrmsInterpolateMax = iFrmEnd - iFrmStart - 1;
	if(nFrmsInterpolate >= nFrmsInterpolateMax)
	{
		for(FrameIndex iFrm = iFrmStart + 1; iFrm < iFrmEnd; ++iFrm)
			iFrmsInterpolate.push_back(iFrm);
	}
	else if(nFrmsInterpolate == 1)
		iFrmsInterpolate.push_back(FrameIndex((iFrmStart + iFrmEnd) >> 1));
	else if(nFrmsInterpolate > 1)
	{
		const float incr = float(iFrmEnd - iFrmStart) / (nFrmsInterpolate + 1);
		float iFrm = iFrmStart + incr;
		for(FrameIndex i = 0; i < nFrmsInterpolate; ++i, iFrm += incr)
			iFrmsInterpolate.push_back(FrameIndex(iFrm + 0.5f));
#if _DEBUG
		assert(iFrmsInterpolate.front() > iFrmStart && iFrmsInterpolate.back() < iFrmEnd);
		for(FrameIndex i = 1; i < nFrmsInterpolate; ++i)
			assert(iFrmsInterpolate[i] != iFrmsInterpolate[i - 1]);
#endif
	}
}

void Sequence::RemoveMarkedFrames(const std::vector<bool> &frmMarks)
{
	const MeasurementIndex nMeasOri = GetMeasurementsNumber();
	MeasurementIndexList iMeasOriToNew(nMeasOri, INVALID_MEASUREMENT_INDEX);

	FrameIndex iFrmOri, iFrmNew;
	MeasurementIndex iMeaOri, iMeaNew;
	const FrameIndex nFrmsOri = GetFramesNumber();
	for(iFrmOri = iFrmNew = 0, iMeaOri = iMeaNew = 0; iFrmOri < nFrmsOri; ++iFrmOri)
	{
		if(frmMarks[iFrmOri])
			continue;
		m_Cs[iFrmNew] = m_Cs[iFrmOri];
		if(m_intrinsicType == INTRINSIC_VARIABLE)
			m_Krs[iFrmNew] = m_Krs[iFrmOri];
		m_Ps[iFrmNew] = m_Ps[iFrmOri];
		m_frmStates[iFrmNew] = m_frmStates[iFrmOri];

		const MeasurementIndex iMeaOri1 = m_mapFrmToMea[iFrmOri], iMeaOri2 = m_mapFrmToMea[iFrmOri + 1];
		m_mapFrmToMea[iFrmNew] = iMeaNew;
		for(iMeaOri = iMeaOri1; iMeaOri < iMeaOri2; ++iMeaOri, ++iMeaNew)
		{
			m_xs[iMeaNew] = m_xs[iMeaOri];
			m_meaStates[iMeaNew] = m_meaStates[iMeaOri];
			m_mapMeaToFrm[iMeaNew] = iFrmNew;
			m_mapMeaToTrk[iMeaNew] = m_mapMeaToTrk[iMeaOri];
			iMeasOriToNew[iMeaOri] = iMeaNew;
		}
		++iFrmNew;
	}
	const FrameIndex nFrmsNew = iFrmNew;
	m_Cs.Resize(nFrmsNew);
	if(m_intrinsicType == INTRINSIC_VARIABLE)
		m_Krs.Resize(nFrmsNew);
	m_Ps.Resize(nFrmsNew);
	m_frmStates.resize(nFrmsNew);
	m_mapFrmToMea[nFrmsNew] = iMeaNew;
	m_mapFrmToMea.resize(nFrmsNew + 1);

	const MeasurementIndex nMeasNew = iMeaNew;
	m_xs.Resize(nMeasNew);
	m_meaStates.resize(nMeasNew);
	m_mapMeaToFrm.resize(nMeasNew);
	m_mapMeaToTrk.resize(nMeasNew);

	FrameIndex i, j;
	const TrackIndex nTrks = GetTracksNumber();
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
		const FrameIndex nCrsps = FrameIndex(iMeas.size());
		for(i = j = 0; i < nCrsps; ++i)
		{
			if((iMeaNew = iMeasOriToNew[iMeas[i]]) != INVALID_MEASUREMENT_INDEX)
				iMeas[j++] = iMeaNew;
		}
		iMeas.resize(j);
	}
}

void Sequence::ComputeProjectiveMatrixes()
{
	//NormalizeMeasurements();
	const FrameIndex nFrms = FrameIndex(m_Ps.Size());
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
			continue;
		switch(m_intrinsicType)
		{
		case INTRINSIC_USER_FIXED:	m_Ps[iFrm].Set(m_Cs[iFrm]);										break;
		case INTRINSIC_CONSTANT:	m_Ps[iFrm].FromIntrinsicExtrinsic(m_Kr.f(), m_Cs[iFrm]);		break;
		case INTRINSIC_VARIABLE:	m_Ps[iFrm].FromIntrinsicExtrinsic(m_Krs[iFrm].f(), m_Cs[iFrm]);	break;
		}
	}
}

bool Sequence::ComputeFrameDepthRange(const FrameIndex &iFrm, const float &ratio, float &depthAvg, float &depthMin, float &depthMax) const
{
	std::vector<float> depths;
	const FeatureIndex nFtrs = GetFrameFeaturesNumber(iFrm);
	if(nFtrs == 0)
	{
		depthAvg = depthMin = depthMax = 0;
		return false;
	}
	const TrackIndex *iTrks = GetFrameTrackIndexes(iFrm);
	const MeasurementState *meaStates = GetFrameMeasurementStates(iFrm);
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		const TrackIndex iTrk = iTrks[iFtr];
		if(iTrk != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) && !(meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
			depths.push_back(m_Cs[iFrm].ComputeDepth(m_Xs[iTrk]));
	}

	const FeatureIndex nInliers = FeatureIndex(depths.size());
	if(nInliers == 0)
	{
		depthAvg = depthMin = depthMax = 0;
		return false;
	}
	else if(nInliers == 1)
	{
		depthAvg = depthMin = depthMax = depths[0];
		return true;
	}
	depthAvg = 0;
	for(FeatureIndex i = 0; i < nInliers; ++i)
		depthAvg += depths[i];
	depthAvg /= nInliers;

	std::vector<float> dDepths1, dDepths2;
	for(FeatureIndex i = 0; i < nInliers; ++i)
	{
		if(depths[i] > depthAvg)
			dDepths1.push_back(depths[i] - depthAvg);
		else
			dDepths2.push_back(depthAvg - depths[i]);
	}
	const TrackIndex N1 = TrackIndex(dDepths1.size()), ith1 = TrackIndex((N1 - 1) * ratio);
	const TrackIndex N2 = TrackIndex(dDepths2.size()), ith2 = TrackIndex((N2 - 1) * ratio);
	std::nth_element(dDepths1.begin(), dDepths1.begin() + ith1, dDepths1.end());
	std::nth_element(dDepths2.begin(), dDepths2.begin() + ith2, dDepths2.end());
	depthMax = depthAvg + dDepths1[ith1];
	depthMin = depthAvg - dDepths2[ith2];

	//printf("Frame %d: depth average = %f, range = (%f, %f)\n", iFrm, depthAvg, depthMin, depthMax);
	return true;
}

float Sequence::ComputeFrameMSE(const FrameIndex &iFrm) const
{
	const Camera &C = m_Cs[iFrm];
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const MeasurementState *meaStates = m_meaStates.data() + iMeaStart;
	const Point2D *xs = m_xs.Data() + iMeaStart;

	TrackIndex iTrk;
	Point2D xr, e;

	FeatureIndex cnt = 0;
	float SSE = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) == INVALID_TRACK_INDEX || !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) || (meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
			continue;
		if(m_measNormalized)
			xr = xs[iFtr];
		else
			m_K.ImageToNormalizedPlane(xs[iFtr], xr);
		if(m_intrinsicType == INTRINSIC_CONSTANT)
			m_Kr.Rectify(xr);
		else if(m_intrinsicType == INTRINSIC_VARIABLE)
			m_Krs[iFrm].Rectify(xr);
		SSE += C.ComputeProjectionSquaredError(m_Xs[iTrk], xr, e);
		++cnt;
	}
	float MSE = SSE * m_K.fxy() / cnt;
	if(m_intrinsicType == INTRINSIC_CONSTANT)
		MSE *= m_Kr.f() * m_Kr.f();
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		MSE *= m_Krs[iFrm].f() * m_Krs[iFrm].f();
	return MSE;
}

void Sequence::ComputeFrameInlierRatioAndMSE(const FrameIndex &iFrm, FeatureIndex &nInliers, float &inlierRatio, float &SSE, float &MSE) const
{
	const Camera &C = m_Cs[iFrm];
	const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
	const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart);
	const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
	const MeasurementState *meaStates = m_meaStates.data() + iMeaStart;
	const Point2D *xs = m_xs.Data() + iMeaStart;

	TrackIndex iTrk;
	Point2D xr, e;

	nInliers = 0;
	SSE = 0;
	for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
	{
		if((iTrk = iTrks[iFtr]) == INVALID_TRACK_INDEX || !(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED) || !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER)
		|| (meaStates[iFtr] & FLAG_MEASUREMENT_STATE_OUTLIER))
			continue;
		if(m_measNormalized)
			xr = xs[iFtr];
		else
			m_K.ImageToNormalizedPlane(xs[iFtr], xr);
		if(m_intrinsicType == INTRINSIC_CONSTANT)
			m_Kr.Rectify(xr);
		else if(m_intrinsicType == INTRINSIC_VARIABLE)
			m_Krs[iFrm].Rectify(xr);
		SSE += C.ComputeProjectionSquaredError(m_Xs[iTrk], xr, e);
		//if(iTrk == 950)
		//{
		//	if(m_intrinsicType == INTRINSIC_CONSTANT)
		//		e *= m_Kr.f();
		//	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		//		e *= m_Krs[iFrm].f();
		//	printf("%f\n", m_K.fxy() * e.SquaredLength());
		//}
		++nInliers;
	}
	inlierRatio = float(nInliers) / CountFrameSolvedFeatures(iFrm);
	SSE *= m_K.fxy();
	MSE = SSE / nInliers;
	if(m_intrinsicType == INTRINSIC_CONSTANT)
		MSE *= m_Kr.f() * m_Kr.f();
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		MSE *= m_Krs[iFrm].f() * m_Krs[iFrm].f();
}

void Sequence::ComputeFrameFeaturesDistribution(const FrameIndex &iFrm, const FeatureIndexList &iFtrs, Point2D &mean, LA::Vector3f &cov) const
{
	if(iFtrs.empty())
	{
		mean.SetZero();
		cov.SetZero();
		return;
	}
	mean.SetZero();
	const Point2D *xs = GetFrameFeatures(iFrm);
	const FeatureIndex nFtrs = ushort(iFtrs.size());
	for(FeatureIndex i = 0; i < nFtrs; ++i)
		mean += xs[iFtrs[i]];
	float norm = 1.0f / nFtrs;
	mean *= norm;

	Point2D dx;
	cov.SetZero();
	for(FeatureIndex i = 0; i < nFtrs; ++i)
	{
		const Point2D &x = xs[iFtrs[i]];
		dx.Set(x.x() - mean.x(), x.y() - mean.y());
		cov.v0() += dx.x() * dx.x();
		cov.v1() += dx.x() * dx.y();
		cov.v2() += dx.y() * dx.y();
	}
	norm = 1.0f / (nFtrs - 1);
	cov *= norm;

	if(m_measNormalized)
	{
		m_K.NormalizedPlaneToImage(mean);
		cov.v0() *= m_K.fx() * m_K.fx();
		cov.v1() *= m_K.fxy();
		cov.v2() *= m_K.fy() * m_K.fy();
	}
}