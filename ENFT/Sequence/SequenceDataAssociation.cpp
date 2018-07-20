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
#include "Utility/Utility.h"

void Sequence::GetSubSequence(const FrameIndexList &iFrms, Sequence &seqSub, TrackIndexList &iTrks, const bool copyDesc, const bool copyClr) const
{
	iTrks.resize(0);
	TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
	FrameIndex iFrmSrc, iFrmDst;
	TrackIndex iTrkSrc, iTrkDst = 0;
	MeasurementIndex iMeaSrc, iMeaDst = 0;
	const FrameIndex nFrmsDst = FrameIndex(iFrms.size());
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrms[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			if(iTrksSrcToDst[iTrkSrc] == INVALID_TRACK_INDEX)
			{
				iTrks.push_back(iTrkSrc);
				iTrksSrcToDst[iTrkSrc] = iTrkDst++;
			}
			++iMeaDst;
		}
	}
	const TrackIndex nTrksDst = iTrkDst;
	const MeasurementIndex nMeasDst = iMeaDst;

	m_tag.GetSubSequence(iFrms, seqSub.m_tag);
	seqSub.m_K = m_K;
	seqSub.m_Kr = m_Kr;
	seqSub.m_intrinsicType = m_intrinsicType;
	const bool copyCam = GetCamerasNumber() == GetFramesNumber();
	const bool copyPt = GetPointsNumber() == GetTracksNumber();
	seqSub.Resize(nFrmsDst, nTrksDst, nMeasDst, copyCam, copyPt);
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrms[iFrmDst];
		if(copyCam)
		{
			seqSub.m_Cs[iFrmDst] = m_Cs[iFrmSrc];
			if(m_intrinsicType == INTRINSIC_VARIABLE)
				seqSub.m_Krs[iFrmDst] = m_Krs[iFrmSrc];
		}
		seqSub.m_frmStates[iFrmDst] = m_frmStates[iFrmSrc];
	}
#if DESCRIPTOR_TRACK
	if(copyDesc)
		seqSub.m_descs.Resize(nTrksDst);
	else
		seqSub.m_descs.Clear();
#endif
	if(copyClr)
		seqSub.m_trkClrs.resize(nTrksDst);
	else
		seqSub.m_trkClrs.clear();
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrks[iTrkDst];
		if(copyPt)
			seqSub.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
		//seqSub.m_mapTrkToPlane[iTrkDst] = m_mapTrkToPlane[iTrkSrc];
		seqSub.m_trkStates[iTrkDst] = m_trkStates[iTrkSrc];
#if DESCRIPTOR_TRACK
		if(copyDesc)
			seqSub.m_descs[iTrkDst] = m_descs[iTrkSrc];
#endif
		if(copyClr)
			seqSub.m_trkClrs[iTrkDst] = m_trkClrs[iTrkSrc];
	}

#if DESCRIPTOR_TRACK == 0
	if(copyDesc)
		seqSub.m_descs.Resize(nMeasDst);
	else
		seqSub.m_descs.Clear();
#endif

	// Step5: copy sub-sequence frame inlier measurements and create correspondence maps
	//iMeasKeyInlier.resize(0);
	iMeaDst = 0;
	seqSub.m_mapFrmToMea[0] = 0;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrms[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX || (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			seqSub.m_xs[iMeaDst] = m_xs[iMeaSrc];
			seqSub.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			seqSub.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			seqSub.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			seqSub.m_meaStates[iMeaDst] = m_meaStates[iMeaSrc];
#if DESCRIPTOR_TRACK == 0
			if(copyDesc)
				seqSub.m_descs[iMeaDst] = m_descs[iMeaSrc];
#endif
			++iMeaDst;
		}
		seqSub.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
	}
	seqSub.m_measNormalized = m_measNormalized;
}

void Sequence::GetSubSequence(const FrameIndexList &iFrms, Sequence &seqSub, TrackIndexList &iTrks, MeasurementIndexList &iMeas, const bool copyDesc, 
							  const bool copyClr) const
{
	iTrks.resize(0);
	TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
	FrameIndex iFrmSrc, iFrmDst;
	TrackIndex iTrkSrc, iTrkDst = 0;
	MeasurementIndex iMeaSrc, iMeaDst = 0;
	const FrameIndex nFrmsDst = FrameIndex(iFrms.size());
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrms[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			if(iTrksSrcToDst[iTrkSrc] == INVALID_TRACK_INDEX)
			{
				iTrks.push_back(iTrkSrc);
				iTrksSrcToDst[iTrkSrc] = iTrkDst++;
			}
			++iMeaDst;
		}
	}
	const TrackIndex nTrksDst = iTrkDst;
	const MeasurementIndex nMeasDst = iMeaDst;

	m_tag.GetSubSequence(iFrms, seqSub.m_tag);
	seqSub.m_K = m_K;
	seqSub.m_Kr = m_Kr;
	seqSub.m_intrinsicType = m_intrinsicType;
	const bool copyCam = GetCamerasNumber() == GetFramesNumber();
	const bool copyPt = GetPointsNumber() == GetTracksNumber();
	seqSub.Resize(nFrmsDst, nTrksDst, nMeasDst, copyCam, copyPt);
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrms[iFrmDst];
		if(copyCam)
		{
			seqSub.m_Cs[iFrmDst] = m_Cs[iFrmSrc];
			if(m_intrinsicType == INTRINSIC_VARIABLE)
				seqSub.m_Krs[iFrmDst] = m_Krs[iFrmSrc];
		}
		seqSub.m_frmStates[iFrmDst] = m_frmStates[iFrmSrc];
	}
#if DESCRIPTOR_TRACK
	if(copyDesc)
		seqSub.m_descs.Resize(nTrksDst);
	else
		seqSub.m_descs.Clear();
#endif
	if(copyClr)
		seqSub.m_trkClrs.resize(nTrksDst);
	else
		seqSub.m_trkClrs.clear();
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrks[iTrkDst];
		if(copyPt)
			seqSub.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
		//seqSub.m_mapTrkToPlane[iTrkDst] = m_mapTrkToPlane[iTrkSrc];
		seqSub.m_trkStates[iTrkDst] = m_trkStates[iTrkSrc];
#if DESCRIPTOR_TRACK
		if(copyDesc)
			seqSub.m_descs[iTrkDst] = m_descs[iTrkSrc];
#endif
		if(copyClr)
			seqSub.m_trkClrs[iTrkDst] = m_trkClrs[iTrkSrc];
	}

#if DESCRIPTOR_TRACK == 0
	if(copyDesc)
		seqSub.m_descs.Resize(nMeasDst);
	else
		seqSub.m_descs.Clear();
#endif

	// Step5: copy sub-sequence frame inlier measurements and create correspondence maps
	iMeas.resize(nMeasDst);
	iMeaDst = 0;
	seqSub.m_mapFrmToMea[0] = 0;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrms[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX || (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			seqSub.m_xs[iMeaDst] = m_xs[iMeaSrc];
			seqSub.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			seqSub.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			seqSub.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			seqSub.m_meaStates[iMeaDst] = m_meaStates[iMeaSrc];
#if DESCRIPTOR_TRACK == 0
			if(copyDesc)
				seqSub.m_descs[iMeaDst] = m_descs[iMeaSrc];
#endif
			iMeas[iMeaDst] = iMeaSrc;
			++iMeaDst;
		}
		seqSub.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
	}
	seqSub.m_measNormalized = m_measNormalized;
}

void Sequence::GetSubSequence(const FrameIndexList &iFrms, const TrackIndexList &iTrks, Sequence &seqSub, const bool copyDesc, const bool copyClr) const
{
	// Step1: Mark subsequence frames
	FrameIndex iFrmSrc, iFrmDst;
	const FrameIndex nFrmsSrc = GetFramesNumber();
	std::vector<bool> frmMarksSrc(nFrmsSrc, false);
	const FrameIndex nFrmsDst = FrameIndex(iFrms.size());
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
		frmMarksSrc[iFrms[iFrmDst]] = true;

	// Step2: Create reverse track index list iTrksSrcToDst and count inlier measurements
	TrackIndex iTrkSrc, iTrkDst;
	MeasurementIndex iMeaSrc, iMeaDst;
	TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
	MeasurementIndex nMeasDst = 0;
	const TrackIndex nTrksDst = TrackIndex(iTrks.size());
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrks[iTrkDst];
		iTrksSrcToDst[iTrkSrc] = iTrkDst;

		const MeasurementIndexList &iMeasSrc = m_mapTrkToMea[iTrkSrc];
		const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
		for(FrameIndex j = 0; j < nCrspsSrc; ++j)
		{
			iMeaSrc = iMeasSrc[j];
			if(frmMarksSrc[m_mapMeaToFrm[iMeaSrc]] && !(m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				++nMeasDst;
		}
	}

	// Step3: copy tag, intrinsic matrix, all sub-sequence frame cameras and all sub-sequence frame inlier points
	m_tag.GetSubSequence(iFrms, seqSub.m_tag);
	seqSub.m_K = m_K;
	seqSub.m_Kr = m_Kr;
	seqSub.m_intrinsicType = m_intrinsicType;
	const bool copyCam = GetCamerasNumber() == GetFramesNumber();
	const bool copyPt = GetPointsNumber() == GetTracksNumber();
	seqSub.Resize(nFrmsDst, nTrksDst, nMeasDst, copyCam, copyPt);
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrms[iFrmDst];
		if(copyCam)
		{
			seqSub.m_Cs[iFrmDst] = m_Cs[iFrmSrc];
			if(m_intrinsicType == INTRINSIC_VARIABLE)
				seqSub.m_Krs[iFrmDst] = m_Krs[iFrmSrc];
		}
		seqSub.m_frmStates[iFrmDst] = m_frmStates[iFrmSrc];
	}
#if DESCRIPTOR_TRACK
	if(copyDesc)
		seqSub.m_descs.Resize(nTrksDst);
	else
		seqSub.m_descs.Clear();
#endif
	if(copyClr)
		seqSub.m_trkClrs.resize(nTrksDst);
	else
		seqSub.m_trkClrs.clear();
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrks[iTrkDst];
		if(copyPt)
			seqSub.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
		//seqSub.m_mapTrkToPlane[iTrkDst] = m_mapTrkToPlane[iTrkSrc];
		seqSub.m_trkStates[iTrkDst] = m_trkStates[iTrkSrc];
#if DESCRIPTOR_TRACK
		if(copyDesc)
			seqSub.m_descs[iTrkDst] = m_descs[iTrkSrc];
#endif
		if(copyClr)
			seqSub.m_trkClrs[iTrkDst] = m_trkClrs[iTrkSrc];
	}

#if DESCRIPTOR_TRACK == 0
	if(copyDesc)
		seqSub.m_descs.Resize(nMeasDst);
	else
		seqSub.m_descs.Clear();
#endif

	// Step5: copy sub-sequence frame inlier measurements and create correspondence maps
	//iMeasKeyInlier.resize(0);
	iMeaDst = 0;
	seqSub.m_mapFrmToMea[0] = 0;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrms[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX
			|| (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			seqSub.m_xs[iMeaDst] = m_xs[iMeaSrc];
			seqSub.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			seqSub.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			seqSub.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			seqSub.m_meaStates[iMeaDst] = m_meaStates[iMeaSrc];
#if DESCRIPTOR_TRACK == 0
			if(copyDesc)
				seqSub.m_descs[iMeaDst] = m_descs[iMeaSrc];
#endif
			++iMeaDst;
		}
		seqSub.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
	}
	seqSub.m_measNormalized = m_measNormalized;

	IO::Assert(iMeaDst == nMeasDst, "iMeaDst = %d, nMeasDst = %d\n", iMeaDst, nMeasDst);
}

void Sequence::GetSubSequence(const FrameIndexList &iFrms, const TrackIndexList &iTrks, Sequence &seqSub, MeasurementIndexList &iMeas, const bool copyDesc, 
							  const bool copyClr) const
{
	// Step1: Mark subsequence frames
	FrameIndex iFrmSrc, iFrmDst;
	const FrameIndex nFrmsSrc = GetFramesNumber();
	std::vector<bool> frmMarksSrc(nFrmsSrc, false);
	const FrameIndex nFrmsDst = FrameIndex(iFrms.size());
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
		frmMarksSrc[iFrms[iFrmDst]] = true;

	// Step2: Create reverse track index list iTrksSrcToDst and count inlier measurements
	TrackIndex iTrkSrc, iTrkDst;
	MeasurementIndex iMeaSrc, iMeaDst;
	TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
	MeasurementIndex nMeasDst = 0;
	const TrackIndex nTrksDst = TrackIndex(iTrks.size());
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrks[iTrkDst];
		iTrksSrcToDst[iTrkSrc] = iTrkDst;

		const MeasurementIndexList &iMeasSrc = m_mapTrkToMea[iTrkSrc];
		const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
		for(FrameIndex j = 0; j < nCrspsSrc; ++j)
		{
			const MeasurementIndex iMeaSrc = iMeasSrc[j];
			if(frmMarksSrc[m_mapMeaToFrm[iMeaSrc]] && !(m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				++nMeasDst;
		}
	}

	// Step3: copy tag, intrinsic matrix, all sub-sequence frame cameras and all sub-sequence frame inlier points
	m_tag.GetSubSequence(iFrms, seqSub.m_tag);
	seqSub.m_K = m_K;
	seqSub.m_Kr = m_Kr;
	seqSub.m_intrinsicType = m_intrinsicType;
	const bool copyCam = GetCamerasNumber() == GetFramesNumber();
	const bool copyPt = GetPointsNumber() == GetTracksNumber();
	seqSub.Resize(nFrmsDst, nTrksDst, nMeasDst, copyCam, copyPt);
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrms[iFrmDst];
		if(copyCam)
		{
			seqSub.m_Cs[iFrmDst] = m_Cs[iFrmSrc];
			if(m_intrinsicType == INTRINSIC_VARIABLE)
				seqSub.m_Krs[iFrmDst] = m_Krs[iFrmSrc];
		}
		seqSub.m_frmStates[iFrmDst] = m_frmStates[iFrmSrc];
	}
#if DESCRIPTOR_TRACK
	if(copyDesc)
		seqSub.m_descs.Resize(nTrksDst);
	else
		seqSub.m_descs.Clear();
#endif
	if(copyClr)
		seqSub.m_trkClrs.resize(nTrksDst);
	else
		seqSub.m_trkClrs.clear();
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrks[iTrkDst];
		if(copyPt)
			seqSub.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
		//seqSub.m_mapTrkToPlane[iTrkDst] = m_mapTrkToPlane[iTrkSrc];
		seqSub.m_trkStates[iTrkDst] = m_trkStates[iTrkSrc];
#if DESCRIPTOR_TRACK
		if(copyDesc)
			seqSub.m_descs[iTrkDst] = m_descs[iTrkSrc];
#endif
		if(copyClr)
			seqSub.m_trkClrs[iTrkDst] = m_trkClrs[iTrkSrc];
	}

#if DESCRIPTOR_TRACK == 0
	if(copyDesc)
		seqSub.m_descs.Resize(nMeasDst);
	else
		seqSub.m_descs.Clear();
#endif

	// Step5: copy sub-sequence frame inlier measurements and create correspondence maps
	//iMeasKeyInlier.resize(0);
	iMeaDst = 0;
	seqSub.m_mapFrmToMea[0] = 0;
	iMeas.resize(nMeasDst);
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrms[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX || (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			seqSub.m_xs[iMeaDst] = m_xs[iMeaSrc];
			seqSub.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			seqSub.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			seqSub.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			seqSub.m_meaStates[iMeaDst] = m_meaStates[iMeaSrc];
#if DESCRIPTOR_TRACK == 0
			if(copyDesc)
				seqSub.m_descs[iMeaDst] = m_descs[iMeaSrc];
#endif
			iMeas[iMeaDst] = iMeaSrc;
			++iMeaDst;
		}
		seqSub.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
	}
	seqSub.m_measNormalized = m_measNormalized;

	IO::Assert(iMeaDst == nMeasDst, "iMeaDst = %d, nMeasDst = %d\n", iMeaDst, nMeasDst);
}

FeatureIndex Sequence::SearchForFrameCommonTracksNumber(const FrameIndex &iFrm1, const FrameIndex &iFrm2) const
{
	const MeasurementIndex iMeaStart1 = m_mapFrmToMea[iFrm1];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMeaStart1;
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMeaStart1, nFtrs2 = GetFrameFeaturesNumber(iFrm2);
	TrackIndex iTrk;
	FeatureIndex cnt = 0;
	for(FeatureIndex iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) != INVALID_TRACK_INDEX && SearchTrackForFrameMeasurementIndex(iTrk, iFrm2) != INVALID_MEASUREMENT_INDEX)
			++cnt;
	}
	return cnt;
}

FeatureIndex Sequence::SearchForFrameCommonTracksNumber(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3) const
{
	const MeasurementIndex iMeaStart1 = m_mapFrmToMea[iFrm1];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMeaStart1;
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMeaStart1, nFtrs2 = GetFrameFeaturesNumber(iFrm2);
	TrackIndex iTrk;
	FeatureIndex cnt = 0;
	for(FeatureIndex iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) != INVALID_TRACK_INDEX && SearchTrackForFrameMeasurementIndex(iTrk, iFrm2) != INVALID_MEASUREMENT_INDEX
		&& SearchTrackForFrameMeasurementIndex(iTrk, iFrm3) != INVALID_MEASUREMENT_INDEX)
			++cnt;
	}
	return cnt;
}

void Sequence::SearchForFrameCommonTracks(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const std::vector<bool> &trkMarks, TrackIndexList &iTrksMarked, 
										  TrackIndexList &iTrksUnmarked) const
{
	const MeasurementIndex iMeaStart1 = m_mapFrmToMea[iFrm1];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMeaStart1;
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMeaStart1, nFtrs2 = GetFrameFeaturesNumber(iFrm2);
	TrackIndex iTrk;
	iTrksMarked.resize(0);
	iTrksUnmarked.resize(0);
	for(FeatureIndex iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) == INVALID_TRACK_INDEX || SearchTrackForFrameMeasurementIndex(iTrk, iFrm2) == INVALID_MEASUREMENT_INDEX)
			continue;
		else if(trkMarks[iTrk])
			iTrksMarked.push_back(iTrk);
		else
			iTrksUnmarked.push_back(iTrk);
	}
}

void Sequence::SearchForFrameCommonTracks(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3, const std::vector<bool> &trkMarks, 
										  TrackIndexList &iTrksMarked, TrackIndexList &iTrksUnmarked) const
{
	const MeasurementIndex iMeaStart1 = m_mapFrmToMea[iFrm1];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMeaStart1;
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMeaStart1, nFtrs2 = GetFrameFeaturesNumber(iFrm2);
	TrackIndex iTrk;
	iTrksMarked.resize(0);
	iTrksUnmarked.resize(0);
	for(FeatureIndex iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) == INVALID_TRACK_INDEX || SearchTrackForFrameMeasurementIndex(iTrk, iFrm2) == INVALID_MEASUREMENT_INDEX
		|| SearchTrackForFrameMeasurementIndex(iTrk, iFrm3) == INVALID_MEASUREMENT_INDEX)
			continue;
		else if(trkMarks[iTrk])
			iTrksMarked.push_back(iTrk);
		else
			iTrksUnmarked.push_back(iTrk);
	}
}

void Sequence::SearchForFrameFeatureMatches(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches) const
{
	const MeasurementIndex iMea1Start = m_mapFrmToMea[iFrm1], iMea2Start = m_mapFrmToMea[iFrm2];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMea1Start;
	TrackIndex iTrk;
	MeasurementIndex iMea2;
	FeatureIndex iFtr1, iFtr2;
	matches.resize(0);
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMea1Start, nFtrs2 = m_mapFrmToMea[iFrm2 + 1] - iMea2Start;
	for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) == INVALID_TRACK_INDEX || (iMea2 = SearchTrackForFrameMeasurementIndex(iTrk, iFrm2)) == INVALID_MEASUREMENT_INDEX)
			continue;
		iFtr2 = iMea2 - iMea2Start;
		matches.push_back(FeatureMatch(iFtr1, iFtr2));
	}
}

void Sequence::SearchForFrameFeatureMatches(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, TrackIndexList &iTrks) const
{
	const MeasurementIndex iMea1Start = m_mapFrmToMea[iFrm1], iMea2Start = m_mapFrmToMea[iFrm2];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMea1Start;
	TrackIndex iTrk;
	MeasurementIndex iMea2;
	FeatureIndex iFtr1, iFtr2;
	matches.resize(0);
	iTrks.resize(0);
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMea1Start, nFtrs2 = m_mapFrmToMea[iFrm2 + 1] - iMea2Start;
	for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) == INVALID_TRACK_INDEX || (iMea2 = SearchTrackForFrameMeasurementIndex(iTrk, iFrm2)) == INVALID_MEASUREMENT_INDEX)
			continue;
		iFtr2 = iMea2 - iMea2Start;
		matches.push_back(FeatureMatch(iFtr1, iFtr2));
		iTrks.push_back(iTrk);
	}
}

static inline void GetMatchSet2D(const Point2D *x1s, const Point2D *x2s, const FeatureMatchList &matches, MatchSet2D &data)
{
	const ushort N = ushort(matches.size());
	data.Resize(N);
	for(ushort i = 0; i < N; ++i)
		data.SetMatch(i, x1s[matches[i].GetIndex1()], x2s[matches[i].GetIndex2()]);
	data.FinishSettingMatches();
}

void Sequence::SearchForFrameFeatureMatches(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, TrackIndexList &iTrks, MatchSet2D &data, 
											const bool rectData) const
{
	SearchForFrameFeatureMatches(iFrm1, iFrm2, matches, iTrks);
	GetMatchSet2D(GetFrameFeatures(iFrm1), GetFrameFeatures(iFrm2), matches, data);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		data.Rectify(m_Kr, m_Kr);
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		data.Rectify(m_Krs[iFrm1], m_Krs[iFrm2]);
}

void Sequence::SearchForFrameFeatureMatches(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, MatchSet2D &data, 
											const bool rectData) const
{
	SearchForFrameFeatureMatches(iFrm1, iFrm2, matches);
	GetMatchSet2D(GetFrameFeatures(iFrm1), GetFrameFeatures(iFrm2), matches, data);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		data.Rectify(m_Kr, m_Kr);
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		data.Rectify(m_Krs[iFrm1], m_Krs[iFrm2]);
}

void Sequence::SearchForFrameFeatureMatchesMarkedTrack(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const std::vector<bool> &trkMarks, 
													   FeatureMatchList &matches, MatchSet2D &data, const bool rectData) const
{
	const MeasurementIndex iMea1Start = m_mapFrmToMea[iFrm1], iMea2Start = m_mapFrmToMea[iFrm2];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMea1Start;
	TrackIndex iTrk;
	MeasurementIndex iMea2;
	FeatureIndex iFtr1, iFtr2;
	matches.resize(0);
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMea1Start, nFtrs2 = m_mapFrmToMea[iFrm2 + 1] - iMea2Start;
	for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) == INVALID_TRACK_INDEX || !trkMarks[iTrk]
		|| (iMea2 = SearchTrackForFrameMeasurementIndex(iTrk, iFrm2)) == INVALID_MEASUREMENT_INDEX)
			continue;
		iFtr2 = iMea2 - iMea2Start;
		matches.push_back(FeatureMatch(iFtr1, iFtr2));
	}
	GetMatchSet2D(GetFrameFeatures(iFrm1), GetFrameFeatures(iFrm2), matches, data);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		data.Rectify(m_Kr, m_Kr);
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		data.Rectify(m_Krs[iFrm1], m_Krs[iFrm2]);
}

void Sequence::SearchForFrameFeatureMatchesUnmarkedTrack(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const std::vector<bool> &trkMarks, 
														 FeatureMatchList &matches, TrackIndexList &iTrks, MatchSet2D &data, const bool rectData) const
{
	const MeasurementIndex iMea1Start = m_mapFrmToMea[iFrm1], iMea2Start = m_mapFrmToMea[iFrm2];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMea1Start;
	TrackIndex iTrk;
	MeasurementIndex iMea2;
	FeatureIndex iFtr1, iFtr2;
	matches.resize(0);
	iTrks.resize(0);
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMea1Start, nFtrs2 = m_mapFrmToMea[iFrm2 + 1] - iMea2Start;
	for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) == INVALID_TRACK_INDEX || trkMarks[iTrk]
		|| (iMea2 = SearchTrackForFrameMeasurementIndex(iTrk, iFrm2)) == INVALID_MEASUREMENT_INDEX)
			continue;
		iFtr2 = iMea2 - iMea2Start;
		matches.push_back(FeatureMatch(iFtr1, iFtr2));
		iTrks.push_back(iTrk);
	}
	GetMatchSet2D(GetFrameFeatures(iFrm1), GetFrameFeatures(iFrm2), matches, data);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		data.Rectify(m_Kr, m_Kr);
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		data.Rectify(m_Krs[iFrm1], m_Krs[iFrm2]);
}

void Sequence::SearchForFrameFeatureMatchesInlierTrackAndMeasurement(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, 
																	 MatchSet2D &data, const bool rectData) const
{
	const MeasurementIndex iMea1Start = m_mapFrmToMea[iFrm1], iMea2Start = m_mapFrmToMea[iFrm2];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMea1Start;
	const MeasurementState *meaStates1 = m_meaStates.data() + iMea1Start;
	TrackIndex iTrk;
	MeasurementIndex iMea2;
	FeatureIndex iFtr1, iFtr2;
	matches.resize(0);
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMea1Start, nFtrs2 = m_mapFrmToMea[iFrm2 + 1] - iMea2Start;
	for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) == INVALID_TRACK_INDEX || !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) || (meaStates1[iFtr1] & FLAG_MEASUREMENT_STATE_OUTLIER)
		|| (iMea2 = SearchTrackForFrameMeasurementIndex(iTrk, iFrm2)) == INVALID_MEASUREMENT_INDEX || (m_meaStates[iMea2] & FLAG_MEASUREMENT_STATE_OUTLIER))
			continue;
		iFtr2 = iMea2 - iMea2Start;
		matches.push_back(FeatureMatch(iFtr1, iFtr2));
	}
	GetMatchSet2D(GetFrameFeatures(iFrm1), GetFrameFeatures(iFrm2), matches, data);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		data.Rectify(m_Kr, m_Kr);
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		data.Rectify(m_Krs[iFrm1], m_Krs[iFrm2]);
}

void Sequence::SearchForFrameFeatureMatchesInlierTrackAndMeasurement(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, 
																	 TrackIndexList &iTrks) const
{
	const MeasurementIndex iMea1Start = m_mapFrmToMea[iFrm1], iMea2Start = m_mapFrmToMea[iFrm2];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMea1Start;
	const MeasurementState *meaStates1 = m_meaStates.data() + iMea1Start;
	TrackIndex iTrk;
	MeasurementIndex iMea2;
	FeatureIndex iFtr1, iFtr2;
	matches.resize(0);
	iTrks.resize(0);
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMea1Start, nFtrs2 = m_mapFrmToMea[iFrm2 + 1] - iMea2Start;
	for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) == INVALID_TRACK_INDEX || !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) || (meaStates1[iFtr1] & FLAG_MEASUREMENT_STATE_OUTLIER)
		|| (iMea2 = SearchTrackForFrameMeasurementIndex(iTrk, iFrm2)) == INVALID_MEASUREMENT_INDEX || (m_meaStates[iMea2] & FLAG_MEASUREMENT_STATE_OUTLIER))
			continue;
		iFtr2 = iMea2 - iMea2Start;
		matches.push_back(FeatureMatch(iFtr1, iFtr2));
		iTrks.push_back(iTrk);
	}
}

void Sequence::SearchForFrameFeatureMatchesInlierMeasurement(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches, MatchSet2D &data, 
															 const bool rectData) const
{
	const MeasurementIndex iMea1Start = m_mapFrmToMea[iFrm1], iMea2Start = m_mapFrmToMea[iFrm2];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMea1Start;
	const MeasurementState *meaStates1 = m_meaStates.data() + iMea1Start;
	TrackIndex iTrk;
	MeasurementIndex iMea2;
	FeatureIndex iFtr1, iFtr2;
	matches.resize(0);
	//iTrks.resize(0);
	const FeatureIndex nFtrs1 = m_mapFrmToMea[iFrm1 + 1] - iMea1Start, nFtrs2 = m_mapFrmToMea[iFrm2 + 1] - iMea2Start;
	for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) == INVALID_TRACK_INDEX || (meaStates1[iFtr1] & FLAG_MEASUREMENT_STATE_OUTLIER)
		|| (iMea2 = SearchTrackForFrameMeasurementIndex(iTrk, iFrm2)) == INVALID_MEASUREMENT_INDEX || (m_meaStates[iMea2] & FLAG_MEASUREMENT_STATE_OUTLIER))
			continue;
		iFtr2 = iMea2 - iMea2Start;
		matches.push_back(FeatureMatch(iFtr1, iFtr2));
		//iTrks.push_back(iTrk);
	}
	GetMatchSet2D(GetFrameFeatures(iFrm1), GetFrameFeatures(iFrm2), matches, data);
	if(!rectData)
		return;
	else if(m_intrinsicType == INTRINSIC_CONSTANT)
		data.Rectify(m_Kr, m_Kr);
	else if(m_intrinsicType == INTRINSIC_VARIABLE)
		data.Rectify(m_Krs[iFrm1], m_Krs[iFrm2]);
}

void Sequence::SearchForSubsequenceCommonTracks(const FrameIndex iFrmStart, const FrameIndex iFrmEnd, TrackIndexList &iTrks, 
												std::vector<MeasurementIndexList> &iMeasList) const
{
	const MeasurementIndex iMeaStart1 = m_mapFrmToMea[iFrmStart], iMeaStart2 = m_mapFrmToMea[iFrmStart + 1];
	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	MeasurementIndexList iMeas;

	iTrks.resize(0);
	iMeasList.resize(0);
	for(MeasurementIndex iMeaStart = iMeaStart1; iMeaStart < iMeaStart2; ++iMeaStart)
	{
		if((iTrk = m_mapMeaToTrk[iMeaStart]) == INVALID_TRACK_INDEX)
			continue;
		iMeas.resize(0);
		iMeas.push_back(iMeaStart);
		for(iFrm = iFrmStart + 1; iFrm <= iFrmEnd; ++iFrm)
		{
			if((iMea = SearchTrackForFrameMeasurementIndex(iTrk, iFrm)) == INVALID_MEASUREMENT_INDEX)
				break;
			else
				iMeas.push_back(iMea);
		}
		if(iFrm <= iFrmEnd)
			continue;
		iTrks.push_back(iTrk);
		iMeasList.push_back(iMeas);
	}
}

void Sequence::ComputeFrameFeatureAverageDisparityPass1(const FrameIndex &iFrm1, FeatureIndexList &mapTrkToFtr1) const
{
	TrackIndex iTrk;
	mapTrkToFtr1.assign(GetTracksNumber(), INVALID_FEATURE_INDEX);
	const MeasurementIndex iMea1Start = m_mapFrmToMea[iFrm1];
	const TrackIndex *iTrks1 = m_mapMeaToTrk.data() + iMea1Start;
	const FeatureIndex nFtrs1 = FeatureIndex(m_mapFrmToMea[iFrm1 + 1] - iMea1Start);
	for(FeatureIndex iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrk = iTrks1[iFtr1]) != INVALID_TRACK_INDEX)
			mapTrkToFtr1[iTrk] = iFtr1;
	}
}

float Sequence::ComputeFrameFeatureAverageDisparityPass2(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureIndexList &mapTrkToFtr1, 
														 FeatureIndex &nCmnPts/*, std::vector<bool> &trkMarks*/) const
{
	TrackIndex iTrk;
	FeatureIndex iFtr1, iFtr2;
	nCmnPts = 0;
	float disp = 0;
	const MeasurementIndex iMea2Start = m_mapFrmToMea[iFrm2];
	const TrackIndex *iTrks2 = m_mapMeaToTrk.data() + iMea2Start;
	const Point2D *ftrs1 = m_xs.Data() + m_mapFrmToMea[iFrm1], *ftrs2 = m_xs.Data() + iMea2Start;
	const FeatureIndex nFtrs2 = FeatureIndex(m_mapFrmToMea[iFrm2 + 1] - iMea2Start);
	//trkMarks.assign(m_trkStates.size(), false);
	for(iFtr2 = 0; iFtr2 < nFtrs2; ++iFtr2)
	{
		if((iTrk = iTrks2[iFtr2]) == INVALID_TRACK_INDEX || (iFtr1 = mapTrkToFtr1[iTrk]) == INVALID_FEATURE_INDEX)
			continue;
		disp += sqrt(ftrs1[iFtr1].SquaredDistance(ftrs2[iFtr2]));
		++nCmnPts;
		//trkMarks[iTrk] = true;
	}
	disp = m_measNormalized ? disp / nCmnPts * sqrt(m_K.fxy()) : disp / nCmnPts;
	return disp;
}

#if DESCRIPTOR_TRACK == 0
void Sequence::MatchFrameFeatures(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureMatchList &matches, std::vector<bool> &marks)
{
	const TrackIndex *iTrks1 = GetFrameTrackIndexes(iFrm1), *iTrks2 = GetFrameTrackIndexes(iFrm2);

	FeatureIndex iFtr1, iFtr2;
	TrackIndex iTrk1, iTrk2;

	const ushort nMatches = ushort(matches.size());
	for(ushort i = 0; i < nMatches; ++i)
	{
		iFtr1 = matches[i].GetIndex1();	iTrk1 = iTrks1[iFtr1];
		iFtr2 = matches[i].GetIndex2();	iTrk2 = iTrks2[iFtr2];
		if(iTrk1 == INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX)
			PushBackTrack(iFrm1, iFtr1, iFrm2, iFtr2);
		else if(iTrk1 != INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX)
			MatchTrackAndFrameFeature(iTrk1, iFrm2, iFtr2);
		else if(iTrk1 == INVALID_TRACK_INDEX && iTrk2 != INVALID_TRACK_INDEX)
			MatchTrackAndFrameFeature(iTrk2, iFrm1, iFtr1);
		else if(iTrk1 != INVALID_TRACK_INDEX && iTrk2 != INVALID_TRACK_INDEX && iTrk1 != iTrk2 && !AreTracksOverlappingInFrames(iTrk1, iTrk2, marks))
			MatchTracks(iTrk1, iTrk2);
	}
}

void Sequence::MatchTrackAndFrameFeature(const TrackIndex &iTrk, const FrameIndex &iFrm, const FeatureIndex &iFtr)
{
#if _DEBUG
	IO::Assert(SearchTrackForFrameFeatureIndex(iTrk, iFrm) == INVALID_FEATURE_INDEX, "trk%d, frm%d, ftr%d", iTrk, iFrm, SearchTrackForFrameFeatureIndex(iTrk, iFrm));
	AssertTrackOrdered(iTrk);
	//const FrameIndex iFrmLast = m_mapMeaToFrm[m_mapTrkToMea[iTrk].back()];
	//assert(iFrmLast < iFrm);
#endif
	const MeasurementIndex iMea = m_mapFrmToMea[iFrm] + iFtr;
	m_mapMeaToTrk[iMea] = iTrk;
	MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
	if(iMeas.empty() || iMeas.back() < iMea)
		iMeas.push_back(iMea);
	else
		iMeas.insert(std::lower_bound(iMeas.begin(), iMeas.end(), iMea), iMea);
}
#else
void Sequence::MatchFrameFeatures(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureMatchList &matches, std::vector<bool> &marks, 
								  const AlignedVector<Descriptor> &descs1, const AlignedVector<Descriptor> &descs2)
{
	const TrackIndex *iTrks1 = GetFrameTrackIndexes(iFrm1), *iTrks2 = GetFrameTrackIndexes(iFrm2);

	FeatureIndex iFtr1, iFtr2;
	TrackIndex iTrk1, iTrk2;

	const ushort nMatches = ushort(matches.size());
	for(ushort i = 0; i < nMatches; ++i)
	{
		iFtr1 = matches[i].GetIndex1();	iTrk1 = iTrks1[iFtr1];
		iFtr2 = matches[i].GetIndex2();	iTrk2 = iTrks2[iFtr2];
		if(iTrk1 == INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX)
			PushBackTrack(iFrm1, iFtr1, iFrm2, iFtr2, descs1[iFtr1], descs2[iFtr2]);
		else if(iTrk1 != INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX)
			MatchTrackAndFrameFeature(iTrk1, iFrm2, iFtr2, descs2[iFtr2]);
		else if(iTrk1 == INVALID_TRACK_INDEX && iTrk2 != INVALID_TRACK_INDEX)
			MatchTrackAndFrameFeature(iTrk2, iFrm1, iFtr1, descs1[iFtr1]);
		else if(iTrk1 != INVALID_TRACK_INDEX && iTrk2 != INVALID_TRACK_INDEX && iTrk1 != iTrk2 && !AreTracksOverlappingInFrames(iTrk1, iTrk2, marks))
			MatchTracks(iTrk1, iTrk2);
	}
}

void Sequence::MatchTrackAndFrameFeature(const TrackIndex &iTrk, const FrameIndex &iFrm, const FeatureIndex &iFtr, const Descriptor &desc)
{
#if _DEBUG
	IO::Assert(SearchTrackForFrameFeatureIndex(iTrk, iFrm) == INVALID_FEATURE_INDEX, "trk%d, frm%d, ftr%d", iTrk, iFrm, SearchTrackForFrameFeatureIndex(iTrk, iFrm));
	AssertTrackOrdered(iTrk);
	//const FrameIndex iFrmLast = m_mapMeaToFrm[m_mapTrkToMea[iTrk].back()];
	//assert(iFrmLast < iFrm);
#endif
	const MeasurementIndex iMea = m_mapFrmToMea[iFrm] + iFtr;
	m_mapMeaToTrk[iMea] = iTrk;
	MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
	const MeasurementIndex iMeaLast = m_mapTrkToMea[iTrk].back();
	if(iMeaLast < iMea)
		iMeas.push_back(iMea);
	else
		iMeas.insert(std::lower_bound(iMeas.begin(), iMeas.end(), iMea), iMea);
	Descriptor::ApB(m_descs[iTrk], desc, m_descs[iTrk]);
}
#endif

void Sequence::MatchTracks(const TrackIndex &iTrk1, const TrackIndex &iTrk2)
{
#if _DEBUG
	std::vector<bool> marks;
	assert(iTrk1 != iTrk2 && !AreTracksOverlappingInFrames(iTrk1, iTrk2, marks));
#endif
	MeasurementIndexList &iMeas1 = m_mapTrkToMea[iTrk1];	const FrameIndex nCrsps1 = FrameIndex(iMeas1.size());
	MeasurementIndexList &iMeas2 = m_mapTrkToMea[iTrk2];	const FrameIndex nCrsps2 = FrameIndex(iMeas2.size());
	iMeas1.insert(iMeas1.end(), iMeas2.begin(), iMeas2.end());
	std::inplace_merge(iMeas1.begin(), iMeas1.begin() + nCrsps1, iMeas1.end());
	for(FrameIndex i = 0; i < nCrsps2; ++i)
		m_mapMeaToTrk[iMeas2[i]] = iTrk1;
	iMeas2.clear();
#if _DEBUG
	const FrameIndex nCrsps = FrameIndex(iMeas1.size());
	for(FrameIndex i = 1; i < nCrsps; ++i)
		assert(m_mapMeaToFrm[iMeas1[i]] != m_mapMeaToFrm[iMeas1[i - 1]]);
#endif

#if DESCRIPTOR_TRACK
	Descriptor::ApB(m_descs[iTrk1], m_descs[iTrk2], m_descs[iTrk1]);
	m_descs[iTrk2].SetZero();
#endif
}

void Sequence::MatchTracks(const TrackMatchList &trkMatches)
{
	const TrackIndex nMatches = TrackIndex(trkMatches.size());
	TrackIndex iTrk1, iTrk2;
	for(TrackIndex i = 0; i < nMatches; ++i)
	{
		iTrk1 = trkMatches[i].GetIndex1();
		iTrk2 = trkMatches[i].GetIndex2();
		MatchTracks(iTrk1, iTrk2);
	}
}

void Sequence::BreakOutlierTracksAndMeasurements()
{
	FrameIndex i, j;
	MeasurementIndex iMea;
	const TrackIndex nTrks = GetTracksNumber();
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		if(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER)
		{
			MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
			const FrameIndex nCrsps = FrameIndex(iMeas.size());
			for(i = j = 0; i < nCrsps; ++i)
			{
				iMea = iMeas[i];
				if(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER)
					m_mapMeaToTrk[iMea] = INVALID_TRACK_INDEX;
				else
					iMeas[j++] = iMea;
			}
			if(j < 2)
				BreakTrack(iTrk);
			else
				iMeas.resize(j);
		}
		else
			BreakTrack(iTrk);
	}
}

void Sequence::BreakSingleTracksAndMeasurements()
{
	const TrackIndex nTrks = GetTracksNumber();
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		if(GetTrackLength(iTrk) == 1)
			BreakTrack(iTrk);
	}
}

void Sequence::RemoveOutlierTracksAndMeasurements()
{
	BreakOutlierTracksAndMeasurements();
	RemoveBrokenTracks();
	RemoveNullMeasurements();
}

void Sequence::RemoveSingleTracksAndMeasurements()
{
	BreakSingleTracksAndMeasurements();
	RemoveBrokenTracks();
	RemoveNullMeasurements();
}