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
#include "SequenceBundleAdjustorData.h"
#include "SequenceBundleAdjustorDataIntrinsicVariable.h"
#include "SequenceBundleAdjustorDataProjective.h"

void Sequence::GetBundleAdjustorData(SequenceBundleAdjustorData &data) const
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_USER_FIXED || m_intrinsicType == INTRINSIC_CONSTANT);
#endif
	data.m_fxy = m_K.fxy();
	data.m_G = m_Kr;
	data.m_Cs = m_Cs;
	data.m_Xs = m_Xs;
	data.m_xs = m_xs;
	data.m_mapFrmToMea = m_mapFrmToMea;
	data.m_mapTrkToMea = m_mapTrkToMea;
	data.m_mapMeaToFrm = m_mapMeaToFrm;
	data.m_mapMeaToTrk = m_mapMeaToTrk;
}

void Sequence::GetBundleAdjustorData(SequenceBundleAdjustorDataIntrinsicVariable &data) const
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_VARIABLE);
#endif
	data.m_fxy = m_K.fxy();
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	data.m_Cs.Resize(nFrms);
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
		data.m_Cs[iFrm].Set(m_Cs[iFrm], m_Krs[iFrm]);
	data.m_Xs.Resize(m_Xs.Size());	data.m_Xs.CopyFrom(m_Xs);
	data.m_xs.Resize(m_xs.Size());	data.m_xs.CopyFrom(m_xs);
	data.m_mapFrmToMea = m_mapFrmToMea;
	data.m_mapTrkToMea = m_mapTrkToMea;
	data.m_mapMeaToFrm = m_mapMeaToFrm;
	data.m_mapMeaToTrk = m_mapMeaToTrk;
}

void Sequence::GetBundleAdjustorData(SequenceBundleAdjustorDataProjective &data) const
{
	data.m_fxy = m_K.fxy();
	//data.m_G = m_Kr;
	data.m_Cs.Resize(m_Ps.Size());	data.m_Cs.CopyFrom(m_Ps);
	data.m_Xs = m_Xs;
	data.m_xs = m_xs;
	data.m_mapFrmToMea = m_mapFrmToMea;
	data.m_mapTrkToMea = m_mapTrkToMea;
	data.m_mapMeaToFrm = m_mapMeaToFrm;
	data.m_mapMeaToTrk = m_mapMeaToTrk;
}

void Sequence::SetBunbleAdjustmentResults(const SequenceBundleAdjustorData &data)
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_USER_FIXED || m_intrinsicType == INTRINSIC_CONSTANT);
#endif
	if(data.IsGlobalValid())
		m_Kr = data.m_G;
	m_Cs.CopyFrom(data.m_Cs);
	m_Xs.CopyFrom(data.m_Xs);
	ComputeProjectiveMatrixes();
}

void Sequence::SetBunbleAdjustmentResults(const SequenceBundleAdjustorDataIntrinsicVariable &data)
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_VARIABLE);
#endif
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
		SetCamera(iFrm, data.m_Cs[iFrm].C(), data.m_Cs[iFrm].Kr());
	m_Xs.CopyFrom(data.m_Xs);
}

void Sequence::SetBunbleAdjustmentResults(const SequenceBundleAdjustorDataProjective &data)
{
	m_Ps.CopyFrom(data.m_Cs);
	m_Xs.CopyFrom(data.m_Xs);
}

void Sequence::GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, SequenceBundleAdjustorData &data, FrameIndexList &iFrmsBA, TrackIndexList &iTrksBA) const
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_USER_FIXED || m_intrinsicType == INTRINSIC_CONSTANT);
#endif
	// Step1: mark adjusted frames
	FrameStateList frmStates = m_frmStates;
	const FrameIndex nFrmsAdj = FrameIndex(iFrmsAdj.size());
	for(FrameIndex i = 0; i < nFrmsAdj; ++i)
		frmStates[iFrmsAdj[i]] |= FLAG_FRAME_STATE_ADJUSTED;

	// Step2: mark fixed frames, collect BA tracks, create reverse track index list iTrksSrcToDst and count BA measurements
	iTrksBA.resize(0);
	TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
	FrameIndex iFrmSrc, iFrmDst;
	TrackIndex iTrkSrc, iTrkDst = 0;
	MeasurementIndex iMeaSrc, iMeaDst = 0;
	for(FrameIndex i = 0; i < nFrmsAdj; ++i)
	{
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmsAdj[i]], iMeaSrc2 = m_mapFrmToMea[iFrmsAdj[i] + 1];
		for(MeasurementIndex j = iMeaSrc1; j < iMeaSrc2; ++j)
		{
			iTrkSrc = m_mapMeaToTrk[j];
			if(iTrkSrc == INVALID_TRACK_INDEX || iTrksSrcToDst[iTrkSrc] != INVALID_TRACK_INDEX || !(m_trkStates[iTrkSrc] & FLAG_TRACK_STATE_INLIER)
			|| (m_meaStates[j] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			iTrksSrcToDst[iTrkSrc] = iTrkDst++;
			iTrksBA.push_back(iTrkSrc);

			const MeasurementIndexList &iMeasSrc = m_mapTrkToMea[iTrkSrc];
			const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
			for(FrameIndex k = 0; k < nCrspsSrc; ++k)
			{
				iMeaSrc = iMeasSrc[k];
				if(m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER)
					continue;
				iFrmSrc = m_mapMeaToFrm[iMeaSrc];
				if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_SOLVED))
					continue;
				++iMeaDst;
				if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_ADJUSTED))
					frmStates[iFrmSrc] |= FLAG_FRAME_STATE_FIXED;
			}
		}
	}

	// Step3: collect BA frames
	iFrmsBA.resize(0);
	const FrameIndex nFrmsSrc = GetFramesNumber();
	for(iFrmSrc = 0; iFrmSrc < nFrmsSrc; ++iFrmSrc)
	{
		if(frmStates[iFrmSrc] & FLAG_FRAME_STATE_FIXED)
			iFrmsBA.push_back(iFrmSrc);
	}
	iFrmsBA.insert(iFrmsBA.end(), iFrmsAdj.begin(), iFrmsAdj.end());

	// Step4: copy tag, intrinsic matrix, all BA cameras and all BA points
	const FrameIndex nFrmsDst = FrameIndex(iFrmsBA.size());
	const TrackIndex nTrksDst = iTrkDst;
	const MeasurementIndex nMeasDst = iMeaDst;
	data.Resize(nFrmsDst, nTrksDst, nMeasDst);

	//m_tag.GetSubSequence(iFrmsBA, data.m_tag);
	data.m_fxy = m_K.fxy();
	data.m_G = m_Kr;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		data.m_Cs[iFrmDst] = m_Cs[iFrmSrc];
		//data.m_frmStates[iFrmDst] = m_frmStates[iFrmSrc];
	}
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrksBA[iTrkDst];
		data.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
		//data.m_trkStates[iTrkDst] = m_trkStates[iTrkSrc];
	}

	// Step5: copy inlier measurements and create correspondence maps
	//iMeasBA.resize(0);
	iMeaDst = 0;
	data.m_mapFrmToMea[0] = 0;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX
			|| (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			//iMeasBA.push_back(iMeaSrc);
			data.m_xs[iMeaDst] = m_xs[iMeaSrc];
			data.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			data.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			data.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			//data.m_meaStates[iMeaDst] = m_meaStates[iMeaSrc];
			++iMeaDst;
		}
		data.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
	}

#if _DEBUG
	assert(iMeaDst == nMeasDst);
#endif
}

void Sequence::GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, SequenceBundleAdjustorDataIntrinsicVariable &data, FrameIndexList &iFrmsBA, 
									 TrackIndexList &iTrksBA) const
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_VARIABLE);
#endif
	// Step1: mark adjusted frames
	FrameStateList frmStates = m_frmStates;
	const FrameIndex nFrmsAdj = FrameIndex(iFrmsAdj.size());
	for(FrameIndex i = 0; i < nFrmsAdj; ++i)
		frmStates[iFrmsAdj[i]] |= FLAG_FRAME_STATE_ADJUSTED;

	// Step2: mark fixed frames, collect BA tracks, create reverse track index list iTrksSrcToDst and count BA measurements
	iTrksBA.resize(0);
	TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
	FrameIndex iFrmSrc, iFrmDst;
	TrackIndex iTrkSrc, iTrkDst = 0;
	MeasurementIndex iMeaSrc, iMeaDst = 0;
	for(FrameIndex i = 0; i < nFrmsAdj; ++i)
	{
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmsAdj[i]], iMeaSrc2 = m_mapFrmToMea[iFrmsAdj[i] + 1];
		for(MeasurementIndex j = iMeaSrc1; j < iMeaSrc2; ++j)
		{
			iTrkSrc = m_mapMeaToTrk[j];
			if(iTrkSrc == INVALID_TRACK_INDEX || iTrksSrcToDst[iTrkSrc] != INVALID_TRACK_INDEX || !(m_trkStates[iTrkSrc] & FLAG_TRACK_STATE_INLIER)
			|| (m_meaStates[j] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			iTrksSrcToDst[iTrkSrc] = iTrkDst++;
			iTrksBA.push_back(iTrkSrc);

			const MeasurementIndexList &iMeasSrc = m_mapTrkToMea[iTrkSrc];
			const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
			for(FrameIndex k = 0; k < nCrspsSrc; ++k)
			{
				iMeaSrc = iMeasSrc[k];
				if(m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER)
					continue;
				iFrmSrc = m_mapMeaToFrm[iMeaSrc];
				if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_SOLVED))
					continue;
				++iMeaDst;
				if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_ADJUSTED))
					frmStates[iFrmSrc] |= FLAG_FRAME_STATE_FIXED;
			}
		}
	}

	// Step3: collect BA frames
	iFrmsBA.resize(0);
	const FrameIndex nFrmsSrc = GetFramesNumber();
	for(iFrmSrc = 0; iFrmSrc < nFrmsSrc; ++iFrmSrc)
	{
		if(frmStates[iFrmSrc] & FLAG_FRAME_STATE_FIXED)
			iFrmsBA.push_back(iFrmSrc);
	}
	iFrmsBA.insert(iFrmsBA.end(), iFrmsAdj.begin(), iFrmsAdj.end());

	// Step4: copy tag, intrinsic matrix, all BA cameras and all BA points
	const FrameIndex nFrmsDst = FrameIndex(iFrmsBA.size());
	const TrackIndex nTrksDst = iTrkDst;
	const MeasurementIndex nMeasDst = iMeaDst;
	data.Resize(nFrmsDst, nTrksDst, nMeasDst);

	//m_tag.GetSubSequence(iFrmsBA, data.m_tag);
	data.m_fxy = m_K.fxy();
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		data.m_Cs[iFrmDst].Set(m_Cs[iFrmSrc], m_Krs[iFrmSrc]);
		//data.m_frmStates[iFrmDst] = m_frmStates[iFrmSrc];
	}
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrksBA[iTrkDst];
		data.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
		//data.m_trkStates[iTrkDst] = m_trkStates[iTrkSrc];
	}

	// Step5: copy inlier measurements and create correspondence maps
	//iMeasBA.resize(0);
	iMeaDst = 0;
	data.m_mapFrmToMea[0] = 0;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX
			|| (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			//iMeasBA.push_back(iMeaSrc);
			data.m_xs[iMeaDst] = m_xs[iMeaSrc];
			data.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			data.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			data.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			//data.m_meaStates[iMeaDst] = m_meaStates[iMeaSrc];
			++iMeaDst;
		}
		data.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
	}

#if _DEBUG
	assert(iMeaDst == nMeasDst);
#endif
}

void Sequence::GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, SequenceBundleAdjustorDataProjective &data, FrameIndexList &iFrmsBA, 
									 TrackIndexList &iTrksBA) const
{
	// Step1: mark adjusted frames
	FrameStateList frmStates = m_frmStates;
	const FrameIndex nFrmsAdj = FrameIndex(iFrmsAdj.size());
	for(FrameIndex i = 0; i < nFrmsAdj; ++i)
		frmStates[iFrmsAdj[i]] |= FLAG_FRAME_STATE_ADJUSTED;

	// Step2: mark fixed frames, collect BA tracks, create reverse track index list iTrksSrcToDst and count BA measurements
	iTrksBA.resize(0);
	TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
	FrameIndex iFrmSrc, iFrmDst;
	TrackIndex iTrkSrc, iTrkDst = 0;
	MeasurementIndex iMeaSrc, iMeaDst = 0;
	for(FrameIndex i = 0; i < nFrmsAdj; ++i)
	{
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmsAdj[i]], iMeaSrc2 = m_mapFrmToMea[iFrmsAdj[i] + 1];
		for(MeasurementIndex j = iMeaSrc1; j < iMeaSrc2; ++j)
		{
			iTrkSrc = m_mapMeaToTrk[j];
			if(iTrkSrc == INVALID_TRACK_INDEX || iTrksSrcToDst[iTrkSrc] != INVALID_TRACK_INDEX || !(m_trkStates[iTrkSrc] & FLAG_TRACK_STATE_INLIER)
			|| (m_meaStates[j] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			iTrksSrcToDst[iTrkSrc] = iTrkDst++;
			iTrksBA.push_back(iTrkSrc);

			const MeasurementIndexList &iMeasSrc = m_mapTrkToMea[iTrkSrc];
			const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
			for(FrameIndex k = 0; k < nCrspsSrc; ++k)
			{
				iMeaSrc = iMeasSrc[k];
				if(m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER)
					continue;
				iFrmSrc = m_mapMeaToFrm[iMeaSrc];
				if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_SOLVED))
					continue;
				++iMeaDst;
				if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_ADJUSTED))
					frmStates[iFrmSrc] |= FLAG_FRAME_STATE_FIXED;
			}
		}
	}

	// Step3: collect BA frames
	iFrmsBA.resize(0);
	const FrameIndex nFrmsSrc = GetFramesNumber();
	for(iFrmSrc = 0; iFrmSrc < nFrmsSrc; ++iFrmSrc)
	{
		if(frmStates[iFrmSrc] & FLAG_FRAME_STATE_FIXED)
			iFrmsBA.push_back(iFrmSrc);
	}
	iFrmsBA.insert(iFrmsBA.end(), iFrmsAdj.begin(), iFrmsAdj.end());

	// Step4: copy tag, intrinsic matrix, all BA cameras and all BA points
	const FrameIndex nFrmsDst = FrameIndex(iFrmsBA.size());
	const TrackIndex nTrksDst = iTrkDst;
	const MeasurementIndex nMeasDst = iMeaDst;
	data.Resize(nFrmsDst, nTrksDst, nMeasDst);

	data.m_fxy = m_K.fxy();
	//data.m_G = m_Kr;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		data.m_Cs[iFrmDst] = m_Ps[iFrmSrc];
	}
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrksBA[iTrkDst];
		data.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
	}

	// Step5: copy inlier measurements and create correspondence maps
	iMeaDst = 0;
	data.m_mapFrmToMea[0] = 0;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX
			|| (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			data.m_xs[iMeaDst] = m_xs[iMeaSrc];
			data.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			data.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			data.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			++iMeaDst;
		}
		data.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
	}

#if _DEBUG
	assert(iMeaDst == nMeasDst);
#endif
}

void Sequence::GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, SequenceBundleAdjustorData &data, 
									 FrameIndexList &iFrmsBA) const
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_USER_FIXED || m_intrinsicType == INTRINSIC_CONSTANT);
#endif
	// Step1: mark adjusted frames
	FrameStateList frmStates = m_frmStates;
	const FrameIndex nFrmsAdj = FrameIndex(iFrmsAdj.size());
	for(FrameIndex i = 0; i < nFrmsAdj; ++i)
		frmStates[iFrmsAdj[i]] |= FLAG_FRAME_STATE_ADJUSTED;

	// Step2: mark fixed frames, create reverse track index list iTrksSrcToDst and count BA measurements
	TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
	FrameIndex iFrmSrc, iFrmDst;
	TrackIndex iTrkSrc, iTrkDst;
	MeasurementIndex iMeaSrc, iMeaDst = 0;
	const TrackIndex nTrksAdj = TrackIndex(iTrksAdj.size());
	for(iTrkDst = 0; iTrkDst < nTrksAdj; ++iTrkDst)
	{
		iTrkSrc = iTrksAdj[iTrkDst];
		iTrksSrcToDst[iTrkSrc] = iTrkDst;

		const MeasurementIndexList &iMeasSrc = m_mapTrkToMea[iTrkSrc];
		const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
		for(FrameIndex k = 0; k < nCrspsSrc; ++k)
		{
			iMeaSrc = iMeasSrc[k];
			if(m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER)
				continue;
			iFrmSrc = m_mapMeaToFrm[iMeaSrc];
			if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_SOLVED))
				continue;
			++iMeaDst;
			if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_ADJUSTED))
				frmStates[iFrmSrc] |= FLAG_FRAME_STATE_FIXED;
		}
	}

	// Step3: collect fixed frames
	iFrmsBA.resize(0);
	const FrameIndex nFrmsSrc = GetFramesNumber();
	for(iFrmSrc = 0; iFrmSrc < nFrmsSrc; ++iFrmSrc)
	{
		if(frmStates[iFrmSrc] & FLAG_FRAME_STATE_FIXED)
			iFrmsBA.push_back(iFrmSrc);
	}
	iFrmsBA.insert(iFrmsBA.end(), iFrmsAdj.begin(), iFrmsAdj.end());

	// Step4: copy tag, intrinsic matrix, all BA cameras and all BA points
	const FrameIndex nFrmsDst = FrameIndex(iFrmsBA.size());
	const TrackIndex nTrksDst = TrackIndex(iTrksAdj.size());
	const MeasurementIndex nMeasDst = iMeaDst;
	data.Resize(nFrmsDst, nTrksDst, nMeasDst);

	//m_tag.GetSubSequence(iFrmsBA, data.m_tag);
	data.m_fxy = m_K.fxy();
	data.m_G = m_Kr;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		data.m_Cs[iFrmDst] = m_Cs[iFrmSrc];
		//data.m_frmStates[iFrmDst] = m_frmStates[iFrmSrc];
	}
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrksAdj[iTrkDst];
		data.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
		//data.m_trkStates[iTrkDst] = m_trkStates[iTrkSrc];
	}

	// Step5: copy inlier measurements and create correspondence maps
	//iMeasBA.resize(0);
	iMeaDst = 0;
	data.m_mapFrmToMea[0] = 0;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX
			|| (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			//iMeasBA.push_back(iMeaSrc);
			data.m_xs[iMeaDst] = m_xs[iMeaSrc];
			data.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			data.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			data.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			//data.m_meaStates[iMeaDst] = m_meaStates[iMeaSrc];
			++iMeaDst;
		}
		data.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
	}

#if _DEBUG
	assert(iMeaDst == nMeasDst);
#endif
}

void Sequence::GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, SequenceBundleAdjustorDataIntrinsicVariable &data, 
									 FrameIndexList &iFrmsBA) const
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_VARIABLE);
#endif
	// Step1: mark adjusted frames
	FrameStateList frmStates = m_frmStates;
	const FrameIndex nFrmsAdj = FrameIndex(iFrmsAdj.size());
	for(FrameIndex i = 0; i < nFrmsAdj; ++i)
		frmStates[iFrmsAdj[i]] |= FLAG_FRAME_STATE_ADJUSTED;

	// Step2: mark fixed frames, create reverse track index list iTrksSrcToDst and count BA measurements
	TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
	FrameIndex iFrmSrc, iFrmDst;
	TrackIndex iTrkSrc, iTrkDst;
	MeasurementIndex iMeaSrc, iMeaDst = 0;
	const TrackIndex nTrksAdj = TrackIndex(iTrksAdj.size());
	for(iTrkDst = 0; iTrkDst < nTrksAdj; ++iTrkDst)
	{
		iTrkSrc = iTrksAdj[iTrkDst];
		iTrksSrcToDst[iTrkSrc] = iTrkDst;

		const MeasurementIndexList &iMeasSrc = m_mapTrkToMea[iTrkSrc];
		const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
		for(FrameIndex k = 0; k < nCrspsSrc; ++k)
		{
			iMeaSrc = iMeasSrc[k];
			if(m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER)
				continue;
			iFrmSrc = m_mapMeaToFrm[iMeaSrc];
			if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_SOLVED))
				continue;
			++iMeaDst;
			if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_ADJUSTED))
				frmStates[iFrmSrc] |= FLAG_FRAME_STATE_FIXED;
		}
	}

	// Step3: collect fixed frames
	iFrmsBA.resize(0);
	const FrameIndex nFrmsSrc = GetFramesNumber();
	for(iFrmSrc = 0; iFrmSrc < nFrmsSrc; ++iFrmSrc)
	{
		if(frmStates[iFrmSrc] & FLAG_FRAME_STATE_FIXED)
			iFrmsBA.push_back(iFrmSrc);
	}
	iFrmsBA.insert(iFrmsBA.end(), iFrmsAdj.begin(), iFrmsAdj.end());

	// Step4: copy tag, intrinsic matrix, all BA cameras and all BA points
	const FrameIndex nFrmsDst = FrameIndex(iFrmsBA.size());
	const TrackIndex nTrksDst = TrackIndex(iTrksAdj.size());
	const MeasurementIndex nMeasDst = iMeaDst;
	data.Resize(nFrmsDst, nTrksDst, nMeasDst);

	//m_tag.GetSubSequence(iFrmsBA, data.m_tag);
	data.m_fxy = m_K.fxy();
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		data.m_Cs[iFrmDst].Set(m_Cs[iFrmSrc], m_Krs[iFrmSrc]);
		//data.m_frmStates[iFrmDst] = m_frmStates[iFrmSrc];
	}
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrksAdj[iTrkDst];
		data.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
		//data.m_trkStates[iTrkDst] = m_trkStates[iTrkSrc];
	}

	// Step5: copy inlier measurements and create correspondence maps
	//iMeasBA.resize(0);
	iMeaDst = 0;
	data.m_mapFrmToMea[0] = 0;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX
			|| (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			//iMeasBA.push_back(iMeaSrc);
			data.m_xs[iMeaDst] = m_xs[iMeaSrc];
			data.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			data.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			data.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			//data.m_meaStates[iMeaDst] = m_meaStates[iMeaSrc];
			++iMeaDst;
		}
		data.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
	}

#if _DEBUG
	assert(iMeaDst == nMeasDst);
#endif
}

void Sequence::GetBundleAdjustorData(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, SequenceBundleAdjustorDataProjective &data, 
									 FrameIndexList &iFrmsBA) const
{
	// Step1: mark adjusted frames
	FrameStateList frmStates = m_frmStates;
	const FrameIndex nFrmsAdj = FrameIndex(iFrmsAdj.size());
	for(FrameIndex i = 0; i < nFrmsAdj; ++i)
		frmStates[iFrmsAdj[i]] |= FLAG_FRAME_STATE_ADJUSTED;

	// Step2: mark fixed frames, create reverse track index list iTrksSrcToDst and count BA measurements
	TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
	FrameIndex iFrmSrc, iFrmDst;
	TrackIndex iTrkSrc, iTrkDst;
	MeasurementIndex iMeaSrc, iMeaDst = 0;
	const TrackIndex nTrksAdj = TrackIndex(iTrksAdj.size());
	for(iTrkDst = 0; iTrkDst < nTrksAdj; ++iTrkDst)
	{
		iTrkSrc = iTrksAdj[iTrkDst];
		iTrksSrcToDst[iTrkSrc] = iTrkDst;

		const MeasurementIndexList &iMeasSrc = m_mapTrkToMea[iTrkSrc];
		const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
		for(FrameIndex k = 0; k < nCrspsSrc; ++k)
		{
			iMeaSrc = iMeasSrc[k];
			if(m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER)
				continue;
			iFrmSrc = m_mapMeaToFrm[iMeaSrc];
			if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_SOLVED))
				continue;
			++iMeaDst;
			if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_ADJUSTED))
				frmStates[iFrmSrc] |= FLAG_FRAME_STATE_FIXED;
		}
	}

	// Step3: collect fixed frames
	iFrmsBA.resize(0);
	const FrameIndex nFrmsSrc = GetFramesNumber();
	for(iFrmSrc = 0; iFrmSrc < nFrmsSrc; ++iFrmSrc)
	{
		if(frmStates[iFrmSrc] & FLAG_FRAME_STATE_FIXED)
			iFrmsBA.push_back(iFrmSrc);
	}
	iFrmsBA.insert(iFrmsBA.end(), iFrmsAdj.begin(), iFrmsAdj.end());

	// Step4: copy tag, intrinsic matrix, all BA cameras and all BA points
	const FrameIndex nFrmsDst = FrameIndex(iFrmsBA.size());
	const TrackIndex nTrksDst = TrackIndex(iTrksAdj.size());
	const MeasurementIndex nMeasDst = iMeaDst;
	data.Resize(nFrmsDst, nTrksDst, nMeasDst);

	//m_tag.GetSubSequence(iFrmsBA, data.m_tag);
	data.m_fxy = m_K.fxy();
	//data.m_G = m_Kr;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		data.m_Cs[iFrmDst] = m_Ps[iFrmSrc];
	}
	for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
	{
		iTrkSrc = iTrksAdj[iTrkDst];
		data.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
	}

	// Step5: copy inlier measurements and create correspondence maps
	iMeaDst = 0;
	data.m_mapFrmToMea[0] = 0;
	for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst)
	{
		iFrmSrc = iFrmsBA[iFrmDst];
		const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc], iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iTrkSrc = m_mapMeaToTrk[iMeaSrc];
			if(iTrkSrc == INVALID_TRACK_INDEX || (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX
			|| (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			data.m_xs[iMeaDst] = m_xs[iMeaSrc];
			data.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			data.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			data.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			++iMeaDst;
		}
		data.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
	}

#if _DEBUG
	assert(iMeaDst == nMeasDst);
#endif
}

void Sequence::SetBunbleAdjustmentResults(const SequenceBundleAdjustorData &data, const FrameIndex &nFrmsFix, const FrameIndexList &iFrmsBA, 
										  const TrackIndexList &iTrksBA)
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_USER_FIXED || m_intrinsicType == INTRINSIC_CONSTANT);
#endif
	if(data.IsGlobalValid())
		m_Kr = data.m_G;
	const FrameIndex nFrmsBA = FrameIndex(iFrmsBA.size());
	for(FrameIndex i = nFrmsFix; i < nFrmsBA; ++i)
		SetCamera(iFrmsBA[i], data.m_Cs[i]);
	SetPoints(iTrksBA, data.GetPoints());
}

void Sequence::SetBunbleAdjustmentResults(const SequenceBundleAdjustorDataIntrinsicVariable &data, const FrameIndex &nFrmsFix, const FrameIndexList &iFrmsBA, 
										  const TrackIndexList &iTrksBA)
{
#if _DEBUG
	assert(m_intrinsicType == INTRINSIC_VARIABLE);
#endif
	const FrameIndex nFrmsBA = FrameIndex(iFrmsBA.size());
	for(FrameIndex i = nFrmsFix; i < nFrmsBA; ++i)
		SetCamera(iFrmsBA[i], data.m_Cs[i].C(), data.m_Cs[i].Kr());
	SetPoints(iTrksBA, data.GetPoints());
}

void Sequence::SetBunbleAdjustmentResults(const SequenceBundleAdjustorDataProjective &data, const FrameIndex &nFrmsFix, const FrameIndexList &iFrmsBA, 
										  const TrackIndexList &iTrksBA)
{
	const FrameIndex nFrmsBA = FrameIndex(iFrmsBA.size());
	for(FrameIndex i = nFrmsFix; i < nFrmsBA; ++i)
		SetCamera(iFrmsBA[i], data.m_Cs[i]);
	SetPoints(iTrksBA, data.GetPoints());
}