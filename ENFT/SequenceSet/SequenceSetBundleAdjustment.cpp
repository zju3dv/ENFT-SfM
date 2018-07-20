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
#include "SequenceSetBundleAdjustorData3DSimilarity.h"
#include "SegmentSetBundleAdjustorData2DSimilarity.h"
#include "SfM/Point3DEstimator.h"
#include "SequenceTransformationOptimizerDataSimilarity.h"
#include "SequenceTransformationOptimizerDataScale.h"

void SequenceSet::GetBundleAdjustorData(const SequenceIndexList &iSeqsAdj, SequenceSetBundleAdjustorData3DSimilarity &data, SequenceIndex &nSeqsFix, SequenceIndexList &iSeqsBA, 
										TrackIndexList &iTrksCmnBA) const
{
	// Step1: mark adjusted sequence
	SequenceStateList seqStates = m_seqStates;
	const SequenceIndex nSeqsAdj = SequenceIndex(iSeqsAdj.size());
	for(SequenceIndex i = 0; i < nSeqsAdj; ++i)
		seqStates[iSeqsAdj[i]] |= FLAG_SEQUENCE_STATE_ADJUSTED;

	// Step2: mark fixed sequences, collect BA common tracks, create reverse common track index list iTrksCmnSrcToDst and count BA individual tracks
	iTrksCmnBA.resize(0);
	const TrackIndex nTrksCmnSrc = GetCommonTracksNumber();
	TrackIndexList iTrksCmnSrcToDst(nTrksCmnSrc, INVALID_TRACK_INDEX);
	std::vector<bool> trkCmnSrcMarks(nTrksCmnSrc, false);
	TrackIndex nTrksIdvDst = 0, iTrkIdvSrc, iTrkCmnSrc;
	SequenceIndex iSeqSrc;
	for(SequenceIndex i = 0; i < nSeqsAdj; ++i)
	{
		const SequenceIndex iSeqAdj = iSeqsAdj[i];
		const Sequence &seqAdj = GetSequence(iSeqAdj);
		const TrackIndexList &iTrksIdvToCmnAdj = m_mapIdvTrkToCmnTrk[iSeqAdj];
		const TrackIndex nTrksIdvAdj = seqAdj.GetTracksNumber();
		for(TrackIndex j = 0; j < nTrksIdvAdj; ++j)
		{
			if((iTrkCmnSrc = iTrksIdvToCmnAdj[j]) == INVALID_TRACK_INDEX || trkCmnSrcMarks[iTrkCmnSrc] || !(seqAdj.GetTrackState(j) & FLAG_TRACK_STATE_INLIER)
			|| (seqAdj.GetTrackState(j) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				continue;
			trkCmnSrcMarks[iTrkCmnSrc] = true;
			const SequenceTrackIndexList &iSeqTrksIdvSrc = m_mapCmnTrkToIdvTrk[iTrkCmnSrc];
			const SequenceIndex nCrspsSrc = SequenceIndex(iSeqTrksIdvSrc.size());
			SequenceIndex nCrspsSrcInlier = 0;
			for(SequenceIndex k = 0; k < nCrspsSrc; ++k)
			{
				iSeqTrksIdvSrc[k].Get(iSeqSrc, iTrkIdvSrc);
				if(!(m_seqStates[iSeqSrc] & FLAG_SEQUENCE_STATE_REGISTRED) || !(GetSequence(iSeqSrc).GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_INLIER)
				|| (GetSequence(iSeqSrc).GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_COMMON_OUTLIER))
					continue;
				++nCrspsSrcInlier;
				if(nCrspsSrcInlier == 2)
					break;
			}
			if(nCrspsSrcInlier < 2)
				continue;
			iTrksCmnSrcToDst[iTrkCmnSrc] = TrackIndex(iTrksCmnBA.size());
			iTrksCmnBA.push_back(iTrkCmnSrc);

			for(SequenceIndex k = 0; k < nCrspsSrc; ++k)
			{
				iSeqTrksIdvSrc[k].Get(iSeqSrc, iTrkIdvSrc);
				if(!(m_seqStates[iSeqSrc] & FLAG_SEQUENCE_STATE_REGISTRED) || !(GetSequence(iSeqSrc).GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_INLIER)
				|| (GetSequence(iSeqSrc).GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_COMMON_OUTLIER))
					continue;
				++nTrksIdvDst;
				if(!(seqStates[iSeqSrc] & FLAG_SEQUENCE_STATE_ADJUSTED))
					seqStates[iSeqSrc] |= FLAG_SEQUENCE_STATE_FIXED;
			}
		}
	}

	// Step3: collect BA sequences
	iSeqsBA.resize(0);
	const SequenceIndex nSeqsSrc = GetSequencesNumber();
	for(iSeqSrc = 0; iSeqSrc < nSeqsSrc; ++iSeqSrc)
	{
		if(seqStates[iSeqSrc] & FLAG_SEQUENCE_STATE_FIXED)
			iSeqsBA.push_back(iSeqSrc);
	}
	nSeqsFix = SequenceIndex(iSeqsBA.size());
	iSeqsBA.insert(iSeqsBA.end(), iSeqsAdj.begin(), iSeqsAdj.end());

	// Step4: initialize all BA transformations and copy all BA common points
	const SequenceIndex nSeqsDst = SequenceIndex(iSeqsBA.size());
	const TrackIndex nTrksCmnDst = TrackIndex(iTrksCmnBA.size());
	data.Resize(nSeqsDst, nTrksCmnDst, nTrksIdvDst);
	for(SequenceIndex iSeqDst = 0; iSeqDst < nSeqsDst; ++iSeqDst)
		data.m_Cs[iSeqDst].MakeIdentity();
	TrackIndex iTrkCmnDst;
	for(iTrkCmnDst = 0; iTrkCmnDst < nTrksCmnDst; ++iTrkCmnDst)
	{
		iTrkCmnSrc = iTrksCmnBA[iTrkCmnDst];
		data.m_Xs[iTrkCmnDst] = m_XsCmn[iTrkCmnSrc];
	}

	// Step5: copy inlier individual tracks and create correspondence maps
	TrackIndex iTrkIdvDst = 0;
	data.m_mapSeqToIdvTrk[0] = 0;
	for(SequenceIndex iSeqDst = 0; iSeqDst < nSeqsDst; ++iSeqDst)
	{
		const SequenceIndex iSeqSrc = iSeqsBA[iSeqDst];
		const Sequence &seqSrc = GetSequence(iSeqSrc);
		const TrackIndexList &iTrksIdvToCmnSrc = m_mapIdvTrkToCmnTrk[iSeqSrc];
		const TrackIndex nTrksIdvSrc = seqSrc.GetTracksNumber();
		for(iTrkIdvSrc = 0; iTrkIdvSrc < nTrksIdvSrc; ++iTrkIdvSrc)
		{
			iTrkCmnSrc = iTrksIdvToCmnSrc[iTrkIdvSrc];
			if(iTrkCmnSrc == INVALID_TRACK_INDEX || (iTrkCmnDst = iTrksCmnSrcToDst[iTrkCmnSrc]) == INVALID_TRACK_INDEX
			|| !(seqSrc.GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_INLIER) || (seqSrc.GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				continue;
			data.m_xs[iTrkIdvDst] = seqSrc.GetPoint(iTrkIdvSrc);
			data.m_mapCmnTrkToIdvTrk[iTrkCmnDst].push_back(iTrkIdvDst);
			data.m_mapIdvTrkToSeq[iTrkIdvDst] = iSeqDst;
			data.m_mapIdvTrkToCmnTrk[iTrkIdvDst] = iTrkCmnDst;
			++iTrkIdvDst;
		}
		data.m_mapSeqToIdvTrk[iSeqDst + 1] = iTrkIdvDst;
	}
#if _DEBUG
	assert(iTrkIdvDst == nTrksIdvDst);
#endif
}

void SequenceSet::SetBundleAdjustmentResults(const SequenceSetBundleAdjustorData3DSimilarity &data, const SequenceIndex &nSeqsFix, const SequenceIndexList &iSeqsBA, 
											 const TrackIndexList &iTrksCmnBA)
{
	SimilarityTransformation3D Sinv;
	ENFT_SSE::__m128 work[2];
	const SequenceIndex nSeqsBA = SequenceIndex(iSeqsBA.size());
	for(SequenceIndex i = nSeqsFix; i < nSeqsBA; ++i)
	{
		data.m_Cs[i].Invert(Sinv, work);
		m_pSeqs[iSeqsBA[i]]->TransformScene(Sinv);
	}
	const TrackIndex nTrksCmnBA = TrackIndex(iTrksCmnBA.size());
	for(TrackIndex i = 0; i < nTrksCmnBA; ++i)
		SetCommonPoint(iTrksCmnBA[i], data.m_Xs[i]);
}

void SequenceSet::GetBundleAdjustorData(const SequenceIndexList &iSeqsAdj, SegmentSetBundleAdjustorData2DSimilarity &data, SequenceIndex &nSeqsFix, 
										SequenceIndexList &iSeqsBA, TrackIndexList &iTrksCmnBA, SequenceTrackIndexList &iSeqTrksIdvBA) const
{
	// Step1: mark adjusted sequence
	SequenceStateList seqStates = m_seqStates;
	const SequenceIndex nSeqsAdj = SequenceIndex(iSeqsAdj.size());
	for(SequenceIndex i = 0; i < nSeqsAdj; ++i)
		seqStates[iSeqsAdj[i]] |= FLAG_SEQUENCE_STATE_ADJUSTED;
//#if _DEBUG
//	std::vector<std::vector<bool> > meaMarksList(GetSequencesNumber());
//	for(SequenceIndex iSeq = 0; iSeq < GetSequencesNumber(); ++iSeq)
//		meaMarksList[iSeq].assign(GetSequence(iSeq).GetMeasurementsNumber(), false);
//#endif

	// Step2: mark fixed sequences, collect BA common tracks, create reverse common track index list iTrksCmnSrcToDst and count corresponding BA measurements
	iTrksCmnBA.resize(0);
	const TrackIndex nTrksCmnSrc = GetCommonTracksNumber();
	TrackIndexList iTrksCmnSrcToDst(nTrksCmnSrc, INVALID_TRACK_INDEX);
	std::vector<bool> trkCmnSrcMarks(nTrksCmnSrc, false);
	TrackIndex iTrkIdvSrc, iTrkCmnSrc;
	SequenceIndex iSeqSrc;
	MeasurementIndex nMeasDst = 0;
	for(SequenceIndex i = 0; i < nSeqsAdj; ++i)
	{
		const SequenceIndex iSeqAdj = iSeqsAdj[i];
		const Sequence &seqAdj = GetSequence(iSeqAdj);
		const TrackIndexList &iTrksIdvToCmnAdj = m_mapIdvTrkToCmnTrk[iSeqAdj];
		const TrackIndex nTrksIdvAdj = seqAdj.GetTracksNumber();
		for(TrackIndex j = 0; j < nTrksIdvAdj; ++j)
		{
			if((iTrkCmnSrc = iTrksIdvToCmnAdj[j]) == INVALID_TRACK_INDEX || trkCmnSrcMarks[iTrkCmnSrc] || !(seqAdj.GetTrackState(j) & FLAG_TRACK_STATE_INLIER)
			|| (seqAdj.GetTrackState(j) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				continue;
			trkCmnSrcMarks[iTrkCmnSrc] = true;
			const SequenceTrackIndexList &iSeqTrksIdvSrc = m_mapCmnTrkToIdvTrk[iTrkCmnSrc];
			const SequenceIndex nCrspsSrc = SequenceIndex(iSeqTrksIdvSrc.size());
			SequenceIndex nCrspsSrcInlier = 0;
			for(SequenceIndex k = 0; k < nCrspsSrc; ++k)
			{
				iSeqTrksIdvSrc[k].Get(iSeqSrc, iTrkIdvSrc);
				if(!(m_seqStates[iSeqSrc] & FLAG_SEQUENCE_STATE_REGISTRED) || !(GetSequence(iSeqSrc).GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_INLIER)
				|| (GetSequence(iSeqSrc).GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_COMMON_OUTLIER))
					continue;
				++nCrspsSrcInlier;
				if(nCrspsSrcInlier == 2)
					break;
			}
			if(nCrspsSrcInlier < 2)
				continue;
			iTrksCmnSrcToDst[iTrkCmnSrc] = TrackIndex(iTrksCmnBA.size());
			iTrksCmnBA.push_back(iTrkCmnSrc);

			for(SequenceIndex k = 0; k < nCrspsSrc; ++k)
			{
				iSeqTrksIdvSrc[k].Get(iSeqSrc, iTrkIdvSrc);
				if(!(m_seqStates[iSeqSrc] & FLAG_SEQUENCE_STATE_REGISTRED) || !(GetSequence(iSeqSrc).GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_INLIER)
				|| (GetSequence(iSeqSrc).GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_COMMON_OUTLIER))
					continue;
				nMeasDst += GetSequence(iSeqSrc).CountTrackInlierMeasurements(iTrkIdvSrc);
				if(!(seqStates[iSeqSrc] & FLAG_SEQUENCE_STATE_ADJUSTED))
					seqStates[iSeqSrc] |= FLAG_SEQUENCE_STATE_FIXED;
//#if _DEBUG
//				const Sequence &seq = GetSequence(iSeqSrc);
//				const MeasurementIndexList &iMeas = seq.GetTrackMeasurementIndexList(iTrkIdvSrc);
//				std::vector<bool> &meaMarks = meaMarksList[iSeqSrc];
//				const FrameIndex nCrsps = FrameIndex(iMeas.size());
//				for(FrameIndex i = 0; i < nCrsps; ++i)
//				{
//					const MeasurementIndex iMea = iMeas[i];
//					if((seq.GetFrameState(seq.GetMeasurementFrameIndex(iMea)) & FLAG_FRAME_STATE_SOLVED) && !(seq.GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER))
//					meaMarks[iMeas[i]] = true;
//				}
//#endif
			}
		}
	}

	// Step3: collect BA sequences
	iSeqsBA.resize(0);
	const SequenceIndex nSeqsSrc = GetSequencesNumber();
	for(iSeqSrc = 0; iSeqSrc < nSeqsSrc; ++iSeqSrc)
	{
		if(seqStates[iSeqSrc] & FLAG_SEQUENCE_STATE_FIXED)
			iSeqsBA.push_back(iSeqSrc);
	}
	nSeqsFix = SequenceIndex(iSeqsBA.size());
	iSeqsBA.insert(iSeqsBA.end(), iSeqsAdj.begin(), iSeqsAdj.end());
	const SequenceIndex nSeqsDst = SequenceIndex(iSeqsBA.size());
	data.m_pSeqs.resize(nSeqsDst);
	for(SequenceIndex i = 0; i < nSeqsDst; ++i)
		data.m_pSeqs[i] = m_pSeqs[iSeqsBA[i]];

	// Step4: segment frames
	std::vector<SegmentIndexList> mapSrcSeqFrmToDstSeg(nSeqsSrc);
	SegmentIndex iSegDst = nSeqsFix;
	data.m_mapSegToSeqFrm.resize(0);
	for(SequenceIndex iSeqDst = 0; iSeqDst < nSeqsFix; ++iSeqDst)
	{
		const SequenceIndex iSeqSrc = iSeqsBA[iSeqDst];
		data.m_mapSegToSeqFrm.push_back(SequenceFramePairIndex(iSeqDst, 0, GetSequence(iSeqSrc).GetFramesNumber()));
	}
	for(SequenceIndex iSeqDst = nSeqsFix; iSeqDst < nSeqsDst; ++iSeqDst)
	{
		const SequenceIndex iSeqSrc = iSeqsBA[iSeqDst];
		const Sequence &seqSrc = GetSequence(iSeqSrc);
		SegmentIndexList &iSegsDst = mapSrcSeqFrmToDstSeg[iSeqSrc];
		const FrameIndex nFrms = seqSrc.GetFramesNumber();
		iSegsDst.resize(nFrms);

		FrameIndex iFrm, iFrm1, iFrm2 = 0;
		while(iFrm2 < nFrms)
		{
			iFrm1 = iFrm2;
			for(iFrm2 = iFrm1 + 1; iFrm2 < nFrms && !(seqSrc.GetFrameState(iFrm2) & FLAG_FRAME_STATE_SPLIT_POINT); ++iFrm2);
			if(iFrm2 != nFrms && iFrm2 != iFrm1 + 1)
				++iFrm2;
			for(iFrm = iFrm1; iFrm < iFrm2; ++iFrm)
				iSegsDst[iFrm] = iSegDst;
			data.m_mapSegToSeqFrm.push_back(SequenceFramePairIndex(iSeqDst, iFrm1, iFrm2));
			++iSegDst;
		}
	}
	const SegmentIndex nSegsDst = iSegDst;

	// Step5: collect BA individual tracks, create reverse individual track index lists mapSeqTrkIdvToDstTrkIdv and count corresponding BA measurements
	TrackIndex iTrkDst = TrackIndex(iTrksCmnBA.size());
	iSeqTrksIdvBA.resize(0);
	std::vector<TrackIndexList> mapSrcSeqTrkIdvToDstTrkIdv(nSeqsSrc);
	for(SequenceIndex iSeqDst = 0; iSeqDst < nSeqsAdj; ++iSeqDst)
	{
		iSeqSrc = iSeqsAdj[iSeqDst];
		const Sequence &seqSrc = GetSequence(iSeqSrc);
		const SegmentIndexList &iSegsDst = mapSrcSeqFrmToDstSeg[iSeqSrc];
		const TrackIndexList &iTrksIdvToCmnSrc = m_mapIdvTrkToCmnTrk[iSeqSrc];
		TrackIndexList &iTrksIdvSrcToDst = mapSrcSeqTrkIdvToDstTrkIdv[iSeqSrc];
		const TrackIndex nTrksIdvSrc = seqSrc.GetTracksNumber();
		iTrksIdvSrcToDst.assign(nTrksIdvSrc, INVALID_TRACK_INDEX);
		for(iTrkIdvSrc = 0; iTrkIdvSrc < nTrksIdvSrc; ++iTrkIdvSrc)
		{
			if(!(seqSrc.GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_INLIER)
			|| (iTrkCmnSrc = iTrksIdvToCmnSrc[iTrkIdvSrc]) != INVALID_TRACK_INDEX && iTrksCmnSrcToDst[iTrkCmnSrc] != INVALID_TRACK_INDEX
			&& !(seqSrc.GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				continue;
			const MeasurementIndexList &iMeasSrc = seqSrc.GetTrackMeasurementIndexList(iTrkIdvSrc);
			if(iMeasSrc.empty() || iSegsDst[seqSrc.GetMeasurementFrameIndex(iMeasSrc.front())] == iSegsDst[seqSrc.GetMeasurementFrameIndex(iMeasSrc.back())])
				continue;
			iSeqTrksIdvBA.push_back(SequenceTrackIndex(iSeqSrc, iTrkIdvSrc));
			iTrksIdvSrcToDst[iTrkIdvSrc] = iTrkDst++;
			nMeasDst += seqSrc.CountTrackInlierMeasurements(iTrkIdvSrc);
//#if _DEBUG
//			const Sequence &seq = seqSrc;
//			const MeasurementIndexList &iMeas = seq.GetTrackMeasurementIndexList(iTrkIdvSrc);
//			std::vector<bool> &meaMarks = meaMarksList[iSeqSrc];
//			const FrameIndex nCrsps = FrameIndex(iMeas.size());
//			for(FrameIndex i = 0; i < nCrsps; ++i)
//			{
//				const MeasurementIndex iMea = iMeas[i];
//				if((seq.GetFrameState(seq.GetMeasurementFrameIndex(iMea)) & FLAG_FRAME_STATE_SOLVED) && !(seq.GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER))
//					meaMarks[iMeas[i]] = true;
//			}
//#endif
		}
	}
	const TrackIndex nTrksDst = iTrkDst;

	// Step6: initialize all BA transformations and copy all BA common points
	data.Resize(nSegsDst, nTrksDst, nMeasDst);
	for(iSegDst = 0; iSegDst < nSegsDst; ++iSegDst)
		data.m_Cs[iSegDst].MakeIdentity();
	const TrackIndex nTrksCmnDst = TrackIndex(iTrksCmnBA.size());
	for(iTrkDst = 0; iTrkDst < nTrksCmnDst; ++iTrkDst)
	{
		const TrackIndex iTrkCmnSrc = iTrksCmnBA[iTrkDst];
		data.m_Xs[iTrkDst] = m_XsCmn[iTrkCmnSrc];
	}
	const TrackIndex nTrksIdvDst = TrackIndex(iSeqTrksIdvBA.size());
	for(TrackIndex i = 0; i < nTrksIdvDst; ++i, ++iTrkDst)
	{
		iSeqTrksIdvBA[i].Get(iSeqSrc, iTrkIdvSrc);
		data.m_Xs[iTrkDst] = GetSequence(iSeqSrc).GetPoint(iTrkIdvSrc);
	}

	// Step7: copy inlier measurements and create correspondence maps
	MeasurementIndex iMeaDst = 0;
	MeasurementIndexList segMeaCnts(nSegsDst, 0);
	for(SequenceIndex iSeqDst = 0; iSeqDst < nSeqsFix; ++iSeqDst)
	{
		const SequenceIndex iSeqSrc = iSeqsBA[iSeqDst];
		const Sequence &seqSrc = GetSequence(iSeqSrc);
		const bool rectData = seqSrc.GetIntrinsicType() == Sequence::INTRINSIC_VARIABLE;
		const TrackIndexList &iTrksIdvToCmnSrc = m_mapIdvTrkToCmnTrk[iSeqSrc];
		const MeasurementIndex nMeasSrc = seqSrc.GetMeasurementsNumber();
		for(MeasurementIndex iMeaSrc = 0; iMeaSrc < nMeasSrc; ++iMeaSrc)
		{
			if((seqSrc.GetMeasurementState(iMeaSrc) & FLAG_MEASUREMENT_STATE_OUTLIER) || (iTrkIdvSrc = seqSrc.GetMeasurementTrackIndex(iMeaSrc)) == INVALID_TRACK_INDEX
			|| (iTrkCmnSrc = iTrksIdvToCmnSrc[iTrkIdvSrc]) == INVALID_TRACK_INDEX || (iTrkDst = iTrksCmnSrcToDst[iTrkCmnSrc]) == INVALID_TRACK_INDEX
			|| !(seqSrc.GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_INLIER) || (seqSrc.GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				continue;
			data.m_xs[iMeaDst] = seqSrc.GetMeasurement(iMeaSrc);
			if(rectData)
				seqSrc.GetIntrinsicRectification(seqSrc.GetMeasurementFrameIndex(iMeaSrc)).Rectify(data.m_xs[iMeaDst]);
			data.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			data.m_mapMeaToSeg[iMeaDst] = iSegDst = iSeqDst;
			data.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			data.m_mapMeaToSeqFrm[iMeaDst].Set(iSeqDst, seqSrc.GetMeasurementFrameIndex(iMeaSrc));
			++segMeaCnts[iSegDst];
			++iMeaDst;
//#if _DEBUG
//			if(!meaMarksList[iSeqSrc][iMeaSrc])
//				printf("seq%d, mea%d\n", iSeqSrc, iMeaSrc);
//#endif
		}
	}
	for(SequenceIndex iSeqDst = nSeqsFix; iSeqDst < nSeqsDst; ++iSeqDst)
	{
		const SequenceIndex iSeqSrc = iSeqsBA[iSeqDst];
		const Sequence &seqSrc = GetSequence(iSeqSrc);
		const bool rectData = seqSrc.GetIntrinsicType() == Sequence::INTRINSIC_VARIABLE;
		const TrackIndexList &iTrksIdvToCmnSrc = m_mapIdvTrkToCmnTrk[iSeqSrc];
		const TrackIndexList &iTrksIdvSrcToDst = mapSrcSeqTrkIdvToDstTrkIdv[iSeqSrc];
		const SegmentIndexList &iSegsDst = mapSrcSeqFrmToDstSeg[iSeqSrc];
		const MeasurementIndex nMeasSrc = seqSrc.GetMeasurementsNumber();
		for(MeasurementIndex iMeaSrc = 0; iMeaSrc < nMeasSrc; ++iMeaSrc)
		{
			if((seqSrc.GetMeasurementState(iMeaSrc) & FLAG_MEASUREMENT_STATE_OUTLIER) || (iTrkIdvSrc = seqSrc.GetMeasurementTrackIndex(iMeaSrc)) == INVALID_TRACK_INDEX
			|| ((iTrkCmnSrc = iTrksIdvToCmnSrc[iTrkIdvSrc]) == INVALID_TRACK_INDEX || (iTrkDst = iTrksCmnSrcToDst[iTrkCmnSrc]) == INVALID_TRACK_INDEX
			|| !(seqSrc.GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_INLIER) || (seqSrc.GetTrackState(iTrkIdvSrc) & FLAG_TRACK_STATE_COMMON_OUTLIER))
			&& (iTrkDst = iTrksIdvSrcToDst[iTrkIdvSrc]) == INVALID_TRACK_INDEX)
				continue;
			data.m_xs[iMeaDst] = seqSrc.GetMeasurement(iMeaSrc);
			if(rectData)
				seqSrc.GetIntrinsicRectification(seqSrc.GetMeasurementFrameIndex(iMeaSrc)).Rectify(data.m_xs[iMeaDst]);
			data.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			data.m_mapMeaToSeg[iMeaDst] = iSegDst = iSegsDst[seqSrc.GetMeasurementFrameIndex(iMeaSrc)];
			data.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			data.m_mapMeaToSeqFrm[iMeaDst].Set(iSeqDst, seqSrc.GetMeasurementFrameIndex(iMeaSrc));
			++segMeaCnts[iSegDst];
			++iMeaDst;
//#if _DEBUG
//			if(!meaMarksList[iSeqSrc][iMeaSrc])
//				printf("seq%d, mea%d\n", iSeqSrc, iMeaSrc);
//#endif
		}
	}
	data.m_mapSegToMea[0] = 0;
	for(iSegDst = 0; iSegDst < nSegsDst; ++iSegDst)
		data.m_mapSegToMea[iSegDst + 1] = data.m_mapSegToMea[iSegDst] + segMeaCnts[iSegDst];

#if _DEBUG
	assert(iMeaDst == nMeasDst);
#endif

	if(m_intrinsicType == Sequence::INTRINSIC_CONSTANT)
	{
		double MSEMin = DBL_MAX, MSE;
		const SequenceIndex nSeqsBA = SequenceIndex(iSeqsBA.size());
		for(SequenceIndex i = 0; i < nSeqsBA; ++i)
		{
			const Camera::IntrinsicParameter &Kr = m_pSeqs[iSeqsBA[i]]->GetIntrinsicRectification();
			if((MSE = data.ComputeMSE(Kr)) > MSEMin)
				continue;
			MSEMin = MSE;
			data.m_G = Kr;
		}
	}
}

void SequenceSet::SetBundleAdjustmentResults(const SegmentSetBundleAdjustorData2DSimilarity &data, const SegmentIndex &nSegsFix, const SequenceIndexList &iSeqsBA, 
											 const TrackIndexList &iTrksCmnBA, const SequenceTrackIndexList &iSeqTrksIdvBA, const bool toInternal)
{
	const TrackIndex nTrksCmnBA = TrackIndex(iTrksCmnBA.size());
	for(TrackIndex i = 0; i < nTrksCmnBA; ++i)
		SetCommonPoint(iTrksCmnBA[i], data.m_Xs[i]);
	if(toInternal)
	{
		const SequenceIndex nSeqsBA = SequenceIndex(iSeqsBA.size());
		std::vector<std::vector<bool> > seqTrkIdvMarksList(GetSequencesNumber());
		SequenceIndex iSeq;
		for(SequenceIndex i = 0; i < nSeqsBA; ++i)
		{
			iSeq = iSeqsBA[i];
			seqTrkIdvMarksList[iSeq].assign(GetSequence(iSeq).GetTracksNumber(), false);
		}

		TrackIndex iTrkCmn, iTrkIdv;
		const TrackIndex nSeqTrksIdvBA = TrackIndex(iSeqTrksIdvBA.size());
		for(TrackIndex i = 0, j = nTrksCmnBA; i < nSeqTrksIdvBA; ++i, ++j)
		{
			iSeqTrksIdvBA[i].Get(iSeq, iTrkIdv);
			m_pSeqs[iSeq]->SetPoint(iTrkIdv, data.m_Xs[j]);
			seqTrkIdvMarksList[iSeq][iTrkIdv] = true;
			if((iTrkCmn = m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv]) != INVALID_TRACK_INDEX && !(m_pSeqs[iSeq]->GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				SetCommonPoint(iTrkCmn, data.m_Xs[j]);
		}

		FrameIndex iFrm, iFrm1, iFrm2;
		SegmentIndex iSeg, iSegEnd, iSegStart;
		SimilarityTransformation3D Sinv;
		Point3DEstimatorData Xdata;
		Point3DEstimator Xestor;
		Point3D Xidv;
		ENFT_SSE::__m128 work[3];
		const SegmentIndex nSegsBA = SegmentIndex(data.m_Cs.Size());
		for(iSegEnd = 0; iSegEnd < nSegsBA; ++iSegEnd)
		{
			data.m_mapSegToSeqFrm[iSegEnd].Get(iSeq, iFrm1, iFrm2);
			iSeq = iSeqsBA[iSeq];
			if(iFrm1 == 0)
				iSegStart = iSegEnd;
			if(iFrm2 < m_pSeqs[iSeq]->GetFramesNumber())
				continue;
			Sequence &seq = *m_pSeqs[iSeq];
			for(iSeg = iSegStart; iSeg <= iSegEnd; ++iSeg)
			{
				data.m_Cs[iSeg].Invert(Sinv, work);
				iFrm1 = data.m_mapSegToSeqFrm[iSeg].GetFrameIndex1();
				iFrm2 = data.m_mapSegToSeqFrm[iSeg].GetFrameIndex2();
				for(iFrm = iFrm1; iFrm < iFrm2; ++iFrm)
					seq.TransformCamera(iFrm, Sinv, work);
			}
			const TrackIndexList &iTrksCmn = m_mapIdvTrkToCmnTrk[iSeq];
			std::vector<bool> &trkIdvMarks = seqTrkIdvMarksList[iSeq];
			for(iSeg = iSegStart; iSeg <= iSegEnd; ++iSeg)
			{
				data.m_Cs[iSeg].Invert(Sinv, work);
				iFrm1 = data.m_mapSegToSeqFrm[iSeg].GetFrameIndex1();
				iFrm2 = data.m_mapSegToSeqFrm[iSeg].GetFrameIndex2();
				for(iFrm = iFrm1; iFrm < iFrm2; ++iFrm)
				{
					const TrackIndex *iTrksIdv = seq.GetFrameTrackIndexes(iFrm);
					const FeatureIndex nFtrs = seq.GetFrameFeaturesNumber(iFrm);
					for(FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr)
					{
						iTrkIdv = iTrksIdv[iFtr];
						const MeasurementIndexList &iMeas = seq.GetTrackMeasurementIndexList(iTrkIdv);
						if(trkIdvMarks[iTrkIdv] || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) || iMeas.empty())
							continue;
						if(seq.GetMeasurementFrameIndex(iMeas.front()) >= iFrm1 && seq.GetMeasurementFrameIndex(iMeas.back()) < iFrm2)
							seq.TransformPoint(iTrkIdv, Sinv, work);
						else
						{
							//printf("seq%d, frm%d, ftr%d...", iSeq, iFrm, iFtr);
							seq.GetPoint3DEstimatorDataInlier(iTrkIdv, Xdata, true);
							if(Xestor.Triangulate(Xdata, Xidv))
								seq.SetPoint(iTrkIdv, Xidv);
							else if((iTrkCmn = iTrksCmn[iTrkIdv]) != INVALID_TRACK_INDEX)
								seq.SetPoint(iTrkIdv, GetCommonPoint(iTrkCmn));
							else
								seq.TransformPoint(iTrkIdv, Sinv, work);
							//printf("done!\n");
						}
						trkIdvMarks[iTrkIdv] = true;
					}
				}
			}
		}
	}
	else
	{
		SequenceIndex iSeq;
		TrackIndex iTrkCmn, iTrkIdv;
		const TrackIndex nSeqTrksIdvBA = TrackIndex(iSeqTrksIdvBA.size());
		for(TrackIndex i = 0, j = nTrksCmnBA; i < nSeqTrksIdvBA; ++i, ++j)
		{
			iSeqTrksIdvBA[i].Get(iSeq, iTrkIdv);
			if((iTrkCmn = m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv]) != INVALID_TRACK_INDEX && !(m_pSeqs[iSeq]->GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				SetCommonPoint(iTrkCmn, data.m_Xs[j]);
		}

		FrameIndex iFrm, iFrm1, iFrm2;
		SegmentIndex iSeg, iSegBest, iSegEnd, iSegStart;
		SimilarityTransformation3D Sinv;
		Point3D center;
		AlignedVector<Point3D> centers;
		Camera C;
		float SSE, SSEMin;
		ENFT_SSE::__m128 work[3];
		const SegmentIndex nSegsBA = SegmentIndex(data.m_Cs.Size());
		for(iSegEnd = 0; iSegEnd < nSegsBA; ++iSegEnd)
		{
			data.m_mapSegToSeqFrm[iSegEnd].Get(iSeq, iFrm1, iFrm2);
			iSeq = iSeqsBA[iSeq];
			if(iFrm1 == 0)
				iSegStart = iSegEnd;
			if(iFrm2 < m_pSeqs[iSeq]->GetFramesNumber())
				continue;
			Sequence &seq = *m_pSeqs[iSeq];
			const FrameIndex nFrms = seq.GetFramesNumber();
			centers.Resize(nFrms);
			for(iSeg = iSegStart; iSeg <= iSegEnd; ++iSeg)
			{
				data.m_Cs[iSeg].Invert(Sinv, work);
				iFrm1 = data.m_mapSegToSeqFrm[iSeg].GetFrameIndex1();
				iFrm2 = data.m_mapSegToSeqFrm[iSeg].GetFrameIndex2();
				for(iFrm = iFrm1; iFrm < iFrm2; ++iFrm)
				{
					Sinv.Apply(seq.GetCamera(iFrm), C, work);
					C.GetCenter(centers[iFrm]);
				}
			}
			iSegBest = ((iSegStart + iSegEnd) >> 1);
			const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
			const TrackIndex nTrksIdv = seq.GetTracksNumber();
			for(iSeg = iSegStart/*, iSegBest = 0*//*, sccCntMax = 0*/, SSEMin = FLT_MAX; iSeg <= iSegEnd; ++iSeg)
			{
				//const SimilarityTransformation &S = data.m_Cs[iSeg];
				//for(iTrkIdv = 0, sccCnt = 0, SSE = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
				//{
				//	if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) == INVALID_TRACK_INDEX || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
				//	|| (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				//		continue;
				//	S.Apply(m_XsCmn[iTrkCmn], Xidv);
				//	if(seq.ComputeTrackProjectionMSE(iTrkIdv, Xidv, MSE))
				//	{
				//		++sccCnt;
				//		SSE += MSE;
				//	}
				//	//else
				//	//{
				//	//	SSE = FLT_MAX;
				//	//	break;
				//	//}
				//}
				data.m_Cs[iSeg].Invert(Sinv, work);
				for(iFrm = 0, SSE = 0; iFrm < nFrms; ++iFrm)
				{
					Sinv.Apply(seq.GetCamera(iFrm), C, work);
					C.GetCenter(center);
					//SSE += center.SquaredDistance(centers[iFrm]);
					SSE = std::max(SSE, center.SquaredDistance(centers[iFrm]));
				}
				if(/*sccCnt > sccCntMax || sccCnt == sccCntMax && */SSE < SSEMin)
				{
					//sccCntMax = sccCnt;
					SSEMin = SSE;
					iSegBest = iSeg;
					//printf("iSeq = %d, sccCntMax = %d, SSEMin = %f, iSegBest = %d\n", iSeq, sccCntMax, SSEMin, iSegBest);
				}
			}
			data.m_Cs[iSegBest].Invert(Sinv, work);
			seq.TransformScene(Sinv);
		}
	}
	if(data.IsGlobalValid())
	{
		const SequenceIndex nSeqs = GetSequencesNumber();
		for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
			m_pSeqs[iSeq]->SetIntrinsicRectification(data.GetGlobal());
	}
}

void SequenceSet::GetSequenceTransformationOptimizerData(SequenceTransformationOptimizerDataSimilarity &data, SequenceIndexList &iSeqsAdj) const
{
	const SequenceIndex nSeqs = GetSequencesNumber();
	iSeqsAdj.resize(0);
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		if(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED)
			iSeqsAdj.push_back(iSeq);
	}
	const SequenceIndex nSeqsAdj = SequenceIndex(iSeqsAdj.size());
	data.m_Ts.Resize(nSeqsAdj);
	//data.m_Tinvs.Resize(nSeqsAdj);
	data.m_mapPairs.resize(0);
	data.m_nPts = 0;

	SequenceIndex i1, i2, iSeq1, iSeq2;
	TrackIndex iTrkIdv1, iTrkIdv2, iTrkCmn;
	TrackIndexList iTrksCmnToIdv1;
	TrackMatchList trkMatches;
	std::vector<TrackMatchList> trkMatchesList;
	uint iBlock = uint(nSeqsAdj);
	const TrackIndex nTrksCmn = GetCommonTracksNumber();
	for(i1 = 0; i1 < nSeqsAdj; ++i1)
	{
		iTrksCmnToIdv1.assign(nTrksCmn, INVALID_TRACK_INDEX);

		iSeq1 = iSeqsAdj[i1];
		const Sequence &seq1 = GetSequence(iSeq1);
		const TrackIndexList &iTrksIdv1ToCmn = m_mapIdvTrkToCmnTrk[iSeq1];
		const TrackIndex nTrksIdv1 = TrackIndex(iTrksIdv1ToCmn.size());
		for(iTrkIdv1 = 0; iTrkIdv1 < nTrksIdv1; ++iTrkIdv1)
		{
			if((iTrkCmn = iTrksIdv1ToCmn[iTrkIdv1]) != INVALID_TRACK_INDEX && (seq1.GetTrackState(iTrkIdv1) & FLAG_TRACK_STATE_INLIER)
			&& !(seq1.GetTrackState(iTrkIdv1) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				iTrksCmnToIdv1[iTrkCmn] = iTrkIdv1;
		}

		for(i2 = i1 + 1; i2 < nSeqsAdj; ++i2)
		{
			trkMatches.resize(0);
			iSeq2 = iSeqsAdj[i2];
			const Sequence &seq2 = GetSequence(iSeq2);
			const TrackIndexList &iTrksIdv2ToCmn = m_mapIdvTrkToCmnTrk[iSeq2];
			const TrackIndex nTrksIdv2 = TrackIndex(iTrksIdv2ToCmn.size());
			for(iTrkIdv2 = 0; iTrkIdv2 < nTrksIdv2; ++iTrkIdv2)
			{
				if((iTrkCmn = iTrksIdv2ToCmn[iTrkIdv2]) != INVALID_TRACK_INDEX && (iTrkIdv1 = iTrksCmnToIdv1[iTrkCmn]) != INVALID_TRACK_INDEX
				&& (seq2.GetTrackState(iTrkIdv2) & FLAG_TRACK_STATE_INLIER) && !(seq2.GetTrackState(iTrkIdv2) & FLAG_TRACK_STATE_COMMON_OUTLIER))
					trkMatches.push_back(TrackMatch(iTrkIdv1, iTrkIdv2));
			}
			const TrackIndex nTrkMatches = TrackIndex(trkMatches.size());
			if(nTrkMatches == 0)
				continue;

			data.m_mapPairs.push_back(SequenceTransformationOptimizerDataSimilarity::MapPair(i1, i2, iBlock++));
			trkMatchesList.push_back(trkMatches);
			//AlignedVector<SequenceTransformationOptimizerDataSimilarity::PointPair> &Xs = data.m_mapPairs.back().Xs();
			//Xs.Resize(nTrkMatches);
			//for(TrackIndex i = 0; i < nTrkMatches; ++i)
			//{
			//	trkMatches[i].Get(iTrkIdv1, iTrkIdv2);
			//	Xs[i].X1() = seq1.GetPoint(iTrkIdv1);
			//	Xs[i].X2() = seq2.GetPoint(iTrkIdv2);
			//}
			data.m_nPts += (nTrkMatches << 1);
		}
	}

	const uint nSeqPairs = uint(trkMatchesList.size());
	for(uint i = 0; i < nSeqPairs; ++i)
	{
		data.m_mapPairs[i].GetIndexes(i1, i2, iBlock);
		iSeq1 = iSeqsAdj[i1];
		iSeq2 = iSeqsAdj[i2];
		const Sequence &seq1 = GetSequence(iSeq1), &seq2 = GetSequence(iSeq2);

		AlignedVector<SequenceTransformationOptimizerDataSimilarity::PointPair> &Xs = data.m_mapPairs[i].Xs();
		const TrackMatchList &trkMatches = trkMatchesList[i];
		const TrackIndex nTrkMatches = TrackIndex(trkMatches.size());
		Xs.Resize(nTrkMatches);
		for(TrackIndex j = 0; j < nTrkMatches; ++j)
		{
			trkMatches[j].Get(iTrkIdv1, iTrkIdv2);
			Xs[j].X1() = seq1.GetPoint(iTrkIdv1);
			Xs[j].X2() = seq2.GetPoint(iTrkIdv2);
		}
	}

	for(SequenceIndex i = 0; i < nSeqsAdj; ++i)
		data.m_Ts[i].MakeIdentity();
}

void SequenceSet::SetSequenceTransformationOptimizationResults(const SequenceTransformationOptimizerDataSimilarity &data, const SequenceIndexList &iSeqsAdj)
{
	const SequenceIndex nSeqsAdj = SequenceIndex(iSeqsAdj.size());
	for(SequenceIndex i = 0; i < nSeqsAdj; ++i)
	{
		const SequenceIndex iSeq = iSeqsAdj[i];
		m_pSeqs[iSeq]->TransformScene(data.m_Ts[iSeq]);
	}
}

void SequenceSet::GetSequenceTransformationOptimizerData(SequenceTransformationOptimizerDataScale &data, SequenceIndexList &iSeqsAdj) const
{
	const SequenceIndex nSeqs = GetSequencesNumber();
	iSeqsAdj.resize(0);
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		if(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED)
			iSeqsAdj.push_back(iSeq);
	}
	const SequenceIndex nSeqsAdj = SequenceIndex(iSeqsAdj.size());
	data.m_Ts.Resize(nSeqsAdj);
	//data.m_Tinvs.Resize(nSeqsAdj);
	data.m_mapPairs.resize(0);
	data.m_nPts = 0;

	SequenceIndex i1, i2, iSeq1, iSeq2;
	TrackIndex iTrkIdv1, iTrkIdv2, iTrkCmn;
	TrackIndexList iTrksCmnToIdv1;
	TrackMatchList trkMatches;
	std::vector<TrackMatchList> trkMatchesList;
	uint iBlock = uint(nSeqsAdj);
	const TrackIndex nTrksCmn = GetCommonTracksNumber();
	for(i1 = 0; i1 < nSeqsAdj; ++i1)
	{
		iTrksCmnToIdv1.assign(nTrksCmn, INVALID_TRACK_INDEX);

		iSeq1 = iSeqsAdj[i1];
		const Sequence &seq1 = GetSequence(iSeq1);
		const TrackIndexList &iTrksIdv1ToCmn = m_mapIdvTrkToCmnTrk[iSeq1];
		const TrackIndex nTrksIdv1 = TrackIndex(iTrksIdv1ToCmn.size());
		for(iTrkIdv1 = 0; iTrkIdv1 < nTrksIdv1; ++iTrkIdv1)
		{
			if((iTrkCmn = iTrksIdv1ToCmn[iTrkIdv1]) != INVALID_TRACK_INDEX && (seq1.GetTrackState(iTrkIdv1) & FLAG_TRACK_STATE_INLIER)
			&& !(seq1.GetTrackState(iTrkIdv1) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				iTrksCmnToIdv1[iTrkCmn] = iTrkIdv1;
		}

		for(i2 = i1 + 1; i2 < nSeqsAdj; ++i2)
		{
			trkMatches.resize(0);
			iSeq2 = iSeqsAdj[i2];
			const Sequence &seq2 = GetSequence(iSeq2);
			const TrackIndexList &iTrksIdv2ToCmn = m_mapIdvTrkToCmnTrk[iSeq2];
			const TrackIndex nTrksIdv2 = TrackIndex(iTrksIdv2ToCmn.size());
			for(iTrkIdv2 = 0; iTrkIdv2 < nTrksIdv2; ++iTrkIdv2)
			{
				if((iTrkCmn = iTrksIdv2ToCmn[iTrkIdv2]) != INVALID_TRACK_INDEX && (iTrkIdv1 = iTrksCmnToIdv1[iTrkCmn]) != INVALID_TRACK_INDEX
				&& (seq2.GetTrackState(iTrkIdv2) & FLAG_TRACK_STATE_INLIER) && !(seq2.GetTrackState(iTrkIdv2) & FLAG_TRACK_STATE_COMMON_OUTLIER))
					trkMatches.push_back(TrackMatch(iTrkIdv1, iTrkIdv2));
			}
			const TrackIndex nTrkMatches = TrackIndex(trkMatches.size());
			if(nTrkMatches == 0)
				continue;
			data.m_mapPairs.push_back(SequenceTransformationOptimizerDataScale::MapPair(i1, i2, iBlock++));
			trkMatchesList.push_back(trkMatches);
		}
	}

	TrackIndex iTrkIdv3, iTrkIdv4;
	const uint nSeqPairs = uint(trkMatchesList.size());
	for(uint i = 0; i < nSeqPairs; ++i)
	{
		data.m_mapPairs[i].GetIndexes(i1, i2, iBlock);
		iSeq1 = iSeqsAdj[i1];
		iSeq2 = iSeqsAdj[i2];
		const Sequence &seq1 = GetSequence(iSeq1), &seq2 = GetSequence(iSeq2);

		AlignedVector<SequenceTransformationOptimizerDataScale::PointPair> &ls = data.m_mapPairs[i].Xs();
		const TrackMatchList &trkMatches = trkMatchesList[i];
		const TrackIndex nTrkMatches = TrackIndex(trkMatches.size()), nLines = nTrkMatches * (nTrkMatches - 1) / 2;

		ls.Resize(nLines);
		for(TrackIndex j1 = 0, k = 0; j1 < nTrkMatches; ++j1)
		{
			trkMatches[j1].Get(iTrkIdv1, iTrkIdv2);
			const Point3D &X1 = seq1.GetPoint(iTrkIdv1), &X2 = seq2.GetPoint(iTrkIdv2);
			for(TrackIndex j2 = j1 + 1; j2 < nTrkMatches; ++j2, ++k)
			{
				trkMatches[j2].Get(iTrkIdv3, iTrkIdv4);
				const Point3D &X3 = seq1.GetPoint(iTrkIdv3), &X4 = seq2.GetPoint(iTrkIdv4);
				ls[k].X1() = sqrt(X1.SquaredDistance(X3));
				ls[k].X2() = sqrt(X2.SquaredDistance(X4));
			}
		}
		data.m_nPts += (nLines << 1);
	}

	for(SequenceIndex i = 0; i < nSeqsAdj; ++i)
		data.m_Ts[i] = 1.0f;
}

void SequenceSet::SetSequenceTransformationOptimizationResults(const SequenceTransformationOptimizerDataScale &data, const SequenceIndexList &iSeqsAdj)
{
	const SequenceIndex nSeqsAdj = SequenceIndex(iSeqsAdj.size());
	for(SequenceIndex i = 0; i < nSeqsAdj; ++i)
	{
		const SequenceIndex iSeq = iSeqsAdj[i];
		m_pSeqs[iSeq]->ScaleScene(data.m_Ts[iSeq]);
	}
}