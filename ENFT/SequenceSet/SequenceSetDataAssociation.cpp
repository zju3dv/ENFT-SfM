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
#include "SfM/Point3DEstimator.h"

void SequenceSet::GetSubSequences(const std::vector<FrameIndexList> &iFrmsList, const std::vector<TrackIndexList> &iTrksListIdv, SequenceSet &seqsSub, 
								  TrackIndexList &iTrksCmn, const bool copyDesc, const bool copyClr) const
{
	const SequenceIndex nSeqs = GetSequencesNumber();
	seqsSub.m_dir = m_dir;
	seqsSub.m_intrinsicType = m_intrinsicType;
#if _DEBUG
	assert(seqsSub.GetSequencesNumber() == nSeqs);
#endif
	seqsSub.CreateSequences(nSeqs);
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		Sequence &seqSub = *seqsSub.m_pSeqs[iSeq];
		GetSequence(iSeq).GetSubSequence(iFrmsList[iSeq], iTrksListIdv[iSeq], seqSub, copyDesc, copyClr);
		seqsSub.m_mapIdvTrkToCmnTrk[iSeq].assign(seqSub.GetTracksNumber(), INVALID_TRACK_INDEX);
	}

	iTrksCmn.resize(0);
	TrackIndexList iTrksCmnSrcToDst(GetCommonTracksNumber(), INVALID_TRACK_INDEX);
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		const TrackIndexList &iTrksIdvToCmnSrc = m_mapIdvTrkToCmnTrk[iSeq];
		const TrackIndexList &iTrksIdv = iTrksListIdv[iSeq];
		const TrackIndex nTrksIdvDst = TrackIndex(iTrksIdv.size());
		for(TrackIndex iTrkIdvDst = 0; iTrkIdvDst < nTrksIdvDst; ++iTrkIdvDst)
		{
			const TrackIndex iTrkCmnSrc = iTrksIdvToCmnSrc[iTrksIdv[iTrkIdvDst]];
			if(iTrkCmnSrc == INVALID_TRACK_INDEX)
				continue;
			const TrackIndex iTrkCmnDst = iTrksCmnSrcToDst[iTrkCmnSrc];
			if(iTrkCmnDst == INVALID_TRACK_INDEX)
			{
				iTrksCmnSrcToDst[iTrkCmnSrc] = seqsSub.GetCommonTracksNumber();
				iTrksCmn.push_back(iTrkCmnSrc);
				seqsSub.PushBackCommonTrack(iSeq, iTrkIdvDst);
			}
			else
				seqsSub.MatchCommonTrackAndIndividualTrack(iTrkCmnDst, iSeq, iTrkIdvDst);
		}
	}

	// Remove single common tracks
	SequenceIndex iSeq;
	TrackIndex iTrkCmnOri, iTrkCmnNew, iTrkIdv;
	const TrackIndex nTrksCmnOri = TrackIndex(iTrksCmn.size());
	for(iTrkCmnOri = iTrkCmnNew = 0; iTrkCmnOri < nTrksCmnOri; ++iTrkCmnOri)
	{
		const SequenceTrackIndexList &iSeqTrksIdv = seqsSub.m_mapCmnTrkToIdvTrk[iTrkCmnOri];
		const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
		if(nCrsps == 1)
		{
			iSeqTrksIdv[0].Get(iSeq, iTrkIdv);
			seqsSub.m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] = INVALID_TRACK_INDEX;
			seqsSub[iSeq].MarkTrackIndividual(iTrkIdv);
		}
		else
		{
			for(SequenceIndex i = 0; i < nCrsps; ++i)
			{
				iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
				seqsSub.m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] = iTrkCmnNew;
			}
			iTrksCmn[iTrkCmnNew] = iTrksCmn[iTrkCmnOri];
			seqsSub.m_mapCmnTrkToIdvTrk[iTrkCmnNew] = iSeqTrksIdv;
			seqsSub.m_cmnTrkStates[iTrkCmnNew] = seqsSub.m_cmnTrkStates[iTrkCmnOri];
			++iTrkCmnNew;
		}
	}
	const TrackIndex nTrksCmnNew = iTrkCmnNew;
	iTrksCmn.resize(nTrksCmnNew);
	seqsSub.m_mapCmnTrkToIdvTrk.resize(nTrksCmnNew);
	seqsSub.m_cmnTrkStates.assign(nTrksCmnNew, FLAG_COMMON_TRACK_STATE_DEFAULT);

	if(m_XsCmn.Empty())
		return;
	seqsSub.SetCommonPointsNumber(nTrksCmnNew);
	for(iTrkCmnNew = 0; iTrkCmnNew < nTrksCmnNew; ++iTrkCmnNew)
		seqsSub.SetCommonPoint(iTrkCmnNew, m_XsCmn[iTrksCmn[iTrkCmnNew]]);
}

void SequenceSet::MarkSequenceConnectedComponent(const SequenceIndex &iSeq, std::vector<bool> &seqMarks) const
{
	const SequenceIndex nSeqs = GetSequencesNumber();
	seqMarks.assign(nSeqs, false);

	TrackIndex iTrkIdv, iTrkCmn;
	const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
	const TrackIndex nTrksIdv = TrackIndex(iTrksIdvToCmn.size());
	for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
	{
		if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) == INVALID_TRACK_INDEX)
			continue;
		const SequenceTrackIndexList &iSeqTrksIdvCC = m_mapCmnTrkToIdvTrk[iTrkCmn];
		const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdvCC.size());
		for(SequenceIndex i = 0; i < nCrsps; ++i)
			seqMarks[iSeqTrksIdvCC[i].GetSequenceIndex()] = true;
	}
}

void SequenceSet::SearchForSequenceConnectedComponent(const SequenceIndex &iSeq, SequenceIndexList &iSeqsCC) const
{
	std::vector<bool> seqMarks;
	MarkSequenceConnectedComponent(iSeq, seqMarks);

	iSeqsCC.resize(0);
	const SequenceIndex nSeqs = GetSequencesNumber();
	for(SequenceIndex iSeqCC = 0; iSeqCC < nSeqs; ++iSeqCC)
	{
		if(seqMarks[iSeqCC])
			iSeqsCC.push_back(iSeqCC);
	}
}

void SequenceSet::SearchForFrameFeatureMatches(const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches) const
{
	const Sequence &seq1 = GetSequence(iSeq1), &seq2 = GetSequence(iSeq2);
	const TrackIndexList &iTrksIdvToCmn1 = m_mapIdvTrkToCmnTrk[iSeq1];
	
	FeatureIndex iFtr1, iFtr2;
	TrackIndex iTrkIdv1, iTrkIdv2, iTrkCmn;

	matches.resize(0);
	const FeatureIndex nFtrs1 = seq1.GetFrameFeaturesNumber(iFrm1);
	const TrackIndex *iTrksIdv1 = seq1.GetFrameTrackIndexes(iFrm1);
	for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1)
	{
		if((iTrkIdv1 = iTrksIdv1[iFtr1]) != INVALID_TRACK_INDEX && (iTrkCmn = iTrksIdvToCmn1[iTrkIdv1]) != INVALID_TRACK_INDEX
		&& (iTrkIdv2 = SearchCommonTrackForIndividualTrack(iTrkCmn, iSeq2)) != INVALID_TRACK_INDEX
		&& (iFtr2 = seq2.SearchTrackForFrameFeatureIndex(iTrkIdv2, iFrm2)) != INVALID_FEATURE_INDEX)
			matches.push_back(FeatureMatch(iFtr1, iFtr2));
	}
}

void SequenceSet::MatchIndividualTracks(const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const TrackMatchList &trkMatches)
{
	const TrackIndexList &iTrksIdvToCmn1 = m_mapIdvTrkToCmnTrk[iSeq1], &iTrksIdvToCmn2 = m_mapIdvTrkToCmnTrk[iSeq2];
	TrackIndex iTrkIdv1, iTrkIdv2, iTrkCmn1, iTrkCmn2;
	std::vector<bool> marks;

	const TrackIndex nMatches = TrackIndex(trkMatches.size());
	for(TrackIndex i = 0; i < nMatches; ++i)
	{
		iTrkIdv1 = trkMatches[i].GetIndex1();		iTrkCmn1 = iTrksIdvToCmn1[iTrkIdv1];
		iTrkIdv2 = trkMatches[i].GetIndex2();		iTrkCmn2 = iTrksIdvToCmn2[iTrkIdv2];
		if(iTrkCmn1 == INVALID_TRACK_INDEX && iTrkCmn2 == INVALID_TRACK_INDEX)
			PushBackCommonTrack(iSeq1, iTrkIdv1, iSeq2, iTrkIdv2);
		else if(iTrkCmn1 != INVALID_TRACK_INDEX && iTrkCmn2 == INVALID_TRACK_INDEX && !AreCommonTrackAndIndividualTrackOverlappingInFrames(iTrkCmn1, iSeq2, iTrkIdv2, marks))
			MatchCommonTrackAndIndividualTrack(iTrkCmn1, iSeq2, iTrkIdv2);
		else if(iTrkCmn1 == INVALID_TRACK_INDEX && iTrkCmn2 != INVALID_TRACK_INDEX && !AreCommonTrackAndIndividualTrackOverlappingInFrames(iTrkCmn2, iSeq1, iTrkIdv1, marks))
			MatchCommonTrackAndIndividualTrack(iTrkCmn2, iSeq1, iTrkIdv1);
		else if(iTrkCmn1 != INVALID_TRACK_INDEX && iTrkCmn2 != INVALID_TRACK_INDEX && iTrkCmn1 != iTrkCmn2 && !AreCommonTracksOverlappingInFrames(iTrkCmn1, iTrkCmn2, marks))
			MatchCommonTracks(iTrkCmn1, iTrkCmn2);
	}
}

void SequenceSet::FinishMatchingIndividualTracks(const float errSqTh)
{
	// Remove null common tracks
	const TrackIndex nTrksCmnOri = GetCommonTracksNumber();
	TrackIndex iTrkCmnOri, iTrkCmnNew;
	SequenceIndex iSeq;
	TrackIndex iTrkIdv;
	for(iTrkCmnOri = 0; iTrkCmnOri < nTrksCmnOri && !m_mapCmnTrkToIdvTrk[iTrkCmnOri].empty(); ++iTrkCmnOri);
	for(iTrkCmnNew = iTrkCmnOri; iTrkCmnOri < nTrksCmnOri; ++iTrkCmnOri)
	{
		const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmnOri];
		const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
		if(nCrsps == 0)
			continue;
		m_mapCmnTrkToIdvTrk[iTrkCmnNew] = iSeqTrksIdv;
		for(SequenceIndex i = 0; i < nCrsps; ++i)
		{
			iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
			m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] = iTrkCmnNew;
		}
		++iTrkCmnNew;
	}
	const TrackIndex nTrksCmnNew = iTrkCmnNew;
	m_mapCmnTrkToIdvTrk.resize(nTrksCmnNew);

	// Merge individual tracks
	SequenceIndex i1, i2, i, j;
	TrackIndex iTrkIdv1, iTrkIdv2;
	Point3DEstimatorData data1, data2;
	Point3DEstimator Xestor;
	Xestor.m_ransacMinNumIters = 0;
	Point3D X;
	std::vector<ushort> inliers;
	for(iTrkCmnNew = 0; iTrkCmnNew < nTrksCmnNew; ++iTrkCmnNew)
	{
		SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmnNew];
		const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
		for(i1 = 0, i2 = 1; i2 < nCrsps; i1 = i2, ++i2)
		{
			iSeqTrksIdv[i1].Get(iSeq, iTrkIdv1);
			while(i2 < nCrsps && iSeqTrksIdv[i2].GetSequenceIndex() == iSeq) ++i2;
			if(i2 == i1 + 1)
				continue;
			Sequence &seq = *m_pSeqs[iSeq];
			Xestor.m_ransacErrorThreshold = errSqTh * seq.GetIntrinsicMatrix().one_over_fxy();
			seq.GetPoint3DEstimatorData(iTrkIdv1, data1, true);
			for(i = i1 + 1; i < i2; ++i)
			{
				iTrkIdv2 = iSeqTrksIdv[i].GetTrackIndex();
				seq.GetPoint3DEstimatorData(iTrkIdv2, data2, true);
				data1.PushBack(data2);
				Xestor.RunRansac(data1, X, inliers);
				if(ushort(inliers.size()) == data1.Size())
				{
					seq.MatchTracks(iTrkIdv1, iTrkIdv2);
					seq.SetPoint(iTrkIdv1, X);
				}
				else
					data1.Resize(data1.Size() - data2.Size());
				iSeqTrksIdv[i].Invalidate();
				m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv2] = INVALID_TRACK_INDEX;
				seq.MarkTrackIndividual(iTrkIdv2);
			}
		}
		for(i = j = 0; i < nCrsps; ++i)
		{
			if(iSeqTrksIdv[i].IsValid())
				iSeqTrksIdv[j++] = iSeqTrksIdv[i];
		}
		iSeqTrksIdv.resize(j);
	}

	// Remove null individual tracks
	TrackIndex iTrkIdvOri, iTrkIdvNew, iTrkCmn;
	const SequenceIndex nSeqs = GetSequencesNumber();
	for(iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		Sequence &seq = *m_pSeqs[iSeq];
		const SequenceTrackIndex iSeqTrkIdvQuery(iSeq, 0);
		const TrackIndex nTrksIdvOri = seq.GetTracksNumber();
		TrackIndexList &iTrksIdvOriToCmn = m_mapIdvTrkToCmnTrk[iSeq];
		for(iTrkIdvOri = iTrkIdvNew = 0; iTrkIdvOri < nTrksIdvOri; ++iTrkIdvOri)
		{
			if(seq.GetTrackLength(iTrkIdvOri) == 0)
				continue;
			if((iTrkCmn = iTrksIdvOriToCmn[iTrkIdvOri]) != INVALID_TRACK_INDEX)
			{
				SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
				std::lower_bound(iSeqTrksIdv.begin(), iSeqTrksIdv.end(), iSeqTrkIdvQuery)->SetTrackIndex(iTrkIdvNew);
			}
			iTrksIdvOriToCmn[iTrkIdvNew++] = iTrkCmn;
		}
		iTrksIdvOriToCmn.resize(iTrkIdvNew);
		seq.RemoveBrokenTracks();
		seq.RemoveNullMeasurements();
	}
}

void SequenceSet::MatchCommonTrackAndIndividualTrack(const TrackIndex &iTrkCmn, const SequenceIndex &iSeq, const TrackIndex &iTrkIdv)
{
#if _DEBUG
	assert(m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] == INVALID_TRACK_INDEX);
#endif
	m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] = iTrkCmn;
	const SequenceTrackIndex iSeqTrkIdvInsert(iSeq, iTrkIdv);
	SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
	iSeqTrksIdv.insert(std::lower_bound(iSeqTrksIdv.begin(), iSeqTrksIdv.end(), iSeqTrkIdvInsert), iSeqTrkIdvInsert);
	m_pSeqs[iSeq]->MarkTrackCommon(iTrkIdv);
}

void SequenceSet::MatchCommonTracks(const TrackIndex &iTrkCmn1, const TrackIndex &iTrkCmn2)
{
#if _DEBUG
	std::vector<bool> marks;
	assert(iTrkCmn1 != iTrkCmn2 && !AreCommonTracksOverlappingInFrames(iTrkCmn1, iTrkCmn2, marks));
#endif
	SequenceTrackIndexList &iSeqTrksIdv1 = m_mapCmnTrkToIdvTrk[iTrkCmn1];	const SequenceIndex nCrsps1 = SequenceIndex(iSeqTrksIdv1.size());
	SequenceTrackIndexList &iSeqTrksIdv2 = m_mapCmnTrkToIdvTrk[iTrkCmn2];	const SequenceIndex nCrsps2 = SequenceIndex(iSeqTrksIdv2.size());
	iSeqTrksIdv1.insert(iSeqTrksIdv1.end(), iSeqTrksIdv2.begin(), iSeqTrksIdv2.end());
	std::inplace_merge(iSeqTrksIdv1.begin(), iSeqTrksIdv1.begin() + nCrsps1, iSeqTrksIdv1.end());
	for(SequenceIndex i = 0; i < nCrsps2; ++i)
		m_mapIdvTrkToCmnTrk[iSeqTrksIdv2[i].GetSequenceIndex()][iSeqTrksIdv2[i].GetTrackIndex()] = iTrkCmn1;
	iSeqTrksIdv2.clear();
//#if _DEBUG
//	const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv1.size());
//	for(SequenceIndex i = 1; i < nCrsps; ++i)
//		assert(iSeqTrksIdv1[i].GetSequenceIndex() != iSeqTrksIdv1[i - 1].GetSequenceIndex());
//#endif
}

void SequenceSet::MatchFrameFeatures(const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matches)
{
	const TrackIndexList &iTrksIdvToCmn1 = m_mapIdvTrkToCmnTrk[iSeq1], &iTrksIdvToCmn2 = m_mapIdvTrkToCmnTrk[iSeq2];
	TrackIndex iTrkIdv1, iTrkIdv2, iTrkCmn1, iTrkCmn2;
	std::vector<bool> marks;

	const TrackIndex *iTrksIdv1 = GetSequence(iSeq1).GetFrameTrackIndexes(iFrm1), *iTrksIdv2 = GetSequence(iSeq2).GetFrameTrackIndexes(iFrm2);
	const ushort nMatches = ushort(matches.size());
	for(ushort i = 0; i < nMatches; ++i)
	{
		iTrkIdv1 = iTrksIdv1[matches[i].GetIndex1()];	iTrkCmn1 = iTrksIdvToCmn1[iTrkIdv1];
		iTrkIdv2 = iTrksIdv2[matches[i].GetIndex2()];	iTrkCmn2 = iTrksIdvToCmn2[iTrkIdv2];
		if(iTrkCmn1 == INVALID_TRACK_INDEX && iTrkCmn2 == INVALID_TRACK_INDEX)
			PushBackCommonTrack(iSeq1, iTrkIdv1, iSeq2, iTrkIdv2);
		else if(iTrkCmn1 != INVALID_TRACK_INDEX && iTrkCmn2 == INVALID_TRACK_INDEX && !AreCommonTrackAndIndividualTrackOverlappingInFrames(iTrkCmn1, iSeq2, iTrkIdv2, marks))
			MatchCommonTrackAndIndividualTrack(iTrkCmn1, iSeq2, iTrkIdv2);
		else if(iTrkCmn1 == INVALID_TRACK_INDEX && iTrkCmn2 != INVALID_TRACK_INDEX && !AreCommonTrackAndIndividualTrackOverlappingInFrames(iTrkCmn2, iSeq1, iTrkIdv1, marks))
			MatchCommonTrackAndIndividualTrack(iTrkCmn2, iSeq1, iTrkIdv1);
		else if(iTrkCmn1 != INVALID_TRACK_INDEX && iTrkCmn2 != INVALID_TRACK_INDEX && iTrkCmn1 != iTrkCmn2 && !AreCommonTracksOverlappingInFrames(iTrkCmn1, iTrkCmn2, marks))
			MatchCommonTracks(iTrkCmn1, iTrkCmn2);
	}
}

void SequenceSet::ConcatenateSequences(Sequence &seqCat, SequenceFrameIndexList &iSeqFrms, TrackIndexList &iTrksCmn, TrackIndexList &iTrksCmnToCat, 
									   SequenceTrackIndexList &iSeqTrksIdv, std::vector<TrackIndexList> &iTrksIdvToCatList, SequenceMeasurementIndexList &iSeqMeas, 
									   const bool copyTag, const bool copyDesc, const bool copyClr) const
{
	// Step1: collect frames
	iSeqFrms.resize(0);
	const SequenceIndex nSeqs = GetSequencesNumber();
	SequenceIndex iSeq;
	FrameIndex iFrm;
	for(iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		const FrameIndex nFrms = m_pSeqs[iSeq]->GetFramesNumber();
		for(iFrm = 0; iFrm < nFrms; ++iFrm)
			iSeqFrms.push_back(SequenceFrameIndex(iSeq, iFrm));
	}
	const FrameIndex nFrmsCat = FrameIndex(iSeqFrms.size());

	std::vector<std::string> imgFileNamesCat(nFrmsCat);
	for(FrameIndex iFrmCat = 0; iFrmCat < nFrmsCat; ++iFrmCat)
	{
		iSeqFrms[iFrmCat].Get(iSeq, iFrm);
		imgFileNamesCat[iFrmCat] = GetSequence(iSeq).GetImageFileName(iFrm);
	}
	seqCat.SetDirectory(GetDirectory());
	if(copyTag)
		seqCat.m_tag.Set(imgFileNamesCat, m_pSeqs[0]->GetSequenceName());

	//std::vector<std::vector<bool> > meaMarksList(nSeqs);
	//for(iSeq = 0; iSeq < nSeqs; ++iSeq)
	//	meaMarksList[iSeq].assign(GetSequence(iSeq).GetMeasurementsNumber(), false);

	// Step2: collect inlier common tracks, create reverse track index list iTrksCmnToCat count inlier common track measurements
	iTrksCmn.resize(0);
	const TrackIndex nTrksCmn = GetCommonTracksNumber();
	TrackIndex iTrkCmn, iTrkIdv, iTrkCat = 0;
	MeasurementIndex nMeasCat = 0;
	iTrksCmnToCat.assign(nTrksCmn, INVALID_TRACK_INDEX);
	for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
		const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
		FrameIndex nCrspsInlier = 0;
		for(SequenceIndex i = 0; i < nCrsps; ++i)
		{
			iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
			const Sequence &seq = GetSequence(iSeq);
			if(!(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER))
				continue;
			const MeasurementIndexList &iMeas = seq.GetTrackMeasurementIndexList(iTrkIdv);
			const FrameIndex nCrspsIdv = FrameIndex(iMeas.size());
			for(FrameIndex j = 0; j < nCrspsIdv; ++j)
			{
				if(!(seq.GetMeasurementState(iMeas[j]) & FLAG_MEASUREMENT_STATE_OUTLIER))
					++nCrspsInlier;
			}
		}
		if(nCrspsInlier < 2)
			continue;
		iTrksCmn.push_back(iTrkCmn);
		iTrksCmnToCat[iTrkCmn] = iTrkCat++;
		nMeasCat += nCrspsInlier;

		//for(SequenceIndex i = 0; i < nCrsps; ++i)
		//{
		//	iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
		//	const Sequence &seq = GetSequence(iSeq);
		//	if(!(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER))
		//		continue;
		//	const MeasurementIndexList &iMeas = seq.GetTrackMeasurementIndexList(iTrkIdv);
		//	const FrameIndex nCrspsIdv = FrameIndex(iMeas.size());
		//	for(FrameIndex j = 0; j < nCrspsIdv; ++j)
		//	{
		//		if(!(seq.GetMeasurementState(iMeas[j]) & FLAG_MEASUREMENT_STATE_OUTLIER))
		//			meaMarksList[iSeq][iMeas[j]] = true;
		//	}
		//}
	}
	const TrackIndex nTrksCatCmn = TrackIndex(iTrksCmn.size());

	// Step3: collect inlier individual tracks, create reverse track index lists iTrksIdvToCatList and count inlier individual track measurements
	iSeqTrksIdv.resize(0);
	iTrksIdvToCatList.resize(nSeqs);
	for(iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		const Sequence &seq = GetSequence(iSeq);
		const TrackIndex nTrksIdv = seq.GetTracksNumber();
		TrackIndexList &iTrksIdvToCat = iTrksIdvToCatList[iSeq];
		iTrksIdvToCat.assign(nTrksIdv, INVALID_TRACK_INDEX);
		for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
		{
			if((seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON) || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER))
				continue;
			const MeasurementIndexList &iMeasIdv = seq.GetTrackMeasurementIndexList(iTrkIdv);
			const FrameIndex nCrspsIdv = FrameIndex(iMeasIdv.size());
			if(nCrspsIdv < 2)
				continue;
			FrameIndex nCrspsInlier = 0;
			for(FeatureIndex i = 0; i < nCrspsIdv; ++i)
			{
				if(!(seq.GetMeasurementState(iMeasIdv[i]) & FLAG_MEASUREMENT_STATE_OUTLIER))
					++nCrspsInlier;
			}
			if(nCrspsInlier < 2)
				continue;
			iSeqTrksIdv.push_back(SequenceTrackIndex(iSeq, iTrkIdv));
			iTrksIdvToCat[iTrkIdv] = iTrkCat++;
			nMeasCat += nCrspsInlier;

			//for(FeatureIndex i = 0; i < nCrspsIdv; ++i)
			//{
			//	if(!(seq.GetMeasurementState(iMeasIdv[i]) & FLAG_MEASUREMENT_STATE_OUTLIER))
			//		meaMarksList[iSeq][iMeasIdv[i]] = true;
			//}
		}
	}
	const TrackIndex nTrksCat = iTrkCat;

	// Step4: copy tags, cameras and points
	seqCat.m_K = GetSequence(0).m_K;
	seqCat.m_Kr = GetSequence(0).m_Kr;
	seqCat.m_intrinsicType = m_intrinsicType;
	seqCat.Resize(nFrmsCat, nTrksCat, nMeasCat);
	FrameIndex iFrmCat;
	for(iFrmCat = 0; iFrmCat < nFrmsCat; ++iFrmCat)
	{
		iSeqFrms[iFrmCat].Get(iSeq, iFrm);
		const Sequence &seq = GetSequence(iSeq);
		seqCat.m_Cs[iFrmCat] = seq.m_Cs[iFrm];
		if(m_intrinsicType == Sequence::INTRINSIC_VARIABLE)
			seqCat.m_Krs[iFrmCat] = seq.m_Krs[iFrm];
		seqCat.m_frmStates[iFrmCat] = seq.m_frmStates[iFrm];
	}
#if DESCRIPTOR_TRACK
	if(copyDesc)
		seqCat.SetDescriptorsNumber(nTrksCat);
	else
		seqCat.ClearDescriptors();
#endif
	if(copyClr)
		seqCat.SetTrackColorsNumber(nTrksCat);
	else
		seqCat.SetTrackColorsNumber(0);
	const TrackState trkStateCmn = (FLAG_TRACK_STATE_SOLVED | FLAG_TRACK_STATE_INLIER | FLAG_TRACK_STATE_COMMON);
	for(iTrkCat = 0; iTrkCat < nTrksCatCmn; ++iTrkCat)
	{
		iTrkCmn = iTrksCmn[iTrkCat];
		const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
		const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
		for(SequenceIndex i = 0; i < nCrsps; ++i)
		{
			iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
			const Sequence &seq = GetSequence(iSeq);
			if(!(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) || (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				continue;
			seqCat.SetPoint(iTrkCat, seq.GetPoint(iTrkIdv));
			seqCat.SetTrackState(iTrkCat, trkStateCmn);
#if DESCRIPTOR_TRACK
			if(copyDesc)
				seqCat.SetDescriptor(iTrkCat, seq.GetDescriptor(iTrkIdv));
#endif
			if(copyClr)
				seqCat.SetTrackColor(iTrkCat, seq.GetTrackColor(iTrkIdv));
			break;
		}
	}
	for(TrackIndex iTrkCatIdv = 0; iTrkCat < nTrksCat; ++iTrkCat, ++iTrkCatIdv)
	{
		iSeqTrksIdv[iTrkCatIdv].Get(iSeq, iTrkIdv);
		const Sequence &seq = GetSequence(iSeq);
		seqCat.SetPoint(iTrkCat, seq.GetPoint(iTrkIdv));
		seqCat.SetTrackState(iTrkCat, seq.GetTrackState(iTrkIdv));
#if DESCRIPTOR_TRACK
		if(copyDesc)
			seqCat.SetDescriptor(iTrkCat, seq.GetDescriptor(iTrkIdv));
#endif
		if(copyClr)
			seqCat.SetTrackColor(iTrkCat, seq.GetTrackColor(iTrkIdv));
	}

	// Step5: copy inlier measurements and create correspondence maps

#if _DEBUG
	for(iSeq = 1; iSeq < nSeqs; ++iSeq)
		assert(m_pSeqs[iSeq]->AreMeasurementsNormalized() == m_pSeqs[iSeq - 1]->AreMeasurementsNormalized());
#endif
	seqCat.m_measNormalized = m_pSeqs[0]->AreMeasurementsNormalized();

#if DESCRIPTOR_TRACK == 0
	if(copyDesc)
		seqCat.m_descs.Resize(nMeasCat);
	else
		seqCat.m_descs.Clear();
#endif

	iSeqMeas.resize(0);
	MeasurementIndex iMeaCat = 0;
	seqCat.m_mapFrmToMea[0] = 0;
	for(iFrmCat = 0; iFrmCat < nFrmsCat; ++iFrmCat)
	{
		iSeqFrms[iFrmCat].Get(iSeq, iFrm);
		const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
		const TrackIndexList &iTrksIdvToCat = iTrksIdvToCatList[iSeq];
		const Sequence &seq = GetSequence(iSeq);
		const MeasurementIndex iMea1 = seq.m_mapFrmToMea[iFrm], iMea2 = seq.m_mapFrmToMea[iFrm + 1];
		for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrkIdv = seq.m_mapMeaToTrk[iMea];
			if(iTrkIdv == INVALID_TRACK_INDEX || !(seq.m_trkStates[iTrkIdv] & FLAG_TRACK_STATE_INLIER) || (seq.m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			else if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX)
				iTrkCat = iTrksCmnToCat[iTrkCmn];
			else
				iTrkCat = iTrksIdvToCat[iTrkIdv];
			if(iTrkCat == INVALID_TRACK_INDEX)
				continue;
			//if(!meaMarksList[iSeq][iMea])
			//	printf("iSeq%d, iFrm%d, iTrkIdv%d, iTrkCmn%d, iTrkCat%d, iMea%d, iMeaCat%d\n", iSeq, iFrm, iTrkIdv, iTrkCmn, iTrkCat, iMea, iMeaCat);
			iSeqMeas.push_back(SequenceMeasurementIndex(iSeq, iMea));
			seqCat.m_xs[iMeaCat] = seq.m_xs[iMea];
#if DESCRIPTOR_TRACK == 0
			if(copyDesc)
				seqCat.m_descs[iMeaCat] = seq.m_descs[iMea];
#endif
			seqCat.m_mapTrkToMea[iTrkCat].push_back(iMeaCat);
			seqCat.m_mapMeaToFrm[iMeaCat] = iFrmCat;
			seqCat.m_mapMeaToTrk[iMeaCat] = iTrkCat;
			seqCat.m_meaStates[iMeaCat] = seq.m_meaStates[iMea];
			if(seq.m_trkStates[iTrkIdv] & FLAG_TRACK_STATE_COMMON_OUTLIER)
				seqCat.m_meaStates[iMeaCat] |= FLAG_MEASUREMENT_STATE_OUTLIER;
			++iMeaCat;
		}
		seqCat.m_mapFrmToMea[iFrmCat + 1] = iMeaCat;
	}
#if _DEBUG
	assert(iMeaCat == nMeasCat);
#endif
	if(m_intrinsicType == Sequence::INTRINSIC_CONSTANT)
		seqCat.SetIntrinsicRectification(m_pSeqs[0]->GetIntrinsicRectification());
	//seqCat.AssertConsistency();
}

void SequenceSet::SplitSequences(const Sequence &seqCat, const FrameIndex &nFrmsPerSeq)
{
	m_dir = seqCat.GetDirectory();
	m_intrinsicType = seqCat.GetIntrinsicType();

	const FrameIndex nFrmsCatTotal = seqCat.GetFramesNumberTotal(), nFrmsCat = seqCat.GetFramesNumber();
	const SequenceIndex nSeqs = (nFrmsCatTotal + nFrmsPerSeq - 1) / nFrmsPerSeq;
	CreateSequences(nSeqs);

	const TrackIndex nTrksCat = seqCat.GetTracksNumber();
	FrameIndex iFrmCat, iFrmCat1, iFrmCat2 = 0, iFrmDst;
	TrackIndex iTrkCat, iTrkDst;
	MeasurementIndex iMeaCat, iMeaDst;
	TrackIndexList iTrksDstToCat;
	std::vector<TrackIndexList> iTrksListCatToDst(nSeqs);
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		Sequence &seqDst = *m_pSeqs[iSeq];

		iFrmCat1 = iFrmCat2;
		iFrmCat2 += nFrmsPerSeq;
		if(iFrmCat2 > nFrmsCatTotal)
			iFrmCat2 = nFrmsCatTotal;

		TrackIndexList &iTrksCatToDst = iTrksListCatToDst[iSeq];
		iTrksCatToDst.assign(nTrksCat, INVALID_TRACK_INDEX);
		iTrksDstToCat.resize(0);
		const MeasurementIndex iMeaCat1 = iFrmCat1 <= nFrmsCat ? seqCat.GetFrameFirstMeasurementIndex(iFrmCat1) : seqCat.GetFrameFirstMeasurementIndex(nFrmsCat);
		const MeasurementIndex iMeaCat2 = iFrmCat2 <= nFrmsCat ? seqCat.GetFrameFirstMeasurementIndex(iFrmCat2) : seqCat.GetFrameFirstMeasurementIndex(nFrmsCat);
		for(iMeaCat = iMeaCat1; iMeaCat < iMeaCat2; ++iMeaCat)
		{
			iTrkCat = seqCat.GetMeasurementTrackIndex(iMeaCat);
			if(iTrksCatToDst[iTrkCat] != INVALID_TRACK_INDEX)
				continue;
			iTrksCatToDst[iTrkCat] = TrackIndex(iTrksDstToCat.size());
			iTrksDstToCat.push_back(iTrkCat);
		}

		const FrameIndex nFrmsDst = (iFrmCat2 <= nFrmsCat ? iFrmCat2 : nFrmsCat) - (iFrmCat1 <= nFrmsCat ? iFrmCat1 : nFrmsCat);
		const TrackIndex nTrksDst = TrackIndex(iTrksDstToCat.size());
		const MeasurementIndex nMeasDst = iMeaCat2 - iMeaCat1;
		const bool copyCam = seqCat.GetCamerasNumber() == nFrmsCat;
		const bool copyPt = seqCat.GetPointsNumber() == nTrksCat;
		const bool copyDesc = seqCat.GetDescriptorsNumber() != 0;
		seqDst.Resize(nFrmsDst, nTrksDst, nMeasDst, copyCam, copyPt, copyDesc);

		seqCat.m_tag.GetSubSequence(iFrmCat1, iFrmCat2, seqDst.m_tag);
		seqDst.m_intrinsicType = seqCat.m_intrinsicType;
		seqDst.m_K = seqCat.m_K;
		seqDst.m_Kr = seqCat.m_Kr;
		seqDst.m_measNormalized = seqCat.m_measNormalized;
		for(iFrmDst = 0, iFrmCat = iFrmCat1; iFrmDst < nFrmsDst; ++iFrmDst, ++iFrmCat)
		{
			if(copyCam)
			{
				seqDst.m_Cs[iFrmDst] = seqCat.m_Cs[iFrmCat];
				seqDst.m_Ps[iFrmDst] = seqCat.m_Ps[iFrmCat];
				if(seqCat.m_intrinsicType == Sequence::INTRINSIC_VARIABLE)
					seqDst.m_Krs[iFrmDst] = seqCat.m_Krs[iFrmCat];
			}
			seqDst.m_mapFrmToMea[iFrmDst] = seqCat.m_mapFrmToMea[iFrmCat] - iMeaCat1;
			seqDst.m_frmStates[iFrmDst] = seqCat.m_frmStates[iFrmCat];
		}
		seqDst.m_mapFrmToMea[iFrmDst] = nMeasDst;
		for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst)
		{
			iTrkCat = iTrksDstToCat[iTrkDst];
			if(copyPt)
				seqDst.m_Xs[iTrkDst] = seqCat.m_Xs[iTrkCat];
			seqDst.m_trkStates[iTrkDst] = seqCat.m_trkStates[iTrkCat];
			seqDst.m_trkClrs[iTrkDst] = seqCat.m_trkClrs[iTrkCat];
		}
		for(iMeaDst = 0, iMeaCat = iMeaCat1; iMeaDst < nMeasDst; ++iMeaDst, ++iMeaCat)
		{
			seqDst.m_xs[iMeaDst] = seqCat.m_xs[iMeaCat];
			if(copyDesc)
				seqDst.m_descs[iMeaDst] = seqCat.m_descs[iMeaCat];
			iTrkCat = seqCat.m_mapMeaToTrk[iMeaCat];
			iTrkDst = iTrksCatToDst[iTrkCat];
			seqDst.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
			iFrmCat = seqCat.m_mapMeaToFrm[iMeaCat];
			iFrmDst = iFrmCat - iFrmCat1;
			seqDst.m_mapMeaToFrm[iMeaDst] = iFrmDst;
			seqDst.m_mapMeaToTrk[iMeaDst] = iTrkDst;
			seqDst.m_meaStates[iMeaDst] = seqCat.m_meaStates[iMeaCat];
		}
#if _DEBUG
		seqDst.AssertConsistency();
#endif
		m_mapIdvTrkToCmnTrk[iSeq].assign(nTrksDst, INVALID_TRACK_INDEX);
	}

	// Collect common tracks
	InitializeCommonPoints();
	SequenceTrackIndexList iSeqTrksDst;
	for(iTrkCat = 0; iTrkCat < nTrksCat; ++iTrkCat)
	{
		iSeqTrksDst.resize(0);
		for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
		{
			if((iTrkDst = iTrksListCatToDst[iSeq][iTrkCat]) != INVALID_TRACK_INDEX)
				iSeqTrksDst.push_back(SequenceTrackIndex(iSeq, iTrkDst));
		}
		if(iSeqTrksDst.size() >= 2)
			PushBackCommonTrack(iSeqTrksDst);
	}
	//SetCommonPointsNumber(GetCommonTracksNumber());
	//for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	//	MarkSequenceRegistered(iSeq);
}