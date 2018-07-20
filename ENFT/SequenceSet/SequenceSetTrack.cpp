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

//const Point3D& SequenceSet::GetCommonPoint(const TrackIndex &iTrkCmn) const
//{
//	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
//	SequenceIndex i;
//	for(i = 0; !(m_seqStates[iSeqTrksIdv[i].GetSequenceIndex()] & FLAG_SEQUENCE_STATE_REGISTRED) || (GetSequence(iSeqTrksIdv[i].GetSequenceIndex()).GetTrackState(iSeqTrksIdv[i].GetTrackIndex()) & FLAG_TRACK_STATE_COMMON_OUTLIER); ++i);
//	return GetSequence(iSeqTrksIdv[i].GetSequenceIndex()).GetPoint(iSeqTrksIdv[i].GetTrackIndex());
//}
//
//const Point3D& SequenceSet::GetCommonPoint(const TrackIndex &iTrkCmn, SequenceIndex &i) const
//{
//	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
//	for(i = 0; !(m_seqStates[iSeqTrksIdv[i].GetSequenceIndex()] & FLAG_SEQUENCE_STATE_REGISTRED) || (GetSequence(iSeqTrksIdv[i].GetSequenceIndex()).GetTrackState(iSeqTrksIdv[i].GetTrackIndex()) & FLAG_TRACK_STATE_COMMON_OUTLIER); ++i);
//	return GetSequence(iSeqTrksIdv[i].GetSequenceIndex()).GetPoint(iSeqTrksIdv[i].GetTrackIndex());
//}

void SequenceSet::GetCommonPoint3DEstimatorDataInlier(const TrackIndex &iTrkCmn, Point3DEstimatorData &data, Point3DEstimatorData &dataTmp) const
{
	SequenceIndex iSeq;
	TrackIndex iTrkIdv;

	data.Resize(0);
	const SequenceTrackIndexList &iSeqTrksIdvSrc = m_mapCmnTrkToIdvTrk[iTrkCmn];
	const SequenceIndex nCrspsSeq = SequenceIndex(iSeqTrksIdvSrc.size());
	for(SequenceIndex i = 0; i < nCrspsSeq; ++i)
	{
		iSeqTrksIdvSrc[i].Get(iSeq, iTrkIdv);
		const Sequence &seq = GetSequence(iSeq);
		if(!(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED) || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) || (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
			continue;
		seq.GetPoint3DEstimatorDataInlier(iTrkIdv, dataTmp, true);
		if(seq.GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT)
			dataTmp.ScaleWeights(sqrt(seq.GetIntrinsicMatrix().fxy()) * seq.GetIntrinsicRectification().f());
		else
			dataTmp.ScaleWeights(sqrt(seq.GetIntrinsicMatrix().fxy()));
		data.PushBack(dataTmp);
	}
}

SequenceIndex SequenceSet::CountCommonTrackRegisteredSequenceIndividualTracks(const TrackIndex &iTrkCmn) const
{
	SequenceIndex cnt = 0;
	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
	const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
	for(SequenceIndex i = 0; i < nCrsps; ++i)
	{
		if(m_seqStates[iSeqTrksIdv[i].GetSequenceIndex()] & FLAG_SEQUENCE_STATE_REGISTRED)
			++cnt;
	}
	return cnt;
}

SequenceIndex SequenceSet::CountCommonTrackInlierIndividualTracks(const TrackIndex &iTrkCmn) const
{
	SequenceIndex cnt = 0;
	SequenceIndex iSeq;
	TrackIndex iTrkIdv;
	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
	const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
	for(SequenceIndex i = 0; i < nCrsps; ++i)
	{
		iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
		const Sequence &seq = GetSequence(iSeq);
		if((seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
		&& (!(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED) || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER)))
			++cnt;
	}
	return cnt;
}

FrameIndex SequenceSet::CountCommonTrackInlierMeasurements(const TrackIndex &iTrkCmn) const
{
	FrameIndex cnt = 0;
	SequenceIndex iSeq;
	TrackIndex iTrkIdv;
	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
	const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
	for(SequenceIndex i = 0; i < nCrsps; ++i)
	{
		iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
		const Sequence &seq = GetSequence(iSeq);
		if((seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
		&& (!(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED) || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER)))
			cnt += seq.CountTrackInlierMeasurements(iTrkIdv);
	}
	return cnt;
}

TrackIndex SequenceSet::SearchCommonTrackForIndividualTrack(const TrackIndex &iTrkCmn, const SequenceIndex &iSeq) const
{
	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
	if(iSeqTrksIdv.front().GetSequenceIndex() > iSeq || iSeqTrksIdv.back().GetSequenceIndex() < iSeq)
		return INVALID_TRACK_INDEX;
	const SequenceTrackIndex iSeqTrkIdvQuery(iSeq, 0);
	const SequenceTrackIndex iSeqTrkIdvResult = *std::lower_bound(iSeqTrksIdv.begin(), iSeqTrksIdv.end(), iSeqTrkIdvQuery);
	if(iSeqTrkIdvResult.GetSequenceIndex() == iSeq)
		return iSeqTrkIdvResult.GetTrackIndex();
	else
		return INVALID_TRACK_INDEX;
}

void SequenceSet::SearchCommonTrackForIndividualTracks(const TrackIndex &iTrkCmn, const SequenceIndex &iSeq, TrackIndexList &iTrksIdv) const
{
	iTrksIdv.resize(0);
	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
	if(iSeqTrksIdv.front().GetSequenceIndex() > iSeq || iSeqTrksIdv.back().GetSequenceIndex() < iSeq)
		return;
	const SequenceTrackIndex iSeqTrkIdvQuery(iSeq, 0);
	const SequenceIndex i1 = SequenceIndex(std::lower_bound(iSeqTrksIdv.begin(), iSeqTrksIdv.end(), iSeqTrkIdvQuery) - iSeqTrksIdv.begin());
	const SequenceIndex i2 = SequenceIndex(iSeqTrksIdv.size());
	if(i1 == i2)
		return;
	for(SequenceIndex i = i1; i < i2 && iSeqTrksIdv[i].GetSequenceIndex() == iSeq; ++i)
		iTrksIdv.push_back(iSeqTrksIdv[i].GetTrackIndex());
}

bool SequenceSet::AreCommonTrackAndIndividualTrackOverlappingInFrames(const TrackIndex &iTrkCmn, const SequenceIndex &iSeq, const TrackIndex &iTrkIdv, std::vector<bool> &marks) const
{
	//const TrackIndex iTrkIdvCmn = SearchCommonTrackForIndividualTrack(iTrkCmn, iSeq);
	//return iTrkIdvCmn != INVALID_TRACK_INDEX && GetSequence(iSeq).AreTracksOverlappingInFrames(iTrkIdvCmn, iTrkIdv, marks);
	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
	if(iSeqTrksIdv.front().GetSequenceIndex() > iSeq || iSeqTrksIdv.back().GetSequenceIndex() < iSeq)
		return false;
	const Sequence &seq = GetSequence(iSeq);
	const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
	for(SequenceIndex i = SequenceIndex(std::lower_bound(iSeqTrksIdv.begin(), iSeqTrksIdv.end(), SequenceTrackIndex(iSeq, 0)) - iSeqTrksIdv.begin()); 
		i < nCrsps && iSeqTrksIdv[i].GetSequenceIndex() == iSeq; ++i)
	{
		if(seq.AreTracksOverlappingInFrames(iSeqTrksIdv[i].GetTrackIndex(), iTrkIdv, marks))
			return true;
	}
	return false;
}

bool SequenceSet::AreCommonTracksOverlappingInFrames(const TrackIndex &iTrkCmn1, const TrackIndex &iTrkCmn2, std::vector<bool> &marks) const
{
	SequenceIndex iSeq;
	TrackIndex iTrkIdv2;
	TrackIndexList iTrksIdv1;
	const SequenceTrackIndexList &iSeqTrksIdv2 = m_mapCmnTrkToIdvTrk[iTrkCmn2];
	const SequenceIndex nCrsps2 = SequenceIndex(iSeqTrksIdv2.size());
	for(SequenceIndex i = 0; i < nCrsps2; ++i)
	{
		iSeqTrksIdv2[i].Get(iSeq, iTrkIdv2);
		SearchCommonTrackForIndividualTracks(iTrkCmn1, iSeq, iTrksIdv1);
		if(iTrksIdv1.empty())
			continue;
		const Sequence &seq = GetSequence(iSeq);
		const SequenceIndex nTrksIdv1 = SequenceIndex(iTrksIdv1.size());
		for(SequenceIndex j = 0; j < nTrksIdv1; ++j)
			if(seq.AreTracksOverlappingInFrames(iTrksIdv1[j], iTrkIdv2, marks))
				return true;
	}
	return false;
}

//void SequenceSet::SetCommonPoint(const TrackIndex &iTrkCmn, const Point3D &X)
//{
//	SequenceIndex iSeq;
//	TrackIndex iTrkIdv;
//	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
//	const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
//	for(SequenceIndex i = 0; i < nCrsps; ++i)
//	{
//		iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
//		Sequence &seq = *m_pSeqs[iSeq];
//		if((m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED) && (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
//		&& !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
//			seq.SetPoint(iTrkIdv, X);
//	}
//}

//void SequenceSet::SetCommonTrackPlaneIndex(const TrackIndex &iTrkCmn, const ubyte &iPlane)
//{
//	SequenceIndex iSeq;
//	TrackIndex iTrkIdv;
//	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
//	const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
//	for(SequenceIndex i = 0; i < nCrsps; ++i)
//	{
//		iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
//		Sequence &seq = *m_pSeqs[iSeq];
//		if((m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED) && (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
//		&& !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
//			seq.SetTrackPlaneIndex(iTrkIdv, iPlane);
//	}
//}

void SequenceSet::UnmarkCommonTrackRegistered(const TrackIndex &iTrkCmn)
{
	m_cmnTrkStates[iTrkCmn] &= ~FLAG_COMMON_TRACK_STATE_REGISTERED;
	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
	const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
	for(SequenceIndex i = 0; i < nCrsps; ++i)
		m_pSeqs[iSeqTrksIdv[i].GetSequenceIndex()]->MarkTrackCommonInlier(iSeqTrksIdv[i].GetTrackIndex());
}

void SequenceSet::PushBackCommonTrack(const SequenceIndex &iSeq, const TrackIndex &iTrkIdv)
{
#if _DEBUG
	assert(m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] == INVALID_TRACK_INDEX);
#endif
	const TrackIndex iTrkCmn = GetCommonTracksNumber();
	m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] = iTrkCmn;
	m_mapCmnTrkToIdvTrk.push_back(SequenceTrackIndexList(1, SequenceTrackIndex(iSeq, iTrkIdv)));
	m_pSeqs[iSeq]->MarkTrackCommon(iTrkIdv);
	m_cmnTrkStates.push_back(FLAG_COMMON_TRACK_STATE_DEFAULT);
}

void SequenceSet::PushBackCommonTrack(const SequenceIndex &iSeq1, const TrackIndex &iTrkIdv1, const SequenceIndex &iSeq2, const TrackIndex &iTrkIdv2)
{
#if _DEBUG
	assert(m_mapIdvTrkToCmnTrk[iSeq1][iTrkIdv1] == INVALID_TRACK_INDEX && m_mapIdvTrkToCmnTrk[iSeq2][iTrkIdv2] == INVALID_TRACK_INDEX);
#endif
	const TrackIndex iTrkCmn = GetCommonTracksNumber();
	m_mapIdvTrkToCmnTrk[iSeq1][iTrkIdv1] = m_mapIdvTrkToCmnTrk[iSeq2][iTrkIdv2] = iTrkCmn;
	m_mapCmnTrkToIdvTrk.push_back(SequenceTrackIndexList(2));
	m_mapCmnTrkToIdvTrk[iTrkCmn][0].Set(iSeq1, iTrkIdv1);
	m_mapCmnTrkToIdvTrk[iTrkCmn][1].Set(iSeq2, iTrkIdv2);
	m_pSeqs[iSeq1]->MarkTrackCommon(iTrkIdv1);
	m_pSeqs[iSeq2]->MarkTrackCommon(iTrkIdv2);

	m_cmnTrkStates.push_back(FLAG_COMMON_TRACK_STATE_DEFAULT);
}

void SequenceSet::PushBackCommonTrack(const SequenceTrackIndexList &iSeqTrksIdv)
{
	const TrackIndex iTrkCmn = GetCommonTracksNumber();
	m_mapCmnTrkToIdvTrk.push_back(iSeqTrksIdv);

	SequenceIndex iSeq;
	TrackIndex iTrkIdv;
	const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
	for(SequenceIndex i = 0; i < nCrsps; ++i)
	{
		iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
#if _DEBUG
		assert(m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] == INVALID_TRACK_INDEX);
#endif
		m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] = iTrkCmn;
		m_pSeqs[iSeq]->MarkTrackCommon(iTrkIdv);
	}

	m_cmnTrkStates.push_back(FLAG_COMMON_TRACK_STATE_DEFAULT);
}

void SequenceSet::RemoveIndividualTracks(const SequenceIndex &iSeq)
{
	Sequence &seq = *m_pSeqs[iSeq];
	const TrackIndex nTrksIdvOri = seq.GetTracksNumber();
	const SequenceTrackIndex iSeqTrkIdvQuery(iSeq, 0);
	TrackIndexList &iTrksIdvOriToCmn = m_mapIdvTrkToCmnTrk[iSeq];

	TrackIndex iTrkIdvOri, iTrkIdvNew, iTrkCmn;
	for(iTrkIdvOri = iTrkIdvNew = 0; iTrkIdvOri < nTrksIdvOri; ++iTrkIdvOri)
	{
		if((iTrkCmn = iTrksIdvOriToCmn[iTrkIdvOri]) == INVALID_TRACK_INDEX)
		{
			seq.BreakTrack(iTrkIdvOri);
			continue;
		}
		iTrksIdvOriToCmn[iTrkIdvNew] = iTrkCmn;
		SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
		std::lower_bound(iSeqTrksIdv.begin(), iSeqTrksIdv.end(), iSeqTrkIdvQuery)->SetTrackIndex(iTrkIdvNew);
		++iTrkIdvNew;
	}
	iTrksIdvOriToCmn.resize(iTrkIdvNew);
	seq.RemoveBrokenTracks();
	seq.RemoveNullMeasurements();
}

bool SequenceSet::ComputeCommonTrackMSE(const TrackIndex &iTrkCmn, const Point3D &Xcmn, float &MSE) const
{
	SequenceIndex iSeq, cnt;
	TrackIndex iTrkIdv;
	float SSEcmn, MSEidv;

	MSE = FLT_MAX;
	SSEcmn = 0;
	cnt = 0;

	const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
	const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
	for(SequenceIndex i = 0; i < nCrsps; ++i)
	{
		iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
		const Sequence &seq = GetSequence(iSeq);
		if(!(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED) || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
		|| (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
			continue;
		if(!seq.ComputeTrackMSE(iTrkIdv, Xcmn, MSEidv))
			return false;
		SSEcmn += MSEidv;
		++cnt;
	}
	MSE = SSEcmn / cnt;
	return true;
}

void SequenceSet::CopyCommonPointsToIndividualPoints()
{
	TrackIndex iTrkCmn, iTrkIdv;
	const SequenceIndex nSeqs = GetSequencesNumber();
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		Sequence &seq = *m_pSeqs[iSeq];
		if(!(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED))
			continue;
		const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
		const TrackIndex nTrksIdv = seq.GetTracksNumber();
		for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
		{
			if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX && (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
			&& !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				seq.SetPoint(iTrkIdv, m_XsCmn[iTrkCmn]);
		}
	}
}

void SequenceSet::CopyCommonPointsToIndividualSinglePoints()
{
	TrackIndex iTrkCmn, iTrkIdv;
	const SequenceIndex nSeqs = GetSequencesNumber();
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		Sequence &seq = *m_pSeqs[iSeq];
		if(!(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED))
			continue;
		const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
		const TrackIndex nTrksIdv = seq.GetTracksNumber();
		for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
		{
			if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX && (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
			&& !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER) && seq.GetTrackLength(iTrkIdv) == 1)
				seq.SetPoint(iTrkIdv, m_XsCmn[iTrkCmn]);
		}
	}
}

void SequenceSet::CopyIndividualPointsToCommonPoints()
{
	SequenceIndex iSeq;
	TrackIndex iTrkCmn, iTrkIdv;
	float MSE, MSEMin;
	const TrackIndex nTrksCmn = GetCommonTracksNumber();
	for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		if(!(m_cmnTrkStates[iTrkCmn] & FLAG_COMMON_TRACK_STATE_REGISTERED))
			continue;
		Point3D &Xcmn = m_XsCmn[iTrkCmn];
		//MSEMin = FLT_MAX;
		ComputeCommonTrackMSE(iTrkCmn, Xcmn, MSEMin);
		const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
		const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
		for(SequenceIndex i = 0; i < nCrsps; ++i)
		{
			iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
			const Sequence &seq = GetSequence(iSeq);
			if(!(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED) || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
			|| (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				continue;
			ComputeCommonTrackMSE(iTrkCmn, seq.GetPoint(iTrkIdv), MSE);
			if(MSE < MSEMin)
			{
				MSEMin = MSE;
				Xcmn = seq.GetPoint(iTrkIdv);
			}
		}
	}
}

void SequenceSet::AverageIndividualPointsToCommonPoints()
{
	Point3D Xcmn, Xidv;
	SequenceIndex iSeq, cnt;
	TrackIndex iTrkCmn, iTrkIdv;
	const TrackIndex nTrksCmn = GetCommonTracksNumber();
	for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		if(!(m_cmnTrkStates[iTrkCmn] & FLAG_COMMON_TRACK_STATE_REGISTERED))
			continue;
		Xcmn.SetZero();
		cnt = 0;
		const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
		const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
		for(SequenceIndex i = 0; i < nCrsps; ++i)
		{
			iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
			const Sequence &seq = GetSequence(iSeq);
			if(!(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED) || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)
			|| (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER))
				continue;
			Xidv = seq.GetPoint(iTrkIdv);

			SequenceIndex _i, _iSeq;
			FrameIndex _j;
			TrackIndex _iTrkIdv;
			for(_i = 0; _i < nCrsps; ++_i)
			{
				iSeqTrksIdv[_i].Get(_iSeq, _iTrkIdv);
				if(_iSeq == iSeq)
					continue;
				const Sequence &_seq = GetSequence(_iSeq);
				const MeasurementIndexList &_iMeas = _seq.GetTrackMeasurementIndexList(_iTrkIdv);
				const FrameIndex _N = FrameIndex(_iMeas.size());
				for(_j = 0; _j < _N; ++_j)
				{
					const FrameIndex _iFrm = _seq.GetMeasurementFrameIndex(_iMeas[_j]);
					if(_seq.GetCamera(_iFrm).ComputeDepth(Xidv) < 0.0f)
						break;
				}
				if(_j < _N)
					break;
			}
			if(_i < nCrsps)
				continue;

			Xcmn += Xidv;
			++cnt;
		}
		Xcmn *= (1.0f / cnt);
		Xcmn.reserve() = 1.0f;
		SetCommonPoint(iTrkCmn, Xcmn);
	}
}

void SequenceSet::RemoveBrokenTracks(const SequenceIndex &iSeq)
{
	TrackIndex iTrkIdvOri, iTrkIdvNew, iTrkCmn;
	Sequence &seq = *m_pSeqs[iSeq];
	const SequenceTrackIndex iSeqTrkIdvQuery(iSeq, 0);
	const TrackIndex nTrksIdvOri = seq.GetTracksNumber();
	TrackIndexList &iTrksIdvOriToCmn = m_mapIdvTrkToCmnTrk[iSeq];
	for(iTrkIdvOri = iTrkIdvNew = 0; iTrkIdvOri < nTrksIdvOri; ++iTrkIdvOri)
	{
		if((iTrkCmn = iTrksIdvOriToCmn[iTrkIdvOri]) != INVALID_TRACK_INDEX)
		{
			SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
			if(seq.GetTrackLength(iTrkIdvOri) == 0)
				iSeqTrksIdv.erase(std::lower_bound(iSeqTrksIdv.begin(), iSeqTrksIdv.end(), iSeqTrkIdvQuery));
			else
				std::lower_bound(iSeqTrksIdv.begin(), iSeqTrksIdv.end(), iSeqTrkIdvQuery)->SetTrackIndex(iTrkIdvNew);
		}
		if(seq.GetTrackLength(iTrkIdvOri) > 0)
			iTrksIdvOriToCmn[iTrkIdvNew++] = iTrkCmn;
	}
	iTrksIdvOriToCmn.resize(iTrkIdvNew);
	seq.RemoveBrokenTracks();
	seq.RemoveNullMeasurements();
}