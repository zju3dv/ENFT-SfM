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
#include "MatchingMatrix.h"
#include "Utility/Utility.h"

void MatchingMatrix::Initialize(const Sequence &seq, const std::vector<TrackIndexList> &iTrkClusters)
{
	m_pSeq1 = m_pSeq2 = &seq;

	const TrackIndex nTrks = seq.GetTracksNumber();
	//m_candidateMatchedTrksList.assign(nTrks, std::vector<CandidateTrack>());
	m_candidateMatchedTrksList.resize(nTrks);
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
		m_candidateMatchedTrksList[iTrk].resize(0);
	m_trkMatchVotes.resize(0);

	const FrameIndex nFrms = seq.GetFramesNumber();
	m_dataInit.Clear();					m_dataInit.Resize(nFrms, nFrms);
	m_dataUpd.Clear();					m_dataUpd.Resize(nFrms, nFrms);
	m_dataMark.Resize(nFrms, nFrms);	m_dataMark.SetZero();

#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
	std::vector<float> idfSqSums(nFrms, 0);
#endif

	const FrameIndex N = seq.GetFramesNumber();
	TrackIndex i1, i2, iTrk1, iTrk2;
	FrameIndex j1, j2, iFrm1, iFrm2, iFrmMin, iFrmMax;
	float *pConfidence;
	const uint nClusters = uint(iTrkClusters.size());
	for(uint i = 0; i < nClusters; ++i)
	{
		const TrackIndexList &iTrks = iTrkClusters[i];
		const TrackIndex nTrks = TrackIndex(iTrks.size());
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_NAIVE
		const float idf = 1.0f;
#else
		FrameIndex Ni = 0;
		for(i1 = 0; i1 < nTrks; ++i1)
			Ni += seq.GetTrackLength(iTrks[i1]);
		if(Ni > N)
			continue;
		const float idf = log(float(N) / Ni);
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
		const float idfSq = idf * idf;
#endif
#endif
		for(i1 = 0; i1 < nTrks; ++i1)
		for(i2 = i1 + 1; i2 < nTrks; ++i2)
		{
			iTrk1 = iTrks[i1];
			iTrk2 = iTrks[i2];
			if(seq.AreTracksOverlappingInFrames(iTrk1, iTrk2, m_marks))
				continue;
			const MeasurementIndexList &iMeas1 = seq.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq.GetTrackMeasurementIndexList(iTrk2);
			const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
			for(j1 = 0; j1 < nCrsps1; ++j1)
			{
				iFrm1 = seq.GetMeasurementFrameIndex(iMeas1[j1]);
				for(j2 = 0; j2 < nCrsps2; ++j2)
				{
					iFrm2 = m_pSeq2->GetMeasurementFrameIndex(iMeas2[j2]);
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
					idfSqSums[iFrm1] += idfSq;
					idfSqSums[iFrm2] += idfSq;
#else
					if(iFrm1 < iFrm2)
					{
						iFrmMin = iFrm1;
						iFrmMax = iFrm2;
					}
					else
					{
						iFrmMin = iFrm2;
						iFrmMax = iFrm1;
					}
					pConfidence = m_dataInit.Get(iFrmMin, iFrmMax);
					if(pConfidence)
						(*pConfidence) += idf;
					else
						m_dataInit.Insert(iFrmMin, iFrmMax, idf);
#endif				
				}
			}
		}
	}
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
	std::vector<float> &norms = idfSqSums;
	for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1)
		norms[iFrm1] = seq.GetFrameFeaturesNumber(iFrm1) / sqrt(idfSqSums[iFrm1]);
	for(uint i = 0; i < nClusters; ++i)
	{
		const TrackIndexList &iTrks = iTrkClusters[i];
		const TrackIndex nTrks = TrackIndex(iTrks.size());
		for(i1 = 0; i1 < nTrks; ++i1)
		for(i2 = i1 + 1; i2 < nTrks; ++i2)
		{
			iTrk1 = iTrks[i1];
			iTrk2 = iTrks[i2];
			if(seq.AreTracksOverlappingInFrames(iTrk1, iTrk2, m_marks))
				continue;
			const MeasurementIndexList &iMeas1 = seq.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq.GetTrackMeasurementIndexList(iTrk2);
			const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
			for(j1 = 0; j1 < nCrsps1; ++j1)
			{
				iFrm1 = seq.GetMeasurementFrameIndex(iMeas1[j1]);
				for(j2 = 0; j2 < nCrsps2; ++j2)
				{
					iFrm2 = seq.GetMeasurementFrameIndex(iMeas2[j2]);
					if(iFrm1 < iFrm2)
					{
						iFrmMin = iFrm1;
						iFrmMax = iFrm2;
					}
					else
					{
						iFrmMin = iFrm2;
						iFrmMax = iFrm1;
					}
					pConfidence = m_dataInit.Get(iFrmMin, iFrmMax);
					if(pConfidence)
						(*pConfidence) += norms[iFrm1] * norms[iFrm2];
					else
						m_dataInit.Insert(iFrmMin, iFrmMax, norms[iFrm1] * norms[iFrm2]);
				}
			}
		}
	}
#endif
}

void MatchingMatrix::Initialize(const Sequence &seq, const SparseMatrix<float> &dataInit)
{
	m_pSeq1 = m_pSeq2 = &seq;

	const TrackIndex nTrks = seq.GetTracksNumber();
	//m_candidateMatchedTrksList.assign(nTrks, std::vector<CandidateTrack>());
	m_candidateMatchedTrksList.resize(nTrks);
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
		m_candidateMatchedTrksList[iTrk].resize(0);
	m_trkMatchVotes.resize(0);

	const FrameIndex nFrms = seq.GetFramesNumber();
	m_dataInit.Clear();					m_dataInit.Resize(nFrms, nFrms);
	m_dataUpd.Clear();					m_dataUpd.Resize(nFrms, nFrms);
	m_dataMark.Resize(nFrms, nFrms);	m_dataMark.SetZero();

	FrameIndex iFrm1, iFrm2, iFrmMin, iFrmMax;
	for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1)
	{
		const FullIndexValueList<float> &elements = dataInit.GetRowData(iFrm1);
		const FrameIndex nElements = FrameIndex(elements.size());
		for(FrameIndex i = 0; i < nElements; ++i)
		{
			iFrm2 = elements[i].GetFullIndex();
			if(iFrm1 < iFrm2)
			{
				iFrmMin = iFrm1;
				iFrmMax = iFrm2;
			}
			else
			{
				iFrmMin = iFrm2;
				iFrmMax = iFrm1;
			}
			if(!m_dataInit.Get(iFrmMin, iFrmMax))
				m_dataInit.Insert(iFrmMin, iFrmMax, elements[i].GetValue());
		}
	}
}

void MatchingMatrix::Initialize(const Sequence &seq1, const Sequence &seq2, const std::vector<TrackIndexListPair> &iTrkClusters, const bool consecutiveSeqs)
{
	m_pSeq1 = &seq1;
	m_pSeq2 = &seq2;

	const TrackIndex nTrks1 = seq1.GetTracksNumber(), nTrks2 = seq2.GetTracksNumber();
	//m_candidateMatchedTrksList.assign(nTrks1, std::vector<CandidateTrack>());
	m_candidateMatchedTrksList.resize(nTrks1);
	for(TrackIndex iTrk1 = 0; iTrk1 < nTrks1; ++iTrk1)
		m_candidateMatchedTrksList[iTrk1].resize(0);
	m_trkMatchVotes.resize(0);

	const FrameIndex nFrms1 = seq1.GetFramesNumber(), nFrms2 = seq2.GetFramesNumber();
	m_dataInit.Clear();						m_dataInit.Resize(nFrms1, nFrms2);	
	m_dataUpd.Clear();						m_dataUpd.Resize(nFrms1, nFrms2);		
	m_dataMark.Resize(nFrms1, nFrms2);		m_dataMark.SetZero();

#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
	std::vector<float> idfSqSums1(nFrms1, 0), idfSqSums2(nFrms2, 0);
#endif

	const FrameIndex N = seq1.GetFramesNumber() + seq2.GetFramesNumber();
	TrackIndex i1, i2, iTrk1, iTrk2;
	FrameIndex j1, j2, iFrm1, iFrm2;
	float *pConfidence;
	const uint nClusters = uint(iTrkClusters.size());
	for(uint i = 0; i < nClusters; ++i)
	{
		const TrackIndexList &iTrks1 = iTrkClusters[i].first, &iTrks2 = iTrkClusters[i].second;
		const TrackIndex nTrks1 = TrackIndex(iTrks1.size()), nTrks2 = TrackIndex(iTrks2.size());
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_NAIVE
		const float idf = 1.0f;
#else
		FrameIndex Ni = 0;
		for(i1 = 0; i1 < nTrks1; ++i1)
			Ni += seq1.GetTrackLength(iTrks1[i1]);
		for(i2 = 0; i2 < nTrks2; ++i2)
			Ni += seq2.GetTrackLength(iTrks2[i2]);
		if(Ni > N)
			continue;
		const float idf = log(float(N) / Ni);
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
		const float idfSq = idf * idf;
#endif
#endif
		for(i1 = 0; i1 < nTrks1; ++i1)
		for(i2 = 0; i2 < nTrks2; ++i2)
		{
			iTrk1 = iTrks1[i1];
			iTrk2 = iTrks2[i2];
			const MeasurementIndexList &iMeas1 = seq1.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq2.GetTrackMeasurementIndexList(iTrk2);
			const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
			for(j1 = 0; j1 < nCrsps1; ++j1)
			{
				iFrm1 = seq1.GetMeasurementFrameIndex(iMeas1[j1]);
				for(j2 = 0; j2 < nCrsps2; ++j2)
				{
					iFrm2 = seq2.GetMeasurementFrameIndex(iMeas2[j2]);
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
					idfSqSums1[iFrm1] += idfSq;
					idfSqSums2[iFrm2] += idfSq;
#else
					pConfidence = m_dataInit.Get(iFrm1, iFrm2);
					if(pConfidence)
						(*pConfidence) += idf;
					else
						m_dataInit.Insert(iFrm1, iFrm2, idf);
#endif
				}
			}
		}
	}

#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
	std::vector<float> &norms1 = idfSqSums1, &norms2 = idfSqSums2;
	for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
		norms1[iFrm1] = seq1.GetFrameFeaturesNumber(iFrm1) / sqrt(idfSqSums1[iFrm1]);
	for(iFrm2 = 0; iFrm2 < nFrms2; ++iFrm2)
		norms2[iFrm2] = seq2.GetFrameFeaturesNumber(iFrm2) / sqrt(idfSqSums2[iFrm2]);
	for(uint i = 0; i < nClusters; ++i)
	{
		const TrackIndexList &iTrks1 = iTrkClusters[i].first, &iTrks2 = iTrkClusters[i].second;
		const TrackIndex nTrks1 = TrackIndex(iTrks1.size()), nTrks2 = TrackIndex(iTrks2.size());
		for(i1 = 0; i1 < nTrks1; ++i1)
		for(i2 = 0; i2 < nTrks2; ++i2)
		{
			iTrk1 = iTrks1[i1];
			iTrk2 = iTrks2[i2];
			const MeasurementIndexList &iMeas1 = seq1.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq2.GetTrackMeasurementIndexList(iTrk2);
			const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
			for(j1 = 0; j1 < nCrsps1; ++j1)
			{
				iFrm1 = seq1.GetMeasurementFrameIndex(iMeas1[j1]);
				for(j2 = 0; j2 < nCrsps2; ++j2)
				{
					iFrm2 = seq2.GetMeasurementFrameIndex(iMeas2[j2]);
					pConfidence = m_dataInit.Get(iFrm1, iFrm2);
					if(pConfidence)
						(*pConfidence) += norms1[iFrm1] * norms2[iFrm2];
					else
						m_dataInit.Insert(iFrm1, iFrm2, norms1[iFrm1] * norms2[iFrm2]);
				}
			}
		}
	}
#endif

	if(!consecutiveSeqs)
		return;
	float confidence, confidenceMax = 0.0f;
	for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
	{
		const FullIndexValueList<float> &elements = m_dataInit.GetRowData(iFrm1);
		const FrameIndex nElements = FrameIndex(elements.size());
		for(FrameIndex i = 0; i < nElements; ++i)
		{
			iFrm2 = elements[i].GetFullIndex();
			if((confidence = elements[i].GetValue()) > confidenceMax)
				confidenceMax = confidence;
		}
	}
	iFrm1 = nFrms1 - 1;
	iFrm2 = 0;
	//confidenceMax += 1.0f;
	pConfidence = m_dataInit.Get(iFrm1, iFrm2);
	if(pConfidence)
		(*pConfidence) = confidenceMax;
	else
		m_dataInit.Insert(iFrm1, iFrm2, confidenceMax);
}

void MatchingMatrix::Initialize(const Sequence &seq1, const Sequence &seq2, const SparseMatrix<float> &dataInit)
{
	m_pSeq1 = &seq1;
	m_pSeq2 = &seq2;

	const TrackIndex nTrks1 = seq1.GetTracksNumber(), nTrks2 = seq2.GetTracksNumber();
	//m_candidateMatchedTrksList.assign(nTrks1, std::vector<CandidateTrack>());
	m_candidateMatchedTrksList.resize(nTrks1);
	for(TrackIndex iTrk1 = 0; iTrk1 < nTrks1; ++iTrk1)
		m_candidateMatchedTrksList[iTrk1].resize(0);
	m_trkMatchVotes.resize(0);

	const FrameIndex nFrms1 = seq1.GetFramesNumber(), nFrms2 = seq2.GetFramesNumber();
	m_dataInit.Clear();						m_dataInit.Resize(nFrms1, nFrms2);	
	m_dataUpd.Clear();						m_dataUpd.Resize(nFrms1, nFrms2);		
	m_dataMark.Resize(nFrms1, nFrms2);		m_dataMark.SetZero();

	FrameIndex iFrm1, iFrm2;
	for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
	{
		const FullIndexValueList<float> &elements = dataInit.GetRowData(iFrm1);
		const FrameIndex nElements = FrameIndex(elements.size());
		for(FrameIndex i = 0; i < nElements; ++i)
		{
			iFrm2 = elements[i].GetFullIndex();
			m_dataInit.Insert(iFrm1, iFrm2, elements[i].GetValue());
		}
	}
}

bool MatchingMatrix::ExtractInitialSeed(FrameIndex &iFrm1Seed, FrameIndex &iFrm2Seed, FeatureMatchList &matchesSeed, TrackIndexList &idxs, float &confidenceSeed)
{
	FrameIndex iFrm1, iFrm2;
	float confidence;
	iFrm1Seed = iFrm2Seed = INVALID_FRAME_INDEX;
	confidenceSeed = 0;
	const FrameIndex nFrms1 = m_pSeq1->GetFramesNumber();
	for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
	{
		const FullIndexValueList<float> &elements = m_dataInit.GetRowData(iFrm1);
		const FrameIndex nElements = FrameIndex(elements.size());
		for(FrameIndex i = 0; i < nElements; ++i)
		{
			iFrm2 = elements[i].GetFullIndex();
			confidence = elements[i].GetValue();
			if(!m_dataMark[iFrm1][iFrm2] && confidence > confidenceSeed)
			{
				iFrm1Seed = iFrm1;
				iFrm2Seed = iFrm2;
				confidenceSeed = confidence;
			}
		}
	}
	if(confidenceSeed == 0)
		return false;
	m_dataMark[iFrm1Seed][iFrm2Seed] = 1;

	const TrackIndex *iTrks1Seed = m_pSeq1->GetFrameTrackIndexes(iFrm1Seed);
	const FeatureIndex nFtrs1Seed = m_pSeq1->GetFrameFeaturesNumber(iFrm1Seed);
	FeatureIndex iFtr1Seed, iFtr2Seed;
	TrackIndex iTrk1Seed, iTrk2Seed;
	matchesSeed.resize(0);
	idxs.resize(0);
	for(iFtr1Seed = 0; iFtr1Seed < nFtrs1Seed; ++iFtr1Seed)
	{
		if((iTrk1Seed = iTrks1Seed[iFtr1Seed]) == INVALID_TRACK_INDEX)
			continue;
		const std::vector<CandidateMatchedTrack> &candidateMatchedTrks = m_candidateMatchedTrksList[iTrk1Seed];
		const TrackIndex nCandidateMatchedTrks = TrackIndex(candidateMatchedTrks.size());
		for(TrackIndex i = 0; i < nCandidateMatchedTrks; ++i)
		{
			iTrk2Seed = candidateMatchedTrks[i].GetTrackIndex();
			if((iFtr2Seed = m_pSeq2->SearchTrackForFrameFeatureIndex(iTrk2Seed, iFrm2Seed)) == INVALID_FEATURE_INDEX)
				continue;
			matchesSeed.push_back(FeatureMatch(iFtr1Seed, iFtr2Seed));
			idxs.push_back(candidateMatchedTrks[i].GetTrackMatchVoteIndex());
		}
	}
	return true;
}

bool MatchingMatrix::ExtractUpdatedSeed(FrameIndex &iFrm1Seed, FrameIndex &iFrm2Seed, FeatureMatchList &matchesSeed, TrackIndexList &idxs)
{
	FrameIndex iFrm1, iFrm2;
	ushort confidence, confidenceSeed = 0;
	const FrameIndex nFrms1 = m_pSeq1->GetFramesNumber();
	for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
	{
		const FullIndexValueList<ushort> &elements = m_dataUpd.GetRowData(iFrm1);
		const FrameIndex nElements = FrameIndex(elements.size());
		for(FrameIndex i = 0; i < nElements; ++i)
		{
			iFrm2 = elements[i].GetFullIndex();
			confidence = elements[i].GetValue();
			if(!m_dataMark[iFrm1][iFrm2] && confidence > confidenceSeed)
			{
				iFrm1Seed = iFrm1;
				iFrm2Seed = iFrm2;
				confidenceSeed = confidence;
			}
		}
	}
	if(confidenceSeed == 0)
		return false;
	m_dataMark[iFrm1Seed][iFrm2Seed] = 1;

	const TrackIndex *iTrks1Seed = m_pSeq1->GetFrameTrackIndexes(iFrm1Seed);
	const FeatureIndex nFtrs1Seed = m_pSeq1->GetFrameFeaturesNumber(iFrm1Seed);
	FeatureIndex iFtr1Seed, iFtr2Seed;
	TrackIndex iTrk1Seed, iTrk2Seed;
	matchesSeed.resize(0);
	idxs.resize(0);
	for(iFtr1Seed = 0; iFtr1Seed < nFtrs1Seed; ++iFtr1Seed)
	{
		if((iTrk1Seed = iTrks1Seed[iFtr1Seed]) == INVALID_TRACK_INDEX)
			continue;
		const std::vector<CandidateMatchedTrack> &candidateMatchedTrks = m_candidateMatchedTrksList[iTrk1Seed];
		const TrackIndex nCandidateMatchedTrks = TrackIndex(candidateMatchedTrks.size());
		for(TrackIndex i = 0; i < nCandidateMatchedTrks; ++i)
		{
			iTrk2Seed = candidateMatchedTrks[i].GetTrackIndex();
			if((iFtr2Seed = m_pSeq2->SearchTrackForFrameFeatureIndex(iTrk2Seed, iFrm2Seed)) == INVALID_FEATURE_INDEX)
				continue;
			matchesSeed.push_back(FeatureMatch(iFtr1Seed, iFtr2Seed));
			idxs.push_back(candidateMatchedTrks[i].GetTrackMatchVoteIndex());
		}
	}
	return true;
}

void MatchingMatrix::VoteCandidateTrackMatches(const TrackIndexList &idxs, const std::vector<ushort> &inliers, const std::vector<ushort> &outliers)
{
	const ushort nInliers = ushort(inliers.size());
	for(ushort i = 0; i < nInliers; ++i)
		m_trkMatchVotes[idxs[inliers[i]]].VotePositive();
	const ushort nOutliers = ushort(outliers.size());
	for(ushort i = 0; i < nOutliers; ++i)
		m_trkMatchVotes[idxs[outliers[i]]].VoteNegative();
}

void MatchingMatrix::AddCandidateTrackMatches_UpdateConfidences(const FrameIndex &iFrm1Seed, const FrameIndex &iFrm2Seed, const FeatureMatchList &matchesSeed)
{
	FrameIndex j1, j2, iFrm1, iFrm2, iFrmMin, iFrmMax;
	TrackIndex iTrk1Seed, iTrk2Seed, idx = TrackIndex(m_trkMatchVotes.size());
	ushort *pConfidence;

	const TrackIndex *iTrks1Seed = m_pSeq1->GetFrameTrackIndexes(iFrm1Seed), *iTrks2Seed = m_pSeq2->GetFrameTrackIndexes(iFrm2Seed);
	const ushort nMatchesSeed = ushort(matchesSeed.size());
	if(m_pSeq1 == m_pSeq2)
	{
		for(ushort i = 0; i < nMatchesSeed; ++i, ++idx)
		{
			iTrk1Seed = iTrks1Seed[matchesSeed[i].GetIndex1()];
			iTrk2Seed = iTrks2Seed[matchesSeed[i].GetIndex2()];
#if _DEBUG
			AssertTrackMatchable(iTrk1Seed, iTrk2Seed);
#endif
			m_candidateMatchedTrksList[iTrk1Seed].push_back(CandidateMatchedTrack(iTrk2Seed, idx));
			m_candidateMatchedTrksList[iTrk2Seed].push_back(CandidateMatchedTrack(iTrk1Seed, idx));
			m_trkMatchVotes.push_back(TrackMatchVote());

			const MeasurementIndexList &iMeas1 = m_pSeq1->GetTrackMeasurementIndexList(iTrk1Seed), &iMeas2 = m_pSeq2->GetTrackMeasurementIndexList(iTrk2Seed);
			const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
			for(j1 = 0; j1 < nCrsps1; ++j1)
			{
				iFrm1 = m_pSeq1->GetMeasurementFrameIndex(iMeas1[j1]);
				for(j2 = 0; j2 < nCrsps2; ++j2)
				{
					iFrm2 = m_pSeq2->GetMeasurementFrameIndex(iMeas2[j2]);
					if(iFrm1 < iFrm2)
					{
						iFrmMin = iFrm1;
						iFrmMax = iFrm2;
					}
					else
					{
						iFrmMin = iFrm2;
						iFrmMax = iFrm1;
					}
					pConfidence = m_dataUpd.Get(iFrmMin, iFrmMax);
					if(pConfidence)
						++(*pConfidence);
					else
						m_dataUpd.Insert(iFrmMin, iFrmMax, 1);
				}
			}
		}
	}
	else
	{
		for(ushort i = 0; i < nMatchesSeed; ++i, ++idx)
		{
			iTrk1Seed = iTrks1Seed[matchesSeed[i].GetIndex1()];
			iTrk2Seed = iTrks2Seed[matchesSeed[i].GetIndex2()];
//#if _DEBUG
#if 0
			AssertTrackMatchable(iTrk1Seed, iTrk2Seed);
#endif
			m_candidateMatchedTrksList[iTrk1Seed].push_back(CandidateMatchedTrack(iTrk2Seed, idx));
			m_trkMatchVotes.push_back(TrackMatchVote());

			const MeasurementIndexList &iMeas1 = m_pSeq1->GetTrackMeasurementIndexList(iTrk1Seed), &iMeas2 = m_pSeq2->GetTrackMeasurementIndexList(iTrk2Seed);
			const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
			for(j1 = 0; j1 < nCrsps1; ++j1)
			{
				iFrm1 = m_pSeq1->GetMeasurementFrameIndex(iMeas1[j1]);
				for(j2 = 0; j2 < nCrsps2; ++j2)
				{
					iFrm2 = m_pSeq2->GetMeasurementFrameIndex(iMeas2[j2]);
					pConfidence = m_dataUpd.Get(iFrm1, iFrm2);
					if(pConfidence)
						++(*pConfidence);
					else
						m_dataUpd.Insert(iFrm1, iFrm2, 1);
				}
			}
		}
	}
}

void MatchingMatrix::GetTrackMatches(TrackMatchList &trkMatches) const
{
	trkMatches.resize(0);
	TrackIndex iTrk1, iTrk2;
	const TrackIndex nTrks1 = TrackIndex(m_pSeq1->GetTracksNumber());

	if(m_pSeq1 == m_pSeq2)
	{
		for(iTrk1 = 0; iTrk1 < nTrks1; ++iTrk1)
		{
			const std::vector<CandidateMatchedTrack> &candidateMatchedTrks = m_candidateMatchedTrksList[iTrk1];
			const TrackIndex nCandidateMatchedTrks = TrackIndex(candidateMatchedTrks.size());
			for(TrackIndex i = 0; i < nCandidateMatchedTrks; ++i)
			{
				if((iTrk2 = candidateMatchedTrks[i].GetTrackIndex()) > iTrk1)
					trkMatches.push_back(TrackMatch(iTrk1, iTrk2));
			}
		}
	}
	else
	{
		for(iTrk1 = 0; iTrk1 < nTrks1; ++iTrk1)
		{
			const std::vector<CandidateMatchedTrack> &candidateMatchedTrks = m_candidateMatchedTrksList[iTrk1];
			const TrackIndex nCandidateMatchedTrks = TrackIndex(candidateMatchedTrks.size());
			for(TrackIndex i = 0; i < nCandidateMatchedTrks; ++i)
			{
				iTrk2 = candidateMatchedTrks[i].GetTrackIndex();
				trkMatches.push_back(TrackMatch(iTrk1, iTrk2));
			}
		}
	}
}

void MatchingMatrix::ExtractInlierMatches(TrackMatchList &trkMatches, const ushort positiveVotingTh, const float positiveVotingRatioTh)
{
	TrackIndex iTrk1, iTrk2, idx;
	const TrackIndex nTrks1 = TrackIndex(m_pSeq1->GetTracksNumber());

	//trkMatches.resize(0);
	m_candidateTrkMatches.resize(0);
	if(m_pSeq1 == m_pSeq2)
	{
		for(iTrk1 = 0; iTrk1 < nTrks1; ++iTrk1)
		{
			std::vector<CandidateMatchedTrack> &candidateMatchedTrks = m_candidateMatchedTrksList[iTrk1];
			const TrackIndex nCandidateMatchedTrks = TrackIndex(candidateMatchedTrks.size());
			for(TrackIndex i = 0; i < nCandidateMatchedTrks; ++i)
			{
				candidateMatchedTrks[i].Get(iTrk2, idx);
				TrackMatchVote &vote = m_trkMatchVotes[idx];
				if(vote.IsInvalid() || vote.GetPositiveCount() < positiveVotingTh && vote.GetPositiveCount() < ushort((vote.GetPositiveCount() + vote.GetNegativeCount()) * positiveVotingRatioTh + 0.5f))
					continue;
				//trkMatches.push_back(TrackMatch(iTrk1, iTrk2));
				m_candidateTrkMatches.push_back(CandidateTrackMatch(iTrk1, iTrk2, vote.GetPositiveCount()));
				vote.Invalidate();
			}
			candidateMatchedTrks.resize(0);
		}
		m_trkMatchVotes.resize(0);
	}
	else
	{
		for(iTrk1 = 0; iTrk1 < nTrks1; ++iTrk1)
		{
			std::vector<CandidateMatchedTrack> &candidateMatchedTrks = m_candidateMatchedTrksList[iTrk1];
			const TrackIndex nCandidateMatchedTrks = TrackIndex(candidateMatchedTrks.size());
			for(TrackIndex i = 0; i < nCandidateMatchedTrks; ++i)
			{
				candidateMatchedTrks[i].Get(iTrk2, idx);
				const TrackMatchVote &vote = m_trkMatchVotes[idx];
#if _DEBUG
				IO::Assert(vote.IsValid(), "(%d, %d) already be extracted\n", iTrk1, iTrk2);
#endif
				if(vote.GetPositiveCount() < positiveVotingTh && vote.GetPositiveCount() < ushort((vote.GetPositiveCount() + vote.GetNegativeCount()) * positiveVotingRatioTh + 0.5f))
					continue;
				//trkMatches.push_back(TrackMatch(iTrk1, iTrk2));
				m_candidateTrkMatches.push_back(CandidateTrackMatch(iTrk1, iTrk2, vote.GetPositiveCount()));
#if _DEBUG
				m_trkMatchVotes[idx].Invalidate();
#endif
			}
			candidateMatchedTrks.resize(0);
		}
		m_trkMatchVotes.resize(0);
	}
	std::sort(m_candidateTrkMatches.begin(), m_candidateTrkMatches.end());
	const TrackIndex nTrkMatches = TrackIndex(m_candidateTrkMatches.size());
	trkMatches.resize(nTrkMatches);
	for(TrackIndex i = 0; i < nTrkMatches; ++i)
		trkMatches[i] = m_candidateTrkMatches[i];
}

#if _DEBUG
void MatchingMatrix::AssertTrackMatchable(const TrackIndex &iTrk1, const TrackIndex &iTrk2)
{
	const std::vector<CandidateMatchedTrack> &candidateMatchedTrks1 = m_candidateMatchedTrksList[iTrk1];
	const TrackIndex nCandidateMatchedTrks1 = TrackIndex(candidateMatchedTrks1.size());
	for(TrackIndex i = 0; i < nCandidateMatchedTrks1; ++i)
		IO::Assert(candidateMatchedTrks1[i].GetTrackIndex() != iTrk2, "(%d, %d) has already existed\n", iTrk1, iTrk2);
	const std::vector<CandidateMatchedTrack> &candidateMatchedTrks2 = m_candidateMatchedTrksList[iTrk2];
	const TrackIndex nCandidateMatchedTrks2 = TrackIndex(candidateMatchedTrks2.size());
	for(TrackIndex i = 0; i < nCandidateMatchedTrks2; ++i)
		IO::Assert(candidateMatchedTrks2[i].GetTrackIndex() != iTrk1, "(%d, %d) has already existed\n", iTrk2, iTrk1);
	//if(m_pSeq1 == m_pSeq2)
	//	IO::Assert(!m_pSeq1->AreTracksOverlappingInFrames(iTrk1, iTrk2, m_marks), "(%d, %d) overlaps\n", iTrk1, iTrk2);
}
#endif