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

#ifndef _MATCHING_MATRIX_H_
#define _MATCHING_MATRIX_H_

#include "Sequence/Sequence.h"
#include "Utility/SparseMatrix.h"
#include "Utility/Table.h"

//#define MATCHING_MATRIX_INITIALIZATION_METHOD		MATCHING_MATRIX_INITIALIZATION_METHOD_NAIVE
#define MATCHING_MATRIX_INITIALIZATION_METHOD		MATCHING_MATRIX_INITIALIZATION_METHOD_TF
#define MATCHING_MATRIX_INITIALIZATION_METHOD_NAIVE	0
#define MATCHING_MATRIX_INITIALIZATION_METHOD_TF	1
#define MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF 2

class MatchingMatrix
{

public:

	inline const Sequence& GetSequence1() const { return *m_pSeq1; }
	inline const Sequence& GetSequence2() const { return *m_pSeq2; }
	//inline const float& GetInitialConfidence(const FrameIndex &iFrm1, const FrameIndex &iFrm2) const { return *m_dataInit.Get(iFrm1, iFrm2); }
	inline TrackIndex GetCandidateTrackMatchesNumber() const { return TrackIndex(m_trkMatchVotes.size()); }

	void Initialize(const Sequence &seq, const std::vector<TrackIndexList> &iTrkClusters);
	void Initialize(const Sequence &seq, const SparseMatrix<float> &dataInit);
	void Initialize(const Sequence &seq1, const Sequence &seq2, const std::vector<TrackIndexListPair> &iTrkClusters, const bool consecutiveSeqs);
	void Initialize(const Sequence &seq1, const Sequence &seq2, const SparseMatrix<float> &dataInit);
	bool ExtractInitialSeed(FrameIndex &iFrm1Seed, FrameIndex &iFrm2Seed, FeatureMatchList &matchesSeed, TrackIndexList &idxs, float &confidenceSeed);
	bool ExtractUpdatedSeed(FrameIndex &iFrm1Seed, FrameIndex &iFrm2Seed, FeatureMatchList &matchesSeed, TrackIndexList &idxs);
	void VoteCandidateTrackMatches(const TrackIndexList &idxs, const std::vector<ushort> &inliers, const std::vector<ushort> &outliers);
	void AddCandidateTrackMatches_UpdateConfidences(const FrameIndex &iFrm1Seed, const FrameIndex &iFrm2Seed, const FeatureMatchList &matchesSeed);
	void GetTrackMatches(TrackMatchList &trkMatches) const;
	void ExtractInlierMatches(TrackMatchList &trkMatches, const ushort positiveVotingTh, const float positiveVotingRatioTh);

#if _DEBUG
	void AssertTrackMatchable(const TrackIndex &iTrk1, const TrackIndex &iTrk2);
#endif

private:

	class CandidateMatchedTrack
	{
	public:
		CandidateMatchedTrack() {}
		CandidateMatchedTrack(const TrackIndex &iTrk, const TrackIndex &idx) : m_iTrk(iTrk), m_idx(idx) {}
		inline void Get(TrackIndex &iTrk, TrackIndex &idx) const { iTrk = m_iTrk; idx = m_idx; }
		inline const TrackIndex& GetTrackIndex() const { return m_iTrk; }
		inline const TrackIndex& GetTrackMatchVoteIndex() const { return m_idx; }
	protected:
		TrackIndex m_iTrk, m_idx;
	};
	std::vector<std::vector<CandidateMatchedTrack> > m_candidateMatchedTrksList;

	class TrackMatchVote
	{
	public:
		TrackMatchVote() : m_cntPositive(1), m_cntNegative(0) {}
		inline void VotePositive() { ++m_cntPositive; }
		inline void VoteNegative() { ++m_cntNegative; }
		inline const ushort& GetPositiveCount() const { return m_cntPositive; }
		inline const ushort& GetNegativeCount() const { return m_cntNegative; }
		inline void Invalidate() { m_cntPositive = 0; }
		inline bool IsValid() const { return m_cntPositive != 0; }
		inline bool IsInvalid() const { return m_cntPositive == 0; }
	protected:
		ushort m_cntPositive, m_cntNegative;
	};
	std::vector<TrackMatchVote> m_trkMatchVotes;

	class CandidateTrackMatch : public TrackMatch
	{
	public:
		inline CandidateTrackMatch() : TrackMatch() {}
		inline CandidateTrackMatch(const TrackIndex &iTrk1, const TrackIndex &iTrk2, const ushort &score) : TrackMatch(iTrk1, iTrk2), m_score(score) {}
		inline bool operator < (const CandidateTrackMatch &trkMatch) const { return m_score > trkMatch.m_score; }
	protected:
		ushort m_score;
	};
	std::vector<CandidateTrackMatch> m_candidateTrkMatches;

	const Sequence *m_pSeq1, *m_pSeq2;
	SparseMatrix<float> m_dataInit;
	SparseMatrix<ushort> m_dataUpd;
	Table<ubyte, FrameIndex> m_dataMark;
	std::vector<bool> m_marks;

};

#endif