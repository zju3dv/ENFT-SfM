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
#include "FeatureTracker.h"

using namespace ENFT_SfM;

void FeatureTracker::RemoveOutlierMatchesFlowVoting(const FrameIndex &iFrm1, const FrameIndex &iFrm2, FeatureMatchList &matchesSift,
        std::vector<FeatureEnftMatch> &matchesEnft) {
    if(m_flowDifferenceTh == 0 || m_flowPositiveVotingTh == 0 || m_flowPositiveVotingRatioTh == 0)
        return;
    m_bucket.ClearBins();

    const AlignedVector<Point2D> &ftrs1 = m_ftrsBuffer[m_bufferManager.GetDataBufferIndex(iFrm1)];
    const AlignedVector<Point2D> &ftrs2 = m_ftrsBuffer[m_bufferManager.GetDataBufferIndex(iFrm2)];
    const ushort nMatchesSift = ushort(matchesSift.size()), nMatchesEnft = ushort(matchesEnft.size()), nMatches = nMatchesSift + nMatchesEnft;
    m_flows.resize(nMatches);

    ushort i, j, iFtr1, iFtr2;
    for(i = 0; i < nMatchesSift; ++i) {
        matchesSift[i].Get(iFtr1, iFtr2);
        const Point2D &x1 = ftrs1[iFtr1], &x2 = ftrs2[iFtr2];
        LA::AmB(x2, x1, m_flows[i]);
        m_bucket.Push(x2.x(), x2.y(), i);
    }
    for(j = 0; j < nMatchesEnft; ++i, ++j) {
        const Point2D &x1 = ftrs1[matchesEnft[j].GetFeatureIndex1()], &x2 = matchesEnft[j].GetFeature2();
        LA::AmB(x2, x1, m_flows[i]);
        m_bucket.Push(x2.x(), x2.y(), i);
    }

    m_flowVotes.resize(nMatches);
    memset(m_flowVotes.data(), 0, sizeof(FlowVote) * nMatches);

    ushort xBin, yBin;
    uint iBin1, iBin2;
    const uint nBins = m_bucket.GetBinsNumber();
    const ushort nBinsX = m_bucket.GetBinSizeX(), nBinsY = m_bucket.GetBinSizeY();
    for(yBin = 1, iBin2 = uint(nBinsX); yBin < nBinsY; ++yBin)
        for(xBin = 1, ++iBin2; xBin < nBinsX; ++xBin, ++iBin2) {
            iBin1 = iBin2;
            VoteNeighboringFlows(iBin1, iBin2, m_flows);
            --iBin1;
            VoteNeighboringFlows(iBin1, iBin2, m_flows);
            iBin1 -= nBinsX;
            VoteNeighboringFlows(iBin1, iBin2, m_flows);
            ++iBin1;
            VoteNeighboringFlows(iBin1, iBin2, m_flows);
            ++iBin1;
            VoteNeighboringFlows(iBin1, iBin2, m_flows);
        }

    for(i = j = 0; i < nMatchesSift; ++i) {
        const FlowVote &vote = m_flowVotes[i];
        if(vote.GetPositiveCount() >= m_flowPositiveVotingTh || vote.GetPositiveCount() >= ushort(vote.GetTotalCount() * m_flowPositiveVotingRatioTh))
            matchesSift[j++] = matchesSift[i];
    }
    matchesSift.resize(j);
    for(i = j = 0; i < nMatchesEnft; ++i) {
        const FlowVote &vote = m_flowVotes[nMatchesSift + i];
        if(vote.GetPositiveCount() >= m_flowPositiveVotingTh || vote.GetPositiveCount() >= ushort(vote.GetTotalCount() * m_flowPositiveVotingRatioTh))
            matchesEnft[j++] = matchesEnft[i];
    }
    matchesEnft.resize(j);

#if VERBOSE_FEATURE_TRACKING/* >= 2*/
    printf("----------------------------------------------------------------\n");
    printf("  Frame (%d, %d): %d - %d = %d matches\n", iFrm1, iFrm2, nMatches, nMatches - matchesSift.size() - matchesEnft.size(),
           matchesSift.size() + matchesEnft.size());
#endif
}

void FeatureTracker::VoteNeighboringFlows(const uint &iBin1, const uint &iBin2, const std::vector<Point2D> &flows) {
    ushort idx1, idx2;
    if(iBin1 == iBin2) {
        const std::vector<ushort> &idxs = m_bucket.GetBin(iBin1);
        const ushort N = ushort(idxs.size());
        for(ushort i1 = 0; i1 < N; ++i1) {
            idx1 = idxs[i1];
            const Point2D &flow1 = flows[idx1];
            for(ushort i2 = i1 + 1; i2 < N; ++i2) {
                idx2 = idxs[i2];
                const Point2D &flow2 = flows[idx2];
                if(fabs(flow1.x() - flow2.x()) + fabs(flow1.y() - flow2.y()) < m_flowDifferenceTh) {
                    m_flowVotes[idx1].VotePositive();
                    m_flowVotes[idx2].VotePositive();
                } else {
                    m_flowVotes[idx1].VoteNegative();
                    m_flowVotes[idx2].VoteNegative();
                }
            }
        }
    } else {
        const std::vector<ushort> &idxs1 = m_bucket.GetBin(iBin1), &idxs2 = m_bucket.GetBin(iBin2);
        const ushort N1 = ushort(idxs1.size()), N2 = ushort(idxs2.size());
        for(ushort i1 = 0; i1 < N1; ++i1) {
            idx1 = idxs1[i1];
            const Point2D &flow1 = flows[idx1];
            for(ushort i2 = 0; i2 < N2; ++i2) {
                idx2 = idxs2[i2];
                const Point2D &flow2 = flows[idx2];
                if(fabs(flow1.x() - flow2.x()) + fabs(flow1.y() - flow2.y()) < m_flowDifferenceTh) {
                    m_flowVotes[idx1].VotePositive();
                    m_flowVotes[idx2].VotePositive();
                } else {
                    m_flowVotes[idx1].VoteNegative();
                    m_flowVotes[idx2].VoteNegative();
                }
            }
        }
    }
}