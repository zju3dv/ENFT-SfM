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
#include "TrackMatcher.h"

using namespace ENFT_SfM;

void TrackMatcher::ClusterTracks(const Sequence &seq, std::vector<TrackIndexList> &iTrkClusters) {
    TrackIndexList iTrksClustering;
    TrackIndex iTrk;
    const TrackIndex nTrks = seq.GetTracksNumber();
    const FrameIndex trkLenMax = seq.GetFramesNumber() / 10;
    const FrameIndex trkLenMin = std::min(m_hkmClusteredTrkLenKF, trkLenMax);
    for(iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(seq.GetTrackLength(iTrk) >= trkLenMin)
            iTrksClustering.push_back(iTrk);
    }
    const TrackIndex nTrksClustering = TrackIndex(iTrksClustering.size());
    m_hkm.Resize(nTrksClustering);
    for (TrackIndex i = 0; i < nTrksClustering; ++i)
        seq.ComputeTrackDescriptor(iTrksClustering[i], m_hkm[i]);

    m_hkm.RunHierarchicalLloyd(m_clusters);
    iTrkClusters.resize(0);
    const TrackIndex nClusters = TrackIndex(m_clusters.size());
    for(TrackIndex iCluster = 0; iCluster < nClusters; ++iCluster) {
        iTrkClusters.push_back(TrackIndexList());
        TrackIndexList &iTrksClustered = iTrkClusters.back();
        const KMeansCluster &cluster = m_clusters[iCluster];
        const TrackIndex nTrksClustered = cluster.Size();
        iTrksClustered.resize(nTrksClustered);
        for(TrackIndex i = 0; i < nTrksClustered; ++i)
            iTrksClustered[i] = iTrksClustering[cluster[i]];
    }
}

void TrackMatcher::ClusterTracks(const SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2,
                                 std::vector<TrackIndexListPair> &iTrkClusters) {
    TrackIndexList iTrks1Clustering, iTrks2Clustering;
    const Sequence &seq1 = seqs[iSeq1];
    const TrackIndex nTrks1 = seq1.GetTracksNumber();
    const FrameIndex trkLenMax1 = seq1.GetFramesNumber() / 10;
    const FrameIndex trkLenMin1 = std::min(m_hkmClusteredTrkLenKF, trkLenMax1);
    for(TrackIndex iTrk = 0; iTrk < nTrks1; ++iTrk) {
        if(seq1.GetTrackLength(iTrk) >= trkLenMin1)
            iTrks1Clustering.push_back(iTrk);
    }
    const TrackIndex nTrks1Clustering = TrackIndex(iTrks1Clustering.size());
    const Sequence &seq2 = seqs[iSeq2];
    const TrackIndex nTrks2 = seq2.GetTracksNumber();
    const FrameIndex trkLenMax2 = seq2.GetFramesNumber() / 10;
    const FrameIndex trkLenMin2 = std::min(m_hkmClusteredTrkLenKF, trkLenMax2);
    for(TrackIndex iTrk = 0; iTrk < nTrks2; ++iTrk) {
        if(seq2.GetTrackLength(iTrk) >= trkLenMin2)
            iTrks2Clustering.push_back(iTrk);
    }
    const TrackIndex nTrks2Clustering = TrackIndex(iTrks2Clustering.size());
    const TrackIndex nTrksClustering = nTrks1Clustering + nTrks2Clustering;
    m_hkm.Resize(nTrksClustering);
    for(TrackIndex idx = 0; idx < nTrks1Clustering; ++idx)
        seq1.ComputeTrackDescriptor(iTrks1Clustering[idx], m_hkm[idx]);
    for(TrackIndex idx = nTrks1Clustering, i = 0; i < nTrks2Clustering; ++idx, ++i)
        seq2.ComputeTrackDescriptor(iTrks2Clustering[i], m_hkm[idx]);

    m_hkm.RunHierarchicalLloyd(m_clusters);

    TrackIndex idx;
    TrackIndexList iTrks1Clustered, iTrks2Clustered;
    iTrkClusters.resize(0);
    const TrackIndex nClusters = TrackIndex(m_clusters.size());
    for(TrackIndex iCluster = 0; iCluster < nClusters; ++iCluster) {
        iTrks1Clustered.resize(0);
        iTrks2Clustered.resize(0);
        const KMeansCluster &cluster = m_clusters[iCluster];
        const TrackIndex nTrksClustered = cluster.Size();
        for(TrackIndex i = 0; i < nTrksClustered; ++i) {
            if((idx = cluster[i]) < nTrks1Clustering)
                iTrks1Clustered.push_back(iTrks1Clustering[idx]);
            else
                iTrks2Clustered.push_back(iTrks2Clustering[idx - nTrks1Clustering]);
        }
        if(!iTrks1Clustered.empty() && !iTrks2Clustered.empty())
            iTrkClusters.push_back(std::make_pair(iTrks1Clustered, iTrks2Clustered));
    }
}

void TrackMatcher::ClusterTracks(const SequenceSet &seqs, const SequenceIndexPairList &iSeqPairs,
                                 std::vector<std::vector<TrackIndexListPair> > &iTrkClustersList) {
    SequenceIndex iSeq1, iSeq2;
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    std::vector<bool> &seqMarks = m_marks1;
    seqMarks.assign(nSeqs, false);
    const int nSeqPairs = int(iSeqPairs.size());
    iTrkClustersList.resize(nSeqPairs);
    for(int i = 0; i < nSeqPairs; ++i) {
        iSeqPairs[i].Get(iSeq1, iSeq2);
        seqMarks[iSeq1] = seqMarks[iSeq2] = true;
        iTrkClustersList[i].resize(0);
    }

    std::vector<TrackIndexList> iTrksListClustering(nSeqs);
    SequenceIndex iSeq;
    TrackIndex iTrk, nTrksClustering = 0;
    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        if(!seqMarks[iSeq])
            continue;
        const Sequence &seq = seqs[iSeq];
        const FrameIndex trkLenMin = std::min(m_hkmClusteredTrkLenKF, seq.GetFramesNumber());
        TrackIndexList &iTrksClustering = iTrksListClustering[iSeq];
        const TrackIndex nTrks = seq.GetTracksNumber();
        for(iTrk = 0; iTrk < nTrks; ++iTrk) {
            if(seq.GetTrackLength(iTrk) >= trkLenMin)
                iTrksClustering.push_back(iTrk);
        }
        nTrksClustering += TrackIndex(iTrksClustering.size());
    }
    m_hkm.Resize(nTrksClustering);
    SequenceTrackIndexList iSeqTrksClustering(nTrksClustering);
    TrackIndex idx = 0;
    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        const Sequence &seq = seqs[iSeq];
        const TrackIndexList &iTrksClustering = iTrksListClustering[iSeq];
        const TrackIndex nTrksClustering = TrackIndex(iTrksClustering.size());
        for(TrackIndex i = 0; i < nTrksClustering; ++i, ++idx) {
            iTrk = iTrksClustering[i];
            seq.ComputeTrackDescriptor(iTrk, m_hkm[idx]);
            iSeqTrksClustering[idx].Set(iSeq, iTrk);
        }
    }

    m_hkm.RunHierarchicalLloyd(m_clusters);

    std::vector<TrackIndexList> iTrksListClustered(nSeqs);
    const TrackIndex nClusters = TrackIndex(m_clusters.size());
    for(TrackIndex iCluster = 0; iCluster < nClusters; ++iCluster) {
        for(iSeq = 0; iSeq < nSeqs; ++iSeq)
            iTrksListClustered[iSeq].resize(0);
        const KMeansCluster &cluster = m_clusters[iCluster];
        const TrackIndex nTrksClustered = cluster.Size();
        for(TrackIndex i = 0; i < nTrksClustered; ++i) {
            iSeqTrksClustering[cluster[i]].Get(iSeq, iTrk);
            iTrksListClustered[iSeq].push_back(iTrk);
        }
        for(int i = 0; i < nSeqPairs; ++i) {
            iSeqPairs[i].Get(iSeq1, iSeq2);
            const TrackIndexList &iTrks1Clustered = iTrksListClustered[iSeq1], &iTrks2Clustered = iTrksListClustered[iSeq2];
            if(!iTrks1Clustered.empty() && !iTrks2Clustered.empty())
                iTrkClustersList[i].push_back(std::make_pair(iTrks1Clustered, iTrks2Clustered));
        }
    }
}