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
#include "Utility/Utility.h"

void SequenceSet::SwapOut(const SequenceIndex &iSeq) {
    if(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_SWAPED_OUT)
        return;
    char fileName[MAX_LINE_LENGTH];
    sprintf(fileName, "%sseq_%02d.txt", GetDirectory().c_str(), iSeq);
    
    FILE *fp = fopen( fileName, "wb");
    m_pSeqs[iSeq]->SaveB(fp);
    m_pSeqs[iSeq]->Clear();
    MarkSequenceSwappedOut(iSeq);
    fclose(fp);
    printf("Saved \'%s\'\n", fileName);
}

void SequenceSet::SwapIn(const SequenceIndex &iSeq) {
    if(!(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_SWAPED_OUT))
        return;
    char fileName[MAX_LINE_LENGTH];
    sprintf(fileName, "%sseq_%02d.txt", GetDirectory().c_str(), iSeq);
    
    FILE *fp = fopen( fileName, "rb");
    m_pSeqs[iSeq]->LoadB(fp);
    MarkSequenceSwappedIn(iSeq);
    fclose(fp);
    printf("Loaded \'%s\'\n", fileName);
}

void SequenceSet::SaveB(FILE *fp) const {
    fwrite(&m_intrinsicType, sizeof(Sequence::IntrinsicType), 1, fp);
    const SequenceIndex nSeqs = GetSequencesNumber();
    fwrite(&nSeqs, sizeof(SequenceIndex), 1, fp);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
        GetSequence(iSeq).SaveB(fp);
    IO::VectorSetSaveB(m_mapCmnTrkToIdvTrk, fp);
    IO::VectorSetSaveB(m_mapIdvTrkToCmnTrk, fp);
    IO::VectorSaveB(m_seqStates, fp);
    IO::VectorSaveB(m_cmnTrkStates, fp);
    m_XsCmn.SaveB(fp);
}

bool SequenceSet::SaveB(const char *fileName) const {
    
	FILE *fp = fopen((GetDirectory() + fileName).c_str(), "wb");
    if (fp == nullptr)
        return false;
    SaveB(fp);
    fclose(fp);
    printf("Saved \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

void SequenceSet::LoadB(FILE *fp) {
    fread(&m_intrinsicType, sizeof(Sequence::IntrinsicType), 1, fp);
    SequenceIndex nSeqs;
    fread(&nSeqs, sizeof(SequenceIndex), 1, fp);
    CreateSequences(nSeqs);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
        m_pSeqs[iSeq]->LoadB(fp);
    IO::VectorSetLoadB(m_mapCmnTrkToIdvTrk, fp);
    IO::VectorSetLoadB(m_mapIdvTrkToCmnTrk, fp);
    IO::VectorLoadB(m_seqStates, fp);
    IO::VectorLoadB(m_cmnTrkStates, fp);
    m_XsCmn.LoadB(fp);
    //m_cmnTrkStates.assign(GetCommonTracksNumber(), FLAG_COMMON_TRACK_STATE_DEFAULT);
    //m_XsCmn.Resize(0);

    ////fread(&m_intrinsicType, sizeof(Sequence::IntrinsicType), 1, fp);
    //m_intrinsicType = Sequence::INTRINSIC_USER_FIXED;
    //SequenceIndex nSeqs;
    //fread(&nSeqs, sizeof(SequenceIndex), 1, fp);
    //CreateSequences(nSeqs);
    //for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
    //  m_pSeqs[iSeq]->LoadB(fp);
    //IO::VectorSetLoadB(m_mapCmnTrkToIdvTrk, fp);
    //IO::VectorSetLoadB(m_mapIdvTrkToCmnTrk, fp);
    //IO::VectorLoadB(m_seqStates, fp);
    //IO::VectorLoadB(m_cmnTrkStates, fp);
    //m_XsCmn.LoadB(fp);
    ////m_cmnTrkStates.assign(GetCommonTracksNumber(), FLAG_COMMON_TRACK_STATE_DEFAULT);
    ////m_XsCmn.Resize(0);
}

bool SequenceSet::LoadB(const char *fileName) {
    
	FILE *fp = fopen((GetDirectory() + fileName).c_str(), "rb");
    if (fp == nullptr)
        return false;
    LoadB(fp);
    fclose(fp);
    printf("Loaded \'%s\'\n", (GetDirectory() + fileName).c_str());
    //const SequenceIndex nSeqs = GetSequencesNumber();
    //for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
    //{
    //  Sequence &seq = *m_pSeqs[iSeq];
    //  const bool measNormaliedBkp = seq.m_measNormalized;
    //  seq.NormalizeMeasurements();
    //  seq.m_tag.SetImageSize(960, 540);
    //  seq.m_K.Load("E:/Data/Registration/Garden/K.txt");
    //  if(!measNormaliedBkp)
    //      seq.DenormalizeMeasurements();
    //}
    //SaveB(fileName);
    return true;
}

void SequenceSet::PrintCommonTrack(const TrackIndex &iTrkCmn) const {
    printf("----------------------------------------------------------------\n");
    printf("Common Track %d, ", iTrkCmn);
    if(iTrkCmn < m_XsCmn.Size() &&
            (m_cmnTrkStates[iTrkCmn] & FLAG_COMMON_TRACK_STATE_REGISTERED))
        m_XsCmn[iTrkCmn].Print();
    else
        printf("UNREGISTERED\n");
    SequenceIndex iSeq;
    TrackIndex iTrkIdv;
    const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
    const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
    for(SequenceIndex i = 0; i < nCrsps; ++i) {
        iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
        printf("  %d: Sequence %d, Track %d", i + 1, iSeq, iTrkIdv);
        const Sequence &seq = GetSequence(iSeq);
        if((m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED) &&
                iTrkIdv < seq.GetPointsNumber() &&
                (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER)) {
            const Point3D &X = seq.GetPoint(iTrkIdv);
            printf(", (%f, %f, %f)", X.X(), X.Y(), X.Z());
            if(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER)
                printf(", COMMON_OUTLIER");
            else
                printf(", COMMON_INLIER");
        }
        printf("\n");
    }
}

void SequenceSet::PrintStates() const {
    SequenceIndex cntSeqsRegistered = 0;
    uint cntFrms = 0, cntFrmsKF = 0;
    TrackIndex cntTrksIdv = 0, cntTrksIdvInlier = 0, cntTrksIdvCmn = 0,
               cntTrksIdvCmnOutlier = 0, cntTrksCmnRegistered = 0;
    const SequenceIndex nSeqs = GetSequencesNumber();
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        const Sequence &seq = GetSequence(iSeq);
        if(m_seqStates[iSeq] & FLAG_SEQUENCE_STATE_REGISTRED)
            ++cntSeqsRegistered;

        cntFrms += uint(seq.GetFramesNumber());
        cntFrmsKF += uint(seq.CountFrames(FLAG_FRAME_STATE_KEY_FRAME));

        const TrackIndex nTrksIdv = seq.GetTracksNumber();
        cntTrksIdv += nTrksIdv;
        for(TrackIndex iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
            const TrackState trkState = seq.GetTrackState(iTrkIdv);
            if(trkState & FLAG_TRACK_STATE_INLIER)
                ++cntTrksIdvInlier;
            if(trkState & FLAG_TRACK_STATE_COMMON) {
                ++cntTrksIdvCmn;
                if(!(trkState & FLAG_TRACK_STATE_INLIER) ||
                        (trkState & FLAG_TRACK_STATE_COMMON_OUTLIER))
                    ++cntTrksIdvCmnOutlier;
            }
        }
    }
    const TrackIndex nTrksCmn = GetCommonTracksNumber();
    for(TrackIndex iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn) {
        if(m_cmnTrkStates[iTrkCmn] & FLAG_COMMON_TRACK_STATE_REGISTERED)
            ++cntTrksCmnRegistered;
    }
    printf("----------------------------------------------------------------\n");
    printf("Sequences: Total = %d, Registered = %d\n", nSeqs, cntSeqsRegistered);
    printf("Common tracks: Total = %d, Registered = %d\n", nTrksCmn,
           cntTrksCmnRegistered);
    printf("Frames: Total = %d, Keyframe = %d\n", cntFrms, cntFrmsKF);
    printf("Tracks: Total = %d, Inlier = %d, Common = %d, Common Outlier = %d\n",
           cntTrksIdv, cntTrksIdvInlier, cntTrksIdvCmn, cntTrksIdvCmnOutlier);
}

void SequenceSet::AssertCommonTrackSorted(const TrackIndex &iTrkCmn) const {
    const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
    const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
    for(SequenceIndex i = 1; i < nCrsps; ++i) {
        IO::Assert(iSeqTrksIdv[i - 1].GetSequenceIndex() <
                   iSeqTrksIdv[i].GetSequenceIndex(),
                   "iTrkCmn = %d, i = %d, iSeq1 = %d, iSeq2 = %d\n", iTrkCmn, i,
                   iSeqTrksIdv[i - 1].GetSequenceIndex(), iSeqTrksIdv[i].GetSequenceIndex());
    }
}

void SequenceSet::AssertConsistency() const {
    const SequenceIndex nSeqs = GetSequencesNumber();
    SequenceIndex iSeq;
    for(iSeq = 0; iSeq < nSeqs; ++iSeq)
        m_pSeqs[iSeq]->AssertConsistency();

    std::vector<std::vector<bool> > seqTrkIdvMarksList(nSeqs);
    for(iSeq = 0; iSeq < nSeqs; ++iSeq)
        seqTrkIdvMarksList[iSeq].resize(GetSequence(iSeq).GetTracksNumber(), false);

    TrackIndex iTrkCmn, iTrkIdv;
    const TrackIndex nTrksCmn = GetCommonTracksNumber();
    for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn) {
        AssertCommonTrackSorted(iTrkCmn);
        const SequenceTrackIndexList &iSeqTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
        const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
        for(SequenceIndex i = 0; i < nCrsps; ++i) {
            iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
            IO::Assert(m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] == iTrkCmn,
                       "iTrkCmn = %d, i = %d, iSeq = %d, iTrkIdv = %d, m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv] = %d\n",
                       iTrkCmn, i, iSeq, iTrkIdv, m_mapIdvTrkToCmnTrk[iSeq][iTrkIdv]);
            IO::Assert(!seqTrkIdvMarksList[iSeq][iTrkIdv],
                       "Already marked! iSeq = %d, iTrkIdv = %d\n", iSeq, iTrkIdv);
            seqTrkIdvMarksList[iSeq][iTrkIdv] = true;
        }
    }
    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        const TrackIndexList &iTrksCmn = m_mapIdvTrkToCmnTrk[iSeq];
        const std::vector<bool> &trkIdvMarks = seqTrkIdvMarksList[iSeq];
        const Sequence &seq = GetSequence(iSeq);
        const TrackIndex nTrksIdv = seq.GetTracksNumber();
        TrackIndex cnt = 0;
        for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
            IO::Assert(iTrksCmn[iTrkIdv] == INVALID_TRACK_INDEX ||
                       trkIdvMarks[iTrkIdv],
                       "iTrkIdv = %d, iTrksCmn[iTrkIdv] = %d, trkIdvMarks[iTrkIdv] = %d",
                       iTrkIdv, iTrksCmn[iTrkIdv], trkIdvMarks[iTrkIdv]);
            if(iTrksCmn[iTrkIdv] != INVALID_TRACK_INDEX)
                ++cnt;
        }
        IO::Assert(cnt == seq.CountTracks(FLAG_TRACK_STATE_COMMON),
                   "cnt = %d, seq.CountTracks(FLAG_TRACK_STATE_COMMON) = %d\n",
                   cnt, seq.CountTracks(FLAG_TRACK_STATE_COMMON));
    }
    //printf("OK!\n");
}