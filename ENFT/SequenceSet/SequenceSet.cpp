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

void SequenceSet::SetCalib(const char *calibFileName, bool focalConst) {
    if (calibFileName!="")
        SetIntrinsicType(Sequence::INTRINSIC_USER_FIXED);
    else if (focalConst)
        SetIntrinsicType(Sequence::INTRINSIC_CONSTANT);
    else
        SetIntrinsicType(Sequence::INTRINSIC_VARIABLE);
}

void SequenceSet::GetSequenceIndexList(const SequenceState &seqState,
                                       SequenceIndexList &iSeqs) const {
    iSeqs.resize(0);
    const SequenceIndex nSeqs = GetSequencesNumber();
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        if(m_seqStates[iSeq] & seqState)
            iSeqs.push_back(iSeq);
    }
}

void SequenceSet::GetSimilarityEstimatorData(const SequenceIndex &iSeq,
        SimilarityTransformationEstimatorData2D &data, TrackIndexList &iTrksIdv) const {
    const Sequence &seq = GetSequence(iSeq);
    const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];

    //iTrksCmn.resize(0);
    iTrksIdv.resize(0);
    const TrackIndex nTrksIdv = seq.GetTracksNumber();
    TrackIndex iTrkCmn, iTrkIdv;
    MeasurementIndex nMeasIdv = 0;
    for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
        if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) == INVALID_TRACK_INDEX ||
                !(m_cmnTrkStates[iTrkCmn] & FLAG_COMMON_TRACK_STATE_REGISTERED)
                || !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER))
            continue;
        //iTrksCmn.push_back(iTrkCmn);
        iTrksIdv.push_back(iTrkIdv);
        nMeasIdv += seq.CountTrackInlierMeasurements(iTrkIdv);
    }

    const uint nPts = uint(iTrksIdv.size()), nMeas = nMeasIdv;
    data.Resize(nPts, nMeas);
    uint i, j;
    FrameIndex k, iFrmSrc;
    MeasurementIndex iMeaSrc;
    for(i = j = 0; i < nPts; ++i) {
        //iTrkCmn = iTrksCmn[i];
        iTrkCmn = iTrksIdvToCmn[iTrksIdv[i]];
        data.X1(i) = GetCommonPoint(iTrkCmn);
        iTrkIdv = iTrksIdv[i];
        data.X2(i) = seq.GetPoint(iTrkIdv);
        data.SetPointMeasurementIndex(i, j);

        const MeasurementIndexList &iMeasSrc = seq.GetTrackMeasurementIndexList(
                iTrkIdv);
        const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
        for(k = 0; k < nCrspsSrc; ++k) {
            iMeaSrc = iMeasSrc[k];
            iFrmSrc = seq.GetMeasurementFrameIndex(iMeaSrc);
            if(!(seq.GetFrameState(iFrmSrc) & FLAG_FRAME_STATE_SOLVED) ||
                    (seq.GetMeasurementState(iMeaSrc) & FLAG_MEASUREMENT_STATE_OUTLIER))
                continue;
            data.pC2(j) = &seq.GetCamera(seq.GetMeasurementFrameIndex(iMeaSrc));
            data.x2(j) = seq.GetMeasurement(iMeaSrc);
            ++j;
        }
    }
    if(m_intrinsicType == Sequence::INTRINSIC_CONSTANT)
        Sequence::RectifyMeasurements(seq.GetIntrinsicRectification(), data.x2s());
    else if(m_intrinsicType == Sequence::INTRINSIC_VARIABLE) {
        data.ValidateWeights();
        for(i = j = 0; i < nPts; ++i) {
            iTrkIdv = iTrksIdv[i];
            const MeasurementIndexList &iMeasSrc = seq.GetTrackMeasurementIndexList(
                    iTrkIdv);
            const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
            for(k = 0; k < nCrspsSrc; ++k) {
                iMeaSrc = iMeasSrc[k];
                iFrmSrc = seq.GetMeasurementFrameIndex(iMeaSrc);
                if(!(seq.GetFrameState(iFrmSrc) & FLAG_FRAME_STATE_SOLVED) ||
                        (seq.GetMeasurementState(iMeaSrc) & FLAG_MEASUREMENT_STATE_OUTLIER))
                    continue;
                seq.GetIntrinsicRectification(iFrmSrc).Rectify(data.x2(j));
                data.SetWeight(j++, seq.GetIntrinsicRectification(iFrmSrc).f());
            }
        }
    }
    data.SetFocal(seq.GetIntrinsicMatrix().fxy());
}

void SequenceSet::operator = (const SequenceSet &seqs) {
    printf("Error!\n");
    exit(0);
    m_dir = seqs.m_dir;
    m_intrinsicType = seqs.m_intrinsicType;
    m_pSeqs = seqs.m_pSeqs;
    m_mapCmnTrkToIdvTrk = seqs.m_mapCmnTrkToIdvTrk;
    m_mapIdvTrkToCmnTrk = seqs.m_mapIdvTrkToCmnTrk;
    m_seqStates = seqs.m_seqStates;
    m_cmnTrkStates = seqs.m_cmnTrkStates;
    m_XsCmn = seqs.m_XsCmn;
}

void SequenceSet::CreateSequences(const SequenceIndex &nSeqs) {
    const SequenceIndex nSeqsOri = SequenceIndex(m_pSeqs.size());
    for(SequenceIndex iSeq = nSeqs; iSeq < nSeqsOri; ++iSeq) {
        if(m_pSeqs[iSeq])
            delete m_pSeqs[iSeq];
    }
    m_pSeqs.resize(nSeqs);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        if(!m_pSeqs[iSeq])
            m_pSeqs[iSeq] = new Sequence();
        m_pSeqs[iSeq]->SetIntrinsicType(m_intrinsicType);
    }
    InitializeCommonPoints();
}

void SequenceSet::ReleaseSequences() {
    const SequenceIndex nSeqs = SequenceIndex(m_pSeqs.size());
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        if(m_pSeqs[iSeq])
            delete m_pSeqs[iSeq];
        m_pSeqs[iSeq] = NULL;
    }
}

void SequenceSet::InitializeCommonPoints() {
    m_mapCmnTrkToIdvTrk.resize(0);
    const SequenceIndex nSeqs = GetSequencesNumber();
    m_mapIdvTrkToCmnTrk.resize(nSeqs);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
        m_mapIdvTrkToCmnTrk[iSeq].assign(GetSequence(iSeq).GetTracksNumber(),
                                         INVALID_TRACK_INDEX);
    m_seqStates.assign(nSeqs, FLAG_SEQUENCE_STATE_DEFAULT);
    m_cmnTrkStates.resize(0);
    m_XsCmn.Resize(0);
}

//void SequenceSet::Clear()
//{
//  const SequenceIndex nSeqs = GetSequencesNumber();
//  for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
//      delete m_pSeqs[iSeq];
//  m_pSeqs.clear();
//  m_mapCmnTrkToIdvTrk.clear();
//  m_mapIdvTrkToCmnTrk.clear();
//  m_seqStates.clear();
//  m_cmnTrkStates.clear();
//  m_XsCmn.Clear();
//}

SequenceIndex SequenceSet::CountSequences(const SequenceState seqState) const {
    SequenceIndex cnt = 0;
    const SequenceIndex nSeqs = GetSequencesNumber();
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        if(m_seqStates[iSeq] & seqState)
            ++cnt;
    }
    return cnt;
}

TrackIndex SequenceSet::CountSequenceRegisteredCommonTracks(
    const SequenceIndex &iSeq) const {
    const Sequence &seq = GetSequence(iSeq);
    const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
    const TrackIndex nTrksIdv = GetSequence(iSeq).GetTracksNumber();
    TrackIndex iTrkIdv, iTrkCmn, cnt = 0;
    for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
        if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX &&
                (m_cmnTrkStates[iTrkCmn] & FLAG_COMMON_TRACK_STATE_REGISTERED)
                && (seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER))
            ++cnt;
    }
    return cnt;
}

TrackIndex SequenceSet::CountSequenceMarkedCommonTracks(
    const SequenceIndex &iSeq, const std::vector<bool> &cmnTrkMarks) const {
    const Sequence &seq = GetSequence(iSeq);
    const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
    const TrackIndex nTrksIdv = GetSequence(iSeq).GetTracksNumber();
    TrackIndex iTrkIdv, iTrkCmn, cnt = 0;
    for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
        if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX &&
                cmnTrkMarks[iTrkCmn])
            ++cnt;
    }
    return cnt;
}

void SequenceSet::GetSequenceMarkedCommonTrackIndexList(
    const SequenceIndex &iSeq, const std::vector<bool> &cmnTrkMarks,
    TrackIndexList &iTrksCmn) const {
    const Sequence &seq = GetSequence(iSeq);
    const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
    const TrackIndex nTrksIdv = GetSequence(iSeq).GetTracksNumber();
    TrackIndex iTrkIdv, iTrkCmn;
    iTrksCmn.resize(0);
    for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
        if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX &&
                cmnTrkMarks[iTrkCmn])
            iTrksCmn.push_back(iTrkCmn);
    }
}

void SequenceSet::MarkSequenceRegistered(const SequenceIndex &iSeq) {
    m_seqStates[iSeq] |= FLAG_SEQUENCE_STATE_REGISTRED;

    float MSEori, MSEnew;

    Sequence &seq = *m_pSeqs[iSeq];
    const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
    const TrackIndex nTrksIdv = seq.GetTracksNumber();
    TrackIndex iTrkIdv, iTrkCmn;
    for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
        //if((seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER) && !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_COMMON_OUTLIER)
        //&& (iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX)
        //  m_cmnTrkStates[iTrkCmn] |= FLAG_COMMON_TRACK_STATE_REGISTERED;
        if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) == INVALID_TRACK_INDEX ||
                !(seq.GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_INLIER))
            continue;
        if(m_cmnTrkStates[iTrkCmn] & FLAG_COMMON_TRACK_STATE_REGISTERED) {
            ComputeCommonTrackMSE(iTrkCmn, m_XsCmn[iTrkCmn], MSEori);
            ComputeCommonTrackMSE(iTrkCmn, seq.GetPoint(iTrkIdv), MSEnew);
            if(MSEnew < MSEori)
                SetCommonPoint(iTrkCmn, seq.GetPoint(iTrkIdv));
        } else {
            SetCommonPoint(iTrkCmn, seq.GetPoint(iTrkIdv));
            m_cmnTrkStates[iTrkCmn] |= FLAG_COMMON_TRACK_STATE_REGISTERED;
        }
    }
}

void SequenceSet::MarkSequencesSwappedOut() {
    const SequenceIndex nSeqs = GetSequencesNumber();
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
        MarkSequenceSwappedOut(iSeq);
}

void SequenceSet::MarkSequenceCommonTracks(const SequenceIndex &iSeq,
        std::vector<bool> &cmnTrkMarks) const {
    TrackIndex iTrkIdv, iTrkCmn;
    const TrackIndexList &iTrksIdvToCmn = m_mapIdvTrkToCmnTrk[iSeq];
    const TrackIndex nTrksIdv = TrackIndex(iTrksIdvToCmn.size());
    for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv) {
        if((iTrkCmn = iTrksIdvToCmn[iTrkIdv]) != INVALID_TRACK_INDEX)
            cmnTrkMarks[iTrkCmn] = true;
    }
}

void SequenceSet::MarkSequencesCommonTracks(const SequenceIndexList &iSeqs,
        std::vector<bool> &cmnTrkMarks) const {
    cmnTrkMarks.assign(GetCommonTracksNumber(), false);
    const SequenceIndex nSeqs = SequenceIndex(iSeqs.size());
    for(SequenceIndex i = 0; i < nSeqs; ++i)
        MarkSequenceCommonTracks(iSeqs[i], cmnTrkMarks);
}