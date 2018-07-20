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
#include "Viewing/ViewerSequenceSet.h"
#include "Utility/Utility.h"
#include "Utility/Timer.h"

using namespace ENFT_SfM;

void TrackMatcher::Initialize(const Sequence &seq,
                              const std::string paramFileName) {
    Configurator param;
    param.Load(paramFileName.c_str());
    Initialize(seq, param);
}

void TrackMatcher::Initialize(/*const */SequenceSet &seqs,
                                        const std::string paramFileName) {
    Configurator param;
    param.Load(paramFileName.c_str());
    Initialize(seqs, param);
}

void TrackMatcher::Run(Sequence &seq, const std::string outputFileName,
                       const std::string tmpFileNameTrkClusters, FILE *fpTiming) {
    if(outputFileName != "" && seq.LoadBwithDir(outputFileName.c_str()) ||
            seq.GetDescriptorsNumber() == 0)
        return;

    //const FrameIndex nFrms = seq.GetFramesNumber();
    //ProgramGL::Initialize(nFrms, nFrms);

#if VERBOSE_TRACK_MATCHING
    printf("****************************************************************\n");
#endif

    Timer timer(2);

    const bool stopBkp = m_stop;

    Sequence seqKF;
    FrameIndexList iFrmsKF;
    TrackIndexList iTrksKF;

    seq.RemoveBrokenTracks(m_idxs);
    seq.RemoveNullMeasurements();
    seq.DenormalizeMeasurements();
    ExtractKeyFrameSequence(seq, seqKF, iFrmsKF, iTrksKF);

    TrackMatchList trkMatchesKF;
    std::vector<TrackIndexList> iTrkClusters;
    const std::string tmpFileName = tmpFileNameTrkClusters == "" ? "" :
                                    seqKF.GetDirectory() + tmpFileNameTrkClusters;
    if(tmpFileName == "" || !LoadTrackClusters(tmpFileName.c_str(), iTrkClusters)) {
        timer.Start(0);
        ClusterTracks(seqKF, iTrkClusters);
        timer.Stop(0);
        if(tmpFileName != "")
            SaveTrackClusters(tmpFileName.c_str(), iTrkClusters);
    }
#if VERBOSE_TRACK_MATCHING
    printf("----------------------------------------------------------------\n");
#endif
    timer.Start(1);
    MatchTracksEnft(seqKF, iTrkClusters, trkMatchesKF);
    ConvertTrackMatchesKeyFrameSequenceToOriginalSequence(iTrksKF, trkMatchesKF);
    const TrackIndex nTrksBkp = seq.GetTracksNumber();
    MergeMatchedTracks_RemoveUnmergeableTrackMatches(seq, trkMatchesKF);
    if(m_removeSingleTrk) {
        const TrackIndex nTrks = seq.GetTracksNumber();
        for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
            if(seq.GetTrackLength(iTrk) == 1)
                seq.BreakTrack(iTrk);
        }
    }
    seq.RemoveBrokenTracks();
    seq.RemoveNullMeasurements();
    timer.Stop(1);

#if _DEBUG
    seq.AssertConsistency();
#endif

    if(fpTiming) {
        fprintf(fpTiming, "Track clustering: ");
        timer.PrintTotalTiming(0, fpTiming);
        fprintf(fpTiming, "Track merging:    ");
        timer.PrintTotalTiming(1, fpTiming);
    } else {
        printf("Track clustering: ");
        timer.PrintTotalTiming(0);
        printf("Track matching:    ");
        timer.PrintTotalTiming(1);
    }

    if(m_view && stopBkp) {
        ViewerSequence viewer;
        ProgramGL::Initialize();
        viewer.Run(seq);
    }
    m_stop = stopBkp;

    //{
    //  const TrackIndex nTrks = seq.GetTracksNumber();
    //  printf("Tracks = %d - %d = %d\n", nTrksBkp, nTrksBkp - nTrks, nTrks);
    //  ViewerSequence viewer;
    //  ProgramGL::Initialize();
    //  viewer.Run(seq);
    //}

    if(outputFileName != "")
        seq.SaveBwithDir(outputFileName.c_str());
}

void TrackMatcher::Run(Sequence &seq, const FrameIndexPairList &iFrmPairs,
                       const std::string outputFileName, FILE *fpTiming) {
    if(outputFileName != "" && seq.LoadBwithDir(outputFileName.c_str()) ||
            seq.GetDescriptorsNumber() == 0)
        return;

#if VERBOSE_TRACK_MATCHING
    printf("****************************************************************\n");
#endif

    Timer timer;
    timer.Start();

    const bool stopBkp = m_stop;

    FrameIndex iFrm1, iFrm2;
    FrameIndexPairList iFrmPairsExp;
    const FrameIndex nFrms = seq.GetFramesNumber();
    ExpandFramePairs(nFrms, nFrms, iFrmPairs, iFrmPairsExp);
    std::vector<bool> frmMarks(nFrms, false);
    const int nFrmPairsExp = int(iFrmPairsExp.size());
    for(int i = 0; i < nFrmPairsExp; ++i) {
        iFrmPairsExp[i].Get(iFrm1, iFrm2);
        frmMarks[iFrm1] = frmMarks[iFrm2] = true;
    }

    Sequence seqKF;
    FrameIndexList iFrmsKF;
    TrackIndexList iTrksKF;
    seq.RemoveBrokenTracks(m_idxs);
    seq.RemoveNullMeasurements();
    seq.DenormalizeMeasurements();
    ExtractKeyFrameSequence(seq, seqKF, iFrmsKF, iTrksKF, frmMarks);

    TrackMatchList trkMatchesKF;
    SparseMatrix<float> matchingMatrixInit;
    const FrameIndex nFrmsKF = seqKF.GetFramesNumber();
    matchingMatrixInit.Resize(nFrmsKF, nFrmsKF);
    for(int i = 0; i < nFrmPairsExp; ++i) {
        iFrmPairsExp[i].Get(iFrm1, iFrm2);
        iFrm1 = FrameIndex(std::lower_bound(iFrmsKF.begin(), iFrmsKF.end(),
                                            iFrm1) - iFrmsKF.begin());
        iFrm2 = FrameIndex(std::lower_bound(iFrmsKF.begin(), iFrmsKF.end(),
                                            iFrm2) - iFrmsKF.begin());
        matchingMatrixInit.Insert(iFrm1, iFrm2, 1);
    }

#if VERBOSE_TRACK_MATCHING
    printf("----------------------------------------------------------------\n");
#endif
    MatchTracksEnft(seqKF, matchingMatrixInit, trkMatchesKF);

    ConvertTrackMatchesKeyFrameSequenceToOriginalSequence(iTrksKF, trkMatchesKF);
    MergeMatchedTracks_RemoveUnmergeableTrackMatches(seq, trkMatchesKF);
    if(m_removeSingleTrk) {
        const TrackIndex nTrks = seq.GetTracksNumber();
        for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
            if(seq.GetTrackLength(iTrk) == 1)
                seq.BreakTrack(iTrk);
        }
    }
    seq.RemoveBrokenTracks();
    seq.RemoveNullMeasurements();

    timer.Stop();
    timer.PrintTotalTiming(0, fpTiming);

    if(m_view && stopBkp) {
        ViewerSequence viewer;
        ProgramGL::Initialize();
        viewer.Run(seq);
    }
    m_stop = stopBkp;

    if(outputFileName != "")
        seq.SaveBwithDir(outputFileName.c_str());
}

void TrackMatcher::Run(Sequence &seq,
                       const SparseMatrix<float> &matchingMatrixInit, const std::string outputFileName,
                       FILE *fpTiming) {
    if(outputFileName != "" && seq.LoadBwithDir(outputFileName.c_str()) ||
            seq.GetDescriptorsNumber() == 0)
        return;

    //ProgramGL::Initialize(seq.GetFramesNumber(), seq.GetFramesNumber());

#if VERBOSE_TRACK_MATCHING
    printf("****************************************************************\n");
#endif

    Timer timer;
    timer.Start();

    const bool stopBkp = m_stop;

    FrameIndex iFrm1, iFrm2;
    const FrameIndex nFrms = seq.GetFramesNumber();
    std::vector<bool> frmMarks(nFrms, false);
    for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1) {
        const FullIndexValueList<float> &elements = matchingMatrixInit.GetRowData(
                    iFrm1);
        const FrameIndex nElements = FrameIndex(elements.size());
        for(FrameIndex i = 0; i < nElements; ++i) {
            iFrm2 = elements[i].GetFullIndex();
            frmMarks[iFrm1] = frmMarks[iFrm2] = true;
        }
    }

    Sequence seqKF;
    FrameIndexList iFrmsKF;
    TrackIndexList iTrksKF;
    seq.RemoveBrokenTracks(m_idxs);
    seq.RemoveNullMeasurements();
    seq.DenormalizeMeasurements();
    ExtractKeyFrameSequence(seq, seqKF, iFrmsKF, iTrksKF, frmMarks);

    const FrameIndex nFrmsKF = FrameIndex(iFrmsKF.size());
    SparseMatrix<float> matchingMatrixInitKF(nFrmsKF, nFrmsKF);
    FrameIndex i1, i2;
    const float *pConfidence;
    for(i1 = 0; i1 < nFrmsKF; ++i1) {
        iFrm1 = iFrmsKF[i1];
        for(i2 = i1 + 1; i2 < nFrmsKF; ++i2) {
            iFrm2 = iFrmsKF[i2];
            if((pConfidence = matchingMatrixInit.Get(iFrm1, iFrm2)) != NULL &&
                    *pConfidence != 0)
                matchingMatrixInitKF.Insert(i1, i2, *pConfidence);
        }
    }
    std::vector<TrackIndexList> iTrkClusters;
    ClusterTracks(seqKF, iTrkClusters);

    TrackMatchList trkMatchesKF;
    //MatchTracksEnft(seqKF, matchingMatrixInitKF, trkMatchesKF);
    MatchTracksEnft(seqKF, iTrkClusters, trkMatchesKF);

    ConvertTrackMatchesKeyFrameSequenceToOriginalSequence(iTrksKF, trkMatchesKF);

    m_img.resize(CVD::ImageRef(int(nFrms), int(nFrms)));
    m_matchingMatrixUpdImg.resize(CVD::ImageRef(int(nFrms), int(nFrms)));
    m_matchingMatrixUpdTex.Generate(nFrms, nFrms);
    m_matchingMatrixUpdTexScaled.Generate(nFrms, nFrms);
    PrepareUpdatingMatchingMatrixImageAndTexture(seq, trkMatchesKF);
    ViewMatchingMatrix();

    MergeMatchedTracks_RemoveUnmergeableTrackMatches(seq, trkMatchesKF);
    if(m_removeSingleTrk) {
        const TrackIndex nTrks = seq.GetTracksNumber();
        for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
            if(seq.GetTrackLength(iTrk) == 1)
                seq.BreakTrack(iTrk);
        }
    }
    seq.RemoveBrokenTracks();
    seq.RemoveNullMeasurements();

    timer.Stop();
    timer.PrintTotalTiming(0, fpTiming);

    if(m_view && stopBkp) {
        ViewerSequence viewer;
        ProgramGL::Initialize();
        viewer.Run(seq);
    }
    m_stop = stopBkp;

    if(outputFileName != "")
        seq.SaveBwithDir(outputFileName.c_str());
}

void TrackMatcher::Run(SequenceSet &seqs,
                       const SequenceIndexPairList &iSeqPairs, const std::string outputFileName,
                       const std::string tmpFileNameTrkClusters,
                       const std::string tmpFileNameTrkMatches, FILE *fpTiming,
                       const bool clearDescs) {
    if(outputFileName != "" && seqs.LoadB(outputFileName.c_str()))
        return;

    Timer timer(2);

    const bool stopBkp = m_stop;

    SequenceIndex iSeq, iSeq1, iSeq2;
    char buf[MAX_LINE_LENGTH];
    std::string tmpFileName;
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    m_iFrmsListKF.resize(nSeqs);
    m_iTrksListKF.resize(nSeqs);
    const Sequence::IntrinsicType intrinsicType = seqs.GetIntrinsicType();
    m_seqsKF.SetDirectory(seqs.GetDirectory());
    m_seqsKF.SetIntrinsicType(intrinsicType);
    m_seqsKF.CreateSequences(nSeqs);
    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        Sequence &seq = seqs[iSeq];
        if(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_SWAPED_OUT)
            seq.SwapIn();
        seq.RemoveOutlierTracksAndMeasurements();
        seq.DenormalizeMeasurements();
        seq.UnmarkTracksMerged();
        ExtractKeyFrameSequence(seq, m_seqsKF[iSeq], m_iFrmsListKF[iSeq],
                                m_iTrksListKF[iSeq]);
        if(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_SWAPED_OUT) {
            seq.SwapOut();
            m_seqsKF.SwapOut(iSeq);
        }
    }
    m_seqsKF.InitializeCommonPoints();

    const int nSeqPairs = int(iSeqPairs.size());
    m_trkMatchesList.resize(nSeqPairs);
    for(int i = 0; i < nSeqPairs; ++i) {
        iSeqPairs[i].Get(iSeq1, iSeq2);
#if VERBOSE_TRACK_MATCHING
        printf("****************************************************************\n");
        printf("Sequence (%d, %d)\n", iSeq1, iSeq2);
#endif
        sprintf(buf, "_%d-%d", iSeq1, iSeq2);
        const std::string suffix = buf;

        m_seqsKF.SwapIn(iSeq1);
        m_seqsKF.SwapIn(iSeq2);
#if VERBOSE_TRACK_MATCHING
        printf("----------------------------------------------------------------\n");
#endif
        tmpFileName = tmpFileNameTrkClusters == "" ? "" : seqs.GetDirectory() +
                      IO::InsertSuffix(tmpFileNameTrkClusters, suffix);
        if(tmpFileName == "" ||
                !LoadTrackClusters(tmpFileName.c_str(), m_iTrkClusters)) {
            timer.Start(0);
            ClusterTracks(m_seqsKF, iSeq1, iSeq2, m_iTrkClusters);
            timer.Stop(0);
            if(tmpFileName != "")
                SaveTrackClusters(tmpFileName.c_str(), m_iTrkClusters);
        }

#if VERBOSE_TRACK_MATCHING
        printf("----------------------------------------------------------------\n");
#endif
        tmpFileName = tmpFileNameTrkMatches == "" ? "" : seqs.GetDirectory() +
                      IO::InsertSuffix(tmpFileNameTrkMatches, suffix);
        TrackMatchList &trkMatches = m_trkMatchesList[i];
        if(tmpFileName == "" || !LoadTrackMatches(tmpFileName.c_str(), trkMatches)) {
            timer.Start(1);
            TrackMatchList &trkMatchesKF = trkMatches;
            MatchTracksEnft(m_seqsKF, iSeq1, iSeq2, m_iTrkClusters, trkMatchesKF);
            const TrackIndexList &iTrks1KF = m_iTrksListKF[iSeq1],
                                  &iTrks2KF = m_iTrksListKF[iSeq2];
            TrackIndex iTrk1, iTrk2;
            const TrackIndex nMatches = TrackIndex(trkMatchesKF.size());
            for(TrackIndex i = 0; i < nMatches; ++i) {
                trkMatchesKF[i].Get(iTrk1, iTrk2);
                iTrk1 = iTrks1KF[iTrk1];
                iTrk2 = iTrks2KF[iTrk2];
                trkMatches[i].Set(iTrk1, iTrk2);
            }
            timer.Stop(1);
            if(tmpFileName != "")
                SaveTrackMatches(tmpFileName.c_str(), trkMatches);
        }
        if(seqs.GetSequenceState(iSeq1) & FLAG_SEQUENCE_STATE_SWAPED_OUT) {
            m_seqsKF[iSeq1].Clear();
            m_seqsKF.MarkSequenceSwappedOut(iSeq1);
        }
        if(seqs.GetSequenceState(iSeq2) & FLAG_SEQUENCE_STATE_SWAPED_OUT) {
            m_seqsKF[iSeq2].Clear();
            m_seqsKF.MarkSequenceSwappedOut(iSeq2);
        }
    }

    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        Sequence &seq = seqs[iSeq];
        if(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_SWAPED_OUT) {
            seq.SwapIn();
            seqs.MarkSequenceSwappedIn(iSeq);
        }
        if(clearDescs)
            seq.ClearDescriptors();
    }
    seqs.InitializeCommonPoints();
    for(int i = 0; i < nSeqPairs; ++i) {
        iSeqPairs[i].Get(iSeq1, iSeq2);
        seqs.MatchIndividualTracks(iSeq1, iSeq2, m_trkMatchesList[i]);
    }
#if VERBOSE_TRACK_MATCHING
    TrackIndexList nTrksIdvBkp(nSeqs);
    for(iSeq = 0; iSeq < nSeqs; ++iSeq)
        nTrksIdvBkp[iSeq] = seqs[iSeq].GetTracksNumber();
#endif
    seqs.FinishMatchingIndividualTracks(m_errSqThReprojPt);
    //seqs.AssertConsistency();
#if VERBOSE_TRACK_MATCHING
    printf("****************************************************************\n");
    for(iSeq = 0; iSeq < nSeqs; ++iSeq)
        printf("Sequence %d: %d common tracks, %d --> %d individual tracks\n", iSeq,
               seqs[iSeq].CountTracks(FLAG_TRACK_STATE_COMMON), nTrksIdvBkp[iSeq],
               seqs[iSeq].GetTracksNumber());
#endif

    if(fpTiming) {
        fprintf(fpTiming, "Track clustering: ");
        timer.PrintTotalAndStableMeanTiming(0, fpTiming);
        fprintf(fpTiming, "Track matching:   ");
        timer.PrintTotalAndStableMeanTiming(1, fpTiming);
    } else {
        printf("Track clustering: ");
        timer.PrintTotalAndStableMeanTiming(0);
        printf("Track matching:   ");
        timer.PrintTotalAndStableMeanTiming(1);
    }

    if(m_view && stopBkp) {
        for(iSeq = 0; iSeq < nSeqs; ++iSeq)
            seqs[iSeq].NormalizeMeasurements();
        ViewerSequenceSet viewer;
        ProgramGL::Initialize();
        viewer.Run(seqs);
    }
    m_stop = stopBkp;

    if(outputFileName != "")
        seqs.SaveB(outputFileName.c_str());//sequenceset savewithDir bug
}


void TrackMatcher::Run(SequenceSet &seqs,
                       const SequenceIndexPairList &iSeqPairs,
                       const std::vector<FrameIndexPairList> &iFrmPairsList,
                       const std::string outputFileName, const std::string tmpFileNameTrkMatches,
                       FILE *fpTiming) {
    if(outputFileName != "" && seqs.LoadB(outputFileName.c_str()))
        return;

    Timer timer(2);

    const bool stopBkp = m_stop;

    SequenceIndex iSeq, iSeq1, iSeq2;
    FrameIndex iFrm1, iFrm2;
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    m_frmMarksList.resize(nSeqs);
    for(iSeq = 0; iSeq < nSeqs; ++iSeq)
        m_frmMarksList[iSeq].assign(seqs[iSeq].GetFramesNumber(), false);

    const int nSeqPairs = int(iSeqPairs.size());
    m_iFrmPairsListExp.resize(nSeqPairs);
    for(int i = 0; i < nSeqPairs; ++i) {
        iSeqPairs[i].Get(iSeq1, iSeq2);
        FrameIndexPairList &iFrmPairsExp = m_iFrmPairsListExp[i];
        ExpandFramePairs(seqs[iSeq1].GetFramesNumber(), seqs[iSeq2].GetFramesNumber(),
                         iFrmPairsList[i], iFrmPairsExp);
        std::vector<bool> &frmMarks1 = m_frmMarksList[iSeq1],
                           &frmMarks2 = m_frmMarksList[iSeq2];
        const int nFrmPairsExp = int(iFrmPairsExp.size());
        for(int j = 0; j < nFrmPairsExp; ++j) {
            iFrmPairsExp[j].Get(iFrm1, iFrm2);
            frmMarks1[iFrm1] = frmMarks2[iFrm2] = true;
        }
    }

    char buf[MAX_LINE_LENGTH];
    std::string tmpFileName;
    const Sequence::IntrinsicType intrinsicType = seqs.GetIntrinsicType();
    m_seqsKF.SetDirectory(seqs.GetDirectory());
    m_seqsKF.SetIntrinsicType(intrinsicType);
    m_seqsKF.CreateSequences(nSeqs);
    m_iFrmsListKF.resize(nSeqs);
    m_iTrksListKF.resize(nSeqs);
    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        Sequence &seq = seqs[iSeq];
        if(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_SWAPED_OUT)
            seq.SwapIn();
        seq.RemoveOutlierTracksAndMeasurements();
        seq.DenormalizeMeasurements();
        seq.UnmarkTracksMerged();
        ExtractKeyFrameSequence(seq, m_seqsKF[iSeq], m_iFrmsListKF[iSeq],
                                m_iTrksListKF[iSeq], m_frmMarksList[iSeq]);
        if(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_SWAPED_OUT) {
            seq.SwapOut();
            m_seqsKF.SwapOut(iSeq);
        }
    }
    m_seqsKF.InitializeCommonPoints();

    m_trkMatchesList.resize(nSeqPairs);
    for(int i = 0; i < nSeqPairs; ++i) {
        iSeqPairs[i].Get(iSeq1, iSeq2);
#if VERBOSE_TRACK_MATCHING
        printf("****************************************************************\n");
        printf("Sequence (%d, %d)\n", iSeq1, iSeq2);
#endif
        sprintf(buf, "_%d-%d", iSeq1, iSeq2);
        const std::string suffix = buf;

        const FrameIndexList &iFrms1KF = m_iFrmsListKF[iSeq1],
                              &iFrms2KF = m_iFrmsListKF[iSeq2];
        const FrameIndex nFrms1KF = m_seqsKF[iSeq1].GetFramesNumber(),
                         nFrms2KF = m_seqsKF[iSeq2].GetFramesNumber();
        m_matchingMatrixInit.Resize(nFrms1KF, nFrms2KF);
        m_matchingMatrixInit.Clear();
        const FrameIndexPairList &iFrmPairsExp = m_iFrmPairsListExp[i];
        const int nFrmPairsExp = int(iFrmPairsExp.size());
        for(int j = 0; j < nFrmPairsExp; ++j) {
            iFrmPairsExp[j].Get(iFrm1, iFrm2);
            iFrm1 = FrameIndex(std::lower_bound(iFrms1KF.begin(), iFrms1KF.end(),
                                                iFrm1) - iFrms1KF.begin());
            iFrm2 = FrameIndex(std::lower_bound(iFrms2KF.begin(), iFrms2KF.end(),
                                                iFrm2) - iFrms2KF.begin());
            m_matchingMatrixInit.Insert(iFrm1, iFrm2, 1);
        }

        m_seqsKF.SwapIn(iSeq1);
        m_seqsKF.SwapIn(iSeq2);

#if VERBOSE_TRACK_MATCHING
        printf("----------------------------------------------------------------\n");
#endif
        tmpFileName = tmpFileNameTrkMatches == "" ? "" : seqs.GetDirectory() +
                      IO::InsertSuffix(tmpFileNameTrkMatches, suffix);
        TrackMatchList &trkMatches = m_trkMatchesList[i];
        if(tmpFileName == "" || !LoadTrackMatches(tmpFileName.c_str(), trkMatches)) {
            timer.Start(1);
            TrackMatchList &trkMatchesKF = trkMatches;
            MatchTracksEnft(m_seqsKF, iSeq1, iSeq2, m_matchingMatrixInit, trkMatchesKF);
            const TrackIndexList &iTrks1KF = m_iTrksListKF[iSeq1],
                                  &iTrks2KF = m_iTrksListKF[iSeq2];
            TrackIndex iTrk1, iTrk2;
            const TrackIndex nMatches = TrackIndex(trkMatchesKF.size());
            for(TrackIndex i = 0; i < nMatches; ++i) {
                trkMatchesKF[i].Get(iTrk1, iTrk2);
                iTrk1 = iTrks1KF[iTrk1];
                iTrk2 = iTrks2KF[iTrk2];
                trkMatches[i].Set(iTrk1, iTrk2);
            }
            timer.Stop(1);
            if(tmpFileName != "")
                SaveTrackMatches(tmpFileName.c_str(), trkMatches);
        }
        if(seqs.GetSequenceState(iSeq1) & FLAG_SEQUENCE_STATE_SWAPED_OUT) {
            m_seqsKF[iSeq1].Clear();
            m_seqsKF.MarkSequenceSwappedOut(iSeq1);
        }
        if(seqs.GetSequenceState(iSeq2) & FLAG_SEQUENCE_STATE_SWAPED_OUT) {
            m_seqsKF[iSeq2].Clear();
            m_seqsKF.MarkSequenceSwappedOut(iSeq2);
        }
    }

    for(iSeq = 0; iSeq < nSeqs; ++iSeq) {
        Sequence &seq = seqs[iSeq];
        if(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_SWAPED_OUT) {
            seq.SwapIn();
            seqs.MarkSequenceSwappedIn(iSeq);
        }
        seq.ClearDescriptors();
    }
    seqs.InitializeCommonPoints();
    for(int i = 0; i < nSeqPairs; ++i) {
        iSeqPairs[i].Get(iSeq1, iSeq2);
        seqs.MatchIndividualTracks(iSeq1, iSeq2, m_trkMatchesList[i]);
    }
#if VERBOSE_TRACK_MATCHING
    TrackIndexList nTrksIdvBkp(nSeqs);
    for(iSeq = 0; iSeq < nSeqs; ++iSeq)
        nTrksIdvBkp[iSeq] = seqs[iSeq].GetTracksNumber();
#endif
    seqs.FinishMatchingIndividualTracks(m_errSqThReprojPt);
    //seqs.AssertConsistency();
#if VERBOSE_TRACK_MATCHING
    printf("****************************************************************\n");
    for(iSeq = 0; iSeq < nSeqs; ++iSeq)
        printf("Sequence %d: %d common tracks, %d --> %d individual tracks\n", iSeq,
               seqs[iSeq].CountTracks(FLAG_TRACK_STATE_COMMON), nTrksIdvBkp[iSeq],
               seqs[iSeq].GetTracksNumber());
#endif

    if(fpTiming) {
        fprintf(fpTiming, "Track clustering: ");
        timer.PrintTotalAndStableMeanTiming(0, fpTiming);
        fprintf(fpTiming, "Track matching:   ");
        timer.PrintTotalAndStableMeanTiming(1, fpTiming);
    } else {
        printf("Track clustering: ");
        timer.PrintTotalAndStableMeanTiming(0);
        printf("Track matching:   ");
        timer.PrintTotalAndStableMeanTiming(1);
    }

    if(m_view && stopBkp) {
        ViewerSequenceSet viewer;
        ProgramGL::Initialize();
        viewer.Run(seqs);
    }
    m_stop = stopBkp;

    if(outputFileName != "")
        seqs.SaveB(outputFileName.c_str());
}

void TrackMatcher::Initialize(/*const ushort &width, const ushort &height, */const
        ushort &maxNumFtrsPerImg, const FrameIndexList &seqFrmCnts,
        const Configurator &param) {
    //ProgramGL::Initialize(width, height);
    ProgramGL::Initialize(0, 0);
    if(maxNumFtrsPerImg == 0)
        m_maxNumFtrsPerImg = ushort(param.GetArgument("sift_max_features_number",
                                    2048)) * 3;
    else
        m_maxNumFtrsPerImg = maxNumFtrsPerImg;
    m_maxNumFtrsPerImg = ((m_maxNumFtrsPerImg + 3) & (~3));

    m_kfMinNumCmnTrksTwoKeyFrms = FeatureIndex(
                                      param.GetArgument("kf_min_common_tracks_number_two_key_frames", 200));
    m_kfMinNumCmnTrksThreeKeyFrms = FeatureIndex(
                                        param.GetArgument("kf_min_common_tracks_number_three_key_frames", 100));

    m_hkmClusteredTrkLenKF = FrameIndex(
                                 param.GetArgument("hkm_clustered_track_length_key_frame", 3));
    m_hkm.m_nClustersPerLevel = uint(
                                    param.GetArgument("hkm_clusters_number_per_level", 2));
    m_hkm.m_maxNumLevels = uint(param.GetArgument("hkm_max_levels_number", 20));
    m_hkm.m_nPtsPerClusterTh = 2;
    m_hkm.m_avgDistortionTh = param.GetArgument("hkm_average_distortion_threshold",
                              0.032f);
    m_hkm.m_kmMaxNumItersTotal = uint(
                                     param.GetArgument("hkm_max_iterations_number_total", 100));
    m_hkm.m_kmMaxNumItersCenterMoving = uint(
                                            param.GetArgument("hkm_max_iterations_number_center_moving", 5));
    m_hkm.m_verbose = VERBOSE_TRACK_MATCHING != 0;

    m_ftrTexWidth = ushort(param.GetArgument("feature_texture_width", 64));
    m_ftrTexWidthLog = ushort(log(float(m_ftrTexWidth)) / log(2.0f));
    m_descTexWidth = (m_ftrTexWidth << 4);
    m_descTexWidthLog = m_ftrTexWidthLog + 4;

    const float errThEp = param.GetArgument("error_threshold_epipolar", 2.0f);
    const float errThReprojCam =
        param.GetArgument("error_threshold_reprojection_camera", 10.0f);
    const float errThReprojPt =
        param.GetArgument("error_threshold_reprojection_point", 2.0f);
    m_Festor.m_ransacErrorThreshold = errThEp * errThEp;
    m_errSqThReprojCam = errThReprojCam * errThReprojCam;
    m_errSqThReprojPt = errThReprojPt * errThReprojPt;
    m_Festor.m_ransacMinNumIters = uint(
                                       param.GetArgument("ransac_min_iterations_number_epipolar", 100));
    m_Festor.m_ransacMaxNumIters = uint(
                                       param.GetArgument("ransac_max_iterations_number_epipolar", 300));
    m_CPestor.m_ransacMinNumIters = uint(
                                        param.GetArgument("ransac_min_iterations_number_camera", 150));
    m_CPestor.m_ransacMaxNumIters = uint(
                                        param.GetArgument("ransac_max_iterations_number_camera", 300));
    m_ransacMinNumInliersCam = FeatureIndex(
                                   param.GetArgument("ransac_min_inliers_number_camera", 20));

    m_descDotTh =
        param.GetArgument("sift_matching_descriptor_dot_product_threshold", 0.95f);
    m_descDistTh = (1 - m_descDotTh) * 2;
    m_nearest1To2RatioTh =
        param.GetArgument("sift_matching_descriptor_nearest_first_to_second_ratio_threshold",
                          0.7f);
    m_ftrMatcherSift.Initialize(m_maxNumFtrsPerImg, m_maxNumFtrsPerImg,
                                m_ftrTexWidth, errThEp, m_nearest1To2RatioTh, m_descDotTh);

    const ushort ftrTexHeight = (m_maxNumFtrsPerImg + m_ftrTexWidth - 1) /
                                m_ftrTexWidth, descTexHeight = ftrTexHeight;
    m_ftrTex1.Generate(m_ftrTexWidth, ftrTexHeight);
    m_ftrTex2.Generate(m_ftrTexWidth, ftrTexHeight);
    m_descTex1.Generate(m_descTexWidth, descTexHeight);
    m_descTex2.Generate(m_descTexWidth, descTexHeight);
    m_ftrs.EnlargeCapacity(m_maxNumFtrsPerImg);
    m_descs.EnlargeCapacity(m_maxNumFtrsPerImg);

    m_enftMaxNumItersInitMatchingMatrix = ushort(
            param.GetArgument("enft_max_iterations_number_initial_matching_matrix", 100));
    m_enftMaxNumItersUpdMatchingMatrix = ushort(
            param.GetArgument("enft_max_iterations_number_updating_matching_matrix", 500));
    m_enftMinConfidenceInitMatchingMatrixRatio =
        param.GetArgument("enft_min_confidence_initial_matching_matrix_ratio", 0.1f);
    m_enftMinConfidenceUpdMatchingMatrix = ushort(
            param.GetArgument("enft_min_confidence_updating_matching_matrix", 50));
    m_enftPositiveVotingTh = ushort(
                                 param.GetArgument("enft_positive_voting_threshold", 5));
    m_enftPositiveVotingRatioTh =
        param.GetArgument("enft_positive_voting_ratio_threshold", 0.8f);
    m_enftFrmPairWinSize = param.GetArgument("enft_frame_pair_window_size", 0);
    m_enftMergeDistanceSqTh = param.GetArgument("enft_merge_distance_threshold",
                              2.0f);

    m_programScale.Initialize();

    m_removeSingleTrk = param.GetArgument("remove_single_track", 1) != 0;

    m_stop = param.GetArgument("stop", 0) != 0;
    m_view = param.GetArgument("view", 1) != 0;
    Viewer::Initialize();
    m_scale = 1.0f;
}

void TrackMatcher::Initialize(const Sequence &seq, const Configurator &param) {
    ushort nFtrs, maxNumFtrsPerImg = 0;
    const FrameIndex nFrms = seq.GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        nFtrs = seq.CountFrameTrackedFeatures(iFrm);
        if(nFtrs > maxNumFtrsPerImg)
            maxNumFtrsPerImg = nFtrs;
    }
    Initialize(/*seq.GetImageWidth(), seq.GetImageHeight(), */maxNumFtrsPerImg,
            FrameIndexList(1, seq.GetFramesNumber()), param);
}

void TrackMatcher::Initialize(/*const */SequenceSet &seqs,
                                        const Configurator &param) {
    ushort nFtrs, maxNumFtrsPerImg = 0;
    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    FrameIndexList seqFrmCnts(nSeqs);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        /*const */Sequence &seq = seqs[iSeq];
        if(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_SWAPED_OUT)
            seq.SwapIn();
        const FrameIndex nFrms = seq.GetFramesNumber();
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            nFtrs = seq.CountFrameTrackedFeatures(iFrm);
            if(nFtrs > maxNumFtrsPerImg)
                maxNumFtrsPerImg = nFtrs;
        }
        seqFrmCnts[iSeq] = seq.GetFramesNumber();
        if(seqs.GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_SWAPED_OUT)
            seq.Clear();
    }
    Initialize(/*seqs[0].GetImageWidth(), seqs[0].GetImageHeight(), */maxNumFtrsPerImg,
            seqFrmCnts, param);
}

void TrackMatcher::ExtractKeyFrameSequence(Sequence &seq, Sequence &seqKF,
        FrameIndexList &iFrmsKF, TrackIndexList &iTrksKF) {
    std::vector<bool> &frmMarksKF = m_marks1;
    seq.MarkKeyFrames(m_kfMinNumCmnTrksTwoKeyFrms, m_kfMinNumCmnTrksThreeKeyFrms,
                      frmMarksKF);

    iFrmsKF.resize(0);
    const FrameIndex nFrms = seq.GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        if(!frmMarksKF[iFrm])
            continue;
        iFrmsKF.push_back(iFrm);
        seq.MarkFrameKeyFrame(iFrm);
    }
#if VERBOSE_TRACK_MATCHING
    printf("Key frames: %d --> %d\n", nFrms, iFrmsKF.size());
#endif

    seq.GetSubSequence(iFrmsKF, seqKF, iTrksKF, true, false);
    //seqKF.AssertConsistency();

#if VERBOSE_TRACK_MATCHING
    printf("Key frame tracks: %d --> %d\n", seq.GetTracksNumber(), iTrksKF.size());
#endif
}

void TrackMatcher::ExtractKeyFrameSequence(Sequence &seq, Sequence &seqKF,
        FrameIndexList &iFrmsKF, TrackIndexList &iTrksKF,
        const std::vector<bool> &frmMarks) {
    std::vector<bool> &frmMarksKF = m_marks1;
    seq.MarkKeyFrames(m_kfMinNumCmnTrksTwoKeyFrms, m_kfMinNumCmnTrksThreeKeyFrms,
                      frmMarksKF);

    iFrmsKF.resize(0);
    const FrameIndex nFrms = seq.GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        if(!frmMarksKF[iFrm] && !frmMarks[iFrm])
            continue;
        iFrmsKF.push_back(iFrm);
        seq.MarkFrameKeyFrame(iFrm);
    }
#if VERBOSE_TRACK_MATCHING
    printf("Key frames: %d --> %d\n", nFrms, iFrmsKF.size());
#endif

    seq.GetSubSequence(iFrmsKF, seqKF, iTrksKF, true, false);
    //seqKF.AssertConsistency();

#if VERBOSE_TRACK_MATCHING
    printf("Key frame tracks: %d --> %d\n", seq.GetTracksNumber(), iTrksKF.size());
#endif
}

void TrackMatcher::ConvertTrackMatchesKeyFrameSequenceToOriginalSequence(
    const TrackIndexList &iTrksKF, TrackMatchList &trkMatches) {
    TrackIndex iTrk1, iTrk2;
    const TrackIndex nTrkMatches = TrackIndex(trkMatches.size());
    for(TrackIndex i = 0; i < nTrkMatches; ++i) {
        trkMatches[i].Get(iTrk1, iTrk2);
        iTrk1 = iTrksKF[iTrk1];
        iTrk2 = iTrksKF[iTrk2];
        trkMatches[i].Set(iTrk1, iTrk2);
    }
}

void TrackMatcher::UploadDescriptorsFromCPU(const Sequence &seq,
        const FrameIndex &iFrm, const TextureGL4 &descTex) {
    const Descriptor *descs = seq.GetFrameDescriptors(iFrm);
    const uint nFtrs = uint(seq.GetFrameFeaturesNumber(iFrm));
    {
        descTex.Bind();
        const uint nDrawPixels = (nFtrs << 4);
        const uint drawHeight1 = (nDrawPixels >> m_descTexWidthLog);
        const uint nDrawPixels1 = (drawHeight1 << m_descTexWidthLog);
        if(nDrawPixels1 != 0)
            descTex.UploadFromCPU((const float *) descs, m_descTexWidth, drawHeight1);
        const uint nDrawPixels2 = nDrawPixels - nDrawPixels1;
        if(nDrawPixels2 != 0)
            descTex.UploadFromCPU((const float *) (descs + (nDrawPixels1 >> 4)), 0,
                                  drawHeight1, nDrawPixels2, 1);
        //ProgramGL::FitViewportGL(descTex);
        //m_programDescNormalize.Run(descTex, m_idxTex, nFtrs, false);
    }
}

void TrackMatcher::UploadDescriptorsFromCPU(const Sequence &seq,
        const FrameIndex &iFrm, const FeatureIndexList &iFtrs,
        const TextureGL4 &descTex) {
    seq.GetFrameDescriptors(iFrm, iFtrs, m_descs);
    const uint nFtrs = m_descs.Size();
    {
        descTex.Bind();
        const uint nDrawPixels = (nFtrs << 4);
        const uint drawHeight1 = (nDrawPixels >> m_descTexWidthLog);
        const uint nDrawPixels1 = (drawHeight1 << m_descTexWidthLog);
        if(nDrawPixels1 != 0)
            descTex.UploadFromCPU((const float *) m_descs.Data(), m_descTexWidth,
                                  drawHeight1);
        const uint nDrawPixels2 = nDrawPixels - nDrawPixels1;
        if(nDrawPixels2 != 0)
            descTex.UploadFromCPU((const float *) (m_descs.Data() + (nDrawPixels1 >> 4)), 0,
                                  drawHeight1, nDrawPixels2, 1);
        //ProgramGL::FitViewportGL(descTex);
        //m_programDescNormalize.Run(descTex, m_idxTex, nFtrs, false);
    }
}

void TrackMatcher::UploadFeaturesAndDescriptorsFromCPU(const Sequence &seq,
        const FrameIndex &iFrm, const TextureGL2 &ftrTex, const TextureGL4 &descTex) {
    const Point2D *ftrs = seq.GetFrameFeatures(iFrm);
    const Descriptor *descs = seq.GetFrameDescriptors(iFrm);
    const uint nFtrs = uint(seq.GetFrameFeaturesNumber(iFrm));
    {
        ftrTex.Bind();
        const uint drawHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
        const uint nDrawPixels1 = (drawHeight1 << m_ftrTexWidthLog);
        if(nDrawPixels1 != 0)
            ftrTex.UploadFromCPU((const float *) ftrs, m_ftrTexWidth, drawHeight1);
        const uint nDrawPixels2 = nFtrs - nDrawPixels1;
        if(nDrawPixels2 != 0)
            ftrTex.UploadFromCPU((const float *) (ftrs + nDrawPixels1), 0, drawHeight1,
                                 nDrawPixels2, 1);
    }
    {
        descTex.Bind();
        const uint nDrawPixels = (nFtrs << 4);
        const uint drawHeight1 = (nDrawPixels >> m_descTexWidthLog);
        const uint nDrawPixels1 = (drawHeight1 << m_descTexWidthLog);
        if(nDrawPixels1 != 0)
            descTex.UploadFromCPU((const float *) descs, m_descTexWidth, drawHeight1);
        const uint nDrawPixels2 = nDrawPixels - nDrawPixels1;
        if(nDrawPixels2 != 0)
            descTex.UploadFromCPU((const float *) (descs + (nDrawPixels1 >> 4)), 0,
                                  drawHeight1, nDrawPixels2, 1);
        //ProgramGL::FitViewportGL(descTex);
        //m_programDescNormalize.Run(descTex, m_idxTex, nFtrs, false);
    }
}

void TrackMatcher::UploadFeaturesAndDescriptorsFromCPU(const Sequence &seq,
        const FrameIndex &iFrm, const FeatureIndexList &iFtrs, const TextureGL2 &ftrTex,
        const TextureGL4 &descTex) {
    seq.GetFrameFeaturesAndDescriptors(iFrm, iFtrs, m_ftrs, m_descs);
    const uint nFtrs = m_ftrs.Size();
    {
        ftrTex.Bind();
        const uint drawHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
        const uint nDrawPixels1 = (drawHeight1 << m_ftrTexWidthLog);
        if(nDrawPixels1 != 0)
            ftrTex.UploadFromCPU((const float *) m_ftrs.Data(), m_ftrTexWidth, drawHeight1);
        const uint nDrawPixels2 = nFtrs - nDrawPixels1;
        if(nDrawPixels2 != 0)
            ftrTex.UploadFromCPU((const float *) (m_ftrs.Data() + nDrawPixels1), 0,
                                 drawHeight1, nDrawPixels2, 1);
    }
    {
        descTex.Bind();
        const uint nDrawPixels = (nFtrs << 4);
        const uint drawHeight1 = (nDrawPixels >> m_descTexWidthLog);
        const uint nDrawPixels1 = (drawHeight1 << m_descTexWidthLog);
        if(nDrawPixels1 != 0)
            descTex.UploadFromCPU((const float *) m_descs.Data(), m_descTexWidth,
                                  drawHeight1);
        const uint nDrawPixels2 = nDrawPixels - nDrawPixels1;
        if(nDrawPixels2 != 0)
            descTex.UploadFromCPU((const float *) (m_descs.Data() + (nDrawPixels1 >> 4)), 0,
                                  drawHeight1, nDrawPixels2, 1);
        //ProgramGL::FitViewportGL(descTex);
        //m_programDescNormalize.Run(descTex, m_idxTex, nFtrs, false);
    }
}

void TrackMatcher::ExpandFramePairs(const FrameIndex nFrms1,
                                    const FrameIndex nFrms2, const FrameIndexPairList &iFrmPairs,
                                    FrameIndexPairList &iFrmPairsExp) {
    FrameIndex iFrm1Center, iFrm1Start, iFrm1End, iFrm1, iFrm2Center, iFrm2Start,
               iFrm2End, iFrm2;
    const int nFrmPairs = int(iFrmPairs.size());
    for(int i = 0; i < nFrmPairs; ++i) {
        iFrmPairs[i].Get(iFrm1Center, iFrm2Center);
        iFrm1Start = iFrm1Center >= m_enftFrmPairWinSize ? iFrm1Center -
                     m_enftFrmPairWinSize : 0;
        iFrm1End = iFrm1Center + m_enftFrmPairWinSize < nFrms1 ? iFrm1Center +
                   m_enftFrmPairWinSize : nFrms1 - 1;
        iFrm2Start = iFrm2Center >= m_enftFrmPairWinSize ? iFrm2Center -
                     m_enftFrmPairWinSize : 0;
        iFrm2End = iFrm2Center + m_enftFrmPairWinSize < nFrms2 ? iFrm2Center +
                   m_enftFrmPairWinSize : nFrms2 - 1;
        for(iFrm1 = iFrm1Start; iFrm1 <= iFrm1End; ++iFrm1)
            for(iFrm2 = iFrm2Start; iFrm2 <= iFrm2End; ++iFrm2)
                iFrmPairsExp.push_back(FrameIndexPair(iFrm1, iFrm2));
    }
}

bool TrackMatcher::MergeMatchedTracks(Sequence &seq, const TrackIndex &iTrk1,
                                      const TrackIndex &iTrk2) {
    const MeasurementIndexList &iMeas1 = seq.GetTrackMeasurementIndexList(iTrk1),
                                &iMeas2 = seq.GetTrackMeasurementIndexList(iTrk2);
    const FrameIndex iFrm1Start = seq.GetMeasurementFrameIndex(iMeas1.front()),
                     iFrm1End = seq.GetMeasurementFrameIndex(iMeas1.back());
    const FrameIndex iFrm2Start = seq.GetMeasurementFrameIndex(iMeas2.front()),
                     iFrm2End = seq.GetMeasurementFrameIndex(iMeas2.back());
    if(iFrm1Start > iFrm2End || iFrm1End < iFrm2Start) {
        seq.MatchTracks(iTrk1, iTrk2);
        seq.MarkTrackMerged(iTrk1);
        seq.MarkTrackMerged(iTrk2);
        return true;
    }

    FrameIndex i, iFrm;
    MeasurementIndex iMea1, iMea2;
    MeasurementIndexList &mapSubToMea1 = m_iMeas;
    const FrameIndex N = iFrm1End - iFrm1Start + 1;
    mapSubToMea1.assign(N, INVALID_MEASUREMENT_INDEX);
    const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()),
                     nCrsps2 = FrameIndex(iMeas2.size());
    for(i = 0; i < nCrsps1; ++i) {
        iMea1 = iMeas1[i];
        mapSubToMea1[seq.GetMeasurementFrameIndex(iMea1) - iFrm1Start] = iMea1;
    }
    m_iMeasOverlapping.resize(0);
    for(i = 0; i < nCrsps2; ++i) {
        iMea2 = iMeas2[i];
        if((iFrm = seq.GetMeasurementFrameIndex(iMea2)) < iFrm1Start ||
                iFrm > iFrm1End ||
                (iMea1 = mapSubToMea1[iFrm - iFrm1Start]) == INVALID_MEASUREMENT_INDEX)
            continue;
        if(seq.GetMeasurement(iMea1).SquaredDistance(seq.GetMeasurement(
                    iMea2)) > m_enftMergeDistanceSqTh)
            return false;
        m_iMeasOverlapping.push_back(Match<MeasurementIndex>(iMea1, iMea2));
    }
    const FrameIndex nCrspsOverlapping = FrameIndex(m_iMeasOverlapping.size());
    for(i = 0; i < nCrspsOverlapping; ++i) {
        m_iMeasOverlapping[i].Get(iMea1, iMea2);
        const ubyte enft1 = (seq.GetMeasurementState(iMea1) &
                             FLAG_MEASUREMENT_STATE_ENFT);
        const ubyte enft2 = (seq.GetMeasurementState(iMea2) &
                             FLAG_MEASUREMENT_STATE_ENFT);
        if(enft1 && !enft2)
            seq.NullifyMeasurement(iMea1);
        else if(!enft1 && enft2)
            seq.NullifyMeasurement(iMea2);
        else if(nCrsps1 < nCrsps2)
            seq.NullifyMeasurement(iMea1);
        else if(nCrsps1 > nCrsps2)
            seq.NullifyMeasurement(iMea2);
        else if(!(seq.GetTrackState(iTrk1) & FLAG_TRACK_STATE_MERGED) &&
                (seq.GetTrackState(iTrk2) & FLAG_TRACK_STATE_MERGED))
            seq.NullifyMeasurement(iMea1);
        else
            seq.NullifyMeasurement(iMea2);
    }
    seq.MatchTracks(iTrk1, iTrk2);
    seq.MarkTrackMerged(iTrk1);
    seq.MarkTrackMerged(iTrk2);
    return true;
}

void TrackMatcher::MergeMatchedTracks_RemoveUnmergeableTrackMatches(
    Sequence &seq, TrackMatchList &trkMatches) {
    const TrackIndex nTrks = seq.GetTracksNumber();
    TrackIndexList &iTrksOriToNew = m_idxs;
    iTrksOriToNew.resize(nTrks);
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
        iTrksOriToNew[iTrk] = iTrk;

    TrackIndex i, j, iTrk1, iTrk2;
    const TrackIndex nTrkMatches = TrackIndex(trkMatches.size());
    for(i = j = 0; i < nTrkMatches; ++i) {
        trkMatches[i].Get(iTrk1, iTrk2);
        while(iTrksOriToNew[iTrk1] != iTrk1) {
#if _DEBUG
            assert(seq.GetTrackLength(iTrk1) == 0);
#endif
            iTrk1 = iTrksOriToNew[iTrk1];
        }
        while(iTrksOriToNew[iTrk2] != iTrk2) {
#if _DEBUG
            assert(seq.GetTrackLength(iTrk2) == 0);
#endif
            iTrk2 = iTrksOriToNew[iTrk2];
        }
        if(iTrk1 != iTrk2 && !MergeMatchedTracks(seq, iTrk1, iTrk2))
            continue;
        iTrksOriToNew[iTrk2] = iTrk1;
        trkMatches[j++] = trkMatches[i];
    }
    trkMatches.resize(j);

#if VERBOSE_TRACK_MATCHING
    printf("Mergable track matches: %d / %d\n", trkMatches.size(), nTrkMatches);
#endif

}