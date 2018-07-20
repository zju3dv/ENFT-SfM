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
#include "SequenceRegisteror.h"
#include "Utility/Utility.h"

using namespace ENFT_SfM;

void SequenceRegisteror::ViewSequences(const SequenceSet &seqs, const SequenceIndex &iSeq) {
    if(!m_view)
        return;
    ProgramGL::UnbindFrameBuffer();
    ViewerSequenceSet::Initialize(seqs, iSeq);
    //PrepareActiveFrame(m_iFrmsActive[iSeq]);
    Resize(m_pWnd->size());
    Draw();
    DrawString();
    m_pWnd->swap_buffers();
    m_pWnd->handle_events(m_handler);
    while(!m_handler.Quit() && m_stop) {
        Draw();
        DrawString();
        m_pWnd->swap_buffers();
        m_pWnd->handle_events(m_handler);
    }
    ProgramGL::BindFrameBuffer();
}

bool SequenceRegisteror::OnKeyDown(const int key) {
    switch(key) {
        case KEY_STOP_SEQUENCE_REGISTRATION:
            m_stop = !m_stop;
            return true;
    }
    return ViewerSequenceSet::OnKeyDown(key);
}

bool SequenceRegisteror::SaveRelativePoses(const char *fileName, const std::vector<AlignedVector<RigidTransformation3D> > &TsList) {
    FILE *fp = fopen( fileName, "wb");
    if(!fp)
        return false;
    const SequenceIndex nSeqs = SequenceIndex(TsList.size());
    fwrite(&nSeqs, sizeof(SequenceIndex), 1, fp);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
        TsList[iSeq].SaveB(fp);
    fclose(fp);
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Saved \'%s\'\n", fileName);
#endif
    return true;
}

bool SequenceRegisteror::LoadRelativePoses(const char *fileName, std::vector<AlignedVector<RigidTransformation3D> > &TsList) {
    FILE *fp = fopen( fileName, "rb");
    if(!fp)
        return false;
    SequenceIndex nSeqs;
    fread(&nSeqs, sizeof(SequenceIndex), 1, fp);
    TsList.resize(nSeqs);
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
        TsList[iSeq].LoadB(fp);
    fclose(fp);
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Loaded \'%s\'\n", fileName);
#endif
    return true;
}

//bool SequenceRegisteror::SaveKeyFrameSequences(const char *fileName, const SequenceSet &seqsKF, const std::vector<FrameIndexList> &iFrmsListKF,
//                                             const std::vector<TrackIndexList> &iTrksListIdvKF, const TrackIndexList &iTrksCmnKF)
//{
//  FILE *fp = fopen( fileName, "wb");
//  if(!fp)
//      return false;
//  seqsKF.SaveB(fp);
//  IO::VectorSetSaveB(iFrmsListKF, fp);
//  IO::VectorSetSaveB(iTrksListIdvKF, fp);
//  IO::VectorSaveB(iTrksCmnKF, fp);
//  fclose(fp);
//#if VERBOSE_SEQUENCE_REGISTRATION
//  printf("Saved \'%s\'\n", fileName);
//#endif
//  return true;
//}
//
//bool SequenceRegisteror::LoadKeyFrameSequences(const char *fileName, SequenceSet &seqsKF, std::vector<FrameIndexList> &iFrmsListKF,
//                                             std::vector<TrackIndexList> &iTrksListIdvKF, TrackIndexList &iTrksCmnKF)
//{
//  FILE *fp = fopen( fileName, "rb");
//  if(!fp)
//      return false;
//  seqsKF.LoadB(fp);
//  IO::VectorSetLoadB(iFrmsListKF, fp);
//  IO::VectorSetLoadB(iTrksListIdvKF, fp);
//  IO::VectorLoadB(iTrksCmnKF, fp);
//  fclose(fp);
//#if VERBOSE_SEQUENCE_REGISTRATION
//  printf("Loaded \'%s\'\n", fileName);
//#endif
//  return true;
//}

bool SequenceRegisteror::SaveB(const char *fileName, const SequenceSet &seqs, const SequenceSet &seqsKF, const std::vector<FrameIndexList> &iFrmsListKF,
                               const std::vector<TrackIndexList> &iTrksListIdvKF, const TrackIndexList &iTrksCmnKF) {
    FILE *fp = fopen( fileName, "wb");
    if(!fp)
        return false;
    seqs.SaveB(fp);
    seqsKF.SaveB(fp);
    IO::VectorSetSaveB(iFrmsListKF, fp);
    IO::VectorSetSaveB(iTrksListIdvKF, fp);
    IO::VectorSaveB(iTrksCmnKF, fp);
    fclose(fp);
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Saved \'%s\'\n", fileName);
#endif
    return true;
}

bool SequenceRegisteror::LoadB(const char *fileName, SequenceSet &seqs, SequenceSet &seqsKF, std::vector<FrameIndexList> &iFrmsListKF,
                               std::vector<TrackIndexList> &iTrksListIdvKF, TrackIndexList &iTrksCmnKF) {
    FILE *fp = fopen( fileName, "rb");
    if(!fp)
        return false;
    seqs.LoadB(fp);
    seqsKF.LoadB(fp);
    seqsKF.SetDirectory(seqs.GetDirectory());
    IO::VectorSetLoadB(iFrmsListKF, fp);
    IO::VectorSetLoadB(iTrksListIdvKF, fp);
    IO::VectorLoadB(iTrksCmnKF, fp);
    fclose(fp);
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Loaded \'%s\'\n", fileName);
#endif
    return true;
}

bool SequenceRegisteror::SaveB(const char *fileName, const SequenceSet &seqs, const SegmentIndex &nSegsPerSeq) {
    FILE *fp = fopen( fileName, "wb");
    if(!fp)
        return false;
    seqs.SaveB(fp);
    fwrite(&nSegsPerSeq, sizeof(SegmentIndex), 1, fp);
    fclose(fp);
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Saved \'%s\'\n", fileName);
#endif
    return true;
}

bool SequenceRegisteror::LoadB(const char *fileName, SequenceSet &seqs, SegmentIndex &nSegsPerSeq) {
    FILE *fp = fopen( fileName, "rb");
    if(!fp)
        return false;
    seqs.LoadB(fp);
    fread(&nSegsPerSeq, sizeof(SegmentIndex), 1, fp);
    fclose(fp);
#if VERBOSE_SEQUENCE_REGISTRATION
    printf("Loaded \'%s\'\n", fileName);
#endif
    return true;
}