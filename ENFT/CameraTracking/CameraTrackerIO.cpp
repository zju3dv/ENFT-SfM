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
#include "CameraTracker.h"
#include "Utility/Utility.h"

using namespace ENFT_SfM;

void CameraTracker::ViewSequence(const Sequence &seq, const FrameIndex &iFrm) {
    if(!m_view)
        return;
    ProgramGL::UnbindFrameBuffer();
    ViewerSequence::Initialize(seq, iFrm);
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

bool CameraTracker::OnKeyDown(const int key) {
    switch(key) {
        case KEY_STOP_CAMERA_TRACKING:
            m_stop = !m_stop;
            return true;
    }
    return ViewerSequence::OnKeyDown(key);
}

void CameraTracker::SaveB(const char *fileName, const Sequence &seq,
                          const Sequence &seqKF, const FrameIndexList &iFrmsKF,
                          const TrackIndexList &iTrksKF,
                          const MeasurementIndexList &iMeasKF,
                          const std::vector<FrameErrorLevel> &frmErrLevels) {

    FILE *fp = fopen( fileName, "wb");
    seq.SaveB(fp);
    seqKF.SaveB(fp);
    IO::VectorSaveB(iFrmsKF, fp);
    IO::VectorSaveB(iTrksKF, fp);
    IO::VectorSaveB(iMeasKF, fp);
    IO::VectorSaveB(frmErrLevels, fp);
    fclose(fp);
    printf("Saved \'%s\'\n", fileName);
}

void CameraTracker::SaveB(const char *fileName, const Sequence &seq,
                          const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3) {

    FILE *fp = fopen( fileName, "wb");
    seq.SaveB(fp);
    fwrite(&iFrm1, sizeof(FrameIndex), 1, fp);
    fwrite(&iFrm2, sizeof(FrameIndex), 1, fp);
    fwrite(&iFrm3, sizeof(FrameIndex), 1, fp);
    fclose(fp);
    printf("Saved \'%s\'\n", fileName);
}

void CameraTracker::SaveB(const char *fileName,
                          const Sequence
                          &seq/*, const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3*/,
                          const std::vector<FrameErrorLevel> &frmErrLevels) {

    FILE *fp = fopen( fileName, "wb");
    seq.SaveB(fp);
    //fwrite(&iFrm1, sizeof(FrameIndex), 1, fp);
    //fwrite(&iFrm2, sizeof(FrameIndex), 1, fp);
    //fwrite(&iFrm3, sizeof(FrameIndex), 1, fp);
    IO::VectorSaveB(frmErrLevels, fp);
    fclose(fp);
    printf("Saved \'%s\'\n", fileName);
}

void CameraTracker::LoadB(const char *fileName, Sequence &seq, Sequence &seqKF,
                          FrameIndexList &iFrmsKF, TrackIndexList &iTrksKF, MeasurementIndexList &iMeasKF,
                          std::vector<FrameErrorLevel> &frmErrLevels) {

    FILE *fp = fopen( fileName, "rb");
    seq.LoadB(fp);
    seqKF.LoadB(fp);
    IO::VectorLoadB(iFrmsKF, fp);
    IO::VectorLoadB(iTrksKF, fp);
    IO::VectorLoadB(iMeasKF, fp);
    IO::VectorLoadB(frmErrLevels, fp);
    fclose(fp);
    printf("Loaded \'%s\'\n", fileName);
}

void CameraTracker::LoadB(const char *fileName, Sequence &seq,
                          FrameIndex &iFrm1, FrameIndex &iFrm2, FrameIndex &iFrm3) {

    FILE *fp = fopen( fileName, "rb");
    seq.LoadB(fp);
    fread(&iFrm1, sizeof(FrameIndex), 1, fp);
    fread(&iFrm2, sizeof(FrameIndex), 1, fp);
    fread(&iFrm3, sizeof(FrameIndex), 1, fp);
    fclose(fp);
    printf("Loaded \'%s\'\n", fileName);
}

void CameraTracker::LoadB(const char *fileName,
                          Sequence &seq/*, FrameIndex &iFrm1, FrameIndex &iFrm2, FrameIndex &iFrm3*/,
                          std::vector<FrameErrorLevel> &frmErrLevels) {

    FILE *fp = fopen( fileName, "rb");
    seq.LoadB(fp);
    //fread(&iFrm1, sizeof(FrameIndex), 1, fp);
    //fread(&iFrm2, sizeof(FrameIndex), 1, fp);
    //fread(&iFrm3, sizeof(FrameIndex), 1, fp);
    IO::VectorLoadB(frmErrLevels, fp);
    fclose(fp);
    printf("Loaded \'%s\'\n", fileName);
}