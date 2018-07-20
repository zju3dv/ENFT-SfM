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
#include "Utility/Utility.h"

using namespace ENFT_SfM;

void FeatureTracker::ViewSequence(const Sequence &seq, const FrameIndex &iFrm) {
    if(!m_view)
        return;
    ProgramGL::UnbindFrameBuffer();
    m_iFrmActive = INVALID_FRAME_INDEX;
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
    //char fileName[MAX_LINE_LENGTH];
    //sprintf(fileName, "E:/tmp/%05d.jpg", g_cnt++);
    //Viewer::SaveView(fileName);
    ProgramGL::BindFrameBuffer();
}

bool FeatureTracker::OnKeyDown(const int key) {
    switch(key) {
        case KEY_STOP_FEATURE_TRACKING:
            m_stop = !m_stop;
            return true;
        case KEY_STEP_ACTIVE_FRAME_FORWARD:
            if(m_stop && m_iFrmActive == m_pSeq->GetFramesNumber() - 1) {
                m_handler.Quit() = true;
                return true;
            }
            break;
    }
    return ViewerSequence::OnKeyDown(key);
}

void FeatureTracker::PushBackTrackColors(const FrameIndex &iFrm, Sequence &seq) {
    MeasurementIndex iMea;
    FeatureIndex iFtr;
    CVD::Rgb<ubyte> clr;

    const ushort iBuffer = m_bufferManager.GetDataBufferIndex(iFrm);
    const AlignedVector<Point2D> &ftrs = m_ftrsBuffer[iBuffer];

    TrackIndex iTrk = seq.GetTrackColorsNumber();
    const TrackIndex nTrks = seq.GetTracksNumber();
    seq.SetTrackColorsNumber(nTrks);
    for(; iTrk < nTrks; ++iTrk) {
        if(seq.GetTrackLength(iTrk) == 0)
            continue;
        iMea = seq.GetTrackMeasurementIndexList(iTrk).back();
#if _DEBUG
        //if(seq.GetMeasurementFrameIndex(iMea) != iFrm)
        //  printf("trk%d, mea%d, frm%d\n", iTrk, iMea, seq.GetMeasurementFrameIndex(iMea));
        assert(seq.GetMeasurementFrameIndex(iMea) == iFrm);
#endif
        iFtr = seq.GetMeasurementFrameFeatureIndex(iMea);
        const Point2D &ftr = ftrs[iFtr];
        const int x = int(ftr.x() + 0.5f);
        const int y = int(ftr.y() + 0.5f);
        if(x >= 0 && x < m_imgRGB.size().x && y >= 0 && y < m_imgRGB.size().y)
            clr = m_imgRGB[y][x];
        else
            clr = CVD::Rgb<ubyte>(0, 0, 0);
        seq.SetTrackColor(iTrk, clr);
        //seq.PushBackTrackColor(clr);
    }
}

void FeatureTracker::SaveB(const char *fileName, const Sequence &seq, const FrameIndex &iFrmKey, const FrameIndex &iFrmLast,
                           const FrameIndex &iFrmCurrent) const {
    FILE *fp = fopen( fileName, "wb");
    seq.SaveB(fp);
    fwrite(&iFrmKey, sizeof(FrameIndex), 1, fp);
    fwrite(&iFrmLast, sizeof(FrameIndex), 1, fp);
    fwrite(&iFrmCurrent, sizeof(FrameIndex), 1, fp);
    m_bufferManager.SaveB(fp);
    const std::vector<uint> &nFtrsSiftBuffer = m_ftrExtractorSift.GetFeaturesNumberBuffer();
    IO::VectorSaveB(nFtrsSiftBuffer, fp);
    for(ushort iBuffer = 0; iBuffer < m_bufferSize; ++iBuffer) {
        m_ftrsBuffer[iBuffer].SaveB(fp);
        m_descsBuffer[iBuffer].SaveB(fp);
        IO::VectorSetSaveB(m_matchesBuffer, fp);
        IO::VectorSetSaveB(m_matchesBufferEnft, fp);
        const uint nFtrsSift = nFtrsSiftBuffer[iBuffer];
        const uint nPixelsFtr = nFtrsSift, nPixelsDesc = (nPixelsFtr << 4);
        m_ftrExtractorSift.GetFeatureTexture(iBuffer).SaveB(fp, nPixelsFtr);
        m_ftrExtractorSift.GetDescriptorTexture(iBuffer).SaveB(fp, nPixelsDesc);
        m_ftrTrackerEnft.GetImageTexture(iBuffer).SaveB(fp);
    }
    m_FBuffer.SaveB(fp);
    fclose(fp);
    printf("Saved \'%s\'\n", fileName);
}

void FeatureTracker::LoadB(const char *fileName, Sequence &seq, FrameIndex &iFrmKey, FrameIndex &iFrmLast, FrameIndex &iFrmCurrent) {
    FILE *fp = fopen( fileName, "rb");
    seq.LoadB(fp);
    fread(&iFrmKey, sizeof(FrameIndex), 1, fp);
    fread(&iFrmLast, sizeof(FrameIndex), 1, fp);
    fread(&iFrmCurrent, sizeof(FrameIndex), 1, fp);
    m_bufferManager.LoadB(fp);
    std::vector<uint> &nFtrsSiftBuffer = m_ftrExtractorSift.GetFeaturesNumberBuffer();
    IO::VectorLoadB(nFtrsSiftBuffer, fp);
    for(ushort iBuffer = 0; iBuffer < m_bufferSize; ++iBuffer) {
        m_ftrsBuffer[iBuffer].LoadB(fp);
        m_descsBuffer[iBuffer].LoadB(fp);
        IO::VectorSetLoadB(m_matchesBuffer, fp);
        IO::VectorSetLoadB(m_matchesBufferEnft, fp);
        const uint nFtrsSift = nFtrsSiftBuffer[iBuffer];
        const uint nPixelsFtr = nFtrsSift, nPixelsDesc = (nPixelsFtr << 4);
        m_ftrExtractorSift.GetFeatureTexture(iBuffer).LoadB(fp, nPixelsFtr);
        m_ftrExtractorSift.GetDescriptorTexture(iBuffer).LoadB(fp, nPixelsDesc);
        m_ftrTrackerEnft.GetImageTexture(iBuffer).LoadB(fp);
    }
    m_FBuffer.LoadB(fp);
    fclose(fp);
    printf("Loaded \'%s\'\n", fileName);
}