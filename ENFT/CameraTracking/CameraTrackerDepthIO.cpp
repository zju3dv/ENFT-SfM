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
#include "CameraTrackerDepth.h"

using namespace ENFT_SfM;

void CameraTrackerDepth::ViewSequence(const Sequence &seq, const FrameIndex &iFrm) {
    if(!m_view)
        return;
    ProgramGL::UnbindFrameBuffer();
    ViewerSequenceDepth::Initialize(seq, iFrm);
    ViewerSequenceDepth::Resize(ViewerSequenceDepth::m_pWnd->size());
    ViewerSequenceDepth::Draw();
    ViewerSequenceDepth::DrawString();
    ViewerSequenceDepth::m_pWnd->swap_buffers();
    ViewerSequenceDepth::m_pWnd->handle_events(ViewerSequenceDepth::m_handler);
    while(!ViewerSequenceDepth::m_handler.Quit() && m_stop) {
        ViewerSequenceDepth::Draw();
        ViewerSequenceDepth::DrawString();
        ViewerSequenceDepth::m_pWnd->swap_buffers();
        ViewerSequenceDepth::m_pWnd->handle_events(ViewerSequenceDepth::m_handler);
    }
    //{
    //  ViewerSequenceDepth::m_pWnd->swap_buffers();
    //  CVD::Image<CVD::Rgba<ubyte> > img(ViewerSequenceDepth::m_pWnd->size());
    //  glReadPixels(0, 0, ViewerSequenceDepth::m_pWnd->size().x, ViewerSequenceDepth::m_pWnd->size().y, GL_RGBA, GL_UNSIGNED_BYTE, img.data());
    //  char fileName[128];
    //  sprintf(fileName, "F:/tmp/sfm/%04d.bmp", m_idx++);
    //  CVD::flipVertical(img);
    //  CVD::img_save(img, fileName);
    //}
    ProgramGL::BindFrameBuffer();
}

bool CameraTrackerDepth::OnKeyDown(const int key) {
    switch(key) {
        case KEY_STOP_CAMERA_TRACKING:
            m_stop = !m_stop;
            return true;
    }
    return ViewerSequenceDepth::OnKeyDown(key);
}