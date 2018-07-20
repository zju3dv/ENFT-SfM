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
#include "ViewerSequenceDepth.h"

void ViewerSequenceDepth::Initialize(const Sequence &seq,
                                     const FrameIndex iFrmActive) {
    ProgramGL::Initialize(seq.GetImageWidth(), seq.GetImageHeight());
    Viewer::Initialize();
    if(m_pSeq) {
        m_pSeq = &seq;
        PrepareActiveFrame(iFrmActive);
    } else {
        m_depView = false;
        m_iFrmDep = m_iFrmDepScaled = m_iFrmLocalCoord = INVALID_FRAME_INDEX;
        m_texDep.Generate(seq.GetImageWidth(), seq.GetImageHeight());
        m_texDepScaled.Generate(seq.GetImageWidth(), seq.GetImageHeight());
        m_texLocalCoord.Generate(seq.GetImageWidth(), seq.GetImageHeight());
        m_programScale.Initialize();
        m_programComputeLocalCoordinate.Initialize(seq.GetIntrinsicMatrix());
        m_depScale = DEPTH_SCALE_INITIAL;
        m_modelActive.Initialize();
        ViewerSequence::Initialize(seq, iFrmActive);
    }
}

bool ViewerSequenceDepth::PrepareImageView(const FrameIndex iFrmImg) {
    if(!m_depView)
        return ViewerSequence::PrepareImageView(iFrmImg);
    else if(iFrmImg == m_iFrmDepScaled) {
        //m_texDep.Bind();
        m_texDepScaled.Bind();
        return false;
    } else {
        LoadDepthTexture(iFrmImg);
        ScaleDepthTexture();
        return true;
    }
}

bool ViewerSequenceDepth::PrepareSceneView(const FrameIndex iFrmScn) {
    if(!m_depView)
        return ViewerSequence::PrepareSceneView(iFrmScn);
    else if(iFrmScn == m_modelActive.GetFrameIndex())
        return false;
    else if(m_pSeq->IsFrameSolved(iFrmScn)) {
        LoadImageTexture(iFrmScn);
        LoadDepthTexture(iFrmScn);
        ComputeLocalCoordinate(iFrmScn);
        m_modelActive.Set(iFrmScn, m_pSeq->GetCamera(iFrmScn), m_img, m_imgLocalCoord);
    }
    return true;
}

void ViewerSequenceDepth::LoadDepthTexture(const FrameIndex iFrmDep) {
    if(m_iFrmDep == iFrmDep)
        return;
    m_iFrmDep = iFrmDep;
    if(((SequenceDepth *) m_pSeq)->GetDepthFileName(m_iFrmDep) != "") {
        
        FILE *fp = fopen( ((SequenceDepth *)m_pSeq)->GetDepthFileName(m_iFrmDep).c_str(),
                "rb");
        int width, height;
        fread(&height, sizeof(int), 1, fp);
        fread(&width, sizeof(int), 1, fp);
        m_imgDep.resize(CVD::ImageRef(width, height));
        fread(m_imgDep.data(), sizeof(float), width * height, fp);
        fclose(fp);

        m_texDep.Bind();
        m_texDep.Resize(m_imgDep.size().x, m_imgDep.size().y);
        m_texDep.UploadFromCPU(m_imgDep.data());
    }
}

void ViewerSequenceDepth::ScaleDepthTexture() {
    if(m_iFrmDepScaled == m_iFrmDep)
        return;
    m_iFrmDepScaled = m_iFrmDep;
    const CVD::ImageRef sizeBkp = m_pWnd->size();
    ProgramGL::BindFrameBuffer();
    ProgramGL::FitViewportGL(m_texDep);
    m_programScale.Run(m_depScale, m_texDep, m_texDepScaled);
    ProgramGL::FitViewportWindows(sizeBkp.x, sizeBkp.y);
    ProgramGL::UnbindFrameBuffer();
    m_texDepScaled.Bind();
}

void ViewerSequenceDepth::ComputeLocalCoordinate(const FrameIndex
        iFrmLocalCoord) {
    if(m_iFrmLocalCoord == iFrmLocalCoord)
        return;
    m_iFrmLocalCoord = iFrmLocalCoord;
    //glMatrixMode(GL_PROJECTION);
    //glPushMatrix();
    //glMatrixMode(GL_MODELVIEW);
    //glPushMatrix();
    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    const CVD::ImageRef sizeBkp = m_pWnd->size();
    ProgramGL::BindFrameBuffer();
    ProgramGL::FitViewportGL(m_texLocalCoord);
    m_programComputeLocalCoordinate.Run(m_texDep, m_texLocalCoord);
    m_imgLocalCoord.resize(m_imgDep.size());
    m_texLocalCoord.DownloadToCPU((float *) m_imgLocalCoord.data());
    ProgramGL::FitViewportWindows(sizeBkp.x, sizeBkp.y);
    ProgramGL::UnbindFrameBuffer();
    glDisable(GL_TEXTURE_RECTANGLE_ARB);
    //glMatrixMode(GL_PROJECTION);
    //glPopMatrix();
    //glMatrixMode(GL_MODELVIEW);
    //glPopMatrix();
}

void ViewerSequenceDepth::DrawFrameImageFileName() {
    glColor3ub(STRING_COLOR_R, STRING_COLOR_G, STRING_COLOR_B);
    if(m_depView)
        Viewer::DrawStringTopLeftLarge("%s",
                                       ((SequenceDepth *) m_pSeq)->GetDepthFileName(m_iFrmActive).c_str());
    else
        Viewer::DrawStringTopLeftLarge("%s",
                                       m_pSeq->GetImageFileName(m_iFrmActive).c_str());
}

void ViewerSequenceDepth::DrawSceneView() {
    if(!m_depView) {
        ViewerSequence::DrawSceneView();
        return;
    }

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixd(m_projMatrix);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixd(m_modelMatrix);

    ApplySceneViewTransformation();

    glEnable(GL_DEPTH_TEST);
    const FrameIndex nModels = FrameIndex(m_models.size());
    for(FrameIndex i = 0; i < nModels; ++i)
        m_models[i].Draw();
    if(m_modelActive.IsValid())
        m_modelActive.Draw();
    glDisable(GL_DEPTH_TEST);

    if(m_drawCamType != DRAW_NO_CAMERA)
        DrawSceneCameras();
    if(m_drawPtTypes[m_imgView] != DRAW_NO_POINT)
        DrawScenePoints();
    //if(m_drawCamType != DRAW_NO_CAMERA)
    //  DrawSceneCameras();
    if(m_iTrkSelected != INVALID_TRACK_INDEX &&
            (m_pSeq->GetTrackState(m_iTrkSelected) & FLAG_TRACK_STATE_SOLVED))
        DrawSelectedScenePoint();
    if(m_drawActivePts)
        DrawActiveScenePoints();

    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

}

void ViewerSequenceDepth::DrawActiveScenePoints() {
    ViewerSequence::DrawActiveScenePoints();
    if(!m_drawErrs ||
            !(m_pSeq->GetFrameState(m_iFrmActive) & FLAG_FRAME_STATE_SOLVED))
        return;
    const SequenceDepth *pSeq = (SequenceDepth *) m_pSeq;
    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    glBegin(GL_LINES);
    glColor3ub(COLOR_FEATURE_ERROR_LINE_R, COLOR_FEATURE_ERROR_LINE_G,
               COLOR_FEATURE_ERROR_LINE_B);
    TrackIndex iTrk;
    Point2D x;
    Point3D X;
    const Camera &C = m_pSeq->GetCamera(m_iFrmActive);
    const MeasurementIndex iMea1 = m_pSeq->GetFrameFirstMeasurementIndex(
                                       m_iFrmActive), iMea2 = m_pSeq->GetFrameFirstMeasurementIndex(m_iFrmActive + 1);
    for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
        if((iTrk = m_pSeq->GetMeasurementTrackIndex(iMea)) == INVALID_TRACK_INDEX ||
                !(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_SOLVED)
                || (X.Z() = pSeq->GetDepth(iMea)) == 0.0f)
            continue;
        x = m_pSeq->GetMeasurement(iMea);
        if(!m_pSeq->AreMeasurementsNormalized())
            K.ImageToNormalizedPlane(x);
        X.X() = x.x() * X.Z();
        X.Y() = x.y() * X.Z();
        C.ApplyInversely(X);
        glVertex3fv(X);
        glVertex3fv(m_pSeq->GetPoint(iTrk));
    }
    glEnd();
}

bool ViewerSequenceDepth::IsMeasurementOutlier(const MeasurementIndex &iMea) {
    if(m_imgView && m_depView || !m_imgView && m_drawErrs)
        return ViewerSequence::IsMeasurementOutlier(iMea) ||
               (m_pSeq->GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH);
    else
        return ViewerSequence::IsMeasurementOutlier(iMea);
}

bool ViewerSequenceDepth::ShouldMeasurementBeHidden(const MeasurementIndex
        &iMea) {
    if(m_imgView && m_depView || !m_imgView && m_drawErrs)
        return ViewerSequence::ShouldMeasurementBeHidden(iMea) ||
               ((SequenceDepth *) m_pSeq)->GetDepth(iMea) == 0.0f;
    else
        return ViewerSequence::ShouldMeasurementBeHidden(iMea);
}

bool ViewerSequenceDepth::OnKeyDown(const int key) {
    if(ViewerSequence::OnKeyDown(key))
        return true;
    switch(key) {
        case KEY_DEPTH_VIEW:
            m_depView = !m_depView;
            if(m_imgView)
                PrepareImageView(m_iFrmActive);
            else
                PrepareSceneView(m_iFrmActive);
            return true;
        case KEY_DEPTH_SCALE_INCREASE:
        case KEY_DEPTH_SCALE_DECREASE: {
            if(!m_imgView || !m_depView)
                return false;
            if(key == KEY_DEPTH_SCALE_INCREASE) {
                if(m_depScale >= 1.0f)
                    ++m_depScale;
                else
                    m_depScale = 1 / float(int(1 / m_depScale + 0.5f) - 1);
            } else {
                if(m_depScale <= 1.0f)
                    m_depScale = 1 / float(int(1 / m_depScale + 0.5f) + 1);
                else
                    --m_depScale;
            }
            m_iFrmDepScaled = INVALID_FRAME_INDEX;
            ScaleDepthTexture();
            return true;
        }
        case KEY_UPDATE_MODEL: {
            if(m_imgView || !m_depView)
                return true;
            FrameIndex i;
            const FrameIndex nModels = FrameIndex(m_models.size());
            for(i = 0; i < nModels &&
                    m_models[i].GetFrameIndex() != m_modelActive.GetFrameIndex(); ++i);
            if(i == nModels) {
                m_models.push_back(m_modelActive);
                m_modelActive.Validate();
            } else {
                m_models.erase(m_models.begin() + i);
                m_modelActive.Invalidate();
            }
            printf("----------------------------------------------------------------\n");
            printf("Models = {");
            const ushort nModelsNew = FrameIndex(m_models.size());
            for(i = 0; i < nModelsNew; ++i)
                printf(" %d", m_models[i].GetFrameIndex());
            printf(" }\n");
            return true;
        }
        case KEY_UPDATE_MODEL_KEY_FRAME: {
            if(m_imgView || !m_depView)
                return true;
            const FrameIndex nModels = FrameIndex(m_models.size());
            if(nModels == m_pSeq->CountFrames(FLAG_FRAME_STATE_KEY_FRAME))
                m_models.resize(0);
            else {
                FrameIndex i, iFrm;
                const FrameIndex nFrms = m_pSeq->GetFramesNumber();
                for(iFrm = 0; iFrm < nFrms; ++iFrm) {
                    printf("\rInserting keyframe models %d%%...", iFrm * 100 / nFrms);
                    if(!m_pSeq->IsFrameSolved(iFrm) || !m_pSeq->IsFrameKeyFrame(iFrm))
                        continue;
                    for(i = 0; i < nModels && m_models[i].GetFrameIndex() != iFrm; ++i);
                    if(i != nModels)
                        continue;
                    LoadImageTexture(iFrm);
                    LoadDepthTexture(iFrm);
                    ComputeLocalCoordinate(iFrm);
                    m_models.push_back(Model(iFrm, m_pSeq->GetCamera(iFrm), m_img,
                                             m_imgLocalCoord));
                }
                printf("\rInserting keyframe models %d%%...\n", 100);
            }
            return true;
        }
    }
    return false;
}