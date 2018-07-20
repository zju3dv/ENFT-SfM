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
#include "ViewerSequence.h"
#include "ProgramGL/ProgramGLUtility.h"
#include <cvd/image_io.h>

ViewerSequence::ViewerSequence() : m_pSeq(NULL),
    m_camVolume(*(CameraVolume *) _aligned_malloc(sizeof(CameraVolume),
                SSE_ALIGNMENT)),
    m_Cg(*(Camera *) _aligned_malloc(sizeof(Camera), SSE_ALIGNMENT)),
    m_translationStart(*(Point3D *) _aligned_malloc(sizeof(Point3D),
                       SSE_ALIGNMENT)),
    m_translation(*(Point3D *) _aligned_malloc(sizeof(Point3D), SSE_ALIGNMENT)),
    m_whereMouseDown3DStart(*(Point3D *) _aligned_malloc(sizeof(Point3D),
                            SSE_ALIGNMENT)),
    m_arcball(*(Arcball *) _aligned_malloc(sizeof(Arcball), SSE_ALIGNMENT)) {}

ViewerSequence::~ViewerSequence() {
    _aligned_free(&m_camVolume);
    _aligned_free(&m_Cg);
    _aligned_free(&m_translationStart);
    _aligned_free(&m_translation);
    _aligned_free(&m_whereMouseDown3DStart);
    _aligned_free(&m_arcball);
}

void ViewerSequence::Run(const Sequence &seq, const FrameIndex iFrmActive) {
    ProgramGL::UnbindFrameBuffer();
    Initialize(seq, iFrmActive);
    Resize(m_pWnd->size());
    m_handler.Quit() = false;
    while(!m_handler.Quit()) {
        Draw();
        DrawString();
        m_pWnd->swap_buffers();
        m_pWnd->handle_events(m_handler);
    }
    ProgramGL::BindFrameBuffer();
}

void ViewerSequence::Initialize(const Sequence &seq,
                                const FrameIndex iFrmActive) {
    ProgramGL::Initialize(seq.GetImageWidth(), seq.GetImageHeight());
    Viewer::Initialize();
    if(m_pSeq) {
        m_pSeq = &seq;
        PrepareActiveFrame(iFrmActive);
    } else {
        m_pSeq = &seq;
        m_minTrkLen = 0;
        m_imgView = true;
        m_iFrmActive = m_iFrmImg = m_iFrmScn = INVALID_FRAME_INDEX;
        m_iFtrSelected = INVALID_FEATURE_INDEX;
        m_iTrkSelected = INVALID_TRACK_INDEX;

        //////////////////////////////////////////////////////////////////////////
        // VIEW_FRAME
        //////////////////////////////////////////////////////////////////////////
        m_texImg.Generate(seq.GetImageWidth(), seq.GetImageHeight());
        ProgramGL::FitViewportGL(m_texImg);
        ProgramGL::BindFrameBuffer();
        ProgramGLClearColorDepth programClearColorDepth;
        programClearColorDepth.Initialize();
        programClearColorDepth.Run(m_texImg);
        ProgramGL::UnbindFrameBuffer();
        m_drawPtTypes[0] = m_drawPtTypes[1] = DRAW_ALL_POINTS;
        m_drawTrkType = DRAW_NO_TRACK;
        m_drawProjType = DRAW_ACTIVE_PROJECTIONS;
        PrepareActiveFrame(iFrmActive);
        //m_pWnd->swap_buffers();
        m_drawErrs = false;
        m_drawDistribution = false;
        //glShadeModel(GL_FLAT);
        glShadeModel(GL_SMOOTH);

        //////////////////////////////////////////////////////////////////////////
        // VIEW_SCENE
        //////////////////////////////////////////////////////////////////////////
        m_controlType = CONTROL_TRANSLATE_Z;
        m_drawArcball = false;
        m_drawCamType = DRAW_PATH_AND_ACTIVE_FRAME;
        m_drawActivePts = true;
        m_backgroundDark = true;
        m_camVolumeScale = 1.0f;
        m_camPathWidth = 1.0f;

        m_viewport[0] = m_viewport[1] = 0;
        ResetSceneViewProjectionMatrix();
        ResetSceneViewModelViewMatrix();
        ResetSceneViewTransformation();
    }
}

bool ViewerSequence::PrepareView(const bool imgView) {
    if(m_imgView == imgView)
        return false;
    m_imgView = imgView;
    if(m_imgView)
        PrepareImageView(m_iFrmActive);
    else
        PrepareSceneView(m_iFrmActive);
    return true;
}

bool ViewerSequence::PrepareActiveFrame(const FrameIndex iFrmActive) {
    if(iFrmActive == m_iFrmActive || iFrmActive >= m_pSeq->GetFramesNumber())
        return false;
    float SSE;
    m_iFrmActive = iFrmActive;
    PrepareSelectedFeature(m_iTrkSelected == INVALID_TRACK_INDEX ?
                           INVALID_FEATURE_INDEX : m_pSeq->SearchTrackForFrameFeatureIndex(m_iTrkSelected,
                                   m_iFrmActive));
    if(IsFrameSolved(m_iFrmActive)) {
        m_pSeq->ComputeFrameInlierRatioAndMSE(m_iFrmActive, m_nInliers, m_inlierRatio,
                                              SSE, m_MSE);

        m_pSeq->GetFrameTrackedFeatureIndexList(m_iFrmActive, m_iFtrs);
        m_pSeq->ComputeFrameFeaturesDistribution(m_iFrmActive, m_iFtrs, m_meanFtrsTrked,
                m_covFtrsTrked);
        m_pSeq->GetFrameInlierFeatureIndexList(m_iFrmActive, m_iFtrs);
        m_pSeq->ComputeFrameFeaturesDistribution(m_iFrmActive, m_iFtrs,
                m_meanFtrsInlier, m_covFtrsInlier);
        m_pSeq->GetFrameOutlierFeatureIndexList(m_iFrmActive, m_iFtrs);
        m_pSeq->ComputeFrameFeaturesDistribution(m_iFrmActive, m_iFtrs,
                m_meanFtrsOutlier, m_covFtrsOutlier);

        m_inlierRatioArea = sqrt((m_covFtrsInlier.v0() * m_covFtrsInlier.v2() -
                                  m_covFtrsInlier.v1() * m_covFtrsInlier.v1())
                                 / (m_covFtrsTrked.v0() * m_covFtrsTrked.v2() - m_covFtrsTrked.v1() *
                                    m_covFtrsTrked.v1()));
    }
    if(m_imgView)
        PrepareImageView(iFrmActive);
    else
        PrepareSceneView(iFrmActive);
    return true;
}

bool ViewerSequence::PrepareImageView(const FrameIndex iFrmImg) {
    if(iFrmImg == m_iFrmImg) {
        m_texImg.Bind();
        return false;
    }
    LoadImageTexture(iFrmImg);
    Resize(m_pWnd->size());
    Draw();
    DrawString();
    m_pWnd->swap_buffers();
    return true;
}

bool ViewerSequence::PrepareSceneView(const FrameIndex iFrmScn) {
    if(iFrmScn == m_iFrmScn)
        return false;
    m_iFrmScn = iFrmScn;
    return true;
}

bool ViewerSequence::PrepareSelectedTrack(const TrackIndex iTrkSelected) {
    if(iTrkSelected == m_iTrkSelected)
        return false;
    m_iTrkSelected = iTrkSelected;
    if(m_iTrkSelected == INVALID_TRACK_INDEX)
        PrepareSelectedFeature(INVALID_FEATURE_INDEX);
    else {
        m_pSeq->PrintTrack(m_iTrkSelected);
        if(m_iFtrSelected == INVALID_FEATURE_INDEX ||
                m_pSeq->GetFrameFeatureTrackIndex(m_iFrmActive,
                        m_iFtrSelected) != m_iTrkSelected)
            PrepareSelectedFeature(m_pSeq->SearchTrackForFrameFeatureIndex(m_iTrkSelected,
                                   m_iFrmActive));
    }
    return true;
}

bool ViewerSequence::PrepareSelectedFeature(const FeatureIndex iFtrSelected) {
    if(iFtrSelected == m_iFtrSelected)
        return false;
    m_iFtrSelected = iFtrSelected;
    //if(m_iFtrSelected == INVALID_FEATURE_INDEX)
    //  PrepareSelectedTrack(INVALID_TRACK_INDEX);
    //else
    //{
    //  m_pSeq->PrintFrameFeature(m_iFrmActive, m_iFtrSelected);
    //  PrepareSelectedTrack(m_pSeq->GetFrameFeatureTrackIndex(m_iFrmActive, m_iFtrSelected));
    //}
    if(m_iFtrSelected != INVALID_FEATURE_INDEX) {
        PrepareSelectedTrack(m_pSeq->GetFrameFeatureTrackIndex(m_iFrmActive,
                             m_iFtrSelected));
        m_pSeq->PrintFrameFeature(m_iFrmActive, m_iFtrSelected);
    }
    return true;
}

template<typename TYPE>
static inline void LoadImage(CVD::Image<TYPE> &img1, CVD::Image<TYPE> &img2,
                             const std::string &fileName) {
    CVD::img_load(img1, fileName);
    if(img1.size().x % 4 == 0)
        img2 = img1;
    else {
        const int width = (img1.size().x + 3) & (~3), height = img1.size().y;
        img2.resize(CVD::ImageRef(width, height));
        img2.zero();
        for(int y = 0; y < height; ++y)
            memcpy(img2[y], img1[y], sizeof(TYPE) * img1.row_stride());
    }
}

void ViewerSequence::LoadImageTexture(const FrameIndex iFrmImg) {
    if(m_iFrmImg == iFrmImg)
        return;
    m_iFrmImg = iFrmImg;
    if(m_pSeq->GetImageFileName(m_iFrmImg) != "") {
        //CVD::img_load(m_img, m_pSeq->GetImageFileName(m_iFrmImg));
        ::LoadImage(m_imgTmp, m_img, m_pSeq->GetImageFileName(m_iFrmImg));
        m_texImg.Bind();
        m_texImg.Resize(m_img.row_stride(), m_img.size().y);
        m_texImg.UploadFromCPU((ubyte *) m_img.data());
    }
}

void ViewerSequence::DrawImageView() {
    Viewer::DrawTexture(m_texImg.GetWidth(), m_texImg.GetHeight());

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    ApplyImageViewTransformation();

    if(m_drawPtTypes[m_imgView] != DRAW_NO_POINT)
        DrawFeaturePoints();
    if(m_drawTrkType != DRAW_NO_TRACK)
        DrawFeatureTracks();
    if(m_drawDistribution)
        DrawFeatureDistribution();
    if(m_iFtrSelected != INVALID_FEATURE_INDEX)
        DrawSelectedFeature();
    if(IsFrameSolved(m_iFrmActive)) {
        switch(m_drawProjType) {
            case DRAW_NO_PROJECTION:
                break;
            case DRAW_ALL_PROJECTIONS:
                DrawAllProjections();
                DrawActiveProjections();
                break;
            case DRAW_ACTIVE_PROJECTIONS:
                DrawActiveProjections();
                break;
        }
    }

    glPopMatrix();
}

void ViewerSequence::DrawFeaturePoints() {
    TrackIndex iTrk;
    Point2D x;
    glPointSize(2);
    glBegin(GL_LINES);
    const bool camSolved = IsFrameSolved(m_iFrmActive);
    const Point2D crossSize(FEATURE_CROSS_SIZE * m_factorWinToImg.x(),
                            FEATURE_CROSS_SIZE * m_factorWinToImg.y());
    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    const MeasurementIndex iMea1 = m_pSeq->GetFrameFirstMeasurementIndex(
                                       m_iFrmActive), iMea2 = m_pSeq->GetFrameFirstMeasurementIndex(m_iFrmActive + 1);
    for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
        if(ShouldMeasurementBeHidden(iMea))
            continue;
        else if((iTrk = m_pSeq->GetMeasurementTrackIndex(iMea)) == INVALID_TRACK_INDEX
                || m_pSeq->GetTrackLength(iTrk) == 1)
            glColor3ub(COLOR_FEATURE_SINGLE_R, COLOR_FEATURE_SINGLE_G,
                       COLOR_FEATURE_SINGLE_B);
        else if(camSolved && IsMeasurementOutlier(iMea))
            glColor3ub(COLOR_FEATURE_OUTLIER_R, COLOR_FEATURE_OUTLIER_G,
                       COLOR_FEATURE_OUTLIER_B);
        else if(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_COMMON)
            glColor3ub(COLOR_FEATURE_COMMON_TRACK_R, COLOR_FEATURE_COMMON_TRACK_G,
                       COLOR_FEATURE_COMMON_TRACK_B);
        else if(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_MERGED)
            glColor3ub(COLOR_FEATURE_MERGED_TRACK_R, COLOR_FEATURE_MERGED_TRACK_G,
                       COLOR_FEATURE_MERGED_TRACK_B);
        else if(m_pSeq->GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_ENFT)
            glColor3ub(COLOR_FEATURE_ENFT_R, COLOR_FEATURE_ENFT_G, COLOR_FEATURE_ENFT_B);
        else
            glColor3ub(COLOR_FEATURE_SIFT_R, COLOR_FEATURE_SIFT_G, COLOR_FEATURE_SIFT_B);
        if(m_pSeq->AreMeasurementsNormalized())
            K.NormalizedPlaneToImage(m_pSeq->GetMeasurement(iMea), x);
        else
            x = m_pSeq->GetMeasurement(iMea);
        Viewer::DrawCross(x.x(), x.y(), crossSize);
    }
    glEnd();
    glPointSize(1);
}

void ViewerSequence::DrawFeatureTracks() {
    const FrameIndex nFrms = m_pSeq->GetFramesNumber();
    if(m_drawTrkType != DRAW_TRACK_TO_NEXT_KEY_FRAME && m_iFrmActive == 0 ||
            m_drawTrkType == DRAW_TRACK_TO_NEXT_KEY_FRAME && m_iFrmActive == nFrms - 1)
        return;
    glBegin(GL_LINES);
    glColor3ub(COLOR_TRACK_TEMPORAL_R, COLOR_TRACK_TEMPORAL_G,
               COLOR_TRACK_TEMPORAL_B);

    FrameIndex iFrmOther;
    switch(m_drawTrkType) {
        case DRAW_TRACK_FROM_LAST_FRAME:
            iFrmOther = m_iFrmActive - 1;
            break;
        case DRAW_TRACK_FROM_LAST_KEY_FRAME:
            for(iFrmOther = m_iFrmActive - 1; iFrmOther > 0 &&
                    !(m_pSeq->GetFrameState(iFrmOther) & FLAG_FRAME_STATE_KEY_FRAME); --iFrmOther);
            break;
        case DRAW_TRACK_TO_NEXT_KEY_FRAME:
            for(iFrmOther = m_iFrmActive + 1; iFrmOther < nFrms &&
                    !(m_pSeq->GetFrameState(iFrmOther) & FLAG_FRAME_STATE_KEY_FRAME); ++iFrmOther);
            break;
    }

    TrackIndex iTrk;
    MeasurementIndex iMeaOther;
    Point2D x;
    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    const MeasurementIndex iMea1 = m_pSeq->GetFrameFirstMeasurementIndex(
                                       m_iFrmActive), iMea2 = m_pSeq->GetFrameFirstMeasurementIndex(m_iFrmActive + 1);
    for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
        if(ShouldMeasurementBeHidden(iMea) ||
                (iTrk = m_pSeq->GetMeasurementTrackIndex(iMea)) == INVALID_TRACK_INDEX
                || (iMeaOther = m_pSeq->SearchTrackForFrameMeasurementIndex(iTrk,
                                iFrmOther)) == INVALID_MEASUREMENT_INDEX)
            continue;
        else if(m_pSeq->AreMeasurementsNormalized()) {
            K.NormalizedPlaneToImage(m_pSeq->GetMeasurement(iMeaOther), x);
            glVertex2fv(x);
            K.NormalizedPlaneToImage(m_pSeq->GetMeasurement(iMea), x);
            glVertex2fv(x);
        } else {
            glVertex2fv(m_pSeq->GetMeasurement(iMeaOther));
            glVertex2fv(m_pSeq->GetMeasurement(iMea));
        }
    }

    glEnd();
}

void ViewerSequence::DrawFeatureDistribution() {
    glLineWidth(2);

    glBegin(GL_LINE_LOOP);
    glColor3ub(COLOR_FEATURE_ELLIPSE_TRACKED_R, COLOR_FEATURE_ELLIPSE_TRACKED_G,
               COLOR_FEATURE_ELLIPSE_TRACKED_B);
    Viewer::ConvertMeanCovarianceToEllipse(m_meanFtrsTrked, m_covFtrsTrked,
                                           m_ellipse);
    Viewer::DrawPoints(m_ellipse);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor3ub(COLOR_FEATURE_ELLIPSE_INLIER_R, COLOR_FEATURE_ELLIPSE_INLIER_G,
               COLOR_FEATURE_ELLIPSE_INLIER_B);
    Viewer::ConvertMeanCovarianceToEllipse(m_meanFtrsInlier, m_covFtrsInlier,
                                           m_ellipse);
    Viewer::DrawPoints(m_ellipse);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor3ub(COLOR_FEATURE_ELLIPSE_OUTLIER_R, COLOR_FEATURE_ELLIPSE_OUTLIER_G,
               COLOR_FEATURE_ELLIPSE_OUTLIER_B);
    Viewer::ConvertMeanCovarianceToEllipse(m_meanFtrsOutlier, m_covFtrsOutlier,
                                           m_ellipse);
    Viewer::DrawPoints(m_ellipse);
    glEnd();

    glLineWidth(1);
}

void ViewerSequence::DrawSelectedFeature() {
    glColor3ub(COLOR_FEATURE_SELECTION_BOX_R, COLOR_FEATURE_SELECTION_BOX_G,
               COLOR_FEATURE_SELECTION_BOX_B);
    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    Point2D x1 = m_pSeq->GetFrameFeature(m_iFrmActive, m_iFtrSelected);
    if(m_pSeq->AreMeasurementsNormalized())
        K.NormalizedPlaneToImage(x1);
    if(IsFrameSolved(m_iFrmActive) && m_iTrkSelected != INVALID_TRACK_INDEX &&
            (m_pSeq->GetTrackState(m_iTrkSelected) & FLAG_TRACK_STATE_SOLVED)) {
        Point2D x2;
        m_pSeq->GetCamera(m_iFrmActive).ProjectToNormalizedPlane(m_pSeq->GetPoint(
                    m_iTrkSelected), x2);
        if(m_pSeq->GetIntrinsicType() == Sequence::INTRINSIC_CONSTANT)
            x2 *= m_pSeq->GetIntrinsicRectification().f();
        else if(m_pSeq->GetIntrinsicType() == Sequence::INTRINSIC_VARIABLE)
            x2 *= m_pSeq->GetIntrinsicRectification(m_iFrmActive).f();
        K.NormalizedPlaneToImage(x2);
        glBegin(GL_LINES);
        glVertex2fv(x1);
        glVertex2fv(x2);
        glEnd();
    }
    glLineWidth(2);
    glBegin(GL_LINES);
    const Point2D boxSize(FEATURE_SELECTION_BOX_SIZE * m_factorWinToImg.x(),
                          FEATURE_SELECTION_BOX_SIZE * m_factorWinToImg.y());
    Viewer::DrawBox(x1.x(), x1.y(), boxSize);
    glEnd();
    glLineWidth(1);
}

void ViewerSequence::DrawActiveProjections() {
    glPointSize(2);

    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    const Sequence::IntrinsicType intrinsicType = m_pSeq->GetIntrinsicType();
    const Camera &C = m_pSeq->GetCamera(m_iFrmActive);

    TrackIndex iTrk;
    Point2D x;
    const MeasurementIndex iMea1 = m_pSeq->GetFrameFirstMeasurementIndex(
                                       m_iFrmActive), iMea2 = m_pSeq->GetFrameFirstMeasurementIndex(m_iFrmActive + 1);
    for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
        if((iTrk = m_pSeq->GetMeasurementTrackIndex(iMea)) == INVALID_TRACK_INDEX ||
                !(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_SOLVED)
                || ShouldMeasurementBeHidden(iMea))
            continue;
        if(m_drawErrs) {
            glBegin(GL_LINES);
            glColor3ub(COLOR_FEATURE_ERROR_LINE_R, COLOR_FEATURE_ERROR_LINE_G,
                       COLOR_FEATURE_ERROR_LINE_B);
            if(m_pSeq->AreMeasurementsNormalized())
                K.NormalizedPlaneToImage(m_pSeq->GetMeasurement(iMea), x);
            else
                x = m_pSeq->GetMeasurement(iMea);
            glVertex2fv(x);
            C.ProjectToNormalizedPlane(m_pSeq->GetPoint(iTrk), x);
            if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
                x *= m_pSeq->GetIntrinsicRectification().f();
            else if(intrinsicType == Sequence::INTRINSIC_VARIABLE)
                x *= m_pSeq->GetIntrinsicRectification(m_iFrmActive).f();
            K.NormalizedPlaneToImage(x);
            glVertex2fv(x);
            glEnd();

            glBegin(GL_POINTS);
            glColor3ub(COLOR_PROJECTION_ACTIVE_R, COLOR_PROJECTION_ACTIVE_G,
                       COLOR_PROJECTION_ACTIVE_B);
            glVertex2fv(x);
            glEnd();
        } else {
            C.ProjectToNormalizedPlane(m_pSeq->GetPoint(iTrk), x);
            if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
                x *= m_pSeq->GetIntrinsicRectification().f();
            else if(intrinsicType == Sequence::INTRINSIC_VARIABLE)
                x *= m_pSeq->GetIntrinsicRectification(m_iFrmActive).f();
            K.NormalizedPlaneToImage(x);
            glBegin(GL_POINTS);
            glColor3ub(COLOR_PROJECTION_ACTIVE_R, COLOR_PROJECTION_ACTIVE_G,
                       COLOR_PROJECTION_ACTIVE_B);
            glVertex2fv(x);
            glEnd();
        }
    }

    glPointSize(1);
}

void ViewerSequence::DrawAllProjections() {
    glPointSize(2);
    glBegin(GL_POINTS);
    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    const Sequence::IntrinsicType intrinsicType = m_pSeq->GetIntrinsicType();
    const Camera &C = m_pSeq->GetCamera(m_iFrmActive);

    Point3D Xc;
    Point2D x;
    const TrackIndex nTrks = m_pSeq->GetTracksNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(!(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_SOLVED) ||
                ShouldTrackBeHidden(iTrk))
            continue;
        C.ProjectToNormalizedPlane(m_pSeq->GetPoint(iTrk), Xc, x);
        if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
            x *= m_pSeq->GetIntrinsicRectification().f();
        else if(intrinsicType == Sequence::INTRINSIC_VARIABLE)
            x *= m_pSeq->GetIntrinsicRectification(m_iFrmActive).f();
        K.NormalizedPlaneToImage(x);
        if(Xc.Z() > 0)
            glColor3ub(COLOR_PROJECTION_FRONT_R, COLOR_PROJECTION_FRONT_G,
                       COLOR_PROJECTION_FRONT_B);
        else
            glColor3ub(COLOR_PROJECTION_BACK_R, COLOR_PROJECTION_BACK_G,
                       COLOR_PROJECTION_BACK_B);
        glVertex2fv(x);
    }

    glEnd();
    glPointSize(1);
}

void ViewerSequence::DrawSceneView() {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixd(m_projMatrix);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixd(m_modelMatrix);

    ApplySceneViewTransformation();
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

void ViewerSequence::DrawSceneCameras() {
    const FrameIndex nFrms = std::min(m_pSeq->GetFramesNumber(),
                                      m_pSeq->GetCamerasNumber());
    if(nFrms == 0)
        return;

    Point3D center1, center2;
    glColor3ub(COLOR_CAMERA_PATH_R, COLOR_CAMERA_PATH_G, COLOR_CAMERA_PATH_B);
    glLineWidth(m_camPathWidth);
    glBegin(GL_LINES);
    m_pSeq->GetCamera(0).GetCenter(center2);
    for(FrameIndex iFrm1 = 0, iFrm2 = 1; iFrm2 < nFrms; iFrm1 = iFrm2, ++iFrm2) {
        if(IsFrameSolved(iFrm1))
            center1 = center2;
        if(IsFrameSolved(iFrm2)) {
            m_pSeq->GetCamera(iFrm2).GetCenter(center2);
            if(IsFrameSolved(iFrm1)) {
                glVertex3fv(center1);
                glVertex3fv(center2);
            }
        }
    }
    glEnd();
    glLineWidth(1.0f);

    if(m_drawCamType == DRAW_PATH_AND_ACTIVE_FRAME && IsFrameSolved(m_iFrmActive)) {
        if(m_pSeq->GetFrameState(m_iFrmActive) & FLAG_FRAME_STATE_KEY_FRAME)
            glColor3ub(COLOR_KEY_FRAME_R, COLOR_KEY_FRAME_G, COLOR_KEY_FRAME_B);
        else
            glColor3ub(COLOR_CAMERA_VOLUME_R, COLOR_CAMERA_VOLUME_G, COLOR_CAMERA_VOLUME_B);
        const Sequence::IntrinsicType intrinsicType = m_pSeq->GetIntrinsicType();
        if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
            m_camVolume.SetScale(m_camVolumeScale *
                                 m_pSeq->GetIntrinsicRectification().f());
        else if(intrinsicType == Sequence::INTRINSIC_VARIABLE)
            m_camVolume.SetScale(m_camVolumeScale * m_pSeq->GetIntrinsicRectification(
                                     m_iFrmActive).f());
        m_camVolume.DrawVolume(m_pSeq->GetCamera(m_iFrmActive));
    }
    if(m_drawCamType == DRAW_PATH_AND_ALL_KEY_FRAMES) {
        const Sequence::IntrinsicType intrinsicType = m_pSeq->GetIntrinsicType();
        if(intrinsicType == Sequence::INTRINSIC_CONSTANT)
            m_camVolume.SetScale(m_camVolumeScale *
                                 m_pSeq->GetIntrinsicRectification().f());
        glColor3ub(COLOR_KEY_FRAME_R, COLOR_KEY_FRAME_G, COLOR_KEY_FRAME_B);
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            if(!IsFrameSolved(iFrm) ||
                    !(m_pSeq->GetFrameState(iFrm) & FLAG_FRAME_STATE_KEY_FRAME))
                continue;
            if(intrinsicType == Sequence::INTRINSIC_VARIABLE)
                m_camVolume.SetScale(m_camVolumeScale * m_pSeq->GetIntrinsicRectification(
                                         iFrm).f());
            m_camVolume.DrawVolume(m_pSeq->GetCamera(iFrm));
        }
    }
}

void ViewerSequence::DrawScenePoints() {
    //glPointSize(3.0f);
    glBegin(GL_POINTS);
    const CVD::Rgb<ubyte> clrDefault = CVD::Rgb<ubyte>(COLOR_TRACK_DEFAULT_R,
                                       COLOR_TRACK_DEFAULT_G, COLOR_TRACK_DEFAULT_B);
    const TrackIndex nTrks = m_pSeq->GetPointsNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(!(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_SOLVED) ||
                ShouldTrackBeHidden(iTrk))
            continue;
        const CVD::Rgb<ubyte> &clr = iTrk < m_pSeq->GetTrackColorsNumber() ?
                                     m_pSeq->GetTrackColor(iTrk) : clrDefault;
        glColor3ub(clr.red, clr.green, clr.blue);
        glVertex3fv(m_pSeq->GetPoint(iTrk));
    }
    glEnd();
    //glPointSize(1.0f);
}

void ViewerSequence::DrawSelectedScenePoint() {
    glLineWidth(2);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, m_pWnd->size().x, m_pWnd->size().y, 0);

    Point2D x;
    m_Cg.ProjectToNormalizedPlane(m_pSeq->GetPoint(m_iTrkSelected), x);
    m_pSeq->GetIntrinsicMatrix().NormalizedPlaneToImage(x);
    x.Scale(m_factorImgToWin);
    glColor3ub(COLOR_FEATURE_SELECTION_BOX_R, COLOR_FEATURE_SELECTION_BOX_G,
               COLOR_FEATURE_SELECTION_BOX_B);
    glBegin(GL_LINES);
    Viewer::DrawBox(x.x(), x.y(), FEATURE_SELECTION_BOX_SIZE);
    glEnd();

    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glLineWidth(1);

    FrameIndex iFrm1, iFrm2;
    Point3D center1, center2;
    m_pSeq->ComputePointMinimalRayAngleDot(m_iTrkSelected, m_rayDirs, iFrm1, iFrm2,
                                           false);
    if(iFrm1 == INVALID_FRAME_INDEX || iFrm2 == INVALID_FRAME_INDEX)
        return;
    m_pSeq->GetCamera(iFrm1).GetCenter(center1);
    m_pSeq->GetCamera(iFrm2).GetCenter(center2);
    glLineWidth(0.5f);
    glColor3ub(COLOR_PROJECTION_ACTIVE_R, COLOR_PROJECTION_ACTIVE_G,
               COLOR_PROJECTION_ACTIVE_B);
    glBegin(GL_LINES);
    glVertex3fv(m_pSeq->GetPoint(m_iTrkSelected));
    glVertex3fv(center1);
    glVertex3fv(m_pSeq->GetPoint(m_iTrkSelected));
    glVertex3fv(center2);
    glEnd();
    glLineWidth(1);
}

void ViewerSequence::DrawActiveScenePoints() {
    glBegin(GL_POINTS);
    TrackIndex iTrk;
    const MeasurementIndex iMea1 = m_pSeq->GetFrameFirstMeasurementIndex(
                                       m_iFrmActive), iMea2 = m_pSeq->GetFrameFirstMeasurementIndex(m_iFrmActive + 1);
    for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
        if((iTrk = m_pSeq->GetMeasurementTrackIndex(iMea)) == INVALID_TRACK_INDEX ||
                !(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_SOLVED))
            continue;
        else if(IsMeasurementOutlier(iMea))
            glColor3ub(COLOR_ACTIVE_POINT_OUTLIER_R, COLOR_ACTIVE_POINT_OUTLIER_G,
                       COLOR_ACTIVE_POINT_OUTLIER_B);
        else
            glColor3ub(COLOR_ACTIVE_POINT_INLIER_R, COLOR_ACTIVE_POINT_INLIER_G,
                       COLOR_ACTIVE_POINT_INLIER_B);
        glVertex3fv(m_pSeq->GetPoint(iTrk));
    }
    glEnd();
}

void ViewerSequence::DrawFrameBar() {
    const ushort pointerSizeHalf = (FRAME_BAR_POINTER_WIDTH >> 1);
    //const float ratio = float(m_pWnd->size().x - FRAME_BAR_POINTER_WIDTH) / (m_pSeq->GetFramesNumber() - 1);
    const float ratio = float(m_pWnd->size().x - FRAME_BAR_POINTER_WIDTH) /
                        (m_pSeq->GetFramesNumberTotal() - 1);
    const Point2D trkBarPointerSize(ratio * 0.5f, TRACK_BAR_POINTER_HEIGHT);
    float y = m_pWnd->size().y - 1 - (STRING_BORDER_Y + STRING_SIZE_LARGE) -
              STRING_BORDER_Y;
    float x;

    if(m_iTrkSelected != INVALID_TRACK_INDEX) {
        glColor3ub(TRACK_BAR_COLOR_R, TRACK_BAR_COLOR_G, TRACK_BAR_COLOR_B);
        glBegin(GL_QUADS);
        const MeasurementIndexList &iMeas = m_pSeq->GetTrackMeasurementIndexList(
                                                m_iTrkSelected);
        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        for(FrameIndex i = 0; i < nCrsps; ++i) {
            x = pointerSizeHalf + m_pSeq->GetMeasurementFrameIndex(iMeas[i]) * ratio;
            DrawQuad(x, y, trkBarPointerSize);
        }
        glEnd();
    }

    glColor3ub(FRAME_BAR_COLOR_R, FRAME_BAR_COLOR_G, FRAME_BAR_COLOR_B);
    glBegin(GL_LINES);
    glVertex2i(0, y);
    glVertex2i(m_pWnd->size().x, y);
    glEnd();

    glBegin(GL_TRIANGLES);
    x = pointerSizeHalf + m_iFrmActive * ratio;
    glVertex2f(x, y);
    y -= FRAME_BAR_POINTER_HEIGHT;
    x += pointerSizeHalf;
    glVertex2f(x, y);
    x -= FRAME_BAR_POINTER_WIDTH;
    glVertex2f(x, y);
    glEnd();

    glColor3ub(STRING_COLOR_R, STRING_COLOR_G, STRING_COLOR_B);
    if(m_drawTrkType == DRAW_NO_TRACK || m_iFrmActive == 0)
        DrawStringBottomLeftLarge("Frame %d", m_iFrmActive);
    else {
        const FrameIndex nFrms = m_pSeq->GetFramesNumber();
        FrameIndex iFrm1, iFrm2;
        switch(m_drawTrkType) {
            case DRAW_TRACK_FROM_LAST_FRAME:
                iFrm1 = m_iFrmActive - 1;
                iFrm2 = m_iFrmActive;
                break;
            case DRAW_TRACK_FROM_LAST_KEY_FRAME:
                for(iFrm1 = m_iFrmActive - 1; iFrm1 < nFrms &&
                        !(m_pSeq->GetFrameState(iFrm1) & FLAG_FRAME_STATE_KEY_FRAME); --iFrm1);
                iFrm2 = m_iFrmActive;
                break;
            case DRAW_TRACK_TO_NEXT_KEY_FRAME:
                iFrm1 = m_iFrmActive;
                for(iFrm2 = m_iFrmActive + 1; iFrm2 < nFrms &&
                        !(m_pSeq->GetFrameState(iFrm2) & FLAG_FRAME_STATE_KEY_FRAME); ++iFrm2);
                break;
        }
        DrawStringBottomLeftLarge("Frame %d --> %d", iFrm1, iFrm2);
    }
    DrawFrameState();
}

void ViewerSequence::DrawFrameState() {
    const FrameState frmState = m_pSeq->GetFrameState(m_iFrmActive);
    if(frmState & FLAG_FRAME_STATE_SOLVED)
        DrawStringCurrentLarge(", SOLVED");
    if(frmState & FLAG_FRAME_STATE_KEY_FRAME)
        DrawStringCurrentLarge(", KEY FRAME");
    if(frmState & FLAG_FRAME_STATE_INTERPOLATED)
        DrawStringCurrentLarge(", INTERPOLATED");
    if(frmState & FLAG_FRAME_STATE_SPLIT_POINT)
        DrawStringCurrentLarge(", SPLIT POINT");
    if(frmState & FLAG_FRAME_STATE_SOLVED)
        DrawStringCurrentLarge(", inliers = %d, inlier ratio = %.2f, inlier area ratio = %.2f, MSE = %.2f",
                               m_nInliers, m_inlierRatio, m_inlierRatioArea, m_MSE);
}

void ViewerSequence::DrawFrameImageFileName() {
    glColor3ub(STRING_COLOR_R, STRING_COLOR_G, STRING_COLOR_B);
    Viewer::DrawStringTopLeftLarge("%s",
                                   m_pSeq->GetImageFileName(m_iFrmActive).c_str());
}

bool ViewerSequence::DragFrameBar(const CVD::ImageRef &from,
                                  const CVD::ImageRef &to) {
    const GLint y2 = m_pWnd->size().y - 1 - (STRING_BORDER_Y + STRING_SIZE_LARGE) -
                     STRING_BORDER_Y + FRAME_BAR_POINTER_HEIGHT;
    const GLint y1 = y2 - (FRAME_BAR_POINTER_HEIGHT << 1);
    if(from.y < y1 || from.y > y2)
        return false;
    const GLint pointerSizeHalf = (FRAME_BAR_POINTER_WIDTH >> 1);
    GLint x = to.x;
    if(x < pointerSizeHalf)
        x = pointerSizeHalf;
    else if(x > m_pWnd->size().x - 1 - pointerSizeHalf)
        x = m_pWnd->size().x - 1 - pointerSizeHalf;
    //const FrameIndex iFrmActive = FrameIndex(GLfloat(x - pointerSizeHalf) * m_pSeq->GetFramesNumber() / (m_pWnd->size().x - FRAME_BAR_POINTER_WIDTH));
    const FrameIndex iFrmActive = FrameIndex(GLfloat(x - pointerSizeHalf) *
                                  m_pSeq->GetFramesNumberTotal() / (m_pWnd->size().x - FRAME_BAR_POINTER_WIDTH));
    PrepareActiveFrame(iFrmActive);
    return true;
}

bool ViewerSequence::IsFrameSolved(const FrameIndex &iFrm) {
    return (m_pSeq->GetFrameState(iFrm) & FLAG_FRAME_STATE_SOLVED) != 0;
}

bool ViewerSequence::ShouldTrackBeHidden(const TrackIndex &iTrk) {
    const bool outlier = IsTrackOutlier(iTrk);
    return m_drawPtTypes[m_imgView] == DRAW_INLIER_POINTS && outlier ||
           m_drawPtTypes[m_imgView] == DRAW_OUTLIER_POINTS && !outlier
           || m_pSeq->GetTrackLength(iTrk) < m_minTrkLen;
}

bool ViewerSequence::ShouldMeasurementBeHidden(const MeasurementIndex &iMea) {
    const TrackIndex iTrk = m_pSeq->GetMeasurementTrackIndex(iMea);
    const bool outlier = IsMeasurementOutlier(iMea);
    return iTrk != INVALID_TRACK_INDEX && ShouldTrackBeHidden(iTrk) ||
           m_drawPtTypes[m_imgView] == DRAW_INLIER_POINTS && outlier
           || m_drawPtTypes[m_imgView] == DRAW_OUTLIER_POINTS && !outlier;
}

bool ViewerSequence::IsTrackOutlier(const TrackIndex &iTrk) {
    return !(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER) ||
           (m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_COMMON_OUTLIER);
}

bool ViewerSequence::IsMeasurementOutlier(const MeasurementIndex &iMea) {
    const TrackIndex iTrk = m_pSeq->GetMeasurementTrackIndex(iMea);
    return iTrk != INVALID_TRACK_INDEX && IsTrackOutlier(iTrk) ||
           (m_pSeq->GetMeasurementState(iMea) & FLAG_MEASUREMENT_STATE_OUTLIER);
}

void ViewerSequence::ConvertWindowCoordinate2DTo3D(const CVD::ImageRef &where2D,
        Point3D &where3D) {
    double X, Y, Z;
    //gluUnProject(where2D.x, m_pWnd->size().y - where2D.y, 0, m_modelMatrix, m_I, m_viewport, &X, &Y, &Z);
    gluUnProject(where2D.x, m_pWnd->size().y - where2D.y, 0, m_modelMatrix,
                 m_projMatrix, m_viewport, &X, &Y, &Z);
    where3D.Set(float(X), float(Y), float(Z));
#if _DEBUG
    assert(Z == WINDOW_ZPLANE);
#endif
}

void ViewerSequence::ConvertWindowCoordinate2DTo3D(const CVD::ImageRef &where2D,
        Point3D &where3D, const float &zPlane) {
    double X, Y, Z;
    //gluUnProject(where2D.x, m_pWnd->size().y - where2D.y, 0, m_modelMatrix, m_I, m_viewport, &X, &Y, &Z);
    gluUnProject(where2D.x, m_pWnd->size().y - where2D.y, 0, m_modelMatrix,
                 m_projMatrix, m_viewport, &X, &Y, &Z);
    //float winz;
    //glReadPixels(where2D.x, where2D.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winz);
    //gluUnProject(where2D.x, where2D.y, winz, m_modelMatrix, m_I, m_viewport, &X, &Y, &Z);
    //printf("(%f, %f, %f) --> ", X, Y, Z);
#if _DEBUG
    assert(Z == WINDOW_ZPLANE);
#endif
    Z = zPlane / Z;
    where3D.Set(float(X * Z), float(Y * Z), zPlane);
    //where3D.Print();
}

void ViewerSequence::ApplyImageViewTransformation() {
    glScalef(m_factorImgToWin.x(), m_factorImgToWin.y(), 1.0f);
}

void ViewerSequence::ApplySceneViewTransformation() {
    if(m_drawArcball) {
        glColor3ub(COLOR_ARCBALL_R, COLOR_ARCBALL_G, COLOR_ARCBALL_B);
        m_arcball.DrawSphere(ARCBALL_SLICES, ARCBALL_STACKS);
    }
    //glTranslatef(m_translation.X(), m_translation.Y(), m_translation.Z());
    m_arcball.ApplyTransformation();
    glTranslatef(m_translation.X(), m_translation.Y(), m_translation.Z());
}

void ViewerSequence::ResetSceneViewProjectionMatrix() {
    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    const float yzRatio = float(m_pSeq->GetImageHeight() * 0.5f * K.one_over_fy());
    const double fovy = atan(yzRatio) * FACTOR_RAD_TO_DEG * 2;
    const double aspect = m_pSeq->GetImageWidth() / double(
                              m_pSeq->GetImageHeight());

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef((K.cx() + K.cx()) / m_pSeq->GetImageWidth() - 1,
                 1 - (K.cy() + K.cy()) / m_pSeq->GetImageHeight(), 0.0f);
    gluPerspective(fovy, aspect, WINDOW_ZPLANE, DBL_MAX);
    glGetDoublev(GL_PROJECTION_MATRIX, m_projMatrix);
    glPopMatrix();
}

void ViewerSequence::ResetSceneViewModelViewMatrix() {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
    glGetDoublev(GL_MODELVIEW_MATRIX, m_modelMatrix);
    glPopMatrix();
}

void ViewerSequence::ResetSceneViewTransformation() {
    if(m_iFrmActive < m_pSeq->GetCamerasNumber())
        m_Cg = m_pSeq->GetCamera(m_iFrmActive);
    else
        m_Cg.MakeIdentity();

    float depthMean, depthRange;
    ComputeDepthMeanAndRange(depthMean, depthRange);

    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    const float zPlane = depthRange * CAMERA_VOLUME_ZPLANE_DEPTH_RANGE_RATIO;
    const float xzRatio = float(m_pSeq->GetImageWidth() * 0.5f * K.one_over_fx());
    const float yzRatio = float(m_pSeq->GetImageHeight() * 0.5f * K.one_over_fy());
    m_camVolume.Initialize(zPlane, xzRatio, yzRatio, m_camVolumeScale);
    //m_translation.SetZero();
    m_arcball.Initialize(m_Cg, depthMean, ComputeArcballRadius(depthMean));
    Point3D t;
    m_Cg.GetTranslation(t);
    LA::AmB(t, m_arcball.GetCenter(), t);
    m_arcball.GetRotation().ApplyInversely(t, m_translation);
    LA::ApB(m_translation, m_arcball.GetCenter(), m_translation);
    //m_Cg.Apply(m_arcball.GetCenter(), m_translation);
    //LA::AmB(m_translation, m_arcball.GetCenter(), m_translation);
    m_dZuint = depthMean / m_pWnd->size().y * TRANSLATE_Z_FACTOR;
}

void ViewerSequence::ResetImageWindowSizeRatio() {
    m_factorImgToWin.Set(float(m_pWnd->size().x) / m_pSeq->GetImageWidth(),
                         float(m_pWnd->size().y) / m_pSeq->GetImageHeight());
    m_factorWinToImg.Set(1 / m_factorImgToWin.x(), 1 / m_factorImgToWin.y());
}

void ViewerSequence::UpdateSceneViewTransformation() {
    m_Cg = m_arcball.GetRotation();
    Point3D t1, t2;
    LA::AmB(m_translation, m_arcball.GetCenter(), t1);
    t1.reserve() = 1;
    m_arcball.GetRotation().Apply(t1, t2);
    LA::ApB(t2, m_arcball.GetCenter(), t2);
    m_Cg.SetTranslation(t2);
    //m_Cg = m_arcball.GetRotation();
    //Point3D t1, t2;
    //LA::ApB(m_arcball.GetCenter(), m_translation, t1);
    //m_arcball.GetRotation().Apply(m_arcball.GetCenter(), t2);
    //LA::AmB(t1, t2, t2);
    //m_Cg.SetTranslation(t2);
    //m_Cg.Print();
}

void ViewerSequence::ComputeDepths(std::vector<float> &depths) {
    Point2D xMin, xMax;
    xMin.Set(0.0f, 0.0f);
    xMax.Set(m_pSeq->GetImageWidth() - 1.0f, m_pSeq->GetImageHeight() - 1.0f);
    m_pSeq->GetIntrinsicMatrix().ImageToNormalizedPlane(xMin);
    m_pSeq->GetIntrinsicMatrix().ImageToNormalizedPlane(xMax);

    Point3D X;
    Point2D x;
    depths.resize(0);
    const TrackIndex nTrks = m_pSeq->GetPointsNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(!(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER))
            continue;
        m_Cg.ProjectToNormalizedPlane(m_pSeq->GetPoint(iTrk), X, x);
        //if(X.Z() > 0 && x > xMin && x < xMax)
        //  depths.push_back(X.Z());
        if(x > xMin && x < xMax)
            depths.push_back(fabs(X.Z()));
    }
}

float ViewerSequence::ComputeDepthMean() {
    ComputeDepths(m_depths);
    if(m_depths.empty()) {
        Point3D center;
        const FrameIndex nFrms = m_pSeq->GetCamerasNumber();
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            m_pSeq->GetCamera(iFrm).GetCenter(center);
            m_depths.push_back(m_Cg.ComputeDepth(center));
        }
        if(m_depths.empty())
            return 0;
    }
    const TrackIndex N = TrackIndex(m_depths.size()), ith = (N >> 1);
    std::nth_element(m_depths.begin(), m_depths.begin() + ith, m_depths.end());
    return m_depths[ith];
}

void ViewerSequence::ComputeDepthMeanAndRange(float &depthMean,
        float &depthRange) {
    if((depthMean = ComputeDepthMean()) == 0) {
        depthRange = 0;
        return;
    }
    const TrackIndex N = TrackIndex(m_depths.size()), ith = (N >> 1);
    for(TrackIndex i = 0; i < N; ++i)
        m_depths[i] = fabs(m_depths[i] - depthMean);
    std::nth_element(m_depths.begin(), m_depths.begin() + ith, m_depths.end());
    depthRange = m_depths[ith] * 2;
}

float ViewerSequence::ComputeArcballRadius(const float &centerZ) {
    const float xWinSize = m_pSeq->GetImageWidth() *
                           m_pSeq->GetIntrinsicMatrix().one_over_fx();
    const float yWinSize = m_pSeq->GetImageHeight() *
                           m_pSeq->GetIntrinsicMatrix().one_over_fy();
    if(xWinSize < yWinSize)
        return xWinSize * ARCBALL_RADIUS_WINDOWN_RATIO * centerZ;
    else
        return yWinSize * ARCBALL_RADIUS_WINDOWN_RATIO * centerZ;
}

void ViewerSequence::OnDraw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if(m_imgView)
        DrawImageView();
    else
        DrawSceneView();
}

void ViewerSequence::OnDrawString() {
    DrawFrameBar();
    DrawFrameImageFileName();
}

bool ViewerSequence::OnKeyDown(const int key) {
    switch(key) {
        case KEY_SWITCH_VIEW:
            PrepareView(!m_imgView);
            return true;
        case KEY_STEP_ACTIVE_FRAME_FORWARD:
            PrepareActiveFrame(m_iFrmActive + 1);
            return true;
        case KEY_STEP_ACTIVE_FRAME_BACKWARD:
            PrepareActiveFrame(m_iFrmActive - 1);
            return true;
        case KEY_STEP_DRAW_POINT_TYPE: {
            switch(m_drawPtTypes[m_imgView]) {
                case DRAW_NO_POINT:
                    m_drawPtTypes[m_imgView] = DRAW_ALL_POINTS;
                    printf("[DRAW_ALL_POINTS]\n");
                    break;
                case DRAW_ALL_POINTS:
                    m_drawPtTypes[m_imgView] = DRAW_INLIER_POINTS;
                    printf("[DRAW_INLIER_POINTS]\n");
                    break;
                case DRAW_INLIER_POINTS:
                    m_drawPtTypes[m_imgView] = DRAW_OUTLIER_POINTS;
                    printf("[DRAW_OUTLIER_POINTS]\n");
                    break;
                case DRAW_OUTLIER_POINTS:
                    m_drawPtTypes[m_imgView] = DRAW_NO_POINT;
                    printf("[DRAW_NO_POINT]\n");
                    break;
            }
            return true;
        }
        case KEY_STEP_SELECTED_TRACK_FRAME: {
            if(m_iTrkSelected != INVALID_TRACK_INDEX) {
                const MeasurementIndexList &iMeas = m_pSeq->GetTrackMeasurementIndexList(
                                                        m_iTrkSelected);
                const FrameIndex nCrsps = FrameIndex(iMeas.size());
                FrameIndex i;
                for(i = 0; i < nCrsps &&
                        m_pSeq->GetMeasurementFrameIndex(iMeas[i]) <= m_iFrmActive; ++i);
                if(i == nCrsps)
                    i = 0;
                PrepareActiveFrame(m_pSeq->GetMeasurementFrameIndex(iMeas[i]));
            }
            return true;
        }
        case KEY_DRAW_ERRORS:
            m_drawErrs = !m_drawErrs;
            return true;
        case KEY_SAVE_VIEW: {
            char fileName[MAX_LINE_LENGTH];
            sprintf(fileName, "%s%s_view_frm_%04d.jpg", m_pSeq->GetDirectory().c_str(),
                    m_imgView ? "img" : "scene", m_iFrmActive);
            Viewer::SaveView(fileName);
            //sprintf(fileName, "%s%s_view_frm_%d_test.jpg", m_pSeq->GetDirectory().c_str(), m_imgView ? "img" : "scene", m_iFrmActive);
            //ProgramGL::BindFrameBuffer();
            ////m_texActive.Bind();
            //TextureGL3 tex;
            //tex.Generate(m_texActive.GetWidth(), m_texActive.GetHeight());
            //ProgramGL::SetOutputTexture(tex);
            //Draw();
            //tex.DownloadToCPU((ubyte *) m_imgActive.data());
            //CVD::flipVertical(m_imgActive);
            //CVD::img_save(m_imgActive, fileName);
            //ProgramGL::UnbindFrameBuffer();
            return true;
        }
        case KEY_INPUT_FRAME: {
            char buf[MAX_LINE_LENGTH];
            printf(">>Frame: ");
            fgets(buf, MAX_LINE_LENGTH, stdin);
            int input;
            sscanf(buf, "%d", &input);
            PrepareActiveFrame(FrameIndex(input));
            return true;
        }
        case KEY_INPUT_TRACK: {
            char buf[MAX_LINE_LENGTH];
            printf(">>Track: ");
            fgets(buf, MAX_LINE_LENGTH, stdin);
            int input;
            sscanf(buf, "%d", &input);
            const TrackIndex iTrkSelected = TrackIndex(input);
            if(iTrkSelected < m_pSeq->GetTracksNumber()) {
                PrepareSelectedTrack(iTrkSelected);
                if(m_iFtrSelected == INVALID_FEATURE_INDEX)
                    PrepareActiveFrame(m_pSeq->GetTrackFirstFrameIndex(m_iTrkSelected));
            }
            return true;
        }
        case KEY_INPUT_MEASUREMENT: {
            char buf[MAX_LINE_LENGTH];
            printf(">>Measurement: ");
            fgets(buf, MAX_LINE_LENGTH, stdin);
            int input;
            sscanf(buf, "%d", &input);
            const MeasurementIndex iMeaSelected = MeasurementIndex(input);
            if(iMeaSelected < m_pSeq->GetMeasurementsNumber()) {
                PrepareSelectedTrack(m_pSeq->GetMeasurementTrackIndex(iMeaSelected));
                if(m_iFtrSelected == INVALID_FEATURE_INDEX)
                    PrepareActiveFrame(m_pSeq->GetTrackFirstFrameIndex(m_iTrkSelected));
            }
            return true;
        }
        case KEY_INPUT_FEATURE: {
            char buf[MAX_LINE_LENGTH];
            printf(">>Feature: ");
            fgets(buf, MAX_LINE_LENGTH, stdin);
            int input;
            sscanf(buf, "%d", &input);
            const FeatureIndex iFtrSelected = FeatureIndex(input);
            if(iFtrSelected < m_pSeq->GetFrameFeaturesNumber(m_iFrmActive))
                PrepareSelectedFeature(iFtrSelected);
            return true;
        }
        case KEY_INPUT_DESCRIPTOR: {
            char buf[MAX_LINE_LENGTH];
            printf(">>Descriptor: ");
            fgets(buf, MAX_LINE_LENGTH, stdin);
            int input;
            sscanf(buf, "%d", &input);
#if DESCRIPTOR_TRACK
            const TrackIndex iTrkSelected = TrackIndex(input);
            if(iTrkSelected < m_pSeq->GetTracksNumber()) {
                PrepareSelectedTrack(iTrkSelected);
                if(m_iFtrSelected == INVALID_FEATURE_INDEX)
                    PrepareActiveFrame(m_pSeq->GetTrackFirstFrameIndex(m_iTrkSelected));
                m_pSeq->GetDescriptor(iTrkSelected).Print();
            }
#else
            const MeasurementIndex iMeaSelected = MeasurementIndex(input);
            if(iMeaSelected < m_pSeq->GetDescriptorsNumber()) {
                PrepareSelectedTrack(m_pSeq->GetMeasurementTrackIndex(iMeaSelected));
                if(m_iFtrSelected == INVALID_FEATURE_INDEX)
                    PrepareActiveFrame(m_pSeq->GetTrackFirstFrameIndex(m_iTrkSelected));
                m_pSeq->GetDescriptor(iMeaSelected).Print();
            }
#endif
            return true;
        }
        case KEY_INPUT_MIN_TRACK_LENGTH: {
            char buf[MAX_LINE_LENGTH];
            printf(">>Min track length: ");
            fgets(buf, MAX_LINE_LENGTH, stdin);
            int input;
            sscanf(buf, "%d", &input);
            m_minTrkLen = FrameIndex(input);
            return true;
        }
        case KEY_PRINT_STATE:
            m_pSeq->PrintCamera(m_iFrmActive);
            m_pSeq->PrintStates();
            return true;
    }
    if(m_imgView) {
        switch(key) {
            case KEY_STEP_PROJECTION_TYPE: {
                switch(m_drawProjType) {
                    case DRAW_NO_PROJECTION:
                        m_drawProjType = DRAW_ALL_PROJECTIONS;
                        printf("[DRAW_ALL_PROJECTIONS]\n");
                        break;
                    case DRAW_ALL_PROJECTIONS:
                        m_drawProjType = DRAW_ACTIVE_PROJECTIONS;
                        printf("[DRAW_ACTIVE_PROJECTIONS]\n");
                        break;
                    case DRAW_ACTIVE_PROJECTIONS:
                        m_drawProjType = DRAW_NO_PROJECTION;
                        printf("[DRAW_NO_PROJECTION]\n");
                        break;
                }
                return true;
            }
            case KEY_STEP_DRAW_TRACK_TYPE: {
                switch(m_drawTrkType) {
                    //case DRAW_NO_TRACK:                   m_drawTrkType = DRAW_TRACK_ALL_FRAMES;          printf("[DRAW_TRACK_ALL_FRAMES]\n");            break;
                    //case DRAW_TRACK_ALL_FRAMES:           m_drawTrkType = DRAW_TRACK_FROM_LAST_FRAME;     printf("[DRAW_TRACK_FROM_LAST_FRAME]\n");       break;
                    case DRAW_NO_TRACK:
                        m_drawTrkType = DRAW_TRACK_FROM_LAST_FRAME;
                        printf("[DRAW_TRACK_FROM_LAST_FRAME]\n");
                        break;
                    case DRAW_TRACK_FROM_LAST_FRAME:
                        m_drawTrkType = DRAW_TRACK_FROM_LAST_KEY_FRAME;
                        printf("[DRAW_TRACK_FROM_LAST_KEY_FRAME]\n");
                        break;
                    case DRAW_TRACK_FROM_LAST_KEY_FRAME:
                        m_drawTrkType = DRAW_TRACK_TO_NEXT_KEY_FRAME;
                        printf("[DRAW_TRACK_TO_NEXT_KEY_FRAME]\n");
                        break;
                    case DRAW_TRACK_TO_NEXT_KEY_FRAME:
                        m_drawTrkType = DRAW_NO_TRACK;
                        printf("[DRAW_NO_TRACK]\n");
                        break;
                }
                return true;
            }
            case KEY_DRAW_DISTRIBUTION:
                m_drawDistribution = !m_drawDistribution;
                return true;
        }
    } else {
        switch(key) {
            case KEY_TRANSLATE_XY:
                m_controlType = CONTROL_TRANSLATE_XY;
                printf("[CONTROL_TRANSLATE_XY]\n");
                return true;
            case KEY_TRANSLATE_Z:
                m_controlType = CONTROL_TRANSLATE_Z;
                printf("[CONTROL_TRANSLATE_Z]\n");
                return true;
            case KEY_ROTATION:
                m_controlType = CONTROL_ROTATION;
                printf("[CONTROL_ROTATION]\n");
                return true;
            case KEY_DRAW_ARCBALL:
                m_drawArcball = !m_drawArcball;
                return true;
            case KEY_STEP_DRAW_CAMERA_TYPE: {
                switch(m_drawCamType) {
                    case DRAW_NO_CAMERA:
                        m_drawCamType = DRAW_PATH;
                        printf("[DRAW_PATH]\n");
                        break;
                    case DRAW_PATH:
                        m_drawCamType = DRAW_PATH_AND_ACTIVE_FRAME;
                        printf("[DRAW_PATH_AND_ACTIVE_FRAME]\n");
                        break;
                    case DRAW_PATH_AND_ACTIVE_FRAME:
                        m_drawCamType = DRAW_PATH_AND_ALL_KEY_FRAMES;
                        printf("[DRAW_PATH_AND_ALL_KEY_FRAMES]\n");
                        break;
                    case DRAW_PATH_AND_ALL_KEY_FRAMES:
                        m_drawCamType = DRAW_NO_CAMERA;
                        printf("[DRAW_NO_CAMERA]\n");
                        break;
                }
                return true;
            }
            case KEY_DRAW_ACTIVE_POINTS:
                m_drawActivePts = !m_drawActivePts;
                return true;
            case KEY_RESET_TRANSFORMATION:
                ResetSceneViewTransformation();
                return true;
            case KEY_SWITCH_BACKGROUND_COLOR:
                m_backgroundDark = !m_backgroundDark;
                if(m_backgroundDark)
                    glClearColor(0, 0, 0, 0);
                else
                    glClearColor(1, 1, 1, 1);
                return true;
            case KEY_CAMERA_VOLUME_SCALE_INCREASE:
                if(m_camVolumeScale >= 1.0f)
                    ++m_camVolumeScale;
                else
                    m_camVolumeScale = 1 / float(int(1 / m_camVolumeScale + 0.5f) - 1);
                m_camVolume.SetScale(m_camVolumeScale);
                return true;
            case KEY_CAMERA_VOLUME_SCALE_DECREASE:
                if(m_camVolumeScale <= 1.0f)
                    m_camVolumeScale = 1 / float(int(1 / m_camVolumeScale + 0.5f) + 1);
                else
                    --m_camVolumeScale;
                m_camVolume.SetScale(m_camVolumeScale);
                return true;
            case KEY_CAMERA_PATH_WIDTH_INCREASE:
                m_camPathWidth += 0.5f;
                return true;
            case KEY_CAMERA_PATH_WIDTH_DECREASE:
                m_camPathWidth -= 0.5f;
                return true;
        }
    }
    return false;
}

void ViewerSequence::OnMouseDown(const CVD::ImageRef &where, const int button) {
    if(m_imgView) {
    } else {
        switch(m_controlType) {
            case CONTROL_TRANSLATE_XY:
                m_translationStart = m_translation;
                ConvertWindowCoordinate2DTo3D(where, m_whereMouseDown3DStart,
                                              m_arcball.GetCenterZ());
                break;
            case CONTROL_TRANSLATE_Z:
                m_translationStart = m_translation;
                break;
            case CONTROL_ROTATION:
                ConvertWindowCoordinate2DTo3D(where, m_whereMouseDown3DStart);
                m_arcball.StartRotation(m_whereMouseDown3DStart);
                break;
        }
    }
}

void ViewerSequence::OnMouseUp(const CVD::ImageRef &where, const int button) {
    if(!m_imgView) {
        if(m_handler.IsMouseDraging() && (m_controlType == CONTROL_TRANSLATE_XY ||
                                          m_controlType == CONTROL_TRANSLATE_Z)) {
            const float depthMean = ComputeDepthMean();
            if(m_controlType == CONTROL_TRANSLATE_Z)
                m_dZuint = ComputeDepthMean() / m_pWnd->size().y * TRANSLATE_Z_FACTOR;
            Point3D C1mC2, dt;
            C1mC2 = m_arcball.GetCenter();
            C1mC2.Z() -= depthMean;
            m_arcball.GetRotation().ApplyInversely(C1mC2, dt);
            LA::AmB(dt, C1mC2, dt);
            LA::ApB(m_translation, dt, m_translation);
            //m_arcball.GetRotation().Apply(C1mC2, dt);
            //LA::AmB(C1mC2, dt, dt);
            //LA::ApB(m_translation, dt, m_translation);
            m_arcball.Set(depthMean, ComputeArcballRadius(depthMean));
            UpdateSceneViewTransformation();
        }
    }
}

bool ViewerSequence::OnMouseDraging(const CVD::ImageRef &from,
                                    const CVD::ImageRef &to, const int button) {
    if(DragFrameBar(from, to))
        return true;
    else if(m_imgView) {
        return false;
    } else {
        switch(m_controlType) {
            case CONTROL_TRANSLATE_XY: {
                Point3D whereMouseDown3D;
                ConvertWindowCoordinate2DTo3D(to, whereMouseDown3D, m_arcball.GetCenterZ());
                //m_translation.X() = m_translationStart.X() + whereMouseDown3D.X() - m_whereMouseDown3DStart.X();
                //m_translation.Y() = m_translationStart.Y() + whereMouseDown3D.Y() - m_whereMouseDown3DStart.Y();
                const RotationTransformation3D &R = m_arcball.GetRotation();
                m_translation.XYZx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(R.r_00_01_02_x(),
                                       ENFT_SSE::_mm_set1_ps(whereMouseDown3D.X() - m_whereMouseDown3DStart.X())),
                                       ENFT_SSE::_mm_mul_ps(R.r_10_11_12_x(),
                                               ENFT_SSE::_mm_set1_ps(whereMouseDown3D.Y() - m_whereMouseDown3DStart.Y()))),
                                       m_translationStart.XYZx());
                break;
            }
            case CONTROL_TRANSLATE_Z: {
                //m_translation.Z() = m_translationStart.Z() + m_dZuint * (from.y - to.y);
                const RotationTransformation3D &R = m_arcball.GetRotation();
                m_translation.XYZx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(R.r_20_21_22_x(),
                                       ENFT_SSE::_mm_set1_ps(m_dZuint * (from.y - to.y))), m_translationStart.XYZx());
                break;
            }
            case CONTROL_ROTATION: {
                Point3D whereMouseDown3D;
                ConvertWindowCoordinate2DTo3D(to, whereMouseDown3D);
                m_arcball.ComputeRotation(whereMouseDown3D);
                break;
            }
        }
        UpdateSceneViewTransformation();
        return true;
    }
    //m_translation.Print();
}

bool ViewerSequence::OnMouseDoubleClicked(const CVD::ImageRef &where,
        const int button) {
    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    const Point2D boxSize(FEATURE_SELECTION_BOX_SIZE * m_factorWinToImg.x(),
                          FEATURE_SELECTION_BOX_SIZE * m_factorWinToImg.y());
    const Point2D click(where.x * m_factorWinToImg.x(),
                        where.y * m_factorWinToImg.y());
    float xDist, yDist, dist, minDist = FLT_MAX;
    if(m_imgView) {
        Point2D x;
        MeasurementIndex iMeaSelected = INVALID_MEASUREMENT_INDEX;
        const MeasurementIndex iMea1 = m_pSeq->GetFrameFirstMeasurementIndex(
                                           m_iFrmActive), iMea2 = m_pSeq->GetFrameFirstMeasurementIndex(m_iFrmActive + 1);
        for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
            if(ShouldMeasurementBeHidden(iMea))
                continue;
            else if(m_pSeq->AreMeasurementsNormalized())
                K.NormalizedPlaneToImage(m_pSeq->GetMeasurement(iMea), x);
            else
                x = m_pSeq->GetMeasurement(iMea);
            xDist = fabs(x.x() - click.x());
            yDist = fabs(x.y() - click.y());
            if(xDist < boxSize.x() && yDist < boxSize.y() &&
                    (dist = std::max(xDist, yDist)) < minDist) {
                iMeaSelected = iMea;
                minDist = dist;
            }
        }
        if(iMeaSelected == INVALID_MEASUREMENT_INDEX) {
            PrepareSelectedFeature(INVALID_FEATURE_INDEX);
            PrepareSelectedTrack(INVALID_TRACK_INDEX);
            return false;
        } else {
            PrepareSelectedFeature(iMeaSelected - iMea1);
            return true;
        }
    } else {
        TrackIndex iTrkSelected = INVALID_TRACK_INDEX;
        Point2D x;
        if(m_drawPtTypes[m_imgView] != DRAW_NO_POINT) {
            const TrackIndex nTrks = m_pSeq->GetPointsNumber();
            for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
                if(!(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_SOLVED) ||
                        ShouldTrackBeHidden(iTrk))
                    continue;
                m_Cg.ProjectToNormalizedPlane(m_pSeq->GetPoint(iTrk), x);
                K.NormalizedPlaneToImage(x);
                xDist = fabs(x.x() - click.x());
                yDist = fabs(x.y() - click.y());
                if(xDist < boxSize.x() && yDist < boxSize.y() &&
                        (dist = std::max(xDist, yDist)) < minDist) {
                    iTrkSelected = iTrk;
                    minDist = dist;
                }
            }
        }
        if(m_drawActivePts) {
            TrackIndex iTrk;
            const MeasurementIndex iMea1 = m_pSeq->GetFrameFirstMeasurementIndex(
                                               m_iFrmActive), iMea2 = m_pSeq->GetFrameFirstMeasurementIndex(m_iFrmActive + 1);
            for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
                iTrk = m_pSeq->GetMeasurementTrackIndex(iMea);
                const ubyte test = (m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_SOLVED);
                if((iTrk = m_pSeq->GetMeasurementTrackIndex(iMea)) == INVALID_TRACK_INDEX ||
                        !(m_pSeq->GetTrackState(iTrk) & FLAG_TRACK_STATE_SOLVED))
                    continue;
                m_Cg.ProjectToNormalizedPlane(m_pSeq->GetPoint(iTrk), x);
                K.NormalizedPlaneToImage(x);
                xDist = fabs(x.x() - click.x());
                yDist = fabs(x.y() - click.y());
                if(xDist < boxSize.x() && yDist < boxSize.y() &&
                        (dist = std::max(xDist, yDist)) < minDist) {
                    iTrkSelected = iTrk;
                    minDist = dist;
                }
            }
        }
        PrepareSelectedTrack(iTrkSelected);
        return iTrkSelected != INVALID_TRACK_INDEX;
    }
}

void ViewerSequence::OnResize(const CVD::ImageRef &size) {
    m_viewport[2] = size.x;
    m_viewport[3] = size.y;
    ResetImageWindowSizeRatio();
}