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
#include "ViewerSequenceSet.h"
#include "Utility/Random.h"

void ViewerSequenceSet::Run(const SequenceSet &seqs,
                            const SequenceIndex &iSeqActive) {
    ProgramGL::UnbindFrameBuffer();
    Initialize(seqs, iSeqActive);
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

void ViewerSequenceSet::Initialize(const SequenceSet &seqs,
                                   const SequenceIndex iSeqActive) {
    if(m_pSeqSet != &seqs) {
        m_iSeqActive = iSeqActive;
        m_iFrmActive = m_iFrmImg = m_iFrmScn = INVALID_FRAME_INDEX;
        m_iFtrSelected = INVALID_FEATURE_INDEX;
        m_iTrkSelected = INVALID_TRACK_INDEX;
        m_pSeqSet = &seqs;
        m_iTrkCmnSelected = INVALID_TRACK_INDEX;
        m_iFrmsActive.assign(seqs.GetSequencesNumber(), 0);
        m_drawSeqType = DRAW_ACTIVE_SEQUENCE;
        m_clrSegs = false;

        const SequenceIndex nSeqs = seqs.GetSequencesNumber();
        m_mapSeqFrmToCatFrm.resize(nSeqs);
        m_mapCatFrmToSeqFrm.resize(0);
        uint iFrmCat = 0;
        for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
            const FrameIndex nFrms = seqs[iSeq].GetFramesNumber();
            std::vector<uint> &iFrmsCat = m_mapSeqFrmToCatFrm[iSeq];
            iFrmsCat.resize(nFrms);
            for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm, ++iFrmCat) {
                iFrmsCat[iFrm] = iFrmCat;
                m_mapCatFrmToSeqFrm.push_back(SequenceFrameIndex(iSeq, iFrm));
            }
        }
    }

    const SequenceIndex nSeqs = seqs.GetSequencesNumber();
    m_mapSeqFrmToSeg.resize(nSeqs);
    m_mapSegToSeqFrm.resize(0);
    SequenceIndex iSeq;
    SegmentIndex iSeg;
    for(iSeq = iSeg = 0; iSeq < nSeqs; ++iSeq) {
        const Sequence &seq = m_pSeqSet->GetSequence(iSeq);
        SegmentIndexList &iSegs = m_mapSeqFrmToSeg[iSeq];
        const FrameIndex nFrms = seq.GetFramesNumber();
        iSegs.resize(nFrms);

        FrameIndex iFrm, iFrm1, iFrm2 = 0;
        while(iFrm2 < nFrms) {
            iFrm1 = iFrm2;
            for(iFrm2 = iFrm1 + 1; iFrm2 < nFrms &&
                    !(seq.GetFrameState(iFrm2) & FLAG_FRAME_STATE_SPLIT_POINT); ++iFrm2);
            if(iFrm2 != nFrms && iFrm2 != iFrm1 + 1)
                ++iFrm2;
            for(iFrm = iFrm1; iFrm < iFrm2; ++iFrm)
                iSegs[iFrm] = iSeg;
            m_mapSegToSeqFrm.push_back(SequenceFramePairIndex(iSeq, iFrm1, iFrm2));
            ++iSeg;
        }
    }

    const SegmentIndex nSegs = iSeg;
    m_pathClrsSeg.resize(nSegs);
    for(iSeg = 0; iSeg < nSegs; ++iSeg)
        m_pathClrsSeg[iSeg] = CVD::Rgb<ubyte>(Random::GenerateUbyte(),
                                              Random::GenerateUbyte(), Random::GenerateUbyte());
    const SequenceIndex nClrs = SequenceIndex(m_pathClrsSeq.size());
    m_pathClrsSeq.resize(nSeqs);
    for(iSeq = nClrs; iSeq < nSeqs; ++iSeq)
        m_pathClrsSeq[iSeq] = m_pathClrsSeg[m_mapSeqFrmToSeg[iSeq][0]];

    ViewerSequence::Initialize(seqs[m_iSeqActive]);
    PrepareActiveSequence(iSeqActive);
}

bool ViewerSequenceSet::PrepareActiveSequence(const SequenceIndex iSeqActive) {
    if(iSeqActive == m_iSeqActive || iSeqActive >= m_pSeqSet->GetSequencesNumber())
        return false;
    m_iFrmsActive[m_iSeqActive] = m_iFrmActive;
    m_iSeqActive = iSeqActive;
    m_pSeq = &m_pSeqSet->GetSequence(m_iSeqActive);
    m_iTrkSelected = m_iTrkCmnSelected == INVALID_TRACK_INDEX ?
                     INVALID_TRACK_INDEX : m_pSeqSet->SearchCommonTrackForIndividualTrack(
                         m_iTrkCmnSelected, m_iSeqActive);
    m_iFrmActive = m_iFrmImg = m_iFrmScn = INVALID_FRAME_INDEX;
    PrepareActiveFrame((m_iTrkSelected == INVALID_TRACK_INDEX
                        || m_pSeqSet->GetSequence(m_iSeqActive).SearchTrackForFrameMeasurementIndex(
                            m_iTrkSelected, m_iFrmsActive[m_iSeqActive]) != INVALID_MEASUREMENT_INDEX) ?
                       m_iFrmsActive[m_iSeqActive] : m_pSeqSet->GetSequence(
                           m_iSeqActive).GetTrackFirstFrameIndex(m_iTrkSelected));
    ResetImageWindowSizeRatio();
    m_pSeqSet->SearchForSequenceConnectedComponent(m_iSeqActive, m_iSeqsCC);
    return true;
}

bool ViewerSequenceSet::PrepareSelectedTrack(const TrackIndex iTrkSelected) {
    if(!ViewerSequence::PrepareSelectedTrack(iTrkSelected))
        return false;
    m_iTrkSelected = iTrkSelected;
    m_iTrkCmnSelected = m_iTrkSelected == INVALID_TRACK_INDEX ?
                        INVALID_TRACK_INDEX : m_pSeqSet->GetIndividualTrackCommonTrack(m_iSeqActive,
                                m_iTrkSelected);
    if(m_iTrkCmnSelected != INVALID_TRACK_INDEX)
        m_pSeqSet->PrintCommonTrack(m_iTrkCmnSelected);
    return true;
}

void ViewerSequenceSet::DrawAllProjections() {
    glPointSize(2);
    glBegin(GL_POINTS);
    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    const Camera &C = m_pSeq->GetCamera(m_iFrmActive);
    const Sequence::IntrinsicType intrinsicType = m_pSeq->GetIntrinsicType();
    const float f = intrinsicType == Sequence::INTRINSIC_VARIABLE ?
                    m_pSeq->GetIntrinsicRectification(m_iFrmActive).f() :
                    m_pSeq->GetIntrinsicRectification().f();

    Point3D Xc;
    Point2D x;
    const SequenceIndex nSeqs = m_pSeqSet->GetSequencesNumber();
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        if(iSeq != m_iSeqActive &&
                !(m_pSeqSet->GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED))
            continue;
        const Sequence &seq = m_pSeqSet->GetSequence(iSeq);
        const Sequence::IntrinsicType intrinsicType = seq.GetIntrinsicType();
        const TrackIndex nTrks = seq.GetTracksNumber();
        for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
            if(!(seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_SOLVED))
                continue;
            C.ProjectToNormalizedPlane(seq.GetPoint(iTrk), Xc, x);
            if(intrinsicType != Sequence::INTRINSIC_USER_FIXED)
                x *= f;
            K.NormalizedPlaneToImage(x);
            if(Xc.Z() > 0)
                glColor3ub(COLOR_PROJECTION_FRONT_R, COLOR_PROJECTION_FRONT_G,
                           COLOR_PROJECTION_FRONT_B);
            else
                glColor3ub(COLOR_PROJECTION_BACK_R, COLOR_PROJECTION_BACK_G,
                           COLOR_PROJECTION_BACK_B);
            glVertex2fv(x);
        }
    }

    glEnd();
    glPointSize(1);
}

void ViewerSequenceSet::DrawSceneView() {
    if(m_drawSeqType != DRAW_ACTIVE_SEQUENCE) {
        const SequenceIndex iSeqActiveBkp = m_iSeqActive;
        const FrameIndex iFrmActiveBkp = m_iFrmActive;
        const DrawCameraType drawCamTypeBkp = m_drawCamType;
        const TrackIndex iTrkSelectedBkp = m_iTrkSelected;
        const FeatureIndex iFtrSelectedBkp = m_iFtrSelected;
        const bool drawActivePtsBkp = m_drawActivePts;
        m_drawCamType = drawCamTypeBkp == DRAW_NO_CAMERA ? DRAW_NO_CAMERA : DRAW_PATH;
        m_graySeqPath = m_drawSeqType != DRAW_ALL_SEQUENCES;
        m_iTrkSelected = INVALID_TRACK_INDEX;
        m_iFtrSelected = INVALID_FEATURE_INDEX;
        m_drawActivePts = false;
        const SequenceIndex nSeqs = m_pSeqSet->GetSequencesNumber();
        for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
            if(iSeq == m_iSeqActive ||
                    m_drawSeqType == DRAW_REGISTERED_SEQUENCES_INACTIVE_GRAY &&
                    !(m_pSeqSet->GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED))
                continue;
            m_iSeqActive = iSeq;
            m_iFrmActive = m_iFrmsActive[iSeq];
            m_pSeq = &m_pSeqSet->GetSequence(iSeq);
            ViewerSequence::DrawSceneView();
        }
        m_iSeqActive = iSeqActiveBkp;
        m_iFrmActive = iFrmActiveBkp;
        m_pSeq = &m_pSeqSet->GetSequence(m_iSeqActive);
        m_drawCamType = drawCamTypeBkp;
        m_iTrkSelected = iTrkSelectedBkp;
        m_iFtrSelected = iFtrSelectedBkp;
        m_drawActivePts = drawActivePtsBkp;
    }

    m_graySeqPath = false;
    ViewerSequence::DrawSceneView();
}

void ViewerSequenceSet::DrawSceneCameras() {
    if(m_drawCamType == DRAW_NO_CAMERA)
        return;
    else if(m_drawCamType != DRAW_PATH)
        ViewerSequence::DrawSceneCameras();

    const FrameIndex nFrms = std::min(m_pSeq->GetFramesNumber(),
                                      m_pSeq->GetCamerasNumber());
    if(nFrms == 0)
        return;
    glLineWidth(m_camPathWidth);
    glBegin(GL_LINE_STRIP);
    const CVD::Rgb<ubyte> clrGray = CVD::Rgb<ubyte>(127, 127, 127);
    SequenceIndex iSeq;
    FrameIndex iFrm1, iFrm2;
    Point3D center;
    const SegmentIndex iSegStart = m_mapSeqFrmToSeg[m_iSeqActive][0],
                       iSegEnd = m_mapSeqFrmToSeg[m_iSeqActive][nFrms - 1];
    for(SegmentIndex iSeg = iSegStart; iSeg <= iSegEnd; ++iSeg) {
        const CVD::Rgb<ubyte> &clr = m_graySeqPath ? clrGray : (m_clrSegs ?
                                     m_pathClrsSeg[iSeg] : m_pathClrsSeq[m_iSeqActive]);
        glColor3ub(clr.red, clr.green, clr.blue);
        m_mapSegToSeqFrm[iSeg].Get(iSeq, iFrm1, iFrm2);
        for(FrameIndex iFrm = iFrm1; iFrm < iFrm2; ++iFrm) {
            if(!IsFrameSolved(iFrm))
                continue;
            m_pSeq->GetCamera(iFrm).GetCenter(center);
            glVertex3fv(center);
        }
    }
    glEnd();
    glLineWidth(1.0f);
}

void ViewerSequenceSet::DrawSelectedScenePoint() {
    if(m_iTrkCmnSelected == INVALID_TRACK_INDEX || !m_drawErrs)
        ViewerSequence::DrawSelectedScenePoint();
    else {
        glBegin(GL_LINES);
        glColor3ub(COLOR_FEATURE_ERROR_LINE_R, COLOR_FEATURE_ERROR_LINE_G,
                   COLOR_FEATURE_ERROR_LINE_B);
        SequenceIndex iSeq;
        TrackIndex iTrkIdv;
        const Point3D &Xcmn = m_pSeqSet->GetCommonPoint(m_iTrkCmnSelected);
        const SequenceTrackIndexList &iSeqTrksIdv =
            m_pSeqSet->GetCommonTrackIndividualTrackIndexList(m_iTrkCmnSelected);
        const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
        for(SequenceIndex i = 0; i < nCrsps; ++i) {
            iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
            if(!(m_pSeqSet->GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED))
                continue;
            glVertex3fv(Xcmn);
            glVertex3fv(m_pSeqSet->GetSequence(iSeq).GetPoint(iTrkIdv));
        }
        glEnd();
    }

}

void ViewerSequenceSet::DrawActiveScenePoints() {
    if(m_drawErrs) {
        glBegin(GL_LINES);
        glColor3ub(COLOR_FEATURE_ERROR_LINE_R, COLOR_FEATURE_ERROR_LINE_G,
                   COLOR_FEATURE_ERROR_LINE_B);
        SequenceIndex iSeq;
        TrackIndex iTrkIdv, iTrkCmn;
        const MeasurementIndex iMea1 = m_pSeq->GetFrameFirstMeasurementIndex(
                                           m_iFrmActive), iMea2 = m_pSeq->GetFrameFirstMeasurementIndex(m_iFrmActive + 1);
        for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
            if((iTrkIdv = m_pSeq->GetMeasurementTrackIndex(iMea)) == INVALID_TRACK_INDEX ||
                    !(m_pSeq->GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_SOLVED)
                    || (iTrkCmn = m_pSeqSet->GetIndividualTrackCommonTrack(m_iSeqActive,
                                  iTrkIdv)) == INVALID_TRACK_INDEX)
                continue;
            const Point3D &Xcmn = m_pSeqSet->GetCommonPoint(iTrkCmn);
            const SequenceTrackIndexList &iSeqTrksIdv =
                m_pSeqSet->GetCommonTrackIndividualTrackIndexList(iTrkCmn);
            const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
            for(SequenceIndex i = 0; i < nCrsps; ++i) {
                iSeqTrksIdv[i].Get(iSeq, iTrkIdv);
                if(!(m_pSeqSet->GetSequenceState(iSeq) & FLAG_SEQUENCE_STATE_REGISTRED))
                    continue;
                glVertex3fv(Xcmn);
                glVertex3fv(m_pSeqSet->GetSequence(iSeq).GetPoint(iTrkIdv));
            }
        }
        glEnd();
    }
    ViewerSequence::DrawActiveScenePoints();
}

void ViewerSequenceSet::DrawFrameBar() {
    const ushort pointerSizeHalf = (FRAME_BAR_POINTER_WIDTH >> 1);
    const float ratio = float(m_pWnd->size().x - FRAME_BAR_POINTER_WIDTH) /
                        (m_mapCatFrmToSeqFrm.size() - 1);
    const Point2D trkBarPointerSize(ratio * 0.5f, TRACK_BAR_POINTER_HEIGHT);
    float y = m_pWnd->size().y - 1 - (STRING_BORDER_Y + STRING_SIZE_LARGE) -
              STRING_BORDER_Y;
    float x;

    const SequenceIndex nSeqsCC = SequenceIndex(m_iSeqsCC.size());
    //glColor3ub(COLOR_SEQUENCE_CONNECTED_COMPONENT_R, COLOR_SEQUENCE_CONNECTED_COMPONENT_G, COLOR_SEQUENCE_CONNECTED_COMPONENT_B);
    glBegin(GL_QUADS);
    for(SequenceIndex i = 0; i < nSeqsCC; ++i) {
        const SequenceIndex iSeqCC = m_iSeqsCC[i];
        const CVD::Rgb<ubyte> &clr = m_pathClrsSeq[iSeqCC];
        glColor3ub(clr.red, clr.green, clr.blue);
        const std::vector<uint> &iFrmsCatCC = m_mapSeqFrmToCatFrm[iSeqCC];
        x = pointerSizeHalf + (iFrmsCatCC.front() + iFrmsCatCC.back()) * ratio * 0.5f;
        const Point2D x1(pointerSizeHalf + (iFrmsCatCC.front() - 0.5f) * ratio,
                         y - TRACK_BAR_POINTER_HEIGHT);
        const Point2D x2(pointerSizeHalf + (iFrmsCatCC.back() + 0.5f) * ratio,
                         y + TRACK_BAR_POINTER_HEIGHT);
        DrawQuad(x1, x2);
    }
    glEnd();

    const std::vector<uint> &iFrmsCat = m_mapSeqFrmToCatFrm[m_iSeqActive];
    if(m_iTrkSelected != INVALID_TRACK_INDEX) {
        glColor3ub(TRACK_BAR_COLOR_R, TRACK_BAR_COLOR_G, TRACK_BAR_COLOR_B);
        glBegin(GL_QUADS);
        const MeasurementIndexList &iMeas = m_pSeq->GetTrackMeasurementIndexList(
                                                m_iTrkSelected);
        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        for(FrameIndex i = 0; i < nCrsps; ++i) {
            x = pointerSizeHalf + iFrmsCat[m_pSeq->GetMeasurementFrameIndex(
                                               iMeas[i])] * ratio;
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
    x = pointerSizeHalf + iFrmsCat[m_iFrmActive] * ratio;
    glVertex2f(x, y);
    y -= FRAME_BAR_POINTER_HEIGHT;
    x += pointerSizeHalf;
    glVertex2f(x, y);
    x -= FRAME_BAR_POINTER_WIDTH;
    glVertex2f(x, y);
    glEnd();

    glColor3ub(STRING_COLOR_R, STRING_COLOR_G, STRING_COLOR_B);
    DrawStringBottomLeftLarge("Sequence %d, Segment %d, Frame %d", m_iSeqActive,
                              m_mapSeqFrmToSeg[m_iSeqActive][m_iFrmActive], m_iFrmActive);
    if(m_pSeqSet->GetSequenceState(m_iSeqActive) & FLAG_SEQUENCE_STATE_REGISTRED)
        DrawStringCurrentLarge(", REGISTERED");
    DrawStringCurrentLarge(", inliers = %d, inlier ratio = %.2f, inlier area ratio = %.2f, MSE = %.2f",
                           m_nInliers, m_inlierRatio, m_inlierRatioArea, m_MSE);
}

bool ViewerSequenceSet::DragFrameBar(const CVD::ImageRef &from,
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
    const uint iFrmCat = uint(GLfloat(x - pointerSizeHalf) *
                              m_mapCatFrmToSeqFrm.size() / (m_pWnd->size().x - FRAME_BAR_POINTER_WIDTH));
    PrepareActiveSequence(m_mapCatFrmToSeqFrm[iFrmCat].GetSequenceIndex());
    PrepareActiveFrame(m_mapCatFrmToSeqFrm[iFrmCat].GetFrameIndex());
    return true;
}

void ViewerSequenceSet::ComputeDepths(std::vector<float> &depths) {
    Point2D xMin, xMax, x;
    Point3D X;
    depths.resize(0);

    const SequenceIndex nSeqs = m_pSeqSet->GetSequencesNumber();
    for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
        const Sequence &seq = m_pSeqSet->GetSequence(iSeq);
        xMin.Set(0.0f, 0.0f);
        xMax.Set(seq.GetImageWidth() - 1.0f, seq.GetImageHeight() - 1.0f);
        seq.GetIntrinsicMatrix().ImageToNormalizedPlane(xMin);
        seq.GetIntrinsicMatrix().ImageToNormalizedPlane(xMax);

        const TrackIndex nTrks = seq.GetPointsNumber();
        for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
            if(!(seq.GetTrackState(iTrk) & FLAG_TRACK_STATE_INLIER))
                continue;
            m_Cg.ProjectToNormalizedPlane(seq.GetPoint(iTrk), X, x);
            if(X.Z() > 0 && x > xMin && x < xMax)
                depths.push_back(X.Z());
        }
    }
}

bool ViewerSequenceSet::OnKeyDown(const int key) {
    switch(key) {
        case KEY_STEP_SELECTED_COMMON_TRACK_SEQUENCE: {
            if(m_iTrkCmnSelected == INVALID_TRACK_INDEX) {
                for(SequenceIndex iSeqActive = m_iSeqActive + 1; iSeqActive != m_iSeqActive;
                        ++iSeqActive) {
                    if(iSeqActive == m_pSeqSet->GetSequencesNumber())
                        iSeqActive = 0;
                    if(m_drawSeqType != DRAW_REGISTERED_SEQUENCES_INACTIVE_GRAY ||
                            (m_pSeqSet->GetSequenceState(iSeqActive) & FLAG_SEQUENCE_STATE_REGISTRED)) {
                        PrepareActiveSequence(iSeqActive);
                        break;
                    }
                }
            } else {
                const SequenceTrackIndexList &iSeqTrksIdv =
                    m_pSeqSet->GetCommonTrackIndividualTrackIndexList(m_iTrkCmnSelected);
                const SequenceIndex nCrsps = SequenceIndex(iSeqTrksIdv.size());
                SequenceIndex i, j;
                for(i = 0; iSeqTrksIdv[i].GetSequenceIndex() != m_iSeqActive; ++i);
                for(j = i + 1; j != i; ++j) {
                    if(j == nCrsps)
                        j = 0;
                    if(m_drawSeqType != DRAW_REGISTERED_SEQUENCES_INACTIVE_GRAY ||
                            (m_pSeqSet->GetSequenceState(iSeqTrksIdv[j].GetSequenceIndex()) &
                             FLAG_SEQUENCE_STATE_REGISTRED)) {
                        PrepareActiveSequence(iSeqTrksIdv[j].GetSequenceIndex());
                        break;
                    }
                }
            }
            return true;
        }
        case KEY_INPUT_SEQUENCE: {
            char buf[MAX_LINE_LENGTH];
            printf(">>Sequence: ");
            fgets(buf, MAX_LINE_LENGTH, stdin);
            int input;
            sscanf(buf, "%d", &input);
            PrepareActiveSequence(SequenceIndex(input));
            return true;
        }
        case KEY_PRINT_STATE:
            m_pSeqSet->PrintStates();
            m_pSeq->PrintStates();
            return true;
        case KEY_STEP_DRAW_SEQUENCE_TYPE: {
            switch(m_drawSeqType) {
                case DRAW_ACTIVE_SEQUENCE:
                    m_drawSeqType = DRAW_REGISTERED_SEQUENCES_INACTIVE_GRAY;
                    printf("[DRAW_REGISTERED_SEQUENCES_INACTIVE_GRAY]\n");
                    break;
                case DRAW_REGISTERED_SEQUENCES_INACTIVE_GRAY:
                    m_drawSeqType = DRAW_ALL_SEQUENCES_INACTIVE_GRAY;
                    printf("[DRAW_ALL_SEQUENCES_INACTIVE_GRAY]\n");
                    break;
                case DRAW_ALL_SEQUENCES_INACTIVE_GRAY:
                    m_drawSeqType = DRAW_ALL_SEQUENCES;
                    printf("[DRAW_ALL_SEQUENCES]\n");
                    break;
                case DRAW_ALL_SEQUENCES:
                    m_drawSeqType = DRAW_ACTIVE_SEQUENCE;
                    printf("[DRAW_ACTIVE_SEQUENCE]\n");
                    break;
            }
            return true;
        }
        case KEY_COLOR_SEGMENTS:
            m_clrSegs = !m_clrSegs;
            return true;
    }
    return ViewerSequence::OnKeyDown(key);
}

bool ViewerSequenceSet::OnMouseDoubleClicked(const CVD::ImageRef &where,
        const int button) {
    if(m_imgView || !m_drawErrs)
        return ViewerSequence::OnMouseDoubleClicked(where, button);
    else if(ViewerSequence::OnMouseDoubleClicked(where, button))
        return true;

    TrackIndex iTrkIdv, iTrkCmn, iTrkSelected = INVALID_TRACK_INDEX;
    Point2D x;
    float xDist, yDist, dist, minDist = FLT_MAX;

    const IntrinsicMatrix &K = m_pSeq->GetIntrinsicMatrix();
    const Point2D boxSize(FEATURE_SELECTION_BOX_SIZE * m_factorWinToImg.x(),
                          FEATURE_SELECTION_BOX_SIZE * m_factorWinToImg.y());
    const Point2D click(where.x * m_factorWinToImg.x(),
                        where.y * m_factorWinToImg.y());
    const MeasurementIndex iMea1 = m_pSeq->GetFrameFirstMeasurementIndex(
                                       m_iFrmActive), iMea2 = m_pSeq->GetFrameFirstMeasurementIndex(m_iFrmActive + 1);
    for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
        if((iTrkIdv = m_pSeq->GetMeasurementTrackIndex(iMea)) == INVALID_TRACK_INDEX ||
                !(m_pSeq->GetTrackState(iTrkIdv) & FLAG_TRACK_STATE_SOLVED)
                || (iTrkCmn = m_pSeqSet->GetIndividualTrackCommonTrack(m_iSeqActive,
                              iTrkIdv)) == INVALID_TRACK_INDEX)
            continue;
        m_Cg.ProjectToNormalizedPlane(m_pSeqSet->GetCommonPoint(iTrkCmn), x);
        K.NormalizedPlaneToImage(x);
        xDist = fabs(x.x() - click.x());
        yDist = fabs(x.y() - click.y());
        if(xDist < boxSize.x() && yDist < boxSize.y() &&
                (dist = std::max(xDist, yDist)) < minDist) {
            iTrkSelected = iTrkIdv;
            minDist = dist;
        }
    }
    PrepareSelectedTrack(iTrkSelected);
    return iTrkSelected != INVALID_TRACK_INDEX;
}