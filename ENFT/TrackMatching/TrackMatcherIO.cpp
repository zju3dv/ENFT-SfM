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
#include "Utility/Utility.h"
#include <cvd/image_io.h>
#include <cvd/image_convert.h>
#include <cvd/draw.h>

#define KEY_SWITCH_VIEW         ' '
#define KEY_STEP_FORWARD        'f'
#define KEY_STOP_MATCHING       19  // Ctrl + s
#define KEY_HIDE_SEED           's'
//#define KEY_SHOW_KEY_FRAMES       'k'
#define KEY_SCALE_INCREASE      '+'
#define KEY_SCALE_DECREASE      '-'

#define CROSS_SIZE  5
//#define CROSS_SIZE    1
#define COLOR_MATCHING_FRAME_R  255
#define COLOR_MATCHING_FRAME_G  0
#define COLOR_MATCHING_FRAME_B  255
#define COLOR_KEY_FRAME_R       0
#define COLOR_KEY_FRAME_G       0
#define COLOR_KEY_FRAME_B       255

using namespace ENFT_SfM;

void TrackMatcher::PrepareViewing(const Sequence &seq, const std::vector<TrackIndexList> &iTrkClusters) {
    if(!m_view)
        return;
    const FrameIndex nFrms = seq.GetFramesNumber();
    ProgramGL::Initialize(nFrms, nFrms);
    //const CVD::ImageRef winSizeBkp = m_pWnd->size();
    //ProgramGL::GetGLWindow()->set_size(CVD::ImageRef(int(nFrms), int(nFrms)));
    m_initView = true;
    m_hideSeed = false;
    //m_showKeyFrms = false;
    m_crossesList.resize(0);
    m_seedClrs.resize(0);
    m_matchingMatrixInitImg.resize(CVD::ImageRef(int(nFrms), int(nFrms)));
    m_matchingMatrixUpdImg.resize(CVD::ImageRef(int(nFrms), int(nFrms)));
    m_img.resize(CVD::ImageRef(int(nFrms), int(nFrms)));
    m_matchingMatrixInitTex.Generate(nFrms, nFrms);
    //m_matchingMatrixInitTexScaled.Generate(nFrms, nFrms);
    m_matchingMatrixUpdTex.Generate(nFrms, nFrms);
    m_matchingMatrixUpdTexScaled.Generate(nFrms, nFrms);
    PrepareInitialMatchingMatrixImageAndTexture(seq, iTrkClusters);
    memset(m_matchingMatrixUpdImg.data(), 0, sizeof(ushort) * m_matchingMatrixUpdImg.totalsize());
    memset(m_img.data(), 0, sizeof(float) * m_img.totalsize());
    m_matchingMatrixUpdTex.Bind();
    m_matchingMatrixUpdTex.UploadFromCPU(m_img.data());
    ScaleMatchingMatrixTexture(m_matchingMatrixUpdTex, m_matchingMatrixUpdTexScaled);
    ViewMatchingMatrix();
}

void TrackMatcher::PrepareViewing(const Sequence &seq, const SparseMatrix<float> &matchingMatrixInit) {
    if(!m_view)
        return;
    const FrameIndex nFrms = seq.GetFramesNumber();
    ProgramGL::Initialize(nFrms, nFrms);
    //const CVD::ImageRef winSizeBkp = m_pWnd->size();
    //ProgramGL::GetGLWindow()->set_size(CVD::ImageRef(int(nFrms), int(nFrms)));
    m_initView = true;
    m_hideSeed = false;
    //m_showKeyFrms = false;
    m_crossesList.resize(0);
    m_seedClrs.resize(0);
    m_matchingMatrixInitImg.resize(CVD::ImageRef(int(nFrms), int(nFrms)));
    m_matchingMatrixUpdImg.resize(CVD::ImageRef(int(nFrms), int(nFrms)));
    m_img.resize(CVD::ImageRef(int(nFrms), int(nFrms)));
    m_matchingMatrixInitTex.Generate(nFrms, nFrms);
    //m_matchingMatrixInitTexScaled.Generate(nFrms, nFrms);
    m_matchingMatrixUpdTex.Generate(nFrms, nFrms);
    m_matchingMatrixUpdTexScaled.Generate(nFrms, nFrms);
    PrepareInitialMatchingMatrixImageAndTexture(seq, matchingMatrixInit);
    memset(m_matchingMatrixUpdImg.data(), 0, sizeof(ushort) * m_matchingMatrixUpdImg.totalsize());
    memset(m_img.data(), 0, sizeof(float) * m_img.totalsize());
    m_matchingMatrixUpdTex.Bind();
    m_matchingMatrixUpdTex.UploadFromCPU(m_img.data());
    ScaleMatchingMatrixTexture(m_matchingMatrixUpdTex, m_matchingMatrixUpdTexScaled);
    ViewMatchingMatrix();
}

void TrackMatcher::PrepareViewing(const SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2,
                                  const std::vector<TrackIndexListPair> &iTrkClusters) {
    if(!m_view)
        return;
    const Sequence &seq1 = seqs[iSeq1], &seq2 = seqs[iSeq2];
    const FrameIndex nFrms1 = seq1.GetFramesNumber(), nFrms2 = seq2.GetFramesNumber();
    ProgramGL::Initialize(nFrms2, nFrms1);
    //const CVD::ImageRef winSizeBkp = m_pWnd->size();
    //ProgramGL::GetGLWindow()->set_size(CVD::ImageRef(int(nFrms2), int(nFrms1)));
    //const bool stopBkp = m_stop;
    m_initView = true;
    m_hideSeed = false;
    //m_showKeyFrms = false;
    m_crossesList.resize(0);
    m_seedClrs.resize(0);
    m_matchingMatrixInitImg.resize(CVD::ImageRef(int(nFrms2), int(nFrms1)));
    m_matchingMatrixUpdImg.resize(CVD::ImageRef(int(nFrms2), int(nFrms1)));
    m_img.resize(CVD::ImageRef(int(nFrms2), int(nFrms1)));
    m_matchingMatrixInitTex.Generate(nFrms2, nFrms1);
    //m_matchingMatrixInitTexScaled.Generate(nFrms2, nFrms1);
    m_matchingMatrixUpdTex.Generate(nFrms2, nFrms1);
    m_matchingMatrixUpdTexScaled.Generate(nFrms2, nFrms1);
    PrepareInitialMatchingMatrixImageAndTexture(seq1, seq2, iTrkClusters);
    PrepareUpdatingMatchingMatrixImageAndTexture(seq1, seq2);
    ViewMatchingMatrix();
}

void TrackMatcher::PrepareViewing(const SequenceSet &seqs, const SequenceIndex &iSeq1, const SequenceIndex &iSeq2,
                                  const SparseMatrix<float> &matchingMatrixInit) {
    if(!m_view)
        return;
    const Sequence &seq1 = seqs[iSeq1], &seq2 = seqs[iSeq2];
    const FrameIndex nFrms1 = seq1.GetFramesNumber(), nFrms2 = seq2.GetFramesNumber();
    ProgramGL::Initialize(nFrms2, nFrms1);
    //const CVD::ImageRef winSizeBkp = m_pWnd->size();
    //ProgramGL::GetGLWindow()->set_size(CVD::ImageRef(int(nFrms2), int(nFrms1)));
    //const bool stopBkp = m_stop;
    m_initView = true;
    m_hideSeed = false;
    //m_showKeyFrms = false;
    m_crossesList.resize(0);
    m_seedClrs.resize(0);
    m_matchingMatrixInitImg.resize(CVD::ImageRef(int(nFrms2), int(nFrms1)));
    m_matchingMatrixUpdImg.resize(CVD::ImageRef(int(nFrms2), int(nFrms1)));
    m_img.resize(CVD::ImageRef(int(nFrms2), int(nFrms1)));
    m_matchingMatrixInitTex.Generate(nFrms2, nFrms1);
    //m_matchingMatrixInitTexScaled.Generate(nFrms2, nFrms1);
    m_matchingMatrixUpdTex.Generate(nFrms2, nFrms1);
    m_matchingMatrixUpdTexScaled.Generate(nFrms2, nFrms1);
    PrepareInitialMatchingMatrixImageAndTexture(seq1, seq2, matchingMatrixInit);
    PrepareUpdatingMatchingMatrixImageAndTexture(seq1, seq2);
    ViewMatchingMatrix();
}

//static int g_cnt = 0;

void TrackMatcher::ViewMatchingMatrix() {
    if(!m_view)
        return;
    Viewer::Initialize();
    m_iFrm1 = m_iFrm2 = 0;
    Resize(m_pWnd->size());
    ProgramGL::UnbindFrameBuffer();
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
    //sprintf(fileName, "E:/matching_matrix/%05d.png", g_cnt++);
    //Viewer::SaveView(fileName);
    //if(!m_initView && m_hideSeed)
    //  Viewer::SaveView("matching_matrix.png");

    ProgramGL::BindFrameBuffer();
}

void TrackMatcher::PrepareInitialMatchingMatrixImageAndTexture(const Sequence &seq, const std::vector<TrackIndexList> &iTrkClusters) {
    if(!m_view)
        return;
    memset(m_matchingMatrixInitImg.data(), 0, sizeof(float) * m_matchingMatrixInitImg.totalsize());
    memset(m_img.data(), 0, sizeof(float) * m_img.totalsize());

    const FrameIndex nFrms = seq.GetFramesNumber(), N = nFrms;
#if MATCHING_MATRIX_INITIALIZATION_TFIDF
    std::vector<float> idfSqSums(nFrms, 0);
#endif

    TrackIndex i1, i2, iTrk1, iTrk2;
    FrameIndex j1, j2, iFrm1, iFrm2;
    MeasurementIndex iMea1, iMea2;
    const uint nClusters = uint(iTrkClusters.size());
    for(uint i = 0; i < nClusters; ++i) {
        const TrackIndexList &iTrks = iTrkClusters[i];
        const TrackIndex nTrks = TrackIndex(iTrks.size());
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_NAIVE
        const float idf = 1;
#else
        FrameIndex Ni = 0;
        for(i1 = 0; i1 < nTrks; ++i1)
            Ni += seq.GetTrackLength(iTrks[i1]);
        if(Ni > N)
            continue;
        const float idf = log(float(N) / Ni);
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
        const float idfSq = idf * idf;
#endif
#endif
        for(i1 = 0; i1 < nTrks; ++i1)
            for(i2 = i1 + 1; i2 < nTrks; ++i2) {
                iTrk1 = iTrks[i1];
                iTrk2 = iTrks[i2];
                if(seq.AreTracksOverlappingInFrames(iTrk1, iTrk2, m_marks1))
                    continue;
                const MeasurementIndexList &iMeas1 = seq.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq.GetTrackMeasurementIndexList(iTrk2);
                const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
                for(j1 = 0; j1 < nCrsps1; ++j1) {
                    iMea1 = iMeas1[j1];
                    iFrm1 = seq.GetMeasurementFrameIndex(iMea1);
                    for(j2 = 0; j2 < nCrsps2; ++j2) {
                        iMea2 = iMeas2[j2];
                        iFrm2 = seq.GetMeasurementFrameIndex(iMea2);
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
                        idfSqSums[iFrm1] += idfSq;
                        idfSqSums[iFrm2] += idfSq;
#else
                        if(iFrm1 < iFrm2)
                            m_matchingMatrixInitImg[iFrm1][iFrm2] += idf;
                        else
                            m_matchingMatrixInitImg[iFrm2][iFrm1] += idf;
#endif
                    }
                }
            }
    }
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
    std::vector<float> &norms = idfSqSums;
    for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1)
        norms[iFrm1] = seq.GetFrameFeaturesNumber(iFrm1) / sqrt(idfSqSums[iFrm1]);
    for(uint i = 0; i < nClusters; ++i) {
        const TrackIndexList &iTrks = iTrkClusters[i];
        const TrackIndex nTrks = TrackIndex(iTrks.size());
        for(i1 = 0; i1 < nTrks; ++i1)
            for(i2 = i1 + 1; i2 < nTrks; ++i2) {
                iTrk1 = iTrks[i1];
                iTrk2 = iTrks[i2];
                if(seq.AreTracksOverlappingInFrames(iTrk1, iTrk2, m_marks))
                    continue;
                const MeasurementIndexList &iMeas1 = seq.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq.GetTrackMeasurementIndexList(iTrk2);
                const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
                for(j1 = 0; j1 < nCrsps1; ++j1) {
                    iMea1 = iMeas1[j1];
                    iFrm1 = seq.GetMeasurementFrameIndex(iMea1);
                    for(j2 = 0; j2 < nCrsps2; ++j2) {
                        iMea2 = iMeas2[j2];
                        iFrm2 = seq.GetMeasurementFrameIndex(iMea2);
                        if(iFrm1 < iFrm2)
                            m_matchingMatrixInitImg[iFrm1][iFrm2] += norms[iFrm1] * norms[iFrm2];
                        else
                            m_matchingMatrixInitImg[iFrm2][iFrm1] += norms[iFrm1] * norms[iFrm2];
                    }
                }
            }
    }
#endif
    float confidence, confidenceMax = 0;
    for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1)
        for(iFrm2 = iFrm1 + 1; iFrm2 < nFrms; ++iFrm2) {
            if((confidence = m_matchingMatrixInitImg[iFrm1][iFrm2]) > confidenceMax)
                confidenceMax = confidence;
        }
    const float norm = 1.0f / confidenceMax;
    const float confidenceMin = confidenceMax * m_enftMinConfidenceInitMatchingMatrixRatio;

    for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1)
        for(iFrm2 = iFrm1 + 1; iFrm2 < nFrms; ++iFrm2) {
            if(m_matchingMatrixInitImg[iFrm1][iFrm2] > confidenceMin)
                m_img[iFrm1][iFrm2] = m_img[iFrm2][iFrm1] = m_matchingMatrixInitImg[iFrm1][iFrm2] * norm;
        }

    m_matchingMatrixInitTex.Bind();
    m_matchingMatrixInitTex.UploadFromCPU(m_img.data());
    //CVD::img_save(m_img, "test.png");
    //ScaleMatchingMatrixTexture(m_matchingMatrixInitTex, m_matchingMatrixInitTexScaled);
}

void TrackMatcher::PrepareInitialMatchingMatrixImageAndTexture(const Sequence &seq, const SparseMatrix<float> &matchingMatrixInit) {
    if(!m_view)
        return;
    memset(m_matchingMatrixInitImg.data(), 0, sizeof(float) * m_matchingMatrixInitImg.totalsize());
    memset(m_img.data(), 0, sizeof(float) * m_img.totalsize());

    FrameIndex iFrm1, iFrm2;
    const FrameIndex nFrms = seq.GetFramesNumber();
    for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1) {
        const FullIndexValueList<float> &elements = matchingMatrixInit.GetRowData(iFrm1);
        const FrameIndex nElements = FrameIndex(elements.size());
        for(FrameIndex i = 0; i < nElements; ++i) {
            iFrm2 = elements[i].GetFullIndex();
            if(iFrm1 < iFrm2)
                m_matchingMatrixInitImg[iFrm1][iFrm2] = elements[i].GetValue();
            else
                m_matchingMatrixInitImg[iFrm2][iFrm1] = elements[i].GetValue();
        }
    }

    float confidence, confidenceMax = 0;
    for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1)
        for(iFrm2 = iFrm1 + 1; iFrm2 < nFrms; ++iFrm2) {
            if((confidence = m_matchingMatrixInitImg[iFrm1][iFrm2]) > confidenceMax)
                confidenceMax = confidence;
        }
    const float norm = 1.0f / confidenceMax;
    const float confidenceMin = confidenceMax * m_enftMinConfidenceInitMatchingMatrixRatio;

    for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1)
        for(iFrm2 = iFrm1 + 1; iFrm2 < nFrms; ++iFrm2) {
            if(m_matchingMatrixInitImg[iFrm1][iFrm2] > confidenceMin)
                m_img[iFrm1][iFrm2] = m_img[iFrm2][iFrm1] = m_matchingMatrixInitImg[iFrm1][iFrm2] * norm;
        }

    m_matchingMatrixInitTex.Bind();
    m_matchingMatrixInitTex.UploadFromCPU(m_img.data());
    //ScaleMatchingMatrixTexture(m_matchingMatrixInitTex, m_matchingMatrixInitTexScaled);
}

void TrackMatcher::PrepareInitialMatchingMatrixImageAndTexture(const Sequence &seq1, const Sequence &seq2,
        const std::vector<TrackIndexListPair> &iTrkClusters) {
    if(!m_view)
        return;
    memset(m_matchingMatrixInitImg.data(), 0, sizeof(float) * m_matchingMatrixInitImg.totalsize());
    memset(m_img.data(), 0, sizeof(float) * m_img.totalsize());

    const FrameIndex nFrms1 = seq1.GetFramesNumber(), nFrms2 = seq2.GetFramesNumber(), N = nFrms1 + nFrms2;
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
    std::vector<float> idfSqSums1(nFrms1, 0), idfSqSums2(nFrms2, 0);
#endif

    TrackIndex i1, i2, iTrk1, iTrk2;
    FrameIndex j1, j2, iFrm1, iFrm2;
    MeasurementIndex iMea1, iMea2;
    const uint nClusters = uint(iTrkClusters.size());
    for(uint i = 0; i < nClusters; ++i) {
        const TrackIndexList &iTrks1 = iTrkClusters[i].first, &iTrks2 = iTrkClusters[i].second;
        const TrackIndex nTrks1 = TrackIndex(iTrks1.size()), nTrks2 = TrackIndex(iTrks2.size());
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_NAIVE
        const float idf = 1.0f;
#else
        FrameIndex Ni = 0;
        for(i1 = 0; i1 < nTrks1; ++i1)
            Ni += seq1.GetTrackLength(iTrks1[i1]);
        for(i2 = 0; i2 < nTrks2; ++i2)
            Ni += seq2.GetTrackLength(iTrks2[i2]);
        if(Ni > N)
            continue;
        const float idf = log(float(N) / Ni);
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
        const float idfSq = idf * idf;
#endif
#endif
        for(i1 = 0; i1 < nTrks1; ++i1)
            for(i2 = 0; i2 < nTrks2; ++i2) {
                iTrk1 = iTrks1[i1];
                iTrk2 = iTrks2[i2];
                const MeasurementIndexList &iMeas1 = seq1.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq2.GetTrackMeasurementIndexList(iTrk2);
                const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
                for(j1 = 0; j1 < nCrsps1; ++j1) {
                    iMea1 = iMeas1[j1];
                    iFrm1 = seq1.GetMeasurementFrameIndex(iMea1);
                    for(j2 = 0; j2 < nCrsps2; ++j2) {
                        iMea2 = iMeas2[j2];
                        iFrm2 = seq2.GetMeasurementFrameIndex(iMea2);
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
                        idfSqSums1[iFrm1] += idfSq;
                        idfSqSums2[iFrm2] += idfSq;
#else
                        m_matchingMatrixInitImg[iFrm1][iFrm2] += idf;
#endif
                    }
                }
            }
    }
#if MATCHING_MATRIX_INITIALIZATION_METHOD == MATCHING_MATRIX_INITIALIZATION_METHOD_TFIDF
    std::vector<float> &norms1 = idfSqSums1, &norms2 = idfSqSums2/*, chks1(nFrms1, 0), chks2(nFrms2, 0)*/;
    for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
        norms1[iFrm1] = seq1.GetFrameFeaturesNumber(iFrm1) / sqrt(idfSqSums1[iFrm1]);
    for(iFrm2 = 0; iFrm2 < nFrms2; ++iFrm2)
        norms2[iFrm2] = seq2.GetFrameFeaturesNumber(iFrm2) / sqrt(idfSqSums2[iFrm2]);
    for(uint i = 0; i < nClusters; ++i) {
        const TrackIndexList &iTrks1 = iTrkClusters[i].first, &iTrks2 = iTrkClusters[i].second;
        const TrackIndex nTrks1 = TrackIndex(iTrks1.size()), nTrks2 = TrackIndex(iTrks2.size());
        for(i1 = 0; i1 < nTrks1; ++i1)
            for(i2 = 0; i2 < nTrks2; ++i2) {
                iTrk1 = iTrks1[i1];
                iTrk2 = iTrks2[i2];
                const MeasurementIndexList &iMeas1 = seq1.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq2.GetTrackMeasurementIndexList(iTrk2);
                const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
                for(j1 = 0; j1 < nCrsps1; ++j1) {
                    iMea1 = iMeas1[j1];
                    iFrm1 = seq1.GetMeasurementFrameIndex(iMea1);
                    for(j2 = 0; j2 < nCrsps2; ++j2) {
                        iMea2 = iMeas2[j2];
                        iFrm2 = seq2.GetMeasurementFrameIndex(iMea2);
                        m_matchingMatrixInitImg[iFrm1][iFrm2] += norms1[iFrm1] * norms2[iFrm2];
                    }
                }
            }
    }
#endif
    float confidence, confidenceMax = 0;
    for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
        for(iFrm2 = 0; iFrm2 < nFrms2; ++iFrm2) {
            if((confidence = m_matchingMatrixInitImg[iFrm1][iFrm2]) > confidenceMax)
                confidenceMax = confidence;
        }
    const float norm = 1.0f / confidenceMax;
    const float confidenceMin = confidenceMax * m_enftMinConfidenceInitMatchingMatrixRatio;

    for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
        for(iFrm2 = 0; iFrm2 < nFrms2; ++iFrm2) {
            if(m_matchingMatrixInitImg[iFrm1][iFrm2] > confidenceMin)
                m_img[iFrm1][iFrm2] = m_matchingMatrixInitImg[iFrm1][iFrm2] * norm;
        }

    m_matchingMatrixInitTex.Bind();
    m_matchingMatrixInitTex.UploadFromCPU(m_img.data());
    //ScaleMatchingMatrixTexture(m_matchingMatrixInitTex, m_matchingMatrixInitTexScaled);
}

void TrackMatcher::PrepareInitialMatchingMatrixImageAndTexture(const Sequence &seq1, const Sequence &seq2, const SparseMatrix<float> &matchingMatrixInit) {
    memset(m_matchingMatrixInitImg.data(), 0, sizeof(float) * m_matchingMatrixInitImg.totalsize());
    memset(m_img.data(), 0, sizeof(float) * m_img.totalsize());

    FrameIndex iFrm1, iFrm2;
    const FrameIndex nFrms1 = seq1.GetFramesNumber(), nFrms2 = seq2.GetFramesNumber();
    for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1) {
        const FullIndexValueList<float> &elements = matchingMatrixInit.GetRowData(iFrm1);
        const FrameIndex nElements = FrameIndex(elements.size());
        for(FrameIndex i = 0; i < nElements; ++i) {
            iFrm2 = elements[i].GetFullIndex();
            m_matchingMatrixInitImg[iFrm1][iFrm2] = elements[i].GetValue();
        }
    }

    float confidence, confidenceMax = 0;
    for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
        for(iFrm2 = 0; iFrm2 < nFrms2; ++iFrm2) {
            if((confidence = m_matchingMatrixInitImg[iFrm1][iFrm2]) > confidenceMax)
                confidenceMax = confidence;
        }
    const float norm = 1.0f / confidenceMax;
    const float confidenceMin = confidenceMax * m_enftMinConfidenceInitMatchingMatrixRatio;

    for(iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
        for(iFrm2 = 0; iFrm2 < nFrms2; ++iFrm2) {
            if(m_matchingMatrixInitImg[iFrm1][iFrm2] > confidenceMin)
                m_img[iFrm1][iFrm2] = m_img[iFrm2][iFrm1] = m_matchingMatrixInitImg[iFrm1][iFrm2] * norm;
        }

    m_matchingMatrixInitTex.Bind();
    m_matchingMatrixInitTex.UploadFromCPU(m_img.data());
    //ScaleMatchingMatrixTexture(m_matchingMatrixInitTex, m_matchingMatrixInitTexScaled);
}

void TrackMatcher::PrepareUpdatingMatchingMatrixImageAndTexture(const Sequence &seq, const FrameIndex &iFrm1Upd, const FrameIndex &iFrm2Upd,
        const FeatureMatchList &matches) {
    if(!m_view)
        return;
    //memset(m_matchingMatrixUpdImg.data(), 0, sizeof(ushort) * m_matchingMatrixUpdImg.totalsize());
    //memset(m_img.data(), 0, sizeof(float) * m_img.totalsize());

    const float incr = 1.0f / m_maxNumFtrsPerImg;
    //const TrackIndexList &iTrks1To2 = m_matchingMatrix.GetTrackIndexList1To2();
    //const std::vector<short> &iTrks1To2Voting = m_matchingMatrix.GetTrackIndexList1To2Voting();
    const TrackIndex *iTrks1 = seq.GetFrameTrackIndexes(iFrm1Upd), *iTrks2 = seq.GetFrameTrackIndexes(iFrm2Upd);

    FrameIndex i1, i2, iFrm1, iFrm2, iFrmMin, iFrmMax;
    TrackIndex iTrk1, iTrk2;
    //const TrackIndex nTrks1 = seq.GetTracksNumber();
    //for(iTrk1 = 0; iTrk1 < nTrks1; ++iTrk1)
    const ushort nMatches = ushort(matches.size());
    for(ushort i = 0; i < nMatches; ++i) {
        //if((iTrk2 = iTrks1To2[iTrk1]) == INVALID_TRACK_INDEX/* || iTrks1To2Voting[iTrk1] <= 0*/)
        //  continue;
        iTrk1 = iTrks1[matches[i].GetIndex1()];
        iTrk2 = iTrks2[matches[i].GetIndex2()];
        const MeasurementIndexList &iMeas1 = seq.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq.GetTrackMeasurementIndexList(iTrk2);
        const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
        for(i1 = 0; i1 < nCrsps1; ++i1) {
            iFrm1 = seq.GetMeasurementFrameIndex(iMeas1[i1]);
            for(i2 = 0; i2 < nCrsps2; ++i2) {
                iFrm2 = seq.GetMeasurementFrameIndex(iMeas2[i2]);
                if(iFrm1 < iFrm2) {
                    iFrmMin = iFrm1;
                    iFrmMax = iFrm2;
                } else {
                    iFrmMin = iFrm2;
                    iFrmMax = iFrm1;
                }
                ++m_matchingMatrixUpdImg[iFrmMin][iFrmMax];
                m_img[iFrmMin][iFrmMax] += incr;
            }
        }
    }
    const FrameIndex nFrms = seq.GetFramesNumber();
    for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1)
        for(iFrm2 = 0; iFrm2 < iFrm1; ++iFrm2) {
            m_matchingMatrixUpdImg[iFrm1][iFrm2] = m_matchingMatrixUpdImg[iFrm2][iFrm1];
            m_img[iFrm1][iFrm2] = m_img[iFrm2][iFrm1];
        }
    m_matchingMatrixUpdTex.Bind();
    m_matchingMatrixUpdTex.UploadFromCPU(m_img.data());
    ScaleMatchingMatrixTexture(m_matchingMatrixUpdTex, m_matchingMatrixUpdTexScaled);
}

void TrackMatcher::PrepareUpdatingMatchingMatrixImageAndTexture(const Sequence &seq, const TrackMatchList &trkMatches) {
    if(!m_view)
        return;
    memset(m_matchingMatrixUpdImg.data(), 0, sizeof(ushort) * m_matchingMatrixUpdImg.totalsize());
    memset(m_img.data(), 0, sizeof(float) * m_img.totalsize());

    const float incr = 1.0f / m_maxNumFtrsPerImg;

    FrameIndex i1, i2, iFrm1, iFrm2, iFrmMin, iFrmMax;
    TrackIndex iTrk1, iTrk2;
    const TrackIndex nMatches = ushort(trkMatches.size());
    for(TrackIndex i = 0; i < nMatches; ++i) {
        trkMatches[i].Get(iTrk1, iTrk2);
        const MeasurementIndexList &iMeas1 = seq.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq.GetTrackMeasurementIndexList(iTrk2);
        const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
        for(i1 = 0; i1 < nCrsps1; ++i1) {
            iFrm1 = seq.GetMeasurementFrameIndex(iMeas1[i1]);
            for(i2 = 0; i2 < nCrsps2; ++i2) {
                iFrm2 = seq.GetMeasurementFrameIndex(iMeas2[i2]);
                if(iFrm1 < iFrm2) {
                    iFrmMin = iFrm1;
                    iFrmMax = iFrm2;
                } else {
                    iFrmMin = iFrm2;
                    iFrmMax = iFrm1;
                }
                ++m_matchingMatrixUpdImg[iFrmMin][iFrmMax];
                m_img[iFrmMin][iFrmMax] += incr;
            }
        }
    }
    const FrameIndex nFrms = seq.GetFramesNumber();
    for(iFrm1 = 0; iFrm1 < nFrms; ++iFrm1)
        for(iFrm2 = 0; iFrm2 < iFrm1; ++iFrm2) {
            m_matchingMatrixUpdImg[iFrm1][iFrm2] = m_matchingMatrixUpdImg[iFrm2][iFrm1];
            m_img[iFrm1][iFrm2] = m_img[iFrm2][iFrm1];
        }
    m_matchingMatrixUpdTex.Bind();
    m_matchingMatrixUpdTex.UploadFromCPU(m_img.data());
    ScaleMatchingMatrixTexture(m_matchingMatrixUpdTex, m_matchingMatrixUpdTexScaled);
}

void TrackMatcher::PrepareUpdatingMatchingMatrixImageAndTexture(const Sequence &seq1, const Sequence &seq2) {
    if(!m_view)
        return;
    memset(m_matchingMatrixUpdImg.data(), 0, sizeof(ushort) * m_matchingMatrixUpdImg.totalsize());
    memset(m_img.data(), 0, sizeof(float) * m_img.totalsize());

    const float incr = 1.0f / m_maxNumFtrsPerImg;

    TrackMatchList trkMatches;
    m_matchingMatrix.GetTrackMatches(trkMatches);

    FrameIndex i1, i2, iFrm1, iFrm2;
    TrackIndex iTrk1, iTrk2;
    const TrackIndex nMatches = TrackIndex(trkMatches.size());
    for(TrackIndex i = 0; i < nMatches; ++i) {
        trkMatches[i].Get(iTrk1, iTrk2);
        const MeasurementIndexList &iMeas1 = seq1.GetTrackMeasurementIndexList(iTrk1), &iMeas2 = seq2.GetTrackMeasurementIndexList(iTrk2);
        const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
        for(i1 = 0; i1 < nCrsps1; ++i1) {
            iFrm1 = seq1.GetMeasurementFrameIndex(iMeas1[i1]);
            for(i2 = 0; i2 < nCrsps2; ++i2) {
                iFrm2 = seq2.GetMeasurementFrameIndex(iMeas2[i2]);
                ++m_matchingMatrixUpdImg[iFrm1][iFrm2];
                m_img[iFrm1][iFrm2] += incr;
            }
        }
    }
    m_matchingMatrixUpdTex.Bind();
    m_matchingMatrixUpdTex.UploadFromCPU(m_img.data());
    ScaleMatchingMatrixTexture(m_matchingMatrixUpdTex, m_matchingMatrixUpdTexScaled);
}

void TrackMatcher::ScaleMatchingMatrixTexture(const TextureGL1 &srcTex, const TextureGL1 &dstTex) {
    if(!m_view)
        return;
    const CVD::ImageRef sizeBkp = m_pWnd->size();
    ProgramGL::FitViewportGL(dstTex);
    m_programScale.Run(m_scale, srcTex, dstTex);
    ProgramGL::FitViewportWindows(sizeBkp.x, sizeBkp.y);
}

bool TrackMatcher::SaveTrackClusters(const char *fileName, const std::vector<TrackIndexList> &iTrkClusters) {
    FILE *fp = fopen(fileName, "w");
    if(!fp)
        return false;
    const TrackIndex nClusters = TrackIndex(iTrkClusters.size());
    fprintf(fp, "%d\n", nClusters);
    for(TrackIndex i = 0; i < nClusters; ++i) {
        const TrackIndexList &iTrks = iTrkClusters[i];
        const TrackIndex nTrks = TrackIndex(iTrks.size());
        fprintf(fp, "%d\n", nTrks);
        for(TrackIndex j = 0; j < nTrks; ++j)
            fprintf(fp, " %d", iTrks[j]);
        fprintf(fp, "\n");
    }
    fclose(fp);
#if VERBOSE_TRACK_MATCHING
    printf("Saved \'%s\'\n", IO::RemoveFileDirectory(fileName).c_str());
#endif
    return true;
}

bool TrackMatcher::LoadTrackClusters(const char *fileName, std::vector<TrackIndexList> &iTrkClusters) {
    FILE *fp = fopen(fileName, "r");
    if(!fp)
        return false;
    int int1;
    fscanf(fp, "%d", &int1);
    const TrackIndex nClusters = TrackIndex(int1);
    iTrkClusters.resize(nClusters);
    for(TrackIndex i = 0; i < nClusters; ++i) {
        fscanf(fp, "%d", &int1);
        const TrackIndex nTrks = TrackIndex(int1);
        TrackIndexList &iTrks = iTrkClusters[i];
        iTrks.resize(nTrks);
        for(TrackIndex j = 0; j < nTrks; ++j) {
            fscanf(fp, "%d", &int1);
            iTrks[j] = TrackIndex(int1);
        }
    }
    fclose(fp);
#if VERBOSE_TRACK_MATCHING
    printf("Loaded \'%s\'\n", IO::RemoveFileDirectory(fileName).c_str());
#endif
    return true;
}

bool TrackMatcher::SaveTrackClusters(const char *fileName, const std::vector<TrackIndexListPair> &iTrkClusters) {
    FILE *fp = fopen(fileName, "w");
    if(!fp)
        return false;
    const TrackIndex nClusters = TrackIndex(iTrkClusters.size());
    fprintf(fp, "%d\n", nClusters);
    for(TrackIndex i = 0; i < nClusters; ++i) {
        const TrackIndexList &iTrks1 = iTrkClusters[i].first;
        const TrackIndex nTrks1 = TrackIndex(iTrks1.size());
        fprintf(fp, "%d\n", nTrks1);
        for(TrackIndex j = 0; j < nTrks1; ++j)
            fprintf(fp, " %d", iTrks1[j]);
        fprintf(fp, "\n");

        const TrackIndexList &iTrks2 = iTrkClusters[i].second;
        const TrackIndex nTrks2 = TrackIndex(iTrks2.size());
        fprintf(fp, "%d\n", nTrks2);
        for(TrackIndex j = 0; j < nTrks2; ++j)
            fprintf(fp, " %d", iTrks2[j]);
        fprintf(fp, "\n");
    }
    fclose(fp);
#if VERBOSE_TRACK_MATCHING
    printf("Saved \'%s\'\n", IO::RemoveFileDirectory(fileName).c_str());
#endif
    return true;
}

bool TrackMatcher::LoadTrackClusters(const char *fileName, std::vector<TrackIndexListPair> &iTrkClusters) {
    FILE *fp = fopen(fileName, "r");
    if(!fp)
        return false;
    int int1;
    fscanf(fp, "%d", &int1);
    const TrackIndex nClusters = TrackIndex(int1);
    iTrkClusters.resize(nClusters);
    for(TrackIndex i = 0; i < nClusters; ++i) {
        fscanf(fp, "%d", &int1);
        const TrackIndex nTrks1 = TrackIndex(int1);
        TrackIndexList &iTrks1 = iTrkClusters[i].first;
        iTrks1.resize(nTrks1);
        for(TrackIndex j = 0; j < nTrks1; ++j) {
            fscanf(fp, "%d", &int1);
            iTrks1[j] = TrackIndex(int1);
        }

        fscanf(fp, "%d", &int1);
        const TrackIndex nTrks2 = TrackIndex(int1);
        TrackIndexList &iTrks2 = iTrkClusters[i].second;
        iTrks2.resize(nTrks2);
        for(TrackIndex j = 0; j < nTrks2; ++j) {
            fscanf(fp, "%d", &int1);
            iTrks2[j] = TrackIndex(int1);
        }
    }
    fclose(fp);
#if VERBOSE_TRACK_MATCHING
    printf("Loaded \'%s\'\n", IO::RemoveFileDirectory(fileName).c_str());
#endif
    return true;
}

bool TrackMatcher::SaveTrackClusters(const char *fileName, const std::vector<std::vector<TrackIndexListPair> > &iTrkClustersList) {
    FILE *fp = fopen(fileName, "w");
    if(!fp)
        return false;
    const int N = int(iTrkClustersList.size());
    fprintf(fp, "%d\n", N);
    for(int i = 0; i < N; ++i) {
        const std::vector<TrackIndexListPair> &iTrkClusters = iTrkClustersList[i];
        const TrackIndex nClusters = TrackIndex(iTrkClusters.size());
        fprintf(fp, "%d\n", nClusters);
        for(TrackIndex j = 0; j < nClusters; ++j) {
            const TrackIndexList &iTrks1 = iTrkClusters[j].first;
            const TrackIndex nTrks1 = TrackIndex(iTrks1.size());
            fprintf(fp, "%d\n", nTrks1);
            for(TrackIndex k = 0; k < nTrks1; ++k)
                fprintf(fp, " %d", iTrks1[k]);
            fprintf(fp, "\n");

            const TrackIndexList &iTrks2 = iTrkClusters[j].second;
            const TrackIndex nTrks2 = TrackIndex(iTrks2.size());
            fprintf(fp, "%d\n", nTrks2);
            for(TrackIndex k = 0; k < nTrks2; ++k)
                fprintf(fp, " %d", iTrks2[k]);
            fprintf(fp, "\n");
        }
    }
    fclose(fp);
#if VERBOSE_TRACK_MATCHING
    printf("Saved \'%s\'\n", IO::RemoveFileDirectory(fileName).c_str());
#endif
    return true;
}

bool TrackMatcher::LoadTrackClusters(const char *fileName, std::vector<std::vector<TrackIndexListPair> > &iTrkClustersList) {
    FILE *fp = fopen(fileName, "r");
    if(!fp)
        return false;
    int int1;
    fscanf(fp, "%d", &int1);
    const int N = int(int1);
    iTrkClustersList.resize(N);
    for(int i = 0; i < N; ++i) {
        fscanf(fp, "%d", &int1);
        const TrackIndex nClusters = TrackIndex(int1);
        std::vector<TrackIndexListPair> &iTrkClusters = iTrkClustersList[i];
        iTrkClusters.resize(nClusters);
        for(TrackIndex j = 0; j < nClusters; ++j) {
            fscanf(fp, "%d", &int1);
            const TrackIndex nTrks1 = TrackIndex(int1);
            TrackIndexList &iTrks1 = iTrkClusters[j].first;
            iTrks1.resize(nTrks1);
            for(TrackIndex k = 0; k < nTrks1; ++k) {
                fscanf(fp, "%d", &int1);
                iTrks1[k] = TrackIndex(int1);
            }

            fscanf(fp, "%d", &int1);
            const TrackIndex nTrks2 = TrackIndex(int1);
            TrackIndexList &iTrks2 = iTrkClusters[j].second;
            iTrks2.resize(nTrks2);
            for(TrackIndex k = 0; k < nTrks2; ++k) {
                fscanf(fp, "%d", &int1);
                iTrks2[k] = TrackIndex(int1);
            }
        }
    }
    fclose(fp);
#if VERBOSE_TRACK_MATCHING
    printf("Loaded \'%s\'\n", IO::RemoveFileDirectory(fileName).c_str());
#endif
    return true;
}

bool TrackMatcher::SaveTrackMatches(const char *fileName, const TrackMatchList &matches) {
    FILE *fp = fopen(fileName, "w");
    if(!fp)
        return false;
    const TrackIndex nMatches = TrackIndex(matches.size());
    fprintf(fp, "%d\n", nMatches);
    for(TrackIndex i = 0; i < nMatches; ++i)
        fprintf(fp, "%d %d\n", matches[i].GetIndex1(), matches[i].GetIndex2());
#if VERBOSE_TRACK_MATCHING
    printf("Saved \'%s\', %d matches\n", IO::RemoveFileDirectory(fileName).c_str(), nMatches);
#endif
    fclose(fp);
    return true;
}

bool TrackMatcher::LoadTrackMatches(const char *fileName, TrackMatchList &matches) {
    FILE *fp = fopen(fileName, "r");
    if(!fp)
        return false;
    int int1, int2;
    fscanf(fp, "%d", &int1);
    const TrackIndex nMatches = TrackIndex(int1);
    matches.resize(nMatches);
    for(TrackIndex i = 0; i < nMatches; ++i) {
        fscanf(fp, "%d %d", &int1, &int2);
        matches[i].Set(TrackIndex(int1), TrackIndex(int2));
    }
#if VERBOSE_TRACK_MATCHING
    printf("Loaded \'%s\', %d matches\n", IO::RemoveFileDirectory(fileName).c_str(), nMatches);
#endif
    fclose(fp);
    return true;
}

template<typename TYPE>
static inline void LoadImage(CVD::Image<TYPE> &img1, CVD::Image<TYPE> &img2, const std::string &fileName) {
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

void TrackMatcher::DrawFeawtureMatches(const Sequence &seq, const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FeatureMatchList &matchesExist,
                                       const std::vector<ushort> &inliers, const std::vector<ushort> &outliers, const FeatureMatchList &matchesNew) {
    CVD::Image<CVD::Rgb<ubyte> > img1, img2, img;
    //CVD::img_load(img1, seq.GetImageFileName(iFrm1));
    //CVD::img_load(img2, seq.GetImageFileName(iFrm2));
    ::LoadImage(img, img1, seq.GetImageFileName(iFrm1));
    ::LoadImage(img, img2, seq.GetImageFileName(iFrm2));
    img.resize(CVD::ImageRef(img1.size().x, img1.size().y + img2.size().y));
    CVD::copy(img1, img, img1.size());
    CVD::copy(img2, img, img2.size(), CVD::ImageRef(), CVD::ImageRef(0, img1.size().y));

    const Point2D *x1s = seq.GetFrameFeatures(iFrm1), *x2s = seq.GetFrameFeatures(iFrm2);
    const ushort nInliers = ushort(inliers.size());
    for(ushort i = 0; i < nInliers; ++i) {
        const Point2D &x1 = x1s[matchesExist[inliers[i]].GetIndex1()], &x2 = x2s[matchesExist[inliers[i]].GetIndex2()];
        CVD::drawLine(img, CVD::ImageRef(int(x1.x()), int(x1.y())), CVD::ImageRef(int(x2.x()), img1.size().y + int(x2.y())),
                      CVD::Rgb<ubyte>(255, 255, 255));
    }
    //const ushort nOutliers = ushort(outliers.size());
    //for(ushort i = 0; i < nOutliers; ++i)
    //{
    //  const Point2D &x1 = x1s[matchesExist[outliers[i]].GetIndex1()], &x2 = x2s[matchesExist[outliers[i]].GetIndex2()];
    //  CVD::drawLine(img, CVD::ImageRef(int(x1.x()), int(x1.y())), CVD::ImageRef(int(x2.x()), img1.size().y + int(x2.y())),
    //      CVD::Rgb<ubyte>(255, 0, 0));
    //}
    const ushort nMatchesNew = ushort(matchesNew.size());
    for(ushort i = 0; i < nMatchesNew; ++i) {
        const Point2D &x1 = x1s[matchesNew[i].GetIndex1()], &x2 = x2s[matchesNew[i].GetIndex2()];
        CVD::drawLine(img, CVD::ImageRef(int(x1.x()), int(x1.y())), CVD::ImageRef(int(x2.x()), img1.size().y + int(x2.y())),
                      CVD::Rgb<ubyte>(255, 255, 255));
    }

    //char fileName[MAX_LINE_LENGTH];
    //sprintf(fileName, "E:/matching_matrix_image/%05d.jpg", g_cnt);
    //CVD::img_save(img, fileName);
}

void TrackMatcher::OnDraw() {
    if(m_initView) {
        //m_matchingMatrixInitTexScaled.Bind();
        //Viewer::DrawTexture(m_matchingMatrixInitTexScaled.GetWidth(), m_matchingMatrixInitTexScaled.GetHeight());
        m_matchingMatrixInitTex.Bind();
        Viewer::DrawTexture(m_matchingMatrixInitTex.GetWidth(), m_matchingMatrixInitTex.GetHeight());
    } else {
        m_matchingMatrixUpdTexScaled.Bind();
        Viewer::DrawTexture(m_matchingMatrixUpdTexScaled.GetWidth(), m_matchingMatrixUpdTexScaled.GetHeight());
    }

    glBegin(GL_LINES);
    //glBegin(GL_POINTS);
    //glPointSize(2.0f);
    float x, y;
    if(!m_hideSeed) {
        const Point2D crossSize(CROSS_SIZE, CROSS_SIZE);
        const ushort nIters = ushort(m_crossesList.size());
        for(ushort iter = 0; iter < nIters; ++iter) {
            const CVD::Rgb<ubyte> &clr = m_seedClrs[iter];
            glColor3ub(clr.red, clr.green, clr.blue);

            //if(iter == 0)
            //  glColor3ub(255, 0, 0);
            //else if(iter == 1)
            //  glColor3ub(0, 255, 0);
            //else if(iter == 2)
            //  glColor3ub(0, 0, 255);

            const FrameIndexPairList &crosses = m_crossesList[iter];
            const uint nCrosses = ushort(crosses.size());
            for(uint i = 0; i < nCrosses; ++i) {
                y = crosses[i].GetIndex1() * m_factorFrmToWin.y();
                x = crosses[i].GetIndex2() * m_factorFrmToWin.x();
                Viewer::DrawCross(x, y, crossSize);
                //glVertex2f(x, y);
            }
        }
    }
    //if(m_showKeyFrms)
    //{
    //  glColor3ub(COLOR_KEY_FRAME_R, COLOR_KEY_FRAME_G, COLOR_KEY_FRAME_B);
    //  x = float(m_pWnd->size().x);
    //  const FrameIndex nFrms1 = m_matchingMatrix.GetSequence1().GetFramesNumber();
    //  for(FrameIndex iFrm1 = 0; iFrm1 < nFrms1; ++iFrm1)
    //  {
    //      if(!(m_matchingMatrix.GetSequence1().GetFrameState(iFrm1) & FLAG_FRAME_STATE_KEY_FRAME))
    //          continue;
    //      y = iFrm1 * m_factorFrmToWin.y();
    //      glVertex2f(0, y);
    //      glVertex2f(x, y);
    //  }
    //  y = float(m_pWnd->size().y);
    //  const FrameIndex nFrms2 = m_matchingMatrix.GetSequence2().GetFramesNumber();
    //  for(FrameIndex iFrm2 = 0; iFrm2 < nFrms2; ++iFrm2)
    //  {
    //      if(!(m_matchingMatrix.GetSequence2().GetFrameState(iFrm2) & FLAG_FRAME_STATE_KEY_FRAME))
    //          continue;
    //      x = iFrm2 * m_factorFrmToWin.x();
    //      glVertex2f(x, 0);
    //      glVertex2f(x, y);
    //  }
    //}
    glEnd();
    //glPointSize(1.0f);
}

void TrackMatcher::OnDrawString() {
    glColor3ub(STRING_COLOR_R, STRING_COLOR_G, STRING_COLOR_B);
    if(m_initView)
        Viewer::DrawStringTopLeftLarge("Initial Matching Matrix: %f", m_matchingMatrixInitImg[m_iFrm1][m_iFrm2]);
    else
        Viewer::DrawStringTopLeftLarge("Updating Matching Matrix: %d", m_matchingMatrixUpdImg[m_iFrm1][m_iFrm2]);
    //if(m_initView && m_matchingMatrixInitImg[m_iFrm1][m_iFrm2] != 0)
    //  Viewer::DrawStringCurrentLarge(", %f", m_matchingMatrix.GetInitialConfidence(m_iFrm1, m_iFrm2));
    Viewer::DrawStringBottomLeftLarge(1, "Frame 1: %d", m_iFrm1);
    Viewer::DrawStringBottomLeftLarge(0, "Frame 2: %d", m_iFrm2);
}

bool TrackMatcher::OnKeyDown(const int key) {
    switch(key) {
        case KEY_SWITCH_VIEW:
            m_initView = !m_initView;
            return true;
        case KEY_STEP_FORWARD:
            m_handler.Quit() = true;
            return true;
        case KEY_STOP_MATCHING:
            m_stop = !m_stop;
            return true;
        case KEY_HIDE_SEED:
            m_hideSeed = !m_hideSeed;
            return true;
        //case KEY_SHOW_KEY_FRAMES:
        //  m_showKeyFrms = !m_showKeyFrms;
        //  return true;
        case KEY_SCALE_INCREASE:
        case KEY_SCALE_DECREASE:
            if(key == KEY_SCALE_INCREASE) {
                if(m_scale >= 1.0f)
                    ++m_scale;
                else
                    m_scale = 1 / float(int(1 / m_scale + 0.5f) - 1);
            } else {
                if(m_scale <= 1.0f)
                    m_scale = 1 / float(int(1 / m_scale + 0.5f) + 1);
                else
                    --m_scale;
            }
            ProgramGL::BindFrameBuffer();
            //ScaleMatchingMatrixTexture(m_matchingMatrixInitTex, m_matchingMatrixInitTexScaled);
            ScaleMatchingMatrixTexture(m_matchingMatrixUpdTex, m_matchingMatrixUpdTexScaled);
            ProgramGL::UnbindFrameBuffer();
            return true;
    }
    return false;
}

void TrackMatcher::OnMouseMove(const CVD::ImageRef &where) {
    m_iFrm1 = FrameIndex(where.y * m_factorWinToFrm.y());
    m_iFrm2 = FrameIndex(where.x * m_factorWinToFrm.x());
}

void TrackMatcher::OnResize(const CVD::ImageRef &size) {
    m_factorFrmToWin.Set(float(size.x) / m_matchingMatrix.GetSequence2().GetFramesNumber(), float(size.y) / m_matchingMatrix.GetSequence1().GetFramesNumber());
    m_factorWinToFrm.Set(1 / m_factorFrmToWin.x(), 1 / m_factorFrmToWin.y());
}