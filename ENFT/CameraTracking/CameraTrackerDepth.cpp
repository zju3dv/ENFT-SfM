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
#include "Utility/Timer.h"

using namespace ENFT_SfM;

void CameraTrackerDepth::Initialize(const SequenceDepth &seq, const std::string paramFileName) {
    Configurator param;
    param.Load(paramFileName.c_str());
    Initialize(seq, param);
}

void CameraTrackerDepth::Run(SequenceDepth &seq, const std::string outputFileName, FILE *fpTiming) {
    if(outputFileName != "" && seq.Sequence::LoadB(outputFileName.c_str()))
        return;

    Timer timer;
    timer.Start();

    const bool stopBkp = m_stop;

#if VERBOSE_CAMERA_TRACKING
    printf("****************************************************************\n");
#endif

#if 1
    seq.InitializeCameras();
    seq.InitializePoints();
    seq.InitializeMeasurements();
    seq.DenormalizeMeasurements();
    seq.LoadDepths();
    ExtractKeyFrameSequence(seq, m_seqKF, m_iFrmsKF, m_iTrksKF, m_iMeasKF);
    //m_seqKF.LoadDepths();
    m_seqKF.NormalizeMeasurements();
    const FrameIndex nFrmsKF = m_seqKF.GetFramesNumber();
    m_frmErrLevelsKF.resize(nFrmsKF);
    for(FrameIndex iFrm = 0; iFrm < nFrmsKF; ++iFrm)
        m_frmErrLevelsKF[iFrm].SetLowest();
    //SaveB("F:/tmp/tmpKF.txt", seq, m_seqKF, m_iFrmsKF, m_iTrksKF, m_iMeasKF, m_frmErrLevelsKF);
    //exit(0);
#else
    LoadB("F:/tmp/tmpKF.txt", seq, m_seqKF, m_iFrmsKF, m_iTrksKF, m_iMeasKF, m_frmErrLevelsKF);
#endif

    FrameIndex iFrm1, iFrm2, iFrm3;
#if 1
    SelectInitialFrames(m_seqKF, iFrm1, iFrm2, iFrm3);
    //SaveB("F:/tmp/tmpInit.txt", m_seqKF, iFrm1, iFrm2, iFrm3);
    //exit(0);
#else
    LoadB("F:/tmp/tmpInit.txt", m_seqKF, iFrm1, iFrm2, iFrm3);
#endif

    EstimateInitialStructureAndMotion(iFrm1, iFrm2, iFrm3, m_seqKF);
    m_stop = stopBkp;
    ViewSequence(m_seqKF, iFrm1);

    RegisterKeyFrameSequence(m_seqKF, m_frmErrLevelsKF, iFrm1, iFrm3, true);
#if VERBOSE_CAMERA_TRACKING
    printf("****************************************************************\n");
#endif
    m_seqKF.PrintStates();
    m_stop = stopBkp;
    ViewSequence(m_seqKF, iFrm1);

    RegisterOriginalSequence(m_seqKF, m_frmErrLevelsKF, iFrm1, iFrm3, m_iFrmsKF, m_iTrksKF, m_iMeasKF, seq);

#if VERBOSE_CAMERA_TRACKING
    printf("****************************************************************\n");
#endif
    seq.PrintStates();
    m_stop = stopBkp;
    ViewSequence(seq, m_iFrmsKF[iFrm1]);
    m_stop = stopBkp;

    timer.Stop();
    timer.PrintTotalTiming(0, fpTiming);

    if(outputFileName != "")
        seq.Sequence::SaveB(outputFileName.c_str());
}

void CameraTrackerDepth::Initialize(const SequenceDepth &seq, const Configurator &param) {
    CameraTracker::Initialize(seq, param, true);
    const float errTh2D = param.GetArgument("error_threshold_2d", 8.0f);
    m_Cestor.m_errTh2D = m_Xestor.m_errTh2D = m_Testor.m_ransacErrorThreshold = errTh2D * errTh2D * seq.GetIntrinsicMatrix().one_over_fxy();
    m_Cestor.m_errTh3D = m_Xestor.m_errTh3D = m_depthErrTh = param.GetArgument("error_threshold_3d", 0.1f);
    m_Testor.m_ransacErrorThreshold = m_Eestor.m_ransacErrorThreshold;

    const float sigmaReproj = param.GetArgument("sigma_reprojection", 2.0f) * sqrt(seq.GetIntrinsicMatrix().one_over_fxy());
    const float sigmaDepth = param.GetArgument("sigma_depth", 0.05f);
    m_depthWeight = param.GetArgument("weight_depth", 1.0f) * sigmaReproj / sigmaDepth;
    m_Cdata.SetDepthWeight(m_depthWeight);
    m_Xdata.SetDepthWeight(m_depthWeight);
    m_baData.SetDepthWeight(m_depthWeight);
}

void CameraTrackerDepth::SelectInitialFrames(SequenceDepth &seq, FrameIndex &iFrm1, FrameIndex &iFrm2, FrameIndex &iFrm3) {
    FrameIndex i;
    const FrameIndex nTriples = seq.GetFramesNumber() - 2;
    //m_frmTripleScores.assign(nTriples, 0.0f);
    m_frmTripleScores.assign(nTriples, -FLT_MAX);
    for(i = 0; i < nTriples; ++i) {
        iFrm1 = i;
        iFrm2 = i + 1;
        iFrm3 = i + 2;
        if(!EstimateInitialStructureAndMotion(iFrm1, iFrm2, iFrm3, seq))
            continue;
        //m_frmTripleScores[i] = ComputeDepthConfidence(seq, iFrm1) + ComputeDepthConfidence(seq, iFrm2) + ComputeDepthConfidence(seq, iFrm3);
        //m_frmTripleScores[i] = -(seq.ComputeFrameMSE(iFrm1) + seq.ComputeFrameMSE(iFrm2) + seq.ComputeFrameMSE(iFrm3));
        m_frmTripleScores[i] = -(ComputeFrameMSE(seq, iFrm1) + ComputeFrameMSE(seq, iFrm2) + ComputeFrameMSE(seq, iFrm3));
        ViewSequence(seq, iFrm1);
    }
    CameraTracker::SelectInitialFrames(m_frmTripleScores, iFrm1, iFrm2, iFrm3);
}
