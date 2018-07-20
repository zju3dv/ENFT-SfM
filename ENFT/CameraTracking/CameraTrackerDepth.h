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

#ifndef _CAMERA_TRACKER_DEPTH_H_
#define _CAMERA_TRACKER_DEPTH_H_

#include "CameraTracker.h"
#include "Viewing/ViewerSequenceDepth.h"
#include "SfM/RelativePoseEstimatorDepth.h"
#include "SfM/CameraEstimatorDepth.h"
#include "SfM/Point3DEstimatorDepth.h"
#include "Sequence/SequenceBundleAdjustorDataDepth.h"

namespace ENFT_SfM {

class CameraTrackerDepth : protected CameraTracker, protected ViewerSequenceDepth {

  public:

    void Initialize(const SequenceDepth &seq, const std::string paramFileName);
    void Run(SequenceDepth &seq, const std::string outputFileName, FILE *fpTiming = NULL);
    virtual void RunPointsOptimization(const TrackIndexList &iTrks, Sequence &seq, const bool metric);

  protected:

    float m_depthErrTh, m_depthWeight;

  protected:

    //////////////////////////////////////////////////////////////////////////
    // Basic
    //////////////////////////////////////////////////////////////////////////
    void Initialize(const SequenceDepth &seq, const Configurator &param);
    void SelectInitialFrames(SequenceDepth &seq, FrameIndex &iFrm1, FrameIndex &iFrm2, FrameIndex &iFrm3);

    //////////////////////////////////////////////////////////////////////////
    // SfM
    //////////////////////////////////////////////////////////////////////////
    bool EstimateInitialStructureAndMotion(const FrameIndex &iFrm1, const FrameIndex &iFrm2, const FrameIndex &iFrm3, SequenceDepth &seq);
    float ComputeFrameDepthConfidence(const SequenceDepth &seq, const FrameIndex &iFrm);
    float ComputeFrameMSE(const SequenceDepth &seq, const FrameIndex &iFrm);
    virtual bool EstimateIncrementalMotion_PnP(const FrameIndex &iFrm, Sequence &seq, FrameErrorLevel &frmErrLevel, const bool metric);
    virtual void TransformScene(const FrameIndex &iFrm1, const FrameIndex &iFrm2, Sequence &seq);
    virtual void BundleAdjust(const FrameIndexList &iFrmsAdj, const TrackIndexList &iTrksAdj, FrameIndexList &iFrmsBA, Sequence &seq, const uint &maxNumIters,
                              const float &stopMSE, const float &stopRelativeReduction, const bool metric);
    virtual bool UpdateStructureAndInlierStates(const std::vector<bool> &trkMarks, Sequence &seq, const std::vector<FrameErrorLevel> &frmErrLevels,
            const bool metric, const bool triangulateUnstablePts);

    //////////////////////////////////////////////////////////////////////////
    // IO
    //////////////////////////////////////////////////////////////////////////
    virtual void ViewSequence(const Sequence &seq, const FrameIndex &iFrm);
    virtual bool OnKeyDown(const int key);

  protected:

    SequenceDepth m_seqKF;

    RelativePoseEstimatorDepth m_Testor;
    RelativePoseEstimatorDataDepth m_Tdata;

    CameraEstimatorDepth m_Cestor;
    CameraEstimatorDataDepth m_Cdata;

    Point3DEstimatorDepth m_Xestor;
    Point3DEstimatorDataDepth m_Xdata;

    BundleAdjustorTemplate<SEQUENCE_BA_ARGUMENT_DEPTH> m_ba;
    SequenceBundleAdjustorDataDepth m_baData;

};

}//namespace ENFT_SfM

#endif
