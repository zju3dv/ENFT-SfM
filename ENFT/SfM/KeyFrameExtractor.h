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
#pragma once

#include "Sequence/Sequence.h"
#include "Utility/Configurator.h"

namespace ENFT_SfM {

class KeyFrameExtractor {
  public:
    inline void Initialize(const std::string paramFileName) {
        Configurator param;
        param.Load(paramFileName.c_str());
        Initialize(param);
    }
    inline void Initialize(const Configurator &cfgor) {
        m_dispThRatio = cfgor.GetArgument("kf_disp_threshold_ratio", 0.03f);
        m_minNumCmnTrksTwoKeyFrms =
            cfgor.GetArgument("kf_min_common_tracks_number_two_key_frames", 100);
        m_minNumCmnTrksThreeKeyFrms =
            cfgor.GetArgument("kf_min_common_tracks_number_three_key_frames", 100);
    }
    inline void Run(const Sequence &seq, FrameIndexList &iFrmsKF) {
        const int w = int(seq.GetImageWidth()), h = int(seq.GetImageHeight());
        const float dispTh = m_dispThRatio * sqrt(float(w * w + h * h));

        FeatureIndexList mapTrkToFtr2;
        std::vector<bool> trkMarks12;
        iFrmsKF.assign(1, 0);
        FrameIndex iFrm1, iFrm2, iFrm3;
        TrackIndex iTrk;
        MeasurementIndex iMea;
        FeatureIndex nCmnPts23;
        const FrameIndex nFrms = seq.GetFramesNumber();
        const TrackIndex nTrks = seq.GetTracksNumber();
        while (1) {
            iFrm1 = iFrmsKF.size() >= 2 ? iFrmsKF[iFrmsKF.size() - 2] : INVALID_FRAME_INDEX;
            iFrm2 = iFrmsKF.back();
            seq.ComputeFrameFeatureAverageDisparityPass1(iFrm2, mapTrkToFtr2);
            if (iFrm1 != INVALID_FRAME_INDEX) {
                trkMarks12.assign(nTrks, false);
                const MeasurementIndex iMea1 = seq.GetFrameFirstMeasurementIndex(iFrm1),
                                       iMea2 = seq.GetFrameFirstMeasurementIndex(iFrm1 + 1);
                for (iMea = iMea1; iMea < iMea2; ++iMea) {
                    iTrk = seq.GetMeasurementTrackIndex(iMea);
                    trkMarks12[iTrk] = mapTrkToFtr2[iTrk] != INVALID_FEATURE_INDEX;
                }
            }
            for (iFrm3 = iFrm2 + 1; iFrm3 < nFrms &&
                    (!(seq.GetFrameState(iFrm3) & FLAG_FRAME_STATE_SOLVED)
                     || (iFrm1 == INVALID_FRAME_INDEX ||
                         seq.CountFrameMarkedTracks(iFrm3, trkMarks12) > m_minNumCmnTrksThreeKeyFrms)
                     && seq.ComputeFrameFeatureAverageDisparityPass2(iFrm2, iFrm3, mapTrkToFtr2,
                             nCmnPts23) < dispTh && nCmnPts23 > m_minNumCmnTrksTwoKeyFrms); ++iFrm3);
            if (--iFrm3 == iFrm2 || !(seq.GetFrameState(iFrm3) & FLAG_FRAME_STATE_SOLVED))
                ++iFrm3;
            if (iFrm3 < nFrms)
                iFrmsKF.push_back(iFrm3);
            else
                break;
        }
        printf("Frames: %d --> %d\n", nFrms, iFrmsKF.size());
    }
  public:
    float m_dispThRatio;
    FeatureIndex m_minNumCmnTrksTwoKeyFrms, m_minNumCmnTrksThreeKeyFrms;
};

}//namespace ENFT_SfM
