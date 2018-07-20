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
// ENFT.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "FeatureTracking/FeatureTracker.h"
#include "TrackMatching/TrackMatcher.h"
#include "Viewing/ViewerSequenceSet.h"
#include "CameraTracking/CameraTracker.h"
#include "SequenceRegistration/SequenceRegisteror.h"
#include "SfM/KeyFrameExtractor.h"

using namespace ENFT_SfM;

typedef struct Video {
    std::string imgFileName;
    const char *actFileName, *keyFileName, *keyImgFileName, *keyActFileName;
    int start, step, end;
} Video;

void RunENFT(const std::vector<Video> &videos, const char *paramDir_s, const char *outputDir_s,
             const char *calibFileName, bool const_focal = true, bool distortion = false,
             const int nFrmsMin = -1, const int nFrmsMax = -1,
             const bool useTmpFile = true, const bool view = true) {
    FeatureTracker ftrTracker;
    TrackMatcher trkMatcher1, trkMatcher2;
    CameraTracker camTracker;
    SequenceRegisteror seqRegister;
    KeyFrameExtractor kfExtractor;

    bool runGlobalBA = true;
    const int video_num = int(videos.size());
    const std::string paramDir = std::string(paramDir_s);

    //Init SequenceSet, Vs for SfM, seqs for segmentBA
    SequenceSet Vs, seqs;
    Vs.SetDirectory("");
    Vs.SetCalib(calibFileName, const_focal);
    Vs.CreateSequences(SequenceIndex(video_num));

    for (int iVideo = 0; iVideo < video_num; ++iVideo) {
        const Video &video = videos[iVideo];
        const std::string dir = IO::ExtractFileDirectory(video.imgFileName);
        const std::string imgFileName = IO::RemoveFileDirectory(video.imgFileName);

        const int iStart = video.start, iStep = video.step, iEnd = video.end;
        Sequence &V = Vs[iVideo];
        V.SetTag(dir, imgFileName, iStart, iStep, iEnd);
        V.SetCalib(calibFileName, const_focal);

        // Initiation every parts
        ftrTracker.Initialize(V, paramDir + "param_ftr_tracking.txt");
        trkMatcher1.Initialize(V, paramDir + "param_trk_matching.txt");
        camTracker.Initialize(V, paramDir + "param_cam_tracking.txt", !distortion);
        seqRegister.Initialize(Vs, paramDir + "param_seq_registration.txt", !distortion);

        // If useTmpFile, try to load V.txt
        const std::string outputDir = strcmp(outputDir_s, "") == 0 ? V.GetDirectory() : std::string(outputDir_s);
        char fileName[MAX_LINE_LENGTH];
        sprintf(fileName, "v%d.txt", iVideo);
        if (useTmpFile && V.LoadBwithDir((outputDir + fileName).c_str())) {
            continue;
        }

        // Feature matching and track matching
        sprintf(fileName, "v%d-ftr.txt", iVideo);
        ftrTracker.Run(V, useTmpFile ? outputDir + fileName : "");
        sprintf(fileName, "v%d-trk.txt", iVideo);
        trkMatcher1.Run(V, useTmpFile ? outputDir + fileName : "");

        const FrameIndex nFrms = V.GetFramesNumberTotal();
        if (nFrmsMax == -1 || int(nFrms) < nFrmsMax) {
            sprintf(fileName, "v%d-cam.txt", iVideo);
            camTracker.Run(V, useTmpFile ? outputDir + fileName : "");
        } else {
            // Slipt sequence and run seqRgister
            FrameIndex nFrmsPerSeq = FrameIndex(nFrmsMax);
            while (nFrms - (nFrms / nFrmsPerSeq) * nFrmsPerSeq < nFrmsMin) {
                --nFrmsPerSeq;
            }
            seqs.SplitSequences(V, nFrmsPerSeq);
            const SequenceIndex nSeqs = seqs.GetSequencesNumber();
            for (SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq) {
                sprintf(fileName, "v%d-cam%02d.txt", iVideo, iSeq);
                camTracker.Run(seqs[iSeq], useTmpFile ? outputDir + fileName : "");
            }

            seqRegister.Run(seqs);

            SequenceFrameIndexList iSeqFrms;
            TrackIndexList iTrksCmn;
            TrackIndexList iTrksCmnToCat;
            SequenceTrackIndexList iSeqTrksIdv;
            std::vector<TrackIndexList> iTrksIdvToCatList;
            SequenceMeasurementIndexList iSeqMeas;
            seqs.ConcatenateSequences(V, iSeqFrms, iTrksCmn, iTrksCmnToCat, iSeqTrksIdv,
                                      iTrksIdvToCatList, iSeqMeas, true, true, true);

            if (runGlobalBA) {
                FrameIndexList iFrmsAdj, iFrmsBA;
                TrackIndexList iTrksAdj;
                V.GetTrackIndexList(FLAG_TRACK_STATE_INLIER, iTrksAdj);
                camTracker.RunPointsOptimization(iTrksAdj, V, true);
                camTracker.RunBundleAdjustmentGlobal(0, INVALID_FRAME_INDEX, iFrmsAdj, iFrmsBA,
                                                     iTrksAdj, V, true);
            }
        }

        if (useTmpFile) {
            sprintf(fileName, "v%d.txt", iVideo);
            V.SaveBwithDir((outputDir + fileName).c_str());
        }
    }

    if (video_num > 1) {
        int iVideo1, iVideo2;
        SequenceIndexPairList iVideoPairs;
        for (iVideo1 = 0; iVideo1 < video_num; ++iVideo1)
            for (iVideo2 = iVideo1 + 1; iVideo2 < video_num; ++iVideo2)
                iVideoPairs.push_back(SequenceIndexPair(SequenceIndex(iVideo1), SequenceIndex(iVideo2)));

        trkMatcher2.Initialize(Vs, paramDir + "param_trk_matching.txt");
        trkMatcher2.Run(Vs, iVideoPairs, useTmpFile ? "v-tm.txt" : "", "", "", NULL, false);

        seqRegister.Initialize(Vs, paramDir + "param_seq_registration.txt", true);
        seqRegister.Run(Vs);
    }

    float depthAvg, depthMin, depthMax;
    Vs[0].ComputeFrameDepthRange(0, 0.8f, depthAvg, depthMin, depthMax);
    const float scale = 300.0f / depthAvg;

    FrameIndexList iFrmsKF;
    TrackIndexList iTrksKF;
    Sequence seqKF;
    for (int iVideo = 0; iVideo < video_num; ++iVideo) {
        Sequence &V = Vs[iVideo];
        V.RemoveOutlierTracksAndMeasurements();
        V.ScaleScene(scale);
        const Video &video = videos[iVideo];
        if (video.actFileName) {
            V.SaveAct(video.actFileName);
        }
        if (video.keyFileName || video.keyImgFileName || video.keyActFileName) {
            kfExtractor.Initialize(paramDir + "param_kf_extraction.txt");
            kfExtractor.Run(V, iFrmsKF);
            if (video.keyFileName) {
                CreateDirectory(IO::ExtractFileDirectory(video.keyFileName).c_str(), 0);
                FILE *fp = fopen( video.keyFileName, "w");
                const FrameIndex nFrms = FrameIndex(iFrmsKF.size());
                for (FrameIndex i = 0; i < nFrms; ++i) {
                    fprintf(fp, "%d\n", iFrmsKF[i]);
                }
                fclose(fp);
                printf("Saved \'%s\'\n", video.keyFileName);
            }
            if (video.keyImgFileName || video.keyActFileName) {
                V.GetSubSequence(iFrmsKF, seqKF, iTrksKF, true, true);
                if (video.keyImgFileName) {
                    const std::string seqDirKF = IO::ExtractFileDirectory(video.keyImgFileName);
                    const std::string seqNameKF = IO::RemoveFileDirectory(video.keyImgFileName);
                    seqKF.ChangeTag(seqDirKF, seqNameKF, 0, 1, int(iFrmsKF.size() - 1), true);
                }
                if (video.keyActFileName) {
                    const std::string seqDirKF = video.keyImgFileName ? IO::ExtractFileDirectory(
                                                     video.keyImgFileName) : V.GetDirectory();
                    const std::string actFileName = video.keyActFileName;
                    seqKF.ChangeTag(seqDirKF, "", 0, 1, int(iFrmsKF.size() - 1), false);
                    seqKF.RemoveSingleTracksAndMeasurements();
                    if (runGlobalBA) {
                        FrameIndexList iFrmsAdj, iFrmsBA;
                        TrackIndexList iTrksAdj;
                        seqKF.GetTrackIndexList(FLAG_TRACK_STATE_INLIER, iTrksAdj);
                        camTracker.RunPointsOptimization(iTrksAdj, V, true);
                        camTracker.RunBundleAdjustmentGlobal(0, INVALID_FRAME_INDEX, iFrmsAdj, iFrmsBA,
                                                             iTrksAdj, seqKF, true);
                    }
                    seqKF.SaveAct(actFileName.c_str());
                }
            }
        }
    }

    if (view) {
        if (video_num > 1) {
            ViewerSequenceSet viewer;
            viewer.Run(Vs);
        } else {
            ViewerSequence viewer;
            viewer.Run(Vs[0]);
        }
    }
}

int main(int argc, char *argv[]) {
    // Load Config
    if (argc != 2) {
        printf("Usage: %s <path-to-config>\n", argv[0]);
        exit(0);
    }

    Configurator cfgor;
    if (!cfgor.Load(argv[1])) {
        printf("open config error");
        exit(0);
    }

    // Init Windows Size
    const ushort width = ushort(cfgor.GetArgument("window_width", 0));
    const ushort height = ushort(cfgor.GetArgument("window_height", 0));

    // Load Videos Infomation
    char buf[MAX_LINE_LENGTH];
    std::vector<Video> videos;
    const int videos_number = cfgor.GetArgument("videos_number", 0);
    videos.resize(videos_number);
    for (int i = 0; i < videos_number; ++i) {
        Video &video = videos[i];
        video.actFileName = NULL;
        video.keyFileName = NULL;
        video.keyImgFileName = NULL;
        video.keyActFileName = NULL;
        sprintf(buf,  "video_%d_image", i);
        video.imgFileName = cfgor.GetArgument(buf);
        sprintf(buf,  "video_%d_start", i);
        video.start = cfgor.GetArgument(buf, 0);
        sprintf(buf,  "video_%d_step", i);
        video.step = cfgor.GetArgument(buf, 0);
        sprintf(buf,  "video_%d_end", i);
        video.end = cfgor.GetArgument(buf, 0);
        if (video.start < 0 || video.step < 0 || video.end < 0) {
            printf("check the video_start|step|end\n");
            exit(0);
        }
    }

    // Run ENFT
    glutInit(&argc, argv);
    ProgramGL::Initialize(width, height);
    RunENFT(videos, cfgor.GetArgument("param_directory", "").c_str(),
            cfgor.GetArgument("output_directory", "").c_str(),
            cfgor.GetArgument("calib_file_name", "").c_str(),
            cfgor.GetArgument("const_focal", 1) != 0,
            cfgor.GetArgument("radio_distortion", 1) != 0,
            cfgor.GetArgument("min_frame_number", -1),
            cfgor.GetArgument("max_frame_number", -1),
            cfgor.GetArgument("use_temporary_file", 1) != 0,
            cfgor.GetArgument("view", 1) != 0);
    return 0;
}

