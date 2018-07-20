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

using namespace ENFT_SfM;

void TrackMatcher::MatchFeatures(/*const */Sequence &seq1, /*const */Sequence
        &seq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2,
        std::vector<ScoredMatch> &matches) {
    const FeatureIndex nFtrs1 = seq1.GetFrameFeaturesNumber(iFrm1),
                       nFtrs2 = seq2.GetFrameFeaturesNumber(iFrm2);

    UploadDescriptorsFromCPU(seq1, iFrm1, m_descTex1);
    UploadDescriptorsFromCPU(seq2, iFrm2, m_descTex2);
    m_ftrMatcherSift.MatchFeatures(m_descTex1, m_descTex2, nFtrs1, nFtrs2);
    m_ftrMatcherSift.DownloadMatches12ToCPU(nFtrs1, nFtrs2, matches);

}

void TrackMatcher::MatchFeatures(/*const */Sequence &seq1, /*const */Sequence
        &seq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2,
        const FeatureIndexList &iFtrs1, const FeatureIndexList &iFtrs2,
        std::vector<ScoredMatch> &matches) {
    const FeatureIndex nFtrs1 = FeatureIndex(iFtrs1.size()),
                       nFtrs2 = FeatureIndex(iFtrs2.size());

    UploadDescriptorsFromCPU(seq1, iFrm1, iFtrs1, m_descTex1);
    UploadDescriptorsFromCPU(seq2, iFrm2, iFtrs2, m_descTex2);
    m_ftrMatcherSift.MatchFeatures(m_descTex1, m_descTex2, nFtrs1, nFtrs2);
    m_ftrMatcherSift.DownloadMatches12ToCPU(nFtrs1, nFtrs2, matches);
    FeatureIndex iFtr1, iFtr2;
    for (ushort i = 0; i < ushort(matches.size()); ++i) {
        matches[i].Get(iFtr1, iFtr2);
        matches[i].SetIndex1(iFtrs1[iFtr1]);
        matches[i].SetIndex2(iFtrs2[iFtr2]);
    }
}

void TrackMatcher::MatchFeatures(/*const */Sequence &seq1, /*const */Sequence
        &seq2, const FrameIndex &iFrm1, const FrameIndex &iFrm2,
        const FeatureIndexList &iFtrs1, const FeatureIndexList &iFtrs2,
        const FundamentalMatrix &F, FeatureMatchList &matches) {
    FeatureIndex iFtr1, iFtr2;
    const FeatureIndex nFtrs1 = FeatureIndex(iFtrs1.size()),
                       nFtrs2 = FeatureIndex(iFtrs2.size());

    UploadFeaturesAndDescriptorsFromCPU(seq1, iFrm1, iFtrs1, m_ftrTex1, m_descTex1);
    UploadFeaturesAndDescriptorsFromCPU(seq2, iFrm2, iFtrs2, m_ftrTex2, m_descTex2);
    m_ftrMatcherSift.MatchFeatures(m_ftrTex1, m_ftrTex2, m_descTex1, m_descTex2,
                                   nFtrs1, nFtrs2, F, true);
    m_ftrMatcherSift.DownloadMatches12ToCPU(nFtrs1, nFtrs2, matches);
    const ushort nMatches = ushort(matches.size());
    for (ushort i = 0; i < nMatches; ++i) {
        matches[i].Get(iFtr1, iFtr2);
        matches[i].Set(iFtrs1[iFtr1], iFtrs2[iFtr2]);
    }
}

void TrackMatcher::MatchFeatures_VerifyMatches(/*const */Sequence &seq,
        const FrameIndex &iFrm1,
        const FrameIndex &iFrm2, /*const */FeatureMatchList &matchesExist,
        FundamentalMatrix &F, std::vector<ushort> &inliers,
        std::vector<ushort> &outliers, FeatureMatchList &matchesNew) {
    FeatureIndex iFtr1, iFtr2;
    const FeatureIndex nFtrs1 = seq.GetFrameFeaturesNumber(iFrm1),
                       nFtrs2 = seq.GetFrameFeaturesNumber(iFrm2);
    const TrackIndex *iTrks1 = seq.GetFrameTrackIndexes(iFrm1);
    const ushort nMatchesExist1 = ushort(matchesExist.size());
    for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1) {
        if((iFtr2 = seq.SearchTrackForFrameFeatureIndex(iTrks1[iFtr1],
                    iFrm2)) != INVALID_FEATURE_INDEX)
            matchesExist.push_back(FeatureMatch(iFtr1, iFtr2));
    }
    const ushort nMatchesExist2 = ushort(matchesExist.size());

    if(nMatchesExist2 == 0) {
        // Step1: feature matching
        MatchFeatures(seq, seq, iFrm1, iFrm2, m_scoredMatches);

        // Step2: filter outliers by epipolar geometry
        m_Fdata.SetMatches(seq.GetFrameFeatures(iFrm1), seq.GetFrameFeatures(iFrm2),
                           m_scoredMatches, m_orders);
        m_Festor.RunLoProsac(m_Fdata, m_orders, F, inliers/*, 3*/);
        const ushort nInliers = ushort(inliers.size());
        matchesNew.resize(nInliers);
        const ushort nMatchesNew = ushort(m_scoredMatches.size());
        for(ushort i = 0; i < nInliers; ++i) {
            m_scoredMatches[inliers[i]].Get(iFtr1, iFtr2);
            matchesNew[i].Set(iFtr1, iFtr2);
        }
        inliers.resize(0);
        outliers.resize(0);
    } else {
        // Step1: feature matching for unmatched features
        std::vector<bool> &ftrMarks1 = m_marks1, &ftrMarks2 = m_marks2;
        const FeatureIndex nFtrs1 = seq.GetFrameFeaturesNumber(iFrm1),
                           nFtrs2 = seq.GetFrameFeaturesNumber(iFrm2);
        ftrMarks1.assign(nFtrs1, false);
        ftrMarks2.assign(nFtrs2, false);
        for(ushort i = 0; i < nMatchesExist2; ++i) {
            matchesExist[i].Get(iFtr1, iFtr2);
            ftrMarks1[iFtr1] = ftrMarks2[iFtr2] = true;
        }
        m_iFtrs1Unmatched.resize(0);
        for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1) {
            if(!ftrMarks1[iFtr1])
                m_iFtrs1Unmatched.push_back(iFtr1);
        }
        m_iFtrs2Unmatched.resize(0);
        for(iFtr2 = 0; iFtr2 < nFtrs2; ++iFtr2) {
            if(!ftrMarks2[iFtr2])
                m_iFtrs2Unmatched.push_back(iFtr2);
        }

        MatchFeatures(seq, seq, iFrm1, iFrm2, m_iFtrs1Unmatched, m_iFtrs2Unmatched,
                      m_scoredMatches);

        const ushort nMatchesNew = ushort(m_scoredMatches.size());
        for(ushort i = 0; i < nMatchesExist1; ++i) {
            matchesExist[i].Get(iFtr1, iFtr2);
            m_scoredMatches.push_back(ScoredMatch(iFtr1, iFtr2, 0.0f));
        }
        for(ushort i = nMatchesExist1; i < nMatchesExist2; ++i) {
            matchesExist[i].Get(iFtr1, iFtr2);
            m_scoredMatches.push_back(ScoredMatch(iFtr1, iFtr2, 1.0f));
        }
        m_Fdata.SetMatches(seq.GetFrameFeatures(iFrm1), seq.GetFrameFeatures(iFrm2),
                           m_scoredMatches, m_orders);
        m_Festor.RunLoProsac(m_Fdata, m_orders, F, inliers);

        const ushort nMatchesTotal = m_Fdata.Size();
        std::vector<bool> &inlierMarks = m_marks1;
        m_Festor.FromInliersToInlierMarks(inliers, nMatchesTotal, inlierMarks);
        matchesNew.resize(0);
        for(ushort i = 0; i < nMatchesNew; ++i) {
            if(!inlierMarks[i])
                continue;
            m_scoredMatches[i].Get(iFtr1, iFtr2);
            matchesNew.push_back(FeatureMatch(iFtr1, iFtr2));
        }
        inliers.resize(0);
        outliers.resize(0);
        for(ushort i = nMatchesNew, j = 0; j < nMatchesExist1; ++i, ++j) {
            if(inlierMarks[i])
                inliers.push_back(j);
            else
                outliers.push_back(j);
        }
    }

    //ushort i, j;
    //TrackIndex iTrk1, iTrk2;
    //const TrackIndex *iTrks2 = seq.GetFrameTrackIndexes(iFrm2);
    //const ushort nMatchesNew = ushort(matchesNew.size());
    //for(i = j = 0; i < nMatchesNew; ++i)
    //{
    //  matchesNew[i].Get(iFtr1, iFtr2);
    //  iTrk1 = iTrks1[iFtr1];
    //  iTrk2 = iTrks2[iFtr2];
    //  if(iTrk1 != iTrk2 && !seq.AreTracksOverlappingInFrames(iTrk1, iTrk2, m_marks1))
    //      matchesNew[j++].Set(iFtr1, iFtr2);
    //}
    //matchesNew.resize(j);

//#if _DEBUG
//  ViewerFeatureMatching viewer;
//  viewer.Initialize(seq.GetImageWidth(), seq.GetImageHeight(), 1, m_maxNumFtrsPerImg, 0);
//  viewer.SetImage(0, seq.GetImageFileName(iFrm1));
//  viewer.SetImage(1, seq.GetImageFileName(iFrm2));
//  viewer.SetFeatures(0, seq.GetFrameFeaturesNumber(iFrm1), seq.GetFrameFeatures(iFrm1));
//  viewer.SetFeatures(1, seq.GetFrameFeaturesNumber(iFrm2), seq.GetFrameFeatures(iFrm2));
//  viewer.SetSiftMatches(0, matchesNew);
//  viewer.Run();
//#endif
}

void TrackMatcher::MatchFeatures_VerifyMatches(/*const */SequenceSet &seqs,
        const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const FrameIndex &iFrm1,
        const FrameIndex &iFrm2, const FeatureMatchList &matchesExist,
        FundamentalMatrix &F,
        std::vector<ushort> &inliers, std::vector<ushort> &outliers,
        FeatureMatchList &matchesNew) {
    FeatureIndex iFtr1, iFtr2;
    /*const */Sequence &seq1 = seqs[iSeq1], &seq2 = seqs[iSeq2];
    const ushort nMatchesExist = ushort(matchesExist.size());

    inliers.resize(0);
    outliers.resize(0);
    if(nMatchesExist == 0) {
        // Step1: feature matching
        MatchFeatures(seq1, seq2, iFrm1, iFrm2, m_scoredMatches);

        // Step2: filter outliers by epipolar geometry
        m_Fdata.SetMatches(seq1.GetFrameFeatures(iFrm1), seq2.GetFrameFeatures(iFrm2),
                           m_scoredMatches, m_orders);
        m_Festor.RunLoProsac(m_Fdata, m_orders, F, inliers/*, 3*/);
        const ushort nInliers = ushort(inliers.size());
        matchesNew.resize(nInliers);
        for(ushort i = 0; i < nInliers; ++i) {
            m_scoredMatches[inliers[i]].Get(iFtr1, iFtr2);
            matchesNew[i].Set(iFtr1, iFtr2);
        }
        inliers.resize(0);
        outliers.resize(0);

//#if _DEBUG
//      //TextureGL3 tex2Src, tex2Dst;
//      //tex2Src.Generate(seq2.GetImageWidth(), seq2.GetImageHeight());
//      //tex2Dst.Generate(seq1.GetImageWidth(), seq1.GetImageHeight());
//      //CVD::Image<CVD::Rgb<ubyte> > img2;
//      //CVD::img_load(img2, seq2.GetImageFileName(iFrm2));
//      //tex2Src.Bind();
//      //tex2Src.UploadFromCPU((ubyte *) img2.data());
//      //ProgramGLResize programResize;
//      //programResize.Initialize();
//      //ProgramGL::FitViewportGL(tex2Dst);
//      //programResize.Run(tex2Src, tex2Dst);
//      //AlignedVector<Point2D> ftrs2(nFtrs2);
//      //ftrs2.CopyFrom(seq2.GetFrameFeatures(iFrm2));
//      //seq2.GetIntrinsicMatrix().ImageToNormalizedPlaneN(ftrs2);
//      //seq1.GetIntrinsicMatrix().NormalizedPlaneToImageN(ftrs2);
//
//      ViewerFeatureMatching viewer;
//      viewer.Initialize(seq1.GetImageWidth(), seq1.GetImageHeight(), 1, m_maxNumFtrsPerImg, 0);
//      viewer.SetImage(0, seq1.GetImageFileName(iFrm1));
//      viewer.SetImage(1, seq2.GetImageFileName(iFrm2));
//      //viewer.SetImage(1, tex2Dst);
//      viewer.SetFeatures(0, seq1.GetFrameFeaturesNumber(iFrm1), seq1.GetFrameFeatures(iFrm1));
//      viewer.SetFeatures(1, seq2.GetFrameFeaturesNumber(iFrm2), seq2.GetFrameFeatures(iFrm2));
//      //viewer.SetFeatures(1, nFtrs2, ftrs2.Data());
//      viewer.SetSiftMatches(0, m_scoredMatches);
//      viewer.SetSiftInliers(0, inliers);
//      viewer.Run();
//#endif
    } else {
        std::vector<bool> &ftrMarks1 = m_marks1, &ftrMarks2 = m_marks2;
        const FeatureIndex nFtrs1 = seq1.GetFrameFeaturesNumber(iFrm1),
                           nFtrs2 = seq2.GetFrameFeaturesNumber(iFrm2);
        ftrMarks1.assign(nFtrs1, false);
        ftrMarks2.assign(nFtrs2, false);
        for(ushort i = 0; i < nMatchesExist; ++i) {
            matchesExist[i].Get(iFtr1, iFtr2);
            ftrMarks1[iFtr1] = ftrMarks2[iFtr2] = true;
        }
        m_iFtrs1Unmatched.resize(0);
        for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1) {
            if(!ftrMarks1[iFtr1])
                m_iFtrs1Unmatched.push_back(iFtr1);
        }
        m_iFtrs2Unmatched.resize(0);
        for(iFtr2 = 0; iFtr2 < nFtrs2; ++iFtr2) {
            if(!ftrMarks2[iFtr2])
                m_iFtrs2Unmatched.push_back(iFtr2);
        }

        MatchFeatures(seq1, seq2, iFrm1, iFrm2, m_iFtrs1Unmatched, m_iFtrs2Unmatched,
                      m_scoredMatches);

        const ushort nMatchesNew = ushort(m_scoredMatches.size());
        for(ushort i = 0; i < nMatchesExist; ++i) {
            matchesExist[i].Get(iFtr1, iFtr2);
            m_scoredMatches.push_back(ScoredMatch(iFtr1, iFtr2, 0.0f));
        }
        m_Fdata.SetMatches(seq1.GetFrameFeatures(iFrm1), seq2.GetFrameFeatures(iFrm2),
                           m_scoredMatches, m_orders);
        m_Festor.RunLoProsac(m_Fdata, m_orders, F, inliers);

        const ushort nMatchesTotal = m_Fdata.Size();
        std::vector<bool> &inlierMarks = m_marks1;
        m_Festor.FromInliersToInlierMarks(inliers, nMatchesTotal, inlierMarks);
        matchesNew.resize(0);
        for(ushort i = 0; i < nMatchesNew; ++i) {
            if(!inlierMarks[i])
                continue;
            m_scoredMatches[i].Get(iFtr1, iFtr2);
            matchesNew.push_back(FeatureMatch(iFtr1, iFtr2));
        }
        inliers.resize(0);
        outliers.resize(0);
        for(ushort i = nMatchesNew, j = 0; j < nMatchesExist; ++i, ++j) {
            if(inlierMarks[i])
                inliers.push_back(j);
            else
                outliers.push_back(j);
        }
    }

    // Step3: filter outliers by 3D
    if(m_errSqThReprojCam == 0)
        return;
    const ushort nMatchesNew = ushort(matchesNew.size());
    FeatureMatchList &matches3D = matchesNew;
    const ushort nInliers = ushort(inliers.size());
    for(ushort i = 0; i < nInliers; ++i)
        matches3D.push_back(matchesExist[inliers[i]]);
    const ushort nMatches3D = ushort(matches3D.size());
    if(nMatches3D < m_ransacMinNumInliersCam) {
        matchesNew.resize(0);
        inliers.resize(0);
        outliers.resize(nMatchesExist);
        for(ushort i = 0; i < nMatchesExist; ++i)
            outliers[i] = i;
        return;
    }
    seqs.GetCameraPairEstimatorData(iSeq1, iSeq2, iFrm1, iFrm2, matches3D,
                                    m_CPdata);
    m_CPestor.m_ransacErrorThreshold = m_errSqThReprojCam * std::min(
                                           seq1.GetIntrinsicMatrix().one_over_fxy(),
                                           seq2.GetIntrinsicMatrix().one_over_fxy());
    m_CPestor.RunLosac(m_CPdata, m_CP, m_inliers3D/*, 4*/);
    const ushort nInliers3D = ushort(m_inliers3D.size());
    if(nInliers3D < m_ransacMinNumInliersCam) {
        matchesNew.resize(0);
        inliers.resize(0);
        outliers.resize(nMatchesExist);
        for(ushort i = 0; i < nMatchesExist; ++i)
            outliers[i] = i;
        return;
    }
    std::vector<bool> &inliersMarks3D = m_marks1;
    m_CPestor.FromInliersToInlierMarks(m_inliers3D, nMatches3D, inliersMarks3D);
    ushort i, j;
    for(i = j = 0; i < nMatchesNew; ++i) {
        if(inliersMarks3D[i])
            matchesNew[j++] = matches3D[i];
    }
    matchesNew.resize(j);
    const ushort nInliers3DNew = j;
    for(i = j = 0; i < nInliers; ++i) {
        if(inliersMarks3D[i + nMatchesNew])
            inliers[j++] = inliers[i];
        else
            outliers.push_back(inliers[i]);
    }
    inliers.resize(j);
//#if VERBOSE_TRACK_MATCHING
//  printf("\r  Frame (%d, %d): inliers = %d/%d = %d%%, error threshold = %.2f", iFrm1, iFrm2, nInliers3D, nMatches3D, nInliers3D * 100 / nMatches3D, errTh / one_over_sqrt_fxy);
//#endif

//#if _DEBUG
//  ViewerFeatureMatching viewer;
//  viewer.Initialize(seq1.GetImageWidth(), seq1.GetImageHeight(), 1, m_maxNumFtrsPerImg, 0);
//  viewer.SetImage(0, seq1.GetImageFileName(iFrm1));
//  viewer.SetImage(1, seq2.GetImageFileName(iFrm2));
//  viewer.SetFeatures(0, seq1.GetFrameFeaturesNumber(iFrm1), seq1.GetFrameFeatures(iFrm1));
//  viewer.SetFeatures(1, seq2.GetFrameFeaturesNumber(iFrm2), seq2.GetFrameFeatures(iFrm2));
//  viewer.SetSiftMatches(0, matchesNew);
//  viewer.Run();
//#endif
}

void TrackMatcher::VerifyMatches_MatchNewFeatures(/*const */Sequence &seq,
        const FrameIndex &iFrm1,
        const FrameIndex &iFrm2, /*const */FeatureMatchList &matchesExist,
        FundamentalMatrix &F, std::vector<ushort> &inliers,
        std::vector<ushort> &outliers,
        FeatureMatchList &matchesNew) {
    // Step1: estimate fundamental matrix and detect outlier matches
    FeatureIndex iFtr1, iFtr2;
    const FeatureIndex nFtrs1 = seq.GetFrameFeaturesNumber(iFrm1),
                       nFtrs2 = seq.GetFrameFeaturesNumber(iFrm2);
    const TrackIndex *iTrks1 = seq.GetFrameTrackIndexes(iFrm1);
    const ushort nMatchesExist1 = ushort(matchesExist.size());
    for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1) {
        if((iFtr2 = seq.SearchTrackForFrameFeatureIndex(iTrks1[iFtr1],
                    iFrm2)) != INVALID_FEATURE_INDEX)
            matchesExist.push_back(FeatureMatch(iFtr1, iFtr2));
    }
    const ushort nMatchesExist2 = ushort(matchesExist.size());

    m_Fdata.SetMatches(seq.GetFrameFeatures(iFrm1), seq.GetFrameFeatures(iFrm2),
                       matchesExist);
    m_Festor.RunLosac(m_Fdata, F, inliers);
    std::vector<bool> &inlierMarks = m_marks1;
    m_Festor.FromInliersToInlierMarks(inliers, nMatchesExist2, inlierMarks);
//#if _DEBUG
//  const std::vector<ushort> inliersBkp = inliers;
//#endif
    inliers.resize(0);
    outliers.resize(0);
    for(ushort i = 0; i < nMatchesExist1; ++i) {
        if(inlierMarks[i])
            inliers.push_back(i);
        else
            outliers.push_back(i);
    }

    // Step2: feature matching for unmatched features & matched outlier features
    std::vector<bool> &ftrMarks1 = m_marks1, &ftrMarks2 = m_marks2;
    ftrMarks1.assign(nFtrs1, false);
    ftrMarks2.assign(nFtrs2, false);
    const ushort nInliers1 = ushort(inliers.size());
    for(ushort i = 0; i < nInliers1; ++i) {
        matchesExist[inliers[i]].Get(iFtr1, iFtr2);
        ftrMarks1[iFtr1] = ftrMarks2[iFtr2] = true;
    }
    for(ushort i = nMatchesExist1; i < nMatchesExist2; ++i) {
        matchesExist[i].Get(iFtr1, iFtr2);
        ftrMarks1[iFtr1] = ftrMarks2[iFtr2] = true;
    }
    m_iFtrs1Unmatched.resize(0);
    for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1) {
        if(!ftrMarks1[iFtr1])
            m_iFtrs1Unmatched.push_back(iFtr1);
    }
    m_iFtrs2Unmatched.resize(0);
    for(iFtr2 = 0; iFtr2 < nFtrs2; ++iFtr2) {
        if(!ftrMarks2[iFtr2])
            m_iFtrs2Unmatched.push_back(iFtr2);
    }
    MatchFeatures(seq, seq, iFrm1, iFrm2, m_iFtrs1Unmatched, m_iFtrs2Unmatched, F,
                  matchesNew);

    //ushort i, j;
    //TrackIndex iTrk1, iTrk2;
    //const TrackIndex *iTrks2 = seq.GetFrameTrackIndexes(iFrm2);
    //for(i = j = 0; i < nMatchesNew; ++i)
    //{
    //  matchesNew[i].Get(iFtr1, iFtr2);
    //  iTrk1 = iTrks1[iFtr1];
    //  iTrk2 = iTrks2[iFtr2];
    //  if(iTrk1 != iTrk2 && !seq.AreTracksOverlappingInFrames(iTrk1, iTrk2, m_marks1))
    //      matchesNew[j++].Set(iFtr1, iFtr2);
    //}
    //matchesNew.resize(j);

//#if _DEBUG
//  //if(nMatchesExist1 != nMatchesExist2)
//  {
//      ViewerFeatureMatching viewer;
//      viewer.Initialize(seq.GetImageWidth(), seq.GetImageHeight(), 2, m_maxNumFtrsPerImg, 0);
//      viewer.SetImage(0, seq.GetImageFileName(iFrm1));
//      viewer.SetImage(1, seq.GetImageFileName(iFrm2));
//      printf("%s\n", seq.GetImageFileName(iFrm1).c_str());
//      printf("%s\n", seq.GetImageFileName(iFrm2).c_str());
//      viewer.SetFeatures(0, seq.GetFrameFeaturesNumber(iFrm1), seq.GetFrameFeatures(iFrm1));
//      viewer.SetFeatures(1, seq.GetFrameFeaturesNumber(iFrm2), seq.GetFrameFeatures(iFrm2));
//      viewer.SetSiftMatches(0, matchesExist);
//      //viewer.SetSiftInliers(0, inliersBkp);
//      viewer.SetSiftMatches(1, matchesNew);
//      viewer.SetSiftString(0, "Existed matches");
//      viewer.SetSiftString(1, "New matches");
//      viewer.Run();
//  }
//#endif
}

void TrackMatcher::VerifyMatches_MatchNewFeatures(/*const */SequenceSet &seqs,
        const SequenceIndex &iSeq1, const SequenceIndex &iSeq2, const FrameIndex &iFrm1,
        const FrameIndex &iFrm2, const FeatureMatchList &matchesExist,
        FundamentalMatrix &F,
        std::vector<ushort> &inliers, std::vector<ushort> &outliers,
        FeatureMatchList &matchesNew) {
    // Step1: estimate fundamental matrix and detect outlier matches
    /*const */Sequence &seq1 = seqs[iSeq1], &seq2 = seqs[iSeq2];
    m_Fdata.SetMatches(seq1.GetFrameFeatures(iFrm1), seq2.GetFrameFeatures(iFrm2),
                       matchesExist);
    m_Festor.RunLosac(m_Fdata, F, inliers);
    const ushort nMatchesExist = ushort(matchesExist.size());
    m_Festor.FromInliersToOutliers(inliers, nMatchesExist, outliers);

    // Step2: feature matching for unmatched features & matched outlier features
    FeatureIndex iFtr1, iFtr2;
    std::vector<bool> &ftrMarks1 = m_marks1, &ftrMarks2 = m_marks2;
    const FeatureIndex nFtrs1 = seq1.GetFrameFeaturesNumber(iFrm1),
                       nFtrs2 = seq2.GetFrameFeaturesNumber(iFrm2);
    ftrMarks1.assign(nFtrs1, false);
    ftrMarks2.assign(nFtrs2, false);
    const ushort nInliers = ushort(inliers.size());
    for(ushort i = 0; i < nInliers; ++i) {
        matchesExist[inliers[i]].Get(iFtr1, iFtr2);
        ftrMarks1[iFtr1] = ftrMarks2[iFtr2] = true;
    }
    m_iFtrs1Unmatched.resize(0);
    for(iFtr1 = 0; iFtr1 < nFtrs1; ++iFtr1) {
        if(!ftrMarks1[iFtr1])
            m_iFtrs1Unmatched.push_back(iFtr1);
    }
    m_iFtrs2Unmatched.resize(0);
    for(iFtr2 = 0; iFtr2 < nFtrs2; ++iFtr2) {
        if(!ftrMarks2[iFtr2])
            m_iFtrs2Unmatched.push_back(iFtr2);
    }
    MatchFeatures(seq1, seq2, iFrm1, iFrm2, m_iFtrs1Unmatched, m_iFtrs2Unmatched, F,
                  matchesNew);

    // Step3: filter outliers by 3D
    if(m_errSqThReprojCam == 0)
        return;
    const ushort nMatchesNew = ushort(matchesNew.size());
    FeatureMatchList &matches3D = matchesNew;
    for(ushort i = 0; i < nInliers; ++i)
        matches3D.push_back(matchesExist[inliers[i]]);
    const ushort nMatches3D = ushort(matches3D.size());
    if(nMatches3D < m_ransacMinNumInliersCam) {
        matchesNew.resize(0);
        inliers.resize(0);
        outliers.resize(nMatchesExist);
        for(ushort i = 0; i < nMatchesExist; ++i)
            outliers[i] = i;
        return;
    }
    seqs.GetCameraPairEstimatorData(iSeq1, iSeq2, iFrm1, iFrm2, matches3D,
                                    m_CPdata);
    m_CPestor.m_ransacErrorThreshold = m_errSqThReprojCam * std::min(
                                           seq1.GetIntrinsicMatrix().one_over_fxy(),
                                           seq2.GetIntrinsicMatrix().one_over_fxy());
    m_CPestor.RunLosac(m_CPdata, m_CP, m_inliers3D);
    const ushort nInliers3D = ushort(m_inliers3D.size());
    if(nInliers3D < m_ransacMinNumInliersCam) {
        matchesNew.resize(0);
        inliers.resize(0);
        outliers.resize(nMatchesExist);
        for(ushort i = 0; i < nMatchesExist; ++i)
            outliers[i] = i;
        return;
    }
    std::vector<bool> &inliersMarks3D = m_marks1;
    m_CPestor.FromInliersToInlierMarks(m_inliers3D, nMatches3D, inliersMarks3D);
    ushort i, j;
    for(i = j = 0; i < nMatchesNew; ++i) {
        if(inliersMarks3D[i])
            matchesNew[j++] = matches3D[i];
    }
    matchesNew.resize(j);
    const ushort nInliers3DNew = j;
    for(i = j = 0; i < nInliers; ++i) {
        if(inliersMarks3D[i + nMatchesNew])
            inliers[j++] = inliers[i];
        else
            outliers.push_back(inliers[i]);
    }
    inliers.resize(j);

//#if VERBOSE_TRACK_MATCHING
//  printf("\r  Frame (%d, %d): inliers = %d/%d = %d%%, error threshold = %.2f", iFrm1, iFrm2, nInliers3D, nMatches3D, nInliers3D * 100 / nMatches3D, errTh / one_over_sqrt_fxy);
//#endif

//#if _DEBUG
//  ViewerFeatureMatching viewer;
//  viewer.Initialize(seq1.GetImageWidth(), seq1.GetImageHeight(), 2, m_maxNumFtrsPerImg, 0);
//  viewer.SetImage(0, seq1.GetImageFileName(iFrm1));
//  viewer.SetImage(1, seq2.GetImageFileName(iFrm2));
//  viewer.SetFeatures(0, seq1.GetFrameFeaturesNumber(iFrm1), seq1.GetFrameFeatures(iFrm1));
//  viewer.SetFeatures(1, seq2.GetFrameFeaturesNumber(iFrm2), seq2.GetFrameFeatures(iFrm2));
//  viewer.SetSiftMatches(0, matchesExist);
//  viewer.SetSiftInliers(0, matchesInlier);
//  viewer.SetSiftMatches(1, matchesNew);
//  viewer.SetSiftString(0, "Existed matches");
//  viewer.SetSiftString(1, "New matches");
//  viewer.Run();
//#endif

}