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

#ifndef _FEATURE_MATCHER_SIFT_H_
#define _FEATURE_MATCHER_SIFT_H_

#include "FeatureExtractorSift.h"
#include "ProgramGL/ProgramGLMatchFeature.h"
#include "LinearAlgebra/Matrix3.h"
#include "SfM/Match.h"

class FeatureMatcherSift {

  public:

    inline void Initialize(const ushort &maxNumFtrs1, const ushort &maxNumFtrs2, const ushort &ftrTexWidth, const float &epErrTh, const float &nearest1To2RatioTh,
                           const float &descDotTh) {
        GLint maxTexSize;
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTexSize);
        m_maxNumFtrs1 = std::min(maxNumFtrs1, ushort(maxTexSize - 2));
        m_maxNumFtrs2 = std::min(maxNumFtrs2, ushort(maxTexSize));
        const ushort matchTexWidth  = m_maxNumFtrs2;
        const ushort matchTexHeight = m_maxNumFtrs1 + 2;
        m_matchTex.Generate(matchTexWidth, matchTexHeight);
        m_programMatchFeaturesPass1[0][0].Initialize(ftrTexWidth);
        m_programMatchFeaturesPass1[0][1].Initialize(ftrTexWidth, USHRT_MAX, epErrTh);
        m_programMatchFeaturesPass2.Initialize(ftrTexWidth, epErrTh);
        m_programMatchFeaturesPass3.Initialize(nearest1To2RatioTh);
        m_programMatchFeaturesPass4.Initialize(nearest1To2RatioTh, descDotTh);
    }
    inline void Initialize(const ushort &maxNumFtrs1, const ushort &maxNumFtrs2, const ushort &ftrTexWidth, const float &epErrTh, const float &nearest1To2RatioTh,
                           const float &descDotTh, const ushort &searchRangeLarger, const ushort &searchRangeSmaller) {
        GLint maxTexSize;
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTexSize);
        m_maxNumFtrs1 = std::min(maxNumFtrs1, ushort(maxTexSize - 2));
        m_maxNumFtrs2 = std::min(maxNumFtrs2, ushort(maxTexSize));
        const ushort matchTexWidth  = m_maxNumFtrs2;
        const ushort matchTexHeight = m_maxNumFtrs1 + 2;
        m_matchTex.Generate(matchTexWidth, matchTexHeight);
        m_programMatchFeaturesPass1[0][0].Initialize(ftrTexWidth, searchRangeSmaller);
        m_programMatchFeaturesPass1[0][1].Initialize(ftrTexWidth, searchRangeSmaller, epErrTh);
        m_programMatchFeaturesPass1[1][0].Initialize(ftrTexWidth, searchRangeLarger);
        m_programMatchFeaturesPass1[1][1].Initialize(ftrTexWidth, searchRangeLarger, epErrTh);
        m_programMatchFeaturesPass2.Initialize(ftrTexWidth, epErrTh);
        m_programMatchFeaturesPass3.Initialize(nearest1To2RatioTh);
        m_programMatchFeaturesPass4.Initialize(nearest1To2RatioTh, descDotTh);
    }
    inline void MatchFeatures(const TextureGL4 &descTex1, const TextureGL4 &descTex2, const ushort &nFtrs1, const ushort &nFtrs2) {
        const ushort _nFtrs1 = std::min(nFtrs1, m_maxNumFtrs1), _nFtrs2 = std::min(nFtrs2, m_maxNumFtrs2);
        ProgramGL::FitViewportGL(std::max(_nFtrs1, _nFtrs2), _nFtrs1 + 2);
        m_programMatchFeaturesPass1[0][0].Run(descTex1, descTex2, _nFtrs1, _nFtrs2, m_matchTex);
        m_programMatchFeaturesPass3.Run(m_matchTex, _nFtrs1, _nFtrs2);
        m_programMatchFeaturesPass4.Run(m_matchTex, _nFtrs1, _nFtrs2);
    }
    template<ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    inline void MatchFeatures(const TextureGL<CHANNELS_NUMBER_1> &ftrTex1, const TextureGL<CHANNELS_NUMBER_2> &ftrTex2, const TextureGL4 &descTex1,
                              const TextureGL4 &descTex2, const ushort &nFtrs1, const ushort &nFtrs2, const FundamentalMatrix &F,
                              const bool &recompPairwiseSimilarity) {
        const ushort _nFtrs1 = std::min(nFtrs1, m_maxNumFtrs1), _nFtrs2 = std::min(nFtrs2, m_maxNumFtrs2);
        ProgramGL::FitViewportGL(std::max(_nFtrs1, _nFtrs2), _nFtrs1 + 2);
        if(recompPairwiseSimilarity)
            m_programMatchFeaturesPass1[0][1].Run(ftrTex1, ftrTex2, descTex1, descTex2, _nFtrs1, _nFtrs2, m_matchTex, F);
        else
            m_programMatchFeaturesPass2.Run(ftrTex1, ftrTex2, m_matchTex, _nFtrs1, _nFtrs2, F);
        m_programMatchFeaturesPass3.Run(m_matchTex, _nFtrs1, _nFtrs2);
        m_programMatchFeaturesPass4.Run(m_matchTex, _nFtrs1, _nFtrs2);
    }
    template<ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    inline void MatchFeatures(const TextureGL<CHANNELS_NUMBER_1> &ftrTex1, const TextureGL<CHANNELS_NUMBER_2> &ftrTex2,
                              const TextureGL4 &descTex1, const TextureGL4 &descTex2,
                              const ushort &nFtrs1, const ushort &nFtrs2, const bool &larger) {
        const ushort _nFtrs1 = std::min(nFtrs1, m_maxNumFtrs1), _nFtrs2 = std::min(nFtrs2, m_maxNumFtrs2);
        ProgramGL::FitViewportGL(std::max(_nFtrs1, _nFtrs2), _nFtrs1 + 2);
        m_programMatchFeaturesPass1[larger][0].Run(ftrTex1, ftrTex2, descTex1, descTex2, _nFtrs1, _nFtrs2, m_matchTex);
        m_programMatchFeaturesPass3.Run(m_matchTex, _nFtrs1, _nFtrs2);
        m_programMatchFeaturesPass4.Run(m_matchTex, _nFtrs1, _nFtrs2);

    }
    template<ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    inline void MatchFeatures(const TextureGL<CHANNELS_NUMBER_1> &ftrTex1, const TextureGL<CHANNELS_NUMBER_2> &ftrTex2, const TextureGL4 &descTex1,
                              const TextureGL4 &descTex2, const ushort &nFtrs1, const ushort &nFtrs2, const FundamentalMatrix &F,
                              const bool &recompPairwiseSimilarity, const bool &larger) {
        const ushort _nFtrs1 = std::min(nFtrs1, m_maxNumFtrs1), _nFtrs2 = std::min(nFtrs2, m_maxNumFtrs2);
        ProgramGL::FitViewportGL(std::max(_nFtrs1, _nFtrs2), _nFtrs1 + 2);
        if(recompPairwiseSimilarity)
            m_programMatchFeaturesPass1[larger][1].Run(ftrTex1, ftrTex2, descTex1, descTex2, _nFtrs1, _nFtrs2, m_matchTex, F);
        else
            m_programMatchFeaturesPass2.Run(ftrTex1, ftrTex2, m_matchTex, _nFtrs1, _nFtrs2, F);
        m_programMatchFeaturesPass3.Run(m_matchTex, _nFtrs1, _nFtrs2);
        m_programMatchFeaturesPass4.Run(m_matchTex, _nFtrs1, _nFtrs2);
    }
    inline void DownloadMatches2ToCPU(const ushort &nFtrs1, const ushort &nFtrs2, std::vector<float> &iFtrs2To1) {
        const ushort _nFtrs1 = std::min(nFtrs1, m_maxNumFtrs1), _nFtrs2 = std::min(nFtrs2, m_maxNumFtrs2);
        iFtrs2To1.resize(nFtrs2);
        m_matchTex.DownloadToCPU(iFtrs2To1.data(), 0, _nFtrs1 + 1, _nFtrs2, 1, GL_RED);
        for(ushort i = _nFtrs2; i < nFtrs2; ++i)
            m_iFtrs2To1[i] = -1.0f;
    }
    inline void DownloadMatches12ToCPU(const ushort &nFtrs1, const ushort &nFtrs2, std::vector<Match<ushort> > &matches) {
        DownloadMatches2ToCPU(nFtrs1, nFtrs2, m_iFtrs2To1);

        ushort iFtr1, iFtr2;
        matches.resize(0);
        for(iFtr2 = 0; iFtr2 < nFtrs2; ++iFtr2) {
            if(m_iFtrs2To1[iFtr2] < 0)
                continue;
            iFtr1 = ushort(m_iFtrs2To1[iFtr2]);
            matches.push_back(Match<ushort>(iFtr1, iFtr2));
        }
    }
    inline void DownloadMatches12ToCPU(const std::vector<ushort> &iFtrs1, const std::vector<ushort> &iFtrs2, std::vector<Match<ushort> > &matches) {
        const ushort N1 = ushort(iFtrs1.size()), N2 = ushort(iFtrs2.size());
        DownloadMatches2ToCPU(N1, N2, m_iFtrs2To1);

        ushort iFtr1, iFtr2;
        matches.resize(0);
        for(ushort i2 = 0; i2 < N2; ++i2) {
            if(m_iFtrs2To1[i2] < 0)
                continue;
            iFtr1 = iFtrs1[ushort(m_iFtrs2To1[i2])];
            iFtr2 = iFtrs2[i2];
            matches.push_back(Match<ushort>(iFtr1, iFtr2));
        }
    }
    inline void DownloadMatches2ToCPU(const ushort &nFtrs1, const ushort &nFtrs2, std::vector<ScoredMatch> &matches) {
        const ushort _nFtrs1 = std::min(nFtrs1, m_maxNumFtrs1), _nFtrs2 = std::min(nFtrs2, m_maxNumFtrs2);
        matches.resize(nFtrs2);
        m_matchTex.DownloadToCPU((float *) matches.data(), 0, _nFtrs1 + 1, _nFtrs2, 1);
        for(ushort i = _nFtrs2; i < nFtrs2; ++i)
            matches[i].Invalidate();
    }
    inline void DownloadMatches12ToCPU(const ushort &nFtrs1, const ushort &nFtrs2, std::vector<ScoredMatch> &matches) {
        DownloadMatches2ToCPU(nFtrs1, nFtrs2, matches);
        ushort iFtr1, iFtr2, i = 0;
        for(iFtr2 = 0; iFtr2 < nFtrs2; ++iFtr2) {
            ScoredMatch &match2 = matches[iFtr2];
            if(match2.IsInvalid())
                continue;
            iFtr1 = ushort(match2.GetIndex());
            matches[i].Set(iFtr1, iFtr2, match2.GetScore());
            ++i;
        }
        const ushort nMatches = i;
        matches.resize(nMatches);
    }
    template<class MATCH_TYPE>
    static inline void FromMatches12ToMatchMarks(const ushort &nFtrs1, const ushort &nFtrs2, const std::vector<MATCH_TYPE> &matches,
            std::vector<bool> &matchMarks1, std::vector<bool> &matchMarks2) {
        matchMarks1.assign(nFtrs1, false);
        matchMarks2.assign(nFtrs2, false);
        const ushort nMatches = ushort(matches.size());
        for(ushort i = 0; i < nMatches; ++i) {
            matchMarks1[matches[i].GetIndex1()] = true;
            matchMarks2[matches[i].GetIndex2()] = true;
        }
    }
    template<class MATCH_TYPE>
    static inline void FromMatches12ToMatchMarks1(const ushort &nFtrs1, const std::vector<MATCH_TYPE> &matches, std::vector<bool> &matchMarks1) {
        matchMarks1.assign(nFtrs1, false);
        const ushort nMatches = ushort(matches.size());
        for(ushort i = 0; i < nMatches; ++i)
            matchMarks1[matches[i].GetIndex1()] = true;
    }
    static inline void FromMatchMarksToUnmatchedFeatureIndexes(const std::vector<bool> &matchMarks, const ushort nFtrs, std::vector<ushort> &iFtrs) {
        iFtrs.resize(0);
        for(ushort iFtr = 0; iFtr < nFtrs; ++iFtr) {
            if(!matchMarks[iFtr])
                iFtrs.push_back(iFtr);
        }
    }
    template<class MATCH_TYPE>
    static inline void FromMatches12ToUnmatchedFeatureIndexes(const ushort &nFtrs1, const ushort &nFtrs2, const std::vector<MATCH_TYPE> &matches,
            std::vector<ushort> &iFtrs1, std::vector<ushort> &iFtrs2, std::vector<bool> &matchMarks1,
            std::vector<bool> &matchMarks2) {
        FromMatches12ToMatchMarks<MATCH_TYPE>(nFtrs1, nFtrs2, matches, matchMarks1, matchMarks2);
        FromMatchMarksToUnmatchedFeatureIndexes(matchMarks1, nFtrs1, iFtrs1);
        FromMatchMarksToUnmatchedFeatureIndexes(matchMarks2, nFtrs2, iFtrs2);
    }
    // matches could contain some ENFT features
    template<class MATCH_TYPE>
    static inline void FromMatches12ToUnmatchedFeatureIndexes(const ushort &nFtrs1, const ushort &nFtrs2, const ushort &nFtrs1Valid, const ushort &nFtrs2Valid,
            const std::vector<MATCH_TYPE> &matches, std::vector<ushort> &iFtrs1, std::vector<ushort> &iFtrs2,
            std::vector<bool> &matchMarks1, std::vector<bool> &matchMarks2) {
        FromMatches12ToMatchMarks<MATCH_TYPE>(nFtrs1, nFtrs2, matches, matchMarks1, matchMarks2);
        FromMatchMarksToUnmatchedFeatureIndexes(matchMarks1, nFtrs1Valid, iFtrs1);
        FromMatchMarksToUnmatchedFeatureIndexes(matchMarks2, nFtrs2Valid, iFtrs2);
    }
    // iFtrs2Invalid and iFtrs2 are allowed to be the same
    template<class MATCH_TYPE>
    static inline void FromMatches12ToUnmatchedFeatureIndexes(const ushort &nFtrs1, const ushort &nFtrs2, const ushort &nFtrs1Valid, const ushort &nFtrs2Valid,
            const std::vector<ushort> &iFtrs2Invalid, const std::vector<MATCH_TYPE> &matches,
            std::vector<ushort> &iFtrs1, std::vector<ushort> &iFtrs2, std::vector<bool> &matchMarks1,
            std::vector<bool> &matchMarks2) {
        FromMatches12ToMatchMarks<MATCH_TYPE>(nFtrs1, nFtrs2, matches, matchMarks1, matchMarks2);
        FromMatchMarksToUnmatchedFeatureIndexes(matchMarks1, nFtrs1Valid, iFtrs1);
        const ushort nFtrs2Invalid = ushort(iFtrs2Invalid.size());
        for(ushort i = 0; i < nFtrs2Invalid; ++i)
            matchMarks2[iFtrs2Invalid[i]] = true;
        FromMatchMarksToUnmatchedFeatureIndexes(matchMarks2, nFtrs2Valid, iFtrs2);
    }
    template<class MATCH_TYPE>
    static inline void FromMatches12ToUnmatchedFeatureIndexes1(const ushort &nFtrs1, const ushort &nFtrs1Valid, const std::vector<MATCH_TYPE> &matches,
            std::vector<ushort> &iFtrs1, std::vector<bool> &matchMarks1) {
        FromMatches12ToMatchMarks1<MATCH_TYPE>(nFtrs1, matches, matchMarks1);
        FromMatchMarksToUnmatchedFeatureIndexes(matchMarks1, nFtrs1Valid, iFtrs1);
    }
    template<class MATCH_TYPE>
    static inline void FromMatchesToFeatureIndexCorrespondences(const ushort &nFtrs1, const ushort &nFtrs2, const std::vector<MATCH_TYPE> &matches,
            std::vector<ushort> &iFtrs1To2, std::vector<ushort> &iFtrs2To1) {
        iFtrs1To2.assign(nFtrs1, USHRT_MAX);
        iFtrs2To1.assign(nFtrs2, USHRT_MAX);
        const ushort nMatches = ushort(matches.size());
        for(ushort i = 0; i < nMatches; ++i) {
            const Match<MATCH_TYPE> &match = matches[i];
            iFtrs1To2[match.GetIndex1()] = match.GetIndex2();
            iFtrs2To1[match.GetIndex2()] = match.GetIndex1();
        }
    }
    template<class MATCH_TYPE>
    static inline void FromMatches13To23(const std::vector<ushort> &iFtrs1To2, std::vector<MATCH_TYPE> &matches13, std::vector<MATCH_TYPE> &matches23) {
        matches23.resize(0);
        const ushort nMatches13 = ushort(matches13.size());
        ushort i, j, iFtr2;
        for(i = 0, j = 0; i < nMatches13; ++i) {
            if((iFtr2 = iFtrs1To2[matches13[i].GetIndex1()]) == USHRT_MAX)
                matches13[j++] = matches13[i];
            else
                matches23.push_back(MATCH_TYPE(iFtr2, matches13[i].GetIndex2()));
        }
        matches13.resize(j);
    }
    template<class MATCH_TYPE>
    static inline void FromMatches13To23(const std::vector<ushort> &iFtrs1To2, std::vector<MATCH_TYPE> &matches13, std::vector<MATCH_TYPE> &matches23,
                                         std::vector<ushort> &iFtrs3Exclusive13) {
        matches23.resize(0);
        iFtrs3Exclusive13.resize(0);
        const ushort nMatches13 = ushort(matches13.size());
        ushort i, j, iFtr2;
        for(i = 0, j = 0; i < nMatches13; ++i) {
            if((iFtr2 = iFtrs1To2[matches13[i].GetIndex1()]) == USHRT_MAX) {
                matches13[j++] = matches13[i];
                iFtrs3Exclusive13.push_back(matches13[i].GetIndex2());
            } else
                matches23.push_back(MATCH_TYPE(iFtr2, matches13[i].GetIndex2()));
        }
        matches13.resize(j);
    }
    template<class MATCH_TYPE>
    static inline void FromMatches13To23(const std::vector<ushort> &iFtrs1To2, const std::vector<MATCH_TYPE> &matches13, std::vector<MATCH_TYPE> &matchesCommon13,
                                         std::vector<MATCH_TYPE> &matchesCommon23) {
        matchesCommon13.resize(0);
        matchesCommon23.resize(0);
        const ushort nMatches13 = ushort(matches13.size());
        ushort i, iFtr2;
        for(i = 0; i < nMatches13; ++i) {
            if((iFtr2 = iFtrs1To2[matches13[i].GetIndex1()]) == USHRT_MAX)
                continue;
            matchesCommon13.push_back(matches13[i]);
            matchesCommon23.push_back(MATCH_TYPE(iFtr2, matches13[i].GetIndex2()));
        }
    }
    template<class MATCH_TYPE>
    static inline void FromMatches13To23(const std::vector<ushort> &iFtrs1To2, std::vector<MATCH_TYPE> &matches) {
        const ushort nMatches = ushort(matches.size());
        for(ushort i = 0; i < nMatches; ++i)
            matches[i].SetIndex1(iFtrs1To2[matches[i].GetIndex1()]);
    }

  private:

    ushort m_maxNumFtrs1, m_maxNumFtrs2;
    TextureGL2 m_matchTex;
    std::vector<float> m_iFtrs2To1;
    ProgramGLMatchFeaturePass1 m_programMatchFeaturesPass1[2][2];
    ProgramGLMatchFeaturePass2 m_programMatchFeaturesPass2;
    ProgramGLMatchFeaturePass3 m_programMatchFeaturesPass3;
    ProgramGLMatchFeaturePass4 m_programMatchFeaturesPass4;
};

#endif