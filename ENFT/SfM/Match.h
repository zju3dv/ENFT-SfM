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

#ifndef _MATCH_H_
#define _MATCH_H_

#include <functional>
#include "IntrinsicMatrix.h"
#include "Camera.h"
#include "Sequence/Feature.h"

template<typename TYPE>
class Match {
  public:
    Match() {}
    Match(const TYPE &idx1, const TYPE &idx2) {
        Set(idx1, idx2);
    }
    inline const TYPE &GetIndex1() const {
        return m_idx1;
    }
    inline const TYPE &GetIndex2() const {
        return m_idx2;
    }
    inline void Get(TYPE &idx1, TYPE &idx2) const {
        idx1 = m_idx1;
        idx2 = m_idx2;
    }
    inline void Set(const TYPE &idx1, const TYPE &idx2) {
        m_idx1 = idx1;
        m_idx2 = idx2;
    }
    inline void SetIndex1(const TYPE &idx1) {
        m_idx1 = idx1;
    }
    inline void SetIndex2(const TYPE &idx2) {
        m_idx2 = idx2;
    }
    inline void IncreaseIndex1(const TYPE &incr) {
        m_idx1 += incr;
    }
    inline void IncreaseIndex2(const TYPE &incr) {
        m_idx2 += incr;
    }
    inline void Invert(TYPE &tmp) {
        SWAP(m_idx1, m_idx2, tmp);
    }
  protected:
    TYPE m_idx1, m_idx2;
};

class ScoredMatch {
  public:
    ScoredMatch() {}
    ScoredMatch(const ushort &idx1, const ushort &idx2, const float &score) : m_idx1(idx1), m_idx2(idx2), m_score(score) {}
    inline void Invalidate() {
        m_idx = -1;
    }
    inline bool IsInvalid() const {
        return m_idx < 0;
    }
    inline const float &GetIndex() const {
        return m_idx;
    }
    inline const ushort &GetIndex1() const {
        return m_idx1;
    }
    inline const ushort &GetIndex2() const {
        return m_idx2;
    }
    inline const float &GetScore() const {
        return m_score;
    }
    inline void Get(Match<ushort> &match) const {
        match.Set(m_idx1, m_idx2);
    }
    inline void Get(ushort &idx1, ushort &idx2) const {
        idx1 = m_idx1;
        idx2 = m_idx2;
    }
    inline void Set(const ushort &idx1, const ushort &idx2, const float &score) {
        m_idx1 = idx1;
        m_idx2 = idx2;
        m_score = score;
    }
    inline void SetIndex1(const ushort &idx1) {
        m_idx1 = idx1;
    }
    inline void SetIndex2(const ushort &idx2) {
        m_idx2 = idx2;
    }
    inline void IncreaseIndex1(const ushort &incr) {
        m_idx1 += incr;
    }
    inline void IncreaseIndex2(const ushort &incr) {
        m_idx2 += incr;
    }
    inline bool operator < (const ScoredMatch &match) const {
        return m_score > match.m_score;
    }
  protected:
    union {
        float m_idx;
        struct {
            ushort m_idx1, m_idx2;
        };
    };
    float m_score;
};

class MatchSet2D {

  public:

    MatchSet2D() {
        memset(this, 0, 6);
    }
    inline const ushort &Size() const {
        return m_N;
    }
    inline void Resize(const ushort &N) {
        if(N == m_N)
            return;
        m_N = N;
        m_nRems = (N & 3);
        m_nPacks = N - m_nRems;
        const ushort nPacksCeil = m_nRems == 0 ? m_nPacks : m_nPacks + 4;
        m_data.Resize(nPacksCeil);
        //m_imgLocations.Resize(N);
    }
    inline void Set(const MatchSet2D &data) {
        m_N = data.m_N;
        m_nPacks = data.m_nPacks;
        m_nRems = data.m_nRems;
        m_data = data.m_data;
        memcpy(m_rems1, data.m_rems1, sizeof(m_rems1));
        memcpy(m_rems2, data.m_rems2, sizeof(m_rems2));
        //m_mean_u1v1u2v2 = data.m_mean_u1v1u2v2;
        //m_scale1 = data.m_scale1;
        //m_scale2 = data.m_scale2;
        //m_scores = data.m_scores;
    }
    inline void SetMatches(const AlignedVector<Point2D> &xs1, const AlignedVector<Point2D> &xs2) {
        const ushort nMatches = ushort(xs1.Size());
        Resize(nMatches);
        for(ushort i = 0; i < nMatches; ++i)
            SetMatch(i, xs1[i], xs2[i]);
        FinishSettingMatches();
    }
    template<class MATCH_TYPE>
    inline void SetMatches(const Point2D *xs1, const Point2D *xs2, const std::vector<MATCH_TYPE> &matches) {
        const ushort nMatches = ushort(matches.size());
        Resize(nMatches);
        ushort idx1, idx2;
        for(ushort i = 0; i < nMatches; ++i) {
            idx1 = matches[i].GetIndex1();
            idx2 = matches[i].GetIndex2();
            SetMatch(i, xs1[idx1], xs2[idx2]);
        }
        FinishSettingMatches();
    }
    template<class MATCH_TYPE>
    inline void SetMatchesInverse(const Point2D *xs1, const Point2D *xs2, const std::vector<MATCH_TYPE> &matches) {
        const ushort nMatches = ushort(matches.size());
        Resize(nMatches);
        ushort iFtr1, iFtr2;
        for(ushort i = 0; i < nMatches; ++i) {
            matches[i].Get(iFtr1, iFtr2);
            SetMatch(i, xs1[iFtr1], xs2[iFtr2]);
        }
        FinishSettingMatches();
    }
    inline void SetMatches(const Point2D *xs1, const Point2D *xs2, const std::vector<ScoredMatch> &matches, std::vector<ushort> &orders) {
        const ushort nMatches = ushort(matches.size());
        Resize(nMatches);
        m_scores.resize(nMatches);
        ushort idx1, idx2;
        for(ushort i = 0; i < nMatches; ++i) {
            idx1 = matches[i].GetIndex1();
            idx2 = matches[i].GetIndex2();
            SetMatch(i, xs1[idx1], xs2[idx2]);
            m_scores[i] = std::make_pair(matches[i].GetScore(), i);
        }
        FinishSettingMatches();

        std::sort(m_scores.begin(), m_scores.end(), std::greater<std::pair<float, ushort> >());
        orders.resize(nMatches);
        for(ushort i = 0; i < nMatches; ++i)
            orders[i] = m_scores[i].second;
    }
    inline void SetMatches(const Point2D *xs1, const Point2D *xs2, const std::vector<ushort> &idxs1, const std::vector<ushort> &idxs2,
                           const std::vector<ScoredMatch> &matches, std::vector<ushort> &orders) {
        const ushort nMatches = ushort(matches.size());
        Resize(nMatches);
        m_scores.resize(nMatches);
        ushort idx1, idx2;
        for(ushort i = 0; i < nMatches; ++i) {
            idx1 = idxs1[matches[i].GetIndex1()];
            idx2 = idxs2[matches[i].GetIndex2()];
            SetMatch(i, xs1[idx1], xs2[idx2]);
            m_scores[i] = std::make_pair(matches[i].GetScore(), i);
        }
        FinishSettingMatches();

        std::sort(m_scores.begin(), m_scores.end(), std::greater<std::pair<float, ushort> >());
        orders.resize(nMatches);
        for(ushort i = 0; i < nMatches; ++i)
            orders[i] = m_scores[i].second;
    }
    inline void SetMatch(const ushort &i, const Point2D &x1, const Point2D &x2) {
        SetMatch(i, x1.x(), x1.y(), x2.x(), x2.y());
    }
    inline void SetMatch(const ushort &i, const float &u1, const float &v1, const float &u2, const float &v2) {
        const ushort idx2 = (i & 3), idx1 = i - idx2;
        m_data[idx1    ].m128_f32[idx2] = u1;
        m_data[idx1 + 1].m128_f32[idx2] = v1;
        m_data[idx1 + 2].m128_f32[idx2] = u2;
        m_data[idx1 + 3].m128_f32[idx2] = v2;
    }
    inline void FinishSettingMatches() {
        if(m_nRems == 0)
            return;
        const ushort iu1 = m_nPacks, iv1 = m_nPacks + 1, iu2 = m_nPacks + 2, iv2 = m_nPacks + 3;
        const ENFT_SSE::__m128 &u1 = m_data[iu1], &v1 = m_data[iv1], &u2 = m_data[iu2], &v2 = m_data[iv2];
        for(ushort iRem = 0; iRem < m_nRems; ++iRem) {
            m_rems1[iRem].Set(u1.m128_f32[iRem], v1.m128_f32[iRem]);
            m_rems2[iRem].Set(u2.m128_f32[iRem], v2.m128_f32[iRem]);
        }
    }
    inline void ImageToNormalizedPlane(const IntrinsicMatrix &K) {
        const ushort nPacksCeil = m_nRems == 0 ? m_nPacks : m_nPacks + 4;
        for(ushort i0 = 0, i1 = 1, i2 = 2, i3 = 3; i0 < nPacksCeil; i0 += 4, i1 += 4, i2 += 4, i3 += 4) {
            ENFT_SSE::__m128 &u1 = m_data[i0], &v1 = m_data[i1], &u2 = m_data[i2], &v2 = m_data[i3];
            K.ImageToNormalizedPlane4(u1, v1);
            K.ImageToNormalizedPlane4(u2, v2);
        }
        FinishSettingMatches();
    }
    inline void NormalizedPlaneToImage(const IntrinsicMatrix &K) {
        const ushort nPacksCeil = m_nRems == 0 ? m_nPacks : m_nPacks + 4;
        for(ushort i0 = 0, i1 = 1, i2 = 2, i3 = 3; i0 < nPacksCeil; i0 += 4, i1 += 4, i2 += 4, i3 += 4) {
            ENFT_SSE::__m128 &u1 = m_data[i0], &v1 = m_data[i1], &u2 = m_data[i2], &v2 = m_data[i3];
            K.NormalizedPlaneToImage4(u1, v1);
            K.NormalizedPlaneToImage4(u2, v2);
        }
        FinishSettingMatches();
    }
    inline void Rectify(const Camera::IntrinsicParameter &Kr1, const Camera::IntrinsicParameter &Kr2) {
        ENFT_SSE::__m128 r2, s;
        const ENFT_SSE::__m128 one = ENFT_SSE::_mm_set1_ps(1.0f), d1 = ENFT_SSE::_mm_set1_ps(Kr1.d()), f1inv = ENFT_SSE::_mm_set1_ps(1 / Kr1.f()),
                               d2 = ENFT_SSE::_mm_set1_ps(Kr2.d()), f2inv = ENFT_SSE::_mm_set1_ps(1 / Kr2.f());
        const ushort nPacksCeil = m_nRems == 0 ? m_nPacks : m_nPacks + 4;
        for(ushort i0 = 0, i1 = 1, i2 = 2, i3 = 3; i0 < nPacksCeil; i0 += 4, i1 += 4, i2 += 4, i3 += 4) {
            ENFT_SSE::__m128 &u1 = m_data[i0], &v1 = m_data[i1], &u2 = m_data[i2], &v2 = m_data[i3];
            r2 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(u1, u1), ENFT_SSE::_mm_mul_ps(v1, v1));
            s = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_add_ps(one, ENFT_SSE::_mm_mul_ps(d1, r2)), f1inv);
            u1 = ENFT_SSE::_mm_mul_ps(s, u1);
            v1 = ENFT_SSE::_mm_mul_ps(s, v1);
            r2 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(u2, u2), ENFT_SSE::_mm_mul_ps(v2, v2));
            s = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_add_ps(one, ENFT_SSE::_mm_mul_ps(d2, r2)), f2inv);
            u2 = ENFT_SSE::_mm_mul_ps(s, u2);
            v2 = ENFT_SSE::_mm_mul_ps(s, v2);
        }
        FinishSettingMatches();
    }
    inline void GetMatch(const ushort &i, float &u1, float &v1, float &u2, float &v2) const {
        const ushort idx2 = (i & 3), idx1 = i - idx2;
        u1 = m_data[idx1    ].m128_f32[idx2];
        v1 = m_data[idx1 + 1].m128_f32[idx2];
        u2 = m_data[idx1 + 2].m128_f32[idx2];
        v2 = m_data[idx1 + 3].m128_f32[idx2];
    }
    inline void GetMatch(const ushort &i, Point2D &x1, Point2D &x2) const {
        GetMatch(i, x1.x(), x1.y(), x2.x(), x2.y());
    }
    inline void GetSubset(const std::vector<ushort> &idxs, MatchSet2D &subset) const {
        const ushort N = ushort(idxs.size());
        subset.Resize(N);

        ushort iDst, iDstp1, iDstp2, iDstp3, iSrc, idx1Src, idx2Src;
        for(iDst = 0, iDstp1 = 1, iDstp2 = 2, iDstp3 = 3; iDst < subset.m_nPacks; iDst += 4, iDstp1 += 4, iDstp2 += 4, iDstp3 += 4) {
            ENFT_SSE::__m128 &u1 = subset.m_data[iDst], &v1 = subset.m_data[iDstp1], &u2 = subset.m_data[iDstp2], &v2 = subset.m_data[iDstp3];
            iSrc = idxs[iDst];
            idx2Src = (iSrc & 3);
            idx1Src = iSrc - idx2Src;
            u1.m128_f32[0] = m_data[idx1Src    ].m128_f32[idx2Src];
            v1.m128_f32[0] = m_data[idx1Src + 1].m128_f32[idx2Src];
            u2.m128_f32[0] = m_data[idx1Src + 2].m128_f32[idx2Src];
            v2.m128_f32[0] = m_data[idx1Src + 3].m128_f32[idx2Src];
            iSrc = idxs[iDstp1];
            idx2Src = (iSrc & 3);
            idx1Src = iSrc - idx2Src;
            u1.m128_f32[1] = m_data[idx1Src    ].m128_f32[idx2Src];
            v1.m128_f32[1] = m_data[idx1Src + 1].m128_f32[idx2Src];
            u2.m128_f32[1] = m_data[idx1Src + 2].m128_f32[idx2Src];
            v2.m128_f32[1] = m_data[idx1Src + 3].m128_f32[idx2Src];
            iSrc = idxs[iDstp2];
            idx2Src = (iSrc & 3);
            idx1Src = iSrc - idx2Src;
            u1.m128_f32[2] = m_data[idx1Src    ].m128_f32[idx2Src];
            v1.m128_f32[2] = m_data[idx1Src + 1].m128_f32[idx2Src];
            u2.m128_f32[2] = m_data[idx1Src + 2].m128_f32[idx2Src];
            v2.m128_f32[2] = m_data[idx1Src + 3].m128_f32[idx2Src];
            iSrc = idxs[iDstp3];
            idx2Src = (iSrc & 3);
            idx1Src = iSrc - idx2Src;
            u1.m128_f32[3] = m_data[idx1Src    ].m128_f32[idx2Src];
            v1.m128_f32[3] = m_data[idx1Src + 1].m128_f32[idx2Src];
            u2.m128_f32[3] = m_data[idx1Src + 2].m128_f32[idx2Src];
            v2.m128_f32[3] = m_data[idx1Src + 3].m128_f32[idx2Src];
        }
        for(ushort iRem = 0; iRem < subset.m_nRems; ++iRem) {
            ENFT_SSE::__m128 &u1 = subset.m_data[iDst], &v1 = subset.m_data[iDstp1], &u2 = subset.m_data[iDstp2], &v2 = subset.m_data[iDstp3];
            Point2D &x1 = subset.m_rems1[iRem], &x2 = subset.m_rems2[iRem];
            iSrc = idxs[iDst + iRem];
            idx2Src = (iSrc & 3);
            idx1Src = iSrc - idx2Src;
            u1.m128_f32[iRem] = x1.x() = m_data[idx1Src ].m128_f32[idx2Src];
            v1.m128_f32[iRem] = x1.y() = m_data[idx1Src + 1].m128_f32[idx2Src];
            u2.m128_f32[iRem] = x2.x() = m_data[idx1Src + 2].m128_f32[idx2Src];
            v2.m128_f32[iRem] = x2.y() = m_data[idx1Src + 3].m128_f32[idx2Src];
        }
    }
    inline void Normalize(ENFT_SSE::__m128 *work6) {
        work6[0] = work6[1] = work6[2] = work6[3] = ENFT_SSE::_mm_setzero_ps();
        for(ushort iPack = 0; iPack < m_nPacks; iPack += 4) {
            work6[0] = ENFT_SSE::_mm_add_ps(m_data[iPack    ], work6[0]);
            work6[1] = ENFT_SSE::_mm_add_ps(m_data[iPack + 1], work6[1]);
            work6[2] = ENFT_SSE::_mm_add_ps(m_data[iPack + 2], work6[2]);
            work6[3] = ENFT_SSE::_mm_add_ps(m_data[iPack + 3], work6[3]);
        }
        for(ushort iRem = 0; iRem < m_nRems; ++iRem) {
            work6[0].m128_f32[iRem] += m_rems1[iRem].x();
            work6[1].m128_f32[iRem] += m_rems1[iRem].y();
            work6[2].m128_f32[iRem] += m_rems2[iRem].x();
            work6[3].m128_f32[iRem] += m_rems2[iRem].y();
        }
        m_mean_u1v1u2v2 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set_ps(ENFT_SSE::SSE::Sum0123(work6[3]), ENFT_SSE::SSE::Sum0123(work6[2]),
                                               ENFT_SSE::SSE::Sum0123(work6[1]), ENFT_SSE::SSE::Sum0123(work6[0])), ENFT_SSE::_mm_set1_ps(1.0f / m_N));

        work6[0] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[0]);
        work6[1] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[1]);
        work6[2] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[2]);
        work6[3] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[3]);
        work6[4] = work6[5] = ENFT_SSE::_mm_setzero_ps();
        for(ushort iPack = 0; iPack < m_nPacks; iPack += 4) {
            m_data[iPack    ] = ENFT_SSE::_mm_sub_ps(m_data[iPack    ], work6[0]);
            m_data[iPack + 1] = ENFT_SSE::_mm_sub_ps(m_data[iPack + 1], work6[1]);
            m_data[iPack + 2] = ENFT_SSE::_mm_sub_ps(m_data[iPack + 2], work6[2]);
            m_data[iPack + 3] = ENFT_SSE::_mm_sub_ps(m_data[iPack + 3], work6[3]);

            work6[4] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_sqrt_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_data[iPack    ], m_data[iPack    ]),
                                            ENFT_SSE::_mm_mul_ps(m_data[iPack + 1], m_data[iPack + 1]))), work6[4]);
            work6[5] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_sqrt_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_data[iPack + 2], m_data[iPack + 2]),
                                            ENFT_SSE::_mm_mul_ps(m_data[iPack + 3], m_data[iPack + 3]))), work6[5]);
        }
        for(ushort iRem = 0; iRem < m_nRems; ++iRem) {
            m_rems1[iRem].x() -= m_mean_u1v1u2v2.m128_f32[0];
            m_rems1[iRem].y() -= m_mean_u1v1u2v2.m128_f32[1];
            m_rems2[iRem].x() -= m_mean_u1v1u2v2.m128_f32[2];
            m_rems2[iRem].y() -= m_mean_u1v1u2v2.m128_f32[3];

            work6[4].m128_f32[iRem] = sqrt(m_rems1[iRem].x() * m_rems1[iRem].x() + m_rems1[iRem].y() * m_rems1[iRem].y()) + work6[4].m128_f32[iRem];
            work6[5].m128_f32[iRem] = sqrt(m_rems2[iRem].x() * m_rems2[iRem].x() + m_rems2[iRem].y() * m_rems2[iRem].y()) + work6[5].m128_f32[iRem];
        }
        //m_scale1 = sqrt(2.0f) * m_N / ENFT_SSE::SSE::Sum0123(work6[4]);
        //m_scale2 = sqrt(2.0f) * m_N / ENFT_SSE::SSE::Sum0123(work6[5]);
        m_scale1 = 1.4142135623730950488016887242097f * m_N / ENFT_SSE::SSE::Sum0123(work6[4]);
        m_scale2 = 1.4142135623730950488016887242097f * m_N / ENFT_SSE::SSE::Sum0123(work6[5]);

        work6[0] = ENFT_SSE::_mm_set1_ps(m_scale1);
        work6[1] = ENFT_SSE::_mm_set1_ps(m_scale2);
        for(ushort iPack = 0; iPack < m_nPacks; iPack += 4) {
            m_data[iPack    ] = ENFT_SSE::_mm_mul_ps(work6[0], m_data[iPack    ]);
            m_data[iPack + 1] = ENFT_SSE::_mm_mul_ps(work6[0], m_data[iPack + 1]);
            m_data[iPack + 2] = ENFT_SSE::_mm_mul_ps(work6[1], m_data[iPack + 2]);
            m_data[iPack + 3] = ENFT_SSE::_mm_mul_ps(work6[1], m_data[iPack + 3]);
        }
        for(ushort iRem = 0; iRem < m_nRems; ++iRem) {
            m_rems1[iRem].x() *= m_scale1;
            m_rems1[iRem].y() *= m_scale1;
            m_rems2[iRem].x() *= m_scale2;
            m_rems2[iRem].y() *= m_scale2;
        }
    }
    inline float ComputeDisparity() const {
        if(m_N == 0)
            return 0;
        std::vector<float> distSqs;
        distSqs.reserve(m_N);
        ENFT_SSE::__m128 du, dv, distSq;
        for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < m_nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4) {
            du = ENFT_SSE::_mm_sub_ps(m_data[i], m_data[ip2]);
            dv = ENFT_SSE::_mm_sub_ps(m_data[ip1], m_data[ip3]);
            distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(du, du), ENFT_SSE::_mm_mul_ps(dv, dv));
            distSqs.push_back(distSq.m128_f32[0]);
            distSqs.push_back(distSq.m128_f32[1]);
            distSqs.push_back(distSq.m128_f32[2]);
            distSqs.push_back(distSq.m128_f32[3]);
        }
        for(ushort iRem = 0; iRem < m_nRems; ++iRem)
            distSqs.push_back(m_rems1[iRem].SquaredDistance(m_rems2[iRem]));
        const ushort ith = (m_N >> 1);
        std::nth_element(distSqs.begin(), distSqs.begin() + ith, distSqs.end());
        return sqrt(distSqs[ith]);
    }
    inline const ENFT_SSE::__m128 &mean_u1v1u2v2() const {
        return m_mean_u1v1u2v2;
    }
    inline const float &scale1() const {
        return m_scale1;
    }
    inline const float &scale2() const {
        return m_scale2;
    }
    inline const ushort &GetPacksNumber    () const {
        return m_nPacks;
    }
    inline const ushort &GetRemindersNumber() const {
        return m_nRems;
    }
    const ENFT_SSE::__m128 &GetPack(const ushort &iPack) const {
        return m_data[iPack];
    }
    const Point2D &GetReminder1(const ushort &iRem) const {
        return m_rems1[iRem];
    }
    const Point2D &GetReminder2(const ushort &iRem) const {
        return m_rems2[iRem];
    }
    inline void SaveB(const char *fileName) const {
        FILE *fp = fopen( fileName, "wb");
        fwrite(&m_N, sizeof(ushort), 1, fp);
        fwrite(&m_nPacks, sizeof(ushort), 1, fp);
        fwrite(&m_nRems, sizeof(ushort), 1, fp);
        m_data.SaveB(fp);
        fwrite(m_rems1, sizeof(Point2D), 3, fp);
        fwrite(m_rems2, sizeof(Point2D), 3, fp);
        fwrite(&m_mean_u1v1u2v2, sizeof(ENFT_SSE::__m128), 1, fp);
        fwrite(&m_scale1, sizeof(float), 1, fp);
        fwrite(&m_scale2, sizeof(float), 1, fp);
        fclose(fp);
    }
    inline void LoadB(const char *fileName) {
        FILE *fp = fopen( fileName, "rb");
        fread(&m_N, sizeof(ushort), 1, fp);
        fread(&m_nPacks, sizeof(ushort), 1, fp);
        fread(&m_nRems, sizeof(ushort), 1, fp);
        m_data.LoadB(fp);
        fread(m_rems1, sizeof(Point2D), 3, fp);
        fread(m_rems2, sizeof(Point2D), 3, fp);
        fread(&m_mean_u1v1u2v2, sizeof(ENFT_SSE::__m128), 1, fp);
        fread(&m_scale1, sizeof(float), 1, fp);
        fread(&m_scale2, sizeof(float), 1, fp);
        fclose(fp);
    }

  protected:

    ushort m_N, m_nPacks, m_nRems;
    AlignedVector<ENFT_SSE::__m128> m_data;         // [0, 3]: u1_0123, v1_0123, u2_0123, v2_0123
    // [4, 7]: u1_4567, v1_4567, u2_4567, v2_4567
    // ...
    Point2D m_rems1[3], m_rems2[3];
    //ENFT_SSE::__m128 m_sum_u1, m_sum_v1, m_sum_u2, m_sum_v2;
    ENFT_SSE::__m128 m_mean_u1v1u2v2;
    float m_scale1, m_scale2;
    std::vector<std::pair<float, ushort> > m_scores;

//public:
//
//  inline void SetImageSize(const ushort &width, const ushort &height) { m_width = width; m_height = height; }
//  inline const ushort& GetImageWidth() const { return m_width; }
//  inline const ushort& GetImageHeight() const { return m_height; }
//  inline void SetImageLocation(const ushort &i, const Point2D &x) { m_imgLocations[i] = x; }
//  inline const Point2D& GetImageLocation(const ushort &i) const { return m_imgLocations[i]; }
//
//protected:
//
//  ushort m_width, m_height;
//  AlignedVector<Point2D> m_imgLocations;

};

class FourMatches2D {

  public:

    inline void Set(const MatchSet2D &data, const ushort &iSrc, const ushort &iDst) {
#if _DEBUG
        assert(iSrc < data.Size() && iDst < 4);
#endif
        data.GetMatch(iSrc, m_u1.m128_f32[iDst], m_v1.m128_f32[iDst], m_u2.m128_f32[iDst], m_v2.m128_f32[iDst]);
    }
    inline const ENFT_SSE::__m128 &u1() const {
        return m_u1;
    }
    inline const ENFT_SSE::__m128 &v1() const {
        return m_v1;
    }
    inline const ENFT_SSE::__m128 &u2() const {
        return m_u2;
    }
    inline const ENFT_SSE::__m128 &v2() const {
        return m_v2;
    } inline const ENFT_SSE::__m128 &mean_u1v1u2v2() const {
        return m_mean_u1v1u2v2;
    }
    inline const float &scale1() const {
        return m_scale1;
    }
    inline const float &scale2() const {
        return m_scale2;
    }
    inline void Normalize(ENFT_SSE::__m128 *work) {
        m_mean_u1v1u2v2 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set_ps(ENFT_SSE::SSE::Sum0123(m_v2), ENFT_SSE::SSE::Sum0123(m_u2), ENFT_SSE::SSE::Sum0123(m_v1),
                                               ENFT_SSE::SSE::Sum0123(m_u1)), ENFT_SSE::_mm_set1_ps(0.25f));
        /*ENFT_SSE::__m128 sub*/work[0] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[0]);
        m_u1 = ENFT_SSE::_mm_sub_ps(m_u1, /*sub*/work[0]);      /*m_dist2_u1_0123*/work[1] = ENFT_SSE::_mm_mul_ps(m_u1, m_u1);
        /*sub*/work[0] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[1]);
        m_v1 = ENFT_SSE::_mm_sub_ps(m_v1, /*sub*/work[0]);      /*m_dist2_v1_0123*/work[2] = ENFT_SSE::_mm_mul_ps(m_v1, m_v1);
        float sumDist = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_sqrt_ps(ENFT_SSE::_mm_add_ps(/*m_dist2_u1_0123, m_dist2_v1_0123*/work[1], work[2])));
        //m_scale1 = 4 * sqrt(2.0f) / sumDist;
        m_scale1 = 5.65685424949238f / sumDist;
        work[0] = ENFT_SSE::_mm_set1_ps(m_scale1);
        m_u1 = ENFT_SSE::_mm_mul_ps(m_u1, work[0]);
        m_v1 = ENFT_SSE::_mm_mul_ps(m_v1, work[0]);

        /*sub*/work[0] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[2]);
        m_u2 = ENFT_SSE::_mm_sub_ps(m_u2, /*sub*/work[0]);      /*m_dist2_u2_0123*/work[1] = ENFT_SSE::_mm_mul_ps(m_u2, m_u2);
        /*sub*/work[0] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[3]);
        m_v2 = ENFT_SSE::_mm_sub_ps(m_v2, /*sub*/work[0]);      /*m_dist2_v2_0123*/work[2] = ENFT_SSE::_mm_mul_ps(m_v2, m_v2);
        sumDist = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_sqrt_ps(ENFT_SSE::_mm_add_ps(/*m_dist2_u2_0123, m_dist2_v2_0123*/work[1], work[2])));
        //m_scale2 = 4 * sqrt(2.0f) / sumDist;
        m_scale2 = 5.65685424949238f / sumDist;
        work[0] = ENFT_SSE::_mm_set1_ps(m_scale2);
        m_u2 = ENFT_SSE::_mm_mul_ps(m_u2, work[0]);
        m_v2 = ENFT_SSE::_mm_mul_ps(m_v2, work[0]);
    }

  private:

    ENFT_SSE::__m128 m_u1, m_v1, m_u2, m_v2, m_mean_u1v1u2v2;
    float m_scale1, m_scale2;

};

class FiveMatches2D {

  public:

    inline const ENFT_SSE::__m128 &u1_0123() const {
        return m_u1_0123;
    }                inline ENFT_SSE::__m128 &u1_0123() {
        return m_u1_0123;
    }
    inline const ENFT_SSE::__m128 &v1_0123() const {
        return m_v1_0123;
    }                inline ENFT_SSE::__m128 &v1_0123() {
        return m_v1_0123;
    }
    inline const ENFT_SSE::__m128 &u2_0123() const {
        return m_u2_0123;
    }                inline ENFT_SSE::__m128 &u2_0123() {
        return m_u2_0123;
    }
    inline const ENFT_SSE::__m128 &v2_0123() const {
        return m_v2_0123;
    }                inline ENFT_SSE::__m128 &v2_0123() {
        return m_v2_0123;
    }
    inline const float &u1_0() const {
        return m_u1_0123.m128_f32[0];
    }      inline float &u1_0() {
        return m_u1_0123.m128_f32[0];
    }
    inline const float &v1_0() const {
        return m_v1_0123.m128_f32[0];
    }      inline float &v1_0() {
        return m_v1_0123.m128_f32[0];
    }
    inline const float &u2_0() const {
        return m_u2_0123.m128_f32[0];
    }      inline float &u2_0() {
        return m_u2_0123.m128_f32[0];
    }
    inline const float &v2_0() const {
        return m_v2_0123.m128_f32[0];
    }      inline float &v2_0() {
        return m_v2_0123.m128_f32[0];
    }
    inline const float &u1_1() const {
        return m_u1_0123.m128_f32[1];
    }      inline float &u1_1() {
        return m_u1_0123.m128_f32[1];
    }
    inline const float &v1_1() const {
        return m_v1_0123.m128_f32[1];
    }      inline float &v1_1() {
        return m_v1_0123.m128_f32[1];
    }
    inline const float &u2_1() const {
        return m_u2_0123.m128_f32[1];
    }      inline float &u2_1() {
        return m_u2_0123.m128_f32[1];
    }
    inline const float &v2_1() const {
        return m_v2_0123.m128_f32[1];
    }      inline float &v2_1() {
        return m_v2_0123.m128_f32[1];
    }
    inline const float &u1_2() const {
        return m_u1_0123.m128_f32[2];
    }      inline float &u1_2() {
        return m_u1_0123.m128_f32[2];
    }
    inline const float &v1_2() const {
        return m_v1_0123.m128_f32[2];
    }      inline float &v1_2() {
        return m_v1_0123.m128_f32[2];
    }
    inline const float &u2_2() const {
        return m_u2_0123.m128_f32[2];
    }      inline float &u2_2() {
        return m_u2_0123.m128_f32[2];
    }
    inline const float &v2_2() const {
        return m_v2_0123.m128_f32[2];
    }      inline float &v2_2() {
        return m_v2_0123.m128_f32[2];
    }
    inline const float &u1_3() const {
        return m_u1_0123.m128_f32[3];
    }      inline float &u1_3() {
        return m_u1_0123.m128_f32[3];
    }
    inline const float &v1_3() const {
        return m_v1_0123.m128_f32[3];
    }      inline float &v1_3() {
        return m_v1_0123.m128_f32[3];
    }
    inline const float &u2_3() const {
        return m_u2_0123.m128_f32[3];
    }      inline float &u2_3() {
        return m_u2_0123.m128_f32[3];
    }
    inline const float &v2_3() const {
        return m_v2_0123.m128_f32[3];
    }      inline float &v2_3() {
        return m_v2_0123.m128_f32[3];
    }
    inline const float &u1_4() const {
        return m_u1_4;
    }                     inline float &u1_4() {
        return m_u1_4;
    }
    inline const float &v1_4() const {
        return m_v1_4;
    }                     inline float &v1_4() {
        return m_v1_4;
    }
    inline const float &u2_4() const {
        return m_u2_4;
    }                     inline float &u2_4() {
        return m_u2_4;
    }
    inline const float &v2_4() const {
        return m_v2_4;
    }                     inline float &v2_4() {
        return m_v2_4;
    }

    inline void Set0(const MatchSet2D &data, const ushort &iSrc) {
        data.GetMatch(iSrc, u1_0(), v1_0(), u2_0(), v2_0());
    }
    inline void Set1(const MatchSet2D &data, const ushort &iSrc) {
        data.GetMatch(iSrc, u1_1(), v1_1(), u2_1(), v2_1());
    }
    inline void Set2(const MatchSet2D &data, const ushort &iSrc) {
        data.GetMatch(iSrc, u1_2(), v1_2(), u2_2(), v2_2());
    }
    inline void Set3(const MatchSet2D &data, const ushort &iSrc) {
        data.GetMatch(iSrc, u1_3(), v1_3(), u2_3(), v2_3());
    }
    inline void Set4(const MatchSet2D &data, const ushort &iSrc) {
        data.GetMatch(iSrc, u1_4(), v1_4(), u2_4(), v2_4());
    }
    inline void SaveB(const char *fileName) const {
        FILE *fp = fopen( fileName, "wb");
        const uint five = 5;

        fwrite(&five, sizeof(uint), 1, fp);
        fwrite(&u1_0(), sizeof(float), 1, fp);
        fwrite(&v1_0(), sizeof(float), 1, fp);
        fwrite(&u1_1(), sizeof(float), 1, fp);
        fwrite(&v1_1(), sizeof(float), 1, fp);
        fwrite(&u1_2(), sizeof(float), 1, fp);
        fwrite(&v1_2(), sizeof(float), 1, fp);
        fwrite(&u1_3(), sizeof(float), 1, fp);
        fwrite(&v1_3(), sizeof(float), 1, fp);
        fwrite(&u1_4(), sizeof(float), 1, fp);
        fwrite(&v1_4(), sizeof(float), 1, fp);

        fwrite(&five, sizeof(uint), 1, fp);
        fwrite(&u2_0(), sizeof(float), 1, fp);
        fwrite(&v2_0(), sizeof(float), 1, fp);
        fwrite(&u2_1(), sizeof(float), 1, fp);
        fwrite(&v2_1(), sizeof(float), 1, fp);
        fwrite(&u2_2(), sizeof(float), 1, fp);
        fwrite(&v2_2(), sizeof(float), 1, fp);
        fwrite(&u2_3(), sizeof(float), 1, fp);
        fwrite(&v2_3(), sizeof(float), 1, fp);
        fwrite(&u2_4(), sizeof(float), 1, fp);
        fwrite(&v2_4(), sizeof(float), 1, fp);

        fclose(fp);
    }

  private:

    ENFT_SSE::__m128 m_u1_0123, m_v1_0123, m_u2_0123, m_v2_0123;
    float m_u1_4, m_v1_4, m_u2_4, m_v2_4;

};

class EightMatches2D {

  public:

    EightMatches2D() {
        m_u1s = &m_u1_0123.m128_f32[0];
        m_v1s = &m_v1_0123.m128_f32[0];
        m_u2s = &m_u2_0123.m128_f32[0];
        m_v2s = &m_v2_0123.m128_f32[0];
    }

    inline const float *u1s() const {
        return m_u1s;
    }           inline float *u1s() {
        return m_u1s;
    }
    inline const float *v1s() const {
        return m_v1s;
    }           inline float *v1s() {
        return m_v1s;
    }
    inline const float *u2s() const {
        return m_u2s;
    }           inline float *u2s() {
        return m_u2s;
    }
    inline const float *v2s() const {
        return m_v2s;
    }           inline float *v2s() {
        return m_v2s;
    }
    inline const ENFT_SSE::__m128 &u1_0123() const {
        return m_u1_0123;
    }    inline ENFT_SSE::__m128 &u1_0123() {
        return m_u1_0123;
    }
    inline const ENFT_SSE::__m128 &u1_4567() const {
        return m_u1_4567;
    }    inline ENFT_SSE::__m128 &u1_4567() {
        return m_u1_4567;
    }
    inline const ENFT_SSE::__m128 &v1_0123() const {
        return m_v1_0123;
    }    inline ENFT_SSE::__m128 &v1_0123() {
        return m_v1_0123;
    }
    inline const ENFT_SSE::__m128 &v1_4567() const {
        return m_v1_4567;
    }    inline ENFT_SSE::__m128 &v1_4567() {
        return m_v1_4567;
    }
    inline const ENFT_SSE::__m128 &u2_0123() const {
        return m_u2_0123;
    }    inline ENFT_SSE::__m128 &u2_0123() {
        return m_u2_0123;
    }
    inline const ENFT_SSE::__m128 &u2_4567() const {
        return m_u2_4567;
    }    inline ENFT_SSE::__m128 &u2_4567() {
        return m_u2_4567;
    }
    inline const ENFT_SSE::__m128 &v2_0123() const {
        return m_v2_0123;
    }    inline ENFT_SSE::__m128 &v2_0123() {
        return m_v2_0123;
    }
    inline const ENFT_SSE::__m128 &v2_4567() const {
        return m_v2_4567;
    }    inline ENFT_SSE::__m128 &v2_4567() {
        return m_v2_4567;
    }
    inline const ENFT_SSE::__m128 &mean_u1v1u2v2() const {
        return m_mean_u1v1u2v2;
    }
    inline const float &scale1() const {
        return m_scale1;
    }
    inline const float &scale2() const {
        return m_scale2;
    }

    inline void Set(const MatchSet2D &data, const ushort &iSrc, const ushort &iDst) {
#if _DEBUG
        assert(iSrc < data.Size() && iDst < 8);
#endif
        data.GetMatch(iSrc, m_u1s[iDst], m_v1s[iDst], m_u2s[iDst], m_v2s[iDst]);
    }
    inline void Normalize(ENFT_SSE::__m128 *work) {
        m_mean_u1v1u2v2 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set_ps(ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(m_v2_0123, m_v2_4567)),
                                               ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(m_u2_0123, m_u2_4567)),
                                               ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(m_v1_0123, m_v1_4567)),
                                               ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(m_u1_0123, m_u1_4567))), ENFT_SSE::_mm_set1_ps(0.125f));
        /*ENFT_SSE::__m128 sub*/work[0] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[0]);
        m_u1_0123 = ENFT_SSE::_mm_sub_ps(m_u1_0123, /*sub*/work[0]);        /*m_dist2_u1_0123*/work[1] = ENFT_SSE::_mm_mul_ps(m_u1_0123, m_u1_0123);
        m_u1_4567 = ENFT_SSE::_mm_sub_ps(m_u1_4567, /*sub*/work[0]);        /*m_dist2_u1_4567*/work[2] = ENFT_SSE::_mm_mul_ps(m_u1_4567, m_u1_4567);
        /*sub*/work[0] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[1]);
        m_v1_0123 = ENFT_SSE::_mm_sub_ps(m_v1_0123, /*sub*/work[0]);        /*m_dist2_v1_0123*/work[3] = ENFT_SSE::_mm_mul_ps(m_v1_0123, m_v1_0123);
        m_v1_4567 = ENFT_SSE::_mm_sub_ps(m_v1_4567, /*sub*/work[0]);        /*m_dist2_v1_4567*/work[4] = ENFT_SSE::_mm_mul_ps(m_v1_4567, m_v1_4567);
        float sumDist = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_sqrt_ps(ENFT_SSE::_mm_add_ps(/*m_dist2_u1_0123, m_dist2_v1_0123*/work[1], work[3])))
                        + ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_sqrt_ps(ENFT_SSE::_mm_add_ps(/*m_dist2_u1_4567, m_dist2_v1_4567*/work[2], work[4])));
        //m_scale1 = 8 * sqrt(2.0f) / sumDist;
        m_scale1 = 11.31370849898476f / sumDist;
        work[0] = ENFT_SSE::_mm_set1_ps(m_scale1);
        m_u1_0123 = ENFT_SSE::_mm_mul_ps(m_u1_0123, work[0]);
        m_u1_4567 = ENFT_SSE::_mm_mul_ps(m_u1_4567, work[0]);
        m_v1_0123 = ENFT_SSE::_mm_mul_ps(m_v1_0123, work[0]);
        m_v1_4567 = ENFT_SSE::_mm_mul_ps(m_v1_4567, work[0]);

        /*sub*/work[0] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[2]);
        m_u2_0123 = ENFT_SSE::_mm_sub_ps(m_u2_0123, /*sub*/work[0]);        /*m_dist2_u2_0123*/work[1] = ENFT_SSE::_mm_mul_ps(m_u2_0123, m_u2_0123);
        m_u2_4567 = ENFT_SSE::_mm_sub_ps(m_u2_4567, /*sub*/work[0]);        /*m_dist2_u2_4567*/work[2] = ENFT_SSE::_mm_mul_ps(m_u2_4567, m_u2_4567);
        /*sub*/work[0] = ENFT_SSE::_mm_set1_ps(m_mean_u1v1u2v2.m128_f32[3]);
        m_v2_0123 = ENFT_SSE::_mm_sub_ps(m_v2_0123, /*sub*/work[0]);        /*m_dist2_v2_0123*/work[3] = ENFT_SSE::_mm_mul_ps(m_v2_0123, m_v2_0123);
        m_v2_4567 = ENFT_SSE::_mm_sub_ps(m_v2_4567, /*sub*/work[0]);        /*m_dist2_v2_4567*/work[4] = ENFT_SSE::_mm_mul_ps(m_v2_4567, m_v2_4567);
        sumDist = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_sqrt_ps(ENFT_SSE::_mm_add_ps(/*m_dist2_u2_0123, m_dist2_v2_0123*/work[1], work[3])))
                  + ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_sqrt_ps(ENFT_SSE::_mm_add_ps(/*m_dist2_u2_4567, m_dist2_v2_4567*/work[2], work[4])));
        //m_scale2 = 8 * sqrt(2.0f) / sumDist;
        m_scale2 = 11.31370849898476f / sumDist;
        work[0] = ENFT_SSE::_mm_set1_ps(m_scale2);
        m_u2_0123 = ENFT_SSE::_mm_mul_ps(m_u2_0123, work[0]);
        m_u2_4567 = ENFT_SSE::_mm_mul_ps(m_u2_4567, work[0]);
        m_v2_0123 = ENFT_SSE::_mm_mul_ps(m_v2_0123, work[0]);
        m_v2_4567 = ENFT_SSE::_mm_mul_ps(m_v2_4567, work[0]);
    }

  private:

    ENFT_SSE::__m128 m_u2_0123, m_u2_4567, m_v2_0123, m_v2_4567, m_u1_0123, m_u1_4567, m_v1_0123, m_v1_4567, m_mean_u1v1u2v2;
    /*ENFT_SSE::__m128 m_dist2_u2_0123, m_dist2_u2_4567, m_dist2_v2_0123, m_dist2_v2_4567, m_dist2_u1_0123, m_dist2_u1_4567, m_dist2_v1_0123, m_dist2_v1_4567;*/
    float m_scale1, m_scale2;
    float *m_u2s, *m_v2s, *m_u1s, *m_v1s;
};

//template<ushort N>
//class MatchSet3DTo2DN
//{
//
//public:
//
//  inline void Set(const AlignedVector<Point3D> &Xs, const AlignedVector<Point2D> &xs, const ushort &iSrc, const ushort &iDst)
//  {
//      m_pXs[iDst] = &Xs[iSrc];
//      m_pxs[iDst] = &xs[iSrc];
//  }
//  inline const Point3D& X(const ushort &i) const { return *m_pXs[i]; }
//  inline const Point2D& x(const ushort &i) const { return *m_pxs[i]; }
//
//protected:
//
//  const Point3D *m_pXs[N];
//  const Point2D *m_pxs[N];
//
//};

template<ushort N>
class MatchSet3DTo2DN {

  public:

    inline void Set(const AlignedVector<Point3D> &Xs, const AlignedVector<Point2D> &xs, const ushort &iSrc, const ushort &iDst) {
        m_Xs[iDst] = Xs[iSrc];
        m_xs[iDst] = xs[iSrc];
    }
    inline const Point3D &X(const ushort &i) const {
        return m_Xs[i];
    }
    inline const Point2D &x(const ushort &i) const {
        return m_xs[i];
    }
    inline const Point3D *Xs() const {
        return m_Xs;
    }
    inline const Point2D *xs() const {
        return m_xs;
    }

    inline const Point3D &cX() const {
        return m_cX;
    }   inline const ENFT_SSE::__m128 &sX() const {
        return m_sX;
    }
    inline const Point2D &cx() const {
        return m_cx;
    }   inline const ENFT_SSE::__m128 &sx() const {
        return m_sx;
    }
    inline void Normalize() {
        m_cX.SetZero();
        for(ushort i = 0; i < N; ++i)
            m_cX += m_Xs[i];
        const float one_over_N = 1.0f / N;
        m_cX *= ENFT_SSE::_mm_set1_ps(one_over_N);

        float sX = 0;
        for(ushort i = 0; i < N; ++i) {
            Point3D &X = m_Xs[i];
            X -= m_cX;
            sX = sqrt(X.SquaredLength()) + sX;
        }
        const float sqrt2_N = 1.4142135623730950488016887242097f * N;
        m_sX = ENFT_SSE::_mm_set1_ps(sqrt2_N / sX);

        for(ushort i = 0; i < N; ++i) {
            m_Xs[i] *= m_sX;
            m_Xs[i].reserve() = 1;
        }

        m_cx.SetZero();
        for(ushort i = 0; i < N; ++i)
            m_cx += m_xs[i];
        m_cx *= one_over_N;

        float sx = 0;
        for(ushort i = 0; i < N; ++i) {
            Point2D &x = m_xs[i];
            x -= m_cx;
            sx = sqrt(x.SquaredLength()) + sx;
        }
        sx = sqrt2_N / sx;

        m_sx = ENFT_SSE::_mm_set1_ps(sx);
        const ushort _N = N - (N & 1);
        ENFT_SSE::__m128 *x2 = (ENFT_SSE::__m128 *) m_xs;
        for(ushort i = 0; i < _N; i += 2, ++x2)
            *x2 = ENFT_SSE::_mm_mul_ps(*x2, m_sx);
        if(_N != N)
            m_xs[_N] *= sx;
    }

  protected:

    Point3D m_Xs[N];
    Point2D m_xs[N];

    Point3D m_cX;
    ENFT_SSE::__m128 m_sX;
    Point2D m_cx;
    ENFT_SSE::__m128 m_sx;

};

typedef MatchSet3DTo2DN<2> TwoMatches3DTo2D;
typedef MatchSet3DTo2DN<6> SixMatches3DTo2D;
typedef MatchSet3DTo2DN<8> EightMatches3DTo2D;

class MatchSet3DTo2DX {

  public:

    inline void Resize(const ushort &N) {
        m_Xs.Resize(N);
        m_xs.Resize(N);
    }
    inline void Reserve(const ushort &N) {
        m_Xs.Reserve(N);
        m_xs.Reserve(N);
    }
    inline const ushort Size() const {
        return m_Xs.Size();
    }
    inline const AlignedVector<Point3D> &Xs() const {
        return m_Xs;
    }
    inline       AlignedVector<Point3D> &Xs()       {
        return m_Xs;
    }
    inline const Point3D &X(const ushort &i) const {
        return m_Xs[i];
    }
    inline       Point3D &X(const ushort &i)       {
        return m_Xs[i];
    }
    inline const AlignedVector<Point2D> &xs() const {
        return m_xs;
    }
    inline       AlignedVector<Point2D> &xs()       {
        return m_xs;
    }
    inline const Point2D &x(const ushort &i) const {
        return m_xs[i];
    }
    inline       Point2D &x(const ushort &i)       {
        return m_xs[i];
    }
    inline void Set(const ushort &i, const Point3D &X, const Point2D &x) {
        m_Xs[i] = X;
        m_xs[i] = x;
    }
    inline void GetSubset(const std::vector<ushort> &idxs, MatchSet3DTo2DX &subset) const {
        const ushort N = ushort(idxs.size());
        subset.Resize(N);
        for(ushort i = 0; i < N; ++i)
            subset.Set(i, m_Xs[idxs[i]], m_xs[idxs[i]]);
    }
    inline void PushBack(const MatchSet3DTo2DX &data) {
        const ushort N1 = Size(), N2 = data.Size(), N = N1 + N2;
        m_Xs.EnlargeCapacity(N);
        m_xs.EnlargeCapacity(N);
        m_Xs.PushBack(data.m_Xs.Data(), N2);
        m_xs.PushBack(data.m_xs.Data(), N2);
    }

    inline const Point3D &cX() const {
        return m_cX;
    }   inline const ENFT_SSE::__m128 &sX() const {
        return m_sX;
    }
    inline const Point2D &cx() const {
        return m_cx;
    }   inline const ENFT_SSE::__m128 &sx() const {
        return m_sx;
    }
    inline void Normalize() {
        const ushort N = Size();
        m_cX.SetZero();
        for(ushort i = 0; i < N; ++i)
            m_cX += m_Xs[i];
        const float one_over_N = 1.0f / N;
        m_cX *= ENFT_SSE::_mm_set1_ps(one_over_N);

        float sX = 0;
        for(ushort i = 0; i < N; ++i) {
            Point3D &X = m_Xs[i];
            X -= m_cX;
            sX = sqrt(X.SquaredLength()) + sX;
        }
        const float sqrt2_N = 1.4142135623730950488016887242097f * N;
        m_sX = ENFT_SSE::_mm_set1_ps(sqrt2_N / sX);

        for(ushort i = 0; i < N; ++i) {
            m_Xs[i] *= m_sX;
            m_Xs[i].reserve() = 1;
        }

        m_cx.SetZero();
        for(ushort i = 0; i < N; ++i)
            m_cx += m_xs[i];
        m_cx *= one_over_N;

        float sx = 0;
        for(ushort i = 0; i < N; ++i) {
            Point2D &x = m_xs[i];
            x -= m_cx;
            sx = sqrt(x.SquaredLength()) + sx;
        }
        sx = sqrt2_N / sx;

        m_sx = ENFT_SSE::_mm_set1_ps(sx);
        const ushort _N = N - (N & 1);
        ENFT_SSE::__m128 *x2 = (ENFT_SSE::__m128 *) m_xs.Data();
        for(ushort i = 0; i < _N; i += 2, ++x2)
            *x2 = ENFT_SSE::_mm_mul_ps(*x2, m_sx);
        if(_N != N)
            m_xs[_N] *= sx;
    }

    inline void SaveB(FILE *fp) const {
        m_Xs.SaveB(fp);
        m_xs.SaveB(fp);
    }
    inline void LoadB(FILE *fp) {
        m_Xs.LoadB(fp);
        m_xs.LoadB(fp);
    }

  protected:

    AlignedVector<Point3D> m_Xs;
    AlignedVector<Point2D> m_xs;

    Point3D m_cX;
    ENFT_SSE::__m128 m_sX;
    Point2D m_cx;
    ENFT_SSE::__m128 m_sx;

};

template<ushort N>
class MatchSet3DN {

  public:

    inline void Set(const ushort &i, const Point3D &X1, const Point3D &X2) {
        m_pX1s[i] = &X1;
        m_pX2s[i] = &X2;
    }
    inline const Point3D &X1(const ushort &i) const {
        return *m_pX1s[i];
    }
    inline const Point3D &X2(const ushort &i) const {
        return *m_pX2s[i];
    }

  private:

    const Point3D *m_pX1s[N], *m_pX2s[N];
};

typedef MatchSet3DN<3> ThreeMatches3D;
typedef MatchSet3DN<4> FourMatches3D;
typedef MatchSet3DN<6> SixMatches3D;
typedef MatchSet3DN<8> EightMatches3D;

class MatchSet3DX {

  public:

    inline const Point3D &X1(const uint &i) const {
        return m_X1s[i];
    }  inline Point3D &X1(const uint &i) {
        return m_X1s[i];
    }
    inline const Point3D &X2(const uint &i) const {
        return m_X2s[i];
    }  inline Point3D &X2(const uint &i) {
        return m_X2s[i];
    }
    inline const AlignedVector<Point3D> &X1s() const {
        return m_X1s;
    }  inline AlignedVector<Point3D> &X1s() {
        return m_X1s;
    }
    inline const AlignedVector<Point3D> &X2s() const {
        return m_X2s;
    }  inline AlignedVector<Point3D> &X2s() {
        return m_X2s;
    }
    inline const uint Size() const {
        return m_X1s.Size();
    }
    inline void Resize(const uint &N) {
        m_X1s.Resize(N);
        m_X2s.Resize(N);
    }
    inline void GetSubset(const std::vector<uint> &idxs, MatchSet3DX &subset) const {
        const uint N = uint(idxs.size());
        subset.Resize(N);
        for(uint i = 0; i < N; ++i) {
            subset.m_X1s[i] = m_X1s[idxs[i]];
            subset.m_X2s[i] = m_X2s[idxs[i]];
        }
    }
    inline void ComputeMeanAndVariance1(Point3D &mean1, float &var1) const {
        Point3D sum;
        sum.SetZero();
        const uint N = Size();
        for(uint i = 0; i < N; ++i)
            LA::ApB(m_X1s[i], sum, sum);
        mean1.XYZx() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1.0f / N), sum.XYZx());

        Point3D dX;
        sum.SetZero();
        for(uint i = 0; i < N; ++i) {
            LA::AmB(m_X1s[i], mean1, dX);
            sum.XYZx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(dX.XYZx(), dX.XYZx()), sum.XYZx());
        }
        var1 = ENFT_SSE::SSE::Sum012(sum.XYZx()) / N;
    }

  protected:

    AlignedVector<Point3D> m_X1s, m_X2s;
};

#endif