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

#ifndef _TRANSLATION_SCALE_OPTIMIZER_H_
#define _TRANSLATION_SCALE_OPTIMIZER_H_

#include "TranslationScaleEstimatorData.h"
#include <levmar.h>
class TranslationScaleOptimizer {

  public:

    inline void Run(TranslationScaleEstimatorData &data, float &scale,
                    const ushort &maxNumIters, AlignedVector<ENFT_SSE::__m128> &work,
                    const bool &verbose = false) {
        Normalize(data, scale);

        Initialize(data, scale);
        const uint N = data.Size(), Nx2 = (N << 1);
        const uint lworkf1 = LM_DER_WORKSZ(1, Nx2);
        //const uint lworkf1 = LM_DIF_WORKSZ(1, Nx2);
        const uint lworkf2 = 1;
        const uint lworkf3 = LM_INFO_SZ;
        const uint lworkf4 = N;
        const uint lworkf5 = Nx2;
        const uint lworkf12345 = lworkf1 + lworkf2 + lworkf3 + lworkf4 + lworkf5;
        const uint lworkm12345 = ((lworkf12345 + 3) >> 2);
        const uint lworkm6 = 2;
        const uint lworkm = lworkm12345 + lworkm6;
        work.Resize(lworkm);
        float *workf = (float *) work.Data();
        float *p = workf + lworkf1;
        float *info = p + lworkf2;
        m_intermediateResults = info + lworkf3;
        m_xs = m_intermediateResults + lworkf4;
        m_work2m = work.Data() + lworkm12345;

        *p = 0;
        slevmar_der(func, jacf, p, (float *) data.xs().Data(), 1, Nx2, int(maxNumIters),
                    NULL, info, workf, NULL, this);
        //slevmar_dif(func, p, (float *) data.xs().Data(), 1, Nx2, int(maxNumIters), NULL, info, workf, NULL, this);
        scale = UpdateScale(p);

        DenormalizeScale(scale);
        if(verbose)
            PrintLMInfo(info);
    }

    static inline void PrintLMInfo(const float *info) {
        std::string stop;
        switch(ushort(info[6])) {
            case 1:
                stop = "J^T e";
                break;
            case 2:
                stop = "dp";
                break;
            case 3:
                stop = "itmax";
                break;
            case 4:
                stop = "singular matrix";
                break;
            case 5:
                stop = "no further error reduction is possible";
                break;
            case 6:
                stop = "||e||_2";
                break;
            case 7:
                stop = "NaN or Inf";
                break;
        }
        printf("  %d iterations: %e --> %e, stop by %s\n", ushort(info[5]), info[0],
               info[1], stop.c_str());
    }

  private:

    inline void Normalize(TranslationScaleEstimatorData &data, float &scale) {
        const float stZ = data.GetTranslation().v2() * scale;
        const ushort N = data.Size();
        m_ds.resize(N);
        for(ushort i = 0; i < N; ++i)
            m_ds[i] = data.X(i).Z() + stZ;
        const ushort ith = (N >> 1);
        std::nth_element(m_ds.begin(), m_ds.begin() + ith, m_ds.end());
        const float dMed = m_ds[ith];
        m_norm =          ::_mm_set1_ps((1 / dMed) / 0.5f);
        m_norm.m128_f32[3] = 1;

        scale *= m_norm.m128_f32[0];
        for(ushort i = 0; i < N; ++i)
            data.X(i) *= m_norm;
    }
    inline void DenormalizeScale(float &scale) {
        scale /= m_norm.m128_f32[0];
    }

    inline void Initialize(const TranslationScaleEstimatorData &data,
                           const float &scaleInit) {
        m_pData = &data;
        m_pScale = &scaleInit;
    }
    inline const float &UpdateScale(const float *const &p) {
        m_scale = *m_pScale + p[0];
        return m_scale;
    }
    inline void Compute_hx(const float *const &p) {
        UpdateScale(p);
        m_p[0] = p[0];

        const TranslationScaleEstimatorData &data = *m_pData;
        ENFT_SSE::__m128 &st = m_work2m[0], &RXpst = m_work2m[1];
        st = ENFT_SSE::_mm_mul_ps(data.GetTranslation().v012x(), ENFT_SSE::_mm_set1_ps(m_scale));
        const ushort N = data.Size();
        float *intermediateResult = m_intermediateResults, *x = m_xs;
        for(ushort i = 0; i < N; ++i, ++intermediateResult, x += 2) {
            RXpst = ENFT_SSE::_mm_add_ps(data.X(i).v012x(), st);
            *intermediateResult = 1 / RXpst.m128_f32[2];
            x[0] = RXpst.m128_f32[0] * (*intermediateResult);
            x[1] = RXpst.m128_f32[1] * (*intermediateResult);
        }
    }
    inline void Get_hx(float *const &hx) {
        memcpy(hx, m_xs, (m_pData->Size() << 3));
    }
    inline void Compute_j(const float *const &p, float *j) {
        if(m_p[0] != p[0])
            Compute_hx(p);
        const TranslationScaleEstimatorData &data = *m_pData;
        const LA::AlignedVector3f &t = data.GetTranslation();
        const ushort N = data.Size();
        const float *intermediateResult = m_intermediateResults, *x = m_xs;
        for(ushort i = 0; i < N; ++i, j += 2, ++intermediateResult, x += 2) {
            j[0] = (t.v0() - t.v2() * x[0]) * (*intermediateResult);
            j[1] = (t.v1() - t.v2() * x[1]) * (*intermediateResult);
        }
    }

  private:

    static void func(float *p, float *hx, int m, int n, void *adata) {
        TranslationScaleOptimizer &optimizer = *(TranslationScaleOptimizer *) adata;
        optimizer.Compute_hx(p);
        optimizer.Get_hx(hx);
    }
    static void jacf(float *p, float *j, int m, int n, void *adata) {
        TranslationScaleOptimizer &optimizer = *(TranslationScaleOptimizer *) adata;
        optimizer.Compute_j(p, j);
    }

  private:

    float m_scale;
    ENFT_SSE::__m128 m_norm;

    const TranslationScaleEstimatorData *m_pData;
    const float *m_pScale;

    float m_p[1];
    std::vector<float> m_ds;
    float *m_intermediateResults, *m_xs;
    ENFT_SSE::__m128 *m_work2m;

};

#endif