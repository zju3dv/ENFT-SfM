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

#ifndef _DESCRIPTOR_H_
#define _DESCRIPTOR_H_

#include "Utility/AlignedVector.h"

#define DESCRIPTOR_DIMENSION        64
//#define DESCRIPTOR_VECTOR_GROWTH  10000
//#define DESCRIPTOR_VECTOR_GROWTH  2000

class Descriptor64f;

#if DESCRIPTOR_DIMENSION == 64
typedef Descriptor64f Descriptor;
#endif

//typedef AlignedVector<Descriptor, DESCRIPTOR_VECTOR_GROWTH> DescriptorVector;

class Descriptor128ub {

  public:

    inline operator const float *() const {
        return (const float *) this;
    }
    inline operator       float *()       {
        return (      float *) this;
    }
    inline void SetZero() {
        memset(this, 0, sizeof(Descriptor128ub));
    }

  private:

    ENFT_SSE::__m128 m_sse0, m_sse1, m_sse2, m_sse3, m_sse4, m_sse5, m_sse6, m_sse7;
};

class Descriptor64f {
  public:
    typedef float ElementType;
    inline operator const float *() const {
        return (const float *) this;
    }
    inline operator       float *()       {
        return (      float *) this;
    }
    inline const int *intptr()  const  {
        return (const int *) this;
    }
    inline int *intptr()          {
        return (int *) this;
    }
    inline void Invalidate() {
        m_sse0.m128_f32[0] = -1;
    }
    inline bool IsValid() const {
        return m_sse0.m128_f32[0] >= 0;
    }
    inline bool IsInvalid() const {
        return m_sse0.m128_f32[0] < 0;
    }
    inline void SetZero() {
        memset(this, 0, sizeof(Descriptor64f));
    }
    inline void Scale(const float &scale) {
        const ENFT_SSE::__m128 s = ENFT_SSE::_mm_set1_ps(scale);
        m_sse0 = ENFT_SSE::_mm_mul_ps(m_sse0, s);
        m_sse1 = ENFT_SSE::_mm_mul_ps(m_sse1, s);
        m_sse2 = ENFT_SSE::_mm_mul_ps(m_sse2, s);
        m_sse3 = ENFT_SSE::_mm_mul_ps(m_sse3, s);
        m_sse4 = ENFT_SSE::_mm_mul_ps(m_sse4, s);
        m_sse5 = ENFT_SSE::_mm_mul_ps(m_sse5, s);
        m_sse6 = ENFT_SSE::_mm_mul_ps(m_sse6, s);
        m_sse7 = ENFT_SSE::_mm_mul_ps(m_sse7, s);
        m_sse8 = ENFT_SSE::_mm_mul_ps(m_sse8, s);
        m_sse9 = ENFT_SSE::_mm_mul_ps(m_sse9, s);
        m_sse10 = ENFT_SSE::_mm_mul_ps(m_sse10, s);
        m_sse11 = ENFT_SSE::_mm_mul_ps(m_sse11, s);
        m_sse12 = ENFT_SSE::_mm_mul_ps(m_sse12, s);
        m_sse13 = ENFT_SSE::_mm_mul_ps(m_sse13, s);
        m_sse14 = ENFT_SSE::_mm_mul_ps(m_sse14, s);
        m_sse15 = ENFT_SSE::_mm_mul_ps(m_sse15, s);
    }
    inline void Scale(const float &scale, const float &maxVal) {
        const ENFT_SSE::__m128 s = ENFT_SSE::_mm_set1_ps(scale), t = ENFT_SSE::_mm_set1_ps(maxVal);
        m_sse0 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse0, s), t);
        m_sse1 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse1, s), t);
        m_sse2 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse2, s), t);
        m_sse3 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse3, s), t);
        m_sse4 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse4, s), t);
        m_sse5 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse5, s), t);
        m_sse6 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse6, s), t);
        m_sse7 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse7, s), t);
        m_sse8 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse8, s), t);
        m_sse9 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse9, s), t);
        m_sse10 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse10, s), t);
        m_sse11 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse11, s), t);
        m_sse12 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse12, s), t);
        m_sse13 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse13, s), t);
        m_sse14 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse14, s), t);
        m_sse15 = ENFT_SSE::_mm_min_ps(ENFT_SSE::_mm_mul_ps(m_sse15, s), t);
    }
    inline float SquaredLength() const {
        return Dot(*this, *this);
    }
    inline void Normalize() {
        const float norm = 1 / sqrt(Dot(*this, *this));
        Scale(norm);
    }
    inline void Normalize(const float &maxVal) {
        float norm = 1 / sqrt(Dot(*this, *this));
        Scale(norm, maxVal/*0.2f*/);
        norm = 1 / sqrt(Dot(*this, *this));
        Scale(norm);
    }
    inline float GetMaximalElement() const {
        const ENFT_SSE::__m128 m0123 = ENFT_SSE::_mm_max_ps(ENFT_SSE::_mm_max_ps(ENFT_SSE::_mm_max_ps(ENFT_SSE::_mm_max_ps(m_sse0,
                                        m_sse1), ENFT_SSE::_mm_max_ps(m_sse2, m_sse3)),
                                        ENFT_SSE::_mm_max_ps(ENFT_SSE::_mm_max_ps(m_sse4, m_sse5), ENFT_SSE::_mm_max_ps(m_sse6, m_sse7))),
                                        ENFT_SSE::_mm_max_ps(ENFT_SSE::_mm_max_ps(ENFT_SSE::_mm_max_ps(m_sse8, m_sse9), ENFT_SSE::_mm_max_ps(m_sse10, m_sse11)),
                                                ENFT_SSE::_mm_max_ps(ENFT_SSE::_mm_max_ps(m_sse12, m_sse13), ENFT_SSE::_mm_max_ps(m_sse14, m_sse15))));
        const float m01 = std::max(m0123.m128_f32[0], m0123.m128_f32[1]);
        const float m23 = std::max(m0123.m128_f32[2], m0123.m128_f32[3]);
        return std::max(m01, m23);
    }
    inline void ConvertTo(Descriptor128ub &desc) const {
        const ENFT_SSE::__m128 s = ENFT_SSE::_mm_set1_ps(USHRT_MAX);
        ENFT_SSE::__m128 src;
        ushort *dst = (ushort *) &desc;
        src=ENFT_SSE::_mm_mul_ps(m_sse0, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse1, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse2, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse3, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse4, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse5, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse6, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse7, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse8, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse9, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse10, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse11, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse12, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse13, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse14, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
        dst+=4;
        src=ENFT_SSE::_mm_mul_ps(m_sse15, s);
        dst[0]=ushort(src.m128_f32[0]);
        dst[1]=ushort(src.m128_f32[1]);
        dst[2]=ushort(src.m128_f32[2]);
        dst[3] = ushort(src.m128_f32[3]);
    }
    inline void ConvertFrom(const Descriptor128ub &desc) {
        const ENFT_SSE::__m128 s = ENFT_SSE::_mm_set1_ps(1.0f / USHRT_MAX);
        const ushort *src = (const ushort *) &desc;
        m_sse0 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                        float(src[3])), s);
        src+=4;
        m_sse1 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                        float(src[3])), s);
        src+=4;
        m_sse2 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                        float(src[3])), s);
        src+=4;
        m_sse3 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                        float(src[3])), s);
        src+=4;
        m_sse4 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                        float(src[3])), s);
        src+=4;
        m_sse5 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                        float(src[3])), s);
        src+=4;
        m_sse6 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                        float(src[3])), s);
        src+=4;
        m_sse7 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                        float(src[3])), s);
        src+=4;
        m_sse8 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                        float(src[3])), s);
        src+=4;
        m_sse9 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                        float(src[3])), s);
        src+=4;
        m_sse10 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                         float(src[3])), s);
        src+=4;
        m_sse11 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                         float(src[3])), s);
        src+=4;
        m_sse12 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                         float(src[3])), s);
        src+=4;
        m_sse13 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                         float(src[3])), s);
        src+=4;
        m_sse14 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                         float(src[3])), s);
        src+=4;
        m_sse15 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(float(src[0]), float(src[1]), float(src[2]),
                                         float(src[3])), s);
        src+=4;
        Normalize();
    }
    inline void Print() const {
        ENFT_SSE::SSE::Print(m_sse0);
        ENFT_SSE::SSE::Print(m_sse1);
        ENFT_SSE::SSE::Print(m_sse2);
        ENFT_SSE::SSE::Print(m_sse3);
        ENFT_SSE::SSE::Print(m_sse4);
        ENFT_SSE::SSE::Print(m_sse5);
        ENFT_SSE::SSE::Print(m_sse6);
        ENFT_SSE::SSE::Print(m_sse7);
        ENFT_SSE::SSE::Print(m_sse8);
        ENFT_SSE::SSE::Print(m_sse9);
        ENFT_SSE::SSE::Print(m_sse10);
        ENFT_SSE::SSE::Print(m_sse11);
        ENFT_SSE::SSE::Print(m_sse12);
        ENFT_SSE::SSE::Print(m_sse13);
        ENFT_SSE::SSE::Print(m_sse14);
        ENFT_SSE::SSE::Print(m_sse15);
    }
  private:
    ENFT_SSE::__m128 m_sse0, m_sse1, m_sse2, m_sse3, m_sse4, m_sse5, m_sse6, m_sse7, m_sse8,
           m_sse9, m_sse10, m_sse11, m_sse12, m_sse13, m_sse14, m_sse15;
  public:

    static inline void ApB(const Descriptor64f &A, const Descriptor64f &B,
                           Descriptor64f &ApB) {
        ApB.m_sse0 = ENFT_SSE::_mm_add_ps(A.m_sse0, B.m_sse0);
        ApB.m_sse1 = ENFT_SSE::_mm_add_ps(A.m_sse1, B.m_sse1);
        ApB.m_sse2 = ENFT_SSE::_mm_add_ps(A.m_sse2, B.m_sse2);
        ApB.m_sse3 = ENFT_SSE::_mm_add_ps(A.m_sse3, B.m_sse3);
        ApB.m_sse4 = ENFT_SSE::_mm_add_ps(A.m_sse4, B.m_sse4);
        ApB.m_sse5 = ENFT_SSE::_mm_add_ps(A.m_sse5, B.m_sse5);
        ApB.m_sse6 = ENFT_SSE::_mm_add_ps(A.m_sse6, B.m_sse6);
        ApB.m_sse7 = ENFT_SSE::_mm_add_ps(A.m_sse7, B.m_sse7);
        ApB.m_sse8 = ENFT_SSE::_mm_add_ps(A.m_sse8, B.m_sse8);
        ApB.m_sse9 = ENFT_SSE::_mm_add_ps(A.m_sse9, B.m_sse9);
        ApB.m_sse10 = ENFT_SSE::_mm_add_ps(A.m_sse10, B.m_sse10);
        ApB.m_sse11 = ENFT_SSE::_mm_add_ps(A.m_sse11, B.m_sse11);
        ApB.m_sse12 = ENFT_SSE::_mm_add_ps(A.m_sse12, B.m_sse12);
        ApB.m_sse13 = ENFT_SSE::_mm_add_ps(A.m_sse13, B.m_sse13);
        ApB.m_sse14 = ENFT_SSE::_mm_add_ps(A.m_sse14, B.m_sse14);
        ApB.m_sse15 = ENFT_SSE::_mm_add_ps(A.m_sse15, B.m_sse15);
    }
    static inline void AmB(const Descriptor64f &A, const Descriptor64f &B,
                           Descriptor64f &AmB) {
        AmB.m_sse0 = ENFT_SSE::_mm_sub_ps(A.m_sse0, B.m_sse0);
        AmB.m_sse1 = ENFT_SSE::_mm_sub_ps(A.m_sse1, B.m_sse1);
        AmB.m_sse2 = ENFT_SSE::_mm_sub_ps(A.m_sse2, B.m_sse2);
        AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
        AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
        AmB.m_sse5 = ENFT_SSE::_mm_sub_ps(A.m_sse5, B.m_sse5);
        AmB.m_sse6 = ENFT_SSE::_mm_sub_ps(A.m_sse6, B.m_sse6);
        AmB.m_sse7 = ENFT_SSE::_mm_sub_ps(A.m_sse7, B.m_sse7);
        AmB.m_sse8 = ENFT_SSE::_mm_sub_ps(A.m_sse8, B.m_sse8);
        AmB.m_sse9 = ENFT_SSE::_mm_sub_ps(A.m_sse9, B.m_sse9);
        AmB.m_sse10 = ENFT_SSE::_mm_sub_ps(A.m_sse10, B.m_sse10);
        AmB.m_sse11 = ENFT_SSE::_mm_sub_ps(A.m_sse11, B.m_sse11);
        AmB.m_sse12 = ENFT_SSE::_mm_sub_ps(A.m_sse12, B.m_sse12);
        AmB.m_sse13 = ENFT_SSE::_mm_sub_ps(A.m_sse13, B.m_sse13);
        AmB.m_sse14 = ENFT_SSE::_mm_sub_ps(A.m_sse14, B.m_sse14);
        AmB.m_sse15 = ENFT_SSE::_mm_sub_ps(A.m_sse15, B.m_sse15);
    }
    static inline void sA(const float &s, const Descriptor64f &A,
                          Descriptor64f &sA) {
        const ENFT_SSE::__m128 s0123 = ENFT_SSE::_mm_set1_ps(s);
        sA.m_sse0 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse0);
        sA.m_sse1 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse1);
        sA.m_sse2 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse2);
        sA.m_sse3 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse3);
        sA.m_sse4 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse4);
        sA.m_sse5 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse5);
        sA.m_sse6 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse6);
        sA.m_sse7 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse7);
        sA.m_sse8 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse8);
        sA.m_sse9 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse9);
        sA.m_sse10 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse10);
        sA.m_sse11 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse11);
        sA.m_sse12 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse12);
        sA.m_sse13 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse13);
        sA.m_sse14 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse14);
        sA.m_sse15 = ENFT_SSE::_mm_mul_ps(s0123, A.m_sse15);
    }
    static inline float Dot(const Descriptor64f &A, const Descriptor64f &B) {
        const ENFT_SSE::__m128 dot = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(
                                          A.m_sse0, B.m_sse0), ENFT_SSE::_mm_mul_ps(A.m_sse1, B.m_sse1)),
                                      ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.m_sse2, B.m_sse2), ENFT_SSE::_mm_mul_ps(A.m_sse3, B.m_sse3))),
                                      ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.m_sse4, B.m_sse4), ENFT_SSE::_mm_mul_ps(A.m_sse5,
                                              B.m_sse5)),
                                              ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.m_sse6, B.m_sse6), ENFT_SSE::_mm_mul_ps(A.m_sse7, B.m_sse7)))),
                                      ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.m_sse8, B.m_sse8),
                                              ENFT_SSE::_mm_mul_ps(A.m_sse9, B.m_sse9)),
                                              ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.m_sse10, B.m_sse10), ENFT_SSE::_mm_mul_ps(A.m_sse11, B.m_sse11))),
                                              ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.m_sse12, B.m_sse12), ENFT_SSE::_mm_mul_ps(A.m_sse13,
                                                      B.m_sse13)),
                                                      ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.m_sse14, B.m_sse14), ENFT_SSE::_mm_mul_ps(A.m_sse15,
                                                              B.m_sse15)))));
        return ENFT_SSE::SSE::Sum0123(dot);
    }
    static inline float SquaredDistance(const Descriptor64f &A,
                                        const Descriptor64f &B) {
//#if _DEBUG
//      assert(Dot(A, A) == 1 && Dot(B, B) == 1);
//#endif
        //const float dot = Dot(A, B);
        //return 2 - (dot + dot);
        ENFT_SSE::__m128 AmB, distSq = ENFT_SSE::_mm_setzero_ps();
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse0, B.m_sse0);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse1, B.m_sse1);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse2, B.m_sse2);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse5, B.m_sse5);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse6, B.m_sse6);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse7, B.m_sse7);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse8, B.m_sse8);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse9, B.m_sse9);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse10, B.m_sse10);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse11, B.m_sse11);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse12, B.m_sse12);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse13, B.m_sse13);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse14, B.m_sse14);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        AmB = ENFT_SSE::_mm_sub_ps(A.m_sse15, B.m_sse15);
        distSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(AmB, AmB), distSq);
        return ENFT_SSE::SSE::Sum0123(distSq);

    }
    static inline void BoundingBox(const AlignedVector<Descriptor> &descs,
                                   Descriptor64f &bbLo, Descriptor64f &bbHi) {
        bbLo = descs[0];
        bbHi = descs[0];
        const uint nDescs = descs.Size();
        for(uint i = 1; i < nDescs; ++i) {
            const Descriptor64f &desc = descs[i];
            bbLo.m_sse0 = ENFT_SSE::_mm_min_ps(bbLo.m_sse0, desc.m_sse0);
            bbHi.m_sse0 = ENFT_SSE::_mm_max_ps(bbHi.m_sse0, desc.m_sse0);
            bbLo.m_sse1 = ENFT_SSE::_mm_min_ps(bbLo.m_sse1, desc.m_sse1);
            bbHi.m_sse1 = ENFT_SSE::_mm_max_ps(bbHi.m_sse1, desc.m_sse1);
            bbLo.m_sse2 = ENFT_SSE::_mm_min_ps(bbLo.m_sse2, desc.m_sse2);
            bbHi.m_sse2 = ENFT_SSE::_mm_max_ps(bbHi.m_sse2, desc.m_sse2);
            bbLo.m_sse3 = ENFT_SSE::_mm_min_ps(bbLo.m_sse3, desc.m_sse3);
            bbHi.m_sse3 = ENFT_SSE::_mm_max_ps(bbHi.m_sse3, desc.m_sse3);
            bbLo.m_sse4 = ENFT_SSE::_mm_min_ps(bbLo.m_sse4, desc.m_sse4);
            bbHi.m_sse4 = ENFT_SSE::_mm_max_ps(bbHi.m_sse4, desc.m_sse4);
            bbLo.m_sse5 = ENFT_SSE::_mm_min_ps(bbLo.m_sse5, desc.m_sse5);
            bbHi.m_sse5 = ENFT_SSE::_mm_max_ps(bbHi.m_sse5, desc.m_sse5);
            bbLo.m_sse6 = ENFT_SSE::_mm_min_ps(bbLo.m_sse6, desc.m_sse6);
            bbHi.m_sse6 = ENFT_SSE::_mm_max_ps(bbHi.m_sse6, desc.m_sse6);
            bbLo.m_sse7 = ENFT_SSE::_mm_min_ps(bbLo.m_sse7, desc.m_sse7);
            bbHi.m_sse7 = ENFT_SSE::_mm_max_ps(bbHi.m_sse7, desc.m_sse7);
            bbLo.m_sse8 = ENFT_SSE::_mm_min_ps(bbLo.m_sse8, desc.m_sse8);
            bbHi.m_sse8 = ENFT_SSE::_mm_max_ps(bbHi.m_sse8, desc.m_sse8);
            bbLo.m_sse9 = ENFT_SSE::_mm_min_ps(bbLo.m_sse9, desc.m_sse9);
            bbHi.m_sse9 = ENFT_SSE::_mm_max_ps(bbHi.m_sse9, desc.m_sse9);
            bbLo.m_sse10 = ENFT_SSE::_mm_min_ps(bbLo.m_sse10, desc.m_sse10);
            bbHi.m_sse10 = ENFT_SSE::_mm_max_ps(bbHi.m_sse10, desc.m_sse10);
            bbLo.m_sse11 = ENFT_SSE::_mm_min_ps(bbLo.m_sse11, desc.m_sse11);
            bbHi.m_sse11 = ENFT_SSE::_mm_max_ps(bbHi.m_sse11, desc.m_sse11);
            bbLo.m_sse12 = ENFT_SSE::_mm_min_ps(bbLo.m_sse12, desc.m_sse12);
            bbHi.m_sse12 = ENFT_SSE::_mm_max_ps(bbHi.m_sse12, desc.m_sse12);
            bbLo.m_sse13 = ENFT_SSE::_mm_min_ps(bbLo.m_sse13, desc.m_sse13);
            bbHi.m_sse13 = ENFT_SSE::_mm_max_ps(bbHi.m_sse13, desc.m_sse13);
            bbLo.m_sse14 = ENFT_SSE::_mm_min_ps(bbLo.m_sse14, desc.m_sse14);
            bbHi.m_sse14 = ENFT_SSE::_mm_max_ps(bbHi.m_sse14, desc.m_sse14);
            bbLo.m_sse15 = ENFT_SSE::_mm_min_ps(bbLo.m_sse15, desc.m_sse15);
            bbHi.m_sse15 = ENFT_SSE::_mm_max_ps(bbHi.m_sse15, desc.m_sse15);
        }
    }
    static inline void BoundingBox(const AlignedVector<Descriptor> &descs,
                                   const uint nDescs, const uint *iDescs, Descriptor64f &bbLo,
                                   Descriptor64f &bbHi) {
        bbLo = descs[iDescs[0]];
        bbHi = descs[iDescs[0]];
        uint iDesc;
        for(uint i = 1; i < nDescs; ++i) {
            iDesc = iDescs[i];
            const Descriptor64f &desc = descs[iDesc];
            bbLo.m_sse0 = ENFT_SSE::_mm_min_ps(bbLo.m_sse0, desc.m_sse0);
            bbHi.m_sse0 = ENFT_SSE::_mm_max_ps(bbHi.m_sse0, desc.m_sse0);
            bbLo.m_sse1 = ENFT_SSE::_mm_min_ps(bbLo.m_sse1, desc.m_sse1);
            bbHi.m_sse1 = ENFT_SSE::_mm_max_ps(bbHi.m_sse1, desc.m_sse1);
            bbLo.m_sse2 = ENFT_SSE::_mm_min_ps(bbLo.m_sse2, desc.m_sse2);
            bbHi.m_sse2 = ENFT_SSE::_mm_max_ps(bbHi.m_sse2, desc.m_sse2);
            bbLo.m_sse3 = ENFT_SSE::_mm_min_ps(bbLo.m_sse3, desc.m_sse3);
            bbHi.m_sse3 = ENFT_SSE::_mm_max_ps(bbHi.m_sse3, desc.m_sse3);
            bbLo.m_sse4 = ENFT_SSE::_mm_min_ps(bbLo.m_sse4, desc.m_sse4);
            bbHi.m_sse4 = ENFT_SSE::_mm_max_ps(bbHi.m_sse4, desc.m_sse4);
            bbLo.m_sse5 = ENFT_SSE::_mm_min_ps(bbLo.m_sse5, desc.m_sse5);
            bbHi.m_sse5 = ENFT_SSE::_mm_max_ps(bbHi.m_sse5, desc.m_sse5);
            bbLo.m_sse6 = ENFT_SSE::_mm_min_ps(bbLo.m_sse6, desc.m_sse6);
            bbHi.m_sse6 = ENFT_SSE::_mm_max_ps(bbHi.m_sse6, desc.m_sse6);
            bbLo.m_sse7 = ENFT_SSE::_mm_min_ps(bbLo.m_sse7, desc.m_sse7);
            bbHi.m_sse7 = ENFT_SSE::_mm_max_ps(bbHi.m_sse7, desc.m_sse7);
            bbLo.m_sse8 = ENFT_SSE::_mm_min_ps(bbLo.m_sse8, desc.m_sse8);
            bbHi.m_sse8 = ENFT_SSE::_mm_max_ps(bbHi.m_sse8, desc.m_sse8);
            bbLo.m_sse9 = ENFT_SSE::_mm_min_ps(bbLo.m_sse9, desc.m_sse9);
            bbHi.m_sse9 = ENFT_SSE::_mm_max_ps(bbHi.m_sse9, desc.m_sse9);
            bbLo.m_sse10 = ENFT_SSE::_mm_min_ps(bbLo.m_sse10, desc.m_sse10);
            bbHi.m_sse10 = ENFT_SSE::_mm_max_ps(bbHi.m_sse10, desc.m_sse10);
            bbLo.m_sse11 = ENFT_SSE::_mm_min_ps(bbLo.m_sse11, desc.m_sse11);
            bbHi.m_sse11 = ENFT_SSE::_mm_max_ps(bbHi.m_sse11, desc.m_sse11);
            bbLo.m_sse12 = ENFT_SSE::_mm_min_ps(bbLo.m_sse12, desc.m_sse12);
            bbHi.m_sse12 = ENFT_SSE::_mm_max_ps(bbHi.m_sse12, desc.m_sse12);
            bbLo.m_sse13 = ENFT_SSE::_mm_min_ps(bbLo.m_sse13, desc.m_sse13);
            bbHi.m_sse13 = ENFT_SSE::_mm_max_ps(bbHi.m_sse13, desc.m_sse13);
            bbLo.m_sse14 = ENFT_SSE::_mm_min_ps(bbLo.m_sse14, desc.m_sse14);
            bbHi.m_sse14 = ENFT_SSE::_mm_max_ps(bbHi.m_sse14, desc.m_sse14);
            bbLo.m_sse15 = ENFT_SSE::_mm_min_ps(bbLo.m_sse15, desc.m_sse15);
            bbHi.m_sse15 = ENFT_SSE::_mm_max_ps(bbHi.m_sse15, desc.m_sse15);
        }
    }
    static inline ubyte GetMaximalElementDimension(const Descriptor &desc) {
        const float maxVal = desc.GetMaximalElement();
        for(ubyte d = 0; d < DESCRIPTOR_DIMENSION; ++d) {
            if(desc[d] == maxVal)
                return d;
        }
        return DESCRIPTOR_DIMENSION;
    }

    static inline ubyte GetDimension() {
        return DESCRIPTOR_DIMENSION;
    }
};

#endif