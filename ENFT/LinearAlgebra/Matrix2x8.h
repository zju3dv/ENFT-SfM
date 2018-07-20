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

#ifndef _MATRIX_2x8_H_
#define _MATRIX_2x8_H_

#include "Utility/SSE.h"
#include "Vector2.h"
#include "Vector8.h"

namespace LA {

class AlignedMatrix2x8f {

  public:

    inline operator const float *() const {
        return (const float *) this;
    }      inline operator float *() {
        return (float *) this;
    }
    inline const ENFT_SSE::__m128 &M_00_01_02_03() const {
        return m_00_01_02_03;
    }      inline ENFT_SSE::__m128 &M_00_01_02_03() {
        return m_00_01_02_03;
    }
    inline const ENFT_SSE::__m128 &M_04_05_06_07() const {
        return m_04_05_06_07;
    }      inline ENFT_SSE::__m128 &M_04_05_06_07() {
        return m_04_05_06_07;
    }
    inline const ENFT_SSE::__m128 &M_10_11_12_13() const {
        return m_10_11_12_13;
    }      inline ENFT_SSE::__m128 &M_10_11_12_13() {
        return m_10_11_12_13;
    }
    inline const ENFT_SSE::__m128 &M_14_15_16_17() const {
        return m_14_15_16_17;
    }      inline ENFT_SSE::__m128 &M_14_15_16_17() {
        return m_14_15_16_17;
    }
    inline const float &M00() const {
        return m_00_01_02_03.m128_f32[0];
    }       inline float &M00() {
        return m_00_01_02_03.m128_f32[0];
    }
    inline const float &M01() const {
        return m_00_01_02_03.m128_f32[1];
    }       inline float &M01() {
        return m_00_01_02_03.m128_f32[1];
    }
    inline const float &M02() const {
        return m_00_01_02_03.m128_f32[2];
    }       inline float &M02() {
        return m_00_01_02_03.m128_f32[2];
    }
    inline const float &M03() const {
        return m_00_01_02_03.m128_f32[3];
    }       inline float &M03() {
        return m_00_01_02_03.m128_f32[3];
    }
    inline const float &M04() const {
        return m_04_05_06_07.m128_f32[0];
    }       inline float &M04() {
        return m_04_05_06_07.m128_f32[0];
    }
    inline const float &M05() const {
        return m_04_05_06_07.m128_f32[1];
    }       inline float &M05() {
        return m_04_05_06_07.m128_f32[1];
    }
    inline const float &M06() const {
        return m_04_05_06_07.m128_f32[2];
    }       inline float &M06() {
        return m_04_05_06_07.m128_f32[2];
    }
    inline const float &M07() const {
        return m_04_05_06_07.m128_f32[3];
    }       inline float &M07() {
        return m_04_05_06_07.m128_f32[3];
    }
    inline const float &M10() const {
        return m_10_11_12_13.m128_f32[0];
    }       inline float &M10() {
        return m_10_11_12_13.m128_f32[0];
    }
    inline const float &M11() const {
        return m_10_11_12_13.m128_f32[1];
    }       inline float &M11() {
        return m_10_11_12_13.m128_f32[1];
    }
    inline const float &M12() const {
        return m_10_11_12_13.m128_f32[2];
    }       inline float &M12() {
        return m_10_11_12_13.m128_f32[2];
    }
    inline const float &M13() const {
        return m_10_11_12_13.m128_f32[3];
    }       inline float &M13() {
        return m_10_11_12_13.m128_f32[3];
    }
    inline const float &M14() const {
        return m_14_15_16_17.m128_f32[0];
    }       inline float &M14() {
        return m_14_15_16_17.m128_f32[0];
    }
    inline const float &M15() const {
        return m_14_15_16_17.m128_f32[1];
    }       inline float &M15() {
        return m_14_15_16_17.m128_f32[1];
    }
    inline const float &M16() const {
        return m_14_15_16_17.m128_f32[2];
    }       inline float &M16() {
        return m_14_15_16_17.m128_f32[2];
    }
    inline const float &M17() const {
        return m_14_15_16_17.m128_f32[3];
    }       inline float &M17() {
        return m_14_15_16_17.m128_f32[3];
    }
    inline void Print() const {
        printf("%f %f %f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05(), M06(), M07());
        printf("%f %f %f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15(), M16(), M17());
    }

  protected:

    ENFT_SSE::__m128 m_00_01_02_03, m_04_05_06_07;
    ENFT_SSE::__m128 m_10_11_12_13, m_14_15_16_17;

};

inline void AddAij2To(const AlignedMatrix2x8f &A, AlignedVector8f &to) {
    to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(),
                                      A.M_10_11_12_13())), to.v0123());
    to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), A.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(),
                                      A.M_14_15_16_17())), to.v4567());
}
inline void AddAij2To(const AlignedMatrix2x8f &A, AlignedVector8f &to, ENFT_SSE::__m128 *work0) {
    AddAij2To(A, to);
}
inline void sA(const AlignedVector8f &s, AlignedMatrix2x8f &A) {
    A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.M_00_01_02_03());
    A.M_04_05_06_07() = ENFT_SSE::_mm_mul_ps(s.v4567(), A.M_04_05_06_07());
    A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.M_10_11_12_13());
    A.M_14_15_16_17() = ENFT_SSE::_mm_mul_ps(s.v4567(), A.M_14_15_16_17());
}
inline void AddATBTo(const LA::AlignedMatrix2x8f &A, const Vector2f &B, LA::AlignedVector8f &to, ENFT_SSE::__m128 *work2) {
    work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());
    work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
    to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])),
                                      to.v0123());
    to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), work2[1])),
                                      to.v4567());
}
inline void AB(const AlignedMatrix2x8f &A, const AlignedVector8f &B, Vector2f &AB) {
    AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), B.v4567())));
    AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), B.v4567())));
}
inline void AB(const AlignedMatrix2x8f &A, const AlignedVector8f &B, Vector2f &AB, ENFT_SSE::__m128 *work0) {
    LA::AB(A, B, AB);
}
inline void ABpCD(const AlignedMatrix2x8f &A, const AlignedVector8f &B, const Matrix2x3f &C, const Vector3f &D, Vector2f &ABpCD) {
    ABpCD.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(),
                                        B.v4567()))) + C.M00() * D.v0() + C.M01() * D.v1() + C.M02() * D.v2();
    ABpCD.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(),
                                        B.v4567()))) + C.M10() * D.v0() + C.M11() * D.v1() + C.M12() * D.v2();
}
inline void ABpCD(const AlignedMatrix2x8f &A, const AlignedVector8f &B, const Matrix2x3f &C, const Vector3f &D, Vector2f &ABpCD, ENFT_SSE::__m128 *work0) {
    LA::ABpCD(A, B, C, D, ABpCD);
}
inline void AddABTo(const AlignedMatrix2x8f &A, const AlignedVector8f &B, Vector2f &to, ENFT_SSE::__m128 *work0) {
    to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(),
                                     B.v4567()))) + to.v0();
    to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(),
                                     B.v4567()))) + to.v1();
}
}

#endif