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

#ifndef _MATRIX_2x3_H_
#define _MATRIX_2x3_H_

#include "Matrix2.h"
#include "Vector3.h"
#include "Utility/SSE.h"

namespace LA {

class AlignedMatrix2x3f {

  public:

    inline const ENFT_SSE::__m128 &M_00_01_02_x() const {
        return m_00_01_02_x;
    }            inline ENFT_SSE::__m128 &M_00_01_02_x() {
        return m_00_01_02_x;
    }
    inline const ENFT_SSE::__m128 &M_10_11_12_x() const {
        return m_10_11_12_x;
    }            inline ENFT_SSE::__m128 &M_10_11_12_x() {
        return m_10_11_12_x;
    }
    inline const float &M00()      const {
        return m_00_01_02_x.m128_f32[0];
    }   inline float &M00()      {
        return m_00_01_02_x.m128_f32[0];
    }
    inline const float &M01()      const {
        return m_00_01_02_x.m128_f32[1];
    }   inline float &M01()      {
        return m_00_01_02_x.m128_f32[1];
    }
    inline const float &M02()      const {
        return m_00_01_02_x.m128_f32[2];
    }   inline float &M02()      {
        return m_00_01_02_x.m128_f32[2];
    }
    inline const float &M10()      const {
        return m_10_11_12_x.m128_f32[0];
    }   inline float &M10()      {
        return m_10_11_12_x.m128_f32[0];
    }
    inline const float &M11()      const {
        return m_10_11_12_x.m128_f32[1];
    }   inline float &M11()      {
        return m_10_11_12_x.m128_f32[1];
    }
    inline const float &M12()      const {
        return m_10_11_12_x.m128_f32[2];
    }   inline float &M12()      {
        return m_10_11_12_x.m128_f32[2];
    }
    inline const float &reserve0() const {
        return m_00_01_02_x.m128_f32[3];
    }   inline float &reserve0() {
        return m_00_01_02_x.m128_f32[3];
    }
    inline const float &reserve1() const {
        return m_10_11_12_x.m128_f32[3];
    }   inline float &reserve1() {
        return m_10_11_12_x.m128_f32[3];
    }
    inline operator const float *() const {
        return (const float *) this;
    }
    inline operator       float *()       {
        return (      float *) this;
    }
    inline void SetZero() {
        memset(this, 0, sizeof(AlignedMatrix2x3f));
    }
    inline void Set(const float &M00, const float &M01, const float &M02, const float &M10, const float &M11, const float &M12) {
        m_00_01_02_x = ENFT_SSE::_mm_setr_ps(M00, M01, M02, 0);
        m_10_11_12_x = ENFT_SSE::_mm_setr_ps(M10, M11, M12, 0);
    }
    inline void Set(const float *M) {
        memcpy(&M00(), M, 12);
        memcpy(&M10(), M + 3, 12);
    }
    inline void Get(float *M) const {
        memcpy(M, &M00(), 12);
        memcpy(M + 3, &M10(), 12);
    }
    inline void Scale(const ENFT_SSE::__m128 &s) {
        m_00_01_02_x = ENFT_SSE::_mm_mul_ps(s, m_00_01_02_x);
        m_10_11_12_x = ENFT_SSE::_mm_mul_ps(s, m_10_11_12_x);
    }
    //inline void Scale(const LA::AlignedVector3f &s) { m_00_01_02_x = ENFT_SSE::_mm_mul_ps(m_00_01_02_x, s.v012x()); m_10_11_12_x = ENFT_SSE::_mm_mul_ps(m_10_11_12_x, s.v012x()); }
    inline void Print() const {
        printf("%f %f %f\n", M00(), M01(), M02());
        printf("%f %f %f\n", M10(), M11(), M12());
    }

  protected:

    ENFT_SSE::__m128 m_00_01_02_x, m_10_11_12_x;

};

inline void AddAij2To(const AlignedMatrix2x3f &A, AlignedVector3f &to) {
    to.v012x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_00_01_02_x()), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(),
                                      A.M_10_11_12_x())), to.v012x());
}
inline void AddAij2To(const AlignedMatrix2x3f &A, Vector3f &to, ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_00_01_02_x()), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_10_11_12_x()));
    to.v0() = work.m128_f32[0] + to.v0();
    to.v1() = work.m128_f32[1] + to.v1();
    to.v2() = work.m128_f32[2] + to.v2();
}

inline void sA(const AlignedVector3f &s, AlignedMatrix2x3f &A) {
    A.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(s.v012x(), A.M_00_01_02_x());
    A.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(s.v012x(), A.M_10_11_12_x());
}
inline void s1s2TA(const Vector2f &s1, const Vector3f &s2, AlignedMatrix2x3f &A, ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_setr_ps(s2.v0(), s2.v1(), s2.v2(), 1.0f);
    A.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v0()), work), A.M_00_01_02_x());
    A.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v1()), work), A.M_10_11_12_x());
}
inline void AddATBTo(const AlignedMatrix2x3f &A, const Vector2f &B, AlignedVector3f &to) {
    to.v012x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), ENFT_SSE::_mm_set1_ps(B.v0())),
                                      ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), ENFT_SSE::_mm_set1_ps(B.v1()))), to.v012x());
}
inline void AddATBTo(const AlignedMatrix2x3f &A, const Vector2f &B, Vector3f &to, ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), ENFT_SSE::_mm_set1_ps(B.v0())), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(),
                                ENFT_SSE::_mm_set1_ps(B.v1())));
    to.v0() = work.m128_f32[0] + to.v0();
    to.v1() = work.m128_f32[1] + to.v1();
    to.v2() = work.m128_f32[2] + to.v2();
}
inline void AddATBTo(const LA::AlignedMatrix2f &A, const LA::AlignedMatrix2x3f &B, LA::AlignedMatrix2x3f &to) {
    to.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B.M_00_01_02_x()),
                        ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_10_11_12_x())), to.M_00_01_02_x());
    to.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B.M_00_01_02_x()),
                        ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_x())), to.M_10_11_12_x());
}
inline void AB(const AlignedMatrix2x3f &A, const AlignedVector3f &B, Vector2f &AB) {
    AB.v0() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B.v012x()));
    AB.v1() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B.v012x()));
}
inline void SubtractABTFrom(const AlignedMatrix2x3f &A, const AlignedMatrix2x3f &B, AlignedMatrix2f &from, ENFT_SSE::__m128 *work0) {
    from.M00() -= ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B.M_00_01_02_x()));
    from.M01() -= ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B.M_10_11_12_x()));
    from.M10() -= ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B.M_00_01_02_x()));
    from.M11() -= ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B.M_10_11_12_x()));
}
inline void SubtractATBFrom(const AlignedMatrix2x3f &A, const Vector2f &B, Vector3f &from, ENFT_SSE::__m128 *work1) {
    work1[0] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), ENFT_SSE::_mm_set1_ps(B.v0())), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(),
                                    ENFT_SSE::_mm_set1_ps(B.v1())));
    from.v0() -= work1[0].m128_f32[0];
    from.v1() -= work1[0].m128_f32[1];
    from.v2() -= work1[0].m128_f32[2];
}
inline void SubtractABFrom(const AlignedMatrix2x3f &A, const Vector3f &B, Vector2f &from, ENFT_SSE::__m128 *work1) {
    work1[0] = ENFT_SSE::_mm_setr_ps(B.v0(), B.v1(), B.v2(), 0.0f);
    from.v0() -= ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work1[0]));
    from.v1() -= ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work1[0]));
}

template<typename TYPE> class Matrix2x3 {

  public:

    inline const TYPE &M00() const {
        return m_data[0];
    }    inline TYPE &M00() {
        return m_data[0];
    }
    inline const TYPE &M01() const {
        return m_data[1];
    }    inline TYPE &M01() {
        return m_data[1];
    }
    inline const TYPE &M02() const {
        return m_data[2];
    }    inline TYPE &M02() {
        return m_data[2];
    }
    inline const TYPE &M10() const {
        return m_data[3];
    }    inline TYPE &M10() {
        return m_data[3];
    }
    inline const TYPE &M11() const {
        return m_data[4];
    }    inline TYPE &M11() {
        return m_data[4];
    }
    inline const TYPE &M12() const {
        return m_data[5];
    }    inline TYPE &M12() {
        return m_data[5];
    }

    inline void Print() const {
        printf("%f %f %f\n", M00(), M01(), M02());
        printf("%f %f %f\n", M10(), M11(), M12());
    }

  private:

    TYPE m_data[6];

};

typedef Matrix2x3<float > Matrix2x3f;
typedef Matrix2x3<double> Matrix2x3d;

template<class TYPE> inline void AddAij2To(const Matrix2x3<TYPE> &A, Vector3<TYPE> &to) {
    to.v0() = A.M00() * A.M00() + A.M10() * A.M10() + to.v0();
    to.v1() = A.M01() * A.M01() + A.M11() * A.M11() + to.v1();
    to.v2() = A.M02() * A.M02() + A.M12() * A.M12() + to.v2();
}

template<class TYPE> inline void sA(const Vector3<TYPE> &s, Matrix2x3<TYPE> &A) {
    A.M00() *= s.v0();
    A.M01() *= s.v1();
    A.M02() *= s.v2();
    A.M10() *= s.v0();
    A.M11() *= s.v1();
    A.M12() *= s.v2();
}
template<class TYPE> inline void AddATBTo(const Matrix2x3<TYPE> &A, const Vector2<TYPE> &B, Vector3<TYPE> &to) {
    to.v0() = A.M00() * B.v0() + A.M10() * B.v1() + to.v0();
    to.v1() = A.M01() * B.v0() + A.M11() * B.v1() + to.v1();
    to.v2() = A.M02() * B.v0() + A.M12() * B.v1() + to.v2();
}

template<class TYPE> inline void AB(const Matrix2x3<TYPE> &A, const Vector3<TYPE> &B, Vector2<TYPE> &AB) {
    AB.v0() = A.M00() * B.v0() + A.M01() * B.v1() + A.M02() * B.v2();
    AB.v1() = A.M10() * B.v0() + A.M11() * B.v1() + A.M12() * B.v2();
}
}

#endif