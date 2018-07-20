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

#ifndef _MATRIX_3_H_
#define _MATRIX_3_H_

#include "Utility/SSE.h"
#include "Matrix2x3.h"
#include "Vector3.h"

namespace LA {

class AlignedMatrix3f : public AlignedMatrix2x3f {

  public:

    inline const ENFT_SSE::__m128 &M_20_21_22_x() const {
        return m_20_21_22_x;
    }              inline ENFT_SSE::__m128 &M_20_21_22_x() {
        return m_20_21_22_x;
    }
    inline const float &M20()      const {
        return m_20_21_22_x.m128_f32[0];
    }       inline float &M20()      {
        return m_20_21_22_x.m128_f32[0];
    }
    inline const float &M21()      const {
        return m_20_21_22_x.m128_f32[1];
    }       inline float &M21()      {
        return m_20_21_22_x.m128_f32[1];
    }
    inline const float &M22()      const {
        return m_20_21_22_x.m128_f32[2];
    }       inline float &M22()      {
        return m_20_21_22_x.m128_f32[2];
    }
    inline const float &reserve2() const {
        return m_20_21_22_x.m128_f32[3];
    }       inline float &reserve2() {
        return m_20_21_22_x.m128_f32[3];
    }
    inline void Set(const float &M00, const float &M01, const float &M02,
                    const float &M10, const float &M11, const float &M12,
                    const float &M20, const float &M21, const float &M22) {
        AlignedMatrix2x3f::Set(M00, M01, M02, M10, M11, M12);
        m_20_21_22_x = ENFT_SSE::_mm_setr_ps(M20, M21, M22, 0);
    }
    inline void Set(const float *M) {
        memcpy(&M00(), M, 12);
        memcpy(&M10(), M + 3, 12);
        memcpy(&M20(), M + 6, 12);
    }
    inline void Get(float *M) const {
        memcpy(M, &M00(), 12);
        memcpy(M + 3, &M10(), 12);
        memcpy(M + 6, &M20(), 12);
    }
    inline void Stride3To4() {
        //memcpy(&M20(), &M12(), 12);
        //memcpy(&M10(), &m_00_01_02_x.m128_f32[3], 12);
        M22() = M20();
        memcpy(&M20(), &M12(), 8);
        M12() = M11();
        M11() = M10();
        M10() = m_00_01_02_x.m128_f32[3];
    }
    inline void SetReserveZero() {
        reserve0() = reserve1() = reserve2() = 0;
    }
    inline void SetReserve2() {
        reserve0() = M02();
        reserve1() = M12();
        reserve2() = M22();
    }
#if _DEBUG
    inline void AssertReserve2() const {
        assert(reserve0() == M02() && reserve1() == M12() && reserve2() == M22());
    }
#endif
    inline void MakeIdentity() {
        memset(this, 0, 48);
        M00() = M11() = M22() = 1;
    }
    inline void SetZero() {
        memset(this, 0, 48);
    }
    inline void Transpose() {
        float tmp;
        SWAP(M01(), M10(), tmp);
        SWAP(M02(), M20(), tmp);
        SWAP(M12(), M21(), tmp);
    }
    inline void GetTranspose(AlignedMatrix3f &AT) const {
        AT.M00() = M00();
        AT.M01() = M10();
        AT.M02() = M20();
        AT.M10() = M01();
        AT.M11() = M11();
        AT.M12() = M21();
        AT.M20() = M02();
        AT.M21() = M12();
        AT.M22() = M22();
    }
    inline void Scale(const ENFT_SSE::__m128 &s) {
        m_00_01_02_x = ENFT_SSE::_mm_mul_ps(s, m_00_01_02_x);
        m_10_11_12_x = ENFT_SSE::_mm_mul_ps(s, m_10_11_12_x);
        m_20_21_22_x = ENFT_SSE::_mm_mul_ps(s, m_20_21_22_x);
    }
    inline float Determinant() const {
        return M00() * (M11() * M22() - M12() * M21()) +
               M01() * (M12() * M20() - M10() * M22()) +
               M02() * (M10() * M21() - M11() * M20());
    }
    inline float FrobeniusNorm() const {
        return sqrt(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_00_01_02_x, m_00_01_02_x),
                                           ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_10_11_12_x, m_10_11_12_x), ENFT_SSE::_mm_mul_ps(m_20_21_22_x,
                                                   m_20_21_22_x)))));
    }
    inline void EnforceUnitFrobeniusNorm() {
        Scale(ENFT_SSE::_mm_set1_ps(1 / FrobeniusNorm()));
    }
    inline void EnforceUnitLastEntry() {
        Scale(ENFT_SSE::_mm_set1_ps(1 / M22()));
    }
    inline void LeftMultiply(const AlignedMatrix3f &Ml, ENFT_SSE::__m128 &tmp) {
        ENFT_SSE::SSE::Set012(M00(), M10(), M20(), tmp);
        M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Ml.M_00_01_02_x(), tmp));
        M10() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Ml.M_10_11_12_x(), tmp));
        M20() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Ml.M_20_21_22_x(), tmp));
        ENFT_SSE::SSE::Set012(M01(), M11(), M21(), tmp);
        M01() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Ml.M_00_01_02_x(), tmp));
        M11() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Ml.M_10_11_12_x(), tmp));
        M21() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Ml.M_20_21_22_x(), tmp));
        ENFT_SSE::SSE::Set012(M02(), M12(), M22(), tmp);
        M02() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Ml.M_00_01_02_x(), tmp));
        M12() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Ml.M_10_11_12_x(), tmp));
        M22() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Ml.M_20_21_22_x(), tmp));
    }
    inline void SetSkewSymmetric(const AlignedVector3f &v) {
        M00() = 0;
        M01() = -v.v2();
        M02() = v.v1();
        M10() = v.v2();
        M11() = 0;
        M12() = -v.v0();
        M20() = -v.v1();
        M21() = v.v0();
        M22() = 0;
    }
    inline void Print() const {
        printf("%f %f %f\n", M00(), M01(), M02());
        printf("%f %f %f\n", M10(), M11(), M12());
        printf("%f %f %f\n", M20(), M21(), M22());
    }
    inline void Save(FILE *fp) const {
        fprintf(fp, "%f %f %f\n", M00(), M01(), M02());
        fprintf(fp, "%f %f %f\n", M10(), M11(), M12());
        fprintf(fp, "%f %f %f\n", M20(), M21(), M22());
    }
    inline void Load(FILE *fp) {
        fscanf(fp, "%f %f %f", &M00(), &M01(), &M02());
        fscanf(fp, "%f %f %f", &M10(), &M11(), &M12());
        fscanf(fp, "%f %f %f", &M20(), &M21(), &M22());
    }
    inline void SaveB(FILE *fp) const {
        fwrite(this, sizeof(AlignedMatrix3f), 1, fp);
    }
    inline void LoadB(FILE *fp) {
        fread(this, sizeof(AlignedMatrix3f), 1, fp);
    }

    inline bool Invert(AlignedMatrix3f &Minv) const {
        Minv.M00() = M11() * M22() - M12() * M21();
        Minv.M01() = M02() * M21() - M01() * M22();
        Minv.M02() = M01() * M12() - M02() * M11();
        Minv.M10() = M12() * M20() - M10() * M22();
        Minv.M11() = M00() * M22() - M02() * M20();
        Minv.M12() = M02() * M10() - M00() * M12();
        Minv.M20() = M10() * M21() - M11() * M20();
        Minv.M21() = M01() * M20() - M00() * M21();
        Minv.M22() = M00() * M11() - M01() * M10();
        const float det = M00() * Minv.M00() + M01() * Minv.M10() + M02() * Minv.M20();
        if(fabs(det) < FLT_EPSILON)
            return false;
        Minv.Scale(ENFT_SSE::_mm_set1_ps(1 / det));
        return true;
    }
    inline void GetDiagonal(AlignedVector3f &d) const {
        d.v0() = M00();
        d.v1() = M11();
        d.v2() = M22();
    }
    inline void SetDiagonal(const AlignedVector3f &d) {
        M00() = d.v0();
        M11() = d.v1();
        M22() = d.v2();
    }
    inline void ScaleDiagonal(const float &lambda) {
        M00() *= lambda;
        M11() *= lambda;
        M22() *= lambda;
    }
    inline void IncreaseDiagonal(const float &lambda) {
        M00() += lambda;
        M11() += lambda;
        M22() += lambda;
    }
    inline void SetLowerFromUpper() {
        M10() = M01();
        M20() = M02();
        M21() = M12();
    }

    inline void ScaleRow2(const float &s) {
        M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(M_20_21_22_x(), ENFT_SSE::_mm_set1_ps(s));
    }
    inline void ScaleColumn2(const float &s) {
        M02() *= s;
        M12() *= s;
        M22() *= s;
    }

  protected:

    ENFT_SSE::__m128 m_20_21_22_x;

};

inline void AddAij2To(const AlignedMatrix3f &A, Vector3f &to, ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_00_01_02_x()),
                                 ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_10_11_12_x())),
                      ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), A.M_20_21_22_x()));
    to.v0() = work.m128_f32[0] + to.v0();
    to.v1() = work.m128_f32[1] + to.v1();
    to.v2() = work.m128_f32[2] + to.v2();
}
inline void AddAij2To(const AlignedMatrix3f &A, AlignedVector3f &to) {
    to.v012x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(),
                                       A.M_00_01_02_x()), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_10_11_12_x())),
                            ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), A.M_20_21_22_x()), to.v012x()));
}
template<> inline void FinishAdditionAij2To<AlignedMatrix3f>
(AlignedVector3f &to) {
    to.reserve() = to.v2();
}

inline void AddATBTo(const AlignedMatrix3f &A, const ENFT_SSE::__m128 &B, Vector3f &to,
                     ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(),
                                            ENFT_SSE::_mm_set1_ps(B.m128_f32[0])), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(),
                                                    ENFT_SSE::_mm_set1_ps(B.m128_f32[1]))),
                      ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), ENFT_SSE::_mm_set1_ps(B.m128_f32[2])));
    to.v0() = work.m128_f32[0] + to.v0();
    to.v1() = work.m128_f32[1] + to.v1();
    to.v2() = work.m128_f32[2] + to.v2();
}
inline void AddATBTo(const AlignedMatrix3f &A, const AlignedVector3f &B,
                     Vector3f &to, ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), ENFT_SSE::_mm_set1_ps(B.v0())),
                                 ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), ENFT_SSE::_mm_set1_ps(B.v1()))),
                      ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), ENFT_SSE::_mm_set1_ps(B.v2())));
    to.v0() = work.m128_f32[0] + to.v0();
    to.v1() = work.m128_f32[1] + to.v1();
    to.v2() = work.m128_f32[2] + to.v2();
}
inline void AddATBTo(const AlignedMatrix3f &A, const Vector3f &B,
                     AlignedVector3f &to) {
    to.v012x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(),
                                       ENFT_SSE::_mm_set1_ps(B.v0())), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), ENFT_SSE::_mm_set1_ps(B.v1()))),
                            ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), ENFT_SSE::_mm_set1_ps(B.v2())), to.v012x()));
}
inline void AddATBTo(const AlignedMatrix3f &A, const AlignedVector3f &B,
                     AlignedVector3f &to) {
    to.v012x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(),
                                       ENFT_SSE::_mm_set1_ps(B.v0())), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), ENFT_SSE::_mm_set1_ps(B.v1()))),
                            ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), ENFT_SSE::_mm_set1_ps(B.v2())), to.v012x()));
}
inline void AddATAToUpper(const AlignedMatrix2x3f &A, AlignedMatrix3f &to) {
    to.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()),
                                   A.M_00_01_02_x()),
                                   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), A.M_10_11_12_x())), to.M_00_01_02_x());
    to.M11() = A.M01() * A.M01() + A.M11() * A.M11() + to.M11();
    to.M12() = A.M01() * A.M02() + A.M11() * A.M12() + to.M12();
    to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + to.M22();
}
inline void AddATAToUpper(const AlignedMatrix3f &A, AlignedMatrix3f &to) {
//#if _DEBUGd
//      A.AssertReserve2();
//#endif
//      to.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(A.M00(), A.M00(), A.M00(), A.M02()), A.M_00_01_02_x()),
//                                                ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(A.M10(), A.M10(), A.M10(), A.M12()), A.M_10_11_12_x())),
//                                     ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(A.M20(), A.M20(), A.M20(), A.M22()), A.M_20_21_22_x()),
//                                                to.M_00_01_02_x()));
//      to.M11() = A.M01() * A.M01() + A.M11() * A.M11() + A.M21() * A.M21() + to.M11();
//      to.M12() = A.M01() * A.M02() + A.M11() * A.M12() + A.M21() * A.M22() + to.M12();
    to.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()),
                                   A.M_00_01_02_x()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), A.M_10_11_12_x())),
                                   ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), A.M_20_21_22_x()),
                                           to.M_00_01_02_x()));
    to.M11() = A.M01() * A.M01() + A.M11() * A.M11() + A.M21() * A.M21() + to.M11();
    to.M12() = A.M01() * A.M02() + A.M11() * A.M12() + A.M21() * A.M22() + to.M12();
    to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + A.M22() * A.M22() + to.M22();
}
template<class MATRIX> inline void FinishAdditionATAToUpper(
    AlignedMatrix3f &to) {}
//template<> inline void FinishAdditionATAToUpper<AlignedMatrix2x3f>(AlignedMatrix3f &to) {}
//template<> inline void FinishAdditionATAToUpper<AlignedMatrix3f>(AlignedMatrix3f &to) { to.M22() = to.reserve0(); }

template<ushort STAGE> inline void SetReserve(AlignedMatrix3f &M) {}
//template<> inline void SetReserve<0>(AlignedMatrix3f &M) {}
//template<> inline void SetReserve<1>(AlignedMatrix3f &M) {}
//template<> inline void SetReserve<2>(AlignedMatrix3f &M) {}

inline void GetDiagonal(const AlignedMatrix3f &M, AlignedVector3f &d) {
    M.GetDiagonal(d);
}
inline void SetDiagonal(const AlignedVector3f &d, AlignedMatrix3f &M) {
    M.SetDiagonal(d);
}
inline void ScaleDiagonal(const float &lambda, AlignedMatrix3f &M) {
    M.ScaleDiagonal(lambda);
}
inline void IncreaseDiagonal(const float &lambda, AlignedMatrix3f &M) {
    M.IncreaseDiagonal(lambda);
}

inline void AB(const AlignedMatrix3f &A, const AlignedVector3f &B,
               AlignedVector3f &AB) {
    AB.v0() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B.v012x()));
    AB.v1() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B.v012x()));
    AB.v2() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), B.v012x()));
}
inline void nAB(const AlignedMatrix3f &A, const ENFT_SSE::__m128 &B, ENFT_SSE::__m128 &nAB) {
    nAB.m128_f32[0] = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B));
    nAB.m128_f32[1] = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B));
    nAB.m128_f32[2] = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), B));
}
inline void ABx(const AlignedMatrix3f &A, const AlignedVector3f &B,
                float &ABx00, float &ABx01, float &ABx02, float &ABx10, float &ABx11,
                float &ABx12,
                float &ABx20, float &ABx21, float &ABx22) {
    ABx00 = A.M01() * B.v2() - A.M02() * B.v1();
    ABx01 = A.M02() * B.v0() - A.M00() * B.v2();
    ABx02 = A.M00() * B.v1() - A.M01() * B.v0();
    ABx10 = A.M11() * B.v2() - A.M12() * B.v1();
    ABx11 = A.M12() * B.v0() - A.M10() * B.v2();
    ABx12 = A.M10() * B.v1() - A.M11() * B.v0();
    ABx20 = A.M21() * B.v2() - A.M22() * B.v1();
    ABx21 = A.M22() * B.v0() - A.M20() * B.v2();
    ABx22 = A.M20() * B.v1() - A.M21() * B.v0();
}
inline void ATB(const AlignedMatrix3f &A, const AlignedVector3f &B,
                AlignedVector3f &ATB) {
    ATB.v012x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), ENFT_SSE::_mm_set1_ps(B.v0())),
                             ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), ENFT_SSE::_mm_set1_ps(B.v1())),
                                        ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), ENFT_SSE::_mm_set1_ps(B.v2()))));
}
inline void AB(const AlignedMatrix3f &A, const AlignedMatrix3f &B,
               AlignedMatrix3f &AB, ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_setr_ps(B.M00(), B.M10(), B.M20(), 0);
    AB.M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work));
    AB.M10() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work));
    AB.M20() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), work));
    work = ENFT_SSE::_mm_setr_ps(B.M01(), B.M11(), B.M21(), 0);
    AB.M01() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work));
    AB.M11() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work));
    AB.M21() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), work));
    work = ENFT_SSE::_mm_setr_ps(B.M02(), B.M12(), B.M22(), 0);
    AB.M02() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work));
    AB.M12() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work));
    AB.M22() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), work));
}
// A and AB are allowed to be the same
inline void AB(const AlignedMatrix3f &A, const AlignedMatrix3f &B,
               AlignedMatrix3f &AB, ENFT_SSE::__m128 *work3) {
    work3[0] = ENFT_SSE::_mm_setr_ps(B.M00(), B.M10(), B.M20(), 0);
    work3[1] = ENFT_SSE::_mm_setr_ps(B.M01(), B.M11(), B.M21(), 0);
    work3[2] = ENFT_SSE::_mm_setr_ps(B.M02(), B.M12(), B.M22(), 0);
    ENFT_SSE::SSE::Set012(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work3[0])),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work3[1])),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work3[2])), AB.M_00_01_02_x());
    ENFT_SSE::SSE::Set012(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work3[0])),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work3[1])),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work3[2])), AB.M_10_11_12_x());
    ENFT_SSE::SSE::Set012(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), work3[0])),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), work3[1])),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), work3[2])), AB.M_20_21_22_x());
}
inline void AB(const AlignedMatrix2x3f &A, const AlignedMatrix3f &B,
               AlignedMatrix2x3f &AB, ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_setr_ps(B.M00(), B.M10(), B.M20(), 0);
    AB.M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work));
    AB.M10() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work));

    work = ENFT_SSE::_mm_setr_ps(B.M01(), B.M11(), B.M21(), 0);
    AB.M01() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work));
    AB.M11() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work));

    work = ENFT_SSE::_mm_setr_ps(B.M02(), B.M12(), B.M22(), 0);
    AB.M02() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work));
    AB.M12() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work));
}
inline void ATB(const AlignedMatrix3f &A, const AlignedMatrix3f &B,
                AlignedMatrix3f &ATB) {
    ATB.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()),
                                    B.M_00_01_02_x()),
                                    ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_10_11_12_x()),
                                            ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), B.M_20_21_22_x())));
    ATB.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()),
                                    B.M_00_01_02_x()),
                                    ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_x()),
                                            ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M21()), B.M_20_21_22_x())));
    ATB.M_20_21_22_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()),
                                    B.M_00_01_02_x()),
                                    ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B.M_10_11_12_x()),
                                            ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M22()), B.M_20_21_22_x())));
}
// A and ATB are allowed to be the same
inline void ABT(const AlignedMatrix3f &A, const AlignedMatrix3f &B,
                AlignedMatrix3f &ABT) {
    ENFT_SSE::SSE::Set012(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B.M_00_01_02_x())),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B.M_10_11_12_x())),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B.M_20_21_22_x())),
                ABT.M_00_01_02_x());
    ENFT_SSE::SSE::Set012(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B.M_00_01_02_x())),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B.M_10_11_12_x())),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B.M_20_21_22_x())),
                ABT.M_10_11_12_x());
    ENFT_SSE::SSE::Set012(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), B.M_00_01_02_x())),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), B.M_10_11_12_x())),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), B.M_20_21_22_x())),
                ABT.M_20_21_22_x());
}
inline void ABT(const AlignedVector3f &A, const AlignedVector3f &B,
                AlignedMatrix3f &ABT) {
    ABT.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v0()), B.v012x());
    ABT.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v1()), B.v012x());
    ABT.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v2()), B.v012x());
}
inline void AB_T(const AlignedMatrix3f &A, const AlignedMatrix3f &B,
                 AlignedMatrix3f &AB_T, ENFT_SSE::__m128 &work) {
    ENFT_SSE::SSE::Set012(B.M00(), B.M10(), B.M20(), work);
    ENFT_SSE::SSE::Set012(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work)),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work)),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), work)), AB_T.M_00_01_02_x());
    ENFT_SSE::SSE::Set012(B.M01(), B.M11(), B.M21(), work);
    ENFT_SSE::SSE::Set012(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work)),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work)),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), work)), AB_T.M_10_11_12_x());
    ENFT_SSE::SSE::Set012(B.M02(), B.M12(), B.M22(), work);
    ENFT_SSE::SSE::Set012(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work)),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work)),
                ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), work)), AB_T.M_20_21_22_x());
}
inline void ABpC(const AlignedMatrix3f &A, const AlignedVector3f &B,
                 const AlignedVector3f &C, AlignedVector3f &ABpC) {
    ABpC.v0() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B.v012x()));
    ABpC.v1() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B.v012x()));
    ABpC.v2() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), B.v012x()));
    ABpC.v012x() = ENFT_SSE::_mm_add_ps(ABpC.v012x(), C.v012x());
}
inline void nABpC(const AlignedMatrix3f &A, const AlignedVector3f &B,
                  const AlignedVector3f &C, AlignedVector3f &nABpC) {
    nABpC.v0() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B.v012x()));
    nABpC.v1() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B.v012x()));
    nABpC.v2() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), B.v012x()));
    nABpC.v012x() = ENFT_SSE::_mm_add_ps(nABpC.v012x(), C.v012x());
}
inline void ssTA(const AlignedVector3f &s, AlignedMatrix3f &A) {
    A.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v0()), s.v012x()),
                                  A.M_00_01_02_x());
    A.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v1()), s.v012x()),
                                  A.M_10_11_12_x());
    A.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v2()), s.v012x()),
                                  A.M_20_21_22_x());
}
bool InvertSymmetricUpper(AlignedMatrix3f &A);
bool InvertSymmetricUpper(const AlignedMatrix3f &A, AlignedMatrix3f &Ainv);
inline bool InvertSymmetricUpper(const AlignedMatrix3f &A,
                                 AlignedMatrix3f &Ainv, float *work0) {
    return InvertSymmetricUpper(A, Ainv);
}
bool SolveLinearSystemSymmetricUpper(AlignedMatrix3f &A, AlignedVector3f &b);
inline bool SolveLinearSystemSymmetricUpper(AlignedMatrix3f &A,
        AlignedVector3f &b, float *work0) {
    return SolveLinearSystemSymmetricUpper(A, b);
}

template<typename TYPE> class Matrix3 {
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
    inline const TYPE &M20() const {
        return m_data[6];
    }    inline TYPE &M20() {
        return m_data[6];
    }
    inline const TYPE &M21() const {
        return m_data[7];
    }    inline TYPE &M21() {
        return m_data[7];
    }
    inline const TYPE &M22() const {
        return m_data[8];
    }    inline TYPE &M22() {
        return m_data[8];
    }
    inline operator const TYPE *() const {
        return (const TYPE *) this;
    }
    inline operator TYPE *() {
        return (TYPE *) this;
    }
    inline void SetZero() {
        memset(this, 0, sizeof(Matrix3<TYPE>));
    }
    inline void Scale(const TYPE &s) {
        m_data[0] *= s;
        m_data[1] *= s;
        m_data[2] *= s;
        m_data[3] *= s;
        m_data[4] *= s;
        m_data[5] *= s;
        m_data[6] *= s;
        m_data[7] *= s;
        m_data[8] *= s;
    }
    inline void Transpose() {
        TYPE tmp;
        SWAP(M01(), M10(), tmp);
        SWAP(M02(), M20(), tmp);
        SWAP(M12(), M21(), tmp);
    }
    inline void GetTranspose(Matrix3<TYPE> &MT) const {
        MT.M00() = M00();
        MT.M10() = M01();
        MT.M20() = M02();
        MT.M01() = M10();
        MT.M11() = M11();
        MT.M21() = M12();
        MT.M02() = M20();
        MT.M12() = M21();
        MT.M22() = M22();
    }
    inline void operator += (const Matrix3<TYPE> &M) {
        M00() = M.M00() + M00();
        M01() = M.M01() + M01();
        M02() = M.M02() + M02();
        M10() = M.M10() + M10();
        M11() = M.M11() + M11();
        M12() = M.M12() + M12();
        M20() = M.M20() + M20();
        M21() = M.M21() + M21();
        M22() = M.M22() + M22();
    }
    inline void operator -= (const Matrix3<TYPE> &M) {
        M00() = -M.M00() + M00();
        M01() = -M.M01() + M01();
        M02() = -M.M02() + M02();
        M10() = -M.M10() + M10();
        M11() = -M.M11() + M11();
        M12() = -M.M12() + M12();
        M20() = -M.M20() + M20();
        M21() = -M.M21() + M21();
        M22() = -M.M22() + M22();
    }
    inline void IncreaseDiagonal(const float &lambda) {
        M00() += lambda;
        M11() += lambda;
        M22() += lambda;
    }
    inline void Print(const bool e = false) const {
        if(e) {
            printf("%e %e %e\n", M00(), M01(), M02());
            printf("%e %e %e\n", M10(), M11(), M12());
            printf("%e %e %e\n", M20(), M21(), M22());
        } else {
            printf("%lf %lf %lf\n", M00(), M01(), M02());
            printf("%lf %lf %lf\n", M10(), M11(), M12());
            printf("%lf %lf %lf\n", M20(), M21(), M22());
        }
    }
    inline void Save(FILE *fp, const bool e = false) const {
        if(e) {
            fprintf(fp, "%e %e %e\n", M00(), M01(), M02());
            fprintf(fp, "%e %e %e\n", M10(), M11(), M12());
            fprintf(fp, "%e %e %e\n", M20(), M21(), M22());
        } else {
            fprintf(fp, "%lf %lf %lf\n", M00(), M01(), M02());
            fprintf(fp, "%lf %lf %lf\n", M10(), M11(), M12());
            fprintf(fp, "%lf %lf %lf\n", M20(), M21(), M22());
        }
    }
    inline void Load(FILE *fp) {
        fscanf(fp, "%lf %lf %lf", &M00(), &M01(), &M02());
        fscanf(fp, "%lf %lf %lf", &M10(), &M11(), &M12());
        fscanf(fp, "%lf %lf %lf", &M20(), &M21(), &M22());
    }
  private:
    TYPE m_data[9];
};

typedef Matrix3<float > Matrix3f;
typedef Matrix3<double> Matrix3d;

template<class TYPE> inline void AmB(const Matrix3<TYPE> &A,
                                     const Matrix3<TYPE> &B, Matrix3<TYPE> &AmB) {
    AmB.M00() = A.M00() - B.M00();
    AmB.M01() = A.M01() - B.M01();
    AmB.M02() = A.M02() - B.M02();
    AmB.M10() = A.M10() - B.M10();
    AmB.M11() = A.M11() - B.M11();
    AmB.M12() = A.M12() - B.M12();
    AmB.M20() = A.M20() - B.M20();
    AmB.M21() = A.M21() - B.M21();
    AmB.M22() = A.M22() - B.M22();
}
template<class TYPE> inline void AddAij2To(const Matrix3<TYPE> &A,
        Vector3<TYPE> &to) {
    to.v0() = A.M00() * A.M00() + A.M10() * A.M10() + A.M20() * A.M20() + to.v0();
    to.v1() = A.M01() * A.M01() + A.M11() * A.M11() + A.M21() * A.M21() + to.v1();
    to.v2() = A.M02() * A.M02() + A.M12() * A.M12() + A.M22() * A.M22() + to.v2();
}
template<typename TYPE> inline void sA(const Vector3<TYPE> &s,
                                       Matrix3<TYPE> &A) {
    A.M00() *= s.v0();
    A.M01() *= s.v1();
    A.M02() *= s.v2();
    A.M10() *= s.v0();
    A.M11() *= s.v1();
    A.M12() *= s.v2();
    A.M20() *= s.v0();
    A.M21() *= s.v1();
    A.M22() *= s.v2();
}
template<typename TYPE> inline void AddsATo(const TYPE &s,
        const Matrix3<TYPE> &A, Matrix3<TYPE> &to) {
    to.M00() = s * A.M00() + to.M00();
    to.M01() = s * A.M01() + to.M01();
    to.M02() = s * A.M02() + to.M02();
    to.M10() = s * A.M10() + to.M10();
    to.M11() = s * A.M11() + to.M11();
    to.M12() = s * A.M12() + to.M12();
    to.M20() = s * A.M20() + to.M20();
    to.M21() = s * A.M21() + to.M21();
    to.M22() = s * A.M22() + to.M22();
}
template<typename TYPE> inline void AddsATTo(const TYPE &s,
        const Matrix3<TYPE> &A, Matrix3<TYPE> &to) {
    to.M00() = s * A.M00() + to.M00();
    to.M01() = s * A.M10() + to.M01();
    to.M02() = s * A.M20() + to.M02();
    to.M10() = s * A.M01() + to.M10();
    to.M11() = s * A.M11() + to.M11();
    to.M12() = s * A.M21() + to.M12();
    to.M20() = s * A.M02() + to.M20();
    to.M21() = s * A.M12() + to.M21();
    to.M22() = s * A.M22() + to.M22();
}
template<class TYPE> inline void AddABTo(const Matrix3<TYPE> &A,
        const Vector3<TYPE> &B, Vector3<TYPE> &to) {
    to.v0() = A.M00() * B.v0() + A.M01() * B.v1() + A.M02() * B.v2() + to.v0();
    to.v1() = A.M10() * B.v0() + A.M11() * B.v1() + A.M12() * B.v2() + to.v1();
    to.v2() = A.M20() * B.v0() + A.M21() * B.v1() + A.M22() * B.v2() + to.v2();
}
template<class TYPE> inline void AddABTo(const Matrix3<TYPE> &A,
        const Matrix3<TYPE> &B, Matrix3<TYPE> &to) {
    to.M00() = A.M00() * B.M00() + A.M01() * B.M10() + A.M02() * B.M20() + to.M00();
    to.M01() = A.M00() * B.M01() + A.M01() * B.M11() + A.M02() * B.M21() + to.M01();
    to.M02() = A.M00() * B.M02() + A.M01() * B.M12() + A.M02() * B.M22() + to.M02();
    to.M10() = A.M10() * B.M00() + A.M11() * B.M10() + A.M12() * B.M20() + to.M10();
    to.M11() = A.M10() * B.M01() + A.M11() * B.M11() + A.M12() * B.M21() + to.M11();
    to.M12() = A.M10() * B.M02() + A.M11() * B.M12() + A.M12() * B.M22() + to.M12();
    to.M20() = A.M20() * B.M00() + A.M21() * B.M10() + A.M22() * B.M20() + to.M20();
    to.M21() = A.M20() * B.M01() + A.M21() * B.M11() + A.M22() * B.M21() + to.M21();
    to.M22() = A.M20() * B.M02() + A.M21() * B.M12() + A.M22() * B.M22() + to.M22();
}
template<class TYPE> inline void AddABTTo(const Matrix3<TYPE> &A,
        const Matrix3<TYPE> &B, Matrix3<TYPE> &to) {
    to.M00() = A.M00() * B.M00() + A.M01() * B.M01() + A.M02() * B.M02() + to.M00();
    to.M01() = A.M00() * B.M10() + A.M01() * B.M11() + A.M02() * B.M12() + to.M01();
    to.M02() = A.M00() * B.M20() + A.M01() * B.M21() + A.M02() * B.M22() + to.M02();
    to.M10() = A.M10() * B.M00() + A.M11() * B.M01() + A.M12() * B.M02() + to.M10();
    to.M11() = A.M10() * B.M10() + A.M11() * B.M11() + A.M12() * B.M12() + to.M11();
    to.M12() = A.M10() * B.M20() + A.M11() * B.M21() + A.M12() * B.M22() + to.M12();
    to.M20() = A.M20() * B.M00() + A.M21() * B.M01() + A.M22() * B.M02() + to.M20();
    to.M21() = A.M20() * B.M10() + A.M21() * B.M11() + A.M22() * B.M12() + to.M21();
    to.M22() = A.M20() * B.M20() + A.M21() * B.M21() + A.M22() * B.M22() + to.M22();
}
template<class TYPE> inline void AddATBTo(const Matrix3<TYPE> &A,
        const AlignedVector3f &B, Vector3<TYPE> &to) {
    to.v0() = A.M00() * B.v0() + A.M10() * B.v1() + A.M20() * B.v2() + to.v0();
    to.v1() = A.M01() * B.v0() + A.M11() * B.v1() + A.M21() * B.v2() + to.v1();
    to.v2() = A.M02() * B.v0() + A.M12() * B.v1() + A.M22() * B.v2() + to.v2();
}
inline void AddATBTo(const AlignedMatrix2x3f &A, AlignedMatrix2x3f &B,
                     Matrix3f &to, ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B.M_00_01_02_x()),
                      ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_10_11_12_x()));
    to.M00() = work.m128_f32[0] + to.M00();
    to.M01() = work.m128_f32[1] + to.M01();
    to.M02() = work.m128_f32[2] + to.M02();

    work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B.M_00_01_02_x()),
                      ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_x()));
    to.M10() = work.m128_f32[0] + to.M10();
    to.M11() = work.m128_f32[1] + to.M11();
    to.M12() = work.m128_f32[2] + to.M12();

    work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), B.M_00_01_02_x()),
                      ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B.M_10_11_12_x()));
    to.M20() = work.m128_f32[0] + to.M20();
    to.M21() = work.m128_f32[1] + to.M21();
    to.M22() = work.m128_f32[2] + to.M22();
}
template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(
    Matrix3f &to) {}
template<typename TYPE> inline bool Invert(const Matrix3<TYPE> &M,
        Matrix3f &Minv) {
    Minv.M00() = M.M11() * M.M22() - M.M12() * M.M21();
    Minv.M01() = M.M02() * M.M21() - M.M01() * M.M22();
    Minv.M02() = M.M01() * M.M12() - M.M02() * M.M11();
    Minv.M10() = M.M12() * M.M20() - M.M10() * M.M22();
    Minv.M11() = M.M00() * M.M22() - M.M02() * M.M20();
    Minv.M12() = M.M02() * M.M10() - M.M00() * M.M12();
    Minv.M20() = M.M10() * M.M21() - M.M11() * M.M20();
    Minv.M21() = M.M01() * M.M20() - M.M00() * M.M21();
    Minv.M22() = M.M00() * M.M11() - M.M01() * M.M10();
    const float det = M.M00() * Minv.M00() + M.M01() * Minv.M10() + M.M02() *
                      Minv.M20();
    if(fabs(det) < FLT_EPSILON)
        return false;
    Minv.Scale(1 / det);
    return true;
}
static inline bool Invert(const AlignedMatrix3f &M, Matrix3f &Minv) {
    Minv.M00() = M.M11() * M.M22() - M.M12() * M.M21();
    Minv.M01() = M.M02() * M.M21() - M.M01() * M.M22();
    Minv.M02() = M.M01() * M.M12() - M.M02() * M.M11();
    Minv.M10() = M.M12() * M.M20() - M.M10() * M.M22();
    Minv.M11() = M.M00() * M.M22() - M.M02() * M.M20();
    Minv.M12() = M.M02() * M.M10() - M.M00() * M.M12();
    Minv.M20() = M.M10() * M.M21() - M.M11() * M.M20();
    Minv.M21() = M.M01() * M.M20() - M.M00() * M.M21();
    Minv.M22() = M.M00() * M.M11() - M.M01() * M.M10();
    const float det = M.M00() * Minv.M00() + M.M01() * Minv.M10() + M.M02() *
                      Minv.M20();
    if(fabs(det) < FLT_EPSILON)
        return false;
    Minv.Scale(1 / det);
    return true;
}
template<typename TYPE> inline void sA(const TYPE &s, const Matrix3<TYPE> &A,
                                       Matrix3<TYPE> &sA) {
    sA.M00() = s * A.M00();
    sA.M01() = s * A.M01();
    sA.M02() = s * A.M02();
    sA.M10() = s * A.M10();
    sA.M11() = s * A.M11();
    sA.M12() = s * A.M12();
    sA.M20() = s * A.M20();
    sA.M21() = s * A.M21();
    sA.M22() = s * A.M22();
}
template<typename TYPE> inline void AB(const Matrix3<TYPE> &A,
                                       const Vector3<TYPE> &B, Vector3<TYPE> &AB) {
    AB.v0() = A.M00() * B.v0() + A.M01() * B.v1() + A.M02() * B.v2();
    AB.v1() = A.M10() * B.v0() + A.M11() * B.v1() + A.M12() * B.v2();
    AB.v2() = A.M20() * B.v0() + A.M21() * B.v1() + A.M22() * B.v2();
}
template<typename TYPE> inline void AB(const Matrix3<TYPE> &A,
                                       const Vector3<TYPE> &B, AlignedVector3f &AB) {
    AB.v0() = float(A.M00() * B.v0() + A.M01() * B.v1() + A.M02() * B.v2());
    AB.v1() = float(A.M10() * B.v0() + A.M11() * B.v1() + A.M12() * B.v2());
    AB.v2() = float(A.M20() * B.v0() + A.M21() * B.v1() + A.M22() * B.v2());
}
template<typename TYPE> inline void SubtractABFrom(const Matrix3<TYPE> &A,
        const Vector3<TYPE> &B, Vector3<TYPE> &from) {
    from.v0() -= A.M00() * B.v0() + A.M01() * B.v1() + A.M02() * B.v2();
    from.v1() -= A.M10() * B.v0() + A.M11() * B.v1() + A.M12() * B.v2();
    from.v2() -= A.M20() * B.v0() + A.M21() * B.v1() + A.M22() * B.v2();
}
template<typename TYPE> inline void SubtractATBFrom(const Matrix3<TYPE> &A,
        const Vector3<TYPE> &B, Vector3<TYPE> &from) {
    from.v0() -= A.M00() * B.v0() + A.M10() * B.v1() + A.M20() * B.v2();
    from.v1() -= A.M01() * B.v0() + A.M11() * B.v1() + A.M21() * B.v2();
    from.v2() -= A.M02() * B.v0() + A.M12() * B.v1() + A.M22() * B.v2();
}
template<typename TYPE> inline void s1s2TA(const Vector3<TYPE> &s1,
        const Vector3<TYPE> &s2, Matrix3<TYPE> &A) {
    A.M00() *= s1.v0() * s2.v0();
    A.M01() *= s1.v0() * s2.v1();
    A.M02() *= s1.v0() * s2.v2();
    A.M10() *= s1.v1() * s2.v0();
    A.M11() *= s1.v1() * s2.v1();
    A.M12() *= s1.v1() * s2.v2();
    A.M20() *= s1.v2() * s2.v0();
    A.M21() *= s1.v2() * s2.v1();
    A.M22() *= s1.v2() * s2.v2();
}

template<typename TYPE> class SymmetricMatrix3 {

  public:

    inline const TYPE &M00() const {
        return m_data[0];
    }        inline TYPE &M00() {
        return m_data[0];
    }
    inline const TYPE &M01() const {
        return m_data[1];
    }        inline TYPE &M01() {
        return m_data[1];
    }
    inline const TYPE &M02() const {
        return m_data[2];
    }        inline TYPE &M02() {
        return m_data[2];
    }
    inline const TYPE &M10() const {
        return m_data[1];
    }        inline TYPE &M10() {
        return m_data[1];
    }
    inline const TYPE &M11() const {
        return m_data[3];
    }        inline TYPE &M11() {
        return m_data[3];
    }
    inline const TYPE &M12() const {
        return m_data[4];
    }        inline TYPE &M12() {
        return m_data[4];
    }
    inline const TYPE &M20() const {
        return m_data[2];
    }        inline TYPE &M20() {
        return m_data[2];
    }
    inline const TYPE &M21() const {
        return m_data[4];
    }        inline TYPE &M21() {
        return m_data[4];
    }
    inline const TYPE &M22() const {
        return m_data[5];
    }        inline TYPE &M22() {
        return m_data[5];
    }
    inline void SetZero() {
        memset(this, 0, sizeof(SymmetricMatrix3<TYPE>));
    }
    inline void Get(Matrix3<TYPE> &M) const {
        M.M00() = M00();
        M.M01() = M01();
        M.M02() = M02();
        M.M10() = M10();
        M.M11() = M11();
        M.M12() = M12();
        M.M20() = M20();
        M.M21() = M21();
        M.M22() = M22();
    }
    inline void GetDiagonal(Vector3<TYPE> &d) const {
        d.v0() = m_data[0];
        d.v1() = m_data[3];
        d.v2() = m_data[5];
    }
    inline void SetDiagonal(const Vector3<TYPE> &d) {
        m_data[0] = d.v0();
        m_data[3] = d.v1();
        m_data[5] = d.v2();
    }
    inline void ScaleDiagonal(const float &lambda) {
        M00() *= lambda;
        M11() *= lambda;
        M22() *= lambda;
    }
    inline void IncreaseDiagonal(const float &lambda) {
        M00() += lambda;
        M11() += lambda;
        M22() += lambda;
    }
    //inline float Determinant() const { return M00() * M00() + M01() * M10() + M02() * M20(); }
    inline float Determinant() const {
        return (M00()*M11() - M01()*M01())*M22() + 2*M01()*M12()*M02() - M02()*M02()
               *M11() - M12()*M12()*M00();
    }
    inline void operator += (const SymmetricMatrix3<TYPE> &M) {
        m_data[0] = M.m_data[0] + m_data[0];
        m_data[1] = M.m_data[1] + m_data[1];
        m_data[2] = M.m_data[2] + m_data[2];
        m_data[3] = M.m_data[3] + m_data[3];
        m_data[4] = M.m_data[4] + m_data[4];
        m_data[5] = M.m_data[5] + m_data[5];
    }
    inline void operator *= (const TYPE &s) {
        m_data[0] *= s;
        m_data[1] *= s;
        m_data[2] *= s;
        m_data[3] *= s;
        m_data[4] *= s;
        m_data[5] *= s;
    }
    inline void Print(const bool e = false) const {
        if(e) {
            printf("%e %e %e\n", M00(), M01(), M02());
            printf("%e %e %e\n", M10(), M11(), M12());
            printf("%e %e %e\n", M20(), M21(), M22());
        } else {
            printf("%lf %lf %lf\n", M00(), M01(), M02());
            printf("%lf %lf %lf\n", M10(), M11(), M12());
            printf("%lf %lf %lf\n", M20(), M21(), M22());
        }
    }
    inline void Save(FILE *fp, const bool e = false) const {
        if(e) {
            fprintf(fp, "%e %e %e\n", M00(), M01(), M02());
            fprintf(fp, "%e %e %e\n", M10(), M11(), M12());
            fprintf(fp, "%e %e %e\n", M20(), M21(), M22());
        } else {
            fprintf(fp, "%lf %lf %lf\n", M00(), M01(), M02());
            fprintf(fp, "%lf %lf %lf\n", M10(), M11(), M12());
            fprintf(fp, "%lf %lf %lf\n", M20(), M21(), M22());
        }
    }

  private:

    TYPE m_data[6];
};

typedef SymmetricMatrix3<float > SymmetricMatrix3f;
typedef SymmetricMatrix3<double> SymmetricMatrix3d;

inline void sA(const float &s, const SymmetricMatrix3f &A,
               AlignedMatrix3f &sA) {
    sA.M00() = s * A.M00();
    sA.M01() = sA.M10() = s * A.M01();
    sA.M02() = sA.M20() = s * A.M02();
    sA.M11() = s * A.M11();
    sA.M12() = sA.M21() = s * A.M12();
    sA.M22() = s * A.M22();
}

template<typename TYPE> inline void AddAATTo(const Matrix3<TYPE> &A,
        SymmetricMatrix3<TYPE> &to) {
    to.M00() = A.M00() * A.M00() + A.M01() * A.M01() + A.M02() * A.M02() + to.M00();
    to.M01() = A.M00() * A.M10() + A.M01() * A.M11() + A.M02() * A.M12() + to.M01();
    to.M02() = A.M00() * A.M20() + A.M01() * A.M21() + A.M02() * A.M22() + to.M02();
    to.M11() = A.M10() * A.M10() + A.M11() * A.M11() + A.M12() * A.M12() + to.M11();
    to.M12() = A.M10() * A.M20() + A.M11() * A.M21() + A.M12() * A.M22() + to.M12();
    to.M22() = A.M20() * A.M20() + A.M21() * A.M21() + A.M22() * A.M22() + to.M22();
}
template<class TYPE> inline void AddABTo(const Matrix3<TYPE> &A,
        const SymmetricMatrix3<TYPE> &B, Matrix3<TYPE> &to) {
    to.M00() = A.M00() * B.M00() + A.M01() * B.M10() + A.M02() * B.M20() + to.M00();
    to.M01() = A.M00() * B.M01() + A.M01() * B.M11() + A.M02() * B.M21() + to.M01();
    to.M02() = A.M00() * B.M02() + A.M01() * B.M12() + A.M02() * B.M22() + to.M02();
    to.M10() = A.M10() * B.M00() + A.M11() * B.M10() + A.M12() * B.M20() + to.M10();
    to.M11() = A.M10() * B.M01() + A.M11() * B.M11() + A.M12() * B.M21() + to.M11();
    to.M12() = A.M10() * B.M02() + A.M11() * B.M12() + A.M12() * B.M22() + to.M12();
    to.M20() = A.M20() * B.M00() + A.M21() * B.M10() + A.M22() * B.M20() + to.M20();
    to.M21() = A.M20() * B.M01() + A.M21() * B.M11() + A.M22() * B.M21() + to.M21();
    to.M22() = A.M20() * B.M02() + A.M21() * B.M12() + A.M22() * B.M22() + to.M22();
}
template<class TYPE> inline void AddABTToUpper(const Matrix3<TYPE> &A,
        const Matrix3<TYPE> &B, SymmetricMatrix3<TYPE> &to) {
//#if _DEBUG
//      const double m10 = A.M10() * B.M00() + A.M11() * B.M01() + A.M12() * B.M02() + to.M10();
//      const double m20 = A.M20() * B.M00() + A.M21() * B.M01() + A.M22() * B.M02() + to.M20();
//      const double m21 = A.M20() * B.M10() + A.M21() * B.M11() + A.M22() * B.M12() + to.M21();
//#endif
    to.M00() = A.M00() * B.M00() + A.M01() * B.M01() + A.M02() * B.M02() + to.M00();
    to.M01() = A.M00() * B.M10() + A.M01() * B.M11() + A.M02() * B.M12() + to.M01();
    to.M02() = A.M00() * B.M20() + A.M01() * B.M21() + A.M02() * B.M22() + to.M02();
    to.M11() = A.M10() * B.M10() + A.M11() * B.M11() + A.M12() * B.M12() + to.M11();
    to.M12() = A.M10() * B.M20() + A.M11() * B.M21() + A.M12() * B.M22() + to.M12();
    to.M22() = A.M20() * B.M20() + A.M21() * B.M21() + A.M22() * B.M22() + to.M22();
//#if _DEBUG
//      IO::AssertEqual(to.M01(), m10);
//      IO::AssertEqual(to.M02(), m20);
//      IO::AssertEqual(to.M12(), m21);
//#endif
}
template<typename TYPE> inline void AddsATo(const TYPE &s,
        const SymmetricMatrix3<TYPE> &A, Matrix3<TYPE> &to) {
    double tmp;
    to.M00() = s * A.M00() + to.M00();
    to.M01() = (tmp = s * A.M01()) + to.M01();
    to.M10() = tmp + to.M10();
    to.M02() = (tmp = s * A.M02()) + to.M02();
    to.M20() = tmp + to.M20();
    to.M11() = s * A.M11() + to.M11();
    to.M12() = (tmp = s * A.M12()) + to.M12();
    to.M21() = tmp + to.M21();
    to.M22() = s * A.M22() + to.M22();
}
template<typename TYPE> inline void AddsAToUpper(const TYPE &s,
        const Matrix3<TYPE> &A, SymmetricMatrix3<TYPE> &to) {
    to.M00() = s * A.M00() + to.M00();
    to.M01() = s * A.M01() + to.M01();
    to.M02() = s * A.M02() + to.M02();
    to.M11() = s * A.M11() + to.M11();
    to.M12() = s * A.M12() + to.M12();
    to.M22() = s * A.M22() + to.M22();
}
template<typename TYPE> inline void AddATAToUpper(const Matrix2x3<TYPE> &A,
        SymmetricMatrix3<TYPE> &to) {
    to.M00() = A.M00() * A.M00() + A.M10() * A.M10() + to.M00();
    to.M01() = A.M00() * A.M01() + A.M10() * A.M11() + to.M01();
    to.M02() = A.M00() * A.M02() + A.M10() * A.M12() + to.M02();
    to.M11() = A.M01() * A.M01() + A.M11() * A.M11() + to.M11();
    to.M12() = A.M01() * A.M02() + A.M11() * A.M12() + to.M12();
    to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + to.M22();
}
inline void AddAATToUpper(const AlignedVector3f &A, SymmetricMatrix3f &to,
                          ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v0()), A.v012x());
    to.M00() = work.m128_f32[0] + to.M00();
    to.M01() = work.m128_f32[1] + to.M01();
    to.M02() = work.m128_f32[2] + to.M02();
    to.M11() = A.v1() * A.v1() + to.M11();
    to.M12() = A.v1() * A.v2() + to.M12();
    to.M22() = A.v2() * A.v2() + to.M22();
}
inline void AddATAToUpper(const AlignedMatrix2x3f &A, SymmetricMatrix3f &to,
                          ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), A.M_00_01_02_x()),
                      ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), A.M_10_11_12_x()));
    to.M00() = work.m128_f32[0] + to.M00();
    to.M01() = work.m128_f32[1] + to.M01();
    to.M02() = work.m128_f32[2] + to.M02();
    to.M11() = A.M01() * A.M01() + A.M11() * A.M11() + to.M11();
    to.M12() = A.M01() * A.M02() + A.M11() * A.M12() + to.M12();
    to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + to.M22();
}
inline void AddATAToUpper(const AlignedMatrix3f &A, SymmetricMatrix3f &to,
                          ENFT_SSE::__m128 &work) {
    work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), A.M_00_01_02_x()),
                                 ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), A.M_10_11_12_x())),
                      ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), A.M_20_21_22_x()));
    to.M00() = work.m128_f32[0] + to.M00();
    to.M01() = work.m128_f32[1] + to.M01();
    to.M02() = work.m128_f32[2] + to.M02();
    to.M11() = A.M01() * A.M01() + A.M11() * A.M11() + A.M21() * A.M21() + to.M11();
    to.M12() = A.M01() * A.M02() + A.M11() * A.M12() + A.M21() * A.M22() + to.M12();
    to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + A.M22() * A.M22() + to.M22();
}
template<typename TYPE> inline void AddATAToUpper(const Matrix3<TYPE> &A,
        SymmetricMatrix3<TYPE> &to) {
    to.M00() = A.M00() * A.M00() + A.M10() * A.M10() + A.M20() * A.M20() + to.M00();
    to.M01() = A.M00() * A.M01() + A.M10() * A.M11() + A.M20() * A.M21() + to.M01();
    to.M02() = A.M00() * A.M02() + A.M10() * A.M12() + A.M20() * A.M22() + to.M02();
    to.M11() = A.M01() * A.M01() + A.M11() * A.M11() + A.M21() * A.M21() + to.M11();
    to.M12() = A.M01() * A.M02() + A.M11() * A.M12() + A.M21() * A.M22() + to.M12();
    to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + A.M22() * A.M22() + to.M22();
}
template<class MATRIX> inline void FinishAdditionATAToUpper(
    SymmetricMatrix3f &to) {}
template<class MATRIX> inline void FinishAdditionATAToUpper(
    SymmetricMatrix3d &to) {}
template<class MATRIX_1, class MATRIX_2> inline void FinishAdditionATAToUpper(
    SymmetricMatrix3f &to) {}
template<class MATRIX_1, class MATRIX_2> inline void FinishAdditionATAToUpper(
    SymmetricMatrix3d &to) {}

template<typename TYPE> inline void SetLowerFromUpper(SymmetricMatrix3<TYPE>
        &M) {}
template<typename TYPE> inline void GetDiagonal(const SymmetricMatrix3<TYPE> &M,
        Vector3<TYPE> &d) {
    M.GetDiagonal(d);
}
template<typename TYPE> inline void SetDiagonal(const Vector3<TYPE> &d,
        SymmetricMatrix3<TYPE> &M) {
    M.SetDiagonal(d);
}
template<typename TYPE> inline void ScaleDiagonal(const float &lambda,
        SymmetricMatrix3<TYPE> &M) {
    M.ScaleDiagonal(lambda);
}
template<typename TYPE> inline void IncreaseDiagonal(const float &lambda,
        SymmetricMatrix3<TYPE> &M) {
    M.IncreaseDiagonal(lambda);
}

template<typename TYPE> inline void AB(const SymmetricMatrix3<TYPE> &A,
                                       const Vector3<TYPE> &B, Vector3<TYPE> &AB) {
    AB.v0() = A.M00() * B.v0() + A.M01() * B.v1() + A.M02() * B.v2();
    AB.v1() = A.M10() * B.v0() + A.M11() * B.v1() + A.M12() * B.v2();
    AB.v2() = A.M20() * B.v0() + A.M21() * B.v1() + A.M22() * B.v2();
}
template<typename TYPE> inline void AddABTo(const SymmetricMatrix3<TYPE> &A,
        const Vector3<TYPE> &B, Vector3<TYPE> &to) {
    to.v0() = A.M00() * B.v0() + A.M01() * B.v1() + A.M02() * B.v2() + to.v0();
    to.v1() = A.M10() * B.v0() + A.M11() * B.v1() + A.M12() * B.v2() + to.v1();
    to.v2() = A.M20() * B.v0() + A.M21() * B.v1() + A.M22() * B.v2() + to.v2();
}
template<typename TYPE> inline void ATB(const SymmetricMatrix3<TYPE> &A,
                                        const Vector3<TYPE> &B, Vector3<TYPE> &AB) {
    AB.v0() = A.M00() * B.v0() + A.M10() * B.v1() + A.M20() * B.v2();
    AB.v1() = A.M01() * B.v0() + A.M11() * B.v1() + A.M21() * B.v2();
    AB.v2() = A.M02() * B.v0() + A.M12() * B.v1() + A.M22() * B.v2();
}
inline bool InvertSymmetricUpper(SymmetricMatrix3f &M) {
    //const float det = M.Determinant();
    const float det = M.M00() * (M.M11() * M.M22() - M.M12() * M.M12()) +
                      M.M01() * (M.M12() * M.M02() - M.M01() * M.M22()) +
                      M.M02() * (M.M01() * M.M12() - M.M11() * M.M02());
    if(fabs(det) < FLT_EPSILON) {
        M.SetZero();
        return false;
    }
    const float t = 1 / det, m00 = M.M00(), m01 = M.M01(), m02 = M.M02(),
                m11 = M.M11();
    M.M00() = (m11 * M.M22() - M.M12() * M.M12()) * t;
    M.M01() = (m02 * M.M12() - m01 * M.M22()) * t;
    M.M02() = (m01 * M.M12() - m02 * m11) * t;
    M.M11() = (m00 * M.M22() - m02 * m02) * t;
    M.M12() = (m02 * m01 - m00 * M.M12()) * t;
    M.M22() = (m00 * m11 - m01 * m01) * t;
    return true;
}
inline bool InvertSymmetricUpper(const SymmetricMatrix3f &M,
                                 SymmetricMatrix3f &Minv) {
    const float det = M.Determinant();
    if(fabs(det) < FLT_EPSILON) {
        Minv.SetZero();
        return false;
    }
    const float t = 1 / det;
    Minv.M00() = (M.M11() * M.M22() - M.M12() * M.M12()) * t;
    Minv.M01() = (M.M02() * M.M12() - M.M01() * M.M22()) * t;
    Minv.M02() = (M.M01() * M.M12() - M.M02() * M.M11()) * t;
    Minv.M11() = (M.M00() * M.M22() - M.M02() * M.M02()) * t;
    Minv.M12() = (M.M02() * M.M01() - M.M00() * M.M12()) * t;
    Minv.M22() = (M.M00() * M.M11() - M.M01() * M.M01()) * t;
    return true;
}
inline bool InvertSymmetricUpper(const AlignedMatrix3f &M,
                                 SymmetricMatrix3f &Minv) {
    const float det = M.Determinant();
    if(fabs(det) < FLT_EPSILON) {
        Minv.SetZero();
        return false;
    }
    const float t = 1 / det;
    Minv.M00() = (M.M11() * M.M22() - M.M12() * M.M12()) * t;
    Minv.M01() = (M.M02() * M.M12() - M.M01() * M.M22()) * t;
    Minv.M02() = (M.M01() * M.M12() - M.M02() * M.M11()) * t;
    Minv.M11() = (M.M00() * M.M22() - M.M02() * M.M02()) * t;
    Minv.M12() = (M.M02() * M.M01() - M.M00() * M.M12()) * t;
    Minv.M22() = (M.M00() * M.M11() - M.M01() * M.M01()) * t;
    return true;
}
template<typename TYPE> inline void ATB(const SymmetricMatrix3<TYPE> &A,
                                        const Matrix3<TYPE> &B, Matrix3<TYPE> &ATB) {
    ATB.M00() = A.M00() * B.M00() + A.M10() * B.M10() + A.M20() * B.M20();
    ATB.M01() = A.M00() * B.M01() + A.M10() * B.M11() + A.M20() * B.M21();
    ATB.M02() = A.M00() * B.M02() + A.M10() * B.M12() + A.M20() * B.M22();
    ATB.M10() = A.M01() * B.M00() + A.M11() * B.M10() + A.M21() * B.M20();
    ATB.M11() = A.M01() * B.M01() + A.M11() * B.M11() + A.M21() * B.M21();
    ATB.M12() = A.M01() * B.M02() + A.M11() * B.M12() + A.M21() * B.M22();
    ATB.M20() = A.M02() * B.M00() + A.M12() * B.M10() + A.M22() * B.M20();
    ATB.M21() = A.M02() * B.M01() + A.M12() * B.M11() + A.M22() * B.M21();
    ATB.M22() = A.M02() * B.M02() + A.M12() * B.M12() + A.M22() * B.M22();
}
template<typename TYPE> inline void SubtractATBFrom(const Matrix3<TYPE> &A,
        const Matrix3<TYPE> &B, SymmetricMatrix3<TYPE> &from) {
    from.M00() -= A.M00() * B.M00() + A.M10() * B.M10() + A.M20() * B.M20();
    from.M01() -= A.M00() * B.M01() + A.M10() * B.M11() + A.M20() * B.M21();
    from.M02() -= A.M00() * B.M02() + A.M10() * B.M12() + A.M20() * B.M22();
    from.M10() -= A.M01() * B.M00() + A.M11() * B.M10() + A.M21() * B.M20();
    from.M11() -= A.M01() * B.M01() + A.M11() * B.M11() + A.M21() * B.M21();
    from.M12() -= A.M01() * B.M02() + A.M11() * B.M12() + A.M21() * B.M22();
    from.M20() -= A.M02() * B.M00() + A.M12() * B.M10() + A.M22() * B.M20();
    from.M21() -= A.M02() * B.M01() + A.M12() * B.M11() + A.M22() * B.M21();
    from.M22() -= A.M02() * B.M02() + A.M12() * B.M12() + A.M22() * B.M22();
}
template<typename TYPE> inline void ssTA(const Vector3<TYPE> &s,
        SymmetricMatrix3<TYPE> &A) {
    A.M00() *= s.v0() * s.v0();
    A.M01() *= s.v0() * s.v1();
    A.M02() *= s.v0() * s.v2();
    A.M11() *= s.v1() * s.v1();
    A.M12() *= s.v1() * s.v2();
    A.M22() *= s.v2() * s.v2();
}
inline void AB(const AlignedMatrix2x3f &A, const SymmetricMatrix3f &B,
               AlignedMatrix2x3f &AB, ENFT_SSE::__m128 *work3) {
    work3[0] = ENFT_SSE::_mm_setr_ps(B.M00(), B.M10(), B.M20(), 0.0f);
    work3[1] = ENFT_SSE::_mm_setr_ps(B.M01(), B.M11(), B.M21(), 0.0f);
    work3[2] = ENFT_SSE::_mm_setr_ps(B.M02(), B.M12(), B.M22(), 0.0f);
    AB.M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work3[0]));
    AB.M01() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work3[1]));
    AB.M02() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work3[2]));
    AB.M10() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work3[0]));
    AB.M11() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work3[1]));
    AB.M12() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work3[2]));
}

}

#endif