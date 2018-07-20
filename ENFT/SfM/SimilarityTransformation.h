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

#ifndef _SIMILARITY_TRANSFORMATION_H_
#define _SIMILARITY_TRANSFORMATION_H_

#include "Camera.h"

class SimilarityTransformation2D : public RigidTransformation2D {

  public:

    inline const ENFT_SSE::__m128 &s() const {
        return m_s;
    }
    inline       ENFT_SSE::__m128 &s()       {
        return m_s;
    }

    inline void MakeIdentity() {
        RigidTransformation2D::MakeIdentity();
        SetScale(1.0f);
    }
    inline const float &GetScale() const {
        return m_s.m128_f32[0];
    }
    inline void SetScale(const float &s) {
        m_s = _mm_set1_ps(s);
    }
    inline void Set(const float &theta, const float &tx, const float &ty,
                    const float &s) {
        RigidTransformation2D::Set(theta, tx, ty);
        SetScale(s);
    }
    inline void Get(LA::AlignedMatrix2x3f &M) const {
        M.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(s(), r_00_01_10_11());
        M.M00() = M.M10();
        M.M01() = M.M11();
        M.M02() = tx();
        M.M10() = M.M12();
        M.M11() = M.reserve1();
        M.M12() = ty();
    }

    inline void Invert(SimilarityTransformation2D &Tinv) const {
        Tinv.SetScale(1 / GetScale());
        RotationTransformation2D::Invert(Tinv);
        Tinv.m_t.v0() = -Tinv.GetScale() * (r00() * tx() + r10() * ty());
        Tinv.m_t.v1() = -Tinv.GetScale() * (r01() * tx() + r11() * ty());
    }
    //inline void Invert(LA::AlignedMatrix2x3f &Tinv) const
    //{
    //  Tinv.reserve0() = 1 / s();
    //  Tinv.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(Tinv.reserve0()), r_00_01_10_11());
    //  Tinv.M00() = Tinv.M10();    Tinv.M01() = Tinv.M12();        Tinv.M02() = -Tinv.reserve0() * (r00() * tx() + r10() * ty());
    //  Tinv.M10() = Tinv.M11();    Tinv.M11() = Tinv.reserve1();   Tinv.M12() = -Tinv.reserve0() * (r01() * tx() + r11() * ty());
    //}

    // Tx = s * R * x + t
    inline void Apply(const Point2D &x, Point2D &Tx) const {
        RotationTransformation2D::Apply(x, Tx);
        Tx.x() = GetScale() * Tx.x() + tx();
        Tx.y() = GetScale() * Tx.y() + ty();
    }

    // x2 - c = s * R * (x1 - c) + t --> x2 = s * R * x1 - s * R * c + t + c
    // s' = s, R' = R, t' = -s * R * c + t + c
    inline void CenterToCorner(const Point2D &c, ENFT_SSE::__m128 &work) {
        work = ENFT_SSE::_mm_mul_ps(M_00_01_10_11(), _mm_setr_ps(c.x(), c.y(), c.x(), c.y()));
        m_t.v0() += c.x() - GetScale() * (work.m128_f32[0] + work.m128_f32[1]);
        m_t.v1() += c.y() - GetScale() * (work.m128_f32[2] + work.m128_f32[3]);
    }

    //inline void GetSSE(ENFT_SSE::__m128 *A) const
    //{
    //  A[0] = _mm_set1_ps(r00());      A[1] = _mm_set1_ps(r01());      A[2] = _mm_set1_ps(tx());
    //  A[3] = _mm_set1_ps(r10());      A[4] = _mm_set1_ps(r11());      A[5] = _mm_set1_ps(tx());
    //}

    inline void SaveB(FILE *fp) const {
        fwrite(this, sizeof(SimilarityTransformation2D), 1, fp);
    }
    inline void LoadB(FILE *fp) {
        fread(this, sizeof(SimilarityTransformation2D), 1, fp);
    }

    inline void Print() const {
        RigidTransformation2D::Print();
        printf("%f\n", s());
    }

  protected:

    ENFT_SSE::__m128 m_s;

  public:

    static inline void ComputeRelativeTransformation(SimilarityTransformation2D &T1,
            const SimilarityTransformation2D &T2, SimilarityTransformation2D &T12,
            ENFT_SSE::__m128 *work2) {
        printf("Error!\n");
        exit(0);
    }

    // x2 = s12 * R12 * x1 + t12 = s12 * R12 * (s1 * (R1 * x) + t1) + t12 = s12*s1 * R12*R1 * x + s12*R12*t1 + t12
    // s2 = s12 * s1, R2 = R12 * R1, t2 = s12 * R12 * t1 + t12
    // T12 and T2 are allowed to be the same
    static inline void AccumulateTransformation(const SimilarityTransformation2D
            &T1, const SimilarityTransformation2D &T12, SimilarityTransformation2D &T2,
            ENFT_SSE::__m128 *work2) {
        T2.SetScale(T12.GetScale() * T1.GetScale());
        work2[0] = ENFT_SSE::_mm_mul_ps(T12.r_00_01_10_11(), _mm_setr_ps(T1.tx(), T1.ty(),
                              T1.tx(), T1.ty()));
        T2.tx() = T12.tx() + T12.GetScale() * (work2[0].m128_f32[0] +
                                               work2[0].m128_f32[1]);
        T2.ty() = T12.ty() + T12.GetScale() * (work2[0].m128_f32[2] +
                                               work2[0].m128_f32[3]);
        RotationTransformation2D::AccumulateTransformation(T1, T12, T2, work2);
    }

    // x2 = A * x1 + t
    // sx2 = A' * x1 + t'
    // A' = s * A12, t12' = s * t12
    // x1 = Ainv * x2 + tinv
    // x1 = Ainv' * sx2 + tinv'
    // Ainv' = Ainv / s, tinv' = tinv;
    static inline void ChangeScale2(const float &s, const float &sinv,
                                    SimilarityTransformation2D &T, SimilarityTransformation2D &Tinv,
                                    ENFT_SSE::__m128 *work0) {
        T.SetScale(T.GetScale() * s);
        T.t() *= s;
        Tinv.SetScale(Tinv.GetScale() * sinv);
    }
};

class SimilarityTransformation3D : public RigidTransformation3D {

  public:

    inline const float &s() const {
        return m_sss1.m128_f32[0];
    }
    inline const ENFT_SSE::__m128 &sss1() const {
        return m_sss1;
    }
    inline       ENFT_SSE::__m128 &sss1()       {
        return m_sss1;
    }
    inline void Set(const RigidTransformation3D &T, const float &s) {
        memcpy(this, T, sizeof(RigidTransformation3D));
        SetScale(s);
    }
    inline void SetScale(const float &s) {
        sss1() = _mm_setr_ps(s, s, s, 1.0f);
    }
    inline void MakeIdentity() {
        RigidTransformation3D::MakeIdentity();
        m_sss1 = _mm_set1_ps(1);
    }
    inline void operator = (const RigidTransformation3D &T) {
        memcpy(this, T, sizeof(RigidTransformation3D));
        m_sss1 = _mm_set1_ps(1.0f);
    }

    // X2 = s * (R * X1 + t)
    inline void Apply(const Point3D &X1, Point3D &X2) const {
        RigidTransformation3D::Apply(X1, X2);
        X2 *= m_sss1;
    }
    inline void Apply(Point3D &X, ENFT_SSE::__m128 *work1) const {
        RigidTransformation3D::Apply(X, work1);
        X *= m_sss1;
    }
    inline void Apply(const ENFT_SSE::__m128 &X1, ENFT_SSE::__m128 &X2) const {
        RigidTransformation3D::Apply(X1, X2);
        X2 = ENFT_SSE::_mm_mul_ps(m_sss1, X2);
    }
    inline void Apply(const Point3D &X, Point3D &RX, Point3D &TX,
                      Point3D &SX) const {
        Apply(X.XYZx(), RX.XYZx(), TX.XYZx(), SX.XYZx());
    }
    inline void Apply(const ENFT_SSE::__m128 &X, ENFT_SSE::__m128 &RX, ENFT_SSE::__m128 &TX, ENFT_SSE::__m128 &SX) const {
        RigidTransformation3D::Apply(X, RX, TX);
        SX = ENFT_SSE::_mm_mul_ps(m_sss1, TX);
    }
    inline void ApplyRigidTransformation(const ENFT_SSE::__m128 &X, ENFT_SSE::__m128 &RX,
                                         ENFT_SSE::__m128 &TX) const {
        RigidTransformation3D::Apply(X, RX, TX);
    }

    inline void GetTranspose(LA::AlignedMatrix4f &TT) const {
        TT.M00() = r00();
        TT.M10() = r01();
        TT.M20() = r02();
        TT.M30() = tX();
        TT.M01() = r10();
        TT.M11() = r11();
        TT.M21() = r12();
        TT.M31() = tY();
        TT.M02() = r20();
        TT.M12() = r21();
        TT.M22() = r22();
        TT.M32() = tZ();
        TT.M03() = 0;
        TT.M13() = 0;
        TT.M23() = 0;
        TT.M33() = 1;
        TT.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(m_sss1, TT.M_00_01_02_03());
        TT.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(m_sss1, TT.M_10_11_12_13());
        TT.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(m_sss1, TT.M_20_21_22_23());
        TT.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(m_sss1, TT.M_30_31_32_33());
    }

    //     R2 * X2 + t2 = s(R1 * X1 + t1)
    // ==> R2 * [s * (R * X1 + t)] + t2 = s(R1 * X1 + t1)
    // ==> R2 = R1 * R^T
    //     t2 = s * (t1 - R2 * t)
    // T1 and T2 are allowed to be the same
    inline void Apply(const RigidTransformation3D &T1, RigidTransformation3D &T2,
                      ENFT_SSE::__m128 *work3) const {
        //LA::ABT(T1, *this, T2);
        //GetTranslation(work2[0]);
        //T2.ApplyRotation(work2[0], work2[1]);
        //T1.GetTranslation(work2[0]);
        //work2[0] = ENFT_SSE::_mm_mul_ps(m_sss1, ENFT_SSE::_mm_sub_ps(work2[0], work2[1]));
        //T2.SetTranslation(work2[0]);
        T1.GetTranslation(work3[0]);
        LA::ABT(T1, *this, T2);
        GetTranslation(work3[1]);
        T2.ApplyRotation(work3[1], work3[2]);
        work3[0] = ENFT_SSE::_mm_mul_ps(m_sss1, ENFT_SSE::_mm_sub_ps(work3[0], work3[2]));
        T2.SetTranslation(work3[0]);
    }
    // (1/s, RT, -sRTt)
    inline void Invert(ENFT_SSE::__m128 *work2) {
        GetTranslation(work2[0]);
        Transpose();
        ApplyRotation(work2[0], work2[1]);
        work2[1] = ENFT_SSE::_mm_mul_ps(m_sss1, work2[1]);
        tX() = -work2[1].m128_f32[0];
        tY() = -work2[1].m128_f32[1];
        tZ() = -work2[1].m128_f32[2];
        work2[0].m128_f32[0] = 1 / s();
        SetScale(work2[0].m128_f32[0]);
    }
    inline void Invert(SimilarityTransformation3D &Sinv, ENFT_SSE::__m128 *work2) const {
        LA::AlignedMatrix3f::GetTranspose(Sinv);
        GetTranslation(work2[0]);
        Sinv.ApplyRotation(work2[0], work2[1]);
        work2[1] = ENFT_SSE::_mm_mul_ps(m_sss1, work2[1]);
        Sinv.tX() = -work2[1].m128_f32[0];
        Sinv.tY() = -work2[1].m128_f32[1];
        Sinv.tZ() = -work2[1].m128_f32[2];
        work2[0].m128_f32[0] = 1 / s();
        Sinv.SetScale(work2[0].m128_f32[0]);
    }
    inline void ComputeTransformationError(const Point3D &X1, const Point3D &X2,
                                           Point3D &e) const {
        Apply(X1, e);
        LA::AmB(X2, e, e);
    }
    inline float ComputeTransformationSquaredError(const Point3D &X1,
            const Point3D &X2, ENFT_SSE::__m128 &work) const {
        Apply(X1.XYZx(), work);
        work = ENFT_SSE::_mm_sub_ps(X2.XYZx(), work);
        work = ENFT_SSE::_mm_mul_ps(work, work);
        return ENFT_SSE::SSE::Sum012(work);
    }

    inline void Interpolate(const SimilarityTransformation3D &S1,
                            const SimilarityTransformation3D &S2, const float &alpha, ENFT_SSE::__m128 *work5) {
        //RigidTransformation3D::Interpolate(S1, S2, alpha, work5);
        //SetScale(S1.s() * alpha + S2.s() * (1 - alpha));

        SetScale(S1.s() * alpha + S2.s() * (1 - alpha));

        ENFT_SSE::__m128 &a1 = work5[0], &a2 = work5[1], &v1 = work5[2], &v2 = work5[3],
                &v = work5[4];
        a1 = _mm_set1_ps(alpha * S1.s() / s());
        a2 = _mm_set1_ps((1 - alpha) * S2.s() / s());

        S1.ToQuaternion(&v1.m128_f32[0]);
        S2.ToQuaternion(&v2.m128_f32[0]);
        v = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(a1, v1), ENFT_SSE::_mm_mul_ps(a2, v2));
        v = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / sqrt(ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(v, v)))), v);
        FromQuaternion(v, v1);

        S1.GetCenter(v1);
        S2.GetCenter(v2);
        v = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(a1, v1), ENFT_SSE::_mm_mul_ps(a2, v2));
        SetCenter(v);
    }

    inline void Print() const {
        RigidTransformation3D::Print();
        printf("s = %f\n", s());
    }
    inline void SaveB(FILE *fp) const {
        fwrite(this, sizeof(SimilarityTransformation3D), 1, fp);
    }
    inline void SaveB(const char *fileName) const {
        FILE *fp = fopen( fileName, "wb");
        SaveB(fp);
        fclose(fp);
    }
    inline void LoadB(FILE *fp) {
        fread(this, sizeof(SimilarityTransformation3D), 1, fp);
    }
    inline void LoadB(const char *fileName) {
        FILE *fp = fopen( fileName, "rb");
        LoadB(fp);
        fclose(fp);
    }

  private:

    ENFT_SSE::__m128 m_sss1;

  public:

    // X2 = s12 * R12 * X1 + s12 * t12
    //    = s12 * R12 * (s1 * R1 * X + s1 * t1) + s12 * t12
    //    = (s1*s12) * (R12*R1) * X + (s1*s12*R12*t1 + s12*t12)
    // X2 = s2 * R2 * X + s2 * t2
    // s2 = s1*s12                  --> s12 = s2 / s1
    // R2 = R12*R1                  --> R12 = R2 * R1^T
    // t2 = R12 * t1 + 1/s1 * t12   --> t12 = s1 * (t2 - R12 * t1)
    static inline void ComputeRelativeTransformation(const
            SimilarityTransformation3D &S1, const SimilarityTransformation3D &S2,
            SimilarityTransformation3D &S12,
            ENFT_SSE::__m128 *work2) {
        S12.SetScale(S2.s() / S1.s());
        LA::ABT(S2, S1, S12);
        S1.GetTranslation(work2[0]);
        S12.ApplyRotation(work2[0], work2[1]);
        S2.GetTranslation(work2[0]);
        work2[0] = ENFT_SSE::_mm_mul_ps(S1.sss1(), ENFT_SSE::_mm_sub_ps(work2[0], work2[1]));
        S12.SetTranslation(work2[0]);
    }

    // X2 = s12 * R12 * X1 + s12 * t12
    //    = s12 * R12 * (s1 * R1 * X + s1 * t1) + s12 * t12
    //    = (s1*s12) * (R12*R1) * X + (s1*s12*R12*t1 + s12*t12)
    // X2 = s2 * R2 * X + s2 * t2 --> s2 = s1 * s12
    //                            --> R2 = R12 * R1
    //                            --> t2 = R12 * t1 + 1/s1 * t12
    static inline void AccumulateTransformation(const SimilarityTransformation3D
            &S1, const SimilarityTransformation3D &S12, SimilarityTransformation3D &S2,
            ENFT_SSE::__m128 *work2) {
        S2.SetScale(S1.s() * S12.s());
        LA::AB(S12, S1, S2, work2[0]);
        S1.GetTranslation(work2[0]);
        S12.ApplyRotation(work2[0], work2[1]);
        S12.GetTranslation(work2[0]);
        work2[0] = ENFT_SSE::_mm_add_ps(work2[1], ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / S1.s()), work2[0]));
        S2.SetTranslation(work2[0]);
    }
    static inline void AccumulateTransformation(const SimilarityTransformation3D
            &S1, const RigidTransformation3D &T12, SimilarityTransformation3D &S2,
            ENFT_SSE::__m128 *work2) {
        S2.SetScale(S1.s());
        LA::AB(T12, S1, S2, work2[0]);
        S1.GetTranslation(work2[0]);
        T12.ApplyRotation(work2[0], work2[1]);
        T12.GetTranslation(work2[0]);
        work2[0] = ENFT_SSE::_mm_add_ps(work2[1], ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / S1.s()), work2[0]));
        S2.SetTranslation(work2[0]);
    }

    // X2 = R12 * X1 + t12
    //    = R12 * (s1 * R1 * X + s1 * t1) + t12
    //    = s1 * (R12*R1) * X + (s1*R12*t1 + t12)
    // X2 = R2 * X + t2 --> R2 = R12 * R1
    //                  --> t2 = R12 * t1 + 1/s1 * t12
    static inline void AccumulateTransformation(const SimilarityTransformation3D
            &S1, const RigidTransformation3D &T12, Camera &C2, ENFT_SSE::__m128 *work2) {
        LA::AB(T12, S1, C2, work2[0]);
        S1.GetTranslation(work2[0]);
        T12.ApplyRotation(work2[0], work2[1]);
        T12.GetTranslation(work2[0]);
        work2[0] = ENFT_SSE::_mm_add_ps(work2[1], ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / S1.s()), work2[0]));
        C2.SetTranslation(work2[0]);
    }

};

#endif