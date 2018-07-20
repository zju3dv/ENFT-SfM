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

#ifndef _RIGID_TRANSFORMATION_H_
#define _RIGID_TRANSFORMATION_H_

#include "RotationTransformation.h"
#include "LinearAlgebra/Matrix2x3.h"
#include "LinearAlgebra/Matrix4.h"
#include "Plane.h"

// x2 = R * x1 + t
class RigidTransformation2D : public RotationTransformation2D {

public:

    inline const LA::Vector2f &t() const {
        return m_t;
    }    inline LA::Vector2f &t() {
        return m_t;
    }
    inline const float &tx() const {
        return m_t.v0();
    }     inline float &tx() {
        return m_t.v0();
    }
    inline const float &ty() const {
        return m_t.v1();
    }     inline float &ty() {
        return m_t.v1();
    }
    inline void MakeIdentity() {
        RotationTransformation2D::MakeIdentity();
        m_t.SetZero();
    }
    inline const LA::Vector2f &GetTranslation() const {
        return m_t;
    }
    inline void SetRotation(const float &theta) {
        RotationTransformation2D::Set(theta);
    }
    inline void SetTranslation(const float &tx, const float &ty) {
        m_t.Set(tx, ty);
    }
    inline void SetTranslation(const Point2D &t) {
        m_t = t;
    }
    inline void Set(const float &theta, const float &tx, const float &ty) {
        SetRotation(theta);
        SetTranslation(tx, ty);
    }
    inline void Get(LA::AlignedMatrix2x3f &M) const {
        M.M00() = r00();
        M.M01() = r01();
        M.M02() = tx();
        M.M10() = r10();
        M.M11() = r11();
        M.M12() = ty();
    }
    inline void ApplyRotation(const Point2D &x, Point2D &Rx) const {
        RotationTransformation2D::Apply(x, Rx);
    }
    inline void ApplyRotationInversely(const Point2D &x, Point2D &RTx) const {
        RotationTransformation2D::ApplyInversely(x, RTx);
    }
    inline void Apply(const Point2D &x, Point2D &Tx) const {
        ApplyRotation(x, Tx);
        Tx += m_t;
    }

    // x2 = T * x1
    // sx2 = T' * sx1
    inline void ChangeScale(const float &s) {
        m_t *= s;
    }
    inline void ChangeScale(const float &sx, const float &sy) {
        tx() *= sx;
        ty() *= sy;
    }

    inline void Invert(RigidTransformation2D &Tinv) const {
        RotationTransformation2D::Invert(Tinv);
        Tinv.m_t.v0() = -(r00() * tx() + r10() * ty());
        Tinv.m_t.v1() = -(r01() * tx() + r11() * ty());
    }
    //inline void Invert(LA::AlignedMatrix2x3f &Tinv) const
    //{
    //  Tinv.M00() = r00();     Tinv.M01() = r10();     Tinv.M02() = -(r00() * tx() + r10() * ty());
    //  Tinv.M10() = r10();     Tinv.M11() = r11();     Tinv.M12() = -(r01() * tx() + r11() * ty());
    //}

    // x2 - c = R * (x1 - c) + t --> x2 = R * x1 - R * c + t + c
    // R' = R, t' = -R * c + t + c
    inline void CenterToCorner(const Point2D &c, ENFT_SSE::__m128 &work) {
        work = ENFT_SSE::_mm_mul_ps(M_00_01_10_11(), _mm_setr_ps(c.x(), c.y(), c.x(),
                                    c.y()));
        m_t.v0() += c.x() - (work.m128_f32[0] + work.m128_f32[1]);
        m_t.v1() += c.y() - (work.m128_f32[2] + work.m128_f32[3]);
    }

    inline void Print() const {
        printf("%f %f %f\n", r00(), r01(), tx());
        printf("%f %f %f\n", r10(), r11(), ty());
    }

protected:

    LA::Vector2f m_t;
    float m_reserve[2];

public:

    // x2 = R12 * x1 + t12 = R12 * (R1 * x + t1) + t12 = R12 * R1 * x + R12 * t1 + t12
    // R2 = R12 * R1, t2 = R12 * t1 + t12
    // T12 and T2 are allowed to be the same
    static inline void AccumulateTransformation(const RigidTransformation2D &T1,
            const RigidTransformation2D &T12, RigidTransformation2D &T2,
            ENFT_SSE::__m128 *work2) {
        work2[0] = ENFT_SSE::_mm_mul_ps(T12.r_00_01_10_11(), _mm_setr_ps(T1.tx(),
                                        T1.ty(),
                                        T1.tx(), T1.ty()));
        T2.tx() = T12.tx() + work2[0].m128_f32[0] + work2[0].m128_f32[1];
        T2.ty() = T12.ty() + work2[0].m128_f32[2] + work2[0].m128_f32[3];
        RotationTransformation2D::AccumulateTransformation(T1, T12, T2, work2);
    }

    // x2 = R * x1 + t
    // sx2 = R' * x1 + t'
    // R' = s * R12, t12' = s * t12
    // x1 = Rinv * x2 + tinv
    // x1 = Rinv' * sx2 + tinv'
    // Rinv' = Rinv / s, tinv' = tinv;
    static inline void ChangeScale2(const float &s, const float &sinv,
                                    RigidTransformation2D &T, RigidTransformation2D &Tinv,
                                    ENFT_SSE::__m128 *work0) {
        T.r_00_01_10_11() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(s), T.r_00_01_10_11());
        T.t() *= s;
        Tinv.r_00_01_10_11() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(sinv),
                               Tinv.r_00_01_10_11());
    }
};

class RigidTransformation3D : public RotationTransformation3D {

public:

    inline const ENFT_SSE::__m128 &r00_r01_r02_tX() const {
        return r_00_01_02_x();
    }      inline ENFT_SSE::__m128 &r00_r01_r02_tX() {
        return r_00_01_02_x();
    }
    inline const ENFT_SSE::__m128 &r10_r11_r12_tY() const {
        return r_10_11_12_x();
    }      inline ENFT_SSE::__m128 &r10_r11_r12_tY() {
        return r_10_11_12_x();
    }
    inline const ENFT_SSE::__m128 &r20_r21_r22_tZ() const {
        return r_20_21_22_x();
    }      inline ENFT_SSE::__m128 &r20_r21_r22_tZ() {
        return r_20_21_22_x();
    }
    inline const float &tX() const {
        return reserve0();
    }
    inline float &tX() {
        return reserve0();
    }
    inline const float &tY() const {
        return reserve1();
    }
    inline float &tY() {
        return reserve1();
    }
    inline const float &tZ() const {
        return reserve2();
    }
    inline float &tZ() {
        return reserve2();
    }
    inline void Set(const RotationTransformation3D &R,
                    const LA::AlignedVector3f &t) {
        memcpy(this, R, sizeof(RotationTransformation3D));
        SetTranslation(t);
    }
    inline void SetTranslation(const float &tX, const float &tY, const float &tZ) {
        reserve0() = tX;
        reserve1() = tY;
        reserve2() = tZ;
    }
    inline void SetTranslation(const LA::AlignedVector3f &t) {
        SetTranslation(t.v0(), t.v1(), t.v2());
    }
    inline void SetTranslation(const ENFT_SSE::__m128 &t) {
        SetTranslation(t.m128_f32[0], t.m128_f32[1], t.m128_f32[2]);
    }
    inline void SetTranslation(const LA::Vector3d &t) {
        tX() = float(t.v0());
        tY() = float(t.v1());
        tZ() = float(t.v2());
    }
    inline void GetCenter(Point3D &center) const {
        center.XYZx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_00_01_02_x,
                                             _mm_set1_ps(m_00_01_02_x.m128_f32[3])),
                                             ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_10_11_12_x,
                                                     _mm_set1_ps(m_10_11_12_x.m128_f32[3])),
                                                     ENFT_SSE::_mm_mul_ps(m_20_21_22_x, _mm_set1_ps(m_20_21_22_x.m128_f32[3]))));
        center.XYZx() = _mm_setr_ps(-center.X(), -center.Y(), -center.Z(), 1);
    }
    inline void GetCenter(ENFT_SSE::__m128 &center) const {
        center = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_00_01_02_x,
                                      _mm_set1_ps(m_00_01_02_x.m128_f32[3])),
                                      ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_10_11_12_x,
                                              _mm_set1_ps(m_10_11_12_x.m128_f32[3])),
                                              ENFT_SSE::_mm_mul_ps(m_20_21_22_x, _mm_set1_ps(m_20_21_22_x.m128_f32[3]))));
        center = _mm_setr_ps(-center.m128_f32[0], -center.m128_f32[1],
                             -center.m128_f32[2], 1);
    }
    inline void GetCenter(float *center) const {
        ENFT_SSE::__m128 tem;
        GetCenter(tem);
        for (int k = 0; k < 3; k++) {
            center[k] = tem.m128_f32[k];
        }
    }
    inline void GetCenter(double *center) const {
        ENFT_SSE::__m128 tem;
        GetCenter(tem);
        for (int k = 0; k < 3; k++) {
            center[k] = (double)tem.m128_f32[k];
        }
    }
    inline void SetCenter(const ENFT_SSE::__m128 &center) {
        tX() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_00_01_02_x, center));
        tY() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_10_11_12_x, center));
        tZ() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_20_21_22_x, center));
    }
    inline void SetCenter(const Point3D &center) {
        tX() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_00_01_02_x,
                                      center.XYZx()));
        tY() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_10_11_12_x,
                                      center.XYZx()));
        tZ() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_20_21_22_x,
                                      center.XYZx()));
    }
    inline void SetCenter(const double *center, ENFT_SSE::__m128 &work) {
        work = _mm_setr_ps(float(center[0]), float(center[1]), float(center[2]), 0);
        tX() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_00_01_02_x, work));
        tY() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_10_11_12_x, work));
        tZ() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_20_21_22_x, work));
    }
    inline void MakeIdentity() {
        memset(this, 0, 48);
        r00() = r11() = r22() = 1;
    }
    inline void GetTranslation(LA::Vector3f &t) const {
        t.Set(tX(), tY(), tZ());
    }
    inline void GetTranslation(LA::AlignedVector3f &t) const {
        t.Set(tX(), tY(), tZ());
    }
    inline void GetTranslation(ENFT_SSE::__m128 &t) const {
        t.m128_f32[0] = tX();
        t.m128_f32[1] = tY();
        t.m128_f32[2] = tZ();
        t.m128_f32[3] = 0;
    }
    inline void Scale(const float &s) {
        tX() *= s;
        tY() *= s;
        tZ() *= s;
    }
    inline void IncreaseTranslation(const Point3D &dt,
                                    RigidTransformation3D &T) const {
        T.tX() = tX() + dt.X();
        T.tY() = tY() + dt.Y();
        T.tZ() = tZ() + dt.Z();
    }
    inline void IncreaseTranslation(const Point3D &dt) {
        tX() += dt.X();
        tY() += dt.Y();
        tZ() += dt.Z();
    }
    inline void DecreaseTranslation(const Point3D &dt) {
        tX() -= dt.X();
        tY() -= dt.Y();
        tZ() -= dt.Z();
    }

    inline void Interpolate(const RigidTransformation3D &T1,
                            const RigidTransformation3D &T2, const float &alpha, ENFT_SSE::__m128 *work5) {
        ENFT_SSE::__m128 &a1 = work5[0], &a2 = work5[1], &v1 = work5[2], &v2 = work5[3],
                          &v = work5[4];
        a1 = _mm_set1_ps(alpha);
        a2 = _mm_set1_ps(1 - alpha);

        T1.ToQuaternion(&v1.m128_f32[0]);
        T2.ToQuaternion(&v2.m128_f32[0]);
        v = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(a1, v1), ENFT_SSE::_mm_mul_ps(a2,
                                 v2));
        v = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / sqrt(ENFT_SSE::SSE::Sum0123(
                ENFT_SSE::_mm_mul_ps(v, v)))), v);
        FromQuaternion(v, v1);

        T1.GetCenter(v1);
        T2.GetCenter(v2);
        v = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(a1, v1), ENFT_SSE::_mm_mul_ps(a2,
                                 v2));
        SetCenter(v);
    }

    inline void Apply(const Point3D &X, Point2D &TX) const {
#if _DEBUG
        assert(X.reserve() == 1);
#endif
        TX.x() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(),
                                        X.XYZx()));
        TX.y() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(),
                                        X.XYZx()));
    }
    inline void Apply(const Point3D &X, Point3D &TX) const {
#if _DEBUG
        assert(X.reserve() == 1);
#endif
        TX.X() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(),
                                        X.XYZx()));
        TX.Y() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(),
                                        X.XYZx()));
        TX.Z() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(),
                                        X.XYZx()));
    }
    inline void Apply(const ENFT_SSE::__m128 &X, Point3D &TX) const {
#if _DEBUG
        assert(X.m128_f32[3] == 1);
#endif
        TX.X() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(), X));
        TX.Y() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(), X));
        TX.Z() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(), X));
    }
    inline void Apply(const Point3D &X, ENFT_SSE::__m128 &TX) const {
#if _DEBUG
        assert(X.reserve() == 1);
#endif
        TX.m128_f32[0] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(),
                                                X.XYZx()));
        TX.m128_f32[1] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(),
                                                X.XYZx()));
        TX.m128_f32[2] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(),
                                                X.XYZx()));
    }
    inline void Apply(const ENFT_SSE::__m128 &X, ENFT_SSE::__m128 &TX) const {
#if _DEBUG
        assert(X.m128_f32[3] == 1);
#endif
        TX.m128_f32[0] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(),
                                                X));
        TX.m128_f32[1] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(),
                                                X));
        TX.m128_f32[2] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(),
                                                X));
        TX.m128_f32[3] = 1.0f;
    }
    inline void Apply(const Point3D &X, Point3D &RX, Point3D &TX) const {
#if _DEBUG
        assert(X.reserve() == 1);
#endif
        ENFT_SSE::SSE::Sum012_0123(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(), X.XYZx()),
                                   RX.X(), TX.X());
        ENFT_SSE::SSE::Sum012_0123(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(), X.XYZx()),
                                   RX.Y(), TX.Y());
        ENFT_SSE::SSE::Sum012_0123(ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(), X.XYZx()),
                                   RX.Z(), TX.Z());
    }
    inline void Apply(const ENFT_SSE::__m128 &X, ENFT_SSE::__m128 &RX,
                      ENFT_SSE::__m128 &TX) const {
#if _DEBUG
        assert(X.m128_f32[3] == 1);
#endif
        ENFT_SSE::SSE::Sum012_0123(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(), X),
                                   RX.m128_f32[0],
                                   TX.m128_f32[0]);
        ENFT_SSE::SSE::Sum012_0123(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(), X),
                                   RX.m128_f32[1],
                                   TX.m128_f32[1]);
        ENFT_SSE::SSE::Sum012_0123(ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(), X),
                                   RX.m128_f32[2],
                                   TX.m128_f32[2]);
    }
    inline void Apply(Point3D &X, ENFT_SSE::__m128 *work1) const {
#if _DEBUG
        assert(X.reserve() == 1);
#endif
        work1[0] = X.XYZx();
        //work1->m128_f32[3] = 1;
        Apply(work1[0], X);
    }
    inline void Apply(const Point3D &X, float &TX_X, float &TX_Y,
                      float &TX_Z) const {
#if _DEBUG
        assert(X.reserve() == 1);
#endif
        TX_X = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(), X.XYZx()));
        TX_Y = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(), X.XYZx()));
        TX_Z = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(), X.XYZx()));
    }
    inline void Apply(const ENFT_SSE::__m128 &X, float &TX_X, float &TX_Y,
                      float &TX_Z) const {
#if _DEBUG
        assert(X.m128_f32[3] == 1);
#endif
        TX_X = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(), X));
        TX_Y = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(), X));
        TX_Z = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(), X));
    }
    inline void Apply(const Point3D &X, float &RX_X, float &RX_Y, float &RX_Z,
                      float &T_X, float &T_Y, float &T_Z) const {
#if _DEBUG
        assert(X.reserve() == 1);
#endif
        ENFT_SSE::SSE::Sum012_0123(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(), X.XYZx()),
                                   RX_X, T_X);
        ENFT_SSE::SSE::Sum012_0123(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(), X.XYZx()),
                                   RX_Y, T_Y);
        ENFT_SSE::SSE::Sum012_0123(ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(), X.XYZx()),
                                   RX_Z, T_Z);
    }

    //     R2 * X2 + t2 = R1 * X1 + t1
    // ==> R2 * (R * X1 + t) + t2 = s(R1 * X1 + t1)
    // ==> R2 = R1 * R^T
    //     t2 = t1 - R2 * t
    // T1 and T2 are allowed to be the same
    inline void Apply(const RigidTransformation3D &T1, RigidTransformation3D &T2,
                      ENFT_SSE::__m128 *work3) const {
        T1.GetTranslation(work3[0]);
        LA::ABT(T1, *this, T2);
        GetTranslation(work3[1]);
        T2.ApplyRotation(work3[1], work3[2]);
        work3[0] = ENFT_SSE::_mm_sub_ps(work3[0], work3[2]);
        T2.SetTranslation(work3[0]);
    }

    inline void Apply(const Plane &P1, Plane &P2, ENFT_SSE::__m128 *work1) const {
        RotationTransformation3D::Apply(P1.ABCD(), work1[0]);
        P2.A() = work1[0].m128_f32[0];
        P2.B() = work1[0].m128_f32[1];
        P2.C() = work1[0].m128_f32[2];
        P2.D() = P1.D() - P2.A() * tX() - P2.B() * tY() - P2.C() * tZ();
    }

    inline float ApplyZ(const Point3D &X) const {
#if _DEBUG
        assert(X.reserve() == 1);
#endif
        return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(), X.XYZx()));
    }
    inline void ApplyInversely(Point3D &X) const {
        X.XYZx() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(),
                                        _mm_set1_ps(X.X() - tX())),
                                        ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), _mm_set1_ps(X.Y() - tY()))),
                                        ENFT_SSE::_mm_mul_ps(r_20_21_22_x(), _mm_set1_ps(X.Z() - tZ())));
        X.reserve() = 1;
    }
    inline void ApplyInversely(const Point3D &TX, Point3D &X) const {
        X.XYZx() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(),
                                        _mm_set1_ps(TX.X() - tX())),
                                        ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), _mm_set1_ps(TX.Y() - tY()))),
                                        ENFT_SSE::_mm_mul_ps(r_20_21_22_x(), _mm_set1_ps(TX.Z() - tZ())));
        X.reserve() = 1;
    }
    inline void ApplyRotation(const Point3D &X, float &RX_X, float &RX_Y,
                              float &RX_Z) const {
        RotationTransformation3D::Apply(X, RX_X, RX_Y, RX_Z);
    }
    inline void ApplyRotation(const ENFT_SSE::__m128 &X, float &RX_X, float &RX_Y,
                              float &RX_Z) const {
        RotationTransformation3D::Apply(X, RX_X, RX_Y, RX_Z);
    }
    inline void ApplyRotation(const Point3D &X, Point3D &RX) const {
        RotationTransformation3D::Apply(X, RX);
    }
    inline void ApplyRotation(const ENFT_SSE::__m128 &X,
                              ENFT_SSE::__m128 &RX) const {
        RotationTransformation3D::Apply(X, RX);
    }
    inline void ApplyRotation(const RotationTransformation3D &R1,
                              RotationTransformation3D &R2) const {
        RotationTransformation3D::Apply(R1, R2);
    }
    inline void ApplyRotation(const Point3DCovariance &P, Point3DCovariance &RPRT,
                              ENFT_SSE::__m128 *work4) const {
        RotationTransformation3D::Apply(P, RPRT, work4);
    }
    inline void ApplyRotationInversely(const Point2D &RX, Point3D &X) const {
        RotationTransformation3D::ApplyInversely(RX, X);
    }
    inline void ApplyRotationInversely(Point3D &X) const {
        RotationTransformation3D::ApplyInversely(X);
    }
    inline void ApplyRotationInversely(const Point3DCovariance &RPRT,
                                       Point3DCovariance &P, RotationTransformation3D &RT,
                                       ENFT_SSE::__m128 *work4) const {
        RotationTransformation3D::ApplyInversely(RPRT, P, RT, work4);
    }

    //inline void GetTranspose(LA::Matrix4f &RT) const
    //{
    //  RT.M00() = r00();   RT.M01() = r10();   RT.M02() = r20();   RT.M03() = 0;
    //  RT.M10() = r01();   RT.M11() = r11();   RT.M12() = r21();   RT.M13() = 0;
    //  RT.M20() = r02();   RT.M21() = r12();   RT.M22() = r22();   RT.M23() = 0;
    //  RT.M30() = tX();    RT.M31() = tY();    RT.M32() = tZ();    RT.M33() = 1.0f;
    //}
    inline void GetRotationTranspose(RotationTransformation3D &RT) const {
        RotationTransformation3D::GetTranspose(RT);
    }
    inline void GetTranspose(LA::Matrix4f &TT) const {
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
    }

    inline void LeftMultiplyRotation(const RotationTransformation3D &Rl,
                                     RotationTransformation3D &RlxR, ENFT_SSE::__m128 &work) const {
        RotationTransformation3D::LeftMultiply(Rl, RlxR, work);
    }
    inline void TransposeTimes(const Point3D &X, Point3D &TTX) const {
        TTX.XYZx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(),
                                          _mm_set1_ps(X.X())),
                                          ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(), _mm_set1_ps(X.Y())),
                                                  ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(), _mm_set1_ps(X.Z()))));
        TTX.XYZx() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / TTX.reserve()), TTX.XYZx());
    }
    inline void TransposeTimes(const ENFT_SSE::__m128 &X,
                               ENFT_SSE::__m128 &TTX) const {
        TTX = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(r00_r01_r02_tX(),
                                   _mm_set1_ps(X.m128_f32[0])),
                                   ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(r10_r11_r12_tY(),
                                           _mm_set1_ps(X.m128_f32[1])),
                                           ENFT_SSE::_mm_mul_ps(r20_r21_r22_tZ(), _mm_set1_ps(X.m128_f32[2]))));
        TTX = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / TTX.m128_f32[3]), TTX);
    }
    inline void Invert(ENFT_SSE::__m128 *work2) {
        GetTranslation(work2[0]);
        Transpose();
        ApplyRotation(work2[0], work2[1]);
        tX() = -work2[1].m128_f32[0];
        tY() = -work2[1].m128_f32[1];
        tZ() = -work2[1].m128_f32[2];
    }
    inline void Invert(RigidTransformation3D &TI, ENFT_SSE::__m128 &work) const {
        GetTranslation(work);
        LA::AlignedMatrix3f::GetTranspose(TI);
        TI.ApplyRotation(work, TI.tX(), TI.tY(), TI.tZ());
        TI.tX() = -TI.tX();
        TI.tY() = -TI.tY();
        TI.tZ() = -TI.tZ();
    }

    inline void ChangeReference(const RigidTransformation3D &Tref,
                                ENFT_SSE::__m128 *work2) {
        Tref.GetTranslation(work2[0]);

        work2[1].m128_f32[0] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   r_00_01_02_x(),
                                   Tref.r_00_01_02_x()));
        work2[1].m128_f32[1] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   r_00_01_02_x(),
                                   Tref.r_10_11_12_x()));
        work2[1].m128_f32[2] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   r_00_01_02_x(),
                                   Tref.r_20_21_22_x()));
        work2[1].m128_f32[3] = tX() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   work2[1], work2[0]));
        r00_r01_r02_tX() = work2[1];

        work2[1].m128_f32[0] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   r_10_11_12_x(),
                                   Tref.r_00_01_02_x()));
        work2[1].m128_f32[1] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   r_10_11_12_x(),
                                   Tref.r_10_11_12_x()));
        work2[1].m128_f32[2] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   r_10_11_12_x(),
                                   Tref.r_20_21_22_x()));
        work2[1].m128_f32[3] = tY() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   work2[1], work2[0]));
        r10_r11_r12_tY() = work2[1];

        work2[1].m128_f32[0] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   r_20_21_22_x(),
                                   Tref.r_00_01_02_x()));
        work2[1].m128_f32[1] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   r_20_21_22_x(),
                                   Tref.r_10_11_12_x()));
        work2[1].m128_f32[2] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   r_20_21_22_x(),
                                   Tref.r_20_21_22_x()));
        work2[1].m128_f32[3] = tZ() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                                   work2[1], work2[0]));
        r20_r21_r22_tZ() = work2[1];
    }
    inline void Print() const {
        //printf("R = [ %f %f %f ]\n", r00(), r01(), r02());
        //printf("    [ %f %f %f ]\n", r10(), r11(), r12());
        //printf("    [ %f %f %f ]\n", r20(), r21(), r22());
        //float w[3];
        //ToRodrigues(w);
        //printf("w = [ %f %f %f ]\n", w[0], w[1], w[2]);
        //printf("t = [ %f %f %f ]\n", tX(), tY(), tZ());
        printf("%f %f %f %f\n", r00(), r01(), r02(), tX());
        printf("%f %f %f %f\n", r10(), r11(), r12(), tY());
        printf("%f %f %f %f\n", r20(), r21(), r22(), tZ());
    }

    inline void Load(FILE *fp) {
        fscanf(fp, "%f %f %f %f", &r00(), &r01(), &r02(), &tX());
        fscanf(fp, "%f %f %f %f", &r10(), &r11(), &r12(), &tY());
        fscanf(fp, "%f %f %f %f", &r20(), &r21(), &r22(), &tZ());
    }
    inline void Load(const char *fileName) {

        FILE *fp = fopen( fileName, "r");
        Load(fp);
        fclose(fp);
    }
    inline void Save(FILE *fp) const {
        fprintf(fp, "%f %f %f %f\n", r00(), r01(), r02(), tX());
        fprintf(fp, "%f %f %f %f\n", r10(), r11(), r12(), tY());
        fprintf(fp, "%f %f %f %f\n", r20(), r21(), r22(), tZ());
    }
    inline void Save(const char *fileName) const {

        FILE *fp = fopen( fileName, "w");
        Save(fp);
        fclose(fp);
    }
    inline void SaveB(FILE *fp) const {
        fwrite(this, sizeof(RigidTransformation3D), 1, fp);
    }
    inline void LoadB(FILE *fp)       {
        fread (this, sizeof(RigidTransformation3D), 1, fp);
    }

    // X2 = R12 * X1 + t12
    //    = R12 * (R1*X+t1) + t12
    //    = (R12*R1) * X + (R12*t1+t12)
    // X2 = R2 * X + t2
    // R2 = R12*R1      --> R12 = R2 * R1^T
    // t2 = R12*t1+t12  --> t12 = t2 - R12*t1
    // T2 and T12 are allowed to be the same
    static inline void ComputeRelativeTransformation(const RigidTransformation3D
            &T1, const RigidTransformation3D &T2, RigidTransformation3D &T12,
            ENFT_SSE::__m128 &work) {
        RotationTransformation3D::ComputeRelativeTransformation(T1, T2, T12);
        work = _mm_setr_ps(T1.tX(), T1.tY(), T1.tZ(), 0);
        T12.tX() = T2.tX() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                       T12.r_00_01_02_x(), work));
        T12.tY() = T2.tY() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                       T12.r_10_11_12_x(), work));
        T12.tZ() = T2.tZ() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(
                       T12.r_20_21_22_x(), work));
    }

    // X2 = R12 * X1 + t12
    //    = R12 * (R1*X+t1) + t12
    //    = (R12*R1) * X + (R12*t1+t12)
    // X2 = R2 * X + t2 --> R2 = R12*R1
    //                  --> t2 = R12*t1+t12
    static inline void AccumulateTransformation(const RigidTransformation3D &T1,
            const RigidTransformation3D &T12, RigidTransformation3D &T2,
            ENFT_SSE::__m128 &work) {
        LA::AB(T12, T1, T2, work);
        work = _mm_setr_ps(T1.tX(), T1.tY(), T1.tZ(), 0);
        T2.tX() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T12.r_00_01_02_x(),
                                        work)) + T12.tX();
        T2.tY() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T12.r_10_11_12_x(),
                                        work)) + T12.tY();
        T2.tZ() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T12.r_20_21_22_x(),
                                        work)) + T12.tZ();
    }
    static inline void AccumulateTransformationScaled(const RigidTransformation3D
            &T1, const RigidTransformation3D &T12, const float &s,
            RigidTransformation3D &T2,
            ENFT_SSE::__m128 &work) {
        LA::AB(T12, T1, T2, work);
        work = _mm_setr_ps(T1.tX(), T1.tY(), T1.tZ(), 0);
        T2.tX() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T12.r_00_01_02_x(),
                                        work)) + T12.tX() * s;
        T2.tY() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T12.r_10_11_12_x(),
                                        work)) + T12.tY() * s;
        T2.tZ() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(T12.r_20_21_22_x(),
                                        work)) + T12.tZ() * s;
    }

};

#endif