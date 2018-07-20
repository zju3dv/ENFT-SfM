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

#ifndef _POINT_H_
#define _POINT_H_

#include "LinearAlgebra/Vector2.h"
#include "LinearAlgebra/Vector3.h"
#include "LinearAlgebra/Matrix3.h"

class Point3D : public LA::AlignedVector3f {

  public:

    inline Point3D() {
        reserve() = 1;
    }
    inline Point3D(const float &X, const float &Y,
                   const float &Z) : LA::AlignedVector3f(X, Y, Z) {
        reserve() = 1;
    }

    inline void Set(const float &X, const float &Y, const float &Z) {
        m_v012x = _mm_setr_ps(X, Y, Z, 1.0f);
    }
    inline void Set(const float *X) {
        m_v012x = _mm_setr_ps(X[0], X[1], X[2], 1.0f);
    }
    inline void Set(const double *X) {
        m_v012x = _mm_setr_ps(float(X[0]), float(X[1]), float(X[2]), 1.0f);
    }
    inline void Get(float *X) const {
        X[0] = m_v012x.m128_f32[0];
        X[1] = m_v012x.m128_f32[1];
        X[2] = m_v012x.m128_f32[2];
    }
    inline void Get(double *X) const {
        X[0] = double(m_v012x.m128_f32[0]);
        X[1] = double(m_v012x.m128_f32[1]);
        X[2] = double(m_v012x.m128_f32[2]);
    }
    inline const float &X() const {
        return v0();
    }          inline float &X() {
        return v0();
    }
    inline const float &Y() const {
        return v1();
    }          inline float &Y() {
        return v1();
    }
    inline const float &Z() const {
        return v2();
    }          inline float &Z() {
        return v2();
    }
    inline const ENFT_SSE::__m128 &XYZx() const {
        return v012x();
    }   inline ENFT_SSE::__m128 &XYZx() {
        return v012x();
    }
    inline void operator = (const AlignedVector3f &v) {
        m_v012x = v.v012x();
    }
    inline void Invalidate() {
        reserve() = FLT_MAX;
    }
    inline bool IsInvalid() const {
        return reserve() != 1;
    }
    inline void Validate() {
        reserve() = 1;
    }
    inline bool IsValid() const {
        return reserve() == 1;
    }
    inline void Save(FILE *fp) const {
        fprintf(fp, "%f %f %f\n", X(), Y(), Z());
    }
    inline void Load(FILE *fp) {
        fscanf(fp, "%f %f %f", &X(), &Y(), &Z());
        reserve() = 1;
    }
    inline void SaveB(FILE *fp) const {
        fwrite(this, sizeof(Point3D), 1, fp);
    }
    inline void LoadB(FILE *fp) {
        fread(this, sizeof(Point3D), 1, fp);
    }
};

class Point3DCovariance : public LA::SymmetricMatrix3f {

  public:

    inline const float &cXX() const {
        return M00();
    }       inline float &cXX() {
        return M00();
    }
    inline const float &cXY() const {
        return M01();
    }       inline float &cXY() {
        return M01();
    }
    inline const float &cXZ() const {
        return M02();
    }       inline float &cXZ() {
        return M02();
    }
    inline const float &cYX() const {
        return M10();
    }       inline float &cYX() {
        return M10();
    }
    inline const float &cYY() const {
        return M11();
    }       inline float &cYY() {
        return M11();
    }
    inline const float &cYZ() const {
        return M12();
    }       inline float &cYZ() {
        return M12();
    }
    inline const float &cZX() const {
        return M20();
    }       inline float &cZX() {
        return M20();
    }
    inline const float &cZY() const {
        return M21();
    }       inline float &cZY() {
        return M21();
    }
    inline const float &cZZ() const {
        return M22();
    }       inline float &cZZ() {
        return M22();
    }

    inline void Get(LA::AlignedMatrix3f &P) const {
        P.M_00_01_02_x() = _mm_setr_ps(M00(), M01(), M02(), 0.0f);
        P.M_10_11_12_x() = _mm_setr_ps(M10(), M11(), M12(), 0.0f);
        P.M_20_21_22_x() = _mm_setr_ps(M20(), M21(), M22(), 0.0f);
    }

    inline bool Invert(Point3DCovariance &Pinv) const {
        return LA::InvertSymmetricUpper(*this, Pinv);
    }

    static inline float ComputeSqauredMahalanobisDistance(const Point3DCovariance
            &Pinv, const Point3D &dX) {
        return dX.X() * (Pinv.cXX() * dX.X() + Pinv.cYX() * dX.Y() + Pinv.cZX() *
                         dX.Z())
               + dX.Y() * (Pinv.cXY() * dX.X() + Pinv.cYY() * dX.Y() + Pinv.cZY() * dX.Z())
               + dX.Z() * (Pinv.cXZ() * dX.X() + Pinv.cYZ() * dX.Y() + Pinv.cZZ() * dX.Z());
    }
};

class Point2D : public LA::Vector2f {

  public:

    Point2D() {}
    Point2D(const Point2D &x) : LA::Vector2f(x) {}
    Point2D(const float &x, const float &y) : LA::Vector2f(x, y) {}

    inline void Set(const float &x, const float &y) {
        m_v0 = x;
        m_v1 = y;
    }
    inline void Set(const float *x) {
        m_v0 = x[0];
        m_v1 = x[1];
    }
    inline void Get(float *x) const {
        x[0] = m_v0;
        x[1] = m_v1;
    }
    inline void Get(ushort &x, ushort &y) const {
        x = ushort(m_v0);
        y = ushort(m_v1);
    }
    inline const float &x() const {
        return v0();
    }          inline float &x() {
        return v0();
    }
    inline const float &y() const {
        return v1();
    }          inline float &y() {
        return v1();
    }

    inline void Scale(const Point2D &s) {
        x() *= s.x();
        y() *= s.y();
    }
    inline void Scale(const float &sx, const float &sy) {
        x() *= sx;
        y() *= sy;
    }
    inline void Save(FILE *fp) const {
        fprintf(fp, "%f %f\n", x(), y());
    }
    inline void Load(FILE *fp) {
        fscanf(fp, "%f %f", &x(), &y());
    }

    inline void Invalidate() {
        x() = FLT_MAX;
    }
    inline bool IsValid() const {
        return x() != FLT_MAX;
    }
    inline bool IsInvalid() const {
        return x() == FLT_MAX;
    }

};

class Point2DCovariance : public LA::Vector3f {

  public:

    inline const float &cxx() const {
        return v0();
    }        inline float &cxx() {
        return v0();
    }
    inline const float &cxy() const {
        return v1();
    }        inline float &cxy() {
        return v1();
    }
    inline const float &cyx() const {
        return v1();
    }        inline float &cyx() {
        return v1();
    }
    inline const float &cyy() const {
        return v2();
    }        inline float &cyy() {
        return v2();
    }
    inline float Determinant() const {
        return cxx() * cyy() - cxy() * cxy();
    }

    inline bool Invert(Point2DCovariance &Pinv) const {
        if((Pinv.cyy() = Determinant()) == 0.0f)
            return false;
        Pinv.cyy() = 1 / Pinv.cyy();
        Pinv.cxx() = Pinv.cyy() * cyy();
        Pinv.cxy() = -Pinv.cyy() * cxy();
        Pinv.cyy() = Pinv.cyy() * cxx();
        return true;
    }

    static inline float ComputeMahalanobisDistance(const Point2DCovariance &Pinv,
            const Point2D &dx) {
        return dx.x() * (Pinv.cxx() * dx.x() + Pinv.cyx() * dx.y())
               + dx.y() * (Pinv.cxy() * dx.x() + Pinv.cyy() * dx.y());
    }

};

#endif