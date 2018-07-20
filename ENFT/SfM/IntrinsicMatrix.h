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

#ifndef _INTRINSIC_MATRIX_H_
#define _INTRINSIC_MATRIX_H_

#include "Utility/SSE.h"
#include "LinearAlgebra/Matrix3.h"
#include "LinearAlgebra/Matrix3x4.h"
#include "Point.h"
#include "Utility/AlignedVector.h"

class IntrinsicMatrix {

  public:

    class Parameter {
      public:
        inline const float &fx() const {
            return m_fx;
        }     inline float &fx() {
            return m_fx;
        }
        inline const float &fy() const {
            return m_fy;
        }     inline float &fy() {
            return m_fy;
        }
        inline const float &cx() const {
            return m_cx;
        }     inline float &cx() {
            return m_cx;
        }
        inline const float &cy() const {
            return m_cy;
        }     inline float &cy() {
            return m_cy;
        }
        inline const float &r () const {
            return m_r ;
        }     inline float &r () {
            return m_r ;
        }
        inline const float &one_over_fx () const {
            return m_one_over_fx ;
        }     inline float &one_over_fx () {
            return m_one_over_fx ;
        }
        inline const float &one_over_fy () const {
            return m_one_over_fy ;
        }     inline float &one_over_fy () {
            return m_one_over_fy ;
        }
        inline const float &one_over_fxy() const {
            return m_one_over_fxy;
        }     inline float &one_over_fxy() {
            return m_one_over_fxy;
        }
      protected:
        float m_fx, m_fy, m_cx, m_cy, m_r, m_one_over_fx, m_one_over_fy, m_one_over_fxy;
    };

    IntrinsicMatrix() {}
    IntrinsicMatrix(const float &fx, const float &fy, const float &cx,
                    const float &cy) {
        Set(fx, fy, cx, cy);
    }

    inline const float &fx() const {
        return m_fx_fy_fx_fy.m128_f32[0];
    }
    inline const float &fy() const {
        return m_fx_fy_fx_fy.m128_f32[1];
    }
    inline const float &cx() const {
        return m_cx_cy_cx_cy.m128_f32[0];
    }
    inline const float &cy() const {
        return m_cx_cy_cx_cy.m128_f32[1];
    }
    inline const float &one_over_fx () const {
        return m_one_over_fx_fy_fx_fy.m128_f32[0] ;
    }
    inline const float &one_over_fy () const {
        return m_one_over_fx_fy_fx_fy.m128_f32[1] ;
    }
    inline const float &fxy()           const {
        return m__fxy__one_over_fxy__cx_over_fx__cy_over_fy.m128_f32[0];
    }
    inline const float &one_over_fxy()  const {
        return m__fxy__one_over_fxy__cx_over_fx__cy_over_fy.m128_f32[1];
    }
    inline const float &cx_over_fx()    const {
        return m__fxy__one_over_fxy__cx_over_fx__cy_over_fy.m128_f32[2];
    }
    inline const float &cy_over_fy()    const {
        return m__fxy__one_over_fxy__cx_over_fx__cy_over_fy.m128_f32[3];
    }

    inline void Set(const float &fx, const float &fy, const float &cx,
                    const float &cy) {
        m_fx_fy_fx_fy = _mm_setr_ps(fx, fy, fx, fy);
        m_cx_cy_cx_cy = _mm_setr_ps(cx, cy, cx, cy);
        m_cx_cx_cy_cy = _mm_setr_ps(cx, cx, cy, cy);
        m_one_over_fx_fy_fx_fy = ENFT_SSE::_mm_div_ps(_mm_set1_ps(1), m_fx_fy_fx_fy);

        m_fx_fx_fx_fx = _mm_set1_ps(fx);
        m_fy_fy_fy_fy = _mm_set1_ps(fy);
        m_cx_cx_cx_cx = _mm_set1_ps(cx);
        m_cy_cy_cy_cy = _mm_set1_ps(cy);
        m_one_over_fx_fx_fx_fx = _mm_set1_ps(m_one_over_fx_fy_fx_fy.m128_f32[0]);
        m_one_over_fy_fy_fy_fy = _mm_set1_ps(m_one_over_fx_fy_fx_fy.m128_f32[1]);

        const float fxy = fx * fy;
        m__fxy__one_over_fxy__cx_over_fx__cy_over_fy = _mm_setr_ps(fxy, 1 / fxy,
                cx * m_one_over_fx_fy_fx_fy.m128_f32[0],
                cy * m_one_over_fx_fy_fx_fy.m128_f32[1]);
    }
    inline void Get(float &fx, float &fy, float &cx, float &cy) const {
        fx = this->fx();
        fy = this->fy();
        cx = this->cx();
        cy = this->cy();
    }
    inline void Get(LA::AlignedMatrix3f &K) const {
        K.M00() = fx();
        K.M01() = 0;
        K.M02() = cx();
        K.M10() = 0;
        K.M11() = fy();
        K.M12() = cy();
        K.M20() = 0;
        K.M21() = 0;
        K.M22() = 1;
    }
    inline void GetInverse(LA::AlignedMatrix3f &K) const {
        K.M00() = one_over_fx();
        K.M01() = 0;
        K.M02() = -cx_over_fx();
        K.M10() = 0;
        K.M11() = one_over_fy();
        K.M12() = -cy_over_fy();
        K.M20() = 0;
        K.M21() = 0;
        K.M22() = 1;
    }
    inline void GetTranspose(LA::AlignedMatrix3f &KT) const {
        KT.M00() = fx();
        KT.M01() = 0;
        KT.M02() = 0;
        KT.M10() = 0;
        KT.M11() = fy();
        KT.M12() = 0;
        KT.M20() = cx();
        KT.M21() = cy();
        KT.M22() = 1;
    }
    inline void GetTransposeInverse(LA::AlignedMatrix3f &KTinv) const {
        KTinv.M00() = one_over_fx();
        KTinv.M01() = 0;
        KTinv.M02() = 0;
        KTinv.M10() = 0;
        KTinv.M11() = one_over_fy();
        KTinv.M12() = 0;
        KTinv.M20() = -cx_over_fx();
        KTinv.M21() = -cy_over_fy();
        KTinv.M22() = 1;
    }
    inline void GetInverseTranspose(LA::AlignedMatrix3f &KinvT) const {
        GetTransposeInverse(KinvT);
    }
    inline void Invert(IntrinsicMatrix &Kinv) const {
        Kinv.Set(one_over_fx(), one_over_fy(), -cx_over_fx(), -cy_over_fy());
    }

    inline void ImageToNormalizedPlane(Point2D &p) const {
        p.x() = (p.x() - cx()) * one_over_fx();
        p.y() = (p.y() - cy()) * one_over_fy();
    }
    inline void ImageToNormalizedPlane(const Point2D &src, Point2D &dst) const {
        dst.x() = (src.x() - cx()) * one_over_fx();
        dst.y() = (src.y() - cy()) * one_over_fy();
    }
    inline void ImageToNormalizedPlane2(ENFT_SSE::__m128 &p2) const {
        p2 = ENFT_SSE::_mm_mul_ps(_mm_sub_ps(p2, m_cx_cy_cx_cy), m_one_over_fx_fy_fx_fy);
    }
    inline void ImageToNormalizedPlane2(const ENFT_SSE::__m128 &src2, ENFT_SSE::__m128 &dst2) const {
        dst2 = ENFT_SSE::_mm_mul_ps(_mm_sub_ps(src2, m_cx_cy_cx_cy), m_one_over_fx_fy_fx_fy);
    }
    inline void ImageToNormalizedPlane4(ENFT_SSE::__m128 &x, ENFT_SSE::__m128 &y) const {
        x = ENFT_SSE::_mm_mul_ps(_mm_sub_ps(x, m_cx_cx_cx_cx), m_one_over_fx_fx_fx_fx);
        y = ENFT_SSE::_mm_mul_ps(_mm_sub_ps(y, m_cy_cy_cy_cy), m_one_over_fy_fy_fy_fy);
    }
    inline void ImageToNormalizedPlaneN(AlignedVector<Point2D> &xs) const {
        const uint N = uint(xs.Size()), _N = N - (N & 1);
        ENFT_SSE::__m128 *p2 = (ENFT_SSE::__m128 *) xs.Data();
        for(uint i = 0; i < _N; i += 2, ++p2)
            ImageToNormalizedPlane2(*p2);
        if(_N != N)
            ImageToNormalizedPlane(xs[_N]);
    }
    inline void ImageToNormalizedPlaneN(const AlignedVector<Point2D> &src,
                                        AlignedVector<Point2D> &dst) const {
        const uint N = uint(src.Size()), _N = N - (N & 1);
        dst.Resize(N);
        const ENFT_SSE::__m128 *pSrc2 = (const ENFT_SSE::__m128 *) src.Data();
        ENFT_SSE::__m128 *pDst2 = (ENFT_SSE::__m128 *) dst.Data();
        for(uint i = 0; i < _N; i += 2, ++pSrc2, ++pDst2)
            ImageToNormalizedPlane2(*pSrc2, *pDst2);
        if(_N != N)
            ImageToNormalizedPlane(src[_N], dst[_N]);
    }
    inline void ImageToNormalizedPlaneN(const uint &N, const Point2D *src,
                                        Point2D *dst) const {
        const uint _N = N - (N & 1);
        const ENFT_SSE::__m128 *pSrc2 = (const ENFT_SSE::__m128 *) src;
        ENFT_SSE::__m128 *pDst2 = (ENFT_SSE::__m128 *) dst;
        for(uint i = 0; i < _N; i += 2, ++pSrc2, ++pDst2)
            ImageToNormalizedPlane2(*pSrc2, *pDst2);
        if(_N != N)
            ImageToNormalizedPlane(src[_N], dst[_N]);
    }
    inline void ImageToNormalizedPlaneN(const uint &N, Point2D *xs) const {
        const uint _N = N - (N & 1);
        ENFT_SSE::__m128 *p2 = (ENFT_SSE::__m128 *) xs;
        for(uint i = 0; i < _N; i += 2, ++p2)
            ImageToNormalizedPlane2(*p2);
        if(_N != N)
            ImageToNormalizedPlane(xs[_N]);
    }

    inline void NormalizedPlaneToImage(Point2D &p) const {
        p.x() = fx() * p.x() + cx();
        p.y() = fy() * p.y() + cy();
    }
    inline void NormalizedPlaneToImage(const Point2D &src, Point2D &dst) const {
        dst.x() = fx() * src.x() + cx();
        dst.y() = fy() * src.y() + cy();
    }
    inline void NormalizedPlaneToImage2(const ENFT_SSE::__m128 &src2, ENFT_SSE::__m128 &dst2) const {
        dst2 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_fx_fy_fx_fy, src2), m_cx_cy_cx_cy);
    }
    inline void NormalizedPlaneToImage2(ENFT_SSE::__m128 &p2) const {
        p2 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_fx_fy_fx_fy, p2), m_cx_cy_cx_cy);
    }
    inline void NormalizedPlaneToImage4(ENFT_SSE::__m128 &x, ENFT_SSE::__m128 &y) const {
        x = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_fx_fx_fx_fx, x), m_cx_cx_cx_cx);
        y = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_fy_fy_fy_fy, y), m_cy_cy_cy_cy);
    }
    inline void NormalizedPlaneToImageN(AlignedVector<Point2D> &xs) const {
        const uint N = uint(xs.Size()), _N = N - (N & 1);
        ENFT_SSE::__m128 *p2 = (ENFT_SSE::__m128 *) xs.Data();
        for(uint i = 0; i < _N; i += 2, ++p2)
            NormalizedPlaneToImage2(*p2);
        if(_N != N)
            NormalizedPlaneToImage(xs[_N]);
    }
    inline void NormalizedPlaneToImageN(const AlignedVector<Point2D> &src,
                                        AlignedVector<Point2D> &dst) const {
        const uint N = uint(src.Size()), _N = N - (N & 1);
        dst.Resize(N);
        const ENFT_SSE::__m128 *pSrc2 = (const ENFT_SSE::__m128 *) src.Data();
        ENFT_SSE::__m128 *pDst2 = (ENFT_SSE::__m128 *) dst.Data();
        for(uint i = 0; i < _N; i += 2, ++pSrc2, ++pDst2)
            NormalizedPlaneToImage2(*pSrc2, *pDst2);
        if(_N != N)
            NormalizedPlaneToImage(src[_N], dst[_N]);
    }
    inline void NormalizedPlaneToImageN(const uint &N, const Point2D *src,
                                        Point2D *dst) const {
        const uint _N = N - (N & 1);
        const ENFT_SSE::__m128 *pSrc2 = (const ENFT_SSE::__m128 *) src;
        ENFT_SSE::__m128 *pDst2 = (ENFT_SSE::__m128 *) dst;
        for(uint i = 0; i < _N; i += 2, ++pSrc2, ++pDst2)
            NormalizedPlaneToImage2(*pSrc2, *pDst2);
        if(_N != N)
            NormalizedPlaneToImage(src[_N], dst[_N]);
    }
    inline void NormalizedPlaneToImageN(const uint &N, Point2D *xs) const {
        const uint _N = N - (N & 1);
        ENFT_SSE::__m128 *p2 = (ENFT_SSE::__m128 *) xs;
        for(uint i = 0; i < _N; i += 2, ++p2)
            NormalizedPlaneToImage2(*p2);
        if(_N != N)
            NormalizedPlaneToImage(xs[_N]);
    }

    inline void ImageCoordinateCornerToCenter(Point2D &p2) const {
        p2.x() = p2.x() - cx();
        p2.y() = p2.y() - cy();
    }
    inline void ImageCoordinateCornerToCenter2(ENFT_SSE::__m128 &p2) const {
        p2 = ENFT_SSE::_mm_sub_ps(p2, m_cx_cy_cx_cy);
    }
    inline void ImageCoordinateCornerToCenterN(AlignedVector<Point2D> &xs) const {
        const uint N = uint(xs.Size()), _N = N - (N & 1);
        ENFT_SSE::__m128 *p2 = (ENFT_SSE::__m128 *) xs.Data();
        for(uint i = 0; i < _N; i += 2, ++p2)
            ImageCoordinateCornerToCenter2(*p2);
        if(_N != N)
            ImageCoordinateCornerToCenter(xs[_N]);
    }

    inline void Load(FILE *fp) {
        float fx, fy, cx, cy;
        //float tmp;
        //fscanf(fp, "%f %f %f", &fx, &tmp, &cx);
        //fscanf(fp, "%f %f %f", &tmp, &fy, &cy);
        //fscanf(fp, "%f %f %f", &tmp, &tmp, &tmp);
        fscanf(fp, "%f %f %f %f", &fx, &fy, &cx, &cy);
        Set(fx, fy, cx, cy);
    }
    inline bool Load(const char *fileName) {
		FILE *fp = fopen(fileName, "r");
        if(fp == nullptr)
            return false;
        Load(fp);
        fclose(fp);
        return true;
    }
    inline void Save(FILE *fp) const {
        //fprintf(fp, "%f 0 %f\n", fx(), 0, cx());
        //fprintf(fp, "0 %f %f\n", 0, fy(), cy());
        //fprintf(fp, "0 0 1\n");
        fprintf(fp, "%f %f %f %f\n", fx(), fy(), cx(), cy());
    }
    inline void Save(const char *fileName) const {
        
        FILE *fp = fopen( fileName, "w");
        Save(fp);
        fclose(fp);
    }
    inline void SaveB(FILE *fp) const {
        fwrite(this, sizeof(IntrinsicMatrix), 1, fp);
    }
    inline void LoadB(FILE *fp)       {
        fread (this, sizeof(IntrinsicMatrix), 1, fp);
    }
    inline void SaveActb(FILE *fp) const {
        double buf[] = {double(fx()), double(fy()), double(cx()), double(cy()), 0, 1};
        fwrite(buf, sizeof(buf), 1, fp);
    }
    inline void LoadActb(FILE *fp) {
        double buf[6];
        fread(buf, sizeof(buf), 1, fp);
        Set(float(buf[0]), float(buf[1]), float(buf[2]), float(buf[3]));
    }
    inline void Print() const {
        printf("%f %f %f %f\n", fx(), fy(), cx(), cy());
    }

    static inline void K_M(const IntrinsicMatrix &K, const LA::AlignedMatrix3f &M,
                           LA::AlignedMatrix3f &K_M) {
        K_M.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(K.m_fx_fx_fx_fx, M.M_00_01_02_x()),
                                        ENFT_SSE::_mm_mul_ps(K.m_cx_cx_cx_cx, M.M_20_21_22_x()));
        K_M.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(K.m_fy_fy_fy_fy, M.M_10_11_12_x()),
                                        ENFT_SSE::_mm_mul_ps(K.m_cy_cy_cy_cy, M.M_20_21_22_x()));
        K_M.M_20_21_22_x() = M.M_20_21_22_x();
    }
    static inline void K_M(const IntrinsicMatrix &K, const LA::AlignedMatrix3f &M,
                           LA::AlignedMatrix3x4f &K_M) {
        K_M.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(K.m_fx_fx_fx_fx, M.M_00_01_02_x()),
                                         ENFT_SSE::_mm_mul_ps(K.m_cx_cx_cx_cx, M.M_20_21_22_x()));
        K_M.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(K.m_fy_fy_fy_fy, M.M_10_11_12_x()),
                                         ENFT_SSE::_mm_mul_ps(K.m_cy_cy_cy_cy, M.M_20_21_22_x()));
        K_M.M_20_21_22_23() = M.M_20_21_22_x();
    }
    static inline void M_Kinv(const LA::AlignedMatrix3f &M,
                              const IntrinsicMatrix &K, LA::AlignedMatrix3f &M_Kinv, ENFT_SSE::__m128 &work) {
        work = _mm_set_ps(K.cy_over_fy(), K.cx_over_fx(), K.one_over_fy(),
                          K.one_over_fx());

        M_Kinv.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(M.M_00_01_02_x(),
                                           M.M_00_01_02_x()), work);
        M_Kinv.M02() = M.M02() - M_Kinv.M02() - M_Kinv.reserve0();

        M_Kinv.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(M.M_10_11_12_x(),
                                           M.M_10_11_12_x()), work);
        M_Kinv.M12() = M.M12() - M_Kinv.M12() - M_Kinv.reserve1();

        M_Kinv.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(M.M_20_21_22_x(),
                                           M.M_20_21_22_x()), work);
        M_Kinv.M22() = M.M22() - M_Kinv.M22() - M_Kinv.reserve2();
    }
    static inline void Kinv_M_K(const IntrinsicMatrix &K,
                                const LA::AlignedMatrix3f &M, LA::AlignedMatrix3f &Kinv_M_K, ENFT_SSE::__m128 *work1) {
        work1[0] = ENFT_SSE::_mm_sub_ps(M.M_00_01_02_x(), ENFT_SSE::_mm_mul_ps(M.M_20_21_22_x(),
                              K.m_cx_cx_cx_cx));
        Kinv_M_K.M00() = work1[0].m128_f32[0];
        Kinv_M_K.M01() = work1[0].m128_f32[1] * K.fy() * K.one_over_fx();
        Kinv_M_K.M02() = (work1[0].m128_f32[2] + work1[0].m128_f32[0] * K.cx() +
                          work1[0].m128_f32[1] * K.cy()) * K.one_over_fx();

        work1[0] = ENFT_SSE::_mm_sub_ps(M.M_10_11_12_x(), ENFT_SSE::_mm_mul_ps(M.M_20_21_22_x(),
                              K.m_cy_cy_cy_cy));
        Kinv_M_K.M10() = work1[0].m128_f32[0] * K.fx() * K.one_over_fy();
        Kinv_M_K.M11() = work1[0].m128_f32[1];
        Kinv_M_K.M12() = (work1[0].m128_f32[2] + work1[0].m128_f32[0] * K.cx() +
                          work1[0].m128_f32[1] * K.cy()) * K.one_over_fy();

        Kinv_M_K.M20() = M.M20() * K.fx();
        Kinv_M_K.M21() = M.M21() * K.fy();
        Kinv_M_K.M22() = M.M20() * K.cx() + M.M21() * K.cy() + M.M22();
    }
    static inline void K_M_Kinv(const IntrinsicMatrix &K,
                                const LA::AlignedMatrix3f &M, LA::AlignedMatrix3f &K_M_Kinv, ENFT_SSE::__m128 *work1) {
        K_M_Kinv.reserve0() = K.cx() * K.one_over_fx();
        K_M_Kinv.reserve1() = K.cy() * K.one_over_fy();

        work1[0] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(M.M_00_01_02_x(), K.m_fx_fx_fx_fx),
                              ENFT_SSE::_mm_mul_ps(M.M_20_21_22_x(), K.m_cx_cx_cx_cx));
        K_M_Kinv.M00() = work1[0].m128_f32[0] * K.one_over_fx();
        K_M_Kinv.M01() = work1[0].m128_f32[1] * K.one_over_fy();
        K_M_Kinv.M02() = work1[0].m128_f32[2] - work1[0].m128_f32[0] *
                         K_M_Kinv.reserve0() - work1[0].m128_f32[1] * K_M_Kinv.reserve1();

        work1[0] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(M.M_10_11_12_x(), K.m_fy_fy_fy_fy),
                              ENFT_SSE::_mm_mul_ps(M.M_20_21_22_x(), K.m_cy_cy_cy_cy));
        K_M_Kinv.M10() = work1[0].m128_f32[0] * K.one_over_fx();
        K_M_Kinv.M11() = work1[0].m128_f32[1] * K.one_over_fy();
        K_M_Kinv.M12() = work1[0].m128_f32[2] - work1[0].m128_f32[0] *
                         K_M_Kinv.reserve0() - work1[0].m128_f32[1] * K_M_Kinv.reserve1();

        K_M_Kinv.M20() = M.M20() * K.one_over_fx();
        K_M_Kinv.M21() = M.M21() * K.one_over_fy();
        K_M_Kinv.M22() = M.M22() - M.M20() * K_M_Kinv.reserve0() - M.M21() *
                         K_M_Kinv.reserve1();
    }
    static inline void K1_M_K2inv(const IntrinsicMatrix &K1,
                                  const LA::AlignedMatrix3f &M, const IntrinsicMatrix &K2,
                                  LA::AlignedMatrix3f &K1_M_K2inv, ENFT_SSE::__m128 *work2) {
        work2[0] = _mm_setr_ps(K2.one_over_fx(), K2.one_over_fy(), K2.cx_over_fx(),
                               K2.cy_over_fy());

        work2[1] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(K1.m_fx_fx_fx_fx, M.M_00_01_02_x()),
                              ENFT_SSE::_mm_mul_ps(K1.m_cx_cx_cx_cx, M.M_20_21_22_x()));
        K1_M_K2inv.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(work2[1], work2[1]),
                                               work2[0]);
        K1_M_K2inv.M02() = work2[1].m128_f32[2] - (K1_M_K2inv.M02() +
                           K1_M_K2inv.reserve0());

        work2[1] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(K1.m_fy_fy_fy_fy, M.M_10_11_12_x()),
                              ENFT_SSE::_mm_mul_ps(K1.m_cy_cy_cy_cy, M.M_20_21_22_x()));
        K1_M_K2inv.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(work2[1], work2[1]),
                                               work2[0]);
        K1_M_K2inv.M12() = work2[1].m128_f32[2] - (K1_M_K2inv.M12() +
                           K1_M_K2inv.reserve1());

        K1_M_K2inv.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(M.M_20_21_22_x(),
                                               M.M_20_21_22_x()), work2[0]);
        K1_M_K2inv.M22() = M.M22() - (K1_M_K2inv.M22() + K1_M_K2inv.reserve2());
    }
    static inline void K1Tinv_M_K2inv(const IntrinsicMatrix &K1,
                                      const LA::AlignedMatrix3f &M, const IntrinsicMatrix &K2,
                                      LA::AlignedMatrix3f &K1Tinv_M_K2inv, ENFT_SSE::__m128 *work3) {
        work3[0] = ENFT_SSE::_mm_mul_ps(M.M_00_01_02_x(), K1.m_one_over_fx_fx_fx_fx);
        work3[1] = ENFT_SSE::_mm_mul_ps(M.M_10_11_12_x(), K1.m_one_over_fy_fy_fy_fy);
        K1Tinv_M_K2inv.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(work3[0], work3[1]),
                                        K2.m_one_over_fx_fy_fx_fy);
        K1Tinv_M_K2inv.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(K1Tinv_M_K2inv.M_00_01_02_x(),
                                        K1.m_cx_cx_cy_cy);
        work3[2] = ENFT_SSE::_mm_mul_ps(K1Tinv_M_K2inv.M_00_01_02_x(), K2.m_cx_cy_cx_cy);
        memcpy(&K1Tinv_M_K2inv.M10(), &K1Tinv_M_K2inv.M02(), 8);
        K1Tinv_M_K2inv.M02() = work3[0].m128_f32[2] - work3[2].m128_f32[0] -
                               work3[2].m128_f32[1];
        K1Tinv_M_K2inv.M12() = work3[1].m128_f32[2] - work3[2].m128_f32[2] -
                               work3[2].m128_f32[3];
        work3[2] = ENFT_SSE::_mm_mul_ps(_mm_set_ps(work3[1].m128_f32[2], work3[0].m128_f32[2],
                                         M.M21(), M.M20()), _mm_movelh_ps(K2.m_one_over_fx_fy_fx_fy, K1.m_cx_cy_cx_cy));
        K1Tinv_M_K2inv.M20() = work3[2].m128_f32[0] - (K1Tinv_M_K2inv.M20() +
                               K1Tinv_M_K2inv.M22());
        K1Tinv_M_K2inv.M21() = work3[2].m128_f32[1] - (K1Tinv_M_K2inv.M21() +
                               K1Tinv_M_K2inv.reserve2());
        K1Tinv_M_K2inv.M22() = M.M22() - K1Tinv_M_K2inv.M20() * K2.cx() -
                               K1Tinv_M_K2inv.M21() * K2.cy() - (work3[2].m128_f32[2] + work3[2].m128_f32[3]);
    }

  private:

    ENFT_SSE::__m128 m_fx_fy_fx_fy, m_cx_cy_cx_cy, m_cx_cx_cy_cy, m_one_over_fx_fy_fx_fy;
    ENFT_SSE::__m128 m_fx_fx_fx_fx, m_fy_fy_fy_fy, m_cx_cx_cx_cx, m_cy_cy_cy_cy,
           m_one_over_fx_fx_fx_fx, m_one_over_fy_fy_fy_fy;
    ENFT_SSE::__m128 m__fxy__one_over_fxy__cx_over_fx__cy_over_fy;

};

#endif