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

#ifndef _PROJECTIVE_MATRIX_H_
#define _PROJECTIVE_MATRIX_H_

#include "LinearAlgebra/Matrix3x4.h"
#include "IntrinsicMatrix.h"
#include "SfM/Camera.h"
#include "Point.h"

class ProjectiveMatrix : public LA::AlignedMatrix3x4f
{

public:

	inline void Set(const Camera &C)
	{
		M_00_01_02_03() = C.r00_r01_r02_tX();
		M_10_11_12_13() = C.r10_r11_r12_tY();
		M_20_21_22_23() = C.r20_r21_r22_tZ();
	}
	inline void FromIntrinsicExtrinsic(const IntrinsicMatrix &K, const Camera &C) { IntrinsicMatrix::K_M(K, C, *this); }
	inline void FromIntrinsicExtrinsic(const float &f, const Camera &C)
	{
		M_20_21_22_23() = _mm_set1_ps(f);
		M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(M_20_21_22_23(), C.r00_r01_r02_tX());
		M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(M_20_21_22_23(), C.r10_r11_r12_tY());
		M_20_21_22_23() = C.r20_r21_r22_tZ();
	}
	bool ToIdealIntrinsicExtrinsic(float &f, Camera &C, ENFT_SSE::__m128 *work4) const;
	inline float ComputeDepth(const Point3D &X) const
	{
		return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_20_21_22_23(), X.XYZx()));
	}
	inline float ComputeProjectionSquaredError(const Point3D &X, const Point2D &x, Point2D &e) const
	{
#if _DEBUG
		assert(X.reserve() == 1.0f);
#endif
		e.y() = 1 / ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_20_21_22_23(), X.XYZx()));
		e.x() = x.x() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_00_01_02_03(), X.XYZx())) * e.y();
		e.y() = x.y() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_10_11_12_13(), X.XYZx())) * e.y();
		return e.SquaredLength();
	}
	inline void Apply(const Point3D &X, Point3D &PX) const
	{
#if _DEBUG
		assert(X.reserve() == 1);
#endif
		PX.X() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_00_01_02_03(), X.XYZx()));
		PX.Y() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_10_11_12_13(), X.XYZx()));
		PX.Z() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_20_21_22_23(), X.XYZx()));
	}
	inline void Apply(const Point3D &X, float &PX_X, float &PX_Y, float &PX_Z) const
	{
#if _DEBUG
		assert(X.reserve() == 1);
#endif
		PX_X = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_00_01_02_03(), X.XYZx()));
		PX_Y = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_10_11_12_13(), X.XYZx()));
		PX_Z = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_20_21_22_23(), X.XYZx()));
	}
	inline void Project(const Point3D &X, Point2D &x) const
	{
		x.y() = 1 / ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_20_21_22_23(), X.XYZx()));
		x.x() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_00_01_02_03(), X.XYZx())) * x.y();
		x.y() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(M_10_11_12_13(), X.XYZx())) * x.y();
	}
	inline void Project(const Point3D &X, float &ZcI, Point2D &x) const
	{
		Apply(X, x.x(), x.y(), ZcI);
		ZcI = 1 / ZcI;
		x *= ZcI;
	}
	inline void ComputeProjectionError2(const Point3D &X0, const Point3D &X1, const ENFT_SSE::__m128 &x2, ENFT_SSE::__m128 &e2, ENFT_SSE::__m128 &work) const
	{
		Apply(X0, e2.m128_f32[0], e2.m128_f32[1], work.m128_f32[0]);		work.m128_f32[0] = work.m128_f32[1] = 1 / work.m128_f32[0];
		Apply(X1, e2.m128_f32[2], e2.m128_f32[3], work.m128_f32[2]);		work.m128_f32[2] = work.m128_f32[3] = 1 / work.m128_f32[2];
		e2 = ENFT_SSE::_mm_sub_ps(x2, ENFT_SSE::_mm_mul_ps(e2, work));
	}
	inline void Normalize(const Point3D &cX, const ENFT_SSE::__m128 &sX, const Point2D &cx, const ENFT_SSE::__m128 &sx)
	{
		M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(sx, ENFT_SSE::_mm_sub_ps(M_00_01_02_03(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(cx.x()), M_20_21_22_23())));
		M03() = sX.m128_f32[0] * (M03() + ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(M_00_01_02_03(), cX.XYZx())));
		M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(sx, ENFT_SSE::_mm_sub_ps(M_10_11_12_13(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(cx.y()), M_20_21_22_23())));
		M13() = sX.m128_f32[0] * (M13() + ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(M_10_11_12_13(), cX.XYZx())));
		M23() = sX.m128_f32[0] * (M23() + ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(M_20_21_22_23(), cX.XYZx())));
	}
	inline void Denormalize(const Point3D &cX, const ENFT_SSE::__m128 &sX, const Point2D &cx, const ENFT_SSE::__m128 &sx, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_mul_ps(sX, M_20_21_22_23());
		work2[0].m128_f32[3] = M23() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work2[0], cX.XYZx()));
		M_20_21_22_23() = work2[0];

		work2[0] = ENFT_SSE::_mm_mul_ps(sX, M_00_01_02_03());
		work2[0].m128_f32[3] = M03() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work2[0], cX.XYZx()));
		work2[1] = _mm_set1_ps(1 / sx.m128_f32[0]);
		M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], work2[1]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(cx.x()), M_20_21_22_23()));

		work2[0] = ENFT_SSE::_mm_mul_ps(sX, M_10_11_12_13());
		work2[0].m128_f32[3] = M13() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work2[0], cX.XYZx()));
		M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], work2[1]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(cx.y()), M_20_21_22_23()));
	}

	inline void Save(FILE *fp) const
	{
		fprintf(fp, "%f %f %f %f\n", M00(), M01(), M02(), M03());
		fprintf(fp, "%f %f %f %f\n", M10(), M11(), M12(), M13());
		fprintf(fp, "%f %f %f %f\n", M20(), M21(), M22(), M23());
	}

};

class ProjectiveMatrixMetric : public ProjectiveMatrix
{

public:

	inline const float& f() const { return m_f; }		inline float& f() { return m_f; }
	inline const Camera& C() const { return m_C; }		inline Camera& C() { return m_C; }

	inline bool Set(const ProjectiveMatrix &P, ENFT_SSE::__m128 *work4)
	{
		if(!P.ToIdealIntrinsicExtrinsic(m_f, m_C, work4))
			return false;
		FromIntrinsicExtrinsic(m_f, m_C);
		return true;
	}
	inline void Set(const float &f, const Camera &C)
	{
		m_f = f;
		m_C = C;
		FromIntrinsicExtrinsic(m_f, m_C);
	}

protected:

	float m_f;
	Camera m_C;

};

#endif