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

#ifndef _AFFINE_TRANSFORMATION_H_
#define _AFFINE_TRANSFORMATION_H_

#include "LinearAlgebra/Matrix2x3.h"
#include "SimilarityTransformation.h"

class AffineTransformation2D : public LA::AlignedMatrix2x3f
{

public:

	inline void MakeIdentity()
	{
		SetZero();
		M00() = M11() = 1.0f;
	}

	//inline void Set(const SimilarityTransformation2D &T)
	//{
	//	M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(T.s(), T.r_00_01_10_11());
	//	M10() = M02();
	//	M11() = reserve0();
	//	M02() = T.tx();
	//	M12() = T.ty();
	//}

	inline void Invalidate() { M00() = FLT_MAX; }
	inline bool IsValid() const { return M00() != FLT_MAX; }
	inline bool IsInvalid() const { return M00() == FLT_MAX; }

	inline void Apply(const Point2D &x, Point2D &Tx) const
	{
		Tx.x() = M00() * x.x() + M01() * x.y() + M02();
		Tx.y() = M10() * x.x() + M11() * x.y() + M12();
	}

	// x2 = T * x1
	// sx2 = T' * sx1
	inline void ChangeScale(const float &s)
	{
		M02() *= s;
		M12() *= s;
	}

	//inline bool Invert(LA::AlignedMatrix2x3f &Tinv) const
	inline bool Invert(AffineTransformation2D &Tinv) const
	{
		return Invert(*this, Tinv);
	}

	// x2 - c = A * (x1 - c) + t --> x2 = A * x1 - A * c + t + c
	// A' = A, t' = -A * c + t + c
	inline void CenterToCorner(const Point2D &c)
	{
		M02() += c.x() - M00() * c.x() - M01() * c.y();
		M12() += c.y() - M10() * c.x() - M11() * c.y();
	}

	// x2 = A12 * x1 + t12 = A12 * (A1 * x + t1) + t12 = A12 * A1 * x + A12 * t1 + t12
	// A2 = A12 * A1, t2 = A12 * t1 + t12
	static inline void AccumulateTransformation(const AffineTransformation2D &T1, const AffineTransformation2D &T12, AffineTransformation2D &T2, ENFT_SSE::__m128 *work0)
	{
		T2.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(T12.M00()), T1.M_00_01_02_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(T12.M01()), T1.M_10_11_12_x()));
		T2.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(T12.M10()), T1.M_00_01_02_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(T12.M11()), T1.M_10_11_12_x()));
		T2.M02() += T12.M02();
		T2.M12() += T12.M12();
	}
	static inline void AccumulateTransformation(const AffineTransformation2D &T1, const SimilarityTransformation2D &T12, AffineTransformation2D &T2, ENFT_SSE::__m128 *work1)
	{
		work1[0] = ENFT_SSE::_mm_mul_ps(T12.s(), T12.r_00_01_10_11());
		T2.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(work1[0].m128_f32[0]), T1.M_00_01_02_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(work1[0].m128_f32[1]), T1.M_10_11_12_x()));
		T2.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(work1[0].m128_f32[2]), T1.M_00_01_02_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(work1[0].m128_f32[3]), T1.M_10_11_12_x()));
		T2.M02() += T12.tx();
		T2.M12() += T12.ty();
	}

	// x2 = A * x1 + t
	// sx2 = A' * x1 + t'
	// A' = s * A12, t12' = s * t12
	// x1 = Ainv * x2 + tinv
	// x1 = Ainv' * sx2 + tinv'
	// Ainv' = Ainv / s, tinv' = tinv;
	static inline void ChangeScale2(const float &s, const float &sinv, AffineTransformation2D &T, AffineTransformation2D &Tinv, ENFT_SSE::__m128 *work1)
	{
		work1[0] = _mm_set1_ps(s);
		T.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(work1[0], T.M_00_01_02_x());
		T.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(work1[0], T.M_10_11_12_x());
		Tinv.M00() *= sinv;		Tinv.M01() *= sinv;
		Tinv.M10() *= sinv;		Tinv.M11() *= sinv;
	}

	static inline bool Invert(const LA::AlignedMatrix2x3f &T, LA::AlignedMatrix2x3f &Tinv)
	{
		if((Tinv.M00() = T.M00() * T.M11() - T.M01() * T.M10()) == 0.0f)
			return false;
		Tinv.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / Tinv.M00()), _mm_setr_ps(T.M11(), -T.M01(), -T.M10(), T.M00()));
		Tinv.M10() = Tinv.M02();
		Tinv.M11() = Tinv.reserve0();
		Tinv.M02() = -(Tinv.M00() * T.M02() + Tinv.M01() * T.M12());
		Tinv.M12() = -(Tinv.M10() * T.M02() + Tinv.M11() * T.M12());
		return true;
	}

	static bool Invert(const SimilarityTransformation2D &T, AffineTransformation2D &Tinv)
	{
		Tinv.reserve0() = 1 / T.GetScale();
		Tinv.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(Tinv.reserve0()), T.r_00_01_10_11());
		Tinv.M00() = Tinv.M10();	Tinv.M01() = Tinv.M12();		Tinv.M02() = -Tinv.reserve0() * (T.r00() * T.tx() + T.r10() * T.ty());
		Tinv.M10() = Tinv.M11();	Tinv.M11() = Tinv.reserve1();	Tinv.M12() = -Tinv.reserve0() * (T.r01() * T.tx() + T.r11() * T.ty());
		return true;
	}
};

#endif