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

#ifndef _HOMOGRAPHY_H_
#define _HOMOGRAPHY_H_

#include "AffineTransformation.h"
#include "Match.h"

class Homography : public LA::AlignedMatrix3f
{

public:

	Homography() {}
	Homography(const Homography &H) { memcpy(this, H, 48); }
	inline void operator = (const Homography &H) { memcpy(this, H, 48); }

	inline void Invalidate() { M00() = FLT_MAX; }
	inline bool IsValid() const { return M00() != FLT_MAX; }
	inline bool IsInvalid() const { return M00() == FLT_MAX; }
	inline void MarkAffineTransformation() { M20() = FLT_MAX; }
	inline bool IsAffineTransformation() const { return M20() == FLT_MAX; }
	inline void Set(const RigidTransformation2D &T, const bool markAffine = true)
	{
		M00() = T.r00();	M01() = T.r01();	M02() = T.tx();
		M10() = T.r10();	M11() = T.r11();	M12() = T.ty();
		if(markAffine)
			MarkAffineTransformation();
		else
		{
			M20() = 0.0f;
			M21() = 0.0f;
			M22() = 1.0f;
		}
	}
	inline void Set(const SimilarityTransformation2D &T, const bool markAffine = true)
	{
		M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(T.s(), T.r_00_01_10_11());
		M00() = M20();		M01() = M21();		M02() = T.tx();
		M10() = M22();		M11() = reserve2();	M12() = T.ty();
		if(markAffine)
			MarkAffineTransformation();
		else
		{
			M20() = 0.0f;
			M21() = 0.0f;
			M22() = 1.0f;
		}
	}
	inline void Set(const AffineTransformation2D &T, const bool markAffine = true)
	{
		M_00_01_02_x() = T.M_00_01_02_x();
		M_10_11_12_x() = T.M_10_11_12_x();
		if(markAffine)
			MarkAffineTransformation();
		else
		{
			M20() = 0.0f;
			M21() = 0.0f;
			M22() = 1.0f;
		}
	}
	inline void Set(const Homography &T, const bool markAffine = true)
	{
		memcpy(this, T, sizeof(Homography));
	}
	inline bool Invert(Homography &Hinv) const
	{
		if(IsAffineTransformation())
		{
			if(!AffineTransformation2D::Invert(*this, Hinv))
				return false;
			Hinv.MarkAffineTransformation();
			return true;
		}
		else
			return LA::AlignedMatrix3f::Invert(Hinv);
	}

	inline void Apply(const Point2D &x, Point2D &Hx) const
	{
		if(IsAffineTransformation())
		{
			Hx.x() = M00() * x.x() + M01() * x.y() + M02();
			Hx.y() = M10() * x.x() + M11() * x.y() + M12();
		}
		else
		{
			Hx.x() = 1 / (M20() * x.x() + M21() * x.y() + M22());
			Hx.y() = (M10() * x.x() + M11() * x.y() + M12()) * Hx.x();
			Hx.x() = (M00() * x.x() + M01() * x.y() + M02()) * Hx.x();
		}
	}
	inline void Apply(const float *x, float *Hx) const
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		Hx[0] = 1 / (M20() * x[0] + M21() * x[1] + M22());
		Hx[1] = (M10() * x[0] + M11() * x[1] + M12()) * Hx[0];
		Hx[0] = (M00() * x[0] + M01() * x[1] + M02()) * Hx[0];
	}
	inline void Apply(const float &x_u, const float &x_v, float &Hx_u, float &Hx_v) const
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		Hx_u = 1 / (M20() * x_u + M21() * x_v + M22());
		Hx_v = (M10() * x_u + M11() * x_v + M12()) * Hx_u;
		Hx_u = (M00() * x_u + M01() * x_v + M02()) * Hx_u;
	}
	inline void Apply(const ENFT_SSE::__m128 &x, ENFT_SSE::__m128 &Hx) const
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		Hx.m128_f32[0] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(M_00_01_02_x(), x));
		Hx.m128_f32[1] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(M_10_11_12_x(), x));
		Hx.m128_f32[2] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(M_20_21_22_x(), x));
	}
	inline void Apply(const int &width, const int &height, Point2D &Hx00, Point2D &Hx01, Point2D &Hx10, Point2D &Hx11) const
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		Hx00.x() = 1 / M22();
		Hx00.y() = M12() * Hx00.x();
		Hx00.x() = M02() * Hx00.x();

		Hx01.x() = 1 / (M21() * height + M22());
		Hx01.y() = (M11() * height + M12()) * Hx01.x();
		Hx01.x() = (M01() * height + M02()) * Hx01.x();

		Hx10.x() = 1 / (M20() * width + M22());
		Hx10.y() = (M10() * width + M12()) * Hx10.x();
		Hx10.x() = (M00() * width + M02()) * Hx10.x();

		Hx11.x() = 1 / (M20() * width + M21() * height + M22());
		Hx11.y() = (M10() * width + M11() * height + M12()) * Hx11.x();
		Hx11.x() = (M00() * width + M01() * height + M02()) * Hx11.x();
	}
	inline void ApplyInversely(const Point2D &Hx, Point2D &x) const
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		x.x() = 1 / ((M10() * M21() - M11() * M20()) * Hx.x() + (M01() * M20() - M00() * M21()) * Hx.y() + (M00() * M11() - M01() * M10()));
		x.y() = ((M12() * M20() - M10() * M22()) * Hx.x() + (M00() * M22() - M02() * M20()) * Hx.y() + (M02() * M10() - M00() * M12())) * x.x();
		x.x() = ((M11() * M22() - M12() * M21()) * Hx.x() + (M02() * M21() - M01() * M22()) * Hx.y() + (M01() * M12() - M02() * M11())) * x.x();
	}

	// x2 = H * x1
	// sx2 = H' * sx1
	inline void ChangeScale(const float &s)
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		M02() *= s;
		M12() *= s;
		M20() /= s;
		M21() /= s;
	}

	// x2 = H * x1
	// x2 + c = H' * (x1 + c)
	inline void CenterToCorner(const Point2D &c)
	{
		M22() -= M20() * c.x() + M21() * c.y();

		M02() -= M00() * c.x() + M01() * c.y() - c.x() * M22();
		M00() += c.x() * M20();
		M01() += c.x() * M21();

		M12() -= M10() * c.x() + M11() * c.y() - c.y() * M22();
		M10() += c.y() * M20();
		M11() += c.y() * M21();
	}

	inline void Denormalize(const ENFT_SSE::__m128 &mean_u1v1u2v2, const float &scale1, const float &scale2, ENFT_SSE::__m128 *work3)
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		work3[0] = _mm_set_ps(mean_u1v1u2v2.m128_f32[1] * scale1, mean_u1v1u2v2.m128_f32[0] * scale1, scale1, scale1);
		work3[1] = _mm_set1_ps(1 / scale2);
		work3[2] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[1], M_00_01_02_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(mean_u1v1u2v2.m128_f32[2]), M_20_21_22_x()));
		M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(work3[2], work3[2]), work3[0]);
		M02() = work3[2].m128_f32[2] - (M02() + reserve0());

		work3[2] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[1], M_10_11_12_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(mean_u1v1u2v2.m128_f32[3]), M_20_21_22_x()));
		M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(work3[2], work3[2]), work3[0]);
		M12() = work3[2].m128_f32[2] - (M12() + reserve1());

		work3[2].m128_f32[2] = M22();
		M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(M_20_21_22_x(), M_20_21_22_x()), work3[0]);
		M22() = work3[2].m128_f32[2] - (M22() + reserve2());
	}
	inline void Normalize(const ENFT_SSE::__m128 &mean_u1v1u2v2, const float &scale1, const float &scale2, ENFT_SSE::__m128 *work5)
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		work5[0] = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(_mm_set1_ps(-scale1), _mm_set1_ps(-scale2)), mean_u1v1u2v2);
		work5[1].m128_f32[0] = 1 / scale1;
		work5[1].m128_f32[1] = 1 / scale2;
		ENFT_SSE::__m128 *work3 = work5 + 2;
		Denormalize(work5[0], work5[1].m128_f32[0], work5[1].m128_f32[1], work3);
	}
	inline void GetSSE(ENFT_SSE::__m128 *H) const
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		H[0] = _mm_set1_ps(M00());		H[1] = _mm_set1_ps(M01());		H[2] = _mm_set1_ps(M02());
		H[3] = _mm_set1_ps(M10());		H[4] = _mm_set1_ps(M11());		H[5] = _mm_set1_ps(M12());
		H[6] = _mm_set1_ps(M20());		H[7] = _mm_set1_ps(M21());		H[8] = _mm_set1_ps(M22());
	}

	inline float ComputeSquaredError(const Point2D &x1, const Point2D &x2, float *work3) const
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		float &t = work3[0], &du = work3[1], &dv = work3[2];
		t = 1 / (M20() * x1.x() + M21() * x1.y() + M22());
		du = (M00() * x1.x() + M01() * x1.y() + M02()) * t - x2.x();
		dv = (M10() * x1.x() + M11() * x1.y() + M12()) * t - x2.y();
		return du * du + dv * dv;
	}
	inline float ComputeSign(const Point2D &x1, const Point2D &x2) const
	{
#if _DEBUG
		assert(!IsAffineTransformation());
#endif
		return (M00() * x1.x() + M01() * x1.y() + M02()) * x2.x() + (M10() * x1.x() + M11() * x1.y() + M12()) * x2.y() + (M20() * x1.x() + M21() * x1.y() + M22());
	}

	bool ToRelativePose(const MatchSet2D &xs, RigidTransformation3D &T1, LA::AlignedVector3f &N1, RigidTransformation3D &T2, LA::AlignedVector3f &N2, const float &sccRatioTh, 
		ENFT_SSE::__m128 *work14);

	static inline void ComputeSquaredError4(const ENFT_SSE::__m128 *H, const ENFT_SSE::__m128 &u1, const ENFT_SSE::__m128 &v1, const ENFT_SSE::__m128 &u2, const ENFT_SSE::__m128 &v2, ENFT_SSE::__m128 &errSq, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_div_ps(_mm_set1_ps(1), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(H[6], u1), ENFT_SSE::_mm_mul_ps(H[7], v1)), H[8]));
		work2[1] = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(H[3], u1), ENFT_SSE::_mm_mul_ps(H[4], v1)), H[5]), work2[0]), v2);
		work2[0] = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(H[0], u1), ENFT_SSE::_mm_mul_ps(H[1], v1)), H[2]), work2[0]), u2);
		errSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], work2[0]), ENFT_SSE::_mm_mul_ps(work2[1], work2[1]));
	}
	static inline void ComputeSign4(const ENFT_SSE::__m128 *H, const ENFT_SSE::__m128 &u1, const ENFT_SSE::__m128 &v1, const ENFT_SSE::__m128 &u2, const ENFT_SSE::__m128 &v2, ENFT_SSE::__m128 &dot)
	{
		dot = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(H[0], u1), ENFT_SSE::_mm_mul_ps(H[1], v1)), H[2]), u2), 
									ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(H[3], u1), ENFT_SSE::_mm_mul_ps(H[4], v1)), H[5]), v2)), 
											   ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(H[6], u1), ENFT_SSE::_mm_mul_ps(H[7], v1)), H[8]));
	}

	// H12 and H2 are allowed to be the same
	static inline void AccumulateTransformation(const SimilarityTransformation2D &H1, const Homography &H12, Homography &H2, ENFT_SSE::__m128 *work2)
	{
#if _DEBUG
		assert(!H12.IsAffineTransformation());
#endif
		work2[1] = ENFT_SSE::_mm_mul_ps(H1.s(), H1.r_00_01_10_11());
		work2[0] = _mm_setr_ps(work2[1].m128_f32[0], work2[1].m128_f32[1], H1.tx(), H12.M02());
		work2[1] = _mm_setr_ps(work2[1].m128_f32[2], work2[1].m128_f32[3], H1.ty(), H12.M12());
		H2.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(H12.M00()), work2[0]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(H12.M01()), work2[1]));
		H2.M02() += work2[0].m128_f32[3];
		H2.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(H12.M10()), work2[0]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(H12.M11()), work2[1]));
		H2.M12() += work2[1].m128_f32[3];
		work2[0].m128_f32[3] = H12.M22();
		H2.M_20_21_22_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(H12.M20()), work2[0]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(H12.M21()), work2[1]));
		H2.M22() += work2[0].m128_f32[3];
	}

	// x2 = H12 * x1 = H12 * (H1 * x) = H12 * H1 * x
	// H2 = H12 * H1
	// H12 and H2 are allowed to be the same
	static inline void AccumulateTransformation(const Homography &H1, const Homography &H12, Homography &H2, ENFT_SSE::__m128 *work3)
	{
#if _DEBUG
		assert(!H1.IsAffineTransformation() && !H12.IsAffineTransformation());
#endif
		LA::AB(H12, H1, H2, work3);
	}

	// x2 = H * x1
	// x2 = H' * sx1
	// x1 = Hinv * x2
	// sx1 = Hinv' * x2
	static inline void ChangeScale1(const float &s, const float &sinv, Homography &H, Homography &Hinv)
	{
		H.ScaleColumn2(s);
		Hinv.ScaleRow2(sinv);
	}
	static inline void ChangeScale1(const float &s, const float &sinv, Homography &H, Homography &Hinv, ENFT_SSE::__m128 *work0)
	{
		ChangeScale1(s, sinv, H, Hinv);
	}

	// x2 = H * x1
	// sx2 = H' * x1
	// x1 = Hinv * x2
	// x1 = Hinv' * sx2
	static inline void ChangeScale2(const float &s, const float &sinv, Homography &H, Homography &Hinv)
	{
		H.ScaleRow2(sinv);
		Hinv.ScaleColumn2(s);
	}
	static inline void ChangeScale2(const float &s, const float &sinv, Homography &H, Homography &Hinv, ENFT_SSE::__m128 *work0)
	{
		ChangeScale2(s, sinv, H, Hinv);
	}

	static bool Invert(const RigidTransformation2D &T, Homography &Tinv)
	{
		Tinv.M00() = T.r00();		Tinv.M01() = T.r10();		Tinv.M02() = -(T.r00() * T.tx() + T.r10() * T.ty());
		Tinv.M10() = T.r10();		Tinv.M11() = T.r11();		Tinv.M12() = -(T.r01() * T.tx() + T.r11() * T.ty());
		Tinv.MarkAffineTransformation();
		return true;
	}
	static bool Invert(const SimilarityTransformation2D &T, Homography &Tinv)
	{
		Tinv.reserve0() = 1 / T.GetScale();
		Tinv.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(Tinv.reserve0()), T.r_00_01_10_11());
		Tinv.M00() = Tinv.M10();	Tinv.M01() = Tinv.M12();		Tinv.M02() = -Tinv.reserve0() * (T.r00() * T.tx() + T.r10() * T.ty());
		Tinv.M10() = Tinv.M11();	Tinv.M11() = Tinv.reserve1();	Tinv.M12() = -Tinv.reserve0() * (T.r01() * T.tx() + T.r11() * T.ty());
		Tinv.MarkAffineTransformation();
		return true;
	}
	static bool Invert(const AffineTransformation2D &T, Homography &Tinv)
	{
		if(!AffineTransformation2D::Invert(T, Tinv))
			return false;
		Tinv.MarkAffineTransformation();
		return true;
	}
	static bool Invert(const Homography &T, Homography &Tinv)
	{
		return T.Invert(Tinv);
	}

};

#endif