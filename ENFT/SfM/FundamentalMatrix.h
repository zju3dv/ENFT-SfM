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

#ifndef _FUNDAMENTAL_MATRIX_H_
#define _FUNDAMENTAL_MATRIX_H_

#include "Point.h"
#include "LinearAlgebra/Matrix3.h"
#include "ProjectiveMatrix.h"

class FundamentalMatrix : public LA::AlignedMatrix3f
{

public:

	FundamentalMatrix() {}
	FundamentalMatrix(const LA::AlignedMatrix3f &F) { memcpy(this, F, 48); }
	inline void operator = (const LA::AlignedMatrix3f &F) { memcpy(this, F, 48); }

	inline void Invalidate() { M00() = FLT_MAX; }
	inline bool IsValid() const { return M00() != FLT_MAX; }
	inline bool IsInvalid() const { return M00() == FLT_MAX; }
	bool EnforceSingularConstraint(ENFT_SSE::__m128 *work11);
	bool ToRelativeProjectiveMatrix(ProjectiveMatrix &P, ENFT_SSE::__m128 *work2) const;
	inline void Denormalize(const ENFT_SSE::__m128 &mean_u1v1u2v2, const float &scale1, const float &scale2, ENFT_SSE::__m128 *work6)
	{
		work6[0] = _mm_set1_ps(scale2);
		work6[1] = _mm_set1_ps(scale1);
		work6[2] = ENFT_SSE::_mm_mul_ps(M_00_01_02_x(), work6[0]);
		work6[3] = ENFT_SSE::_mm_mul_ps(M_10_11_12_x(), work6[0]);
		M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(work6[2], work6[3]), work6[1]);
		work6[4] = ENFT_SSE::_mm_mul_ps(M_00_01_02_x(), _mm_unpackhi_ps(mean_u1v1u2v2, mean_u1v1u2v2));
		work6[5] = ENFT_SSE::_mm_mul_ps(M_00_01_02_x(), _mm_movelh_ps(mean_u1v1u2v2, mean_u1v1u2v2));
		memcpy(&M10(), &M02(), 8);
		M02() = work6[2].m128_f32[2] - work6[5].m128_f32[0] - work6[5].m128_f32[1];
		M12() = work6[3].m128_f32[2] - work6[5].m128_f32[2] - work6[5].m128_f32[3];
		work6[5] = ENFT_SSE::_mm_mul_ps(_mm_set_ps(work6[3].m128_f32[2], work6[2].m128_f32[2], M21(), M20()), _mm_movelh_ps(work6[1], _mm_movehl_ps(mean_u1v1u2v2, mean_u1v1u2v2)));
		M20() = work6[5].m128_f32[0] - (work6[4].m128_f32[0] + work6[4].m128_f32[2]);
		M21() = work6[5].m128_f32[1] - (work6[4].m128_f32[1] + work6[4].m128_f32[3]);
		M22() = M22() - M20() * mean_u1v1u2v2.m128_f32[0] - M21() * mean_u1v1u2v2.m128_f32[1] - (work6[5].m128_f32[2] + work6[5].m128_f32[3]);
	}
	inline void Normalize(const ENFT_SSE::__m128 &mean_u1v1u2v2, const float &scale1, const float &scale2, ENFT_SSE::__m128 *work8)
	{
		work8[0] = ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(_mm_set1_ps(-scale1), _mm_set1_ps(-scale2)), mean_u1v1u2v2);
		work8[1].m128_f32[0] = 1 / scale1;
		work8[1].m128_f32[1] = 1 / scale2;
		ENFT_SSE::__m128 *work6 = work8 + 2;
		Denormalize(work8[0], work8[1].m128_f32[0], work8[1].m128_f32[1], work6);
	}
	inline void GetSSE(ENFT_SSE::__m128 *F) const
	{
		F[0] = _mm_set1_ps(M00());		F[1] = _mm_set1_ps(M01());		F[2] = _mm_set1_ps(M02());
		F[3] = _mm_set1_ps(M10());		F[4] = _mm_set1_ps(M11());		F[5] = _mm_set1_ps(M12());
		F[6] = _mm_set1_ps(M20());		F[7] = _mm_set1_ps(M21());		F[8] = _mm_set1_ps(M22());
	}

	inline void ComputeEpipolarLine(const float *x1, float *l2) const
	{
		l2[0] = M00() * x1[0] + M01() * x1[1] + M02();
		l2[1] = M10() * x1[0] + M11() * x1[1] + M12();
		l2[2] = M20() * x1[0] + M21() * x1[1] + M22();
	}
	inline void ComputeEpipolarLine(const Point2D &x1, LA::AlignedVector3f &l2) const
	{
		l2.v0() = M00() * x1.x() + M01() * x1.y() + M02();
		l2.v1() = M10() * x1.x() + M11() * x1.y() + M12();
		l2.v2() = M20() * x1.x() + M21() * x1.y() + M22();
	}
	inline void ComputeEpipolarLineInverse(const Point2D &x2, LA::AlignedVector3f &l1) const
	{
		l1.v012x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(M_00_01_02_x(), _mm_set1_ps(x2.x())), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(M_10_11_12_x(), _mm_set1_ps(x2.y())), M_20_21_22_x()));
	}
	inline float ComputeEpipolarError(const Point2D &x1, const Point2D &x2, float *work3) const
	{
		float &l2x = work3[0], &l2y = work3[1], &x2TFx1 = work3[2];
		/*float */l2x = M00() * x1.x() + M01() * x1.y() + M02();
		/*float */l2y = M10() * x1.x() + M11() * x1.y() + M12();
		/*float */x2TFx1 = x2.x() * l2x + x2.y() * l2y + M20() * x1.x() + M21() * x1.y() + M22();
		return fabs(x2TFx1) / sqrt(l2x * l2x + l2y * l2y);
	}
	inline float ComputeEpipolarSquaredError(const Point2D &x1, const Point2D &x2, float *work3) const
	{
		float &l2x = work3[0], &l2y = work3[1], &x2TFx1 = work3[2];
		/*float */l2x = M00() * x1.x() + M01() * x1.y() + M02();
		/*float */l2y = M10() * x1.x() + M11() * x1.y() + M12();
		/*float */x2TFx1 = x2.x() * l2x + x2.y() * l2y + M20() * x1.x() + M21() * x1.y() + M22();
		return x2TFx1 * x2TFx1 / (l2x * l2x + l2y * l2y);
	}
	inline float ComputeEpipolarSquaredError(const Point2D &x1, const ushort &x2, const ushort &y2, float *work3) const
	{
		float &l2x = work3[0], &l2y = work3[1], &x2TFx1 = work3[2];
		/*float */l2x = M00() * x1.x() + M01() * x1.y() + M02();
		/*float */l2y = M10() * x1.x() + M11() * x1.y() + M12();
		/*float */x2TFx1 = x2 * l2x + y2 * l2y + M20() * x1.x() + M21() * x1.y() + M22();
		return x2TFx1 * x2TFx1 / (l2x * l2x + l2y * l2y);
	}
	inline float ComputeEpipolarErrorInverse(const Point2D &x1, const Point2D &x2, float *work3)
	{
		float &l1x = work3[0], &l1y = work3[1], &x2TFx1 = work3[2];
		/*float */l1x = M00() * x2.x() + M10() * x2.y() + M20();
		/*float */l1y = M01() * x2.x() + M11() * x2.y() + M21();
		/*float */x2TFx1 = x1.x() * l1x + x1.y() * l1y + M02() * x2.x() + M12() * x2.y() + M22();
		return fabs(x2TFx1) / sqrt(l1x * l1x + l1y * l1y);
	}
	static inline void ComputeSymmetricSquaredError4(const ENFT_SSE::__m128 *F, const ENFT_SSE::__m128 &u1, const ENFT_SSE::__m128 &v1, const ENFT_SSE::__m128 &u2, const ENFT_SSE::__m128 &v2, ENFT_SSE::__m128 &errSq, 
													 ENFT_SSE::__m128 *work6)
	{
		/*ENFT_SSE::__m128 l2x*/work6[0] = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(F[0], u1), ENFT_SSE::_mm_mul_ps(F[1], v1)), F[2]);
		/*ENFT_SSE::__m128 l2y*/work6[1] = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(F[3], u1), ENFT_SSE::_mm_mul_ps(F[4], v1)), F[5]);
		/*ENFT_SSE::__m128 d  */work6[2] = ENFT_SSE::_mm_add_ps(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(u2, /*l2x*/F[9]), ENFT_SSE::_mm_mul_ps(v2, /*l2y*/work6[1])), 
													   ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(F[6], u1), ENFT_SSE::_mm_mul_ps(F[7], v1))), F[8]);
		/*ENFT_SSE::__m128 d2 */work6[3] = ENFT_SSE::_mm_mul_ps(/*d, d*/work6[2], work6[2]);
		/*ENFT_SSE::__m128 l1x*/work6[4] = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(F[0], u2), ENFT_SSE::_mm_mul_ps(F[3], v2)), F[6]);
		/*ENFT_SSE::__m128 l1y*/work6[5] = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(F[1], u2), ENFT_SSE::_mm_mul_ps(F[4], v2)), F[7]);
		errSq = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_div_ps(/*d2*/work6[3], ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(/*l1x, l1x*/work6[4], work6[4]), ENFT_SSE::_mm_mul_ps(/*l1y, l1y*/work6[5], work6[5]))), 
						   ENFT_SSE::_mm_div_ps(/*d2*/work6[3], ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(/*l2x, l2x*/work6[0], work6[0]), ENFT_SSE::_mm_mul_ps(/*l2y, l2y*/work6[1], work6[1]))));
	}
	inline float ComputeSymmetricSquaredError(const Point2D &x1, const Point2D &x2, float *work6) const
	{
		float &l2x = work6[0], &l2y = work6[1], &d = work6[2], &d2 = work6[3], &l1x = work6[4], &l1y = work6[5];
		/*float */l2x = M00() * x1.x() + M01() * x1.y() + M02();
		/*float */l2y = M10() * x1.x() + M11() * x1.y() + M12();
		/*float */d   = x2.x() * l2x + x2.y() * l2y + M20() * x1.x() + M21() * x1.y() + M22();
		/*float */d2  = d * d;
		/*float */l1x = M00() * x2.x() + M10() * x2.y() + M20();
		/*float */l1y = M01() * x2.x() + M11() * x2.y() + M21();
		return d2 / (l1x * l1x + l1y * l1y) + d2 / (l2x * l2x + l2y * l2y);
	}
	inline float ComputeSymmetricError(const Point2D &x1, const Point2D &x2, float *work6)
	{
		return sqrt(ComputeSymmetricSquaredError(x1, x2, work6));
	}

	inline void Print(const bool normalize) const
	{
		FundamentalMatrix F = *this;
		if(normalize)
			F.EnforceUnitLastEntry();
			//F.EnforceUnitFrobeniusNorm();
		F.AlignedMatrix3f::Print();
	}
	inline void Save(FILE *fp, const bool normalize) const
	{
		FundamentalMatrix F = *this;
		if(normalize)
			F.EnforceUnitLastEntry();
			//F.EnforceUnitFrobeniusNorm();
		F.AlignedMatrix3f::Save(fp);
	}
	inline void Save(const char *fileName, const bool normalize) const
	{
		FILE *fp = fopen( fileName, "w");
		Save(fp, normalize);
		fclose(fp);
	}
};

#endif