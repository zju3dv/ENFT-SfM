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

#ifndef _MATRIX_7x3_H_
#define _MATRIX_7x3_H_

#include "Matrix6x3.h"

namespace LA
{

	class AlignedMatrix7x3f : public AlignedMatrix6x3f
	{

	public:

		inline const ENFT_SSE::__m128& M_60_61_62_x() const { return m_sse6; }			inline ENFT_SSE::__m128& M_60_61_62_x() { return m_sse6; }
		inline const float& M60()	   const { return m_sse6.m128_f32[0]; }		inline float& M60()		 { return m_sse6.m128_f32[0]; }
		inline const float& M61()	   const { return m_sse6.m128_f32[1]; }		inline float& M61()		 { return m_sse6.m128_f32[1]; }
		inline const float& M62()	   const { return m_sse6.m128_f32[2]; }		inline float& M62()		 { return m_sse6.m128_f32[2]; }
		inline const float& reserve6() const { return m_sse6.m128_f32[3]; }		inline float& reserve6() { return m_sse6.m128_f32[3]; }

		inline void SetZero() { memset(this, 0, 112); }

	private:

		ENFT_SSE::__m128 m_sse6;
	};

	inline void sA(const AlignedVector7f &s, AlignedMatrix7x3f &A)
	{
		A.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v0()), A.M_00_01_02_x());
		A.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v1()), A.M_10_11_12_x());
		A.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v2()), A.M_20_21_22_x());
		A.M_30_31_32_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v3()), A.M_30_31_32_x());
		A.M_40_41_42_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v4()), A.M_40_41_42_x());
		A.M_50_51_52_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v5()), A.M_50_51_52_x());
		A.M_60_61_62_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v6()), A.M_60_61_62_x());
	}
	inline void AddABTo(const AlignedMatrix7x3f &A, const AlignedVector3f &B, AlignedVector7f &to)
	{
		to.v0() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B.v012x())) + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B.v012x())) + to.v1();
		to.v2() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), B.v012x())) + to.v2();
		to.v3() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_x(), B.v012x())) + to.v3();
		to.v4() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_x(), B.v012x())) + to.v4();
		to.v5() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_x(), B.v012x())) + to.v5();
		to.v6() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_x(), B.v012x())) + to.v6();
	}
	inline void AddAij2To(const AlignedMatrix7x3f &A, AlignedVector7f &to)
	{
		to.v0() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_00_01_02_x())) + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_10_11_12_x())) + to.v1();
		to.v2() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), A.M_20_21_22_x())) + to.v2();
		to.v3() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_x(), A.M_30_31_32_x())) + to.v3();
		to.v4() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_x(), A.M_40_41_42_x())) + to.v4();
		to.v5() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_x(), A.M_50_51_52_x())) + to.v5();
		to.v6() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_x(), A.M_60_61_62_x())) + to.v6();
	}
}

#endif