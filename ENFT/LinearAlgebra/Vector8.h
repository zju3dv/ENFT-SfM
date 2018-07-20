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

#ifndef _VECTOR_8_H_
#define _VECTOR_8_H_

#include "Utility/SSE.h"

namespace LA
{

	class AlignedVector8f
	{

	public:

		AlignedVector8f() {}
		AlignedVector8f(const float &v) { v0123() = v4567() = ENFT_SSE::_mm_set1_ps(v); }
		AlignedVector8f(const ENFT_SSE::__m128 &v0123, const ENFT_SSE::__m128 &v4567) { m_0123 = v0123; m_4567 = v4567; }

		inline operator float* () { return (float *) this; }
		inline operator const float* () const { return (const float *) this; }
		inline const ENFT_SSE::__m128& v0123() const { return m_0123; }					inline ENFT_SSE::__m128& v0123() { return m_0123; }
		inline const ENFT_SSE::__m128& v4567() const { return m_4567; }					inline ENFT_SSE::__m128& v4567() { return m_4567; }
		inline const float& v0() const { return m_0123.m128_f32[0]; }			inline float& v0() { return m_0123.m128_f32[0]; }
		inline const float& v1() const { return m_0123.m128_f32[1]; }			inline float& v1() { return m_0123.m128_f32[1]; }
		inline const float& v2() const { return m_0123.m128_f32[2]; }			inline float& v2() { return m_0123.m128_f32[2]; }
		inline const float& v3() const { return m_0123.m128_f32[3]; }			inline float& v3() { return m_0123.m128_f32[3]; }
		inline const float& v4() const { return m_4567.m128_f32[0]; }			inline float& v4() { return m_4567.m128_f32[0]; }
		inline const float& v5() const { return m_4567.m128_f32[1]; }			inline float& v5() { return m_4567.m128_f32[1]; }
		inline const float& v6() const { return m_4567.m128_f32[2]; }			inline float& v6() { return m_4567.m128_f32[2]; }
		inline const float& v7() const { return m_4567.m128_f32[3]; }			inline float& v7() { return m_4567.m128_f32[3]; }
		inline void operator *= (const ENFT_SSE::__m128 &s) { v0123() = ENFT_SSE::_mm_mul_ps(s, v0123()); v4567() = ENFT_SSE::_mm_mul_ps(s, v4567()); }
		inline void SetZero() { memset(this, 0, sizeof(LA::AlignedVector8f)); }
		inline void MakeReciprocal()
		{
			v0123() = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(v0123(), ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), v0123()));
			v4567() = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(v4567(), ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), v4567()));
		}
		inline void MakeSquareRoot()
		{
			v0123() = ENFT_SSE::_mm_sqrt_ps(v0123());
			v4567() = ENFT_SSE::_mm_sqrt_ps(v4567());
		}
		inline float SquaredLength() const { return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(v0123(), v0123()), ENFT_SSE::_mm_mul_ps(v4567(), v4567()))); }
		inline float Dot(const AlignedVector8f &v) const { return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(v0123(), v.v0123()), ENFT_SSE::_mm_mul_ps(v4567(), v.v4567()))); }
		inline void Print() const { printf("%f %f %f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5(), v6(), v7()); }

	protected:

		ENFT_SSE::__m128 m_0123, m_4567;

	};

	template<class MATRIX> inline void FinishAdditionAij2To(AlignedVector8f &to) {}
	template<class MATRIX> inline void FinishAdditionATBTo(AlignedVector8f &to) {}
	template<class MATRIX_1, class MATRIX_2> inline void FinishAdditionAij2To(AlignedVector8f &to) {}
	template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(AlignedVector8f &to) {}

	template<ubyte STAGE> inline void SetReserve(AlignedVector8f &v) {}
	inline void MakeReciprocal(AlignedVector8f &v) { v.MakeReciprocal(); }
	inline void MakeSquareRoot(AlignedVector8f &v) { v.MakeSquareRoot(); }
	inline float NormL2_2(const AlignedVector8f &v) { return v.SquaredLength(); }
	inline float NormLinf(const AlignedVector8f &v)
	{
		float tmp, normLinf = fabs(v.v0());
		if((tmp = fabs(v.v1())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v2())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v3())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v4())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v5())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v6())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v7())) > normLinf)
			normLinf = tmp;
		return normLinf;
	}
	inline float NormWL2_2(const AlignedVector8f &v, const AlignedVector8f &w)
	{
		return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(v.v0123(), ENFT_SSE::_mm_mul_ps(v.v0123(), w.v0123())), ENFT_SSE::_mm_mul_ps(v.v4567(), ENFT_SSE::_mm_mul_ps(v.v4567(), w.v4567()))));
	}
	inline float Dot(const AlignedVector8f &A, const AlignedVector8f &B) { return A.Dot(B); }
	inline void sA(const AlignedVector8f &s, AlignedVector8f &A)
	{
		A.v0123() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.v0123());
		A.v4567() = ENFT_SSE::_mm_mul_ps(s.v4567(), A.v4567());
	}
	inline void sA(const ENFT_SSE::__m128 &s, const AlignedVector8f &A, AlignedVector8f &sA)
	{
		sA.v0123() = ENFT_SSE::_mm_mul_ps(s, A.v0123());
		sA.v4567() = ENFT_SSE::_mm_mul_ps(s, A.v4567());
	}
	inline void sApB(const ENFT_SSE::__m128 &s, const AlignedVector8f &A, const AlignedVector8f &B, AlignedVector8f &sApB)
	{
		sApB.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s, A.v0123()), B.v0123());
		sApB.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s, A.v4567()), B.v4567());
	}
	inline void ApB(const AlignedVector8f &A, const AlignedVector8f &B, AlignedVector8f &ApB)
	{
		ApB.v0123() = ENFT_SSE::_mm_add_ps(A.v0123(), B.v0123());
		ApB.v4567() = ENFT_SSE::_mm_add_ps(A.v4567(), B.v4567());
	}
	inline void AmB(const AlignedVector8f &A, const AlignedVector8f &B, AlignedVector8f &AmB)
	{
		AmB.v0123() = ENFT_SSE::_mm_sub_ps(A.v0123(), B.v0123());
		AmB.v4567() = ENFT_SSE::_mm_sub_ps(A.v4567(), B.v4567());
	}
}

#endif