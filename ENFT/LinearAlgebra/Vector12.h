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

#ifndef _VECTOR_12_H_
#define _VECTOR_12_H_

#include "Vector8.h"

namespace LA
{

	class AlignedVector12f : public AlignedVector8f
	{
	public:
		inline const ENFT_SSE::__m128& v891011() const { return m_891011; }
		inline		 ENFT_SSE::__m128& v891011()		 { return m_891011; }
		inline const float& v8 () const { return m_891011.m128_f32[0]; }	inline float& v8 () { return m_891011.m128_f32[0]; }
		inline const float& v9 () const { return m_891011.m128_f32[1]; }	inline float& v9 () { return m_891011.m128_f32[1]; }
		inline const float& v10() const { return m_891011.m128_f32[2]; }	inline float& v10() { return m_891011.m128_f32[2]; }
		inline const float& v11() const { return m_891011.m128_f32[3]; }	inline float& v11() { return m_891011.m128_f32[3]; }
		inline void operator *= (const ENFT_SSE::__m128 &s)
		{
			m_0123 = ENFT_SSE::_mm_mul_ps(m_0123, s);
			m_4567 = ENFT_SSE::_mm_mul_ps(m_4567, s);
			m_891011 = ENFT_SSE::_mm_mul_ps(m_891011, s);
		}
		inline void SetZero() { memset(this, 0, sizeof(AlignedVector12f)); }
		inline void MakeReciprocal()
		{
			v0123() = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(v0123(), ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), v0123()));
			v4567() = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(v4567(), ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), v4567()));
			v891011() = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(v891011(), ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), v891011()));
		}
		inline void MakeSquareRoot()
		{
			v0123() = ENFT_SSE::_mm_sqrt_ps(v0123());
			v4567() = ENFT_SSE::_mm_sqrt_ps(v4567());
			v891011() = ENFT_SSE::_mm_sqrt_ps(v891011());
		}
		inline float SquaredLength() const
		{
			return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(v0123(), v0123()), ENFT_SSE::_mm_mul_ps(v4567(), v4567())), ENFT_SSE::_mm_mul_ps(v891011(), v891011())));
		}
		inline float Dot(const AlignedVector12f &v) const
		{
			return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(v0123(), v.v0123()), ENFT_SSE::_mm_mul_ps(v4567(), v.v4567())), ENFT_SSE::_mm_mul_ps(v891011(), v.v891011())));
		}
		inline void Print() const
		{
			printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", v0(), v1(), v2(), v3(), v4(), v5(), v6(), v7(), v8(), v9(), v10(), v11());
		}

	protected:

		ENFT_SSE::__m128 m_891011;
	};

	inline void MakeReciprocal(AlignedVector12f &v) { v.MakeReciprocal(); }
	inline void MakeSquareRoot(AlignedVector12f &v) { v.MakeSquareRoot(); }
	inline float NormL2_2(const AlignedVector12f &v) { return v.SquaredLength(); }
	inline float NormLinf(const AlignedVector12f &v)
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
		if((tmp = fabs(v.v8())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v9())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v10())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v11())) > normLinf)
			normLinf = tmp;
		return normLinf;
	}
	inline float NormWL2_2(const AlignedVector12f &v, const AlignedVector12f &w)
	{
		return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(v.v0123(), ENFT_SSE::_mm_mul_ps(v.v0123(), w.v0123())), 
												  ENFT_SSE::_mm_mul_ps(v.v4567(), ENFT_SSE::_mm_mul_ps(v.v4567(), w.v4567()))), 
												  ENFT_SSE::_mm_mul_ps(v.v891011(), ENFT_SSE::_mm_mul_ps(v.v891011(), w.v891011()))));
	}
	inline float Dot(const AlignedVector12f &A, const AlignedVector12f &B) { return A.Dot(B); }
	inline void sA(const AlignedVector12f &s, AlignedVector12f &A)
	{
		A.v0123() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.v0123());
		A.v4567() = ENFT_SSE::_mm_mul_ps(s.v4567(), A.v4567());
		A.v891011() = ENFT_SSE::_mm_mul_ps(s.v891011(), A.v891011());
	}
	inline void sA(const ENFT_SSE::__m128 &s, const AlignedVector12f &A, AlignedVector12f &sA)
	{
		sA.v0123() = ENFT_SSE::_mm_mul_ps(s, A.v0123());
		sA.v4567() = ENFT_SSE::_mm_mul_ps(s, A.v4567());
		sA.v891011() = ENFT_SSE::_mm_mul_ps(s, A.v891011());
	}
	inline void sApB(const ENFT_SSE::__m128 &s, const AlignedVector12f &A, const AlignedVector12f &B, AlignedVector12f &sApB)
	{
		sApB.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s, A.v0123()), B.v0123());
		sApB.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s, A.v4567()), B.v4567());
		sApB.v891011() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s, A.v891011()), B.v891011());
	}
	inline void ApB(const AlignedVector12f &A, const AlignedVector12f &B, AlignedVector12f &ApB)
	{
		ApB.v0123() = ENFT_SSE::_mm_add_ps(A.v0123(), B.v0123());
		ApB.v4567() = ENFT_SSE::_mm_add_ps(A.v4567(), B.v4567());
		ApB.v891011() = ENFT_SSE::_mm_add_ps(A.v891011(), B.v891011());
	}
	inline void AmB(const AlignedVector12f &A, const AlignedVector12f &B, AlignedVector12f &AmB)
	{
		AmB.v0123() = ENFT_SSE::_mm_sub_ps(A.v0123(), B.v0123());
		AmB.v4567() = ENFT_SSE::_mm_sub_ps(A.v4567(), B.v4567());
		AmB.v891011() = ENFT_SSE::_mm_sub_ps(A.v891011(), B.v891011());
	}
}

#endif