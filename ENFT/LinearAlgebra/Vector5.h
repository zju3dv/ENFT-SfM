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

#ifndef _VECTOR_5_H_
#define _VECTOR_5_H_

#include "Vector4.h"

namespace LA
{

	class AlignedVector5f/* : public AlignedVector4f*/
	{
	public:
		inline const ENFT_SSE::__m128 &v0123() const { return m_0123; }				inline ENFT_SSE::__m128 &v0123 () { return m_0123; }
		inline const ENFT_SSE::__m128 &v4xxx() const { return m_4xxx; }				inline ENFT_SSE::__m128 &v4xxx () { return m_4xxx; }
		inline const float& v0() const { return m_0123.m128_f32[0]; }	inline float& v0() { return m_0123.m128_f32[0]; }
		inline const float& v1() const { return m_0123.m128_f32[1]; }	inline float& v1() { return m_0123.m128_f32[1]; }
		inline const float& v2() const { return m_0123.m128_f32[2]; }	inline float& v2() { return m_0123.m128_f32[2]; }
		inline const float& v3() const { return m_0123.m128_f32[3]; }	inline float& v3() { return m_0123.m128_f32[3]; }
		inline const float& v4() const { return m_4xxx.m128_f32[0]; }	inline float& v4() { return m_4xxx.m128_f32[0]; }
		inline operator float* () { return (float *) this; }
		inline operator const float* () const { return (const float *) this; }
		inline void SetZero() { v0123() = ENFT_SSE::_mm_setzero_ps(); v4() = 0.0f; }
		inline void MakeReciprocal()
		{
			v0123() = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(v0123(), ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), v0123()));
			if(v4() != 0.0f)
				v4() = 1 / v4();
		}
		inline void MakeSquareRoot()
		{
			v0123() = ENFT_SSE::_mm_sqrt_ps(v0123());
			v4() = sqrt(v4());
		}
		inline float SquaredLength() const { return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(v0123(), v0123())) + v4() * v4(); }
		inline float Dot(const AlignedVector5f &v) const { return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(v0123(), v.v0123())) + v4() * v.v4(); }
		inline void Print() const { printf("%f, %f, %f, %f %f\n", v0(), v1(), v2(), v3(), v4()); }
	protected:
		ENFT_SSE::__m128 m_0123, m_4xxx;
	};

	inline void MakeReciprocal(AlignedVector5f &v) { v.MakeReciprocal(); }
	inline void MakeSquareRoot(AlignedVector5f &v) { v.MakeSquareRoot(); }
	inline float NormL2_2(const AlignedVector5f &v) { return v.SquaredLength(); }
	inline float NormLinf(const AlignedVector5f &v)
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
		return normLinf;
	}
	inline float NormWL2_2(const AlignedVector5f &v, const AlignedVector5f &w)
	{
		return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(v.v0123(), ENFT_SSE::_mm_mul_ps(v.v0123(), w.v0123()))) + v.v4() * v.v4() * w.v4();
	}
	inline float Dot(const AlignedVector5f &A, const AlignedVector5f &B) { return A.Dot(B); }

	inline void AddABTo(const float &A, const AlignedVector5f &B, AlignedVector5f &to)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A), B.v0123()), to.v0123());
		to.v4() = A * B.v4() + to.v4();
	}
	inline void sA(const AlignedVector5f &s, AlignedVector5f &A)
	{
		A.v0123() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.v0123());
		A.v4() *= s.v4();
	}
	inline void AmB(const AlignedVector5f &A, const AlignedVector5f &B, AlignedVector5f &AmB)
	{
		AmB.v0123() = ENFT_SSE::_mm_sub_ps(A.v0123(), B.v0123());
		AmB.v4() = A.v4() - B.v4();
	}

	template<typename TYPE> class Vector5 : public Vector4<TYPE>
	{
	public:
		Vector5() {}
		Vector5(const TYPE &v0, const TYPE &v1, const TYPE &v2, const TYPE &v3, const TYPE &v4) : Vector4<TYPE>(v0, v1, v2, v3), m_v4(v4) {}
		inline const TYPE& v4() const { return m_v4; }		inline TYPE& v4() { return m_v4; }
		inline void Set(const TYPE &v0, const TYPE &v1, const TYPE &v2, const TYPE &v3, const TYPE &v4) { Vector4<TYPE>::Set(v0, v1, v2, v3); m_v4 = v4; }
		inline void Get(TYPE &v0, TYPE &v1, TYPE &v2, TYPE &v3, TYPE &v4) const { Vector4<TYPE>::Get(v0, v1, v2, v3); v4 = m_v4; }
		inline TYPE SquaredLength() const { return Vector4<TYPE>::SquaredLength() + m_v4 * m_v4; }
	protected:
		TYPE m_v4;
	};

	typedef Vector5< float> Vector5f;
	typedef Vector5<double> Vector5d;

}

#endif