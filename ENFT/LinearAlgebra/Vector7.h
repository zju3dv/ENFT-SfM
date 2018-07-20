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


#ifndef _VECTOR_7_H_
#define _VECTOR_7_H_

#include "Utility/SSE.h"
#include "Vector6.h"

namespace LA
{

	class AlignedVector7f
	{

	public:

		inline const ENFT_SSE::__m128& v0123()  const { return m_0123; }						inline ENFT_SSE::__m128& v0123()	{ return m_0123; }
		inline const ENFT_SSE::__m128& v456x()  const { return m_456x; }						inline ENFT_SSE::__m128& v456x()	{ return m_456x; }
		inline const float& v0()	  const { return m_0123.m128_f32[0]; }			inline float& v0()		{ return m_0123.m128_f32[0]; }
		inline const float& v1()	  const { return m_0123.m128_f32[1]; }			inline float& v1()		{ return m_0123.m128_f32[1]; }
		inline const float& v2()	  const { return m_0123.m128_f32[2]; }			inline float& v2()		{ return m_0123.m128_f32[2]; }
		inline const float& v3()	  const { return m_0123.m128_f32[3]; }			inline float& v3()		{ return m_0123.m128_f32[3]; }
		inline const float& v4()	  const { return m_456x.m128_f32[0]; }			inline float& v4()		{ return m_456x.m128_f32[0]; }
		inline const float& v5()	  const { return m_456x.m128_f32[1]; }			inline float& v5()		{ return m_456x.m128_f32[1]; }
		inline const float& v6()	  const { return m_456x.m128_f32[2]; }			inline float& v6()		{ return m_456x.m128_f32[2]; }
		inline const float& reserve() const { return m_456x.m128_f32[3]; }			inline float& reserve()	{ return m_456x.m128_f32[3]; }
		inline operator const float* () const { return (const float *) this; }
		inline operator		  float* ()		  { return (float *) this; }
		inline void SetZero() { memset(this, 0, 32); }
		inline void MakeReciprocal()
		{
			m_0123 = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(m_0123, ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), m_0123));
			m_456x = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(m_456x, ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), m_456x));
		}
		inline void MakeSquareRoot()
		{
			m_0123 = ENFT_SSE::_mm_sqrt_ps(m_0123);
			m_456x = ENFT_SSE::_mm_sqrt_ps(m_456x);
		}
		inline float SquaredLength() const
		{
#if _DEBUG
			assert(reserve() == 0);
#endif
			return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_0123, m_0123), ENFT_SSE::_mm_mul_ps(m_456x, m_456x)));
		}
		inline float Dot(const AlignedVector7f &v) const
		{
#if _DEBUG
			assert(reserve() == 0 || v.reserve() == 0);
#endif
			return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_0123, v.m_0123), ENFT_SSE::_mm_mul_ps(m_456x, v.m_456x)));
		}
		inline void Scale(const ENFT_SSE::__m128 &s) { m_0123 = ENFT_SSE::_mm_mul_ps(m_0123, s); m_456x = ENFT_SSE::_mm_mul_ps(m_456x, s); }

		inline void Print() const {	printf("%f %f %f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5(), v6(), reserve()); }
		//inline void Print() const {	printf("%f %f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5(), v6()); }

	protected:

		ENFT_SSE::__m128 m_0123, m_456x;

	};

	template<class MATRIX> inline void FinishAdditionAij2To(AlignedVector7f &to) {}
	template<class MATRIX_1, class MATRIX_2> inline void FinishAdditionAij2To(AlignedVector7f &to) {}
	template<class MATRIX> inline void FinishAdditionATBTo(AlignedVector7f &to) {}
	template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(AlignedVector7f &to) {}

	template<ubyte STAGE> inline void SetReserve(AlignedVector7f &v) {}
	template<> inline void SetReserve<2>(AlignedVector7f &v) { v.reserve() = 0; }
	template<> inline void SetReserve<3>(AlignedVector7f &v) { v.reserve() = 0; }
	template<> inline void SetReserve<5>(AlignedVector7f &v) { v.reserve() = 0; }
	template<> inline void SetReserve<6>(AlignedVector7f &v) { v.reserve() = 0; }
	inline void MakeReciprocal(AlignedVector7f &v) { v.MakeReciprocal(); }
	inline void MakeSquareRoot(AlignedVector7f &v) { v.MakeSquareRoot(); }

	inline float NormL2_2(const AlignedVector7f &v) { return v.SquaredLength(); }
	inline float NormLinf(const AlignedVector7f &v)
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
		return normLinf;
	}
	inline float NormWL2_2(const AlignedVector7f &v, const AlignedVector7f &w)
	{
#if _DEBUG
		assert(v.reserve() == 0 || w.reserve() == 0);
#endif
		return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(v.v0123(), ENFT_SSE::_mm_mul_ps(v.v0123(), w.v0123())), 
									   ENFT_SSE::_mm_mul_ps(v.v456x(), ENFT_SSE::_mm_mul_ps(v.v456x(), w.v456x()))));
	}
	inline float Dot(const AlignedVector7f &A, const AlignedVector7f &B) { return A.Dot(B); }

	inline void sA(const AlignedVector7f &s, AlignedVector7f &A)
	{
		A.v0123() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.v0123());
		A.v456x() = ENFT_SSE::_mm_mul_ps(s.v456x(), A.v456x());
	}
	inline void sA(const ENFT_SSE::__m128 &s, const AlignedVector7f &A, AlignedVector7f &sA)
	{
		sA.v0123() = ENFT_SSE::_mm_mul_ps(s, A.v0123());
		sA.v456x() = ENFT_SSE::_mm_mul_ps(s, A.v456x());
	}
	inline void sApB(const ENFT_SSE::__m128 &s, const AlignedVector7f &A, const AlignedVector7f &B, AlignedVector7f &sApB)
	{
		sApB.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s, A.v0123()), B.v0123());
		sApB.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s, A.v456x()), B.v456x());
	}
	inline void AmB(const AlignedVector7f &A, const AlignedVector7f &B, AlignedVector7f &AmB)
	{
		AmB.v0123() = ENFT_SSE::_mm_sub_ps(A.v0123(), B.v0123());
		AmB.v456x() = ENFT_SSE::_mm_sub_ps(A.v456x(), B.v456x());
	}
	inline void AddABTo(const float &A, AlignedVector7f &B, AlignedVector7f &to, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_set1_ps(A);
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, B.v0123()), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, B.v456x()), to.v456x());
	}

	template<typename TYPE> class Vector7 : public Vector6<TYPE>
	{
	public:
		Vector7() {}
		Vector7(const TYPE &v0, const TYPE &v1, const TYPE &v2, const TYPE &v3, const TYPE &v4, const TYPE &v5, const TYPE &v6) : Vector6<TYPE>(v0, v1, v2, v3, v4, v5), m_v6(v6) {}
		inline const TYPE& v6() const { return m_v6; }		inline TYPE& v6() { return m_v6; }
		inline void Set(const TYPE &v0, const TYPE &v1, const TYPE &v2, const TYPE &v3, const TYPE &v4, const TYPE &v5, const TYPE &v6) { Vector6<TYPE>::Set(v0, v1, v2, v3, v4, v5); m_v6 = v6; }
		inline void Get(TYPE &v0, TYPE &v1, TYPE &v2, TYPE &v3, TYPE &v4, TYPE &v5, TYPE &v6) const { Vector6<TYPE>::Get(v0, v1, v2, v3, v4, v5); v6 = m_v6; }
		inline TYPE SquaredLength() const { return Vector6<TYPE>::SquaredLength() + m_v6 * m_v6; }
	protected:
		TYPE m_v6;
	};

	typedef Vector7< float> Vector7f;
	typedef Vector7<double> Vector7d;

}

#endif