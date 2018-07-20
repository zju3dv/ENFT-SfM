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


#ifndef _VECTOR_6_H_
#define _VECTOR_6_H_

#include "Utility/SSE.h"
#include "Vector5.h"
#if _DEBUG
#include "Utility/Utility.h"
#endif

namespace LA
{

	class AlignedVector6f
	{

	public:

		inline const ENFT_SSE::__m128& v0123()   const { return m_0123; }						inline ENFT_SSE::__m128& v0123()	 { return m_0123; }
		inline const ENFT_SSE::__m128& v45xx()   const { return m_45xx; }						inline ENFT_SSE::__m128& v45xx()	 { return m_45xx; }
		inline const float& v0()	   const { return m_0123.m128_f32[0]; }			inline float& v0()		 { return m_0123.m128_f32[0]; }
		inline const float& v1()	   const { return m_0123.m128_f32[1]; }			inline float& v1()		 { return m_0123.m128_f32[1]; }
		inline const float& v2()	   const { return m_0123.m128_f32[2]; }			inline float& v2()		 { return m_0123.m128_f32[2]; }
		inline const float& v3()	   const { return m_0123.m128_f32[3]; }			inline float& v3()		 { return m_0123.m128_f32[3]; }
		inline const float& v4()	   const { return m_45xx.m128_f32[0]; }			inline float& v4()		 { return m_45xx.m128_f32[0]; }
		inline const float& v5()	   const { return m_45xx.m128_f32[1]; }			inline float& v5()		 { return m_45xx.m128_f32[1]; }
		inline const float& reserve0() const { return m_45xx.m128_f32[2]; }			inline float& reserve0() { return m_45xx.m128_f32[2]; }
		inline const float& reserve1() const { return m_45xx.m128_f32[3]; }			inline float& reserve1() { return m_45xx.m128_f32[3]; }
		inline operator const float* () const { return (float *) this; }
		inline operator		  float* ()		  { return (float *) this; }
		inline void AddReserveTo45  () { v4() = reserve0() + v4(); v5() = reserve1() + v5(); reserve0() = reserve1() = 0.0f; }
		inline void SetReserve45() { memcpy(&reserve0(), &v4(), 8); }
		inline void SetReserve55() { reserve0() = reserve1() = v5(); }
		inline void SetZero() { memset(this, 0, 32); }
		inline void SetReserveZero  () { memset(&m_45xx.m128_f32[2], 0, 8); }
		inline void MakeReciprocal()
		{
			m_0123 = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(m_0123, ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), m_0123));
			m_45xx = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(m_45xx, ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), m_45xx));
		}
		inline void MakeSquareRoot()
		{
			m_0123 = ENFT_SSE::_mm_sqrt_ps(m_0123);
			m_45xx = ENFT_SSE::_mm_sqrt_ps(m_45xx);
		}
		inline float SquaredLength() const
		{
			return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(m_0123, m_0123)) + v4() * v4() + v5() * v5();
		}
		inline float Dot(const AlignedVector6f &v) const
		{
			return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(v0123(), v.v0123())) + v4() * v.v4() + v5() * v.v5();
		}
		inline float Dot(const AlignedVector5f &v) const
		{
			return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(v0123(), v.v0123())) + v4() * v.v4() + v5();
		}
		inline void Print() const {	printf("%f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5()); }

#if _DEBUG
		inline void AssertReserve45() const { IO::Assert(reserve0() == v4() && reserve1() == v5(), "v45 = (%f, %f), reserve = (%f, %f)", v4(), v5(), reserve0(), reserve1()); }
		inline void AssertReserve55() const { IO::Assert(reserve0() == v4() && reserve1() == v5(), "v5 = %f, reserve = (%f, %f)", v5(), reserve0(), reserve1()); }
#endif

	protected:

		ENFT_SSE::__m128 m_0123, m_45xx;

	};

	template<class MATRIX> inline void FinishAdditionAij2To(AlignedVector6f &to) { to.AddReserveTo45(); }
	template<class MATRIX> inline void FinishAdditionATBTo(AlignedVector6f &to) { to.AddReserveTo45(); }
	template<class MATRIX_1, class MATRIX_2> inline void FinishAdditionAij2To(AlignedVector6f &to) { to.AddReserveTo45(); }
	template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(AlignedVector6f &to) { to.AddReserveTo45(); }

	template<ubyte STAGE> inline void SetReserve(AlignedVector6f &v) {}
	template<> inline void SetReserve<1>(AlignedVector6f &v) { v.SetReserve45(); }
	template<> inline void SetReserve<2>(AlignedVector6f &v) { v.SetReserve45(); }
	template<> inline void SetReserve<3>(AlignedVector6f &v) { v.SetReserve45(); }
	template<> inline void SetReserve<5>(AlignedVector6f &v) { v.SetReserve45(); }
	template<> inline void SetReserve<6>(AlignedVector6f &v) { v.SetReserve45(); }
	inline void MakeReciprocal(AlignedVector6f &v) { v.MakeReciprocal(); }
	inline void MakeSquareRoot(AlignedVector6f &v) { v.MakeSquareRoot(); }

	inline float NormL2_2(const AlignedVector6f &v) { return v.SquaredLength(); }
	inline float NormLinf(const AlignedVector6f &v)
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
		return normLinf;
	}
	inline float NormWL2_2(const AlignedVector6f &v, const AlignedVector6f &w)
	{
		return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(v.v0123(), ENFT_SSE::_mm_mul_ps(v.v0123(), w.v0123()))) + v.v4() * v.v4() * w.v4() + v.v5() * v.v5() * w.v5();
	}
	inline float Dot(const AlignedVector6f &A, const AlignedVector6f &B) { return A.Dot(B); }
	inline void sA(const float &s, const AlignedVector6f &A, AlignedVector6f &sA)
	{
		sA.v0123() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.v0123());
		sA.v4() = s * A.v4();
		sA.v5() = s * A.v5();
	}
	inline void sA(const ENFT_SSE::__m128 &s, AlignedVector6f &A)
	{
		A.v0123() = ENFT_SSE::_mm_mul_ps(s, A.v0123());
		A.v4() *= s.m128_f32[0];
		A.v5() *= s.m128_f32[0];
	}
	inline void sA(const ENFT_SSE::__m128 &s, const AlignedVector6f &A, AlignedVector6f &sA)
	{
		sA.v0123() = ENFT_SSE::_mm_mul_ps(s, A.v0123());
		sA.v4() = s.m128_f32[0] * A.v4();
		sA.v5() = s.m128_f32[0] * A.v5();
	}
	inline void sA(const AlignedVector6f &s, AlignedVector6f &A)
	{
		A.v0123() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.v0123());
		A.v45xx() = ENFT_SSE::_mm_mul_ps(s.v45xx(), A.v45xx());
	}
	inline void sApB(const float &s, const AlignedVector6f &A, const AlignedVector6f &B, AlignedVector6f &sApB)
	{
		sApB.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.v0123()), B.v0123());
		sApB.v4() = s * A.v4() + B.v4();
		sApB.v5() = s * A.v5() + B.v5();
	}
	inline void sApB(const ENFT_SSE::__m128 &s, const AlignedVector6f &A, const AlignedVector6f &B, AlignedVector6f &sApB)
	{
		sApB.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s, A.v0123()), B.v0123());
		sApB.v4() = s.m128_f32[0] * A.v4() + B.v4();
		sApB.v5() = s.m128_f32[0] * A.v5() + B.v5();
	}
	inline void AmB(const AlignedVector6f &A, const AlignedVector6f &B, AlignedVector6f &AmB)
	{
		AmB.v0123() = ENFT_SSE::_mm_sub_ps(A.v0123(), B.v0123());
		AmB.v4() = A.v4() - B.v4();
		AmB.v5() = A.v5() - B.v5();
	}
	inline void AddATBTo(const AlignedVector6f &A, const float &B, AlignedVector6f &to)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), ENFT_SSE::_mm_set1_ps(B)), to.v0123());
		to.v4() = A.v4() * B + to.v4();
		to.v5() = A.v5() * B + to.v5();
	}
	inline void AddAij2To(const AlignedVector6f &A, AlignedVector6f &to)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), A.v0123()), to.v0123());
		to.v4() = A.v4() * A.v4() + to.v4();
		to.v5() = A.v5() * A.v5() + to.v5();
	}

	template<typename TYPE> class Vector6 : public Vector5<TYPE>
	{
	public:
		Vector6() {}
		Vector6(const TYPE &v0, const TYPE &v1, const TYPE &v2, const TYPE &v3, const TYPE &v4, const TYPE &v5) : Vector5<TYPE>(v0, v1, v2, v3, v4), m_v5(v5) {}
		inline const TYPE& v5() const { return m_v5; }		inline TYPE& v5() { return m_v5; }
		inline void Set(const TYPE &v0, const TYPE &v1, const TYPE &v2, const TYPE &v3, const TYPE &v4, const TYPE &v5) { Vector5<TYPE>::Set(v0, v1, v2, v3, v4); m_v5 = v5; }
		inline void Get(TYPE &v0, TYPE &v1, TYPE &v2, TYPE &v3, TYPE &v4, TYPE &v5) const { Vector5<TYPE>::Get(v0, v1, v2, v3, v4); v5 = m_v5; }
		inline TYPE SquaredLength() const { return Vector5<TYPE>::SquaredLength() + m_v5 * m_v5; }
	protected:
		TYPE m_v5;
	};

	typedef Vector6< float> Vector6f;
	typedef Vector6<double> Vector6d;

}

#endif