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

#ifndef _VECTOR_4_H_
#define _VECTOR_4_H_

#include "Utility/SSE.h"
#include "Vector3.h"

namespace LA
{

	class AlignedVector4f
	{

	public:

		inline AlignedVector4f() {}
		inline AlignedVector4f(const float &v0, const float &v1, const float &v2, const float &v3) { v0123() = ENFT_SSE::_mm_setr_ps(v0, v1, v2, v3); }

		inline const ENFT_SSE::__m128 &v0123 () const { return m_v0123; }				inline ENFT_SSE::__m128 &v0123 () { return m_v0123; }
		inline const float& v0	   () const { return m_v0123.m128_f32[0]; }	inline float& v0	 ()	{ return m_v0123.m128_f32[0]; }
		inline const float& v1	   () const { return m_v0123.m128_f32[1]; }	inline float& v1	 () { return m_v0123.m128_f32[1]; }
		inline const float& v2	   () const { return m_v0123.m128_f32[2]; }	inline float& v2	 () { return m_v0123.m128_f32[2]; }
		inline const float& v3	   () const { return m_v0123.m128_f32[3]; }	inline float& v3	 () { return m_v0123.m128_f32[3]; }
		inline operator float* () { return (float *) this; }
		inline operator const float* () const { return (const float *) this; }
		inline void Set(const float &v0, const float &v1, const float &v2, const float &v3) { v0123() = ENFT_SSE::_mm_setr_ps(v0, v1, v2, v3); }
		inline void SetZero() { v0123() = ENFT_SSE::_mm_setzero_ps(); }
		inline void Scale(const float &s) { v0123() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), v0123()); }
		inline void MakeReciprocal() { v0123() = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(v0123(), ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1), v0123())); }
		inline void MakeSquareRoot() { v0123() = ENFT_SSE::_mm_sqrt_ps(v0123()); }
		inline float SquaredLength() const { return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(v0123(), v0123())); }
		inline float SquaredDistance(const AlignedVector4f &v) const
		{
			return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_sub_ps(m_v0123, v.m_v0123), ENFT_SSE::_mm_sub_ps(m_v0123, v.m_v0123)));
		}
		inline float Dot(const AlignedVector4f &v) const { return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(v0123(), v.v0123())); }
		inline void Normalize() { m_v0123 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1 / sqrt(ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(m_v0123, m_v0123)))), m_v0123); }
		inline void operator += (const LA::AlignedVector4f &v) { m_v0123 = ENFT_SSE::_mm_add_ps(m_v0123, v.m_v0123); }
		inline void operator *= (const float &s) { m_v0123 = ENFT_SSE::_mm_mul_ps(m_v0123, ENFT_SSE::_mm_set1_ps(s)); }
		inline void Print() const { printf("%f, %f, %f, %f\n", v0(), v1(), v2(), v3()); }

	protected:

		ENFT_SSE::__m128 m_v0123;
	};

	inline void AmB(const LA::AlignedVector4f &A, const LA::AlignedVector4f &B, LA::AlignedVector4f &AmB)
	{
		AmB.v0123() = ENFT_SSE::_mm_sub_ps(A.v0123(), B.v0123());
	}
	inline void ComputeError(const AlignedVector4f &A, const AlignedVector4f &B, float &absErr, float &relErr)
	{
		LA::AlignedVector4f AmB;
		LA::AmB(A, B, AmB);
		absErr = AmB.SquaredLength();
		const float lenA = sqrt(A.SquaredLength()), lenB = sqrt(B.SquaredLength()), lenMax = std::max(lenA, lenB);
		if(lenMax == 0)
			relErr = 0;
		else
			relErr = absErr / lenMax;
	}
	inline void AddABTo(const float &A, const AlignedVector4f &B, AlignedVector4f &to)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A), B.v0123()), to.v0123());
	}

	template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(AlignedVector4f &to) {}
	template<class MATRIX> inline void FinishAdditionAij2To(AlignedVector4f &to) {}
	template<ubyte STAGE> inline void SetReserve(AlignedVector4f &v) {}
	inline void MakeReciprocal(AlignedVector4f &v) { v.MakeReciprocal(); }
	inline void MakeSquareRoot(AlignedVector4f &v) { v.MakeSquareRoot(); }
	inline float NormL2_2(const AlignedVector4f &v) { return v.SquaredLength(); }
	inline float NormLinf(const AlignedVector4f &v)
	{
		float tmp, normLinf = fabs(v.v0());
		if((tmp = fabs(v.v1())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v2())) > normLinf)
			normLinf = tmp;
		if((tmp = fabs(v.v3())) > normLinf)
			normLinf = tmp;
		return normLinf;
	}
	inline float NormWL2_2(const AlignedVector4f &v, const AlignedVector4f &w)
	{
		return ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(v.v0123(), ENFT_SSE::_mm_mul_ps(v.v0123(), w.v0123())));
	}
	inline float Dot(const AlignedVector4f &A, const AlignedVector4f &B) { return A.Dot(B); }
	inline void sA(const float &s, const AlignedVector4f &A, AlignedVector4f &sA) { sA.v0123() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.v0123()); }
	inline void sA(const AlignedVector4f &s, AlignedVector4f &A) { A.v0123() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.v0123()); }
	inline void sApB(const float &s, const AlignedVector4f &A, const AlignedVector4f &B, AlignedVector4f &sApB)
	{
		sApB.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.v0123()), B.v0123());
	}

	inline void AddAij2To(const AlignedVector3f &A, AlignedVector4f &to)
	{
#if _DEBUG
		assert(A.reserve() == 1.0f);
#endif
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v012x(), A.v012x()), to.v0123());
	}
	inline void AddsATo(const float &s, const AlignedVector3f &A, AlignedVector4f &to)
	{
#if _DEBUG
		assert(A.reserve() == 1.0f);
#endif
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.v012x()), to.v0123());
	}

	template<typename TYPE> class Vector4
	{

	public:

		Vector4() {}
		Vector4(const TYPE &v) : m_v0(v), m_v1(v), m_v2(v), m_v3(v) {}
		Vector4(const TYPE &v0, const TYPE &v1, const TYPE &v2, const TYPE &v3) : m_v0(v0), m_v1(v1), m_v2(v2), m_v3(v3) {}

		inline const TYPE& v0() const { return m_v0; }		inline TYPE& v0() { return m_v0; }
		inline const TYPE& v1() const { return m_v1; }		inline TYPE& v1() { return m_v1; }
		inline const TYPE& v2() const { return m_v2; }		inline TYPE& v2() { return m_v2; }
		inline const TYPE& v3() const { return m_v3; }		inline TYPE& v3() { return m_v3; }

		inline void Set(const TYPE &v0, const TYPE &v1, const TYPE &v2, const TYPE &v3) { m_v0 = v0; m_v1 = v1; m_v2 = v2; m_v3 = v3; }
		inline void Get(TYPE &v0, TYPE &v1, TYPE &v2, TYPE &v3) const { v0 = m_v0; v1 = m_v1; v2 = m_v2; v3 = m_v3; }
		inline operator TYPE* () { return &m_v0; }
		inline operator const TYPE* () const { return &m_v0; }
		inline TYPE Dot(const Vector4<TYPE> &v) const { return m_v0 * v.m_v0 + m_v1 * v.m_v1 + m_v2 * v.m_v2 + m_v3 * v.m_v3; }
		inline TYPE SquaredLength() const { return m_v0 * m_v0 + m_v1 * m_v1 + m_v2 * m_v2 + m_v3 * m_v3; }
		inline TYPE SquaredDistance(const Vector4<TYPE> &v) const
		{
			return (m_v0 - v.m_v0) * (m_v0 - v.m_v0)
				 + (m_v1 - v.m_v1) * (m_v1 - v.m_v1)
				 + (m_v2 - v.m_v2) * (m_v2 - v.m_v2)
				 + (m_v3 - v.m_v3) * (m_v3 - v.m_v3);
		}
		inline void operator += (const LA::Vector4<TYPE> &v) { m_v0 += v.m_v0; m_v1 += v.m_v1; m_v2 += v.m_v2; m_v3 += v.m_v3; }
		inline void operator *= (const TYPE &s) { m_v0 *= s; m_v1 *= s; m_v2 *= s; m_v3 *= s; }
		inline void Normalize() { const TYPE norm = 1 / sqrt(SquaredLength()); m_v0 *= norm; m_v1 *= norm; m_v2 *= norm; m_v3 *= norm; }
		inline void Print(const bool e = false) const
		{
			if(e)
				printf("%e %e %e %e\n", v0(), v1(), v2(), v3());
			else
				printf("%lf %lf %lf %lf\n", v0(), v1(), v2(), v3());
		}
		inline void Save(FILE *fp, const bool e = false) const
		{
			if(e)
				fprintf(fp, "%e %e %e %e\n", v0(), v1(), v2(), v3());
			else
				fprintf(fp, "%lf %lf %lf %lf\n", v0(), v1(), v2(), v3());
		}
		inline void Load(FILE *fp) { fscanf(fp, "%lf %lf %lf %lf", &v0(), &v1(), &v2(), &v3()); }

	protected:

		TYPE m_v0, m_v1, m_v2, m_v3;
	};

	typedef Vector4<   int> Vector4i;
	typedef Vector4< float> Vector4f;
	typedef Vector4<double> Vector4d;
	typedef Vector4<ubyte>	Vector4ub;

	template<typename TYPE> inline void AmB(const Vector4<TYPE> &A, const Vector4<TYPE> &B, Vector4<TYPE> &AmB)
	{
		AmB.v0() = A.v0() - B.v0();
		AmB.v1() = A.v1() - B.v1();
		AmB.v2() = A.v2() - B.v2();
		AmB.v3() = A.v3() - B.v3();
	}
	template<typename TYPE> inline void ComputeError(const Vector4<TYPE> &A, const Vector4<TYPE> &B, TYPE &absErr, TYPE &relErr)
	{
		absErr = sqrt((A.v0() - B.v0()) * (A.v0() - B.v0()) + (A.v1() - B.v1()) * (A.v1() - B.v1()) + 
					  (A.v2() - B.v2()) * (A.v2() - B.v2()) + (A.v3() - B.v3()) * (A.v3() - B.v3()));
		const TYPE lenA = sqrt(A.SquaredLength()), lenB = sqrt(B.SquaredLength()), lenMax = std::max(lenA, lenB);
		if(lenMax == 0)
			relErr = 0;
		else
			relErr = absErr / lenMax;
	}

}

#endif
