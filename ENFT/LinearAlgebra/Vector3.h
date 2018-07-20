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

#ifndef _VECTOR_3_H_
#define _VECTOR_3_H_

#include "Utility/SSE.h"
#include "Vector2.h"               

namespace LA
{

	class AlignedVector3f
	{

	public:

		inline AlignedVector3f() {}
		inline AlignedVector3f(const float &v0, const float &v1, const float &v2){ m_v012x = ENFT_SSE::_mm_setr_ps(v0, v1, v2, 1.0f); }
		inline AlignedVector3f(const ENFT_SSE::__m128 &v012x) : m_v012x(v012x) {}

		inline const ENFT_SSE::__m128 &v012x () const { return m_v012x; }				inline ENFT_SSE::__m128 &v012x () { return m_v012x; }
		inline const float& v0	   () const { return m_v012x.m128_f32[0]; }	inline float& v0	 ()	{ return m_v012x.m128_f32[0]; }
		inline const float& v1	   () const { return m_v012x.m128_f32[1]; }	inline float& v1	 () { return m_v012x.m128_f32[1]; }
		inline const float& v2	   () const { return m_v012x.m128_f32[2]; }	inline float& v2	 () { return m_v012x.m128_f32[2]; }
		inline const float& reserve() const { return m_v012x.m128_f32[3]; }	inline float& reserve() { return m_v012x.m128_f32[3]; }

		inline operator const float* () const { return (const float *) this; }
		inline operator		  float* ()		  { return (float *) this; }
		inline void operator = (const AlignedVector3f &v) { m_v012x = v.m_v012x; }
		//inline AlignedVector3f operator + (const AlignedVector3f &v) {
		//	return VALUE_FROM_SSE(AlignedVector3f, ENFT_SSE::_mm_add_ps(m_v012x, v.m_v012x));
		//}
		inline void operator += (const ENFT_SSE::__m128 &v012x) { m_v012x = ENFT_SSE::_mm_add_ps(m_v012x, v012x); }
		inline void operator += (const AlignedVector3f &v) { m_v012x = ENFT_SSE::_mm_add_ps(m_v012x, v.m_v012x); }
		inline AlignedVector3f operator - (const AlignedVector3f &v) { return AlignedVector3f(ENFT_SSE::_mm_sub_ps(m_v012x, v.m_v012x)); }
		inline void operator -= (const ENFT_SSE::__m128 &v012x) { m_v012x = ENFT_SSE::_mm_sub_ps(m_v012x, v012x); }
		inline void operator -= (const AlignedVector3f &v) { m_v012x = ENFT_SSE::_mm_sub_ps(m_v012x, v.m_v012x); }
		inline void operator *= (const float &s) { m_v012x = ENFT_SSE::_mm_mul_ps(m_v012x, ENFT_SSE::_mm_set1_ps(s)); }
		inline void operator *= (const ENFT_SSE::__m128 &s) { m_v012x = ENFT_SSE::_mm_mul_ps(m_v012x, s); }

		inline void SetZero() { memset(this, 0, 16); }
		inline void Set(const float &v0, const float &v1, const float &v2) { m_v012x = ENFT_SSE::_mm_setr_ps(v0, v1, v2, 1.0f); }
		inline void Set(const double *v012) { Set(float(v012[0]), float(v012[1]), float(v012[2])); }
		inline void Set(const ENFT_SSE::__m128 &v012x) { m_v012x = v012x; }
		inline void Get(double *v012) const { v012[0] = double(v0()); v012[1] = double(v1()); v012[2] = double(v2()); }
		inline float Length() const { return sqrt(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_v012x, m_v012x))); }
		inline float SquaredLength() const { return ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_v012x, m_v012x)); }
		inline float SquaredDistance(const AlignedVector3f &v) const
		{
			return ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_sub_ps(m_v012x, v.m_v012x), ENFT_SSE::_mm_sub_ps(m_v012x, v.m_v012x)));
		}
		inline void Normalize() { m_v012x = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1 / sqrt(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_v012x, m_v012x)))), m_v012x); }
		inline const float MaximalElement() const { return std::max(std::max(v0(), v1()), v2()); }
		inline float Dot(const AlignedVector3f &v) const { return ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_v012x, v.m_v012x)); }
		inline void Scale(const float &s) { m_v012x = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), m_v012x); }
		inline void Scale(const ENFT_SSE::__m128 &s) { m_v012x = ENFT_SSE::_mm_mul_ps(s, m_v012x); }

		inline void MakeReciprocal() { m_v012x = ENFT_SSE::_mm_and_ps(ENFT_SSE::_mm_cmpneq_ps(m_v012x, ENFT_SSE::_mm_setzero_ps()), ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_set1_ps(1.0f), m_v012x)); }
		inline void MakeSquareRoot() { m_v012x = ENFT_SSE::_mm_sqrt_ps(m_v012x); }

		inline void Print() const { printf("(%f, %f, %f)\n", m_v012x.m128_f32[0], m_v012x.m128_f32[1], m_v012x.m128_f32[2]); }

	protected:

		ENFT_SSE::__m128 m_v012x;

	};

	template<class MATRIX> inline void FinishAdditionAij2To(AlignedVector3f &to) {}
	template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(AlignedVector3f &to) {}

	template<ubyte STAGE> inline void SetReserve(AlignedVector3f &v) {}
	inline void MakeReciprocal(AlignedVector3f &v) { v.MakeReciprocal(); }
	inline void MakeSquareRoot(AlignedVector3f &v) { v.MakeSquareRoot(); }
	inline float NormL2_2(const AlignedVector3f &v) { return v.SquaredLength(); }
	inline float NormLinf(const AlignedVector3f &v) { return std::max(std::max(fabs(v.v0()), fabs(v.v1())), fabs(v.v2())); }
	inline float NormWL2_2(const AlignedVector3f &v, const AlignedVector3f &w) { return ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(v.v012x(), ENFT_SSE::_mm_mul_ps(v.v012x(), w.v012x()))); }
	inline void ApB(const AlignedVector3f &A, const AlignedVector3f &B, AlignedVector3f &ApB)
	{
		ApB.v012x() = ENFT_SSE::_mm_add_ps(A.v012x(), B.v012x());
	}
	inline void AmB(const AlignedVector3f &A, const AlignedVector3f &B, AlignedVector3f &AmB)
	{
		AmB.v012x() = ENFT_SSE::_mm_sub_ps(A.v012x(), B.v012x());
	}
	inline void sA(const float &s, const AlignedVector3f &A, AlignedVector3f &sA)
	{
		sA.v012x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.v012x());
	}
	inline void sA(const AlignedVector3f &s, AlignedVector3f &sA)
	{
		sA.v012x() = ENFT_SSE::_mm_mul_ps(s.v012x(), sA.v012x());
	}
	inline void sApB(const float &s, const AlignedVector3f &A, const AlignedVector3f &B, AlignedVector3f &sApB)
	{
		sApB.v012x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.v012x()), B.v012x());
	}
	inline void Normalize(const AlignedVector3f &A, AlignedVector3f &nA)
	{
		nA.v012x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1 / A.Length()), A.v012x());
	}
	inline float Dot(const AlignedVector3f &A, const AlignedVector3f &B)
	{
		return ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.v012x(), B.v012x()));
	}
	inline float Dot(const ENFT_SSE::__m128 &A, const ENFT_SSE::__m128 &B)
	{
		return ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A, B));
	}
	inline void Cross(const AlignedVector3f &A, const AlignedVector3f &B, AlignedVector3f &AcB)
	{
		AcB.v0() = A.v1() * B.v2() - A.v2() * B.v1();
		AcB.v1() = A.v2() * B.v0() - A.v0() * B.v2();
		AcB.v2() = A.v0() * B.v1() - A.v1() * B.v0();
	}
	inline void Cross(const ENFT_SSE::__m128 &A, const ENFT_SSE::__m128 &B, ENFT_SSE::__m128 &AcB)
	{
		AcB.m128_f32[0] = A.m128_f32[1] * B.m128_f32[2] - A.m128_f32[2] * B.m128_f32[1];
		AcB.m128_f32[1] = A.m128_f32[2] * B.m128_f32[0] - A.m128_f32[0] * B.m128_f32[2];
		AcB.m128_f32[2] = A.m128_f32[0] * B.m128_f32[1] - A.m128_f32[1] * B.m128_f32[0];
	}
	inline bool Rotate(const AlignedVector3f &Vfrom, const AlignedVector3f &Vto, AlignedVector3f &axis, float &Cos)
	{
		Cross(Vfrom, Vto, axis);
		const float len2 = axis.SquaredLength();
		if(len2 < FLT_EPSILON)
			return false;
		axis.Scale(1 / sqrt(len2));
		Cos = Dot(Vfrom, Vto);
		return Cos >= -1.0f && Cos <= 1.0f;
	}
	inline void ComputeError(const AlignedVector3f &A, const AlignedVector3f &B, float &absErr, float &relErr)
	{
		const ENFT_SSE::__m128 e = ENFT_SSE::_mm_sub_ps(A.v012x(), B.v012x());
		absErr = sqrt(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(e, e)));
		const float lenA = A.Length(), lenB = B.Length(), lenMax = std::max(lenA, lenB);
		if(lenMax == 0)
			relErr = 0;
		else
			relErr = absErr / lenMax;
	}

	template<typename TYPE> class Vector3
	{

	public:

		Vector3() {}
		Vector3(const Vector3 &v) : m_v0(v.m_v0), m_v1(v.m_v1), m_v2(v.m_v2) {}
		Vector3(const TYPE &v0, const TYPE &v1, const TYPE &v2) : m_v0(v0), m_v1(v1), m_v2(v2) {}

		inline operator TYPE* () { return (TYPE *) this; }
		inline operator const TYPE* () const { return (const TYPE *) this; }
		inline void operator = (const Vector3 &v) { m_v0 = v.m_v0; m_v1 = v.m_v1; m_v2 = v.m_v2; }
		inline Vector3 operator + (const Vector3 &v) { return Vector3(m_v0 + v.m_v0, m_v1 + v.m_v1, m_v2 + v.m_v2); }
		inline void operator += (const Vector3 &v) { m_v0 = v.m_v0 + m_v0; m_v1 = v.m_v1 + m_v1; m_v2 = v.m_v2 + m_v2; }
		inline Vector3 operator - (const Vector3 &v) { return Vector3(m_v0 - v.m_v0, m_v1 - v.m_v1, m_v2 - v.m_v2); }
		inline void operator -= (const Vector3 &v) { m_v0 = -v.m_v0 + m_v0; m_v1 = -v.m_v1 + m_v1; m_v2 = -v.m_v2 + m_v2; }
		inline void operator *= (const TYPE &s) { m_v0 *= s; m_v1 *= s; m_v2 *= s; }

		inline void Set(const TYPE &v0, const TYPE &v1, const TYPE &v2) { m_v0 = v0; m_v1 = v1; m_v2 = v2; }
		inline void SetZero() { memset(this, 0, sizeof(Vector3<TYPE>)); }
		inline TYPE Length() const { return sqrt(m_v0 * m_v0 + m_v1 * m_v1 + m_v2 * m_v2); }
		inline TYPE SquaredLength() const { return m_v0 * m_v0 + m_v1 * m_v1 + m_v2 * m_v2; }
		inline void Normalize()
		{
			const TYPE t = 1 / sqrt(m_v0 * m_v0 + m_v1 * m_v1 + m_v2 * m_v2);
			m_v0 *= t; m_v1 *= t; m_v2 *= t;
		}
		inline void NormL2(TYPE &normL2_2, TYPE &normL2) const
		{
			normL2_2 = m_v0 * m_v0 + m_v1 * m_v1 + m_v2 * m_v2;
			normL2 = sqrt(normL2_2);
		}
		inline TYPE Dot(const Vector3 &v) const { return m_v0 * v.m_v0 + m_v1 * v.m_v1 + m_v2 * v.m_v2; }

		inline void MakeReciprocal()
		{
			if(m_v0 != 0)
				m_v0 = 1 / m_v0;
			if(m_v1 != 0)
				m_v1 = 1 / m_v1;
			if(m_v2 != 0)
				m_v2 = 1 / m_v2;
		}
		inline void MakeSquareRoot()
		{
			m_v0 = sqrt(m_v0);
			m_v1 = sqrt(m_v1);
			m_v2 = sqrt(m_v2);
		}

		inline const TYPE& v0() const { return m_v0; }		inline TYPE& v0() { return m_v0; }
		inline const TYPE& v1() const { return m_v1; }		inline TYPE& v1() { return m_v1; }
		inline const TYPE& v2() const { return m_v2; }		inline TYPE& v2() { return m_v2; }

		inline void Print(const bool e = false) const
		{
			if(e)
				printf("%e %e %e\n", m_v0, m_v1, m_v2);
			else
				printf("%lf %lf %lf\n", m_v0, m_v1, m_v2);
		}
		inline void Save(FILE *fp, const bool e = false) const
		{
			if(e)
				fprintf(fp, "%e %e %e\n", v0(), v1(), v2());
			else
				fprintf(fp, "%lf %lf %lf\n", v0(), v1(), v2());
		}
		inline void Load(FILE *fp) { fscanf(fp, "%lf %lf %lf", &v0(), &v1(), &v2()); }

	protected:

		TYPE m_v0, m_v1, m_v2;

	};

	typedef Vector3<float > Vector3f;
	typedef Vector3<double> Vector3d;
	typedef Vector3<int>	Vector3i;
	typedef Vector3<ubyte>	Vector3ub;

	inline void AddAij2To(const AlignedVector3f &A, Vector3f &to, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_mul_ps(A.v012x(), A.v012x());
		to.v0() = work.m128_f32[0] + to.v0();
		to.v1() = work.m128_f32[1] + to.v1();
		to.v2() = work.m128_f32[2] + to.v2();
	}
	inline void AddsATo(const float &s, const AlignedVector3f &A, Vector3f &to, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.v012x());
		to.v0() = work.m128_f32[0] + to.v0();
		to.v1() = work.m128_f32[1] + to.v1();
		to.v2() = work.m128_f32[2] + to.v2();
	}
	template<class MATRIX> inline void FinishAdditionAij2To(Vector3f &to) {}
	template<class MATRIX> inline void FinishAdditionAij2To(Vector3d &to) {}
	template<class MATRIX_1, class MATRIX_2> inline void FinishAdditionAij2To(Vector3f &to) {}
	template<class MATRIX_1, class MATRIX_2> inline void FinishAdditionAij2To(Vector3d &to) {}
	template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(Vector3f &to) {}
	template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(Vector3d &to) {}

	inline void SetReserve(Vector3f &v) {}
	template<ubyte STAGE> inline void SetReserve(Vector3f &v) {}
	template<ubyte STAGE> inline void SetReserve(Vector3d &v) {}
	template<typename TYPE> inline void MakeReciprocal(Vector3<TYPE> &v) { v.MakeReciprocal(); }
	template<typename TYPE> inline void MakeSquareRoot(Vector3<TYPE> &v) { v.MakeSquareRoot(); }
	template<typename TYPE> inline TYPE NormL2_2(const Vector3<TYPE> &v) { return v.SquaredLength(); }
	template<typename TYPE> inline TYPE NormLinf(const Vector3<TYPE> &v) { return std::max(std::max(fabs(v.v0()), fabs(v.v1())), fabs(v.v2())); }
	template<typename TYPE> inline TYPE NormWL2_2(const Vector3<TYPE> &v, const Vector3<TYPE> &w)
	{
		return v.v0() * v.v0() * w.v0() + v.v1() * v.v1() * w.v1() + v.v2() * v.v2() * w.v2();
	}

	template<typename TYPE> inline void sA(const Vector3<TYPE> &s, Vector3<TYPE> &A)
	{
		A.v0() *= s.v0();
		A.v1() *= s.v1();
		A.v2() *= s.v2();
	}
	template<typename TYPE> inline void ApB(const Vector3<TYPE> &A, const Vector3<TYPE> &B, Vector3<TYPE> &ApB)
	{
		ApB.v0() = A.v0() + B.v0();
		ApB.v1() = A.v1() + B.v1();
		ApB.v2() = A.v2() + B.v2();
	}
	template<typename TYPE> inline void AmB(const Vector3<TYPE> &A, const Vector3<TYPE> &B, Vector3<TYPE> &AmB)
	{
		AmB.v0() = A.v0() - B.v0();
		AmB.v1() = A.v1() - B.v1();
		AmB.v2() = A.v2() - B.v2();
	}
	template<typename TYPE> inline void sA(const TYPE &s, const Vector3<TYPE> &A, Vector3<TYPE> &sA)
	{
		sA.v0() = s * A.v0();
		sA.v1() = s * A.v1();
		sA.v2() = s * A.v2();
	}
	template<typename TYPE> inline void sApB(const TYPE &s, const Vector3<TYPE> &A, const Vector3<TYPE> &B, Vector3<TYPE> &sApB)
	{
		sApB.v0() = s * A.v0() + B.v0();
		sApB.v1() = s * A.v1() + B.v1();
		sApB.v2() = s * A.v2() + B.v2();
	}
	template<typename TYPE> inline void AddsATo(const TYPE &s, const Vector3<TYPE> &A, Vector3<TYPE> &to)
	{
		to.v0() = s * A.v0() + to.v0();
		to.v1() = s * A.v1() + to.v1();
		to.v2() = s * A.v2() + to.v2();
	}
	template<typename TYPE> inline void Normalize(const Vector3<TYPE> &A, Vector3<TYPE> &nA)
	{
		const TYPE t = 1 / sqrt(A.v0() * A.v0() + A.v1() * A.v1() + A.v2() * A.v2());
		nA.v0() = A.v0() * t;
		nA.v1() = A.v1() * t;
		nA.v2() = A.v2() * t;
	}
	template<typename TYPE> inline TYPE Dot(const Vector3<TYPE> &A, const Vector3<TYPE> &B)
	{
		return A.v0() * B.v0() + A.v1() * B.v1() + A.v2() * B.v2();
	}
	template<typename TYPE> inline void Cross(const Vector3<TYPE> &A, const Vector3<TYPE> &B, Vector3<TYPE> &AcB)
	{
		AcB.v0() = A.v1() * B.v2() - A.v2() * B.v1();
		AcB.v1() = A.v2() * B.v0() - A.v0() * B.v2();
		AcB.v2() = A.v0() * B.v1() - A.v1() * B.v0();
	}
	template<typename TYPE> inline void Rotate(const Vector3<TYPE> &Vfrom, const Vector3<TYPE> &Vto, Vector3<TYPE> &axis, TYPE &Cos)
	{
		Cross(Vfrom, Vto, axis);
		axis.Normalize();
		Cos = Dot(Vfrom, Vto);
	}
	template<typename TYPE> inline void ComputeError(const Vector3<TYPE> &A, const Vector3<TYPE> &B, TYPE &absErr, TYPE &relErr)
	{
		absErr = sqrt((A.v0() - B.v0()) * (A.v0() - B.v0()) + (A.v1() - B.v1()) * (A.v1() - B.v1()) + (A.v2() - B.v2()) * (A.v2() - B.v2()) );
		const TYPE lenA = A.Length(), lenB = B.Length(), lenMax = std::max(lenA, lenB);
		if(lenMax == 0)
			relErr = 0;
		else
			relErr = absErr / lenMax;
	}
}

#endif