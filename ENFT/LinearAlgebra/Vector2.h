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

#ifndef _VECTOR_2_H_
#define _VECTOR_2_H_

namespace LA
{
	template<typename TYPE> class Vector2
	{

	public:

		Vector2() {}
		Vector2(const TYPE &v0, const TYPE &v1) : m_v0(v0), m_v1(v1) {}
		Vector2(const Vector2<TYPE> &v) { memcpy(this, &v, sizeof(Vector2<TYPE>)); }
		inline void operator = (const Vector2<TYPE> &v) { memcpy(this, &v, sizeof(Vector2<TYPE>)); }
		inline operator const TYPE* () const { return (TYPE *) this; }
		inline operator		  TYPE* ()		 { return (TYPE *) this; }
		inline void operator += (const Vector2<TYPE> &v) { m_v0 += v.m_v0; m_v1 += v.m_v1; }
		inline void operator -= (const Vector2<TYPE> &v) { m_v0 -= v.m_v0; m_v1 -= v.m_v1; }
		inline void operator *= (const TYPE &s) { m_v0 *= s; m_v1 *= s; }

		inline void SetZero() { m_v0 = m_v1 = 0; }
		inline void Set(const TYPE &v0, const TYPE &v1) { m_v0 = v0; m_v1 = v1; }
		inline void Get(TYPE &v0, TYPE &v1) const { v0 = m_v0; v1 = m_v1; }
		inline const TYPE& v0() const { return m_v0; }
		inline		 TYPE& v0()		  { return m_v0; }
		inline const TYPE& v1() const { return m_v1; }
		inline		 TYPE& v1()		  { return m_v1; }
		template<typename _TYPE> inline bool operator > (const Vector2<_TYPE> &v) const { return m_v0 > v.m_v0 && m_v1 > v.m_v1; }
		template<typename _TYPE> inline bool operator < (const Vector2<_TYPE> &v) const { return m_v0 < v.m_v0 && m_v1 < v.m_v1; }
		template<typename _TYPE> inline bool operator >= (const Vector2<_TYPE> &v) const { return m_v0 >= v.m_v0 && m_v1 >= v.m_v1; }
		template<typename _TYPE> inline bool operator <= (const Vector2<_TYPE> &v) const { return m_v0 <= v.m_v0 && m_v1 <= v.m_v1; }
		inline TYPE Dot(const Vector2<TYPE> &v) const { return m_v0 * v.v0() + m_v1 * v.v1(); } 
		inline TYPE SquaredLength() const { return m_v0 * m_v0 + m_v1 * m_v1; }
		inline TYPE SquaredDistance(const LA::Vector2<TYPE> &v) const { return (m_v0 - v.m_v0) * (m_v0 - v.m_v0) + (m_v1 - v.m_v1) * (m_v1 - v.m_v1); }
		inline TYPE SquaredDistance(const TYPE *v) const { return (m_v0 - v[0]) * (m_v0 - v[0]) + (m_v1 - v[1]) * (m_v1 - v[1]); }
		template<typename _TYPE>
		inline TYPE SquaredDistance(const _TYPE &v0, const _TYPE &v1) const { return (m_v0 - v0) * (m_v0 - v0) + (m_v1 - v1) * (m_v1 - v1); }
		inline void Normalize() { const TYPE norm = 1 / sqrt(SquaredLength()); v0() *= norm; v1() *= norm; }

		inline void MakeReciprocal()
		{
			if(m_v0 != 0)
				m_v0 = 1 / m_v0;
			if(m_v1 != 0)
				m_v1 = 1 / m_v1;
		}
		inline void MakeSquareRoot()
		{
			m_v0 = sqrt(m_v0);
			m_v1 = sqrt(m_v1);
		}

		inline void Print() const { printf("(%f, %f)\n", m_v0, m_v1); }

	protected:

		TYPE m_v0, m_v1;
	};

	typedef Vector2<int	  > Vector2i;
	typedef Vector2<float > Vector2f;
	typedef Vector2<double> Vector2d;
	typedef Vector2<short>	Vector2s;
	typedef Vector2<ushort> Vector2us;

	template<typename TYPE> inline TYPE NormL2_2(const Vector2<TYPE> &v) { return v.SquaredLength(); }
	template<typename TYPE> inline void ApB(const Vector2<TYPE> &A, const Vector2<TYPE> &B, Vector2<TYPE> &ApB)
	{
		ApB.v0() = A.v0() + B.v0();
		ApB.v1() = A.v1() + B.v1();
	}
	template<typename TYPE> inline void AmB(const Vector2<TYPE> &A, const Vector2<TYPE> &B, Vector2<TYPE> &AmB)
	{
		AmB.v0() = A.v0() - B.v0();
		AmB.v1() = A.v1() - B.v1();
	}
	template<typename TYPE> inline void sA(const Vector2<TYPE> &s, Vector2<TYPE> &A)
	{
		A.v0() = s.v0() * A.v0();
		A.v1() = s.v1() * A.v1();
	}
	template<typename TYPE> inline void sA(const float &s, const Vector2<TYPE> &A, Vector2<TYPE> &sA)
	{
		sA.v0() = s * A.v0();
		sA.v1() = s * A.v1();
	}
	template<typename TYPE> inline void sApB(const float &s, const Vector2<TYPE> &A, const Vector2<TYPE> &B, Vector2<TYPE> &sApB)
	{
		sApB.v0() = s * A.v0() + B.v0();
		sApB.v1() = s * A.v1() + B.v1();
	}
	template<ubyte STAGE> inline void SetReserve(Vector2f &v) {}
	inline void MakeReciprocal(Vector2f &v) { v.MakeReciprocal(); }
	inline void MakeSquareRoot(Vector2f &v) { v.MakeSquareRoot(); }
	inline float NormL2_2(const Vector2f &v) { return v.SquaredLength(); }
	inline float NormLinf(const Vector2f &v) { return std::max(fabs(v.v0()), fabs(v.v1())); }
	inline float NormWL2_2(const Vector2f &v, const Vector2f &w) { return v.v0() * v.v0() * w.v0() + v.v1() * v.v1() * w.v1(); }
	inline float Dot(const Vector2f &A, const Vector2f &B) { return A.v0() * B.v0() + A.v1() * B.v1(); }
}

#endif