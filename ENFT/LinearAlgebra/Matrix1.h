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

#ifndef _MATRIX_1_H_
#define _MATRIX_1_H_

#include "Vector3.h"
#include "Matrix3.h"

namespace LA
{

	class Matrix1f
	{
	public:
		inline void Set(const float &data) { m_data = data; }
		inline operator const float& () const { return m_data; }
		inline operator		  float& ()		  { return m_data; }
	protected:
		float m_data;
	};
	inline void AddAij2To(const Vector3f &A, float &to) { to = A.v0() * A.v0() + A.v1() * A.v1() + A.v2() * A.v2() + to; }
	inline void AddAij2To(const Vector3f &A, float &to, ENFT_SSE::__m128 *work0) { AddAij2To(A, to); }
	inline void AddAij2To(const float &A, Vector3f &to) { to.v0() = A * A + to.v0(); to.v1() = A * A + to.v1(); to.v2() = A * A + to.v2(); }
	template<class MATRIX> inline void FinishAdditionAij2To(float &to) {}
	template<ubyte STAGE> inline void SetReserve(float &v) {}
	inline void MakeReciprocal(float &v) { v = (v == 0 ? 0 : 1 / v); }
	inline void MakeSquareRoot(float &v) { v = sqrt(v); }
	inline void sA(const float &s, Vector3f &A) { A *= s; }
	inline void sA(const Vector3f &s, float &A)
	{
#if _DEBUG
		assert(s.v0() == s.v1() && s.v0() == s.v2());
#endif
		//s.Print();
		A *= s.v0();
	}
	inline void sA(const float &s, const float &A, float &sA) { sA = s * A; }
	inline void sA(const ENFT_SSE::__m128 &s, const float &A, float &sA) { sA = s.m128_f32[0] * A; }
	inline void sApB(const float &s, const float &A, const float &B, float &sApB) { sApB = s * A + B; }
	inline void sApB(const ENFT_SSE::__m128 &s, const float &A, const float &B, float &sApB) { sApB = s.m128_f32[0] * A + B; }
	inline void AddATAToUpper(const float &A, float &to) { to = A * A + to; }
	inline void AddATAToUpper(const Vector3f &A, float &to) { to = A.v0() * A.v0() + A.v1() * A.v1() + A.v2() * A.v2() + to; }
	inline void AddATAToUpper(const Vector3f &A, float &to, ENFT_SSE::__m128 *work0) { AddATAToUpper(A, to); }
	template<class MATRIX> inline void FinishAdditionATAToUpper(float &to) {}
	inline void SetLowerFromUpper(float &M) {}
	inline void GetDiagonal(const float &M, float &d) { d = M; }
	inline void GetDiagonal(const float &M, Vector3f &d) { d.v0() = d.v1() = d.v2() = M; }
	inline void SetDiagonal(const float &d, float &M) { M = d; }
	inline void SetDiagonal(const Vector3f &d, float &M)
	{
#if _DEBUG
		assert(d.v0() == d.v1() && d.v0() == d.v2());
#endif
		//d.Print(); 
		M = d.v0();
	}
	inline void ScaleDiagonal(const float &lambda, float &M) { M *= lambda; }
	inline void IncreaseDiagonal(const float &lambda, float &M) { M += lambda; }
	inline void InvertSymmetricUpper(float &M, float *work0) { M = (M == 0 ? 0 : 1 / M); }
	inline void InvertSymmetricUpper(const float &M, float &Minv) { Minv = (M == 0 ? 0 : 1 / M); }
	inline void InvertSymmetricUpper(const float &M, float &Minv, float *work0) { Minv = (M == 0 ? 0 : 1 / M); }
	inline float NormLinf(const float &M) { return M; }
	inline float NormL2_2(const float &M) { return M * M; }
	inline float NormWL2_2(const float &M, const float &W) { return M * M * W; }
	inline float Dot(const float &A, const float &B) { return A * B; }
	inline void ATB(const float &A, const Vector3f &B, Vector3f &ATB)
	{
		ATB.v0() = A * B.v0();
		ATB.v1() = A * B.v1();
		ATB.v2() = A * B.v2();
	}
	inline void ATB(const float &A, const Vector3f &B, Vector3f &ATB, ENFT_SSE::__m128 *work0) { LA::ATB(A, B, ATB); }
	inline void AB(const Vector3f &A, const float &B, AlignedVector3f &AB) { AB.v0() = A.v0() * B; AB.v1() = A.v1() * B; AB.v2() = A.v2() * B; }
	inline void AB(const Vector3f &A, const float &B, AlignedVector3f &AB, ENFT_SSE::__m128 *work0) { LA::AB(A, B, AB); }
	inline void AB(const float &A, const Vector3f &B, AlignedVector3f &AB) { AB.v0() = A * B.v0(); AB.v1() = A * B.v1(); AB.v2() = A * B.v2(); }
	inline void ABpCD(const Vector3f &A, const float &B, const float &C, const Vector3f &D, AlignedVector3f &ABpCD)
	{
		ABpCD.v0() = A.v0() * B + C * D.v0();
		ABpCD.v1() = A.v1() * B + C * D.v1();
		ABpCD.v2() = A.v2() * B + C * D.v2();
	}
	inline void ABpCD(const Vector3f &A, const float &B, const float &C, const Vector3f &D, AlignedVector3f &ABpCD, ENFT_SSE::__m128 *work0) { LA::ABpCD(A, B, C, D, ABpCD); }
	inline void SubtractATBFrom(const Vector3f &A, const Vector3f &B, float &from) { from -= A.Dot(B); }
	inline void SubtractATBFrom(const Vector3f &A, const Vector3f &B, float &from, ENFT_SSE::__m128 *work0) { SubtractATBFrom(A, B, from); }
	inline void AddATo(const Vector3f &A, Vector3f &to) { to.v0() = A.v0() + to.v0(); to.v1() = A.v1() + to.v1(); to.v2() = A.v2() + to.v2(); }
	inline void SubtractABFrom(const Vector3f &A, const float &B, Vector3f &from)
	{
		from.v0() -= A.v0() * B;
		from.v1() -= A.v1() * B;
		from.v2() -= A.v2() * B;
	}
	inline void SubtractABFrom(const Vector3f &A, const float &B, Vector3f &from, ENFT_SSE::__m128 *work0) { SubtractABFrom(A, B, from); }
	inline void AB(const float &A, const Vector3f &B, Vector3f &AB) { AB.v0() = A * B.v0(); AB.v1() = A * B.v1(); AB.v2() = A * B.v2(); }
	inline void AddATBTo(const Vector3f &A, const AlignedVector3f &B, float &to) { to = A.v0() * B.v0() + A.v1() * B.v1() + A.v2() * B.v2() + to; }
	inline void AddATBTo(const Vector3f &A, const AlignedVector3f &B, float &to, ENFT_SSE::__m128 *work0) { AddATBTo(A, B, to); }
	template<class MATRIX> inline void FinishAdditionATBTo(float &to) {}
	inline void AddATBTo(const float &A, const AlignedVector3f &B, Vector3f &to)
	{
		to.v0() = A * B.v0() + to.v0();
		to.v1() = A * B.v1() + to.v1();
		to.v2() = A * B.v2() + to.v2();
	}
	inline void AB(const float &A, const float &B, float &AB) { AB = A * B; }
	inline void AB(const float &A, const float &B, float &AB, ENFT_SSE::__m128 *work0) { LA::AB(A, B, AB); }
	inline void AddATBTo(const Vector3f &A, const Vector3f &B, float &to) { to = A.Dot(B) + to; }
	inline void AddATBTo(const Vector3f &A, const Vector3f &B, float &to, ENFT_SSE::__m128 *work0) { AddATBTo(A, B, to); }
	inline void AddATBTo(const float &A, const float &B, float &to) { to = A * B + to; }
	inline void AddATBTo(const float &A, const float &B, float &to, ENFT_SSE::__m128 *work0) { AddATBTo(A, B, to); }
	inline void AddABTo(const Vector3f &A, const float &B, Vector3f &to)
	{
		to.v0() = A.v0() * B + to.v0();
		to.v1() = A.v1() * B + to.v1();
		to.v2() = A.v2() * B + to.v2();
	}
	inline void AddABTo(const Vector3f &A, const float &B, Vector3f &to, ENFT_SSE::__m128 *work0) { AddABTo(A, B, to); }
	inline void AddABTo(const float &A, const float &B, float &to) { to = A * B + to; }
	inline void AddABTo(const float &A, const float &B, float &to, ENFT_SSE::__m128 *work0) { AddABTo(A, B, to); }
	inline void ABmC(const float &A, const float &B, const float &C, float &ABmC) { ABmC = A * B - C; }
	inline void ABmC(const float &A, const float &B, const float &C, float &ABmC, ENFT_SSE::__m128 *work0) { LA::ABmC(A, B, C, ABmC); }
}

#endif