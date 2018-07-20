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

#ifndef _MATRIX_2_H_
#define _MATRIX_2_H_

#include "Vector2.h"
#include "Utility/SSE.h"

namespace LA
{

	class AlignedMatrix2f
	{

	public:

		inline const ENFT_SSE::__m128& M_00_01_10_11() const { return m_00_01_10_11; }		inline ENFT_SSE::__m128& M_00_01_10_11() { return m_00_01_10_11; }
		inline const float& M00() const { return m_00_01_10_11.m128_f32[0]; }		inline float& M00() { return m_00_01_10_11.m128_f32[0]; }
		inline const float& M01() const { return m_00_01_10_11.m128_f32[1]; }		inline float& M01() { return m_00_01_10_11.m128_f32[1]; }
		inline const float& M10() const { return m_00_01_10_11.m128_f32[2]; }		inline float& M10() { return m_00_01_10_11.m128_f32[2]; }
		inline const float& M11() const { return m_00_01_10_11.m128_f32[3]; }		inline float& M11()	{ return m_00_01_10_11.m128_f32[3]; }
		inline void MakeIdentity() { m_00_01_10_11 = ENFT_SSE::_mm_setr_ps(1.0f, 0.0f, 0.0f, 1.0f); }
		inline void SetZero() { memset(this, 0, sizeof(AlignedMatrix2f)); }
		inline void GetDiagonal(Vector2f &d) const { d.v0() = M00(); d.v1() = M11(); }
		inline void SetDiagonal(const Vector2f &d) { M00() = d.v0(); M11() = d.v1(); }
		inline void ScaleDiagonal(const float &lambda) { M00() *= lambda; M11() *= lambda; }
		inline void IncreaseDiagonal(const float &lambda) { M00() += lambda; M11() += lambda; }
		inline void SetLowerFromUpper() { M10() = M01(); }
		inline float Determinant() const { return M00() * M11() - M01() * M10(); }
		inline void Print() const
		{
			printf("%f %f\n", M00(), M01());
			printf("%f %f\n", M10(), M11());
		}

	protected:

		ENFT_SSE::__m128 m_00_01_10_11;
	};

	inline void GetDiagonal(const AlignedMatrix2f &M, Vector2f &d) { M.GetDiagonal(d); }
	inline void SetDiagonal(const Vector2f &d, AlignedMatrix2f &M) { M.SetDiagonal(d); }
	inline void ScaleDiagonal(const float &lambda, AlignedMatrix2f &M) { M.ScaleDiagonal(lambda); }
	inline void IncreaseDiagonal(const float &lambda, AlignedMatrix2f &M) { M.IncreaseDiagonal(lambda); }
	inline void SetLowerFromUpper(LA::AlignedMatrix2f &A) { A.SetLowerFromUpper(); }

	inline void AB(const AlignedMatrix2f &A, const Vector2f &B, Vector2f &AB)
	{
		AB.v0() = A.M00() * B.v0() + A.M01() * B.v1();
		AB.v1() = A.M10() * B.v0() + A.M11() * B.v1();
	}
	inline void AddABTo(const AlignedMatrix2f &A, const Vector2f &B, Vector2f &to, ENFT_SSE::__m128 *work0)
	{
		to.v0() = A.M00() * B.v0() + A.M01() * B.v1() + to.v0();
		to.v1() = A.M10() * B.v0() + A.M11() * B.v1() + to.v1();
	}
	inline void ATA(const AlignedMatrix2f &A, AlignedMatrix2f &ATA, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_mul_ps(A.M_00_01_10_11(), A.M_00_01_10_11());
		ATA.M00() = work.m128_f32[0] + work.m128_f32[2] ;
		ATA.M11() = work.m128_f32[1] + work.m128_f32[3];
		ATA.M01() = ATA.M10() = A.M00() * A.M01() + A.M10() * A.M11();
	}
	inline void AddATAToUpper(const AlignedMatrix2f &A, AlignedMatrix2f &to, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_mul_ps(A.M_00_01_10_11(), A.M_00_01_10_11());
		to.M00() = work.m128_f32[0] + work.m128_f32[2] + to.M00();
		to.M11() = work.m128_f32[1] + work.m128_f32[3] + to.M11();
		to.M01() = A.M00() * A.M01() + A.M10() * A.M11() + to.M01();
	}
	template<class MATRIX> inline void FinishAdditionATAToUpper(AlignedMatrix2f &to) {}
	inline void ATB(const AlignedMatrix2f &A, const Vector2f &B, Vector2f &ATB, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_mul_ps(A.M_00_01_10_11(), ENFT_SSE::_mm_setr_ps(B.v0(), B.v0(), B.v1(), B.v1()));
		ATB.v0() = work.m128_f32[0] + work.m128_f32[2];
		ATB.v1() = work.m128_f32[1] + work.m128_f32[3];
	}
	inline void AddATBTo(const AlignedMatrix2f &A, const Vector2f &B, Vector2f &to, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_mul_ps(A.M_00_01_10_11(), ENFT_SSE::_mm_setr_ps(B.v0(), B.v0(), B.v1(), B.v1()));
		to.v0() = work.m128_f32[0] + work.m128_f32[2] + to.v0();
		to.v1() = work.m128_f32[1] + work.m128_f32[3] + to.v1();
	}
	template<class MATRIX_1, class MATRIX_2> inline void FinishAdditionATBTo(Vector2f &to) {}
	inline void Aij2(const AlignedMatrix2f &A, Vector2f &Aij2, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_mul_ps(A.M_00_01_10_11(), A.M_00_01_10_11());
		Aij2.v0() = work.m128_f32[0] + work.m128_f32[2];
		Aij2.v1() = work.m128_f32[1] + work.m128_f32[3];
	}
	inline void AddAij2To(const AlignedMatrix2f &A, Vector2f &to, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_mul_ps(A.M_00_01_10_11(), A.M_00_01_10_11());
		to.v0() = work.m128_f32[0] + work.m128_f32[2] + to.v0();
		to.v1() = work.m128_f32[1] + work.m128_f32[3] + to.v1();
	}
	template<class MATRIX> inline void FinishAdditionAij2To(Vector2f &to) {}

	inline void ssTA(const Vector2f &s, AlignedMatrix2f &A)
	{
		A.M_00_01_10_11() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(s.v0() * s.v0(), s.v0() * s.v1(), s.v1() * s.v0(), s.v1() * s.v1()), A.M_00_01_10_11());
	}

	inline bool InvertSymmetricUpper(const AlignedMatrix2f &A, AlignedMatrix2f &Ainv, float *work0)
	{
		if((Ainv.M00() = A.Determinant()) == 0.0f)
		{
			Ainv.MakeIdentity();
			return false;
		}
		else
		{
			Ainv.M_00_01_10_11() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1 / Ainv.M00()), ENFT_SSE::_mm_setr_ps(A.M11(), -A.M01(), -A.M10(), A.M00()));
			return true;
		}
	}
	inline bool SolveLinearSystemSymmetricUpper(AlignedMatrix2f &A, Vector2f &b, float *work1)
	{
		if((work1[0] = A.Determinant()) == 0.0f)
			return false;
		A.M_00_01_10_11() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1 / work1[0]), ENFT_SSE::_mm_setr_ps(A.M11(), -A.M01(), -A.M10(), A.M00()));
		work1[0] = A.M00() * b.v0() + A.M01() * b.v1();
		b.v1() = A.M10() * b.v0() + A.M11() * b.v1();
		b.v0() = work1[0];
		return true;
	}
}

#endif