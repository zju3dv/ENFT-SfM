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

#ifndef _MATRIX_5_H_
#define _MATRIX_5_H_

#include "Matrix2x5.h"

namespace LA
{

	class AlignedMatrix5f : public AlignedMatrix2x5f
	{

	public:

		inline const ENFT_SSE::__m128& M_20_21_22_23() const { return m_20_21_22_23; }	inline ENFT_SSE::__m128& M_20_21_22_23() { return m_20_21_22_23; }
		inline const ENFT_SSE::__m128& M_30_31_32_33() const { return m_30_31_32_33; }	inline ENFT_SSE::__m128& M_30_31_32_33() { return m_30_31_32_33; }
		inline const ENFT_SSE::__m128& M_40_41_42_43() const { return m_40_41_42_43; }	inline ENFT_SSE::__m128& M_40_41_42_43() { return m_40_41_42_43; }
		inline const ENFT_SSE::__m128& M_24_x_x_x() const { return m_24_x_x_x; }			inline ENFT_SSE::__m128& M_24_x_x_x() { return m_24_x_x_x; }
		inline const ENFT_SSE::__m128& M_34_x_x_x() const { return m_34_x_x_x; }			inline ENFT_SSE::__m128& M_34_x_x_x() { return m_34_x_x_x; }
		inline const ENFT_SSE::__m128& M_44_x_x_x() const { return m_44_x_x_x; }			inline ENFT_SSE::__m128& M_44_x_x_x() { return m_44_x_x_x; }
		inline const float& M20() const { return m_20_21_22_23.m128_f32[0]; }	inline float& M20()	{ return m_20_21_22_23.m128_f32[0]; }
		inline const float& M21() const { return m_20_21_22_23.m128_f32[1]; }	inline float& M21()	{ return m_20_21_22_23.m128_f32[1]; }
		inline const float& M22() const { return m_20_21_22_23.m128_f32[2]; }	inline float& M22()	{ return m_20_21_22_23.m128_f32[2]; }
		inline const float& M23() const { return m_20_21_22_23.m128_f32[3]; }	inline float& M23() { return m_20_21_22_23.m128_f32[3]; }
		inline const float& M30() const { return m_30_31_32_33.m128_f32[0]; }	inline float& M30()	{ return m_30_31_32_33.m128_f32[0]; }
		inline const float& M31() const { return m_30_31_32_33.m128_f32[1]; }	inline float& M31()	{ return m_30_31_32_33.m128_f32[1]; }
		inline const float& M32() const { return m_30_31_32_33.m128_f32[2]; }	inline float& M32()	{ return m_30_31_32_33.m128_f32[2]; }
		inline const float& M33() const { return m_30_31_32_33.m128_f32[3]; }	inline float& M33() { return m_30_31_32_33.m128_f32[3]; }
		inline const float& M40() const { return m_40_41_42_43.m128_f32[0]; }	inline float& M40()	{ return m_40_41_42_43.m128_f32[0]; }
		inline const float& M41() const { return m_40_41_42_43.m128_f32[1]; }	inline float& M41()	{ return m_40_41_42_43.m128_f32[1]; }
		inline const float& M42() const { return m_40_41_42_43.m128_f32[2]; }	inline float& M42()	{ return m_40_41_42_43.m128_f32[2]; }
		inline const float& M43() const { return m_40_41_42_43.m128_f32[3]; }	inline float& M43() { return m_40_41_42_43.m128_f32[3]; }
		inline const float& M24() const { return m_24_x_x_x.m128_f32[0]; }		inline float& M24() { return m_24_x_x_x.m128_f32[0]; }
		inline const float& M34() const { return m_34_x_x_x.m128_f32[0]; }		inline float& M34() { return m_34_x_x_x.m128_f32[0]; }
		inline const float& M44() const { return m_44_x_x_x.m128_f32[0]; }		inline float& M44() { return m_44_x_x_x.m128_f32[0]; }
		inline operator const float* () const { return (const float *) this; }
		inline operator		  float* ()		  { return (	  float *) this; }
		inline void SetZero() { memset(this, 0, sizeof(AlignedMatrix5f)); }
		inline void GetDiagonal(AlignedVector5f &d) const { d.v0() = M00();	d.v1() = M11();	d.v2() = M22(); d.v3() = M33(); d.v4() = M44(); }
		inline void SetDiagonal(const AlignedVector5f &d) { M00() = d.v0(); M11() = d.v1(); M22() = d.v2(); M33() = d.v3(); M44() = d.v4(); }
		inline void ScaleDiagonal(const float &lambda) { M00() *= lambda; M11() *= lambda; M22() *= lambda; M33() *= lambda; M44() *= lambda; }
		inline void IncreaseDiagonal(const float &lambda) { M00() += lambda; M11() += lambda; M22() += lambda; M33() += lambda; M44() += lambda; }
		inline void SetLowerFromUpper()
		{
			M10() = M01();
			M20() = M02();	M21() = M12();
			M30() = M03();	M31() = M13();	M32() = M23();
			M40() = M04();	M41() = M14();	M42() = M24();	M43() = M34();
		}
		inline void Print() const
		{
			printf("%f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04());
			printf("%f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14());
			printf("%f %f %f %f %f\n", M20(), M21(), M22(), M23(), M24());
			printf("%f %f %f %f %f\n", M30(), M31(), M32(), M33(), M34());
			printf("%f %f %f %f %f\n", M40(), M41(), M42(), M43(), M44());
		}

	protected:

		ENFT_SSE::__m128 m_00_01_02_03, m_04_x_x_x, m_10_11_12_13, m_14_x_x_x, m_20_21_22_23, m_24_x_x_x, m_30_31_32_33, m_34_x_x_x, m_40_41_42_43, m_44_x_x_x;

	};

	inline void GetDiagonal(const AlignedMatrix5f &M, AlignedVector5f &d) { M.GetDiagonal(d); }
	inline void SetDiagonal(const AlignedVector5f &d, AlignedMatrix5f &M) { M.SetDiagonal(d); }
	inline void ScaleDiagonal(const float &lambda, AlignedMatrix5f &M) { M.ScaleDiagonal(lambda); }
	inline void IncreaseDiagonal(const float &lambda, AlignedMatrix5f &M) { M.IncreaseDiagonal(lambda); }
	inline void SetLowerFromUpper(AlignedMatrix5f &A) { A.SetLowerFromUpper(); }

	inline void AB(const AlignedMatrix5f &A, const AlignedVector5f &B, AlignedVector5f &AB)
	{
		AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + A.M04() * B.v4();
		AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + A.M14() * B.v4();
		AB.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + A.M24() * B.v4();
		AB.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123())) + A.M34() * B.v4();
		AB.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123())) + A.M44() * B.v4();
	}
	inline void AddATAToUpper(const AlignedMatrix2x5f &A, AlignedMatrix5f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), A.M_10_11_12_13())), to.M_00_01_02_03());
		to.M04() = A.M00() * A.M04() + A.M10() * A.M14() + to.M04();

		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), A.M_10_11_12_13())), to.M_10_11_12_13());
		to.M14() = A.M01() * A.M04() + A.M11() * A.M14() + to.M14();

		to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + to.M22();
		to.M23() = A.M02() * A.M03() + A.M12() * A.M13() + to.M23();
		to.M24() = A.M02() * A.M04() + A.M12() * A.M14() + to.M24();

		to.M33() = A.M03() * A.M03() + A.M13() * A.M13() + to.M33();
		to.M34() = A.M03() * A.M04() + A.M13() * A.M14() + to.M34();

		to.M44() = A.M04() * A.M04() + A.M14() * A.M14() + to.M44();
	}
	inline void AddAATToUpper(const AlignedVector5f &A, AlignedMatrix5f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v0()), A.v0123()), to.M_00_01_02_03());
		to.M04() = A.v0() * A.v4() + to.M04();
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v1()), A.v0123()), to.M_10_11_12_13());
		to.M14() = A.v1() * A.v4() + to.M14();
		to.M22() = A.v2() * A.v2() + to.M22();
		to.M23() = A.v2() * A.v3() + to.M23();
		to.M24() = A.v2() * A.v4() + to.M24();
		to.M33() = A.v3() * A.v3() + to.M33();
		to.M34() = A.v3() * A.v4() + to.M34();
		to.M44() = A.v4() * A.v4() + to.M44();
	}
	inline void ssTA(const AlignedVector5f &s, AlignedMatrix5f &A)
	{
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v0()), s.v0123()), A.M_00_01_02_03());		A.M04() *= s.v0() * s.v4();
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v1()), s.v0123()), A.M_10_11_12_13());		A.M14() *= s.v1() * s.v4();
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v2()), s.v0123()), A.M_20_21_22_23());		A.M24() *= s.v2() * s.v4();
		A.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v3()), s.v0123()), A.M_30_31_32_33());		A.M34() *= s.v3() * s.v4();
		A.M_40_41_42_43() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v4()), s.v0123()), A.M_40_41_42_43());		A.M44() *= s.v4() * s.v4();
	}

	bool SolveLinearSystemSymmetricUpper(AlignedMatrix5f &A, AlignedVector5f &b);
	inline bool SolveLinearSystemSymmetricUpper(AlignedMatrix5f &A, AlignedVector5f &b, float *work0) { return SolveLinearSystemSymmetricUpper(A, b); }

}

#endif