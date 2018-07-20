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

#ifndef _MATRIX_5x6_H_
#define _MATRIX_5x6_H_

#include "Matrix2x6.h"
#include "Matrix5.h"

namespace LA
{

	class AlignedMatrix5x6f : public AlignedMatrix2x6f
	{

	public:

		inline const ENFT_SSE::__m128& M_20_21_22_23() const { return m_20_21_22_23; }		inline ENFT_SSE::__m128& M_20_21_22_23() { return m_20_21_22_23; }
		inline const ENFT_SSE::__m128& M_24_25_x_x  () const { return m_24_25_x_x; }			inline ENFT_SSE::__m128& M_24_25_x_x  () { return m_24_25_x_x; }
		inline const ENFT_SSE::__m128& M_30_31_32_33() const { return m_30_31_32_33; }		inline ENFT_SSE::__m128& M_30_31_32_33() { return m_30_31_32_33; }
		inline const ENFT_SSE::__m128& M_34_35_x_x  () const { return m_34_35_x_x; }			inline ENFT_SSE::__m128& M_34_35_x_x  () { return m_34_35_x_x; }
		inline const ENFT_SSE::__m128& M_40_41_42_43() const { return m_40_01_02_03; }		inline ENFT_SSE::__m128& M_40_41_42_43() { return m_40_01_02_03; }
		inline const ENFT_SSE::__m128& M_44_45_x_x  () const { return m_44_45_x_x; }			inline ENFT_SSE::__m128& M_44_45_x_x  () { return m_44_45_x_x; }

		inline const float& M20() const { return m_20_21_22_23.m128_f32[0]; }		inline float& M20() { return m_20_21_22_23.m128_f32[0]; }
		inline const float& M21() const { return m_20_21_22_23.m128_f32[1]; }		inline float& M21() { return m_20_21_22_23.m128_f32[1]; }
		inline const float& M22() const { return m_20_21_22_23.m128_f32[2]; }		inline float& M22() { return m_20_21_22_23.m128_f32[2]; }
		inline const float& M23() const { return m_20_21_22_23.m128_f32[3]; }		inline float& M23() { return m_20_21_22_23.m128_f32[3]; }
		inline const float& M24() const { return m_24_25_x_x.m128_f32[0]; }			inline float& M24() { return m_24_25_x_x.m128_f32[0]; }
		inline const float& M25() const { return m_24_25_x_x.m128_f32[1]; }			inline float& M25() { return m_24_25_x_x.m128_f32[1]; }
		inline const float& M30() const { return m_30_31_32_33.m128_f32[0]; }		inline float& M30() { return m_30_31_32_33.m128_f32[0]; }
		inline const float& M31() const { return m_30_31_32_33.m128_f32[1]; }		inline float& M31() { return m_30_31_32_33.m128_f32[1]; }
		inline const float& M32() const { return m_30_31_32_33.m128_f32[2]; }		inline float& M32() { return m_30_31_32_33.m128_f32[2]; }
		inline const float& M33() const { return m_30_31_32_33.m128_f32[3]; }		inline float& M33() { return m_30_31_32_33.m128_f32[3]; }
		inline const float& M34() const { return m_34_35_x_x.m128_f32[0]; }			inline float& M34() { return m_34_35_x_x.m128_f32[0]; }
		inline const float& M35() const { return m_34_35_x_x.m128_f32[1]; }			inline float& M35() { return m_34_35_x_x.m128_f32[1]; }
		inline const float& M40() const { return m_40_01_02_03.m128_f32[0]; }		inline float& M40() { return m_40_01_02_03.m128_f32[0]; }
		inline const float& M41() const { return m_40_01_02_03.m128_f32[1]; }		inline float& M41() { return m_40_01_02_03.m128_f32[1]; }
		inline const float& M42() const { return m_40_01_02_03.m128_f32[2]; }		inline float& M42() { return m_40_01_02_03.m128_f32[2]; }
		inline const float& M43() const { return m_40_01_02_03.m128_f32[3]; }		inline float& M43() { return m_40_01_02_03.m128_f32[3]; }
		inline const float& M44() const { return m_44_45_x_x.m128_f32[0]; }			inline float& M44() { return m_44_45_x_x.m128_f32[0]; }
		inline const float& M45() const { return m_44_45_x_x.m128_f32[1]; }			inline float& M45() { return m_44_45_x_x.m128_f32[1]; }

		inline void Print() const
		{
			printf("%f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05());
			printf("%f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15());
			printf("%f %f %f %f %f %f\n", M20(), M21(), M22(), M23(), M24(), M25());
			printf("%f %f %f %f %f %f\n", M30(), M31(), M32(), M33(), M34(), M35());
			printf("%f %f %f %f %f %f\n", M40(), M41(), M42(), M43(), M44(), M45());
		}

	protected:

		ENFT_SSE::__m128 m_20_21_22_23, m_24_25_x_x, m_30_31_32_33, m_34_35_x_x, m_40_01_02_03, m_44_45_x_x;

	};

	inline void AddAATToUpper(const AlignedMatrix5x6f &A, AlignedMatrix5f &to)
	{
		to.M00() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_00_01_02_03())) + A.M04() * A.M04() + A.M05() * A.M05() + to.M00();
		to.M01() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_10_11_12_13())) + A.M04() * A.M14() + A.M05() * A.M15() + to.M01();
		to.M02() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_20_21_22_23())) + A.M04() * A.M24() + A.M05() * A.M25() + to.M02();
		to.M03() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_30_31_32_33())) + A.M04() * A.M34() + A.M05() * A.M35() + to.M03();
		to.M04() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_40_41_42_43())) + A.M04() * A.M44() + A.M05() * A.M45() + to.M04();

		to.M11() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_10_11_12_13())) + A.M14() * A.M14() + A.M15() * A.M15() + to.M11();
		to.M12() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_20_21_22_23())) + A.M14() * A.M24() + A.M15() * A.M25() + to.M12();
		to.M13() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_30_31_32_33())) + A.M14() * A.M34() + A.M15() * A.M35() + to.M13();
		to.M14() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_40_41_42_43())) + A.M14() * A.M44() + A.M15() * A.M45() + to.M14();

		to.M22() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), A.M_20_21_22_23())) + A.M24() * A.M24() + A.M25() * A.M25() + to.M22();
		to.M23() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), A.M_30_31_32_33())) + A.M24() * A.M34() + A.M25() * A.M35() + to.M23();
		to.M24() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), A.M_40_41_42_43())) + A.M24() * A.M44() + A.M25() * A.M45() + to.M24();

		to.M33() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), A.M_30_31_32_33())) + A.M34() * A.M34() + A.M35() * A.M35() + to.M33();
		to.M34() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), A.M_40_41_42_43())) + A.M34() * A.M44() + A.M35() * A.M45() + to.M34();

		to.M44() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), A.M_40_41_42_43())) + A.M44() * A.M44() + A.M45() * A.M45() + to.M44();
	}
	inline void AddABTo(const AlignedMatrix5x6f &A, const AlignedVector6f &B, AlignedVector5f &to)
	{
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + A.M04() * B.v4() + A.M05() * B.v5() + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + A.M14() * B.v4() + A.M15() * B.v5() + to.v1();
		to.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + A.M24() * B.v4() + A.M25() * B.v5() + to.v2();
		to.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123())) + A.M34() * B.v4() + A.M35() * B.v5() + to.v3();
		to.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123())) + A.M44() * B.v4() + A.M45() * B.v5() + to.v4();
	}

}

#endif