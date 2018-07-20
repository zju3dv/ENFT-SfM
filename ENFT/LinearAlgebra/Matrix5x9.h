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

#ifndef _MATRIX_5x9_H_
#define _MATRIX_5x9_H_

#include "Vector9.h"

namespace LA
{

	class AlignedMatrix5x9f
	{

	public:

		inline operator const float* () const { return (const float *) this; }		inline operator float* () { return (float *) this; }
		inline const ENFT_SSE::__m128& M_00_01_02_03() const { return m_00_01_02_03; }		inline ENFT_SSE::__m128& M_00_01_02_03() { return m_00_01_02_03; }
		inline const ENFT_SSE::__m128& M_04_05_06_07() const { return m_04_05_06_07; }		inline ENFT_SSE::__m128& M_04_05_06_07() { return m_04_05_06_07; }
		inline const ENFT_SSE::__m128& M_10_11_12_13() const { return m_10_11_12_13; }		inline ENFT_SSE::__m128& M_10_11_12_13() { return m_10_11_12_13; }
		inline const ENFT_SSE::__m128& M_14_15_16_17() const { return m_14_15_16_17; }		inline ENFT_SSE::__m128& M_14_15_16_17() { return m_14_15_16_17; }
		inline const ENFT_SSE::__m128& M_20_21_22_23() const { return m_20_21_22_23; }		inline ENFT_SSE::__m128& M_20_21_22_23() { return m_20_21_22_23; }
		inline const ENFT_SSE::__m128& M_24_25_26_27() const { return m_24_25_26_27; }		inline ENFT_SSE::__m128& M_24_25_26_27() { return m_24_25_26_27; }
		inline const ENFT_SSE::__m128& M_30_31_32_33() const { return m_30_31_32_33; }		inline ENFT_SSE::__m128& M_30_31_32_33() { return m_30_31_32_33; }
		inline const ENFT_SSE::__m128& M_34_35_36_37() const { return m_34_35_36_37; }		inline ENFT_SSE::__m128& M_34_35_36_37() { return m_34_35_36_37; }
		inline const ENFT_SSE::__m128& M_40_41_42_43() const { return m_40_41_42_43; }		inline ENFT_SSE::__m128& M_40_41_42_43() { return m_40_41_42_43; }
		inline const ENFT_SSE::__m128& M_44_45_46_47() const { return m_44_45_46_47; }		inline ENFT_SSE::__m128& M_44_45_46_47() { return m_44_45_46_47; }
		inline const float& M00() const { return m_00_01_02_03.m128_f32[0]; }		inline float& M00() { return m_00_01_02_03.m128_f32[0]; }
		inline const float& M01() const { return m_00_01_02_03.m128_f32[1]; }		inline float& M01() { return m_00_01_02_03.m128_f32[1]; }
		inline const float& M02() const { return m_00_01_02_03.m128_f32[2]; }		inline float& M02() { return m_00_01_02_03.m128_f32[2]; }
		inline const float& M03() const { return m_00_01_02_03.m128_f32[3]; }		inline float& M03() { return m_00_01_02_03.m128_f32[3]; }
		inline const float& M04() const { return m_04_05_06_07.m128_f32[0]; }		inline float& M04() { return m_04_05_06_07.m128_f32[0]; }
		inline const float& M05() const { return m_04_05_06_07.m128_f32[1]; }		inline float& M05() { return m_04_05_06_07.m128_f32[1]; }
		inline const float& M06() const { return m_04_05_06_07.m128_f32[2]; }		inline float& M06() { return m_04_05_06_07.m128_f32[2]; }
		inline const float& M07() const { return m_04_05_06_07.m128_f32[3]; }		inline float& M07() { return m_04_05_06_07.m128_f32[3]; }
		inline const float& M10() const { return m_10_11_12_13.m128_f32[0]; }		inline float& M10() { return m_10_11_12_13.m128_f32[0]; }
		inline const float& M11() const { return m_10_11_12_13.m128_f32[1]; }		inline float& M11() { return m_10_11_12_13.m128_f32[1]; }
		inline const float& M12() const { return m_10_11_12_13.m128_f32[2]; }		inline float& M12() { return m_10_11_12_13.m128_f32[2]; }
		inline const float& M13() const { return m_10_11_12_13.m128_f32[3]; }		inline float& M13() { return m_10_11_12_13.m128_f32[3]; }
		inline const float& M14() const { return m_14_15_16_17.m128_f32[0]; }		inline float& M14() { return m_14_15_16_17.m128_f32[0]; }
		inline const float& M15() const { return m_14_15_16_17.m128_f32[1]; }		inline float& M15() { return m_14_15_16_17.m128_f32[1]; }
		inline const float& M16() const { return m_14_15_16_17.m128_f32[2]; }		inline float& M16() { return m_14_15_16_17.m128_f32[2]; }
		inline const float& M17() const { return m_14_15_16_17.m128_f32[3]; }		inline float& M17() { return m_14_15_16_17.m128_f32[3]; }
		inline const float& M20() const { return m_20_21_22_23.m128_f32[0]; }		inline float& M20() { return m_20_21_22_23.m128_f32[0]; }
		inline const float& M21() const { return m_20_21_22_23.m128_f32[1]; }		inline float& M21() { return m_20_21_22_23.m128_f32[1]; }
		inline const float& M22() const { return m_20_21_22_23.m128_f32[2]; }		inline float& M22() { return m_20_21_22_23.m128_f32[2]; }
		inline const float& M23() const { return m_20_21_22_23.m128_f32[3]; }		inline float& M23() { return m_20_21_22_23.m128_f32[3]; }
		inline const float& M24() const { return m_24_25_26_27.m128_f32[0]; }		inline float& M24() { return m_24_25_26_27.m128_f32[0]; }
		inline const float& M25() const { return m_24_25_26_27.m128_f32[1]; }		inline float& M25() { return m_24_25_26_27.m128_f32[1]; }
		inline const float& M26() const { return m_24_25_26_27.m128_f32[2]; }		inline float& M26() { return m_24_25_26_27.m128_f32[2]; }
		inline const float& M27() const { return m_24_25_26_27.m128_f32[3]; }		inline float& M27() { return m_24_25_26_27.m128_f32[3]; }
		inline const float& M30() const { return m_30_31_32_33.m128_f32[0]; }		inline float& M30() { return m_30_31_32_33.m128_f32[0]; }
		inline const float& M31() const { return m_30_31_32_33.m128_f32[1]; }		inline float& M31() { return m_30_31_32_33.m128_f32[1]; }
		inline const float& M32() const { return m_30_31_32_33.m128_f32[2]; }		inline float& M32() { return m_30_31_32_33.m128_f32[2]; }
		inline const float& M33() const { return m_30_31_32_33.m128_f32[3]; }		inline float& M33() { return m_30_31_32_33.m128_f32[3]; }
		inline const float& M34() const { return m_34_35_36_37.m128_f32[0]; }		inline float& M34() { return m_34_35_36_37.m128_f32[0]; }
		inline const float& M35() const { return m_34_35_36_37.m128_f32[1]; }		inline float& M35() { return m_34_35_36_37.m128_f32[1]; }
		inline const float& M36() const { return m_34_35_36_37.m128_f32[2]; }		inline float& M36() { return m_34_35_36_37.m128_f32[2]; }
		inline const float& M37() const { return m_34_35_36_37.m128_f32[3]; }		inline float& M37() { return m_34_35_36_37.m128_f32[3]; }
		inline const float& M40() const { return m_40_41_42_43.m128_f32[0]; }		inline float& M40() { return m_40_41_42_43.m128_f32[0]; }
		inline const float& M41() const { return m_40_41_42_43.m128_f32[1]; }		inline float& M41() { return m_40_41_42_43.m128_f32[1]; }
		inline const float& M42() const { return m_40_41_42_43.m128_f32[2]; }		inline float& M42() { return m_40_41_42_43.m128_f32[2]; }
		inline const float& M43() const { return m_40_41_42_43.m128_f32[3]; }		inline float& M43() { return m_40_41_42_43.m128_f32[3]; }
		inline const float& M44() const { return m_44_45_46_47.m128_f32[0]; }		inline float& M44() { return m_44_45_46_47.m128_f32[0]; }
		inline const float& M45() const { return m_44_45_46_47.m128_f32[1]; }		inline float& M45() { return m_44_45_46_47.m128_f32[1]; }
		inline const float& M46() const { return m_44_45_46_47.m128_f32[2]; }		inline float& M46() { return m_44_45_46_47.m128_f32[2]; }
		inline const float& M47() const { return m_44_45_46_47.m128_f32[3]; }		inline float& M47() { return m_44_45_46_47.m128_f32[3]; }
		inline const float& M08() const { return m_08_x_x_x.m128_f32[0]; }			inline float& M08() { return m_08_x_x_x.m128_f32[0]; }
		inline const float& M18() const { return m_18_x_x_x.m128_f32[0]; }			inline float& M18() { return m_18_x_x_x.m128_f32[0]; }
		inline const float& M28() const { return m_28_x_x_x.m128_f32[0]; }			inline float& M28() { return m_28_x_x_x.m128_f32[0]; }
		inline const float& M38() const { return m_38_x_x_x.m128_f32[0]; }			inline float& M38() { return m_38_x_x_x.m128_f32[0]; }
		inline const float& M48() const { return m_48_x_x_x.m128_f32[0]; }			inline float& M48() { return m_48_x_x_x.m128_f32[0]; }

	protected:

		ENFT_SSE::__m128 m_00_01_02_03, m_04_05_06_07, m_08_x_x_x;
		ENFT_SSE::__m128 m_10_11_12_13, m_14_15_16_17, m_18_x_x_x;
		ENFT_SSE::__m128 m_20_21_22_23, m_24_25_26_27, m_28_x_x_x;
		ENFT_SSE::__m128 m_30_31_32_33, m_34_35_36_37, m_38_x_x_x;
		ENFT_SSE::__m128 m_40_41_42_43, m_44_45_46_47, m_48_x_x_x;

	};

	inline void AddAATToUpper(const AlignedMatrix5x9f &A, AlignedMatrix5f &to)
	{
		to.M00() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), A.M_04_05_06_07()))) + A.M08() * A.M08() + to.M00();
		to.M01() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_10_11_12_13()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), A.M_14_15_16_17()))) + A.M08() * A.M18() + to.M01();
		to.M02() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_20_21_22_23()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), A.M_24_25_26_27()))) + A.M08() * A.M28() + to.M02();
		to.M03() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_30_31_32_33()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), A.M_34_35_36_37()))) + A.M08() * A.M38() + to.M03();
		to.M04() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_40_41_42_43()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), A.M_44_45_46_47()))) + A.M08() * A.M48() + to.M04();

		to.M11() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_10_11_12_13()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), A.M_14_15_16_17()))) + A.M18() * A.M18() + to.M11();
		to.M12() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_20_21_22_23()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), A.M_24_25_26_27()))) + A.M18() * A.M28() + to.M12();
		to.M13() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_30_31_32_33()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), A.M_34_35_36_37()))) + A.M18() * A.M38() + to.M13();
		to.M14() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_40_41_42_43()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), A.M_44_45_46_47()))) + A.M18() * A.M48() + to.M14();

		to.M22() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), A.M_20_21_22_23()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), A.M_24_25_26_27()))) + A.M28() * A.M28() + to.M22();
		to.M23() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), A.M_30_31_32_33()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), A.M_34_35_36_37()))) + A.M28() * A.M38() + to.M23();
		to.M24() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), A.M_40_41_42_43()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), A.M_44_45_46_47()))) + A.M28() * A.M48() + to.M24();

		to.M33() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), A.M_30_31_32_33()), ENFT_SSE::_mm_mul_ps(A.M_34_35_36_37(), A.M_34_35_36_37()))) + A.M38() * A.M38() + to.M33();
		to.M34() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), A.M_40_41_42_43()), ENFT_SSE::_mm_mul_ps(A.M_34_35_36_37(), A.M_44_45_46_47()))) + A.M38() * A.M48() + to.M34();

		to.M44() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), A.M_40_41_42_43()), ENFT_SSE::_mm_mul_ps(A.M_44_45_46_47(), A.M_44_45_46_47()))) + A.M48() * A.M48() + to.M44();
	}
	inline void AddABTo(const AlignedMatrix5x9f &A, const AlignedVector9f &B, AlignedVector5f &to)
	{
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), B.v4567()))) + A.M08() * B.v8() + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), B.v4567()))) + A.M18() * B.v8() + to.v1();
		to.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), B.v4567()))) + A.M28() * B.v8() + to.v2();
		to.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_34_35_36_37(), B.v4567()))) + A.M38() * B.v8() + to.v3();
		to.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_44_45_46_47(), B.v4567()))) + A.M48() * B.v8() + to.v4();
	}

}

#endif