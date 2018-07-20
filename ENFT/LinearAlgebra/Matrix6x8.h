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

#ifndef _MATRIX_6x8_H_
#define _MATRIX_6x8_H_

#include "Matrix3x8.h"
#include "Vector6.h"
#include "Vector8.h"

namespace LA
{

	class AlignedMatrix6x8f : public AlignedMatrix3x8f
	{

	public:

		inline const ENFT_SSE::__m128& M_30_31_32_33() const { return m_30_31_32_33; }		inline ENFT_SSE::__m128& M_30_31_32_33() { return m_30_31_32_33; }
		inline const ENFT_SSE::__m128& M_34_35_36_37() const { return m_34_35_36_37; }		inline ENFT_SSE::__m128& M_34_35_36_37() { return m_34_35_36_37; }
		inline const ENFT_SSE::__m128& M_40_41_42_43() const { return m_40_41_42_43; }		inline ENFT_SSE::__m128& M_40_41_42_43() { return m_40_41_42_43; }
		inline const ENFT_SSE::__m128& M_44_45_46_47() const { return m_44_45_46_47; }		inline ENFT_SSE::__m128& M_44_45_46_47() { return m_44_45_46_47; }
		inline const ENFT_SSE::__m128& M_50_51_52_53() const { return m_50_51_52_53; }		inline ENFT_SSE::__m128& M_50_51_52_53() { return m_50_51_52_53; }
		inline const ENFT_SSE::__m128& M_54_55_56_57() const { return m_54_55_56_57; }		inline ENFT_SSE::__m128& M_54_55_56_57() { return m_54_55_56_57; }
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
		inline const float& M50() const { return m_50_51_52_53.m128_f32[0]; }		inline float& M50() { return m_50_51_52_53.m128_f32[0]; }
		inline const float& M51() const { return m_50_51_52_53.m128_f32[1]; }		inline float& M51() { return m_50_51_52_53.m128_f32[1]; }
		inline const float& M52() const { return m_50_51_52_53.m128_f32[2]; }		inline float& M52() { return m_50_51_52_53.m128_f32[2]; }
		inline const float& M53() const { return m_50_51_52_53.m128_f32[3]; }		inline float& M53() { return m_50_51_52_53.m128_f32[3]; }
		inline const float& M54() const { return m_54_55_56_57.m128_f32[0]; }		inline float& M54() { return m_54_55_56_57.m128_f32[0]; }
		inline const float& M55() const { return m_54_55_56_57.m128_f32[1]; }		inline float& M55() { return m_54_55_56_57.m128_f32[1]; }
		inline const float& M56() const { return m_54_55_56_57.m128_f32[2]; }		inline float& M56() { return m_54_55_56_57.m128_f32[2]; }
		inline const float& M57() const { return m_54_55_56_57.m128_f32[3]; }		inline float& M57() { return m_54_55_56_57.m128_f32[3]; }
		inline void SetZero() { memset(this, 0, sizeof(AlignedMatrix6x8f)); }
		inline void Print() const
		{
			printf("%f %f %f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05(), M06(), M07());
			printf("%f %f %f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15(), M16(), M17());
			printf("%f %f %f %f %f %f %f %f\n", M20(), M21(), M22(), M23(), M24(), M25(), M26(), M27());
			printf("%f %f %f %f %f %f %f %f\n", M30(), M31(), M32(), M33(), M34(), M35(), M36(), M37());
			printf("%f %f %f %f %f %f %f %f\n", M40(), M41(), M42(), M43(), M44(), M45(), M46(), M47());
			printf("%f %f %f %f %f %f %f %f\n", M50(), M51(), M52(), M53(), M54(), M55(), M56(), M57());
		}

	protected:

		ENFT_SSE::__m128 m_30_31_32_33, m_34_35_36_37;
		ENFT_SSE::__m128 m_40_41_42_43, m_44_45_46_47;
		ENFT_SSE::__m128 m_50_51_52_53, m_54_55_56_57;

	};

	inline void ATBT(const AlignedVector8f &A, AlignedMatrix6x8f &B, AlignedVector6f &ATBT)
	{
		ATBT.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(A.v4567(), B.M_04_05_06_07())));
		ATBT.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), B.M_10_11_12_13()), ENFT_SSE::_mm_mul_ps(A.v4567(), B.M_14_15_16_17())));
		ATBT.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), B.M_20_21_22_23()), ENFT_SSE::_mm_mul_ps(A.v4567(), B.M_24_25_26_27())));
		ATBT.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), B.M_30_31_32_33()), ENFT_SSE::_mm_mul_ps(A.v4567(), B.M_34_35_36_37())));
		ATBT.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), B.M_40_41_42_43()), ENFT_SSE::_mm_mul_ps(A.v4567(), B.M_44_45_46_47())));
		ATBT.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), B.M_50_51_52_53()), ENFT_SSE::_mm_mul_ps(A.v4567(), B.M_54_55_56_57())));
	}

}

#endif