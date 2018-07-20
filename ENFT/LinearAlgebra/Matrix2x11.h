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

#ifndef _MATRIX_2x11_H_
#define _MATRIX_2x11_H_

#include "Vector2.h"

namespace LA
{

	class AlignedMatrix2x11f
	{


	public:
		inline const ENFT_SSE::__m128& M_00_01_02_03() const { return m_00_01_02_03; }			inline ENFT_SSE::__m128& M_00_01_02_03() { return m_00_01_02_03; }
		inline const ENFT_SSE::__m128& M_04_05_06_07() const { return m_04_05_06_07; }			inline ENFT_SSE::__m128& M_04_05_06_07() { return m_04_05_06_07; }
		inline const ENFT_SSE::__m128& M_08_09_010_x() const { return m_08_09_010_x; }			inline ENFT_SSE::__m128& M_08_09_010_x() { return m_08_09_010_x; }

		inline const ENFT_SSE::__m128& M_10_11_12_13() const { return m_10_11_12_13; }			inline ENFT_SSE::__m128& M_10_11_12_13() { return m_10_11_12_13; }
		inline const ENFT_SSE::__m128& M_14_15_16_17() const { return m_14_15_16_17; }			inline ENFT_SSE::__m128& M_14_15_16_17() { return m_14_15_16_17; }
		inline const ENFT_SSE::__m128& M_18_19_110_x() const { return m_18_19_110_x; }			inline ENFT_SSE::__m128& M_18_19_110_x() { return m_18_19_110_x; }

		inline const float& M00() const { return m_00_01_02_03.m128_f32[0]; }			inline float& M00() { return m_00_01_02_03.m128_f32[0]; }
		inline const float& M01() const { return m_00_01_02_03.m128_f32[1]; }			inline float& M01() { return m_00_01_02_03.m128_f32[1]; }
		inline const float& M02() const { return m_00_01_02_03.m128_f32[2]; }			inline float& M02() { return m_00_01_02_03.m128_f32[2]; }
		inline const float& M03() const { return m_00_01_02_03.m128_f32[3]; }			inline float& M03() { return m_00_01_02_03.m128_f32[3]; }
		inline const float& M04() const { return m_04_05_06_07.m128_f32[0]; }			inline float& M04() { return m_04_05_06_07.m128_f32[0]; }
		inline const float& M05() const { return m_04_05_06_07.m128_f32[1]; }			inline float& M05() { return m_04_05_06_07.m128_f32[1]; }
		inline const float& M06() const { return m_04_05_06_07.m128_f32[2]; }			inline float& M06() { return m_04_05_06_07.m128_f32[2]; }
		inline const float& M07() const { return m_04_05_06_07.m128_f32[3]; }			inline float& M07() { return m_04_05_06_07.m128_f32[3]; }
		inline const float& M08() const { return m_08_09_010_x.m128_f32[0]; }			inline float& M08() { return m_08_09_010_x.m128_f32[0]; }
		inline const float& M09() const { return m_08_09_010_x.m128_f32[1]; }			inline float& M09() { return m_08_09_010_x.m128_f32[1]; }
		inline const float& M010() const { return m_08_09_010_x.m128_f32[2]; }			inline float& M010() { return m_08_09_010_x.m128_f32[2]; }
		inline const float& reserve0() const { return m_08_09_010_x.m128_f32[3]; }		inline float& reserve0() { return m_08_09_010_x.m128_f32[3]; }

		inline const float& M10() const { return m_10_11_12_13.m128_f32[0]; }			inline float& M10() { return m_10_11_12_13.m128_f32[0]; }
		inline const float& M11() const { return m_10_11_12_13.m128_f32[1]; }			inline float& M11() { return m_10_11_12_13.m128_f32[1]; }
		inline const float& M12() const { return m_10_11_12_13.m128_f32[2]; }			inline float& M12() { return m_10_11_12_13.m128_f32[2]; }
		inline const float& M13() const { return m_10_11_12_13.m128_f32[3]; }			inline float& M13() { return m_10_11_12_13.m128_f32[3]; }
		inline const float& M14() const { return m_14_15_16_17.m128_f32[0]; }			inline float& M14() { return m_14_15_16_17.m128_f32[0]; }
		inline const float& M15() const { return m_14_15_16_17.m128_f32[1]; }			inline float& M15() { return m_14_15_16_17.m128_f32[1]; }
		inline const float& M16() const { return m_14_15_16_17.m128_f32[2]; }			inline float& M16() { return m_14_15_16_17.m128_f32[2]; }
		inline const float& M17() const { return m_14_15_16_17.m128_f32[3]; }			inline float& M17() { return m_14_15_16_17.m128_f32[3]; }
		inline const float& M18() const { return m_18_19_110_x.m128_f32[0]; }			inline float& M18() { return m_18_19_110_x.m128_f32[0]; }
		inline const float& M19() const { return m_18_19_110_x.m128_f32[1]; }			inline float& M19() { return m_18_19_110_x.m128_f32[1]; }
		inline const float& M110() const { return m_18_19_110_x.m128_f32[2]; }			inline float& M110() { return m_18_19_110_x.m128_f32[2]; }
		inline const float& reserve1() const { return m_18_19_110_x.m128_f32[3]; }		inline float& reserve1() { return m_18_19_110_x.m128_f32[3]; }

		inline void Print() const
		{
			const float *row = &M00();
			for(int i = 0; i < 2; ++i, row += 12)
			{
				for(int j = 0; j < 11; ++j)
					printf(" %.2f", row[j]);
				printf("\n");
			}
		}

	protected:

		ENFT_SSE::__m128 m_00_01_02_03, m_04_05_06_07, m_08_09_010_x;
		ENFT_SSE::__m128 m_10_11_12_13, m_14_15_16_17, m_18_19_110_x;

	};

	inline void AddATBTo(const AlignedMatrix2x11f &A, const Vector2f &B, AlignedVector11f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());		work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), work2[1])), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), work2[1])), to.v8910x());
	}
	template<> inline void FinishAdditionATBTo<AlignedMatrix2x11f>(AlignedVector11f &to) { to.reserve() = 0.0f; }
	inline void AddABTo(const AlignedMatrix2x11f &A, const AlignedVector11f &B, Vector2f &to, ENFT_SSE::__m128 *work0)
	{
#if _DEBUG
		assert(B.reserve() == 0);
#endif
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), B.v8910x()))) + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), B.v8910x()))) + to.v1();
	}

}

#endif