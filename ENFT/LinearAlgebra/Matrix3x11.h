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

#ifndef _MATRIX_3x11_H_
#define _MATRIX_3x11_H_

#include "Matrix2x11.h"
#include "Matrix3.h"

namespace LA
{

	class AlignedMatrix3x11f : public AlignedMatrix2x11f
	{

	public:

		inline operator const float* () const { return (const float *) this; }		inline operator float* () { return (float *) this; }

		inline const ENFT_SSE::__m128& M_20_21_22_23() const { return m_20_21_22_23; }		inline ENFT_SSE::__m128& M_20_21_22_23() { return m_20_21_22_23; }
		inline const ENFT_SSE::__m128& M_24_25_26_27() const { return m_24_25_26_27; }		inline ENFT_SSE::__m128& M_24_25_26_27() { return m_24_25_26_27; }
		inline const ENFT_SSE::__m128& M_28_29_210_x() const { return m_28_29_210_x; }		inline ENFT_SSE::__m128& M_28_29_210_x() { return m_28_29_210_x; }

		inline const float& M20() const { return m_20_21_22_23.m128_f32[0]; }		inline float& M20() { return m_20_21_22_23.m128_f32[0]; }
		inline const float& M21() const { return m_20_21_22_23.m128_f32[1]; }		inline float& M21() { return m_20_21_22_23.m128_f32[1]; }
		inline const float& M22() const { return m_20_21_22_23.m128_f32[2]; }		inline float& M22() { return m_20_21_22_23.m128_f32[2]; }
		inline const float& M23() const { return m_20_21_22_23.m128_f32[3]; }		inline float& M23() { return m_20_21_22_23.m128_f32[3]; }
		inline const float& M24() const { return m_24_25_26_27.m128_f32[0]; }		inline float& M24() { return m_24_25_26_27.m128_f32[0]; }
		inline const float& M25() const { return m_24_25_26_27.m128_f32[1]; }		inline float& M25() { return m_24_25_26_27.m128_f32[1]; }
		inline const float& M26() const { return m_24_25_26_27.m128_f32[2]; }		inline float& M26() { return m_24_25_26_27.m128_f32[2]; }
		inline const float& M27() const { return m_24_25_26_27.m128_f32[3]; }		inline float& M27() { return m_24_25_26_27.m128_f32[3]; }
		inline const float& M28() const { return m_28_29_210_x.m128_f32[0]; }		inline float& M28() { return m_28_29_210_x.m128_f32[0]; }
		inline const float& M29() const { return m_28_29_210_x.m128_f32[1]; }		inline float& M29() { return m_28_29_210_x.m128_f32[1]; }
		inline const float& M210() const { return m_28_29_210_x.m128_f32[2]; }		inline float& M210() { return m_28_29_210_x.m128_f32[2]; }
		inline const float& reserve2() const { return m_28_29_210_x.m128_f32[3]; }	inline float& reserve2() { return m_28_29_210_x.m128_f32[3]; }

		inline void Print() const
		{
			const float *row = &M00();
			for(int i = 0; i < 3; ++i, row += 12)
			{
				for(int j = 0; j < 11; ++j)
					printf(" %.2f", row[j]);
				printf("\n");
			}
		}

	protected:

		ENFT_SSE::__m128 m_20_21_22_23, m_24_25_26_27, m_28_29_210_x;

	};

	inline void s1s2TA(const Vector3f &s1, const AlignedVector11f &s2, AlignedMatrix3x11f &A, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_set1_ps(s1.v0());
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v0123()), A.M_00_01_02_03());
		A.M_04_05_06_07() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v4567()), A.M_04_05_06_07());
		A.M_08_09_010_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v8910x()), A.M_08_09_010_x());
		work = ENFT_SSE::_mm_set1_ps(s1.v1());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v0123()), A.M_10_11_12_13());
		A.M_14_15_16_17() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v4567()), A.M_14_15_16_17());
		A.M_18_19_110_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v8910x()), A.M_18_19_110_x());
		work = ENFT_SSE::_mm_set1_ps(s1.v2());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v0123()), A.M_20_21_22_23());
		A.M_24_25_26_27() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v4567()), A.M_24_25_26_27());
		A.M_28_29_210_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v8910x()), A.M_28_29_210_x());
	}

	inline void ATB(const SymmetricMatrix3f &A, const AlignedMatrix3x11f &B, AlignedMatrix3x11f &ATB, ENFT_SSE::__m128 *work4)
	{
		work4[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work4[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		work4[2] = ENFT_SSE::_mm_set1_ps(A.M20());
		ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work4[1], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work4[2], B.M_20_21_22_23()));
		ATB.M_04_05_06_07() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work4[1], B.M_14_15_16_17())), 
													ENFT_SSE::_mm_mul_ps(work4[2], B.M_24_25_26_27()));
		ATB.M_08_09_010_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work4[1], B.M_18_19_110_x())), 
													ENFT_SSE::_mm_mul_ps(work4[2], B.M_28_29_210_x()));
		work4[0] = ENFT_SSE::_mm_set1_ps(A.M11());
		work4[3] = ENFT_SSE::_mm_set1_ps(A.M21());
		ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[1], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work4[0], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work4[3], B.M_20_21_22_23()));
		ATB.M_14_15_16_17() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[1], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work4[0], B.M_14_15_16_17())), 
													ENFT_SSE::_mm_mul_ps(work4[3], B.M_24_25_26_27()));
		ATB.M_18_19_110_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[1], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work4[0], B.M_18_19_110_x())), 
													ENFT_SSE::_mm_mul_ps(work4[3], B.M_28_29_210_x()));

		work4[0] = ENFT_SSE::_mm_set1_ps(A.M22());
		ATB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[2], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work4[3], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work4[0], B.M_20_21_22_23()));
		ATB.M_24_25_26_27() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[2], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work4[3], B.M_14_15_16_17())), 
													ENFT_SSE::_mm_mul_ps(work4[0], B.M_24_25_26_27()));
		ATB.M_28_29_210_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[2], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work4[3], B.M_18_19_110_x())), 
													ENFT_SSE::_mm_mul_ps(work4[0], B.M_28_29_210_x()));
//#if _DEBUG
//		const float _A[3][3] = {{A.M00(), A.M01(), A.M02()}, {A.M10(), A.M11(), A.M12()}, {A.M20(), A.M21(), A.M22()}};
//		const float* _B[3] = {&B.M00(), &B.M10(), &B.M20()};
//		const float* _ATB[3] = {&ATB.M00(), &ATB.M10(), &ATB.M20()};
//		float v1, v2;
//		for(int i = 0; i < 3; ++i)
//		for(int j = 0; j < 11; ++j)
//		{
//			v1 = 0.0f;
//			for(int k = 0; k < 3; ++k)
//				v1 += _A[k][i] * _B[k][j];
//			v2 = _ATB[i][j];
//			if(!EQUAL(v1, v2))
//				printf("%f - %f = %f\n", v1, v2, v1 - v2);
//		}
//#endif
	}
	inline void SubtractATBFrom(const AlignedMatrix3x11f &A, const Vector3f &B, AlignedVector11f &from, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work3[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		work3[2] = ENFT_SSE::_mm_set1_ps(B.v2());
		from.v0123() = ENFT_SSE::_mm_sub_ps(from.v0123(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work3[1])), ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work3[2])));
		from.v4567() = ENFT_SSE::_mm_sub_ps(from.v4567(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), work3[1])), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), work3[2])));
		from.v8910x() = ENFT_SSE::_mm_sub_ps(from.v8910x(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), work3[1])), ENFT_SSE::_mm_mul_ps(A.M_28_29_210_x(), work3[2])));
	}
	inline void SubtractABFrom(const AlignedMatrix2x3f &A, AlignedMatrix3x11f &B, AlignedMatrix2x11f &from, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M01());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M02());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_04_05_06_07() = ENFT_SSE::_mm_sub_ps(from.M_04_05_06_07(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_08_09_010_x() = ENFT_SSE::_mm_sub_ps(from.M_08_09_010_x(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M10());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M12());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_14_15_16_17() = ENFT_SSE::_mm_sub_ps(from.M_14_15_16_17(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_18_19_110_x() = ENFT_SSE::_mm_sub_ps(from.M_18_19_110_x(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));
	}
	inline void SubtractABFrom(const AlignedMatrix3x11f &A, const AlignedVector11f &B, Vector3f &from)
	{
#if _DEBUG
		assert(A.reserve0() == 0 && A.reserve1() == 0 && A.reserve2() == 0 || B.reserve() == 0);
#endif
		from.v0() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), B.v4567())), 
														ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), B.v8910x())));
		from.v1() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), B.v4567())), 
														ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), B.v8910x())));
		from.v2() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), B.v4567())), 
														ENFT_SSE::_mm_mul_ps(A.M_28_29_210_x(), B.v8910x())));
	}
	inline void SubtractABFrom(const AlignedMatrix3x11f &A, const AlignedVector11f &B, Vector3f &from, ENFT_SSE::__m128 *work0) { SubtractABFrom(A, B, from); }

}

#endif