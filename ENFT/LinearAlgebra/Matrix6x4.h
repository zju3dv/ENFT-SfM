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

#ifndef _MATRIX_6x4_H_
#define _MATRIX_6x4_H_

#include "Matrix2x6.h"
#include "Matrix5x4.h"

namespace LA
{
	class AlignedMatrix6x4f : public AlignedMatrix5x4f
	{

	public:

		inline const ENFT_SSE::__m128& M_50_51_52_53() const { return m_50_51_52_53; }	inline ENFT_SSE::__m128& M_50_51_52_53() { return m_50_51_52_53; }
		inline const float& M50() const { return m_50_51_52_53.m128_f32[0]; }	inline float& M50()	{ return m_50_51_52_53.m128_f32[0]; }
		inline const float& M51() const { return m_50_51_52_53.m128_f32[1]; }	inline float& M51()	{ return m_50_51_52_53.m128_f32[1]; }
		inline const float& M52() const { return m_50_51_52_53.m128_f32[2]; }	inline float& M52()	{ return m_50_51_52_53.m128_f32[2]; }
		inline const float& M53() const { return m_50_51_52_53.m128_f32[3]; }	inline float& M53() { return m_50_51_52_53.m128_f32[3]; }
		inline void SetZero() { memset(this, 0, sizeof(AlignedMatrix6x4f)); }

	protected:

		ENFT_SSE::__m128 m_50_51_52_53;

	};

	inline void AddATBTo(const AlignedCompactMatrix2x6f &A, const AlignedMatrix2x4f &B, AlignedMatrix6x4f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B.M_00_01_02_03()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_10_11_12_13())), to.M_00_01_02_03());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B.M_00_01_02_03()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_13())), to.M_10_11_12_13());
		to.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), B.M_00_01_02_03()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B.M_10_11_12_13())), to.M_20_21_22_23());
		to.M_30_31_32_33() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M03()), B.M_00_01_02_03()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M13()), B.M_10_11_12_13())), to.M_30_31_32_33());
		to.M_40_41_42_43() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M04()), B.M_00_01_02_03()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M14()), B.M_10_11_12_13())), to.M_40_41_42_43());
		to.M_50_51_52_53() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M05()), B.M_00_01_02_03()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M15()), B.M_10_11_12_13())), to.M_50_51_52_53());
	}
	inline void SubtractATBFrom(const AlignedCompactMatrix3x6f &A, const AlignedMatrix3x4f &B, AlignedMatrix6x4f &from)
	{
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), B.M_20_21_22_23())));
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M21()), B.M_20_21_22_23())));
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M22()), B.M_20_21_22_23())));
		from.M_30_31_32_33() = ENFT_SSE::_mm_sub_ps(from.M_30_31_32_33(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M03()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M13()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M23()), B.M_20_21_22_23())));
		from.M_40_41_42_43() = ENFT_SSE::_mm_sub_ps(from.M_40_41_42_43(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M04()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M14()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M24()), B.M_20_21_22_23())));
		from.M_50_51_52_53() = ENFT_SSE::_mm_sub_ps(from.M_50_51_52_53(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M05()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M15()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M25()), B.M_20_21_22_23())));
	}
	inline void AddATBTo(const AlignedMatrix6x4f &A, const AlignedVector6f &B, AlignedVector4f &to)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), ENFT_SSE::_mm_set1_ps(B.v0())), 
													  ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), ENFT_SSE::_mm_set1_ps(B.v1()))), 
													  ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), ENFT_SSE::_mm_set1_ps(B.v2()))), to.v0123());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), ENFT_SSE::_mm_set1_ps(B.v3())), 
													  ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), ENFT_SSE::_mm_set1_ps(B.v4()))), 
													  ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), ENFT_SSE::_mm_set1_ps(B.v5()))), to.v0123());
	}
	inline void SubtractATBFrom(const AlignedMatrix6x4f &A, const AlignedVector6f &B, AlignedVector4f &from)
	{
		from.v0123() = ENFT_SSE::_mm_sub_ps(from.v0123(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), ENFT_SSE::_mm_set1_ps(B.v0())), 
																	  ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), ENFT_SSE::_mm_set1_ps(B.v1()))), 
																	  ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), ENFT_SSE::_mm_set1_ps(B.v2()))));
		from.v0123() = ENFT_SSE::_mm_sub_ps(from.v0123(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), ENFT_SSE::_mm_set1_ps(B.v3())), 
																	  ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), ENFT_SSE::_mm_set1_ps(B.v4()))), 
																	  ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), ENFT_SSE::_mm_set1_ps(B.v5()))));
	}
	inline void AddABTo(const AlignedMatrix6x4f &A, const AlignedVector4f &B, AlignedVector6f &to)
	{
		to.v0() += ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()));
		to.v1() += ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()));
		to.v2() += ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()));
		to.v3() += ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()));
		to.v4() += ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123()));
		to.v5() += ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123()));
	}
	inline void SubtractABFrom(const AlignedMatrix6x4f &A, const AlignedVector4f &B, AlignedVector6f &from)
	{
		from.v0() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()));
		from.v1() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()));
		from.v2() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()));
		from.v3() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()));
		from.v4() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123()));
		from.v5() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123()));
	}
	inline void AddATo(const AlignedMatrix6x4f &A, AlignedMatrix6x4f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(A.M_00_01_02_03(), to.M_00_01_02_03());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(A.M_10_11_12_13(), to.M_10_11_12_13());
		to.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(A.M_20_21_22_23(), to.M_20_21_22_23());
		to.M_30_31_32_33() = ENFT_SSE::_mm_add_ps(A.M_30_31_32_33(), to.M_30_31_32_33());
		to.M_40_41_42_43() = ENFT_SSE::_mm_add_ps(A.M_40_41_42_43(), to.M_40_41_42_43());
		to.M_50_51_52_53() = ENFT_SSE::_mm_add_ps(A.M_50_51_52_53(), to.M_50_51_52_53());
	}
	template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(AlignedMatrix6x4f &to) {}
	inline void s1s2TA(const AlignedVector6f &s1, const AlignedVector4f &s2, AlignedMatrix6x4f &A)
	{
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v0()), s2.v0123()), A.M_00_01_02_03());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v1()), s2.v0123()), A.M_10_11_12_13());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v2()), s2.v0123()), A.M_20_21_22_23());
		A.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v3()), s2.v0123()), A.M_30_31_32_33());
		A.M_40_41_42_43() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v4()), s2.v0123()), A.M_40_41_42_43());
		A.M_50_51_52_53() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v5()), s2.v0123()), A.M_50_51_52_53());
	}
	inline void AB(const AlignedMatrix6x4f &A, const AlignedMatrix4f &B, AlignedMatrix6x4f &AB)
	{
		AB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), B.M_20_21_22_23()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M03()), B.M_30_31_32_33())));
		AB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B.M_20_21_22_23()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M13()), B.M_30_31_32_33())));
		AB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M21()), B.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M22()), B.M_20_21_22_23()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M23()), B.M_30_31_32_33())));
		AB.M_30_31_32_33() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M30()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M31()), B.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M32()), B.M_20_21_22_23()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M33()), B.M_30_31_32_33())));
		AB.M_40_41_42_43() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M40()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M41()), B.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M42()), B.M_20_21_22_23()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M43()), B.M_30_31_32_33())));
		AB.M_50_51_52_53() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M50()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M51()), B.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M52()), B.M_20_21_22_23()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M53()), B.M_30_31_32_33())));
	}
#if _DEBUG
	inline void AB_Debug(const AlignedMatrix6x4f &A, const AlignedMatrix4f &B, AlignedMatrix6x4f &AB)
	{
		const float* _A[6] = {&A.M00(), &A.M10(), &A.M20(), &A.M30(), &A.M40(), &A.M50()};
		const float* _B[4] = {&B.M00(), &B.M10(), &B.M20(), &B.M30()};
		float* _AB[6] = {&AB.M00(), &AB.M10(), &AB.M20(), &AB.M30(), &AB.M40(), &AB.M50()};
		for(int i = 0; i < 6; ++i)
		for(int j = 0; j < 4; ++j)
		{
			_AB[i][j] = 0;
			for(int k = 0; k < 4; ++k)
				_AB[i][j] += _A[i][k] * _B[k][j];
		}
	}
	inline void SubtractABFrom_Debug(const AlignedMatrix6x4f &A, const AlignedVector4f &B, AlignedVector6f &from)
	{
		const float* _A[6] = {&A.M00(), &A.M10(), &A.M20(), &A.M30(), &A.M40(), &A.M50()};
		const float* _B = B;
		float* _from = from;
		for(int i = 0; i < 6; ++i)
		for(int j = 0; j < 4; ++j)
			_from[i] -= _A[i][j] * B[j];
	}
	inline void AddATBTo_Debug(const AlignedCompactMatrix2x6f &A, const AlignedMatrix2x4f &B, AlignedMatrix6x4f &to)
	{
		float _A[2][6];
		A.ConvertToConventionalStorage(&_A[0][0]);
		const float* _B[2] = {&B.M00(), &B.M10()};
		float* _to[6] = {&to.M00(), &to.M10(), &to.M20(), &to.M30(), &to.M40(), &to.M50()};
		for(int i = 0; i < 6; ++i)
		for(int j = 0; j < 4; ++j)
		for(int k = 0; k < 2; ++k)
			_to[i][j] += _A[k][i] * _B[k][j];
	}
	inline void SubtractATBFrom_Debug(const AlignedCompactMatrix3x6f &A, const AlignedMatrix3x4f &B, AlignedMatrix6x4f &from)
	{
		float _A[3][6];
		A.ConvertToConventionalStorage(&_A[0][0]);
		const float* _B[3] = {&B.M00(), &B.M10(), &B.M20()};
		float* _from[6] = {&from.M00(), &from.M10(), &from.M20(), &from.M30(), &from.M40(), &from.M50()};
		for(int i = 0; i < 6; ++i)
		for(int j = 0; j < 4; ++j)
		for(int k = 0; k < 3; ++k)
			_from[i][j] -= _A[k][i] * _B[k][j];
	}
#endif
}

#endif