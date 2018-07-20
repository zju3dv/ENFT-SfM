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

#ifndef _MATRIX_3x4_H_
#define _MATRIX_3x4_H_

#include "Matrix2x3.h"
#include "Matrix2x4.h"
#include "Matrix3.h"

namespace LA
{

	class AlignedMatrix3x4f : public AlignedMatrix2x4f
	{

	public:

		inline const ENFT_SSE::__m128& M_20_21_22_23() const { return m_20_21_22_23; }	inline ENFT_SSE::__m128& M_20_21_22_23() { return m_20_21_22_23; }
		inline const float& M20() const { return m_20_21_22_23.m128_f32[0]; }	inline float& M20()	{ return m_20_21_22_23.m128_f32[0]; }
		inline const float& M21() const { return m_20_21_22_23.m128_f32[1]; }	inline float& M21()	{ return m_20_21_22_23.m128_f32[1]; }
		inline const float& M22() const { return m_20_21_22_23.m128_f32[2]; }	inline float& M22()	{ return m_20_21_22_23.m128_f32[2]; }
		inline const float& M23() const { return m_20_21_22_23.m128_f32[3]; }	inline float& M23() { return m_20_21_22_23.m128_f32[3]; }
		inline float FrobeniusNorm() const
		{
			return sqrt(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_00_01_02_03, m_00_01_02_03), 
									ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(m_10_11_12_13, m_10_11_12_13), 
											   ENFT_SSE::_mm_mul_ps(m_20_21_22_23, m_20_21_22_23)))));
		}
		inline void Print() const
		{
			printf("%f %f %f %f\n", M00(), M01(), M02(), M03());
			printf("%f %f %f %f\n", M10(), M11(), M12(), M13());
			printf("%f %f %f %f\n", M20(), M21(), M22(), M23());
		}

	protected:

		ENFT_SSE::__m128 m_20_21_22_23;

	};

	inline void AddATBTo(const AlignedMatrix2x3f &A, const AlignedMatrix2x4f &B, AlignedMatrix3x4f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_10_11_12_13())), to.M_00_01_02_03());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_13())), to.M_10_11_12_13());
		to.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B.M_10_11_12_13())), to.M_20_21_22_23());
	}
	inline void AddATBTo(const AlignedMatrix3x4f &A, const Vector3f &B, AlignedVector4f &to)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), ENFT_SSE::_mm_set1_ps(B.v0())), 
													  ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), ENFT_SSE::_mm_set1_ps(B.v1()))), 
													  ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), ENFT_SSE::_mm_set1_ps(B.v2()))), to.v0123());
	}
	inline void SubtractATBFrom(const AlignedMatrix3x4f &A, const Vector3f &B, AlignedVector4f &from)
	{
		from.v0123() = ENFT_SSE::_mm_sub_ps(from.v0123(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), ENFT_SSE::_mm_set1_ps(B.v0())), 
																	  ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), ENFT_SSE::_mm_set1_ps(B.v1()))), 
																	  ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), ENFT_SSE::_mm_set1_ps(B.v2()))));
	}
	inline void AddABTo(const AlignedMatrix3x4f &A, const AlignedVector4f &B, Vector3f &to)
	{
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + to.v1();
		to.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + to.v2();
	}
	inline void SubtractABFrom(const AlignedMatrix3x4f &A, const AlignedVector4f &B, Vector3f &from)
	{
		from.v0() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()));
		from.v1() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()));
		from.v2() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()));
	}
	template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(AlignedMatrix3x4f &to) {}
	inline void s1s2TA(const Vector3f &s1, const AlignedVector4f &s2, AlignedMatrix3x4f &A)
	{
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v0()), s2.v0123()), A.M_00_01_02_03());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v1()), s2.v0123()), A.M_10_11_12_13());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v2()), s2.v0123()), A.M_20_21_22_23());
	}
	inline void AB(const AlignedMatrix2x3f &A, const AlignedMatrix3x4f &B, AlignedMatrix2x4f &AB)
	{
		AB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B.M_10_11_12_13())), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), B.M_20_21_22_23()));
		AB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_13())), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B.M_20_21_22_23()));
	}
	inline void ATB(const SymmetricMatrix3f &A, const AlignedMatrix3x4f &B, AlignedMatrix3x4f &AB)
	{
		AB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_10_11_12_13())), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), B.M_20_21_22_23()));
		AB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_13())), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M21()), B.M_20_21_22_23()));
		AB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B.M_10_11_12_13())), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M22()), B.M_20_21_22_23()));
	}
	inline void ABT(const AlignedVector3f &A, const AlignedVector3f &B, AlignedMatrix3x4f &ABT)
	{
#if _DEBUG
		assert(B.reserve() == 1.0f);
#endif
		ABT.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v0()), B.v012x());
		ABT.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v1()), B.v012x());
		ABT.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v2()), B.v012x());
	}

#if _DEBUG
	inline void AB_Debug(const AlignedMatrix2x3f &A, const AlignedMatrix3x4f &B, AlignedMatrix2x4f &AB)
	{
		const float* _A[2] = {&A.M00(), &A.M10()};
		const float* _B[3] = {&B.M00(), &B.M10(), &B.M20()};
		float* _AB[2] = {&AB.M00(), &AB.M10()};
		for(int i = 0; i < 2; ++i)
		for(int j = 0; j < 4; ++j)
		{
			_AB[i][j] = 0;
			for(int k = 0; k < 3; ++k)
				_AB[i][j] += _A[i][k] * _B[k][j];
		}
	}
	inline void AddATBTo_Debug(const AlignedMatrix2x3f &A, const AlignedMatrix2x4f &B, AlignedMatrix3x4f &to)
	{
		const float* _A[2] = {&A.M00(), &A.M10()};
		const float* _B[2] = {&B.M00(), &B.M10()};
		float* _to[3] = {&to.M00(), &to.M10(), &to.M20()};
		for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 4; ++j)
		for(int k = 0; k < 2; ++k)
			_to[i][j] += _A[k][i] * _B[k][j];
	}
	inline void ATB_Debug(const SymmetricMatrix3f &A, const AlignedMatrix3x4f &B, AlignedMatrix3x4f &AB)
	{
		const float _A[3][3] = {{A.M00(), A.M01(), A.M02()}, {A.M10(), A.M11(), A.M12()}, {A.M20(), A.M21(), A.M22()}};
		const float* _B[3] = {&B.M00(), &B.M10(), &B.M20()};
		float* _AB[3] = {&AB.M00(), &AB.M10(), &AB.M20()};
		for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 4; ++j)
		{
			_AB[i][j] = 0;
			for(int k = 0; k < 3; ++k)
				_AB[i][j] += _A[k][i] * _B[k][j];
		}
	}
	inline void SubtractATBFrom_Debug(const AlignedMatrix3x4f &A, const Vector3f &B, AlignedVector4f &from)
	{
		const float* _A[3] = {&A.M00(), &A.M10(), &A.M20()};
		const float* _B = B;
		float* _from = from;
		for(int i = 0; i < 4; ++i)
		for(int j = 0; j < 3; ++j)
			_from[i] -= _A[j][i] * _B[j];
	}
#endif
}

#endif