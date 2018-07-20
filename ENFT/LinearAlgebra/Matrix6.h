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

#ifndef _MATRIX_6_H_
#define _MATRIX_6_H_

#include "Utility/SSE.h"
#include "Matrix3x6.h"
#include "Matrix4x6.h"
#include "Matrix6x4.h"
#include "Vector6.h"

namespace LA
{

	class AlignedCompactMatrix6f : public AlignedCompactMatrix4x6f
	{

	public:

		inline const ENFT_SSE::__m128& M_40_41_42_43() const { return m_40_41_42_43; }	inline ENFT_SSE::__m128& M_40_41_42_43() { return m_40_41_42_43; }
		inline const ENFT_SSE::__m128& M_44_45_54_55() const { return m_44_45_54_55; }	inline ENFT_SSE::__m128& M_44_45_54_55() { return m_44_45_54_55; }
		inline const ENFT_SSE::__m128& M_50_51_52_53() const { return m_50_51_52_53; }	inline ENFT_SSE::__m128& M_50_51_52_53() { return m_50_51_52_53; }
		inline const float& M40() const { return m_40_41_42_43.m128_f32[0]; }	inline float& M40() { return m_40_41_42_43.m128_f32[0]; }
		inline const float& M41() const { return m_40_41_42_43.m128_f32[1]; }	inline float& M41() { return m_40_41_42_43.m128_f32[1]; }
		inline const float& M42() const { return m_40_41_42_43.m128_f32[2]; }	inline float& M42() { return m_40_41_42_43.m128_f32[2]; }
		inline const float& M43() const { return m_40_41_42_43.m128_f32[3]; }	inline float& M43() { return m_40_41_42_43.m128_f32[3]; }
		inline const float& M44() const { return m_44_45_54_55.m128_f32[0]; }	inline float& M44() { return m_44_45_54_55.m128_f32[0]; }
		inline const float& M45() const { return m_44_45_54_55.m128_f32[1]; }	inline float& M45() { return m_44_45_54_55.m128_f32[1]; }
		inline const float& M50() const { return m_50_51_52_53.m128_f32[0]; }	inline float& M50() { return m_50_51_52_53.m128_f32[0]; }
		inline const float& M51() const { return m_50_51_52_53.m128_f32[1]; }	inline float& M51() { return m_50_51_52_53.m128_f32[1]; }
		inline const float& M52() const { return m_50_51_52_53.m128_f32[2]; }	inline float& M52() { return m_50_51_52_53.m128_f32[2]; }
		inline const float& M53() const { return m_50_51_52_53.m128_f32[3]; }	inline float& M53() { return m_50_51_52_53.m128_f32[3]; }
		inline const float& M54() const { return m_44_45_54_55.m128_f32[2]; }	inline float& M54() { return m_44_45_54_55.m128_f32[2]; }
		inline const float& M55() const { return m_44_45_54_55.m128_f32[3]; }	inline float& M55() { return m_44_45_54_55.m128_f32[3]; }

		inline void SetZero() { memset(this, 0, sizeof(AlignedCompactMatrix6f)); }
		inline void GetDiagonal(AlignedVector6f &d) const
		{
			d.v0() = M00();		d.v1() = M11();		d.v2() = M22();		d.v3() = M33();		d.v4() = M44();		d.v5() = M55();
		}
		inline void SetDiagonal(const AlignedVector6f &d)
		{
			M00() = d.v0();		M11() = d.v1();		M22() = d.v2();		M33() = d.v3();		M44() = d.v4();		M55() = d.v5();
		}
		inline void ScaleDiagonal(const float &lambda)
		{
			M00() *= lambda;	M11() *= lambda;	M22() *= lambda;	M33() *= lambda;	M44() *= lambda;	M55() *= lambda;
		}
		inline void IncreaseDiagonal(const float &lambda)
		{
			M00() += lambda;	M11() += lambda;	M22() += lambda;	M33() += lambda;	M44() += lambda;	M55() += lambda;
		}
		inline void ConvertToConventionalStorage(float *work2)
		{
//#if _DEBUG
//			float _M[36];
//			_M[ 0] = M00();		_M[ 1] = M01();		_M[ 2] = M02();		_M[ 3] = M03();		_M[ 4] = M04();		_M[ 5] = M05();
//			_M[ 6] = M10();		_M[ 7] = M11();		_M[ 8] = M12();		_M[ 9] = M13();		_M[10] = M14();		_M[11] = M15();
//			_M[12] = M20();		_M[13] = M21();		_M[14] = M22();		_M[15] = M23();		_M[16] = M24();		_M[17] = M25();
//			_M[18] = M30();		_M[19] = M31();		_M[20] = M32();		_M[21] = M33();		_M[22] = M34();		_M[23] = M35();
//			_M[24] = M40();		_M[25] = M41();		_M[26] = M42();		_M[27] = M43();		_M[28] = M44();		_M[29] = M45();
//			_M[30] = M50();		_M[31] = M51();		_M[32] = M52();		_M[33] = M53();		_M[34] = M54();		_M[35] = M55();
//#endif
			AlignedCompactMatrix4x6f::ConvertToConventionalStorage(work2);
			memcpy(work2, &M54(), 8);
			memcpy(&M54(), &M50(), 16);
			memcpy(&M52(), work2, 8);
		}
		inline void ConvertToConventionalStorage(float *M) const
		{
			AlignedCompactMatrix4x6f::ConvertToConventionalStorage(M);
			memcpy(M + 24, &M40(), 24);
			memcpy(M + 30, &M50(), 16);
			memcpy(M + 34, &M54(), 8);
		}
		inline void ConvertToSpecialStorage(float *work2)
		{
//#if _DEBUG
//			float *p = (float *) this;
//			AlignedCompactMatrix6f _M;
//			_M.M00() = p[ 0];	_M.M01() = p[ 1];	_M.M02() = p[ 2];	_M.M03() = p[ 3];	_M.M04() = p[ 4];	_M.M05() = p[ 5];
//			_M.M10() = p[ 6];	_M.M11() = p[ 7];	_M.M12() = p[ 8];	_M.M13() = p[ 9];	_M.M14() = p[10];	_M.M15() = p[11];
//			_M.M20() = p[12];	_M.M21() = p[13];	_M.M22() = p[14];	_M.M23() = p[15];	_M.M24() = p[16];	_M.M35() = p[17];
//			_M.M30() = p[18];	_M.M31() = p[19];	_M.M32() = p[20];	_M.M33() = p[21];	_M.M34() = p[22];	_M.M35() = p[23];
//			_M.M40() = p[24];	_M.M41() = p[25];	_M.M42() = p[26];	_M.M43() = p[27];	_M.M44() = p[28];	_M.M45() = p[29];
//			_M.M50() = p[30];	_M.M51() = p[31];	_M.M52() = p[32];	_M.M53() = p[33];	_M.M54() = p[34];	_M.M55() = p[35];
//#endif
			AlignedCompactMatrix4x6f::ConvertToSpecialStorage(work2);
			memcpy(work2, &M52(), 8);
			//memcpy(&M50(), &M54(), 16);
			memcpy(&M52(), &M50(), 8);
			memcpy(&M50(), &M54(), 8);
			memcpy(&M54(), work2, 8);
		}
		inline void SetLowerFromUpper()
		{
			M10() = M01();
			M20() = M02();	M21() = M12();
			M30() = M03();	M31() = M13();	M32() = M23();
			M40() = M04();	M41() = M14();	M42() = M24();	M43() = M34();
			M50() = M05();	M51() = M15();	M52() = M25();	M53() = M35();	M54() = M45();
		}
		inline void SetUpperFromLower()
		{
			M01() = M10();
			M02() = M20();	M12() = M21();
			M03() = M30();	M13() = M31();	M23() = M32();
			M04() = M40();	M14() = M41();	M24() = M42();	M34() = M43();
			M05() = M50();	M15() = M51();	M25() = M52();	M35() = M53();	M45() = M54();
		}

		inline void Print() const
		{
			printf("%f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05());
			printf("%f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15());
			printf("%f %f %f %f %f %f\n", M20(), M21(), M22(), M23(), M24(), M25());
			printf("%f %f %f %f %f %f\n", M30(), M31(), M32(), M33(), M34(), M35());
			printf("%f %f %f %f %f %f\n", M40(), M41(), M42(), M43(), M44(), M45());
			printf("%f %f %f %f %f %f\n", M50(), M51(), M52(), M53(), M54(), M55());
		}

	protected:

		ENFT_SSE::__m128 m_40_41_42_43, m_44_45_54_55, m_50_51_52_53;

	};

	inline void AddATAToUpper(const AlignedCompactMatrix2x6f &A, AlignedCompactMatrix6f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), A.M_10_11_12_13())), 
										to.M_00_01_02_03());
		to.M04() = A.M00() * A.M04() + A.M10() * A.M14() + to.M04();
		to.M05() = A.M00() * A.M05() + A.M10() * A.M15() + to.M05();

		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), A.M_10_11_12_13())), 
										to.M_10_11_12_13());
		to.M14() = A.M01() * A.M04() + A.M11() * A.M14() + to.M14();
		to.M15() = A.M01() * A.M05() + A.M11() * A.M15() + to.M15();

		to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + to.M22();
		to.M23() = A.M02() * A.M03() + A.M12() * A.M13() + to.M23();
		to.M24() = A.M02() * A.M04() + A.M12() * A.M14() + to.M24();
		to.M25() = A.M02() * A.M05() + A.M12() * A.M15() + to.M25();

		to.M33() = A.M03() * A.M03() + A.M13() * A.M13() + to.M33();
		to.M34() = A.M03() * A.M04() + A.M13() * A.M14() + to.M34();
		to.M35() = A.M03() * A.M05() + A.M13() * A.M15() + to.M35();

		to.M44() = A.M04() * A.M04() + A.M14() * A.M14() + to.M44();
		to.M45() = A.M04() * A.M05() + A.M14() * A.M15() + to.M45();

		to.M55() = A.M05() * A.M05() + A.M15() * A.M15() + to.M55();
	}
	inline void AddATAToUpper(const AlignedCompactMatrix2x6f &A, AlignedCompactMatrix6f &to, ENFT_SSE::__m128 *work0) { AddATAToUpper(A, to); }
	template<class MATRIX> inline void FinishAdditionATAToUpper(AlignedCompactMatrix6f &to) {}
	template<class MATRIX_1, class MATRIX_2> inline void FinishAdditionATAToUpper(AlignedCompactMatrix6f &to) {}

	inline void AddAATToUpper(const AlignedVector6f &A, AlignedCompactMatrix6f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v0()), A.v0123()), to.M_00_01_02_03());
		to.M04() = A.v0() * A.v4() + to.M04();
		to.M05() = A.v0() * A.v5() + to.M05();
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v1()), A.v0123()), to.M_10_11_12_13());
		to.M14() = A.v1() * A.v4() + to.M14();
		to.M15() = A.v1() * A.v5() + to.M15();
		to.M22() = A.v2() * A.v2() + to.M22();
		to.M23() = A.v2() * A.v3() + to.M23();
		to.M24() = A.v2() * A.v4() + to.M24();
		to.M25() = A.v2() * A.v5() + to.M25();
		to.M33() = A.v3() * A.v3() + to.M33();
		to.M34() = A.v3() * A.v4() + to.M34();
		to.M35() = A.v3() * A.v5() + to.M35();
		to.M44() = A.v4() * A.v4() + to.M44();
		to.M45() = A.v4() * A.v5() + to.M45();
		to.M55() = A.v5() * A.v5() + to.M55();
	}

	inline void AddATAToUpper(const AlignedCompactMatrix3x6f &A, AlignedCompactMatrix6f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), A.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), A.M_20_21_22_23()), to.M_00_01_02_03()));
		to.M04() = A.M00() * A.M04() + A.M10() * A.M14() + A.M20() * A.M24() + to.M04();
		to.M05() = A.M00() * A.M05() + A.M10() * A.M15() + A.M20() * A.M25() + to.M05();

		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), A.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M21()), A.M_20_21_22_23()), to.M_10_11_12_13()));
		to.M14() = A.M01() * A.M04() + A.M11() * A.M14() + A.M21() * A.M24() + to.M14();
		to.M15() = A.M01() * A.M05() + A.M11() * A.M15() + A.M21() * A.M25() + to.M15();

		to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + A.M22() * A.M22() + to.M22();
		to.M23() = A.M02() * A.M03() + A.M12() * A.M13() + A.M22() * A.M23() + to.M23();
		to.M24() = A.M02() * A.M04() + A.M12() * A.M14() + A.M22() * A.M24() + to.M24();
		to.M25() = A.M02() * A.M05() + A.M12() * A.M15() + A.M22() * A.M25() + to.M25();

		to.M33() = A.M03() * A.M03() + A.M13() * A.M13() + A.M23() * A.M23() + to.M33();
		to.M34() = A.M03() * A.M04() + A.M13() * A.M14() + A.M23() * A.M24() + to.M34();
		to.M35() = A.M03() * A.M05() + A.M13() * A.M15() + A.M23() * A.M25() + to.M35();

		to.M44() = A.M04() * A.M04() + A.M14() * A.M14() + A.M24() * A.M24() + to.M44();
		to.M45() = A.M04() * A.M05() + A.M14() * A.M15() + A.M24() * A.M25() + to.M45();

		to.M55() = A.M05() * A.M05() + A.M15() * A.M15() + A.M25() * A.M25() + to.M55();
	}
	inline void AddATAToUpper(const AlignedCompactMatrix3x6f &A, AlignedCompactMatrix6f &to, ENFT_SSE::__m128 *work0) { AddATAToUpper(A, to); }

	inline void SetLowerFromUpper(AlignedCompactMatrix6f &M) { M.SetLowerFromUpper(); }
	inline void GetDiagonal(const AlignedCompactMatrix6f &M, AlignedVector6f &d) { M.GetDiagonal(d); }
	inline void SetDiagonal(const AlignedVector6f &d, AlignedCompactMatrix6f &M) { M.SetDiagonal(d); }
	inline void ScaleDiagonal(const float &lambda, AlignedCompactMatrix6f &M) { M.ScaleDiagonal(lambda); }
	inline void IncreaseDiagonal(const float &lambda, AlignedCompactMatrix6f &M) { M.IncreaseDiagonal(lambda); }

	inline void AB(const AlignedCompactMatrix6f &A, const AlignedVector6f &B, AlignedVector6f &AB, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1];
		AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3];

		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_24_25_34_35(), B.v45xx());
		AB.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1];
		AB.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3];

		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_44_45_54_55(), B.v45xx());
		AB.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1];
		AB.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3];
	}
	inline void AB(const AlignedCompactMatrix6f &A, const AlignedVector6f &B, AlignedVector6f &AB)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		ENFT_SSE::__m128 work;
		work = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work.m128_f32[0] + work.m128_f32[1];
		AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work.m128_f32[2] + work.m128_f32[3];

		work = ENFT_SSE::_mm_mul_ps(A.M_24_25_34_35(), B.v45xx());
		AB.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + work.m128_f32[0] + work.m128_f32[1];
		AB.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123())) + work.m128_f32[2] + work.m128_f32[3];

		work = ENFT_SSE::_mm_mul_ps(A.M_44_45_54_55(), B.v45xx());
		AB.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123())) + work.m128_f32[0] + work.m128_f32[1];
		AB.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123())) + work.m128_f32[2] + work.m128_f32[3];
	}
	inline void ABmC(const AlignedCompactMatrix6f &A, const AlignedVector6f &B, const AlignedVector6f &C, AlignedVector6f &ABmC, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		ABmC.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] - C.v0();
		ABmC.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] - C.v1();

		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_24_25_34_35(), B.v45xx());
		ABmC.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] - C.v2();
		ABmC.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] - C.v3();

		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_44_45_54_55(), B.v45xx());
		ABmC.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] - C.v4();
		ABmC.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] - C.v5();
	}
	inline void AddABTo(const AlignedCompactMatrix6f &A, const AlignedVector6f &B, AlignedVector6f &to, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + to.v1();

		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_24_25_34_35(), B.v45xx());
		to.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + to.v2();
		to.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + to.v3();

		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_44_45_54_55(), B.v45xx());
		to.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + to.v4();
		to.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + to.v5();
	}
	inline void AddATBTo(const AlignedCompactMatrix6f &A, const AlignedVector6f &B, AlignedVector6f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])), to.v0123());
		work2[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]));
		to.v4() = work2[0].m128_f32[0] + work2[0].m128_f32[2] + to.v4();
		to.v5() = work2[0].m128_f32[1] + work2[0].m128_f32[3] + to.v5();

		work2[0] = ENFT_SSE::_mm_set1_ps(B.v2());
		work2[1] = ENFT_SSE::_mm_set1_ps(B.v3());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), work2[1])), to.v0123());
		work2[0] = ENFT_SSE::_mm_mul_ps(A.M_24_25_34_35(), ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]));
		to.v4() = work2[0].m128_f32[0] + work2[0].m128_f32[2] + to.v4();
		to.v5() = work2[0].m128_f32[1] + work2[0].m128_f32[3] + to.v5();

		work2[0] = ENFT_SSE::_mm_set1_ps(B.v4());
		work2[1] = ENFT_SSE::_mm_set1_ps(B.v5());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), work2[1])), to.v0123());
		work2[0] = ENFT_SSE::_mm_mul_ps(A.M_44_45_54_55(), ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]));
		to.v4() = work2[0].m128_f32[0] + work2[0].m128_f32[2] + to.v4();
		to.v5() = work2[0].m128_f32[1] + work2[0].m128_f32[3] + to.v5();
	}
	inline void SubtractATBFrom(const AlignedCompactMatrix3x6f &A, const AlignedCompactMatrix3x6f &B, AlignedCompactMatrix6f &from, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), B.M_20_21_22_23())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M04() -= work2[0].m128_f32[0] + work2[0].m128_f32[2] + A.M20() * B.M24();
		from.M05() -= work2[0].m128_f32[1] + work2[0].m128_f32[3] + A.M20() * B.M25();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M21()), B.M_20_21_22_23())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M14() -= work2[0].m128_f32[0] + work2[0].m128_f32[2] + A.M21() * B.M24();
		from.M15() -= work2[0].m128_f32[1] + work2[0].m128_f32[3] + A.M21() * B.M25();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M22()), B.M_20_21_22_23())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M24() -= work2[0].m128_f32[0] + work2[0].m128_f32[2] + A.M22() * B.M24();
		from.M25() -= work2[0].m128_f32[1] + work2[0].m128_f32[3] + A.M22() * B.M25();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M03());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M13());
		from.M_30_31_32_33() = ENFT_SSE::_mm_sub_ps(from.M_30_31_32_33(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M23()), B.M_20_21_22_23())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M34() -= work2[0].m128_f32[0] + work2[0].m128_f32[2] + A.M23() * B.M24();
		from.M35() -= work2[0].m128_f32[1] + work2[0].m128_f32[3] + A.M23() * B.M25();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M04());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M14());
		from.M_40_41_42_43() = ENFT_SSE::_mm_sub_ps(from.M_40_41_42_43(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M24()), B.M_20_21_22_23())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M44() -= work2[0].m128_f32[0] + work2[0].m128_f32[2] + A.M24() * B.M24();
		from.M45() -= work2[0].m128_f32[1] + work2[0].m128_f32[3] + A.M24() * B.M25();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M05());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M15());
		from.M_50_51_52_53() = ENFT_SSE::_mm_sub_ps(from.M_50_51_52_53(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M25()), B.M_20_21_22_23())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M54() -= work2[0].m128_f32[0] + work2[0].m128_f32[2] + A.M25() * B.M24();
		from.M55() -= work2[0].m128_f32[1] + work2[0].m128_f32[3] + A.M25() * B.M25();
	}
	inline void SubtractATBFrom(const AlignedCompactMatrix2x6f &A, const AlignedCompactMatrix2x6f &B, AlignedCompactMatrix6f &from, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M04() -= work2[0].m128_f32[0] + work2[0].m128_f32[2];
		from.M05() -= work2[0].m128_f32[1] + work2[0].m128_f32[3];

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M14() -= work2[0].m128_f32[0] + work2[0].m128_f32[2];
		from.M15() -= work2[0].m128_f32[1] + work2[0].m128_f32[3];

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M24() -= work2[0].m128_f32[0] + work2[0].m128_f32[2];
		from.M25() -= work2[0].m128_f32[1] + work2[0].m128_f32[3];

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M03());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M13());
		from.M_30_31_32_33() = ENFT_SSE::_mm_sub_ps(from.M_30_31_32_33(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M34() -= work2[0].m128_f32[0] + work2[0].m128_f32[2];
		from.M35() -= work2[0].m128_f32[1] + work2[0].m128_f32[3];

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M04());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M14());
		from.M_40_41_42_43() = ENFT_SSE::_mm_sub_ps(from.M_40_41_42_43(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M44() -= work2[0].m128_f32[0] + work2[0].m128_f32[2];
		from.M45() -= work2[0].m128_f32[1] + work2[0].m128_f32[3];

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M05());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M15());
		from.M_50_51_52_53() = ENFT_SSE::_mm_sub_ps(from.M_50_51_52_53(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		from.M54() -= work2[0].m128_f32[0] + work2[0].m128_f32[2];
		from.M55() -= work2[0].m128_f32[1] + work2[0].m128_f32[3];
	}
	inline void SubtractABTFrom(const AlignedMatrix6x4f &A, const AlignedMatrix6x4f &B, AlignedCompactMatrix6f &from)
	{
		from.M00() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.M_00_01_02_03()));
		from.M01() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.M_10_11_12_13()));
		from.M02() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.M_20_21_22_23()));
		from.M03() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.M_30_31_32_33()));
		from.M04() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.M_40_41_42_43()));
		from.M05() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.M_50_51_52_53()));
		from.M10() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.M_00_01_02_03()));
		from.M11() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.M_10_11_12_13()));
		from.M12() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.M_20_21_22_23()));
		from.M13() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.M_30_31_32_33()));
		from.M14() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.M_40_41_42_43()));
		from.M15() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.M_50_51_52_53()));
		from.M20() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.M_00_01_02_03()));
		from.M21() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.M_10_11_12_13()));
		from.M22() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.M_20_21_22_23()));
		from.M23() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.M_30_31_32_33()));
		from.M24() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.M_40_41_42_43()));
		from.M25() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.M_50_51_52_53()));
		from.M30() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.M_00_01_02_03()));
		from.M31() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.M_10_11_12_13()));
		from.M32() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.M_20_21_22_23()));
		from.M33() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.M_30_31_32_33()));
		from.M34() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.M_40_41_42_43()));
		from.M35() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.M_50_51_52_53()));
		from.M40() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.M_00_01_02_03()));
		from.M41() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.M_10_11_12_13()));
		from.M42() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.M_20_21_22_23()));
		from.M43() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.M_30_31_32_33()));
		from.M44() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.M_40_41_42_43()));
		from.M45() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.M_50_51_52_53()));
		from.M50() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.M_00_01_02_03()));
		from.M51() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.M_10_11_12_13()));
		from.M52() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.M_20_21_22_23()));
		from.M53() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.M_30_31_32_33()));
		from.M54() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.M_40_41_42_43()));
		from.M55() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.M_50_51_52_53()));
	}

	inline void ssTA(const AlignedVector6f &s, AlignedCompactMatrix6f &A, ENFT_SSE::__m128 *work3)
	{
#if _DEBUG
		s.AssertReserve45();
#endif
		work3[0] = ENFT_SSE::_mm_set1_ps(s.v0());		work3[1] = ENFT_SSE::_mm_set1_ps(s.v1());		work3[2] = ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]);
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work3[0], s.v0123()), A.M_00_01_02_03());
		A.M_04_05_14_15() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work3[2], s.v45xx()), A.M_04_05_14_15());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work3[1], s.v0123()), A.M_10_11_12_13());

		work3[0] = ENFT_SSE::_mm_set1_ps(s.v2());		work3[1] = ENFT_SSE::_mm_set1_ps(s.v3());		work3[2] = ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]);
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work3[0], s.v0123()), A.M_20_21_22_23());
		A.M_24_25_34_35() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work3[2], s.v45xx()), A.M_24_25_34_35());
		A.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work3[1], s.v0123()), A.M_30_31_32_33());

		work3[0] = ENFT_SSE::_mm_set1_ps(s.v4());		work3[1] = ENFT_SSE::_mm_set1_ps(s.v5());		work3[2] = ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]);
		A.M_40_41_42_43() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work3[0], s.v0123()), A.M_40_41_42_43());
		A.M_44_45_54_55() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work3[2], s.v45xx()), A.M_44_45_54_55());
		A.M_50_51_52_53() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work3[1], s.v0123()), A.M_50_51_52_53());
	}

	bool InvertSymmetricUpper(AlignedCompactMatrix6f &A, float *work36);
	bool InvertSymmetricUpper(const AlignedCompactMatrix6f &A, AlignedCompactMatrix6f &Ainv, float *work36);
	bool SolveLinearSystemSymmetricUpper(AlignedCompactMatrix6f &A, AlignedVector6f &b, float *work2);

#if _DEBUG
	inline void SubtractABTFrom_Debug(const AlignedMatrix6x4f &A, const AlignedMatrix6x4f &B, AlignedCompactMatrix6f &from)
	{
		const float* _A[6] = {&A.M00(), &A.M10(), &A.M20(), &A.M30(), &A.M40(), &A.M50()};
		const float* _B[6] = {&B.M00(), &B.M10(), &B.M20(), &B.M30(), &B.M40(), &B.M50()};
		float work[2];
		from.ConvertToConventionalStorage(work);
		float* _from[6] = {from, from + 6, from + 12, from + 18, from + 24, from + 30};
		for(int i = 0; i < 6; ++i)
		for(int j = 0; j < 6; ++j)
		for(int k = 0; k < 4; ++k)
			_from[i][j] -= _A[i][k] * _B[j][k];
		from.ConvertToSpecialStorage(work);
	}
#endif

}

#endif