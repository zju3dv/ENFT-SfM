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

#ifndef _MATRIX_3x6_H_
#define _MATRIX_3x6_H_

#include "Utility/SSE.h"
#include "Matrix2x6.h"
#include "Matrix3.h"
#include "Vector6.h"

namespace LA
{

	class AlignedCompactMatrix3x6f : public AlignedCompactMatrix2x6f
	{

	public:

		inline const ENFT_SSE::__m128& M_20_21_22_23() const { return m_20_21_22_23; }				inline ENFT_SSE::__m128& M_20_21_22_23() { return m_20_21_22_23; }
		inline const ENFT_SSE::__m128& M_24_25_x_x  () const { return m_24_25_x_x  ; }				inline ENFT_SSE::__m128& M_24_25_x_x  () { return m_24_25_x_x  ; }
		inline const  float& M20()			 const { return m_20_21_22_23.m128_f32[0]; }	inline  float& M20()		   { return m_20_21_22_23.m128_f32[0]; }
		inline const  float& M21()			 const { return m_20_21_22_23.m128_f32[1]; }	inline  float& M21()		   { return m_20_21_22_23.m128_f32[1]; }
		inline const  float& M22()			 const { return m_20_21_22_23.m128_f32[2]; }	inline  float& M22()		   { return m_20_21_22_23.m128_f32[2]; }
		inline const  float& M23()			 const { return m_20_21_22_23.m128_f32[3]; }	inline  float& M23()		   { return m_20_21_22_23.m128_f32[3]; }
		inline const  float& M24()			 const { return m_24_25_x_x  .m128_f32[0]; }	inline  float& M24()		   { return m_24_25_x_x  .m128_f32[0]; }
		inline const  float& M25()			 const { return m_24_25_x_x  .m128_f32[1]; }	inline  float& M25()		   { return m_24_25_x_x  .m128_f32[1]; }
		inline const  float& reserve0()		 const { return m_24_25_x_x  .m128_f32[2]; }	inline  float& reserve0()	   { return m_24_25_x_x  .m128_f32[2]; }
		inline const  float& reserve1()		 const { return m_24_25_x_x  .m128_f32[3]; }	inline  float& reserve1()	   { return m_24_25_x_x  .m128_f32[3]; }
		inline void ConvertToConventionalStorage(float* M) const
		{
			AlignedCompactMatrix2x6f::ConvertToConventionalStorage(M);
			memcpy(M + 12, &M20(), 24);
		}
		inline void SetColumn0(const float &m00, const float &m10, const float &m20) { M00() = m00; M10() = m10; M20() = m20; }
		inline void SetColumn1(const float &m01, const float &m11, const float &m21) { M01() = m01; M11() = m11; M21() = m21; }
		inline void SetColumn2(const float &m02, const float &m12, const float &m22) { M02() = m02; M12() = m12; M22() = m22; }
		inline void SetColumn3(const float &m03, const float &m13, const float &m23) { M03() = m03; M13() = m13; M23() = m23; }
		inline void SetColumn4(const float &m04, const float &m14, const float &m24) { M04() = m04; M14() = m14; M24() = m24; }
		inline void SetColumn5(const float &m05, const float &m15, const float &m25) { M05() = m05; M15() = m15; M25() = m25; }
		inline void SetReserveZero() { reserve0() = reserve1() = 0; }

	protected:

		ENFT_SSE::__m128 m_20_21_22_23, m_24_25_x_x;

	};

	template<ushort STAGE> inline void SetReserve(AlignedCompactMatrix3x6f &M) {}
	inline void SetReserve(AlignedCompactMatrix3x6f &M) {}

	inline void AddAij2To(const AlignedCompactMatrix3x6f &A, AlignedVector6f &to)
	{
#if _DEBUG
		assert(A.reserve0() == 0 && A.reserve1() == 0);
#endif
		to.v0123() = ENFT_SSE::_mm_add_ps(
			ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_10_11_12_13())), 
			ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), A.M_20_21_22_23()), to.v0123())
			);
		to.v45xx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), A.M_04_05_14_15()), ENFT_SSE::_mm_mul_ps(A.M_24_25_x_x(), A.M_24_25_x_x())), to.v45xx());
	}

	inline void sA(const ENFT_SSE::__m128 &s, AlignedCompactMatrix3x6f &A)
	{
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(s, A.M_00_01_02_03());
		A.M_04_05_14_15() = ENFT_SSE::_mm_mul_ps(s, A.M_04_05_14_15());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(s, A.M_10_11_12_13());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(s, A.M_20_21_22_23());
		A.M_24_25_x_x() = ENFT_SSE::_mm_mul_ps(s, A.M_24_25_x_x());
	}
	inline void sA(const AlignedVector6f &s, AlignedCompactMatrix3x6f &A)
	{
#if _DEBUG
		assert(s.reserve0() == s.v4() && s.reserve1() == s.v5());
#endif
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.M_00_01_02_03());
		A.M_04_05_14_15() = ENFT_SSE::_mm_mul_ps(s.v45xx(), A.M_04_05_14_15());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.M_10_11_12_13());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.M_20_21_22_23());
		A.M_24_25_x_x() = ENFT_SSE::_mm_mul_ps(s.v45xx(), A.M_24_25_x_x());
	}

	inline void AddATBTo(const AlignedCompactMatrix3x6f &A, const AlignedVector3f &B, AlignedVector6f &to, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work3[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		work3[2] = ENFT_SSE::_mm_set1_ps(B.v2());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work3[1])), 
								ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work3[2]), to.v0123()));
		to.v45xx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(work3[0], work3[1])), to.v45xx());
		to.v4() = A.M24() * B.v2() + to.v4();
		to.v5() = A.M25() * B.v2() + to.v5();
	}
	template<> inline void FinishAdditionATBTo<AlignedCompactMatrix3x6f>(AlignedVector6f &to) { to.AddReserveTo45(); }
	template<class MATRIX_A, class MATRIX_B> inline void FinishAdditionATBTo(AlignedCompactMatrix3x6f &to) {}
#if _DEBUG
	inline void DebugAddATBTo(const AlignedCompactMatrix3x6f &A, const AlignedVector3f &B, AlignedVector6f &to)
	{
		to.v0() += A.M00() * B.v0() + A.M10() * B.v1() + A.M20() * B.v2();
		to.v1() += A.M01() * B.v0() + A.M11() * B.v1() + A.M21() * B.v2();
		to.v2() += A.M02() * B.v0() + A.M12() * B.v1() + A.M22() * B.v2();
		to.v3() += A.M03() * B.v0() + A.M13() * B.v1() + A.M23() * B.v2();
		to.v4() += A.M04() * B.v0() + A.M14() * B.v1() + A.M24() * B.v2();
		to.v5() += A.M05() * B.v0() + A.M15() * B.v1() + A.M25() * B.v2();
	}
#endif

	inline void AB(const AlignedCompactMatrix3x6f &A, const AlignedVector6f &B, AlignedVector3f &AB, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1];
		AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3];
		AB.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + A.M24() * B.v4() + A.M25() * B.v5();
	}
	inline void ABpCD(const AlignedCompactMatrix3x6f &A, const AlignedVector6f &B, const AlignedCompactMatrix3x6f &C, const AlignedVector6f &D, AlignedVector3f &ABpCD, 
					  ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
		D.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx()), ENFT_SSE::_mm_mul_ps(C.M_04_05_14_15(), D.v45xx()));
		ABpCD.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(C.M_00_01_02_03(), D.v0123()))) + work1[0].m128_f32[0] + work1[0].m128_f32[1];
		ABpCD.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(C.M_10_11_12_13(), D.v0123()))) + work1[0].m128_f32[2] + work1[0].m128_f32[3];
		ABpCD.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(C.M_20_21_22_23(), D.v0123()))) + A.M24() * B.v4() + A.M25() * B.v5() + C.M24() * D.v4() + C.M25() * D.v5();
	}
	inline void ATB(const Matrix2x3f &A, const AlignedCompactMatrix2x6f &B, AlignedCompactMatrix3x6f &ATB, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M04() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M05() = work2[0].m128_f32[1] + work2[0].m128_f32[3];

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M14() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M15() = work2[0].m128_f32[1] + work2[0].m128_f32[3];

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		ATB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M24() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M25() = work2[0].m128_f32[1] + work2[0].m128_f32[3];
	}
	inline void ATB(const AlignedMatrix2x3f &A, const AlignedCompactMatrix2x6f &B, AlignedCompactMatrix3x6f &ATB, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M04() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M05() = work2[0].m128_f32[1] + work2[0].m128_f32[3];

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M14() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M15() = work2[0].m128_f32[1] + work2[0].m128_f32[3];

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		ATB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M24() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M25() = work2[0].m128_f32[1] + work2[0].m128_f32[3];
	}
	inline void AddATBTo(const AlignedMatrix2x3f &A, const AlignedCompactMatrix2x6f &B, AlignedCompactMatrix3x6f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), to.M_00_01_02_03());
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		to.M04() = work2[0].m128_f32[0] + work2[0].m128_f32[2] + to.M04();
		to.M05() = work2[0].m128_f32[1] + work2[0].m128_f32[3] + to.M05();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), to.M_10_11_12_13());
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		to.M14() = work2[0].m128_f32[0] + work2[0].m128_f32[2] + to.M14();
		to.M15() = work2[0].m128_f32[1] + work2[0].m128_f32[3] + to.M15();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		to.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), to.M_20_21_22_23());
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		to.M24() = work2[0].m128_f32[0] + work2[0].m128_f32[2] + to.M24();
		to.M25() = work2[0].m128_f32[1] + work2[0].m128_f32[3] + to.M25();
	}
	inline void ATB(const AlignedMatrix3f &A, const AlignedCompactMatrix3x6f &B, AlignedCompactMatrix3x6f &ATB, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M20());
		ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23()));
		work3[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]), B.M_04_05_14_15());
		ATB.M04() = work3[0].m128_f32[0] + work3[0].m128_f32[2] + A.M20() * B.M24();
		ATB.M05() = work3[0].m128_f32[1] + work3[0].m128_f32[3] + A.M20() * B.M25();

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M21());
		ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23()));
		work3[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]), B.M_04_05_14_15());
		ATB.M14() = work3[0].m128_f32[0] + work3[0].m128_f32[2] + A.M21() * B.M24();
		ATB.M15() = work3[0].m128_f32[1] + work3[0].m128_f32[3] + A.M21() * B.M25();

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M22());
		ATB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23()));
		work3[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]), B.M_04_05_14_15());
		ATB.M24() = work3[0].m128_f32[0] + work3[0].m128_f32[2] + A.M22() * B.M24();
		ATB.M25() = work3[0].m128_f32[1] + work3[0].m128_f32[3] + A.M22() * B.M25();
	}
	inline void AddATBTo(const AlignedMatrix3f &A, const AlignedCompactMatrix3x6f &B, AlignedCompactMatrix3x6f &to, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M20());
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23()), to.M_00_01_02_03()));
		work3[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]), B.M_04_05_14_15());
		to.M04() = work3[0].m128_f32[0] + work3[0].m128_f32[2] + A.M20() * B.M24() + to.M04();
		to.M05() = work3[0].m128_f32[1] + work3[0].m128_f32[3] + A.M20() * B.M25() + to.M05();

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M21());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23()), to.M_10_11_12_13()));
		work3[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]), B.M_04_05_14_15());
		to.M14() = work3[0].m128_f32[0] + work3[0].m128_f32[2] + A.M21() * B.M24() + to.M14();
		to.M15() = work3[0].m128_f32[1] + work3[0].m128_f32[3] + A.M21() * B.M25() + to.M15();

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M22());
		to.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23()), to.M_20_21_22_23()));
		work3[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]), B.M_04_05_14_15());
		to.M24() = work3[0].m128_f32[0] + work3[0].m128_f32[2] + A.M22() * B.M24() + to.M24();
		to.M25() = work3[0].m128_f32[1] + work3[0].m128_f32[3] + A.M22() * B.M25() + to.M25();
	}
	inline void ATB(const SymmetricMatrix3f &A, const AlignedCompactMatrix3x6f &B, AlignedCompactMatrix3x6f &ATB, ENFT_SSE::__m128 *work4)
	{
		work4[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work4[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		work4[2] = ENFT_SSE::_mm_set1_ps(A.M20());
		ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work4[1], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work4[2], B.M_20_21_22_23()));
		work4[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work4[0], work4[1]), B.M_04_05_14_15());
		ATB.M04() = work4[0].m128_f32[0] + work4[0].m128_f32[2] + A.M20() * B.M24();
		ATB.M05() = work4[0].m128_f32[1] + work4[0].m128_f32[3] + A.M20() * B.M25();

		work4[0] = ENFT_SSE::_mm_set1_ps(A.M11());
		work4[3] = ENFT_SSE::_mm_set1_ps(A.M21());
		ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[1], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work4[0], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work4[3], B.M_20_21_22_23()));
		work4[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work4[1], work4[0]), B.M_04_05_14_15());
		ATB.M14() = work4[0].m128_f32[0] + work4[0].m128_f32[2] + A.M21() * B.M24();
		ATB.M15() = work4[0].m128_f32[1] + work4[0].m128_f32[3] + A.M21() * B.M25();

		ATB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[2], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work4[3], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M22()), B.M_20_21_22_23()));
		work4[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work4[2], work4[3]), B.M_04_05_14_15());
		ATB.M24() = work4[0].m128_f32[0] + work4[0].m128_f32[2] + A.M22() * B.M24();
		ATB.M25() = work4[0].m128_f32[1] + work4[0].m128_f32[3] + A.M22() * B.M25();
	}
	inline void AddATBTo(const AlignedCompactMatrix3x6f &A, const Vector3f &B, AlignedVector6f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])), 
								ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), ENFT_SSE::_mm_set1_ps(B.v2())), to.v0123()));
		work2[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]));
		to.v4() = work2[0].m128_f32[0] + work2[0].m128_f32[2] + A.M24() * B.v2() + to.v4();
		to.v5() = work2[0].m128_f32[1] + work2[0].m128_f32[3] + A.M25() * B.v2() + to.v5();
	}
	inline void SubtractATBFrom(const AlignedCompactMatrix3x6f &A, const Vector3f &B, AlignedVector6f &from, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work3[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		work3[2] = ENFT_SSE::_mm_set1_ps(B.v2());
		from.v0123() = ENFT_SSE::_mm_sub_ps(from.v0123(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work3[1])), ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work3[2])));
		work3[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]));
		from.v4() -= work3[0].m128_f32[0] + work3[0].m128_f32[2] + A.M24() * B.v2();
		from.v5() -= work3[0].m128_f32[1] + work3[0].m128_f32[3] + A.M25() * B.v2();
	}
	inline void SubtractATBFrom(const Matrix3f &A, const AlignedCompactMatrix3x6f &B, AlignedCompactMatrix3x6f &from, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M00());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M10());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M20());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		work3[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]), B.M_04_05_14_15());
		from.M04() -= work3[0].m128_f32[0] + work3[0].m128_f32[2] + A.M20() * B.M24();
		from.M05() -= work3[0].m128_f32[1] + work3[0].m128_f32[3] + A.M20() * B.M25();
		
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M01());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M11());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M21());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		work3[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]), B.M_04_05_14_15());
		from.M14() -= work3[0].m128_f32[0] + work3[0].m128_f32[2] + A.M21() * B.M24();
		from.M15() -= work3[0].m128_f32[1] + work3[0].m128_f32[3] + A.M21() * B.M25();
		
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M02());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M12());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M22());
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		work3[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]), B.M_04_05_14_15());
		from.M24() -= work3[0].m128_f32[0] + work3[0].m128_f32[2] + A.M22() * B.M24();
		from.M25() -= work3[0].m128_f32[1] + work3[0].m128_f32[3] + A.M22() * B.M25();
	}
	inline void AddABTo(const AlignedCompactMatrix3x6f &A, const AlignedVector6f &B, Vector3f &to, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + to.v1();
		to.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + A.M24() * B.v4() + A.M25() * B.v5()		   + to.v2();
	}
	inline void SubtractABFrom(const AlignedCompactMatrix3x6f &A, const AlignedVector6f &B, Vector3f &from, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		from.v0() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1];
		from.v1() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3];
		from.v2() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + A.M24() * B.v4() + A.M25() * B.v5();
	}
	inline void SubtractABFrom(const AlignedMatrix2x3f &A, AlignedCompactMatrix3x6f &B, AlignedCompactMatrix2x6f &from, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_setr_ps(B.M04(), B.M14(), B.M24(), 0.0f);
		work2[1] = ENFT_SSE::_mm_setr_ps(B.M05(), B.M15(), B.M25(), 0.0f);
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), B.M_20_21_22_23())));
		from.M04() -= ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work2[0]));
		from.M05() -= ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), work2[1]));
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B.M_20_21_22_23())));
		from.M14() -= ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work2[0]));
		from.M15() -= ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), work2[1]));
	}
	inline void AddATo(const AlignedCompactMatrix3x6f &A, AlignedCompactMatrix3x6f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(A.M_00_01_02_03(), to.M_00_01_02_03());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(A.M_10_11_12_13(), to.M_10_11_12_13());
		to.M_04_05_14_15() = ENFT_SSE::_mm_add_ps(A.M_04_05_14_15(), to.M_04_05_14_15());
		to.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(A.M_20_21_22_23(), to.M_20_21_22_23());
		to.M24() = A.M24() + to.M24();
		to.M25() = A.M25() + to.M25();
	}

	inline void s1s2TA(const Vector3f &s1, const AlignedVector6f &s2, AlignedCompactMatrix3x6f &A, ENFT_SSE::__m128 *work2)
	{
#if _DEBUG
		s2.AssertReserve45();
#endif
		work2[0] = ENFT_SSE::_mm_set1_ps(s1.v0());
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work2[0], s2.v0123()), A.M_00_01_02_03());
		work2[1] = ENFT_SSE::_mm_set1_ps(s1.v1());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work2[1], s2.v0123()), A.M_10_11_12_13());
		A.M_04_05_14_15() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), s2.v45xx()), A.M_04_05_14_15());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s1.v2()), s2.v0123()), A.M_20_21_22_23());
		A.M24() *= s1.v2() * s2.v4();
		A.M25() *= s1.v2() * s2.v5();
	}
}

#endif