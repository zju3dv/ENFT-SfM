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

#ifndef _MATRIX_3x7_H_
#define _MATRIX_3x7_H_

#include "Matrix2x7.h"
#include "Matrix3.h"
#if _DEBUG
#include "Utility/Utility.h"
#endif

namespace LA
{

	class AlignedMatrix3x7f : public AlignedMatrix2x7f
	{

	public:

		inline const ENFT_SSE::__m128& M_20_21_22_23() const { return m_20_21_22_23; }		inline ENFT_SSE::__m128& M_20_21_22_23() { return m_20_21_22_23; }
		inline const ENFT_SSE::__m128& M_24_25_26_x () const { return m_24_25_26_x; }			inline ENFT_SSE::__m128& M_24_25_26_x () { return m_24_25_26_x; }
		inline const float& M20() const { return m_20_21_22_23.m128_f32[0]; }		inline float& M20() { return m_20_21_22_23.m128_f32[0]; }
		inline const float& M21() const { return m_20_21_22_23.m128_f32[1]; }		inline float& M21() { return m_20_21_22_23.m128_f32[1]; }
		inline const float& M22() const { return m_20_21_22_23.m128_f32[2]; }		inline float& M22() { return m_20_21_22_23.m128_f32[2]; }
		inline const float& M23() const { return m_20_21_22_23.m128_f32[3]; }		inline float& M23() { return m_20_21_22_23.m128_f32[3]; }
		inline const float& M24() const { return m_24_25_26_x.m128_f32[0]; }		inline float& M24() { return m_24_25_26_x.m128_f32[0]; }
		inline const float& M25() const { return m_24_25_26_x.m128_f32[1]; }		inline float& M25() { return m_24_25_26_x.m128_f32[1]; }
		inline const float& M26() const { return m_24_25_26_x.m128_f32[2]; }		inline float& M26() { return m_24_25_26_x.m128_f32[2]; }
		inline const float& reserve2() const { return m_24_25_26_x.m128_f32[3]; }	inline float& reserve2() { return m_24_25_26_x.m128_f32[3]; }
		inline void operator += (const AlignedMatrix3x7f &M)
		{
			m_00_01_02_03 = ENFT_SSE::_mm_add_ps(M.m_00_01_02_03, m_00_01_02_03);
			m_04_05_06_x  = ENFT_SSE::_mm_add_ps(M.m_04_05_06_x , m_04_05_06_x );
			m_10_11_12_13 = ENFT_SSE::_mm_add_ps(M.m_10_11_12_13, m_10_11_12_13);
			m_14_15_16_x  = ENFT_SSE::_mm_add_ps(M.m_14_15_16_x , m_14_15_16_x );
			m_20_21_22_23 = ENFT_SSE::_mm_add_ps(M.m_20_21_22_23, m_20_21_22_23);
			m_24_25_26_x  = ENFT_SSE::_mm_add_ps(M.m_24_25_26_x , m_24_25_26_x );
		}
		inline void Print() const
		{
			printf("%f %f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05(), M06());
			printf("%f %f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15(), M16());
			printf("%f %f %f %f %f %f %f\n", M20(), M21(), M22(), M23(), M24(), M25(), M26());
		}

	protected:

		ENFT_SSE::__m128 m_20_21_22_23, m_24_25_26_x;
	};

	inline void SetReserve(AlignedMatrix3x7f &M) { M.reserve0() = M.reserve1() = M.reserve2() = 0; }

	inline void AddATBTo(const AlignedMatrix2x3f &A, const AlignedMatrix2x7f &B, AlignedMatrix3x7f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), to.M_00_01_02_03());
		to.M_04_05_06_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())), to.M_04_05_06_x ());

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), to.M_10_11_12_13());
		to.M_14_15_16_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())), to.M_14_15_16_x ());

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		to.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), to.M_20_21_22_23());
		to.M_24_25_26_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())), to.M_24_25_26_x ());
	}
	inline void ATB(const Matrix2x3f &A, const AlignedCompactMatrix2x7f &B, AlignedMatrix3x7f &ATB, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M04() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M05() = work2[0].m128_f32[1] + work2[0].m128_f32[3];
		ATB.M06() = A.M00() * B.M06() + A.M10() * B.M16();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M14() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M15() = work2[0].m128_f32[1] + work2[0].m128_f32[3];
		ATB.M16() = A.M01() * B.M06() + A.M11() * B.M16();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		ATB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M24() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M25() = work2[0].m128_f32[1] + work2[0].m128_f32[3];
		ATB.M26() = A.M02() * B.M06() + A.M12() * B.M16();
	}
	inline void ATB(const AlignedMatrix2x3f &A, const AlignedMatrix2x7f &B, AlignedMatrix3x7f &ATB, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		ATB.M_04_05_06_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ()));

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		ATB.M_14_15_16_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ()));

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		ATB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		ATB.M_24_25_26_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ()));
	}
	inline void ATB(const SymmetricMatrix3f &A, const AlignedMatrix3x7f &B, AlignedMatrix3x7f &ATB, ENFT_SSE::__m128 *work4)
	{
		work4[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work4[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		work4[2] = ENFT_SSE::_mm_set1_ps(A.M20());
		ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work4[1], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work4[2], B.M_20_21_22_23()));
		ATB.M_04_05_06_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work4[1], B.M_14_15_16_x ())), 
													ENFT_SSE::_mm_mul_ps(work4[2], B.M_24_25_26_x ()));
		work4[0] = ENFT_SSE::_mm_set1_ps(A.M11());
		work4[3] = ENFT_SSE::_mm_set1_ps(A.M21());
		ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[1], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work4[0], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work4[3], B.M_20_21_22_23()));
		ATB.M_14_15_16_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[1], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work4[0], B.M_14_15_16_x ())), 
													ENFT_SSE::_mm_mul_ps(work4[3], B.M_24_25_26_x ()));

		work4[0] = ENFT_SSE::_mm_set1_ps(A.M22());
		ATB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[2], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work4[3], B.M_10_11_12_13())), 
													ENFT_SSE::_mm_mul_ps(work4[0], B.M_20_21_22_23()));
		ATB.M_24_25_26_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work4[2], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work4[3], B.M_14_15_16_x ())), 
													ENFT_SSE::_mm_mul_ps(work4[0], B.M_24_25_26_x ()));
	}
	inline void AddATBTo(const AlignedMatrix3x7f &A, const Vector3f &B, AlignedVector7f &to, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work3[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		work3[2] = ENFT_SSE::_mm_set1_ps(B.v2());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work3[1])), 
								ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work3[2]), to.v0123()));
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x(), work3[1])), 
								ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_24_25_26_x(), work3[2]), to.v456x()));
	}
	inline void AddATBTo(const AlignedMatrix3x7f &A, const ENFT_SSE::__m128 &B, AlignedVector7f &to, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(B.m128_f32[0]);
		work3[1] = ENFT_SSE::_mm_set1_ps(B.m128_f32[1]);
		work3[2] = ENFT_SSE::_mm_set1_ps(B.m128_f32[2]);
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work3[1])), 
								ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work3[2]), to.v0123()));
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x(), work3[1])), 
								ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_24_25_26_x(), work3[2]), to.v456x()));
	}
	inline void SubtractATBFrom(const AlignedMatrix3x7f &A, const Vector3f &B, AlignedVector7f &from, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work3[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		work3[2] = ENFT_SSE::_mm_set1_ps(B.v2());
		from.v0123() = ENFT_SSE::_mm_sub_ps(from.v0123(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work3[1])), ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work3[2])));
		from.v456x() = ENFT_SSE::_mm_sub_ps(from.v456x(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x (), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x (), work3[1])), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_x (), work3[2])));
	}
	inline void AddAij2To(const AlignedMatrix3x7f &A, AlignedVector7f &to)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_10_11_12_13())), 
								ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), A.M_20_21_22_23()), to.v0123()));
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x (), A.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x (), A.M_14_15_16_x ())), 
								ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_24_25_26_x (), A.M_24_25_26_x ()), to.v456x()));
	}
	inline void AddABTo(const AlignedMatrix3x7f &A, const AlignedVector7f &B, Vector3f &to)
	{
#if _DEBUG
		assert(A.reserve0() == 0 && A.reserve1() == 0 && A.reserve2() == 0 || B.reserve() == 0);
#endif
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x(), B.v456x()))) + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x(), B.v456x()))) + to.v1();
		to.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_x(), B.v456x()))) + to.v2();
	}
	inline void AddABTo(const AlignedMatrix3x7f &A, const AlignedVector7f &B, Vector3f &to, ENFT_SSE::__m128 *work0) { AddABTo(A, B, to); }
	inline void SubtractABFrom(const AlignedMatrix3x7f &A, const AlignedVector7f &B, Vector3f &from)
	{
#if _DEBUG
		assert(A.reserve0() == 0 && A.reserve1() == 0 && A.reserve2() == 0 || B.reserve() == 0);
#endif
		from.v0() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x(), B.v456x())));
		from.v1() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x(), B.v456x())));
		from.v2() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_x(), B.v456x())));
	}
	inline void SubtractABFrom(const AlignedMatrix3x7f &A, const AlignedVector7f &B, Vector3f &from, ENFT_SSE::__m128 *work0) { SubtractABFrom(A, B, from); }
	
	inline void SubtractABFrom(const AlignedMatrix2x3f &A, AlignedMatrix3x7f &B, AlignedMatrix2x7f &from, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M01());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M02());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_04_05_06_x () = ENFT_SSE::_mm_sub_ps(from.M_04_05_06_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x ())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x ())));
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M10());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M12());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_14_15_16_x () = ENFT_SSE::_mm_sub_ps(from.M_14_15_16_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x ())), 
																					  ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x ())));
	}
	inline void AddATo(const AlignedMatrix3x7f &A, AlignedMatrix3x7f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(A.M_00_01_02_03(), to.M_00_01_02_03());
		to.M_04_05_06_x () = ENFT_SSE::_mm_add_ps(A.M_04_05_06_x (), to.M_04_05_06_x ());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(A.M_10_11_12_13(), to.M_10_11_12_13());
		to.M_14_15_16_x () = ENFT_SSE::_mm_add_ps(A.M_14_15_16_x (), to.M_14_15_16_x ());
		to.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(A.M_20_21_22_23(), to.M_20_21_22_23());
		to.M_24_25_26_x () = ENFT_SSE::_mm_add_ps(A.M_24_25_26_x (), to.M_24_25_26_x ());
	}
	inline void s1s2TA(const Vector3f &s1, const AlignedVector7f &s2, AlignedMatrix3x7f &A, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_set1_ps(s1.v0());
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v0123()), A.M_00_01_02_03());
		A.M_04_05_06_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v456x()), A.M_04_05_06_x ());
		work = ENFT_SSE::_mm_set1_ps(s1.v1());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v0123()), A.M_10_11_12_13());
		A.M_14_15_16_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v456x()), A.M_14_15_16_x ());
		work = ENFT_SSE::_mm_set1_ps(s1.v2());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v0123()), A.M_20_21_22_23());
		A.M_24_25_26_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v456x()), A.M_24_25_26_x ());
	}

	class AlignedCompactMatrix3x7f : public AlignedCompactMatrix2x7f
	{

	public:

		inline const ENFT_SSE::__m128& M_20_21_22_23() const { return m_20_21_22_23; }	inline ENFT_SSE::__m128& M_20_21_22_23() { return m_20_21_22_23; }
		inline const ENFT_SSE::__m128& M_24_25_26_x() const { return m_24_25_26_x; }		inline ENFT_SSE::__m128& M_24_25_26_x() { return m_24_25_26_x; }
		inline const float& M20() const { return m_20_21_22_23.m128_f32[0]; }	inline float& M20() { return m_20_21_22_23.m128_f32[0]; }
		inline const float& M21() const { return m_20_21_22_23.m128_f32[1]; }	inline float& M21() { return m_20_21_22_23.m128_f32[1]; }
		inline const float& M22() const { return m_20_21_22_23.m128_f32[2]; }	inline float& M22() { return m_20_21_22_23.m128_f32[2]; }
		inline const float& M23() const { return m_20_21_22_23.m128_f32[3]; }	inline float& M23() { return m_20_21_22_23.m128_f32[3]; }
		inline const float& M24() const { return m_24_25_26_x.m128_f32[0]; }	inline float& M24() { return m_24_25_26_x.m128_f32[0]; }
		inline const float& M25() const { return m_24_25_26_x.m128_f32[1]; }	inline float& M25() { return m_24_25_26_x.m128_f32[1]; }
		inline const float& M26() const { return m_24_25_26_x.m128_f32[2]; }	inline float& M26() { return m_24_25_26_x.m128_f32[2]; }

		inline void Print() const
		{
			printf("%f %f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05(), M06());
			printf("%f %f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15(), M16());
			printf("%f %f %f %f %f %f %f\n", M20(), M21(), M22(), M23(), M24(), M25(), M26());
		}

	protected:

		ENFT_SSE::__m128 m_20_21_22_23, m_24_25_26_x;
	};

	inline void ATB(const Matrix2x3f &A, const AlignedCompactMatrix2x7f &B, AlignedCompactMatrix3x7f &ATB, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M04() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M05() = work2[0].m128_f32[1] + work2[0].m128_f32[3];
		ATB.M06() = A.M00() * B.M06() + A.M10() * B.M16();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M14() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M15() = work2[0].m128_f32[1] + work2[0].m128_f32[3];
		ATB.M16() = A.M01() * B.M06() + A.M11() * B.M16();

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		ATB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		work2[0] = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), B.M_04_05_14_15());
		ATB.M24() = work2[0].m128_f32[0] + work2[0].m128_f32[2];
		ATB.M25() = work2[0].m128_f32[1] + work2[0].m128_f32[3];
		ATB.M26() = A.M02() * B.M06() + A.M12() * B.M16();
	}
	inline void AddATBTo(const AlignedCompactMatrix3x7f &A, const Vector3f &B, AlignedVector7f &to, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work3[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		work3[2] = ENFT_SSE::_mm_set1_ps(B.v2());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work3[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work3[1])), 
								ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work3[2]), to.v0123()));
		work3[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(work3[0], work3[1]));
		work3[1] = ENFT_SSE::_mm_mul_ps(A.M_24_25_26_x(), work3[2]);
		to.v4() = work3[0].m128_f32[0] + work3[0].m128_f32[2] + work3[1].m128_f32[0] + to.v4();
		to.v5() = work3[0].m128_f32[1] + work3[0].m128_f32[3] + work3[1].m128_f32[1] + to.v5();
		to.v6() = A.M06() * B.v0() + A.M16() * B.v1()		  + work3[1].m128_f32[2] + to.v6();
	}
}

#endif
