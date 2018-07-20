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

#ifndef _MATRIX_2x7_H_
#define _MATRIX_2x7_H_

#include "Utility/SSE.h"
#include "Matrix2x6.h"
#include "Vector7.h"

namespace LA
{

	class AlignedMatrix2x7f
	{

	public:

		inline const ENFT_SSE::__m128& M_00_01_02_03() const { return m_00_01_02_03; }		inline ENFT_SSE::__m128& M_00_01_02_03() { return m_00_01_02_03; }
		inline const ENFT_SSE::__m128& M_04_05_06_x () const { return m_04_05_06_x; }			inline ENFT_SSE::__m128& M_04_05_06_x () { return m_04_05_06_x; }
		inline const ENFT_SSE::__m128& M_10_11_12_13() const { return m_10_11_12_13; }		inline ENFT_SSE::__m128& M_10_11_12_13() { return m_10_11_12_13; }
		inline const ENFT_SSE::__m128& M_14_15_16_x () const { return m_14_15_16_x; }			inline ENFT_SSE::__m128& M_14_15_16_x () { return m_14_15_16_x; }
		inline const float& M00() const { return m_00_01_02_03.m128_f32[0]; }		inline float& M00() { return m_00_01_02_03.m128_f32[0]; }
		inline const float& M01() const { return m_00_01_02_03.m128_f32[1]; }		inline float& M01() { return m_00_01_02_03.m128_f32[1]; }
		inline const float& M02() const { return m_00_01_02_03.m128_f32[2]; }		inline float& M02() { return m_00_01_02_03.m128_f32[2]; }
		inline const float& M03() const { return m_00_01_02_03.m128_f32[3]; }		inline float& M03() { return m_00_01_02_03.m128_f32[3]; }
		inline const float& M04() const { return m_04_05_06_x.m128_f32[0]; }		inline float& M04() { return m_04_05_06_x.m128_f32[0]; }
		inline const float& M05() const { return m_04_05_06_x.m128_f32[1]; }		inline float& M05() { return m_04_05_06_x.m128_f32[1]; }
		inline const float& M06() const { return m_04_05_06_x.m128_f32[2]; }		inline float& M06() { return m_04_05_06_x.m128_f32[2]; }
		inline const float& M10() const { return m_10_11_12_13.m128_f32[0]; }		inline float& M10() { return m_10_11_12_13.m128_f32[0]; }
		inline const float& M11() const { return m_10_11_12_13.m128_f32[1]; }		inline float& M11() { return m_10_11_12_13.m128_f32[1]; }
		inline const float& M12() const { return m_10_11_12_13.m128_f32[2]; }		inline float& M12() { return m_10_11_12_13.m128_f32[2]; }
		inline const float& M13() const { return m_10_11_12_13.m128_f32[3]; }		inline float& M13() { return m_10_11_12_13.m128_f32[3]; }
		inline const float& M14() const { return m_14_15_16_x.m128_f32[0]; }		inline float& M14() { return m_14_15_16_x.m128_f32[0]; }
		inline const float& M15() const { return m_14_15_16_x.m128_f32[1]; }		inline float& M15() { return m_14_15_16_x.m128_f32[1]; }
		inline const float& M16() const { return m_14_15_16_x.m128_f32[2]; }		inline float& M16() { return m_14_15_16_x.m128_f32[2]; }
		inline operator const float* () const { return (const float *) this; }		inline operator float* () { return (float *) this; }
		inline const float& reserve0() const { return m_04_05_06_x.m128_f32[3]; }	inline float& reserve0() { return m_04_05_06_x.m128_f32[3]; }
		inline const float& reserve1() const { return m_14_15_16_x.m128_f32[3]; }	inline float& reserve1() { return m_14_15_16_x.m128_f32[3]; }
		inline void Print() const
		{
			printf("%f %f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05(), M06());
			printf("%f %f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15(), M16());
		}

	protected:

		ENFT_SSE::__m128 m_00_01_02_03, m_04_05_06_x;
		ENFT_SSE::__m128 m_10_11_12_13, m_14_15_16_x;

	};

	inline void SetReserve(AlignedMatrix2x7f &M) { M.reserve0() = M.reserve1() = 0; }

	inline void AddAij2To(const AlignedMatrix2x7f &A, AlignedVector7f &to)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_10_11_12_13())), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x (), A.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x (), A.M_14_15_16_x ())), to.v456x());
	}

	inline void AddATBTo(const AlignedMatrix2x7f &A, const Vector2f &B, AlignedVector7f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());		work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x (), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x (), work2[1])), to.v456x());
	}
	template<> inline void FinishAdditionATBTo<AlignedMatrix2x7f>(AlignedVector7f &to) { to.reserve() = 0.0f; }
	inline void SubtractATBFrom(const AlignedMatrix2x7f &A, const Vector2f &B, AlignedVector7f &from, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());		work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		from.v0123() = ENFT_SSE::_mm_sub_ps(from.v0123(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])));
		from.v456x() = ENFT_SSE::_mm_sub_ps(from.v456x(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x (), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x (), work2[1])));
	}
	inline void AddATBTo(const AlignedMatrix2f &A, const AlignedMatrix2x7f &B, AlignedMatrix2x7f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), to.M_00_01_02_03());
		to.M_04_05_06_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())), to.M_04_05_06_x ());
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())), to.M_10_11_12_13());
		to.M_14_15_16_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())), to.M_14_15_16_x ());
	}
	inline void ATB(const AlignedMatrix2f &A, const AlignedMatrix2x7f &B, AlignedMatrix2x7f &ATB, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		ATB.M_04_05_06_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ()));
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13()));
		ATB.M_14_15_16_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ()));
	}
	inline void s1s2TA(const Vector2f &s1, const AlignedVector7f &s2, AlignedMatrix2x7f &A, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_set1_ps(s1.v0());
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v0123()), A.M_00_01_02_03());
		A.M_04_05_06_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v456x()), A.M_04_05_06_x ());
		work = ENFT_SSE::_mm_set1_ps(s1.v1());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v0123()), A.M_10_11_12_13());
		A.M_14_15_16_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s2.v456x()), A.M_14_15_16_x ());
	}
	inline void AddABTo(const AlignedMatrix2x7f &A, const AlignedVector7f &B, Vector2f &to, ENFT_SSE::__m128 *work0)
	{
#if _DEBUG
		assert(B.reserve() == 0);
#endif
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x(), B.v456x()))) + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x(), B.v456x()))) + to.v1();
	}
	inline void SubtractABFrom(const AlignedMatrix2x7f &A, const AlignedVector7f &B, Vector2f &from, ENFT_SSE::__m128 *work0)
	{
#if _DEBUG
		assert(B.reserve() == 0);
#endif
		from.v0() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x(), B.v456x())));
		from.v1() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x(), B.v456x())));
	}

	class AlignedCompactMatrix2x7f : public AlignedCompactMatrix2x6f
	{

	public:

		inline const ENFT_SSE::__m128& M_06_16_x_x() const { return m_06_16_x_x; }		inline ENFT_SSE::__m128& M_06_16_x_x() { return m_06_16_x_x; }
		inline const float& M06() const { return m_06_16_x_x.m128_f32[0]; }		inline float& M06() { return m_06_16_x_x.m128_f32[0]; }
		inline const float& M16() const { return m_06_16_x_x.m128_f32[1]; }		inline float& M16() { return m_06_16_x_x.m128_f32[1]; }

		inline void Print() const
		{
			printf("%f %f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05(), M06());
			printf("%f %f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15(), M16());
		}

	protected:

		ENFT_SSE::__m128 m_06_16_x_x;
	};

	inline void AddAij2To(const AlignedCompactMatrix2x7f &A, AlignedVector7f &to, ENFT_SSE::__m128 *work1)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_10_11_12_13())), to.v0123());
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), A.M_04_05_14_15());
		to.v4() = work1[0].m128_f32[0] + work1[0].m128_f32[2] + to.v4();
		to.v5() = work1[0].m128_f32[1] + work1[0].m128_f32[3] + to.v5();
		to.v6() = A.M06() * A.M06() + A.M16() * A.M16() + to.v6();
	}

	inline void sA(const AlignedVector7f &s, AlignedCompactMatrix2x7f &A)
	{
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.M_00_01_02_03());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.M_10_11_12_13());
		A.M_04_05_14_15() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(s.v456x(), s.v456x()), A.M_04_05_14_15());
		A.M06() *= s.v6();
		A.M16() *= s.v6();
	}

	inline void AddATBTo(const AlignedCompactMatrix2x7f &A, const Vector2f &B, AlignedVector7f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])), to.v0123());
		work2[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]));
		to.v4() = work2[0].m128_f32[0] + work2[0].m128_f32[2] + to.v4();
		to.v5() = work2[0].m128_f32[1] + work2[0].m128_f32[3] + to.v5();
		to.v6() = A.M06() * B.v0() + A.M16() * B.v1() + to.v6();
	}
	template<> inline void FinishAdditionATBTo<AlignedCompactMatrix2x7f>(AlignedVector7f &to) {}
	inline void SubtractATBFrom(const AlignedCompactMatrix2x7f &A, const Vector2f &B, AlignedVector7f &from, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		from.v0123() = ENFT_SSE::_mm_sub_ps(from.v0123(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])));
		work2[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]));
		from.v4() -= work2[0].m128_f32[0] + work2[0].m128_f32[2];
		from.v5() -= work2[0].m128_f32[1] + work2[0].m128_f32[3];
		from.v6() -= A.M06() * B.v0() + A.M16() * B.v1();
	}

	inline void AB(const AlignedCompactMatrix2x7f &A, const AlignedVector7f &B, Vector2f &AB, ENFT_SSE::__m128 *work1)
	{
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(B.v456x(), B.v456x()));
		AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + A.M06() * B.v6();
		AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + A.M16() * B.v6();
	}
	inline void SubtractABFrom(const AlignedCompactMatrix2x7f &A, const AlignedVector7f &B, Vector2f &from, ENFT_SSE::__m128 *work1)
	{
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(B.v456x(), B.v456x()));
		from.v0() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + A.M06() * B.v6();
		from.v1() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + A.M16() * B.v6();
	}
	template<typename TYPE>
	inline void ABpCD(const AlignedCompactMatrix2x7f &A, const AlignedVector7f &B, const Matrix2x3<TYPE> &C, const Vector3<TYPE> &D, Vector2f &ABpCD, ENFT_SSE::__m128 *work1)
	{
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(B.v456x(), B.v456x()));
		ABpCD.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + A.M06() * B.v6() + C.M00() * D.v0() + C.M01() * D.v1() + C.M02() * D.v2();
		ABpCD.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + A.M16() * B.v6() + C.M10() * D.v0() + C.M11() * D.v1() + C.M12() * D.v2();
	}
}

#endif