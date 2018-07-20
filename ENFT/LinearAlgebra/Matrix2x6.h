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

#ifndef _MATRIX_2x6_H_
#define _MATRIX_2x6_H_

#include "Utility/SSE.h"
#include "Matrix2x3.h"
#include "Vector6.h"

namespace LA
{

	class AlignedCompactMatrix2x6f
	{

	public:

		inline const ENFT_SSE::__m128& M_00_01_02_03() const { return m_00_01_02_03; }	inline ENFT_SSE::__m128& M_00_01_02_03() { return m_00_01_02_03; }
		inline const ENFT_SSE::__m128& M_04_05_14_15() const { return m_04_05_14_15; }	inline ENFT_SSE::__m128& M_04_05_14_15() { return m_04_05_14_15; }
		inline const ENFT_SSE::__m128& M_10_11_12_13() const { return m_10_11_12_13; }	inline ENFT_SSE::__m128& M_10_11_12_13() { return m_10_11_12_13; }
		inline const float& M00() const { return m_00_01_02_03.m128_f32[0]; }	inline float& M00() { return m_00_01_02_03.m128_f32[0]; }
		inline const float& M01() const { return m_00_01_02_03.m128_f32[1]; }	inline float& M01() { return m_00_01_02_03.m128_f32[1]; }
		inline const float& M02() const { return m_00_01_02_03.m128_f32[2]; }	inline float& M02() { return m_00_01_02_03.m128_f32[2]; }
		inline const float& M03() const { return m_00_01_02_03.m128_f32[3]; }	inline float& M03() { return m_00_01_02_03.m128_f32[3]; }
		inline const float& M04() const { return m_04_05_14_15.m128_f32[0]; }	inline float& M04() { return m_04_05_14_15.m128_f32[0]; }
		inline const float& M05() const { return m_04_05_14_15.m128_f32[1]; }	inline float& M05() { return m_04_05_14_15.m128_f32[1]; }
		inline const float& M10() const { return m_10_11_12_13.m128_f32[0]; }	inline float& M10() { return m_10_11_12_13.m128_f32[0]; }
		inline const float& M11() const { return m_10_11_12_13.m128_f32[1]; }	inline float& M11() { return m_10_11_12_13.m128_f32[1]; }
		inline const float& M12() const { return m_10_11_12_13.m128_f32[2]; }	inline float& M12() { return m_10_11_12_13.m128_f32[2]; }
		inline const float& M13() const { return m_10_11_12_13.m128_f32[3]; }	inline float& M13() { return m_10_11_12_13.m128_f32[3]; }
		inline const float& M14() const { return m_04_05_14_15.m128_f32[2]; }	inline float& M14() { return m_04_05_14_15.m128_f32[2]; }
		inline const float& M15() const { return m_04_05_14_15.m128_f32[3]; }	inline float& M15() { return m_04_05_14_15.m128_f32[3]; }
		inline operator const float* () const { return (const float *) this; }	inline operator float* () { return (float *) this; }
		inline void SetZero() { memset(this, 0, sizeof(AlignedCompactMatrix2x6f)); }
		inline void Scale(const ENFT_SSE::__m128 &s)
		{
			m_00_01_02_03 = ENFT_SSE::_mm_mul_ps(s, m_00_01_02_03);
			m_04_05_14_15 = ENFT_SSE::_mm_mul_ps(s, m_04_05_14_15);
			m_10_11_12_13 = ENFT_SSE::_mm_mul_ps(s, m_10_11_12_13);
		}
		inline void ConvertToConventionalStorage(float *work2)
		{
			memcpy(work2, &M14(), 8);
			memcpy(&M14(), &M10(), 16);
			memcpy(&M12(), work2, 8);
		}
		inline void ConvertToConventionalStorage(float* M) const
		{
			memcpy(M, &M00(), 24);
			memcpy(M + 6, &M10(), 16);
			memcpy(M + 10, &M14(), 8);
		}
		inline void ConvertToSpecialStorage(float *work2)
		{
			memcpy(work2, &M12(), 8);
			memcpy(&M12(), &M10(), 8);
			memcpy(&M10(), &M14(), 8);
			memcpy(&M14(), work2, 8);
		}
		inline void Print() const
		{
			printf("%f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05());
			printf("%f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15());
		}

	protected:

		ENFT_SSE::__m128 m_00_01_02_03, m_04_05_14_15, m_10_11_12_13;

	};

	inline void AddAij2To(const AlignedCompactMatrix2x6f &A, AlignedVector6f &to)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), A.M_10_11_12_13())), to.v0123());
		to.v45xx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), A.M_04_05_14_15()), to.v45xx());
	}
	inline void AddAij2To(const AlignedCompactMatrix2x6f &A, AlignedVector6f &to, ENFT_SSE::__m128 *work0) { AddAij2To(A, to); }

	inline void sA(const AlignedVector6f &s, AlignedCompactMatrix2x6f &A)
	{
#if _DEBUG
		assert(s.reserve0() == s.v4() && s.reserve1() == s.v5());
#endif
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.M_00_01_02_03());
		A.M_04_05_14_15() = ENFT_SSE::_mm_mul_ps(s.v45xx(), A.M_04_05_14_15());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(s.v0123(), A.M_10_11_12_13());
	}
	inline void s1s2TA(const Vector2f &s1, const AlignedVector6f &s2, AlignedCompactMatrix2x6f &A, ENFT_SSE::__m128 *work2)
	{
#if _DEBUG
		assert(s2.reserve0() == s2.v4() && s2.reserve1() == s2.v5());
#endif
		work2[0] = ENFT_SSE::_mm_set1_ps(s1.v0());
		work2[1] = ENFT_SSE::_mm_set1_ps(s1.v1());
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work2[0], s2.v0123()), A.M_00_01_02_03());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work2[1], s2.v0123()), A.M_10_11_12_13());
		A.M_04_05_14_15() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]), s2.v45xx()), A.M_04_05_14_15());
	}

	inline void AddATBTo(const AlignedCompactMatrix2x6f &A, const Vector2f &B, AlignedVector6f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])), to.v0123());
		to.v45xx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(work2[0], work2[1])), to.v45xx());
	}
	inline void SubtractATBFrom(const AlignedCompactMatrix2x6f &A, const Vector2f &B, AlignedVector6f &from, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
		from.v0123() = ENFT_SSE::_mm_sub_ps(from.v0123(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])));
		work2[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]));
		from.v4() -= work2[0].m128_f32[0] + work2[0].m128_f32[2];
		from.v5() -= work2[0].m128_f32[1] + work2[0].m128_f32[3];
	}
	inline void AddATBTo(const AlignedMatrix2f &A, const AlignedCompactMatrix2x6f &B, AlignedCompactMatrix2x6f &to, ENFT_SSE::__m128 *work2)
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
	}
	inline void ATB(const AlignedMatrix2f &A, const AlignedCompactMatrix2x6f &B, AlignedCompactMatrix2x6f &ATB, ENFT_SSE::__m128 *work2)
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
	}

	inline void AB(const AlignedCompactMatrix2x6f &A, const AlignedVector6f &B, Vector2f &AB, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1];
		AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3];
	}
	inline void AddABTo(const AlignedCompactMatrix2x6f &A, const AlignedVector6f &B, Vector2f &to, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + to.v1();
	}
	inline void SubtractABFrom(const AlignedCompactMatrix2x6f &A, const AlignedVector6f &B, Vector2f &from, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		from.v0() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1];
		from.v1() -= ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3];
	}
	inline void ABpCD(const AlignedCompactMatrix2x6f &A, const AlignedVector6f &B, const AlignedMatrix2x3f &C, const AlignedVector3f &D, Vector2f &ABpCD, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		ABpCD.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(C.M_00_01_02_x(), D.v012x()));
		ABpCD.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(C.M_10_11_12_x(), D.v012x()));
	}
	template<typename TYPE>
	inline void ABpCD(const AlignedCompactMatrix2x6f &A, const AlignedVector6f &B, const Matrix2x3<TYPE> &C, const Vector3<TYPE> &D, Vector2f &ABpCD, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		ABpCD.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + C.M00() * D.v0() + C.M01() * D.v1() + C.M02() * D.v2();
		ABpCD.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + C.M10() * D.v0() + C.M11() * D.v1() + C.M12() * D.v2();
	}
	template<typename TYPE>
	inline void ABpCD(const AlignedCompactMatrix2x6f &A, const AlignedVector6f &B, const AlignedMatrix2x3f &C, const Vector3<TYPE> &D, Vector2f &ABpCD, ENFT_SSE::__m128 *work1)
	{
#if _DEBUG
		B.AssertReserve45();
#endif
		work1[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), B.v45xx());
		ABpCD.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work1[0].m128_f32[0] + work1[0].m128_f32[1] + C.M00() * D.v0() + C.M01() * D.v1() + C.M02() * D.v2();
		ABpCD.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work1[0].m128_f32[2] + work1[0].m128_f32[3] + C.M10() * D.v0() + C.M11() * D.v1() + C.M12() * D.v2();
	}

	class AlignedMatrix2x6f
	{

	public:

		inline const ENFT_SSE::__m128& M_00_01_02_03() const { return m_00_01_02_03; }		inline ENFT_SSE::__m128& M_00_01_02_03() { return m_00_01_02_03; }
		inline const ENFT_SSE::__m128& M_04_05_x_x  () const { return m_04_05_x_x; }			inline ENFT_SSE::__m128& M_04_05_x_x  () { return m_04_05_x_x; }
		inline const ENFT_SSE::__m128& M_10_11_12_13() const { return m_10_11_12_13; }		inline ENFT_SSE::__m128& M_10_11_12_13() { return m_10_11_12_13; }
		inline const ENFT_SSE::__m128& M_14_15_x_x  () const { return m_14_15_x_x; }			inline ENFT_SSE::__m128& M_14_15_x_x () { return m_14_15_x_x; }
		inline const float& M00() const { return m_00_01_02_03.m128_f32[0]; }		inline float& M00() { return m_00_01_02_03.m128_f32[0]; }
		inline const float& M01() const { return m_00_01_02_03.m128_f32[1]; }		inline float& M01() { return m_00_01_02_03.m128_f32[1]; }
		inline const float& M02() const { return m_00_01_02_03.m128_f32[2]; }		inline float& M02() { return m_00_01_02_03.m128_f32[2]; }
		inline const float& M03() const { return m_00_01_02_03.m128_f32[3]; }		inline float& M03() { return m_00_01_02_03.m128_f32[3]; }
		inline const float& M04() const { return m_04_05_x_x.m128_f32[0]; }			inline float& M04() { return m_04_05_x_x.m128_f32[0]; }
		inline const float& M05() const { return m_04_05_x_x.m128_f32[1]; }			inline float& M05() { return m_04_05_x_x.m128_f32[1]; }
		inline const float& M10() const { return m_10_11_12_13.m128_f32[0]; }		inline float& M10() { return m_10_11_12_13.m128_f32[0]; }
		inline const float& M11() const { return m_10_11_12_13.m128_f32[1]; }		inline float& M11() { return m_10_11_12_13.m128_f32[1]; }
		inline const float& M12() const { return m_10_11_12_13.m128_f32[2]; }		inline float& M12() { return m_10_11_12_13.m128_f32[2]; }
		inline const float& M13() const { return m_10_11_12_13.m128_f32[3]; }		inline float& M13() { return m_10_11_12_13.m128_f32[3]; }
		inline const float& M14() const { return m_14_15_x_x.m128_f32[0]; }			inline float& M14() { return m_14_15_x_x.m128_f32[0]; }
		inline const float& M15() const { return m_14_15_x_x.m128_f32[1]; }			inline float& M15() { return m_14_15_x_x.m128_f32[1]; }
		inline operator const float* () const { return (const float *) this; }		inline operator float* () { return (float *) this; }
		inline void Print() const
		{
			printf("%f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05());
			printf("%f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15());
		}

	protected:

		ENFT_SSE::__m128 m_00_01_02_03, m_04_05_x_x;
		ENFT_SSE::__m128 m_10_11_12_13, m_14_15_x_x;

	};
}

#endif