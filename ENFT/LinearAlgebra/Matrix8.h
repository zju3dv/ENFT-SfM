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

#ifndef _MATRIX_8_H_
#define _MATRIX_8_H_

#include "Matrix6x8.h"

namespace LA
{

	class AlignedMatrix8f : public AlignedMatrix6x8f
	{

	public:

		inline const ENFT_SSE::__m128& M_60_61_62_63() const { return m_60_61_62_63; }		inline ENFT_SSE::__m128& M_60_61_62_63() { return m_60_61_62_63; }
		inline const ENFT_SSE::__m128& M_64_65_66_67() const { return m_64_65_66_67; }		inline ENFT_SSE::__m128& M_64_65_66_67() { return m_64_65_66_67; }
		inline const ENFT_SSE::__m128& M_70_71_72_73() const { return m_70_71_72_73; }		inline ENFT_SSE::__m128& M_70_71_72_73() { return m_70_71_72_73; }
		inline const ENFT_SSE::__m128& M_74_75_76_77() const { return m_74_75_76_77; }		inline ENFT_SSE::__m128& M_74_75_76_77() { return m_74_75_76_77; }
		inline const float& M60() const { return m_60_61_62_63.m128_f32[0]; }		inline float& M60() { return m_60_61_62_63.m128_f32[0]; }
		inline const float& M61() const { return m_60_61_62_63.m128_f32[1]; }		inline float& M61() { return m_60_61_62_63.m128_f32[1]; }
		inline const float& M62() const { return m_60_61_62_63.m128_f32[2]; }		inline float& M62() { return m_60_61_62_63.m128_f32[2]; }
		inline const float& M63() const { return m_60_61_62_63.m128_f32[3]; }		inline float& M63() { return m_60_61_62_63.m128_f32[3]; }
		inline const float& M64() const { return m_64_65_66_67.m128_f32[0]; }		inline float& M64() { return m_64_65_66_67.m128_f32[0]; }
		inline const float& M65() const { return m_64_65_66_67.m128_f32[1]; }		inline float& M65() { return m_64_65_66_67.m128_f32[1]; }
		inline const float& M66() const { return m_64_65_66_67.m128_f32[2]; }		inline float& M66() { return m_64_65_66_67.m128_f32[2]; }
		inline const float& M67() const { return m_64_65_66_67.m128_f32[3]; }		inline float& M67() { return m_64_65_66_67.m128_f32[3]; }
		inline const float& M70() const { return m_70_71_72_73.m128_f32[0]; }		inline float& M70() { return m_70_71_72_73.m128_f32[0]; }
		inline const float& M71() const { return m_70_71_72_73.m128_f32[1]; }		inline float& M71() { return m_70_71_72_73.m128_f32[1]; }
		inline const float& M72() const { return m_70_71_72_73.m128_f32[2]; }		inline float& M72() { return m_70_71_72_73.m128_f32[2]; }
		inline const float& M73() const { return m_70_71_72_73.m128_f32[3]; }		inline float& M73() { return m_70_71_72_73.m128_f32[3]; }
		inline const float& M74() const { return m_74_75_76_77.m128_f32[0]; }		inline float& M74() { return m_74_75_76_77.m128_f32[0]; }
		inline const float& M75() const { return m_74_75_76_77.m128_f32[1]; }		inline float& M75() { return m_74_75_76_77.m128_f32[1]; }
		inline const float& M76() const { return m_74_75_76_77.m128_f32[2]; }		inline float& M76() { return m_74_75_76_77.m128_f32[2]; }
		inline const float& M77() const { return m_74_75_76_77.m128_f32[3]; }		inline float& M77() { return m_74_75_76_77.m128_f32[3]; }

		inline void SetZero() { memset(this, 0, sizeof(AlignedMatrix8f)); }
		inline void MakeIdentity()
		{
			SetZero();
			M00() = M11() = M22() = M33() = M44() = M55() = M66() = M77() = 1.0f;
		}
		inline void GetDiagonal(AlignedVector8f &d) const
		{
			d.v0() = M00();		d.v1() = M11();		d.v2() = M22();		d.v3() = M33();		d.v4() = M44();		d.v5() = M55();		d.v6() = M66();		d.v7() = M77();
		}
		inline void SetDiagonal(const AlignedVector8f &d)
		{
			M00() = d.v0();		M11() = d.v1();		M22() = d.v2();		M33() = d.v3();		M44() = d.v4();		M55() = d.v5();		M66() = d.v6();		M77() = d.v7();
		}
		inline void ScaleDiagonal(const float &lambda)
		{
			M00() *= lambda;	M11() *= lambda;	M22() *= lambda;	M33() *= lambda;	M44() *= lambda;	M55() *= lambda;	M66() *= lambda;	M77() *= lambda;
		}
		inline void IncreaseDiagonal(const float &lambda)
		{
			M00() += lambda;	M11() += lambda;	M22() += lambda;	M33() += lambda;	M44() += lambda;	M55() += lambda;	M66() += lambda;	M77() += lambda;
		}
		inline void SetLowerFromUpper()
		{
			M10() = M01();
			M20() = M02();	M21() = M12();
			M30() = M03();	M31() = M13();	M32() = M23();
			M40() = M04();	M41() = M14();	M42() = M24();	M43() = M34();
			M50() = M05();	M51() = M15();	M52() = M25();	M53() = M35();	M54() = M45();
			M60() = M06();	M61() = M16();	M62() = M26();	M63() = M36();	M64() = M46();	M65() = M56();
			M70() = M07();	M71() = M17();	M72() = M27();	M73() = M37();	M74() = M47();	M75() = M57();	M76() = M67();
		}
		inline void Print() const
		{
			printf("%f %f %f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05(), M06(), M07());
			printf("%f %f %f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15(), M16(), M17());
			printf("%f %f %f %f %f %f %f %f\n", M20(), M21(), M22(), M23(), M24(), M25(), M26(), M27());
			printf("%f %f %f %f %f %f %f %f\n", M30(), M31(), M32(), M33(), M34(), M35(), M36(), M37());
			printf("%f %f %f %f %f %f %f %f\n", M40(), M41(), M42(), M43(), M44(), M45(), M46(), M47());
			printf("%f %f %f %f %f %f %f %f\n", M50(), M51(), M52(), M53(), M54(), M55(), M56(), M57());
			printf("%f %f %f %f %f %f %f %f\n", M60(), M61(), M62(), M63(), M64(), M65(), M66(), M67());
			printf("%f %f %f %f %f %f %f %f\n", M70(), M71(), M72(), M73(), M74(), M75(), M76(), M77());
		}

	protected:

		ENFT_SSE::__m128 m_60_61_62_63, m_64_65_66_67;
		ENFT_SSE::__m128 m_70_71_72_73, m_74_75_76_77;

	};

	inline void AddATAToUpper(const AlignedMatrix2x8f &A, AlignedMatrix8f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], A.M_10_11_12_13())), to.M_00_01_02_03());
		to.M_04_05_06_07() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], A.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], A.M_14_15_16_17())), to.M_04_05_06_07());
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], A.M_10_11_12_13())), to.M_10_11_12_13());
		to.M_14_15_16_17() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], A.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], A.M_14_15_16_17())), to.M_14_15_16_17());
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + to.M22();	to.M23() = A.M02() * A.M03() + A.M12() * A.M13() + to.M23();
		to.M_24_25_26_27() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], A.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], A.M_14_15_16_17())), to.M_24_25_26_27());
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M03());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M13());
		to.M33() = A.M03() * A.M03() + A.M13() * A.M13() + to.M33();
		to.M_34_35_36_37() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], A.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], A.M_14_15_16_17())), to.M_34_35_36_37());

		to.M_44_45_46_47() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M04()), A.M_04_05_06_07()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M14()), A.M_14_15_16_17())), to.M_44_45_46_47());
		to.M_54_55_56_57() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M05()), A.M_04_05_06_07()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M15()), A.M_14_15_16_17())), to.M_54_55_56_57());
		to.M66() = A.M06() * A.M06() + A.M16() * A.M16() + to.M66();	to.M67() = A.M06() * A.M07() + A.M16() * A.M17() + to.M67();
		to.M77() = A.M07() * A.M07() + A.M17() * A.M17() + to.M77();
	}
	template<class MATRIX> inline void FinishAdditionATAToUpper(AlignedMatrix8f &to) {}
	inline void SetLowerFromUpper(AlignedMatrix8f &A) { A.SetLowerFromUpper(); }
	inline void GetDiagonal(const AlignedMatrix8f &M, AlignedVector8f &d) { M.GetDiagonal(d); }
	inline void SetDiagonal(const AlignedVector8f &d, AlignedMatrix8f &M) { M.SetDiagonal(d); }
	inline void ScaleDiagonal(const float &lambda, AlignedMatrix8f &M) { M.ScaleDiagonal(lambda); }
	inline void IncreaseDiagonal(const float &lambda, AlignedMatrix8f &M) { M.IncreaseDiagonal(lambda); }

	inline void AB(const AlignedMatrix8f &A, const AlignedVector8f &B, AlignedVector8f &AB)
	{
		AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), B.v4567())));
		AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), B.v4567())));
		AB.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), B.v4567())));
		AB.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_34_35_36_37(), B.v4567())));
		AB.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_44_45_46_47(), B.v4567())));
		AB.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_54_55_56_57(), B.v4567())));
		AB.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_64_65_66_67(), B.v4567())));
		AB.v7() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_70_71_72_73(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_74_75_76_77(), B.v4567())));
	}
	inline void AB(const AlignedMatrix8f &A, const AlignedVector8f &B, AlignedVector8f &AB, ENFT_SSE::__m128 *work0) { LA::AB(A, B, AB); }
	inline void ABmC(const AlignedMatrix8f &A, const AlignedVector8f &B, const AlignedVector8f &C, AlignedVector8f &ABmC, ENFT_SSE::__m128 *work1)
	{
		work1[0].m128_f32[0] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), B.v4567())));
		work1[0].m128_f32[1] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), B.v4567())));
		work1[0].m128_f32[2] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), B.v4567())));
		work1[0].m128_f32[3] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_34_35_36_37(), B.v4567())));
		ABmC.v0123() = ENFT_SSE::_mm_sub_ps(work1[0], C.v0123());
		work1[0].m128_f32[0] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_44_45_46_47(), B.v4567())));
		work1[0].m128_f32[1] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_54_55_56_57(), B.v4567())));
		work1[0].m128_f32[2] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_64_65_66_67(), B.v4567())));
		work1[0].m128_f32[3] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_70_71_72_73(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_74_75_76_77(), B.v4567())));
		ABmC.v4567() = ENFT_SSE::_mm_sub_ps(work1[0], C.v4567());
	}
	inline void AddABTo(const LA::AlignedMatrix8f &A, const LA::AlignedVector8f &B, LA::AlignedVector8f &to, ENFT_SSE::__m128 *work1)
	{
		work1[0].m128_f32[0] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), B.v4567())));
		work1[0].m128_f32[1] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), B.v4567())));
		work1[0].m128_f32[2] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), B.v4567())));
		work1[0].m128_f32[3] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_34_35_36_37(), B.v4567())));
		to.v0123() = ENFT_SSE::_mm_add_ps(work1[0], to.v0123());
		work1[0].m128_f32[0] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_44_45_46_47(), B.v4567())));
		work1[0].m128_f32[1] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_54_55_56_57(), B.v4567())));
		work1[0].m128_f32[2] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_64_65_66_67(), B.v4567())));
		work1[0].m128_f32[3] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_70_71_72_73(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_74_75_76_77(), B.v4567())));
		to.v4567() = ENFT_SSE::_mm_add_ps(work1[0], to.v4567());
	}
	inline void AddATBTo(const LA::AlignedMatrix8f &A, const LA::AlignedVector8f &B, LA::AlignedVector8f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]);
		to.v0() = work2[1].m128_f32[0] + to.v0();
		to.v1() = work2[1].m128_f32[1] + to.v1();
		to.v2() = work2[1].m128_f32[2] + to.v2();
		to.v3() = work2[1].m128_f32[3] + to.v3();
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), work2[0]);
		to.v4() = work2[1].m128_f32[0] + to.v4();
		to.v5() = work2[1].m128_f32[1] + to.v5();
		to.v6() = work2[1].m128_f32[2] + to.v6();
		to.v7() = work2[1].m128_f32[3] + to.v7();

		work2[0] = ENFT_SSE::_mm_set1_ps(B.v1());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[0]);
		to.v0() = work2[1].m128_f32[0] + to.v0();
		to.v1() = work2[1].m128_f32[1] + to.v1();
		to.v2() = work2[1].m128_f32[2] + to.v2();
		to.v3() = work2[1].m128_f32[3] + to.v3();
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), work2[0]);
		to.v4() = work2[1].m128_f32[0] + to.v4();
		to.v5() = work2[1].m128_f32[1] + to.v5();
		to.v6() = work2[1].m128_f32[2] + to.v6();
		to.v7() = work2[1].m128_f32[3] + to.v7();

		work2[0] = ENFT_SSE::_mm_set1_ps(B.v2());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work2[0]);
		to.v0() = work2[1].m128_f32[0] + to.v0();
		to.v1() = work2[1].m128_f32[1] + to.v1();
		to.v2() = work2[1].m128_f32[2] + to.v2();
		to.v3() = work2[1].m128_f32[3] + to.v3();
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), work2[0]);
		to.v4() = work2[1].m128_f32[0] + to.v4();
		to.v5() = work2[1].m128_f32[1] + to.v5();
		to.v6() = work2[1].m128_f32[2] + to.v6();
		to.v7() = work2[1].m128_f32[3] + to.v7();

		work2[0] = ENFT_SSE::_mm_set1_ps(B.v3());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), work2[0]);
		to.v0() = work2[1].m128_f32[0] + to.v0();
		to.v1() = work2[1].m128_f32[1] + to.v1();
		to.v2() = work2[1].m128_f32[2] + to.v2();
		to.v3() = work2[1].m128_f32[3] + to.v3();
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_34_35_36_37(), work2[0]);
		to.v4() = work2[1].m128_f32[0] + to.v4();
		to.v5() = work2[1].m128_f32[1] + to.v5();
		to.v6() = work2[1].m128_f32[2] + to.v6();
		to.v7() = work2[1].m128_f32[3] + to.v7();

		work2[0] = ENFT_SSE::_mm_set1_ps(B.v4());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), work2[0]);
		to.v0() = work2[1].m128_f32[0] + to.v0();
		to.v1() = work2[1].m128_f32[1] + to.v1();
		to.v2() = work2[1].m128_f32[2] + to.v2();
		to.v3() = work2[1].m128_f32[3] + to.v3();
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_44_45_46_47(), work2[0]);
		to.v4() = work2[1].m128_f32[0] + to.v4();
		to.v5() = work2[1].m128_f32[1] + to.v5();
		to.v6() = work2[1].m128_f32[2] + to.v6();
		to.v7() = work2[1].m128_f32[3] + to.v7();

		work2[0] = ENFT_SSE::_mm_set1_ps(B.v5());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), work2[0]);
		to.v0() = work2[1].m128_f32[0] + to.v0();
		to.v1() = work2[1].m128_f32[1] + to.v1();
		to.v2() = work2[1].m128_f32[2] + to.v2();
		to.v3() = work2[1].m128_f32[3] + to.v3();
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_54_55_56_57(), work2[0]);
		to.v4() = work2[1].m128_f32[0] + to.v4();
		to.v5() = work2[1].m128_f32[1] + to.v5();
		to.v6() = work2[1].m128_f32[2] + to.v6();
		to.v7() = work2[1].m128_f32[3] + to.v7();

		work2[0] = ENFT_SSE::_mm_set1_ps(B.v6());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), work2[0]);
		to.v0() = work2[1].m128_f32[0] + to.v0();
		to.v1() = work2[1].m128_f32[1] + to.v1();
		to.v2() = work2[1].m128_f32[2] + to.v2();
		to.v3() = work2[1].m128_f32[3] + to.v3();
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_64_65_66_67(), work2[0]);
		to.v4() = work2[1].m128_f32[0] + to.v4();
		to.v5() = work2[1].m128_f32[1] + to.v5();
		to.v6() = work2[1].m128_f32[2] + to.v6();
		to.v7() = work2[1].m128_f32[3] + to.v7();

		work2[0] = ENFT_SSE::_mm_set1_ps(B.v7());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_70_71_72_73(), work2[0]);
		to.v0() = work2[1].m128_f32[0] + to.v0();
		to.v1() = work2[1].m128_f32[1] + to.v1();
		to.v2() = work2[1].m128_f32[2] + to.v2();
		to.v3() = work2[1].m128_f32[3] + to.v3();
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_74_75_76_77(), work2[0]);
		to.v4() = work2[1].m128_f32[0] + to.v4();
		to.v5() = work2[1].m128_f32[1] + to.v5();
		to.v6() = work2[1].m128_f32[2] + to.v6();
		to.v7() = work2[1].m128_f32[3] + to.v7();
	}
	inline void SubtractATBFrom(const AlignedMatrix2x8f &A, const AlignedMatrix2x8f &B, AlignedMatrix8f &from, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_04_05_06_07() = ENFT_SSE::_mm_sub_ps(from.M_04_05_06_07(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17())));
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_14_15_16_17() = ENFT_SSE::_mm_sub_ps(from.M_14_15_16_17(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17())));
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_24_25_26_27() = ENFT_SSE::_mm_sub_ps(from.M_24_25_26_27(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17())));
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M03());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M13());
		from.M_30_31_32_33() = ENFT_SSE::_mm_sub_ps(from.M_30_31_32_33(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_34_35_36_37() = ENFT_SSE::_mm_sub_ps(from.M_34_35_36_37(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17())));
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M04());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M14());
		from.M_40_41_42_43() = ENFT_SSE::_mm_sub_ps(from.M_40_41_42_43(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_44_45_46_47() = ENFT_SSE::_mm_sub_ps(from.M_44_45_46_47(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17())));
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M05());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M15());
		from.M_50_51_52_53() = ENFT_SSE::_mm_sub_ps(from.M_50_51_52_53(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_54_55_56_57() = ENFT_SSE::_mm_sub_ps(from.M_54_55_56_57(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17())));
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M06());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M16());
		from.M_60_61_62_63() = ENFT_SSE::_mm_sub_ps(from.M_60_61_62_63(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_64_65_66_67() = ENFT_SSE::_mm_sub_ps(from.M_64_65_66_67(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17())));
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M07());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M17());
		from.M_70_71_72_73() = ENFT_SSE::_mm_sub_ps(from.M_70_71_72_73(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_74_75_76_77() = ENFT_SSE::_mm_sub_ps(from.M_74_75_76_77(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17())));
	}
	inline void SubtractATBFrom(const AlignedMatrix3x8f &A, const AlignedMatrix3x8f &B, AlignedMatrix8f &from, ENFT_SSE::__m128 *work3)
	{
//#if _DEBUG
//		const float* _A[3] = {&A.M00(), &A.M10(), &A.M20()};
//		const float* _B[8] = {&B.M00(), &B.M10(), &B.M20()};
//		float* _M1[8] = {&from.M00(), &from.M10(), &from.M20(), &from.M30(), &from.M40(), &from.M50(), &from.M60(), &from.M70()}, _M2[8][8];
//		for(int i = 0; i < 8; ++i)
//		for(int j = 0; j < 8; ++j)
//			_M2[i][j] = _M1[i][j] - (_A[0][i] * _B[0][j] + _A[1][i] * _B[1][j] + _A[2][i] * _B[2][j]);
//#endif
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M00());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M10());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M20());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_04_05_06_07() = ENFT_SSE::_mm_sub_ps(from.M_04_05_06_07(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M01());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M11());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M21());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_14_15_16_17() = ENFT_SSE::_mm_sub_ps(from.M_14_15_16_17(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M02());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M12());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M22());
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_24_25_26_27() = ENFT_SSE::_mm_sub_ps(from.M_24_25_26_27(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M03());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M13());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M23());
		from.M_30_31_32_33() = ENFT_SSE::_mm_sub_ps(from.M_30_31_32_33(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_34_35_36_37() = ENFT_SSE::_mm_sub_ps(from.M_34_35_36_37(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M04());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M14());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M24());
		from.M_40_41_42_43() = ENFT_SSE::_mm_sub_ps(from.M_40_41_42_43(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_44_45_46_47() = ENFT_SSE::_mm_sub_ps(from.M_44_45_46_47(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M05());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M15());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M25());
		from.M_50_51_52_53() = ENFT_SSE::_mm_sub_ps(from.M_50_51_52_53(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_54_55_56_57() = ENFT_SSE::_mm_sub_ps(from.M_54_55_56_57(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M06());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M16());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M26());
		from.M_60_61_62_63() = ENFT_SSE::_mm_sub_ps(from.M_60_61_62_63(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_64_65_66_67() = ENFT_SSE::_mm_sub_ps(from.M_64_65_66_67(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M07());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M17());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M27());
		from.M_70_71_72_73() = ENFT_SSE::_mm_sub_ps(from.M_70_71_72_73(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_74_75_76_77() = ENFT_SSE::_mm_sub_ps(from.M_74_75_76_77(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
//#if _DEBUG
//		for(int i = 0; i < 8; ++i)
//		for(int j = 0; j < 8; ++j)
//		{
//			const float v1 = _M1[i][j], v2 = _M2[i][j];
//			if(!EQUAL(v1, v2))
//				printf("(%d, %d): %f - %f = %f\n", i, j, v1, v2, v1 - v2);
//		}
//#endif
	}
	inline void ABT(const AlignedVector8f &s1, const AlignedVector8f &s2, AlignedMatrix8f &ABT, ENFT_SSE::__m128 *work1)
	{
		work1[0] = ENFT_SSE::_mm_set1_ps(s1.v0());
		ABT.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v0123());
		ABT.M_04_05_06_07() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v4567());

		work1[0] = ENFT_SSE::_mm_set1_ps(s1.v1());
		ABT.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v0123());
		ABT.M_14_15_16_17() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v4567());

		work1[0] = ENFT_SSE::_mm_set1_ps(s1.v2());
		ABT.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v0123());
		ABT.M_24_25_26_27() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v4567());

		work1[0] = ENFT_SSE::_mm_set1_ps(s1.v3());
		ABT.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v0123());
		ABT.M_34_35_36_37() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v4567());

		work1[0] = ENFT_SSE::_mm_set1_ps(s1.v4());
		ABT.M_40_41_42_43() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v0123());
		ABT.M_44_45_46_47() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v4567());

		work1[0] = ENFT_SSE::_mm_set1_ps(s1.v5());
		ABT.M_50_51_52_53() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v0123());
		ABT.M_54_55_56_57() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v4567());

		work1[0] = ENFT_SSE::_mm_set1_ps(s1.v6());
		ABT.M_60_61_62_63() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v0123());
		ABT.M_64_65_66_67() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v4567());

		work1[0] = ENFT_SSE::_mm_set1_ps(s1.v7());
		ABT.M_70_71_72_73() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v0123());
		ABT.M_74_75_76_77() = ENFT_SSE::_mm_mul_ps(work1[0], s2.v4567());
	}
	inline void ssTA(const AlignedVector8f &s, AlignedMatrix8f &A, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_set1_ps(s.v0());
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_00_01_02_03());
		A.M_04_05_06_07() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_04_05_06_07());

		work = ENFT_SSE::_mm_set1_ps(s.v1());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_10_11_12_13());
		A.M_14_15_16_17() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_14_15_16_17());

		work = ENFT_SSE::_mm_set1_ps(s.v2());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_20_21_22_23());
		A.M_24_25_26_27() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_24_25_26_27());

		work = ENFT_SSE::_mm_set1_ps(s.v3());
		A.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_30_31_32_33());
		A.M_34_35_36_37() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_34_35_36_37());

		work = ENFT_SSE::_mm_set1_ps(s.v4());
		A.M_40_41_42_43() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_40_41_42_43());
		A.M_44_45_46_47() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_44_45_46_47());

		work = ENFT_SSE::_mm_set1_ps(s.v5());
		A.M_50_51_52_53() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_50_51_52_53());
		A.M_54_55_56_57() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_54_55_56_57());

		work = ENFT_SSE::_mm_set1_ps(s.v6());
		A.M_60_61_62_63() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_60_61_62_63());
		A.M_64_65_66_67() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_64_65_66_67());

		work = ENFT_SSE::_mm_set1_ps(s.v7());
		A.M_70_71_72_73() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_70_71_72_73());
		A.M_74_75_76_77() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_74_75_76_77());
	}
	inline void sA(const AlignedMatrix8f &s, AlignedMatrix8f &A)
	{
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(s.M_00_01_02_03(), A.M_00_01_02_03());		A.M_04_05_06_07() = ENFT_SSE::_mm_mul_ps(s.M_04_05_06_07(), A.M_04_05_06_07());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(s.M_10_11_12_13(), A.M_10_11_12_13());		A.M_14_15_16_17() = ENFT_SSE::_mm_mul_ps(s.M_14_15_16_17(), A.M_14_15_16_17());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(s.M_20_21_22_23(), A.M_20_21_22_23());		A.M_24_25_26_27() = ENFT_SSE::_mm_mul_ps(s.M_24_25_26_27(), A.M_24_25_26_27());
		A.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(s.M_30_31_32_33(), A.M_30_31_32_33());		A.M_34_35_36_37() = ENFT_SSE::_mm_mul_ps(s.M_34_35_36_37(), A.M_34_35_36_37());
		A.M_40_41_42_43() = ENFT_SSE::_mm_mul_ps(s.M_40_41_42_43(), A.M_40_41_42_43());		A.M_44_45_46_47() = ENFT_SSE::_mm_mul_ps(s.M_44_45_46_47(), A.M_44_45_46_47());
		A.M_50_51_52_53() = ENFT_SSE::_mm_mul_ps(s.M_50_51_52_53(), A.M_50_51_52_53());		A.M_54_55_56_57() = ENFT_SSE::_mm_mul_ps(s.M_54_55_56_57(), A.M_54_55_56_57());
		A.M_60_61_62_63() = ENFT_SSE::_mm_mul_ps(s.M_60_61_62_63(), A.M_60_61_62_63());		A.M_64_65_66_67() = ENFT_SSE::_mm_mul_ps(s.M_64_65_66_67(), A.M_64_65_66_67());
		A.M_70_71_72_73() = ENFT_SSE::_mm_mul_ps(s.M_70_71_72_73(), A.M_70_71_72_73());		A.M_74_75_76_77() = ENFT_SSE::_mm_mul_ps(s.M_74_75_76_77(), A.M_74_75_76_77());
	}

	bool InvertSymmetricUpper(const AlignedMatrix8f &A, AlignedMatrix8f &Ainv, float *work0);
	bool SolveLinearSystemSymmetricUpper(AlignedMatrix8f &A, AlignedVector8f &b);
	inline bool SolveLinearSystemSymmetricUpper(AlignedMatrix8f &A, AlignedVector8f &b, float *work0) { return SolveLinearSystemSymmetricUpper(A, b); }
	bool SolveLinearSystem(AlignedMatrix8f &A, AlignedVector8f &b, float *work24);

}

#endif