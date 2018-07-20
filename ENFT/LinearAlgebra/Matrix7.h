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

#ifndef _MATRIX_7_H_
#define _MATRIX_7_H_

#include "Vector7.h"
#include "Matrix6x7.h"
#include "Matrix6.h"
#include "Matrix7x3.h"

namespace LA
{

	class AlignedMatrix7f : public AlignedMatrix6x7f
	{

	public:

		inline const ENFT_SSE::__m128& M_60_61_62_63() const { return m_60_61_62_63; }		inline ENFT_SSE::__m128& M_60_61_62_63() { return m_60_61_62_63; }
		inline const ENFT_SSE::__m128& M_64_65_66_x () const { return m_64_65_66_x; }			inline ENFT_SSE::__m128& M_64_65_66_x () { return m_64_65_66_x; }
		inline const float& M60() const { return m_60_61_62_63.m128_f32[0]; }		inline float& M60() { return m_60_61_62_63.m128_f32[0]; }
		inline const float& M61() const { return m_60_61_62_63.m128_f32[1]; }		inline float& M61() { return m_60_61_62_63.m128_f32[1]; }
		inline const float& M62() const { return m_60_61_62_63.m128_f32[2]; }		inline float& M62() { return m_60_61_62_63.m128_f32[2]; }
		inline const float& M63() const { return m_60_61_62_63.m128_f32[3]; }		inline float& M63() { return m_60_61_62_63.m128_f32[3]; }
		inline const float& M64() const { return m_64_65_66_x.m128_f32[0]; }		inline float& M64() { return m_64_65_66_x.m128_f32[0]; }
		inline const float& M65() const { return m_64_65_66_x.m128_f32[1]; }		inline float& M65() { return m_64_65_66_x.m128_f32[1]; }
		inline const float& M66() const { return m_64_65_66_x.m128_f32[2]; }		inline float& M66() { return m_64_65_66_x.m128_f32[2]; }
		inline const float& reserve6() const { return m_64_65_66_x.m128_f32[3]; }	inline float& reserve6() { return m_64_65_66_x.m128_f32[3]; }
		inline void SetZero() { memset(this, 0, sizeof(AlignedMatrix7f)); }
		inline void GetDiagonal(AlignedVector7f &d) const
		{
			d.v0() = M00();		d.v1() = M11();		d.v2() = M22();		d.v3() = M33();		d.v4() = M44();		d.v5() = M55();		d.v6() = M66();		d.reserve() = 0;
		}
		inline void SetDiagonal(const AlignedVector7f &d)
		{
			M00() = d.v0();		M11() = d.v1();		M22() = d.v2();		M33() = d.v3();		M44() = d.v4();		M55() = d.v5();		M66() = d.v6();
		}
		inline void ScaleDiagonal(const float &lambda)
		{
			M00() *= lambda;	M11() *= lambda;	M22() *= lambda;	M33() *= lambda;	M44() *= lambda;	M55() *= lambda;	M66() *= lambda;
		}
		inline void IncreaseDiagonal(const float &lambda)
		{
			M00() += lambda;	M11() += lambda;	M22() += lambda;	M33() += lambda;	M44() += lambda;	M55() += lambda;	M66() += lambda;
		}
		inline void SetLowerFromUpper()
		{
			M10() = M01();
			M20() = M02();	M21() = M12();
			M30() = M03();	M31() = M13();	M32() = M23();
			M40() = M04();	M41() = M14();	M42() = M24();	M43() = M34();
			M50() = M05();	M51() = M15();	M52() = M25();	M53() = M35();	M54() = M45();
			M60() = M06();	M61() = M16();	M62() = M26();	M63() = M36();	M64() = M46();	M65() = M56();
		}
#if _DEBUG
		inline const float* operator[] (const int &i) const
		{
			switch(i)
			{
			case 0:	return &m_00_01_02_03.m128_f32[0];	break;
			case 1:	return &m_10_11_12_13.m128_f32[0];	break;
			case 2:	return &m_20_21_22_23.m128_f32[0];	break;
			case 3:	return &m_30_31_32_33.m128_f32[0];	break;
			case 4:	return &m_40_41_42_43.m128_f32[0];	break;
			case 5:	return &m_50_51_52_53.m128_f32[0];	break;
			case 6:	return &m_60_61_62_63.m128_f32[0];	break;
			default: return NULL;
			}
		}
#endif
		inline void Print() const
		{
			printf("%f %f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05(), M06());
			printf("%f %f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15(), M16());
			printf("%f %f %f %f %f %f %f\n", M20(), M21(), M22(), M23(), M24(), M25(), M26());
			printf("%f %f %f %f %f %f %f\n", M30(), M31(), M32(), M33(), M34(), M35(), M36());
			printf("%f %f %f %f %f %f %f\n", M40(), M41(), M42(), M43(), M44(), M45(), M46());
			printf("%f %f %f %f %f %f %f\n", M50(), M51(), M52(), M53(), M54(), M55(), M56());
			printf("%f %f %f %f %f %f %f\n", M60(), M61(), M62(), M63(), M64(), M65(), M66());
		}

	protected:

		ENFT_SSE::__m128 m_60_61_62_63, m_64_65_66_x;

	};

	inline void SetReserve(AlignedMatrix7f &M) { M.reserve0() = M.reserve1() = M.reserve2() = M.reserve3() = M.reserve4() = M.reserve5() = M.reserve6() = 0; }

	inline void AddATAToUpper(const AlignedMatrix2x7f &A, AlignedMatrix7f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], A.M_10_11_12_13())), to.M_00_01_02_03());
		to.M_04_05_06_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], A.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], A.M_14_15_16_x ())), to.M_04_05_06_x ());
		
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], A.M_10_11_12_13())), to.M_10_11_12_13());
		to.M_14_15_16_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], A.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], A.M_14_15_16_x ())), to.M_14_15_16_x ());

		to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + to.M22();
		to.M23() = A.M02() * A.M03() + A.M12() * A.M13() + to.M23();
		to.M_24_25_26_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), A.M_04_05_06_x ()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), A.M_14_15_16_x ())), to.M_24_25_26_x ());

		to.M33() = A.M03() * A.M03() + A.M13() * A.M13() + to.M33();
		to.M_34_35_36_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M03()), A.M_04_05_06_x ()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M13()), A.M_14_15_16_x ())), to.M_34_35_36_x ());

		to.M_44_45_46_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M04()), A.M_04_05_06_x ()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M14()), A.M_14_15_16_x ())), to.M_44_45_46_x ());

		to.M55() = A.M05() * A.M05() + A.M15() * A.M15() + to.M55();
		to.M56() = A.M05() * A.M06() + A.M15() * A.M16() + to.M56();

		to.M66() = A.M06() * A.M06() + A.M16() * A.M16() + to.M66();
	}
	inline void AddATAToUpper(const AlignedMatrix3x7f &A, AlignedMatrix7f &to, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M00());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M10());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M20());
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], A.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[2], A.M_20_21_22_23()), to.M_00_01_02_03()));
		to.M_04_05_06_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], A.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], A.M_14_15_16_x ())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[2], A.M_24_25_26_x ()), to.M_04_05_06_x ()));

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M01());	work3[1] = ENFT_SSE::_mm_set1_ps(A.M11());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M21());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], A.M_10_11_12_13())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[2], A.M_20_21_22_23()), to.M_10_11_12_13()));
		to.M_14_15_16_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], A.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], A.M_14_15_16_x ())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[2], A.M_24_25_26_x ()), to.M_14_15_16_x ()));

		to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + A.M22() * A.M22() + to.M22();
		to.M23() = A.M02() * A.M03() + A.M12() * A.M13() + A.M22() * A.M23() + to.M23();
		to.M_24_25_26_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), A.M_04_05_06_x()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), A.M_14_15_16_x())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M22()), A.M_24_25_26_x()), to.M_14_15_16_x()));

		to.M33() = A.M03() * A.M03() + A.M13() * A.M13() + A.M23() * A.M23() + to.M33();
		to.M_34_35_36_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M03()), A.M_04_05_06_x()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M13()), A.M_14_15_16_x())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M23()), A.M_24_25_26_x()), to.M_34_35_36_x()));

		to.M_44_45_46_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M04()), A.M_04_05_06_x()), 
												   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M14()), A.M_14_15_16_x())), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M24()), A.M_24_25_26_x()), to.M_44_45_46_x()));

		to.M55() = A.M05() * A.M05() + A.M15() * A.M15() + A.M25() * A.M25() + to.M55();
		to.M56() = A.M05() * A.M06() + A.M15() * A.M16() + A.M25() * A.M26() + to.M56();

		to.M66() = A.M06() * A.M06() + A.M16() * A.M16() + A.M26() * A.M26() + to.M66();
	}
	template<class MATRIX> inline void FinishAdditionATAToUpper(AlignedMatrix7f &to) {}

	inline void AddAATToUpper(const AlignedVector7f &A, AlignedMatrix7f &to, ENFT_SSE::__m128 &work)
	{
		work =  ENFT_SSE::_mm_set1_ps(A.v0());
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.v0123()), to.M_00_01_02_03());
		to.M_04_05_06_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.v456x()), to.M_04_05_06_x ());

		work =  ENFT_SSE::_mm_set1_ps(A.v1());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.v0123()), to.M_10_11_12_13());
		to.M_14_15_16_x () = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.v456x()), to.M_14_15_16_x ());

		to.M22() = A.v2() * A.v2() + to.M22();
		to.M23() = A.v2() * A.v3() + to.M23();
		to.M_24_25_26_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v2()), A.v456x()), to.M_24_25_26_x());

		to.M33() = A.v3() * A.v3() + to.M33();
		to.M_34_35_36_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v3()), A.v456x()), to.M_34_35_36_x());

		to.M_44_45_46_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v4()), A.v456x()), to.M_44_45_46_x());
		
		to.M55() = A.v5() * A.v5() + to.M55();
		to.M56() = A.v5() * A.v6() + to.M56();

		to.M66() = A.v6() * A.v6() + to.M66();
	}
	inline void AddAATToUpper(const AlignedMatrix7x3f &A, AlignedMatrix7f &to)
	{
		to.M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_00_01_02_x())) + to.M00();
		to.M01() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_10_11_12_x())) + to.M01();
		to.M02() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_20_21_22_x())) + to.M02();
		to.M03() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_30_31_32_x())) + to.M03();
		to.M04() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_40_41_42_x())) + to.M04();
		to.M05() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_50_51_52_x())) + to.M05();
		to.M06() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_60_61_62_x())) + to.M06();

		to.M11() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_10_11_12_x())) + to.M11();
		to.M12() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_20_21_22_x())) + to.M12();
		to.M13() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_30_31_32_x())) + to.M13();
		to.M14() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_40_41_42_x())) + to.M14();
		to.M15() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_50_51_52_x())) + to.M15();
		to.M16() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_60_61_62_x())) + to.M16();

		to.M22() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), A.M_20_21_22_x())) + to.M22();
		to.M23() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), A.M_30_31_32_x())) + to.M23();
		to.M24() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), A.M_40_41_42_x())) + to.M24();
		to.M25() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), A.M_50_51_52_x())) + to.M25();
		to.M26() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), A.M_60_61_62_x())) + to.M26();

		to.M33() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_x(), A.M_30_31_32_x())) + to.M33();
		to.M34() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_x(), A.M_40_41_42_x())) + to.M34();
		to.M35() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_x(), A.M_50_51_52_x())) + to.M35();
		to.M36() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_x(), A.M_60_61_62_x())) + to.M36();

		to.M44() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_x(), A.M_40_41_42_x())) + to.M44();
		to.M45() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_x(), A.M_50_51_52_x())) + to.M45();
		to.M46() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_x(), A.M_60_61_62_x())) + to.M46();

		to.M55() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_x(), A.M_50_51_52_x())) + to.M55();
		to.M56() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_x(), A.M_60_61_62_x())) + to.M56();

		to.M66() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_x(), A.M_60_61_62_x())) + to.M66();
	}

	inline void SetLowerFromUpper(AlignedMatrix7f &M) { M.SetLowerFromUpper(); }
	inline void GetDiagonal(const AlignedMatrix7f &M, AlignedVector7f &d) { M.GetDiagonal(d); }
	inline void SetDiagonal(const AlignedVector7f &d, AlignedMatrix7f &M) { M.SetDiagonal(d); }
	inline void ScaleDiagonal(const float &lambda, AlignedMatrix7f &M) { M.ScaleDiagonal(lambda); }
	inline void IncreaseDiagonal(const float &lambda, AlignedMatrix7f &M) { M.IncreaseDiagonal(lambda); }

	inline void AB(const AlignedMatrix7f &A, const AlignedVector7f &B, AlignedVector7f &AB)
	{
#if _DEBUG
		assert(B.reserve() == 0);
#endif
		AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x(), B.v456x())));
		AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x(), B.v456x())));
		AB.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_x(), B.v456x())));
		AB.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_34_35_36_x(), B.v456x())));
		AB.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_44_45_46_x(), B.v456x())));
		AB.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_54_55_56_x(), B.v456x())));
		AB.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_64_65_66_x(), B.v456x())));
	}
	inline void AddABTo(const AlignedMatrix7f &A, const AlignedVector7f &B, AlignedVector7f &to)
	{
#if _DEBUG
		assert(B.reserve() == 0);
#endif
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x(), B.v456x()))) + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x(), B.v456x()))) + to.v1();
		to.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_x(), B.v456x()))) + to.v2();
		to.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_34_35_36_x(), B.v456x()))) + to.v3();
		to.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_44_45_46_x(), B.v456x()))) + to.v4();
		to.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_54_55_56_x(), B.v456x()))) + to.v5();
		to.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_64_65_66_x(), B.v456x()))) + to.v6();
	}
	inline void AddABTo(const AlignedMatrix7f &A, const AlignedVector7f &B, AlignedVector7f &to, ENFT_SSE::__m128 *work0) { AddABTo(A, B, to); }
	inline void AddATBTo(const AlignedMatrix7f &A, const AlignedVector7f &B, AlignedVector7f &to, ENFT_SSE::__m128 *work1)
	{
		work1[0] = ENFT_SSE::_mm_set1_ps(B.v0());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work1[0]), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_x (), work1[0]), to.v456x());

		work1[0] = ENFT_SSE::_mm_set1_ps(B.v1());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work1[0]), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_14_15_16_x (), work1[0]), to.v456x());

		work1[0] = ENFT_SSE::_mm_set1_ps(B.v2());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work1[0]), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_24_25_26_x (), work1[0]), to.v456x());

		work1[0] = ENFT_SSE::_mm_set1_ps(B.v3());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), work1[0]), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_34_35_36_x (), work1[0]), to.v456x());

		work1[0] = ENFT_SSE::_mm_set1_ps(B.v4());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), work1[0]), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_44_45_46_x (), work1[0]), to.v456x());

		work1[0] = ENFT_SSE::_mm_set1_ps(B.v5());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), work1[0]), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_54_55_56_x (), work1[0]), to.v456x());

		work1[0] = ENFT_SSE::_mm_set1_ps(B.v6());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), work1[0]), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_64_65_66_x (), work1[0]), to.v456x());
	}
	inline void SubtractATBFrom(const AlignedMatrix3x7f &A, const AlignedMatrix3x7f &B, AlignedMatrix7f &from, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M00());		work3[1] = ENFT_SSE::_mm_set1_ps(A.M10());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M20());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_04_05_06_x () = ENFT_SSE::_mm_sub_ps(from.M_04_05_06_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x ())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x ())));

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M01());		work3[1] = ENFT_SSE::_mm_set1_ps(A.M11());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M21());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_14_15_16_x () = ENFT_SSE::_mm_sub_ps(from.M_14_15_16_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x ())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x ())));

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M02());		work3[1] = ENFT_SSE::_mm_set1_ps(A.M12());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M22());
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_24_25_26_x () = ENFT_SSE::_mm_sub_ps(from.M_24_25_26_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x ())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x ())));

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M03());		work3[1] = ENFT_SSE::_mm_set1_ps(A.M13());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M23());
		from.M_30_31_32_33() = ENFT_SSE::_mm_sub_ps(from.M_30_31_32_33(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_34_35_36_x () = ENFT_SSE::_mm_sub_ps(from.M_34_35_36_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x ())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x ())));

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M04());		work3[1] = ENFT_SSE::_mm_set1_ps(A.M14());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M24());
		from.M_40_41_42_43() = ENFT_SSE::_mm_sub_ps(from.M_40_41_42_43(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_44_45_46_x () = ENFT_SSE::_mm_sub_ps(from.M_44_45_46_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x ())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x ())));

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M05());		work3[1] = ENFT_SSE::_mm_set1_ps(A.M15());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M25());
		from.M_50_51_52_53() = ENFT_SSE::_mm_sub_ps(from.M_50_51_52_53(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_54_55_56_x () = ENFT_SSE::_mm_sub_ps(from.M_54_55_56_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x ())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x ())));

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M06());		work3[1] = ENFT_SSE::_mm_set1_ps(A.M16());	work3[2] = ENFT_SSE::_mm_set1_ps(A.M26());
		from.M_60_61_62_63() = ENFT_SSE::_mm_sub_ps(from.M_60_61_62_63(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_64_65_66_x () = ENFT_SSE::_mm_sub_ps(from.M_64_65_66_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x ())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x ())));
	}
	inline void SubtractATBFrom(const AlignedMatrix2x7f &A, const AlignedMatrix2x7f &B, AlignedMatrix7f &from, ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());		work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_04_05_06_x () = ENFT_SSE::_mm_sub_ps(from.M_04_05_06_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())));

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());		work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_14_15_16_x () = ENFT_SSE::_mm_sub_ps(from.M_14_15_16_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())));

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());		work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_24_25_26_x () = ENFT_SSE::_mm_sub_ps(from.M_24_25_26_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())));

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M03());		work2[1] = ENFT_SSE::_mm_set1_ps(A.M13());
		from.M_30_31_32_33() = ENFT_SSE::_mm_sub_ps(from.M_30_31_32_33(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_34_35_36_x () = ENFT_SSE::_mm_sub_ps(from.M_34_35_36_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())));

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M04());		work2[1] = ENFT_SSE::_mm_set1_ps(A.M14());
		from.M_40_41_42_43() = ENFT_SSE::_mm_sub_ps(from.M_40_41_42_43(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_44_45_46_x () = ENFT_SSE::_mm_sub_ps(from.M_44_45_46_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())));

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M05());		work2[1] = ENFT_SSE::_mm_set1_ps(A.M15());
		from.M_50_51_52_53() = ENFT_SSE::_mm_sub_ps(from.M_50_51_52_53(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_54_55_56_x () = ENFT_SSE::_mm_sub_ps(from.M_54_55_56_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())));

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M06());		work2[1] = ENFT_SSE::_mm_set1_ps(A.M16());
		from.M_60_61_62_63() = ENFT_SSE::_mm_sub_ps(from.M_60_61_62_63(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_10_11_12_13())));
		from.M_64_65_66_x () = ENFT_SSE::_mm_sub_ps(from.M_64_65_66_x (), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_04_05_06_x ()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_x ())));
	}
	inline void sA(const AlignedMatrix7f &s, AlignedMatrix7f &A)
	{
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(s.M_00_01_02_03(), A.M_00_01_02_03());		A.M_04_05_06_x() = ENFT_SSE::_mm_mul_ps(s.M_04_05_06_x(), A.M_04_05_06_x());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(s.M_10_11_12_13(), A.M_10_11_12_13());		A.M_14_15_16_x() = ENFT_SSE::_mm_mul_ps(s.M_14_15_16_x(), A.M_14_15_16_x());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(s.M_20_21_22_23(), A.M_20_21_22_23());		A.M_24_25_26_x() = ENFT_SSE::_mm_mul_ps(s.M_24_25_26_x(), A.M_24_25_26_x());
		A.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(s.M_30_31_32_33(), A.M_30_31_32_33());		A.M_34_35_36_x() = ENFT_SSE::_mm_mul_ps(s.M_34_35_36_x(), A.M_34_35_36_x());
		A.M_40_41_42_43() = ENFT_SSE::_mm_mul_ps(s.M_40_41_42_43(), A.M_40_41_42_43());		A.M_44_45_46_x() = ENFT_SSE::_mm_mul_ps(s.M_44_45_46_x(), A.M_44_45_46_x());
		A.M_50_51_52_53() = ENFT_SSE::_mm_mul_ps(s.M_50_51_52_53(), A.M_50_51_52_53());		A.M_54_55_56_x() = ENFT_SSE::_mm_mul_ps(s.M_54_55_56_x(), A.M_54_55_56_x());
		A.M_60_61_62_63() = ENFT_SSE::_mm_mul_ps(s.M_60_61_62_63(), A.M_60_61_62_63());		A.M_64_65_66_x() = ENFT_SSE::_mm_mul_ps(s.M_64_65_66_x(), A.M_64_65_66_x());
	}
	inline void ABT(const AlignedVector7f &A, const AlignedVector7f &B, AlignedMatrix7f &ABT, ENFT_SSE::__m128 *work1)
	{
		work1[0] = ENFT_SSE::_mm_set1_ps(A.v0());		ABT.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(work1[0], B.v0123());		ABT.M_04_05_06_x() = ENFT_SSE::_mm_mul_ps(work1[0], B.v456x());
		work1[0] = ENFT_SSE::_mm_set1_ps(A.v1());		ABT.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(work1[0], B.v0123());		ABT.M_14_15_16_x() = ENFT_SSE::_mm_mul_ps(work1[0], B.v456x());
		work1[0] = ENFT_SSE::_mm_set1_ps(A.v2());		ABT.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(work1[0], B.v0123());		ABT.M_24_25_26_x() = ENFT_SSE::_mm_mul_ps(work1[0], B.v456x());
		work1[0] = ENFT_SSE::_mm_set1_ps(A.v3());		ABT.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(work1[0], B.v0123());		ABT.M_34_35_36_x() = ENFT_SSE::_mm_mul_ps(work1[0], B.v456x());
		work1[0] = ENFT_SSE::_mm_set1_ps(A.v4());		ABT.M_40_41_42_43() = ENFT_SSE::_mm_mul_ps(work1[0], B.v0123());		ABT.M_44_45_46_x() = ENFT_SSE::_mm_mul_ps(work1[0], B.v456x());
		work1[0] = ENFT_SSE::_mm_set1_ps(A.v5());		ABT.M_50_51_52_53() = ENFT_SSE::_mm_mul_ps(work1[0], B.v0123());		ABT.M_54_55_56_x() = ENFT_SSE::_mm_mul_ps(work1[0], B.v456x());
		work1[0] = ENFT_SSE::_mm_set1_ps(A.v6());		ABT.M_60_61_62_63() = ENFT_SSE::_mm_mul_ps(work1[0], B.v0123());		ABT.M_64_65_66_x() = ENFT_SSE::_mm_mul_ps(work1[0], B.v456x());
	}

	inline void ssTA(const AlignedVector7f &s, AlignedMatrix7f &A, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_set1_ps(s.v0());
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_00_01_02_03());
		A.M_04_05_06_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v456x()), A.M_04_05_06_x ());

		work = ENFT_SSE::_mm_set1_ps(s.v1());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_10_11_12_13());
		A.M_14_15_16_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v456x()), A.M_14_15_16_x ());

		work = ENFT_SSE::_mm_set1_ps(s.v2());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_20_21_22_23());
		A.M_24_25_26_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v456x()), A.M_24_25_26_x ());

		work = ENFT_SSE::_mm_set1_ps(s.v3());
		A.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_30_31_32_33());
		A.M_34_35_36_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v456x()), A.M_34_35_36_x ());

		work = ENFT_SSE::_mm_set1_ps(s.v4());
		A.M_40_41_42_43() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_40_41_42_43());
		A.M_44_45_46_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v456x()), A.M_44_45_46_x ());

		work = ENFT_SSE::_mm_set1_ps(s.v5());
		A.M_50_51_52_53() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_50_51_52_53());
		A.M_54_55_56_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v456x()), A.M_54_55_56_x ());

		work = ENFT_SSE::_mm_set1_ps(s.v6());
		A.M_60_61_62_63() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_60_61_62_63());
		A.M_64_65_66_x () = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v456x()), A.M_64_65_66_x ());
	}

	bool InvertSymmetricUpper(AlignedMatrix7f &A);
	bool InvertSymmetricUpper(const AlignedMatrix7f &A, AlignedMatrix7f &Ainv);
	inline bool InvertSymmetricUpper(const AlignedMatrix7f &A, AlignedMatrix7f &Ainv, float *work0) { return InvertSymmetricUpper(A, Ainv); }
	bool SolveLinearSystemSymmetricUpper(AlignedMatrix7f &A, AlignedVector7f &b);
	inline bool SolveLinearSystemSymmetricUpper(AlignedMatrix7f &A, AlignedVector7f &b, float *work0) { return SolveLinearSystemSymmetricUpper(A, b); }

	class AlignedCompactMatrix7f : public AlignedCompactMatrix6f
	{

	public:
		
		inline const ENFT_SSE::__m128& M_06_16_26_36() const { return m_06_16_26_36; }		inline ENFT_SSE::__m128& M_06_16_26_36() { return m_06_16_26_36; }
		inline const ENFT_SSE::__m128& M_46_56_x_x  () const { return m_46_56_x_x; }			inline ENFT_SSE::__m128& M_46_56_x_x  () { return m_46_56_x_x; }
		inline const ENFT_SSE::__m128& M_60_61_62_63() const { return m_60_61_62_63; }		inline ENFT_SSE::__m128& M_60_61_62_63() { return m_60_61_62_63; }
		inline const ENFT_SSE::__m128& M_64_65_66_x () const { return m_64_65_66_x; }			inline ENFT_SSE::__m128& M_64_65_66_x () { return m_64_65_66_x; }
		inline const float& M06() const { return m_06_16_26_36.m128_f32[0]; }		inline float& M06() { return m_06_16_26_36.m128_f32[0]; }
		inline const float& M16() const { return m_06_16_26_36.m128_f32[1]; }		inline float& M16() { return m_06_16_26_36.m128_f32[1]; }
		inline const float& M26() const { return m_06_16_26_36.m128_f32[2]; }		inline float& M26() { return m_06_16_26_36.m128_f32[2]; }
		inline const float& M36() const { return m_06_16_26_36.m128_f32[3]; }		inline float& M36() { return m_06_16_26_36.m128_f32[3]; }
		inline const float& M46() const { return m_46_56_x_x.m128_f32[0]; }			inline float& M46() { return m_46_56_x_x.m128_f32[0]; }
		inline const float& M56() const { return m_46_56_x_x.m128_f32[1]; }			inline float& M56() { return m_46_56_x_x.m128_f32[1]; }
		inline const float& M60() const { return m_60_61_62_63.m128_f32[0]; }		inline float& M60() { return m_60_61_62_63.m128_f32[0]; }
		inline const float& M61() const { return m_60_61_62_63.m128_f32[1]; }		inline float& M61() { return m_60_61_62_63.m128_f32[1]; }
		inline const float& M62() const { return m_60_61_62_63.m128_f32[2]; }		inline float& M62() { return m_60_61_62_63.m128_f32[2]; }
		inline const float& M63() const { return m_60_61_62_63.m128_f32[3]; }		inline float& M63() { return m_60_61_62_63.m128_f32[3]; }
		inline const float& M64() const { return m_64_65_66_x.m128_f32[0]; }		inline float& M64() { return m_64_65_66_x.m128_f32[0]; }
		inline const float& M65() const { return m_64_65_66_x.m128_f32[1]; }		inline float& M65() { return m_64_65_66_x.m128_f32[1]; }
		inline const float& M66() const { return m_64_65_66_x.m128_f32[2]; }		inline float& M66() { return m_64_65_66_x.m128_f32[2]; }
		inline const float& reserve0() const { return m_46_56_x_x.m128_f32[2]; }	inline float& reserve0() { return m_46_56_x_x.m128_f32[2]; }
		inline const float& reserve1() const { return m_46_56_x_x.m128_f32[3]; }	inline float& reserve1() { return m_46_56_x_x.m128_f32[3]; }
		inline const float& reserve2() const { return m_64_65_66_x.m128_f32[3]; }	inline float& reserve2() { return m_64_65_66_x.m128_f32[3]; }

		inline void SetZero() { memset(this, 0, sizeof(AlignedCompactMatrix7f)); }
		inline void GetDiagonal(AlignedVector7f &d) const
		{
			d.v0() = M00();		d.v1() = M11();		d.v2() = M22();		d.v3() = M33();		d.v4() = M44();		d.v5() = M55();		d.v6() = M66();		d.reserve() = 0;
		}
		inline void SetDiagonal(const AlignedVector7f &d)
		{
			M00() = d.v0();		M11() = d.v1();		M22() = d.v2();		M33() = d.v3();		M44() = d.v4();		M55() = d.v5();		M66() = d.v6();
		}
		inline void ScaleDiagonal(const float &lambda)
		{
			M00() *= lambda;	M11() *= lambda;	M22() *= lambda;	M33() *= lambda;	M44() *= lambda;	M55() *= lambda;	M66() *= lambda;
		}
		inline void IncreaseDiagonal(const float &lambda)
		{
			M00() += lambda;	M11() += lambda;	M22() += lambda;	M33() += lambda;	M44() += lambda;	M55() += lambda;	M66() += lambda;
		}
		inline void ConvertToConventionalStorage(float *work49)
		{
			memcpy(work49, &M00(), 24);
			work49[6] = M06();
			memcpy(work49 + 7, &M10(), 16);
			memcpy(work49 + 11, &M14(), 8);
			work49[13] = M16();

			memcpy(work49 + 14, &M20(), 24);
			work49[20] = M26();
			memcpy(work49 + 21, &M30(), 16);
			memcpy(work49 + 25, &M34(), 8);
			work49[27] = M36();

			memcpy(work49 + 28, &M40(), 24);
			work49[34] = M46();
			memcpy(work49 + 35, &M50(), 16);
			memcpy(work49 + 39, &M54(), 8);
			work49[41] = M56();

			memcpy(work49 + 42, &M60(), 28);

			memcpy(this, work49, 196);
		}
		inline void ConvertToConventionalStorage(float *M) const
		{
			memcpy(M, &M00(), 24);
			M[6] = M06();
			memcpy(M + 7, &M10(), 16);
			memcpy(M + 11, &M14(), 8);
			M[13] = M16();

			memcpy(M + 14, &M20(), 24);
			M[20] = M26();
			memcpy(M + 21, &M30(), 16);
			memcpy(M + 25, &M34(), 8);
			M[27] = M36();

			memcpy(M + 28, &M40(), 24);
			M[34] = M46();
			memcpy(M + 35, &M50(), 16);
			memcpy(M + 39, &M54(), 8);
			M[41] = M56();

			memcpy(M + 42, &M60(), 28);
		}
		inline void ConvertToSpecialStorage(float *work49)
		{
			memcpy(work49, this, 196);

			memcpy(&M00(), work49, 24);
			M06() = work49[6];
			memcpy(&M10(), work49 + 7, 16);
			memcpy(&M14(), work49 + 11, 8);
			M16() = work49[13];

			memcpy(&M20(), work49 + 14, 24);
			M26() = work49[20];
			memcpy(&M30(), work49 + 21, 16);
			memcpy(&M34(), work49 + 25, 8);
			M36() = work49[27];

			memcpy(&M40(), work49 + 28, 24);
			M46() = work49[34];
			memcpy(&M50(), work49 + 35, 16);
			memcpy(&M54(), work49 + 39, 8);
			M56() = work49[41];

			memcpy(&M60(), work49 + 42, 28);
		}
		inline void SetLowerFromUpper()
		{
			M10() = M01();
			M20() = M02();	M21() = M12();
			M30() = M03();	M31() = M13();	M32() = M23();
			M40() = M04();	M41() = M14();	M42() = M24();	M43() = M34();
			M50() = M05();	M51() = M15();	M52() = M25();	M53() = M35();	M54() = M45();
			M60() = M06();	M61() = M16();	M62() = M26();	M63() = M36();	M64() = M46();	M65() = M56();
			m_46_56_x_x.m128_f32[2] = m_46_56_x_x.m128_f32[3] = m_64_65_66_x.m128_f32[3] = 0;
		}
		inline void SetUpperFromLower()
		{
			M01() = M10();
			M02() = M20();	M12() = M21();
			M03() = M30();	M13() = M31();	M23() = M32();
			M04() = M40();	M14() = M41();	M24() = M42();	M34() = M43();
			M05() = M50();	M15() = M51();	M25() = M52();	M35() = M53();	M45() = M54();
			M06() = M60();	M16() = M61();	M26() = M62();	M36() = M63();	M46() = M64();	M56() = M65();
			m_46_56_x_x.m128_f32[2] = m_46_56_x_x.m128_f32[3] = m_64_65_66_x.m128_f32[3] = 0;
		}

		inline void Print() const
		{
			printf("%f %f %f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04(), M05(), M06());
			printf("%f %f %f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14(), M15(), M16());
			printf("%f %f %f %f %f %f %f\n", M20(), M21(), M22(), M23(), M24(), M25(), M26());
			printf("%f %f %f %f %f %f %f\n", M30(), M31(), M32(), M33(), M34(), M35(), M36());
			printf("%f %f %f %f %f %f %f\n", M40(), M41(), M42(), M43(), M44(), M45(), M46());
			printf("%f %f %f %f %f %f %f\n", M50(), M51(), M52(), M53(), M54(), M55(), M56());
			printf("%f %f %f %f %f %f %f\n", M60(), M61(), M62(), M63(), M64(), M65(), M66());
			printf("%f %f %f\n", m_46_56_x_x.m128_f32[2], m_46_56_x_x.m128_f32[3], m_64_65_66_x.m128_f32[3]);
		}

	protected:

		ENFT_SSE::__m128 m_06_16_26_36, m_46_56_x_x, m_60_61_62_63, m_64_65_66_x;
	};

	inline void AddATAToUpper(const AlignedCompactMatrix2x7f &A, AlignedCompactMatrix7f &to, ENFT_SSE::__m128 *work2)
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

		work2[0] = ENFT_SSE::_mm_set1_ps(A.M06());
		work2[1] = ENFT_SSE::_mm_set1_ps(A.M16());
		to.M_60_61_62_63() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work2[1])), to.M_60_61_62_63());
		work2[0] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), ENFT_SSE::_mm_movelh_ps(work2[0], work2[1]));
		to.M64() = work2[0].m128_f32[0] + work2[0].m128_f32[2] + to.M64();
		to.M65() = work2[0].m128_f32[1] + work2[0].m128_f32[3] + to.M65();
		to.M66() = A.M06() * A.M06() + A.M16() * A.M16() + to.M66();
	}
	template<class MATRIX> inline void FinishAdditionATAToUpper(AlignedCompactMatrix7f &to);
	template<> inline void FinishAdditionATAToUpper<AlignedCompactMatrix2x7f>(AlignedCompactMatrix7f &to)
	{
		to.M_06_16_26_36() = to.M_60_61_62_63();
		to.M_46_56_x_x() = to.M_64_65_66_x();
	}
	template<class MATRIX_1, class MATRIX_2> inline void FinishAdditionATAToUpper(AlignedCompactMatrix7f &to);

	inline void SetLowerFromUpper(AlignedCompactMatrix7f &M) { M.SetLowerFromUpper(); }
	inline void GetDiagonal(const AlignedCompactMatrix7f &M, AlignedVector7f &d) { M.GetDiagonal(d); }
	inline void SetDiagonal(const AlignedVector7f &d, AlignedCompactMatrix7f &M) { M.SetDiagonal(d); }
	inline void ScaleDiagonal(const float &lambda, AlignedCompactMatrix7f &M) { M.ScaleDiagonal(lambda); }
	inline void IncreaseDiagonal(const float &lambda, AlignedCompactMatrix7f &M) { M.IncreaseDiagonal(lambda); }

	inline void AB(const AlignedCompactMatrix7f &A, const AlignedVector7f &B, AlignedVector7f &AB, ENFT_SSE::__m128 *work2)
	{
#if _DEBUG
		assert(B.reserve() == 0);
#endif

		work2[0] = ENFT_SSE::_mm_movelh_ps(B.v456x(), B.v456x());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), work2[0]);
		AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work2[1].m128_f32[0] + work2[1].m128_f32[1] + A.M06() * B.v6();
		AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work2[1].m128_f32[2] + work2[1].m128_f32[3] + A.M16() * B.v6();

		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_24_25_34_35(), work2[0]);
		AB.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + work2[1].m128_f32[0] + work2[1].m128_f32[1] + A.M26() * B.v6();
		AB.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123())) + work2[1].m128_f32[2] + work2[1].m128_f32[3] + A.M36() * B.v6();

		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_44_45_54_55(), work2[0]);
		AB.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123())) + work2[1].m128_f32[0] + work2[1].m128_f32[1] + A.M46() * B.v6();
		AB.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123())) + work2[1].m128_f32[2] + work2[1].m128_f32[3] + A.M56() * B.v6();

		AB.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_64_65_66_x(), B.v456x())));
	}
	inline void ABmC(const AlignedCompactMatrix7f &A, const AlignedVector7f &B, const AlignedVector7f &C, AlignedVector7f &ABmC, ENFT_SSE::__m128 *work2)
	{
#if _DEBUG
		assert(B.reserve() == 0);
#endif

		work2[0] = ENFT_SSE::_mm_movelh_ps(B.v456x(), B.v456x());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), work2[0]);
		ABmC.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work2[1].m128_f32[0] + work2[1].m128_f32[1] + A.M06() * B.v6() - C.v0();
		ABmC.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work2[1].m128_f32[2] + work2[1].m128_f32[3] + A.M16() * B.v6() - C.v1();

		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_24_25_34_35(), work2[0]);
		ABmC.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + work2[1].m128_f32[0] + work2[1].m128_f32[1] + A.M26() * B.v6() - C.v2();
		ABmC.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123())) + work2[1].m128_f32[2] + work2[1].m128_f32[3] + A.M36() * B.v6() - C.v3();

		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_44_45_54_55(), work2[0]);
		ABmC.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123())) + work2[1].m128_f32[0] + work2[1].m128_f32[1] + A.M46() * B.v6() - C.v4();
		ABmC.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123())) + work2[1].m128_f32[2] + work2[1].m128_f32[3] + A.M56() * B.v6() - C.v5();

		ABmC.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_64_65_66_x(), B.v456x()))) - C.v6();
	}
	inline void AddABTo(const AlignedCompactMatrix7f &A, const AlignedVector7f &B, AlignedVector7f &to, ENFT_SSE::__m128 *work2)
	{
#if _DEBUG
		assert(B.reserve() == 0);
#endif

		work2[0] = ENFT_SSE::_mm_movelh_ps(B.v456x(), B.v456x());
		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_04_05_14_15(), work2[0]);
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + work2[1].m128_f32[0] + work2[1].m128_f32[1] + A.M06() * B.v6() + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + work2[1].m128_f32[2] + work2[1].m128_f32[3] + A.M16() * B.v6() + to.v1();

		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_24_25_34_35(), work2[0]);
		to.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + work2[1].m128_f32[0] + work2[1].m128_f32[1] + A.M26() * B.v6() + to.v2();
		to.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123())) + work2[1].m128_f32[2] + work2[1].m128_f32[3] + A.M36() * B.v6() + to.v3();

		work2[1] = ENFT_SSE::_mm_mul_ps(A.M_44_45_54_55(), work2[0]);
		to.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123())) + work2[1].m128_f32[0] + work2[1].m128_f32[1] + A.M46() * B.v6() + to.v4();
		to.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123())) + work2[1].m128_f32[2] + work2[1].m128_f32[3] + A.M56() * B.v6() + to.v5();

		to.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_64_65_66_x(), B.v456x()))) + to.v6();
	}
	inline void AddATBTo(const AlignedCompactMatrix7f &A, const AlignedVector7f &B, AlignedVector7f &to, ENFT_SSE::__m128 *work2)
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

		work2[0] = ENFT_SSE::_mm_set1_ps(B.v6());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), work2[0]), to.v0123());
		to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_64_65_66_x (), work2[0]), to.v456x());
		to.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_06_16_26_36(), B.v0123())) + A.M46() * B.v4() + A.M56() * B.v5() + to.v6();
	}
	inline void SubtractATBFrom(const AlignedMatrix3x7f &A, const AlignedMatrix3x7f &B, AlignedCompactMatrix7f &from, ENFT_SSE::__m128 *work3)
	{
		work3[0] = ENFT_SSE::_mm_set1_ps(A.M00());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M10());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M20());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		work3[0] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x()));
		from.M04() -= work3[0].m128_f32[0];
		from.M05() -= work3[0].m128_f32[1];
		from.M06() -= work3[0].m128_f32[2];

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M01());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M11());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M21());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		work3[0] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x()));
		from.M14() -= work3[0].m128_f32[0];
		from.M15() -= work3[0].m128_f32[1];
		from.M16() -= work3[0].m128_f32[2];

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M02());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M12());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M22());
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		work3[0] = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x()));
		from.M24() -= work3[0].m128_f32[0];
		from.M25() -= work3[0].m128_f32[1];
		from.M26() -= work3[0].m128_f32[2];

		work3[0] = ENFT_SSE::_mm_set1_ps(A.M03());
		work3[1] = ENFT_SSE::_mm_set1_ps(A.M13());
		work3[2] = ENFT_SSE::_mm_set1_ps(A.M23());
		from.M_30_31_32_33() = ENFT_SSE::_mm_sub_ps(from.M_30_31_32_33(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		work3[0] = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x()));
		from.M34() -= work3[0].m128_f32[0];
		from.M35() -= work3[0].m128_f32[1];
		from.M36() -= work3[0].m128_f32[2];

		work3[0] = _mm_set1_ps(A.M04());
		work3[1] = _mm_set1_ps(A.M14());
		work3[2] = _mm_set1_ps(A.M24());
		from.M_40_41_42_43() = ENFT_SSE::_mm_sub_ps(from.M_40_41_42_43(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		work3[0] = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x()));
		from.M44() -= work3[0].m128_f32[0];
		from.M45() -= work3[0].m128_f32[1];
		from.M46() -= work3[0].m128_f32[2];

		work3[0] = _mm_set1_ps(A.M05());
		work3[1] = _mm_set1_ps(A.M15());
		work3[2] = _mm_set1_ps(A.M25());
		from.M_50_51_52_53() = ENFT_SSE::_mm_sub_ps(from.M_50_51_52_53(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		work3[0] = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x()));
		from.M54() -= work3[0].m128_f32[0];
		from.M55() -= work3[0].m128_f32[1];
		from.M56() -= work3[0].m128_f32[2];

		work3[0] = _mm_set1_ps(A.M06());
		work3[1] = _mm_set1_ps(A.M16());
		work3[2] = _mm_set1_ps(A.M26());
		from.M_60_61_62_63() = ENFT_SSE::_mm_sub_ps(from.M_60_61_62_63(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		work3[0] = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_x()));
		from.M64() -= work3[0].m128_f32[0];
		from.M65() -= work3[0].m128_f32[1];
		from.M66() -= work3[0].m128_f32[2];
	}

	bool InvertSymmetricUpper(AlignedCompactMatrix7f &A, float *work49);
	bool InvertSymmetricUpper(const AlignedCompactMatrix7f &A, AlignedCompactMatrix7f &Ainv, float *work49);
}

#endif
