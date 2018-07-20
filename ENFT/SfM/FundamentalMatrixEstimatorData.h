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

#ifndef _FUNDAMENTAL_MATRIX_ESTIMATOR_DATA_H_
#define _FUNDAMENTAL_MATRIX_ESTIMATOR_DATA_H_

#include "SfM/FundamentalMatrix.h"
#include "SfM/Match.h"
#include "Estimation/EstimatorArsacData.h"
#include "Optimization/OptimizerData.h"
#include "LinearAlgebra/Matrix6.h"
#include "LinearAlgebra/Matrix7.h"
#include "LinearAlgebra/Matrix8.h"

class FundamentalMatrixEstimatorData : public EstimatorArsacData, public MatchSet2D, public OptimizerDataTemplate<FundamentalMatrix, LA::AlignedVector8f, LA::AlignedMatrix8f>
{

public:

	virtual void NormalizeData(const float &dataNormalizeMedian, FundamentalMatrix &F)
	{
		if(dataNormalizeMedian == 0)
		{
			m_scale1 = m_scale2 = 1.0f;
			m_normalized = false;
		}
		else
		{
			Normalize(m_work);
			F.Normalize(mean_u1v1u2v2(), scale1(), scale2(), m_work);
			m_normalized = true;
		}
	}
	virtual void DenormalizeData(FundamentalMatrix &F)
	{
		if(m_normalized)
			F.Denormalize(mean_u1v1u2v2(), scale1(), scale2(), m_work);
	}
	virtual double ComputeSSE(const FundamentalMatrix &F)
	{
		ENFT_SSE::__m128 *f = m_work;
		F.GetSSE(f);

		ENFT_SSE::__m128 &lx = m_work[9], &ly = m_work[10], &d = m_work[11], &sum = m_work[12];
		sum = ENFT_SSE::_mm_setzero_ps();
		const ushort nPacks = GetPacksNumber(), nRems = GetRemindersNumber();
		for(ushort iu1 = 0, iv1 = 1, iu2 = 2, iv2 = 3; iu1 < nPacks; iu1 += 4, iv1 += 4, iu2 += 4, iv2 += 4)
		{
			const ENFT_SSE::__m128 &u1 = GetPack(iu1), &v1 = GetPack(iv1), &u2 = GetPack(iu2), &v2 = GetPack(iv2);
			lx = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(f[0], u1), ENFT_SSE::_mm_mul_ps(f[1], v1)), f[2]);
			ly = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(f[3], u1), ENFT_SSE::_mm_mul_ps(f[4], v1)), f[5]);
			d = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(u2, lx), ENFT_SSE::_mm_mul_ps(v2, ly)), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(f[6], u1), ENFT_SSE::_mm_mul_ps(f[7], v1)), f[8]));
			sum = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_div_ps(ENFT_SSE::_mm_mul_ps(d, d), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(lx, lx), ENFT_SSE::_mm_mul_ps(ly, ly))), sum);
		}
		for(ushort iRem = 0; iRem < nRems; ++iRem)
		{
			float *work = (float *) m_work;
			sum.m128_f32[0] = F.ComputeEpipolarSquaredError(GetReminder1(iRem), GetReminder2(iRem), work) + sum.m128_f32[0];
		}
		return double(ENFT_SSE::SSE::Sum0123(sum));
	}
	virtual double GetFactorSSEToMSE() { return double(1 / (scale1() * scale2() * Size())); }
	virtual void ConstructNormalEquation(const FundamentalMatrix &F, LA::AlignedMatrix8f &A, LA::AlignedVector8f &b, LA::AlignedVector8f &s)
	{
		ENFT_SSE::__m128 *f = m_work;
		F.GetSSE(f);

		ENFT_SSE::__m128 &lx = m_work[9], &ly = m_work[10], &d = m_work[11], &jlx = m_work[12], &jly = m_work[13], &jlz = m_work[14], &jlz2 = m_work[15];
		ENFT_SSE::__m128 &one = m_work[16], &zero = m_work[17], *j = &m_work[18], &e = m_work[26];
		zero = ENFT_SSE::_mm_setzero_ps();
		one = _mm_set1_ps(1.0f);
		A.SetZero();
		b.SetZero();
		s.SetZero();
		const ushort nPacks = GetPacksNumber(), nRems = GetRemindersNumber();
		for(ushort iu1 = 0, iv1 = 1, iu2 = 2, iv2 = 3; iu1 < nPacks; iu1 += 4, iv1 += 4, iu2 += 4, iv2 += 4)
		{
			const ENFT_SSE::__m128 &u1 = GetPack(iu1), &v1 = GetPack(iv1), &u2 = GetPack(iu2), &v2 = GetPack(iv2);
			lx = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(f[0], u1), ENFT_SSE::_mm_mul_ps(f[1], v1)), f[2]);
			ly = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(f[3], u1), ENFT_SSE::_mm_mul_ps(f[4], v1)), f[5]);
			d = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(u2, lx), ENFT_SSE::_mm_mul_ps(v2, ly)), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(f[6], u1), ENFT_SSE::_mm_mul_ps(f[7], v1)), f[8]));
			jlz2 = ENFT_SSE::_mm_div_ps(one, ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(lx, lx), ENFT_SSE::_mm_mul_ps(ly, ly)));
			jlz = _mm_sqrt_ps(jlz2);
			jlx = ENFT_SSE::_mm_mul_ps(jlz, ENFT_SSE::_mm_sub_ps(u2, ENFT_SSE::_mm_mul_ps(jlz2, ENFT_SSE::_mm_mul_ps(d, lx))));
			jly = ENFT_SSE::_mm_mul_ps(jlz, ENFT_SSE::_mm_sub_ps(v2, ENFT_SSE::_mm_mul_ps(jlz2, ENFT_SSE::_mm_mul_ps(d, ly))));
			j[0] = ENFT_SSE::_mm_mul_ps(jlx, u1);		j[1] = ENFT_SSE::_mm_mul_ps(jlx, v1);		j[2] = jlx;
			j[3] = ENFT_SSE::_mm_mul_ps(jly, u1);		j[4] = ENFT_SSE::_mm_mul_ps(jly, v1);		j[5] = jly;
			j[6] = ENFT_SSE::_mm_mul_ps(jlz, u1);		j[7] = ENFT_SSE::_mm_mul_ps(jlz, v1);
			e = ENFT_SSE::_mm_sub_ps(zero, ENFT_SSE::_mm_mul_ps(d, jlz));
			A.M00() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[0])) + A.M00();
			A.M01() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[1])) + A.M01();
			A.M02() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[2])) + A.M02();
			A.M03() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[3])) + A.M03();
			A.M04() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[4])) + A.M04();
			A.M05() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[5])) + A.M05();
			A.M06() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[6])) + A.M06();
			A.M07() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[7])) + A.M07();
			A.M11() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[1])) + A.M11();
			A.M12() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[2])) + A.M12();
			A.M13() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[3])) + A.M13();
			A.M14() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[4])) + A.M14();
			A.M15() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[5])) + A.M15();
			A.M16() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[6])) + A.M16();
			A.M17() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[7])) + A.M17();
			A.M22() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[2])) + A.M22();
			A.M23() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[3])) + A.M23();
			A.M24() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[4])) + A.M24();
			A.M25() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[5])) + A.M25();
			A.M26() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[6])) + A.M26();
			A.M27() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[7])) + A.M27();
			A.M33() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[3])) + A.M33();
			A.M34() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[4])) + A.M34();
			A.M35() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[5])) + A.M35();
			A.M36() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[6])) + A.M36();
			A.M37() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[7])) + A.M37();
			A.M44() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], j[4])) + A.M44();
			A.M45() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], j[5])) + A.M45();
			A.M46() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], j[6])) + A.M46();
			A.M47() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], j[7])) + A.M47();
			A.M55() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[5], j[5])) + A.M55();
			A.M56() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[5], j[6])) + A.M56();
			A.M57() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[5], j[7])) + A.M57();
			A.M66() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[6], j[6])) + A.M66();
			A.M67() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[6], j[7])) + A.M67();
			A.M77() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[7], j[7])) + A.M77();
			b.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], e)) + b.v0();
			b.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], e)) + b.v1();
			b.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], e)) + b.v2();
			b.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], e)) + b.v3();
			b.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], e)) + b.v4();
			b.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[5], e)) + b.v5();
			b.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[6], e)) + b.v6();
			b.v7() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[7], e)) + b.v7();
			s.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[0])) + s.v0();
			s.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[1])) + s.v1();
			s.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[2])) + s.v2();
			s.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[3])) + s.v3();
			s.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], j[4])) + s.v4();
			s.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[5], j[5])) + s.v5();
			s.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[6], j[6])) + s.v6();
			s.v7() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[7], j[7])) + s.v7();
		}
		for(ushort iRem = 0; iRem < nRems; ++iRem)
		{
			float *work = (float *) m_work;
			float &lx = work[0], &ly = work[1], &d = work[2], &jlx = work[3], &jly = work[4], &jlz = work[5], &jlz2 = work[6], &e = work[7];
			ENFT_SSE::__m128 &j4 = m_work[20], &e4 = m_work[20];
			const Point2D &x1 = GetReminder1(iRem), &x2 = GetReminder2(iRem);
			lx = F.M00() * x1.x() + F.M01() * x1.y() + F.M02();
			ly = F.M10() * x1.x() + F.M11() * x1.y() + F.M12();
			d = x2.x() * lx + x2.y() * ly + F.M20() * x1.x() + F.M21() * x1.y() + F.M22();
			jlz2 = 1 / (lx * lx + ly * ly);
			jlz = sqrt(jlz2);
			jlx = jlz * (x2.x() - jlz2 * d * lx);
			jly = jlz * (x2.y() - jlz2 * d * ly);
			j[0].m128_f32[0] = jlx * x1.x();		j[0].m128_f32[1] = jlx * x1.y();		j[0].m128_f32[2] = jlx;
			j[0].m128_f32[3] = jly * x1.x();		j[1].m128_f32[0] = jly * x1.y();		j[1].m128_f32[1] = jly;
			j[1].m128_f32[2] = jlz * x1.x();		j[1].m128_f32[3] = jlz * x1.y();
			e = -d * jlz;
			j4 = _mm_set1_ps(j[0].m128_f32[0]);
			A.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[0]), A.M_00_01_02_03());
			A.M_04_05_06_07() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[1]), A.M_04_05_06_07());
			j4 = _mm_set1_ps(j[0].m128_f32[1]);
			A.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[0]), A.M_10_11_12_13());
			A.M_14_15_16_17() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[1]), A.M_14_15_16_17());
			j4 = _mm_set1_ps(j[0].m128_f32[2]);
			A.M22() = j[0].m128_f32[2] * j[0].m128_f32[2] + A.M22();
			A.M23() = j[0].m128_f32[2] * j[0].m128_f32[3] + A.M23();
			A.M_24_25_26_27() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[1]), A.M_24_25_26_27());
			j4 = _mm_set1_ps(j[0].m128_f32[3]);
			A.M33() = j[0].m128_f32[3] * j[0].m128_f32[3] + A.M33();
			A.M_34_35_36_37() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[1]), A.M_34_35_36_37());
			A.M_44_45_46_47() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(j[1].m128_f32[0]), j[1]), A.M_44_45_46_47());
			A.M_54_55_56_57() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(j[1].m128_f32[1]), j[1]), A.M_54_55_56_57());
			A.M66() = j[1].m128_f32[2] * j[1].m128_f32[2] + A.M66();
			A.M67() = j[1].m128_f32[2] * j[1].m128_f32[3] + A.M67();
			A.M77() = j[1].m128_f32[3] * j[1].m128_f32[3] + A.M77();
			e4 = _mm_set1_ps(e);
			b.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e4, j[0]), b.v0123());
			b.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e4, j[1]), b.v4567());
			s.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j[0], j[0]), s.v0123());
			s.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j[1], j[1]), s.v4567());
		}
		A.SetLowerFromUpper();
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const FundamentalMatrix &F, const LA::AlignedVector8f &s, LA::AlignedMatrix8f &A, LA::AlignedVector8f &b)
	{
		ENFT_SSE::__m128 *f = m_work;
		F.GetSSE(f);

		ENFT_SSE::__m128 &lx = m_work[9], &ly = m_work[10], &d = m_work[11], &jlx = m_work[12], &jly = m_work[13], &jlz = m_work[14], &jlz2 = m_work[15];
		ENFT_SSE::__m128 &one = m_work[16], &zero = m_work[17], *j = &m_work[18], &e = m_work[26];
		zero = ENFT_SSE::_mm_setzero_ps();
		one = _mm_set1_ps(1.0f);
		A.SetZero();
		b.SetZero();
		const ushort nPacks = GetPacksNumber(), nRems = GetRemindersNumber();
		for(ushort iu1 = 0, iv1 = 1, iu2 = 2, iv2 = 3; iu1 < nPacks; iu1 += 4, iv1 += 4, iu2 += 4, iv2 += 4)
		{
			const ENFT_SSE::__m128 &u1 = GetPack(iu1), &v1 = GetPack(iv1), &u2 = GetPack(iu2), &v2 = GetPack(iv2);
			lx = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(f[0], u1), ENFT_SSE::_mm_mul_ps(f[1], v1)), f[2]);
			ly = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(f[3], u1), ENFT_SSE::_mm_mul_ps(f[4], v1)), f[5]);
			d = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(u2, lx), ENFT_SSE::_mm_mul_ps(v2, ly)), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(f[6], u1), ENFT_SSE::_mm_mul_ps(f[7], v1)), f[8]));
			jlz2 = ENFT_SSE::_mm_div_ps(one, ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(lx, lx), ENFT_SSE::_mm_mul_ps(ly, ly)));
			jlz = _mm_sqrt_ps(jlz2);
			jlx = ENFT_SSE::_mm_mul_ps(jlz, ENFT_SSE::_mm_sub_ps(u2, ENFT_SSE::_mm_mul_ps(jlz2, ENFT_SSE::_mm_mul_ps(d, lx))));
			jly = ENFT_SSE::_mm_mul_ps(jlz, ENFT_SSE::_mm_sub_ps(v2, ENFT_SSE::_mm_mul_ps(jlz2, ENFT_SSE::_mm_mul_ps(d, ly))));
			j[0] = ENFT_SSE::_mm_mul_ps(jlx, u1);		j[1] = ENFT_SSE::_mm_mul_ps(jlx, v1);		j[2] = jlx;
			j[3] = ENFT_SSE::_mm_mul_ps(jly, u1);		j[4] = ENFT_SSE::_mm_mul_ps(jly, v1);		j[5] = jly;
			j[6] = ENFT_SSE::_mm_mul_ps(jlz, u1);		j[7] = ENFT_SSE::_mm_mul_ps(jlz, v1);
			e = ENFT_SSE::_mm_sub_ps(zero, ENFT_SSE::_mm_mul_ps(d, jlz));
			A.M00() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[0])) + A.M00();
			A.M01() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[1])) + A.M01();
			A.M02() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[2])) + A.M02();
			A.M03() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[3])) + A.M03();
			A.M04() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[4])) + A.M04();
			A.M05() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[5])) + A.M05();
			A.M06() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[6])) + A.M06();
			A.M07() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], j[7])) + A.M07();
			A.M11() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[1])) + A.M11();
			A.M12() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[2])) + A.M12();
			A.M13() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[3])) + A.M13();
			A.M14() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[4])) + A.M14();
			A.M15() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[5])) + A.M15();
			A.M16() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[6])) + A.M16();
			A.M17() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], j[7])) + A.M17();
			A.M22() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[2])) + A.M22();
			A.M23() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[3])) + A.M23();
			A.M24() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[4])) + A.M24();
			A.M25() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[5])) + A.M25();
			A.M26() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[6])) + A.M26();
			A.M27() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], j[7])) + A.M27();
			A.M33() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[3])) + A.M33();
			A.M34() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[4])) + A.M34();
			A.M35() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[5])) + A.M35();
			A.M36() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[6])) + A.M36();
			A.M37() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], j[7])) + A.M37();
			A.M44() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], j[4])) + A.M44();
			A.M45() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], j[5])) + A.M45();
			A.M46() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], j[6])) + A.M46();
			A.M47() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], j[7])) + A.M47();
			A.M55() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[5], j[5])) + A.M55();
			A.M56() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[5], j[6])) + A.M56();
			A.M57() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[5], j[7])) + A.M57();
			A.M66() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[6], j[6])) + A.M66();
			A.M67() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[6], j[7])) + A.M67();
			A.M77() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[7], j[7])) + A.M77();
			b.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[0], e)) + b.v0();
			b.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[1], e)) + b.v1();
			b.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[2], e)) + b.v2();
			b.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[3], e)) + b.v3();
			b.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[4], e)) + b.v4();
			b.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[5], e)) + b.v5();
			b.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[6], e)) + b.v6();
			b.v7() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(j[7], e)) + b.v7();
		}
		for(ushort iRem = 0; iRem < nRems; ++iRem)
		{
			float *work = (float *) m_work;
			float &lx = work[0], &ly = work[1], &d = work[2], &jlx = work[3], &jly = work[4], &jlz = work[5], &jlz2 = work[6], &e = work[7];
			ENFT_SSE::__m128 &j4 = m_work[20], &e4 = m_work[20];
			const Point2D &x1 = GetReminder1(iRem), &x2 = GetReminder2(iRem);
			lx = F.M00() * x1.x() + F.M01() * x1.y() + F.M02();
			ly = F.M10() * x1.x() + F.M11() * x1.y() + F.M12();
			d = x2.x() * lx + x2.y() * ly + F.M20() * x1.x() + F.M21() * x1.y() + F.M22();
			jlz2 = 1 / (lx * lx + ly * ly);
			jlz = sqrt(jlz2);
			jlx = jlz * (x2.x() - jlz2 * d * lx);
			jly = jlz * (x2.y() - jlz2 * d * ly);
			j[0].m128_f32[0] = jlx * x1.x();		j[0].m128_f32[1] = jlx * x1.y();		j[0].m128_f32[2] = jlx;
			j[0].m128_f32[3] = jly * x1.x();		j[1].m128_f32[0] = jly * x1.y();		j[1].m128_f32[1] = jly;
			j[1].m128_f32[2] = jlz * x1.x();		j[1].m128_f32[3] = jlz * x1.y();
			e = -d * jlz;
			j4 = _mm_set1_ps(j[0].m128_f32[0]);
			A.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[0]), A.M_00_01_02_03());
			A.M_04_05_06_07() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[1]), A.M_04_05_06_07());
			j4 = _mm_set1_ps(j[0].m128_f32[1]);
			A.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[0]), A.M_10_11_12_13());
			A.M_14_15_16_17() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[1]), A.M_14_15_16_17());
			j4 = _mm_set1_ps(j[0].m128_f32[2]);
			A.M22() = j[0].m128_f32[2] * j[0].m128_f32[2] + A.M22();
			A.M23() = j[0].m128_f32[2] * j[0].m128_f32[3] + A.M23();
			A.M_24_25_26_27() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[1]), A.M_24_25_26_27());
			j4 = _mm_set1_ps(j[0].m128_f32[3]);
			A.M33() = j[0].m128_f32[3] * j[0].m128_f32[3] + A.M33();
			A.M_34_35_36_37() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(j4, j[1]), A.M_34_35_36_37());
			A.M_44_45_46_47() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(j[1].m128_f32[0]), j[1]), A.M_44_45_46_47());
			A.M_54_55_56_57() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(j[1].m128_f32[1]), j[1]), A.M_54_55_56_57());
			A.M66() = j[1].m128_f32[2] * j[1].m128_f32[2] + A.M66();
			A.M67() = j[1].m128_f32[2] * j[1].m128_f32[3] + A.M67();
			A.M77() = j[1].m128_f32[3] * j[1].m128_f32[3] + A.M77();
			e4 = _mm_set1_ps(e);
			b.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e4, j[0]), b.v0123());
			b.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e4, j[1]), b.v4567());
		}
		A.SetLowerFromUpper();
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}
	virtual void UpdateModel(const LA::AlignedVector8f &s, const LA::AlignedVector8f &x, const FundamentalMatrix &Fold, FundamentalMatrix &Fnew)
	{
		m_work[0] = ENFT_SSE::_mm_mul_ps(s.v0123(), x.v0123());
		Fnew.M00() = m_work[0].m128_f32[0] + Fold.M00();
		Fnew.M01() = m_work[0].m128_f32[1] + Fold.M01();
		Fnew.M02() = m_work[0].m128_f32[2] + Fold.M02();
		Fnew.M10() = m_work[0].m128_f32[3] + Fold.M10();
		m_work[0] = ENFT_SSE::_mm_mul_ps(s.v4567(), x.v4567());
		Fnew.M11() = m_work[0].m128_f32[0] + Fold.M11();
		Fnew.M12() = m_work[0].m128_f32[1] + Fold.M12();
		Fnew.M20() = m_work[0].m128_f32[2] + Fold.M20();
		Fnew.M21() = m_work[0].m128_f32[3] + Fold.M21();
	}

protected:

	ENFT_SSE::__m128 m_work[27];
	bool m_normalized;
};

#endif