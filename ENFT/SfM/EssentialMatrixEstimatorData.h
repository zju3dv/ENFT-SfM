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

#ifndef _ESSENTIAL_MATRIX_ESTIMATOR_DATA_H_
#define _ESSENTIAL_MATRIX_ESTIMATOR_DATA_H_

#include "EssentialMatrix.h"
#include "SfM/Match.h"
#include "Estimation/EstimatorArsacData.h"
#include "Optimization/OptimizerData.h"
#include "LinearAlgebra/Matrix6.h"
#include "LinearAlgebra/Matrix8.h"

class EssentialMatrixEstimatorData : public EstimatorArsacData, public MatchSet2D, public OptimizerDataTemplate<RigidTransformation3D, LA::AlignedVector6f, LA::AlignedCompactMatrix6f>
{

public:

	inline void SetFocal(const float &fxy) { m_fxy = fxy; }

	virtual void NormalizeData(const float &dataNormalizeMedian, RigidTransformation3D &T) {}
	virtual void DenormalizeData(RigidTransformation3D &T) {}
	virtual double ComputeSSE(const RigidTransformation3D &T)
	{
		ENFT_SSE::__m128 *f = m_work;
		m_E.FromRelativePose(T);
		m_E.GetSSE(f);

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
			sum.m128_f32[0] = m_E.ComputeEpipolarSquaredError(GetReminder1(iRem), GetReminder2(iRem), work) + sum.m128_f32[0];
		}
		return double(ENFT_SSE::SSE::Sum0123(sum));
	}
	virtual double GetFactorSSEToMSE() { return double(m_fxy / Size()); }
	virtual void ConstructNormalEquation(const RigidTransformation3D &T, LA::AlignedCompactMatrix6f &A, LA::AlignedVector6f &b, LA::AlignedVector6f &s)
	{
		ENFT_SSE::__m128 *f = m_work;
		m_E.FromRelativePose(T);
		m_E.GetSSE(f);

		m_JetT.M00() = 0.0f;		m_JetT.M10() = -m_E.M02();	m_JetT.M20() = m_E.M01();
		m_JetT.M01() = m_E.M02();	m_JetT.M11() = -0.0f;		m_JetT.M21() = -m_E.M00();
		m_JetT.M02() = -m_E.M01();	m_JetT.M12() = m_E.M00();	m_JetT.M22() = 0.0f;
		m_JetT.M03() = 0.0f;		m_JetT.M13() = -m_E.M12();	m_JetT.M23() = m_E.M11();
		m_JetT.M04() = m_E.M12();	m_JetT.M14() = -0.0f;		m_JetT.M24() = -m_E.M10();
		m_JetT.M05() = -m_E.M11();	m_JetT.M15() = m_E.M10();	m_JetT.M25() = 0.0f;
		m_JetT.M06() = 0.0f;		m_JetT.M16() = -m_E.M22();	m_JetT.M26() = m_E.M21();
		m_JetT.M07() = m_E.M22();	m_JetT.M17() = -0.0f;		m_JetT.M27() = -m_E.M20();

		m_JetT.M30() = 0.0f;		m_JetT.M40() = T.r20();		m_JetT.M50() = -T.r10();
		m_JetT.M31() = 0.0f;		m_JetT.M41() = T.r21();		m_JetT.M51() = -T.r11();
		m_JetT.M32() = 0.0f;		m_JetT.M42() = T.r22();		m_JetT.M52() = -T.r12();
		m_JetT.M33() = -T.r20();	m_JetT.M43() = 0.0f;		m_JetT.M53() = T.r00();
		m_JetT.M34() = -T.r21();	m_JetT.M44() = 0.0f;		m_JetT.M54() = T.r01();
		m_JetT.M35() = -T.r22();	m_JetT.M45() = 0.0f;		m_JetT.M55() = T.r02();
		m_JetT.M36() = T.r10();		m_JetT.M46() = -T.r00();	m_JetT.M56() = 0.0f;
		m_JetT.M37() = T.r11();		m_JetT.M47() = -T.r01();	m_JetT.M57() = 0.0f;

		ENFT_SSE::__m128 &lx = m_work[9], &ly = m_work[10], &d = m_work[11], &jlx = m_work[12], &jly = m_work[13], &jlz = m_work[14], &jlz2 = m_work[15];
		ENFT_SSE::__m128 &one = m_work[16], &zero = m_work[17], *je = &m_work[18], &e = m_work[26];
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
			je[0] = ENFT_SSE::_mm_mul_ps(jlx, u1);		je[1] = ENFT_SSE::_mm_mul_ps(jlx, v1);		je[2] = jlx;
			je[3] = ENFT_SSE::_mm_mul_ps(jly, u1);		je[4] = ENFT_SSE::_mm_mul_ps(jly, v1);		je[5] = jly;
			je[6] = ENFT_SSE::_mm_mul_ps(jlz, u1);		je[7] = ENFT_SSE::_mm_mul_ps(jlz, v1);
			e = ENFT_SSE::_mm_sub_ps(zero, ENFT_SSE::_mm_mul_ps(d, jlz));
			for(ushort j = 0; j < 4; ++j)
			{
				m_Je.v0123() = _mm_setr_ps(je[0].m128_f32[j], je[1].m128_f32[j], je[2].m128_f32[j], je[3].m128_f32[j]);
				m_Je.v4567() = _mm_setr_ps(je[4].m128_f32[j], je[5].m128_f32[j], je[6].m128_f32[j], je[7].m128_f32[j]);
				LA::ATBT(m_Je, m_JetT, m_Jt);
				LA::AddAATToUpper(m_Jt, A);
				LA::AddATBTo(m_Jt, e.m128_f32[j], b);
				LA::AddAij2To(m_Jt, s);
			}
		}
		for(ushort iRem = 0; iRem < nRems; ++iRem)
		{
			float *work = (float *) m_work;
			float &lx = work[0], &ly = work[1], &d = work[2], &jlx = work[3], &jly = work[4], &jlz = work[5], &jlz2 = work[6], &e = work[7];
			const Point2D &x1 = GetReminder1(iRem), &x2 = GetReminder2(iRem);
			lx = m_E.M00() * x1.x() + m_E.M01() * x1.y() + m_E.M02();
			ly = m_E.M10() * x1.x() + m_E.M11() * x1.y() + m_E.M12();
			d = x2.x() * lx + x2.y() * ly + m_E.M20() * x1.x() + m_E.M21() * x1.y() + m_E.M22();
			jlz2 = 1 / (lx * lx + ly * ly);
			jlz = sqrt(jlz2);
			jlx = jlz * (x2.x() - jlz2 * d * lx);
			jly = jlz * (x2.y() - jlz2 * d * ly);
			m_Je.v0() = jlx * x1.x();		m_Je.v1() = jlx * x1.y();		m_Je.v2() = jlx;
			m_Je.v3() = jly * x1.x();		m_Je.v4() = jly * x1.y();		m_Je.v5() = jly;
			m_Je.v6() = jlz * x1.x();		m_Je.v7() = jlz * x1.y();
			e = -d * jlz;
			LA::ATBT(m_Je, m_JetT, m_Jt);
			LA::AddAATToUpper(m_Jt, A);
			LA::AddATBTo(m_Jt, e, b);
			LA::AddAij2To(m_Jt, s);
		}
		LA::FinishAdditionATAToUpper<LA::AlignedVector6f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedVector6f, LA::Vector2f>(b);
		LA::FinishAdditionAij2To<LA::AlignedVector6f>(s);
		LA::SetReserve<2>(b);
		LA::SetReserve<1>(s);
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ssTA(s, A, m_work);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const RigidTransformation3D &T, const LA::AlignedVector6f &s, LA::AlignedCompactMatrix6f &A, LA::AlignedVector6f &b)
	{
		ENFT_SSE::__m128 *f = m_work;
		m_E.FromRelativePose(T);
		m_E.GetSSE(f);

		m_JetT.M00() = 0.0f;		m_JetT.M10() = -m_E.M02();	m_JetT.M20() = m_E.M01();
		m_JetT.M01() = m_E.M02();	m_JetT.M11() = -0.0f;		m_JetT.M21() = -m_E.M00();
		m_JetT.M02() = -m_E.M01();	m_JetT.M12() = m_E.M00();	m_JetT.M22() = 0.0f;
		m_JetT.M03() = 0.0f;		m_JetT.M13() = -m_E.M12();	m_JetT.M23() = m_E.M11();
		m_JetT.M04() = m_E.M12();	m_JetT.M14() = -0.0f;		m_JetT.M24() = -m_E.M10();
		m_JetT.M05() = -m_E.M11();	m_JetT.M15() = m_E.M10();	m_JetT.M25() = 0.0f;
		m_JetT.M06() = 0.0f;		m_JetT.M16() = -m_E.M22();	m_JetT.M26() = m_E.M21();
		m_JetT.M07() = m_E.M22();	m_JetT.M17() = -0.0f;		m_JetT.M27() = -m_E.M20();

		m_JetT.M30() = 0.0f;		m_JetT.M40() = T.r20();		m_JetT.M50() = -T.r10();
		m_JetT.M31() = 0.0f;		m_JetT.M41() = T.r21();		m_JetT.M51() = -T.r11();
		m_JetT.M32() = 0.0f;		m_JetT.M42() = T.r22();		m_JetT.M52() = -T.r12();
		m_JetT.M33() = -T.r20();	m_JetT.M43() = 0.0f;		m_JetT.M53() = T.r00();
		m_JetT.M34() = -T.r21();	m_JetT.M44() = 0.0f;		m_JetT.M54() = T.r01();
		m_JetT.M35() = -T.r22();	m_JetT.M45() = 0.0f;		m_JetT.M55() = T.r02();
		m_JetT.M36() = T.r10();		m_JetT.M46() = -T.r00();	m_JetT.M56() = 0.0f;
		m_JetT.M37() = T.r11();		m_JetT.M47() = -T.r01();	m_JetT.M57() = 0.0f;

		ENFT_SSE::__m128 &lx = m_work[9], &ly = m_work[10], &d = m_work[11], &jlx = m_work[12], &jly = m_work[13], &jlz = m_work[14], &jlz2 = m_work[15];
		ENFT_SSE::__m128 &one = m_work[16], &zero = m_work[17], *je = &m_work[18], &e = m_work[26];
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
			je[0] = ENFT_SSE::_mm_mul_ps(jlx, u1);		je[1] = ENFT_SSE::_mm_mul_ps(jlx, v1);		je[2] = jlx;
			je[3] = ENFT_SSE::_mm_mul_ps(jly, u1);		je[4] = ENFT_SSE::_mm_mul_ps(jly, v1);		je[5] = jly;
			je[6] = ENFT_SSE::_mm_mul_ps(jlz, u1);		je[7] = ENFT_SSE::_mm_mul_ps(jlz, v1);
			e = ENFT_SSE::_mm_sub_ps(zero, ENFT_SSE::_mm_mul_ps(d, jlz));
			for(ushort j = 0; j < 4; ++j)
			{
				m_Je.v0123() = _mm_setr_ps(je[0].m128_f32[j], je[1].m128_f32[j], je[2].m128_f32[j], je[3].m128_f32[j]);
				m_Je.v4567() = _mm_setr_ps(je[4].m128_f32[j], je[5].m128_f32[j], je[6].m128_f32[j], je[7].m128_f32[j]);
				LA::ATBT(m_Je, m_JetT, m_Jt);
				LA::AddAATToUpper(m_Jt, A);
				LA::AddATBTo(m_Jt, e.m128_f32[j], b);
			}
		}
		for(ushort iRem = 0; iRem < nRems; ++iRem)
		{
			float *work = (float *) m_work;
			float &lx = work[0], &ly = work[1], &d = work[2], &jlx = work[3], &jly = work[4], &jlz = work[5], &jlz2 = work[6], &e = work[7];
			const Point2D &x1 = GetReminder1(iRem), &x2 = GetReminder2(iRem);
			lx = m_E.M00() * x1.x() + m_E.M01() * x1.y() + m_E.M02();
			ly = m_E.M10() * x1.x() + m_E.M11() * x1.y() + m_E.M12();
			d = x2.x() * lx + x2.y() * ly + m_E.M20() * x1.x() + m_E.M21() * x1.y() + m_E.M22();
			jlz2 = 1 / (lx * lx + ly * ly);
			jlz = sqrt(jlz2);
			jlx = jlz * (x2.x() - jlz2 * d * lx);
			jly = jlz * (x2.y() - jlz2 * d * ly);
			m_Je.v0() = jlx * x1.x();		m_Je.v1() = jlx * x1.y();		m_Je.v2() = jlx;
			m_Je.v3() = jly * x1.x();		m_Je.v4() = jly * x1.y();		m_Je.v5() = jly;
			m_Je.v6() = jlz * x1.x();		m_Je.v7() = jlz * x1.y();
			e = -d * jlz;
			LA::ATBT(m_Je, m_JetT, m_Jt);
			LA::AddAATToUpper(m_Jt, A);
			LA::AddATBTo(m_Jt, e, b);
		}
		LA::FinishAdditionATAToUpper<LA::AlignedVector6f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedVector6f, LA::Vector2f>(b);
		LA::SetReserve<2>(b);
		LA::ssTA(s, A, m_work);
		LA::sA(s, b);
	}
	virtual void UpdateModel(const LA::AlignedVector6f &s, const LA::AlignedVector6f &x, const RigidTransformation3D &Told, RigidTransformation3D &Tnew)
	{
		m_work[0] = ENFT_SSE::_mm_mul_ps(s.v0123(), x.v0123());
		m_t.X() = m_work[0].m128_f32[3] + Told.tX();
		m_t.Y() = s.v4() * x.v4()		+ Told.tY();
		m_t.Z() = s.v5() * x.v5()		+ Told.tZ();
		m_t.Normalize();
		Tnew.SetTranslation(m_t);
		m_dR.FromRodrigues(m_work[0].m128_f32[0], m_work[0].m128_f32[1], m_work[0].m128_f32[2], (float *) &m_work[1]);
		Told.LeftMultiplyRotation(m_dR, Tnew, m_work[0]);
	}

protected:

	float m_fxy;
	EssentialMatrix m_E;
	RotationTransformation3D m_dR;
	LA::AlignedMatrix6x8f m_JetT;
	LA::AlignedVector8f m_Je;
	LA::AlignedVector6f m_Jt;
	Point3D m_t;
	ENFT_SSE::__m128 m_work[27];
};

#endif