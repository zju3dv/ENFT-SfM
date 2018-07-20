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

#include "stdafx.h"
#include "TranslationSolver.h"
#include "LinearAlgebra/LinearSystem.h"

bool TranslationSolver::Run(const TwoMatches3DTo2D &data, Translation &T, AlignedVector<ENFT_SSE::__m128> &work)
{
	//const Translation Tbkp = T;
	LA::AlignedVector3f &ATb = T;
	m_ATA.SetZero();
	ATb.SetZero();
	m_ATA.M00() = m_ATA.M11() = 2.0f;

	work.Resize(2);
	ENFT_SSE::__m128 &ATb01 = work[0], &ATb2 = work[1];
	const ENFT_SSE::__m128 *px = (ENFT_SSE::__m128 *) data.xs();
	m_ATA.M02() -= px->m128_f32[0] + px->m128_f32[2];
	m_ATA.M12() -= px->m128_f32[1] + px->m128_f32[3];
	m_ATA.M22() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(*px, *px)) + m_ATA.M22();

	const Point3D &X1 = data.X(0), &X2 = data.X(1);
	ATb01.m128_f32[0] = px->m128_f32[0] * X1.Z() - X1.X();
	ATb01.m128_f32[1] = px->m128_f32[1] * X1.Z() - X1.Y();
	ATb01.m128_f32[2] = px->m128_f32[2] * X2.Z() - X2.X();
	ATb01.m128_f32[3] = px->m128_f32[3] * X2.Z() - X2.Y();
	ATb2 = ENFT_SSE::_mm_mul_ps(*px, ATb01);
	ATb.v0() = ATb01.m128_f32[0] + ATb01.m128_f32[2] + ATb.v0();
	ATb.v1() = ATb01.m128_f32[1] + ATb01.m128_f32[3] + ATb.v1();
	ATb.v2() -= ENFT_SSE::SSE::Sum0123(ATb2);

	//LA::AlignedVector3f ATbChk, e;
	//LA::AB(m_ATA, Tbkp, ATbChk);
	//LA::AmB(ATb, ATbChk, e);
	//e.Print();

	return LA::SolveLinearSystemSymmetricUpper(m_ATA, ATb);
}

bool TranslationSolver::Run(const MatchSet3DTo2DX &data, Translation &T, AlignedVector<ENFT_SSE::__m128> &work)
{
	//const Translation Tbkp = T;
	LA::AlignedVector3f &ATb = T;
	const ushort N = data.Size();
	m_ATA.SetZero();
	ATb.SetZero();
	m_ATA.M00() = m_ATA.M11() = float(N);

	ushort i;
	const ENFT_SSE::__m128 *px;
	work.Resize(2);
	ENFT_SSE::__m128 &ATb01 = work[0], &ATb2 = work[1];
	const ushort _N = N - (N & 1);
	for(i = 0, px = (ENFT_SSE::__m128 *) data.xs().Data(); i < _N; i += 2, ++px)
	{
		m_ATA.M02() -= px->m128_f32[0] + px->m128_f32[2];
		m_ATA.M12() -= px->m128_f32[1] + px->m128_f32[3];
		m_ATA.M22() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(*px, *px)) + m_ATA.M22();

		const Point3D &RX1 = data.X(i), &RX2 = data.X(i + 1);
		ATb01.m128_f32[0] = px->m128_f32[0] * RX1.Z() - RX1.X();
		ATb01.m128_f32[1] = px->m128_f32[1] * RX1.Z() - RX1.Y();
		ATb01.m128_f32[2] = px->m128_f32[2] * RX2.Z() - RX2.X();
		ATb01.m128_f32[3] = px->m128_f32[3] * RX2.Z() - RX2.Y();
		ATb2 = ENFT_SSE::_mm_mul_ps(*px, ATb01);
		ATb.v0() = ATb01.m128_f32[0] + ATb01.m128_f32[2] + ATb.v0();
		ATb.v1() = ATb01.m128_f32[1] + ATb01.m128_f32[3] + ATb.v1();
		ATb.v2() -= ENFT_SSE::SSE::Sum0123(ATb2);
	}
	if(_N != N)
	{
		const Point2D &x = data.x(_N);
		m_ATA.M02() -= x.x();
		m_ATA.M12() -= x.y();
		m_ATA.M22() = x.x() * x.x() + x.y() * x.y() + m_ATA.M22();

		const Point3D &RX = data.X(_N);
		ATb01.m128_f32[0] = x.x() * RX.Z() - RX.X();
		ATb01.m128_f32[1] = x.y() * RX.Z() - RX.Y();
		ATb.v0() = ATb01.m128_f32[0] + ATb.v0();
		ATb.v1() = ATb01.m128_f32[1] + ATb.v1();
		ATb.v2() -= x.x() * ATb01.m128_f32[0] + x.y() * ATb01.m128_f32[1];
	}

	//LA::AlignedVector3f ATbChk, e;
	//LA::AB(m_ATA, Tbkp, ATbChk);
	//LA::AmB(ATb, ATbChk, e);
	//e.Print();

	return LA::SolveLinearSystemSymmetricUpper(m_ATA, ATb);
}

void TranslationSolver::Run(const RotationTransformation3D &R, const AlignedVector<Point3D> &X1s, const AlignedVector<Point3D> &X2s, LA::AlignedVector3f &t)
{
	Point3D mean1, mean2;
	mean1 = ENFT_SSE::_mm_setzero_ps();
	const ushort N = ushort(X1s.Size());
	for(ushort i = 0; i < N; ++i)
		mean1.XYZx() = ENFT_SSE::_mm_add_ps(X1s[i].XYZx(), mean1.XYZx());
	const ENFT_SSE::__m128 one_over_N = _mm_set1_ps(1.0f / N);
	mean1.XYZx() = ENFT_SSE::_mm_mul_ps(mean1.XYZx(), one_over_N);
	mean2.XYZx() = ENFT_SSE::_mm_setzero_ps();
	for(ushort i = 0; i < N; ++i)
		mean2.XYZx() = ENFT_SSE::_mm_add_ps(X2s[i].XYZx(), mean2.XYZx());
	mean2 = ENFT_SSE::_mm_mul_ps(mean2.XYZx(), one_over_N);

	t.v0() = mean2.X() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(R.r_00_01_02_x(), mean1.XYZx()));
	t.v1() = mean2.Y() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(R.r_10_11_12_x(), mean1.XYZx()));
	t.v2() = mean2.Z() - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(R.r_20_21_22_x(), mean1.XYZx()));
}