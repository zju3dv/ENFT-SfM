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
#include "Point3DSolver.h"

bool Point3DSolver::Run(const Point3DEstimatorMinimalSample &data, Point3D &X, AlignedVector<ENFT_SSE::__m128> &work, const bool metric)
{
	const Camera &C1 = data.GetFirstCamera(), &C2 = data.GetSecondCamera();
	const Point2D &x1 = data.GetFirstMeasurement(), &x2 = data.GetSecondMeasurement();

	LA::AlignedVector3f &ATb = X;
	work.Resize(4);
	ENFT_SSE::__m128 &A0 = work[0], &A1 = work[1], &ATA0 = work[2], &ATA1 = work[3];
	A0 = ENFT_SSE::_mm_sub_ps(C1.r00_r01_r02_tX(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x1.x()), C1.r20_r21_r22_tZ()));
	A1 = ENFT_SSE::_mm_sub_ps(C1.r10_r11_r12_tY(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x1.y()), C1.r20_r21_r22_tZ()));
	m_ATA.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A0.m128_f32[0]), A0), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A1.m128_f32[0]), A1));
	m_ATA.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A0.m128_f32[1]), A0), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A1.m128_f32[1]), A1));
	m_ATA.M22() = A0.m128_f32[2] * A0.m128_f32[2] + A1.m128_f32[2] * A1.m128_f32[2];
	ATb.v0() = -m_ATA.reserve0();
	ATb.v1() = -m_ATA.reserve1();
	ATb.v2() = -(A0.m128_f32[2] * A0.m128_f32[3] + A1.m128_f32[2] * A1.m128_f32[3]);

	A0 = ENFT_SSE::_mm_sub_ps(C2.r00_r01_r02_tX(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x2.x()), C2.r20_r21_r22_tZ()));
	A1 = ENFT_SSE::_mm_sub_ps(C2.r10_r11_r12_tY(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x2.y()), C2.r20_r21_r22_tZ()));
	ATA0 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A0.m128_f32[0]), A0), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A1.m128_f32[0]), A1));
	ATA1 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A0.m128_f32[1]), A0), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A1.m128_f32[1]), A1));
	m_ATA.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ATA0, m_ATA.M_00_01_02_x());
	m_ATA.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ATA1, m_ATA.M_10_11_12_x());
	m_ATA.M22() = A0.m128_f32[2] * A0.m128_f32[2] + A1.m128_f32[2] * A1.m128_f32[2] + m_ATA.M22();
	ATb.v0() -= ATA0.m128_f32[3];
	ATb.v1() -= ATA1.m128_f32[3];
	ATb.v2() -= A0.m128_f32[2] * A0.m128_f32[3] + A1.m128_f32[2] * A1.m128_f32[3];

	//printf("----------------------------------------------------------------\n");
	//m_ATA.Print();
	//printf("----------------------------------------------------------------\n");
	//ATb.Print();

	if(!LA::SolveLinearSystemSymmetricUpper(m_ATA, ATb))
		return false;
	X.reserve() = 1.0f;
	return !metric || C1.CheckCheirality(X) && C2.CheckCheirality(X);
}

bool Point3DSolver::Run(const Point3DEstimatorData &data, Point3D &X, AlignedVector<ENFT_SSE::__m128> &work, const bool metric)
{
#if _DEBUG
	static int g_cnt = 0;
	++g_cnt;
#endif
	LA::AlignedVector3f &ATb = X;
	m_ATA.SetZero();
	ATb.SetZero();
	work.Resize(4);
	ENFT_SSE::__m128 &A0 = work[0], &A1 = work[1], &ATA0 = work[2], &ATA1 = work[3];
	const ushort N = data.Size();
	for(ushort i = 0; i < N; ++i)
	{
		const Camera &C = data.GetCamera(i);
		const Point2D &x = data.x(i);
		A0 = ENFT_SSE::_mm_sub_ps(C.r00_r01_r02_tX(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.x()), C.r20_r21_r22_tZ()));
		A1 = ENFT_SSE::_mm_sub_ps(C.r10_r11_r12_tY(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.y()), C.r20_r21_r22_tZ()));
		ATA0 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A0.m128_f32[0]), A0), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A1.m128_f32[0]), A1));
		ATA1 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A0.m128_f32[1]), A0), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A1.m128_f32[1]), A1));
		m_ATA.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ATA0, m_ATA.M_00_01_02_x());
		m_ATA.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ATA1, m_ATA.M_10_11_12_x());
		m_ATA.M22() = A0.m128_f32[2] * A0.m128_f32[2] + A1.m128_f32[2] * A1.m128_f32[2] + m_ATA.M22();
		ATb.v0() -= ATA0.m128_f32[3];
		ATb.v1() -= ATA1.m128_f32[3];
		ATb.v2() -= A0.m128_f32[2] * A0.m128_f32[3] + A1.m128_f32[2] * A1.m128_f32[3];
	}
//#if _DEBUG
//	if(g_cnt == 322)
//	{
//		m_ATA.Print();
//		ATb.Print();
//	}
//	const LA::AlignedMatrix3f ATABkp = m_ATA;
//#endif
	if(!LA::SolveLinearSystemSymmetricUpper(m_ATA, ATb))
		return false;
	X.reserve() = 1.0f;
//#if _DEBUG
//	if(g_cnt == 322)
//	{
//		LA::AlignedVector3f ATbChk;
//		LA::AB(ATABkp, X, ATbChk);
//		X.Print();
//		ATbChk.Print();
//	}
//#endif

	if(!metric)
		return true;
	for(ushort i = 0; i < N; ++i)
	{
		if(!data.GetCamera(i).CheckCheirality(X))
			return false;
	}
	return true;
}