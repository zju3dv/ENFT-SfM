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
#include "ProjectiveMatrixSolver.h"

//static LA::AlignedVector11f g_x;

bool ProjectiveMatrixSolver::Run(const SixMatches3DTo2D &data, ProjectiveMatrix &P, AlignedVector<ENFT_SSE::__m128> &work)
{
	m_ATA.SetZero();
	m_ATb.SetZero();
	m_ATA.M33() = m_ATA.M77() = 6.0f;

	work.Resize(8);
	ENFT_SSE::__m128 &X_X_X_X = work[0], &Y_Y_Y_Y = work[1], &Z_Z_Z_Z = work[2], &x_x_x_x = work[3], &y_y_y_y = work[4], &xX_xY_xZ_x = work[5], &yX_yY_yZ_y = work[6], rX_rY_rZ_r = work[7];
	for(ushort i = 0; i < 6; ++i)
	{
		const Point3D &X = data.X(i);
		const Point2D &x = data.x(i);
#if _DEBUG
		assert(X.reserve() == 1.0f);
#endif
		X_X_X_X = _mm_set1_ps(X.X());
		Y_Y_Y_Y = _mm_set1_ps(X.Y());
		Z_Z_Z_Z = _mm_set1_ps(X.Z());
		x_x_x_x = _mm_set1_ps(x.x());
		y_y_y_y = _mm_set1_ps(x.y());
		xX_xY_xZ_x = ENFT_SSE::_mm_mul_ps(x_x_x_x, X.XYZx());
		yX_yY_yZ_y = ENFT_SSE::_mm_mul_ps(y_y_y_y, X.XYZx());
		rX_rY_rZ_r = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.x() * x.x() + x.y() * x.y()), X.XYZx());
		m_ATA.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(X_X_X_X, X.XYZx()), m_ATA.M_00_01_02_03());
		m_ATA.M_08_09_010_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_08_09_010_x(), ENFT_SSE::_mm_mul_ps(X_X_X_X, xX_xY_xZ_x));
		m_ATA.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(Y_Y_Y_Y, X.XYZx()), m_ATA.M_10_11_12_13());
		m_ATA.M_18_19_110_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_18_19_110_x(), ENFT_SSE::_mm_mul_ps(Y_Y_Y_Y, xX_xY_xZ_x));
		m_ATA.M22() = X.Z() * X.Z() + m_ATA.M22();
		m_ATA.M23() = X.Z() + m_ATA.M23();
		m_ATA.M_28_29_210_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_28_29_210_x(), ENFT_SSE::_mm_mul_ps(Z_Z_Z_Z, xX_xY_xZ_x));
		m_ATA.M_38_39_310_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_38_39_310_x(), xX_xY_xZ_x);
		m_ATA.M_48_49_410_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_48_49_410_x(), ENFT_SSE::_mm_mul_ps(X_X_X_X, yX_yY_yZ_y));
		m_ATA.M_58_59_510_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_58_59_510_x(), ENFT_SSE::_mm_mul_ps(Y_Y_Y_Y, yX_yY_yZ_y));
		m_ATA.M_68_69_610_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_68_69_610_x(), ENFT_SSE::_mm_mul_ps(Z_Z_Z_Z, yX_yY_yZ_y));
		m_ATA.M_78_79_710_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_78_79_710_x(), yX_yY_yZ_y);
		m_ATA.M_88_89_810_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(rX_rY_rZ_r, X_X_X_X), m_ATA.M_88_89_810_x());
		m_ATA.M99() = rX_rY_rZ_r.m128_f32[1] * X.Y() + m_ATA.M99();
		m_ATA.M910() = rX_rY_rZ_r.m128_f32[2] * X.Y() + m_ATA.M910();
		m_ATA.M1010() = rX_rY_rZ_r.m128_f32[2] * X.Z() + m_ATA.M1010();

		m_ATb.v0123() = ENFT_SSE::_mm_add_ps(xX_xY_xZ_x, m_ATb.v0123());
		m_ATb.v4567() = ENFT_SSE::_mm_add_ps(yX_yY_yZ_y, m_ATb.v4567());
		m_ATb.v8910x() = ENFT_SSE::_mm_sub_ps(m_ATb.v8910x(), rX_rY_rZ_r);
	}
	m_ATA.M_44_45_46_47() = m_ATA.M_00_01_02_03();
	m_ATA.M_54_55_56_57() = m_ATA.M_10_11_12_13();
	m_ATA.M66() = m_ATA.M22();
	m_ATA.M67() = m_ATA.M23();
//#if _DEBUG
//	//printf("----------------------------------------------------------------\n");
//	//m_ATA.Print();
//	//printf("----------------------------------------------------------------\n");
//	//m_ATb.Print();
//	printf("----------------------------------------------------------------\n");
//	const LA::AlignedMatrix11f ATABkp = m_ATA;
//	m_ATb.Print(); 
//#endif
	if(!LA::SolveLinearSystemSymmetricUpper(m_ATA, m_ATb))
		return false;
//#if _DEBUG
//	printf("----------------------------------------------------------------\n");
//	m_ATb.reserve() = 0.0f;
//	LA::AlignedVector11f ATbChk;
//	LA::AB(ATABkp, m_ATb, ATbChk);
//	ATbChk.Print();
//#endif
	memcpy(P, m_ATb, 44);
	P.M23() = 1;
//#if _DEBUG
////	//printf("----------------------------------------------------------------\n");
////	//P.Print();
//	g_x = m_ATb;
////	//ProjectiveMatrixEstimatorData test;
////	//test.Resize(6);
////	//for(ushort i = 0; i < 6; ++i)
////	//{
////	//	test.X(i) = data.X(i);
////	//	test.x(i) = data.x(i);
////	//}
////	//Run(test, P, work);
//#endif
	return true;
}

bool ProjectiveMatrixSolver::Run(const ProjectiveMatrixEstimatorData &data, ProjectiveMatrix &P, AlignedVector<ENFT_SSE::__m128> &work)
{
	m_ATA.SetZero();
	m_ATb.SetZero();
	const ushort N = data.Size();
	m_ATA.M33() = m_ATA.M77() = float(N);

//#if _DEBUG
//	Point2D e;
//	memcpy(P, g_x, 44);
//	P.M23() = 1;
//#endif

	work.Resize(8);
	ENFT_SSE::__m128 &X_X_X_X = work[0], &Y_Y_Y_Y = work[1], &Z_Z_Z_Z = work[2], &x_x_x_x = work[3], &y_y_y_y = work[4], &xX_xY_xZ_x = work[5], &yX_yY_yZ_y = work[6], rX_rY_rZ_r = work[7];
	for(ushort i = 0; i < N; ++i)
	{
		const Point3D &X = data.X(i);
		const Point2D &x = data.x(i);
#if _DEBUG
		assert(X.reserve() == 1.0f);
#endif
		X_X_X_X = _mm_set1_ps(X.X());
		Y_Y_Y_Y = _mm_set1_ps(X.Y());
		Z_Z_Z_Z = _mm_set1_ps(X.Z());
		x_x_x_x = _mm_set1_ps(x.x());
		y_y_y_y = _mm_set1_ps(x.y());
		xX_xY_xZ_x = ENFT_SSE::_mm_mul_ps(x_x_x_x, X.XYZx());
		yX_yY_yZ_y = ENFT_SSE::_mm_mul_ps(y_y_y_y, X.XYZx());
		rX_rY_rZ_r = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.x() * x.x() + x.y() * x.y()), X.XYZx());
		m_ATA.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(X_X_X_X, X.XYZx()), m_ATA.M_00_01_02_03());
		m_ATA.M_08_09_010_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_08_09_010_x(), ENFT_SSE::_mm_mul_ps(X_X_X_X, xX_xY_xZ_x));
		m_ATA.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(Y_Y_Y_Y, X.XYZx()), m_ATA.M_10_11_12_13());
		m_ATA.M_18_19_110_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_18_19_110_x(), ENFT_SSE::_mm_mul_ps(Y_Y_Y_Y, xX_xY_xZ_x));
		m_ATA.M22() = X.Z() * X.Z() + m_ATA.M22();
		m_ATA.M23() = X.Z() + m_ATA.M23();
		m_ATA.M_28_29_210_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_28_29_210_x(), ENFT_SSE::_mm_mul_ps(Z_Z_Z_Z, xX_xY_xZ_x));
		m_ATA.M_38_39_310_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_38_39_310_x(), xX_xY_xZ_x);
		m_ATA.M_48_49_410_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_48_49_410_x(), ENFT_SSE::_mm_mul_ps(X_X_X_X, yX_yY_yZ_y));
		m_ATA.M_58_59_510_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_58_59_510_x(), ENFT_SSE::_mm_mul_ps(Y_Y_Y_Y, yX_yY_yZ_y));
		m_ATA.M_68_69_610_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_68_69_610_x(), ENFT_SSE::_mm_mul_ps(Z_Z_Z_Z, yX_yY_yZ_y));
		m_ATA.M_78_79_710_x() = ENFT_SSE::_mm_sub_ps(m_ATA.M_78_79_710_x(), yX_yY_yZ_y);
		m_ATA.M_88_89_810_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(rX_rY_rZ_r, X_X_X_X), m_ATA.M_88_89_810_x());
		m_ATA.M99() = rX_rY_rZ_r.m128_f32[1] * X.Y() + m_ATA.M99();
		m_ATA.M910() = rX_rY_rZ_r.m128_f32[2] * X.Y() + m_ATA.M910();
		m_ATA.M1010() = rX_rY_rZ_r.m128_f32[2] * X.Z() + m_ATA.M1010();

		m_ATb.v0123() = ENFT_SSE::_mm_add_ps(xX_xY_xZ_x, m_ATb.v0123());
		m_ATb.v4567() = ENFT_SSE::_mm_add_ps(yX_yY_yZ_y, m_ATb.v4567());
		m_ATb.v8910x() = ENFT_SSE::_mm_sub_ps(m_ATb.v8910x(), rX_rY_rZ_r);
	}
	m_ATA.M_44_45_46_47() = m_ATA.M_00_01_02_03();
	m_ATA.M_54_55_56_57() = m_ATA.M_10_11_12_13();
	m_ATA.M66() = m_ATA.M22();
	m_ATA.M67() = m_ATA.M23();
//#if _DEBUG
//	m_ATA.SetLowerFromUpper();
//	//printf("----------------------------------------------------------------\n");
//	//m_ATA.Print();
//	printf("----------------------------------------------------------------\n");
//	m_ATb.Print();
//	printf("----------------------------------------------------------------\n");
//	LA::AlignedVector11f ATbChk;
//	//memcpy(g_x, P, 44);
//	//g_x *= _mm_set1_ps(1 / P.M23());
//	g_x.reserve() = 0.0f;
//	LA::AB(m_ATA, g_x, ATbChk);
//	ATbChk.Print();
//	//const LA::AlignedMatrix11f ATABkp = m_ATA;
//#endif
	if(!LA::SolveLinearSystemSymmetricUpper(m_ATA, m_ATb))
		return false;
//#if _DEBUG
//	//printf("----------------------------------------------------------------\n");
//	//m_ATb.reserve() = 0.0f;
//	//LA::AB(ATABkp, m_ATb, ATbChk);
//	//ATbChk.Print();
//	//printf("----------------------------------------------------------------\n");
//	//LA::AlignedVector11f xChk;
//	//LA::AmB(m_ATb, g_x, xChk);
//	//xChk.Print();
//	//P.Print();
//	printf("----------------------------------------------------------------\n");
//	g_x.Print();
//	printf("----------------------------------------------------------------\n");
//	m_ATb.Print();
//#endif
	memcpy(P, m_ATb, 44);
	P.M23() = 1;
//#if _DEBUG
//	printf("----------------------------------------------------------------\n");
//	P.Print();
//#endif
	return true;
}