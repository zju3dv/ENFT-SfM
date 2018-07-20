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
#include "SimilarityTransformationSolver.h"

bool SimilarityTransformationSolver::Run(const ThreeMatches3D &data, SimilarityTransformation3D &S, AlignedVector<ENFT_SSE::__m128> &work)
{
	ComputeMeanScaleAndCovariance(data, work);
	if(!RecoverRotation(S, work))
		return false;
	S.sss1() = m_sss1;
	work[0] = _mm_set1_ps(1.0f / S.s());
	work[0] = ENFT_SSE::_mm_mul_ps(m_mean2.XYZx(), work[0]);
	S.tX() = work[0].m128_f32[0] - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(S.M_00_01_02_x(), m_mean1.XYZx()));
	S.tY() = work[0].m128_f32[1] - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(S.M_10_11_12_x(), m_mean1.XYZx()));
	S.tZ() = work[0].m128_f32[2] - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(S.M_20_21_22_x(), m_mean1.XYZx()));
	return true;
}

bool SimilarityTransformationSolver::Run(const FourMatches3D &data, SimilarityTransformation3D &S, AlignedVector<ENFT_SSE::__m128> &work)
{
	//AlignedVector<Point3D> X1s(4), X2s(4);
	//for(ushort i = 0; i < 4; ++i)
	//{
	//	X1s[i] = data.X1(i);
	//	X2s[i] = data.X2(i);
	//}
	//return Run(X1s, X2s, S, work);

	ComputeMeanScaleAndCovariance(data, work);
	if(!RecoverRotation(S, work))
		return false;
	S.sss1() = m_sss1;
	work[0] = _mm_set1_ps(1.0f / S.s());
	work[0] = ENFT_SSE::_mm_mul_ps(m_mean2.XYZx(), work[0]);
	S.tX() = work[0].m128_f32[0] - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(S.M_00_01_02_x(), m_mean1.XYZx()));
	S.tY() = work[0].m128_f32[1] - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(S.M_10_11_12_x(), m_mean1.XYZx()));
	S.tZ() = work[0].m128_f32[2] - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(S.M_20_21_22_x(), m_mean1.XYZx()));
	return true;
}

bool SimilarityTransformationSolver::Run(const AlignedVector<Point3D> &X1s, const AlignedVector<Point3D> &X2s, SimilarityTransformation3D &S, AlignedVector<ENFT_SSE::__m128> &work)
{
	ComputeMeanScaleAndCovariance(X1s, X2s, work);
	if(!RecoverRotation(S, work))
		return false;
	S.sss1() = m_sss1;
	work[0] = _mm_set1_ps(1.0f / S.s());
	work[0] = ENFT_SSE::_mm_mul_ps(m_mean2.XYZx(), work[0]);
	S.tX() = work[0].m128_f32[0] - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(S.M_00_01_02_x(), m_mean1.XYZx()));
	S.tY() = work[0].m128_f32[1] - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(S.M_10_11_12_x(), m_mean1.XYZx()));
	S.tZ() = work[0].m128_f32[2] - ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(S.M_20_21_22_x(), m_mean1.XYZx()));
	return true;
}

void SimilarityTransformationSolver::ComputeMeanScaleAndCovariance(const ThreeMatches3D &data, AlignedVector<ENFT_SSE::__m128> &work)
{
#if _DEBUG
	assert(data.X1(0).reserve() == 1 && data.X1(1).reserve() == 1 && data.X1(2).reserve() == 1);
#endif

	work.Resize(8);
	work[0] = _mm_set1_ps(1.0f / 3);
	m_mean1.XYZx() = ENFT_SSE::_mm_mul_ps(_mm_add_ps(data.X1(0).XYZx(), ENFT_SSE::_mm_add_ps(data.X1(1).XYZx(), data.X1(2).XYZx())), work[0]);
	m_mean2.XYZx() = ENFT_SSE::_mm_mul_ps(_mm_add_ps(data.X2(0).XYZx(), ENFT_SSE::_mm_add_ps(data.X2(1).XYZx(), data.X2(2).XYZx())), work[0]);

	ENFT_SSE::__m128 &distSq1 = work[0], &distSq2 = work[1];
	ENFT_SSE::__m128 *dX1s = work.Data() + 2, *dX2s = work.Data() + 5;
	dX1s[0] = ENFT_SSE::_mm_sub_ps(data.X1(0).XYZx(), m_mean1.XYZx());	dX1s[1] = ENFT_SSE::_mm_sub_ps(data.X1(1).XYZx(), m_mean1.XYZx());	dX1s[2] = ENFT_SSE::_mm_sub_ps(data.X1(2).XYZx(), m_mean1.XYZx());
	distSq1 = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(dX1s[0], dX1s[0]), ENFT_SSE::_mm_mul_ps(dX1s[1], dX1s[1])), ENFT_SSE::_mm_mul_ps(dX1s[2], dX1s[2]));
	dX2s[0] = ENFT_SSE::_mm_sub_ps(data.X2(0).XYZx(), m_mean2.XYZx());	dX2s[1] = ENFT_SSE::_mm_sub_ps(data.X2(1).XYZx(), m_mean2.XYZx());	dX2s[2] = ENFT_SSE::_mm_sub_ps(data.X2(2).XYZx(), m_mean2.XYZx());
	distSq2 = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(dX2s[0], dX2s[0]), ENFT_SSE::_mm_mul_ps(dX2s[1], dX2s[1])), ENFT_SSE::_mm_mul_ps(dX2s[2], dX2s[2]));
	//ENFT_SSE::SSE::Print(distSq1);
	//ENFT_SSE::SSE::Print(distSq2);
	const float s = sqrt(ENFT_SSE::SSE::Sum012(distSq2) / ENFT_SSE::SSE::Sum012(distSq1));
	//const float s = 1.0f;
	m_sss1 = _mm_setr_ps(s, s, s, 1.0f);

	ENFT_SSE::__m128 *nX1s = dX1s, *nX2s = dX2s;
	nX1s[0] = ENFT_SSE::_mm_mul_ps(m_sss1, dX1s[0]);		nX1s[1] = ENFT_SSE::_mm_mul_ps(m_sss1, dX1s[1]);		nX1s[2] = ENFT_SSE::_mm_mul_ps(m_sss1, dX1s[2]);
	m_C.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[0], _mm_set1_ps(nX2s[0].m128_f32[0])), 
						 ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[1], _mm_set1_ps(nX2s[1].m128_f32[0])), ENFT_SSE::_mm_mul_ps(nX1s[2], _mm_set1_ps(nX2s[2].m128_f32[0]))));
	m_C.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[0], _mm_set1_ps(nX2s[0].m128_f32[1])), 
						 ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[1], _mm_set1_ps(nX2s[1].m128_f32[1])), ENFT_SSE::_mm_mul_ps(nX1s[2], _mm_set1_ps(nX2s[2].m128_f32[1]))));
	m_C.M_20_21_22_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[0], _mm_set1_ps(nX2s[0].m128_f32[2])), 
						 ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[1], _mm_set1_ps(nX2s[1].m128_f32[2])), ENFT_SSE::_mm_mul_ps(nX1s[2], _mm_set1_ps(nX2s[2].m128_f32[2]))));
	//m_C.Print();
}

void SimilarityTransformationSolver::ComputeMeanScaleAndCovariance(const FourMatches3D &data, AlignedVector<ENFT_SSE::__m128> &work)
{
#if _DEBUG
	assert(data.X1(0).reserve() == 1 && data.X1(1).reserve() == 1 && data.X1(2).reserve() == 1 && data.X1(3).reserve() == 1);
#endif

	work.Resize(10);
	work[0] = _mm_set1_ps(0.25f);
	m_mean1.XYZx() = ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(data.X1(0).XYZx(), data.X1(1).XYZx()), ENFT_SSE::_mm_add_ps(data.X1(2).XYZx(), data.X1(3).XYZx())), work[0]);
	m_mean2.XYZx() = ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(data.X2(0).XYZx(), data.X2(1).XYZx()), ENFT_SSE::_mm_add_ps(data.X2(2).XYZx(), data.X2(3).XYZx())), work[0]);

	ENFT_SSE::__m128 &distSq1 = work[0], &distSq2 = work[1];
	ENFT_SSE::__m128 *dX1s = work.Data() + 2, *dX2s = work.Data() + 6;
	dX1s[0] = ENFT_SSE::_mm_sub_ps(data.X1(0).XYZx(), m_mean1.XYZx());	dX1s[1] = ENFT_SSE::_mm_sub_ps(data.X1(1).XYZx(), m_mean1.XYZx());
	dX1s[2] = ENFT_SSE::_mm_sub_ps(data.X1(2).XYZx(), m_mean1.XYZx());	dX1s[3] = ENFT_SSE::_mm_sub_ps(data.X1(3).XYZx(), m_mean1.XYZx());
	distSq1 = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(dX1s[0], dX1s[0]), ENFT_SSE::_mm_mul_ps(dX1s[1], dX1s[1])), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(dX1s[2], dX1s[2]), ENFT_SSE::_mm_mul_ps(dX1s[3], dX1s[3])));
	dX2s[0] = ENFT_SSE::_mm_sub_ps(data.X2(0).XYZx(), m_mean2.XYZx());	dX2s[1] = ENFT_SSE::_mm_sub_ps(data.X2(1).XYZx(), m_mean2.XYZx());	
	dX2s[2] = ENFT_SSE::_mm_sub_ps(data.X2(2).XYZx(), m_mean2.XYZx());	dX2s[3] = ENFT_SSE::_mm_sub_ps(data.X2(3).XYZx(), m_mean2.XYZx());
	distSq2 = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(dX2s[0], dX2s[0]), ENFT_SSE::_mm_mul_ps(dX2s[1], dX2s[1])), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(dX2s[2], dX2s[2]), ENFT_SSE::_mm_mul_ps(dX2s[3], dX2s[3])));
	//ENFT_SSE::SSE::Print(distSq1);
	//ENFT_SSE::SSE::Print(distSq2);
	const float s = sqrt(ENFT_SSE::SSE::Sum012(distSq2) / ENFT_SSE::SSE::Sum012(distSq1));
	//const float s = 1.0f;
	m_sss1 = _mm_setr_ps(s, s, s, 1.0f);

	ENFT_SSE::__m128 *nX1s = dX1s, *nX2s = dX2s;
	nX1s[0] = ENFT_SSE::_mm_mul_ps(m_sss1, dX1s[0]);		nX1s[1] = ENFT_SSE::_mm_mul_ps(m_sss1, dX1s[1]);		nX1s[2] = ENFT_SSE::_mm_mul_ps(m_sss1, dX1s[2]);
	m_C.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[0], _mm_set1_ps(nX2s[0].m128_f32[0])), ENFT_SSE::_mm_mul_ps(nX1s[1], _mm_set1_ps(nX2s[1].m128_f32[0]))), 
									_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[2], _mm_set1_ps(nX2s[2].m128_f32[0])), ENFT_SSE::_mm_mul_ps(nX1s[3], _mm_set1_ps(nX2s[3].m128_f32[0]))));
	m_C.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[0], _mm_set1_ps(nX2s[0].m128_f32[1])), ENFT_SSE::_mm_mul_ps(nX1s[1], _mm_set1_ps(nX2s[1].m128_f32[1]))), 
									_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[2], _mm_set1_ps(nX2s[2].m128_f32[1])), ENFT_SSE::_mm_mul_ps(nX1s[3], _mm_set1_ps(nX2s[3].m128_f32[1]))));
	m_C.M_20_21_22_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[0], _mm_set1_ps(nX2s[0].m128_f32[2])), ENFT_SSE::_mm_mul_ps(nX1s[1], _mm_set1_ps(nX2s[1].m128_f32[2]))), 
									_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1s[2], _mm_set1_ps(nX2s[2].m128_f32[2])), ENFT_SSE::_mm_mul_ps(nX1s[3], _mm_set1_ps(nX2s[3].m128_f32[2]))));
	//m_C.Print();
}

void SimilarityTransformationSolver::ComputeMeanScaleAndCovariance(const AlignedVector<Point3D> &X1s, const AlignedVector<Point3D> &X2s, AlignedVector<ENFT_SSE::__m128> &work)
{
	const ushort N = ushort(X1s.Size()), Nx2 = (N << 1);
	work.Resize(2 + Nx2);

	m_mean1 = ENFT_SSE::_mm_setzero_ps();
	for(ushort i = 0; i < N; ++i)
		m_mean1.XYZx() = ENFT_SSE::_mm_add_ps(X1s[i].XYZx(), m_mean1.XYZx());
	ENFT_SSE::__m128 &one_over_N = work[0];
	one_over_N = ENFT_SSE::_mm_set1_ps(1.0f / N);
	m_mean1.XYZx() = ENFT_SSE::_mm_mul_ps(m_mean1.XYZx(), one_over_N);
	m_mean2.XYZx() = ENFT_SSE::_mm_setzero_ps();
	for(ushort i = 0; i < N; ++i)
		m_mean2.XYZx() = ENFT_SSE::_mm_add_ps(X2s[i].XYZx(), m_mean2.XYZx());
	m_mean2.XYZx() = ENFT_SSE::_mm_mul_ps(m_mean2.XYZx(), one_over_N);

	ENFT_SSE::__m128 &distSq1 = work[0], &distSq2 = work[1];
	ENFT_SSE::__m128 *dXs = work.Data() + 2;
	distSq1 = distSq2 = ENFT_SSE::_mm_setzero_ps();
	for(ushort i = 0, i1 = 0, i2 = 1; i < N; ++i, i1 += 2, i2 += 2)
	{
		ENFT_SSE::__m128 &dX1 = dXs[i1], &dX2 = dXs[i2];
		dX1 = ENFT_SSE::_mm_sub_ps(X1s[i].XYZx(), m_mean1.XYZx());		distSq1 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(dX1, dX1), distSq1);
		dX2 = ENFT_SSE::_mm_sub_ps(X2s[i].XYZx(), m_mean2.XYZx());		distSq2 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(dX2, dX2), distSq2);
	}
	//ENFT_SSE::SSE::Print(distSq1);
	//ENFT_SSE::SSE::Print(distSq2);
	const float s = sqrt(ENFT_SSE::SSE::Sum012(distSq2) / ENFT_SSE::SSE::Sum012(distSq1));
	//const float s = 1.0f;
	m_sss1 = _mm_setr_ps(s, s, s, 1.0f);

	m_C.SetZero();
	for(ushort i = 0, i1 = 0, i2 = 1; i < N; ++i, i1 += 2, i2 += 2)
	{
		ENFT_SSE::__m128 &dX1 = dXs[i1], &nX1 = dX1, &nX2 = dXs[i2];
		nX1 = ENFT_SSE::_mm_mul_ps(m_sss1, dX1);
		m_C.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1, _mm_set1_ps(nX2.m128_f32[0])), m_C.M_00_01_02_x());
		m_C.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1, _mm_set1_ps(nX2.m128_f32[1])), m_C.M_10_11_12_x());
		m_C.M_20_21_22_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(nX1, _mm_set1_ps(nX2.m128_f32[2])), m_C.M_20_21_22_x());
	}
	//m_C.Print();
}