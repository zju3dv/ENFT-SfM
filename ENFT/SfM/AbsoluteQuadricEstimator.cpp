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
#include "AbsoluteQuadricEstimator.h"
#include "LinearAlgebra/Vector5.h"
#include "LinearAlgebra/Vector6.h"

#if _DEBUG
#include "Utility/Random.h"
//#define _DEBUG_WITH_EIGEN_Q_LINEAR
#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
#include <Eigen>
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenMatrix;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> EigenVector;
#endif
#endif

AbsoluteQuadricEstimator::AbsoluteQuadricEstimator()
{
	m_weights[0] = m_weights[1] = 1 / 4.0f;
	//m_weights[0] = m_weights[1] = 1 / 9.0f;
	//m_weights[0] = m_weights[1] = 1 / 90.0f;
	//m_weights[0] = m_weights[1] = 0.0f;
	m_weights[2] = 1 / 0.2f;
	m_weights[3] = m_weights[4] = 1 / 0.1f;
	m_weights[5] = 1 / 0.01f;
//#if _DEBUG
//	memset(m_weights, 0, sizeof(m_weights));
//#endif
}

bool AbsoluteQuadricEstimator::Run(const AlignedVector<ProjectiveMatrix> &Ps, AbsoluteQuadric &Q, float &Ecalib)
{
//#if _DEBUG
#if 0
	//m_Ps = Ps;
	for(uint i = 0; i < Ps.Size(); ++i)
	{
		printf("----------------------------------------------------------------\n");
		Ps[i].Print();
	}
#endif
	m_data.Set(Ps, m_weights);
	//m_data.Set(Ps, m_weights, 0.0f);
	//m_data.Set(Ps, m_weights, 1.0f);
//	if(!SolveAbsoluteQuadricLinear(m_data, Q, true))
//		return false;
////#if _DEBUG
////	printf("----------------------------------------------------------------\n");
////	Q.Print();
////#endif
//	if(!SolveAbsoluteQuadricLinear(m_data, Q, false))
//		return false;
////#if _DEBUG
////	printf("----------------------------------------------------------------\n");
////	Q.Print();
	////#endif
	float EcalibMin = FLT_MAX;
	Q.Invalidate();
	m_idxs.resize(3);
	const uint N = Ps.Size() - 2;
	for(uint i = 0; i < N; ++i)
	{
		m_idxs[0] = i;
		m_idxs[1] = i + 1;
		m_idxs[2] = i + 2;
		if(!SolveAbsoluteQuadricLinear(m_data, m_idxs, m_Q, true))
			continue;
		if(!SolveAbsoluteQuadricLinear(m_data, m_idxs, m_Q, false))
			continue;
		if((Ecalib = float(m_data.ComputeSSE(m_Q))) > EcalibMin)
			continue;
		EcalibMin = Ecalib;
		Q = m_Q;
	}
//#if _DEBUG
//	const float maxNoise = 0.1f;
//	Q.f2() += Random::GenerateFloat(-maxNoise, maxNoise);
//	Q.a0() += Random::GenerateFloat(-maxNoise, maxNoise);
//	Q.a1() += Random::GenerateFloat(-maxNoise, maxNoise);
//	Q.a2() += Random::GenerateFloat(-maxNoise, maxNoise);
//	Q.b() += Random::GenerateFloat(-maxNoise, maxNoise);
//	printf("----------------------------------------------------------------\n");
//	Q.Print();
//#endif
	Q.EnforceSingularConstraint();
//#if _DEBUG
//	printf("----------------------------------------------------------------\n");
//	Q.Print();
//#endif
	m_optimizer.Run(m_data, Q);
	//m_optimizer.Run(m_data, Q, 2);
//#if _DEBUG
//	printf("----------------------------------------------------------------\n");
//	Q.Print();
//#endif
	Q.EnforceSingularConstraint();
//#if _DEBUG
//	printf("----------------------------------------------------------------\n");
//	Q.Print();
//#endif
	Ecalib = float(m_data.ComputeSSE(Q));
//#if _DEBUG
//	printf("----------------------------------------------------------------\n");
//	printf("Ecalib = %f\n", Ecalib);
//#endif
	return Q.f2() > 0.0f;
}

#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
static inline void SetM(const LA::AlignedMatrix5f &Msrc, EigenMatrix &Mdst)
{
	int i, j;
	Mdst.resize(5, 5);
	const float *pMsrc = Msrc;
	for(i = 0; i < 5; ++i, pMsrc += 8)
	for(j = 0; j < 5; ++j)
		Mdst(i, j) = pMsrc[j];
}
static inline void SetM(const LA::AlignedMatrix3x4f &Msrc, EigenMatrix &Mdst)
{
	int i, j;
	Mdst.resize(3, 4);
	const float *pMsrc = Msrc;
	for(i = 0; i < 3; ++i, pMsrc += 4)
	for(j = 0; j < 4; ++j)
		Mdst(i, j) = pMsrc[j];
}
static inline void SetV(const LA::AlignedVector5f &Vsrc, EigenVector &Vdst)
{
	Vdst.resize(5);
	for(int i = 0; i < 5; ++i)
		Vdst(i) = Vsrc[i];
}
static inline void CheckM(const EigenMatrix &M1, const EigenMatrix &M2)
{
	int i, j;
	float v1, v2;
	const int nRows = int(M1.rows()), nCols = int(M1.cols());
	for(i = 0; i < nRows; ++i)
	for(j = 0; j < nCols; ++j)
	{
		v1 = M1(i, j);
		v2 = M2(i, j);
		if(!EQUAL(v1, v2))
			printf("(%d, %d): %f - %f = %f\n", i, j, v1, v2, v1 - v2);
	}
}
static inline void CheckV(const EigenVector &V1, const EigenVector &V2)
{
	float v1, v2;
	const int nRows = int(V1.rows());
	for(int i = 0; i < nRows; ++i)
	{
		v1 = V1(i);
		v2 = V2(i);
		if(!EQUAL(v1, v2))
			printf("%d: %f - %f = %f\n", i, v1, v2, v1 - v2);
	}
}
static inline void PrintM(const EigenMatrix &M)
{
	int r, c;
	const int nRows = int(M.rows()), nCols = int(M.cols());
	for(r = 0; r < nRows; ++r)
	{
		for(c = 0; c < nCols; ++c)
			printf("%f ", M(r, c));
		printf("\n");
	}
}
static inline void PrintV(const EigenMatrix &V)
{
	const int nRows = int(V.rows());
	for(int i = 0; i < nRows; ++i)
		printf("%f ", V(i));
	printf("\n");
}
#endif

bool AbsoluteQuadricEstimator::SolveAbsoluteQuadricLinear(const AbsoluteQuadricOptimizerData &data, const std::vector<uint> &idxs, AbsoluteQuadric &Q, const bool initial)
{
	const uint N = uint(idxs.size());
	//const float norm = 1.0f;
	//const float norm = 1.0f / N;
	const float norm = 0.5f / N;
	if(initial)
	{
		for(int j = 0; j < 6; ++j)
			m_ws[j] = _mm_set1_ps(norm * m_weights[j]);
	}

	m_ATA.SetZero();
	m_ATb.SetZero();

#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
	EigenMatrix A;
	EigenVector b;
	const int nEqs = int(N) * 6, nVars = 5;
	int iEq = 0;
	A.resize(nEqs, nVars);	A.setZero();
	b.resize(nEqs);			b.setZero();
	EigenMatrix Qp;
	if(!initial)
	{
		Qp.resize(4, 4);
		Qp.setZero();
		Qp(0, 0) = Qp(1, 1) = Q.f2();
		Qp(2, 2) = 1.0f;
		Qp(0, 3) = Qp(3, 0) = Q.a0();
		Qp(1, 3) = Qp(3, 1) = Q.a1();
		Qp(2, 3) = Qp(3, 2) = Q.a2();
		Qp(3, 3) = Q.b();
		//printf("----------------------------------------------------------------\n");
		//PrintM(Qp);
	}
#endif

	for(uint i = 0; i < N; ++i)
	{
		const AbsoluteQuadricOptimizerData::Coeffient &c = data.GetCoefficient(idxs[i]);
		if(!initial)
		{
			const float w = norm / c.c22().Dot(Q);
			for(int j = 0; j < 6; ++j)
				m_ws[j] = _mm_set1_ps(m_weights[j] * w);
//#if _DEBUG
//			printf("----------------------------------------------------------------\n");
//			//if(m_weights[0] != 0.0f)
//			//printf("%f\n", c.c00().Dot(Q) - c.c22().Dot(Q));
//			//if(m_weights[1] != 0.0f)
//			//printf("%f\n", c.c11().Dot(Q) - c.c22().Dot(Q));
//			//printf("%f\n", c.c00().Dot(Q) - c.c11().Dot(Q));
//			//printf("%f\n", c.c02().Dot(Q));
//			//printf("%f\n", c.c12().Dot(Q));
//			//printf("%f\n", c.c01().Dot(Q));
//			printf("%f %f %f\n", c.c00().Dot(Q), c.c01().Dot(Q), c.c02().Dot(Q));
//			printf("%f %f %f\n", c.c01().Dot(Q), c.c11().Dot(Q), c.c12().Dot(Q));
//			printf("%f %f %f\n", c.c02().Dot(Q), c.c12().Dot(Q), c.c22().Dot(Q));
////#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
////		printf("----------------------------------------------------------------\n");
////		EigenMatrix P;
////		SetM(m_Ps[i], P);
////		const EigenMatrix q = P * Qp * P.transpose();
////		PrintM(q);
////#endif
//#endif
		}
		
//#if _DEBUG
//		printf("----------------------------------------------------------------\n");
//		m_Ps[i].Print();
//		c.c00().Print();
//		c.c01().Print();
//		c.c02().Print();
//		c.c11().Print();
//		c.c12().Print();
//		c.c22().Print();
//#endif
		LA::AmB(c.c00(), c.c22(), m_eq);
		LA::sA(m_ws[0], m_eq);
		m_A.M_00_01_02_03() = m_eq.v0123();
		m_A.M04() = m_eq.v4();
		m_b.v0() = -m_eq.v5();
#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
		for(int iVar = 0; iVar < nVars; ++iVar)
			A(iEq, iVar) = m_eq[iVar];
		b(iEq) = -m_eq[nVars];
		++iEq;
#endif

		LA::AmB(c.c11(), c.c22(), m_eq);
		LA::sA(m_ws[1], m_eq);
		m_A.M_10_11_12_13() = m_eq.v0123();
		m_A.M14() = m_eq.v4();
		m_b.v1() = -m_eq.v5();
#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
		for(int iVar = 0; iVar < nVars; ++iVar)
			A(iEq, iVar) = m_eq[iVar];
		b(iEq) = -m_eq[nVars];
		++iEq;
#endif

		LA::AmB(c.c00(), c.c11(), m_eq);
		LA::sA(m_ws[2], m_eq);
		m_A.M_20_21_22_23() = m_eq.v0123();
		m_A.M24() = m_eq.v4();
		m_b.v2() = -m_eq.v5();
#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
		for(int iVar = 0; iVar < nVars; ++iVar)
			A(iEq, iVar) = m_eq[iVar];
		b(iEq) = -m_eq[nVars];
		++iEq;
#endif

		LA::sA(m_ws[3], c.c02(), m_eq);
		m_A.M_30_31_32_33() = m_eq.v0123();
		m_A.M34() = m_eq.v4();
		m_b.v3() = -m_eq.v5();
#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
		for(int iVar = 0; iVar < nVars; ++iVar)
			A(iEq, iVar) = m_eq[iVar];
		b(iEq) = -m_eq[nVars];
		++iEq;
#endif

		LA::sA(m_ws[4], c.c12(), m_eq);
		m_A.M_40_41_42_43() = m_eq.v0123();
		m_A.M44() = m_eq.v4();
		m_b.v4() = -m_eq.v5();
#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
		for(int iVar = 0; iVar < nVars; ++iVar)
			A(iEq, iVar) = m_eq[iVar];
		b(iEq) = -m_eq[nVars];
		++iEq;
#endif

		LA::sA(m_ws[5], c.c01(), m_eq);
		m_A.M_50_51_52_53() = m_eq.v0123();
		m_A.M54() = m_eq.v4();
		m_b.v5() = -m_eq.v5();
#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
		for(int iVar = 0; iVar < nVars; ++iVar)
			A(iEq, iVar) = m_eq[iVar];
		b(iEq) = -m_eq[nVars];
		++iEq;
#endif

		m_A.GetTranpose(m_AT);
		LA::AddAATToUpper(m_AT, m_ATA);
		LA::AddABTo(m_AT, m_b, m_ATb);
//#if _DEBUG
#if 0
		printf("----------------------------------------------------------------\n");
		printf("A\n");
		m_A.Print();
		printf("----------------------------------------------------------------\n");
		printf("ATA\n");
		m_ATA.SetLowerFromUpper();
		m_ATA.Print();
		//m_ATb.Print();
#endif
	}
	m_ATA.SetLowerFromUpper();
#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
	const EigenMatrix AT = A.transpose();
	const EigenMatrix ATA1 = AT * A;
	const EigenVector ATb1 = AT * b;
	EigenMatrix ATA2;
	EigenVector ATb2;
	SetM(m_ATA, ATA2);
	SetV(m_ATb, ATb2);
	//printf("----------------------------------------------------------------\n");
	//PrintM(ATA1);
	//printf("----------------------------------------------------------------\n");
	//m_ATA.Print();
	//printf("----------------------------------------------------------------\n");
	//PrintV(ATb1);
	//printf("----------------------------------------------------------------\n");
	//m_ATb.Print();
	CheckM(ATA1, ATA2);
	CheckV(ATb1, ATb2);
#endif
#if _DEBUG
//	const LA::AlignedMatrix5f ATABkp = m_ATA;
//	const LA::AlignedVector5f ATbBkp = m_ATb;
	//printf("----------------------------------------------------------------\n");
	//printf("ATA:\n");
	//m_ATA.Print();
	//printf("----------------------------------------------------------------\n");
	//printf("ATb:\n");
	//m_ATb.Print();
#endif
//#if _DEBUG
//	if(g_cnt == 53)
//	{
//		printf("----------------------------------------------------------------\n");
//		printf("ATA:\n");
//		m_ATA.Print();
//		printf("----------------------------------------------------------------\n");
//		printf("ATb:\n");
//		m_ATb.Print();
//	}
//#endif
	//m_ATA.Print();
	//m_ATb.Print();
	if(!LA::SolveLinearSystemSymmetricUpper(m_ATA, m_ATb))
		return false;
	Q = m_ATb;
//#if _DEBUG
//	printf("----------------------------------------------------------------\n");
//	Q.Print();
//	//printf("----------------------------------------------------------------\n");
//	//LA::AlignedVector5f e;
//	//LA::AB(ATABkp, Q, e);
//	//LA::AmB(e, ATbBkp, e);
//	//e.Print();
//#ifdef _DEBUG_WITH_EIGEN_Q_LINEAR
//	const EigenVector x1 = ATA1.ldlt().solve(ATb1);
//	PrintV(x1);
//	EigenVector x2;
//	SetV(Q, x2);
//	CheckV(x1, x2);
//	const EigenVector Ax = A * x1;
//	//const EigenVector Ax = A * x2;
//	CheckV(Ax, b);
//	const EigenVector ATAx = ATA1 * x2;
//	//CheckV(ATAx, ATb1);
//	//PrintV(ATAx);
//	PrintV(ATAx - ATb1);
//#endif
//#endif
	return Q.f2() > 0;
}