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

#ifndef _ABSOLUTE_QUADRIC_OPTIMIZER_DATA_H_
#define _ABSOLUTE_QUADRIC_OPTIMIZER_DATA_H_

#include "AbsoluteQuadric.h"
#include "ProjectiveMatrix.h"
#include "Optimization/OptimizerData.h"
#include "LinearAlgebra/Vector6.h"
#include "LinearAlgebra/Matrix9x5.h"

#if _DEBUG
//#define _DEBUG_JACOBIAN_Q
#ifdef _DEBUG_JACOBIAN_Q
static const AbsoluteQuadric *g_pQ = NULL;
#endif
//#define _DEBUG_WITH_EIGEN_Q_NON_LINEAR
#ifdef _DEBUG_WITH_EIGEN_Q_NON_LINEAR
#include <Eigen>
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenMatrix;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> EigenVector;
static EigenMatrix g_J;
static EigenVector g_e;
static int g_iEq;
static inline void Check(const LA::AlignedMatrix5f &A, const LA::AlignedVector5f &b)
{
	float v1, v2;
	const EigenMatrix JT = g_J.transpose(), M1 = g_J.transpose() * g_J;
	const float *M2[5] = {&A.M00(), &A.M10(), &A.M20(), &A.M30(), &A.M40()};
	for(int i = 0; i < 5; ++i)
	for(int j = 0; j < 5; ++j)
	{
		v1 = M1(i, j);
		v2 = M2[i][j];
		//if(!EQUAL(v1, v2))
		if(v1 != v2)
			printf("(%d, %d): %f - %f = %f\n", i, j, v1, v2, v1 - v2);
	}
	const EigenVector V1 = JT * g_e;
	const float *V2 = b;
	for(int i = 0; i < 5; ++i)
	{
		v1 = V1(i);
		v2 = V2[i];
		//if(!EQUAL(v1, v2))
		if(v1 != v2)
			printf("%d: %f - %f = %f\n", i, v1, v2, v1 - v2);
	}
}
#endif
#endif

class AbsoluteQuadricOptimizerData : public OptimizerDataTemplate<AbsoluteQuadric, LA::AlignedVector5f, LA::AlignedMatrix5f>
{

public:

	class Coeffient
	{
	public:
		inline const LA::AlignedVector6f &c00() const { return m_c00; }		inline LA::AlignedVector6f &c00() { return m_c00; }
		inline const LA::AlignedVector6f &c01() const { return m_c01; }		inline LA::AlignedVector6f &c01() { return m_c01; }
		inline const LA::AlignedVector6f &c02() const { return m_c02; }		inline LA::AlignedVector6f &c02() { return m_c02; }
		inline const LA::AlignedVector6f &c11() const { return m_c11; }		inline LA::AlignedVector6f &c11() { return m_c11; }
		inline const LA::AlignedVector6f &c12() const { return m_c12; }		inline LA::AlignedVector6f &c12() { return m_c12; }
		inline const LA::AlignedVector6f &c22() const { return m_c22; }		inline LA::AlignedVector6f &c22() { return m_c22; }
	protected:
		LA::AlignedVector6f m_c00, m_c01, m_c02, m_c11, m_c12, m_c22;
	};

public:

	inline const uint& Size() const { return m_cs.Size(); }

	inline void Set(const AlignedVector<ProjectiveMatrix> &Ps, const float *w1, const float w2 = 1.0f, const float epsilon = 1e-7)
	{
		const uint N = Ps.Size();
		m_cs.Resize(N);
		for(uint i = 0; i < N; ++i)
		{
			const ProjectiveMatrix &P = Ps[i];
			Coeffient &c = m_cs[i];
			ComputeCoefficientPQPT(P.M_00_01_02_03(), P.M_00_01_02_03(), c.c00(), m_work);
			ComputeCoefficientPQPT(P.M_00_01_02_03(), P.M_10_11_12_13(), c.c01(), m_work);
			ComputeCoefficientPQPT(P.M_00_01_02_03(), P.M_20_21_22_23(), c.c02(), m_work);
			ComputeCoefficientPQPT(P.M_10_11_12_13(), P.M_10_11_12_13(), c.c11(), m_work);
			ComputeCoefficientPQPT(P.M_10_11_12_13(), P.M_20_21_22_23(), c.c12(), m_work);
			ComputeCoefficientPQPT(P.M_20_21_22_23(), P.M_20_21_22_23(), c.c22(), m_work);
		}
		const float norm = 1.0f / N;
		for(int i = 0; i < 6; ++i)
			m_ws[i] = w1[i] * norm;
		m_ws[6] = w2;
		m_epsilon = epsilon;
	}
	inline const Coeffient& GetCoefficient(const uint &i) const { return m_cs[i]; }

	virtual void NormalizeData(const float &dataNormalizeMedian, AbsoluteQuadric &Q) {}
	virtual void DenormalizeData(AbsoluteQuadric &Q) {}
	virtual double ComputeSSE(const AbsoluteQuadric &Q)
	{
		double SSE = 0.0;
		const uint N = m_cs.Size();
		for(uint i = 0; i < N; ++i)
		{
			const Coeffient &c = m_cs[i];
			m_d00 = c.c00().Dot(Q);
			m_d01 = c.c01().Dot(Q);
			m_d02 = c.c02().Dot(Q);
			m_d11 = c.c11().Dot(Q);
			m_d12 = c.c12().Dot(Q);
			m_d22 = c.c22().Dot(Q);
			m_d00I = 1 / m_d00;
			m_d11I = 1 / m_d11;
			m_d22I = 1 / m_d22;
			ComputeEquationPQPT(m_ws[0], m_d00, m_d22I, 1.0f, m_e1.v0());
			ComputeEquationPQPT(m_ws[0], m_d22, m_d00I, 1.0f, m_e1.v1());
			ComputeEquationPQPT(m_ws[1], m_d11, m_d22I, 1.0f, m_e1.v2());
			ComputeEquationPQPT(m_ws[1], m_d22, m_d11I, 1.0f, m_e1.v3());
			ComputeEquationPQPT(m_ws[2], m_d00, m_d11I, 1.0f, m_e1.v4());
			ComputeEquationPQPT(m_ws[2], m_d11, m_d00I, 1.0f, m_e1.v5());
			ComputeEquationPQPT(m_ws[3], m_d02, m_d22I, 0.0f, m_e1.v6());
			ComputeEquationPQPT(m_ws[4], m_d12, m_d22I, 0.0f, m_e1.v7());
			ComputeEquationPQPT(m_ws[5], m_d01, m_d22I, 0.0f, m_e1.v8());
			SSE += m_e1.SquaredLength();
		}
		ComputeEquationSingular(m_ws[6], m_epsilon, Q, m_v, m_e2);
		SSE += m_e2.SquaredLength();
		return SSE;
	}

	virtual double GetFactorSSEToMSE() { return 1.0; }
	virtual void ConstructNormalEquation(const AbsoluteQuadric &Q, LA::AlignedMatrix5f &A, LA::AlignedVector5f &b, LA::AlignedVector5f &s)
	{
#ifdef _DEBUG_JACOBIAN_Q
		g_pQ = &Q;
#endif
#ifdef _DEBUG_WITH_EIGEN_Q_NON_LINEAR
		const int nEqs = m_cs.Size() * 9 + 2, nVars = 5;
		g_J.resize(nEqs, nVars);
		g_J.setZero();
		g_e.resize(nEqs);
		g_e.setZero();
		g_iEq = 0;
#endif
		A.SetZero();
		b.SetZero();
		s.SetZero();

		const uint N = m_cs.Size();
		for(uint i = 0; i < N; ++i)
		{
			const Coeffient &c = m_cs[i];
			m_d00 = c.c00().Dot(Q);
			m_d01 = c.c01().Dot(Q);
			m_d02 = c.c02().Dot(Q);
			m_d11 = c.c11().Dot(Q);
			m_d12 = c.c12().Dot(Q);
			m_d22 = c.c22().Dot(Q);
			m_d00I = 1 / m_d00;
			m_d11I = 1 / m_d11;
			m_d22I = 1 / m_d22;
			ComputeEquationPQPT(m_ws[0], c.c00(), c.c22(), m_d00, m_d22I, 1.0f, m_e1.v0(), m_J1.M_00_01_02_03(), m_J1.M04());
			ComputeEquationPQPT(m_ws[0], c.c22(), c.c00(), m_d22, m_d00I, 1.0f, m_e1.v1(), m_J1.M_10_11_12_13(), m_J1.M14());
			ComputeEquationPQPT(m_ws[1], c.c11(), c.c22(), m_d11, m_d22I, 1.0f, m_e1.v2(), m_J1.M_20_21_22_23(), m_J1.M24());
			ComputeEquationPQPT(m_ws[1], c.c22(), c.c11(), m_d22, m_d11I, 1.0f, m_e1.v3(), m_J1.M_30_31_32_33(), m_J1.M34());
			ComputeEquationPQPT(m_ws[2], c.c00(), c.c11(), m_d00, m_d11I, 1.0f, m_e1.v4(), m_J1.M_40_41_42_43(), m_J1.M44());
			ComputeEquationPQPT(m_ws[2], c.c11(), c.c00(), m_d11, m_d00I, 1.0f, m_e1.v5(), m_J1.M_50_51_52_53(), m_J1.M54());
			ComputeEquationPQPT(m_ws[3], c.c02(), c.c22(), m_d02, m_d22I, 0.0f, m_e1.v6(), m_J1.M_60_61_62_63(), m_J1.M64());
			ComputeEquationPQPT(m_ws[4], c.c12(), c.c22(), m_d12, m_d22I, 0.0f, m_e1.v7(), m_J1.M_70_71_72_73(), m_J1.M74());
			ComputeEquationPQPT(m_ws[5], c.c01(), c.c22(), m_d01, m_d22I, 0.0f, m_e1.v8(), m_J1.M_80_81_82_83(), m_J1.M84());
			m_J1.GetTranpose(m_J1T);
			LA::AddAATToUpper(m_J1T, A);
			LA::AddABTo(m_J1T, m_e1, b);
			LA::AddAij2To(m_J1, s);
		}
		ComputeEquationSingular(m_ws[6], m_epsilon, Q, m_v, m_e2, m_J2);
		LA::AddATAToUpper(m_J2, A);
		LA::AddATBTo(m_J2, m_e2, b);
		LA::AddAij2To(m_J2, s);
		A.SetLowerFromUpper();
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
#ifdef _DEBUG_WITH_EIGEN_Q_NON_LINEAR
		s.v0123() = s.v4xxx() = _mm_set1_ps(1.0f);
#endif
		LA::ssTA(s, A);
		LA::sA(s, b);
#ifdef _DEBUG_WITH_EIGEN_Q_NON_LINEAR
		Check(A, b);
#endif
//#if _DEBUG
//		A.Print();
//#endif
	}
	virtual void ConstructNormalEquation(const AbsoluteQuadric &Q, const LA::AlignedVector5f &s, LA::AlignedMatrix5f &A, LA::AlignedVector5f &b)
	{
#ifdef _DEBUG_JACOBIAN_Q
		g_pQ = &Q;
#endif
#ifdef _DEBUG_WITH_EIGEN_Q_NON_LINEAR
		const int nEqs = m_cs.Size() * 9 + 2, nVars = 5;
		g_J.resize(nEqs, nVars);
		g_J.setZero();
		g_e.resize(nEqs);
		g_e.setZero();
		g_iEq = 0;
#endif
		A.SetZero();
		b.SetZero();

		const uint N = m_cs.Size();
		for(uint i = 0; i < N; ++i)
		{
			const Coeffient &c = m_cs[i];
			m_d00 = c.c00().Dot(Q);
			m_d01 = c.c01().Dot(Q);
			m_d02 = c.c02().Dot(Q);
			m_d11 = c.c11().Dot(Q);
			m_d12 = c.c12().Dot(Q);
			m_d22 = c.c22().Dot(Q);
			m_d00I = 1 / m_d00;
			m_d11I = 1 / m_d11;
			m_d22I = 1 / m_d22;
			ComputeEquationPQPT(m_ws[0], c.c00(), c.c22(), m_d00, m_d22I, 1.0f, m_e1.v0(), m_J1.M_00_01_02_03(), m_J1.M04());
			ComputeEquationPQPT(m_ws[0], c.c22(), c.c00(), m_d22, m_d00I, 1.0f, m_e1.v1(), m_J1.M_10_11_12_13(), m_J1.M14());
			ComputeEquationPQPT(m_ws[1], c.c11(), c.c22(), m_d11, m_d22I, 1.0f, m_e1.v2(), m_J1.M_20_21_22_23(), m_J1.M24());
			ComputeEquationPQPT(m_ws[1], c.c22(), c.c11(), m_d22, m_d11I, 1.0f, m_e1.v3(), m_J1.M_30_31_32_33(), m_J1.M34());
			ComputeEquationPQPT(m_ws[2], c.c00(), c.c11(), m_d00, m_d11I, 1.0f, m_e1.v4(), m_J1.M_40_41_42_43(), m_J1.M44());
			ComputeEquationPQPT(m_ws[2], c.c11(), c.c00(), m_d11, m_d00I, 1.0f, m_e1.v5(), m_J1.M_50_51_52_53(), m_J1.M54());
			ComputeEquationPQPT(m_ws[3], c.c02(), c.c22(), m_d02, m_d22I, 0.0f, m_e1.v6(), m_J1.M_60_61_62_63(), m_J1.M64());
			ComputeEquationPQPT(m_ws[4], c.c12(), c.c22(), m_d12, m_d22I, 0.0f, m_e1.v7(), m_J1.M_70_71_72_73(), m_J1.M74());
			ComputeEquationPQPT(m_ws[5], c.c01(), c.c22(), m_d01, m_d22I, 0.0f, m_e1.v8(), m_J1.M_80_81_82_83(), m_J1.M84());
			m_J1.GetTranpose(m_J1T);
			LA::AddAATToUpper(m_J1T, A);
			LA::AddABTo(m_J1T, m_e1, b);
		}
		ComputeEquationSingular(m_ws[6], m_epsilon, Q, m_v, m_e2, m_J2);
		LA::AddATAToUpper(m_J2, A);
		LA::AddATBTo(m_J2, m_e2, b);
		A.SetLowerFromUpper();
		//printf("\n");
		//A.Print();
		LA::ssTA(s, A);
		LA::sA(s, b);
#ifdef _DEBUG_WITH_EIGEN_Q_NON_LINEAR
		Check(A, b);
#endif
	}
	virtual void UpdateModel(const LA::AlignedVector5f &s, const LA::AlignedVector5f &x, const AbsoluteQuadric &Qold, AbsoluteQuadric &Qnew)
	{
		Qnew.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s.v0123(), x.v0123()), Qold.v0123());
		Qnew.v4() = s.v4() * x.v4() + Qold.v4();
		//Qnew.EnforceSingularConstraint();
	}

protected:

	// pi^T * Q * pj = c^T * [f2, a0, a1, a2, b, 1]
	static inline void ComputeCoefficientPQPT(const ENFT_SSE::__m128 &pi, const ENFT_SSE::__m128 &pj, LA::AlignedVector6f &cij, ENFT_SSE::__m128 &work)
	{
		work = ENFT_SSE::_mm_mul_ps(pi, pj);
		cij.v0() = work.m128_f32[0] + work.m128_f32[1];
		cij.v5() = work.m128_f32[2];
		cij.v4() = work.m128_f32[3];
		work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(pi.m128_f32[3]), pj), ENFT_SSE::_mm_mul_ps(pi, _mm_set1_ps(pj.m128_f32[3])));
		cij.v1() = work.m128_f32[0];
		cij.v2() = work.m128_f32[1];
		cij.v3() = work.m128_f32[2];
	}

	// e = w * d1 / d2 = w * c1^T*q / c2^T*q
	static inline void ComputeEquationPQPT(const float &w, const float &d1, const float &d2I, const float &v, float &e)
	{
		e = w * (v - d1 * d2I);
	}
	static inline void ComputeEquationPQPT(const float &w, const LA::AlignedVector6f &c1, const LA::AlignedVector6f &c2, const float &d1, const float &d2I, 
										   const float &v, float &e, ENFT_SSE::__m128 &j0123, float &j4)
	{
		j4 = w * d2I;
		e = d1 * d2I;
		j0123 = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(j4), ENFT_SSE::_mm_sub_ps(c1.v0123(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(e), c2.v0123())));
		j4 = j4 * (c1.v4() - e * c2.v4());
		e = w * (v - e);
#ifdef _DEBUG_JACOBIAN_Q
		const float *j1 = &j0123.m128_f32[0];
		float j2[5];
		j2[0] = (c1.v0()*w)/(c2.v5() + g_pQ->a0()*c2.v1() + g_pQ->a1()*c2.v2() + g_pQ->a2()*c2.v3() + g_pQ->b()*c2.v4() + c2.v0()*g_pQ->f2()) - (c2.v0()*w*(c1.v5() + g_pQ->a0()*c1.v1() + g_pQ->a1()*c1.v2() + g_pQ->a2()*c1.v3() + g_pQ->b()*c1.v4() + c1.v0()*g_pQ->f2()))/powf(c2.v5() + g_pQ->a0()*c2.v1() + g_pQ->a1()*c2.v2() + g_pQ->a2()*c2.v3() + g_pQ->b()*c2.v4() + c2.v0()*g_pQ->f2(), 2);
		j2[1] = (c1.v1()*w)/(c2.v5() + g_pQ->a0()*c2.v1() + g_pQ->a1()*c2.v2() + g_pQ->a2()*c2.v3() + g_pQ->b()*c2.v4() + c2.v0()*g_pQ->f2()) - (c2.v1()*w*(c1.v5() + g_pQ->a0()*c1.v1() + g_pQ->a1()*c1.v2() + g_pQ->a2()*c1.v3() + g_pQ->b()*c1.v4() + c1.v0()*g_pQ->f2()))/powf(c2.v5() + g_pQ->a0()*c2.v1() + g_pQ->a1()*c2.v2() + g_pQ->a2()*c2.v3() + g_pQ->b()*c2.v4() + c2.v0()*g_pQ->f2(), 2);
		j2[2] = (c1.v2()*w)/(c2.v5() + g_pQ->a0()*c2.v1() + g_pQ->a1()*c2.v2() + g_pQ->a2()*c2.v3() + g_pQ->b()*c2.v4() + c2.v0()*g_pQ->f2()) - (c2.v2()*w*(c1.v5() + g_pQ->a0()*c1.v1() + g_pQ->a1()*c1.v2() + g_pQ->a2()*c1.v3() + g_pQ->b()*c1.v4() + c1.v0()*g_pQ->f2()))/powf(c2.v5() + g_pQ->a0()*c2.v1() + g_pQ->a1()*c2.v2() + g_pQ->a2()*c2.v3() + g_pQ->b()*c2.v4() + c2.v0()*g_pQ->f2(), 2);
		j2[3] = (c1.v3()*w)/(c2.v5() + g_pQ->a0()*c2.v1() + g_pQ->a1()*c2.v2() + g_pQ->a2()*c2.v3() + g_pQ->b()*c2.v4() + c2.v0()*g_pQ->f2()) - (c2.v3()*w*(c1.v5() + g_pQ->a0()*c1.v1() + g_pQ->a1()*c1.v2() + g_pQ->a2()*c1.v3() + g_pQ->b()*c1.v4() + c1.v0()*g_pQ->f2()))/powf(c2.v5() + g_pQ->a0()*c2.v1() + g_pQ->a1()*c2.v2() + g_pQ->a2()*c2.v3() + g_pQ->b()*c2.v4() + c2.v0()*g_pQ->f2(), 2);
		j2[4] = (c1.v4()*w)/(c2.v5() + g_pQ->a0()*c2.v1() + g_pQ->a1()*c2.v2() + g_pQ->a2()*c2.v3() + g_pQ->b()*c2.v4() + c2.v0()*g_pQ->f2()) - (c2.v4()*w*(c1.v5() + g_pQ->a0()*c1.v1() + g_pQ->a1()*c1.v2() + g_pQ->a2()*c1.v3() + g_pQ->b()*c1.v4() + c1.v0()*g_pQ->f2()))/powf(c2.v5() + g_pQ->a0()*c2.v1() + g_pQ->a1()*c2.v2() + g_pQ->a2()*c2.v3() + g_pQ->b()*c2.v4() + c2.v0()*g_pQ->f2(), 2);
		float v1, v2;
		for(int i = 0; i < 5; ++i)
		{
			v1 = j1[i];
			v2 = j2[i];
			if(!EQUAL(v1, v2))
				printf("%d: %f - %f = %f\n", i, v1, v2, v1 - v2);
		}
#endif
#ifdef _DEBUG_WITH_EIGEN_Q_NON_LINEAR
		g_J(g_iEq, 0) = j0123.m128_f32[0];
		g_J(g_iEq, 1) = j0123.m128_f32[1];
		g_J(g_iEq, 2) = j0123.m128_f32[2];
		g_J(g_iEq, 3) = j0123.m128_f32[3];
		g_J(g_iEq, 4) = j4;
		g_e(g_iEq) = e;
		++g_iEq;
#endif
	}
	static inline void ComputeEquationSingular(const float &w, const float &epsilon, const AbsoluteQuadric &Q, LA::Vector2f &v, LA::Vector2f &e)
	{
		v.v0() = Q.f2() * Q.b() + epsilon;
		v.v1() = Q.a0() * Q.a0() + Q.a1() * Q.a1() + Q.f2() * Q.a2() * Q.a2() + epsilon;
		e.v0() = w * (1 - v.v0() / v.v1());
		e.v1() = w * (1 - v.v1() / v.v0());
		//e.v0() = e.v1() = w * (v.v1() - v.v0());
	}
	static inline void ComputeEquationSingular(const float &w, const float &epsilon, const AbsoluteQuadric &Q, LA::Vector2f &v, LA::Vector2f &e, LA::AlignedMatrix2x5f &J)
	{
		const float &j00 = Q.b(), &j04 = Q.f2();
		float &j10 = J.reserve00(), &j11 = J.reserve01(), &j12 = J.reserve02(), &j13 = J.reserve10();
		float &c00 = e.v0(), &c01 = J.reserve11(), &c10 = e.v1(), &c11 = J.reserve12();
		j10 = Q.a2() * Q.a2();
		j11 = Q.a0() + Q.a0();
		j12 = Q.a1() + Q.a1();
		j13 = Q.f2() * (Q.a2() + Q.a2());
		v.v0() = Q.f2() * Q.b() + epsilon;
		v.v1() = Q.a0() * Q.a0() + Q.a1() * Q.a1() + Q.f2() * j10 + epsilon;
		c00 = w / v.v1();	c01 = c00 * v.v0() / v.v1();
		c10 = w / v.v0();	c11 = c10 * v.v1() / v.v0();
		J.M00() = c00*j00 - c01*j10;	J.M01() = -c01*j11;		J.M02() = -c01*j12;		J.M03() = -c01*j13;		J.M04() =  c00*j04;
		J.M10() = c10*j10 - c11*j00;	J.M11() =  c10*j11;		J.M12() =  c10*j12;		J.M13() =  c10*j13;		J.M14() = -c11*j04;
		e.v0() = w - c00 * v.v0();
		e.v1() = w - c10 * v.v1();
		//v.v0() = Q.f2() * Q.b();
		//v.v1() = Q.a0() * Q.a0() + Q.a1() * Q.a1() + Q.f2() * Q.a2() * Q.a2();
		//J.M00() = J.M10() = w * (Q.b() - Q.a2() * Q.a2());
		//J.M01() = J.M11() = -2 * w * Q.a0();
		//J.M02() = J.M12() = -2 * w * Q.a1();
		//J.M03() = J.M13() = -2 * w * Q.a2() * Q.f2();
		//J.M04() = J.M14() = w * Q.f2();
		//e.v0() = e.v1() = w * (v.v1() - v.v0());

#ifdef _DEBUG_JACOBIAN_Q
		const float *J1[] = {&J.M00(), &J.M10()};
		float J2[2][5];
		J2[0][0] = w*(Q.b()/(Q.a0()*Q.a0() + Q.a1()*Q.a1() + Q.f2()*Q.a2()*Q.a2() + epsilon) - (Q.a2()*Q.a2()*(epsilon + Q.b()*Q.f2()))/powf(Q.a0()*Q.a0() + Q.a1()*Q.a1() + Q.f2()*Q.a2()*Q.a2() + epsilon, 2));
		J2[0][1] = -(2*Q.a0()*w*(epsilon + Q.b()*Q.f2()))/powf(Q.a0()*Q.a0() + Q.a1()*Q.a1() + Q.f2()*Q.a2()*Q.a2() + epsilon, 2);
		J2[0][2] = -(2*Q.a1()*w*(epsilon + Q.b()*Q.f2()))/powf(Q.a0()*Q.a0() + Q.a1()*Q.a1() + Q.f2()*Q.a2()*Q.a2() + epsilon, 2);
		J2[0][3] = -(2*Q.a2()*Q.f2()*w*(epsilon + Q.b()*Q.f2()))/powf(Q.a0()*Q.a0() + Q.a1()*Q.a1() + Q.f2()*Q.a2()*Q.a2() + epsilon, 2);
		J2[0][4] = (Q.f2()*w)/(Q.a0()*Q.a0() + Q.a1()*Q.a1() + Q.f2()*Q.a2()*Q.a2() + epsilon);
		J2[1][0] = w*(Q.a2()*Q.a2()/(epsilon + Q.b()*Q.f2()) - (Q.b()*(Q.a0()*Q.a0() + Q.a1()*Q.a1() + Q.f2()*Q.a2()*Q.a2() + epsilon))/powf(epsilon + Q.b()*Q.f2(), 2));
		J2[1][1] = (2*Q.a0()*w)/(epsilon + Q.b()*Q.f2());
		J2[1][2] = (2*Q.a1()*w)/(epsilon + Q.b()*Q.f2());
		J2[1][3] = (2*Q.a2()*Q.f2()*w)/(epsilon + Q.b()*Q.f2());
		J2[1][4] = -(Q.f2()*w*(Q.a0()*Q.a0() + Q.a1()*Q.a1() + Q.f2()*Q.a2()*Q.a2() + epsilon))/powf(epsilon + Q.b()*Q.f2(), 2);
		float v1, v2;
		for(int i = 0; i < 2; ++i)
		for(int j = 0; j < 5; ++j)
		{
			v1 = J1[i][j];
			v2 = J2[i][j];
			if(!EQUAL(v1, v2))
				printf("(%d, %d): %f - %f = %f\n", i, j, v1, v2, v1 - v2);
		}
#endif

//#if _DEBUG
//		J.Print();
//		AbsoluteQuadric _Q;
//		LA::Vector2f _e;
//		const float delta = 0.1f;
//		float *_J[2] = {&J.M00(), &J.M10()};
//		ComputeEquationSingular(w, epsilon, Q, v, e);
//		//for(int i = 0; i < 5; ++i)
//		for(int i = 4; i >= 0; --i)
//		{
//			_Q = Q;
//			_Q[i] += delta;
//			ComputeEquationSingular(w, epsilon, _Q, v, _e);
//			_J[0][i] = -(_e.v0() - e.v0()) / delta;
//			_J[1][i] = -(_e.v1() - e.v1()) / delta;
//		}
//		J.Print();
//#endif

//#if _DEBUG
//		J.Print();
//#endif

#ifdef _DEBUG_WITH_EIGEN_Q_NON_LINEAR
		g_J(g_iEq, 0) = J.M00();
		g_J(g_iEq, 1) = J.M01();
		g_J(g_iEq, 2) = J.M02();
		g_J(g_iEq, 3) = J.M03();
		g_J(g_iEq, 4) = J.M04();
		g_e(g_iEq) = e.v0();
		++g_iEq;
		g_J(g_iEq, 0) = J.M10();
		g_J(g_iEq, 1) = J.M11();
		g_J(g_iEq, 2) = J.M12();
		g_J(g_iEq, 3) = J.M13();
		g_J(g_iEq, 4) = J.M14();
		g_e(g_iEq) = e.v1();
		++g_iEq;
#endif
	}

protected:

	float m_ws[7], m_epsilon;
	float m_d00, m_d01, m_d02, m_d11, m_d12, m_d22, m_d00I, m_d11I, m_d22I, m_d00I2, m_d11I2, m_d22I2;
	AlignedVector<Coeffient> m_cs;
	LA::AlignedMatrix9x5f m_J1;
	LA::AlignedMatrix5x9f m_J1T;
	LA::AlignedMatrix2x5f m_J2;
	LA::AlignedVector9f m_e1;
	LA::Vector2f m_v, m_e2;
	ENFT_SSE::__m128 m_work;

};


#endif