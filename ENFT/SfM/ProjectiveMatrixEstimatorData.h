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

#ifndef _PROJECTIVE_MATRIX_ESTIMATOR_DATA_H_
#define _PROJECTIVE_MATRIX_ESTIMATOR_DATA_H_

#include "SfM/Match.h"
#include "Estimation/EstimatorArsacData.h"
#include "Optimization/OptimizerData.h"
#include "ProjectiveMatrix.h"
#include "LinearAlgebra/Matrix11.h"
#include "LinearAlgebra/Matrix7.h"
#include "LinearAlgebra/Matrix8.h"

class ProjectiveMatrixEstimatorData : public MatchSet3DTo2DX, public EstimatorArsacData, 
									  public OptimizerDataTemplate<ProjectiveMatrix, LA::AlignedVector11f, LA::AlignedMatrix11f>
{

public:

	inline void SetArsacData(const ushort &width, const ushort &height, const IntrinsicMatrix &K, const bool &measNormalized)
	{
		SetImageSize(width, height);
		if(measNormalized)
			K.NormalizedPlaneToImageN(m_xs, m_imgLocations);
		else
		{
			m_xs.Swap(m_imgLocations);
			K.ImageToNormalizedPlaneN(m_imgLocations, m_xs);
		}
		m_fxy = K.fxy();
	}

	inline void GetSubset(const std::vector<ushort> &idxs, ProjectiveMatrixEstimatorData &subset) const
	{
		MatchSet3DTo2DX::GetSubset(idxs, subset);
		subset.m_fxy = m_fxy;
	}

	inline void SetFocal(const float &fxy) { m_fxy = fxy; }
	inline const float& GetFocal() const { return m_fxy; }

	virtual void NormalizeData(const float &dataNormalizeMedian, ProjectiveMatrix &P)
	{
		if(dataNormalizeMedian == 0)
		//if(1)
		{
			m_sX = m_sx = _mm_set1_ps(1.0f);
			m_cX.SetZero();
			m_cx.SetZero();
		}
		else
		{
			Normalize();
			P.Normalize(m_cX, m_sX, m_cx, m_sx);
		}
	}
	virtual void DenormalizeData(ProjectiveMatrix &P)
	{
		P.Denormalize(m_cX, m_sX, m_cx, m_sx, m_work);
	}
	virtual double ComputeSSE(const ProjectiveMatrix &P)
	{
		ENFT_SSE::__m128 e2, work;
		ENFT_SSE::__m128 SSE = ENFT_SSE::_mm_setzero_ps();
		const ushort N = Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) m_xs.Data();
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
		{
			P.ComputeProjectionError2(m_Xs[ix2], m_Xs[ix2p1], *px2, e2, work);
			SSE = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e2, e2), SSE);
		}
		if(_N != N)
		{
			Point2D e;
			SSE.m128_f32[0] += P.ComputeProjectionSquaredError(m_Xs[_N], m_xs[_N], e);
		}
		return ENFT_SSE::SSE::Sum0123(SSE);
	}

	virtual double GetFactorSSEToMSE() { return m_fxy / (m_sx.m128_f32[0] * m_sx.m128_f32[0] * Size()); }
	virtual void ConstructNormalEquation(const ProjectiveMatrix &P, LA::AlignedMatrix11f &A, LA::AlignedVector11f &b, LA::AlignedVector11f &s)
	{
		float ZcI;
		Point2D x, &e = x;

		A.SetZero();
		b.SetZero();
		s.SetZero();
		m_J.M_04_05_06_07() = m_J.M_10_11_12_13() = ENFT_SSE::_mm_setzero_ps();
		const ushort N = Size();
		for(ushort i = 0; i < N; ++i)
		{
			const Point3D &X = m_Xs[i];
			P.Project(X, ZcI, x);
			m_work[0] = _mm_set1_ps(ZcI);
			m_work[1] = _mm_set1_ps(-x.x() * ZcI);
			m_work[2] = _mm_set1_ps(-x.y() * ZcI);
			m_J.M_00_01_02_03() = m_J.M_14_15_16_17() = ENFT_SSE::_mm_mul_ps(m_work[0], X.XYZx());
			m_J.M_08_09_010_x() = ENFT_SSE::_mm_mul_ps(m_work[1], X.XYZx());
			m_J.M_18_19_110_x() = ENFT_SSE::_mm_mul_ps(m_work[2], X.XYZx());
			LA::AmB(m_xs[i], x, e);
			AddATAToUpper(m_J, A, m_work[0]);
			AddATBTo(m_J, e, b, m_work);
			AddAij2To(m_J, s);
		}
		FinishAdditionATAToUpper(A);
		A.SetLowerFromUpper();
		//A.Print();
		FinishAdditionAij2To(s);
		LA::SetReserve<2>(b);
		LA::SetReserve<1>(s);
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		//s.v0123() = s.v4567() = s.v8910x() = _mm_set1_ps(1.0f);
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const ProjectiveMatrix &P, const LA::AlignedVector11f &s, LA::AlignedMatrix11f &A, LA::AlignedVector11f &b)
	{
		float ZcI;
		Point2D x, &e = x;

		A.SetZero();
		b.SetZero();
		m_J.M_04_05_06_07() = m_J.M_10_11_12_13() = ENFT_SSE::_mm_setzero_ps();
		const ushort N = Size();
		for(ushort i = 0; i < N; ++i)
		{
			const Point3D &X = m_Xs[i];
			P.Project(X, ZcI, x);
			m_work[0] = _mm_set1_ps(ZcI);
			m_work[1] = _mm_set1_ps(-x.x() * ZcI);
			m_work[2] = _mm_set1_ps(-x.y() * ZcI);
			m_J.M_00_01_02_03() = m_J.M_14_15_16_17() = ENFT_SSE::_mm_mul_ps(m_work[0], X.XYZx());
			m_J.M_08_09_010_x() = ENFT_SSE::_mm_mul_ps(m_work[1], X.XYZx());
			m_J.M_18_19_110_x() = ENFT_SSE::_mm_mul_ps(m_work[2], X.XYZx());
			LA::AmB(m_xs[i], x, e);
			AddATAToUpper(m_J, A, m_work[0]);
			AddATBTo(m_J, e, b, m_work);
		}
		FinishAdditionATAToUpper(A);
		A.SetLowerFromUpper();
		//A.Print();
		LA::SetReserve<2>(b);
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}
	virtual void UpdateModel(const LA::AlignedVector11f &s, const LA::AlignedVector11f &x, const ProjectiveMatrix &Pold, ProjectiveMatrix &Pnew)
	{
		Pnew.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s.v0123(), x.v0123()), Pold.M_00_01_02_03());
		Pnew.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s.v4567(), x.v4567()), Pold.M_10_11_12_13());
		Pnew.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s.v8910x(), x.v8910x()), Pold.M_20_21_22_23());
#if _DEBUG
		assert(Pnew.M23() == Pold.M23());
#endif

	}

protected:

	static inline void AddATAToUpper(const LA::AlignedMatrix2x11f &A, LA::AlignedMatrix11f &to, ENFT_SSE::__m128 &work)
	{
		work = _mm_set1_ps(A.M00());
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.M_00_01_02_03()), to.M_00_01_02_03());
		to.M_08_09_010_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.M_08_09_010_x()), to.M_08_09_010_x());
		work = _mm_set1_ps(A.M01());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.M_00_01_02_03()), to.M_10_11_12_13());
		to.M_18_19_110_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.M_08_09_010_x()), to.M_18_19_110_x());
		to.M22() = A.M02() * A.M02() + to.M22();
		to.M23() = A.M02() * A.M03() + to.M23();
		to.M_28_29_210_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A.M02()), A.M_08_09_010_x()), to.M_28_29_210_x());
		to.M33() = A.M03() * A.M03() + to.M33();
		to.M_38_39_310_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A.M03()), A.M_08_09_010_x()), to.M_38_39_310_x());
		to.M_48_49_410_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A.M14()), A.M_18_19_110_x()), to.M_48_49_410_x());
		to.M_58_59_510_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A.M15()), A.M_18_19_110_x()), to.M_58_59_510_x());
		to.M_68_69_610_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A.M16()), A.M_18_19_110_x()), to.M_68_69_610_x());
		to.M_78_79_710_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A.M17()), A.M_18_19_110_x()), to.M_78_79_710_x());
		to.M_88_89_810_x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A.M08()), A.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A.M18()), A.M_18_19_110_x())), to.M_88_89_810_x());
		to.M99() = A.M09() * A.M09() + A.M19() * A.M19() + to.M99();
		to.M910() = A.M09() * A.M010() + A.M19() * A.M110() + to.M910();
		to.M1010() = A.M010() * A.M010() + A.M110() * A.M110() + to.M1010();
	}
	static inline void FinishAdditionATAToUpper(LA::AlignedMatrix11f &to)
	{
		to.M_44_45_46_47() = to.M_00_01_02_03();
		to.M_54_55_56_57() = to.M_10_11_12_13();
		to.M66() = to.M22();
		to.M67() = to.M23();
		to.M77() = to.M33();
	}
	static inline void AddATBTo(const LA::AlignedMatrix2x11f &A, const LA::Vector2f &B, LA::AlignedVector11f &to, ENFT_SSE::__m128 *work2)
	{
		work2[0] = _mm_set1_ps(B.v0());
		work2[1] = _mm_set1_ps(B.v1());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), work2[1]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), work2[1])), to.v8910x());
	}
	static inline void AddAij2To(const LA::AlignedMatrix2x11f &A, LA::AlignedVector11f &to)
	{
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_00_01_02_03()), to.v0123());
		to.v8910x() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), A.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), A.M_18_19_110_x())), to.v8910x());
	}
	static inline void FinishAdditionAij2To(LA::AlignedVector11f &to)
	{
		to.v4567() = to.v0123();
	}

protected:

	float m_fxy;
	RotationTransformation3D m_dR;
	LA::AlignedMatrix2x11f m_J;
	ENFT_SSE::__m128 m_work[7];

};

class ProjectiveMatrixEstimatorDataMetric : public MatchSet3DTo2DX, public EstimatorArsacData, 
											public OptimizerDataTemplate<ProjectiveMatrixMetric, LA::AlignedVector7f, LA::AlignedMatrix7f>
{

public:

	inline void SetArsacData(const ushort &width, const ushort &height, const IntrinsicMatrix &K, const bool &measNormalized)
	{
		SetImageSize(width, height);
		if(measNormalized)
			K.NormalizedPlaneToImageN(m_xs, m_imgLocations);
		else
		{
			m_xs.Swap(m_imgLocations);
			K.ImageToNormalizedPlaneN(m_imgLocations, m_xs);
		}
		m_fxy = K.fxy();
	}

	inline void GetSubset(const std::vector<ushort> &idxs, ProjectiveMatrixEstimatorDataMetric &subset) const
	{
		MatchSet3DTo2DX::GetSubset(idxs, subset);
		subset.m_fxy = m_fxy;
		subset.m_wPrior = m_wPrior;
	}

	inline void operator = (const ProjectiveMatrixEstimatorData &data)
	{
		m_Xs = data.Xs();
		m_xs = data.xs();
		m_fxy = data.GetFocal();
	}

	inline void SetFocal(const float &fxy) { m_fxy = fxy; }
	inline const float& GetFocal() const { return m_fxy; }
	inline void SetPriorWeight(const float &w) { m_wPrior = w; }
	inline const float& GetPriorWeight() const { return m_wPrior; }

	virtual void NormalizeData(const float &dataNormalizeMedian, ProjectiveMatrixMetric &P)
	{
		if(dataNormalizeMedian == 0)
		//if(1)
		{
			m_sX = m_sx = _mm_set1_ps(1.0f);
			m_cX.SetZero();
			m_cx.SetZero();
			m_fPrior = 1.0f;
		}
		else
		{
			// Translate scene
			const ushort N = Size();
			m_cX.SetZero();
			for(ushort i = 0; i < N; ++i)
				m_cX += m_Xs[i];
			m_cX *= _mm_set1_ps(1.0f / N);
			m_cX.reserve() = 0;
			for(ushort i = 0; i < N; ++i)
				m_Xs[i] -= m_cX;
			P.C().ApplyRotation(m_cX, m_dt);
			P.C().IncreaseTranslation(m_dt);

			// Scale scene
			m_ds.resize(N);
			for(ushort i = 0; i < N; ++i)
				m_ds[i] = P.C().ComputeDepth(m_Xs[i]);
			const ushort ith = ushort(m_ds.size() >> 1);
			std::nth_element(m_ds.begin(), m_ds.begin() + ith, m_ds.end());
			const float dMed = m_ds[ith];
			m_sX = _mm_set1_ps((1 / dMed) / dataNormalizeMedian);
			P.C().Scale(m_sX.m128_f32[0]);
			for(ushort i = 0; i < N; ++i)
			{
				m_Xs[i] *= m_sX;
				m_Xs[i].reserve() = 1.0f;
			}

			// Normalize focal
			m_sx = _mm_set1_ps(dataNormalizeMedian / P.f());
			P.f() *= m_sx.m128_f32[0];
			const ushort _N = N - (N & 1);
			ENFT_SSE::__m128 *x2 = (ENFT_SSE::__m128 *) m_xs.Data();
			for(ushort i = 0; i < _N; i += 2, ++x2)
				*x2 = ENFT_SSE::_mm_mul_ps(*x2, m_sx);
			if(_N != N)
				m_xs[_N] *= m_sx.m128_f32[0];
			m_cx.SetZero();

			P.Normalize(m_cX, m_sX, m_cx, m_sx);

			m_fPrior = dataNormalizeMedian;
		}
	}
	virtual void DenormalizeData(ProjectiveMatrixMetric &P)
	{
		P.Denormalize(m_cX, m_sX, m_cx, m_sx, m_work);
		P.f() /= m_sx.m128_f32[0];
		P.C().Scale(1 / m_sX.m128_f32[0]);
		P.C().ApplyRotation(m_cX, m_dt);
		P.C().DecreaseTranslation(m_dt);
	}
	virtual double ComputeSSE(const ProjectiveMatrixMetric &P)
	{
		ENFT_SSE::__m128 e2, work;
		ENFT_SSE::__m128 SSE = ENFT_SSE::_mm_setzero_ps();
		const ushort N = Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) m_xs.Data();
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
		{
			P.ComputeProjectionError2(m_Xs[ix2], m_Xs[ix2p1], *px2, e2, work);
			SSE = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e2, e2), SSE);
		}
		Point2D e;
		if(_N != N)
			SSE.m128_f32[0] += P.ComputeProjectionSquaredError(m_Xs[_N], m_xs[_N], e);
		e.v0() = P.f() / m_fPrior - 1;
		e.v1() = m_fPrior / P.f() - 1;
		e *= m_wPrior * N;
		SSE.m128_f32[0] += e.SquaredLength();
		return ENFT_SSE::SSE::Sum0123(SSE);
	}

	virtual double GetFactorSSEToMSE() { return m_fxy / (m_sx.m128_f32[0] * m_sx.m128_f32[0] * Size()); }
	virtual void ConstructNormalEquation(const ProjectiveMatrixMetric &P, LA::AlignedMatrix7f &A, LA::AlignedVector7f &b, LA::AlignedVector7f &s)
	{
		float RX_X, RX_Y, RX_Z, ZcI;
		Point2D x, &e = x;

		A.SetZero();
		b.SetZero();
		s.SetZero();
		const float f = P.f();
		const Camera &C = P.C();
		const ushort N = Size();
		for(ushort i = 0; i < N; ++i)
		{
			C.ProjectToNormalizedPlane(m_Xs[i], RX_X, RX_Y, RX_Z, ZcI, x.x(), x.y());
			m_J.M00()=x.x();	m_J.M01()=f*ZcI;	m_J.M02()=0;			m_J.M03()=-x.x()*m_J.M01();	m_J.M04()=m_J.M03()*RX_Y;					m_J.M05()=m_J.M01()*RX_Z-m_J.M03()*RX_X;	m_J.M06()=-m_J.M01()*RX_Y;
			m_J.M10()=x.y();	m_J.M11()=0;		m_J.M12()=m_J.M01();	m_J.M13()=-x.y()*m_J.M12();	m_J.M14()=-m_J.M12()*RX_Z+m_J.M13()*RX_Y;	m_J.M15()=-m_J.M13()*RX_X;					m_J.M16()=m_J.M12()*RX_X;
			//m_J.Print();
			x *= f;
			LA::AmB(m_xs[i], x, e);
			LA::AddATAToUpper(m_J, A, m_work);
			LA::AddATBTo(m_J, e, b, m_work);
			LA::AddAij2To(m_J, s);
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x7f>(A);
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x7f, LA::Vector2f>(b);
		LA::FinishAdditionAij2To<LA::AlignedMatrix2x7f>(s);
		const float w = m_wPrior * N, w2 = w * w;
		const float j1 = 1 / m_fPrior, e1 = f / m_fPrior - 1;
		const float j2 = -m_fPrior / (f * f), e2 = m_fPrior / f - 1;
		const float wjTwj = w2 * (j1 * j1 + j2 * j2);
		A.M00() += wjTwj;
		b.v0() -= w2 * (j1 * e1 + j2 * e2);
		s.v0() += wjTwj;
		A.SetLowerFromUpper();
		//A.Print();
		LA::SetReserve<2>(b);
		LA::SetReserve<1>(s);
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const ProjectiveMatrixMetric &P, const LA::AlignedVector7f &s, LA::AlignedMatrix7f &A, LA::AlignedVector7f &b)
	{
		float RX_X, RX_Y, RX_Z, ZcI;
		Point2D x, &e = x;

		A.SetZero();
		b.SetZero();
		const float f = P.f();
		const Camera &C = P.C();
		const ushort N = Size();
		for(ushort i = 0; i < N; ++i)
		{
			C.ProjectToNormalizedPlane(m_Xs[i], RX_X, RX_Y, RX_Z, ZcI, x.x(), x.y());
			m_J.M00()=x.x();	m_J.M01()=f*ZcI;	m_J.M02()=0;			m_J.M03()=-x.x()*m_J.M01();	m_J.M04()=m_J.M03()*RX_Y;					m_J.M05()=m_J.M01()*RX_Z-m_J.M03()*RX_X;	m_J.M06()=-m_J.M01()*RX_Y;
			m_J.M10()=x.y();	m_J.M11()=0;		m_J.M12()=m_J.M01();	m_J.M13()=-x.y()*m_J.M12();	m_J.M14()=-m_J.M12()*RX_Z+m_J.M13()*RX_Y;	m_J.M15()=-m_J.M13()*RX_X;					m_J.M16()=m_J.M12()*RX_X;
			x *= f;
			LA::AmB(m_xs[i], x, e);
			LA::AddATAToUpper(m_J, A, m_work);
			LA::AddATBTo(m_J, e, b, m_work);
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x7f>(A);
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x7f, LA::Vector2f>(b);
		const float w = m_wPrior * N, w2 = w * w;
		const float j1 = 1 / m_fPrior, e1 = f / m_fPrior - 1;
		const float j2 = -m_fPrior / (f * f), e2 = m_fPrior / f - 1;
		const float wjTwj = w2 * (j1 * j1 + j2 * j2);
		A.M00() += wjTwj;
		b.v0() -= w2 * (j1 * e1 + j2 * e2);
		A.SetLowerFromUpper();
		LA::SetReserve<2>(b);
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}
	virtual void UpdateModel(const LA::AlignedVector7f &s, const LA::AlignedVector7f &x, const ProjectiveMatrixMetric &Pold, ProjectiveMatrixMetric &Pnew)
	{
		m_work[0] = ENFT_SSE::_mm_mul_ps(s.v0123(), x.v0123());
		Pnew.f() = m_work[0].m128_f32[0] + Pold.f();
		Pnew.C().tX() = m_work[0].m128_f32[1] + Pold.C().tX();
		Pnew.C().tY() = m_work[0].m128_f32[2] + Pold.C().tY();
		Pnew.C().tZ() = m_work[0].m128_f32[3] + Pold.C().tZ();
		m_work[0] = ENFT_SSE::_mm_mul_ps(s.v456x(), x.v456x());
		m_dR.FromRodrigues(m_work[0].m128_f32[0], m_work[0].m128_f32[1], m_work[0].m128_f32[2], (float *) &m_work[1]);
		Pold.C().LeftMultiplyRotation(m_dR, Pnew.C(), m_work[0]);
		Pnew.FromIntrinsicExtrinsic(Pnew.f(), Pnew.C());
	}

protected:

	float m_fxy, m_wPrior, m_fPrior;
	Point3D m_dt;
	RotationTransformation3D m_dR;
	LA::AlignedMatrix2x7f m_J;
	std::vector<float> m_ds;
	ENFT_SSE::__m128 m_work[7];

};

#endif