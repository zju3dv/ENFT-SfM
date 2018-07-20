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

#ifndef _SIMILARITY_TRANSFORMATION_ESTIMATOR_DATA_H_
#define _SIMILARITY_TRANSFORMATION_ESTIMATOR_DATA_H_

#include "SimilarityTransformation.h"
#include "SfM/Match.h"
#include "Optimization/OptimizerData.h"
#include "LinearAlgebra/Matrix7.h"

class SimilarityTransformationEstimatorData3D : public MatchSet3DX, public OptimizerDataTemplate<SimilarityTransformation3D, LA::AlignedVector7f, LA::AlignedMatrix7f>
{

public:

	virtual void NormalizeData(const float &dataNormalizeMedian, SimilarityTransformation3D &S) {}
	virtual void DenormalizeData(SimilarityTransformation3D &S) {}
	virtual double ComputeSSE(const SimilarityTransformation3D &S)
	{
		Point3D SX1;
		double sse = 0.0;
		const uint N = m_X1s.Size();
		for(uint i = 0; i < N; ++i)
		{
			S.Apply(m_X1s[i], SX1);
			sse += SX1.SquaredDistance(m_X2s[i]);
		}
		return sse;
	}

	virtual double GetFactorSSEToMSE() { return 1.0 / Size(); }

	virtual void ConstructNormalEquation(const SimilarityTransformation3D &S, LA::AlignedMatrix7f &A, LA::AlignedVector7f &b, LA::AlignedVector7f &s)
	{
		m_J.M04() = S.s();				m_J.M05() = 0.0f;				m_J.M06() = 0.0f;
		m_J.M14() = 0.0f;				m_J.M15() = S.s();				m_J.M16() = 0.0f;
		m_J.M24() = 0.0f;				m_J.M25() = 0.0f;				m_J.M26() = S.s();

		ENFT_SSE::__m128 &RX = m_work[3], &TX = m_work[4], &SX = m_work[5], &sRX = m_work[6], &e = m_work[7];
		A.SetZero();
		b.SetZero();
		s.SetZero();
		const uint N = m_X1s.Size();
		for(uint i = 0; i < N; ++i)
		{
			S.Apply(m_X1s[i].XYZx(), RX, TX, SX);
			sRX = ENFT_SSE::_mm_mul_ps(S.sss1(), RX);
			m_J.M00() = TX.m128_f32[0];		m_J.M01() = 0.0f;				m_J.M02() = sRX.m128_f32[2];	m_J.M03() = -sRX.m128_f32[1];
			m_J.M10() = TX.m128_f32[1];		m_J.M11() = -sRX.m128_f32[2];	m_J.M12() = 0.0f;				m_J.M13() = sRX.m128_f32[0];
			m_J.M20() = TX.m128_f32[2];		m_J.M21() = sRX.m128_f32[1];	m_J.M22() = -sRX.m128_f32[0];	m_J.M23() = 0.0f;
			e = ENFT_SSE::_mm_sub_ps(m_X2s[i].XYZx(), SX);
			LA::AddATAToUpper(m_J, A, m_work);
			LA::AddATBTo(m_J, e, b, m_work);
			LA::AddAij2To(m_J, s);
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix3x7f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedMatrix3x7f, ENFT_SSE::__m128>(b);
		LA::FinishAdditionAij2To<LA::AlignedMatrix3x7f>(s);
		LA::SetReserve<2>(b);
		LA::SetReserve<1>(s);
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}

	virtual void ConstructNormalEquation(const SimilarityTransformation3D &S, const LA::AlignedVector7f &s, LA::AlignedMatrix7f &A, LA::AlignedVector7f &b)
	{
		m_J.M04() = S.s();				m_J.M05() = 0.0f;				m_J.M06() = 0.0f;
		m_J.M14() = 0.0f;				m_J.M15() = S.s();				m_J.M16() = 0.0f;
		m_J.M24() = 0.0f;				m_J.M25() = 0.0f;				m_J.M26() = S.s();

		ENFT_SSE::__m128 &RX = m_work[3], &TX = m_work[4], &SX = m_work[5], &sRX = m_work[6], &e = m_work[7];
		A.SetZero();
		b.SetZero();
		const uint N = m_X1s.Size();
		for(uint i = 0; i < N; ++i)
		{
			S.Apply(m_X1s[i].XYZx(), RX, TX, SX);
			sRX = ENFT_SSE::_mm_mul_ps(S.sss1(), RX);
			m_J.M00() = TX.m128_f32[0];		m_J.M01() = 0.0f;				m_J.M02() = sRX.m128_f32[2];	m_J.M03() = -sRX.m128_f32[1];
			m_J.M10() = TX.m128_f32[1];		m_J.M11() = -sRX.m128_f32[2];	m_J.M12() = 0.0f;				m_J.M13() = sRX.m128_f32[0];
			m_J.M20() = TX.m128_f32[2];		m_J.M21() = sRX.m128_f32[1];	m_J.M22() = -sRX.m128_f32[0];	m_J.M23() = 0.0f;
			e = ENFT_SSE::_mm_sub_ps(m_X2s[i].XYZx(), SX);
			LA::AddATAToUpper(m_J, A, m_work);
			LA::AddATBTo(m_J, e, b, m_work);
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix3x7f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedMatrix3x7f, ENFT_SSE::__m128>(b);
		LA::SetReserve<2>(b);
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}

	virtual void UpdateModel(const LA::AlignedVector7f &s, const LA::AlignedVector7f &x, const SimilarityTransformation3D &Sold, SimilarityTransformation3D &Snew)
	{
		m_work[0] = ENFT_SSE::_mm_mul_ps(s.v0123(), x.v0123());
		Snew.SetScale(Sold.s() + m_work[0].m128_f32[0]);
		m_dR.FromRodrigues(m_work[0].m128_f32[1], m_work[0].m128_f32[2], m_work[0].m128_f32[3], (float *) &m_work[1]);
		Sold.LeftMultiplyRotation(m_dR, Snew, m_work[0]);
		m_work[0] = ENFT_SSE::_mm_mul_ps(s.v456x(), x.v456x());
		Snew.tX() = m_work[0].m128_f32[0] + Sold.tX();
		Snew.tY() = m_work[0].m128_f32[1] + Sold.tY();
		Snew.tZ() = m_work[0].m128_f32[2] + Sold.tZ();
	}

protected:

	LA::AlignedMatrix3x7f m_J;
	RotationTransformation3D m_dR;
	ENFT_SSE::__m128 m_work[10];

};

class SimilarityTransformationEstimatorData2D : public SimilarityTransformationEstimatorData3D
{

public:

	inline const Camera* pC2(const uint &i) const { return m_pC2s[i]; }		inline const Camera* &pC2(const uint &i) { return m_pC2s[i]; }
	inline const Point2D& x2(const uint &i) const { return m_x2s[i]; }		inline Point2D& x2(const uint &i) { return m_x2s[i]; }
	inline const AlignedVector<Point2D>& x2s() const { return m_x2s; }		inline AlignedVector<Point2D>& x2s() { return m_x2s; }
	inline const uint& GetPointMeasurementIndex(const uint &iPt) const { return m_mapPtToMea[iPt]; }
	inline void SetPointMeasurementIndex(const uint &iPt, const uint &iMea) { m_mapPtToMea[iPt] = iMea; }
	inline void Resize(const uint &nPts, const uint &nMeas)
	{
		SimilarityTransformationEstimatorData3D::Resize(nPts);
		m_mapPtToMea.resize(nPts + 1);
		m_mapPtToMea[nPts] = nMeas;
		m_pC2s.resize(nMeas);
		m_x2s.Resize(nMeas);
		m_ws.Resize(0);
		m_wsSq.Resize(0);
	}
	inline bool AreWeightsValid() const { return m_ws.Size() == m_pC2s.size(); }
	inline void ValidateWeights() { m_ws.Resize(uint(m_pC2s.size())); m_wsSq.Resize(m_ws.Size()); }
	inline const float& GetSquaredWeight(const uint &iMea) const { return m_wsSq[iMea]; }
	inline void SetWeight(const ushort &iMea, const float &w) { m_ws[iMea] = w; m_wsSq[iMea] = w * w; }
	inline void SetSquaredWeight(const uint &iMea, const float &wSq) { m_wsSq[iMea] = wSq; m_ws[iMea] = sqrt(wSq); }
	inline void GetSubset(const std::vector<uint> &idxs, SimilarityTransformationEstimatorData2D &subset) const
	{
		const uint nPtsDst = uint(idxs.size());
		uint nMeasDst = 0;
		uint iPtSrc, iPtDst;
		for(iPtDst = 0; iPtDst < nPtsDst; ++iPtDst)
		{
			iPtSrc = idxs[iPtDst];
			nMeasDst += m_mapPtToMea[iPtSrc + 1] - m_mapPtToMea[iPtSrc];
		}
		subset.Resize(nPtsDst, nMeasDst);

		uint iMeaSrc, iMeaDst;
		for(iPtDst = 0, iMeaDst = 0; iPtDst < nPtsDst; ++iPtDst)
		{
			iPtSrc = idxs[iPtDst];
			subset.m_X1s[iPtDst] = m_X1s[iPtSrc];
			subset.m_X2s[iPtDst] = m_X2s[iPtSrc];
			subset.m_mapPtToMea[iPtDst] = iMeaDst;
			const uint iMeaSrc1 = m_mapPtToMea[iPtSrc], iMeaSrc2 = m_mapPtToMea[iPtSrc + 1];
			for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc, ++iMeaDst)
			{
				subset.m_pC2s[iMeaDst] = m_pC2s[iMeaSrc];
				subset.m_x2s[iMeaDst] = m_x2s[iMeaSrc];
			}
		}
		subset.m_mapPtToMea[iPtDst] = iMeaDst;

		if(AreWeightsValid())
		{
			subset.ValidateWeights();
			uint iMeaSrc, iMeaDst;
			for(iPtDst = 0, iMeaDst = 0; iPtDst < nPtsDst; ++iPtDst)
			{
				iPtSrc = idxs[iPtDst];
				const uint iMeaSrc1 = m_mapPtToMea[iPtSrc], iMeaSrc2 = m_mapPtToMea[iPtSrc + 1];
				for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc, ++iMeaDst)
				{
					subset.m_ws[iMeaDst] = m_ws[iMeaSrc];
					subset.m_wsSq[iMeaDst] = m_wsSq[iMeaSrc];
				}
			}
		}

		subset.m_fxy = m_fxy;
	}
	inline void SetFocal(const float &fxy) { m_fxy = fxy; }

	virtual double ComputeSSE(const SimilarityTransformation3D &S)
	{
		uint i, j;
		Point3D SX1;
		Point2D e;
		double sse = 0.0;
		const uint nPts = m_X1s.Size();
		if(AreWeightsValid())
		{
			for(i = 0; i < nPts; ++i)
			{
				S.Apply(m_X1s[i], SX1);
				const uint j1 = m_mapPtToMea[i], j2 = m_mapPtToMea[i + 1];
				for(j = j1; j < j2; ++j)
					sse += m_pC2s[j]->ComputeProjectionSquaredError(SX1, m_x2s[j], e) * m_wsSq[j];
			}
		}
		else
		{
			for(i = 0; i < nPts; ++i)
			{
				S.Apply(m_X1s[i], SX1);
				const uint j1 = m_mapPtToMea[i], j2 = m_mapPtToMea[i + 1];
				for(j = j1; j < j2; ++j)
					sse += m_pC2s[j]->ComputeProjectionSquaredError(SX1, m_x2s[j], e);
			}
		}
		return sse;
	}

	virtual double GetFactorSSEToMSE() { return double(m_fxy / m_mapPtToMea.back()); }

	virtual void ConstructNormalEquation(const SimilarityTransformation3D &S, LA::AlignedMatrix7f &A, LA::AlignedVector7f &b, LA::AlignedVector7f &s)
	{
		uint i, j;
		ENFT_SSE::__m128 &RX = m_work[3], &TX = m_work[4], &SX = m_work[5], &sRX = m_work[6], &ZcI = m_work[7], &wZcI = m_work[7], &JxSX = m_work[8], &JySX = m_work[9];
		Point2D x, &e = x;
		A.SetZero();
		b.SetZero();
		s.SetZero();
		TX.m128_f32[3] = 1.0f;
		const uint nPts = m_X1s.Size();
		if(AreWeightsValid())
		{
			for(i = 0; i < nPts; ++i)
			{
				S.Apply(m_X1s[i].XYZx(), RX, TX, SX);
				sRX = ENFT_SSE::_mm_mul_ps(S.sss1(), RX);
				const uint j1 = m_mapPtToMea[i], j2 = m_mapPtToMea[i + 1];
				for(j = j1; j < j2; ++j)
				{
					const Camera &C = *m_pC2s[j];
					C.ProjectToNormalizedPlane(SX, wZcI.m128_f32[0], x.x(), x.y());
					wZcI = _mm_set1_ps(m_ws[j] * wZcI.m128_f32[0]);
					JxSX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(wZcI, C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(wZcI.m128_f32[0] * x.x()), C.r_20_21_22_x()));
					JySX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(wZcI, C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(wZcI.m128_f32[0] * x.y()), C.r_20_21_22_x()));
					m_J.M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(JxSX, TX));
					m_J.M01() = JxSX.m128_f32[2] * sRX.m128_f32[1] - JxSX.m128_f32[1] * sRX.m128_f32[2];
					m_J.M02() = JxSX.m128_f32[0] * sRX.m128_f32[2] - JxSX.m128_f32[2] * sRX.m128_f32[0];
					m_J.M03() = JxSX.m128_f32[1] * sRX.m128_f32[0] - JxSX.m128_f32[0] * sRX.m128_f32[1];
					m_J.M_04_05_06_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), JxSX);
					m_J.M10() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(JySX, TX));
					m_J.M11() = JySX.m128_f32[2] * sRX.m128_f32[1] - JySX.m128_f32[1] * sRX.m128_f32[2];
					m_J.M12() = JySX.m128_f32[0] * sRX.m128_f32[2] - JySX.m128_f32[2] * sRX.m128_f32[0];
					m_J.M13() = JySX.m128_f32[1] * sRX.m128_f32[0] - JySX.m128_f32[0] * sRX.m128_f32[1];
					m_J.M_14_15_16_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), JySX);
					LA::AmB(m_x2s[j], x, e);
					e *= m_ws[i];
					LA::AddATAToUpper(m_J, A, m_work);
					LA::AddATBTo(m_J, e, b, m_work);
					LA::AddAij2To(m_J, s);
				}
			}
		}
		else
		{
			for(i = 0; i < nPts; ++i)
			{
				S.Apply(m_X1s[i].XYZx(), RX, TX, SX);
				sRX = ENFT_SSE::_mm_mul_ps(S.sss1(), RX);
				const uint j1 = m_mapPtToMea[i], j2 = m_mapPtToMea[i + 1];
				for(j = j1; j < j2; ++j)
				{
					const Camera &C = *m_pC2s[j];
					C.ProjectToNormalizedPlane(SX, ZcI.m128_f32[0], x.x(), x.y());
					ZcI = _mm_set1_ps(ZcI.m128_f32[0]);
					JxSX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ZcI, C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI.m128_f32[0] * x.x()), C.r_20_21_22_x()));
					JySX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ZcI, C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI.m128_f32[0] * x.y()), C.r_20_21_22_x()));
					m_J.M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(JxSX, TX));
					m_J.M01() = JxSX.m128_f32[2] * sRX.m128_f32[1] - JxSX.m128_f32[1] * sRX.m128_f32[2];
					m_J.M02() = JxSX.m128_f32[0] * sRX.m128_f32[2] - JxSX.m128_f32[2] * sRX.m128_f32[0];
					m_J.M03() = JxSX.m128_f32[1] * sRX.m128_f32[0] - JxSX.m128_f32[0] * sRX.m128_f32[1];
					m_J.M_04_05_06_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), JxSX);
					m_J.M10() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(JySX, TX));
					m_J.M11() = JySX.m128_f32[2] * sRX.m128_f32[1] - JySX.m128_f32[1] * sRX.m128_f32[2];
					m_J.M12() = JySX.m128_f32[0] * sRX.m128_f32[2] - JySX.m128_f32[2] * sRX.m128_f32[0];
					m_J.M13() = JySX.m128_f32[1] * sRX.m128_f32[0] - JySX.m128_f32[0] * sRX.m128_f32[1];
					m_J.M_14_15_16_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), JySX);
					LA::AmB(m_x2s[j], x, e);
					LA::AddATAToUpper(m_J, A, m_work);
					LA::AddATBTo(m_J, e, b, m_work);
					LA::AddAij2To(m_J, s);
				}
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix3x7f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedMatrix3x7f, ENFT_SSE::__m128>(b);
		LA::SetReserve<2>(b);
		LA::FinishAdditionAij2To<LA::AlignedMatrix3x7f>(s);
		LA::SetReserve<1>(s);
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}

	virtual void ConstructNormalEquation(const SimilarityTransformation3D &S, const LA::AlignedVector7f &s, LA::AlignedMatrix7f &A, LA::AlignedVector7f &b)
	{
		uint i, j;
		ENFT_SSE::__m128 &RX = m_work[3], &TX = m_work[4], &SX = m_work[5], &sRX = m_work[6], &ZcI = m_work[7], &wZcI = m_work[7], &JxSX = m_work[8], &JySX = m_work[9];
		Point2D x, &e = x;
		A.SetZero();
		b.SetZero();
		TX.m128_f32[3] = 1.0f;
		const uint nPts = m_X1s.Size();
		if(AreWeightsValid())
		{
			for(i = 0; i < nPts; ++i)
			{
				S.Apply(m_X1s[i].XYZx(), RX, TX, SX);
				sRX = ENFT_SSE::_mm_mul_ps(S.sss1(), RX);
				const uint j1 = m_mapPtToMea[i], j2 = m_mapPtToMea[i + 1];
				for(j = j1; j < j2; ++j)
				{
					const Camera &C = *m_pC2s[j];
					C.ProjectToNormalizedPlane(SX, wZcI.m128_f32[0], x.x(), x.y());
					wZcI = _mm_set1_ps(m_ws[j] * wZcI.m128_f32[0]);
					JxSX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(wZcI, C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(wZcI.m128_f32[0] * x.x()), C.r_20_21_22_x()));
					JySX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(wZcI, C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(wZcI.m128_f32[0] * x.y()), C.r_20_21_22_x()));
					m_J.M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(JxSX, TX));
					m_J.M01() = JxSX.m128_f32[2] * sRX.m128_f32[1] - JxSX.m128_f32[1] * sRX.m128_f32[2];
					m_J.M02() = JxSX.m128_f32[0] * sRX.m128_f32[2] - JxSX.m128_f32[2] * sRX.m128_f32[0];
					m_J.M03() = JxSX.m128_f32[1] * sRX.m128_f32[0] - JxSX.m128_f32[0] * sRX.m128_f32[1];
					m_J.M_04_05_06_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), JxSX);
					m_J.M10() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(JySX, TX));
					m_J.M11() = JySX.m128_f32[2] * sRX.m128_f32[1] - JySX.m128_f32[1] * sRX.m128_f32[2];
					m_J.M12() = JySX.m128_f32[0] * sRX.m128_f32[2] - JySX.m128_f32[2] * sRX.m128_f32[0];
					m_J.M13() = JySX.m128_f32[1] * sRX.m128_f32[0] - JySX.m128_f32[0] * sRX.m128_f32[1];
					m_J.M_14_15_16_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), JySX);
					LA::AmB(m_x2s[j], x, e);
					e *= m_ws[i];
					LA::AddATAToUpper(m_J, A, m_work);
					LA::AddATBTo(m_J, e, b, m_work);
				}
			}
		}
		else
		{
			for(i = 0; i < nPts; ++i)
			{
				S.Apply(m_X1s[i].XYZx(), RX, TX, SX);
				sRX = ENFT_SSE::_mm_mul_ps(S.sss1(), RX);
				const uint j1 = m_mapPtToMea[i], j2 = m_mapPtToMea[i + 1];
				for(j = j1; j < j2; ++j)
				{
					const Camera &C = *m_pC2s[j];
					C.ProjectToNormalizedPlane(SX, ZcI.m128_f32[0], x.x(), x.y());
					ZcI = _mm_set1_ps(ZcI.m128_f32[0]);
					JxSX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ZcI, C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI.m128_f32[0] * x.x()), C.r_20_21_22_x()));
					JySX = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ZcI, C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(ZcI.m128_f32[0] * x.y()), C.r_20_21_22_x()));
					m_J.M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(JxSX, TX));
					m_J.M01() = JxSX.m128_f32[2] * sRX.m128_f32[1] - JxSX.m128_f32[1] * sRX.m128_f32[2];
					m_J.M02() = JxSX.m128_f32[0] * sRX.m128_f32[2] - JxSX.m128_f32[2] * sRX.m128_f32[0];
					m_J.M03() = JxSX.m128_f32[1] * sRX.m128_f32[0] - JxSX.m128_f32[0] * sRX.m128_f32[1];
					m_J.M_04_05_06_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), JxSX);
					m_J.M10() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(JySX, TX));
					m_J.M11() = JySX.m128_f32[2] * sRX.m128_f32[1] - JySX.m128_f32[1] * sRX.m128_f32[2];
					m_J.M12() = JySX.m128_f32[0] * sRX.m128_f32[2] - JySX.m128_f32[2] * sRX.m128_f32[0];
					m_J.M13() = JySX.m128_f32[1] * sRX.m128_f32[0] - JySX.m128_f32[0] * sRX.m128_f32[1];
					m_J.M_14_15_16_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), JySX);
					LA::AmB(m_x2s[j], x, e);
					LA::AddATAToUpper(m_J, A, m_work);
					LA::AddATBTo(m_J, e, b, m_work);
				}
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix3x7f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedMatrix3x7f, ENFT_SSE::__m128>(b);
		LA::SetReserve<2>(b);
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}

protected:

	float m_fxy;
	std::vector<uint> m_mapPtToMea;
	std::vector<const Camera *> m_pC2s;
	AlignedVector<Point2D> m_x2s;
	AlignedVector<float> m_ws, m_wsSq;
	LA::AlignedMatrix2x7f m_J;

};

#endif