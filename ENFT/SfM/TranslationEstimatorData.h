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

#ifndef _TRANSLATION_ESTIMATOR_DATA_H_
#define _TRANSLATION_ESTIMATOR_DATA_H_

#include "SfM/Match.h"
#include "Estimation/EstimatorArsacData.h"
#include "Translation.h"
#include "Optimization/OptimizerData.h"
#include "LinearAlgebra/Matrix3.h"

class TranslationEstimatorData : public MatchSet3DTo2DX, public EstimatorArsacData, public OptimizerDataTemplate<Translation, LA::AlignedVector3f, LA::AlignedMatrix3f>
{

public:

	inline void SetArsacData(const ushort &width, const ushort &height, const IntrinsicMatrix &K, const bool &measNormalized)
	{
		SetImageSize(width, height);
		if(measNormalized)
			K.NormalizedPlaneToImageN(m_xs);
		m_imgLocations = m_xs;
		K.ImageCoordinateCornerToCenterN(m_xs);
	}

	inline void SetArsacData(const ushort &width, const ushort &height)
	{
		SetImageSize(width, height);
		m_imgLocations = m_xs;

		const Point2D cx((width - 1) * 0.5f, (height - 1) * 0.5f);
		const ENFT_SSE::__m128 cx_cy_cx_cy = _mm_setr_ps(cx.x(), cx.y(), cx.x(), cx.y());
		const uint N = uint(m_xs.Size()), _N = N - (N & 1);
		ENFT_SSE::__m128 *p2 = (ENFT_SSE::__m128 *) m_xs.Data();
		for(uint i = 0; i < _N; i += 2, ++p2)
			*p2 = ENFT_SSE::_mm_sub_ps(*p2, cx_cy_cx_cy);
		if(_N != N)
			m_xs[_N] -= cx;
	}

	inline void GetSubset(const std::vector<ushort> &idxs, TranslationEstimatorData &subset) const
	{
		MatchSet3DTo2DX::GetSubset(idxs, subset);
		subset.m_fxy = m_fxy;
	}

	inline void SetFocal(const float &fxy) { m_fxy = fxy; }

	virtual void NormalizeData(const float &dataNormalizeMedian, Translation &T)
	{
		if(dataNormalizeMedian == 0)
		{
			m_sX = _mm_set1_ps(1.0f);
			m_cX.SetZero();
			return;
		}

		// Translate scene
		const ushort N = Size();
		m_cX.SetZero();
		for(ushort i = 0; i < N; ++i)
			m_cX += m_Xs[i];
		m_cX *= _mm_set1_ps(1.0f / N);
		m_cX.reserve() = 0;
		for(ushort i = 0; i < N; ++i)
			m_Xs[i] -= m_cX;
		T += m_cX;

		// Scale scene
		m_ds.resize(N);
		for(ushort i = 0; i < N; ++i)
			m_ds[i] = m_Xs[i].Z() + T.v2();
		const ushort ith = ushort(m_ds.size() >> 1);
		std::nth_element(m_ds.begin(), m_ds.begin() + ith, m_ds.end());
		const float dMed = m_ds[ith];
		m_sX = _mm_set1_ps((1 / dMed) / dataNormalizeMedian);
		T *= m_sX;
		for(ushort i = 0; i < N; ++i)
		{
			m_Xs[i] *= m_sX;
			m_Xs[i].reserve() = 1.0f;
		}
	}
	virtual void DenormalizeData(Translation &T)
	{
		// De-scale scene
		if(m_sX.m128_f32[0] != 1.0f)
		{
			m_sX = _mm_set1_ps(1 / m_sX.m128_f32[0]);
			T *= m_sX;
		}

		// De-translate scene
		if(m_cX.SquaredLength() != 0)
			T -= m_cX;
	}
	virtual double ComputeSSE(const Translation &T)
	{
		ENFT_SSE::__m128 e2, work[2];
		ENFT_SSE::__m128 SSE = ENFT_SSE::_mm_setzero_ps();
		const ushort N = Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) m_xs.Data();
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
		{
			T.ComputeProjectionError2(m_Xs[ix2], m_Xs[ix2p1], *px2, e2, work);
			SSE = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e2, e2), SSE);
		}
		if(_N != N)
		{
			Point2D e;
			SSE.m128_f32[0] += T.ComputeProjectionSquaredError(m_Xs[_N], m_xs[_N], e);
		}
		return ENFT_SSE::SSE::Sum0123(SSE);
	}

	virtual double GetFactorSSEToMSE() { return m_fxy / Size(); }
	virtual void ConstructNormalEquation(const Translation &T, LA::AlignedMatrix3f &A, LA::AlignedVector3f &b, LA::AlignedVector3f &s)
	{
		float ZcI;
		Point2D x, &e = x;
		LA::AlignedMatrix2x3f J;

		A.SetZero();
		b.SetZero();
		s.SetZero();
		const ushort N = Size();
		for(ushort i = 0; i < N; ++i)
		{
			T.ProjectToNormalizedPlane(m_Xs[i], ZcI, x);
			J.M00() = ZcI;		J.M01() = 0;	J.M02() = -x.x() * ZcI;
			J.M10() = 0;		J.M11() = ZcI;	J.M12() = -x.y() * ZcI;
			LA::AmB(m_xs[i], x, e);
			LA::AddATAToUpper(J, A);
			LA::AddATBTo(J, e, b);
			LA::AddAij2To(J, s);
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x3f>(A);
		A.SetLowerFromUpper();
		//printf("\n");
		//A.Print();
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x3f, LA::Vector2f>(b);
		LA::FinishAdditionAij2To<LA::AlignedMatrix2x3f>(s);
		LA::SetReserve<2>(b);
		LA::SetReserve<1>(s);
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ssTA(s, A);
		//printf("\n");
		//A.Print();
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const Translation &T, const LA::AlignedVector3f &s, LA::AlignedMatrix3f &A, LA::AlignedVector3f &b)
	{
		float ZcI;
		Point2D x, &e = x;
		LA::AlignedMatrix2x3f J;

		A.SetZero();
		b.SetZero();
		const ushort N = Size();
		for(ushort i = 0; i < N; ++i)
		{
			T.ProjectToNormalizedPlane(m_Xs[i], ZcI, x);
			J.M00() = ZcI;		J.M01() = 0;	J.M02() = -x.x() * ZcI;
			J.M10() = 0;		J.M11() = ZcI;	J.M12() = -x.y() * ZcI;
			LA::AmB(m_xs[i], x, e);
			LA::AddATAToUpper(J, A);
			LA::AddATBTo(J, e, b);
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x3f>(A);
		A.SetLowerFromUpper();
		//printf("\n");
		//A.Print();
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x3f, LA::Vector2f>(b);
		LA::SetReserve<2>(b);
		LA::ssTA(s, A);
		//printf("\n");
		//A.Print();
		LA::sA(s, b);
	}
	virtual void UpdateModel(const LA::AlignedVector3f &s, const LA::AlignedVector3f &x, const Translation &Told, Translation &Tnew)
	{
		Tnew.tXYZ() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s.v012x(), x.v012x()), Told.tXYZ());
	}

protected:

	Point3D m_cX;
	ENFT_SSE::__m128 m_sX;
	float m_fxy;
	std::vector<float> m_ds;

};

#endif