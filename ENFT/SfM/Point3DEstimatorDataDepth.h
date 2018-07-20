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

#ifndef _POINT_3D_ESTIMATOR_DATA_DEPTH_H_
#define _POINT_3D_ESTIMATOR_DATA_DEPTH_H_

#include "Point3DEstimatorData.h"

class Point3DEstimatorDataDepth : public Point3DEstimatorData
{

public:

	inline const AlignedVector<float>& ds() const { return m_ds; }
	inline		 AlignedVector<float>& ds()		  { return m_ds; }
	inline const AlignedVector<Point3D>& Xs() const { return m_Xs; }
	inline		 AlignedVector<Point3D>& Xs()		{ return m_Xs; }
	inline const float& d(const ushort &i) const { return m_ds[i]; }
	inline		 float& d(const ushort &i)		 { return m_ds[i]; }
	inline void SetDepthWeight(const float &depthWeight) { m_depthWeight = depthWeight; }

	inline void Resize(const ushort &N)
	{
		Point3DEstimatorData::Resize(N);
		m_ds.Resize(N);
	}
	inline void SetMatch(const ushort &i, const Camera &C, const Point2D &x, const float &d)
	{
		Point3DEstimatorData::Set(i, C, x);
		m_ds[i] = d;
	}
	inline void FinishSettingMatches()
	{
		uint i, j;
		const uint N = m_ds.Size();
		m_Xs.Resize(N);
		for(i = j = 0; i < N; ++i)
		{
			if(m_ds[i] == 0.0f)
				continue;
			m_Xs[j].Set(m_xs[i].x() * m_ds[i], m_xs[i].y() * m_ds[i], m_ds[i]);
			m_Cs[i]->ApplyInversely(m_Xs[j]);
			++j;
		}
		m_Xs.Resize(j);
	}
	inline void GetSubset(const std::vector<ushort> &idxs, Point3DEstimatorDataDepth &subset) const
	{
		Point3DEstimatorData::GetSubset(idxs, subset);
		const ushort N = ushort(idxs.size());
		subset.m_ds.Resize(N);
		for(ushort i = 0; i < N; ++i)
			subset.m_ds[i] = m_ds[idxs[i]];
		subset.m_Xs.Resize(0);
		subset.m_depthWeight = m_depthWeight;
	}

	virtual double ComputeSSE(const Point3D &X)
	{
		ENFT_SSE::__m128 e1, work, SSE1 = ENFT_SSE::_mm_setzero_ps();
		float e2, SSE2 = 0.0f;
		LA::Vector2f d;
		const ENFT_SSE::__m128 *px = (const ENFT_SSE::__m128 *) m_xs.Data();
		const ushort N = ushort(m_Cs.size()), _N = N - (N & 1);
		if(AreWeightsValid())
		{
			for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px)
			{
				Camera::ComputeProjectionError2(GetCamera(ix2), GetCamera(ix2p1), X, X, *px, d, e1, work);
				e1 = ENFT_SSE::_mm_mul_ps(_mm_setr_ps(m_ws[ix2], m_ws[ix2], m_ws[ix2p1], m_ws[ix2p1]), e1);
				SSE1 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e1, e1), SSE1);
				if(m_ds[ix2] != 0.0f)
				{
					e2 = m_ds[ix2] - d.v0();
					SSE2 += e2 * e2;
				}
				if(m_ds[ix2p1] != 0.0f)
				{
					e2 = m_ds[ix2p1] - d.v1();
					SSE2 += e2 * e2;
				}
			}
			if(_N != N)
			{
				float d;
				Point2D e1;
				GetCamera(_N).ComputeProjectionError(X, m_xs[_N], d, e1);
				e1 *= m_ws[_N];
				SSE1.m128_f32[0] += e1.SquaredLength();
				if(m_ds[_N] != 0.0f)
				{
					e2 = m_ds[_N] - d;
					SSE2 += e2 * e2;
				}
			}
		}
		else
		{
			for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px)
			{
				Camera::ComputeProjectionError2(GetCamera(ix2), GetCamera(ix2p1), X, X, *px, d, e1, work);
				SSE1 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e1, e1), SSE1);
				if(m_ds[ix2] != 0.0f)
				{
					e2 = m_ds[ix2] - d.v0();
					SSE2 += e2 * e2;
				}
				if(m_ds[ix2p1] != 0.0f)
				{
					e2 = m_ds[ix2p1] - d.v1();
					SSE2 += e2 * e2;
				}
			}
			if(_N != N)
			{
				float d;
				Point2D e1;
				GetCamera(_N).ComputeProjectionError(X, m_xs[_N], d, e1);
				SSE1.m128_f32[0] += e1.SquaredLength();
				if(m_ds[_N] != 0.0f)
				{
					e2 = m_ds[_N] - d;
					SSE2 += e2 * e2;
				}
			}
		}
		return double(ENFT_SSE::SSE::Sum0123(SSE1) + m_depthWeight * SSE2);
	}
	virtual void ConstructNormalEquation(const Point3D &X, LA::AlignedMatrix3f &A, LA::AlignedVector3f &b, LA::AlignedVector3f &s)
	{
		Point2D e1;
		Point3D e2, x;
		LA::AlignedMatrix2x3f J1;
		LA::AlignedMatrix3f J2;

		A.SetZero();
		b.SetZero();
		s.SetZero();
		const ushort N = ushort(m_Cs.size());
		const ENFT_SSE::__m128 w = _mm_set1_ps(m_depthWeight);
		if(AreWeightsValid())
		{
			ENFT_SSE::__m128 wZcI;
			for(ushort i = 0; i < N; ++i)
			{
				const Camera &C = GetCamera(i);
				C.Apply(X, x);
				wZcI.m128_f32[0] = 1 / x.Z();
				x.X() *= wZcI.m128_f32[0];
				x.Y() *= wZcI.m128_f32[0];
				wZcI = _mm_set1_ps(m_ws[i] * wZcI.m128_f32[0]);
				if(m_ds[i] == 0.0f)
				{
					J1.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.X()), C.r_20_21_22_x())));
					J1.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.Y()), C.r_20_21_22_x())));
					e1.x() = m_ws[i] * (m_xs[i].x() - x.X());
					e1.y() = m_ws[i] * (m_xs[i].y() - x.Y());
					LA::AddATAToUpper(J1, A);
					LA::AddATBTo(J1, e1, b);
					LA::AddAij2To(J1, s);
				}
				else
				{
					J2.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.X()), C.r_20_21_22_x())));
					J2.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.Y()), C.r_20_21_22_x())));
					J2.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(w, C.r_20_21_22_x());
					e2.X() = m_ws[i] * (m_xs[i].x() - x.X());
					e2.Y() = m_ws[i] * (m_xs[i].y() - x.Y());
					e2.Z() = m_depthWeight * (m_ds[i] - x.Z());
					LA::AddATAToUpper(J2, A);
					LA::AddATBTo(J2, e2, b);
					LA::AddAij2To(J2, s);
				}
			}
		}
		else
		{
			ENFT_SSE::__m128 ZcI;
			for(ushort i = 0; i < N; ++i)
			{
				const Camera &C = GetCamera(i);
				C.Apply(X, x);
				ZcI.m128_f32[0] = 1 / x.Z();
				x.X() *= ZcI.m128_f32[0];
				x.Y() *= ZcI.m128_f32[0];
				ZcI = _mm_set1_ps(ZcI.m128_f32[0]);
				if(m_ds[i] == 0.0f)
				{
					J1.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.X()), C.r_20_21_22_x())));
					J1.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.Y()), C.r_20_21_22_x())));
					e1.x() = m_xs[i].x() - x.X();
					e1.y() = m_xs[i].y() - x.Y();
					LA::AddATAToUpper(J1, A);
					LA::AddATBTo(J1, e1, b);
					LA::AddAij2To(J1, s);
				}
				else
				{
					J2.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.X()), C.r_20_21_22_x())));
					J2.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.Y()), C.r_20_21_22_x())));
					J2.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(w, C.r_20_21_22_x());
					e2.X() = m_xs[i].x() - x.X();
					e2.Y() = m_xs[i].y() - x.Y();
					e2.Z() = m_depthWeight * (m_ds[i] - x.Z());
					LA::AddATAToUpper(J2, A);
					LA::AddATBTo(J2, e2, b);
					LA::AddAij2To(J2, s);
				}
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix3f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedMatrix3f, LA::Vector2f>(b);
		LA::FinishAdditionAij2To<LA::AlignedMatrix3f>(s);
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		//A.Print();
		//b.Print();
		//s.Print();
		//s.v012x() = _mm_set1_ps(1.0f);
		LA::ssTA(s, A);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const Point3D &X, const LA::AlignedVector3f &s, LA::AlignedMatrix3f &A, LA::AlignedVector3f &b)
	{
		Point2D e1;
		Point3D x, e2;
		LA::AlignedMatrix2x3f J1;
		LA::AlignedMatrix3f J2;

		A.SetZero();
		b.SetZero();
		const ushort N = ushort(m_Cs.size());
		const ENFT_SSE::__m128 w = _mm_set1_ps(m_depthWeight);
		if(AreWeightsValid())
		{
			ENFT_SSE::__m128 wZcI;
			for(ushort i = 0; i < N; ++i)
			{
				const Camera &C = GetCamera(i);
				C.Apply(X, x);
				wZcI.m128_f32[0] = 1 / x.Z();
				x.X() *= wZcI.m128_f32[0];
				x.Y() *= wZcI.m128_f32[0];
				wZcI = _mm_set1_ps(m_ws[i] * wZcI.m128_f32[0]);
				if(m_ds[i] == 0.0f)
				{
					J1.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.X()), C.r_20_21_22_x())));
					J1.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.Y()), C.r_20_21_22_x())));
					e1.x() = m_ws[i] * (m_xs[i].x() - x.X());
					e1.y() = m_ws[i] * (m_xs[i].y() - x.Y());
					LA::AddATAToUpper(J1, A);
					LA::AddATBTo(J1, e1, b);
				}
				else
				{
					J2.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.X()), C.r_20_21_22_x())));
					J2.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.Y()), C.r_20_21_22_x())));
					J2.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(w, C.r_20_21_22_x());
					e2.X() = m_ws[i] * (m_xs[i].x() - x.X());
					e2.Y() = m_ws[i] * (m_xs[i].y() - x.Y());
					e2.Z() = m_depthWeight * (m_ds[i] - x.Z());
					LA::AddATAToUpper(J2, A);
					LA::AddATBTo(J2, e2, b);
				}
			}
		}
		else
		{
			ENFT_SSE::__m128 ZcI;
			for(ushort i = 0; i < N; ++i)
			{
				const Camera &C = GetCamera(i);
				C.Apply(X, x);
				ZcI.m128_f32[0] = 1 / x.Z();
				x.X() *= ZcI.m128_f32[0];
				x.Y() *= ZcI.m128_f32[0];
				ZcI = _mm_set1_ps(ZcI.m128_f32[0]);
				if(m_ds[i] == 0.0f)
				{
					J1.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.X()), C.r_20_21_22_x())));
					J1.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.Y()), C.r_20_21_22_x())));
					e1.x() = m_xs[i].x() - x.X();
					e1.y() = m_xs[i].y() - x.Y();
					LA::AddATAToUpper(J1, A);
					LA::AddATBTo(J1, e1, b);
				}
				else
				{
					J2.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.X()), C.r_20_21_22_x())));
					J2.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(x.Y()), C.r_20_21_22_x())));
					J2.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(w, C.r_20_21_22_x());
					e2.X() = m_xs[i].x() - x.X();
					e2.Y() = m_xs[i].y() - x.Y();
					e2.Z() = m_depthWeight * (m_ds[i] - x.Z());
					LA::AddATAToUpper(J2, A);
					LA::AddATBTo(J2, e2, b);
				}
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix3f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedMatrix3f, LA::Vector2f>(b);
		LA::ssTA(s, A);
		LA::sA(s, b);
	}

protected:

	float m_depthWeight;
	AlignedVector<float> m_ds;
	AlignedVector<Point3D> m_Xs;

};

#endif