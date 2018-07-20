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
                                                                                                                                                                                                                                                                                                                                                                                                                                                              
#ifndef _CAMERA_ESTIMATOR_DEPTH_DATA_H_
#define _CAMERA_ESTIMATOR_DEPTH_DATA_H_

#include "CameraEstimatorData.h"

class CameraEstimatorDataDepth : public CameraEstimatorData
{

public:

	inline void SetDepthWeight(const float &depthWeight) { m_depthWeight = depthWeight; }
	inline void Resize(const ushort &N) { MatchSet3DTo2DX::Resize(N); m_ds.Resize(N); }
	inline const AlignedVector<float>& ds() const { return m_ds; }
	inline		 AlignedVector<float>& ds()		  { return m_ds; }
	inline const float& d(const ushort &i) const { return m_ds[i]; }
	inline		 float& d(const ushort &i)		 { return m_ds[i]; }
	inline const MatchSet3DX& GetMatches3D() const { return m_matches3D; }
	inline const ushort& GetIndex3D(const ushort &i2D) const { return m_map2DTo3D[i2D]; }
	inline void SetMatch(const ushort &i, const Point3D &X, const Point2D &x, const float &d)
	{
		MatchSet3DTo2DX::Set(i, X, x);
		m_ds[i] = d;
	}
	inline void FinishSettingMatches()
	{
		ushort i, j;
		const ushort N = Size();
		m_matches3D.Resize(N);
		m_map2DTo3D.assign(N, USHRT_MAX);
		for(i = j = 0; i < N; ++i)
		{
			if(m_ds[i] == 0.0f)
				continue;
			m_matches3D.X1(j) = m_Xs[i];
			m_matches3D.X2(j).Set(m_xs[i].x() * m_ds[i], m_xs[i].y() * m_ds[i], m_ds[i]);
			m_map2DTo3D[i] = j;
			++j;
		}
		m_matches3D.Resize(j);
	}
	inline void GetSubset(const std::vector<ushort> &idxs, MatchSet3DX &subset) const
	{
		ushort i, j, i3D;
		const ushort N = ushort(idxs.size());
		subset.Resize(N);
		for(i = j = 0; i < N; ++i)
		{
			if((i3D = m_map2DTo3D[idxs[i]]) == USHRT_MAX)
				continue;
			subset.X1(j) = m_matches3D.X1(i3D);
			subset.X2(j) = m_matches3D.X2(i3D);
			++j;
		}
		subset.Resize(j);
	}
	inline void GetSubset(const std::vector<ushort> &idxs, CameraEstimatorDataDepth &subset) const
	{
		CameraEstimatorData::GetSubset(idxs, subset);
		const ushort N = ushort(idxs.size());
		subset.m_ds.Resize(N);
		for(ushort i = 0; i < N; ++i)
			subset.m_ds[i] = m_ds[idxs[i]];
		subset.m_matches3D.Resize(0);
		subset.m_map2DTo3D.assign(subset.Size(), USHRT_MAX);
		subset.m_depthWeight = m_depthWeight;
	}

	virtual void NormalizeData(const float &dataNormalizeMedian, Camera &C)
	{
		CameraEstimatorData::NormalizeData(dataNormalizeMedian, C);
		const float s = m_sX.m128_f32[0];
		if(s == 1.0f)
			return;
		const uint N = m_ds.Size();
		for(uint i = 0; i < N; ++i)
		{
			if(m_ds[i] != 0.0f)
				m_ds[i] *= s;
		}
	}
	virtual double ComputeSSE(const Camera &C)
	{
		ENFT_SSE::__m128 &e1 = m_work[0], &SSE1 = m_work[1], &work = m_work[2];
		float e2, SSE2;
		LA::Vector2f d;
		SSE1 = ENFT_SSE::_mm_setzero_ps();
		SSE2 = 0.0f;
		const ENFT_SSE::__m128 *px = (const ENFT_SSE::__m128 *) m_xs.Data();
		const ushort N = Size(), _N = N - (N & 1);
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px)
		{
			C.ComputeProjectionError2(m_Xs[ix2], m_Xs[ix2p1], *px, d, e1, work);
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
			C.ComputeProjectionError(m_Xs[_N], m_xs[_N], d, e1);
			SSE1.m128_f32[0] += e1.SquaredLength();
			if(m_ds[_N] != 0.0f)
			{
				e2 = m_ds[_N] - d;
				SSE2 += e2 * e2;
			}
		}
		return double(ENFT_SSE::SSE::Sum0123(SSE1) + m_depthWeight * SSE2);
	}

	virtual void ConstructNormalEquation(const Camera &C, LA::AlignedCompactMatrix6f &A, LA::AlignedVector6f &b, LA::AlignedVector6f &s)
	{
		Point3D RX;
		float ZcI;
		Point2D e1;
		Point3D e2, x;
		LA::AlignedCompactMatrix2x6f J1;
		LA::AlignedCompactMatrix3x6f J2;

		A.SetZero();
		b.SetZero();
		s.SetZero();
		J2.SetReserveZero();
		const ushort N = Size();
		for(ushort i = 0; i < N; ++i)
		{
			C.Apply(m_Xs[i], RX, x);
			ZcI = 1 / x.Z();
			x.X() *= ZcI;
			x.Y() *= ZcI;
			if(m_ds[i] == 0.0f)
			{
				J1.M03() = ZcI;										J1.M04() = 0;										J1.M05() = -x.X() * ZcI;
				J1.M13() = 0;										J1.M14() = ZcI;										J1.M15() = -x.Y() * ZcI;
				J1.M00() = J1.M05() * RX.Y();						J1.M01() = J1.M03() * RX.Z() - J1.M05() * RX.X();	J1.M02() = -J1.M03() * RX.Y();
				J1.M10() = -J1.M14() * RX.Z() + J1.M15() * RX.Y();	J1.M11() = -J1.M15() * RX.X();						J1.M12() = J1.M14() * RX.X();
				e1.x() = m_xs[i].x() - x.X();
				e1.y() = m_xs[i].y() - x.Y();
				LA::AddATAToUpper(J1, A);
				LA::AddATBTo(J1, e1, b, m_work);
				LA::AddAij2To(J1, s);
			}
			else
			{
				J2.M03() = ZcI;										J2.M04() = 0;										J2.M05() = -x.X() * ZcI;
				J2.M13() = 0;										J2.M14() = ZcI;										J2.M15() = -x.Y() * ZcI;
				J2.M23() = 0;										J2.M24() = 0;										J2.M25() = m_depthWeight;
				J2.M00() = J2.M05() * RX.Y();						J2.M01() = J2.M03() * RX.Z() - J2.M05() * RX.X();	J2.M02() = -J2.M03()*RX.Y();
				J2.M10() = -J2.M14() * RX.Z() + J2.M15() * RX.Y();	J2.M11() = -J2.M15() * RX.X();						J2.M12() = J2.M14()*RX.X();
				J2.M20() = m_depthWeight * RX.Y();					J2.M21() = -m_depthWeight * RX.X();					J2.M22() = 0;
				e2.X() = m_xs[i].x() - x.X();
				e2.Y() = m_xs[i].y() - x.Y();
				e2.Z() = m_depthWeight * (m_ds[i] - x.Z());
				LA::AddATAToUpper(J2, A);
				LA::AddATBTo(J2, e2, b, m_work);
				LA::AddAij2To(J2, s);
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedCompactMatrix2x6f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedCompactMatrix2x6f, LA::Vector2f>(b);
		LA::FinishAdditionAij2To<LA::AlignedCompactMatrix2x6f>(s);
		LA::SetReserve<2>(b);
		LA::SetReserve<1>(s);
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ssTA(s, A, m_work);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const Camera &C, const LA::AlignedVector6f &s, LA::AlignedCompactMatrix6f &A, LA::AlignedVector6f &b)
	{
		Point3D RX;
		float ZcI;
		Point2D e1;
		Point3D e2, x;
		LA::AlignedCompactMatrix2x6f J1;
		LA::AlignedCompactMatrix3x6f J2;

		A.SetZero();
		b.SetZero();
		J2.SetReserveZero();
		const ushort N = Size();
		for(ushort i = 0; i < N; ++i)
		{
			C.Apply(m_Xs[i], RX, x);
			ZcI = 1 / x.Z();
			x.X() *= ZcI;
			x.Y() *= ZcI;
			if(m_ds[i] == 0.0f)
			{
				J1.M03() = ZcI;										J1.M04() = 0;										J1.M05() = -x.X() * ZcI;
				J1.M13() = 0;										J1.M14() = ZcI;										J1.M15() = -x.Y() * ZcI;
				J1.M00() = J1.M05() * RX.Y();						J1.M01() = J1.M03() * RX.Z() - J1.M05() * RX.X();	J1.M02() = -J1.M03() * RX.Y();
				J1.M10() = -J1.M14() * RX.Z() + J1.M15() * RX.Y();	J1.M11() = -J1.M15() * RX.X();						J1.M12() = J1.M14() * RX.X();
				e1.x() = m_xs[i].x() - x.X();
				e1.y() = m_xs[i].y() - x.Y();
				LA::AddATAToUpper(J1, A);
				LA::AddATBTo(J1, e1, b, m_work);
			}
			else
			{
				J2.M03() = ZcI;										J2.M04() = 0;										J2.M05() = -x.X() * ZcI;
				J2.M13() = 0;										J2.M14() = ZcI;										J2.M15() = -x.Y() * ZcI;
				J2.M23() = 0;										J2.M24() = 0;										J2.M25() = m_depthWeight;
				J2.M00() = J2.M05() * RX.Y();						J2.M01() = J2.M03() * RX.Z() - J2.M05() * RX.X();	J2.M02() = -J2.M03()*RX.Y();
				J2.M10() = -J2.M14() * RX.Z() + J2.M15() * RX.Y();	J2.M11() = -J2.M15() * RX.X();						J2.M12() = J2.M14()*RX.X();
				J2.M20() = m_depthWeight * RX.Y();					J2.M21() = -m_depthWeight * RX.X();					J2.M22() = 0;
				e2.X() = m_xs[i].x() - x.X();
				e2.Y() = m_xs[i].y() - x.Y();
				e2.Z() = m_depthWeight * (m_ds[i] - x.Z());
				LA::AddATAToUpper(J2, A);
				LA::AddATBTo(J2, e2, b, m_work);
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedCompactMatrix2x6f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedCompactMatrix2x6f, LA::Vector2f>(b);
		LA::SetReserve<2>(b);
		LA::ssTA(s, A, m_work);
		LA::sA(s, b);
	}

protected:

	float m_depthWeight;
	AlignedVector<float> m_ds;
	MatchSet3DX m_matches3D;
	std::vector<ushort> m_map2DTo3D;

};

#endif