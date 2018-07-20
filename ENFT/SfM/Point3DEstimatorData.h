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

#ifndef _POINT_3D_ESTIMATOR_DATA_H_
#define _POINT_3D_ESTIMATOR_DATA_H_

#include "Camera.h"
#include "Optimization/OptimizerData.h"

class Point3DEstimatorData : public OptimizerDataTemplate<Point3D, LA::AlignedVector3f, LA::AlignedMatrix3f>
{

public:

	inline Point3DEstimatorData() {}
	inline Point3DEstimatorData(const ushort &N) { Resize(N); }
	inline void Resize(const ushort &N) { m_Cs.resize(N); m_xs.Resize(N); m_ws.Resize(0); m_wsSq.Resize(0); }
	inline const ushort Size() const { return ushort(m_Cs.size()); }
	inline const Camera& GetCamera(const ushort &i) const { return *m_Cs[i]; }
	inline const AlignedVector<Point2D>& xs() const { return m_xs; }
	inline		 AlignedVector<Point2D>& xs()		{ return m_xs; }
	inline const Point2D& x(const ushort &i) const { return m_xs[i]; }
	inline		 Point2D& x(const ushort &i)	   { return m_xs[i]; }
	inline void Set(const ushort &i, const Camera &C, const Point2D &x) { m_Cs[i] = &C; m_xs[i] = x; }
	inline void SetCamera(const ushort &i, const Camera &C) { m_Cs[i] = &C; }
	inline void ComputeRayDirections(AlignedVector<Point3D> &rayDirs) const
	{
		Point3D center;
		const ushort N = Size();
		rayDirs.Resize(N);
		for(ushort i = 0; i < N; ++i)
			GetCamera(i).ComputeRayDirection(m_xs[i], rayDirs[i]);
	}
	inline float ComputeMinimalRayAngleDot(AlignedVector<Point3D> &rayDirs) const
	{
		ComputeRayDirections(rayDirs);
		const ushort N = ushort(rayDirs.Size());
		float dot, dotMin = FLT_MAX;
		for(ushort i1 = 0; i1 < N; ++i1)
		for(ushort i2 = i1 + 1; i2 < N; ++i2)
		{
			if((dot = rayDirs[i1].Dot(rayDirs[i2])) < dotMin)
				dotMin = dot;
		}
		return dotMin;
	}
	inline bool AreWeightsValid() const { return m_ws.Size() == m_Cs.size(); }
	inline void ValidateWeights() { m_ws.Resize(uint(m_Cs.size())); m_wsSq.Resize(m_ws.Size()); }
	inline const float& GetSquaredWeight(const ushort &i) const { return m_wsSq[i]; }
	inline const AlignedVector<float>& GetSquaredWeights() const { return m_wsSq; }
	inline const AlignedVector<float>& GetWeights() const { return m_ws; }
	inline void SetWeight(const ushort &i, const float &w) { m_ws[i] = w; m_wsSq[i] = w * w; }
	inline void SetSquaredWeight(const ushort &i, const float &wSq) { m_wsSq[i] = wSq; m_ws[i] = sqrt(wSq); }
	inline void ScaleWeights(const float &scale)
	{
		const uint N = uint(Size());
		if(AreWeightsValid())
		{
			const uint _N = N - (N & 3);
			const ENFT_SSE::__m128 s = ENFT_SSE::_mm_set1_ps(scale);
			ENFT_SSE::__m128 *w = (ENFT_SSE::__m128 *) m_ws.Data(), *wSq = (ENFT_SSE::__m128 *) m_wsSq.Data();
			for(uint i = 0; i < _N; i += 4, ++w, ++wSq)
			{
				*w = ENFT_SSE::_mm_mul_ps(*w, s);
				*wSq = ENFT_SSE::_mm_mul_ps(*w, *w);
			}
			for(uint i = _N; i < N; ++i)
			{
				m_ws[i] *= scale;
				m_wsSq[i] = m_ws[i] * m_ws[i];
			}
		}
		else
		{
			m_ws.Resize(N);
			m_wsSq.Resize(N);
			const float scaleSq = scale * scale;
			for(uint i = 0; i < N; ++i)
			{
				m_ws[i] = scale;
				m_wsSq[i] = scaleSq;
			}
		}
	}
	inline void GetSubset(const std::vector<ushort> &idxs, Point3DEstimatorData &subset) const
	{
		const ushort N = ushort(idxs.size());
		if(AreWeightsValid())
		{
			subset.m_ws.Resize(N);
			subset.m_wsSq.Resize(N);
			for(ushort i = 0; i < N; ++i)
			{
				subset.m_ws[i] = m_ws[idxs[i]];
				subset.m_wsSq[i] = m_wsSq[idxs[i]];
			}
		}
		subset.Resize(N);
		for(ushort i = 0; i < N; ++i)
		{
			subset.m_Cs[i] = m_Cs[idxs[i]];
			subset.m_xs[i] = m_xs[idxs[i]];
		}
		subset.m_fxy = m_fxy;
	}
	inline void SetSubset(const std::vector<ushort> &idxs)
	{
		const ushort N = ushort(idxs.size());
		if(AreWeightsValid())
		{
			for(ushort i = 0; i < N; ++i)
			{
				m_ws[i] = m_ws[idxs[i]];
				m_wsSq[i] = m_wsSq[idxs[i]];
			}
			m_ws.Resize(N);
			m_wsSq.Resize(N);
		}
		for(ushort i = 0; i < N; ++i)
		{
			m_Cs[i] = m_Cs[idxs[i]];
			m_xs[i] = m_xs[idxs[i]];
		}
		m_Cs.resize(N);
		m_xs.Resize(N);
	}
	inline void PushBack(const Point3DEstimatorData &data)
	{
		if(AreWeightsValid() && data.AreWeightsValid())
		{
			m_ws.PushBack(data.m_ws.Data(), data.m_ws.Size());
			m_wsSq.PushBack(data.m_wsSq.Data(), data.m_wsSq.Size());
		}
		m_Cs.insert(m_Cs.end(), data.m_Cs.begin(), data.m_Cs.end());
		m_xs.EnlargeCapacity(m_xs.Size() + data.m_xs.Size());
		m_xs.PushBack(data.m_xs.Data(), data.m_xs.Size());
	}
	inline const float& GetFocal() const { return m_fxy; }
	inline void SetFocal(const float &fxy) { m_fxy = fxy; }

	virtual void NormalizeData(const float &dataNormalizeMedian, Point3D &X) {}
	virtual void DenormalizeData(Point3D &X) {}
	virtual double GetFactorSSEToMSE() { return m_fxy / Size(); }

	virtual double ComputeSSE(const Point3D &X)
	{
		ENFT_SSE::__m128 SSE = ENFT_SSE::_mm_setzero_ps();
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) m_xs.Data();
		ENFT_SSE::__m128 e2, work;
		const ushort N = ushort(m_Cs.size()), _N = N - (N & 1);
		if(AreWeightsValid())
		{
			for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
			{
				Camera::ComputeProjectionError2(GetCamera(ix2), GetCamera(ix2p1), X, X, *px2, e2, work);
				e2 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(m_ws[ix2], m_ws[ix2], m_ws[ix2p1], m_ws[ix2p1]), e2);
				SSE = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e2, e2), SSE);
			}
			if(_N != N)
			{
				Point2D e;
				GetCamera(_N).ComputeProjectionError(X, m_xs[_N], e);
				e *= m_ws[_N];
				SSE.m128_f32[0] += e.SquaredLength();
			}
		}
		else
		{
			for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
			{
				Camera::ComputeProjectionError2(GetCamera(ix2), GetCamera(ix2p1), X, X, *px2, e2, work);
				SSE = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e2, e2), SSE);
			}
			if(_N != N)
			{
				Point2D e;
				GetCamera(_N).ComputeProjectionError(X, m_xs[_N], e);
				SSE.m128_f32[0] += e.SquaredLength();
			}
		}
		return ENFT_SSE::SSE::Sum0123(SSE);
	}
	virtual void ConstructNormalEquation(const Point3D &X, LA::AlignedMatrix3f &A, LA::AlignedVector3f &b, LA::AlignedVector3f &s)
	{
		Point2D x, &e = x;
		LA::AlignedMatrix2x3f J;

		A.SetZero();
		b.SetZero();
		s.SetZero();
		const ushort N = ushort(m_Cs.size());
		if(AreWeightsValid())
		{
			ENFT_SSE::__m128 wZcI;
			for(ushort i = 0; i < N; ++i)
			{
				const Camera &C = GetCamera(i);
				C.ProjectToNormalizedPlane(X, wZcI.m128_f32[0], x.x(), x.y());
				wZcI = ENFT_SSE::_mm_set1_ps(m_ws[i] * wZcI.m128_f32[0]);
				J.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.x()), C.r_20_21_22_x())));
				J.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.y()), C.r_20_21_22_x())));
				LA::AmB(m_xs[i], x, e);
				e *= m_ws[i];
				LA::AddATAToUpper(J, A);
				LA::AddATBTo(J, e, b);
				LA::AddAij2To(J, s);
			}
		}
		else
		{
			ENFT_SSE::__m128 ZcI;
			for(ushort i = 0; i < N; ++i)
			{
				const Camera &C = GetCamera(i);
				C.ProjectToNormalizedPlane(X, ZcI.m128_f32[0], x.x(), x.y());
				ZcI = ENFT_SSE::_mm_set1_ps(ZcI.m128_f32[0]);
				J.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.x()), C.r_20_21_22_x())));
				J.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.y()), C.r_20_21_22_x())));
				LA::AmB(m_xs[i], x, e);
				LA::AddATAToUpper(J, A);
				LA::AddATBTo(J, e, b);
				LA::AddAij2To(J, s);
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x3f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x3f, LA::Vector2f>(b);
		LA::FinishAdditionAij2To<LA::AlignedMatrix2x3f>(s);
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		//s.v012x() = ENFT_SSE::_mm_set1_ps(1.0f);
		LA::ssTA(s, A);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const Point3D &X, const LA::AlignedVector3f &s, LA::AlignedMatrix3f &A, LA::AlignedVector3f &b)
	{
		Point2D x, &e = x;
		LA::AlignedMatrix2x3f J;

		A.SetZero();
		b.SetZero();
		const ushort N = ushort(m_Cs.size());
		if(AreWeightsValid())
		{
			ENFT_SSE::__m128 wZcI;
			for(ushort i = 0; i < N; ++i)
			{
				const Camera &C = GetCamera(i);
				C.ProjectToNormalizedPlane(X, wZcI.m128_f32[0], x.x(), x.y());
				wZcI = ENFT_SSE::_mm_set1_ps(m_ws[i] * wZcI.m128_f32[0]);
				J.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.x()), C.r_20_21_22_x())));
				J.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(wZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.y()), C.r_20_21_22_x())));
				LA::AmB(m_xs[i], x, e);
				e *= m_ws[i];
				LA::AddATAToUpper(J, A);
				LA::AddATBTo(J, e, b);
			}
		}
		else
		{
			ENFT_SSE::__m128 ZcI;
			for(ushort i = 0; i < N; ++i)
			{
				const Camera &C = GetCamera(i);
				C.ProjectToNormalizedPlane(X, ZcI.m128_f32[0], x.x(), x.y());
				ZcI = ENFT_SSE::_mm_set1_ps(ZcI.m128_f32[0]);
				J.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.x()), C.r_20_21_22_x())));
				J.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.y()), C.r_20_21_22_x())));
				LA::AmB(m_xs[i], x, e);
				LA::AddATAToUpper(J, A);
				LA::AddATBTo(J, e, b);
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x3f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x3f, LA::Vector2f>(b);
		LA::ssTA(s, A);
		LA::sA(s, b);
	}
	virtual void UpdateModel(const LA::AlignedVector3f &s, const LA::AlignedVector3f &x, const Point3D &Xold, Point3D &Xnew)
	{
		Xnew.XYZx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s.v012x(), x.v012x()), Xold.XYZx());
		Xnew.reserve() = 1.0f;
	}

protected:

	std::vector<const Camera *> m_Cs;
	AlignedVector<Point2D> m_xs;
	AlignedVector<float> m_ws, m_wsSq;
	float m_fxy;

};

#endif