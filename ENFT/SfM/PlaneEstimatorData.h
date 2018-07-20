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

#ifndef _PLANE_ESTIMATOR_DATA_H_
#define _PLANE_ESTIMATOR_DATA_H_

#include "Utility/AlignedVector.h"
#include "SfM/Camera.h"
#include "Plane.h"
#include "Optimization/OptimizerData.h"

class PlaneEstimatorData3D : public OptimizerDataTemplate<Plane, LA::AlignedVector4f, LA::AlignedMatrix4f>
{

public:

	inline void Resize(const uint &N) { m_Xs.Resize(N); }
	inline uint Size() const { return m_Xs.Size(); }
	inline const Point3D& X(const uint &i) const { return m_Xs[i]; }
	inline		 Point3D& X(const uint &i)		 { return m_Xs[i]; }
	inline void GetSubset(const std::vector<uint> &idxs, PlaneEstimatorData3D &subset) const
	{
		const uint N = uint(idxs.size());
		subset.Resize(N);
		for(uint i = 0; i < N; ++i)
			subset.m_Xs[i] = m_Xs[idxs[i]];
	}

	virtual double ComputeSSE(const Plane &P)
	{
		double sse = 0;
		float d;
		const uint nPts = Size();
		for(uint iPt = 0; iPt < nPts; ++iPt)
		{
			d = P.ComputeDistance(m_Xs[iPt]);
			sse += d * d;
		}
		return sse;
	}
	virtual double GetFactorSSEToMSE() { return 1.0 / m_Xs.Size(); }

	virtual void ConstructNormalEquation(const Plane &P, LA::AlignedMatrix4f &A, LA::AlignedVector4f &b, LA::AlignedVector4f &s)
	{
		A.SetZero();
		b.SetZero();
		s.SetZero();
		float d;
		const uint nPts = Size();
		for(uint iPt = 0; iPt < nPts; ++iPt)
		{
			const Point3D &X = m_Xs[iPt];
			LA::AddAATToUpper(X, A);
			d = -P.ComputeDistance(X);
			LA::AddsATo(d, X, b);
			LA::AddAij2To(X, s);
		}
		A.SetLowerFromUpper();
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ssTA(s, A);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const Plane &P, const LA::AlignedVector4f &s, LA::AlignedMatrix4f &A, LA::AlignedVector4f &b)
	{
		A.SetZero();
		b.SetZero();
		float d;
		const uint nPts = Size();
		for(uint iPt = 0; iPt < nPts; ++iPt)
		{
			const Point3D &X = m_Xs[iPt];
			LA::AddAATToUpper(X, A);
			d = -P.ComputeDistance(X);
			LA::AddsATo(d, X, b);
		}
		A.SetLowerFromUpper();
		LA::ssTA(s, A);
		LA::sA(s, b);
	}
	virtual void UpdateModel(const LA::AlignedVector4f &s, const LA::AlignedVector4f &x, const Plane &Pold, Plane &Pnew)
	{
		Pnew.ABCD() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(s.v0123(), x.v0123()), Pold.ABCD());
		Pnew.Normalize();
	}

protected:

	AlignedVector<Point3D> m_Xs;

};

class PlaneEstimatorData2D : public PlaneEstimatorData3D
{

public:

	PlaneEstimatorData2D() : m_measureIn3D(false) {}

	inline const Camera* pC(const uint &i) const { return m_pCs[i]; }	inline const Camera* &pC(const uint &i) { return m_pCs[i]; }
	inline const Point2D& x(const uint &i) const { return m_xs[i]; }	inline Point2D& x(const uint &i) { return m_xs[i]; }
	inline const uint& GetPointMeasurementIndex(const uint &iPt) const { return m_mapPtToMea[iPt]; }
	inline uint GetPointMeasurementsNumber(const uint &iPt) const { return m_mapPtToMea[iPt + 1] - m_mapPtToMea[iPt]; }
	inline void SetPointMeasurementIndex(const uint &iPt, const uint &iMea) { m_mapPtToMea[iPt] = iMea; }
	inline const uint& GetPointsNumber() const { return m_Xs.Size(); }
	inline const uint& GetMeasurementsNumber() const { return m_mapPtToMea.back(); }
	inline uint GetPointLastMeasurementIndex(const uint &iPt) const { return m_mapPtToMea[iPt + 1] - 1; }
	inline void Resize(const uint &nPts, const uint &nMeas)
	{
		PlaneEstimatorData3D::Resize(nPts);
		m_mapPtToMea.resize(nPts + 1);
		m_mapPtToMea[nPts] = nMeas;
		m_pCs.resize(nMeas);
		m_xs.Resize(nMeas);
	}
	inline void Resize(const uint &nPts)
	{
		PlaneEstimatorData3D::Resize(nPts);
		m_mapPtToMea.assign(nPts, 0);
		m_pCs.resize(0);
		m_xs.Resize(0);
	}
	inline void PushBackMeasurements(const std::vector<const Camera *> &pCs, const AlignedVector<Point2D> &xs)
	{
		m_pCs.insert(m_pCs.end(), pCs.begin(), pCs.end());
		m_xs.PushBack(xs.Data(), xs.Size());
	}
	inline void GetSubset(const std::vector<uint> &idxs, PlaneEstimatorData2D &subset) const
	{
		subset.m_fxy = m_fxy;
		subset.m_measureIn3D = m_measureIn3D;

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
			subset.m_Xs[iPtDst] = m_Xs[iPtSrc];
			subset.m_mapPtToMea[iPtDst] = iMeaDst;
			const uint iMeaSrc1 = m_mapPtToMea[iPtSrc], iMeaSrc2 = m_mapPtToMea[iPtSrc + 1];
			for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc, ++iMeaDst)
			{
				subset.m_pCs[iMeaDst] = m_pCs[iMeaSrc];
				subset.m_xs[iMeaDst] = m_xs[iMeaSrc];
			}
		}
		subset.m_mapPtToMea[iPtDst] = iMeaDst;
#if _DEBUG
		assert(iMeaDst == nMeasDst);
#endif
	}

	static inline float ComputeSSE2D(const Point3D &Xp, const Camera* const *pCs, const Point2D *xs, const ushort N)
	{
		Point2D e;
		float sse = 0;
		for(ushort i = 0; i < N; ++i)
			sse += pCs[i]->ComputeProjectionSquaredError(Xp, xs[i], e);
		return sse;
	}
	static inline float ComputeSSE2D(const Point3D &X, const Plane &P, const Camera* const *pCs, const Point2D *xs, const ushort N, float &d, Point3D &Xp)
	{
		P.Project(X, d, Xp);
		return ComputeSSE2D(Xp, pCs, xs, N);
	}
	inline float ComputeSSE2D(const uint &iPt, const Point3D &Xp) const
	{
		const uint iMea1 = m_mapPtToMea[iPt], iMea2 = m_mapPtToMea[iPt + 1];
		return ComputeSSE2D(Xp, m_pCs.data() + iMea1, m_xs.Data() + iMea1, ushort(iMea2 - iMea1));
	}
	inline float ComputeSSE2D(const uint &iPt, const Plane &P, float &d, Point3D &Xp) const
	{
		P.Project(m_Xs[iPt], d, Xp);
		return ComputeSSE2D(iPt, Xp);
	}

	inline double ComputeSSE2D(const Plane &P)
	{
		double sse = 0;
		Point2D e;
		const uint nPts = Size();
		m_ds.resize(nPts);
		m_Xps.Resize(nPts);
		for(uint iPt = 0; iPt < nPts; ++iPt)
			sse += ComputeSSE2D(iPt, P, m_ds[iPt], m_Xps[iPt]);
		return sse;
	}

	inline void SetFocal(const float &fxy) { m_fxy = fxy; }
	virtual void NormalizeData(const float &dataNormalizeMedian, Plane &P) {}
	virtual void DenormalizeData(Plane &P) {}
	virtual double ComputeSSE(const Plane &P)
	{
		if(m_measureIn3D)
			return PlaneEstimatorData3D::ComputeSSE(P);
		else
			return ComputeSSE2D(P);
	}
	virtual double GetFactorSSEToMSE() { return m_measureIn3D ? PlaneEstimatorData3D::GetFactorSSEToMSE() : m_fxy / m_mapPtToMea.back(); }
	virtual void ConstructNormalEquation(const Plane &P, LA::AlignedMatrix4f &A, LA::AlignedVector4f &b, LA::AlignedVector4f &s)
	{
		if(m_measureIn3D)
		{
			PlaneEstimatorData3D::ConstructNormalEquation(P, A, b, s);
			return;
		}

		Point2D x, &e = x;
		ENFT_SSE::__m128 work, &ZcI = work;
		LA::AlignedMatrix2x3f JxXp;
		LA::AlignedMatrix3x4f JXpP;
		LA::AlignedMatrix2x4f JxP;

		A.SetZero();
		b.SetZero();
		s.SetZero();
		const uint nPts = Size();
		for(uint iPt = 0; iPt < nPts; ++iPt)
		{
			const Point3D &X = m_Xs[iPt], &Xp = m_Xps[iPt];
			const float &d = m_ds[iPt];
			work = ENFT_SSE::_mm_set1_ps(-P.A());		JXpP.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(work, X.XYZx());	JXpP.M00() -= d;
			work = ENFT_SSE::_mm_set1_ps(-P.B());		JXpP.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(work, X.XYZx());	JXpP.M11() -= d;
			work = ENFT_SSE::_mm_set1_ps(-P.C());		JXpP.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(work, X.XYZx());	JXpP.M22() -= d;
			const uint iMea1 = m_mapPtToMea[iPt], iMea2 = m_mapPtToMea[iPt + 1];
			for(uint iMea = iMea1; iMea < iMea2; ++iMea)
			{
				const Camera &C = *m_pCs[iMea];
				C.ProjectToNormalizedPlane(Xp, ZcI.m128_f32[0], x.x(), x.y());
				ZcI = ENFT_SSE::_mm_set1_ps(ZcI.m128_f32[0]);
				JxXp.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.x()), C.r_20_21_22_x())));
				JxXp.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.y()), C.r_20_21_22_x())));
				LA::AB(JxXp, JXpP, JxP);

				LA::AmB(m_xs[iMea], x, e);
				LA::AddATAToUpper(JxP, A);
				LA::AddATBTo(JxP, e, b);
				LA::AddAij2To(JxP, s);
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x4f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x4f, LA::Vector2f>(b);
		LA::FinishAdditionAij2To<LA::AlignedMatrix2x4f>(s);
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ssTA(s, A);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const Plane &P, const LA::AlignedVector4f &s, LA::AlignedMatrix4f &A, LA::AlignedVector4f &b)
	{
		if(m_measureIn3D)
		{
			PlaneEstimatorData3D::ConstructNormalEquation(P, s, A, b);
			return;
		}

		Point2D x, &e = x;
		ENFT_SSE::__m128 work, &ZcI = work;
		LA::AlignedMatrix2x3f JxXp;
		LA::AlignedMatrix3x4f JXpP;
		LA::AlignedMatrix2x4f JxP;

		A.SetZero();
		b.SetZero();
		const uint nPts = Size();
		for(uint iPt = 0; iPt < nPts; ++iPt)
		{
			const Point3D &X = m_Xs[iPt], &Xp = m_Xps[iPt];
			const float &d = m_ds[iPt];
			work = ENFT_SSE::_mm_set1_ps(-P.A());		JXpP.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(work, X.XYZx());	JXpP.M00() -= d;
			work = ENFT_SSE::_mm_set1_ps(-P.B());		JXpP.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(work, X.XYZx());	JXpP.M11() -= d;
			work = ENFT_SSE::_mm_set1_ps(-P.C());		JXpP.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(work, X.XYZx());	JXpP.M22() -= d;
			const uint iMea1 = m_mapPtToMea[iPt], iMea2 = m_mapPtToMea[iPt + 1];
			for(uint iMea = iMea1; iMea < iMea2; ++iMea)
			{
				const Camera &C = *m_pCs[iMea];
				C.ProjectToNormalizedPlane(Xp, ZcI.m128_f32[0], x.x(), x.y());
				ZcI = ENFT_SSE::_mm_set1_ps(ZcI.m128_f32[0]);
				JxXp.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_00_01_02_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.x()), C.r_20_21_22_x())));
				JxXp.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(ZcI, ENFT_SSE::_mm_sub_ps(C.r_10_11_12_x(), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(x.y()), C.r_20_21_22_x())));
				LA::AB(JxXp, JXpP, JxP);

				LA::AmB(m_xs[iMea], x, e);
				LA::AddATAToUpper(JxP, A);
				LA::AddATBTo(JxP, e, b);
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x4f>(A);
		A.SetLowerFromUpper();
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x4f, LA::Vector2f>(b);
		LA::ssTA(s, A);
		LA::sA(s, b);
	}

public:

	bool m_measureIn3D;

protected:

	std::vector<uint> m_mapPtToMea;
	std::vector<const Camera *> m_pCs;
	AlignedVector<Point2D> m_xs;

	float m_fxy;
	std::vector<float> m_ds;
	AlignedVector<Point3D> m_Xps;

};

#endif