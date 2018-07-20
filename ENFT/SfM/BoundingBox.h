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

#ifndef _BOUNDING_BOX_H_
#define _BOUNDING_BOX_H_

#include "Point.h"
#include "IntrinsicMatrix.h"

class BoundingBox2D
{

public:

	BoundingBox2D() {}
	BoundingBox2D(const Point2D &xMin, const Point2D &xMax) : m_xMin(xMin), m_xMax(xMax) {}
	inline void Initialize() { m_xMin.Set(FLT_MAX, FLT_MAX); m_xMax.Set(-FLT_MAX, -FLT_MAX); }
	inline void Include(const Point2D &x)
	{
		if(x.x() < m_xMin.x())
			m_xMin.x() = x.x();
		if(x.y() < m_xMin.y())
			m_xMin.y() = x.y();
		if(x.x() > m_xMax.x())
			m_xMax.x() = x.x();
		if(x.y() > m_xMax.y())
			m_xMax.y() = x.y();
	}
	inline void Set(const Point2D &xMin, const Point2D &xMax) { m_xMin = xMin; m_xMax = xMax; }
	inline void Get(Point2D &xMin, Point2D &xMax) const { xMin = m_xMin; xMax = m_xMax; }
	inline const Point2D& GetMinimal() const { return m_xMin; }
	inline const Point2D& GetMaximal() const { return m_xMax; }
	inline bool Inside(const Point2D &x) const { return x > m_xMin && x < m_xMax; }
	inline float ComputeDiagonalLength() const { return sqrt(m_xMin.SquaredDistance(m_xMax)); }
	inline void NormalizedPlaneToImage(const IntrinsicMatrix &K) { K.NormalizedPlaneToImage(m_xMin); K.NormalizedPlaneToImage(m_xMax); }
	inline float GetWidth() const { return m_xMax.x() - m_xMin.x(); }
	inline float GetHeight() const { return m_xMax.y() - m_xMin.y(); }

protected:

	Point2D m_xMin, m_xMax;

};

class AxisAlignedBoundingBox3D
{

public:

	inline void SetLowerBound(const float &X, const float &Y, const float &Z) { m_Xmin.Set(X, Y, Z); }
	inline void SetUpperBound(const float &X, const float &Y, const float &Z) { m_Xmax.Set(X, Y, Z); }
	inline const Point3D& GetLowerBound() const { return m_Xmin; }
	inline const Point3D& GetUpperBound() const { return m_Xmax; }
	inline void Scale(const float &s)
	{
		const ENFT_SSE::__m128 sss1 = _mm_setr_ps(s, s, s, 1.0f);
		m_Xmin *= sss1;
		m_Xmax *= sss1;
	}

	inline void Initialize() { m_Xmin.Set(FLT_MAX, FLT_MAX, FLT_MAX); m_Xmax.Set(-FLT_MAX, -FLT_MAX, -FLT_MAX); m_Xmin.reserve() = m_Xmax.reserve() =1.0f; }
	inline void Update(const Point3D &X)
	{
		m_Xmin.XYZx() = _mm_min_ps(m_Xmin.XYZx(), X.XYZx());
		m_Xmax.XYZx() = _mm_max_ps(m_Xmax.XYZx(), X.XYZx());
	}

protected:

	Point3D m_Xmin, m_Xmax;

public:

	static inline bool ComputeIntersection(const AlignedVector<AxisAlignedBoundingBox3D> &Bs, const Point3D &Xori, const Point3D &dir, Point3D &Xintersect)
	{
		const Point3D one_over_dir(1 / dir.X(), 1 / dir.Y(), 1 / dir.Z());
		float t, tMin = FLT_MAX;
		const uint nBoxes = Bs.Size();
		for(uint iBox = 0; iBox < nBoxes; ++iBox)
		{
			const Point3D &Xmin = Bs[iBox].m_Xmin, &Xmax = Bs[iBox].m_Xmax;
			const float t1 = (Xmin.X() - Xori.X()) * one_over_dir.X();
			const float t2 = (Xmax.X() - Xori.X()) * one_over_dir.X();
			const float t3 = (Xmin.Y() - Xori.Y()) * one_over_dir.Y();
			const float t4 = (Xmax.Y() - Xori.Y()) * one_over_dir.Y();
			const float t5 = (Xmin.Z() - Xori.Z()) * one_over_dir.Z();
			const float t6 = (Xmax.Z() - Xori.Z()) * one_over_dir.Z();
			const float t7 = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

			t = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
			if(t7 >= 0 && t7 >= t && t < tMin)
				tMin = t;
		}
		if(tMin == FLT_MAX)
			return false;
		LA::sA(tMin, dir, Xintersect);
		Xintersect += Xori;
		Xintersect.reserve() = 1.0f;
		return true;
	}
	static inline bool ComputeIntersection(const AlignedVector<AxisAlignedBoundingBox3D> &Bs, const Point3D &Xori, const Point3D &dir, Point3D &Xintersect, 
										   ushort &iBoxIntersect, ushort &iPlaneIntersect)
	{
		const Point3D one_over_dir(1 / dir.X(), 1 / dir.Y(), 1 / dir.Z());
		float t, tMin = FLT_MAX;
		iBoxIntersect = iPlaneIntersect = USHRT_MAX;
		const ushort nBoxes = ushort(Bs.Size());
		for(ushort iBox = 0; iBox < nBoxes; ++iBox)
		{
			const Point3D &Xmin = Bs[iBox].m_Xmin, &Xmax = Bs[iBox].m_Xmax;
			const float t1 = (Xmin.X() - Xori.X()) * one_over_dir.X();
			const float t2 = (Xmax.X() - Xori.X()) * one_over_dir.X();
			const float t3 = (Xmin.Y() - Xori.Y()) * one_over_dir.Y();
			const float t4 = (Xmax.Y() - Xori.Y()) * one_over_dir.Y();
			const float t5 = (Xmin.Z() - Xori.Z()) * one_over_dir.Z();
			const float t6 = (Xmax.Z() - Xori.Z()) * one_over_dir.Z();
			const float t7 = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

			t = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
			if(t7 >= 0 && t7 >= t && t < tMin)
			{
				tMin = t;
				iBoxIntersect = iBox;
			}
		}
		if(iBoxIntersect == USHRT_MAX)
			return false;
		LA::sA(tMin, dir, Xintersect);
		Xintersect += Xori;
		Xintersect.reserve() = 1.0f;

		const Point3D &Xmin = Bs[iBoxIntersect].m_Xmin, &Xmax = Bs[iBoxIntersect].m_Xmax;
		const float t1 = fabs(Xmin.X() - Xintersect.X());
		const float t2 = fabs(Xmax.X() - Xintersect.X());
		const float t3 = fabs(Xmin.Y() - Xintersect.Y());
		const float t4 = fabs(Xmax.Y() - Xintersect.Y());
		const float t5 = fabs(Xmin.Z() - Xintersect.Z());
		const float t6 = fabs(Xmax.Z() - Xintersect.Z());
		t = std::min(std::min(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
		if(t == t1)
			iPlaneIntersect = 1;
		else if(t == t2)
			iPlaneIntersect = 0;
		else if(t == t3)
			iPlaneIntersect = 2;
		else if(t == t4)
			return false;
		else if(t == t5)
			iPlaneIntersect = 4;
		else if(t == t6)
			iPlaneIntersect = 3;
		return true;
	}

};

#endif