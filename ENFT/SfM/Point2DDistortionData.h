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

#ifndef _POINT_2D_DISTORTION_DATA_H_
#define _POINT_2D_DISTORTION_DATA_H_

#include "Point.h"
#include "Optimization/OptimizerData.h"
#include "LinearAlgebra/Matrix2.h"

class Point2DDistortionData : public OptimizerDataTemplate<Point2D, LA::Vector2f, LA::AlignedMatrix2f>
{

public:

	inline void SetFocal(const float &fxy) { m_fxy = fxy; }
	inline void SetDistortion(const float &d) { m_d = d; }
	inline void SetPoint(const Point2D &x) { m_x = x; }

	virtual void NormalizeData(const float &dataNormalizeMedian, Point2D &xd) {}
	virtual void DenormalizeData(Point2D &xd) {}
	virtual double GetFactorSSEToMSE() { return m_fxy; }
	
	virtual double ComputeSSE(const Point2D &xd)
	{
		const float r2 = xd.SquaredLength();
		const float c = 1 + m_d * r2;
		m_e.x() = m_x.x() - c * xd.x();
		m_e.y() = m_x.y() - c * xd.y();
		return double(m_e.SquaredLength());
	}
	virtual void ConstructNormalEquation(const Point2D &xd, LA::AlignedMatrix2f &A, LA::Vector2f &b, LA::Vector2f &s)
	{
		m_dxd2.x() = m_d * xd.x() * xd.x();
		m_dxd2.y() = m_d * xd.y() * xd.y();
		m_J.M00() = 1 + 3 * m_dxd2.x() + m_dxd2.y();
		m_J.M11() = 1 + 3 * m_dxd2.y() + m_dxd2.x();
		m_J.M01() = m_J.M10() = 2 * m_d * xd.x() * xd.y();
		const float c = 1 + m_dxd2.x() + m_dxd2.y();
		m_e.x() = m_x.x() - c * xd.x();
		m_e.y() = m_x.y() - c * xd.y();
		LA::ATA(m_J, A, m_work);
		LA::ATB(m_J, m_e, b, m_work);
		LA::Aij2(m_J, s, m_work);
		LA::ssTA(s, A);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const Point2D &xd, const LA::Vector2f &s, LA::AlignedMatrix2f &A, LA::Vector2f &b)
	{
		m_dxd2.x() = m_d * xd.x() * xd.x();
		m_dxd2.y() = m_d * xd.y() * xd.y();
		m_J.M00() = 1 + 3 * m_dxd2.x() + m_dxd2.y();
		m_J.M11() = 1 + 3 * m_dxd2.y() + m_dxd2.x();
		m_J.M01() = m_J.M10() = 2 * m_d * xd.x() * xd.y();
		const float c = 1 + m_dxd2.x() + m_dxd2.y();
		m_e.x() = m_x.x() - c * xd.x();
		m_e.y() = m_x.y() - c * xd.y();
		LA::ATA(m_J, A, m_work);
		LA::ATB(m_J, m_e, b, m_work);
		LA::ssTA(s, A);
		LA::sA(s, b);
	}
	virtual void UpdateModel(const LA::Vector2f &s, const LA::Vector2f &x, const Point2D &xdOld, Point2D &xdNew)
	{
		xdNew.x() = xdOld.x() + s.v0() * x.v0();
		xdNew.y() = xdOld.y() + s.v1() * x.v1();
	}

protected:

	float m_fxy, m_d;
	Point2D m_x, m_e, m_dxd2;
	LA::AlignedMatrix2f m_J;
	ENFT_SSE::__m128 m_work;
};

#endif