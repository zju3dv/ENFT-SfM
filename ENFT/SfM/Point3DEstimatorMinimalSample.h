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

#ifndef _POINT_3D_ESTIMATOR_MINIMAL_SAMPLE_H_
#define _POINT_3D_ESTIMATOR_MINIMAL_SAMPLE_H_

#include "Point.h"

class Point3DEstimatorMinimalSample
{

public:

	inline void SetFirstCamera(const Camera &C) { m_C1 = &C; }
	inline void SetSecondCamera(const Camera &C) { m_C2 = &C; }
	inline void SetFirstMeasurement(const Point2D &x) { m_x1 = &x; }
	inline void SetSecondMeasurement(const Point2D &x) { m_x2 = &x; }
	inline const Camera& GetFirstCamera() const { return *m_C1; }
	inline const Camera& GetSecondCamera() const { return *m_C2; }
	inline const Point2D& GetFirstMeasurement() const { return *m_x1; }
	inline const Point2D& GetSecondMeasurement() const { return *m_x2; }

private:

	const Camera *m_C1, *m_C2;
	const Point2D *m_x1, *m_x2;

};

#endif