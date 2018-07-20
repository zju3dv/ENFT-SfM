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

#ifndef _PLANE_ESTIMATOR_MINIMAL_SAMPLE_H_
#define _PLANE_ESTIMATOR_MINIMAL_SAMPLE_H_

#include "Point.h"

class PlaneEstimatorMinimalSample
{

public:

	inline const Point3D& X(const uint &i) const { return m_Xs[i]; }
	inline		 Point3D& X(const uint &i)		 { return m_Xs[i]; }

private:

	Point3D m_Xs[3];
};

#endif