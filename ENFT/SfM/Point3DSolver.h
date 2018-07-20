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

#ifndef _POINT_3D_SOLVER_H_
#define _POINT_3D_SOLVER_H_

#include "Point3DEstimatorData.h"
#include "Point3DEstimatorMinimalSample.h"
#include "LinearAlgebra/Matrix3.h"

class Point3DSolver
{

public:

	bool Run(const Point3DEstimatorMinimalSample &data, Point3D &X, AlignedVector<ENFT_SSE::__m128> &work, const bool metric);
	bool Run(const Point3DEstimatorData &data, Point3D &X, AlignedVector<ENFT_SSE::__m128> &work, const bool metric);

private:

	LA::AlignedMatrix3f m_ATA;

};

#endif