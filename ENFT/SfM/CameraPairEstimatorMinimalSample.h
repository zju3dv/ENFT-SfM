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

#ifndef _CAMERA_PAIR_ESTIMATOR_MINIMAL_SAMPLE_H_
#define _CAMERA_PAIR_ESTIMATOR_MINIMAL_SAMPLE_H_

#include "Match.h"

class CameraPairEstimatorMinimalSample : public FourMatches3D
{

public:

	inline void SetCameraPair(const Camera &C1, const Camera &C2) { m_pC1 = &C1; m_pC2 = &C2; }
	inline const Camera& C1() const { return *m_pC1; }
	inline const Camera& C2() const { return *m_pC2; }

private:

	const Camera *m_pC1, *m_pC2;
};

#endif