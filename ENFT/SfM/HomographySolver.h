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

#ifndef _HOMOGRAPHY_SOLVER_H_
#define _HOMOGRAPHY_SOLVER_H_

#include "Homography.h"
#include "Match.h"
#include "LinearAlgebra/Matrix8.h"

class HomographySolver
{

public:

	bool Run(const FourMatches2D &data, Homography &H, AlignedVector<ENFT_SSE::__m128> &work);
	bool Run(const MatchSet2D &data, Homography &H, AlignedVector<ENFT_SSE::__m128> &work);

protected:

	LA::AlignedMatrix8f m_A, m_ATA;
	LA::AlignedVector8f m_b, m_ATb;
};

#endif