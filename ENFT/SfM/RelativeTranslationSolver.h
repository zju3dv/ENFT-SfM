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

#ifndef _RELATIVE_TRANSLATION_SOLVER_H_
#define _RELATIVE_TRANSLATION_SOLVER_H_

#include "SfM/Match.h"
#include "RelativeTranslation.h"
#include "LinearAlgebra/MatrixMxN.h"

class RelativeTranslationSolver
{

public:

	bool Run(const TwoMatches3DTo2D &data, RelativeTranslation &T, AlignedVector<ENFT_SSE::__m128> &work);
	bool Run(const MatchSet3DTo2DX &data, RelativeTranslation &T, AlignedVector<ENFT_SSE::__m128> &work);

protected:

	float m_AT3x2[3][2], m_vt[3][3], m_s[2];
	LA::AlignedMatrix3xXf m_AT3xX;

};

#endif