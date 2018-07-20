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

#ifndef _TRANSLATION_SOLVER_H_
#define _TRANSLATION_SOLVER_H_

#include "Match.h"
#include "Translation.h"
#include "RotationTransformation.h"

class TranslationSolver
{

public:

	bool Run(const TwoMatches3DTo2D &data, Translation &T, AlignedVector<ENFT_SSE::__m128> &work);
	bool Run(const MatchSet3DTo2DX &data, Translation &T, AlignedVector<ENFT_SSE::__m128> &work);
	void Run(const RotationTransformation3D &R, const AlignedVector<Point3D> &X1s, const AlignedVector<Point3D> &X2s, LA::AlignedVector3f &t);

private:

	LA::AlignedMatrix3f m_ATA;

};

#endif