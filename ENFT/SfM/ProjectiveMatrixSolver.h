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

#ifndef _PROJECTIVE_MATRIX_SOLVER_H_
#define _PROJECTIVE_MATRIX_SOLVER_H_

#include "ProjectiveMatrix.h"
#include "Match.h"
#include "ProjectiveMatrixEstimatorData.h"
#include "LinearAlgebra/Matrix11.h"

class ProjectiveMatrixSolver
{

public:

	bool Run(const SixMatches3DTo2D &data, ProjectiveMatrix &P, AlignedVector<ENFT_SSE::__m128> &work);
	bool Run(const ProjectiveMatrixEstimatorData &data, ProjectiveMatrix &P, AlignedVector<ENFT_SSE::__m128> &work);

protected:

	LA::AlignedMatrix11f m_ATA;
	LA::AlignedVector11f m_ATb;

};

#endif