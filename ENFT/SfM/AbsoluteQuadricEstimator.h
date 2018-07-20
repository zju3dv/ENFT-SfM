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

#ifndef _SELF_CALIBRATOR_H_
#define _SELF_CALIBRATOR_H_

#include "AbsoluteQuadricOptimizerData.h"
#include "LinearAlgebra/Matrix6x5.h"
#include "Optimization/Optimizer.h"

class AbsoluteQuadricEstimator
{

public:

	AbsoluteQuadricEstimator();
	bool Run(const AlignedVector<ProjectiveMatrix> &Ps, AbsoluteQuadric &Q, float &Ecalib);

protected:

	bool SolveAbsoluteQuadricLinear(const AbsoluteQuadricOptimizerData &data, const std::vector<uint> &idxs, AbsoluteQuadric &Q, const bool initial);

protected:

	AbsoluteQuadricOptimizerData m_data;
	std::vector<uint> m_idxs;
	AbsoluteQuadric m_Q;
	OptimizerTemplate<AbsoluteQuadric, LA::AlignedVector5f, LA::AlignedMatrix5f> m_optimizer;
	LA::AlignedVector6f m_eq;
	LA::AlignedMatrix6x5f m_A;
	LA::AlignedVector6f m_b;
	LA::AlignedMatrix5x6f m_AT;
	LA::AlignedMatrix5f m_ATA;
	LA::AlignedVector5f m_ATb;
	ENFT_SSE::__m128 m_ws[6], m_work;
	float m_weights[6];
//#if _DEBUG
//	AlignedVector<ProjectiveMatrix> m_Ps;
//#endif

};

#endif