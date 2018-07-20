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

#ifndef _FIVE_POINT_ALGORITHM_H_
#define _FIVE_POINT_ALGORITHM_H_

#include "EssentialMatrix.h"
#include "SfM/Match.h"
#include "LinearAlgebra/Matrix9x5.h"
#include "LinearAlgebra/MatrixMxN.h"
#include "Utility/Polynomial.h"

class EssentialMatrixFivePointAlgorithm
{

public:

	bool Run(const FiveMatches2D &data, AlignedVector<EssentialMatrix> &Es, AlignedVector<ENFT_SSE::__m128> &work);
	bool Run(const MatchSet2D &data, AlignedVector<EssentialMatrix> &Es, AlignedVector<ENFT_SSE::__m128> &work);

protected:

	bool ExtractNullspace(const FiveMatches2D &data, AlignedVector<ENFT_SSE::__m128> &work);
	bool ExtractNullspace(const MatchSet2D &data, AlignedVector<ENFT_SSE::__m128> &work);
	void ExpandCubicConstraints(AlignedVector<ENFT_SSE::__m128> &work);
	bool ComputeGrobnerBasis(AlignedVector<ENFT_SSE::__m128> &work);
	void ComputeActionMatrix();
	bool ComputeExtrinsicMatrixes(AlignedVector<EssentialMatrix> &Es, AlignedVector<ENFT_SSE::__m128> &work);

protected:

	LA::AlignedMatrix9x5f m_QT9x5;
	LA::AlignedMatrix9xXf m_QT9xX;
	LA::AlignedVectorXf m_s;
	float m_wr[10], m_wi[10];
	LA::AlignedMatrix9f m_vt;
	Polynomial::V3E1f m_polyE[3][3];
	Polynomial::V3E2f m_polyTmp2[3], m_polyEET[6], m_polyTrace;
	Polynomial::V3E3f m_polyConstraints[10];
	Polynomial::V3E3fPermutation m_Gb[10];
	LA::AlignedMatrix10f m_At, m_vr;

};

#endif