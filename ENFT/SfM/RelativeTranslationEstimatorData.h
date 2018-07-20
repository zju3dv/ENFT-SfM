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

#ifndef _RELATIVE_TRANSLATION_ESTIMATOR_DATA_H_
#define _RELATIVE_TRANSLATION_ESTIMATOR_DATA_H_

#include "Match.h"

class RelativeTranslationEstimatorData : public MatchSet3DTo2DX
{

public:

	inline const RotationTransformation3D& R() const { return m_R; }	inline RotationTransformation3D& R() { return m_R; }
	inline const MatchSet2D* const &pxs() const { return m_pxs; }		inline const MatchSet2D* &pxs() { return m_pxs; }
	inline void GetSubset(const std::vector<ushort> &idxs, RelativeTranslationEstimatorData &subset) const
	{
		MatchSet3DTo2DX::GetSubset(idxs, subset);
		subset.m_R = m_R;
		subset.m_pxs = m_pxs;
	}

protected:

	RotationTransformation3D m_R;
	const MatchSet2D *m_pxs;

};

#endif