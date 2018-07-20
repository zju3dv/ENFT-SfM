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

#ifndef _TRANSLATION_SCALE_SOLVER_H_
#define _TRANSLATION_SCALE_SOLVER_H_

#include "TranslationScaleEstimatorData.h"
#include "Utility/AlignedVector.h"

class TranslationScaleSolver
{

public:

	static inline void Run(const Point3D &RX, const Point2D &x, const LA::AlignedVector3f &t, LA::Vector2f &scale)
	{
		scale.v0() = (x.x() * RX.Z() - RX.X()) / (t.v0() - x.x() * t.v2());
		scale.v1() = (x.y() * RX.Z() - RX.Y()) / (t.v1() - x.y() * t.v2());
	}
	inline bool Run(const TranslationScaleEstimatorData &data, float &s)
	{
		m_scales.resize(0);
		const ushort N = ushort(data.Size());
		for(ushort i = 0; i < N; ++i)
		{
			const LA::Vector2f &scale = data.GetScale(i);
			//if(scale.v0() > 0)
			//	m_scales.push_back(scale.v0());
			//if(scale.v1() > 0)
			//	m_scales.push_back(scale.v1());
			if(scale.v0() != 0)
				m_scales.push_back(scale.v0());
			if(scale.v1() != 0)
				m_scales.push_back(scale.v1());
		}
		if(m_scales.empty())
			return false;
		const ushort ith = ushort(m_scales.size() >> 1);
		std::nth_element(m_scales.begin(), m_scales.begin() + ith, m_scales.end());
		s = m_scales[ith];
		return true;
	}

private:

	std::vector<float> m_scales;

};

#endif