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

#ifndef _SCALE_ESTIMATOR_DATA_H_
#define _SCALE_ESTIMATOR_DATA_H_

#include "Utility/AlignedVector.h"

class ScaleEstimatorData
{

public:

	inline const AlignedVector<float>& d1s() const { return m_d1s; }	inline AlignedVector<float>& d1s() { return m_d1s; }
	inline const AlignedVector<float>& d2s() const { return m_d2s; }	inline AlignedVector<float>& d2s() { return m_d2s; }
	inline const float& d1(const uint &i) const { return m_d1s[i]; }	inline float& d1(const uint &i) { return m_d1s[i]; }
	inline const float& d2(const uint &i) const { return m_d2s[i]; }	inline float& d2(const uint &i) { return m_d2s[i]; }
	inline const float& s (const uint &i) const { return m_ss[i]; }		inline float& s (const uint &i) { return m_ss[i]; }
	inline const uint& Size() const { return m_ss.Size(); }
	inline void Resize(const uint &N)
	{
		m_d1s.Resize(N);
		m_d2s.Resize(N);
		m_ss.Resize(N);
	}
	inline void ComputeScales()
	{
		const ENFT_SSE::__m128 *d1 = (ENFT_SSE::__m128 *) m_d1s.Data(), *d2 = (ENFT_SSE::__m128 *) m_d2s.Data();
		ENFT_SSE::__m128 *s = (ENFT_SSE::__m128 *) m_ss.Data();
		const uint N = m_ss.Size(), _N = N - (N & 3);
		for(uint i = 0; i < _N; i += 4, ++d1, ++d2, ++s)
			*s = ENFT_SSE::_mm_div_ps(*d2, *d1);
		for(uint i = _N; i < N; ++i)
			m_ss[i] = m_d2s[i] / m_d1s[i];
	}
	//inline void GetSubset(const std::vector<uint> &idxs, ScaleEstimatorData &subset) const
	//{
	//	const uint N = uint(idxs.size());
	//	subset.Resize(N);
	//	for(uint i = 0; i < N; ++i)
	//	{
	//		subset.m_d1s[i] = m_d1s[idxs[i]];
	//		subset.m_d2s[i] = m_d2s[idxs[i]];
	//		subset.m_ss[i] = m_ss[idxs[i]];
	//	}
	//}
	inline void GetSubset(const std::vector<uint> &idxs, std::vector<float> &subset) const
	{
		const uint N = uint(idxs.size());
		subset.resize(N);
		for(uint i = 0; i < N; ++i)
			subset[i] = m_ss[idxs[i]];
	}

protected:

	AlignedVector<float> m_d1s, m_d2s, m_ss;
	
};

#endif