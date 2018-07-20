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

#ifndef _RELATIVE_POSE_ESTIMATOR_DATA_DEPTH_H_
#define _RELATIVE_POSE_ESTIMATOR_DATA_DEPTH_H_

#include "Match.h"

class RelativePoseEstimatorDataDepth
{

public:

	inline const ushort& Size() const { return m_matches2D.Size(); }
	inline void Resize2D(const ushort &N) { m_matches2D.Resize(N); }
	inline void Resize3D(const ushort &N) { m_matches3D.Resize(N); }
	inline const MatchSet2D& GetMatches2D() const { return m_matches2D; }
	inline const MatchSet3DX& GetMatches3D() const { return m_matches3D; }
	inline void SetMatch2D(const ushort &i2D, const Point2D &x1, const Point2D &x2) { m_matches2D.SetMatch(i2D, x1, x2); }
	inline void FinishSettingMatches2D() { m_matches2D.FinishSettingMatches(); m_map2DTo3D.assign(m_matches2D.Size(), USHRT_MAX); }
	inline const ushort& GetIndex3D(const ushort &i2D) const { return m_map2DTo3D[i2D]; }
	inline void SetMatch3D(const ushort &i3D, const ushort &i2D, const Point3D &X1, const Point3D &X2)
	{
		m_matches3D.X1(i3D) = X1;
		m_matches3D.X2(i3D) = X2;
		m_map2DTo3D[i2D] = i3D;
	}
	inline void GetSubset(const std::vector<ushort> &idxs, MatchSet3DX &subset) const
	{
		ushort i, j, i3D;
		const ushort N = ushort(idxs.size());
		subset.Resize(N);
		for(i = j = 0; i < N; ++i)
		{
			if((i3D = m_map2DTo3D[idxs[i]]) == USHRT_MAX)
				continue;
			subset.X1(j) = m_matches3D.X1(i3D);
			subset.X2(j) = m_matches3D.X2(i3D);
			++j;
		}
		subset.Resize(j);
	}
	inline void GetSubset(const std::vector<ushort> &idxs, RelativePoseEstimatorDataDepth &subset) const
	{
		m_matches2D.GetSubset(idxs, subset.m_matches2D);
		subset.m_matches3D.Resize(0);
		subset.m_map2DTo3D.assign(subset.m_matches2D.Size(), USHRT_MAX);
	}

protected:

	MatchSet2D m_matches2D;
	MatchSet3DX m_matches3D;
	std::vector<ushort> m_map2DTo3D;

};

#endif