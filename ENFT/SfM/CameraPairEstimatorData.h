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

#ifndef _CAMERA_PAIR_ESTIMATOR_DATA_H_
#define _CAMERA_PAIR_ESTIMATOR_DATA_H_

#include "SfM/Camera.h"
#include "Estimation/EstimatorArsacData.h"

class CameraPairEstimatorData : public EstimatorArsacData
{

public:

	inline const Point3D& X1(const ushort &i) const { return m_X1s[i]; }		inline Point3D& X1(const ushort &i) { return m_X1s[i]; }
	inline const Point3D& X2(const ushort &i) const { return m_X2s[i]; }		inline Point3D& X2(const ushort &i) { return m_X2s[i]; }
	inline const AlignedVector<Point3D>& X1s() const { return m_X1s; }			inline AlignedVector<Point3D>& X1s() { return m_X1s; }
	inline const AlignedVector<Point3D>& X2s() const { return m_X2s; }			inline AlignedVector<Point3D>& X2s() { return m_X2s; }

	inline const Point2D& x1(const ushort &i) const { return m_x1s[i]; }		inline Point2D& x1(const ushort &i) { return m_x1s[i]; }
	inline const Point2D& x2(const ushort &i) const { return m_x2s[i]; }		inline Point2D& x2(const ushort &i) { return m_x2s[i]; }
	inline const AlignedVector<Point2D>& x1s() const { return m_x1s; }			inline AlignedVector<Point2D>& x1s() { return m_x1s; }
	inline const AlignedVector<Point2D>& x2s() const { return m_x2s; }			inline AlignedVector<Point2D>& x2s() { return m_x2s; }

	inline void SetCameraPair(const Camera &C1, const Camera &C2) { m_pC1 = &C1; m_pC2 = &C2; }
	inline const Camera& C1() const { return *m_pC1; }
	inline const Camera& C2() const { return *m_pC2; }

	inline const ushort Size() const { return m_X1s.Size(); }
	inline void Resize(const ushort &N) { m_X1s.Resize(N); m_X2s.Resize(N); m_x1s.Resize(N); m_x2s.Resize(N); /*m_imgLocations.Resize(N);*/ }
	inline void GetSubset(const std::vector<ushort> &idxs, CameraPairEstimatorData &subset) const
	{
		const ushort N = ushort(idxs.size());
		subset.Resize(N);
		for(ushort i = 0; i < N; ++i)
		{
			const ushort idx = idxs[i];
			subset.m_X1s[i] = m_X1s[idx];
			subset.m_X2s[i] = m_X2s[idx];
			subset.m_x1s[i] = m_x1s[idx];
			subset.m_x2s[i] = m_x2s[idx];
		}
		subset.m_pC1 = m_pC1;
		subset.m_pC2 = m_pC2;
	}

protected:

	AlignedVector<Point3D> m_X1s, m_X2s;
	AlignedVector<Point2D> m_x1s, m_x2s;
	const Camera *m_pC1, *m_pC2;
};

#endif