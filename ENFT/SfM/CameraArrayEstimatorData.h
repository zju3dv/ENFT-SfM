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

#ifndef _CAMERA_ARRAY_ESTIMATOR_DATA_H_
#define _CAMERA_ARRAY_ESTIMATOR_DATA_H_

#include "CameraEstimatorData.h"
#include "CameraArrayCalibrationParameter.h"

class CameraArrayEstimatorData : public CameraEstimatorData
{

public:

	inline void Resize(const ushort &N) { m_Xs.Resize(N); m_xs.Resize(N); m_iCams.resize(N); }
	inline void Reserve(const ushort &N) { m_Xs.Reserve(N); m_xs.Reserve(N); m_iCams.reserve(N); }
	inline const CameraIndex& GetCameraIndex(const ushort &i) const { return m_iCams[i]; }
	inline const std::vector<CameraIndex>& GetCameraIndexes() const { return m_iCams; }
	//inline const CameraIndex* GetCameraIndexes() const { return m_iCams.data(); }
	inline const CameraArrayCalibrationParameter& GetCalibrationParameter() const { return *m_pCalibParam; }
	inline void SetCalibrationParameter(const CameraArrayCalibrationParameter &calibParam) { m_pCalibParam = &calibParam; }
	inline void Set(const ushort &i, const Point3D &X, const Point2D &x, const CameraIndex &iCam) { m_Xs[i] = X; m_xs[i] = x; m_iCams[i] = iCam; }
	inline void GetSubset(const std::vector<ushort> &idxs, CameraArrayEstimatorData &subset) const
	{
		subset.SetCalibrationParameter(GetCalibrationParameter());
		const ushort N = ushort(idxs.size());
		subset.Resize(N);
		for(ushort i = 0; i < N; ++i)
			subset.Set(i, m_Xs[idxs[i]], m_xs[idxs[i]], m_iCams[idxs[i]]);
	}

private:

	const CameraArrayCalibrationParameter *m_pCalibParam;
	std::vector<CameraIndex> m_iCams;
};

#endif