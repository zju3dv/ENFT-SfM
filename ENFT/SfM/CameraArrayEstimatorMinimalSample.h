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

#ifndef _CAMERA_ARRAY_ESTIMATOR_MINIMAL_SAMPLE_H_
#define _CAMERA_ARRAY_ESTIMATOR_MINIMAL_SAMPLE_H_

#include "CameraArrayCalibrationParameter.h"
#include "Match.h"

class CameraArrayEstimatorMinimalSample : public EightMatches3DTo2D
{
public:
	inline const CameraArrayCalibrationParameter& GetCalibrationParameter() const { return *m_pCalibParam; }
	inline void SetCalibrationParameter(const CameraArrayCalibrationParameter &calibParam) { m_pCalibParam = &calibParam; }
	inline void Set(const AlignedVector<Point3D> &Xs, const AlignedVector<Point2D> &xs, const std::vector<CameraIndex> &iCams, const ushort &iSrc, const ushort &iDst)
	{
		//m_pXs[iDst] = &Xs[iSrc];
		//m_pxs[iDst] = &xs[iSrc];
		m_Xs[iDst] = Xs[iSrc];
		m_xs[iDst] = xs[iSrc];
		m_iCams[iDst] = iCams[iSrc];
	}
	inline const CameraIndex& GetCameraIndex(const ushort &i) const { return m_iCams[i]; }
	inline const CameraIndex* GetCameraIndexes() const { return m_iCams; }
private:
	const CameraArrayCalibrationParameter *m_pCalibParam;
	CameraIndex m_iCams[8];
};

#endif