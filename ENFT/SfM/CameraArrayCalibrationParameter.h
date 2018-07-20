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

#ifndef _CAMERA_ARRAY_CALIBRATION_PARAMETER_H_
#define _CAMERA_ARRAY_CALIBRATION_PARAMETER_H_

#include "IntrinsicMatrix.h"
#include "RigidTransformation.h"
#include "EssentialMatrix.h"
#include "FundamentalMatrix.h"

#define CAMERAS_NUMBER 2
typedef ubyte CameraIndex;

class CameraArrayCalibrationParameter
{
public:
	inline const IntrinsicMatrix& GetIntrinsicMatrix(const CameraIndex &iCam) const { return m_Ks[iCam]; }
	inline void SetIntrinsicMatrixes(const float &fx, const float &fy, const float &cx, const float &cy)
	{
		for(CameraIndex iCam = 0; iCam < CAMERAS_NUMBER; ++iCam)
			m_Ks[iCam].Set(fx, fy, cx, cy);
	}
	inline void SetIntrinsicMatrixes(const IntrinsicMatrix &K)
	{
		for(CameraIndex iCam = 0; iCam < CAMERAS_NUMBER; ++iCam)
			m_Ks[iCam] = K;
	}
	inline void SetRelativePose(const CameraIndex &iCam, const RigidTransformation3D &T) { m_Ts[iCam] = T; m_TsScaled[iCam] = T; }
	inline const RigidTransformation3D& GetRelativePose(const CameraIndex &iCam) const { return m_Ts[iCam]; }
	inline const RigidTransformation3D& GetRelativePoseScaled(const CameraIndex &iCam) const { return m_TsScaled[iCam]; }
	inline void ScaleRelativePoses(const float &s)
	{
		if(s == 1)
		{
			for(CameraIndex iCam = 0; iCam < CAMERAS_NUMBER; ++iCam)
			{
				m_TsScaled[iCam].tX() = m_Ts[iCam].tX();
				m_TsScaled[iCam].tY() = m_Ts[iCam].tY();
				m_TsScaled[iCam].tZ() = m_Ts[iCam].tZ();
			}
		}
		else
		{
			for(CameraIndex iCam = 0; iCam < CAMERAS_NUMBER; ++iCam)
			{
				m_TsScaled[iCam].tX() = m_Ts[iCam].tX() * s;
				m_TsScaled[iCam].tY() = m_Ts[iCam].tY() * s;
				m_TsScaled[iCam].tZ() = m_Ts[iCam].tZ() * s;
			}
		}
	}
	inline void SaveB(FILE *fp) const
	{
		for(CameraIndex iCam = 0; iCam < CAMERAS_NUMBER; ++iCam)
		{
			m_Ks[iCam].SaveB(fp);
			m_Ts[iCam].SaveB(fp);
		}
	}
	inline void LoadB(FILE *fp)
	{
		for(CameraIndex iCam = 0; iCam < CAMERAS_NUMBER; ++iCam)
		{
			m_Ks[iCam].LoadB(fp);
			m_Ts[iCam].LoadB(fp);
			m_TsScaled[iCam] = m_Ts[iCam];
		}
	}
	inline bool Load(const char *fileName)
	{
		FILE *fp = fopen( fileName, "r");
		if(!fp)
			return false;
		for(CameraIndex iCam = 0; iCam < CAMERAS_NUMBER; ++iCam)
		{
			m_Ks[iCam].Load(fp);
			m_Ts[iCam].Load(fp);
		}
		printf("[CameraArrayCalibrationParameter] Loaded \'%s\'\n", fileName);
		fclose(fp);
		return true;
	}
protected:
	IntrinsicMatrix m_Ks[CAMERAS_NUMBER];
	RigidTransformation3D m_Ts[CAMERAS_NUMBER], m_TsScaled[CAMERAS_NUMBER];
};

#endif