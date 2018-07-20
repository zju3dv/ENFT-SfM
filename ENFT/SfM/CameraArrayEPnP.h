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

#ifndef _CAMERA_ARRAY_EPNP_H_
#define _CAMERA_ARRAY_EPNP_H_

#include "CameraArray.h"
#include "CameraArrayEstimatorData.h"
#include "CameraArrayEstimatorMinimalSample.h"
#include "RigidTransformationSolver.h"
#include "LinearAlgebra/Vector4.h"
#include "LinearAlgebra/Matrix3x8.h"
#include "LinearAlgebra/MatrixMxN.h"

class CameraArrayEPnP
{

public:

	bool Run(const CameraArrayEstimatorMinimalSample &data, CameraArray &CA, AlignedVector<ENFT_SSE::__m128> &work);
	bool Run(const CameraArrayEstimatorData &data, CameraArray &CA, AlignedVector<ENFT_SSE::__m128> &work);
	bool Debug(const CameraArrayEstimatorData &data, const CameraArray &CA, AlignedVector<ENFT_SSE::__m128> &work);

protected:

	bool ComputeCws(const CameraArrayEstimatorMinimalSample &data, AlignedVector<ENFT_SSE::__m128> &work);
	bool ComputeCws(const CameraArrayEstimatorData &data, AlignedVector<ENFT_SSE::__m128> &work);
	bool ComputeAlphas(const CameraArrayEstimatorMinimalSample &data, AlignedVector<ENFT_SSE::__m128> &work);
	bool ComputeAlphas(const CameraArrayEstimatorData &data, AlignedVector<ENFT_SSE::__m128> &work);
	void ConstructLinearSystem(const CameraArrayEstimatorMinimalSample &data, AlignedVector<ENFT_SSE::__m128> &work);
	void ConstructLinearSystem(const CameraArrayEstimatorData &data, AlignedVector<ENFT_SSE::__m128> &work);
	bool ComputeCcs(AlignedVector<ENFT_SSE::__m128> &work);
	bool ComputeXcs(const CameraArrayCalibrationParameter &calibParam, const CameraIndex *iCams);

protected:

	AlignedVector<Point3D> m_Xcs;
	AlignedVectorN<Point3D, 3> m_pds;
	AlignedVectorN<Point3D, 4> m_Cws, m_Ccs;
	LA::AlignedMatrix3x8f m_AT3x8;
	LA::AlignedMatrix3xXf m_AT3xX;
	LA::AlignedMatrix3f m_ATA;
	LA::AlignedVector16f m_w;
	LA::AlignedMatrix12xXd m_MT;
	LA::AlignedVectorXd m_b;
	AlignedVector<LA::AlignedVector4f> m_alphas;
	RigidTransformationSolver m_RtSolver;
};

#endif