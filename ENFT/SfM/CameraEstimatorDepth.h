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

#ifndef _CAMERA_ESTIMATOR_DEPTH_H_
#define _CAMERA_ESTIMATOR_DEPTH_H_

#include "Estimation/Estimator.h"
#include "CameraEstimatorDataDepth.h"
#include "RigidTransformationSolver.h"
#include "Optimization/Optimizer.h"
#include "Utility/Random.h"

typedef CameraEstimatorDataDepth													CEDData;
typedef SixMatches3D																CEDMinimalSample;
typedef MatchSet3DX																	CEDNonMinimalSample;
typedef Camera																		CEDModel;
typedef RigidTransformationSolver													CEDSolver;
typedef OptimizerTemplate<Camera, LA::AlignedVector6f, LA::AlignedCompactMatrix6f>	CEDOptimizer;
typedef ushort																		CEDIndex;
class CameraEstimatorDepth : public Estimator<CEDData, CEDMinimalSample, CEDNonMinimalSample, CEDModel, CEDSolver, CEDOptimizer, CEDIndex>
{

public:

	CameraEstimatorDepth(const float &errTh2D = 0, const float &errTh3D = 0.0f) : 
	  Estimator<CEDData, CEDMinimalSample, CEDNonMinimalSample, CEDModel, CEDSolver, CEDOptimizer, CEDIndex>(errTh2D), m_errTh2D(errTh2D), m_errTh3D(errTh3D) {}

	virtual const ushort MinimalSampleSize() const
	{
		return 6;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		const float tmp = epsilon * epsilon;
		return tmp * tmp * tmp;
	}

	virtual void DrawMinimalSample(const CEDData &data, CEDMinimalSample &sample) const
	{
		const MatchSet3DX &matches3D = data.GetMatches3D();
		const ushort N = matches3D.Size();
		ushort i0 = Random::Generate(N);
		sample.Set(0, matches3D.X1(i0), matches3D.X2(i0));

		ushort i1 = Random::Generate(N);
		while(i1 == i0)
			i1 = Random::Generate(N);
		sample.Set(1, matches3D.X1(i1), matches3D.X2(i1));

		ushort i2 = Random::Generate(N);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(N);
		sample.Set(2, matches3D.X1(i2), matches3D.X2(i2));

		ushort i3 = Random::Generate(N);
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = Random::Generate(N);
		sample.Set(3, matches3D.X1(i3), matches3D.X2(i3));

		ushort i4 = Random::Generate(N);
		while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
			i4 = Random::Generate(N);
		sample.Set(4, matches3D.X1(i4), matches3D.X2(i4));

		ushort i5 = Random::Generate(N);
		while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4)
			i5 = Random::Generate(N);
		sample.Set(5, matches3D.X1(i5), matches3D.X2(i5));
	}

	virtual void DrawMinimalSampleOrdered(const CEDData &data, const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne, 
										  CEDMinimalSample &sample) const
	{
		const MatchSet3DX &matches3D = data.GetMatches3D();
		ushort i0 = Random::Generate(n), i;
		while((i = data.GetIndex3D(orders[i0])) == USHRT_MAX)
			i0 = Random::Generate(n);
		sample.Set(0, matches3D.X1(i), matches3D.X2(i));

		ushort i1 = Random::Generate(n);
		while(i1 == i0 || (i = data.GetIndex3D(orders[i1])) == USHRT_MAX)
			i1 = Random::Generate(n);
		sample.Set(1, matches3D.X1(i), matches3D.X2(i));

		ushort i2 = Random::Generate(n);
		while(i2 == i0 || i2 == i1 || (i = data.GetIndex3D(orders[i2])) == USHRT_MAX)
			i2 = Random::Generate(n);
		sample.Set(2, matches3D.X1(i), matches3D.X2(i));

		ushort i3 = Random::Generate(n);
		while(i3 == i0 || i3 == i1 || i3 == i2 || (i = data.GetIndex3D(orders[i3])) == USHRT_MAX)
			i3 = Random::Generate(n);
		sample.Set(3, matches3D.X1(i), matches3D.X2(i));

		ushort i4 = Random::Generate(n);
		while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3 || (i = data.GetIndex3D(orders[i4])) == USHRT_MAX)
			i4 = Random::Generate(n);
		sample.Set(4, matches3D.X1(i), matches3D.X2(i));

		ushort i5 = n - 1;
		if(!sampleLastOne || i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4 || (i = data.GetIndex3D(orders[i5])) == USHRT_MAX)
		{
			i5 = Random::Generate(n);
			while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4 || (i = data.GetIndex3D(orders[i5])) == USHRT_MAX)
				i5 = Random::Generate(n);
		}
		sample.Set(5, matches3D.X1(i), matches3D.X2(i));
	}

	virtual void GenerateModels(CEDMinimalSample &sample, AlignedVector<CEDModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0], m_work))
			models.Resize(0);
	}

	virtual void GenerateModels(CEDNonMinimalSample &sample, AlignedVector<CEDModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample.X1s(), sample.X2s(), models[0], m_work))
			models.Resize(0);
	}

	virtual void VerifyModel(const CEDData &data, const CEDModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		fitErr = 0;

		Point2D e;
		const ushort N = data.Size();
		for(ushort i = 0; i < N; ++i)
		{
			if(inlierMarks[i])
				fitErr += model.ComputeProjectionSquaredError(data.X(i), data.x(i), e);
		}
	}

	virtual void VerifyModel(const CEDData &data, const CEDModel &model, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		m_work.Resize(2);
		ENFT_SSE::__m128 &errSq = m_work[0], &work = m_work[1];
		const ushort N = data.Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
		Point2D d;
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
		{
			model.ComputeProjectionError2(data.X(ix2), data.X(ix2p1), *px2, d, errSq, work);
			errSq = ENFT_SSE::_mm_mul_ps(errSq, errSq);
			if(d.v0() > 0 && ((errSq.m128_f32[0] = errSq.m128_f32[0] + errSq.m128_f32[1]) < m_ransacErrorThreshold
			|| data.d(ix2) != 0.0f && errSq.m128_f32[0] < m_errTh2D && fabs(data.d(ix2) - d.v0()) < m_errTh3D))
			{
				inliers.push_back(ix2);
				fitErr += errSq.m128_f32[0];
			}
			if(d.v1() > 0 && ((errSq.m128_f32[2] = errSq.m128_f32[2] + errSq.m128_f32[3]) < m_ransacErrorThreshold
			|| data.d(ix2p1) != 0.0f && errSq.m128_f32[2] < m_errTh2D && fabs(data.d(ix2p1) - d.v1()) < m_errTh3D))
			{
				inliers.push_back(ix2p1);
				fitErr += errSq.m128_f32[2];
			}
		}
		if(_N != N)
		{
			float d;
			Point2D e;
			model.ComputeProjectionError(data.X(_N), data.x(_N), d, e);
			if(d > 0 && ((errSq.m128_f32[0] = e.SquaredLength()) < m_ransacErrorThreshold)
			|| data.d(_N) != 0.0f && errSq.m128_f32[0] < m_errTh2D && fabs(data.d(_N) - d) < m_errTh3D)
			{
				inliers.push_back(_N);
				fitErr += errSq.m128_f32[0];
			}
		}
	}

	virtual void VerifyModel(const CEDData &data, const CEDModel &model, const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		m_work.Resize(2);
		ENFT_SSE::__m128 &errSq = m_work[0], &work = m_work[1];
		const ushort N = data.Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
		Point2D d;
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
		{
			model.ComputeProjectionError2(data.X(ix2), data.X(ix2p1), *px2, d, errSq, work);
			errSq = ENFT_SSE::_mm_mul_ps(errSq, errSq);
			if(d.v0() > 0 && ((errSq.m128_f32[0] = errSq.m128_f32[0] + errSq.m128_f32[1]) < m_ransacErrorThreshold
			|| data.d(ix2) != 0.0f && errSq.m128_f32[0] < m_errTh2D && fabs(data.d(ix2) - d.v0()) < m_errTh3D))
			{
				inliers.push_back(ix2);
				fitErr += errSq.m128_f32[0];
			}
			else if(inlierMarksFix[ix2])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
			if(d.v1() > 0 && ((errSq.m128_f32[2] = errSq.m128_f32[2] + errSq.m128_f32[3]) < m_ransacErrorThreshold
			|| data.d(ix2p1) != 0.0f && errSq.m128_f32[2] < m_errTh2D && fabs(data.d(ix2p1) - d.v1()) < m_errTh3D))
			{
				inliers.push_back(ix2p1);
				fitErr += errSq.m128_f32[2];
			}
			else if(inlierMarksFix[ix2p1])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
		}
		if(_N != N)
		{
			float d;
			Point2D e;
			model.ComputeProjectionError(data.X(_N), data.x(_N), d, e);
			if(d > 0 && ((errSq.m128_f32[0] = e.SquaredLength()) < m_ransacErrorThreshold)
			|| data.d(_N) != 0.0f && errSq.m128_f32[0] < m_errTh2D && fabs(data.d(_N) - d) < m_errTh3D)
			{
				inliers.push_back(_N);
				fitErr += errSq.m128_f32[0];
			}
			else if(inlierMarksFix[_N])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
		}
	}

	virtual void OptimizeModel(CEDData &data, CEDModel &model, const ubyte verbose = 0)
	{
		AlignedVector<float> &ds = data.ds();
		const ushort N = data.Size();
		for(ushort i = 0; i < N; ++i)
		{
			if(ds[i] != 0.0f && fabs(ds[i] - model.ComputeDepth(data.X(i))) > m_errTh3D)
				ds[i] = 0.0f;
		}

		m_optimizer.m_lmMaxNumIters = m_optimizeMaxNumIters;
		m_optimizer.Run(data, model, verbose >= 2 ? verbose - 2 : 0);
	}

public:

	float m_errTh2D, m_errTh3D;

};

#endif