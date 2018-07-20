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

#ifndef _CAMERA_PAIR_ESTIMATOR_H_
#define _CAMERA_PAIR_ESTIMATOR_H_

//#include "Estimator.h"
#include "Estimation/EstimatorArsac.h"
#include "CameraPair.h"
#include "CameraPairEstimatorData.h"
#include "CameraPairEstimatorMinimalSample.h"
#include "SimilarityTransformationSolver.h"
#include "Optimization/Optimizer.h"
#include "Utility/Random.h"

typedef CameraPairEstimatorData														CPEData;
typedef CameraPairEstimatorMinimalSample											CPEMinimalSample;
typedef CameraPairEstimatorData														CPENonMinimalSample;
typedef CameraPair																	CPEModel;
typedef SimilarityTransformationSolver												CPESolver;
typedef OptimizerTemplate<Camera, LA::AlignedVector6f, LA::AlignedCompactMatrix6f>	CPEOptimizer;
typedef ushort																		CPEIndex;
//class CameraPair : public Estimator<CPEData, CPEMinimalSample, CPENonMinimalSample, CPEModel, CPESolver, CPEOptimizer, CPEIndex>
class CameraPairEstimator : public EstimatorArsac<CPEData, CPEMinimalSample, CPENonMinimalSample, CPEModel, CPESolver, CPEOptimizer, CPEIndex>
{

public:

	//CameraPairEstimator((const float &errTh = 0) : Estimator<CPEData, CPEMinimalSample, CPENonMinimalSample, CPEModel, CPESolver, CPEOptimizer, CPEIndex>(errTh) {}
	CameraPairEstimator(const float &errTh = 0) : EstimatorArsac<CPEData, CPEMinimalSample, CPENonMinimalSample, CPEModel, CPESolver, CPEOptimizer, CPEIndex>(errTh) {}
	
	virtual const ushort MinimalSampleSize() const
	{
		return 4;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		return epsilon * epsilon * epsilon;
	}

	virtual void DrawMinimalSample(const CPEData &data, CPEMinimalSample &sample) const
	{
		sample.SetCameraPair(data.C1(), data.C2());

		const ushort N = data.Size();
		ushort i0 = ushort(rand()) % N;
		sample.Set(0, data.X1(i0), data.X2(i0));

		ushort i1 = ushort(rand()) % N;
		while(i1 == i0)
			i1 = ushort(rand()) % N;
		sample.Set(1, data.X1(i1), data.X2(i1));

		ushort i2 = ushort(rand()) % N;
		while(i2 == i0 || i2 == i1)
			i2 = ushort(rand()) % N;
		sample.Set(2, data.X1(i2), data.X2(i2));

		ushort i3 = ushort(rand()) % N;
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = ushort(rand()) % N;
		sample.Set(3, data.X1(i3), data.X2(i3));
	}

	virtual void DrawMinimalSampleOrdered(const CPEData &data, const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne, CPEMinimalSample &sample) const
	{
		sample.SetCameraPair(data.C1(), data.C2());

		ushort i0 = rand() % n, i = orders[i0];
		sample.Set(0, data.X1(i), data.X2(i));

		ushort i1 = rand() % n;
		while(i1 == i0)
			i1 = rand() % n;
		i = orders[i1];
		sample.Set(1, data.X1(i), data.X2(i));

		ushort i2 = rand() % n;
		while(i2 == i0 || i2 == i1)
			i2 = rand() % n;
		i = orders[i2];
		sample.Set(2, data.X1(i), data.X2(i));

		ushort i3 = n - 1;
		if(!sampleLastOne || i3 == i0 || i3 == i1 || i3 == i2)
		{
			i3 = ushort(rand()) % n;
			while(i3 == i0 || i3 == i1 || i3 == i2)
				i3 = ushort(rand()) % n;
		}
		i = orders[i3];
		sample.Set(3, data.X1(i), data.X2(i));
	}

	virtual void DrawMinimalSample(const CPEData &data, const std::vector<std::vector<ushort> > &validBinData, CPEMinimalSample &sample) const
	{
		sample.SetCameraPair(data.C1(), data.C2());

		//iBinsValidSampled.resize(6);

		const ushort nBinsValid = ushort(validBinData.size());
		const uint i0 = rand() % nBinsValid;
		ushort idx = validBinData[i0][rand() % validBinData[i0].size()];
		sample.Set(0, data.X1(idx), data.X2(idx));
		//iBinsValidSampled[0] = i0;

		uint i1 = rand() % nBinsValid;
		while(i1 == i0)
			i1 = rand() % nBinsValid;
		idx = validBinData[i1][rand() % validBinData[i1].size()];
		sample.Set(1, data.X1(idx), data.X2(idx));
		//iBinsValidSampled[1] = i1;

		uint i2 = rand() % nBinsValid;
		while(i2 == i0 || i2 == i1)
			i2 = rand() % nBinsValid;
		idx = validBinData[i2][rand() % validBinData[i2].size()];
		sample.Set(2, data.X1(idx), data.X2(idx));
		//iBinsValidSampled[2] = i2;

		uint i3 = rand() % nBinsValid;
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = rand() % nBinsValid;
		idx = validBinData[i3][rand() % validBinData[i3].size()];
		sample.Set(3, data.X1(idx), data.X2(idx));
		//iBinsValidSampled[3] = i3;
	}

	virtual void GenerateModels(CPEMinimalSample &sample, AlignedVector<CPEModel> &models)
	{
		models.Resize(1);
		CameraPair &CP = models[0];
		if(!m_solver.Run(sample, CP.S(), m_work))
			models.Resize(0);
		else
		{
			m_work.Resize(2);
			SimilarityTransformation3D::AccumulateTransformation(CP.S(), sample.C2(), CP.C12(), m_work.Data());
			CP.S().Invert(m_work.Data());
			SimilarityTransformation3D::AccumulateTransformation(CP.S(), sample.C1(), CP.C21(), m_work.Data());
		}
	}

	virtual void GenerateModels(CPENonMinimalSample &sample, AlignedVector<CPEModel> &models)
	{
		models.Resize(1);
		CameraPair &CP = models[0];
		if(!m_solver.Run(sample.X1s(), sample.X2s(), CP.S(), m_work))
			models.Resize(0);
		else
		{
			m_work.Resize(2);
			SimilarityTransformation3D::AccumulateTransformation(CP.S(), sample.C2(), CP.C12(), m_work.Data());
			CP.S().Invert(m_work.Data());
			SimilarityTransformation3D::AccumulateTransformation(CP.S(), sample.C1(), CP.C21(), m_work.Data());
		}
	}

	virtual void VerifyModel(const CPEData &data, const CPEModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		fitErr = 0;

		m_work.Resize(2);
		ENFT_SSE::__m128 &e = m_work[0], &work1 = m_work[1];
		LA::Vector2f Zc;
		float errSq1, errSq2;

		const Camera &C12 = model.C12(), &C21 = model.C21();
		const ushort N = data.Size();
		for(ushort i = 0; i < N; ++i)
		{
			if(!inlierMarks[i])
				continue;
			Camera::ComputeProjectionError2(C12, C21, data.X1(i), data.X2(i), data.x2(i), data.x1(i), Zc, e, work1);
			e = ENFT_SSE::_mm_mul_ps(e, e);
			errSq1 = Zc.v0() > 0 ? e.m128_f32[0] + e.m128_f32[1] : FLT_MAX;
			errSq2 = Zc.v1() > 0 ? e.m128_f32[2] + e.m128_f32[3] : FLT_MAX;
			fitErr += errSq1 + errSq2;
		}
	}

	virtual void VerifyModel(const CPEData &data, const CPEModel &model, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		m_work.Resize(2);
		ENFT_SSE::__m128 &e = m_work[0], &work1 = m_work[1];
		LA::Vector2f Zc;
		float errSq1, errSq2;

		const Camera &C12 = model.C12(), &C21 = model.C21();
		const ushort N = data.Size();
		for(ushort i = 0; i < N; ++i)
		{
			Camera::ComputeProjectionError2(C12, C21, data.X1(i), data.X2(i), data.x2(i), data.x1(i), Zc, e, work1);
			e = ENFT_SSE::_mm_mul_ps(e, e);
			errSq1 = Zc.v0() > 0 ? e.m128_f32[0] + e.m128_f32[1] : FLT_MAX;
			errSq2 = Zc.v1() > 0 ? e.m128_f32[2] + e.m128_f32[3] : FLT_MAX;
			if(errSq1 < m_ransacErrorThreshold && errSq2 < m_ransacErrorThreshold)
			{
				inliers.push_back(i);
				fitErr += errSq1 + errSq2;
			}
		}
	}

	virtual void VerifyModel(const CPEData &data, const CPEModel &model, const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		m_work.Resize(2);
		ENFT_SSE::__m128 &e = m_work[0], &work1 = m_work[1];
		LA::Vector2f Zc;
		float errSq1, errSq2;

		const Camera &C12 = model.C12(), &C21 = model.C21();
		const ushort N = data.Size();
		for(ushort i = 0; i < N; ++i)
		{
			Camera::ComputeProjectionError2(C12, C21, data.X1(i), data.X2(i), data.x2(i), data.x1(i), Zc, e, work1);
			e = ENFT_SSE::_mm_mul_ps(e, e);
			errSq1 = Zc.v0() > 0 ? e.m128_f32[0] + e.m128_f32[1] : FLT_MAX;
			errSq2 = Zc.v1() > 0 ? e.m128_f32[2] + e.m128_f32[3] : FLT_MAX;
			if(errSq1 < m_ransacErrorThreshold && errSq2 < m_ransacErrorThreshold)
			{
				inliers.push_back(i);
				fitErr += errSq1 + errSq2;
			}
			else if(inlierMarksFix[i])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
		}
	}

	virtual void OptimizeModel(CPEData &data, CPEModel &model, const ubyte verbose = 0)
	{
		m_optimizer.m_lmMaxNumIters = m_optimizeMaxNumIters;

		m_data.SetFocal(1.0f);
		m_data.Xs() = data.X1s();
		m_data.xs() = data.x2s();
		m_optimizer.Run(m_data, model.C12(), verbose >= 2 ? verbose - 2 : 0);

		m_data.Xs() = data.X2s();
		m_data.xs() = data.x1s();
		m_optimizer.Run(m_data, model.C21(), verbose >= 2 ? verbose - 2 : 0);
	}

protected:

	CameraEstimatorData m_data;

};

#endif