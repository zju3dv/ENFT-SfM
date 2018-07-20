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

#ifndef _TRANSLATION_ESTIMATOR_H_
#define _TRANSLATION_ESTIMATOR_H_

#include "Estimation/EstimatorParsac.h"
#include "TranslationEstimatorData.h"
#include "TranslationSolver.h"
#include "Optimization/Optimizer.h"
#include "Utility/Random.h"

typedef TranslationEstimatorData													TEData;
typedef TwoMatches3DTo2D															TEMinimalSample;
typedef TranslationEstimatorData													TENonMinimalSample;
typedef Translation																	TEModel;
typedef TranslationSolver															TESolver;
typedef OptimizerTemplate<Translation, LA::AlignedVector3f, LA::AlignedMatrix3f>	TEOptimizer;
typedef ushort																		TEIndex;

class TranslationEstimator : public EstimatorParsac<TEData, TEMinimalSample, TENonMinimalSample, TEModel, TESolver, TEOptimizer, TEIndex>
{

public:

	TranslationEstimator(const float &errTh = 0) : EstimatorParsac<TEData, TEMinimalSample, TENonMinimalSample, TEModel, TESolver, TEOptimizer, TEIndex>(errTh) {}
	
	virtual const ushort MinimalSampleSize() const
	{
		return 2;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		return epsilon * epsilon;
	}

	virtual void DrawMinimalSample(const TEData &data, TEMinimalSample &sample) const
	{
		const ushort N = data.Size();
		ushort i0 = Random::Generate(N);
		sample.Set(data.Xs(), data.xs(), i0, 0);
		ushort i1 = Random::Generate(N);
		while(i1 == i0)
			i1 = Random::Generate(N);
		sample.Set(data.Xs(), data.xs(), i1, 1);
	}

	virtual void DrawMinimalSampleOrdered(const TEData &data, const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne, TEMinimalSample &sample) const
	{
		ushort i0 = Random::Generate(n), i = orders[i0];
		sample.Set(data.Xs(), data.xs(), i, 0);

		ushort i1 = n - 1;
		if(!sampleLastOne || i1 == i0)
		{
			i1 = Random::Generate(n);
			while(i1 == i0)
				i1 = Random::Generate(n);
		}
		i = orders[i1];
		sample.Set(data.Xs(), data.xs(), i, 1);
	}

	virtual void DrawMinimalSample(const TEData &data, const std::vector<std::vector<ushort> > &validBinData, TEMinimalSample &sample) const
	{
		//iBinsValidSampled.resize(6);

		const ushort nBinsValid = ushort(validBinData.size());
		const uint i0 = Random::Generate(nBinsValid);
		ushort idx = validBinData[i0][rand() % validBinData[i0].size()];
		sample.Set(data.Xs(), data.xs(), idx, 0);
		//iBinsValidSampled[0] = i0;

		uint i1 = Random::Generate(nBinsValid);
		while(i1 == i0)
			i1 = Random::Generate(nBinsValid);
		idx = validBinData[i1][rand() % validBinData[i1].size()];
		sample.Set(data.Xs(), data.xs(), idx, 1);
		//iBinsValidSampled[1] = i1;
	}

	virtual void DrawMinimalSample(const TEData &data, const std::vector<float> &validBinConfidencesAccumulated, const std::vector<std::vector<ushort> > &validBinData, 
								   TEMinimalSample &sample) const
	{
		//iBinsValidSampled.resize(6);

		const uint i0 = Random::GenerateUint(validBinConfidencesAccumulated);
		ushort idx = validBinData[i0][rand() % validBinData[i0].size()];
		sample.Set(data.Xs(), data.xs(), idx, 0);
		//iBinsValidSampled[0] = i0;

		uint i1 = Random::GenerateUint(validBinConfidencesAccumulated);
		while(i1 == i0)
			i1 = Random::GenerateUint(validBinConfidencesAccumulated);
		idx = validBinData[i1][rand() % validBinData[i1].size()];
		sample.Set(data.Xs(), data.xs(), idx, 1);
		//iBinsValidSampled[1] = i1;
	}

	virtual void GenerateModels(TEMinimalSample &sample, AlignedVector<TEModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0], m_work))
			models.Resize(0);
	}

	virtual void GenerateModels(TENonMinimalSample &sample, AlignedVector<TEModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0], m_work))
			models.Resize(0);
	}

	virtual void VerifyModel(const TEData &data, const TEModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
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

	virtual void VerifyModel(const TEData &data, const TEModel &model, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		m_work.Resize(3);
		ENFT_SSE::__m128 *work = m_work.Data(), &errSq = m_work[2];
		const ushort N = data.Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
		Point2D Zc2;
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
		{
			model.ComputeProjectionError2(data.X(ix2), data.X(ix2p1), *px2, Zc2, errSq, work);
			errSq = ENFT_SSE::_mm_mul_ps(errSq, errSq);
			if(Zc2.v0() > 0 && (errSq.m128_f32[0] = errSq.m128_f32[0] + errSq.m128_f32[1]) < m_ransacErrorThreshold)
			{
				inliers.push_back(ix2);
				fitErr += errSq.m128_f32[0];
			}
			if(Zc2.v1() > 0 && (errSq.m128_f32[2] = errSq.m128_f32[2] + errSq.m128_f32[3]) < m_ransacErrorThreshold)
			{
				inliers.push_back(ix2p1);
				fitErr += errSq.m128_f32[2];
			}
		}
		if(_N != N)
		{
			Point2D &e = Zc2;
			if(model.ComputeProjectionError_CheckCheirality(data.X(_N), data.x(_N), e) && (errSq.m128_f32[0] = e.SquaredLength()) < m_ransacErrorThreshold)
			{
				inliers.push_back(_N);
				fitErr += errSq.m128_f32[0];
			}
		}
	}

	virtual void VerifyModel(const TEData &data, const TEModel &model, const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		m_work.Resize(3);
		ENFT_SSE::__m128 *work = m_work.Data(), &errSq = m_work[2];
		const ushort N = data.Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
		Point2D Zc2;
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
		{
			model.ComputeProjectionError2(data.X(ix2), data.X(ix2p1), *px2, Zc2, errSq, work);
			errSq = ENFT_SSE::_mm_mul_ps(errSq, errSq);
			if(Zc2.v0() > 0 && (errSq.m128_f32[0] = errSq.m128_f32[0] + errSq.m128_f32[1]) < m_ransacErrorThreshold)
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
			if(Zc2.v1() > 0 && (errSq.m128_f32[2] = errSq.m128_f32[2] + errSq.m128_f32[3]) < m_ransacErrorThreshold)
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
			Point2D &e = Zc2;
			if(model.ComputeProjectionError_CheckCheirality(data.X(_N), data.x(_N), e) && (errSq.m128_f32[0] = e.SquaredLength()) < m_ransacErrorThreshold)
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

	virtual void OptimizeModel(TEData &data, TEModel &model, const ubyte verbose = 0)
	{
		m_optimizer.m_lmMaxNumIters = m_optimizeMaxNumIters;
		m_optimizer.Run(data, model, verbose >= 2 ? verbose - 2 : 0);
	}
};

#endif