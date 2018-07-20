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

#ifndef _FUNDAMENTAL_MATRIX_ESTIMATOR_H_
#define _FUNDAMENTAL_MATRIX_ESTIMATOR_H_

//#include "Estimator.h"
#include "Estimation/EstimatorParsac.h"
#include "FundamentalMatrixEstimatorData.h"
#include "Optimization/Optimizer.h"
#include "FundamentalMatrixEightPointAlgorithm.h"

//typedef MatchSet2D																	FMEData;
typedef FundamentalMatrixEstimatorData													FMEData;
typedef EightMatches2D																	FMEMinimalSample;
//typedef MatchSet2D																	FMENonMinimalSample;
typedef FundamentalMatrixEstimatorData													FMENonMinimalSample;
typedef FundamentalMatrix																FMEModel;
typedef FundamentalMatrixEightPointAlgorithm											FMESolver;
typedef OptimizerTemplate<FundamentalMatrix, LA::AlignedVector8f, LA::AlignedMatrix8f>	FMEOptimizer;
typedef ushort																			FMEIndex;

//class FundamentalMatrixEstimator : public Estimator<FMEData, FMEMinimalSample, FMENonMinimalSample, FMEModel, FMESolver, FMEOptimizer, FMEIndex>
class FundamentalMatrixEstimator : public EstimatorParsac<FMEData, FMEMinimalSample, FMENonMinimalSample, FMEModel, FMESolver, FMEOptimizer, FMEIndex>
{

public:

	//FundamentalMatrixEstimator(const float &errTh = 0) : Estimator<FMEData, FMEMinimalSample, FMENonMinimalSample, FMEModel, FMESolver, FMEOptimizer>(errTh) {}
	FundamentalMatrixEstimator(const float &errTh = 0) : EstimatorParsac<FMEData, FMEMinimalSample, FMENonMinimalSample, FMEModel, FMESolver, FMEOptimizer, FMEIndex>(errTh) {}

	virtual const ushort MinimalSampleSize() const
	{
		return 8;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		float tmp = epsilon * epsilon;
		tmp = tmp * tmp;
		return tmp * tmp;
	}

	virtual void DrawMinimalSample(const FMEData &data, FMEMinimalSample &sample) const
	{
		const ushort N = data.Size();
		ushort i0 = Random::Generate(N);
		sample.Set(data, i0, 0);

		ushort i1 = Random::Generate(N);
		while(i1 == i0)
			i1 = Random::Generate(N);
		sample.Set(data, i1, 1);

		ushort i2 = Random::Generate(N);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(N);
		sample.Set(data, i2, 2);

		ushort i3 = Random::Generate(N);
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = Random::Generate(N);
		sample.Set(data, i3, 3);

		ushort i4 = Random::Generate(N);
		while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
			i4 = Random::Generate(N);
		sample.Set(data, i4, 4);

		ushort i5 = Random::Generate(N);
		while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4)
			i5 = Random::Generate(N);
		sample.Set(data, i5, 5);

		ushort i6 = Random::Generate(N);
		while(i6 == i0 || i6 == i1 || i6 == i2 || i6 == i3 || i6 == i4 || i6 == i5)
			i6 = Random::Generate(N);
		sample.Set(data, i6, 6);

		ushort i7 = Random::Generate(N);
		while(i7 == i0 || i7 == i1 || i7 == i2 || i7 == i3 || i7 == i4 || i7 == i5 || i7 == i6)
			i7 = Random::Generate(N);
		sample.Set(data, i7, 7);
	}

	virtual void DrawMinimalSampleOrdered(const FMEData &data, const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne, FMEMinimalSample &sample) const
	{
		ushort i0 = Random::Generate(n), i = orders[i0];
		sample.Set(data, i, 0);

		ushort i1 = Random::Generate(n);
		while(i1 == i0)
			i1 = Random::Generate(n);
		i = orders[i1];
		sample.Set(data, i, 1);

		ushort i2 = Random::Generate(n);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(n);
		i = orders[i2];
		sample.Set(data, i, 2);

		ushort i3 = Random::Generate(n);
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = Random::Generate(n);
		i = orders[i3];
		sample.Set(data, i, 3);

		ushort i4 = Random::Generate(n);
		while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
			i4 = Random::Generate(n);
		i = orders[i4];
		sample.Set(data, i, 4);

		ushort i5 = Random::Generate(n);
		while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4)
			i5 = Random::Generate(n);
		i = orders[i5];
		sample.Set(data, i, 5);

		ushort i6 = Random::Generate(n);
		while(i6 == i0 || i6 == i1 || i6 == i2 || i6 == i3 || i6 == i4 || i6 == i5)
			i6 = Random::Generate(n);
		i = orders[i6];
		sample.Set(data, i, 6);

		ushort i7 = n - 1;
		if(!sampleLastOne || i7 == i0 || i7 == i1 || i7 == i2 || i7 == i3 || i7 == i4 || i7 == i5 || i7 == i6)
		{
			i7 = Random::Generate(n);
			while(i7 == i0 || i7 == i1 || i7 == i2 || i7 == i3 || i7 == i4 || i7 == i5 || i7 == i6)
				i7 = Random::Generate(n);
		}
		i = orders[i7];
		sample.Set(data, i, 7);
	}

	virtual void DrawMinimalSample(const FMEData &data, const std::vector<std::vector<ushort> > &validBinData, FMEMinimalSample &sample) const
	{
		const ushort nBinsValid = ushort(validBinData.size());
		const uint i0 = Random::Generate(nBinsValid);
		ushort idx = validBinData[i0][rand() % validBinData[i0].size()];
		sample.Set(data, idx, 0);
		
		uint i1 = Random::Generate(nBinsValid);
		while(i1 == i0)
			i1 = Random::Generate(nBinsValid);
		idx = validBinData[i1][rand() % validBinData[i1].size()];
		sample.Set(data, idx, 1);
		
		uint i2 = Random::Generate(nBinsValid);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(nBinsValid);
		idx = validBinData[i2][rand() % validBinData[i2].size()];
		sample.Set(data, idx, 2);
		
		uint i3 = Random::Generate(nBinsValid);
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = Random::Generate(nBinsValid);
		idx = validBinData[i3][rand() % validBinData[i3].size()];
		sample.Set(data, idx, 3);
		
		uint i4 = Random::Generate(nBinsValid);
		while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
			i4 = Random::Generate(nBinsValid);
		idx = validBinData[i4][rand() % validBinData[i4].size()];
		sample.Set(data, idx, 4);
		
		uint i5 = Random::Generate(nBinsValid);
		while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4)
			i5 = Random::Generate(nBinsValid);
		idx = validBinData[i5][rand() % validBinData[i5].size()];
		sample.Set(data, idx, 5);
		
		uint i6 = Random::Generate(nBinsValid);
		while(i6 == i0 || i6 == i1 || i6 == i2 || i6 == i3 || i6 == i4 || i6 == i5)
			i6 = Random::Generate(nBinsValid);
		idx = validBinData[i6][rand() % validBinData[i6].size()];
		sample.Set(data, idx, 6);
		
		uint i7 = Random::Generate(nBinsValid);
		while(i7 == i0 || i7 == i1 || i7 == i2 || i7 == i3 || i7 == i4 || i7 == i5 || i7 == i6)
			i7 = Random::Generate(nBinsValid);
		idx = validBinData[i7][rand() % validBinData[i7].size()];
		sample.Set(data, idx, 7);
	}

	virtual void DrawMinimalSample(const FMEData &data, const std::vector<float> &validBinConfidencesAccumulated, const std::vector<std::vector<ushort> > &validBinData, 
								   FMEMinimalSample &sample) const
	{
		const uint i0 = Random::GenerateUint(validBinConfidencesAccumulated);
		ushort idx = validBinData[i0][rand() % validBinData[i0].size()];
		sample.Set(data, idx, 0);
		
		uint i1 = Random::GenerateUint(validBinConfidencesAccumulated);
		while(i1 == i0)
			i1 = Random::GenerateUint(validBinConfidencesAccumulated);
		idx = validBinData[i1][rand() % validBinData[i1].size()];
		sample.Set(data, idx, 1);
		
		uint i2 = Random::GenerateUint(validBinConfidencesAccumulated);
		while(i2 == i0 || i2 == i1)
			i2 = Random::GenerateUint(validBinConfidencesAccumulated);
		idx = validBinData[i2][rand() % validBinData[i2].size()];
		sample.Set(data, idx, 2);
		
		uint i3 = Random::GenerateUint(validBinConfidencesAccumulated);
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = Random::GenerateUint(validBinConfidencesAccumulated);
		idx = validBinData[i3][rand() % validBinData[i3].size()];
		sample.Set(data, idx, 3);
		
		uint i4 = Random::GenerateUint(validBinConfidencesAccumulated);
		while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
			i4 = Random::GenerateUint(validBinConfidencesAccumulated);
		idx = validBinData[i4][rand() % validBinData[i4].size()];
		sample.Set(data, idx, 4);
		
		uint i5 = Random::GenerateUint(validBinConfidencesAccumulated);
		while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4)
			i5 = Random::GenerateUint(validBinConfidencesAccumulated);
		idx = validBinData[i5][rand() % validBinData[i5].size()];
		sample.Set(data, idx, 5);
		
		uint i6 = Random::GenerateUint(validBinConfidencesAccumulated);
		while(i6 == i0 || i6 == i1 || i6 == i2 || i6 == i3 || i6 == i4 || i6 == i5)
			i6 = Random::GenerateUint(validBinConfidencesAccumulated);
		idx = validBinData[i6][rand() % validBinData[i6].size()];
		sample.Set(data, idx, 6);
		
		uint i7 = Random::GenerateUint(validBinConfidencesAccumulated);
		while(i7 == i0 || i7 == i1 || i7 == i2 || i7 == i3 || i7 == i4 || i7 == i5 || i7 == i6)
			i7 = Random::GenerateUint(validBinConfidencesAccumulated);
		idx = validBinData[i7][rand() % validBinData[i7].size()];
		sample.Set(data, idx, 7);
	}

	virtual void GenerateModels(FMEMinimalSample &sample, AlignedVector<FMEModel> &models)
	{
		models.Resize(1);
		m_work.Resize(6);
		sample.Normalize(m_work.Data());
		if(m_solver.Run(sample, models[0], m_work))
			models[0].Denormalize(sample.mean_u1v1u2v2(), sample.scale1(), sample.scale2(), m_work.Data());
		else
			models.Resize(0);
	}

	virtual void GenerateModels(FMENonMinimalSample &sample, AlignedVector<FMEModel> &models)
	{
		models.Resize(1);
		m_work.Resize(6);
		sample.Normalize(m_work.Data());
		if(m_solver.Run(sample, models[0], m_work))
			models[0].Denormalize(sample.mean_u1v1u2v2(), sample.scale1(), sample.scale2(), m_work.Data());
		else
			models.Resize(0);
	}

	virtual void VerifyModel(const FMEData &data, const FMEModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		m_work.Resize(16);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *F = &m_work[1], *work = &m_work[10];
		model.GetSSE(F);
		fitErr = 0;

		const ushort nPacks = data.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			FundamentalMatrix::ComputeSymmetricSquaredError4(F, data.GetPack(i), data.GetPack(ip1), data.GetPack(ip2), data.GetPack(ip3), errSq, work);
			if(inlierMarks[i])
				fitErr += errSq.m128_f32[0];
			if(inlierMarks[ip1])
				fitErr += errSq.m128_f32[1];
			if(inlierMarks[ip2])
				fitErr += errSq.m128_f32[2];
			if(inlierMarks[ip3])
				fitErr += errSq.m128_f32[3];
		}
		for(ushort iRem = 0; iRem < data.GetRemindersNumber(); ++iRem)
		{
			if(inlierMarks[nPacks + iRem])
				fitErr += model.ComputeSymmetricSquaredError(data.GetReminder1(iRem), data.GetReminder2(iRem), (float *) work);
		}
	}

	virtual void VerifyModel(const FMEData &data, const FMEModel &model, std::vector<ushort> &inliers, double &fitErr)
	{
		m_work.Resize(16);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *F = &m_work[1], *work = &m_work[10];
		model.GetSSE(F);
		inliers.resize(0);
		fitErr = 0;

		const ushort nPacks = data.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			FundamentalMatrix::ComputeSymmetricSquaredError4(F, data.GetPack(i), data.GetPack(ip1), data.GetPack(ip2), data.GetPack(ip3), errSq, work);
			if(errSq.m128_f32[0] < m_ransacErrorThreshold)
			{
				inliers.push_back(i);
				fitErr += errSq.m128_f32[0];
			}
			if(errSq.m128_f32[1] < m_ransacErrorThreshold)
			{
				inliers.push_back(ip1);
				fitErr += errSq.m128_f32[1];
			}
			if(errSq.m128_f32[2] < m_ransacErrorThreshold)
			{
				inliers.push_back(ip2);
				fitErr += errSq.m128_f32[2];
			}
			if(errSq.m128_f32[3] < m_ransacErrorThreshold)
			{
				inliers.push_back(ip3);
				fitErr += errSq.m128_f32[3];
			}
		}
		for(ushort iRem = 0; iRem < data.GetRemindersNumber(); ++iRem)
		{
			const float errSq = model.ComputeSymmetricSquaredError(data.GetReminder1(iRem), data.GetReminder2(iRem), (float *) work);
			if(errSq < m_ransacErrorThreshold)
			{
				inliers.push_back(nPacks + iRem);
				fitErr += errSq;
			}
		}
	}

	virtual void VerifyModel(const FMEData &data, const FMEModel &model, const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers, double &fitErr)
	{
		m_work.Resize(16);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *F = &m_work[1], *work = &m_work[10];
		model.GetSSE(F);
		inliers.resize(0);
		fitErr = 0;

		const ushort nPacks = data.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			FundamentalMatrix::ComputeSymmetricSquaredError4(F, data.GetPack(i), data.GetPack(ip1), data.GetPack(ip2), data.GetPack(ip3), errSq, work);
			if(errSq.m128_f32[0] < m_ransacErrorThreshold)
			{
				inliers.push_back(i);
				fitErr += errSq.m128_f32[0];
			}
			else if(inlierMarksFix[i])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
			if(errSq.m128_f32[1] < m_ransacErrorThreshold)
			{
				inliers.push_back(ip1);
				fitErr += errSq.m128_f32[1];
			}
			else if(inlierMarksFix[ip1])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
			if(errSq.m128_f32[2] < m_ransacErrorThreshold)
			{
				inliers.push_back(ip2);
				fitErr += errSq.m128_f32[2];
			}
			else if(inlierMarksFix[ip2])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
			if(errSq.m128_f32[3] < m_ransacErrorThreshold)
			{
				inliers.push_back(ip3);
				fitErr += errSq.m128_f32[3];
			}
			else if(inlierMarksFix[ip3])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
		}
		for(ushort iRem = 0; iRem < data.GetRemindersNumber(); ++iRem)
		{
			const float errSq = model.ComputeSymmetricSquaredError(data.GetReminder1(iRem), data.GetReminder2(iRem), (float *) work);
			if(errSq < m_ransacErrorThreshold)
			{
				inliers.push_back(nPacks + iRem);
				fitErr += errSq;
			}
			else if(inlierMarksFix[nPacks + iRem])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
		}
	}

	virtual void OptimizeModel(FMEData &data, FMEModel &model, const ubyte verbose = 0)
	{
		m_optimizer.m_lmMaxNumIters = m_optimizeMaxNumIters;
		m_optimizer.Run(data, model, verbose >= 2 ? verbose - 2 : 0);
	}

};

#endif