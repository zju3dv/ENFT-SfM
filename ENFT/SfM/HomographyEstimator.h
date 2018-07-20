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

#ifndef _HOMOGRAPHY_ESTIMATOR_H_
#define _HOMOGRAPHY_ESTIMATOR_H_

#include "Estimation/Estimator.h"
#include "HomographyEstimatorData.h"
#include "Optimization/Optimizer.h"
#include "HomographySolver.h"

typedef HomographyEstimatorData													HEData;
typedef FourMatches2D															HEMinimalSample;
typedef HomographyEstimatorData													HENonMinimalSample;
typedef Homography																HEModel;
typedef HomographySolver														HESolver;
typedef OptimizerTemplate<Homography, LA::AlignedVector8f, LA::AlignedMatrix8f>	HEOptimizer;
typedef ushort																	HEIndex;

class HomographyEstimator : public Estimator<HEData, HEMinimalSample, HENonMinimalSample, HEModel, HESolver, HEOptimizer, HEIndex>
{

public:

	HomographyEstimator(const float &errTh = 0) : Estimator<HEData, HEMinimalSample, HENonMinimalSample, HEModel, HESolver, HEOptimizer, HEIndex>(errTh) {}

	virtual const ushort MinimalSampleSize() const
	{
		return 4;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		float tmp = epsilon * epsilon;
		return tmp * tmp;
	}

	virtual void DrawMinimalSample(const HEData &data, HEMinimalSample &sample) const
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
	}

	virtual void DrawMinimalSampleOrdered(const HEData &data, const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne, HEMinimalSample &sample) const
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

		ushort i3 = n - 1;
		if(!sampleLastOne || i3 == i0 || i3 == i1 || i3 == i2)
		{
			i3 = Random::Generate(n);
			while(i3 == i0 || i3 == i1 || i3 == i2)
				i3 = Random::Generate(n);
		}
		i = orders[i3];
		sample.Set(data, i, 3);
	}

	//virtual void SampleOneDatum(const HEData &data, HEMinimalSample &sample, const ushort &iSrc, const ushort &iDst) const
	//{
	//	sample.Set(data, iSrc, iDst);
	//}

	virtual void GenerateModels(HEMinimalSample &sample, AlignedVector<HEModel> &models)
	{
		models.Resize(1);
		m_work.Resize(3);
		sample.Normalize(m_work.Data());
		if(m_solver.Run(sample, models[0], m_work))
			models[0].Denormalize(sample.mean_u1v1u2v2(), sample.scale1(), sample.scale2(), m_work.Data());
		else
			models.Resize(0);
	}

	virtual void GenerateModels(HENonMinimalSample &sample, AlignedVector<HEModel> &models)
	{
		models.Resize(1);
		m_work.Resize(6);
		sample.Normalize(m_work.Data());
		if(m_solver.Run(sample, models[0], m_work))
			models[0].Denormalize(sample.mean_u1v1u2v2(), sample.scale1(), sample.scale2(), m_work.Data());
		else
			models.Resize(0);
	}

	virtual void VerifyModel(const HEData &data, const HEModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		m_work.Resize(12);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *H = &m_work[1], *work = &m_work[10];
		model.GetSSE(H);
		fitErr = 0;

		const ushort nPacks = data.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			Homography::ComputeSquaredError4(H, data.GetPack(i), data.GetPack(ip1), data.GetPack(ip2), data.GetPack(ip3), errSq, work);
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
				fitErr += model.ComputeSquaredError(data.GetReminder1(iRem), data.GetReminder2(iRem), (float *) work);
		}
	}

	virtual void VerifyModel(const HEData &data, const HEModel &model, std::vector<float> &errSqs)
	{
		m_work.Resize(12);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *H = &m_work[1], *work = &m_work[10];
		model.GetSSE(H);

		errSqs.resize(data.Size());
		const ushort nPacks = data.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			Homography::ComputeSquaredError4(H, data.GetPack(i), data.GetPack(ip1), data.GetPack(ip2), data.GetPack(ip3), errSq, work);
			errSqs[i] = errSq.m128_f32[0];
			errSqs[ip1] = errSq.m128_f32[1];
			errSqs[ip2] = errSq.m128_f32[2];
			errSqs[ip3] = errSq.m128_f32[3];
		}
		for(ushort iRem = 0; iRem < data.GetRemindersNumber(); ++iRem)
			errSqs[nPacks + iRem] = model.ComputeSquaredError(data.GetReminder1(iRem), data.GetReminder2(iRem), (float *) work);
	}

	virtual void VerifyModel(const HEData &data, const HEModel &model, std::vector<ushort> &inliers, double &fitErr)
	{
		m_work.Resize(12);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *H = &m_work[1], *work = &m_work[10];
		model.GetSSE(H);
		inliers.resize(0);
		fitErr = 0;

		const ushort nPacks = data.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			Homography::ComputeSquaredError4(H, data.GetPack(i), data.GetPack(ip1), data.GetPack(ip2), data.GetPack(ip3), errSq, work);
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
			const float errSq = model.ComputeSquaredError(data.GetReminder1(iRem), data.GetReminder2(iRem), (float *) work);
			if(errSq < m_ransacErrorThreshold)
			{
				inliers.push_back(nPacks + iRem);
				fitErr += errSq;
			}
		}
	}

	virtual void VerifyModel(const HEData &data, const HEModel &model, const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers, double &fitErr)
	{
		m_work.Resize(12);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *H = &m_work[1], *work = &m_work[10];
		model.GetSSE(H);
		inliers.resize(0);
		fitErr = 0;

		const ushort nPacks = data.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			Homography::ComputeSquaredError4(H, data.GetPack(i), data.GetPack(ip1), data.GetPack(ip2), data.GetPack(ip3), errSq, work);
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
			const float errSq = model.ComputeSquaredError(data.GetReminder1(iRem), data.GetReminder2(iRem), (float *) work);
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

	virtual void OptimizeModel(HEData &data, HEModel &model, const ubyte verbose = 0)
	{
		m_optimizer.m_lmMaxNumIters = m_optimizeMaxNumIters;
		m_optimizer.Run(data, model, verbose >= 2 ? verbose - 2 : 0);
	}
};

#endif