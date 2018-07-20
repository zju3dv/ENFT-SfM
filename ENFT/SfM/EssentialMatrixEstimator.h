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

#ifndef _ESSENTIAL_MATRIX_ESTIMATOR_H_
#define _ESSENTIAL_MATRIX_ESTIMATOR_H_

#include "Estimation/Estimator.h"
#include "EssentialMatrixEstimatorData.h"
#include "Optimization/Optimizer.h"
#include "EssentialMatrixFivePointAlgorithm.h"

typedef EssentialMatrixEstimatorData																EMEData;
typedef FiveMatches2D																				EMEMinimalSample;
typedef EssentialMatrixEstimatorData																EMENonMinimalSample;
typedef EssentialMatrix																				EMEModel;
typedef EssentialMatrixFivePointAlgorithm															EMESolver;
typedef OptimizerTemplate<RigidTransformation3D, LA::AlignedVector6f, LA::AlignedCompactMatrix6f>	EMEOptimizer;
typedef ushort																						EMEIndex;

class EssentialMatrixEstimator : public Estimator<EMEData, EMEMinimalSample, EMENonMinimalSample, EMEModel, EMESolver, EMEOptimizer, EMEIndex>
{

public:

	EssentialMatrixEstimator(const float &errTh = 0)
		: Estimator<EMEData, EMEMinimalSample, EMENonMinimalSample, EMEModel, EMESolver, EMEOptimizer, EMEIndex>(errTh), m_sccRatioTh(0.5f) {}

	virtual const ushort MinimalSampleSize() const
	{
		return 5;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		float tmp = epsilon * epsilon;
		return tmp * tmp * epsilon;
	}

	virtual void DrawMinimalSample(const EMEData &data, EMEMinimalSample &sample) const
	{
		const ushort N = data.Size();
		ushort i0 = Random::Generate(N);
		sample.Set0(data, i0);

		ushort i1 = Random::Generate(N);
		while(i1 == i0)
			i1 = Random::Generate(N);
		sample.Set1(data, i1);

		ushort i2 = Random::Generate(N);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(N);
		sample.Set2(data, i2);

		ushort i3 = Random::Generate(N);
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = Random::Generate(N);
		sample.Set3(data, i3);

		ushort i4 = Random::Generate(N);
		while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
			i4 = Random::Generate(N);
		sample.Set4(data, i4);
	}

	virtual void DrawMinimalSampleOrdered(const EMEData &data, const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne, 
										  EMEMinimalSample &sample) const
	{
		ushort i0 = Random::Generate(n), i = orders[i0];
		sample.Set0(data, i);

		ushort i1 = Random::Generate(n);
		while(i1 == i0)
			i1 = Random::Generate(n);
		i = orders[i1];
		sample.Set1(data, i);

		ushort i2 = Random::Generate(n);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(n);
		i = orders[i2];
		sample.Set2(data, i);

		ushort i3 = Random::Generate(n);
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = Random::Generate(n);
		i = orders[i3];
		sample.Set3(data, i);

		ushort i4 = n - 1;
		if(!sampleLastOne || i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
		{
			i4 = Random::Generate(n);
			while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
				i4 = Random::Generate(n);
		}
		i = orders[i4];
		sample.Set4(data, i);
	}

	virtual void GenerateModels(EMEMinimalSample &sample, AlignedVector<EMEModel> &models)
	{
		m_solver.Run(sample, models, m_work);
	}

	virtual void GenerateModels(EMENonMinimalSample &sample, AlignedVector<EMEModel> &models)
	{
		m_solver.Run(sample, models, m_work);
	}

	virtual void VerifyModel(const EMEData &data, const EMEModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		fitErr = 0;
		m_work.Resize(17);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *E = &m_work[1], *work = &m_work[10];
		model.GetSSE(E);
		const ushort nPacks = data.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			model.ComputeSymmetricSquaredError4(E, data.GetPack(i), data.GetPack(ip1), data.GetPack(ip2), data.GetPack(ip3), errSq, work);
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

	virtual void VerifyModel(const EMEData &data, const EMEModel &model, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;
		m_work.Resize(38);
		if(!model.ToRelativePose(data, m_T, m_sccRatioTh, m_work.Data()))
			return;
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *E = &m_work[1], *work = &m_work[10];
		model.GetSSE(E);
		const ushort nPacks = data.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			model.ComputeSymmetricSquaredError4(E, data.GetPack(i), data.GetPack(ip1), data.GetPack(ip2), data.GetPack(ip3), errSq, work);
			//errSq = _mm_sqrt_ps(errSq);
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

	virtual void VerifyModel(const EMEData &data, const EMEModel &model, const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;
		m_work.Resize(38);
		if(!model.ToRelativePose(data, m_T, m_sccRatioTh, m_work.Data()))
			return;
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *E = &m_work[1], *work = &m_work[10];
		model.GetSSE(E);
		const ushort nPacks = data.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			model.ComputeSymmetricSquaredError4(E, data.GetPack(i), data.GetPack(ip1), data.GetPack(ip2), data.GetPack(ip3), errSq, work);
			//errSq = _mm_sqrt_ps(errSq);
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

	virtual void OptimizeModel(EMEData &data, EMEModel &model, const ubyte verbose = 0)
	{
		m_work.Resize(38);
		if(!model.ToRelativePose(data, m_T, m_sccRatioTh, m_work.Data()))
			return;
		m_optimizer.m_lmMaxNumIters = m_optimizeMaxNumIters;
		m_optimizer.Run(data, m_T, verbose >= 2 ? verbose - 2 : 0);
		model.FromRelativePose(m_T);
	}

public:

	float m_sccRatioTh;

protected:

	RigidTransformation3D m_T;

};

#endif