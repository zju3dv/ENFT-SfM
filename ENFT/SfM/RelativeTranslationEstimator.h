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

#ifndef _RELATIVE_TRANSLATION_ESTIMATOR_H_
#define _RELATIVE_TRANSLATION_ESTIMATOR_H_

#include "Estimation/Estimator.h"
#include "RelativeTranslationEstimatorData.h"
#include "RelativeTranslationEstimatorMinimalSample.h"
#include "RelativeTranslationSolver.h"
#include "EssentialMatrix.h"

typedef RelativeTranslationEstimatorData			RTEData;
typedef RelativeTranslationEstimatorMinimalSample	RTEMinimalSample;
typedef RelativeTranslationEstimatorData			RTENonMinimalSample;
typedef RelativeTranslation							RTEModel;
typedef RelativeTranslationSolver					RTESolver;
typedef ubyte										RTEOptimizer;
typedef ushort										RTEIndex;

class RelativeTranslationEstimator : public Estimator<RTEData, RTEMinimalSample, RTENonMinimalSample, RTEModel, RTESolver, RTEOptimizer, RTEIndex>
{

public:

	RelativeTranslationEstimator(const float &errTh = 0) : Estimator<RTEData, RTEMinimalSample, RTENonMinimalSample, RTEModel, RTESolver, RTEOptimizer, RTEIndex>(errTh) {}

	virtual const ushort MinimalSampleSize() const
	{
		return 2;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		return epsilon * epsilon;
	}

	virtual void DrawMinimalSample(const RTEData &data, RTEMinimalSample &sample) const
	{
		const ushort N = data.Size();
		ushort i0 = Random::Generate(N);
		sample.Set(data.Xs(), data.xs(), i0, 0);
		ushort i1 = Random::Generate(N);
		while(i1 == i0)
			i1 = Random::Generate(N);
		sample.Set(data.Xs(), data.xs(), i1, 1);
		sample.R() = data.R();
		sample.pxs() = data.pxs();
	}

	virtual void DrawMinimalSampleOrdered(const RTEData &data, const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne, RTEMinimalSample &sample) const
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
		sample.R() = data.R();
		sample.pxs() = data.pxs();
	}

	virtual void GenerateModels(RTEMinimalSample &sample, AlignedVector<RTEModel> &models)
	{
		models.Resize(1);
		m_work.Resize(28);
		if(m_solver.Run(sample, models[0], m_work) && models[0].ToRelativePose(*sample.pxs(), sample.R(), m_T, m_E, m_sccRatioTh, m_work.Data()))
			m_T.GetTranslation(models[0]);
		else
			models.Resize(0);
	}

	virtual void GenerateModels(RTENonMinimalSample &sample, AlignedVector<RTEModel> &models)
	{
		models.Resize(1);
		m_work.Resize(28);
		if(m_solver.Run(sample, models[0], m_work) && models[0].ToRelativePose(*sample.pxs(), sample.R(), m_T, m_E, m_sccRatioTh, m_work.Data()))
			m_T.GetTranslation(models[0]);
		else
			models.Resize(0);
	}

	virtual void VerifyModel(const RTEData &data, const RTEModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		fitErr = 0;

		m_work.Resize(1);
		float *work = (float *) m_work.Data();
		const ushort N = data.Size();
		for(ushort i = 0; i < N; ++i)
		{
			if(inlierMarks[i])
				fitErr += model.ComputeSuqaredError(data.X(i), data.x(i), work);
		}
	}

	virtual void VerifyModel(const RTEData &data, const RTEModel &model, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;
		float errSq;
		m_work.Resize(1);
		float *work = (float *) m_work.Data();
		const ushort N = data.Size();
		for(ushort i = 0; i < N; ++i)
		{
			if((errSq = model.ComputeSuqaredError(data.X(i), data.x(i), work)) < m_ransacErrorThreshold)
			{
				inliers.push_back(i);
				fitErr += errSq;
			}
		}
	}

	virtual void VerifyModel(const RTEData &data, const RTEModel &model, const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;
		float errSq;
		m_work.Resize(1);
		float *work = (float *) m_work.Data();
		const ushort N = data.Size();
		for(ushort i = 0; i < N; ++i)
		{
			if((errSq = model.ComputeSuqaredError(data.X(i), data.x(i), work)) < m_ransacErrorThreshold)
			{
				inliers.push_back(i);
				fitErr += errSq;
			}
			else if(inlierMarksFix[i])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
		}
	}

	virtual void OptimizeModel(RTEData &data, RTEModel &model, const ubyte verbose = 0)
	{
	}

public:

		float m_sccRatioTh;

protected:

	RigidTransformation3D m_T;
	EssentialMatrix m_E;

};

#endif