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

#ifndef _SIMILARITY_TRANSFORMATION_ESTIMATOR_H_
#define _SIMILARITY_TRANSFORMATION_ESTIMATOR_H_

#include "Estimation/Estimator.h"
#include "SimilarityTransformationEstimatorData.h"
#include "Optimization/Optimizer.h"
#include "SimilarityTransformationSolver.h"

typedef SimilarityTransformationEstimatorData3D													STE3DData;
typedef FourMatches3D																			STE3DMinimalSample;
typedef SimilarityTransformationEstimatorData3D													STE3DNonMinimalSample;
typedef SimilarityTransformation3D																STE3DModel;
typedef SimilarityTransformationSolver															STE3DSolver;
typedef OptimizerTemplate<SimilarityTransformation3D, LA::AlignedVector7f, LA::AlignedMatrix7f>	STE3DOptimizer;
typedef uint																					STE3DIndex;

class SimilarityTransformationEstimator3D : public Estimator<STE3DData, STE3DMinimalSample, STE3DNonMinimalSample, STE3DModel, STE3DSolver, STE3DOptimizer, STE3DIndex>
{

public:

	SimilarityTransformationEstimator3D(const float &errTh = 0) : Estimator<STE3DData, STE3DMinimalSample, STE3DNonMinimalSample, STE3DModel, STE3DSolver, STE3DOptimizer, STE3DIndex>(errTh) {}

	virtual const uint MinimalSampleSize() const
	{
		return 4;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		//return epsilon * epsilon * epsilon;
		float tmp = epsilon * epsilon;
		return tmp * tmp;
	}

	virtual void DrawMinimalSample(const STE3DData &data, STE3DMinimalSample &sample) const
	{
		const uint N = data.Size();
		uint i0 = Random::Generate(N);
		sample.Set(0, data.X1(i0), data.X2(i0));

		uint i1 = Random::Generate(N);
		while(i1 == i0)
			i1 = Random::Generate(N);
		sample.Set(1, data.X1(i1), data.X2(i1));

		uint i2 = Random::Generate(N);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(N);
		sample.Set(2, data.X1(i2), data.X2(i2));

		uint i3 = Random::Generate(N);
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = Random::Generate(N);
		sample.Set(3, data.X1(i3), data.X2(i3));
	}

	virtual void DrawMinimalSampleOrdered(const STE3DData &data, const std::vector<uint> &orders, const uint &n, const bool &sampleLastOne, STE3DMinimalSample &sample) const
	{
		uint i0 = Random::Generate(n), i = orders[i0];
		sample.Set(0, data.X1(i), data.X2(i));

		uint i1 = Random::Generate(n);
		while(i1 == i0)
			i1 = Random::Generate(n);
		i = orders[i1];
		sample.Set(1, data.X1(i), data.X2(i));

		uint i2 = Random::Generate(n);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(n);
		i = orders[i2];
		sample.Set(2, data.X1(i), data.X2(i));

		uint i3 = n - 1;
		if(!sampleLastOne || i3 == i0 || i3 == i1 || i3 == i2)
		{
			i3 = Random::Generate(n);
			while(i3 == i0 || i3 == i1 || i3 == i2)
				i3 = Random::Generate(n);
		}
		i = orders[i3];
		sample.Set(3, data.X1(i), data.X2(i));
	}

	virtual void GenerateModels(STE3DMinimalSample &sample, AlignedVector<STE3DModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0], m_work))
			models.Resize(0);
	}

	virtual void GenerateModels(STE3DNonMinimalSample &sample, AlignedVector<STE3DModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample.X1s(), sample.X2s(), models[0], m_work))
			models.Resize(0);
	}

	virtual void VerifyModel(const STE3DData &data, const STE3DModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		fitErr = 0;

		m_work.Resize(1);
		const uint N = data.Size();
		for(uint i = 0; i < N; ++i)
		{
			if(inlierMarks[i])
				fitErr += model.ComputeTransformationSquaredError(data.X1(i), data.X2(i), m_work[0]);
		}
	}

	virtual void VerifyModel(const STE3DData &data, const STE3DModel &model, std::vector<uint> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		float errSq;
		m_work.Resize(1);

		const uint N = data.Size();
		for(uint i = 0; i < N; ++i)
		{
			errSq = model.ComputeTransformationSquaredError(data.X1(i), data.X2(i), m_work[0]);
			if(errSq < m_ransacErrorThreshold)
			{
				inliers.push_back(i);
				fitErr += errSq;
			}
		}
	}

	virtual void VerifyModel(const STE3DData &data, const STE3DModel &model, const std::vector<bool> &inlierMarksFix, std::vector<uint> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		float errSq;
		m_work.Resize(1);

		const uint N = data.Size();
		for(uint i = 0; i < N; ++i)
		{
			errSq = model.ComputeTransformationSquaredError(data.X1(i), data.X2(i), m_work[0]);
			if(errSq < m_ransacErrorThreshold)
			{
				inliers.push_back(i);
				fitErr += errSq;
			}
			else if(inlierMarksFix[i])
			{
				inliers.resize(0);
				return;
			}
		}
	}

	virtual void OptimizeModel(STE3DData &data, STE3DModel &model, const ubyte verbose = 0)
	{
		m_optimizer.m_lmMaxNumIters = m_optimizeMaxNumIters;
		m_optimizer.Run(data, model, verbose >= 2 ? verbose - 2 : 0);
	}
};

typedef SimilarityTransformationEstimatorData2D													STE2DData;
typedef FourMatches3D																			STE2DMinimalSample;
typedef SimilarityTransformationEstimatorData2D													STE2DNonMinimalSample;
typedef SimilarityTransformation3D																STE2DModel;
typedef SimilarityTransformationSolver															STE2DSolver;
typedef OptimizerTemplate<SimilarityTransformation3D, LA::AlignedVector7f, LA::AlignedMatrix7f>	STE2DOptimizer;
typedef uint																					STE2DIndex;

class SimilarityTransformationEstimator2D : public Estimator<STE2DData, STE2DMinimalSample, STE2DNonMinimalSample, STE2DModel, STE2DSolver, STE2DOptimizer, STE2DIndex>
{

public:

	SimilarityTransformationEstimator2D(const float &errTh = 0) : Estimator<STE2DData, STE2DMinimalSample, STE2DNonMinimalSample, STE2DModel, STE2DSolver, STE2DOptimizer, STE2DIndex>(errTh) {}

	virtual const uint MinimalSampleSize() const
	{
		return 4;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		//return epsilon * epsilon * epsilon;
		float tmp = epsilon * epsilon;
		return tmp * tmp;
	}

	virtual void DrawMinimalSample(const STE2DData &data, STE2DMinimalSample &sample) const
	{
		const uint N = data.Size();
		uint i0 = Random::Generate(N);
		sample.Set(0, data.X1(i0), data.X2(i0));

		uint i1 = Random::Generate(N);
		while(i1 == i0)
			i1 = Random::Generate(N);
		sample.Set(1, data.X1(i1), data.X2(i1));

		uint i2 = Random::Generate(N);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(N);
		sample.Set(2, data.X1(i2), data.X2(i2));

		uint i3 = Random::Generate(N);
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = Random::Generate(N);
		sample.Set(3, data.X1(i3), data.X2(i3));
	}

	virtual void DrawMinimalSampleOrdered(const STE2DData &data, const std::vector<uint> &orders, const uint &n, const bool &sampleLastOne, STE2DMinimalSample &sample) const
	{
		uint i0 = Random::Generate(n), i = orders[i0];
		sample.Set(0, data.X1(i), data.X2(i));

		uint i1 = Random::Generate(n);
		while(i1 == i0)
			i1 = Random::Generate(n);
		i = orders[i1];
		sample.Set(1, data.X1(i), data.X2(i));

		uint i2 = Random::Generate(n);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(n);
		i = orders[i2];
		sample.Set(2, data.X1(i), data.X2(i));

		uint i3 = n - 1;
		if(!sampleLastOne || i3 == i0 || i3 == i1 || i3 == i2)
		{
			i3 = Random::Generate(n);
			while(i3 == i0 || i3 == i1 || i3 == i2)
				i3 = Random::Generate(n);
		}
		i = orders[i3];
		sample.Set(3, data.X1(i), data.X2(i));
	}

	virtual void GenerateModels(STE2DMinimalSample &sample, AlignedVector<STE2DModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0], m_work))
			models.Resize(0);
	}

	virtual void GenerateModels(STE2DNonMinimalSample &sample, AlignedVector<STE2DModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample.X1s(), sample.X2s(), models[0], m_work))
			models.Resize(0);
	}

	virtual void VerifyModel(const STE2DData &data, const STE2DModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		fitErr = 0;

		float errSqAvg;
		Point3D X2;
		Point2D e;

		const uint nPts = data.Size();
		uint iMea = 0, i;
		if(data.AreWeightsValid())
		{
			for(uint iPt = 0; iPt < nPts; ++iPt)
			{
				if(!inlierMarks[iPt])
					continue;
				model.Apply(data.X1(iPt), X2);
				errSqAvg = 0;
				iMea = data.GetPointMeasurementIndex(iPt);
				const uint nMeas = data.GetPointMeasurementIndex(iPt + 1) - iMea;
				for(i = 0; i < nMeas; ++i, ++iMea)
					errSqAvg += data.pC2(iMea)->ComputeProjectionSquaredError(X2, data.x2(iMea), e) * data.GetSquaredWeight(iMea);
				errSqAvg /= nMeas;
				fitErr += errSqAvg;
			}
		}
		else
		{
			for(uint iPt = 0; iPt < nPts; ++iPt)
			{
				if(!inlierMarks[iPt])
					continue;
				model.Apply(data.X1(iPt), X2);
				errSqAvg = 0;
				iMea = data.GetPointMeasurementIndex(iPt);
				const uint nMeas = data.GetPointMeasurementIndex(iPt + 1) - iMea;
				for(i = 0; i < nMeas; ++i, ++iMea)
					errSqAvg += data.pC2(iMea)->ComputeProjectionSquaredError(X2, data.x2(iMea), e);
				errSqAvg /= nMeas;
				fitErr += errSqAvg;
			}
		}
	}

	virtual void VerifyModel(const STE2DData &data, const STE2DModel &model, std::vector<uint> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		float errSqAvg;
		Point3D X2;
		Point2D e;

		const uint nPts = data.Size();
		uint iMea = 0, i;
		if(data.AreWeightsValid())
		{
			for(uint iPt = 0; iPt < nPts; ++iPt)
			{
				model.Apply(data.X1(iPt), X2);
				errSqAvg = 0;
				iMea = data.GetPointMeasurementIndex(iPt);
				const uint nMeas = data.GetPointMeasurementIndex(iPt + 1) - iMea;
				for(i = 0; i < nMeas; ++i, ++iMea)
					errSqAvg += data.pC2(iMea)->ComputeProjectionSquaredError(X2, data.x2(iMea), e) * data.GetSquaredWeight(iMea);
				errSqAvg /= nMeas;
				if(errSqAvg < m_ransacErrorThreshold)
				{
					inliers.push_back(iPt);
					fitErr += errSqAvg;
				}
			}
		}
		else
		{
			for(uint iPt = 0; iPt < nPts; ++iPt)
			{
				model.Apply(data.X1(iPt), X2);
				errSqAvg = 0;
				iMea = data.GetPointMeasurementIndex(iPt);
				const uint nMeas = data.GetPointMeasurementIndex(iPt + 1) - iMea;
				for(i = 0; i < nMeas; ++i, ++iMea)
					errSqAvg += data.pC2(iMea)->ComputeProjectionSquaredError(X2, data.x2(iMea), e);
				errSqAvg /= nMeas;
				if(errSqAvg < m_ransacErrorThreshold)
				{
					inliers.push_back(iPt);
					fitErr += errSqAvg;
				}
			}
		}
	}

	virtual void VerifyModel(const STE2DData &data, const STE2DModel &model, const std::vector<bool> &inlierMarksFix, std::vector<uint> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		float errSqAvg;
		Point3D X2;
		Point2D e;

		const uint nPts = data.Size();
		uint iMea = 0, i;
		if(data.AreWeightsValid())
		{
			for(uint iPt = 0; iPt < nPts; ++iPt)
			{
				model.Apply(data.X1(iPt), X2);
				errSqAvg = 0;
				iMea = data.GetPointMeasurementIndex(iPt);
				const uint nMeas = data.GetPointMeasurementIndex(iPt + 1) - iMea;
				for(i = 0; i < nMeas; ++i, ++iMea)
					errSqAvg += data.pC2(iMea)->ComputeProjectionSquaredError(X2, data.x2(iMea), e) * data.GetSquaredWeight(iMea);
				errSqAvg /= nMeas;
				if(errSqAvg < m_ransacErrorThreshold)
				{
					inliers.push_back(iPt);
					fitErr += errSqAvg;
				}
				else if(inlierMarksFix[iPt])
				{
					inliers.resize(0);
					return;
				}
			}
		}
		else
		{
			for(uint iPt = 0; iPt < nPts; ++iPt)
			{
				model.Apply(data.X1(iPt), X2);
				errSqAvg = 0;
				iMea = data.GetPointMeasurementIndex(iPt);
				const uint nMeas = data.GetPointMeasurementIndex(iPt + 1) - iMea;
				for(i = 0; i < nMeas; ++i, ++iMea)
					errSqAvg += data.pC2(iMea)->ComputeProjectionSquaredError(X2, data.x2(iMea), e);
				errSqAvg /= nMeas;
				if(errSqAvg < m_ransacErrorThreshold)
				{
					inliers.push_back(iPt);
					fitErr += errSqAvg;
				}
				else if(inlierMarksFix[iPt])
				{
					inliers.resize(0);
					return;
				}
			}
		}
	}

	virtual void OptimizeModel(STE2DData &data, STE2DModel &model, const ubyte verbose = 0)
	{
		m_optimizer.m_lmMaxNumIters = m_optimizeMaxNumIters;
		m_optimizer.Run(data, model, verbose >= 2 ? verbose - 2 : 0);
	}
};

#endif