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

#ifndef _PLANE_ESTIMATOR_H_
#define _PLANE_ESTIMATOR_H_

#include "Estimation/Estimator.h"
#include "Plane.h"
#include "PlaneEstimatorData.h"
#include "PlaneEstimatorMinimalSample.h"
#include "PlaneSolver.h"
#include "Optimization/Optimizer.h"

typedef PlaneEstimatorData2D												PEData;
typedef PlaneEstimatorMinimalSample											PEMinimalSample;
typedef PlaneEstimatorData2D												PENonMinimalSample;
typedef Plane																PEModel;
typedef PlaneSolver															PESolver;
typedef OptimizerTemplate<Plane, LA::AlignedVector4f, LA::AlignedMatrix4f>	PEOptimizer;
typedef uint																PEIndex;

class PlaneEstimator : public Estimator<PEData, PEMinimalSample, PENonMinimalSample, PEModel, PESolver, PEOptimizer, PEIndex>
{

public:

	PlaneEstimator(const float &errTh = 0) : Estimator<PEData, PEMinimalSample, PENonMinimalSample, PEModel, PESolver, PEOptimizer, PEIndex>(errTh) {}
	
	virtual const uint MinimalSampleSize() const
	{
		return 3;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		return epsilon * epsilon * epsilon;
	}

	virtual void DrawMinimalSample(const PEData &data, PEMinimalSample &sample) const
	{
		const uint N = data.Size();
		uint i0 = Random::Generate(N);
		sample.X(0) = data.X(i0);

		uint i1 = Random::Generate(N);
		while(i1 == i0)
			i1 = Random::Generate(N);
		sample.X(1) = data.X(i1);

		uint i2 = Random::Generate(N);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(N);
		sample.X(2) = data.X(i2);
	}

	virtual void DrawMinimalSampleOrdered(const PEData &data, const std::vector<uint> &orders, const uint &n, const bool &sampleLastOne, PEMinimalSample &sample) const
	{
		uint i0 = Random::Generate(n), i = orders[i0];
		sample.X(0) = data.X(i);

		uint i1 = Random::Generate(n);
		while(i1 == i0)
			i1 = Random::Generate(n);
		i = orders[i1];
		sample.X(1) = data.X(i);

		uint i2 = n - 1;
		if(!sampleLastOne || i2 == i0 || i2 == i1)
		{
			i2 = Random::Generate(n);
			while(i2 == i0 || i2 == i1)
				i2 = Random::Generate(n);
		}
		i = orders[i2];
		sample.X(2) = data.X(i);
	}

	virtual void GenerateModels(PEMinimalSample &sample, AlignedVector<PEModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0], m_work))
			models.Resize(0);
	}

	virtual void GenerateModels(PENonMinimalSample &sample, AlignedVector<PEModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0], m_work))
			models.Resize(0);
	}

	virtual void VerifyModel(const PEData &data, const PEModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		fitErr = 0;

		const uint nPts = data.GetPointsNumber();
		for(uint iPt = 0; iPt < nPts; ++iPt)
		{
			if(inlierMarks[iPt])
				fitErr += fabs(model.ComputeDistance(data.X(iPt)));
		}
	}

	virtual void VerifyModel(const PEData &data, const PEModel &model, std::vector<uint> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		const uint nPts = data.GetPointsNumber();
		//if(g_ransacErrorThreshold3D == 0.0f)
		//{
			float sse, d;
			Point3D Xp;
			for(uint iPt = 0; iPt < nPts; ++iPt)
			{
				if((sse = data.ComputeSSE2D(iPt, model, d, Xp)) < m_ransacErrorThreshold * data.GetPointMeasurementsNumber(iPt) && fabs(d) < g_ransacErrorThreshold3D)
				{
					inliers.push_back(iPt);
					//fitErr += sse;
					fitErr += fabs(d);
				}
			}
		//}
		//else
		//{
		//	float e;
		//	for(uint iPt = 0; iPt < nPts; ++iPt)
		//	{
		//		if((e = fabs(model.ComputeDistance(data.X(iPt)))) < g_ransacErrorThreshold3D)
		//		{
		//			inliers.push_back(iPt);
		//			fitErr += e;
		//		}
		//	}
		//}
	}

	virtual void VerifyModel(const PEData &data, const PEModel &model, const std::vector<bool> &inlierMarksFix, std::vector<uint> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		const uint nPts = data.GetPointsNumber();
		//if(g_ransacErrorThreshold3D == 0.0f)
		//{
			float sse, d;
			Point3D Xp;
			for(uint iPt = 0; iPt < nPts; ++iPt)
			{
				if((sse = data.ComputeSSE2D(iPt, model, d, Xp)) < m_ransacErrorThreshold * data.GetPointMeasurementsNumber(iPt) && fabs(d) < g_ransacErrorThreshold3D)
				{
					inliers.push_back(iPt);
					//fitErr += sse;
					fitErr += fabs(d);
				}
				else if(inlierMarksFix[iPt])
				{
					inliers.resize(0);
					fitErr = 0;
					return;
				}
			}
		//}
		//else
		//{
		//	float e;
		//	for(uint iPt = 0; iPt < nPts; ++iPt)
		//	{
		//		if((e = fabs(model.ComputeDistance(data.X(iPt)))) < g_ransacErrorThreshold3D)
		//		{
		//			inliers.push_back(iPt);
		//			fitErr += e;
		//		}
		//		else if(inlierMarksFix[iPt])
		//		{
		//			inliers.resize(0);
		//			fitErr = 0;
		//			return;
		//		}
		//	}
		//}
	}

	virtual void OptimizeModel(PEData &data, PEModel &model, const ubyte verbose = 0)
	{
		m_optimizer.m_lmMaxNumIters = m_optimizeMaxNumIters;
		//sample.m_measureIn3D = g_ransacErrorThreshold3D != 0.0f;
		m_optimizer.Run(data, model, verbose >= 2 ? verbose - 2 : 0);
	}

public:

	static float g_ransacErrorThreshold3D;

};

#endif