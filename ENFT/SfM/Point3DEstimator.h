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

#ifndef _POINT_3D_ESTIMATOR_H_
#define _POINT_3D_ESTIMATOR_H_

#include "Estimation/Estimator.h"
#include "Point3DEstimatorData.h"
#include "Point3DEstimatorMinimalSample.h"
#include "Point3DSolver.h"
#include "Optimization/Optimizer.h"
#include "SfM/Match.h"

typedef Point3DEstimatorData													P3DEData;
typedef Point3DEstimatorMinimalSample											P3DEMinimalSample;
typedef Point3DEstimatorData													P3DENonMinimalSample;
typedef Point3D																	P3DEModel;
typedef Point3DSolver															P3DESolver;
typedef OptimizerTemplate<Point3D, LA::AlignedVector3f, LA::AlignedMatrix3f>	P3DEOptimizer;
typedef ushort																	P3DEIndex;

class Point3DEstimator : public Estimator<P3DEData, P3DEMinimalSample, P3DENonMinimalSample, P3DEModel, P3DESolver, P3DEOptimizer, P3DEIndex>
{

public:

	Point3DEstimator(const float &errTh = 0) : 
	  Estimator<P3DEData, P3DEMinimalSample, P3DENonMinimalSample, P3DEModel, P3DESolver, P3DEOptimizer, P3DEIndex>(errTh) {}
	
	inline void SetMetric() { m_metric = true; }
	inline void SetProjective() { m_metric = false; }

	virtual const ushort MinimalSampleSize() const
	{
		return 2;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		return epsilon * epsilon;
	}

	virtual void DrawMinimalSample(const P3DEData &data, P3DEMinimalSample &sample) const
	{
		const ushort N = data.Size();
		ushort i0 = Random::Generate(N);
		sample.SetFirstCamera(data.GetCamera(i0));
		sample.SetFirstMeasurement(data.x(i0));

		ushort i1 = Random::Generate(N);
		while(i1 == i0)
			i1 = Random::Generate(N);
		sample.SetSecondCamera(data.GetCamera(i1));
		sample.SetSecondMeasurement(data.x(i1));
	}

	virtual void DrawMinimalSampleOrdered(const P3DEData &data, const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne, 
										  P3DEMinimalSample &sample) const
	{
		ushort i0 = Random::Generate(n), i = orders[i0];
		sample.SetFirstCamera(data.GetCamera(i));
		sample.SetFirstMeasurement(data.x(i));

		ushort i1 = n - 1;
		if(!sampleLastOne || i1 == i0)
		{
			i1 = Random::Generate(n);
			while(i1 == i0)
				i1 = Random::Generate(n);
		}
		i = orders[i1];
		sample.SetSecondCamera(data.GetCamera(i));
		sample.SetSecondMeasurement(data.x(i));
	}

	virtual void GenerateModels(P3DEMinimalSample &sample, AlignedVector<P3DEModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0], m_work, m_metric))
			models.Resize(0);
	}

	virtual void GenerateModels(P3DENonMinimalSample &sample, AlignedVector<P3DEModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0], m_work, m_metric))
			models.Resize(0);
	}

	virtual void VerifyModel(const P3DEData &data, const P3DEModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		fitErr = 0;

		Point2D e;
		const ushort N = data.Size();
		if(data.AreWeightsValid())
		{
			for(ushort i = 0; i < N; ++i)
			{
				if(inlierMarks[i])
					fitErr += data.GetCamera(i).ComputeProjectionSquaredError(model, data.x(i), e) * data.GetSquaredWeight(i);
			}
		}
		else
		{
			for(ushort i = 0; i < N; ++i)
			{
				if(inlierMarks[i])
					fitErr += data.GetCamera(i).ComputeProjectionSquaredError(model, data.x(i), e);
			}
		}
	}

	virtual void VerifyModel(const P3DEData &data, const P3DEModel &model, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		m_work.Resize(2);
		ENFT_SSE::__m128 &errSq = m_work[0], &work = m_work[1];
		const ushort N = data.Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
		Point2D Zc2;

		if(data.AreWeightsValid())
		{
			for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
			{
				Camera::ComputeProjectionError2(data.GetCamera(ix2), data.GetCamera(ix2p1), model, model, *px2, Zc2, errSq, work);
				errSq = ENFT_SSE::_mm_mul_ps(errSq, errSq);
				if((!m_metric || Zc2.v0() > 0)
				&& (errSq.m128_f32[0] = (errSq.m128_f32[0] + errSq.m128_f32[1]) * data.GetSquaredWeight(ix2)) < m_ransacErrorThreshold)
				{
					inliers.push_back(ix2);
					fitErr += errSq.m128_f32[0];
				}
				if((!m_metric || Zc2.v1() > 0)
				&& (errSq.m128_f32[2] = (errSq.m128_f32[2] + errSq.m128_f32[3]) * data.GetSquaredWeight(ix2p1)) < m_ransacErrorThreshold)
				{
					inliers.push_back(ix2p1);
					fitErr += errSq.m128_f32[2];
				}
			}
			if(_N != N)
			{
				Point2D e;
				if((!m_metric || data.GetCamera(_N).ComputeDepth(model) > 0)
				&& (errSq.m128_f32[0] = data.GetCamera(_N).ComputeProjectionSquaredError(model, data.x(_N), e) * data.GetSquaredWeight(_N)) 
				< m_ransacErrorThreshold)
				{
					inliers.push_back(_N);
					fitErr += errSq.m128_f32[0];
				}
			}
		}
		else
		{
			for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
			{
				Camera::ComputeProjectionError2(data.GetCamera(ix2), data.GetCamera(ix2p1), model, model, *px2, Zc2, errSq, work);
				errSq = ENFT_SSE::_mm_mul_ps(errSq, errSq);
				if((!m_metric || Zc2.v0() > 0)
				&& (errSq.m128_f32[0] = errSq.m128_f32[0] + errSq.m128_f32[1]) < m_ransacErrorThreshold)
				{
					inliers.push_back(ix2);
					fitErr += errSq.m128_f32[0];
				}
				if((!m_metric || Zc2.v1() > 0)
				&& (errSq.m128_f32[2] = errSq.m128_f32[2] + errSq.m128_f32[3]) < m_ransacErrorThreshold)
				{
					inliers.push_back(ix2p1);
					fitErr += errSq.m128_f32[2];
				}
			}
			if(_N != N)
			{
				Point2D e;
				if((!m_metric || data.GetCamera(_N).ComputeDepth(model) > 0)
				&& (errSq.m128_f32[0] = data.GetCamera(_N).ComputeProjectionSquaredError(model, data.x(_N), e)) < m_ransacErrorThreshold)
				{
					inliers.push_back(_N);
					fitErr += errSq.m128_f32[0];
				}
			}
		}
	}

	virtual void VerifyModel(const P3DEData &data, const P3DEModel &model, const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		m_work.Resize(2);
		ENFT_SSE::__m128 &errSq = m_work[0], &work = m_work[1];
		const ushort N = data.Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
		Point2D Zc2;

		if(data.AreWeightsValid())
		{
			for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
			{
				Camera::ComputeProjectionError2(data.GetCamera(ix2), data.GetCamera(ix2p1), model, model, *px2, Zc2, errSq, work);
				errSq = ENFT_SSE::_mm_mul_ps(errSq, errSq);
				if((!m_metric || Zc2.v0() > 0)
				&& (errSq.m128_f32[0] = (errSq.m128_f32[0] + errSq.m128_f32[1]) * data.GetSquaredWeight(ix2)) < m_ransacErrorThreshold)
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
				if((!m_metric || Zc2.v1() > 0)
				&& (errSq.m128_f32[2] = (errSq.m128_f32[2] + errSq.m128_f32[3]) * data.GetSquaredWeight(ix2p1)) < m_ransacErrorThreshold)
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
				Point2D e;
				if((!m_metric || data.GetCamera(_N).ComputeDepth(model) > 0)
				&& (errSq.m128_f32[0] = data.GetCamera(_N).ComputeProjectionSquaredError(model, data.x(_N), e) * data.GetSquaredWeight(_N)) 
				< m_ransacErrorThreshold)
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
			if(!inliers.empty())
				fitErr = fitErr / inliers.size() + N - (inliers.back() - inliers.front());
		}
		else
		{
			for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
			{
				Camera::ComputeProjectionError2(data.GetCamera(ix2), data.GetCamera(ix2p1), model, model, *px2, Zc2, errSq, work);
				errSq = ENFT_SSE::_mm_mul_ps(errSq, errSq);
				if((!m_metric || Zc2.v0() > 0)
				&& (errSq.m128_f32[0] = errSq.m128_f32[0] + errSq.m128_f32[1]) < m_ransacErrorThreshold)
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
				if((!m_metric || Zc2.v1() > 0)
				&& (errSq.m128_f32[2] = errSq.m128_f32[2] + errSq.m128_f32[3]) < m_ransacErrorThreshold)
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
				Point2D e;
				if((!m_metric || data.GetCamera(_N).ComputeDepth(model) > 0)
				&& (errSq.m128_f32[0] = data.GetCamera(_N).ComputeProjectionSquaredError(model, data.x(_N), e)) < m_ransacErrorThreshold)
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
	}

	virtual bool SolveModel(const P3DENonMinimalSample &sample, P3DEModel &model)
	{
		return m_solver.Run(sample, model, m_work, m_metric);
	}

	virtual void OptimizeModel(P3DEData &data, P3DEModel &model, const ubyte verbose = 0)
	{
		m_optimizer.m_lmMaxNumIters  = m_optimizeMaxNumIters;
		m_optimizer.Run(data, model, verbose >= 2 ? verbose - 2 : 0);
	}

	inline bool Triangulate(/*const */Point3DEstimatorData &data, Point3D &X)
	{
		if(!m_solver.Run(data, X, m_work, m_metric) || m_ransacErrorThreshold != 0 && CountInliers(data, X) != data.Size())
			return false;
		OptimizeModel(data, X);
		return true;
	}

	inline bool Triangulate(/*const */Point3DEstimatorData &data, Point3D &X, std::vector<ushort> &inliers, const ushort nInliersTh = 2)
	{
		if(!m_solver.Run(data, X, m_work, m_metric))
			return false;
		//OptimizeModel(data, X);
		//double fitErr;
		//VerifyModel(data, X, inliers, fitErr);
		//return ushort(inliers.size()) >= nInliersTh;
		double fitErr;
		VerifyModel(data, X, inliers, fitErr);
		if(ushort(inliers.size()) < nInliersTh)
			return false;
		data.GetSubset(inliers, m_nonMinimalSample);
		OptimizeModel(m_nonMinimalSample, X);
		VerifyModel(data, X, inliers, fitErr);
		return true;
	}

	//inline bool Triangulate(const Point3DEstimatorData &data, Point3D &X, std::vector<ushort> &inliers, const std::vector<Match<ushort> > &triPairs, 
	//						const ushort nInliersTh = 2)
	//{
	//	ushort i1, i2, nInliers = 0;
	//	double fitErr;
	//	const uint nPairs = uint(triPairs.size());
	//	for(uint i = 0; i < nPairs; ++i)
	//	{
	//		triPairs[i].Get(i1, i2);
	//		m_minimalSample.SetFirstCamera(data.GetCamera(i1));			m_minimalSample.SetFirstMeasurement(data.x(i1));
	//		m_minimalSample.SetSecondCamera(data.GetCamera(i2));		m_minimalSample.SetSecondMeasurement(data.x(i2));
	//		if(!m_solver.Run(m_minimalSample, X, m_work, m_metric))
	//			continue;
	//		VerifyModel(data, X, inliers, fitErr);
	//		if((nInliers = ushort(inliers.size())) >= nInliersTh)
	//			break;
	//	}
	//	if(nInliers < nInliersTh)
	//		return false;
	//	data.GetSubset(inliers, m_nonMinimalSample);
	//	OptimizeModel(m_nonMinimalSample, X);
	//	VerifyModel(data, X, inliers, fitErr);
	//	return true;
	//}

protected:

	bool m_metric;

};

#endif