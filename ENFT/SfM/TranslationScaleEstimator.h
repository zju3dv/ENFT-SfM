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

#ifndef _TRANSLATION_SCALE_ESTIMATOR_H_
#define _TRANSLATION_SCALE_ESTIMATOR_H_

#include "Estimation/Estimator.h"
#include "TranslationScaleEstimatorData.h"
#include "TranslationScaleSolver.h"
#include "TranslationScaleOptimizer.h"

typedef TranslationScaleEstimatorData	TSEData;
typedef LA::Vector2f					TSEMinimalSample;
typedef TranslationScaleEstimatorData	TSENonMinimalSample;
typedef float							TSEModel;
typedef TranslationScaleSolver			TSESolver;
typedef TranslationScaleOptimizer		TSEOptimizer;
typedef ushort							TSEIndex;

class TranslationScaleEstimator : public Estimator<TSEData, TSEMinimalSample, TSENonMinimalSample, TSEModel, TSESolver, TSEOptimizer, TSEIndex>
{

public:

	TranslationScaleEstimator(const float &errTh = 0) : Estimator<TSEData, TSEMinimalSample, TSENonMinimalSample, TSEModel, TSESolver, TSEOptimizer, TSEIndex>(errTh) {}

	virtual const ushort MinimalSampleSize() const
	{
		return 1;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		return epsilon;
	}

	virtual void DrawMinimalSample(const TSEData &data, TSEMinimalSample &sample) const
	{
		const ushort i = ushort(rand()) % data.Size();
		sample = data.GetScale(i);
	}

	virtual void DrawMinimalSampleOrdered(const TSEData &data, const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne, TSEMinimalSample &sample) const
	{
		const ushort i = sampleLastOne ? n - 1 : ushort(rand() % data.Size());
		sample = data.GetScale(i);
	}

	virtual void GenerateModels(TSEMinimalSample &sample, AlignedVector<TSEModel> &models)
	{
		models.EnlargeCapacity(2);
		models.Resize(0);
		//if(sample.v0() > 0)
		//	models.PushBack(sample.v0());
		//if(sample.v1() > 0)
		//	models.PushBack(sample.v1());
		if(sample.v0() != 0)
			models.PushBack(sample.v0());
		if(sample.v1() != 0)
			models.PushBack(sample.v1());
	}

	virtual void GenerateModels(TSENonMinimalSample &sample, AlignedVector<TSEModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0]))
			models.Resize(0);
	}

	virtual void VerifyModel(const TSEData &data, const TSEModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		fitErr = 0;

		m_work.Resize(4);
		ENFT_SSE::__m128 &st = m_work[0], &RX1pst = m_work[1], &RX2pst = m_work[2], &e = m_work[3], &sqErr = m_work[3], &work1 = m_work[3];
		st = ENFT_SSE::_mm_mul_ps(data.GetTranslation().v012x(), _mm_set1_ps(model));
		const ushort N = data.Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
		{
			RX1pst = ENFT_SSE::_mm_add_ps(data.X(ix2).XYZx(), st);		work1.m128_f32[0] = work1.m128_f32[1] = 1 / RX1pst.m128_f32[2];
			RX2pst = ENFT_SSE::_mm_add_ps(data.X(ix2p1).XYZx(), st);		work1.m128_f32[2] = work1.m128_f32[3] = 1 / RX2pst.m128_f32[2];
			e = ENFT_SSE::_mm_sub_ps(*px2, ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(RX1pst, RX2pst), work1));
			sqErr = ENFT_SSE::_mm_mul_ps(e, e);
			if(inlierMarks[ix2])
				fitErr += sqErr.m128_f32[0] + sqErr.m128_f32[1];
			if(inlierMarks[ix2p1])
				fitErr += sqErr.m128_f32[2] + sqErr.m128_f32[3];
		}
		if(_N != N)
		{
			RX1pst = ENFT_SSE::_mm_add_ps(data.X(_N).XYZx(), st);
			work1.m128_f32[0] = 1 / RX1pst.m128_f32[2];
			const Point2D x(RX1pst.m128_f32[0] * work1.m128_f32[0], RX1pst.m128_f32[1] * work1.m128_f32[0]);
			if(inlierMarks[_N])
				fitErr += data.x(_N).SquaredDistance(x);
		}
	}

	virtual void VerifyModel(const TSEData &data, const TSEModel &model, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		m_work.Resize(4);
		ENFT_SSE::__m128 &st = m_work[0], &RX1pst = m_work[1], &RX2pst = m_work[2], &e = m_work[3], &sqErr = m_work[3], &work1 = m_work[3];
		st = ENFT_SSE::_mm_mul_ps(data.GetTranslation().v012x(), _mm_set1_ps(model));
		const ushort N = data.Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
		{
			RX1pst = ENFT_SSE::_mm_add_ps(data.X(ix2).XYZx(), st);		work1.m128_f32[0] = work1.m128_f32[1] = 1 / RX1pst.m128_f32[2];
			RX2pst = ENFT_SSE::_mm_add_ps(data.X(ix2p1).XYZx(), st);		work1.m128_f32[2] = work1.m128_f32[3] = 1 / RX2pst.m128_f32[2];
			e = ENFT_SSE::_mm_sub_ps(*px2, ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(RX1pst, RX2pst), work1));
			sqErr = ENFT_SSE::_mm_mul_ps(e, e);
			if(RX1pst.m128_f32[2] > 0 && (sqErr.m128_f32[0] = sqErr.m128_f32[0] + sqErr.m128_f32[1]) < m_ransacErrorThreshold)
			{
				inliers.push_back(ix2);
				fitErr += sqErr.m128_f32[0];
			}
			if(RX2pst.m128_f32[2] > 0 && (sqErr.m128_f32[2] = sqErr.m128_f32[2] + sqErr.m128_f32[3]) < m_ransacErrorThreshold)
			{
				inliers.push_back(ix2p1);
				fitErr += sqErr.m128_f32[2];
			}
		}
		if(_N != N)
		{
			RX1pst = ENFT_SSE::_mm_add_ps(data.X(_N).XYZx(), st);
			work1.m128_f32[0] = 1 / RX1pst.m128_f32[2];
			const Point2D x(RX1pst.m128_f32[0] * work1.m128_f32[0], RX1pst.m128_f32[1] * work1.m128_f32[0]);
			if(RX1pst.m128_f32[2] > 0 && (sqErr.m128_f32[0] = data.x(_N).SquaredDistance(x)) < m_ransacErrorThreshold)
			{
				inliers.push_back(_N);
				fitErr += sqErr.m128_f32[0];
			}
		}
	}

	virtual void VerifyModel(const TSEData &data, const TSEModel &model, const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;

		m_work.Resize(4);
		ENFT_SSE::__m128 &st = m_work[0], &RX1pst = m_work[1], &RX2pst = m_work[2], &e = m_work[3], &sqErr = m_work[3], &work1 = m_work[3];
		st = ENFT_SSE::_mm_mul_ps(data.GetTranslation().v012x(), _mm_set1_ps(model));
		const ushort N = data.Size(), _N = N - (N & 1);
		const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
		for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2)
		{
			RX1pst = ENFT_SSE::_mm_add_ps(data.X(ix2).XYZx(), st);		work1.m128_f32[0] = work1.m128_f32[1] = 1 / RX1pst.m128_f32[2];
			RX2pst = ENFT_SSE::_mm_add_ps(data.X(ix2p1).XYZx(), st);		work1.m128_f32[2] = work1.m128_f32[3] = 1 / RX2pst.m128_f32[2];
			e = ENFT_SSE::_mm_sub_ps(*px2, ENFT_SSE::_mm_mul_ps(_mm_movelh_ps(RX1pst, RX2pst), work1));
			sqErr = ENFT_SSE::_mm_mul_ps(e, e);
			if(RX1pst.m128_f32[2] > 0 && (sqErr.m128_f32[0] = sqErr.m128_f32[0] + sqErr.m128_f32[1]) < m_ransacErrorThreshold)
			{
				inliers.push_back(ix2);
				fitErr += sqErr.m128_f32[0];
			}
			else if(inlierMarksFix[ix2])
			{
				inliers.resize(0);
				fitErr = 0;
				return;
			}
			if(RX2pst.m128_f32[2] > 0 && (sqErr.m128_f32[2] = sqErr.m128_f32[2] + sqErr.m128_f32[3]) < m_ransacErrorThreshold)
			{
				inliers.push_back(ix2p1);
				fitErr += sqErr.m128_f32[2];
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
			RX1pst = ENFT_SSE::_mm_add_ps(data.X(_N).XYZx(), st);
			work1.m128_f32[0] = 1 / RX1pst.m128_f32[2];
			const Point2D x(RX1pst.m128_f32[0] * work1.m128_f32[0], RX1pst.m128_f32[1] * work1.m128_f32[0]);
			if(RX1pst.m128_f32[2] > 0 && (sqErr.m128_f32[0] = data.x(_N).SquaredDistance(x)) < m_ransacErrorThreshold)
			{
				inliers.push_back(_N);
				fitErr += sqErr.m128_f32[0];
			}
		}
	}

	virtual void OptimizeModel(TSEData &data, TSEModel &model, const ubyte verbose = 0)
	{
		m_optimizer.Run(data, model, m_optimizeMaxNumIters, m_work, verbose >= 3);
	}
};

#endif