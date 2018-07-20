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

#ifndef _RELATIVE_POSE_ESTIMATOR_DEPTH_H_
#define _RELATIVE_POSE_ESTIMATOR_DEPTH_H_

#include "Estimation/Estimator.h"
#include "RelativePoseEstimatorDataDepth.h"
#include "RigidTransformationSolver.h"
#include "EssentialMatrix.h"

typedef RelativePoseEstimatorDataDepth	RPEDData;
typedef SixMatches3D					RPEDMinimalSample;
typedef MatchSet3DX						RPEDNonMinimalSample;
typedef RigidTransformation3D			RPEDModel;
typedef RigidTransformationSolver		RPEDSolver;
typedef int								RPEDOptimizer;
typedef ushort							RPEDIndex;

class RelativePoseEstimatorDepth : public Estimator<RPEDData, RPEDMinimalSample, RPEDNonMinimalSample, RPEDModel, RPEDSolver, RPEDOptimizer, RPEDIndex>
{

public:

	RelativePoseEstimatorDepth(const float &errTh = 0) 
		: Estimator<RPEDData, RPEDMinimalSample, RPEDNonMinimalSample, RPEDModel, RPEDSolver, RPEDOptimizer, RPEDIndex>(errTh) {}

	virtual const ushort MinimalSampleSize() const
	{
		return 6;
	}

	virtual float epsilon_exp_m(const float &epsilon) const
	{
		const float tmp = epsilon * epsilon;
		return tmp * tmp * tmp;
	}

	virtual void DrawMinimalSample(const RPEDData &data, RPEDMinimalSample &sample) const
	{
		const MatchSet3DX &matches3D = data.GetMatches3D();
		const ushort N = matches3D.Size();
		ushort i0 = Random::Generate(N);
		sample.Set(0, matches3D.X1(i0), matches3D.X2(i0));

		ushort i1 = Random::Generate(N);
		while(i1 == i0)
			i1 = Random::Generate(N);
		sample.Set(1, matches3D.X1(i1), matches3D.X2(i1));

		ushort i2 = Random::Generate(N);
		while(i2 == i0 || i2 == i1)
			i2 = Random::Generate(N);
		sample.Set(2, matches3D.X1(i2), matches3D.X2(i2));

		ushort i3 = Random::Generate(N);
		while(i3 == i0 || i3 == i1 || i3 == i2)
			i3 = Random::Generate(N);
		sample.Set(3, matches3D.X1(i3), matches3D.X2(i3));

		ushort i4 = Random::Generate(N);
		while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
			i4 = Random::Generate(N);
		sample.Set(4, matches3D.X1(i4), matches3D.X2(i4));

		ushort i5 = Random::Generate(N);
		while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4)
			i5 = Random::Generate(N);
		sample.Set(5, matches3D.X1(i5), matches3D.X2(i5));
	}

	virtual void DrawMinimalSampleOrdered(const RPEDData &data, const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne, 
										  RPEDMinimalSample &sample) const
	{
		const MatchSet3DX &matches3D = data.GetMatches3D();
		ushort i0 = Random::Generate(n), i;
		while((i = data.GetIndex3D(orders[i0])) == USHRT_MAX)
			i0 = Random::Generate(n);
		sample.Set(0, matches3D.X1(i), matches3D.X2(i));

		ushort i1 = Random::Generate(n);
		while(i1 == i0 || (i = data.GetIndex3D(orders[i1])) == USHRT_MAX)
			i1 = Random::Generate(n);
		sample.Set(1, matches3D.X1(i), matches3D.X2(i));

		ushort i2 = Random::Generate(n);
		while(i2 == i0 || i2 == i1 || (i = data.GetIndex3D(orders[i2])) == USHRT_MAX)
			i2 = Random::Generate(n);
		sample.Set(2, matches3D.X1(i), matches3D.X2(i));

		ushort i3 = Random::Generate(n);
		while(i3 == i0 || i3 == i1 || i3 == i2 || (i = data.GetIndex3D(orders[i3])) == USHRT_MAX)
			i3 = Random::Generate(n);
		sample.Set(3, matches3D.X1(i), matches3D.X2(i));

		ushort i4 = Random::Generate(n);
		while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3 || (i = data.GetIndex3D(orders[i4])) == USHRT_MAX)
			i4 = Random::Generate(n);
		sample.Set(4, matches3D.X1(i), matches3D.X2(i));

		ushort i5 = n - 1;
		if(!sampleLastOne || i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4 || (i = data.GetIndex3D(orders[i5])) == USHRT_MAX)
		{
			i5 = Random::Generate(n);
			while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4 || (i = data.GetIndex3D(orders[i5])) == USHRT_MAX)
				i5 = Random::Generate(n);
		}
		sample.Set(5, matches3D.X1(i), matches3D.X2(i));
	}

	virtual void GenerateModels(RPEDMinimalSample &sample, AlignedVector<RPEDModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample, models[0], m_work))
			models.Resize(0);
	}

	virtual void GenerateModels(RPEDNonMinimalSample &sample, AlignedVector<RPEDModel> &models)
	{
		models.Resize(1);
		if(!m_solver.Run(sample.X1s(), sample.X2s(), models[0], m_work))
			models.Resize(0);
	}

	virtual void VerifyModel(const RPEDData &data, const RPEDModel &model, const std::vector<bool> &inlierMarks, double &fitErr)
	{
		fitErr = 0;
		m_work.Resize(17);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *E = &m_work[1], *work = &m_work[10];
		m_E.FromRelativePose(model);
		m_E.GetSSE(E);
		const MatchSet2D &matches2D = data.GetMatches2D();
		const ushort nPacks = matches2D.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			m_E.ComputeSymmetricSquaredError4(E, matches2D.GetPack(i), matches2D.GetPack(ip1), matches2D.GetPack(ip2), matches2D.GetPack(ip3), errSq, work);
			if(inlierMarks[i])
				fitErr += errSq.m128_f32[0];
			if(inlierMarks[ip1])
				fitErr += errSq.m128_f32[1];
			if(inlierMarks[ip2])
				fitErr += errSq.m128_f32[2];
			if(inlierMarks[ip3])
				fitErr += errSq.m128_f32[3];
		}
		for(ushort iRem = 0; iRem < matches2D.GetRemindersNumber(); ++iRem)
		{
			if(inlierMarks[nPacks + iRem])
				fitErr += m_E.ComputeSymmetricSquaredError(matches2D.GetReminder1(iRem), matches2D.GetReminder2(iRem), (float *) work);
		}
	}

	virtual void VerifyModel(const RPEDData &data, const RPEDModel &model, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;
		m_work.Resize(17);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *E = &m_work[1], *work = &m_work[10];
		m_E.FromRelativePose(model);
		m_E.GetSSE(E);
		const MatchSet2D &matches2D = data.GetMatches2D();
		const ushort nPacks = matches2D.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			m_E.ComputeSymmetricSquaredError4(E, matches2D.GetPack(i), matches2D.GetPack(ip1), matches2D.GetPack(ip2), matches2D.GetPack(ip3), errSq, work);
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
		for(ushort iRem = 0; iRem < matches2D.GetRemindersNumber(); ++iRem)
		{
			const float errSq = m_E.ComputeSymmetricSquaredError(matches2D.GetReminder1(iRem), matches2D.GetReminder2(iRem), (float *) work);
			if(errSq < m_ransacErrorThreshold)
			{
				inliers.push_back(nPacks + iRem);
				fitErr += errSq;
			}
		}
	}

	virtual void VerifyModel(const RPEDData &data, const RPEDModel &model, const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers, double &fitErr)
	{
		inliers.resize(0);
		fitErr = 0;
		m_work.Resize(17);
		ENFT_SSE::__m128 &errSq = m_work[0];
		ENFT_SSE::__m128 *E = &m_work[1], *work = &m_work[10];
		m_E.FromRelativePose(model);
		m_E.GetSSE(E);
		const MatchSet2D &matches2D = data.GetMatches2D();
		const ushort nPacks = matches2D.GetPacksNumber();
		for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks; i += 4, ip1 += 4, ip2 += 4, ip3 += 4)
		{
			m_E.ComputeSymmetricSquaredError4(E, matches2D.GetPack(i), matches2D.GetPack(ip1), matches2D.GetPack(ip2), matches2D.GetPack(ip3), errSq, work);
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
		for(ushort iRem = 0; iRem < matches2D.GetRemindersNumber(); ++iRem)
		{
			const float errSq = m_E.ComputeSymmetricSquaredError(matches2D.GetReminder1(iRem), matches2D.GetReminder2(iRem), (float *) work);
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

	virtual void OptimizeModel(RPEDData &data, RPEDModel &model, const ubyte verbose = 0)
	{
	}

protected:

	EssentialMatrix m_E;

};

#endif