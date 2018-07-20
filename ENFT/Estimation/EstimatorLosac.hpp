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

#include "Utility/Random.h"

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline uint Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::RunLosac(const Data &data, Model &model, std::vector<Index> &inliers, const ubyte verbose)
{
	inliers.resize(0);
	const Index N = data.Size(), m = MinimalSampleSize();
	if(N < m)
		return 0;

	// Ransac
	const Index nInliersMin = std::max(Index(ceil(N * m_ransacMinInlierProportion)), m);
	const float logEta = log(1 - m_ransacStopConfidence);
	Index nInliersMax = m - 1, nInliers;
	uint nIters = m_ransacMaxNumIters;
	float epsilon;
	double fitErrMin = DBL_MAX, fitErr;
	
	// Losac
	bool inner = false;
	uint innerIter = 0;
	m_marks.assign(N, false);

	uint iter;
	for(iter = 0; iter < nIters; ++iter)
	{
		if(inner)
		{
			if(++innerIter == m_losacNumInnerIters)
			{
				inner = false;
				innerIter = 0;
			}
			if(DrawNonMinimalSample(data, inliers, m_nonMinimalSample, m_idxs, m_marks))
				GenerateModels(m_nonMinimalSample, m_models);
			else
				m_models.Resize(0);
		}
		else
		{
			DrawMinimalSample(data, m_minimalSample);
			GenerateModels(m_minimalSample, m_models);
		}
		VerifyModels(data, m_models, m_inliers, m_inliers2, fitErr);
		if((nInliers = Index(m_inliers.size())) == nInliersMax && fitErr < fitErrMin)
		{
			//printf("%d: %d\n", iter, nInliers);
			nInliersMax = nInliers;
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;
		}
		else if(nInliers > nInliersMax)
		{
			//printf("%d: %d\n", iter, nInliers);
			nInliersMax = nInliers;
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;

			epsilon = float(nInliersMax) / N;
			nIters = uint(ceil(logEta / log(1 - epsilon_exp_m(epsilon))));
			if(nIters > m_ransacMaxNumIters || nInliers < nInliersMin)
				nIters = m_ransacMaxNumIters;
			else if(nIters < m_ransacMinNumIters)
				nIters = m_ransacMinNumIters;

			inner = true;
			//innerIter = 0;
		}
	}

	//printf("%d iterations\n", nIters);
	if(m_solveByAllInliers)
		SolveByAllInliers(data, model, inliers, fitErrMin, verbose);
	if(m_optimizeByAllInliers)
		OptimizeByAllInliers(data, model, inliers, fitErrMin, verbose);

	if(verbose >= 1)
		printf("  %d iterations, %d / %d inliers, error = %e\n", iter, inliers.size(), data.Size(), fitErrMin / inliers.size());

	return iter;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline uint Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::RunLosacFixInliers(const Data &data, const std::vector<bool> &inlierMarksFix, Model &model, 
																	   std::vector<Index> &inliers, const ubyte verbose)
{
	inliers.resize(0);
	const Index N = data.Size(), m = MinimalSampleSize();
	if(N < m)
		return 0;

	double fitErrMin;
	//VerifyModel(data, model, inlierMarksFix, inliers, fitErrMin);
	FromInlierMarksToInliers(inlierMarksFix, inliers);
	VerifyModel(data, model, inlierMarksFix, fitErrMin);
	Index nInliersMax = Index(inliers.size());
	const Index nInliersMin = max(nInliersMax, m);

	const float logEta = log(1 - m_ransacStopConfidence);
	uint nIters;
	float epsilon = float(nInliersMax) / N;
	nIters = uint(ceil(logEta / log(1 - epsilon_exp_m(epsilon))));
	if(nIters > m_ransacMaxNumIters || nInliersMax < max(Index(ceil(N * m_ransacMinInlierProportion)), m))
		nIters = m_ransacMaxNumIters;
	else if(nIters < m_ransacMinNumIters)
		nIters = m_ransacMinNumIters;

	Index nInliers;
	double fitErr;
	
	bool inner = nInliersMax > max(Index(ceil(N * m_ransacMinInlierProportion)), m);
	uint innerIter = 0;
	m_marks.assign(N, false);

	uint iter;
	for(iter = 0; iter < nIters; ++iter)
	{
		if(inner)
		{
			if(++innerIter == m_losacNumInnerIters)
			{
				inner = false;
				innerIter = 0;
			}
			if(DrawNonMinimalSample(data, inliers, m_nonMinimalSample, m_idxs, m_marks))
				GenerateModels(m_nonMinimalSample, m_models);
			else
				m_models.Resize(0);
		}
		else
		{
			DrawMinimalSample(data, m_minimalSample);
			GenerateModels(m_minimalSample, m_models);
		}
		VerifyModels(data, inlierMarksFix, m_models, m_inliers, m_inliers2, fitErr);
		if((nInliers = Index(m_inliers.size())) < nInliersMin)
			continue;
		if(nInliers == nInliersMax && fitErr < fitErrMin)
		{
			//printf("%d: %d\n", iter, nInliers);
			nInliersMax = nInliers;
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;
		}
		else if(nInliers > nInliersMax)
		{
			//printf("%d: %d\n", iter, nInliers);
			nInliersMax = nInliers;
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;

			epsilon = float(nInliersMax) / N;
			nIters = uint(ceil(logEta / log(1 - epsilon_exp_m(epsilon))));
			if(nIters > m_ransacMaxNumIters)
				nIters = m_ransacMaxNumIters;
			else if(nIters < m_ransacMinNumIters)
				nIters = m_ransacMinNumIters;

			inner = true;
			//innerIter = 0;
		}
	}

	//printf("%d iterations\n", nIters);
	if(m_solveByAllInliers)
		SolveByAllInliers(data, inlierMarksFix, model, inliers, fitErrMin, verbose);
	if(m_optimizeByAllInliers)
		OptimizeByAllInliers(data, inlierMarksFix, model, inliers, fitErrMin, verbose);

	if(verbose >= 1)
		printf("  %d iterations, %d / %d inliers, error = %e\n", iter, inliers.size(), data.Size(), fitErrMin / inliers.size());

	return iter;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline bool Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::RunLosacInner(const Data &data, Model &model, std::vector<Index> &inliers, const ubyte verbose)
{
	const Index N = data.Size(), m = MinimalSampleSize();
	if(N < m)
		return 0;

	// Ransac
	const Index nInliersMin = max(Index(ceil(N * m_ransacMinInlierProportion)), m);
	const float logEta = log(1 - m_ransacStopConfidence);
	Index nInliersMax = m - 1, nInliers;
	uint nIters = m_losacNumInnerIters;
	float epsilon;
	double fitErrMin = DBL_MAX, fitErr;
	
	// Losac
	m_marks.assign(N, false);

	uint iter;
	for(iter = 0; iter < nIters; ++iter)
	{
		if(DrawNonMinimalSample(data, inliers, m_nonMinimalSample, m_idxs, m_marks))
			GenerateModels(m_nonMinimalSample, m_models);
		else
			m_models.Resize(0);
		VerifyModels(data, m_models, m_inliers, m_inliers2, fitErr);
		if((nInliers = Index(m_inliers.size())) < nInliersMin)
			continue;
		if(nInliers == nInliersMax && fitErr < fitErrMin)
		{
			//printf("%d: %d\n", iter, nInliers);
			nInliersMax = nInliers;
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;
		}
		else if(nInliers > nInliersMax)
		{
			//printf("%d: %d\n", iter, nInliers);
			nInliersMax = nInliers;
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;

			epsilon = float(nInliersMax) / N;
			nIters = uint(ceil(logEta / log(1 - epsilon_exp_m(epsilon))));
			if(nIters > m_losacNumInnerIters || nInliers < nInliersMin)
				nIters = m_losacNumInnerIters;
		}
	}

	if(m_solveByAllInliers)
		SolveByAllInliers(data, model, inliers, fitErrMin, verbose);
	if(m_optimizeByAllInliers)
		OptimizeByAllInliers(data, model, inliers, fitErrMin, verbose);

	if(verbose >= 1)
		printf("  %d iterations, %d / %d inliers, error = %e\n", iter, nInliersMax, data.Size(), fitErrMin / nInliersMax);

	return nInliersMax >= nInliersMin;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline bool Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::RunLosacInnerFixInliers(const Data &data, const std::vector<bool> &inlierMarksFix, Model &model, 
																			std::vector<Index> &inliers, const ubyte verbose)
{
	const Index N = data.Size(), m = MinimalSampleSize();
	if(N < m)
		return 0;

	double fitErrMin;
	//VerifyModel(data, model, inlierMarksFix, inliers, fitErrMin);
	FromInlierMarksToInliers(inlierMarksFix, inliers);
	VerifyModel(data, model, inlierMarksFix, fitErrMin);
	const Index nInliersMin = max(Index(inliers.size()), max(Index(ceil(N * m_ransacMinInlierProportion)), m));

	// Ransac
	const float logEta = log(1 - m_ransacStopConfidence);
	Index nInliersMax = m - 1, nInliers;
	uint nIters = m_losacNumInnerIters;
	float epsilon;
	double fitErr;
	
	// Losac
	m_marks.assign(N, false);

	uint iter;
	for(iter = 0; iter < nIters; ++iter)
	{
		if(DrawNonMinimalSample(data, inliers, m_nonMinimalSample, m_idxs, m_marks))
			GenerateModels(m_nonMinimalSample, m_models);
		else
			m_models.Resize(0);
		VerifyModels(data, inlierMarksFix, m_models, m_inliers, m_inliers2, fitErr);
		if((nInliers = Index(m_inliers.size())) < nInliersMin)
			continue;
		if(nInliers == nInliersMax && fitErr < fitErrMin)
		{
			//printf("%d: %d\n", iter, nInliers);
			nInliersMax = nInliers;
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;
		}
		else if(nInliers > nInliersMax)
		{
			//printf("%d: %d\n", iter, nInliers);
			nInliersMax = nInliers;
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;

			epsilon = float(nInliersMax) / N;
			nIters = uint(ceil(logEta / log(1 - epsilon_exp_m(epsilon))));
			if(nIters > m_losacNumInnerIters || nInliers < nInliersMin)
				nIters = m_losacNumInnerIters;
		}
	}

	if(m_solveByAllInliers)
		SolveByAllInliers(data, inlierMarksFix, model, inliers, fitErrMin, verbose);
	if(m_optimizeByAllInliers)
		OptimizeByAllInliers(data, inlierMarksFix, model, inliers, fitErrMin, verbose);

	if(verbose >= 1)
		printf("  %d iterations, %d / %d inliers, error = %e\n", iter, nInliersMax, data.Size(), fitErrMin / nInliersMax);

	return nInliersMax >= nInliersMin;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::RunLosacMultiple(const Data &data, AlignedVector<Model> &modelList, std::vector<std::vector<Index> > &inliersList, 
																	 const uint maxNumModels, const Index minNumInliersPerModel, const ubyte verbose)
{
	modelList.Resize(0);
	inliersList.resize(0);
	Index N = data.Size();
	if(N < std::min(MinimalSampleSize(), minNumInliersPerModel))
		return;

	m_idxs2.resize(N);
	for(Index i = 0; i < N; ++i)
		m_idxs2[i] = i;

	modelList.Resize(maxNumModels);
	inliersList.resize(maxNumModels);
	for(uint iModel = 0; iModel < maxNumModels; ++iModel)
	{
		if(verbose >= 1)
			printf("Model %d:\n", iModel);
		std::vector<Index> &inliers = inliersList[iModel];
		if(iModel == 0)
			RunLosac(data, modelList[iModel], inliers, verbose);
		else
			RunLosac(m_subset, modelList[iModel], inliers, verbose);
		const Index nInliers = Index(inliers.size());
		if(nInliers < minNumInliersPerModel)
		{
			modelList.Resize(iModel);
			inliersList.resize(iModel);
			break;
		}

		N = Index(m_idxs2.size());
		m_marks.assign(N, false);
		for(Index i = 0; i < nInliers; ++i)
		{
			m_marks[inliers[i]] = true;
			inliers[i] = m_idxs2[inliers[i]];
		}

		for(Index i = 0, j = 0; i < N; ++i)
		{
			if(m_marks[i])
				continue;
			m_idxs2[j] = m_idxs2[i];
			++j;
		}
		N -= nInliers;
		m_idxs2.resize(N);
		//if(N < minNumInliersPerModel)
		//{
		//	modelList.Resize(iModel + 1);
		//	inliersList.resize(iModel + 1);
		//	break;
		//}
		data.GetSubset(m_idxs2, m_subset);
	}
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline bool Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::DrawNonMinimalSample(const Data &data, const std::vector<Index> &inliers, NonMinimalSample &sample, 
																		 std::vector<Index> &sampleIdxs, std::vector<bool> &sampleMasks) const
{
	const Index nInliers = Index(inliers.size());
	if(nInliers < MinimalSampleSize())
		return false;
	Index sampleSz = Index(ceil(nInliers * m_losacInnerSampleRatio));
	if(sampleSz < MinimalSampleSize())
		sampleSz = MinimalSampleSize();
	sampleIdxs.resize(sampleSz);

	Index idx;
	for(Index i = 0; i < sampleSz; i++)
	{
		while(sampleMasks[(idx = inliers[Random::Generate(nInliers)])]);
		sampleMasks[idx] = true;
		sampleIdxs[i] = idx;
	}
	for(Index i = 0; i < sampleSz; i++)
		sampleMasks[sampleIdxs[i]] = false;
	data.GetSubset(sampleIdxs, sample);
	return true;
}