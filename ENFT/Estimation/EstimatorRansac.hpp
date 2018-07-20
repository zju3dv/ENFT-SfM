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

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline uint Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::RunRansac(const Data &data, Model &model, std::vector<Index> &inliers, const ubyte verbose)
{
	inliers.resize(0);
	const Index N = data.Size(), m = MinimalSampleSize();
	if(N < m)
		return 0;

	const Index nInliersMin = std::max(Index(ceil(N * m_ransacMinInlierProportion)), m);
	const float logEta = log(1 - m_ransacStopConfidence);
	Index nInliersMax = m - 1, nInliers;
	uint nIters = m_ransacMaxNumIters;
	float epsilon;
	double fitErrMin = DBL_MAX, fitErr;

	uint iter;
	for(iter = 0; iter < nIters; ++iter)
	{
		DrawMinimalSample(data, m_minimalSample);
		GenerateModels(m_minimalSample, m_models);
		VerifyModels(data, m_models, m_inliers, m_inliers2, fitErr);
		if((nInliers = Index(m_inliers.size())) == nInliersMax && fitErr < fitErrMin)
		{
			nInliersMax = nInliers;
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;
		}
		else if(nInliers > nInliersMax)
		{
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
		}
	}

	if(m_solveByAllInliers)
		SolveByAllInliers(data, model, inliers, fitErrMin, verbose);
	if(m_optimizeByAllInliers)
		OptimizeByAllInliers(data, model, inliers, fitErrMin, verbose);

	if(verbose >= 1)
		printf("  %d iterations, %d / %d inliers, error = %e\n", iter, nInliersMax, data.Size(), fitErrMin / nInliersMax);

	return iter;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::VerifyModels(const Data &data, AlignedVector<Model> &models, std::vector<Index> &inliers, std::vector<Index> &inliersTmp, 
																 double &fitErr)
{
	const uint nModels = models.Size();
	if(nModels == 0)
	{
		inliers.resize(0);
		fitErr = DBL_MAX;
	}
	else
	{
		VerifyModel(data, models[0], inliers, fitErr);
		for(uint i = 1; i < nModels; ++i)
		{
			double _fitErr;
			VerifyModel(data, models[i], inliersTmp, _fitErr);
			if(inliersTmp.size() > inliers.size() || inliersTmp.size() == inliers.size() && _fitErr < fitErr)
			{
				models[0] = models[i];
				inliers = inliersTmp;
				fitErr = _fitErr;
			}
		}
	}
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::VerifyModels(const Data &data, const std::vector<bool> &inlierMarksFix, AlignedVector<Model> &models, 
																 std::vector<Index> &inliers, std::vector<Index> &inliersTmp, double &fitErr)
{
	const uint nModels = models.Size();
	if(nModels == 0)
	{
		inliers.resize(0);
		fitErr = DBL_MAX;
	}
	else
	{
		VerifyModel(data, models[0], inlierMarksFix, inliers, fitErr);
		for(uint i = 1; i < nModels; ++i)
		{
			double _fitErr;
			VerifyModel(data, models[i], inlierMarksFix, inliersTmp, _fitErr);
			if(inliersTmp.size() > inliers.size() || inliersTmp.size() == inliers.size() && _fitErr < fitErr)
			{
				models[0] = models[i];
				inliers = inliersTmp;
				fitErr = _fitErr;
			}
		}
	}
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline bool Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::SolveByAllInliers(const Data &data, Model &model, std::vector<Index> &inliers, double &fitErr, 
																	  const ubyte verbose)
{
	const Index nInliersBefore = Index(inliers.size());
	if(nInliersBefore < MinimalSampleSize())
		return false;
	data.GetSubset(inliers, m_nonMinimalSample);
	GenerateModels(m_nonMinimalSample, m_models);
	if(m_models.Empty())
		return false;
	double fitErrAfter;
	VerifyModels(data, m_models, m_inliers, m_inliers2, fitErrAfter);
	const Index nInliersAfter = Index(m_inliers.size());
	if(fitErr == DBL_MAX)
	{
		if(nInliersAfter < Index(data.Size() * m_ransacMinInlierProportion + 0.5f))
			return false;
		model = m_models[0];
		inliers = m_inliers;
		return true;
	}
	const double fitErrBefore = fitErr;
	if(nInliersAfter > nInliersBefore || nInliersAfter == nInliersBefore && fitErrAfter < fitErrBefore)
	{
		if(verbose >= 2)
		{
			printf("  [SolveByAllInliers]\n");
			printf("      %d / %d inliers, error = %e\n", nInliersBefore, data.Size(), fitErrBefore / nInliersBefore);
			printf("  --> %d / %d inliers, error = %e\n", nInliersAfter, data.Size(), fitErrAfter / nInliersAfter);
		}
		model = m_models[0];
		inliers = m_inliers;
		fitErr = fitErrAfter;
		return true;
	}
	else
		return false;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline bool Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::SolveByAllInliers(const Data &data, const std::vector<bool> &inlierMarksFix, Model &model, 
																	  std::vector<Index> &inliers, double &fitErr, const ubyte verbose)
{
	const Index nInliersBefore = Index(inliers.size());
	if(nInliersBefore < MinimalSampleSize())
		return false;
	data.GetSubset(inliers, m_nonMinimalSample);
	GenerateModels(m_nonMinimalSample, m_models);
	if(m_models.Empty())
		return false;
	double fitErrAfter;
	VerifyModels(data, inlierMarksFix, m_models, m_inliers, m_inliers2, fitErrAfter);
	const Index nInliersAfter = Index(m_inliers.size());
	if(fitErr == DBL_MAX)
	{
		if(nInliersAfter < Index(data.Size() * m_ransacMinInlierProportion + 0.5f))
			return false;
		model = m_models[0];
		inliers = m_inliers;
		return true;
	}
	const double fitErrBefore = fitErr;
	if(nInliersAfter > nInliersBefore || nInliersAfter == nInliersBefore && fitErrAfter < fitErrBefore)
	{
		if(verbose >= 2)
		{
			printf("  [SolveByAllInliers]\n");
			printf("      %d / %d inliers, error = %e\n", nInliersBefore, data.Size(), fitErrBefore / nInliersBefore);
			printf("  --> %d / %d inliers, error = %e\n", nInliersAfter, data.Size(), fitErrAfter / nInliersAfter);
		}
		model = m_models[0];
		inliers = m_inliers;
		fitErr = fitErrAfter;
		return true;
	}
	else
		return false;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline bool Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::OptimizeByAllInliers(const Data &data, Model &model, std::vector<Index> &inliers, double &fitErr, 
																		 const ubyte verbose)
{
	const Index nInliersBefore = Index(inliers.size());
	if(nInliersBefore < MinimalSampleSize())
		return false;
	data.GetSubset(inliers, m_dataInlier);
	m_models.Resize(1);
	m_models[0] = model;
	OptimizeModel(m_dataInlier, m_models[0], verbose);
	double fitErrAfter;
	VerifyModels(data, m_models, m_inliers, m_inliers2, fitErrAfter);
	const Index nInliersAfter = Index(m_inliers.size());
	if(fitErr == DBL_MAX)
	{
		if(nInliersAfter < Index(data.Size() * m_ransacMinInlierProportion + 0.5f))
			return false;
		model = m_models[0];
		inliers = m_inliers;
		return true;
	}
	const double fitErrBefore = fitErr;
	if(nInliersAfter > nInliersBefore || nInliersAfter == nInliersBefore && fitErrAfter < fitErrBefore)
	{
		if(verbose >= 2)
		{
			printf("  [OptimizeByAllInliers]\n");
			printf("      %d / %d inliers, error = %e\n", nInliersBefore, data.Size(), fitErrBefore / nInliersBefore);
			printf("  --> %d / %d inliers, error = %e\n", nInliersAfter, data.Size(), fitErrAfter / nInliersAfter);
		}
		model = m_models[0];
		inliers = m_inliers;
		fitErr = fitErrAfter;
		return true;
	}
	else
		return false;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline bool Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::OptimizeByAllInliers(const Data &data, const std::vector<bool> &inlierMarksFix, Model &model, 
																		 std::vector<Index> &inliers, double &fitErr, const ubyte verbose)
{
	const Index nInliersBefore = Index(inliers.size());
	if(nInliersBefore < MinimalSampleSize())
		return false;
	data.GetSubset(inliers, m_nonMinimalSample);
	m_models.Resize(1);
	m_models[0] = model;
	OptimizeModel(m_nonMinimalSample, m_models[0], verbose);
	double fitErrAfter;
	VerifyModels(data, inlierMarksFix, m_models, m_inliers, m_inliers2, fitErrAfter);
	const Index nInliersAfter = Index(m_inliers.size());
	if(fitErr == DBL_MAX)
	{
		if(nInliersAfter < Index(data.Size() * m_ransacMinInlierProportion + 0.5f))
			return false;
		model = m_models[0];
		inliers = m_inliers;
		return true;
	}
	const double fitErrBefore = fitErr;
	VerifyModel(data, m_models[0], inlierMarksFix, m_inliers, fitErrAfter);
	if(nInliersAfter > nInliersBefore || nInliersAfter == nInliersBefore && fitErrAfter < fitErrBefore)
	{
		if(verbose >= 2)
		{
			printf("  [OptimizeByAllInliers]\n");
			printf("      %d / %d inliers, error = %e\n", nInliersBefore, data.Size(), fitErrBefore / nInliersBefore);
			printf("  --> %d / %d inliers, error = %e\n", nInliersAfter, data.Size(), fitErrAfter / nInliersAfter);
		}
		model = m_models[0];
		inliers = m_inliers;
		fitErr = fitErrAfter;
		return true;
	}
	else
		return false;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline Index Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::CountInliers(const Data &data, const Model &model)
{
	double fitErr;
	VerifyModel(data, model, m_inliers, fitErr);
	return Index(m_inliers.size());
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::FromInliersToInlierMarks(const std::vector<Index> &inliers, const Index N, std::vector<bool> &inlierMarks)
{
	inlierMarks.assign(N, false);
	const Index nInliers = Index(inliers.size());
	for(Index i = 0; i < nInliers; ++i)
		inlierMarks[inliers[i]] = true;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::FromInliersToOutliers(const std::vector<Index> &inliers, const Index N, std::vector<Index> &outliers)
{
	FromInliersToInlierMarks(inliers, N, m_marks);
	outliers.resize(0);
	for(Index i = 0; i < N; ++i)
	{
		if(!m_marks[i])
			outliers.push_back(i);
	}	
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
template<typename TYPE>
inline void Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::FromInlierMarksToInliers(const std::vector<bool> &inlierMarks, std::vector<TYPE> &inliers)
{
	inliers.resize(0);
	const TYPE N = TYPE(inlierMarks.size());
	for(TYPE i = 0; i < N; ++i)
	{
		if(inlierMarks[i])
			inliers.push_back(i);
	}
}