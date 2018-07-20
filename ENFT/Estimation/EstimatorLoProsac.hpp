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
inline void Estimator<ESTIMATOR_TEMPLATE_ARGUMENT>::RunLoProsac(const Data &data, const std::vector<Index> &orders, Model &model, std::vector<Index> &inliers, 
																const ubyte verbose)
{
#if _DEBUG
	assert(data.Size() == orders.size());
#endif

	inliers.resize(0);
	const Index N = data.Size(), m = MinimalSampleSize();
	if(N < m)
		return;

	// Ransac
	const Index nInliersMin = std::max(Index(ceil(N * m_ransacMinInlierProportion)), m);
	const float logEta = log(1 - m_ransacStopConfidence);
	Index I_N_best = m - 1;
	uint nIters = m_ransacMaxNumIters;
	double fitErrMin = DBL_MAX, fitErr;

	// Losac
	m_marks.assign(N, false);

	// Prosac
	const float beta = m_prosacRandomConsistencyProbability;
	const float one_m_beta_x2p706 = (1 - beta) * 2.706f;
	const uint T_N = uint(ceil(logEta / log(1 - epsilon_exp_m(m_ransacMinInlierProportion))));
	uint T_n_prime = 1, k_n_star = T_N;
	Index n = m, n_star = N, I_n_star = 0;
	float T_n = (float) T_N;
	for(Index i = 0; i < m; ++i)
		T_n *= float(m - i) / (N - i);
	//std::vector<Index> U_n;
	//for(Index i = 0; i < m; i++)
	//	U_n.push_back(i);

	uint t;
	for(t = 0; t < nIters; ++t)
	{
		// 1. Choice of the hypothesis generation set
		// According to Equation 5 in the text, not the algorithm
		if(t >= T_n_prime && n < n_star)
		{
			if(++n >= N)
				break;
			//U_n.push_back(n - 1);

			float T_n_last = T_n;
			T_n *= float(n) / (n - m);
			T_n_prime += uint(ceil(T_n - T_n_last));
		}

		// 2. Semi-random sample M_t of size m
		DrawMinimalSampleOrdered(data, orders, n, t < T_n_prime, m_minimalSample);

		// 3. Model parameter estimation
		GenerateModels(m_minimalSample, m_models);

		// 4. Model verification
		//Find support (i.e. consistent data points) of the model with parameters p_t
		VerifyModels(data, m_models, m_inliers, m_inliers2, fitErr);

		Index I_N = Index(m_inliers.size());
		if(I_N == I_N_best && fitErr < fitErrMin)
		{
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;
		}
		else if(I_N > I_N_best)
		{
			I_N_best = I_N;
			fitErrMin = fitErr;
			model = m_models[0];
			inliers = m_inliers;

			for(uint innerIter = 0; innerIter < m_losacNumInnerIters; ++innerIter)
			{
				DrawNonMinimalSample(data, inliers, m_nonMinimalSample, m_idxs, m_marks);
				GenerateModels(m_nonMinimalSample, m_models);
				VerifyModels(data, m_models, m_inliers, m_inliers2, fitErr);
				I_N = Index(m_inliers.size());
				if(I_N > I_N_best || I_N == I_N_best && fitErr < fitErrMin)
				{
					I_N_best = I_N;
					fitErrMin = fitErr;
					model = m_models[0];
					inliers = m_inliers;
				}
			}

			// Select termination length n_star if possible according to Section 2.2
			// * Non-randomness : I_n >= I_n*_min (eq. (9))
			//	
			// so that the probability of having missed a set of inliers is below eta
			// This is the classical RANSAC termination criterion,
			// except that it takes into account only the n first samples (not the total number of samples)
			// k_n_star = log(eta) / log(1-(I_n_star/n_star)^m) (eq. (12))
			// We have to minimize k_n_star, e.g. maximize I_n_star/n_star
			// a straightforward implementation would use the following test:
			// if (I_n_test > epsilon_n_best*n_test) {...}
			// However, since I_n is binomial, and in the case of evenly distributed inliers,
			// a better test would be to reduce n_star only if there's a significant improvement in
			// epsilon. Thus we use a Chi-squared test (P=0.10), together with the normal approximation
			// to the binomial (mu = epsilon_n_star*n_test, sigma=sqrt(n_test*epsilon_n_star*(1-epsilon_n_star)).
			// There is a significant difference between the two tests
			// We do the cheap test first, and the expensive test only if the cheap one passes.
			Index n_best = N, I_n_best = I_N_best, n_test = N, I_n_test = I_N_best;
			double epsilon_n_best = double(I_n_best) / n_best;
			for(typename std::vector<Index>::const_reverse_iterator pIdx = inliers.rbegin(), pIdxEnd = inliers.rend(); pIdx != pIdxEnd && *pIdx > m; ++pIdx, --I_n_test)
			{
				n_test = (*pIdx) + 1;
				if(I_n_test * n_best > I_n_best * n_test && I_n_test > epsilon_n_best*n_test + sqrt(n_test*epsilon_n_best*(1-epsilon_n_best)*2.706))
				{
					//double mu = n_test * m_prosacRandomConsistencyProbability;
					//double sigma = sqrt(mu * m_prosacRandomConsistencyProbability * (1 - m_prosacRandomConsistencyProbability));
					//// Imin(n) (equation (8) can then be obtained with the Chi-squared test with P=2*psi=0.10 (Chi2=2.706)
					//Index I_n_min = m + Index(mu + sigma * sqrt2p706);
					Index I_n_min = m + Index(ceil((n_test + sqrt(n_test * one_m_beta_x2p706)) * beta));
					if(I_n_test < I_n_min)
						break;
					n_best = n_test;
					I_n_best = I_n_test;
					epsilon_n_best = double(I_n_best) / n_best;
				}
			}

			//if(I_n_best * n_star > I_n_star * n_best)
			if(I_n_star < epsilon_n_best * n_star)
			{
				n_star = n_best;
				I_n_star = I_n_best;
				float epsilon_n_star = float(epsilon_n_best); 
				k_n_star = uint(ceil(logEta / log(1 - epsilon_exp_m(epsilon_n_star))));
				if(k_n_star > T_N)
					k_n_star = T_N;

				nIters = k_n_star;
				if(nIters > m_ransacMaxNumIters || I_N_best < nInliersMin)
					nIters = m_ransacMaxNumIters;
				else if(nIters < m_ransacMinNumIters)
					nIters = m_ransacMinNumIters;
			}
		}
	}

	//printf("%d iterations\n", nIters);
	if(m_solveByAllInliers)
		SolveByAllInliers(data, model, inliers, fitErrMin, verbose);
	if(m_optimizeByAllInliers)
		OptimizeByAllInliers(data, model, inliers, fitErrMin, verbose);
	I_N_best = Index(inliers.size());

	if(verbose >= 1)
		printf("  %d iterations, %d / %d inliers, error = %e\n", t, I_N_best, data.Size(), fitErrMin / I_N_best);
}