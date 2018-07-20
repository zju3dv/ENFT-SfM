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
inline uint EstimatorParsac<ESTIMATOR_TEMPLATE_ARGUMENT>::RunLoParsac(const Data &data, Model &model, std::vector<Index> &inliers, std::vector<float> &binConfidences, 
																	  const ubyte verbose)
{
	inliers.resize(0);
	this->m_validBinInliersBest.assign(this->GetBinsNumber(), std::vector<Index>());
	const Index N = data.Size(), m = this->MinimalSampleSize();
	if(N < m)
		return 0;
	
	// Ransac
	const Index nInliersMin = max(Index(ceil(N * this->m_ransacMinInlierProportion)), m);
	const float logEta = log(1 - this->m_ransacStopConfidence);
	uint nIters = this->m_ransacMaxNumIters;
	float confidenceMax = 0, confidence;
	Index nInliersMax = 0, nInliers;
	double fitErrMin = DBL_MAX, fitErr;
	
	// Losac
	bool inner = false;
	uint innerIter = 0;
	this->m_marks.assign(N, false);

	// Parsac
	SetImageSize(data.GetImageWidth(), data.GetImageHeight());
	BucketData(data, this->m_mapBinToValidBin, this->m_mapValidBinToBin, this->m_mapDataToValidBin, this->m_validBinData, this->m_validBinDataSizes);
	ConvertConfidencesBinToValidBin(this->m_mapValidBinToBin, binConfidences, this->m_valibBinConfidencesPrior);
	const uint nBinsValid = uint(this->m_mapValidBinToBin.size());
	if(nBinsValid < m || CountNonZeroConfidences(m_valibBinConfidencesPrior) < m)
	{
		nIters = RunLosac(data, model, inliers, verbose);
		ConvertInliersListToValidBin(nBinsValid, inliers, this->m_mapDataToValidBin, this->m_validBinInliers);
		EstimateConfidence(this->m_validBinDataSizes, this->m_validBinInliers, this->m_mapValidBinToBin, this->m_validBinConfidences);
		ConvertConfidencesValidBinToBin(this->m_mapBinToValidBin, this->m_validBinConfidences, binConfidences);
		return nIters;
	}
	ThresholdAndNormalizeConfidences(m_valibBinConfidencesPrior);
	AccumulateConfidences(m_valibBinConfidencesPrior, m_validBinConfidencesAccumulatedPrior);

	//m_iBinsListValidSampled.resize(0);
	//m_inliersList.resize(0);
	uint iter;
	for(iter = 0; iter < nIters; ++iter)
	{
		if(inner)
		{
			if(++innerIter == this->m_losacNumInnerIters)
			{
				inner = false;
				innerIter = 0;
			}
			if(!DrawNonMinimalSample(data, this->m_validBinInliersBest, this->m_nonMinimalSample, this->m_idxs, this->m_marks))
			{
				inner = false;
				innerIter = 0;
				continue;
			}
			GenerateModels(this->m_nonMinimalSample, this->m_models);
		}
		else
		{
			DrawMinimalSample(data, this->m_validBinConfidencesAccumulatedPrior, this->m_validBinData, this->m_minimalSample);
//#if _DEBUG
//			DrawMinimalSample(data, m_validBinConfidencesAccumulatedPrior, m_validBinData, m_minimalSample, m_idxs/*, false*/);
//#endif
			GenerateModels(this->m_minimalSample, this->m_models);
		}
		const uint nModels = this->m_models.Size();
		if(nModels == 0)
			continue;
		for(uint iModel = 0; iModel < nModels; ++iModel)
		{
			VerifyModel(data, this->m_models[iModel], this->m_inliers, fitErr);
			nInliers = Index(this->m_inliers.size());
			if(nInliers < m)
				continue;
			ConvertInliersListToValidBin(nBinsValid, this->m_inliers, this->m_mapDataToValidBin, this->m_validBinInliers);
			confidence = EstimateConfidence(this->m_validBinDataSizes, this->m_validBinInliers, this->m_mapValidBinToBin, this->m_validBinConfidences);
//#if _DEBUG
//			SetTentativeResult(data, m_idxs, m_models[iModel], m_validBinInliers);
//#endif
			if(confidence > confidenceMax || confidence == confidenceMax && (nInliers > nInliersMax || nInliers == nInliersMax && fitErr < fitErrMin))
			{
				confidenceMax = confidence;
				nInliersMax = nInliers;
				fitErrMin = fitErr;
				model = this->m_models[iModel];
				inliers = this->m_inliers;
				this->m_validBinInliersBest = this->m_validBinInliers;
				nIters = UpdateIterationsNumber(this->m_valibBinConfidencesPrior, this->m_validBinConfidences, logEta);
				if(nInliers < nInliersMin)
					nIters = this->m_ransacMaxNumIters;
//#if _DEBUG
//				SetBetterResult(data, m_idxs, model, m_validBinInliers);
//#endif
				//inner = true;
				//innerIter = 0;
				inner = this->m_losacNumInnerIters != 0;
			}
//#if _DEBUG
//			VisualizeResult();
//#endif
		}
	}

	//printf("%d iterations\n", nIters);
	if(this->m_solveByAllInliers)
		SolveByAllInliers(data, model, inliers, fitErrMin, verbose);
	if(this->m_optimizeByAllInliers)
		OptimizeByAllInliers(data, model, inliers, fitErrMin, verbose);

	EstimateConfidence(this->m_validBinDataSizes, this->m_validBinInliersBest, this->m_mapValidBinToBin, this->m_validBinConfidences);
	ConvertConfidencesValidBinToBin(this->m_mapBinToValidBin, this->m_validBinConfidences, binConfidences);

	if(verbose >= 1)
		printf("  %d iterations, %d / %d inliers, error = %e\n", iter, inliers.size(), data.Size(), fitErrMin / inliers.size());

	return iter;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void EstimatorParsac<ESTIMATOR_TEMPLATE_ARGUMENT>::ConvertConfidencesBinToValidBin(const std::vector<uint> &mapValidBinToBin, const std::vector<float> &binConfidences, 
																						  std::vector<float> &validBinConfidences) const
{
	const uint nBinsValid = uint(mapValidBinToBin.size());
	validBinConfidences.resize(nBinsValid);
	for(uint iBinValid = 0; iBinValid < nBinsValid; ++iBinValid)
		validBinConfidences[iBinValid] = binConfidences[mapValidBinToBin[iBinValid]];
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void EstimatorParsac<ESTIMATOR_TEMPLATE_ARGUMENT>::ConvertConfidencesValidBinToBin(const std::vector<uint> &mapBinToValidBin, const std::vector<float> &validBinConfidences, 
																						  std::vector<float> &binConfidences) const
{
	const uint nBins = uint(mapBinToValidBin.size());
	binConfidences.resize(nBins);
	for(uint iBin = 0; iBin < nBins; ++iBin)
	{
		if(mapBinToValidBin[iBin] == UINT_MAX)
			binConfidences[iBin] = 0;
		else
			binConfidences[iBin] = validBinConfidences[mapBinToValidBin[iBin]];
	}
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void EstimatorParsac<ESTIMATOR_TEMPLATE_ARGUMENT>::ThresholdAndNormalizeConfidences(std::vector<float> &confidences) const
{
	float confidenceSum = 0;
	const uint N = uint(confidences.size());
	for(uint i = 0; i < N; ++i)
	{
		confidences[i] = std::max(confidences[i], m_parsacMinPriorBinConfidence);
		confidenceSum += confidences[i];
	}
	const float norm = 1 / confidenceSum;
	for(uint i = 0; i < N; ++i)
		confidences[i] *= norm;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void EstimatorParsac<ESTIMATOR_TEMPLATE_ARGUMENT>::AccumulateConfidences(const std::vector<float> &confidences, std::vector<float> &confidencesAccumulated) const
{
	const uint N = uint(confidences.size());
	confidencesAccumulated.resize(N + 1);
	confidencesAccumulated[0] = 0;
	for(uint i = 0; i < N; ++i)
		confidencesAccumulated[i + 1] = confidencesAccumulated[i] + confidences[i];
	const float norm = 1 / confidencesAccumulated[N];
	for(uint i = 0; i < N; ++i)
		confidencesAccumulated[i] *= norm;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline uint EstimatorParsac<ESTIMATOR_TEMPLATE_ARGUMENT>::CountNonZeroConfidences(const std::vector<float> &confidences) const
{
	const uint N = uint(confidences.size());
	uint cnt = 0;
	for(uint i = 0; i < N; ++i)
	{
		if(confidences[i] != 0)
			++cnt;
	}
	return cnt;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline uint EstimatorParsac<ESTIMATOR_TEMPLATE_ARGUMENT>::UpdateIterationsNumber(const std::vector<float> &validBinConfidencesProir, 
																				 const std::vector<float> &validBinConfidences, const float &logEta) const
{	
	float probOneInlier = 0;
	const uint nBinsValid = uint(validBinConfidencesProir.size());
	for(uint iBinValid = 0; iBinValid < nBinsValid; ++iBinValid)
		probOneInlier += validBinConfidencesProir[iBinValid] * validBinConfidences[iBinValid];
	uint nIters = uint(ceil(logEta / log(1 - this->epsilon_exp_m(probOneInlier))));
	if(nIters > this->m_ransacMaxNumIters)
		nIters = this->m_ransacMaxNumIters;
	else if(nIters < this->m_ransacMinNumIters)
		nIters = this->m_ransacMinNumIters;
	return nIters;
}