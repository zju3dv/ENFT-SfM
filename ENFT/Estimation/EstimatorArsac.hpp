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
inline uint EstimatorArsac<ESTIMATOR_TEMPLATE_ARGUMENT>::RunLoArsac(const Data &data, Model &model, std::vector<Index> &inliers, const ubyte verbose)
{
	inliers.resize(0);
	const Index N = data.Size(), m = this->MinimalSampleSize();
	if(N < m)
		return 0;
	
	// Ransac
	const Index nInliersMin = std::max(Index(ceil(N * this->m_ransacMinInlierProportion)), m);
	const float logEta = log(1 - this->m_ransacStopConfidence);
	uint nIters = this->m_ransacMaxNumIters;
	float confidenceMax = 0, confidence;
	Index nInliersMax = 0, nInliers;
	double fitErrMin = DBL_MAX, fitErr;
	float epsilon;
	
	// Losac
	bool inner = false;
	uint innerIter = 0;
	this->m_marks.assign(N, false);

	// Arsac
	SetImageSize(data.GetImageWidth(), data.GetImageHeight());
	BucketData(data, m_mapBinToValidBin, m_mapValidBinToBin, m_mapDataToValidBin, m_validBinData, m_validBinDataSizes);
	const uint nBinsValid = uint(m_mapValidBinToBin.size());
	if(nBinsValid < m)
		return this->RunLosac(data, model, inliers, verbose);

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
			if(!DrawNonMinimalSample(data, m_validBinInliersBest, this->m_nonMinimalSample, this->m_idxs, this->m_marks))
			{
				inner = false;
				innerIter = 0;
				continue;
			}
			this->GenerateModels(this->m_nonMinimalSample, this->m_models);
		}
		else
		{
			DrawMinimalSample(data, m_validBinData, this->m_minimalSample);
			this->GenerateModels(this->m_minimalSample, this->m_models);
		}
		const uint nModels = this->m_models.Size();
		if(nModels == 0)
			continue;
		for(uint iModel = 0; iModel < nModels; ++iModel)
		{
			this->VerifyModel(data, this->m_models[iModel], this->m_inliers, fitErr);
			nInliers = Index(this->m_inliers.size());
			if(nInliers < m)
				continue;
			ConvertInliersListToValidBin(nBinsValid, this->m_inliers, m_mapDataToValidBin, m_validBinInliers);
			confidence = EstimateConfidence(m_validBinDataSizes, m_validBinInliers, m_mapValidBinToBin, m_validBinConfidences);
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
				m_validBinInliersBest = m_validBinInliers;

				epsilon = float(nInliersMax) / N;
				nIters = uint(ceil(logEta / log(1 - this->epsilon_exp_m(epsilon))));
				if(nIters > this->m_ransacMaxNumIters || nInliers < nInliersMin)
					nIters = this->m_ransacMaxNumIters;
				else if(nIters < this->m_ransacMinNumIters)
					nIters = this->m_ransacMinNumIters;
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
		this->SolveByAllInliers(data, model, inliers, fitErrMin, verbose);
	if(this->m_optimizeByAllInliers)
		this->OptimizeByAllInliers(data, model, inliers, fitErrMin, verbose);

	if(verbose >= 1)
		printf("  %d iterations, %d / %d inliers, error = %e\n", iter, inliers.size(), data.Size(), fitErrMin / inliers.size());

	return iter;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void EstimatorArsac<ESTIMATOR_TEMPLATE_ARGUMENT>::SetBinSize(const ushort &nBinsX, const ushort &nBinsY)
{
	if(m_nBinsX == nBinsX && m_nBinsY == nBinsY)
		return;
	m_nBinsX = nBinsX;
	m_nBinsY = nBinsY;
	m_nBins = uint(nBinsX) * uint(nBinsY);
	if(!m_iBinsX.empty())
		CreateBucket(ushort(m_iBinsX.size() - 1), ushort(m_iBinsY.size() - 1));
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void EstimatorArsac<ESTIMATOR_TEMPLATE_ARGUMENT>::SetImageSize(const ushort &width, const ushort &height)
{
	if(m_iBinsX.size() == width + 1 && m_iBinsY.size() == height + 1)
		return;
	CreateBucket(width, height);
	m_factorSqrtDetCovToImgRatio = PI / (uint(width) * uint(height));
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void EstimatorArsac<ESTIMATOR_TEMPLATE_ARGUMENT>::CreateBucket(const ushort width, const ushort height)
{
	m_iBinsX.resize(width + 1);
	const float binWidth = float(width) / m_nBinsX;
	float x1 = 0, x2 = binWidth;
	ushort x1i = 0, x2i = ushort(x2 + 0.5f);
	for(ushort i = 0; i < m_nBinsX; ++i, x1 = x2, x2 += binWidth, x1i = x2i, x2i = ushort(x2 + 0.5f))
		for(ushort x = x1i; x < x2i; ++x)
			m_iBinsX[x] = i;

	m_iBinsY.resize(height + 1);
	const float binHeight = float(height) / m_nBinsY;
	float y1 = 0, y2 = binHeight;
	ushort y1i = 0, y2i = ushort(y2 + 0.5f);
	int iBin = 0;
	for(uint i = 0; i < m_nBinsY; ++i, iBin += m_nBinsX, y1 = y2, y2 += binHeight, y1i = y2i, y2i = ushort(y2 + 0.5f))
		for(ushort y = y1i; y < y2i; ++y)
			m_iBinsY[y] = iBin;

	m_binLocations.resize(m_nBins);
	iBin = 0;
	float y = binHeight * 0.5f;
	for(ushort i = 0; i < m_nBinsY; ++i, y += binHeight)
	{
		float x = binWidth * 0.5f;
		for(ushort j = 0; j < m_nBinsX; ++j, ++iBin, x += binWidth)
			m_binLocations[iBin].Set(x, y);
	}
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void EstimatorArsac<ESTIMATOR_TEMPLATE_ARGUMENT>::BucketData(const Data &data, std::vector<uint> &mapBinToValidBin, std::vector<uint> &mapValidBinToBin, 
																	std::vector<uint> &mapDataToValidBin, std::vector<std::vector<Index> > &validBinData, 
																	std::vector<Index> &validBinDataSizes) const
{
	mapBinToValidBin.assign(m_nBins, UINT_MAX);
	mapValidBinToBin.resize(0);
	const Index N = data.Size();
	mapDataToValidBin.resize(N);
	validBinData.resize(0);
	validBinDataSizes.resize(0);
	for(Index i = 0; i < N; ++i)
	{
		const Point2D &x = data.GetImageLocation(i);
		const uint iBin = m_iBinsY[size_t(x.y() + 0.5f)] + m_iBinsX[size_t(x.x() + 0.5f)];
		const uint iBinValid = mapBinToValidBin[iBin];
		if(iBinValid == UINT_MAX)
		{
			mapBinToValidBin[iBin] = mapDataToValidBin[i] = uint(mapValidBinToBin.size());
			mapValidBinToBin.push_back(iBin);
			validBinData.push_back(std::vector<Index>(1, i));
			validBinDataSizes.push_back(1);
		}
		else
		{
			mapDataToValidBin[i] = iBinValid;
			validBinData[iBinValid].push_back(i);
			++validBinDataSizes[iBinValid];
		}
	}
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline bool EstimatorArsac<ESTIMATOR_TEMPLATE_ARGUMENT>::DrawNonMinimalSample(const Data &data, const std::vector<std::vector<Index> > &validBinInliers, NonMinimalSample &sample, 
																			  std::vector<Index> &sampleIdxs, std::vector<bool> &sampleMasks/*, std::vector<uint> &iBinsValidSampled*/) const
{
	Index idx;
	sampleIdxs.resize(0);
	//iBinsValidSampled.resize(0);
	const Index nInliersMin = Index(1 / this->m_losacInnerSampleRatio);
	const uint nBinsValid = uint(validBinInliers.size());
	for(uint iBinValid = 0; iBinValid < nBinsValid; ++iBinValid)
	{
		const std::vector<Index> &inliers = validBinInliers[iBinValid];
		const Index nInliers = Index(inliers.size());
		if(nInliers < nInliersMin)
			continue;
		const Index sampleSz = Index(nInliers * this->m_losacInnerSampleRatio + 0.5f);
		for(Index i = 0; i < sampleSz; i++)
		{
			while(sampleMasks[(idx = inliers[Random::Generate(nInliers)])]);
			sampleMasks[idx] = true;
			sampleIdxs.push_back(idx);
		}
		//iBinsValidSampled.push_back(iBinValid);
	}

	const Index sampleSz = Index(sampleIdxs.size());
	for(Index i = 0; i < sampleSz; i++)
		sampleMasks[sampleIdxs[i]] = false;
	if(sampleSz < this->MinimalSampleSize())
		return false;
	data.GetSubset(sampleIdxs, sample);
	return true;
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void EstimatorArsac<ESTIMATOR_TEMPLATE_ARGUMENT>::ConvertInliersListToValidBin(const uint nBinsValid, const std::vector<Index> &inliers, 
																					  const std::vector<uint> &mapDataToValidBin, 
																					  std::vector<std::vector<Index> > &validBinInliers) const
{
	validBinInliers.resize(nBinsValid);
	for(uint iBinValid = 0; iBinValid < nBinsValid; ++iBinValid)
		validBinInliers[iBinValid].resize(0);
	const Index nInliers = Index(inliers.size());
	for(Index i = 0; i < nInliers; ++i)
	{
		const Index idx = inliers[i];
		validBinInliers[mapDataToValidBin[idx]].push_back(idx);
	}
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline void EstimatorArsac<ESTIMATOR_TEMPLATE_ARGUMENT>::ConvertInliersValidBinToList(const std::vector<std::vector<Index> > &validBinInliers, std::vector<Index> &inliers) const
{
	inliers.resize(0);
	const uint nBinsValid = uint(validBinInliers.size());
	for(uint iBinValid = 0; iBinValid < nBinsValid; ++iBinValid)
	{
		const std::vector<Index> &src = validBinInliers[iBinValid];
		inliers.insert(inliers.end(), src.begin(), src.end());
	}
}

template<ESTIMATOR_TEMPLATE_PARAMETER>
inline float EstimatorArsac<ESTIMATOR_TEMPLATE_ARGUMENT>::EstimateConfidence(const std::vector<Index> &validBinDataSizes, const std::vector<std::vector<Index> > &validBinInliers, 
																			 const std::vector<uint> &mapValidBinToBin, std::vector<float> &validBinConfidences) const
{
	Point2D sum(0, 0);
	float validBinConfidenceSum = 0, validBinConfidenceSqSum = 0;

	const uint nBinsValid = uint(validBinDataSizes.size());
	validBinConfidences.resize(nBinsValid);
	Point2D x;
	for(uint iBinValid = 0; iBinValid < nBinsValid; ++iBinValid)
	{
		const float validBinConfidence = float(validBinInliers[iBinValid].size()) / validBinDataSizes[iBinValid];
		validBinConfidences[iBinValid] = validBinConfidence;
		x = m_binLocations[mapValidBinToBin[iBinValid]];
		x *= validBinConfidence;
		sum += x;
		validBinConfidenceSum += validBinConfidence;
		validBinConfidenceSqSum += validBinConfidence * validBinConfidence;
	}
	float norm = 1 / validBinConfidenceSum;
	Point2D &mean = sum;
	mean *= norm;

	Point2D dx;
	float Cxx = 0, Cxy = 0, Cyy = 0;
	for(uint iBinValid = 0; iBinValid < nBinsValid; ++iBinValid)
	{
		const float validBinConfidence = validBinConfidences[iBinValid];
		x = m_binLocations[mapValidBinToBin[iBinValid]];
		dx.Set(x.x() - mean.x(), x.y() - mean.y());
		Cxx += (dx.x() * dx.x()) * validBinConfidence;
		Cxy += (dx.x() * dx.y()) * validBinConfidence;
		Cyy += (dx.y() * dx.y()) * validBinConfidence;
	}
	norm = validBinConfidenceSum / (validBinConfidenceSum * validBinConfidenceSum - validBinConfidenceSqSum);
	const float imgRatio =  norm * sqrt(Cxx * Cyy - Cxy * Cxy) * m_factorSqrtDetCovToImgRatio;
	return imgRatio * validBinConfidenceSum;
}