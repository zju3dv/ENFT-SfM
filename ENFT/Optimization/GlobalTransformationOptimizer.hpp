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

template<GTO_TEMPLATE_PARAMETER>
inline GTOLMResult GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Run(GlobalTransformationOptimizerDataTemplate<GTO_TEMPLATE_ARGUMENT> &data, 
																					 const ubyte verbose, FILE *fp)
{
	Initialize(data, verbose, fp);
	data.NormalizeData(m_dataNormalizeMedian);
	data.CopyTransformations(m_TsBkp);
	const GTOLMResult res = RunLM();
	data.DenormalizeData();
	if(verbose > 0)
		PrintLMResult(res);
	return res;
}

template<GTO_TEMPLATE_PARAMETER>
void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::PrintDataSize()
{
	if(m_fp)
	{
		fprintf(m_fp, "----------------------------------------------------------------\n");
		fprintf(m_fp, "Maps: %d, Map Pairs: %d, Points: %d\n", m_pData->GetMapsNumber(), m_pData->GetMapPairsNumber(), m_pData->GetPointsNumber());
	}
	else
	{
		printf("----------------------------------------------------------------\n");
		printf("Maps: %d, Map Pairs: %d, Points: %d\n", m_pData->GetMapsNumber(), m_pData->GetMapPairsNumber(), m_pData->GetPointsNumber());
	}
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::PrintLMResult(const GTOLMResult &res)
{
	switch(res)
	{
	case GTO_LM_ENOUGH_ITERATIONS:
		if(m_fp)
			fprintf(m_fp, "Stop by GTO_LM_ENOUGH_ITERATIONS\n");
		else
			printf("Stop by GTO_LM_ENOUGH_ITERATIONS\n");
		break;
	case GTO_LM_NORMAL_EQUATION_FAIL:
		if(m_fp)
			fprintf(m_fp, "Stop by GTO_LM_NORMAL_EQUATION_FAIL\n");
		else
			printf("Stop by GTO_LM_NORMAL_EQUATION_FAIL\n");
		break;
	case GTO_LM_SMALL_DELTA:
		if(m_fp)
			fprintf(m_fp, "Stop by GTO_LM_SMALL_DELTA\n");
		else
			printf("Stop by GTO_LM_SMALL_DELTA\n");
		break;
	case GTO_LM_INFINIT_SSE:
		if(m_fp)
			fprintf(m_fp, "Stop by GTO_LM_INFINIT_SSE\n");
		else
			printf("Stop by GTO_LM_INFINIT_SSE\n");
		break;
	case GTO_LM_SMALL_MSE:
		if(m_fp)
			fprintf(m_fp, "Stop by GTO_LM_SMALL_MSE\n");
		else
			printf("Stop by GTO_LM_SMALL_MSE\n");
		break;
	case GTO_LM_SMALL_RELATIVE_REDUCTION:
		if(m_fp)
			fprintf(m_fp, "Stop by GTO_LM_SMALL_RELATIVE_REDUCTION\n");
		else
			printf("Stop by GTO_LM_SMALL_RELATIVE_REDUCTION\n");
		break;
	case GTO_LM_SMALL_JTE:
		if(m_fp)
			fprintf(m_fp, "Stop by GTO_LM_SMALL_JTE\n");
		else
			printf("Stop by GTO_LM_SMALL_JTE\n");
		break;
	}
}

template<GTO_TEMPLATE_PARAMETER>
void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Initialize(GlobalTransformationOptimizerDataTemplate<GTO_TEMPLATE_ARGUMENT> &data, 
																			  const ubyte verbose, FILE *fp)
{
	m_pData = &data;
	m_verbose = verbose;
	m_fp = fp;

	const MapIndex nMaps = m_pData->GetMapsNumber();
	m_TsBkp.Resize(nMaps);

	const uint nBlocks = m_pData->GetBlocksNumber();
	m_As.Resize(nBlocks);
	m_Ms.Resize(nMaps);

	m_ss.Resize(nMaps);
	m_ds.Resize(nMaps);
	m_ps.Resize(nMaps);
	m_xs.Resize(nMaps);
	m_zs.Resize(nMaps);
	m_rs.Resize(nMaps);
	m_Aps.Set(m_zs.Data(), nMaps);
	m_bs.Set(m_rs.Data(), nMaps);
	//m_bs.Resize(nMaps);

	m_idxsList.resize(nMaps);
	for(MapIndex iMap = 0; iMap < nMaps; ++iMap)
		m_idxsList[iMap].resize(0);

	MapIndex iMap1, iMap2;
	uint iBlock;
	const uint nMapPairs = m_pData->GetMapPairsNumber();
	for(uint i = 0; i < nMapPairs; ++i)
	{
		m_pData->GetMapPair(i).GetIndexes(iMap1, iMap2, iBlock);
		m_idxsList[iMap2].push_back(std::make_pair(iMap1, iBlock));
	}
	for(MapIndex iMap = 0; iMap < nMaps; ++iMap)
		m_idxsList[iMap].push_back(std::make_pair(iMap, iMap));
	for(uint i = 0; i < nMapPairs; ++i)
	{
		m_pData->GetMapPair(i).GetIndexes(iMap1, iMap2, iBlock);
		m_idxsList[iMap1].push_back(std::make_pair(iMap2, iBlock));
	}

	if(m_verbose > 0)
		PrintDataSize();
}

template<GTO_TEMPLATE_PARAMETER>
template<ubyte STAGE>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::ComputeAx(const AlignedVector<TransformationParameter> &xs, 
																					AlignedVector<TransformationParameter> &Axs)
{
	Axs.SetZero();
	ENFT_SSE::__m128 work[1];
	MapIndex iMap1, iMap2, i;
	const MapIndex nMaps = m_pData->GetMapsNumber();
	for(iMap1 = 0; iMap1 < nMaps; ++iMap1)
	{
		TransformationParameter &Ax = Axs[iMap1];
		const std::vector<std::pair<MapIndex, uint> > &idxs = m_idxsList[iMap1];
		const MapIndex nMaps2 = MapIndex(idxs.size());
		for(i = 0; i < nMaps2 && (iMap2 = idxs[i].first) < iMap1; ++i)
			LA::AddATBTo(m_As[idxs[i].second], xs[iMap2], Ax, work);
		for(; i < nMaps2; ++i)
			LA::AddABTo(m_As[idxs[i].second], xs[idxs[i].first], Ax);
		LA::SetReserve<STAGE>(Ax);
	}
}

template<GTO_TEMPLATE_PARAMETER>
template<ubyte STAGE>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::ComputeMx(const AlignedVector<TransformationParameter> &xs, 
																					AlignedVector<TransformationParameter> &Mxs)
{
	const MapIndex nMaps = m_pData->GetMapsNumber();
	for(MapIndex iMap = 0; iMap < nMaps; ++iMap)
	{
		//Mxs[iMap] = xs[iMap];
		LA::AB(m_Ms[iMap], xs[iMap], Mxs[iMap]);
		LA::SetReserve<STAGE>(Mxs[iMap]);
	}
}

template<GTO_TEMPLATE_PARAMETER>
inline float GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::NormLinf(const AlignedVector<TransformationParameter> &bs)
{
	float n, nMax = 0;
	const uint N = bs.Size();
	for(uint i = 0; i < N; ++i)
	{
		if((n = LA::NormLinf(bs[i])) > nMax)
			nMax = n;
	}
	return nMax;
}

template<GTO_TEMPLATE_PARAMETER>
inline double GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::NormL2_2(const AlignedVector<TransformationParameter> &xs)
{
	double nSum = 0;
	const uint N = xs.Size();
	for(uint i = 0; i < N; ++i)
		nSum += LA::NormL2_2(xs[i]);
	return nSum;
}

template<GTO_TEMPLATE_PARAMETER>
inline double GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::NormWL2_2(const AlignedVector<TransformationParameter> &xs, 
																					  const AlignedVector<TransformationParameter> &ws)
{
	double nSum = 0;
	const uint N = xs.Size();
	for(uint i = 0; i < N; ++i)
		nSum += LA::NormWL2_2(xs[i], ws[i]);
	return nSum;
}

template<GTO_TEMPLATE_PARAMETER>
inline double GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Dot(const AlignedVector<TransformationParameter> &xs, 
																				const AlignedVector<TransformationParameter> &bs)
{
	double nSum = 0;
	const uint N = xs.Size();
	for(uint i = 0; i < N; ++i)
		nSum += LA::Dot(xs[i], bs[i]);
	return nSum;
}

template<GTO_TEMPLATE_PARAMETER>
template<ubyte STAGE>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::sA(const float &s, const AlignedVector<TransformationParameter> &As, 
																			 AlignedVector<TransformationParameter> &sAs)
{
	const ENFT_SSE::__m128 s4 = ENFT_SSE::_mm_set1_ps(s);
	const uint N = As.Size();
	sAs.Resize(N);
	for(uint i = 0; i < N; ++i)
	{
		LA::sA(s4, As[i], sAs[i]);
		LA::SetReserve<STAGE>(sAs[i]);
	}
}

template<GTO_TEMPLATE_PARAMETER>
template<ubyte STAGE>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::sApB(const float &s, const AlignedVector<TransformationParameter> &As, 
																			   const AlignedVector<TransformationParameter> &Bs, 
																			   AlignedVector<TransformationParameter> &sApBs)
{
	const ENFT_SSE::__m128 s4 = ENFT_SSE::_mm_set1_ps(s);
	const uint N = As.Size();
	sApBs.Resize(N);
	for(uint i = 0; i < N; ++i)
	{
		LA::sApB(s4, As[i], Bs[i], sApBs[i]);
		LA::SetReserve<STAGE>(sApBs[i]);
	}
}