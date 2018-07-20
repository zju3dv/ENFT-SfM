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

template<BA_TEMPLATE_PARAMETER>
BAResult BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Run(BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> &data, const CameraIndex nCamsFix, const bool global, 
														   const ubyte verbose, FILE *fp)
{
	Initialize(data, nCamsFix, global, verbose, fp);
	data.NormalizeData(m_dataNormalizeMedian);
	const BAResult res = RunLM();
	data.DenormalizeData();
	if(verbose > 0)
		PrintLMResult(res);
	return res;
}

template<BA_TEMPLATE_PARAMETER>
void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::PrintLMResult(const BAResult &res)
{
	switch(res)
	{
	case BA_ENOUGH_ITERATIONS:
		if(m_fp)
			fprintf(m_fp, "Stop by BA_ENOUGH_ITERATIONS\n");
		else
			printf("Stop by BA_ENOUGH_ITERATIONS\n");
		break;
	case BA_NORMAL_EQUATION_FAIL:
		if(m_fp)
			fprintf(m_fp, "Stop by BA_NORMAL_EQUATION_FAIL\n");
		else
			printf("Stop by BA_NORMAL_EQUATION_FAIL\n");
		break;
	case BA_SMALL_DELTA:
		if(m_fp)
			fprintf(m_fp, "Stop by BA_SMALL_DELTA\n");
		else
			printf("Stop by BA_SMALL_DELTA\n");
		break;
	case BA_INFINIT_SSE:
		if(m_fp)
			fprintf(m_fp, "Stop by BA_INFINIT_SSE\n");
		else
			printf("Stop by BA_INFINIT_SSE\n");
		break;
	case BA_SMALL_MSE:
		if(m_fp)
			fprintf(m_fp, "Stop by BA_SMALL_MSE\n");
		else
			printf("Stop by BA_SMALL_MSE\n");
		break;
	case BA_SMALL_RELATIVE_REDUCTION:
		if(m_fp)
			fprintf(m_fp, "Stop by BA_SMALL_RELATIVE_REDUCTION\n");
		else
			printf("Stop by BA_SMALL_RELATIVE_REDUCTION\n");
		break;
	case BA_SMALL_JTE:
		if(m_fp)
			fprintf(m_fp, "Stop by BA_SMALL_JTE\n");
		else
			printf("Stop by BA_SMALL_JTE\n");
		break;
	}
}

template<BA_TEMPLATE_PARAMETER>
void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Initialize(BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> &data, const CameraIndex nCamsFix, 
															  const bool global, const ubyte verbose, FILE *fp)
{
	m_pData = &data;
	m_nCamsFix = nCamsFix;
	m_verbose = verbose;
	m_fp = fp;
	const CameraIndex nCams = m_pData->GetCamerasNumber(), nCamsAdj = nCams - nCamsFix;
	const PointIndex nPts = m_pData->GetPointsNumber();
	const MeasurementIndex nMeas = m_pData->GetMeasurementsNumber();

	m_iBlockTableCC.Initialize(nCamsAdj);
	m_mapCamToBlockCX.assign(nCams + 1, UINT_MAX);
	m_iMeasOriToNew.resize(nMeas);

	CameraIndex iCam;
	PointIndex iPt;
	MeasurementIndex iMea;
	FeatureIndex i;
	uint iBlockCX;

	for(iCam = 0, iMea = 0; iCam < nCamsFix; ++iCam)
	{
		const PointIndex *iPts = m_pData->GetCameraPointIndexes(iCam);
		const FeatureIndex nFtrs = m_pData->GetCameraFeaturesNumber(iCam);
		m_iPtFtrs.resize(nFtrs);
		for(i = 0; i < nFtrs; ++i)
			m_iPtFtrs[i].Set(iPts[i], i);
		std::sort(m_iPtFtrs.begin(), m_iPtFtrs.end());
		MeasurementIndex *iMeas = m_iMeasOriToNew.data() + m_pData->GetCameraFirstMeasurementIndex(iCam);
		for(i = 0; i < nFtrs; ++i)
			iMeas[m_iPtFtrs[i].GetFeatureIndex()] = iMea++;
	}
	for(iBlockCX = 0; iCam < nCams; ++iCam)
	{
		const PointIndex *iPts = m_pData->GetCameraPointIndexes(iCam);
		const FeatureIndex nFtrs = m_pData->GetCameraFeaturesNumber(iCam);
		m_iPtFtrs.resize(nFtrs);
		for(i = 0; i < nFtrs; ++i)
			m_iPtFtrs[i].Set(iPts[i], i);
		std::sort(m_iPtFtrs.begin(), m_iPtFtrs.end());
		m_mapCamToBlockCX[iCam] = iBlockCX;
		MeasurementIndex *iMeas = m_iMeasOriToNew.data() + m_pData->GetCameraFirstMeasurementIndex(iCam);
		for(i = 0, iPt = nPts; i < nFtrs; ++i)
		{
			iMeas[m_iPtFtrs[i].GetFeatureIndex()] = iMea++;
			if(m_iPtFtrs[i].GetPointIndex() == iPt)
				continue;
			iPt = m_iPtFtrs[i].GetPointIndex();
			++iBlockCX;
		}
	}
	m_mapCamToBlockCX[iCam] = iBlockCX;
	m_pData->ReorderMeasurements(m_iMeasOriToNew);

	CameraIndex iRow, iCol, iCamRow, iCamCol;
	for(iRow = 0; iRow < nCamsAdj; ++iRow)
	{
		m_iBlockTableCC.PushBackBlock(iRow, iRow);
		m_ptMarks.assign(nPts, false);
		iCamRow = iRow + m_nCamsFix;
		const PointIndex *iPtsRow = m_pData->GetCameraPointIndexes(iCamRow);
		const FeatureIndex nFtrsRow = m_pData->GetCameraFeaturesNumber(iCamRow);
		if(nFtrsRow == 0)
			continue;
		const PointIndex iPtRowStart = iPtsRow[0], iPtRowEnd = iPtsRow[nFtrsRow - 1];
		for(i = 0; i < nFtrsRow; ++i)
			m_ptMarks[iPtsRow[i]] = true;

		for(iCol = iRow + 1; iCol < nCamsAdj; ++iCol)
		{
			iCamCol = iCol + m_nCamsFix;
			const PointIndex *iPtsCol = m_pData->GetCameraPointIndexes(iCamCol);;
			const FeatureIndex nFtrsCol = m_pData->GetCameraFeaturesNumber(iCamCol);
			const PointIndex iPtColStart = iPtsCol[0], iPtColEnd = iPtsCol[nFtrsCol - 1];
			if(iPtRowStart > iPtColEnd || iPtRowEnd < iPtColStart)
				continue;
			for(i = 0; i < nFtrsCol && !m_ptMarks[iPtsCol[i]]; ++i);
			if(i < nFtrsCol)
				m_iBlockTableCC.PushBackBlock(iRow, iCol);
		}
	}
	m_CsBkp.Resize(nCams);
	m_XsBkp.Resize(nPts);
	m_Dcs.Resize(nCamsAdj);
	m_Dxs.Resize(nPts);
	m_Wxcs.Resize(iBlockCX);
	m_Accs.Resize(m_iBlockTableCC.GetBlocksNumber());
	if(global)
	{
		m_Wgcs.Resize(nCamsAdj);
		m_Wgxs.Resize(nPts);
#if BA_ELIMINATE_POINTS
		m_Agcs.Resize(nCamsAdj);
#endif
	}
	m_JcTes.Resize(nCamsAdj);
	m_JxTes.Resize(nPts);
	m_scs.Resize(nCamsAdj);
	m_sxs.Resize(nPts);
	if(m_verbose > 0)
	{
		if(m_fp)
		{
			fprintf(m_fp, "----------------------------------------------------------------\n");
			fprintf(m_fp, "Cameras = %d+%d=%d, Points = %d, Blocks = %d+%d=%d\n", nCamsFix, nCamsAdj, nCams, nPts, m_iBlockTableCC.GetBlocksNumber(), 
				iBlockCX, m_iBlockTableCC.GetBlocksNumber() + iBlockCX);
		}
		else
		{
			printf("----------------------------------------------------------------\n");
			printf("Cameras = %d+%d=%d, Points = %d, Blocks = %d+%d=%d\n", nCamsFix, nCamsAdj, nCams, nPts, m_iBlockTableCC.GetBlocksNumber(), 
				iBlockCX, m_iBlockTableCC.GetBlocksNumber() + iBlockCX);
		}
	}

	if(global)
		m_pData->ValidateGlobal();
	else
		m_pData->InvalidateGlobal();
}

template<BA_TEMPLATE_PARAMETER>
template<class Block, class Parameter>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::GetDiagonal(const AlignedVector<Block> &Ds, AlignedVector<Parameter> &ds)
{
	const uint N = Ds.Size();
	ds.Resize(N);
	for(uint i = 0; i < N; ++i)
		LA::GetDiagonal(Ds[i], ds[i]);
}

template<BA_TEMPLATE_PARAMETER>
template<class Parameter, class Block>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::SetDiagonal(const AlignedVector<Parameter> &ds, AlignedVector<Block> &Ds)
{
	const uint N = Ds.Size();
	for(uint i = 0; i < N; ++i)
		LA::SetDiagonal(ds[i], Ds[i]);
}

template<BA_TEMPLATE_PARAMETER>
template<class Block>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::ScaleDiagonal(const float &lambda, AlignedVector<Block> &Ds)
{
	const uint N = Ds.Size();
	for(uint i = 0; i < N; ++i)
		LA::ScaleDiagonal(lambda, Ds[i]);
}

template<BA_TEMPLATE_PARAMETER>
template<class Parameter>
inline float BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::NormLinf(const AlignedVector<Parameter> &bs)
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

template<BA_TEMPLATE_PARAMETER>
template<class Parameter>
inline double BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::NormL2_2(const AlignedVector<Parameter> &xs)
{
	double nSum = 0;
	const uint N = xs.Size();
	for(uint i = 0; i < N; ++i)
		nSum += LA::NormL2_2(xs[i]);
	return nSum;
}

template<BA_TEMPLATE_PARAMETER>
template<class Parameter>
inline double BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::NormWL2_2(const AlignedVector<Parameter> &xs, const AlignedVector<Parameter> &ws)
{
	double nSum = 0;
	const uint N = xs.Size();
	for(uint i = 0; i < N; ++i)
		nSum += LA::NormWL2_2(xs[i], ws[i]);
	return nSum;
}

template<BA_TEMPLATE_PARAMETER>
template<class Parameter>
inline double BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Dot(const AlignedVector<Parameter> &xs, const AlignedVector<Parameter> &bs)
{
	double nSum = 0;
	const uint N = xs.Size();
	for(uint i = 0; i < N; ++i)
		nSum += LA::Dot(xs[i], bs[i]);
	return nSum;
}

template<BA_TEMPLATE_PARAMETER>
template<class Block>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::InvertSymmetricUpper(const AlignedVector<Block> &As, AlignedVector<Block> &AIs)
{
	const uint N = As.Size();
	AIs.Resize(N);
	for(uint i = 0; i < N; ++i)
		LA::InvertSymmetricUpper(As[i], AIs[i]);
}

template<BA_TEMPLATE_PARAMETER>
template<class Block, class Parameter, ubyte STAGE>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::AB(const AlignedVector<Block> &As, const AlignedVector<Parameter> &Bs, 
															 AlignedVector<Parameter> &ABs)
{
	const uint N = As.Size();
	ABs.Resize(N);
	for(uint i = 0; i < N; ++i)
	{
		LA::AB(As[i], Bs[i], ABs[i]);
		LA::SetReserve<STAGE>(ABs[i]);
	}
}

template<BA_TEMPLATE_PARAMETER>
template<class Parameter, ubyte STAGE>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::sA(const float &s, const AlignedVector<Parameter> &As, AlignedVector<Parameter> &sAs)
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

template<BA_TEMPLATE_PARAMETER>
template<class Parameter, ubyte STAGE>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::sApB(const float &s, const AlignedVector<Parameter> &As, const AlignedVector<Parameter> &Bs, 
															   AlignedVector<Parameter> &sApBs)
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

template<BA_TEMPLATE_PARAMETER>
template<class Parameter>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::AmB(const AlignedVector<Parameter> &As, const AlignedVector<Parameter> &Bs, 
															  AlignedVector<Parameter> &AmBs)
{
	const uint N = As.Size();
	AmBs.Resize(N);
	for(uint i = 0; i < N; ++i)
		LA::AmB(As[i], Bs[i], AmBs[i]);
}