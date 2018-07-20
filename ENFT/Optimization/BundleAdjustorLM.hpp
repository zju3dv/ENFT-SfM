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
BAResult BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::RunLM()
{
	m_eCnt = m_jCnt = 0;
	m_CsBkp.CopyFrom(m_pData->GetCameras().Data());
	m_XsBkp.CopyFrom(m_pData->GetPoints().Data());
	if(m_pData->IsGlobalValid())
		m_GBkp = m_pData->GetGlobal();

	const float factorSseToMse = m_pData->GetFactorSSEToMSE();
	double sseCur, mseCur, reductionRel = 0, deltaNormL2 = 0;
	float JTeNormLinf = 0;
	sseCur = ComputeSSE();
	mseCur = sseCur * factorSseToMse;

	m_damping = m_lmDampingInit;
	float dampingAdjust = 2.0f;

	if(m_verbose == 1)
	{
		if(m_fp)
			fprintf(m_fp, "  MSE = %f", mseCur);
		else
			printf("  MSE = %f", mseCur);
	}
	else if(m_verbose > 1)
	{
		if(m_fp)
		{
			fprintf(m_fp, "   #LM   MSE  rel_reduction  ||JTe||_oo  ||dx||^2  damping   #PCG\n");
			fprintf(m_fp, "       %f\n", mseCur);
		}
		else
		{
			printf("   #LM   MSE  rel_reduction  ||JTe||_oo  ||dx||^2  damping   #PCG\n");
			printf("       %f\n", mseCur);
		}
	}

	JTeNormLinf = ConstructNormalEquation(true);

	BAResult res = BA_ENOUGH_ITERATIONS;
	uint iterLM, nItersPCG;
	for(iterLM = 0; iterLM < m_lmMaxNumIters; ++iterLM)
	{
		nItersPCG = SolveNormalEquation();
		if(nItersPCG == 0)
		{
			res = BA_NORMAL_EQUATION_FAIL;
			break;
		}

		deltaNormL2 = ComputeDeltaNormL2();
		if(deltaNormL2 < m_lmStopDeltaNormL2)
		{
			res = BA_SMALL_DELTA;
			break;
		}

		const double ssePre = sseCur;
		//sseCur = ComputeSSE();
		sseCur = UpdateSolution();
		if(!std::isfinite(sseCur))
		{
			RollBackSolution();
			res = BA_INFINIT_SSE;
			break;
		}
		mseCur = sseCur * factorSseToMse;

		const double reduction = (ssePre - sseCur);
		if(reduction > 0)
		{
			//RollBackPoints();

			if(mseCur < m_lmStopMSE)
			{
				res = BA_SMALL_MSE;
				break;
			}
			reductionRel = reduction / ssePre;
			if(reductionRel < m_lmStopRelativeReduction)
			{
				res = BA_SMALL_RELATIVE_REDUCTION;
				break;
			}

			const double gainRatio = reduction / ComputeExpectedReduction();
			double t = gainRatio + gainRatio - 1;
			const double adaptiveAdjust = 1 - t * t * t;
			t = 1 / 3.0;
			const double autoAdjust = std::max(t, adaptiveAdjust);

			m_damping = float(m_damping * autoAdjust);
			dampingAdjust = 2.0f;

			JTeNormLinf = ConstructNormalEquation(true);
			if(JTeNormLinf < m_lmStopJTeNormLinf)
			{
				res = BA_SMALL_JTE;
				break;
			}

			if(m_verbose > 1)
			{
				if(m_fp)
					fprintf(m_fp, "--> %d  %f  %f  %f  %f  %e  %d\n", iterLM, mseCur, reductionRel, JTeNormLinf, deltaNormL2, m_damping, nItersPCG);
				else
					printf("--> %d  %f  %f  %f  %f  %e  %d\n", iterLM, mseCur, reductionRel, JTeNormLinf, deltaNormL2, m_damping, nItersPCG);
			}
		}
		else
		{
			if(m_verbose > 1)
			{
				if(m_fp)
					fprintf(m_fp, "--> %d  %f\n", iterLM, mseCur);
				else
					printf("--> %d  %f\n", iterLM, mseCur);
			}
			RollBackSolution();

			sseCur = ssePre;
			mseCur = sseCur * factorSseToMse;
			m_damping = m_damping * dampingAdjust;
			dampingAdjust = dampingAdjust + dampingAdjust;

			ConstructNormalEquation(false);
			//ConstructNormalEquation(true);
		}

		//ComputeSSE();
		//printf("%f\n", ComputeSSE() * factorSseToMse);
	}

	if(m_verbose == 1)
	{
		if(m_fp)
			fprintf(m_fp, " --> %f\n", mseCur);
		else
			printf(" --> %f\n", mseCur);
	}
	else if(m_verbose > 1)
	{
		if(m_fp)
			fprintf(m_fp, "--> %d  %f  %f  %f  %f  %e  %d\n", iterLM, mseCur, reductionRel, JTeNormLinf, deltaNormL2, m_damping, nItersPCG);
		else
			printf("--> %d  %f  %f  %f  %f  %e  %d\n", iterLM, mseCur, reductionRel, JTeNormLinf, deltaNormL2, m_damping, nItersPCG);
	}
	return res;
}

template<BA_TEMPLATE_PARAMETER>
double BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::ComputeSSE()
{
	++m_eCnt;
	return m_pData->ComputeSSE(m_ptSSEs);
}

template<BA_TEMPLATE_PARAMETER>
float BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::ConstructNormalEquation(const bool recompute)
{
	const bool global = m_pData->IsGlobalValid();
	if(recompute)
	{
		if(m_jCnt == 0 || !m_fixScales)
			m_pData->ConstructNormalEquation(m_Dcs, m_Dxs, m_Dg, m_Wxcs, m_Wgcs, m_Wgxs, m_JcTes, m_JxTes, m_JgTe, m_scs, m_sxs, m_sg);
		else
			m_pData->ConstructNormalEquation(m_scs, m_sxs, m_sg, m_Dcs, m_Dxs, m_Dg, m_Wxcs, m_Wgcs, m_Wgxs, m_JcTes, m_JxTes, m_JgTe);
		GetDiagonal(m_Dcs, m_dcs);
		GetDiagonal(m_Dxs, m_dxs);
		if(global)
			LA::GetDiagonal(m_Dg, m_dg);
		++m_jCnt;
	}
	else
	{
		SetDiagonal(m_dcs, m_Dcs);
		SetDiagonal(m_dxs, m_Dxs);
		if(global)
			LA::SetDiagonal(m_dg, m_Dg);
	}
	const float lambda = m_damping + 1;
	ScaleDiagonal(lambda, m_Dcs);
	ScaleDiagonal(lambda, m_Dxs);
	if(global)
		LA::ScaleDiagonal(lambda, m_Dg);
	const float nc = NormLinf(m_JcTes);
	const float nx = NormLinf(m_JxTes);
	const float ncx = std::max(nc, nx);
	if(!global)
		return ncx;
	const float ng = LA::NormLinf(m_JgTe);
	return std::max(ncx, ng);
}

template<BA_TEMPLATE_PARAMETER>
uint BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::SolveNormalEquation()
{
	ConstructSchurComplement();
	const uint nIters = RunPCG();
	if(nIters > 0)
		BackSubstituteSchurComplement();
//#if _DEBUG
#if 0
	m_pData->DebugSolution(m_damping, m_xcs, m_xxs, m_xg);
#endif
	return nIters;
}

template<BA_TEMPLATE_PARAMETER>
void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::ConstructSchurComplement()
{
#if BA_ELIMINATE_POINTS
	m_Accs.SetZero();
	m_bcs = m_JcTes;
	InvertSymmetricUpper(m_Dxs, m_DxIs);
	
	CameraIndex iRow, iCol, iCamCol, iCamRow, i;
	PointIndex iPt;
	FeatureIndex j, k;
	uint iBlockCC;
	ENFT_SSE::__m128 work[4];
	const bool global = m_pData->IsGlobalValid();
	const PointIndex nPts = m_pData->GetPointsNumber();
	const CameraIndex nCams = m_pData->GetCamerasNumber(), nRows = nCams - m_nCamsFix;
	for(iRow = 0; iRow < nRows; ++iRow)
	{
		iCamRow = m_nCamsFix + iRow;
		CameraParameter &bc = m_bcs[iRow];
		const PointCameraBlock *WxcsRow = m_Wxcs.Data() + m_mapCamToBlockCX[iCamRow];
		const PointIndex *iPtsRow = m_pData->GetCameraPointIndexes(iCamRow);
		const FeatureIndex nFtrsRow = m_pData->GetCameraFeaturesNumber(iCamRow);
		m_mapPtToYxc.assign(nPts, nFtrsRow);
		m_Yxcs.Resize(nFtrsRow);
		for(j = k = 0, iPt = nPts; j < nFtrsRow; ++j)
		{
			if(iPtsRow[j] == iPt)
				continue;
			iPt = iPtsRow[j];
			LA::ATB(m_DxIs[iPt], WxcsRow[k], m_Yxcs[k], work);
			LA::SubtractATBFrom(m_Yxcs[k], m_JxTes[iPt], bc, work);
#if _DEBUG
			assert(m_mapPtToYxc[iPt] == nFtrsRow);
#endif
			m_mapPtToYxc[iPt] = k++;
		}
		m_Yxcs.Resize(k);
#if _DEBUG
		assert(k == m_mapCamToBlockCX[iCamRow + 1] - m_mapCamToBlockCX[iCamRow]);
#endif

		const std::vector<typename CameraBlockIndexTable::Index> &iBlocksCC = m_iBlockTableCC.GetRow(iRow).GetUpperBlockIndexes();
		const CameraIndex nCols = CameraIndex(iBlocksCC.size());
		for(i = 0; i < nCols; ++i)
		{
			iBlocksCC[i].Get(iCol, iBlockCC);
			iCamCol = m_nCamsFix + iCol;
			CameraBlock &Acc = m_Accs[iBlockCC];
			if(iCol == iRow)
				Acc = m_Dcs[iRow];
			const PointCameraBlock *WxcsCol = m_Wxcs.Data() + m_mapCamToBlockCX[iCamCol];
			const PointIndex *iPtsCol = m_pData->GetCameraPointIndexes(iCamCol);
			const FeatureIndex nFtrsCol = m_pData->GetCameraFeaturesNumber(iCamCol);
			for(j = k = 0, iPt = nPts; j < nFtrsCol; ++j)
			{
				if(iPtsCol[j] == iPt)
					continue;
				iPt = iPtsCol[j];
				if(m_mapPtToYxc[iPt] < nFtrsRow)
					LA::SubtractATBFrom(m_Yxcs[m_mapPtToYxc[iPt]], WxcsCol[k], Acc, work);
				++k;
			}
#if _DEBUG
			assert(k == m_mapCamToBlockCX[iCamCol + 1] - m_mapCamToBlockCX[iCamCol]);
#endif
		}

		if(!global)
			continue;
		GlobalCameraBlock &Agc = m_Agcs[iRow];
		Agc = m_Wgcs[iRow];
		for(j = k = 0, iPt = nPts; j < nFtrsRow; ++j)
		{
			if(iPtsRow[j] == iPt)
				continue;
			iPt = iPtsRow[j];
			LA::SubtractABFrom(m_Wgxs[iPt], m_Yxcs[k++], Agc, work);
		}
#if _DEBUG
		assert(k == FeatureIndex(m_Yxcs.Size()));
#endif
	}
	if(global)
	{
		GlobalPointBlock Ygx;
		m_Agg = m_Dg;
		m_bg = m_JgTe;
		for(iPt = 0; iPt < nPts; ++iPt)
		{
			LA::AB(m_Wgxs[iPt], m_DxIs[iPt], Ygx, work);
			LA::SubtractABTFrom(Ygx, m_Wgxs[iPt], m_Agg, work);
			LA::SubtractABFrom(Ygx, m_JxTes[iPt], m_bg, work);
		}
	}
#if _DEBUG
	std::vector<std::pair<CameraIndex, CameraIndex> > iPairs(m_iBlockTableCC.GetBlocksNumber());
	for(iRow = 0; iRow < nRows; ++iRow)
	{
		const std::vector<typename CameraBlockIndexTable::Index> &iBlocksCC = m_iBlockTableCC.GetRow(iRow).GetUpperBlockIndexes();
		const CameraIndex nCols = CameraIndex(iBlocksCC.size());
		for(i = 0; i < nCols; ++i)
		{
			iBlocksCC[i].Get(iCol, iBlockCC);
			iPairs[iBlockCC] = std::make_pair(iRow, iCol);
		}
	}
	m_pData->DebugSchurComplement(m_damping, iPairs, m_Accs, m_Agg, m_Agcs, m_bcs, m_bg);
#endif
	for(iRow = 0; iRow < nRows; ++iRow)
		LA::SetReserve<BA_STAGE_B>(m_bcs[iRow]);
#endif
}

template<BA_TEMPLATE_PARAMETER>
void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::BackSubstituteSchurComplement()
{
#if BA_ELIMINATE_POINTS
	CameraIndex iCam, i;
	ENFT_SSE::__m128 work[1];
	FeatureIndex j, k;
	PointIndex iPt;
	PointParameter xx;
	m_xxs = m_JxTes;
	const CameraIndex nCams = m_pData->GetCamerasNumber();
	const PointIndex nPts = m_pData->GetPointsNumber();
	for(iCam = m_nCamsFix, i = 0; iCam < nCams; ++iCam, ++i)
	{
		const CameraParameter &xc = m_xcs[i];
		const PointCameraBlock *Wxcs = m_Wxcs.Data() + m_mapCamToBlockCX[iCam];
		const PointIndex *iPts = m_pData->GetCameraPointIndexes(iCam);
		const FeatureIndex nFtrs = m_pData->GetCameraFeaturesNumber(iCam);
		for(j = k = 0, iPt = nPts; j < nFtrs; ++j)
		{
			if(iPts[j] == iPt)
				continue;
			iPt = iPts[j];
			LA::SubtractABFrom(Wxcs[k++], xc, m_xxs[iPt], work);
		}
#if _DEBUG
		assert(k == m_mapCamToBlockCX[iCam + 1] - m_mapCamToBlockCX[iCam]);
#endif
	}
	const bool global = m_pData->IsGlobalValid();
	if(global)
	{
		for(iPt = 0; iPt < nPts; ++iPt)
			LA::SubtractATBFrom(m_Wgxs[iPt], m_xg, m_xxs[iPt], work);
	}
	for(iPt = 0; iPt < nPts; ++iPt)
	{
		xx = m_xxs[iPt];
		LA::AB(m_DxIs[iPt], xx, m_xxs[iPt]);
	}
#endif
}

template<BA_TEMPLATE_PARAMETER>
double BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::ComputeDeltaNormL2()
{
	const double nc = NormL2_2(m_xcs);
	const double nx = NormL2_2(m_xxs);
	const double ncx = nc + nx;
	if(!m_pData->IsGlobalValid())
		return sqrt(ncx);
	const float ng = LA::NormL2_2(m_xg);
	return sqrt(ncx + ng);
}

template<BA_TEMPLATE_PARAMETER>
double BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::UpdateSolution()
{
	++m_eCnt;
	m_pData->SwapCameras(m_CsBkp);
	m_pData->UpdateCameras(m_scs, m_xcs, m_CsBkp);
	if(m_pData->IsGlobalValid())
	{
		m_GBkp = m_pData->GetGlobal();
		m_pData->UpdateGlobal(m_sg, m_xg, m_GBkp);
	}
	m_ptSSEs.swap(m_ptSSEsBkp1);
	m_pData->ComputeSSE(m_ptSSEsBkp2);
	m_pData->SwapPoints(m_XsBkp);
	m_pData->UpdatePoints(m_sxs, m_xxs, m_XsBkp);
	double SSE = m_pData->ComputeSSE(m_ptSSEs);
	const PointIndex nPts = m_pData->GetPointsNumber();
	for(PointIndex iPt = 0; iPt < nPts; ++iPt)
	{
		if(m_ptSSEs[iPt] < m_ptSSEsBkp2[iPt])
			continue;
		m_pData->SetPoint(iPt, m_XsBkp[iPt]);
		SSE -= m_ptSSEs[iPt] - m_ptSSEsBkp2[iPt];
		m_ptSSEs[iPt] = m_ptSSEsBkp2[iPt];
	}
	return SSE;
}

template<BA_TEMPLATE_PARAMETER>
void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::RollBackSolution()
{
	m_pData->SwapCameras(m_CsBkp);
	m_pData->SwapPoints(m_XsBkp);
	m_ptSSEs.swap(m_ptSSEsBkp1);
	if(m_pData->IsGlobalValid())
		m_pData->SetGlobal(m_GBkp);
}

//template<BA_TEMPLATE_PARAMETER>
//void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::RollBackPoints()
//{
//	const PointIndex nPts = m_pData->GetPointsNumber();
//	for(PointIndex iPt = 0; iPt < nPts; ++iPt)
//	{
//		if(m_ptSSEs[iPt] < m_ptSSEsBkp[iPt])
//			continue;
//		m_pData->SetPoint(iPt, m_XsBkp[iPt]);
//		//SSE -= m_ptSSEs[iPt] - m_ptSSEsBkp[iPt];
//		m_ptSSEs[iPt] = m_ptSSEsBkp[iPt];
//	}
//}

template<BA_TEMPLATE_PARAMETER>
double BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::ComputeExpectedReduction()
{
	double xTx = NormWL2_2(m_xcs, m_dcs) + NormWL2_2(m_xxs, m_dxs);
	double xTb = Dot(m_xcs, m_JcTes) + Dot(m_xxs, m_JxTes);
	if(m_pData->IsGlobalValid())
	{
		xTx += LA::NormWL2_2(m_xg, m_dg);
		xTb += LA::Dot(m_xg, m_JgTe);
	}
	return m_damping * xTx + xTb;
}