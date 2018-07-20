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
inline uint BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::RunPCG()
{
	Compute_M();
	Compute_r0_b();
	Compute_z0p0_Mr0();

	double rTzPre, rTzCur = Compute_r0Tz0();
	double rTzMin = m_pcgStopRTZRatioMin * fabs(rTzCur), rTzMax = m_pcgStopRTZRatioMax * fabs(rTzCur);
	Compute_Ap();
	float a = float(rTzCur / Compute_pTAp());
	//printf("%f, %f\n", rTzCur, a);
	if(!std::isfinite(a))
		return rTzCur < rTzMin && rTzCur > -rTzMin ? 1 : 0;
	//const double rTzInit = rTzCur;

	Compute_x1_ap0(a);
	Update_r_rmaAp(a);

	uint k = 1;
	bool scc;
	while(true)
	{
		Compute_z_Mr();

		rTzPre = rTzCur;
		rTzCur = Compute_rTz();
		if(rTzCur == 0.0f || rTzCur <= rTzMin && rTzCur >= -rTzMin && k >= m_pcgMinNumIters)
		{
			scc = true;
			break;
		}
		else if(rTzCur > rTzMax || rTzCur < -rTzMax)
		{
			scc = true;
			//scc = false;
			break;
		}

		float b = float(rTzCur / rTzPre);
		Update_p_zpbp(b);

		Compute_Ap();
		a = float(rTzCur / Compute_pTAp());
		//printf("%f, %f, %f\n", rTzCur, a, b);

		if(!std::isfinite(a))
		{
			//scc = false;
			//scc = rTzCur < rTzMin && rTzCur > -rTzMin;
			scc = true;
			break;
		}

		Update_x_xpap(a);
		if(++k >= m_pcgMaxNumIters)
		{
			scc = true;
			break;
		}

		Compute_e_Axmb();
//#if BA_ELIMINATE_POINTS
//		printf("%d: %f (%f, %f)\n", k, rTzCur, NormL2_2(m_ecs), LA::NormL2_2(m_eg));
//#else
//		printf("%d: %f (%f, %f, %f)\n", k, rTzCur, NormL2_2(m_ecs), NormL2_2(m_exs), LA::NormL2_2(m_eg));
//#endif

		Update_r_rmaAp(a);
	}

	if(!scc)
		k = 0;
	return k;
}

template<BA_TEMPLATE_PARAMETER>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Compute_M()
{
#if BA_ELIMINATE_POINTS
//	CameraIndex iRow, iCol;
//	uint iBlock;
//	float work[49];
//	const CameraIndex nRows = m_iBlockTableCC.GetRowsNumber();
//	m_Mcs.Resize(nRows);
//	for(iRow = 0; iRow < nRows; ++iRow)
//	{
//		m_iBlockTableCC.GetRow(iRow).GetUpperBlockIndexes().front().Get(iCol, iBlock);
//#if _DEBUG
//		assert(iRow == iCol);
//#endif
//		LA::InvertSymmetricUpper(m_Acs[iBlock], m_Mcs[iRow], work);
//	}
	float work[49];
	const uint N = m_Dcs.Size();
	m_Mcs.Resize(N);
	for(uint i = 0; i < N; ++i)
		LA::InvertSymmetricUpper(m_Dcs[i], m_Mcs[i], work);
	//LA::InvertSymmetricUpper(m_Ag, m_Mg, work);
	LA::InvertSymmetricUpper(m_Dg, m_Mg, work);
#else
	float work[49];
	const uint N = m_Dcs.Size();
	m_Mcs.Resize(N);
	for(uint i = 0; i < N; ++i)
		LA::InvertSymmetricUpper(m_Dcs[i], m_Mcs[i], work);
	InvertSymmetricUpper(m_Dxs, m_Mxs);
	LA::InvertSymmetricUpper(m_Dg, m_Mg, work);
#endif
}

template<BA_TEMPLATE_PARAMETER>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Compute_Ap()
{
	const bool global = m_pData->IsGlobalValid();
	const CameraIndex nRows = m_iBlockTableCC.GetRowsNumber();
	m_Apcs.Resize(nRows);
	m_Apcs.SetZero();
#if BA_ELIMINATE_POINTS
	m_Apg.SetZero();
	CameraIndex iRow, iCol, i;
	uint iBlock;
	ENFT_SSE::__m128 work[2];
	for(iRow = 0; iRow < nRows; ++iRow)
	{
		CameraParameter &Apc = m_Apcs[iRow];
		const std::vector<typename CameraBlockIndexTable::Index> &idxsLower = m_iBlockTableCC.GetRow(iRow).GetLowerBlockIndexes();
		const CameraIndex nColsLower = CameraIndex(idxsLower.size());
		for(i = 0; i < nColsLower; ++i)
		{
			idxsLower[i].Get(iCol, iBlock);
			LA::AddATBTo(m_Accs[iBlock], m_pcs[iCol], Apc, work);
		}
		const std::vector<typename CameraBlockIndexTable::Index> &idxsUpper = m_iBlockTableCC.GetRow(iRow).GetUpperBlockIndexes();
		const CameraIndex nColsUpper = CameraIndex(idxsUpper.size());
		for(i = 0; i < nColsUpper; ++i)
		{
			idxsUpper[i].Get(iCol, iBlock);
			LA::AddABTo(m_Accs[iBlock], m_pcs[iCol], Apc, work);
		}
		if(global)
		{
			LA::AddATBTo(m_Agcs[iRow], m_pg, Apc, work);
			LA::AddABTo(m_Agcs[iRow], m_pcs[iRow], m_Apg, work);
			LA::FinishAdditionATBTo<GlobalCameraBlock>(Apc);
		}
	}
	if(global)
		LA::AddABTo(m_Agg, m_pg, m_Apg, work);
#if _DEBUG
	m_pData->DebugAx(m_pcs, m_pg, m_Apcs, m_Apg);
#endif
#endif
}

template<BA_TEMPLATE_PARAMETER>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Compute_r0_b()
{
	m_rcs = m_bcs;
#if BA_ELIMINATE_POINTS == 0
	m_rxs = m_bxs;
#endif
	if(m_pData->IsGlobalValid())
		m_rg = m_bg;
}

template<BA_TEMPLATE_PARAMETER>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Compute_z0p0_Mr0()
{
	AB<CameraBlock, CameraParameter, BA_STAGE_P>(m_Mcs, m_rcs, m_pcs);
#if BA_ELIMINATE_POINTS == 0
	AB< PointBlock,  PointParameter, BA_STAGE_P>(m_Mxs, m_rxs, m_pxs);
#endif
	if(m_pData->IsGlobalValid())
	{
		LA::AB(m_Mg, m_rg, m_pg);
		LA::SetReserve<BA_STAGE_P>(m_pg);
	}
}

template<BA_TEMPLATE_PARAMETER>
inline double BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Compute_r0Tz0()
{
	double dot = Dot(m_rcs, m_pcs);
#if BA_ELIMINATE_POINTS == 0
	dot += Dot(m_rxs, m_pxs);
#endif
	if(m_pData->IsGlobalValid())
		dot += LA::Dot(m_rg, m_pg);
	return dot;
}

template<BA_TEMPLATE_PARAMETER>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Compute_x1_ap0(const float &a)
{
	sA<CameraParameter, BA_STAGE_X>(a, m_pcs, m_xcs);
#if BA_ELIMINATE_POINTS == 0
	sA< PointParameter, BA_STAGE_X>(a, m_pxs, m_xxs);
#endif
	if(m_pData->IsGlobalValid())
	{
		LA::sA(a, m_pg, m_xg);
		LA::SetReserve<BA_STAGE_X>(m_xg);
	}
}

template<BA_TEMPLATE_PARAMETER>
inline double BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Compute_pTAp()
{
	double dot = Dot(m_pcs, m_Apcs);
#if BA_ELIMINATE_POINTS == 0
	dot += Dot(m_pxs, m_Apxs);
#endif
	if(m_pData->IsGlobalValid())
		dot += LA::Dot(m_pg, m_Apg);
	return dot;
}

template<BA_TEMPLATE_PARAMETER>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Compute_z_Mr()
{
	AB<CameraBlock, CameraParameter, BA_STAGE_Z>(m_Mcs, m_rcs, m_zcs);
#if BA_ELIMINATE_POINTS == 0
	AB< PointBlock,  PointParameter, BA_STAGE_Z>(m_Mxs, m_rxs, m_zxs);
#endif
	if(m_pData->IsGlobalValid())
		LA::AB(m_Mg, m_rg, m_zg);
}

template<BA_TEMPLATE_PARAMETER>
inline double BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Compute_rTz()
{
	double dot = Dot(m_rcs, m_zcs);
#if BA_ELIMINATE_POINTS == 0
	dot += Dot(m_rxs, m_zxs);
#endif
	if(m_pData->IsGlobalValid())
		dot += LA::Dot(m_rg, m_zg);
	return dot;
}

template<BA_TEMPLATE_PARAMETER>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Update_p_zpbp(const float &b)
{
	sApB<CameraParameter, BA_STAGE_P>(b, m_pcs, m_zcs, m_pcs);
#if BA_ELIMINATE_POINTS == 0
	sApB< PointParameter, BA_STAGE_P>(b, m_pxs, m_zxs, m_pxs);
#endif
	if(m_pData->IsGlobalValid())
	{
		LA::sApB(b, m_pg, m_zg, m_pg);
		LA::SetReserve<BA_STAGE_P>(m_pg);
	}
}

template<BA_TEMPLATE_PARAMETER>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Update_x_xpap(const float &a)
{
	sApB<CameraParameter, BA_STAGE_X>(a, m_pcs, m_xcs, m_xcs);
#if BA_ELIMINATE_POINTS == 0
	sApB< PointParameter, BA_STAGE_X>(a, m_pxs, m_xxs, m_xxs);
#endif
	if(m_pData->IsGlobalValid())
	{
		LA::sApB(a, m_pg, m_xg, m_xg);
		LA::SetReserve<BA_STAGE_X>(m_xg);
	}
}

template<BA_TEMPLATE_PARAMETER>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Update_r_rmaAp(const float &a)
{
	sApB<CameraParameter, BA_STAGE_R>(-a, m_Apcs, m_rcs, m_rcs);
#if BA_ELIMINATE_POINTS == 0
	sApB< PointParameter, BA_STAGE_R>(-a, m_Apxs, m_rxs, m_rxs);
#endif
	if(m_pData->IsGlobalValid())
	{
		LA::sApB(-a, m_Apg, m_rg, m_rg);
		LA::SetReserve<BA_STAGE_R>(m_rg);
	}
}

template<BA_TEMPLATE_PARAMETER>
inline void BundleAdjustorTemplate<BA_TEMPLATE_ARGUMENT>::Compute_e_Axmb()
{
	const bool global = m_pData->IsGlobalValid();
	const CameraIndex nRows = m_iBlockTableCC.GetRowsNumber();
	m_Axcs.Resize(nRows);
	m_Axcs.SetZero();
#if BA_ELIMINATE_POINTS
	if(global)
		m_Axg.SetZero();
	CameraIndex iRow, iCol, i;
	uint iBlock;
	ENFT_SSE::__m128 work[2];
	for(iRow = 0; iRow < nRows; ++iRow)
	{
		CameraParameter &Axc = m_Axcs[iRow];
		const std::vector<typename CameraBlockIndexTable::Index> &idxsLower = m_iBlockTableCC.GetRow(iRow).GetLowerBlockIndexes();
		const CameraIndex nColsLower = CameraIndex(idxsLower.size());
		for(i = 0; i < nColsLower; ++i)
		{
			idxsLower[i].Get(iCol, iBlock);
			LA::AddATBTo(m_Accs[iBlock], m_xcs[iCol], Axc, work);
		}
		const std::vector<typename CameraBlockIndexTable::Index> &idxsUpper = m_iBlockTableCC.GetRow(iRow).GetUpperBlockIndexes();
		const CameraIndex nColsUpper = CameraIndex(idxsUpper.size());
		for(i = 0; i < nColsUpper; ++i)
		{
			idxsUpper[i].Get(iCol, iBlock);
			LA::AddABTo(m_Accs[iBlock], m_xcs[iCol], Axc, work);
		}
		if(global)
		{
			LA::AddATBTo(m_Agcs[iRow], m_xg, Axc, work);
			LA::AddABTo(m_Agcs[iRow], m_xcs[iRow], m_Axg, work);
			LA::FinishAdditionATBTo<GlobalCameraBlock>(Axc);
		}
	}
	if(global)
		LA::AddABTo(m_Agg, m_xg, m_Axg, work);
	AmB(m_Axcs, m_bcs, m_ecs);
	if(global)
		LA::AmB(m_Axg, m_bg, m_eg);
#endif
}