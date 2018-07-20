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
inline GTOLMResult GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::RunLM()
{
	m_eCnt = m_jCnt = 0;

	const float sseToMseFactor = 1.0f / m_pData->GetPointsNumber();
	double sseCur, mseCur, reductionRel = 0, deltaNormL2 = 0;
	float JTeNormLinf = 0;
	sseCur = ComputeTransformationSSE();
	mseCur = sseCur * sseToMseFactor;

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

	ConstructNormalEquation(true);
	JTeNormLinf = NormLinf(m_bs);

	GTOLMResult res = GTO_LM_ENOUGH_ITERATIONS;
	uint iterLM, nItersPCG;
	for(iterLM = 0; iterLM < m_lmMaxNumIters; ++iterLM)
	{
		nItersPCG = SolveNormalEquation();
		if(nItersPCG == 0)
		{
			res = GTO_LM_NORMAL_EQUATION_FAIL;
			break;
		}

		deltaNormL2 = ComputeDeltaNormL2();
		if(deltaNormL2 < m_lmStopDeltaNormL2)
		{
			res = GTO_LM_SMALL_DELTA;
			break;
		}

		const double ssePre = sseCur;
		UpdateSolution();
		sseCur = ComputeTransformationSSE();
		if(!std::isfinite(sseCur))
		{
			RollBackSolution();
			res = GTO_LM_INFINIT_SSE;
			break;
		}
		mseCur = sseCur * sseToMseFactor;

		const double reduction = (ssePre - sseCur);
		if(reduction > 0)
		{
			if(mseCur < m_lmStopMSE)
			{
				res = GTO_LM_SMALL_MSE;
				break;
			}
			reductionRel = reduction / ssePre;
			if(reductionRel < m_lmStopRelativeReduction)
			{
				res = GTO_LM_SMALL_RELATIVE_REDUCTION;
				break;
			}

			const double gainRatio = reduction / ComputeExpectedReduction();
			double t = gainRatio + gainRatio - 1;
			const double adaptiveAdjust = 1 - t * t * t;
			t = 1 / 3.0;
			const double autoAdjust = std::max(t, adaptiveAdjust);

			m_damping = float(m_damping * autoAdjust);
			dampingAdjust = 2.0f;
			if(m_damping < m_lmDampingMin)
				m_damping = m_lmDampingMin;
			else if(m_damping > m_lmDampingMax)
				m_damping = m_lmDampingMax;

			ConstructNormalEquation(true);
			JTeNormLinf = NormLinf(m_bs);
			if(JTeNormLinf < m_lmStopJTeNormLinf)
			{
				res = GTO_LM_SMALL_JTE;
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
			mseCur = sseCur * sseToMseFactor;
			if(m_damping == m_lmDampingMax)
			{
				res = GTO_LM_LARGE_DAMPING;
				break;
			}
			m_damping = m_damping * dampingAdjust;
			if(m_damping > m_lmDampingMax)
				m_damping = m_lmDampingMax;
			dampingAdjust = dampingAdjust + dampingAdjust;

			ConstructNormalEquation(false);
		}
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

template<GTO_TEMPLATE_PARAMETER>
inline double GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::ComputeTransformationSSE()
{
	++m_eCnt;
	return m_pData->ComputeTransformationSSE();
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::ConstructNormalEquation(const bool recompute)
{
	const MapIndex nMaps = m_pData->GetMapsNumber();
	if(recompute)
	{
		if(m_jCnt == 0 || !m_fixScales)
			m_pData->ConstructNormalEquation(m_As, m_bs, m_ss);
		else
			m_pData->ConstructNormalEquation(m_ss, m_As, m_bs);
		++m_jCnt;

		for(MapIndex i = 0; i < nMaps; ++i)
		{
			TransformationBlock &A = m_As[i];
			LA::GetDiagonal(A, m_ds[i]);
			//LA::IncreaseDiagonal(m_damping, A);
			LA::ScaleDiagonal(1 + m_damping, A);
			LA::InvertSymmetricUpper(A, m_Ms[i]);
		}
	}
	else
	{
		for(MapIndex i = 0; i < nMaps; ++i)
		{
			TransformationBlock &A = m_As[i];
			LA::SetDiagonal(m_ds[i], A);
			//LA::IncreaseDiagonal(m_damping, A);
			LA::ScaleDiagonal(1 + m_damping, A);
			LA::InvertSymmetricUpper(A, m_Ms[i]);
		}
	}
}

template<GTO_TEMPLATE_PARAMETER>
uint GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::SolveNormalEquation()
{
	return RunPCG();
}

template<GTO_TEMPLATE_PARAMETER>
inline double GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::ComputeDeltaNormL2()
{
	return sqrt(NormL2_2(m_xs));
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::UpdateSolution()
{
	SwapBackups();
	m_pData->UpdateTransformations(m_ss, m_xs, m_TsBkp);
}

template<GTO_TEMPLATE_PARAMETER>
void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::RollBackSolution()
{
	m_pData->RollBackTransformations(m_TsBkp);
}

template<GTO_TEMPLATE_PARAMETER>
void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::SwapBackups()
{
	m_pData->SwapTransformations(m_TsBkp);
}

template<GTO_TEMPLATE_PARAMETER>
inline double GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::ComputeExpectedReduction()
{
	//const double xTx = NormL2_2(m_xs);
	const double xTx = NormWL2_2(m_xs, m_ds);
	const double xTb = Dot(m_xs, m_bs);
	return m_damping * xTx + xTb;
}