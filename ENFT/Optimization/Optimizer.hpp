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
#include <cmath>

template<OPT_TEMPLATE_PARAMETER>
inline OPTLMResult OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::Run(OptimizerDataTemplate<OPT_TEMPLATE_ARGUMENT> &data, Model &model, const ubyte verbose)
{
	m_pData = &data;
	m_pModel = &model;
	data.NormalizeData(m_dataNormalizeMedian, model);
	//m_modelBkp = data.GetModel();
	const OPTLMResult res = RunLM(verbose);
	data.DenormalizeData(model);
	if(verbose > 1)
		PrintLMResult(res);
	return res;
}

template<OPT_TEMPLATE_PARAMETER>
inline void OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::PrintLMResult(const OPTLMResult &res)
{
	switch(res)
	{
	case OPT_LM_ENOUGH_ITERATIONS:
		printf("  Stop by OPT_LM_ENOUGH_ITERATIONS\n");
		break;
	case OPT_LM_NORMAL_EQUATION_FAIL:
		printf("  Stop by OPT_LM_NORMAL_EQUATION_FAIL\n");
		break;
	case OPT_LM_SMALL_DELTA:
		printf("  Stop by OPT_LM_SMALL_DELTA\n");
		break;
	case OPT_LM_INFINIT_SSE:
		printf("  Stop by OPT_LM_INFINIT_SSE\n");
		break;
	case OPT_LM_SMALL_MSE:
		printf("  Stop by OPT_LM_SMALL_MSE\n");
		break;
	case OPT_LM_SMALL_RELATIVE_REDUCTION:
		printf("  Stop by OPT_LM_SMALL_RELATIVE_REDUCTION\n");
		break;
	case OPT_LM_SMALL_JTE:
		printf("  Stop by OPT_LM_SMALL_JTE\n");
		break;
	}
}

template<OPT_TEMPLATE_PARAMETER>
inline OPTLMResult OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::RunLM(const ubyte verbose)
{
	m_eCnt = m_jCnt = 0;

	const double factorSseToMse = m_pData->GetFactorSSEToMSE();
	double sseCur, mseCur, reductionRel = 0, deltaNormL2 = 0;
	float JTeNormLinf = 0;
	sseCur = ComputeSSE();
	mseCur = sseCur * factorSseToMse;

	m_damping = m_lmDampingInit;
	float dampingAdjust = 2.0f;

	if(verbose == 1)
		printf("  MSE = %f", mseCur);
	else if(verbose > 1)
	{
		printf("   #LM   MSE  rel_reduction  ||JTe||_oo  ||dx||^2  damping\n");
		printf("       %f\n", mseCur);
	}

	JTeNormLinf = ConstructNormalEquation(true);

	OPTLMResult res = OPT_LM_ENOUGH_ITERATIONS;
	uint iterLM;
	for(iterLM = 0; iterLM < m_lmMaxNumIters; ++iterLM)
	{
		if(!SolveNormalEquation())
		{
			res = OPT_LM_NORMAL_EQUATION_FAIL;
			break;
		}

		deltaNormL2 = ComputeDeltaNormL2();
		if(deltaNormL2 < m_lmStopDeltaNormL2)
		{
			res = OPT_LM_SMALL_DELTA;
			break;
		}

		const double ssePre = sseCur;
		UpdateSolution();
		sseCur = ComputeSSE();
		if(!std::isfinite(sseCur))
		{
			RollBackSolution();
			res = OPT_LM_INFINIT_SSE;
			break;
		}
		mseCur = sseCur * factorSseToMse;

		const double reduction = (ssePre - sseCur);
		if(reduction > 0)
		{
			if(mseCur < m_lmStopMSE)
			{
				res = OPT_LM_SMALL_MSE;
				break;
			}
			reductionRel = reduction / ssePre;
			if(reductionRel < m_lmStopRelativeReduction)
			{
				res = OPT_LM_SMALL_RELATIVE_REDUCTION;
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
				res = OPT_LM_SMALL_JTE;
				break;
			}

			if(verbose > 1)
				printf("--> %d  %f  %f  %f  %f  %e\n", iterLM, mseCur, reductionRel, JTeNormLinf, deltaNormL2, m_damping);
		}
		else
		{
			if(verbose > 1)
				printf("--> %d  %f\n", iterLM, mseCur);
			RollBackSolution();

			sseCur = ssePre;
			mseCur = sseCur * factorSseToMse;
			m_damping = m_damping * dampingAdjust;
			dampingAdjust = dampingAdjust + dampingAdjust;

			ConstructNormalEquation(false);
		}
	}

	if(verbose == 1)
		printf(" --> %f\n", mseCur);
	else if(verbose > 1)
		printf("--> %d  %f  %f  %f  %f  %e\n", iterLM, mseCur, reductionRel, JTeNormLinf, deltaNormL2, m_damping);

	return res;
}

template<OPT_TEMPLATE_PARAMETER>
inline double OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::ComputeSSE()
{
	++m_eCnt;
	return m_pData->ComputeSSE(*m_pModel);
}

template<OPT_TEMPLATE_PARAMETER>
inline float OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::ConstructNormalEquation(const bool recompute)
{
	if(recompute)
	{
		if(m_jCnt == 0 || !m_fixScales)
			m_pData->ConstructNormalEquation(*m_pModel, m_A, m_b, m_s);
		else
			m_pData->ConstructNormalEquation(*m_pModel, m_s, m_A, m_b);
		LA::GetDiagonal(m_A, m_d);
		//LA::IncreaseDiagonal(m_damping, m_A);
		LA::ScaleDiagonal(m_damping + 1, m_A);
		++m_jCnt;
	}
	else
	{
		LA::SetDiagonal(m_d, m_A);
		//LA::IncreaseDiagonal(m_damping, m_A);
		LA::ScaleDiagonal(m_damping + 1, m_A);
	}
	return LA::NormLinf(m_b);
}

template<OPT_TEMPLATE_PARAMETER>
bool OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::SolveNormalEquation()
{
	//float work[2];
	//if(LA::InvertSymmetricUpper(m_A, m_Ainv, work))
	//{
	//	LA::AB(m_Ainv, m_b, m_x);
	//	LA::SetReserve<5>(m_x);
	//	return true;
	//}
	//return false;
	float work[2];
	m_x = m_b;
	return LA::SolveLinearSystemSymmetricUpper(m_A, m_x, work);
}

template<OPT_TEMPLATE_PARAMETER>
inline double OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::ComputeDeltaNormL2()
{
	return sqrt(LA::NormL2_2(m_x));
}

template<OPT_TEMPLATE_PARAMETER>
inline void OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::UpdateSolution()
{
	BackupModel();
	m_pData->UpdateModel(m_s, m_x, m_modelBkp, *m_pModel);
}

template<OPT_TEMPLATE_PARAMETER>
void OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::RollBackSolution()
{
	*m_pModel = m_modelBkp;
}

template<OPT_TEMPLATE_PARAMETER>
void OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::BackupModel()
{
	m_modelBkp = *m_pModel;
}

template<OPT_TEMPLATE_PARAMETER>
inline double OptimizerTemplate<OPT_TEMPLATE_ARGUMENT>::ComputeExpectedReduction()
{
	//const double xTx = LA::NormL2_2(m_x);
	const double xTx = LA::NormWL2_2(m_x, m_d);
	const double xTb = LA::Dot(m_x, m_b);
	return m_damping * xTx + xTb;
}