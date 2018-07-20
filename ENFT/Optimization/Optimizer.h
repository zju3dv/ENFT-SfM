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

#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include "OptimizerData.h"

enum OPTLMResult{ OPT_LM_ENOUGH_ITERATIONS, OPT_LM_NORMAL_EQUATION_FAIL, OPT_LM_SMALL_DELTA, OPT_LM_INFINIT_SSE, OPT_LM_SMALL_MSE, 
				  OPT_LM_SMALL_RELATIVE_REDUCTION, OPT_LM_SMALL_JTE };

template<OPT_TEMPLATE_PARAMETER>
class OptimizerTemplate
{

public:

	inline OptimizerTemplate() : m_fixScales(true), m_dataNormalizeMedian(0.5f), m_lmMaxNumIters(50), m_lmDampingInit(0.001f), m_lmStopMSE(0.0f), 
								 m_lmStopRelativeReduction(0.0001f), m_lmStopJTeNormLinf(0.00001f), m_lmStopDeltaNormL2(1e-6f), 
								 m_modelBkp(*(Model *) _aligned_malloc(sizeof(Model), SSE_ALIGNMENT)), 
								 m_A(*(ModelBlock *) _aligned_malloc(sizeof(ModelBlock), SSE_ALIGNMENT)), 
								 m_s(*(ModelParameter *) _aligned_malloc(sizeof(ModelParameter), SSE_ALIGNMENT)), 
								 m_b(*(ModelParameter *) _aligned_malloc(sizeof(ModelParameter), SSE_ALIGNMENT)), 
								 m_d(*(ModelParameter *) _aligned_malloc(sizeof(ModelParameter), SSE_ALIGNMENT)), 
								 m_x(*(ModelParameter *) _aligned_malloc(sizeof(ModelParameter), SSE_ALIGNMENT)) {}
	inline ~OptimizerTemplate()
	{
		_aligned_free(&m_modelBkp);
		_aligned_free(&m_A);
		_aligned_free(&m_s);
		_aligned_free(&m_b);
		_aligned_free(&m_d);
		_aligned_free(&m_x);
	}

	inline OPTLMResult Run(OptimizerDataTemplate<OPT_TEMPLATE_ARGUMENT> &data, Model &model, const ubyte verbose = 0);

	inline void PrintLMResult(const OPTLMResult &res);

public:

	bool m_fixScales;

	float m_dataNormalizeMedian;
	uint m_lmMaxNumIters;
	float m_lmDampingInit, m_lmStopMSE, m_lmStopRelativeReduction, m_lmStopJTeNormLinf, m_lmStopDeltaNormL2;

protected:

	//////////////////////////////////////////////////////////////////////////
	// LM
	//////////////////////////////////////////////////////////////////////////
	inline OPTLMResult RunLM(const ubyte verbose);
	inline double ComputeSSE();
	inline float ConstructNormalEquation(const bool recompute);
	inline bool SolveNormalEquation();
	inline double ComputeDeltaNormL2();
	inline void UpdateSolution();
	inline void RollBackSolution();
	inline void BackupModel();
	inline double ComputeExpectedReduction();

protected:

	OptimizerDataTemplate<OPT_TEMPLATE_ARGUMENT> *m_pData;
	Model *m_pModel;

	uint m_eCnt, m_jCnt;
	float m_damping;

	Model &m_modelBkp;
	ModelBlock &m_A/*, &m_Ainv*/;
	ModelParameter &m_s, &m_b, &m_d, &m_x;

};

#include "Optimizer.hpp"

#endif