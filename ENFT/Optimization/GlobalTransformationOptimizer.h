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

#ifndef _GLOBAL_TRANSFORMATION_OPTIMIZER_H_
#define _GLOBAL_TRANSFORMATION_OPTIMIZER_H_

#include "GlobalTransformationOptimizerData.h"

enum GTOLMResult{ GTO_LM_ENOUGH_ITERATIONS, GTO_LM_NORMAL_EQUATION_FAIL, GTO_LM_SMALL_DELTA, GTO_LM_INFINIT_SSE, GTO_LM_SMALL_MSE, GTO_LM_SMALL_RELATIVE_REDUCTION, GTO_LM_SMALL_JTE, GTO_LM_LARGE_DAMPING };

template<GTO_TEMPLATE_PARAMETER>
class GlobalTransformationOptimizerTemplate
{

public:

	GlobalTransformationOptimizerTemplate() : m_fixScales(true), m_dataNormalizeMedian(0.5f), m_lmMaxNumIters(50), m_lmDampingInit(0.001f), m_lmDampingMin(1e-10f), m_lmDampingMax(100000), 
											  m_lmStopMSE(0.001f), m_lmStopRelativeReduction(0.0001f), m_lmStopJTeNormLinf(0.00001f), m_lmStopDeltaNormL2(1e-6f), 
											  m_pcgMinNumIters(10), m_pcgMaxNumIters(500), m_pcgStopRTZRatioMin(0.001f), m_pcgStopRTZRatioMax(10.0f) {}
	inline GTOLMResult Run(GlobalTransformationOptimizerDataTemplate<GTO_TEMPLATE_ARGUMENT> &data, const ubyte verbose = 0, FILE *fp = NULL);
	inline void PrintDataSize();
	inline void PrintLMResult(const GTOLMResult &res);

public:

	bool m_fixScales;

	float m_dataNormalizeMedian;
	uint m_lmMaxNumIters;
	float m_lmDampingInit, m_lmDampingMin, m_lmDampingMax;
	float m_lmStopMSE, m_lmStopRelativeReduction, m_lmStopJTeNormLinf, m_lmStopDeltaNormL2;

	uint m_pcgMinNumIters, m_pcgMaxNumIters;
	float m_pcgStopRTZRatioMin, m_pcgStopRTZRatioMax;

protected:

	//////////////////////////////////////////////////////////////////////////
	// Basic
	//////////////////////////////////////////////////////////////////////////
	inline void Initialize(GlobalTransformationOptimizerDataTemplate<GTO_TEMPLATE_ARGUMENT> &data, const ubyte verbose, FILE *fp);
	template<ubyte STAGE>
	inline void ComputeMx(const AlignedVector<TransformationParameter> &xs, AlignedVector<TransformationParameter> &Mxs);
	template<ubyte STAGE>
	inline void ComputeAx(const AlignedVector<TransformationParameter> &xs, AlignedVector<TransformationParameter> &Axs);
	static inline float NormLinf(const AlignedVector<TransformationParameter> &bs);
	static inline double NormL2_2(const AlignedVector<TransformationParameter> &xs);
	static inline double NormWL2_2(const AlignedVector<TransformationParameter> &xs, const AlignedVector<TransformationParameter> &ws);
	static inline double Dot(const AlignedVector<TransformationParameter> &xs, const AlignedVector<TransformationParameter> &bs);
	template<ubyte STAGE>
	static inline void sA(const float &s, const AlignedVector<TransformationParameter> &As, AlignedVector<TransformationParameter> &sAs);
	template<ubyte STAGE>
	static inline void sApB(const float &s, const AlignedVector<TransformationParameter> &As, const AlignedVector<TransformationParameter> &Bs, 
		AlignedVector<TransformationParameter> &sApBs);

	//////////////////////////////////////////////////////////////////////////
	// LM
	//////////////////////////////////////////////////////////////////////////
	inline GTOLMResult RunLM();
	inline double ComputeTransformationSSE();
	inline void ConstructNormalEquation(const bool recompute);
	inline uint SolveNormalEquation();
	inline double ComputeDeltaNormL2();
	inline void UpdateSolution();
	inline void RollBackSolution();
	inline void SwapBackups();
	inline double ComputeExpectedReduction();

	//////////////////////////////////////////////////////////////////////////
	// PCG
	//////////////////////////////////////////////////////////////////////////
	inline uint RunPCG();
	inline void Compute_r0_b();
	inline void Compute_z0p0_Mr0();
	inline double Compute_r0Tz0();
	inline void Compute_x1_a0p0(const float &a0);
	inline void Compute_r1_r0ma0Ap0(const float &a0);
	inline void Compute_z_Mr();
	inline double Compute_rTz();
	inline double Compute_pTAp();
	inline void Update_p_zpbp(const float &b);
	inline void Update_x_xpap(const float &a);
	inline void Update_r_rmaAp(const float &a);

protected:

	GlobalTransformationOptimizerDataTemplate<GTO_TEMPLATE_ARGUMENT> *m_pData;

	uint m_eCnt, m_jCnt;
	float m_damping;

	AlignedVector<Transformation> m_TsBkp;
	AlignedVector<TransformationBlock> m_As, m_Ms;
	AlignedVector<TransformationParameter> m_ss, m_bs, m_ds, m_ps, m_xs, m_zs, m_rs, m_Aps;
	std::vector<std::vector<std::pair<MapIndex, uint> > > m_idxsList;

	ubyte m_verbose;
	FILE *m_fp;

};

#include "GlobalTransformationOptimizer.hpp"
#include "GlobalTransformationOptimizerLM.hpp"
#include "GlobalTransformationOptimizerPCG.hpp"

#endif