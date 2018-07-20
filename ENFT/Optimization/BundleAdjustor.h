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

#ifndef _BUNDLE_ADJUSTOR_H_
#define _BUNDLE_ADJUSTOR_H_

#include "BundleAdjustorData.h"

#define BA_ELIMINATE_POINTS		1

enum BAResult{ BA_ENOUGH_ITERATIONS, BA_NORMAL_EQUATION_FAIL, BA_SMALL_DELTA, BA_INFINIT_SSE, BA_SMALL_MSE, BA_SMALL_RELATIVE_REDUCTION, BA_SMALL_JTE };

template<BA_TEMPLATE_PARAMETER>
class BundleAdjustorTemplate
{

public:

	BundleAdjustorTemplate() : m_fixScales(false), m_dataNormalizeMedian(0.5f), 
							   m_lmMaxNumIters(50), m_lmDampingInit(0.001f), m_lmStopMSE(0.001f), m_lmStopRelativeReduction(0.0001f), 
							   m_lmStopJTeNormLinf(0.00001f), m_lmStopDeltaNormL2(1e-6f), 
							   m_pcgMinNumIters(10), m_pcgMaxNumIters(500), m_pcgStopRTZRatioMin(0.001f), m_pcgStopRTZRatioMax(10.0f) {}
	
	virtual BAResult Run(BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> &data, const CameraIndex nCamsFix, const bool global, const ubyte verbose = 0, FILE *fp = NULL);
	virtual void PrintLMResult(const BAResult &res);

public:

	bool m_fixScales;
	float m_dataNormalizeMedian;

	uint m_lmMaxNumIters;
	float m_lmDampingInit, m_lmStopMSE, m_lmStopRelativeReduction, m_lmStopJTeNormLinf, m_lmStopDeltaNormL2;

	uint m_pcgMinNumIters, m_pcgMaxNumIters;
	float m_pcgStopRTZRatioMin, m_pcgStopRTZRatioMax;

protected:

	//////////////////////////////////////////////////////////////////////////
	// Basic
	//////////////////////////////////////////////////////////////////////////
	virtual void Initialize(BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> &data, const CameraIndex nCamsFix, const bool global, const ubyte verbose, FILE *fp);
	template<class Block, class Parameter>
	static inline void GetDiagonal(const AlignedVector<Block> &Ds, AlignedVector<Parameter> &ds);
	template<class Parameter, class Block>
	static inline void SetDiagonal(const AlignedVector<Parameter> &ds, AlignedVector<Block> &Ds);
	template<class Block>
	static inline void ScaleDiagonal(const float &lambda, AlignedVector<Block> &Ds);
	template<class Parameter>
	static inline float NormLinf(const AlignedVector<Parameter> &bs);
	template<class Parameter>
	static inline double NormL2_2(const AlignedVector<Parameter> &xs);
	template<class Parameter>
	static inline double NormWL2_2(const AlignedVector<Parameter> &xs, const AlignedVector<Parameter> &ws);
	template<class Parameter>
	static inline double Dot(const AlignedVector<Parameter> &xs, const AlignedVector<Parameter> &bs);
	template<class Block>
	static inline void InvertSymmetricUpper(const AlignedVector<Block> &As, AlignedVector<Block> &AIs);
	template<class Block, class Parameter, ubyte STAGE>
	static inline void AB(const AlignedVector<Block> &As, const AlignedVector<Parameter> &Bs, AlignedVector<Parameter> &ABs);
	template<class Parameter, ubyte STAGE>
	static inline void sA(const float &s, const AlignedVector<Parameter> &As, AlignedVector<Parameter> &sAs);
	template<class Parameter, ubyte STAGE>
	static inline void sApB(const float &s, const AlignedVector<Parameter> &As, const AlignedVector<Parameter> &Bs, AlignedVector<Parameter> &sApBs);
	template<class Parameter>
	static inline void AmB(const AlignedVector<Parameter> &As, const AlignedVector<Parameter> &Bs, AlignedVector<Parameter> &AmBs);

	//////////////////////////////////////////////////////////////////////////
	// LM
	//////////////////////////////////////////////////////////////////////////
	virtual BAResult RunLM();
	virtual double ComputeSSE();
	virtual float ConstructNormalEquation(const bool recompute);
	virtual uint SolveNormalEquation();
	virtual void ConstructSchurComplement();
	virtual void BackSubstituteSchurComplement();
	virtual double ComputeDeltaNormL2();
	virtual double UpdateSolution();
	virtual void RollBackSolution();
	//virtual void RollBackPoints();
	virtual double ComputeExpectedReduction();

	//////////////////////////////////////////////////////////////////////////
	// PCG
	//////////////////////////////////////////////////////////////////////////
	inline uint RunPCG();
	inline void Compute_M();
	inline void Compute_Ap();
	inline void Compute_r0_b();
	inline void Compute_z0p0_Mr0();
	inline double Compute_r0Tz0();
	inline void Compute_x1_ap0(const float &a);
	inline void Compute_z_Mr();
	inline double Compute_rTz();
	inline double Compute_pTAp();
	inline void Update_p_zpbp(const float &b);
	inline void Update_x_xpap(const float &a);
	inline void Update_r_rmaAp(const float &a);
	inline void Compute_e_Axmb();

protected:

	BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> *m_pData;
	CameraIndex m_nCamsFix;

	uint m_eCnt, m_jCnt;
	float m_damping;

	AlignedVector<Camera>				m_CsBkp;
	AlignedVector<Point>				m_XsBkp;
	Global								m_GBkp;
	AlignedVector<CameraBlock>			m_Dcs, m_Mcs;
	AlignedVector<PointBlock>			m_Dxs;
	GlobalBlock							m_Dg, m_Mg;
	AlignedVector<PointCameraBlock>		m_Wxcs;
	AlignedVector<GlobalCameraBlock>	m_Wgcs;
	AlignedVector<GlobalPointBlock>		m_Wgxs;
	AlignedVector<CameraParameter>		m_JcTes, m_scs, m_dcs, m_xcs, m_bcs, m_rcs, m_pcs, m_Apcs, m_zcs, m_Axcs, m_ecs;
	AlignedVector<PointParameter>		m_JxTes, m_sxs, m_dxs, m_xxs;
	GlobalParameter						m_JgTe, m_sg, m_dg, m_xg, m_bg, m_rg, m_pg, m_Apg, m_zg, m_Axg, m_eg;
#if BA_ELIMINATE_POINTS
	AlignedVector<CameraBlock>			m_Accs;
	AlignedVector<PointBlock>			m_DxIs;
	GlobalBlock							m_Agg;
	AlignedVector<GlobalCameraBlock>	m_Agcs;
#else
	AlignedVector<PointParameter>		m_bxs, m_rxs, m_pxs, m_Apxs, m_zxs, m_Axxs, m_exs;
	AlignedVector<PointBlock>			m_Mxs;
#endif

	std::vector<float> m_ptSSEs, m_ptSSEsBkp1, m_ptSSEsBkp2;

	class CameraBlockIndexTable
	{
	public:
		class Index
		{
		public:
			inline Index() {}
			inline Index(const CameraIndex &iCol, const uint &iBlock) : m_iCol(iCol), m_iBlock(iBlock) {}
			inline void Get(CameraIndex &iCol, uint &iBlock) const { iCol = m_iCol; iBlock = m_iBlock; }
		protected:
			CameraIndex m_iCol;
			uint m_iBlock;
		};
		class Row
		{
		public:
			inline void Initialize() { m_idxsUpper.resize(0); m_idxsLower.resize(0); }
			inline const std::vector<Index>& GetUpperBlockIndexes() const { return m_idxsUpper; }
			inline const std::vector<Index>& GetLowerBlockIndexes() const { return m_idxsLower; }
			inline uint GetUpperBlocksNumber() const { return m_idxsUpper.size(); }
			inline uint GetLowerBlocksNumber() const { return m_idxsLower.size(); }
			inline void PushBackUpperBlockIndex(const CameraIndex &iCol, const uint &iBlock) { m_idxsUpper.push_back(Index(iCol, iBlock)); }
			inline void PushBackLowerBlockIndex(const CameraIndex &iCol, const uint &iBlock) { m_idxsLower.push_back(Index(iCol, iBlock)); }
		protected:
			std::vector<Index> m_idxsLower, m_idxsUpper;
		};
	public:
		inline void Initialize(const CameraIndex &nRows)
		{
			m_nBlocks = 0;
			m_rows.resize(nRows);
			for(CameraIndex iRow = 0; iRow < nRows; ++iRow)
				m_rows[iRow].Initialize();
		}
		inline CameraIndex GetRowsNumber() const { return CameraIndex(m_rows.size()); }
		inline const uint& GetBlocksNumber() const { return m_nBlocks; }
		inline const Row& GetRow(const CameraIndex &iRow) const { return m_rows[iRow]; }
		inline void PushBackBlock(const CameraIndex &iRow, const CameraIndex &iCol)
		{
			m_rows[iRow].PushBackUpperBlockIndex(iCol, m_nBlocks);
			if(iRow != iCol)
				m_rows[iCol].PushBackLowerBlockIndex(iRow, m_nBlocks);
			++m_nBlocks;
		}
	protected:
		uint m_nBlocks;
		std::vector<Row> m_rows;
	};
	CameraBlockIndexTable m_iBlockTableCC;
	std::vector<uint> m_mapCamToBlockCX;
	std::vector<MeasurementIndex> m_iMeasOriToNew;

	class PointFeatureIndex
	{
	public:
		inline void Set(const PointIndex &iPt, const FeatureIndex &iFtr) { m_iPt = iPt; m_iFtr = iFtr; }
		inline const PointIndex& GetPointIndex() const { return m_iPt; }
		inline const FeatureIndex& GetFeatureIndex() const { return m_iFtr; }
		inline bool operator < (const PointFeatureIndex &iPtFtr) const { return m_iPt < iPtFtr.m_iPt || m_iPt == iPtFtr.m_iPt && m_iFtr < iPtFtr.m_iFtr; }
	protected:
		PointIndex m_iPt;
		FeatureIndex m_iFtr;
	};
	std::vector<PointFeatureIndex> m_iPtFtrs;
	std::vector<bool> m_ptMarks;

#if BA_ELIMINATE_POINTS
	std::vector<FeatureIndex> m_mapPtToYxc;
	AlignedVector<PointCameraBlock> m_Yxcs;
#endif

	ubyte m_verbose;
	FILE *m_fp;

};

#include "BundleAdjustor.hpp"
#include "BundleAdjustorLM.hpp"
#include "BundleAdjustorPCG.hpp"

#endif