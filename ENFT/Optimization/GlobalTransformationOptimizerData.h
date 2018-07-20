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

#ifndef _GLOBAL_TRANSFORMATION_OPTIMIZER_DATA_H_
#define _GLOBAL_TRANSFORMATION_OPTIMIZER_DATA_H_

#include "Utility/AlignedVector.h"

#define GTO_STAGE_GENERAL	0
#define GTO_STAGE_S			1
#define GTO_STAGE_JTE		2
#define GTO_STAGE_P			3
#define GTO_STAGE_Z			4
#define GTO_STAGE_X			5
#define GTO_STAGE_R			6

#ifndef GTO_TEMPLATE_PARAMETER
#define GTO_TEMPLATE_PARAMETER typename MapIndex, class Transformation, class TransformationParameter, class TransformationBlock, \
							   typename PointIndex, class Point
#endif
#ifndef GTO_TEMPLATE_ARGUMENT
#define GTO_TEMPLATE_ARGUMENT MapIndex, Transformation, TransformationParameter, TransformationBlock, \
							  PointIndex, Point
#endif

template<GTO_TEMPLATE_PARAMETER>
class GlobalTransformationOptimizerDataTemplate
{

public:

	class PointPair
	{
	public:
		inline const Point& X1() const { return m_X1; }		inline Point& X1() { return m_X1; }
		inline const Point& X2() const { return m_X2; }		inline Point& X2() { return m_X2; }
	protected:
		Point m_X1, m_X2;
	};

	class MapPair
	{
	public:
		MapPair() {}
		MapPair(const MapIndex &iMap1, const MapIndex &iMap2, const uint &iBlock) : m_iMap1(iMap1), m_iMap2(iMap2), m_iBlock(iBlock) {}
		inline const MapIndex& GetMapIndex1() const { return m_iMap1; }
		inline const MapIndex& GetMapIndex2() const { return m_iMap2; }
		inline const uint& GetBlockIndex() const { return m_iBlock; }
		inline void GetIndexes(MapIndex &iMap1, MapIndex &iMap2, uint &iBlock) const { iMap1 = m_iMap1; iMap2 = m_iMap2; iBlock = m_iBlock; }
		inline void SetIndexes(const MapIndex &iMap1, const MapIndex &iMap2, const uint &iBlock) { m_iMap1 = iMap1; m_iMap2 = iMap2; m_iBlock = iBlock; }
		inline const AlignedVector<PointPair>& Xs() const { return m_Xs; }
		inline		 AlignedVector<PointPair>& Xs()		  { return m_Xs; }
	protected:
		MapIndex m_iMap1, m_iMap2;
		uint m_iBlock;
		AlignedVector<PointPair> m_Xs;
	};

public:

	virtual void NormalizeData(const float dataNormalizeMedian) = 0;
	virtual void DenormalizeData() = 0;

	//virtual void InvertTransformations() = 0;
	virtual double ComputeTransformationSSE() const = 0;
	virtual void ConstructNormalEquation(AlignedVector<TransformationBlock> &As, AlignedVector<TransformationParameter> &bs, 
		AlignedVector<TransformationParameter> &ss) const = 0;
	virtual void ConstructNormalEquation(const AlignedVector<TransformationParameter> &ss, AlignedVector<TransformationBlock> &As, 
		AlignedVector<TransformationParameter> &bs) const = 0;
	virtual void UpdateTransformations(const AlignedVector<TransformationParameter> &ws, const AlignedVector<TransformationParameter> &dps, 
		const AlignedVector<Transformation> &TsOld) = 0;

	inline MapIndex GetMapsNumber() const { return MapIndex(m_Ts.Size()); }
	inline uint GetMapPairsNumber() const { return uint(m_mapPairs.size()); }
	inline const PointIndex& GetPointsNumber() const { return m_nPts; }
	inline void SetPointsNumber(const PointIndex &nPts) { m_nPts = nPts; }
	inline uint GetBlocksNumber() const { return uint(m_Ts.Size() + m_mapPairs.size()); }
	inline const Transformation& GetTransformation(const MapIndex &iMap) const { return m_Ts[iMap]; }
	inline const AlignedVector<Transformation>& GetTransformations() const { return m_Ts; }
	inline const MapPair& GetMapPair(const uint &i) const { return m_mapPairs[i]; }
	inline void GetMapIndexes(const uint &i, MapIndex &iMap1, MapIndex &iMap2) const { iMap1 = m_mapPairs[i].GetMapIndex1(); iMap2 = m_mapPairs[i].GetMapIndex2(); }
	inline const AlignedVector<PointPair>& Xs(const uint &i) const { return m_mapPairs[i].Xs(); }
	inline		 AlignedVector<PointPair>& Xs(const uint &i)	   { return m_mapPairs[i].Xs(); }
	inline void CopyTransformations(AlignedVector<Transformation> &TsDst) const { TsDst.CopyFrom(m_Ts.Data()); }
	inline void SwapTransformations(AlignedVector<Transformation> &Ts) { m_Ts.Swap(Ts); }
	inline void RollBackTransformations(AlignedVector<Transformation> &Ts) { m_Ts.Swap(Ts); }

protected:

	AlignedVector<Transformation> m_Ts/*, m_Tinvs*/;
	std::vector<MapPair> m_mapPairs;
	PointIndex m_nPts;

	LA::Vector3f m_translation;
	float m_scale;

};

#endif