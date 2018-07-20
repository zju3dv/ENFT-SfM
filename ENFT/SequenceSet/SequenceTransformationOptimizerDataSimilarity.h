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

#ifndef _SEQUENCE_TRANSFORMATION_OPTIMIZER_DATA_SIMILARITY_H_
#define _SEQUENCE_TRANSFORMATION_OPTIMIZER_DATA_SIMILARITY_H_

#include "SequenceSet.h"
#include "Optimization/GlobalTransformationOptimizerData.h"
#include "LinearAlgebra/Matrix7.h"

#define GTO_TEMPLATE_ARGUMENT_SIMILARITY SequenceIndex, SimilarityTransformation3D, LA::AlignedVector7f, LA::AlignedMatrix7f, \
										 TrackIndex, Point3D
class SequenceTransformationOptimizerDataSimilarity : public GlobalTransformationOptimizerDataTemplate<GTO_TEMPLATE_ARGUMENT_SIMILARITY>
{

public:

	//SequenceTransformationOptimizerDataSimilarity() : m_fixScales(false) {}

	virtual void NormalizeData(const float dataNormalizeMedian);
	virtual void DenormalizeData();

	//virtual void InvertTransformations();
	virtual double ComputeTransformationSSE() const;
	virtual void ConstructNormalEquation(AlignedVector<LA::AlignedMatrix7f> &As, AlignedVector<LA::AlignedVector7f> &bs, 
		AlignedVector<LA::AlignedVector7f> &ss) const;
	virtual void ConstructNormalEquation(const AlignedVector<LA::AlignedVector7f> &ss, AlignedVector<LA::AlignedMatrix7f> &As, 
		AlignedVector<LA::AlignedVector7f> &bs) const;
	virtual void UpdateTransformations(const AlignedVector<LA::AlignedVector7f> &ws, const AlignedVector<LA::AlignedVector7f> &dps, 
		const AlignedVector<SimilarityTransformation3D> &TsOld);
	void SetScales(const AlignedVector<float> &scales);

protected:

	//bool m_fixScales;

	friend SequenceSet;
};

#endif