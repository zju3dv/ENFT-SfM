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

#ifndef _SEGMENT_TRANSFORMATION_OPTIMIZER_DATA_SCALE_H_
#define _SEGMENT_TRANSFORMATION_OPTIMIZER_DATA_SCALE_H_

#include "SequenceSet.h"
#include "Optimization/GlobalTransformationOptimizerData.h"
#include "LinearAlgebra/Matrix1.h"

#define GTO_TEMPLATE_ARGUMENT_SCALE SegmentIndex, float, float, LA::Matrix1f, TrackIndex, float

class SequenceTransformationOptimizerDataScale : public GlobalTransformationOptimizerDataTemplate<GTO_TEMPLATE_ARGUMENT_SCALE>
{

public:

	virtual void NormalizeData(const float dataNormalizeMedian);
	virtual void DenormalizeData();

	//virtual void InvertTransformations();
	virtual double ComputeTransformationSSE() const;
	virtual void ConstructNormalEquation(AlignedVector<LA::Matrix1f> &As, AlignedVector<float> &bs, AlignedVector<float> &ss) const;
	virtual void ConstructNormalEquation(const AlignedVector<float> &ss, AlignedVector<LA::Matrix1f> &As, AlignedVector<float> &bs) const;
	virtual void UpdateTransformations(const AlignedVector<float> &ws, const AlignedVector<float> &dps, const AlignedVector<float> &TsOld);

	friend SequenceSet;

};

#endif