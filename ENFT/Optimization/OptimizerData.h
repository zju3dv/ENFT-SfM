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

#ifndef _OPTIMIZER_DATA_H_
#define _OPTIMIZER_DATA_H_

#include "Utility/AlignedVector.h"

#ifndef OPT_TEMPLATE_PARAMETER
#define OPT_TEMPLATE_PARAMETER class Model, class ModelParameter, class ModelBlock
#endif
#ifndef OPT_TEMPLATE_ARGUMENT
#define OPT_TEMPLATE_ARGUMENT Model, ModelParameter, ModelBlock
#endif

template<OPT_TEMPLATE_PARAMETER>
class OptimizerDataTemplate
{

public:

	virtual void NormalizeData(const float &dataNormalizeMedian, Model &model) = 0;
	virtual void DenormalizeData(Model &model) = 0;
	virtual double ComputeSSE(const Model &model) = 0;
	virtual double GetFactorSSEToMSE() = 0;
	virtual void ConstructNormalEquation(const Model &model, ModelBlock &A, ModelParameter &b, ModelParameter &s) = 0;
	virtual void ConstructNormalEquation(const Model &model, const ModelParameter &s, ModelBlock &A, ModelParameter &b) = 0;
	virtual void UpdateModel(const ModelParameter &s, const ModelParameter &x, const Model &modelOld, Model &modelNew) = 0;

};

#endif