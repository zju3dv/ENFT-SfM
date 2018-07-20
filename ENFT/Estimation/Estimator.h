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

#ifndef _RANSAC_H_
#define _RANSAC_H_

#include "Utility/AlignedVector.h"

#ifndef ESTIMATOR_TEMPLATE_PARAMETER
#define ESTIMATOR_TEMPLATE_PARAMETER class Data, class MinimalSample, class NonMinimalSample, class Model, class Solver, class Optimizer, typename Index
#endif
#ifndef ESTIMATOR_TEMPLATE_ARGUMENT
#define ESTIMATOR_TEMPLATE_ARGUMENT Data, MinimalSample, NonMinimalSample, Model, Solver, Optimizer, Index
#endif

template<ESTIMATOR_TEMPLATE_PARAMETER> class Estimator
{

public:

	Estimator(const float &errTh = 0) : 
	  m_ransacErrorThreshold(errTh), m_ransacStopConfidence(0.9f), m_ransacMinInlierProportion(0.2f), m_ransacMaxNumIters(10000), m_ransacMinNumIters(300), 
	  m_losacInnerSampleRatio(0.5f), m_losacNumInnerIters(20), 
	  m_prosacRandomConsistencyProbability(0.01f), 
	  m_solveByAllInliers(true), m_optimizeByAllInliers(true), m_optimizeMaxNumIters(100) {}

	inline uint RunRansac(const Data &data, Model &model, std::vector<Index> &inliers, const ubyte verbose = 0);
	inline uint RunLosac(const Data &data, Model &model, std::vector<Index> &inliers, const ubyte verbose = 0);
	inline uint RunLosacFixInliers(const Data &data, const std::vector<bool> &inlierMarksFix, Model &model, std::vector<Index> &inliers, const ubyte verbose = 0);
	inline bool RunLosacInner(const Data &data, Model &model, std::vector<Index> &inliers, const ubyte verbose = 0);
	inline bool RunLosacInnerFixInliers(const Data &data, const std::vector<bool> &inlierMarksFix, Model &model, std::vector<Index> &inliers, const ubyte verbose = 0);
	inline void RunLosacMultiple(const Data &data, AlignedVector<Model> &modelList, std::vector<std::vector<Index> > &inliersList, const uint maxNumModels, 
		const Index minNumInliersPerModel, const ubyte verbose = 0);
	inline void RunProsac(const Data &data, const std::vector<Index> &orders, Model &model, std::vector<Index> &inliers, const ubyte verbose = 0);
	inline void RunLoProsac(const Data &data, const std::vector<Index> &orders, Model &model, std::vector<Index> &inliers, const ubyte verbose = 0);

	inline Index CountInliers(const Data &data, const Model &model);
	inline void FromInliersToInlierMarks(const std::vector<Index> &inliers, const Index N, std::vector<bool> &inlierMarks);
	inline void FromInliersToOutliers(const std::vector<Index> &inliers, const Index N, std::vector<Index> &outliers);
	template<typename TYPE>
	inline void FromInlierMarksToInliers(const std::vector<bool> &inlierMarks, std::vector<TYPE> &inliers);

	// Ransac
	virtual const Index MinimalSampleSize() const = 0;
	virtual float epsilon_exp_m(const float &epsilon) const = 0;
	virtual void DrawMinimalSample(const Data &data, MinimalSample &sample) const = 0;
	virtual void GenerateModels(MinimalSample &sample, AlignedVector<Model> &models) = 0;
	virtual void GenerateModels(NonMinimalSample &sample, AlignedVector<Model> &models) = 0;
	virtual void VerifyModel(const Data &data, const Model &model, const std::vector<bool> &inlierMarks, double &fitErr) = 0;
	virtual void VerifyModel(const Data &data, const Model &model, std::vector<Index> &inliers, double &fitErr) = 0;
	virtual void VerifyModel(const Data &data, const Model &model, const std::vector<bool> &inlierMarksFix, std::vector<Index> &inliers, double &fitErr) = 0;
	virtual void OptimizeModel(Data &data, Model &model, const ubyte verbose = 0) = 0;

	// Post-process
	inline void VerifyModels(const Data &data, AlignedVector<Model> &models, std::vector<Index> &inliers, std::vector<Index> &inliersTmp, double &fitErr);
	inline void VerifyModels(const Data &data, const std::vector<bool> &inlierMarksFix, AlignedVector<Model> &models, std::vector<Index> &inliers, 
		std::vector<Index> &inliersTmp, double &fitErr);
	inline bool SolveByAllInliers(const Data &data, Model &model, std::vector<Index> &inliers, double &fitErr, const ubyte verbose = 0);
	inline bool SolveByAllInliers(const Data &data, const std::vector<bool> &inlierMarksFix, Model &model, std::vector<Index> &inliers, double &fitErr, 
		const ubyte verbose = 0);
	inline bool OptimizeByAllInliers(const Data &data, Model &model, std::vector<Index> &inliers, double &fitErr, const ubyte verbose = 0);
	inline bool OptimizeByAllInliers(const Data &data, const std::vector<bool> &inlierMarksFix, Model &model, std::vector<Index> &inliers, double &fitErr, 
		const ubyte verbose = 0);

	// Losac
	inline bool DrawNonMinimalSample(const Data &data, const std::vector<Index> &inliers, NonMinimalSample &sample, std::vector<Index> &sampleIdxs, 
		std::vector<bool> &sampleMasks) const;

	// Prosac
	virtual void DrawMinimalSampleOrdered(const Data &data, const std::vector<Index> &orders, const Index &n, const bool &sampleLastOne, 
		MinimalSample &sample) const = 0;

public:

	// Ransac
	float m_ransacErrorThreshold, m_ransacStopConfidence, m_ransacMinInlierProportion;
	uint m_ransacMaxNumIters, m_ransacMinNumIters;

	// Losac
	float m_losacInnerSampleRatio;
	uint m_losacNumInnerIters;

	// Prosac
	float m_prosacRandomConsistencyProbability;//, m_prosacNonRandomnessConfidence;

	// Post-process
	bool m_solveByAllInliers, m_optimizeByAllInliers;
	uint m_optimizeMaxNumIters;

	Solver m_solver;
	Optimizer m_optimizer;
	AlignedVector<ENFT_SSE::__m128> m_work;

protected:

	// Ransac
	Data m_subset, m_dataInlier;
	MinimalSample m_minimalSample;
	NonMinimalSample m_nonMinimalSample;
	AlignedVector<Model> m_models;
	std::vector<Index> m_inliers, m_inliers2, m_idxs, m_idxs2;

	// Losac
	std::vector<bool> m_marks;

};

#include "EstimatorRansac.hpp"
#include "EstimatorLosac.hpp"
#include "EstimatorProsac.hpp"
#include "EstimatorLoProsac.hpp"

#endif