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

#ifndef _CAMERA_ESTIMATOR_H_
#define _CAMERA_ESTIMATOR_H_

#include "Estimation/EstimatorParsac.h"
#include "CameraEstimatorData.h"
#include "CameraEPnP.h"
#include "Optimization/Optimizer.h"
#include "Utility/Random.h"

typedef CameraEstimatorData CEData;
typedef SixMatches3DTo2D    CEMinimalSample;
typedef CameraEstimatorData CENonMinimalSample;
typedef Camera              CEModel;
typedef CameraEPnP          CESolver;
typedef OptimizerTemplate<Camera, LA::AlignedVector6f, LA::AlignedCompactMatrix6f>
CEOptimizer;
typedef ushort              CEIndex;
class CameraEstimator : public
    EstimatorParsac<CEData, CEMinimalSample, CENonMinimalSample, CEModel, CESolver, CEOptimizer, CEIndex> {

  public:

    CameraEstimator(const float &errTh = 0) :
        EstimatorParsac<CEData, CEMinimalSample, CENonMinimalSample, CEModel, CESolver, CEOptimizer, CEIndex>
        (errTh) {}

    virtual const ushort MinimalSampleSize() const {
        return 6;
    }

    virtual float epsilon_exp_m(const float &epsilon) const {
        const float tmp = epsilon * epsilon;
        return tmp * tmp * tmp;
    }

    virtual void DrawMinimalSample(const CEData &data,
                                   CEMinimalSample &sample) const {
        const ushort N = data.Size();
        ushort i0 = Random::Generate(N);
        sample.Set(data.Xs(), data.xs(), i0, 0);

        ushort i1 = Random::Generate(N);
        while(i1 == i0)
            i1 = Random::Generate(N);
        sample.Set(data.Xs(), data.xs(), i1, 1);

        ushort i2 = Random::Generate(N);
        while(i2 == i0 || i2 == i1)
            i2 = Random::Generate(N);
        sample.Set(data.Xs(), data.xs(), i2, 2);

        ushort i3 = Random::Generate(N);
        while(i3 == i0 || i3 == i1 || i3 == i2)
            i3 = Random::Generate(N);
        sample.Set(data.Xs(), data.xs(), i3, 3);

        ushort i4 = Random::Generate(N);
        while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
            i4 = Random::Generate(N);
        sample.Set(data.Xs(), data.xs(), i4, 4);

        ushort i5 = Random::Generate(N);
        while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4)
            i5 = Random::Generate(N);
        sample.Set(data.Xs(), data.xs(), i5, 5);
    }

    virtual void DrawMinimalSampleOrdered(const CEData &data,
                                          const std::vector<ushort> &orders, const ushort &n, const bool &sampleLastOne,
                                          CEMinimalSample &sample) const {
        ushort i0 = Random::Generate(n), i = orders[i0];
        sample.Set(data.Xs(), data.xs(), i, 0);

        ushort i1 = Random::Generate(n);
        while(i1 == i0)
            i1 = Random::Generate(n);
        i = orders[i1];
        sample.Set(data.Xs(), data.xs(), i, 1);

        ushort i2 = Random::Generate(n);
        while(i2 == i0 || i2 == i1)
            i2 = Random::Generate(n);
        i = orders[i2];
        sample.Set(data.Xs(), data.xs(), i, 2);

        ushort i3 = Random::Generate(n);
        while(i3 == i0 || i3 == i1 || i3 == i2)
            i3 = Random::Generate(n);
        i = orders[i3];
        sample.Set(data.Xs(), data.xs(), i, 3);

        ushort i4 = Random::Generate(n);
        while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
            i4 = Random::Generate(n);
        i = orders[i4];
        sample.Set(data.Xs(), data.xs(), i, 4);

        ushort i5 = n - 1;
        if(!sampleLastOne || i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4) {
            i5 = Random::Generate(n);
            while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4)
                i5 = Random::Generate(n);
        }
        i = orders[i5];
        sample.Set(data.Xs(), data.xs(), i, 5);
    }

    virtual void DrawMinimalSample(const CEData &data,
                                   const std::vector<std::vector<ushort> > &validBinData,
                                   CEMinimalSample &sample) const {
        //iBinsValidSampled.resize(6);

        const ushort nBinsValid = ushort(validBinData.size());
        const uint i0 = Random::Generate(nBinsValid);
        ushort idx = validBinData[i0][rand() % validBinData[i0].size()];
        sample.Set(data.Xs(), data.xs(), idx, 0);
        //iBinsValidSampled[0] = i0;

        uint i1 = Random::Generate(nBinsValid);
        while(i1 == i0)
            i1 = Random::Generate(nBinsValid);
        idx = validBinData[i1][rand() % validBinData[i1].size()];
        sample.Set(data.Xs(), data.xs(), idx, 1);
        //iBinsValidSampled[1] = i1;

        uint i2 = Random::Generate(nBinsValid);
        while(i2 == i0 || i2 == i1)
            i2 = Random::Generate(nBinsValid);
        idx = validBinData[i2][rand() % validBinData[i2].size()];
        sample.Set(data.Xs(), data.xs(), idx, 2);
        //iBinsValidSampled[2] = i2;

        uint i3 = Random::Generate(nBinsValid);
        while(i3 == i0 || i3 == i1 || i3 == i2)
            i3 = Random::Generate(nBinsValid);
        idx = validBinData[i3][rand() % validBinData[i3].size()];
        sample.Set(data.Xs(), data.xs(), idx, 3);
        //iBinsValidSampled[3] = i3;

        uint i4 = Random::Generate(nBinsValid);
        while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
            i4 = Random::Generate(nBinsValid);
        idx = validBinData[i4][rand() % validBinData[i4].size()];
        sample.Set(data.Xs(), data.xs(), idx, 4);
        //iBinsValidSampled[4] = i4;

        uint i5 = Random::Generate(nBinsValid);
        while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4)
            i5 = Random::Generate(nBinsValid);
        idx = validBinData[i5][rand() % validBinData[i5].size()];
        sample.Set(data.Xs(), data.xs(), idx, 5);
        //iBinsValidSampled[5] = i5;
    }

    virtual void DrawMinimalSample(const CEData &data,
                                   const std::vector<float> &validBinConfidencesAccumulated,
                                   const std::vector<std::vector<ushort> > &validBinData,
                                   CEMinimalSample &sample) const {
        //iBinsValidSampled.resize(6);

        const uint i0 = Random::GenerateUint(validBinConfidencesAccumulated);
        ushort idx = validBinData[i0][rand() % validBinData[i0].size()];
        sample.Set(data.Xs(), data.xs(), idx, 0);
        //iBinsValidSampled[0] = i0;

        uint i1 = Random::GenerateUint(validBinConfidencesAccumulated);
        while(i1 == i0)
            i1 = Random::GenerateUint(validBinConfidencesAccumulated);
        idx = validBinData[i1][rand() % validBinData[i1].size()];
        sample.Set(data.Xs(), data.xs(), idx, 1);
        //iBinsValidSampled[1] = i1;

        uint i2 = Random::GenerateUint(validBinConfidencesAccumulated);
        while(i2 == i0 || i2 == i1)
            i2 = Random::GenerateUint(validBinConfidencesAccumulated);
        idx = validBinData[i2][rand() % validBinData[i2].size()];
        sample.Set(data.Xs(), data.xs(), idx, 2);
        //iBinsValidSampled[2] = i2;

        uint i3 = Random::GenerateUint(validBinConfidencesAccumulated);
        while(i3 == i0 || i3 == i1 || i3 == i2)
            i3 = Random::GenerateUint(validBinConfidencesAccumulated);
        idx = validBinData[i3][rand() % validBinData[i3].size()];
        sample.Set(data.Xs(), data.xs(), idx, 3);
        //iBinsValidSampled[3] = i3;

        uint i4 = Random::GenerateUint(validBinConfidencesAccumulated);
        while(i4 == i0 || i4 == i1 || i4 == i2 || i4 == i3)
            i4 = Random::GenerateUint(validBinConfidencesAccumulated);
        idx = validBinData[i4][rand() % validBinData[i4].size()];
        sample.Set(data.Xs(), data.xs(), idx, 4);
        //iBinsValidSampled[4] = i4;

        uint i5 = Random::GenerateUint(validBinConfidencesAccumulated);
        while(i5 == i0 || i5 == i1 || i5 == i2 || i5 == i3 || i5 == i4)
            i5 = Random::GenerateUint(validBinConfidencesAccumulated);
        idx = validBinData[i5][rand() % validBinData[i5].size()];
        sample.Set(data.Xs(), data.xs(), idx, 5);
        //iBinsValidSampled[5] = i5;
    }

    virtual void GenerateModels(CEMinimalSample &sample,
                                AlignedVector<CEModel> &models) {
        models.Resize(1);
        if(!m_solver.Run(sample, models[0], m_work))
            models.Resize(0);
    }

    virtual void GenerateModels(CENonMinimalSample &sample,
                                AlignedVector<CEModel> &models) {
        models.Resize(1);
        if(!m_solver.Run(sample, models[0], m_work))
            models.Resize(0);
    }

    virtual void VerifyModel(const CEData &data, const CEModel &model,
                             const std::vector<bool> &inlierMarks, double &fitErr) {
        fitErr = 0;

        Point2D e;
        const ushort N = data.Size();
        for(ushort i = 0; i < N; ++i) {
            if(inlierMarks[i])
                fitErr += model.ComputeProjectionSquaredError(data.X(i), data.x(i), e);
        }
    }

    virtual void VerifyModel(const CEData &data, const CEModel &model,
                             std::vector<ushort> &inliers, double &fitErr) {
        inliers.resize(0);
        fitErr = 0;

        m_work.Resize(2);
        ENFT_SSE::__m128 &errSq = m_work[0], &work = m_work[1];
        const ushort N = data.Size(), _N = N - (N & 1);
        const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
        Point2D Zc2;
        for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2) {
            model.ComputeProjectionError2(data.X(ix2), data.X(ix2p1), *px2, Zc2, errSq,
                                          work);
            errSq = ENFT_SSE::_mm_mul_ps(errSq, errSq);
            if(Zc2.v0() > 0 &&
                    (errSq.m128_f32[0] = errSq.m128_f32[0] + errSq.m128_f32[1]) <
                    m_ransacErrorThreshold) {
                inliers.push_back(ix2);
                fitErr += errSq.m128_f32[0];
            }
            if(Zc2.v1() > 0 &&
                    (errSq.m128_f32[2] = errSq.m128_f32[2] + errSq.m128_f32[3]) <
                    m_ransacErrorThreshold) {
                inliers.push_back(ix2p1);
                fitErr += errSq.m128_f32[2];
            }
        }
        if(_N != N) {
            Point2D &e = Zc2;
            if(model.ComputeProjectionError_CheckCheirality(data.X(_N), data.x(_N), e) &&
                    (errSq.m128_f32[0] = e.SquaredLength()) < m_ransacErrorThreshold) {
                inliers.push_back(_N);
                fitErr += errSq.m128_f32[0];
            }
        }
    }

    virtual void VerifyModel(const CEData &data, const CEModel &model,
                             const std::vector<bool> &inlierMarksFix, std::vector<ushort> &inliers,
                             double &fitErr) {
        inliers.resize(0);
        fitErr = 0;

        m_work.Resize(2);
        ENFT_SSE::__m128 &errSq = m_work[0], &work = m_work[1];
        const ushort N = data.Size(), _N = N - (N & 1);
        const ENFT_SSE::__m128 *px2 = (const ENFT_SSE::__m128 *) data.xs().Data();
        Point2D Zc2;
        for(ushort ix2 = 0, ix2p1 = 1; ix2 < _N; ix2 += 2, ix2p1 += 2, ++px2) {
            model.ComputeProjectionError2(data.X(ix2), data.X(ix2p1), *px2, Zc2, errSq,
                                          work);
            errSq = ENFT_SSE::_mm_mul_ps(errSq, errSq);
            if(Zc2.v0() > 0 &&
                    (errSq.m128_f32[0] = errSq.m128_f32[0] + errSq.m128_f32[1]) <
                    m_ransacErrorThreshold) {
                inliers.push_back(ix2);
                fitErr += errSq.m128_f32[0];
            } else if(inlierMarksFix[ix2]) {
                inliers.resize(0);
                fitErr = 0;
                return;
            }
            if(Zc2.v1() > 0 &&
                    (errSq.m128_f32[2] = errSq.m128_f32[2] + errSq.m128_f32[3]) <
                    m_ransacErrorThreshold) {
                inliers.push_back(ix2p1);
                fitErr += errSq.m128_f32[2];
            } else if(inlierMarksFix[ix2p1]) {
                inliers.resize(0);
                fitErr = 0;
                return;
            }
        }
        if(_N != N) {
            Point2D &e = Zc2;
            if(model.ComputeProjectionError_CheckCheirality(data.X(_N), data.x(_N), e) &&
                    (errSq.m128_f32[0] = e.SquaredLength()) < m_ransacErrorThreshold) {
                inliers.push_back(_N);
                fitErr += errSq.m128_f32[0];
            } else if(inlierMarksFix[_N]) {
                inliers.resize(0);
                fitErr = 0;
                return;
            }
        }
    }

    virtual void OptimizeModel(CEData &data, CEModel &model,
                               const ubyte verbose = 0) {
        m_optimizer.m_lmMaxNumIters = m_optimizeMaxNumIters;
        m_optimizer.Run(data, model, verbose >= 2 ? verbose - 2 : 0);
    }
};

#endif