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

#include "stdafx.h"
#include "Sequence/Sequence.h"
#include "Utility/Random.h"
#include "LinearAlgebra/VectorN.h"
#include "SfM/BoundingBox.h"

void Sequence::AddNoiseToCamerasOrPoints(const float percent,
        const bool &toCams, const bool &toPts) {
    std::vector<float> depths;
    ComputeMeasurementDepths(depths);
    const MeasurementIndex ith = (GetMeasurementsNumber() >> 1);
    std::nth_element(depths.begin(), depths.begin() + ith, depths.end());
    const float noiseRange = depths[ith] * percent;

    //srand(5);
    if(toCams) {
        Point3D center;
        float w[3], work[24];
        const FrameIndex nFrms = GetCamerasNumber();
        for(FrameIndex iFrm = 1; iFrm < nFrms; ++iFrm) {
            Camera &C = m_Cs[iFrm];
            C.GetCenter(center);
            center.X() = center.X() + noiseRange * 2 * (Random::GenerateProbability() -
                         0.5f);
            center.Y() = center.Y() + noiseRange * 2 * (Random::GenerateProbability() -
                         0.5f);
            center.Z() = center.Z() + noiseRange * 2 * (Random::GenerateProbability() -
                         0.5f);

            C.ToRodrigues(w);
            w[0] *= 1 + percent * 2 * (Random::GenerateProbability() - 0.5f);
            w[1] *= 1 + percent * 2 * (Random::GenerateProbability() - 0.5f);
            w[2] *= 1 + percent * 2 * (Random::GenerateProbability() - 0.5f);
            C.FromRodrigues(w[0], w[1], w[2], work);
            C.SetCenter(center);
        }
        ComputeProjectiveMatrixes();
    }

    if(toPts) {
        const TrackIndex nTrks = GetPointsNumber();
        for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
            Point3D &X = m_Xs[iTrk];
            X.X() = X.X() + noiseRange * 2 * (Random::GenerateProbability() - 0.5f);
            X.Y() = X.Y() + noiseRange * 2 * (Random::GenerateProbability() - 0.5f);
            X.Z() = X.Z() + noiseRange * 2 * (Random::GenerateProbability() - 0.5f);
        }
    }
}

void Sequence::AddNoiseToMeasurements(const float sigma, const float errTh) {
    const float imgBorder = 5.0f;
    const Point2D xMin(imgBorder, imgBorder), xMax(GetImageWidth() - imgBorder - 1,
            GetImageHeight() - imgBorder - 1);

    const bool measNormaliedBkp = m_measNormalized;
    DenormalizeMeasurements();

    //
    //FILE *fp = fopen("F:/matlab/pdf/pdf.txt", "w");
    const float sigmaSqx2 = sigma * sigma * 2;
    Point2D e;
    float p, err;
    MeasurementIndex nMeas = MeasurementIndex(m_xs.Size());
    for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea) {
        //do p = Random::GenerateProbability(); while(p == 0 || p == 1 || (err = sqrt(-sigmaSqx2 * logf(p))) > errTh);
        //err = sqrt(-sigmaSqx2 * logf(p));
        //e.x() = 2 * Random::GenerateProbability() - 1;
        //e.y() = 2 * Random::GenerateProbability() - 1;
        //e.Normalize();
        //e *= err;
        do {
            e.x() = 2 * Random::GenerateProbability() - 1;
            e.y() = 2 * Random::GenerateProbability() - 1;
            p = e.SquaredLength();
        } while(p > 1 || p == 0 || (err = sqrt(-sigmaSqx2 * logf(p))) > errTh);
        //e *= err / sqrt(p);
        e.Normalize();
        e *= err;
        Point2D &x = m_xs[iMea];
        m_xs[iMea] += e;
        if(x.x() < xMin.x())
            x.x() = xMin.x();
        if(x.x() > xMax.x())
            x.x() = xMax.x();
        if(x.y() < xMin.y())
            x.y() = xMin.y();
        if(x.y() > xMax.y())
            x.y() = xMax.y();

        //fprintf(fp, "%f\n", p);
        //fprintf(fp, "%f\n", e.x() > 0 ? err : -err);
        //fprintf(fp, "%f\n", e.x());
        //fprintf(fp, "%f\n", e.y());
        //fprintf(fp, "%f\n", e.x() > 0 ? sqrt(e.SquaredLength()) : -sqrt(e.SquaredLength()));
    }
    //fclose(fp);
    if(measNormaliedBkp)
        NormalizeMeasurements();
}

void Sequence::AddNoiseToFrameSegments(const float percent) {
    std::vector<float> depths;
    ComputeMeasurementDepths(depths);
    const MeasurementIndex ith = (GetMeasurementsNumber() >> 1);
    std::nth_element(depths.begin(), depths.begin() + ith, depths.end());
    const float noiseRange = depths[ith] * percent;

    //srand(5);
    Point3D dCenter;
    float w[3], work24f[24];
    ENFT_SSE::__m128 work3m[3];
    RigidTransformation3D dT;
    const FrameIndex nFrms = GetCamerasNumber();
    FrameIndex iFrm, iFrm1, iFrm2 = 0;
    TrackIndex iTrk;
    MeasurementIndex iMea;
    AlignedVector<Point3D> Xs(GetMeasurementsNumber());
    while(iFrm2 < nFrms) {
        dCenter.X() = noiseRange * 2 * (Random::GenerateProbability() - 0.5f);
        dCenter.Y() = noiseRange * 2 * (Random::GenerateProbability() - 0.5f);
        dCenter.Z() = noiseRange * 2 * (Random::GenerateProbability() - 0.5f);
        w[0] = percent * 2 * (Random::GenerateProbability() - 0.5f);
        w[1] = percent * 2 * (Random::GenerateProbability() - 0.5f);
        w[2] = percent * 2 * (Random::GenerateProbability() - 0.5f);
        dT.FromRodrigues(w[0], w[1], w[2], work24f);
        dT.SetCenter(dCenter);

        iFrm1 = iFrm2;
        for(iFrm2 = iFrm1 + 1; iFrm2 < nFrms &&
                !(m_frmStates[iFrm2] & FLAG_FRAME_STATE_SPLIT_POINT); ++iFrm2);
        if(iFrm2 != nFrms && iFrm2 != iFrm1 + 1)
            ++iFrm2;
        for(iFrm = iFrm1; iFrm < iFrm2; ++iFrm)
            TransformCamera(iFrm, dT, work3m);

        const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm1],
                               iMea2 = m_mapFrmToMea[iFrm2];
        for(iMea = iMea1; iMea < iMea2; ++iMea) {
            if((iTrk = m_mapMeaToTrk[iMea]) != INVALID_TRACK_INDEX)
                dT.Apply(m_Xs[iTrk], Xs[iMea]);
        }
    }
    ComputeProjectiveMatrixes();
    const TrackIndex nTrks = TrackIndex(m_Xs.Size());
    for(iTrk = 0; iTrk < nTrks; ++iTrk) {
        Point3D &X = m_Xs[iTrk];
        const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        X.SetZero();
        for(FrameIndex i = 0; i < nCrsps; ++i)
            X += Xs[iMeas[i]];
        X *= 1.0f / nCrsps;
        X.reserve() = 1.0f;
    }
}

void Sequence::SynthesizeLoop(const ushort &width, const ushort &height,
                              const float &fx, const float &fy, const float &cx, const float &cy,
                              const FrameIndex nFrms,
                              const TrackIndex nTrks, const FeatureIndex nFtrsPerFrm,
                              const float dCenterPercentMax, const float dAngleMax) {
    const int imgBorder = 5;
    const ushort maxNumTrails = 1000;
    //const float loopSize = 1000.0f;
    //const float sceneSize = 2000.0f;
    //const float minDepth = 10.0f;
    const float loopSize = 1.0f;
    const float sceneSize = 2.0f;
    const float minDepth = 0.01f;
    const float maxDepth = loopSize + sceneSize;
    const MeasurementIndex nMeas = MeasurementIndex(nFrms) * MeasurementIndex(
                                       nFtrsPerFrm);
#if _DEBUG
    assert(nMeas % nTrks == 0);
#endif
    const FrameIndex trkLen = nMeas / nTrks;
//#if _DEBUG
//  assert(nFtrsPerFrm % trkLen == 0);
//#endif
#if _DEBUG
    assert(nTrks % nFrms == 0);
#endif
    const FeatureIndex nFtrsPerStage = nFtrsPerFrm / trkLen;

    //printf("  nFrms = %d, nTrks = %d, nFtrsPerFrm = %d\n", nFrms, nTrks, nFtrsPerFrm);
    //printf("  trkLen = %d, nFtrsPerFrmPerState = %d\n", trkLen, nFtrsPerStage);

    m_tag.Set("", "", 0, 1, nFrms - 1);
    m_tag.SetImageSize(width, height);
    m_K.Set(fx, fy, cx, cy);

    m_Cs.Resize(nFrms);
    m_Ps.Resize(nFrms);
    m_Xs.Resize(nTrks);
    m_xs.Resize(nMeas);
    m_mapFrmToMea.resize(nFrms + 1);
    m_mapTrkToMea.assign(nTrks, MeasurementIndexList());
    //m_mapTrkToPlane.assign(nTrks, INVALID_PLANE_INDEX);
    m_mapMeaToFrm.resize(nMeas);
    m_mapMeaToTrk.resize(nMeas);
    m_frmStates.assign(nFrms, FLAG_FRAME_STATE_SOLVED);
    m_trkStates.assign(nTrks, FLAG_TRACK_STATE_SOLVED | FLAG_TRACK_STATE_INLIER);
    m_meaStates.assign(nMeas, FLAG_MEASUREMENT_STATE_DEFAULT);
    m_trkClrs.resize(nTrks);

    const ENFT_SSE::__m128 loopSize4 = ENFT_SSE::_mm_set1_ps(loopSize);
    const Point3D axis(0, 1, 0);
    const float angleIncr = PIx2 / nFrms;
    const float dCMax = loopSize * dCenterPercentMax;
    Point3D dC, RTt;
    float dAngles[3];
    RotationTransformation3D dR;
    ENFT_SSE::__m128 work;
    float angle = 0;
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm, angle += angleIncr) {
        Camera &C = m_Cs[iFrm];
        C.FromAxisAngle(axis, angle);
        Random::GenerateFloats(-dCMax, dCMax, 3, dC);
        RTt.XYZx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(C.r_20_21_22_x(), loopSize4), dC.XYZx());
        Random::GenerateAngles(-dAngleMax, dAngleMax, 3, dAngles);
        dR.FromEulerAnglesZYX(dAngles[2], dAngles[1], dAngles[0]);
        C.LeftMultiply(dR, C, work);
        C.ApplyRotation(RTt, C.tX(), C.tY(), C.tZ());
        //if(iFrm % 2 == 1)
        //{
        //  const float b = loopSize * PIx2 * 5 / nFrms;
        //  RigidTransformation Rt;
        //  Rt.MakeIdentity();
        //  Rt.tX() = -b;
        //  RigidTransformation::AccumulateTransformation(m_Cs[iFrm - 1], Rt, C, work);
        //}
    }

    Point2D xMin, xMax;
    xMin.Set(float(imgBorder), float(imgBorder));
    xMax.Set(width - imgBorder - 1.0f, height - imgBorder - 1.0f);
    //const IntrinsicMatrix &K = CameraArrayCalibrationParameter::GetIntrinsicMatrix();
    m_K.ImageToNormalizedPlane(xMin);
    m_K.ImageToNormalizedPlane(xMax);
    ComputeProjectiveMatrixes();

    Point2D x;
    Point3D Xc;
    float invZc;
    FrameIndexList iFrms(trkLen);
    std::vector<Point2D> xs(nMeas);
    MeasurementFrameMap mapMeaToFrm(nMeas);
    MeasurementTrackMap mapMeaToTrk(nMeas);
    TrackIndex iTrk = 0;
    MeasurementIndex iMea = 0;
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        //printf("Frame %d: Track %d ...", iFrm, iTrk);
        const Camera &C = m_Cs[iFrm];
        const FrameIndex nFrmsStart = (iFrm > nFrms - trkLen) ? (trkLen -
                                      (nFrms - iFrm)) : 0;
        for(FrameIndex i = 0; i < nFrmsStart; ++i)
            iFrms[i] = i;
        for(FrameIndex i = nFrmsStart, iFrmConsec = iFrm; i < trkLen; ++i, ++iFrmConsec)
            iFrms[i] = iFrmConsec;

        for(FeatureIndex i = 0; i < nFtrsPerStage; ++i) {
            Point3D &Xw = m_Xs[iTrk];
            //for(trialCnt = 0; trialCnt < maxNumTrails; ++trialCnt)
            while(1) {
                x.x() = Random::GenerateFloat(xMin.x(), xMax.x());
                x.y() = Random::GenerateFloat(xMin.y(), xMax.y());
                Xc.Z() = Random::GenerateFloat(minDepth, maxDepth);
                Xc.X() = x.x() * Xc.Z();
                Xc.Y() = x.y() * Xc.Z();
                C.ApplyInversely(Xc, Xw);

                const MeasurementIndex iMeaBkp = iMea;
                bool scc = true;
                for(FrameIndex j = 0; j < trkLen; ++j) {
                    m_Cs[iFrms[j]].ProjectToNormalizedPlane(Xw, invZc, x.x(), x.y());
                    if(invZc < 0 || !(x > xMin && x < xMax)) {
                        scc = false;
                        break;
                    }
                    xs[iMea] = x;
                    mapMeaToFrm[iMea] = iFrms[j];
                    mapMeaToTrk[iMea] = iTrk;
                    ++iMea;
                }
                if(scc)
                    break;
                else
                    iMea = iMeaBkp;
            }
            //if(trialCnt == maxNumTrails)
            //{
            //  Synthesize(fx, fy, cx, cy, width, height, nFrms, nTrks, nFtrsPerFrm, trkLen, dAngleMax, dtMax);
            //  return;
            //}

            ++iTrk;
            printf("\rSynthesizing...%d%%", iTrk * 100 / nTrks);
        }
        //printf(" %d\n", iTrk);
    }
    //SetReferenceFrame(0);
    printf("\rSynthesizing...Done!\n");

    // Reorder measurements from TRACK-ORDER to FRAME_ORDER
    MeasurementIndexList frmMeaIdxs(nFrms);
    m_mapFrmToMea[0] = 0;
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        m_mapFrmToMea[iFrm + 1] = m_mapFrmToMea[iFrm] + nFtrsPerFrm;
        frmMeaIdxs[iFrm] = m_mapFrmToMea[iFrm];
    }
    for(iMea = 0; iMea < nMeas; ++iMea) {
        const FrameIndex iFrm = mapMeaToFrm[iMea];
        const TrackIndex iTrk = mapMeaToTrk[iMea];
        const MeasurementIndex iMeaRe = frmMeaIdxs[iFrm]++;
        m_xs[iMeaRe] = xs[iMea];
        m_mapMeaToFrm[iMeaRe] = iFrm;
        m_mapMeaToTrk[iMeaRe] = iTrk;
        m_mapTrkToMea[iTrk].push_back(iMeaRe);
    }

    m_measNormalized = true;
}

void Sequence::SynthesizeLoopPlanarScene(const ushort &width,
        const ushort &height, const float &fx, const float &fy, const float &cx,
        const float &cy, const FrameIndex nFrms,
        const TrackIndex nTrks, const FeatureIndex nFtrsPerFrm,
        AlignedVector<LA::AlignedVector4f> &Pcs, const float dCenterPercentMax,
        const float dAngleMax) {
    const int imgBorder = 5;
    const ushort maxNumTrails = 1000;
    const float loopSize = 1000.0f;
    const MeasurementIndex nMeas = MeasurementIndex(nFrms) * MeasurementIndex(
                                       nFtrsPerFrm);
    const float maxNx = 1.0f;
    const float maxNy = 1.0f;
#if _DEBUG
    assert(nMeas % nTrks == 0);
#endif
    const FrameIndex trkLen = nMeas / nTrks;
//#if _DEBUG
//  assert(nFtrsPerFrm % trkLen == 0);
//#endif
#if _DEBUG
    assert(nTrks % nFrms == 0);
#endif
    const FeatureIndex nFtrsPerStage = nFtrsPerFrm / trkLen;

    //printf("  nFrms = %d, nTrks = %d, nFtrsPerFrm = %d\n", nFrms, nTrks, nFtrsPerFrm);
    //printf("  trkLen = %d, nFtrsPerFrmPerState = %d\n", trkLen, nFtrsPerStage);

    m_tag.Set("", "", 0, 1, nFrms - 1);
    m_tag.SetImageSize(width, height);
    m_K.Set(fx, fy, cx, cy);

    m_Cs.Resize(nFrms);
    m_Ps.Resize(nFrms);
    m_Xs.Resize(nTrks);
    m_xs.Resize(nMeas);
    m_mapFrmToMea.resize(nFrms + 1);
    m_mapTrkToMea.assign(nTrks, MeasurementIndexList());
    //m_mapTrkToPlane.assign(nTrks, INVALID_PLANE_INDEX);
    m_mapMeaToFrm.resize(nMeas);
    m_mapMeaToTrk.resize(nMeas);
    m_frmStates.assign(nFrms, FLAG_FRAME_STATE_SOLVED);
    m_trkStates.assign(nTrks, FLAG_TRACK_STATE_SOLVED | FLAG_TRACK_STATE_INLIER);
    m_meaStates.assign(nMeas, FLAG_MEASUREMENT_STATE_DEFAULT);
    m_trkClrs.resize(0);

    const ENFT_SSE::__m128 loopSize4 = ENFT_SSE::_mm_set1_ps(loopSize);
    const Point3D axis(0, 1, 0);
    const float angleIncr = PIx2 / nFrms;
    const float dCMax = loopSize * dCenterPercentMax;
    Point3D dC, RTt;
    float dAngles[3];
    RotationTransformation3D dR;
    ENFT_SSE::__m128 work;
    float angle = 0;
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm, angle += angleIncr) {
        Camera &C = m_Cs[iFrm];
        C.FromAxisAngle(axis, angle);
        Random::GenerateFloats(-dCMax, dCMax, 3, dC);
        RTt.XYZx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(C.r_20_21_22_x(), loopSize4), dC.XYZx());
        Random::GenerateAngles(-dAngleMax, dAngleMax, 3, dAngles);
        dR.FromEulerAnglesZYX(dAngles[2], dAngles[1], dAngles[0]);
        C.LeftMultiply(dR, C, work);
        C.ApplyRotation(RTt, C.tX(), C.tY(), C.tZ());
    }

    Point2D xMin, xMax;
    xMin.Set(float(imgBorder), float(imgBorder));
    xMax.Set(width - imgBorder - 1.0f, height - imgBorder - 1.0f);
    m_K.ImageToNormalizedPlane(xMin);
    m_K.ImageToNormalizedPlane(xMax);
    ComputeProjectiveMatrixes();

    const float Nx = Random::GenerateFloat(-maxNx, maxNx);
    const float Ny = Random::GenerateFloat(-maxNy, maxNy);
    const float Nz = sqrt(1 - Nx * Nx - Ny * Ny);
    //const float Nx = 0, Ny = 1, Nz = 0;
    const LA::AlignedVector3f Nw(Nx, Ny, Nz);
    Point2D x;
    Point3D center, Xc;
    float invZc;
    FrameIndexList iFrms(trkLen);
    std::vector<Point2D> xs(nMeas);
    MeasurementFrameMap mapMeaToFrm(nMeas);
    MeasurementTrackMap mapMeaToTrk(nMeas);
    TrackIndex iTrk = 0;
    MeasurementIndex iMea = 0;
    Pcs.Resize(nFrms);
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        //printf("Frame %d: Track %d ...", iFrm, iTrk);
        const Camera &C = m_Cs[iFrm];
        const FrameIndex nFrmsStart = (iFrm > nFrms - trkLen) ? (trkLen -
                                      (nFrms - iFrm)) : 0;
        for(FrameIndex i = 0; i < nFrmsStart; ++i)
            iFrms[i] = i;
        for(FrameIndex i = nFrmsStart, iFrmConsec = iFrm; i < trkLen; ++i, ++iFrmConsec)
            iFrms[i] = iFrmConsec;

        LA::AlignedVector4f &Pc = Pcs[iFrm];
        C.ApplyRotation(Nw.v012x(), Pc.v0123());
        if(Pc.v2() > 0) {
            Pc.v0() = -Pc.v0();
            Pc.v1() = -Pc.v1();
            Pc.v2() = -Pc.v2();
        }
        C.GetCenter(center);
        Pc.v3() = fabs(center.Dot(Nw));
        if(Pc.v3() < 0)
            Pc.v3() = -Pc.v3();
        for(FeatureIndex i = 0; i < nFtrsPerStage; ++i) {
            Point3D &Xw = m_Xs[iTrk];
            //for(trialCnt = 0; trialCnt < maxNumTrails; ++trialCnt)
            while(1) {
                x.x() = Random::GenerateFloat(xMin.x(), xMax.x());
                x.y() = Random::GenerateFloat(xMin.y(), xMax.y());
                Xc.Z() = -Pc.v3() / (Pc.v0() * x.x() + Pc.v1() * x.y() + Pc.v2());
                Xc.X() = x.x() * Xc.Z();
                Xc.Y() = x.y() * Xc.Z();
                C.ApplyInversely(Xc, Xw);

#if _DEBUG
                const float chk = fabs(Xw.Dot(Nw));
#endif

                const MeasurementIndex iMeaBkp = iMea;
                bool scc = true;
                for(FrameIndex j = 0; j < trkLen; ++j) {
                    m_Cs[iFrms[j]].ProjectToNormalizedPlane(Xw, invZc, x.x(), x.y());
                    if(invZc < 0 || !(x > xMin && x < xMax)) {
                        scc = false;
                        break;
                    }
                    xs[iMea] = x;
                    mapMeaToFrm[iMea] = iFrms[j];
                    mapMeaToTrk[iMea] = iTrk;
                    ++iMea;
                }
                if(scc)
                    break;
                else
                    iMea = iMeaBkp;
            }
            //if(trialCnt == maxNumTrails)
            //{
            //  Synthesize(fx, fy, cx, cy, width, height, nFrms, nTrks, nFtrsPerFrm, trkLen, dAngleMax, dtMax);
            //  return;
            //}

            ++iTrk;
            printf("\rSynthesizing...%d%%", iTrk * 100 / nTrks);
        }
        //printf(" %d\n", iTrk);
    }
    SetReferenceFrame(0);
    printf("\rSynthesizing...Done!\n");

    // Reorder measurements from TRACK-ORDER to FRAME_ORDER
    MeasurementIndexList frmMeaIdxs(nFrms);
    m_mapFrmToMea[0] = 0;
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        m_mapFrmToMea[iFrm + 1] = m_mapFrmToMea[iFrm] + nFtrsPerFrm;
        frmMeaIdxs[iFrm] = m_mapFrmToMea[iFrm];
    }
    for(iMea = 0; iMea < nMeas; ++iMea) {
        const FrameIndex iFrm = mapMeaToFrm[iMea];
        const TrackIndex iTrk = mapMeaToTrk[iMea];
        const MeasurementIndex iMeaRe = frmMeaIdxs[iFrm]++;
        m_xs[iMeaRe] = xs[iMea];
        m_mapMeaToFrm[iMeaRe] = iFrm;
        m_mapMeaToTrk[iMeaRe] = iTrk;
        m_mapTrkToMea[iTrk].push_back(iMeaRe);
    }

    m_measNormalized = true;
}

void Sequence::SynthesizeTransformation(const float percentRigid,
                                        const float maxScale, SimilarityTransformation3D &S) const {
    Point3D sum;
    sum.SetZero();
    const TrackIndex nTrks = GetTracksNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
        LA::ApB(m_Xs[iTrk], sum, sum);
    Point3D mean;
    mean.XYZx() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1.0f / nTrks), sum.XYZx());

    Point3D dX;
    std::vector<float> distSqs(nTrks);
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        LA::AmB(m_Xs[iTrk], mean, dX);
        distSqs[iTrk] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(dX.XYZx(), dX.XYZx()));
    }
    const TrackIndex ith = (nTrks >> 1);
    std::nth_element(distSqs.begin(), distSqs.begin() + ith, distSqs.end());
    const float distSqMed = distSqs[ith];
    const float rangeRigid = sqrt(distSqMed) * percentRigid;

    const float sMin = 1 / maxScale, sMax = maxScale;
    Point3D center;
    float w[3], work[24];
    w[0] = Random::GenerateProbability() > 0.5f ? percentRigid *
           (Random::GenerateProbability() - 0.5f) : percentRigid *
           (Random::GenerateProbability() - 0.5f);
    w[1] = Random::GenerateProbability() > 0.5f ? percentRigid *
           (Random::GenerateProbability() - 0.5f) : percentRigid *
           (Random::GenerateProbability() - 0.5f);
    w[2] = Random::GenerateProbability() > 0.5f ? percentRigid *
           (Random::GenerateProbability() - 0.5f) : percentRigid *
           (Random::GenerateProbability() - 0.5f);
    //memset(w, 0, sizeof(w));
    center.X() = percentRigid * 2 * (Random::GenerateProbability() - 0.5f);
    center.Y() = percentRigid * 2 * (Random::GenerateProbability() - 0.5f);
    center.Z() = percentRigid * 2 * (Random::GenerateProbability() - 0.5f);
    //center.SetZero();
    S.FromRodrigues(w[0], w[1], w[2], work);
    S.SetCenter(center);
    S.SetScale(Random::GenerateFloat(sMin, sMax));
}

static inline void ComputeBoundingBox(const AlignedVector<Point2D> &xs,
                                      BoundingBox2D &B) {
    B.Initialize();
    const uint N = xs.Size();
    for(uint i = 0; i < N; ++i)
        B.Include(xs[i]);
}

void Sequence::SynthesizeIntrinsicRectification(const float maxFocal,
        const float maxDistortion) {
    if(m_intrinsicType == INTRINSIC_USER_FIXED)
        return;
    NormalizeMeasurements();

    BoundingBox2D B1, B2;
    ComputeBoundingBox(m_xs, B1);

//#if _DEBUG
//  const AlignedVector<Point2D> xs1 = m_xs;
//#endif

    const float minFocal = 1 / maxFocal;
    if(m_intrinsicType == INTRINSIC_CONSTANT) {
        m_Kr.f() = Random::GenerateFloat(minFocal, maxFocal);
        m_Kr.d() = Random::GenerateFloat(-maxDistortion, maxDistortion);
        ScaleMeasurements(m_Kr.f(), m_xs);
        DistortMeasurements(m_K.fxy(), m_Kr.d(), m_xs);
//#if _DEBUG
//      m_Kr.Print();
//#endif
    } else {
        FrameIndex iFrm;
        const FrameIndex nFrms = FrameIndex(m_Cs.Size());
        std::vector<float> fs(nFrms);
        for(iFrm = 0; iFrm < nFrms; ++iFrm)
            fs[iFrm] = Random::GenerateFloat(minFocal, maxFocal);
        std::sort(fs.begin(), fs.end());

        AlignedVector<Point2D> xsTmp;
        const float d = Random::GenerateFloat(-maxDistortion,
                                              maxDistortion) * fs[0] * fs[0];
        for(iFrm = 0; iFrm < nFrms; ++iFrm) {
            Camera::IntrinsicParameter &Kr = m_Krs[iFrm];
            m_Krs[iFrm].Set(fs[iFrm], d / (fs[iFrm] * fs[iFrm]));
            Point2D *xs = m_xs.Data() + m_mapFrmToMea[iFrm];
            const MeasurementIndex nMeas = m_mapFrmToMea[iFrm + 1] - m_mapFrmToMea[iFrm];
            xsTmp.Resize(nMeas);
            xsTmp.CopyFrom(xs);
            ScaleMeasurements(Kr.f(), xsTmp);
            DistortMeasurements(m_K.fxy(), Kr.d(), xsTmp);
            xsTmp.CopyTo(xs);
//#if _DEBUG
//          printf("Frame %d: ", iFrm);
//          Kr.Print();
//#endif
        }
    }
    ComputeProjectiveMatrixes();

    ComputeBoundingBox(m_xs, B2);
    const float sx = B2.GetWidth() / B1.GetWidth(),
                sy = B2.GetHeight() / B1.GetHeight(), s = std::max(sx, sy);
    const ushort width = ushort(m_tag.GetImageWidth() * s + 0.5f),
                 height = ushort(m_tag.GetImageHeight() * s + 0.5f);
    const float cx = m_K.cx() * (width - 1) / (m_tag.GetImageWidth() - 1),
                cy = m_K.cy() * (height - 1) / (m_tag.GetImageHeight() - 1);
    m_tag.SetImageSize(width, height);
    m_K.Set(m_K.fx(), m_K.fy(), cx, cy);

//#if _DEBUG
//  const AlignedVector<Point2D> xs2 = m_xs;
//  RectifyMeasurements(m_xs);
//  const uint N = m_xs.Size();
//  for(uint i = 0; i < N; ++i)
//  {
//      const Point2D &x1 = xs1[i], &x2 = xs2[i], &x3 = m_xs[i];
//      printf("%d: (%.2f, %.2f) --> (%.2f, %.2f) --> (%.2f, %.2f), error = %f\n", i, x1.x(), x1.y(), x2.x(), x2.y(), x3.x(), x3.y(), x1.SquaredDistance(x3));
//  }
//#endif
}

void Sequence::SynthesizeProjectiveTransformation(const float maxVal,
        AbsoluteQuadric &Q) {
    SetReferenceFrame(0);

    switch(m_intrinsicType) {
        case INTRINSIC_USER_FIXED:
            Q.f2() = 1.0f;
            break;
        case INTRINSIC_CONSTANT:
            Q.f2() = m_Kr.f() * m_Kr.f();
            break;
        case INTRINSIC_VARIABLE:
            Q.f2() = m_Krs[0].f() * m_Krs[0].f();
            break;
    }
    const float p0 = Random::GenerateFloat(-maxVal, maxVal);
    const float p1 = Random::GenerateFloat(-maxVal, maxVal);
    const float p2 = Random::GenerateFloat(-maxVal, maxVal);
    Q.a0() = -Q.f2() * p0;
    Q.a1() = -Q.f2() * p1;
    Q.a2() = -p2;
    Q.b() = Q.f2() * p0 * p0 + Q.f2() * p1 * p1 + p2 * p2;
//#if _DEBUG
//  Q.Print();
//#endif

    //const float f = sqrt(Q.f2()), fI = 1 / f, nfp0 = -f * p0, nfp1 = -f * p1, nfp2 = -f * p2;
    const float f = sqrt(Q.f2()), fI = 1 / f;
    const ENFT_SSE::__m128 tI3 = ENFT_SSE::_mm_setr_ps(-f * p0, -f * p1, -p2, 1.0f);

#ifdef _DEBUG_WITH_EIGEN
    EigenMatrix T;
    T.resize(4, 4);
    T.setZero();
    T(0, 0) = T(1, 1) = fI;
    T(2, 2) = T(3, 3) = 1.0f;
    T(3, 0) = p0;
    T(3, 1) = p1;
    T(3, 2) = p2;
    const EigenMatrix TI = T.inverse();
    //PrintM(TI);

    EigenMatrix Qp;
    Qp.resize(4, 4);
    Qp.setZero();
    Qp(0, 0) = Qp(1, 1) = Q.f2();
    Qp(2, 2) = 1.0f;
    Qp(0, 3) = Qp(3, 0) = Q.a0();
    Qp(1, 3) = Qp(3, 1) = Q.a1();
    Qp(2, 3) = Qp(3, 2) = Q.a2();
    Qp(3, 3) = Q.b();
    //const EigenMatrix Qm = T * Qp * T.transpose();
    //PrintM(Qm);
#endif

    ProjectiveMatrix Pm;
    const FrameIndex nFrms = FrameIndex(m_Ps.Size());
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
            continue;
        Pm = m_Ps[iFrm];
        ProjectiveMatrix &Pp = m_Ps[iFrm];
        Pp.M00() = Pm.M00() * fI + Pm.M03() * p0;
        Pp.M01() = Pm.M01() * fI + Pm.M03() * p1;
        Pp.M02() = Pm.M02() + Pm.M03() * p2;
        Pp.M03() = Pm.M03();
        Pp.M10() = Pm.M10() * fI + Pm.M13() * p0;
        Pp.M11() = Pm.M11() * fI + Pm.M13() * p1;
        Pp.M12() = Pm.M12() + Pm.M13() * p2;
        Pp.M13() = Pm.M13();
        Pp.M20() = Pm.M20() * fI + Pm.M23() * p0;
        Pp.M21() = Pm.M21() * fI + Pm.M23() * p1;
        Pp.M22() = Pm.M22() + Pm.M23() * p2;
        Pp.M23() = Pm.M23();
#ifdef _DEBUG_WITH_EIGEN
        EigenMatrix P1, P2;
        SetM(Pm, P1);
        P1 = P1 * T;
        SetM(Pp, P2);
        CheckM(P1, P2);
#endif
    }

    Point3D Xm;
    const TrackIndex nTrks = TrackIndex(m_Xs.Size());
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(!(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED))
            continue;
        Xm = m_Xs[iTrk];
#if _DEBUG
        assert(Xm.reserve() == 1.0f);
#endif
        Point3D &Xp = m_Xs[iTrk];
        Xp.X() = f * Xm.X();
        Xp.Y() = f * Xm.Y();
        Xp.Z() = Xm.Z();
        Xp *= 1 / ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(tI3, Xm.XYZx()));
        Xp.reserve() = 1.0f;
#ifdef _DEBUG_WITH_EIGEN
        EigenVector X1, X2;
        SetV(Xm, X1);
        X1 = TI * X1;
        X1 /= X1(3);
        SetV(Xp, X2);
        CheckV(X1, X2);
#endif
    }
}

void Sequence::AnalyzeError(const std::vector<float> &errs, float &avg,
                            float &std, float &inf, uint &iMax, const float &errMedScaleFactor) {
    const uint N = uint(errs.size());
    if(N == 0) {
        avg = std = inf = FLT_MAX;
        iMax = 0;
    } else if(N == 1) {
        avg = std = inf = errs[0];
        iMax = 1;
    } else {
        std::vector<std::pair<float, uint> > errsOrdered(N);
        for(uint i = 0; i < N; ++i)
            errsOrdered[i] = std::make_pair(errs[i], i);
        std::sort(errsOrdered.begin(), errsOrdered.end());

        const float errMed = errsOrdered[(N + 1) >> 1].first;
        const float errMax = errMedScaleFactor == FLT_MAX ? FLT_MAX : errMed *
                             errMedScaleFactor;
        LA::AlignedVectorXf errsStable;
        errsStable.Resize(N);
        for(iMax = 0; iMax < N && errsOrdered[iMax].first <= errMax; ++iMax)
            errsStable[iMax] = errsOrdered[iMax].first;
        errsStable.Resize(iMax);

        LA::Distribution(errsStable, avg, std);
        inf = LA::NormLinf(errsStable);
    }
}

void Sequence::AnalyzeAndPrintError(const std::vector<float> &errs,
                                    const float &errMedScaleFactor) {
    float avg, std, inf;
    uint iMax;
    AnalyzeError(errs, avg, std, inf, iMax, errMedScaleFactor);
    printf("%f +- %f <= %f (%d / %d)\n", avg, std, inf, iMax, errs.size());
}

void Sequence::PushBackCameraError(const Camera &C1, const Camera &C2,
                                   std::vector<float> &qAbsErrs, std::vector<float> &qRelErrs,
                                   std::vector<float> &wAbsErrs,
                                   std::vector<float> &wRelErrs, std::vector<float> &tAbsErrs,
                                   std::vector<float> &tRelErrs, std::vector<float> &cAbsErrs,
                                   std::vector<float> &cRelErrs) {
    float absErr, relErr;
    LA::Vector4f q1, q2;
    LA::Vector3f w1, w2;
    Point3D t1, t2, c1, c2;

    C1.ToQuaternion(q1);
    C2.ToQuaternion(q2);
    LA::ComputeError(q1, q2, absErr, relErr);
    qAbsErrs.push_back(absErr);
    qRelErrs.push_back(relErr);

    C1.ToRodrigues(w1);
    C2.ToRodrigues(w2);
    LA::ComputeError(w1, w2, absErr, relErr);
    wAbsErrs.push_back(absErr);
    wRelErrs.push_back(relErr);

    C1.GetTranslation(t1);
    C2.GetTranslation(t2);
    LA::ComputeError(t1, t2, absErr, relErr);
    tAbsErrs.push_back(absErr);
    tRelErrs.push_back(relErr);

    C1.GetCenter(c1);
    C2.GetCenter(c2);
    LA::ComputeError(c1, c2, absErr, relErr);
    cAbsErrs.push_back(absErr);
    cRelErrs.push_back(relErr);
}

void Sequence::PrintCameraErrors(const std::vector<float> &qAbsErrs,
                                 const std::vector<float> &qRelErrs, const std::vector<float> &wAbsErrs,
                                 const std::vector<float> &wRelErrs, const std::vector<float> &tAbsErrs,
                                 const std::vector<float> &tRelErrs,
                                 const std::vector<float> &cAbsErrs, const std::vector<float> &cRelErrs,
                                 const float errMedScaleFactor) {
    printf("Camera absolute error\n");
    printf("  q = ");
    AnalyzeAndPrintError(qAbsErrs, errMedScaleFactor);
    printf("  w = ");
    AnalyzeAndPrintError(wAbsErrs, errMedScaleFactor);
    printf("  t = ");
    AnalyzeAndPrintError(tAbsErrs, errMedScaleFactor);
    printf("  c = ");
    AnalyzeAndPrintError(cAbsErrs, errMedScaleFactor);
    printf("Camera relative error\n");
    printf("  q = ");
    AnalyzeAndPrintError(qRelErrs, errMedScaleFactor);
    printf("  w = ");
    AnalyzeAndPrintError(wRelErrs, errMedScaleFactor);
    printf("  t = ");
    AnalyzeAndPrintError(tRelErrs, errMedScaleFactor);
    printf("  c = ");
    AnalyzeAndPrintError(cRelErrs, errMedScaleFactor);
}

void Sequence::PushBackPointError(const Point3D &X1, const Point3D &X2,
                                  std::vector<float> &absErrs, std::vector<float> &relErrs) {
    float absErr, relErr;
    LA::ComputeError(X1, X2, absErr, relErr);
    absErrs.push_back(absErr);
    relErrs.push_back(relErr);
}

void Sequence::PrintPointErrors(const std::vector<float> &absErrs,
                                const std::vector<float> &relErrs, const float errMedScaleFactor) {
    printf("Point absolute error = ");
    AnalyzeAndPrintError(absErrs, errMedScaleFactor);
    printf("Point relative error = ");
    AnalyzeAndPrintError(relErrs, errMedScaleFactor);
}

void Sequence::PushBackIntrinsicRectificationError(const
        Camera::IntrinsicParameter &Kr1, const Camera::IntrinsicParameter &Kr2,
        std::vector<float> &absErrs,
        std::vector<float> &relErrs) {
    float absErr, relErr;
    const LA::Vector2f e(Kr1.f() - Kr2.f(), Kr1.d() - Kr2.d());
    absErr = sqrt(e.SquaredLength());
    const float len1 = sqrt(Kr1.f() * Kr1.f() + Kr1.d() * Kr1.d()),
                len2 = sqrt(Kr2.f() * Kr2.f() + Kr2.d() * Kr2.d()), lenMax = std::max(len1,
                        len2);
    if(lenMax == 0)
        relErr = 0;
    else
        relErr = absErr / lenMax;
    absErrs.push_back(absErr);
    relErrs.push_back(relErr);
}

void Sequence::PushBackIntrinsicRectificationError(const float &f1,
        const float &f2, std::vector<float> &absErrs, std::vector<float> &relErrs) {
    float absErr, relErr;
    absErr = fabs(f1 - f2);
    const float fMax = std::max(f1, f2);
    if(fMax == 0)
        relErr = 0;
    else
        relErr = absErr / fMax;
    absErrs.push_back(absErr);
    relErrs.push_back(relErr);
}

void Sequence::PrintIntrinsicRectificationErrors(const std::vector<float>
        &absErrs, const std::vector<float> &relErrs,
        const float errMedScaleFactor /* = 2.0f */) {
    printf("Intrinsic absolute error = ");
    AnalyzeAndPrintError(absErrs, errMedScaleFactor);
    printf("Intrinsic relative error = ");
    AnalyzeAndPrintError(relErrs, errMedScaleFactor);
}

void Sequence::PushBackProjectiveMatrixError(const ProjectiveMatrix &P1,
        const ProjectiveMatrix &P2, std::vector<float> &absErrs,
        std::vector<float> &relErrs) {
    LA::AlignedVector11f p1, p2, e;
    const ENFT_SSE::__m128 s1 = ENFT_SSE::_mm_set1_ps(1 / P1.FrobeniusNorm()),
                 s2 = ENFT_SSE::_mm_set1_ps(1 / P2.FrobeniusNorm());
    p1.v0123() = ENFT_SSE::_mm_mul_ps(s1, P1.M_00_01_02_03());
    p2.v0123() = ENFT_SSE::_mm_mul_ps(s2, P2.M_00_01_02_03());
    p1.v4567() = ENFT_SSE::_mm_mul_ps(s1, P1.M_10_11_12_13());
    p2.v4567() = ENFT_SSE::_mm_mul_ps(s2, P2.M_10_11_12_13());
    p1.v8910x() = ENFT_SSE::_mm_mul_ps(s1, P1.M_20_21_22_23());
    p2.v8910x() = ENFT_SSE::_mm_mul_ps(s2, P2.M_20_21_22_23());
    p1.reserve() = 0.0f;
    p2.reserve() = 0.0f;
    LA::AmB(p1, p2, e);

    float absErr, relErr;
    absErr = fabs(e.SquaredLength());
    const float len1 = sqrt(p1.SquaredLength()), len2 = sqrt(p2.SquaredLength()),
                lenMax = std::max(len1, len2);
    if(lenMax == 0)
        relErr = 0;
    else
        relErr = absErr / lenMax;
    absErrs.push_back(absErr);
    relErrs.push_back(relErr);
}

void Sequence::PrintProjectiveMatrixErrors(const std::vector<float> &absErrs,
        const std::vector<float> &relErrs, const float errMedScaleFactor) {
    printf("Projective absolute error = ");
    AnalyzeAndPrintError(absErrs, errMedScaleFactor);
    printf("Projective relative error = ");
    AnalyzeAndPrintError(relErrs, errMedScaleFactor);
}