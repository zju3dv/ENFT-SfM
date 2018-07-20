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
#include "SequenceDepth.h"
#include "Sequence/SequenceBundleAdjustorDataDepth.h"
#include "Utility/Utility.h"
#include "Utility/Random.h"
#include <cvd/image_io.h>

void SequenceDepth::operator = (const SequenceDepth &seq) {
    *((Sequence *) this) = seq;
    m_tagDep = seq.m_tagDep;
    m_tagNormal = seq.m_tagNormal;
    m_ds = seq.m_ds;
}

void SequenceDepth::Swap(SequenceDepth &seq) {
    Sequence::Swap(seq);
    m_tagDep.Swap(seq.m_tagDep);
    m_tagNormal.Swap(seq.m_tagNormal);
    m_ds.Swap(seq.m_ds);
}

void SequenceDepth::SetTag(const std::string &seqDir,
                           const std::string &seqNameRGB, const std::string &seqNameDep,
                           const std::string &seqNameNormal,
                           const int &iStart, const int &iStep,
                           const int &iEnd) {
    Sequence::SetTag(seqDir, seqNameRGB, iStart, iStep, iEnd);
    m_tagDep.Set(seqDir, seqNameDep, iStart, iStep, iEnd, m_tag.GetImageWidth(),
                 m_tag.GetImageHeight());
    m_tagNormal.Set(seqDir, seqNameNormal, iStart, iStep, iEnd,
                    m_tag.GetImageWidth(), m_tag.GetImageHeight());
}

void SequenceDepth::GetFrameInlierDepthMeasurementIndexList(
    const FrameIndex &iFrm, MeasurementIndexList &iMeas) const {
    TrackIndex iTrk;
    MeasurementIndex iMea;
    iMeas.resize(0);
    const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm],
                           iMea2 = m_mapFrmToMea[iFrm + 1];
    for(iMea = iMea1; iMea < iMea2; ++iMea) {
        if(m_ds[iMea] != 0.0f && (iTrk = m_mapMeaToTrk[iMea]) != INVALID_TRACK_INDEX &&
                (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER)
                && !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER) &&
                !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH))
            iMeas.push_back(iMea);
    }
}

FrameIndex SequenceDepth::CountTrackInlierDepths(const TrackIndex &iTrk) const {
    FrameIndex cnt = 0;
    MeasurementIndex iMea;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for(FrameIndex i = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        if((m_frmStates[m_mapMeaToFrm[iMea]] & FLAG_FRAME_STATE_SOLVED) &&
                !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER)
                && !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH) &&
                m_ds[iMea] > 0.0f)
            ++cnt;
    }
    return cnt;
}

void SequenceDepth::GetCameraEstimatorData(const FrameIndex &iFrm,
        CameraEstimatorDataDepth &data, MeasurementIndexList &iMeas) const {
    const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm],
                           iMea2 = m_mapFrmToMea[iFrm + 1];
    const FeatureIndex N = iMea2 - iMea1;
    data.Resize(N);
    iMeas.resize(N);

    TrackIndex iTrk;
    FeatureIndex i = 0;
    for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
        if((iTrk = m_mapMeaToTrk[iMea]) == INVALID_TRACK_INDEX ||
                !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
            continue;
        data.SetMatch(i, m_Xs[iTrk], m_xs[iMea], m_ds[iMea]);
        iMeas[i] = iMea;
        ++i;
    }
    data.Resize(i);
    iMeas.resize(i);
    if(!m_measNormalized)
        m_K.ImageToNormalizedPlaneN(data.xs());
    data.FinishSettingMatches();
    data.SetFocal(m_K.fxy());
    //data.SetArsacData(GetImageWidth(), GetImageHeight(), m_K, m_measNormalized);
}

void SequenceDepth::GetPoint3DEstimatorData(const TrackIndex &iTrk,
        Point3DEstimatorDataDepth &data, MeasurementIndexList &iMeas) const {
    const MeasurementIndexList &iMeasAll = m_mapTrkToMea[iTrk];
    MeasurementIndex iMea;
    FrameIndex iFrm;
    FrameIndex i, j;
    const FrameIndex nCrsps = FrameIndex(iMeasAll.size());
    data.Resize(nCrsps);
    iMeas.resize(nCrsps);
    for(i = j = 0; i < nCrsps; ++i) {
        iMea = iMeasAll[i];
        iFrm = m_mapMeaToFrm[iMea];
        if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
            continue;
        data.SetMatch(j, m_Cs[iFrm], m_xs[iMea], m_ds[iMea]);
        iMeas[j++] = iMea;
    }
    const FrameIndex N = j;
    data.Resize(N);
    iMeas.resize(N);
    if(!m_measNormalized)
        m_K.ImageToNormalizedPlaneN(data.xs());
    data.FinishSettingMatches();
    data.SetFocal(m_K.fxy());
}

void SequenceDepth::GetPoint3DEstimatorDataInlier(const TrackIndex &iTrk,
        Point3DEstimatorDataDepth &data) const {
#if _DEBUG
    assert(m_intrinsicType == INTRINSIC_USER_FIXED);
#endif
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    MeasurementIndex iMea;
    FrameIndex iFrm;
    FrameIndex i, j;
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    data.Resize(nCrsps);
    for(i = 0, j = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        iFrm = m_mapMeaToFrm[iMea];
        if((m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) &&
                !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
            data.SetMatch(j++, m_Cs[iFrm], m_xs[iMea], m_ds[iMea]);
    }
    data.Resize(j);
    if(!m_measNormalized)
        m_K.ImageToNormalizedPlaneN(data.xs());
    data.FinishSettingMatches();
    data.SetFocal(m_K.fxy());
}

//void SequenceDepth::GetScaleEstimatorData(ScaleEstimatorData &data, MeasurementIndexList &iMeas) const
//{
//#if _DEBUG
//  assert(m_intrinsicType == INTRINSIC_USER_FIXED);
//#endif
//  iMeas.resize(0);
//  FrameIndex iFrm;
//  TrackIndex iTrk;
//  MeasurementIndex iMea;
//  const FrameIndex nFrms = GetFramesNumber();
//  for(iFrm = 0; iFrm < nFrms; ++iFrm)
//  {
//      if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
//          continue;
//      const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
//      for(iMea = iMea1; iMea < iMea2; ++iMea)
//      {
//          if(m_ds[iMea] > 0.0f && (iTrk = m_mapMeaToTrk[iMea]) != INVALID_TRACK_INDEX
//          && (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) && !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
//              iMeas.push_back(iMea);
//      }
//  }
//
//  const uint N = uint(iMeas.size());
//  data.Resize(N);
//  for(uint i = 0; i < N; ++i)
//  {
//      iMea = iMeas[i];
//      data.d1(i) = m_Cs[m_mapMeaToFrm[iMea]].ComputeDepth(m_Xs[m_mapMeaToTrk[iMea]]);
//      data.d2(i) = m_ds[iMea];
//  }
//  data.ComputeScales();
//}

void SequenceDepth::SearchForFrameFeatureMatches(const FrameIndex &iFrm1,
        const FrameIndex &iFrm2, FeatureMatchList &matches,
        RelativePoseEstimatorDataDepth &data) const {
    Sequence::SearchForFrameFeatureMatches(iFrm1, iFrm2, matches);

    ushort i, j;
    FeatureIndex iFtr1, iFtr2;
    float d1, d2;
    Point3D X1, X2;
    const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm1],
                           iMea2 = m_mapFrmToMea[iFrm2];
    const Point2D *xs1 = m_xs.Data() + iMea1, *xs2 = m_xs.Data() + iMea2;
    const float *ds1 = m_ds.Data() + iMea1, *ds2 = m_ds.Data() + iMea2;
    const ushort nMatches2D = ushort(matches.size());
    data.Resize2D(nMatches2D);
    data.Resize3D(nMatches2D);
    for(i = 0; i < nMatches2D; ++i) {
        matches[i].Get(iFtr1, iFtr2);
        const Point2D &x1 = xs1[iFtr1], &x2 = xs2[iFtr2];
        data.SetMatch2D(i, x1, x2);
    }
    data.FinishSettingMatches2D();
    for(i = j = 0; i< nMatches2D; ++i) {
        matches[i].Get(iFtr1, iFtr2);
        const Point2D &x1 = xs1[iFtr1], &x2 = xs2[iFtr2];
        if((d1 = ds1[iFtr1]) == 0.0f || (d2 = ds2[iFtr2]) == 0.0f)
            continue;
        X1.Set(x1.x() * d1, x1.y() * d1, d1);
        X2.Set(x2.x() * d2, x2.y() * d2, d2);
        data.SetMatch3D(j++, i, X1, X2);
    }
    data.Resize3D(j);
}

void SequenceDepth::InitializeMeasurements() {
    const MeasurementIndex nMeas = MeasurementIndex(m_meaStates.size());
    for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea) {
        MarkMeasurementInlier(iMea);
        MarkMeasurementDepthOutlier(iMea);
    }
}

void SequenceDepth::RemoveNullMeasurements() {
    if(m_ds.Empty()) {
        Sequence::RemoveNullMeasurements();
        return;
    }

    const MeasurementIndex nMeasOri = GetMeasurementsNumber();
    MeasurementIndexList iMeasOriToNew(nMeasOri, INVALID_MEASUREMENT_INDEX);

    MeasurementIndex iMeaOri, iMeaNew = 0;
    const FrameIndex nFrms = GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        const MeasurementIndex iMeaOri1 = m_mapFrmToMea[iFrm],
                               iMeaOri2 = m_mapFrmToMea[iFrm + 1];
        m_mapFrmToMea[iFrm] = iMeaNew;
        for(iMeaOri = iMeaOri1; iMeaOri < iMeaOri2; ++iMeaOri) {
            if(m_mapMeaToTrk[iMeaOri] == INVALID_TRACK_INDEX)
                continue;
            m_xs[iMeaNew] = m_xs[iMeaOri];
            m_ds[iMeaNew] = m_ds[iMeaOri];
            m_mapMeaToFrm[iMeaNew] = m_mapMeaToFrm[iMeaOri];
            m_mapMeaToTrk[iMeaNew] = m_mapMeaToTrk[iMeaOri];
            m_meaStates[iMeaNew] = m_meaStates[iMeaOri];
#if DESCRIPTOR_TRACK == 0
            m_descs[iMeaNew] = m_descs[iMeaOri];
#endif
            iMeasOriToNew[iMeaOri] = iMeaNew;
            ++iMeaNew;
        }
    }
    const MeasurementIndex nMeasNew = iMeaNew;
    m_xs.Resize(nMeasNew);
    m_ds.Resize(nMeasNew);
    m_mapFrmToMea.back() = nMeasNew;
    m_mapMeaToFrm.resize(nMeasNew);
    m_mapMeaToTrk.resize(nMeasNew);
    m_meaStates.resize(nMeasNew);
#if DESCRIPTOR_TRACK == 0
    m_descs.Resize(nMeasNew);
#endif

    const TrackIndex nTrks = GetTracksNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        for(FrameIndex i = 0; i < nCrsps; ++i) {
            iMeaNew = iMeasOriToNew[iMeas[i]];
#if _DEBUG
            assert(iMeaNew != INVALID_MEASUREMENT_INDEX);
#endif
            iMeas[i] = iMeaNew;
        }
    }
}

void SequenceDepth::GetSubSequence(const FrameIndexList &iFrms,
                                   Sequence &seqSub, TrackIndexList &iTrks, MeasurementIndexList &iMeas,
                                   const bool copyDesc,
                                   const bool copyClr) const {
    Sequence::GetSubSequence(iFrms, seqSub, iTrks, iMeas, copyDesc, copyClr);
    SequenceDepth *pSeqSub = (SequenceDepth *) &seqSub;
    m_tagDep.GetSubSequence(iFrms, pSeqSub->m_tagDep);
    m_tagNormal.GetSubSequence(iFrms, pSeqSub->m_tagNormal);
    const MeasurementIndex nMeas = MeasurementIndex(iMeas.size());
    pSeqSub->m_ds.Resize(nMeas);
    for(MeasurementIndex i = 0; i < nMeas; ++i)
        pSeqSub->m_ds[i] = m_ds[iMeas[i]];
}

void SequenceDepth::GetSubSequence(const FrameIndexList &iFrms,
                                   const TrackIndexList &iTrks, Sequence &seqSub, MeasurementIndexList &iMeas,
                                   const bool copyDesc,
                                   const bool copyClr) const {
    Sequence::GetSubSequence(iFrms, iTrks, seqSub, iMeas, copyDesc, copyClr);
    SequenceDepth *pSeqSub = (SequenceDepth *) &seqSub;
    m_tagDep.GetSubSequence(iFrms, pSeqSub->m_tagDep);
    m_tagNormal.GetSubSequence(iFrms, pSeqSub->m_tagNormal);
    const MeasurementIndex nMeas = MeasurementIndex(iMeas.size());
    pSeqSub->m_ds.Resize(nMeas);
    for(MeasurementIndex i = 0; i < nMeas; ++i)
        pSeqSub->m_ds[i] = m_ds[iMeas[i]];
}

void SequenceDepth::GetBundleAdjustorData(SequenceBundleAdjustorDataDepth &data)
const {
#if _DEBUG
    assert(m_intrinsicType == INTRINSIC_USER_FIXED);
#endif
    data.m_fxy = m_K.fxy();
    data.m_G = m_Kr;
    data.m_Cs = m_Cs;
    data.m_Xs = m_Xs;
    data.m_mapFrmToMea = m_mapFrmToMea;
    data.m_mapTrkToMea = m_mapTrkToMea;
    data.m_mapMeaToFrm = m_mapMeaToFrm;
    data.m_mapMeaToTrk = m_mapMeaToTrk;
    const MeasurementIndex nMeas = MeasurementIndex(m_xs.Size());
    data.m_xs.Resize(nMeas);
    for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea)
        data.m_xs[iMea].Set(m_xs[iMea].x(), m_xs[iMea].y(), m_ds[iMea]);
}

void SequenceDepth::SetBunbleAdjustmentResults(const
        SequenceBundleAdjustorDataDepth &data) {
#if _DEBUG
    assert(m_intrinsicType == INTRINSIC_USER_FIXED);
#endif
    m_Cs.CopyFrom(data.m_Cs);
    m_Xs.CopyFrom(data.m_Xs);
    ComputeProjectiveMatrixes();
}

void SequenceDepth::GetBundleAdjustorData(const FrameIndexList &iFrmsAdj,
        SequenceBundleAdjustorDataDepth &data, FrameIndexList &iFrmsBA,
        TrackIndexList &iTrksBA) const {
#if _DEBUG
    assert(m_intrinsicType == INTRINSIC_USER_FIXED);
#endif
    // Step1: mark adjusted frames
    FrameStateList frmStates = m_frmStates;
    const FrameIndex nFrmsAdj = FrameIndex(iFrmsAdj.size());
    for(FrameIndex i = 0; i < nFrmsAdj; ++i)
        frmStates[iFrmsAdj[i]] |= FLAG_FRAME_STATE_ADJUSTED;

    // Step2: mark fixed frames, collect BA tracks, create reverse track index list iTrksSrcToDst and count BA measurements
    iTrksBA.resize(0);
    TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
    FrameIndex iFrmSrc, iFrmDst;
    TrackIndex iTrkSrc, iTrkDst = 0;
    MeasurementIndex iMeaSrc, iMeaDst = 0;
    for(FrameIndex i = 0; i < nFrmsAdj; ++i) {
        const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmsAdj[i]],
                               iMeaSrc2 = m_mapFrmToMea[iFrmsAdj[i] + 1];
        for(MeasurementIndex j = iMeaSrc1; j < iMeaSrc2; ++j) {
            iTrkSrc = m_mapMeaToTrk[j];
            if(iTrkSrc == INVALID_TRACK_INDEX ||
                    iTrksSrcToDst[iTrkSrc] != INVALID_TRACK_INDEX ||
                    !(m_trkStates[iTrkSrc] & FLAG_TRACK_STATE_INLIER)
                    || (m_meaStates[j] & FLAG_MEASUREMENT_STATE_OUTLIER))
                continue;
            iTrksSrcToDst[iTrkSrc] = iTrkDst++;
            iTrksBA.push_back(iTrkSrc);

            const MeasurementIndexList &iMeasSrc = m_mapTrkToMea[iTrkSrc];
            const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
            for(FrameIndex k = 0; k < nCrspsSrc; ++k) {
                iMeaSrc = iMeasSrc[k];
                if(m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER)
                    continue;
                iFrmSrc = m_mapMeaToFrm[iMeaSrc];
                if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_SOLVED))
                    continue;
                ++iMeaDst;
                if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_ADJUSTED))
                    frmStates[iFrmSrc] |= FLAG_FRAME_STATE_FIXED;
            }
        }
    }

    // Step3: collect BA frames
    iFrmsBA.resize(0);
    const FrameIndex nFrmsSrc = GetFramesNumber();
    for(iFrmSrc = 0; iFrmSrc < nFrmsSrc; ++iFrmSrc) {
        if(frmStates[iFrmSrc] & FLAG_FRAME_STATE_FIXED)
            iFrmsBA.push_back(iFrmSrc);
    }
    iFrmsBA.insert(iFrmsBA.end(), iFrmsAdj.begin(), iFrmsAdj.end());

    // Step4: copy tag, intrinsic matrix, all BA cameras and all BA points
    const FrameIndex nFrmsDst = FrameIndex(iFrmsBA.size());
    const TrackIndex nTrksDst = iTrkDst;
    const MeasurementIndex nMeasDst = iMeaDst;
    data.Resize(nFrmsDst, nTrksDst, nMeasDst);

    //m_tag.GetSubSequence(iFrmsBA, data.m_tag);
    data.m_fxy = m_K.fxy();
    data.m_G = m_Kr;
    for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst) {
        iFrmSrc = iFrmsBA[iFrmDst];
        data.m_Cs[iFrmDst] = m_Cs[iFrmSrc];
        //data.m_frmStates[iFrmDst] = m_frmStates[iFrmSrc];
    }
    for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst) {
        iTrkSrc = iTrksBA[iTrkDst];
        data.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
        //data.m_trkStates[iTrkDst] = m_trkStates[iTrkSrc];
    }

    // Step5: copy inlier measurements and create correspondence maps
    //iMeasBA.resize(0);
    iMeaDst = 0;
    data.m_mapFrmToMea[0] = 0;
    for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst) {
        iFrmSrc = iFrmsBA[iFrmDst];
        const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc],
                               iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
        for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc) {
            iTrkSrc = m_mapMeaToTrk[iMeaSrc];
            if(iTrkSrc == INVALID_TRACK_INDEX ||
                    (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX
                    || (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
                continue;
            //iMeasBA.push_back(iMeaSrc);
            data.m_xs[iMeaDst].Set(m_xs[iMeaSrc].x(), m_xs[iMeaSrc].y(),
                                   (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH) ? 0.0f :
                                   m_ds[iMeaSrc]);
            data.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
            data.m_mapMeaToFrm[iMeaDst] = iFrmDst;
            data.m_mapMeaToTrk[iMeaDst] = iTrkDst;
            //data.m_meaStates[iMeaDst] = m_meaStates[iMeaSrc];
            ++iMeaDst;
        }
        data.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
    }

#if _DEBUG
    assert(iMeaDst == nMeasDst);
#endif
}

void SequenceDepth::GetBundleAdjustorData(const FrameIndexList &iFrmsAdj,
        const TrackIndexList &iTrksAdj, SequenceBundleAdjustorDataDepth &data,
        FrameIndexList &iFrmsBA) const {
#if _DEBUG
    assert(m_intrinsicType == INTRINSIC_USER_FIXED ||
           m_intrinsicType == INTRINSIC_CONSTANT);
#endif
    // Step1: mark adjusted frames
    FrameStateList frmStates = m_frmStates;
    const FrameIndex nFrmsAdj = FrameIndex(iFrmsAdj.size());
    for(FrameIndex i = 0; i < nFrmsAdj; ++i)
        frmStates[iFrmsAdj[i]] |= FLAG_FRAME_STATE_ADJUSTED;

    // Step2: mark fixed frames, create reverse track index list iTrksSrcToDst and count BA measurements
    TrackIndexList iTrksSrcToDst(GetTracksNumber(), INVALID_TRACK_INDEX);
    FrameIndex iFrmSrc, iFrmDst;
    TrackIndex iTrkSrc, iTrkDst;
    MeasurementIndex iMeaSrc, iMeaDst = 0;
    const TrackIndex nTrksAdj = TrackIndex(iTrksAdj.size());
    for(iTrkDst = 0; iTrkDst < nTrksAdj; ++iTrkDst) {
        iTrkSrc = iTrksAdj[iTrkDst];
        iTrksSrcToDst[iTrkSrc] = iTrkDst;

        const MeasurementIndexList &iMeasSrc = m_mapTrkToMea[iTrkSrc];
        const FrameIndex nCrspsSrc = FrameIndex(iMeasSrc.size());
        for(FrameIndex k = 0; k < nCrspsSrc; ++k) {
            iMeaSrc = iMeasSrc[k];
            if(m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER)
                continue;
            iFrmSrc = m_mapMeaToFrm[iMeaSrc];
            if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_SOLVED))
                continue;
            ++iMeaDst;
            if(!(frmStates[iFrmSrc] & FLAG_FRAME_STATE_ADJUSTED))
                frmStates[iFrmSrc] |= FLAG_FRAME_STATE_FIXED;
        }
    }

    // Step3: collect fixed frames
    iFrmsBA.resize(0);
    const FrameIndex nFrmsSrc = GetFramesNumber();
    for(iFrmSrc = 0; iFrmSrc < nFrmsSrc; ++iFrmSrc) {
        if(frmStates[iFrmSrc] & FLAG_FRAME_STATE_FIXED)
            iFrmsBA.push_back(iFrmSrc);
    }
    iFrmsBA.insert(iFrmsBA.end(), iFrmsAdj.begin(), iFrmsAdj.end());

    // Step4: copy tag, intrinsic matrix, all BA cameras and all BA points
    const FrameIndex nFrmsDst = FrameIndex(iFrmsBA.size());
    const TrackIndex nTrksDst = TrackIndex(iTrksAdj.size());
    const MeasurementIndex nMeasDst = iMeaDst;
    data.Resize(nFrmsDst, nTrksDst, nMeasDst);

    //m_tag.GetSubSequence(iFrmsBA, data.m_tag);
    data.m_fxy = m_K.fxy();
    data.m_G = m_Kr;
    for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst) {
        iFrmSrc = iFrmsBA[iFrmDst];
        data.m_Cs[iFrmDst] = m_Cs[iFrmSrc];
        //data.m_frmStates[iFrmDst] = m_frmStates[iFrmSrc];
    }
    for(iTrkDst = 0; iTrkDst < nTrksDst; ++iTrkDst) {
        iTrkSrc = iTrksAdj[iTrkDst];
        data.m_Xs[iTrkDst] = m_Xs[iTrkSrc];
        //data.m_trkStates[iTrkDst] = m_trkStates[iTrkSrc];
    }

    // Step5: copy inlier measurements and create correspondence maps
    //iMeasBA.resize(0);
    iMeaDst = 0;
    data.m_mapFrmToMea[0] = 0;
    for(iFrmDst = 0; iFrmDst < nFrmsDst; ++iFrmDst) {
        iFrmSrc = iFrmsBA[iFrmDst];
        const MeasurementIndex iMeaSrc1 = m_mapFrmToMea[iFrmSrc],
                               iMeaSrc2 = m_mapFrmToMea[iFrmSrc + 1];
        for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc) {
            iTrkSrc = m_mapMeaToTrk[iMeaSrc];
            if(iTrkSrc == INVALID_TRACK_INDEX ||
                    (iTrkDst = iTrksSrcToDst[iTrkSrc]) == INVALID_TRACK_INDEX
                    || (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER))
                continue;
            //iMeasBA.push_back(iMeaSrc);
            data.m_xs[iMeaDst].Set(m_xs[iMeaSrc].x(), m_xs[iMeaSrc].y(),
                                   (m_meaStates[iMeaSrc] & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH) ? 0.0f :
                                   m_ds[iMeaSrc]);
            data.m_mapTrkToMea[iTrkDst].push_back(iMeaDst);
            data.m_mapMeaToFrm[iMeaDst] = iFrmDst;
            data.m_mapMeaToTrk[iMeaDst] = iTrkDst;
            //data.m_meaStates[iMeaDst] = m_meaStates[iMeaSrc];
            ++iMeaDst;
        }
        data.m_mapFrmToMea[iFrmDst + 1] = iMeaDst;
    }

#if _DEBUG
    assert(iMeaDst == nMeasDst);
#endif
}

void SequenceDepth::SetBunbleAdjustmentResults(const
        SequenceBundleAdjustorDataDepth &data, const FrameIndex &nFrmsFix,
        const FrameIndexList &iFrmsBA,
        const TrackIndexList &iTrksBA) {
#if _DEBUG
    assert(m_intrinsicType == INTRINSIC_USER_FIXED);
#endif
    const FrameIndex nFrmsBA = FrameIndex(iFrmsBA.size());
    for(FrameIndex i = nFrmsFix; i < nFrmsBA; ++i)
        SetCamera(iFrmsBA[i], data.m_Cs[i]);
    SetPoints(iTrksBA, data.GetPoints());
}

static inline void LoadDepthMap(const std::string &fileName,
                                CVD::Image<float> &depthMap) {
    if(IO::ExtractFileExtension(fileName) == "raw") {

        FILE *fp = fopen( fileName.c_str(), "rb");
        int width, height;
        fread(&height, sizeof(int), 1, fp);
        fread(&width, sizeof(int), 1, fp);
        depthMap.resize(CVD::ImageRef(width, height));
        fread(depthMap.data(), sizeof(float), width * height, fp);
        fclose(fp);
    } else
        CVD::img_load(depthMap, fileName);
}

static inline void LoadNormalMap(const std::string &fileName,
                                 CVD::Image<LA::Vector3f> &normalMap) {

    FILE *fp = fopen( fileName.c_str(), "rb");
    int width, height;
    fread(&height, sizeof(int), 1, fp);
    fread(&width, sizeof(int), 1, fp);
    normalMap.resize(CVD::ImageRef(width, height));
    fread(normalMap.data(), sizeof(float) * 3, width * height, fp);
    fclose(fp);
}

void SequenceDepth::LoadDepths() {
    const bool measNormalizedBkp = m_measNormalized;
    DenormalizeMeasurements();
    m_ds.Resize(m_xs.Size());
    m_ds.SetZero();

    int ix0, ix1, iy0, iy1;
    float dx0, dx1, dy0, dy1;
    ENFT_SSE::__m128 d, w;
    CVD::Image<float> depthMap;
    const FrameIndex nFrms = GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        LoadDepthMap(m_tagDep.GetImageFileName(iFrm), depthMap);
        const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm],
                               iMea2 = m_mapFrmToMea[iFrm + 1];
        for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
            //if(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH)
            //  continue;
            const Point2D &x = m_xs[iMea];
            ix0 = int(x.x());
            ix1 = ix0 + 1;
            iy0 = int(x.y());
            iy1 = iy0 + 1;
            dx0 = x.x() - ix0;
            dx1 = 1 - dx0;
            dy0 = x.y() - iy0;
            dy1 = 1 - dy0;
            if((d.m128_f32[0] = depthMap[iy0][ix0]) == 0.0f ||
                    (d.m128_f32[1] = depthMap[iy0][ix1]) == 0.0f
                    || (d.m128_f32[2] = depthMap[iy1][ix0]) == 0.0f ||
                    (d.m128_f32[3] = depthMap[iy1][ix1]) == 0.0f)
                continue;
            w.m128_f32[0] = dx1 * dy1;
            w.m128_f32[1] = dx0 * dy1;
            w.m128_f32[2] = dx1 * dy0;
            w.m128_f32[3] = dx0 * dy0;
            m_ds[iMea] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(w, d));
            MarkMeasurementDepthOutlier(iMea);
        }
        printf("\rLoading measurement depths...%d%%", iFrm * 100 / nFrms);
    }
    printf("\rLoading measurement depths...100%%\n");

    if(measNormalizedBkp)
        NormalizeMeasurements();
}

void SequenceDepth::LoadDepths(const float errTh) {
    const bool measNormalizedBkp = m_measNormalized;
    DenormalizeMeasurements();
    m_ds.Resize(m_xs.Size());
    m_ds.SetZero();

    TrackIndex iTrk;
    int ix0, ix1, iy0, iy1;
    float dx0, dx1, dy0, dy1;
    ENFT_SSE::__m128 d, w;
    CVD::Image<float> depthMap;
    const FrameIndex nFrms = GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        LoadDepthMap(m_tagDep.GetImageFileName(iFrm), depthMap);

        const Camera &C = m_Cs[iFrm];
        if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
            continue;
        const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm],
                               iMea2 = m_mapFrmToMea[iFrm + 1];
        for(MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea) {
            if((m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER) ||
                    (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH)
                    || (iTrk = m_mapMeaToTrk[iMea]) == INVALID_TRACK_INDEX ||
                    !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
                continue;
            const Point2D &x = m_xs[iMea];
            ix0 = int(x.x());
            ix1 = ix0 + 1;
            iy0 = int(x.y());
            iy1 = iy0 + 1;
            dx0 = x.x() - ix0;
            dx1 = 1 - dx0;
            dy0 = x.y() - iy0;
            dy1 = 1 - dy0;
            if((d.m128_f32[0] = depthMap[iy0][ix0]) == 0.0f ||
                    (d.m128_f32[1] = depthMap[iy0][ix1]) == 0.0f
                    || (d.m128_f32[2] = depthMap[iy1][ix0]) == 0.0f ||
                    (d.m128_f32[3] = depthMap[iy1][ix1]) == 0.0f)
                continue;
            w.m128_f32[0] = dx1 * dy1;
            w.m128_f32[1] = dx0 * dy1;
            w.m128_f32[2] = dx1 * dy0;
            w.m128_f32[3] = dx0 * dy0;
            m_ds[iMea] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(w, d));
            if(fabs(m_ds[iMea] - C.ComputeDepth(m_Xs[iTrk])) > errTh)
                m_ds[iMea] = 0.0f;
        }
        printf("\rLoading measurement depths...%d%%", iFrm * 100 / nFrms);
    }
    printf("\rLoading measurement depths...100%%\n");

    if(measNormalizedBkp)
        NormalizeMeasurements();
}

void SequenceDepth::SynthesizeDepths(const float ratio) {
    m_ds.Resize(m_xs.Size());
    m_ds.SetZero();

    FeatureIndex iFtr;
    std::vector<bool> ftrMarks;
    const FrameIndex nFrms = GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm];
        const FeatureIndex nFtrs = FeatureIndex(m_mapFrmToMea[iFrm + 1] - iMeaStart),
                           nDeps = FeatureIndex(nFtrs * ratio + 0.5f);
        ftrMarks.assign(nFtrs, false);
        for(FeatureIndex i = 0; i < nDeps; ++i) {
            do iFtr = Random::GenerateUshort(nFtrs);
            while(ftrMarks[iFtr]);
            ftrMarks[iFtr] = true;
        }
        const Camera &C = m_Cs[iFrm];
        const TrackIndex *iTrks = m_mapMeaToTrk.data() + iMeaStart;
        float *ds = m_ds.Data() + iMeaStart;
        for(iFtr = 0; iFtr < nFtrs; ++iFtr) {
            if(ftrMarks[iFtr])
                ds[iFtr] = C.ComputeDepth(m_Xs[iTrks[iFtr]]);
        }
    }
}

void SequenceDepth::SaveB(FILE *fp) const {
    Sequence::SaveB(fp);
    m_tagDep.SaveB(fp);
    m_tagNormal.SaveB(fp);
    m_ds.SaveB(fp);
}

void SequenceDepth::LoadB(FILE *fp) {
    Sequence::LoadB(fp);
    m_tagDep.SetDirectory(GetDirectory());
    m_tagDep.LoadB(fp);
    m_tagNormal.SetDirectory(GetDirectory());
    m_tagNormal.LoadB(fp);
    m_ds.LoadB(fp);
}

void SequenceDepth::PrintFrameFeature(const FrameIndex &iFrm,
                                      const FeatureIndex &iFtr) const {
    const MeasurementIndex iMea = m_mapFrmToMea[iFrm] + iFtr;
    printf("----------------------------------------------------------------\n");
    printf("Frame %d, Feature %d, Measurement %d", iFrm, iFtr, iMea);
    if(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_ENFT)
        printf(", ENFT");
    else
        printf(", SIFT");
    if(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) {
        if(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER)
            printf(", OUTLIER");
        else
            printf(", INLIER");
        if(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH)
            printf(", OUTLIER_DEPTH");
        else
            printf(", INLIER_DEPTH");
    }
    printf("\n");
    Point2D x;
    if(m_measNormalized)
        m_K.NormalizedPlaneToImage(m_xs[iMea], x);
    else
        x = m_xs[iMea];
    printf("  Feature = (%f, %f)", x.x(), x.y());
    if(m_ds[iMea] > 0.0f)
        printf(", Depth = %f", m_ds[iMea]);
    printf("\n");
    const TrackIndex iTrk = m_mapMeaToTrk[iMea];
    if((m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) &&
            iTrk != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED)) {
        Point2D xp;
        m_Cs[iFrm].ProjectToNormalizedPlane(m_Xs[iTrk], xp);
        if(m_measNormalized)
            m_K.NormalizedPlaneToImage(xp);
        printf("  Error = (%f, %f) - (%f, %f) = %f\n", x.x(), x.y(), xp.x(), xp.y(),
               sqrt(x.SquaredDistance(xp)));
        if(m_ds[iMea] > 0.0f) {
            const float d = m_Cs[iFrm].ComputeDepth(m_Xs[iTrk]);
            printf("          %f - %f = %f\n", m_ds[iMea], d, fabs(m_ds[iMea] - d));
        }
    }
}

static inline uint CountDepths(const AlignedVector<float> &ds) {
    uint cnt = 0;
    const uint N = ds.Size();
    for(uint i = 0; i < N; ++i) {
        if(ds[i] != 0.0f)
            ++cnt;
    }
    return cnt;
}

void SequenceDepth::PrintStates() const {
    if(m_ds.Empty()) {
        Sequence::PrintStates();
        return;
    }
    printf("----------------------------------------------------------------\n");
    printf("Intrinsic Parameter: (%.2f, %.2f, %.2f, %.2f)\n", m_K.fx(), m_K.fy(),
           m_K.cx(), m_K.cy());
    printf("Frames: Total = %d, Keyframe = %d, Solved = %d\n", GetFramesNumber(),
           CountFrames(FLAG_FRAME_STATE_KEY_FRAME), CountFrames(FLAG_FRAME_STATE_SOLVED));
    printf("Tracks: Total = %d, Solved = %d, Inlier = %d\n", GetTracksNumber(),
           CountTracks(FLAG_TRACK_STATE_SOLVED), CountTracks(FLAG_TRACK_STATE_INLIER));
    printf("Measurements: Total = %d, ENFT = %d, Outlier = %d\n",
           GetMeasurementsNumber(), CountMeasurements(FLAG_MEASUREMENT_STATE_ENFT),
           CountMeasurements(FLAG_MEASUREMENT_STATE_OUTLIER));
    printf("Depth: Total = %d, Outlier = %d\n", CountDepths(m_ds),
           CountMeasurements(FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH));

    FrameIndex iFrm, iFrmInlierMin, iFrmInlierRatioMin, iFrmMSEMax,
               iFrmInlierAreaRatioMin;
    FeatureIndex nInliers, nInliersMin;
    float inlierRatio, inlierRatioMin, MSE, MSEMax, SSE, inlierAreaRatio,
          inlierAreaRatioMin;
    FeatureIndexList iFtrs;
    Point2D meanFtrTrked, meanFtrInlier;
    LA::Vector3f covFtrTrked, covFtrInlier;
    iFrmInlierMin = iFrmInlierRatioMin = iFrmMSEMax = iFrmInlierAreaRatioMin =
            INVALID_FRAME_INDEX;
    nInliersMin = INVALID_FEATURE_INDEX;
    inlierRatioMin = inlierAreaRatioMin = FLT_MAX;
    MSEMax = 0;
    const FrameIndex nFrms = GetFramesNumber();
    for(iFrm = 0; iFrm < nFrms; ++iFrm) {
        if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
            continue;
        ComputeFrameInlierRatioAndMSE(iFrm, nInliers, inlierRatio, SSE, MSE);
        if(nInliers < nInliersMin) {
            nInliersMin = nInliers;
            iFrmInlierMin = iFrm;
        }
        if(inlierRatio < inlierRatioMin) {
            inlierRatioMin = inlierRatio;
            iFrmInlierRatioMin = iFrm;
        }
        if(MSE > MSEMax) {
            MSEMax = MSE;
            iFrmMSEMax = iFrm;
        }
        GetFrameTrackedFeatureIndexList(iFrm, iFtrs);
        ComputeFrameFeaturesDistribution(iFrm, iFtrs, meanFtrTrked, covFtrTrked);
        GetFrameInlierFeatureIndexList(iFrm, iFtrs);
        ComputeFrameFeaturesDistribution(iFrm, iFtrs, meanFtrInlier, covFtrInlier);
        inlierAreaRatio = sqrt((covFtrInlier.v0() * covFtrInlier.v2() -
                                covFtrInlier.v1() * covFtrInlier.v1())
                               / (covFtrTrked.v0() * covFtrTrked.v2() - covFtrTrked.v1() * covFtrTrked.v1()));
        //inlierRatioArea = sqrt((covFtrsInlier.v0() * covFtrsInlier.v2() - covFtrsInlier.v1() * covFtrsInlier.v1())
        //  / (covFtrsTrked.v0() * covFtrsTrked.v2() - covFtrsTrked.v1() * covFtrsTrked.v1()));
        if(inlierAreaRatio < inlierAreaRatioMin) {
            inlierAreaRatioMin = inlierAreaRatio;
            iFrmInlierAreaRatioMin = iFrm;
        }
    }
    if(iFrmInlierMin != INVALID_FRAME_INDEX)
        printf("  Inliers           >= %d (Frame %d)\n", nInliersMin, iFrmInlierMin);
    if(iFrmInlierRatioMin != INVALID_FRAME_INDEX)
        printf("  Inlier ratio      >= %f (Frame %d)\n", inlierRatioMin,
               iFrmInlierRatioMin);
    if(iFrmInlierAreaRatioMin != INVALID_FRAME_INDEX)
        printf("  Inlier area ratio >= %f (Frame %d)\n", inlierAreaRatioMin,
               iFrmInlierAreaRatioMin);
    if(iFrmMSEMax != INVALID_FRAME_INDEX)
        printf("  MSE               <= %f (Frame %d)\n", MSEMax, iFrmMSEMax);

    MSEMax = 0;
    TrackIndex iTrk, iTrkMSEMax = INVALID_TRACK_INDEX;
    const TrackIndex nTrks = GetTracksNumber();
    for(iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(!(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
            continue;
        ComputeTrackMSE(iTrk, m_Xs[iTrk], MSE);
        if(MSE > MSEMax) {
            MSEMax = MSE;
            iTrkMSEMax = iTrk;
        }
    }
    if(iTrkMSEMax != INVALID_TRACK_INDEX)
        printf("  Track MSE         <= %f (Track %d)\n", MSEMax, iTrkMSEMax);

    float errDep, errDepMax = 0;
    MeasurementIndex iMea, iMeaMax = INVALID_MEASUREMENT_INDEX;
    const MeasurementIndex nMeas = GetMeasurementsNumber();
    for(iMea = 0; iMea < nMeas; ++iMea) {
        if(m_ds[iMea] == 0.0f || (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER) ||
                (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER_DEPTH))
            continue;
        iFrm = m_mapMeaToFrm[iMea];
        iTrk = m_mapMeaToTrk[iMea];
        if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) ||
                !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
            continue;
        if((errDep = fabs(m_ds[iMea] - m_Cs[iFrm].ComputeDepth(m_Xs[iTrk]))) >
                errDepMax) {
            errDepMax = errDep;
            iMeaMax = iMea;
        }
    }
    if(iMeaMax != INVALID_MEASUREMENT_INDEX)
        printf("  Depth error       <= %f (Frame %d, Track %d, Measurement %d)\n",
               errDepMax, m_mapMeaToFrm[iMeaMax], m_mapMeaToTrk[iMeaMax], iMeaMax);
}

bool SequenceDepth::SavePly(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if(fp == nullptr)
        return false;
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");

    ushort x, y;
    FrameIndex iFrm;
    CVD::Image<float> depthMap;
    CVD::Image<LA::Vector3f> normapMap;

    uint cnt = 0;
    const bool normal = m_tagNormal.GetSequenceName() != "";
    const FrameIndex nFrms = GetFramesNumber();
    for(iFrm = 0; iFrm < nFrms; ++iFrm) {
        if(!IsFrameKeyFrame(iFrm))
            continue;
        LoadDepthMap(GetDepthFileName(iFrm), depthMap);
        if(normal)
            LoadNormalMap(GetNormalFileName(iFrm), normapMap);
        const ushort width = ushort(depthMap.size().x),
                     height = ushort(depthMap.size().y);
        for(y = 0; y < height; ++y)
            for(x = 0; x < width; ++x) {
                if(depthMap[y][x] != 0.0f || normal && !std::isfinite(normapMap[y][x].v0()))
                    ++cnt;
            }
    }
    fprintf(fp, "element vertex %d\n", cnt);

    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    if(normal) {
        fprintf(fp, "property float nx\n");
        fprintf(fp, "property float ny\n");
        fprintf(fp, "property float nz\n");
    }
    fprintf(fp, "property uchar diffuse_red\n");
    fprintf(fp, "property uchar diffuse_green\n");
    fprintf(fp, "property uchar diffuse_blue\n");
    fprintf(fp, "end_header\n");

    Point3D X, N;
    CVD::Image<CVD::Rgb<ubyte> > img;
    for(iFrm = 0; iFrm < nFrms; ++iFrm) {
        printf("\rSaving \'%s\'...%d%%", (GetDirectory() + fileName).c_str(),
               iFrm * 100 / nFrms);
        if(!IsFrameKeyFrame(iFrm))
            continue;
        CVD::img_load(img, GetImageFileName(iFrm));
        LoadDepthMap(GetDepthFileName(iFrm), depthMap);
        if(normal)
            LoadNormalMap(GetNormalFileName(iFrm), normapMap);
        const Camera &C = m_Cs[iFrm];
        const ushort width = ushort(depthMap.size().x),
                     height = ushort(depthMap.size().y);
        for(y = 0; y < height; ++y)
            for(x = 0; x < width; ++x) {
                X.Z() = depthMap[y][x];
                if(normal)
                    N.Set(normapMap[y][x]);
                if(X.Z() == 0.0f || normal && !std::isfinite(N.X()))
                    continue;
                X.X() = (x - m_K.cx()) * m_K.one_over_fx() * X.Z();
                X.Y() = (y - m_K.cy()) * m_K.one_over_fy() * X.Z();
                C.ApplyInversely(X);
                const CVD::Rgb<ubyte> &clr = img[y][x];
                if(normal) {
                    C.ApplyRotationInversely(N);
                    fprintf(fp, "%f %f %f %f %f %f %d %d %d\n", X.X(), X.Y(), X.Z(), N.X(), N.Y(),
                            N.Z(), clr.red, clr.green, clr.blue);
                } else
                    fprintf(fp, "%f %f %f %d %d %d\n", X.X(), X.Y(), X.Z(), clr.red, clr.green,
                            clr.blue);
            }
    }
    printf("\rSaving \'%s\'...%d%%\n", (GetDirectory() + fileName).c_str(), 100);

    fclose(fp);
    return true;
}

bool SequenceDepth::SaveObj(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if (fp == nullptr)
        return false;
    ushort x, y;
    Point3D X;
    CVD::Image<float> depthMap;
    CVD::Image<CVD::Rgb<ubyte> > img;
    const FrameIndex nFrms = GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        printf("\rSaving \'%s\'...%d%%", (GetDirectory() + fileName).c_str(),
               iFrm * 100 / nFrms);
        if(!IsFrameKeyFrame(iFrm))
            continue;
        CVD::img_load(img, GetImageFileName(iFrm));
        LoadDepthMap(GetDepthFileName(iFrm), depthMap);
        const Camera &C = m_Cs[iFrm];
        const ushort width = ushort(depthMap.size().x),
                     height = ushort(depthMap.size().y);
        for(y = 0; y < height; ++y)
            for(x = 0; x < width; ++x) {
                if((X.Z() = depthMap[y][x]) == 0.0f)
                    continue;
                X.X() = (x - m_K.cx()) * m_K.one_over_fx() * X.Z();
                X.Y() = (y - m_K.cy()) * m_K.one_over_fy() * X.Z();
                C.ApplyInversely(X);
                const CVD::Rgb<ubyte> &clr = img[y][x];
                fprintf(fp, "v %f %f %f %f %f %f\n", X.X(), X.Y(), X.Z(), clr.red / 255.0f,
                        clr.green / 255.0f, clr.blue / 255.0f);
            }
    }
    printf("\rSaving \'%s\'...%d%%\n", (GetDirectory() + fileName).c_str(), 100);

    fclose(fp);
    return true;
}