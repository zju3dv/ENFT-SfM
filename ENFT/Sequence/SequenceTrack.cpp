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

void Sequence::InitializePoints() {
    const TrackIndex nTrks = GetTracksNumber();
    m_Xs.Resize(nTrks);
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        m_trkStates[iTrk] &= ~FLAG_TRACK_STATE_INITIAL;
        m_trkStates[iTrk] &= ~FLAG_TRACK_STATE_SOLVED;
        m_trkStates[iTrk] &= ~FLAG_TRACK_STATE_INLIER;
    }
}

void Sequence::GetTrackIndexList(const TrackState trkState, TrackIndexList &iTrks) const {
    iTrks.resize(0);
    const TrackIndex nTrks = GetTracksNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(m_trkStates[iTrk] & trkState)
            iTrks.push_back(iTrk);
    }
}

void Sequence::SetPoints(const TrackIndexList &iTrks, const AlignedVector<Point3D> &Xs) {
    const TrackIndex nTrksNew = TrackIndex(iTrks.size());
    TrackIndex iTrk;
    if(GetTracksNumber() != GetPointsNumber()) {
        if(nTrksNew == 0)
            return;
        TrackIndex iTrkMax = iTrks[0];
        for(TrackIndex i = 1; i < nTrksNew; ++i) {
            iTrk = iTrks[i];
            iTrkMax = std::max(iTrkMax, iTrk);
        }
        const TrackIndex nPtsTotal = iTrkMax + 1;
        if(nPtsTotal > GetPointsNumber()) {
            const TrackIndex nPtsExpect = TrackIndex(float(GetPointsNumber()) / GetFramesNumber() * GetFramesNumberTotal() + 0.5f);
            const TrackIndex capacity = std::max(nPtsExpect, nPtsTotal);
            m_Xs.EnlargeCapacity(capacity);
            m_Xs.Resize(nPtsTotal);
        }
    }
    for(TrackIndex i = 0; i < nTrksNew; ++i) {
        iTrk = iTrks[i];
        m_Xs[iTrk] = Xs[i];
    }
}

void Sequence::SetTrackStates(const TrackState &trkState) {
    const TrackIndex nTrks = GetTracksNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
        m_trkStates[iTrk] = trkState;
}

void Sequence::SetTrackStates(const TrackIndexList &iTrks, const TrackState &trkState) {
    const TrackIndex nTrks = TrackIndex(iTrks.size());
    for(TrackIndex i = 0; i < nTrks; ++i)
        m_trkStates[iTrks[i]] = trkState;
}

//void Sequence::SetTrackColor(const TrackIndex &iTrk, const CVD::Rgb<ubyte> &clr)
//{
//  if(iTrk >= GetTrackColorsNumber())
//      m_trkClrs.resize(iTrk + 1);
//  m_trkClrs[iTrk] = clr;
//}

void Sequence::MarkTrackOutlier(const TrackIndex &iTrk) {
    m_trkStates[iTrk] &= ~FLAG_TRACK_STATE_INLIER;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    //if(nCrsps == 0)
    //  return;
    //MarkMeasurementInlier(iMeas[0]);
    //for(FrameIndex i = 1; i < nCrsps; ++i)
    //  MarkMeasurementOutlier(iMeas[i]);
    for(FrameIndex i = 0; i < nCrsps; ++i)
        MarkMeasurementInlier(iMeas[i]);
}

void Sequence::UnmarkTracksMerged() {
    const TrackIndex nTrks = GetTracksNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
        m_trkStates[iTrk] &= ~FLAG_TRACK_STATE_MERGED;
}

FrameIndex Sequence::GetPoint3DEstimatorData(const TrackIndex &iTrk, Point3DEstimatorData &data, const bool rectData) const {
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    MeasurementIndex iMea;
    FrameIndex iFrm;
    FrameIndex i, j;
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    data.Resize(nCrsps);
    for(i = j = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        iFrm = m_mapMeaToFrm[iMea];
        if(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED)
            data.Set(j++, m_Cs[iFrm], m_xs[iMea]);
    }
    data.Resize(j);
    if(!m_measNormalized)
        m_K.ImageToNormalizedPlaneN(data.xs());
    data.SetFocal(m_K.fxy());
    if(!rectData)
        return j;
    else if(m_intrinsicType == INTRINSIC_CONSTANT)
        RectifyMeasurements(m_Kr, data.xs());
    else if(m_intrinsicType == INTRINSIC_VARIABLE) {
        data.ValidateWeights();
        for(i = j = 0; i < nCrsps; ++i) {
            iFrm = m_mapMeaToFrm[iMeas[i]];
            if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
                continue;
            m_Krs[iFrm].Rectify(data.x(j));
            data.SetWeight(j++, m_Krs[iFrm].f());
        }
    }
    return j;
}

FrameIndex Sequence::GetPoint3DEstimatorData(const TrackIndex &iTrk, Point3DEstimatorData &data, MeasurementIndexList &iMeas, const bool rectData) const {
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
        data.Set(j, m_Cs[iFrm], m_xs[iMea]);
        iMeas[j++] = iMea;
    }
    const FrameIndex N = j;
    data.Resize(N);
    iMeas.resize(N);
    if(!m_measNormalized)
        m_K.ImageToNormalizedPlaneN(data.xs());
    data.SetFocal(m_K.fxy());
    if(!rectData)
        return N;
    else if(m_intrinsicType == INTRINSIC_CONSTANT)
        RectifyMeasurements(m_Kr, data.xs());
    else if(m_intrinsicType == INTRINSIC_VARIABLE) {
        data.ValidateWeights();
        for(i = 0; i < N; ++i) {
            iFrm = m_mapMeaToFrm[iMeas[i]];
            m_Krs[iFrm].Rectify(data.x(i));
            data.SetWeight(i, m_Krs[iFrm].f());
        }
    }
    return N;
}

FrameIndex Sequence::GetPoint3DEstimatorDataInlier(const TrackIndex &iTrk, Point3DEstimatorData &data, const bool rectData) const {
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    MeasurementIndex iMea;
    FrameIndex iFrm;
    FrameIndex i, j;
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    data.Resize(nCrsps);
    for(i = 0, j = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        iFrm = m_mapMeaToFrm[iMea];
        if((m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) && !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
            data.Set(j++, m_Cs[iFrm], m_xs[iMea]);
    }
    data.Resize(j);
    if(!m_measNormalized)
        m_K.ImageToNormalizedPlaneN(data.xs());
    data.SetFocal(m_K.fxy());
    if(!rectData)
        return j;
    else if(m_intrinsicType == INTRINSIC_CONSTANT)
        RectifyMeasurements(m_Kr, data.xs());
    else if(m_intrinsicType == INTRINSIC_VARIABLE) {
        data.ValidateWeights();
        for(i = j = 0; i < nCrsps; ++i) {
            iMea = iMeas[i];
            iFrm = m_mapMeaToFrm[iMea];
            if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) || (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
                continue;
            m_Krs[iFrm].Rectify(data.x(j));
            data.SetWeight(j++, m_Krs[iFrm].f());
        }
    }
    return j;
}

TrackIndex Sequence::CountTracks(const TrackState trkState) const {
    TrackIndex cnt = 0;
    const TrackIndex nTrks = GetTracksNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        if(m_trkStates[iTrk] & trkState)
            ++cnt;
    }
    return cnt;
}

TrackIndex Sequence::CountTracks(const TrackState trkState1, const TrackState trkState2) const {
    TrackIndex cnt = 0;
    const TrackIndex nTrks = GetTracksNumber();
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        if((m_trkStates[iTrk] & trkState1) && (m_trkStates[iTrk] & trkState2))
            ++cnt;
    }
    return cnt;
}

FrameIndex Sequence::CountTrackKeyFrameMeasurements(const TrackIndex &iTrk) const {
    FrameIndex cnt = 0;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for(FrameIndex i = 0; i < nCrsps; ++i) {
        if(m_frmStates[m_mapMeaToFrm[iMeas[i]]] & FLAG_FRAME_STATE_KEY_FRAME)
            ++cnt;
    }
    return cnt;
}

FrameIndex Sequence::CountTrackSolvedFrameMeasurements(const TrackIndex &iTrk) const {
    FrameIndex cnt = 0;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for(FrameIndex i = 0; i < nCrsps; ++i) {
        if(m_frmStates[m_mapMeaToFrm[iMeas[i]]] & FLAG_FRAME_STATE_SOLVED)
            ++cnt;
    }
    return cnt;
}

FrameIndex Sequence::CountTrackSiftMeasurements(const TrackIndex &iTrk) const {
    FrameIndex cnt = 0;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for(FrameIndex i = 0; i < nCrsps; ++i) {
        if(!(m_meaStates[iMeas[i]] & FLAG_MEASUREMENT_STATE_ENFT))
            ++cnt;
    }
    return cnt;
}

FrameIndex Sequence::CountTrackInlierMeasurements(const TrackIndex &iTrk) const {
    FrameIndex cnt = 0;
    MeasurementIndex iMea;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for(FrameIndex i = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        if((m_frmStates[m_mapMeaToFrm[iMea]] & FLAG_FRAME_STATE_SOLVED) && !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
            ++cnt;
    }
    return cnt;
}

FrameIndex Sequence::CountTrackMarkedFrameMeasurements(const TrackIndex &iTrk, const std::vector<bool> &frmMarks) const {
    FrameIndex cnt = 0;
    MeasurementIndex iMea;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for(FrameIndex i = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        if(frmMarks[m_mapMeaToFrm[iMea]])
            ++cnt;
    }
    return cnt;
}

FrameIndex Sequence::CountTrackMarkedFrameInlierMeasurements(const TrackIndex &iTrk, const std::vector<bool> &frmMarks) const {
    FrameIndex cnt = 0;
    MeasurementIndex iMea;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for(FrameIndex i = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        if(frmMarks[m_mapMeaToFrm[iMea]] && !(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
            ++cnt;
    }
    return cnt;
}

void Sequence::CountTrackSolvedFrameInlierMeasurements(const TrackIndex &iTrk, FrameIndex &cntSolved, FrameIndex &cntInlier) const {
    MeasurementIndex iMea;
    cntSolved = cntInlier = 0;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for(FrameIndex i = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        if(m_frmStates[m_mapMeaToFrm[iMea]] & FLAG_FRAME_STATE_SOLVED) {
            ++cntSolved;
            if(!(m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
                ++cntInlier;
        }
    }
}

MeasurementIndex Sequence::SearchTrackForFrameMeasurementIndex(const TrackIndex &iTrk, const FrameIndex &iFrm) const {
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
    if(iMeas.empty() || iMeas.front() >= iMea2 || iMeas.back() < iMea1)
        return INVALID_MEASUREMENT_INDEX;
    const MeasurementIndex iMea = *std::lower_bound(iMeas.begin(), iMeas.end(), iMea1);
    if(iMea >= iMea2)
        return INVALID_MEASUREMENT_INDEX;
    else
        return iMea;
}

FeatureIndex Sequence::SearchTrackForFrameFeatureIndex(const TrackIndex &iTrk, const FrameIndex &iFrm) const {
    const MeasurementIndex iMea = SearchTrackForFrameMeasurementIndex(iTrk, iFrm);
    if(iMea == INVALID_MEASUREMENT_INDEX)
        return INVALID_FEATURE_INDEX;
    else
        return FeatureIndex(iMea - m_mapFrmToMea[iFrm]);
}

bool Sequence::AreTracksOverlappingInFrames(const TrackIndex &iTrk1, const TrackIndex &iTrk2, std::vector<bool> &marks) const {
#if _DEBUG
    assert(iTrk1 != iTrk2);
#endif
    //AssertPointMeasurementSorted(iTrk1);
    //AssertPointMeasurementSorted(iTrk2);
    const MeasurementIndexList &iMeas1 = m_mapTrkToMea[iTrk1], &iMeas2 = m_mapTrkToMea[iTrk2];
    const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
    if(nCrsps1 == 0 || nCrsps2 == 0)
        return false;
    const FrameIndex iFrm1Start = m_mapMeaToFrm[iMeas1.front()], iFrm1End = m_mapMeaToFrm[iMeas1.back()];
    const FrameIndex iFrm2Start = m_mapMeaToFrm[iMeas2.front()], iFrm2End = m_mapMeaToFrm[iMeas2.back()];
    if(iFrm1Start > iFrm2End || iFrm1End < iFrm2Start)
        return false;
    const FrameIndex N = iFrm1End - iFrm1Start + 1;
    marks.assign(N, false);
    for(FrameIndex i = 0; i < nCrsps1; ++i)
        marks[m_mapMeaToFrm[iMeas1[i]] - iFrm1Start] = true;
    FrameIndex iFrm;
    for(FrameIndex i = 0; i < nCrsps2; ++i) {
        if((iFrm = m_mapMeaToFrm[iMeas2[i]]) >= iFrm1Start && iFrm <= iFrm1End && marks[iFrm - iFrm1Start])
            return true;
    }
    return false;
}

//bool Sequence::AreTracksOverlappingInFrames(const TrackIndex &iTrk1, const TrackIndex &iTrk2, MeasurementIndexList &iMeas1Overlapping,
//                                          MeasurementIndexList &iMeas2Overlapping) const
//{
//#if _DEBUG
//  assert(iTrk1 != iTrk2);
//  CheckTrackSorted(iTrk1);
//  CheckTrackSorted(iTrk2);
//#endif
//  const MeasurementIndexList &iMeas1 = m_mapTrkToMea[iTrk1], &iMeas2 = m_mapTrkToMea[iTrk2];
//  const FrameIndex nCrsps1 = FrameIndex(iMeas1.size()), nCrsps2 = FrameIndex(iMeas2.size());
//  if(nCrsps1 == 0 || nCrsps2 == 0)
//      return false;
//  const FrameIndex iFrm1Start = m_mapMeaToFrm[iMeas1.front()], iFrm1End = m_mapMeaToFrm[iMeas1.back()];
//  const FrameIndex iFrm2Start = m_mapMeaToFrm[iMeas2.front()], iFrm2End = m_mapMeaToFrm[iMeas2.back()];
//  if(iFrm1Start > iFrm2End || iFrm1End < iFrm2Start)
//      return false;
//  const FrameIndex N = iFrm1End - iFrm1Start + 1;
//  iMeas1Overlapping.assign(N, INVALID_MEASUREMENT_INDEX);
//  for(FrameIndex i = 0; i < nCrsps1; ++i)
//      iMeas1Overlapping[m_mapMeaToFrm[iMeas1[i]] - iFrm1Start] = iMeas1[i];
//  MeasurementIndex iMea1, iMea2;
//  FrameIndex iFrm2;
//  iMeas2Overlapping.resize(0);
//  ushort j = 0;
//  for(FrameIndex i = 0; i < nCrsps2; ++i)
//  {
//      iMea2 = iMeas2[i];
//      iFrm2 = m_mapMeaToFrm[iMea2];
//      if(iFrm2 >= iFrm1Start && iFrm2 <= iFrm1End && (iMea1 = iMeas1Overlapping[iFrm2 - iFrm1Start]) != INVALID_MEASUREMENT_INDEX)
//      {
//          iMeas1Overlapping[j++] = iMea1;
//          iMeas2Overlapping.push_back(iMea2);
//      }
//  }
//  iMeas1Overlapping.resize(j);
//  return j != 0;
//  return false;
//}

FrameIndex Sequence::CountTracksOverlappingFrames(const TrackIndex &iTrk1, const TrackIndex &iTrk2) const {
    FrameIndex cnt = 0;
    const MeasurementIndexList &iMeas1 = m_mapTrkToMea[iTrk1];
    const FrameIndex nCrsps1 = FrameIndex(iMeas1.size());
    for(FeatureIndex i = 0; i < nCrsps1; ++i) {
        const FrameIndex iFrm1 = m_mapMeaToFrm[iMeas1[i]];
        if(SearchTrackForFrameMeasurementIndex(iTrk2, iFrm1) != INVALID_MEASUREMENT_INDEX)
            ++cnt;
    }
    return cnt;
}

#if DESCRIPTOR_TRACK == 0
void Sequence::PushBackTrack(const FrameIndex &iFrm1, const FeatureIndex &iFtr1, const FrameIndex &iFrm2, const FeatureIndex &iFtr2) {
    const TrackIndex iTrk = GetTracksNumber();
    const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm1] + iFtr1, iMea2 = m_mapFrmToMea[iFrm2] + iFtr2;
    m_mapTrkToMea.push_back(MeasurementIndexList(2));
    //m_mapTrkToPlane.push_back(INVALID_PLANE_INDEX);
    if(iMea1 < iMea2) {
        m_mapTrkToMea[iTrk][0] = iMea1;
        m_mapTrkToMea[iTrk][1] = iMea2;
    } else {
        m_mapTrkToMea[iTrk][0] = iMea2;
        m_mapTrkToMea[iTrk][1] = iMea1;
    }
    m_mapMeaToTrk[iMea1] = m_mapMeaToTrk[iMea2] = iTrk;
    m_trkStates.push_back(FLAG_TRACK_STATE_DEFAULT);
}
#else
void Sequence::PushBackTrack(const FrameIndex &iFrm1, const FeatureIndex &iFtr1, const FrameIndex &iFrm2, const FeatureIndex &iFtr2, const Descriptor &desc1,
                             const Descriptor &desc2) {
    const TrackIndex iTrk = GetTracksNumber();
    const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm1] + iFtr1, iMea2 = m_mapFrmToMea[iFrm2] + iFtr2;
    m_mapTrkToMea.push_back(MeasurementIndexList(2));
    //m_mapTrkToPlane.push_back(INVALID_PLANE_INDEX);
    if(iMea1 < iMea2) {
        m_mapTrkToMea[iTrk][0] = iMea1;
        m_mapTrkToMea[iTrk][1] = iMea2;
    } else {
        m_mapTrkToMea[iTrk][0] = iMea2;
        m_mapTrkToMea[iTrk][1] = iMea1;
    }
    m_mapMeaToTrk[iMea1] = m_mapMeaToTrk[iMea2] = iTrk;
    m_trkStates.push_back(FLAG_TRACK_STATE_DEFAULT);
#if _DEBUG
    assert(m_descs.Size() == iTrk);
#endif
    m_descs.PushBack(desc1);
    Descriptor::ApB(m_descs[iTrk], desc2, m_descs[iTrk]);
}
#endif

void Sequence::PushBackTrack(const FrameFeatureIndexList &iFrmFtrs) {
    const TrackIndex iTrk = GetTracksNumber();
    const FrameIndex nCrsps = FrameIndex(iFrmFtrs.size());
    m_mapTrkToMea.push_back(MeasurementIndexList(nCrsps));
    //m_mapTrkToPlane.push_back(INVALID_PLANE_INDEX);
    MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    MeasurementIndex iMea;
    for(FrameIndex i = 0; i < nCrsps; ++i) {
        iMea = m_mapFrmToMea[iFrmFtrs[i].GetFrameIndex()] + iFrmFtrs[i].GetFeatureIndex();
        iMeas[i] = iMea;
        m_mapMeaToTrk[iMea] = iTrk;
    }
    m_trkStates.push_back(FLAG_TRACK_STATE_DEFAULT);
}

void Sequence::BreakTrack(const TrackIndex &iTrk) {
    MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for(FrameIndex i = 0; i < nCrsps; ++i)
        m_mapMeaToTrk[iMeas[i]] = INVALID_TRACK_INDEX;
    iMeas.clear();
    //m_trkDescCnts[iTrk] = 0;
}

void Sequence::RemoveBrokenTracks() {
    TrackIndexList iTrksOriToNew;
    RemoveBrokenTracks(iTrksOriToNew);
}

void Sequence::RemoveBrokenTracks(TrackIndexList &iTrksOriToNew) {
    const TrackIndex nTrksOri = GetTracksNumber();
    iTrksOriToNew.assign(nTrksOri, INVALID_TRACK_INDEX);
    TrackIndex iTrkOri, iTrkNew;
    for(iTrkOri = iTrkNew = 0; iTrkOri < nTrksOri; ++iTrkOri) {
        if(GetTrackLength(iTrkOri) == 0)
            continue;
        m_mapTrkToMea[iTrkNew] = m_mapTrkToMea[iTrkOri];
        //m_mapTrkToPlane[iTrkNew] = m_mapTrkToPlane[iTrkOri];
        m_trkStates[iTrkNew] = m_trkStates[iTrkOri];
        iTrksOriToNew[iTrkOri] = iTrkNew;
        ++iTrkNew;
    }
    const TrackIndex nTrksNew = iTrkNew;
    if(nTrksNew == nTrksOri)
        return;
    m_mapTrkToMea.resize(nTrksNew);
    //m_mapTrkToPlane.resize(nTrksNew);
    m_trkStates.resize(nTrksNew);

    const TrackIndex nPtsOri = GetPointsNumber();
    if(nPtsOri != 0) {
        for(iTrkOri = nPtsOri - 1; iTrkOri != INVALID_TRACK_INDEX && iTrksOriToNew[iTrkOri] == INVALID_TRACK_INDEX; --iTrkOri);
        const TrackIndex nPtsNew = iTrkOri == INVALID_TRACK_INDEX ? 0 : iTrksOriToNew[iTrkOri] + 1;
        AlignedVector<Point3D> XsNew(nPtsNew);
        for(iTrkOri = 0; iTrkOri < nPtsOri; ++iTrkOri) {
            if((iTrkNew = iTrksOriToNew[iTrkOri]) != INVALID_TRACK_INDEX)
                XsNew[iTrkNew] = m_Xs[iTrkOri];
        }
        m_Xs.Swap(XsNew);
        XsNew.Clear();
    }
#if DESCRIPTOR_TRACK
    const TrackIndex nDescsOri = TrackIndex(m_descs.Size());
    if(nDescsOri != 0) {
        for(iTrkOri = nDescsOri - 1; iTrkOri != INVALID_TRACK_INDEX && iTrksOriToNew[iTrkOri] == INVALID_TRACK_INDEX; --iTrkOri);
        const TrackIndex nDescsNew = iTrkOri == INVALID_TRACK_INDEX ? 0 : iTrksOriToNew[iTrkOri] + 1;
        AlignedVector<Descriptor> descsNew(nDescsNew);
        for(iTrkOri = 0; iTrkOri < nDescsOri; ++iTrkOri) {
            if((iTrkNew = iTrksOriToNew[iTrkOri]) != INVALID_TRACK_INDEX)
                descsNew[iTrkNew] = m_descs[iTrkOri];
        }
        m_descs.Swap(descsNew);
        descsNew.Clear();
    }
#endif
    const TrackIndex nTrkClrsOri = GetTrackColorsNumber();
    if(nTrkClrsOri != 0) {
        for(iTrkOri = nTrkClrsOri - 1; iTrkOri != INVALID_TRACK_INDEX && iTrksOriToNew[iTrkOri] == INVALID_TRACK_INDEX; --iTrkOri);
        const TrackIndex nTrkClrsNew = iTrkOri == INVALID_TRACK_INDEX ? 0 : iTrksOriToNew[iTrkOri] + 1;
        for(iTrkOri = 0; iTrkOri < nTrkClrsOri; ++iTrkOri) {
            if((iTrkNew = iTrksOriToNew[iTrkOri]) != INVALID_TRACK_INDEX)
                m_trkClrs[iTrkNew] = m_trkClrs[iTrkOri];
        }
        m_trkClrs.resize(nTrkClrsNew);
    }

    const MeasurementIndex nMeas = GetMeasurementsNumber();
    for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea) {
        if((iTrkOri = m_mapMeaToTrk[iMea]) == INVALID_TRACK_INDEX)
            continue;
        iTrkNew = iTrksOriToNew[iTrkOri];
#if _DEBUG
        assert(iTrkNew != INVALID_TRACK_INDEX);
#endif
        m_mapMeaToTrk[iMea] = iTrkNew;
    }
}

#if DESCRIPTOR_TRACK == 0
void Sequence::ComputeTrackDescriptor(const TrackIndex &iTrk, Descriptor &desc) const {
    desc.SetZero();
    bool scc = false;
    MeasurementIndex iMea;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());

    for (FrameIndex i = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        if (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_ENFT)
            continue;
        scc = true;
        Descriptor::ApB(m_descs[iMea], desc, desc);
    }
    if (!scc) {
        for (FrameIndex i = 0; i < nCrsps; ++i)
            Descriptor::ApB(m_descs[iMeas[i]], desc, desc);
    }
    desc.Normalize();
}
#endif

bool Sequence::ComputeTrackSSE(const TrackIndex &iTrk, const Point3D &X, float &SSE, FrameIndex &nCrspsInlier) const {
    FrameIndex iFrm;
    MeasurementIndex iMea;
    Point2D xr, xp, e;

    //const Point3D &X = m_Xs[iTrk];
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    SSE = 0;
    nCrspsInlier = 0;
    for(FrameIndex i = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        iFrm = m_mapMeaToFrm[iMea];
        if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) || (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
            continue;
        if(m_measNormalized)
            xr = m_xs[iMea];
        else
            m_K.ImageToNormalizedPlane(m_xs[iMea], xr);
        switch(m_intrinsicType) {
            case INTRINSIC_USER_FIXED:
                if(!m_Cs[iFrm].ComputeProjectionError_CheckCheirality(X, xr, e))
                    return false;
                break;
            case INTRINSIC_CONSTANT:
                m_Kr.Rectify(xr);
                if(!m_Cs[iFrm].ComputeProjectionError_CheckCheirality(X, xr, e))
                    return false;
                break;
            case INTRINSIC_VARIABLE:
                m_Krs[iFrm].Rectify(xr);
                if(!m_Cs[iFrm].ComputeProjectionError_CheckCheirality(X, xr, e))
                    return false;
                e *= m_Krs[iFrm].f();
                break;
        }
        SSE += e.SquaredLength();
        ++nCrspsInlier;
    }
    if(m_intrinsicType == INTRINSIC_CONSTANT)
        SSE *= m_Kr.f() * m_Kr.f();
    return true;
}

bool Sequence::ComputeTrackMSE(const TrackIndex &iTrk, const Point3D &X, float &MSE) const {
    float SSE;
    FrameIndex nCrspsInlier;
    if(!ComputeTrackSSE(iTrk, X, SSE, nCrspsInlier))
        return false;
    MSE = SSE / nCrspsInlier * m_K.fxy();
    return true;
}

void Sequence::ComputePointRayDirections(const TrackIndex &iTrk, AlignedVector<Point3D> &rayDirs, const bool rectData) const {
//#if _DEBUG
//  assert(m_measNormalized);
//#endif
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    rayDirs.Resize(nCrsps);

    Point2D xr;
    FrameIndex iFrm, i, j;
    MeasurementIndex iMea;
    for(i = j = 0; i < nCrsps; ++i) {
        iMea = iMeas[i];
        iFrm = m_mapMeaToFrm[iMea];
        if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) || (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
            continue;
        if(!rectData || m_intrinsicType == INTRINSIC_USER_FIXED)
            xr = m_xs[iMea];
        else if(m_intrinsicType == INTRINSIC_CONSTANT)
            m_Kr.Rectify(m_xs[iMea], xr);
        else if(m_intrinsicType == INTRINSIC_VARIABLE)
            m_Krs[iFrm].Rectify(m_xs[iMea], xr);
        m_Cs[iFrm].ComputeRayDirection(xr, rayDirs[j++]);
    }
    rayDirs.Resize(j);
}

float Sequence::ComputePointMinimalRayAngleDot(const TrackIndex &iTrk, AlignedVector<Point3D> &rayDirs, const bool rectData) const {
    ComputePointRayDirections(iTrk, rayDirs, rectData);
    const FrameIndex N = FrameIndex(rayDirs.Size());
    float dot, dotMin = FLT_MAX;
    for(FrameIndex i1 = 0; i1 < N; ++i1)
        for(FrameIndex i2 = i1 + 1; i2 < N; ++i2) {
            if((dot = rayDirs[i1].Dot(rayDirs[i2])) < dotMin)
                dotMin = dot;
        }
    return dotMin;
}

float Sequence::ComputePointMinimalRayAngleDot(const TrackIndex &iTrk, AlignedVector<Point3D> &rayDirs, FrameIndex &iFrm1Min, FrameIndex &iFrm2Min,
        const bool rectData) const {
    ComputePointRayDirections(iTrk, rayDirs, rectData);

    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];

    FrameIndex iFrm1, iFrm2;
    MeasurementIndex iMea1, iMea2;
    float dot, dotMin = FLT_MAX;
    iFrm1Min = iFrm2Min = INVALID_FRAME_INDEX;
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for(FrameIndex i1 = 0, j1 = 0; i1 < nCrsps; ++i1) {
        iMea1 = iMeas[i1];
        iFrm1 = m_mapMeaToFrm[iMea1];
        if(!(m_frmStates[iFrm1] & FLAG_FRAME_STATE_SOLVED) || (m_meaStates[iMea1] & FLAG_MEASUREMENT_STATE_OUTLIER))
            continue;
        const Point3D &rayDir1 = rayDirs[j1];
        for(FrameIndex i2 = i1 + 1, j2 = j1 + 1; i2 < nCrsps; ++i2) {
            iMea2 = iMeas[i2];
            iFrm2 = m_mapMeaToFrm[iMea2];
            if(!(m_frmStates[iFrm2] & FLAG_FRAME_STATE_SOLVED) || (m_meaStates[iMea2] & FLAG_MEASUREMENT_STATE_OUTLIER))
                continue;
            const Point3D &rayDir2 = rayDirs[j2];
            if((dot = rayDir1.Dot(rayDir2)) < dotMin) {
                dotMin = dot;
                iFrm1Min = iFrm1;
                iFrm2Min = iFrm2;
            }
            ++j2;
        }
        ++j1;
    }
    return dotMin;
}

float Sequence::ComputePointMaximalRayAngle(const TrackIndex &iTrk, AlignedVector<Point3D> &rayDirs, const bool rectData) const {
    return acos(ComputePointMinimalRayAngleDot(iTrk, rayDirs, rectData)) * FACTOR_RAD_TO_DEG;
}

float Sequence::ComputePointMaximalRayAngle(const TrackIndex &iTrk, AlignedVector<Point3D> &rayDirs, FrameIndex &iFrm1, FrameIndex &iFrm2,
        const bool rectData) const {
    return acos(ComputePointMinimalRayAngleDot(iTrk, rayDirs, iFrm1, iFrm2, rectData)) * FACTOR_RAD_TO_DEG;
}