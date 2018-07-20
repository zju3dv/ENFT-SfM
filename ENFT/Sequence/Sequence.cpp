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
#include "Sequence.h"
#include "Utility/Utility.h"
#include <cvd/image.h>
#include <cvd/image_io.h>

void Sequence::operator = (const Sequence &seq) {
    m_tag = seq.m_tag;
    m_K = seq.m_K;
    m_Kr = seq.m_Kr;
    m_intrinsicType = seq.m_intrinsicType;
    m_Cs = seq.m_Cs;
    m_Krs = seq.m_Krs;
    m_Ps = seq.m_Ps;
    m_Xs = seq.m_Xs;
    m_xs = seq.m_xs;
    m_descs = seq.m_descs;
    m_mapFrmToMea = seq.m_mapFrmToMea;
    m_mapTrkToMea = seq.m_mapTrkToMea;
    m_mapMeaToFrm = seq.m_mapMeaToFrm;
    m_mapMeaToTrk = seq.m_mapMeaToTrk;
    m_frmStates = seq.m_frmStates;
    m_trkStates = seq.m_trkStates;
    m_meaStates = seq.m_meaStates;
    m_trkClrs = seq.m_trkClrs;
    m_measNormalized = seq.m_measNormalized;
}

void Sequence::Swap(Sequence &seq) {
    m_tag.Swap(seq.m_tag);
    IntrinsicMatrix KTmp;
    Camera::IntrinsicParameter KrTmp;
    IntrinsicType intrinsicTypeTmp;
    SWAP(m_K, seq.m_K, KTmp);
    SWAP(m_Kr, seq.m_Kr, KrTmp);
    SWAP(m_intrinsicType, seq.m_intrinsicType, intrinsicTypeTmp);

    m_Cs.Swap(seq.m_Cs);
    m_Krs.Swap(seq.m_Krs);
    m_Ps.Swap(seq.m_Ps);
    m_Xs.Swap(seq.m_Xs);
    m_xs.Swap(seq.m_xs);
    m_descs.Swap(seq.m_descs);

    m_mapFrmToMea.swap(seq.m_mapFrmToMea);
    m_mapTrkToMea.swap(seq.m_mapTrkToMea);
    m_mapMeaToFrm.swap(seq.m_mapMeaToFrm);
    m_mapMeaToTrk.swap(seq.m_mapMeaToTrk);

    m_frmStates.swap(seq.m_frmStates);
    m_trkStates.swap(seq.m_trkStates);
    m_meaStates.swap(seq.m_meaStates);

    m_trkClrs.swap(seq.m_trkClrs);

    bool bTmp;
    SWAP(m_measNormalized, seq.m_measNormalized, bTmp);
}

void Sequence::SetTag(const SequenceTag &tag) {
    m_tag = tag;
    Initialize();
}

void Sequence::SetTag(const std::string &seqDir, const std::string &seqName,
                      const int &iStart, const int &iStep, const int &iEnd) {
    m_tag.Set(seqDir, seqName, iStart, iStep, iEnd);
    Initialize();
}

void Sequence::SetTag(const std::vector<std::string> &imgFileNames) {
    m_tag.Set(imgFileNames);
    Initialize();
}

void Sequence::ChangeTag(const std::string &seqDir, const std::string &seqName,
                         const int iStart, const int iStep, const int iEnd, const bool copyImgs) {
    char command[MAX_LINE_LENGTH];
    const SequenceTag tagBkp = m_tag;
    if(seqName == "")
        m_tag.Set(seqDir, tagBkp.GetSequenceName(), iStart, iStep, iEnd,
                  tagBkp.GetImageWidth(), tagBkp.GetImageHeight());
    else
        m_tag.Set(seqDir, seqName, iStart, iStep, iEnd, tagBkp.GetImageWidth(),
                  tagBkp.GetImageHeight());

    if(!copyImgs)
        return;
    CVD::Image<CVD::Rgb<ubyte> > img;
    //system("Cmd");
    CreateDirectory(seqDir.c_str(), 0);
    const FrameIndex nFrms = m_tag.GetFramesNumber();
    for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        const std::string &imgFileName1 = tagBkp.GetImageFileName(iFrm);
        const std::string &imgFileName2 = m_tag.GetImageFileName(iFrm);
        if(imgFileName1 == imgFileName2 || access(imgFileName2.c_str(), 0) == 0)
            continue;
        const std::string ext1 = IO::ExtractFileExtension(imgFileName1);
        const std::string ext2 = IO::ExtractFileExtension(imgFileName2);
        if(ext1 == ext2) {
            sprintf(command, "copy %s %s", imgFileName1.c_str(), imgFileName2.c_str());
            printf("\r%s", command);
            //system(command);
            system(IO::ReplaceSubString(command, "/", "\\").c_str());
        } else {
            CVD::img_load(img, imgFileName1);
            CVD::img_save(img, imgFileName2);
            printf("\rSaved \'%s\'", imgFileName2.c_str());
        }
    }
    printf("\n");
}

void Sequence::SetCalib(const char *calibFileName, bool focalConst) {
    if (calibFileName!="")
        SetIntrinsicType(INTRINSIC_USER_FIXED);
    else if (focalConst)
        SetIntrinsicType(INTRINSIC_CONSTANT);
    else
        SetIntrinsicType(INTRINSIC_VARIABLE);
    LoadCalibrationFile(calibFileName);
}

void Sequence::Initialize() {
    const FrameIndex nFrms = m_tag.GetFramesNumber();
    m_Cs.Reserve(nFrms);
    m_Cs.Resize(0);
    m_Krs.Reserve(nFrms);
    m_Krs.Resize(0);
    m_Ps.Reserve(nFrms);
    m_Ps.Resize(0);
    m_Xs.Resize(0);
    m_xs.Resize(0);
    m_descs.Resize(0);

    m_mapFrmToMea.assign(1, 0);
    m_mapTrkToMea.resize(0);
    m_mapMeaToFrm.resize(0);
    m_mapMeaToTrk.resize(0);

    m_frmStates.resize(0);
    m_trkStates.resize(0);
    m_meaStates.resize(0);
    m_trkClrs.resize(0);
}

void Sequence::Clear(const bool clearStates) {
    m_tag.Clear();

    m_Cs.Clear();
    m_Krs.Clear();
    m_Ps.Clear();
    m_Xs.Clear();
    m_xs.Clear();
    m_descs.Clear();

    m_mapFrmToMea.clear();
    m_mapTrkToMea.clear();
    m_mapMeaToFrm.clear();
    m_mapMeaToTrk.clear();

    if(clearStates) {
        m_frmStates.clear();
        m_trkStates.clear();
        m_meaStates.clear();
    }
    m_trkClrs.clear();
}

void Sequence::SetIntrinsicRectification(const float &f, const float &d) {
    if(m_intrinsicType == Sequence::INTRINSIC_VARIABLE) {
        const FrameIndex nFrms = FrameIndex(m_Krs.Size());
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
            m_Krs[iFrm].Set(f, d);
    } else
        m_Kr.Set(f, d);
    ComputeProjectiveMatrixes();
}

void Sequence::GetIntrinsicRectificationFocalRange(float &fMin,
        float &fMax) const {
    if(m_intrinsicType == INTRINSIC_VARIABLE) {
        float f;
        fMin = FLT_MAX;
        fMax = 0.0f;
        const FrameIndex nFrms = FrameIndex(m_Krs.Size());
        for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
                continue;
            f = m_Krs[iFrm].f();
            if(f < fMin)
                fMin = f;
            if(f > fMax)
                fMax = f;
        }
    } else
        fMin = fMax = 1.0f;
}

void Sequence::Resize(const FrameIndex &nFrms, const TrackIndex &nTrks,
                      const MeasurementIndex &nMeas, const bool resizeCam, const bool resizePt,
                      const bool resizeDesc) {
    if(resizeCam) {
        m_Cs.Resize(nFrms);
        if(m_intrinsicType == INTRINSIC_VARIABLE)
            m_Krs.Resize(nFrms);
        m_Ps.Resize(nFrms);
    }
    if(resizePt)
        m_Xs.Resize(nTrks);
    m_xs.Resize(nMeas);
    if(resizeDesc) {
#if DESCRIPTOR_TRACK
        m_descs.Resize(nTrks);
#else
        m_descs.Resize(nMeas);
#endif
    }

    m_mapFrmToMea.resize(nFrms + 1);
    //m_mapTrkToMea.assign(nTrks, MeasurementIndexList());
    m_mapTrkToMea.resize(nTrks);
    for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
        m_mapTrkToMea[iTrk].resize(0);
    m_mapMeaToFrm.resize(nMeas);
    m_mapMeaToTrk.resize(nMeas);

    m_frmStates.resize(nFrms);
    m_trkStates.resize(nTrks);
    m_meaStates.resize(nMeas);
    m_trkClrs.resize(nTrks);
}