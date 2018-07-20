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
#include "SfM/Quaternion.h"
#include "Utility/Table.h"
#include "Utility/Utility.h"
#include <cvd/image_io.h>

void Sequence::LoadCalibrationFile(const char *fileName) {
    if (fileName && m_K.Load(fileName)) {
        printf("Loaded \'%s\'\n", fileName);
    } else {
        const float fx = (GetImageWidth() + GetImageHeight()) * 0.5f, fy = fx,
                    cx = (GetImageWidth() - 1) * 0.5f, cy = (GetImageHeight() - 1) * 0.5f;
        m_K.Set(fx, fy, cx, cy);
        //m_K.Print();
    }
    m_Kr.Set(1.0f, 0.0f);
}

void Sequence::LoadTrackColors() {
    TrackIndex nTrksClrLoaded = TrackIndex(m_trkClrs.size());
    const TrackIndex nTrksAll = GetTracksNumber();
    if (nTrksClrLoaded == nTrksAll) {
        return;
    }
    std::vector<bool> trkClrLoadedMarks(nTrksAll, true);
    for (TrackIndex iTrk = nTrksClrLoaded; iTrk < nTrksAll; ++iTrk) {
        trkClrLoadedMarks[iTrk] = false;
    }
    m_trkClrs.resize(nTrksAll);

    CVD::Image<CVD::Rgb<ubyte> > img;

    const FrameIndex nFrms = GetFramesNumber();
    for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        const FeatureIndex nFtrs = GetFrameFeaturesNumber(iFrm);
        const TrackIndex *iTrks = GetFrameTrackIndexes(iFrm);
        bool hasNewTrk = false;
        for (FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr) {
            const TrackIndex iTrk = iTrks[iFtr];
            if (iTrk != INVALID_TRACK_INDEX && !trkClrLoadedMarks[iTrk]) {
                hasNewTrk = true;
                break;
            }
        }
        if (!hasNewTrk) {
            continue;
        }

        CVD::img_load(img, GetImageFileName(iFrm));
        const Point2D *ftrs = GetFrameFeatures(iFrm);
        for (FeatureIndex iFtr = 0; iFtr < nFtrs; ++iFtr) {
            const TrackIndex iTrk = iTrks[iFtr];
            if (iTrk == INVALID_TRACK_INDEX || trkClrLoadedMarks[iTrk]) {
                continue;
            }
            trkClrLoadedMarks[iTrk] = true;
            printf("\rLoading track colors...%d%%", nTrksClrLoaded * 100 / nTrksAll);
            ++nTrksClrLoaded;

            Point2D x = ftrs[iFtr];
            if (m_measNormalized) {
                m_K.NormalizedPlaneToImage(x);
            }
            m_trkClrs[iTrk] = img[int(x.y() + 0.5f)][int(x.x() + 0.5f)];
        }
    }
    printf("\rLoading track colors...100%%\n");
}

bool Sequence::HasSwappedOut() const {
    char buf[MAX_LINE_LENGTH];
    sprintf(buf, "seq_%d-%d-%d.txt", GetStartFrame(), GetStepFrame(),
            GetEndFrame());
    return access((GetDirectory() + buf).c_str(), 0) == 0;
}

void Sequence::SwapOut() {
    char buf[MAX_LINE_LENGTH];
    sprintf(buf, "seq_%d-%d-%d.txt", GetStartFrame(), GetStepFrame(),
            GetEndFrame());
    SaveB(buf);
    Clear(false);
}

void Sequence::SwapIn() {
    char buf[MAX_LINE_LENGTH];
    sprintf(buf, "seq_%d-%d-%d.txt", GetStartFrame(), GetStepFrame(),
            GetEndFrame());
    LoadB(buf);
}

bool Sequence::SaveActb(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if (fp == nullptr) {
        return false;
    }

    AlignedVector<Point2D> xs(m_xs.Size());
    if (m_measNormalized) {
        m_K.NormalizedPlaneToImageN(m_xs, xs);
    } else {
        xs.CopyFrom(m_xs);
    }

    char buf[MAX_LINE_LENGTH];
    strcpy(buf, "#camera track binary project file 1.0\n");
    fwrite(buf, 1, strlen(buf), fp);
    m_tag.SaveActb(fp);

    const int motionType = 1;   // Free move
    const int intrinsicType = m_intrinsicType == INTRINSIC_USER_FIXED ? 0x1111 :
                              (m_intrinsicType == INTRINSIC_CONSTANT ? 0x1112 : 0x1114);
    fwrite(&motionType, sizeof(int), 1, fp);
    fwrite(&intrinsicType, sizeof(int), 1, fp);

    const double fx = double(m_K.fx() * m_Kr.f()), fy = double(m_K.fy() * m_Kr.f()),
                 cx = double(m_K.cx()), cy = double(m_K.cy()), skew = 0.0, aspectRatio = 1.0;
    fwrite(&fx, sizeof(double), 1, fp);
    fwrite(&fy, sizeof(double), 1, fp);
    fwrite(&cx, sizeof(double), 1, fp);
    fwrite(&cy, sizeof(double), 1, fp);
    fwrite(&skew, sizeof(double), 1, fp);
    fwrite(&aspectRatio, sizeof(double), 1, fp);

    bool bool1 = true;
    fwrite(&bool1, sizeof(bool), 1, fp);

    const int dim = m_descs.Empty() ? 0 : DESCRIPTOR_DIMENSION;
    fwrite(&dim, sizeof(int), 1, fp);

    MeasurementIndex iMea;
    const Point3D zero(0, 0, 0);
    int int1, int3[3];
    double dbl3[3];
    float flt2[2];
    const TrackIndex nTrks = GetTracksNumber();
    int1 = int(nTrks);
    fwrite(&int1, sizeof(int), 1, fp);
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        const Point3D &X = (m_Xs.Empty() ||
                            !(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED)) ? zero : m_Xs[iTrk];
        dbl3[0] = double(X.X());
        dbl3[1] = double(X.Y());
        dbl3[2] = double(X.Z());
        const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
        int3[0] = int(iMeas.size());
        int3[1] = (m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED) ? 0 : -1;
        int3[2] = (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) ? 1 : 0;
        fwrite(int3, sizeof(int), 3, fp);
        fwrite(dbl3, sizeof(double), 3, fp);

        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        for (FrameIndex i = 0; i < nCrsps; ++i) {
            iMea = iMeas[i];
            int1 = int(m_mapMeaToFrm[iMea]);
            flt2[0] = xs[iMea].x();
            flt2[1] = xs[iMea].y();
            fwrite(&int1, sizeof(int), 1, fp);
            fwrite(flt2, sizeof(float), 2, fp);
        }
#if DESCRIPTOR_TRACK
        if (dim != 0) {
            fwrite(m_descs[iTrk], sizeof(Descriptor), 1, fp);
        }
#else
        if (dim != 0) {
            fwrite(m_descs[iMeas[0]], sizeof(Descriptor), 1, fp);
        }
#endif
    }

    bool1 = false;
    fwrite(&bool1, sizeof(bool), 1, fp);

    int1 = 2;
    fwrite(&int1, sizeof(int), 1, fp);

    const FrameIndex nFrms = GetFramesNumber();
    int1 = int(nFrms);
    fwrite(&int1, sizeof(int), 1, fp);
    double dbl17[17];
    //dbl17[ 0] = (m_K.fx() + m_K.fy()) * 0.5;
    dbl17[13] = 0;
    dbl17[14] = 0;
    dbl17[15] = 0;
    dbl17[16] = 1;
    if (m_Cs.Empty()) {
        dbl17[ 0] = -1.0;
        dbl17[ 1] = 1.0;
        dbl17[ 2] = 0.0;
        dbl17[ 3] = 0.0;
        dbl17[ 4] = 0.0;
        dbl17[ 5] = 0.0;
        dbl17[ 6] = 1.0;
        dbl17[ 7] = 0.0;
        dbl17[ 8] = 0.0;
        dbl17[ 9] = 0.0;
        dbl17[10] = 0.0;
        dbl17[11] = 1.0;
        dbl17[12] = 0.0;
        for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            fwrite(dbl17, sizeof(double), 17, fp);
        }
    } else {
        for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            if (!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED)) {
                dbl17[0] = -1.0;
            } else if (m_intrinsicType == INTRINSIC_VARIABLE) {
                dbl17[0] = (m_K.fx() + m_K.fy()) * 0.5 * m_Krs[iFrm].f();
            } else {
                dbl17[0] = (fx + fy) * 0.5;
            }
            const Camera &C = m_Cs[iFrm];
            dbl17[ 1] = double(C.r00());
            dbl17[ 2] = double(C.r01());
            dbl17[ 3] = double(C.r02());
            dbl17[ 4] = double(C.tX());
            dbl17[ 5] = double(C.r10());
            dbl17[ 6] = double(C.r11());
            dbl17[ 7] = double(C.r12());
            dbl17[ 8] = double(C.tY());
            dbl17[ 9] = double(C.r20());
            dbl17[10] = double(C.r21());
            dbl17[11] = double(C.r22());
            dbl17[12] = double(C.tZ());
            fwrite(dbl17, sizeof(double), 17, fp);
        }
    }

    //int1 = 0;
    //fwrite(&int1, sizeof(int), 1, fp);

    int1 = int(m_trkClrs.size());
    fwrite(&int1, sizeof(int), 1, fp);
    fwrite(m_trkClrs.data(), sizeof(CVD::Rgb<ubyte>), int1, fp);

    fclose(fp);

    printf("Saved \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::LoadActb(const char *fileName, const bool normalizeMeas,
                        const bool loadClr) {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "rb");
    if (fp == nullptr) {
        return false;
    }

    char buf[MAX_LINE_LENGTH];
    fgets(buf, MAX_LINE_LENGTH, fp);
    if (strcmp(buf, "#camera track binary project file 1.0\n") != 0) {
        return false;
    }

    //Parse sequence
    m_tag.LoadActb(fp);
    const FrameIndex nFrms = FrameIndex(m_tag.GetFramesNumber());

    // Parse motion type
    int motionType, intrinsicType;
    fread(&motionType, sizeof(int), 1, fp);
    fread(&intrinsicType, sizeof(int), 1, fp);
    intrinsicType = intrinsicType & 0x000f;
    switch (intrinsicType) {
        case 1:
            m_intrinsicType = INTRINSIC_USER_FIXED;
            break;
        case 2:
            m_intrinsicType = INTRINSIC_CONSTANT;
            break;
        case 4:
            m_intrinsicType = INTRINSIC_VARIABLE;
            break;
        default:
            m_intrinsicType = INTRINSIC_USER_FIXED;
            break;
    }

    // Parse intrinsic
    double dbl6[6];
    fread(dbl6, sizeof(dbl6), 1, fp);
    m_K.Set(float(dbl6[0]), float(dbl6[1]), float(dbl6[2]), float(dbl6[3]));
    m_Kr.Set(1.0f, 0.0f);

    // Parse tracks
    bool bool1;
    fread(&bool1, sizeof(bool), 1, fp);
    int dim;
    fread(&dim, sizeof(int), 1, fp);

    int int1, int3[3];
    float flt2[2];
    fread(&int1, sizeof(int), 1, fp);
    const TrackIndex nTrks = TrackIndex(int1);
#if DESCRIPTOR_TRACK
    if (dim != 0) {
        m_descs.Resize(nTrks);
    }
#else
    AlignedVector<Descriptor> descs;
    if (dim != 0) {
        descs.Resize(nTrks);
    }
#endif

    m_Cs.Resize(nFrms);
    if (m_intrinsicType == INTRINSIC_VARIABLE) {
        m_Krs.Resize(nFrms);
    }
    m_Xs.Resize(nTrks);
    m_mapFrmToMea.resize(nFrms + 1);
    m_mapTrkToMea.assign(nTrks, MeasurementIndexList());
    //m_mapTrkToPlane.assign(nTrks, INVALID_PLANE_INDEX);
    m_frmStates.assign(nFrms,
                       FLAG_FRAME_STATE_SOLVED/* | FLAG_FRAME_STATE_KEY_FRAME*/);
    m_frmStates.front() = m_frmStates.back() |= FLAG_FRAME_STATE_KEY_FRAME;
    m_trkStates.assign(nTrks, FLAG_TRACK_STATE_DEFAULT);

    FrameMeasurementMap frmFtrCnts(nFrms, 0);
    std::vector<Point2D> xs;
    MeasurementFrameMap mapMea2Frm;
    MeasurementTrackMap mapMea2Trk;
    MeasurementStateList meaStates;
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        fread(int3, sizeof(int), 3, fp);
        if (int3[1] != -1) {
            m_trkStates[iTrk] |= FLAG_TRACK_STATE_SOLVED;
            if (int3[2] == 1) {
                m_trkStates[iTrk] |= FLAG_TRACK_STATE_INLIER;
            }
        }
        const ubyte inlier = (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER);
        //if(int3[2] == 1)
        //  m_trkStates[iTrk] |= FLAG_TRACK_STATE_INLIER;

        fread(dbl6, sizeof(double), 3, fp);
        m_Xs[iTrk].Set(float(dbl6[0]), float(dbl6[1]), float(dbl6[2]));

        const FrameIndex nCrsps = FrameIndex(int3[0]);
        for (FrameIndex i = 0; i < nCrsps; ++i) {
            fread(&int1, sizeof(int), 1, fp);
            const FrameIndex iFrm = FrameIndex(int1);
            ++frmFtrCnts[iFrm];
            fread(flt2, sizeof(float), 2, fp);
            xs.push_back(Point2D(flt2[0], flt2[1]));
            mapMea2Frm.push_back(iFrm);
            mapMea2Trk.push_back(iTrk);
            //meaStates.push_back(inlier ? FLAG_MEASUREMENT_STATE_DEFAULT : FLAG_MEASUREMENT_STATE_OUTLIER);
            meaStates.push_back(FLAG_MEASUREMENT_STATE_DEFAULT);
        }
        if (dim == 0) {
            continue;
        }
#if DESCRIPTOR_TRACK
        fread(m_descs[iTrk], sizeof(float), dim, fp);
        m_descs[iTrk].Normalize();
#else
        fread(descs[iTrk], sizeof(float), dim, fp);
        descs[iTrk].Normalize();
#endif
    }

    // Reorder measurements from TRACK-ORDER to FRAME_ORDER
    FrameMeasurementMap &frmMeaIdxs = frmFtrCnts;
    m_mapFrmToMea[0] = 0;
    for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        m_mapFrmToMea[iFrm + 1] = m_mapFrmToMea[iFrm] + frmFtrCnts[iFrm];
        frmMeaIdxs[iFrm] = m_mapFrmToMea[iFrm];
    }
    const MeasurementIndex nMeas = MeasurementIndex(xs.size());
    m_xs.Resize(nMeas);
#if DESCRIPTOR_TRACK == 0
    if (dim != 0) {
        m_descs.Resize(nMeas);
    }
#endif
    m_mapMeaToFrm.resize(nMeas);
    m_mapMeaToTrk.resize(nMeas);
    m_meaStates.resize(nMeas);
    for (MeasurementIndex iMea = 0; iMea < nMeas; ++iMea) {
        const FrameIndex iFrm = mapMea2Frm[iMea];
        const TrackIndex iTrk = mapMea2Trk[iMea];
        const MeasurementIndex iMeaRe = frmMeaIdxs[iFrm]++;
        m_xs[iMeaRe] = xs[iMea];
#if DESCRIPTOR_TRACK == 0
        if (dim != 0) {
            m_descs[iMeaRe] = descs[iTrk];
        }
#endif
        m_mapMeaToFrm[iMeaRe] = iFrm;
        m_mapMeaToTrk[iMeaRe] = iTrk;
        m_meaStates[iMeaRe] = meaStates[iMea];
        m_mapTrkToMea[iTrk].push_back(iMeaRe);
    }
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        std::sort(m_mapTrkToMea[iTrk].begin(), m_mapTrkToMea[iTrk].end());
    }
    m_measNormalized = false;
    if (normalizeMeas) {
        NormalizeMeasurements();
    }

    // Parse cameras
    fread(buf, sizeof(bool) + sizeof(int), 1, fp);
    double dbl17[17];
    fread(&int1, sizeof(int), 1, fp);
    for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        fread(&dbl17, sizeof(double), 17, fp);
        if (m_intrinsicType == INTRINSIC_VARIABLE) {
            m_Krs[iFrm].Set(float(dbl17[0] / ((m_K.fx() + m_K.fy()) * 0.5)), 0.0f);
        }
        Camera &C = m_Cs[iFrm];
        C.r00() = float(dbl17[ 1]);
        C.r01() = float(dbl17[ 2]);
        C.r02() = float(dbl17[ 3]);
        C.tX() = float(dbl17[ 4]);
        C.r10() = float(dbl17[ 5]);
        C.r11() = float(dbl17[ 6]);
        C.r12() = float(dbl17[ 7]);
        C.tY() = float(dbl17[ 8]);
        C.r20() = float(dbl17[ 9]);
        C.r21() = float(dbl17[10]);
        C.r22() = float(dbl17[11]);
        C.tZ() = float(dbl17[12]);
    }
    //SetReferenceFrame(0);

    if (loadClr && IO::VectorLoadB(m_trkClrs, fp) == 0) {
        LoadTrackColors();
    }
    fclose(fp);

    printf("Loaded \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::SaveAct(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if (fp == 0) {
        return false;
    }

    AlignedVector<Point2D> xs(m_xs.Size());
    if (m_measNormalized) {
        m_K.NormalizedPlaneToImageN(m_xs, xs);
    } else {
        xs.CopyFrom(m_xs);
    }

    fprintf(fp, "#camera track project file\n");
    fprintf(fp, "<Image Sequence>\n");
    m_tag.SaveAct(fp);
    fprintf(fp, "</Image Sequence>\n");

    fprintf(fp, "<Motion Type>\n");
    fprintf(fp, "FREE_MOVE\n");
    switch (m_intrinsicType) {
        case INTRINSIC_USER_FIXED:
            fprintf(fp, "FOCAL_KNOWN\n");
            break;
        case INTRINSIC_CONSTANT:
            fprintf(fp, "FOCAL_CONSTANT\n");
            break;
        case INTRINSIC_VARIABLE:
            fprintf(fp, "FOCAL_VARIABLE\n");
            break;
    }
    fprintf(fp, "PRINCIPAL_KNOWN\n");
    fprintf(fp, "SKEW_KNOWN\n");
    fprintf(fp, "</Motion Type>\n");

    fprintf(fp, "<intrinsic parameter>\n");
    const float fx = m_K.fx() * m_Kr.f(), fy = m_K.fy() * m_Kr.f(), cx = m_K.cx(),
                cy = m_K.cy(), skew = 0.0f, aspectRatio = 1.0f;
    fprintf(fp, "%f %f %f %f %f %f", fx, fy, cx, cy, skew, aspectRatio);
    if (m_intrinsicType == INTRINSIC_CONSTANT) {
        fprintf(fp, " %f\n", m_Kr.d() * m_Kr.f() * m_Kr.f());
    } else {
        fprintf(fp, "\n");
    }
    fprintf(fp, "</intrinsic parameter>\n");

    const Point3D zero(0, 0, 0);
    fprintf(fp, "<Feature Tracks>\n");
    const int dim = m_descs.Empty() ? 0 : DESCRIPTOR_DIMENSION;
    if (dim != 0) {
        fprintf(fp, "kpts: %d\n", dim);
    }
    const TrackIndex nTrks = GetTracksNumber();
    fprintf(fp, "%d\n", nTrks);
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        fprintf(fp, "%d %d %d ", nCrsps,
                (m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED) ? 0 : -1,
                (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) ? 1 : 0);
        const Point3D &X = (m_Xs.Empty() ||
                            !(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED)) ? zero : m_Xs[iTrk];
        X.Save(fp);

        MeasurementIndex iMea;
        for (FrameIndex i = 0; i < nCrsps; ++i) {
            iMea = iMeas[i];
            fprintf(fp, "%d %f %f ", m_mapMeaToFrm[iMea], xs[iMea].x(), xs[iMea].y());
        }
        fprintf(fp, "\n");
        if (dim == 0) {
            continue;
        }
#if DESCRIPTOR_TRACK
        const Descriptor &desc = m_descs[iTrk];
#else
        const Descriptor &desc = m_descs[iMeas[0]];
#endif
        for (int d = 0; d < dim; ++d) {
            fprintf(fp, "%f ", desc[d]);
        }
        fprintf(fp, "\n");
    }
    fprintf(fp, "</Feature Tracks>\n");

    fprintf(fp, "<Camera Track>\n");
    const FrameIndex nFrms = GetFramesNumber();
    if (m_Cs.Empty()) {
        for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            fprintf(fp, "<FRAME%d>\n", iFrm);
            //fprintf(fp, "%f\n", m_K.fx());
            fprintf(fp, "-1\n");
            fprintf(fp, "1, 0, 0, 0\n");
            fprintf(fp, "0, 1, 0, 0\n");
            fprintf(fp, "0, 0, 1, 0\n");
            fprintf(fp, "0, 0, 0, 1\n");
            fprintf(fp, "</FRAME%d>\n", iFrm);
        }
    } else {
        for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            fprintf(fp, "<FRAME%d>\n", iFrm);
            if (!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED)) {
                fprintf(fp, "-1\n");
            } else if (m_intrinsicType == INTRINSIC_VARIABLE) {
                fprintf(fp, "%f\n", (m_K.fx() + m_K.fy()) * 0.5 * m_Krs[iFrm].f());
            } else {
                fprintf(fp, "%f\n", (fx + fy) * 0.5);
            }
            m_Cs[iFrm].Save(fp);
            fprintf(fp, "0, 0, 0, 1\n");
            fprintf(fp, "</FRAME%d>\n", iFrm);
        }
    }
    fprintf(fp, "</Camera Track>\n");
    fclose(fp);

    printf("Saved \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::SaveActc(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if (fp == nullptr) {
        return false;
    }
    AlignedVector<Point2D> xs(m_xs.Size());
    if (m_measNormalized) {
        m_K.NormalizedPlaneToImageN(m_xs, xs);
    } else {
        xs.CopyFrom(m_xs);
    }

    fprintf(fp, "#camera track project file\n");
    fprintf(fp, "<Image Sequence>\n");
    m_tag.SaveAct(fp);
    fprintf(fp, "</Image Sequence>\n");
    fprintf(fp, "<Camera Track>\n");
    const FrameIndex nFrms = GetFramesNumber();
    const float fx = m_K.fx() * m_Kr.f(), fy = m_K.fy() * m_Kr.f(), cx = m_K.cx(),
                cy = m_K.cy(), skew = 0.0f, aspectRatio = 1.0f;
    if (m_Cs.Empty()) {
        for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            fprintf(fp, "<FRAME%d>\n", iFrm);
            //fprintf(fp, "%f\n", m_K.fx());
            fprintf(fp, "-1\n");
            fprintf(fp, "1, 0, 0, 0\n");
            fprintf(fp, "0, 1, 0, 0\n");
            fprintf(fp, "0, 0, 1, 0\n");
            fprintf(fp, "0, 0, 0, 1\n");
            fprintf(fp, "</FRAME%d>\n", iFrm);
        }
    } else {
        for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            fprintf(fp, "<FRAME%d>\n", iFrm);
            if (!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED)) {
                fprintf(fp, "-1\n");
            } else if (m_intrinsicType == INTRINSIC_VARIABLE) {
                fprintf(fp, "%f\n", (m_K.fx() + m_K.fy()) * 0.5 * m_Krs[iFrm].f());
            } else {
                fprintf(fp, "%f\n", (fx + fy) * 0.5);
            }
            m_Cs[iFrm].Save(fp);
            fprintf(fp, "0, 0, 0, 1\n");
            fprintf(fp, "</FRAME%d>\n", iFrm);
        }
    }
    fprintf(fp, "</Camera Track>\n");
    fclose(fp);

    printf("Saved \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

inline static bool GetLine(FILE *fp, const char *content, char *line) {
    fgets(line, MAX_LINE_LENGTH, fp);
    if (content) {
        while (strstr(line, content) == NULL) {
            fgets(line, MAX_LINE_LENGTH, fp);
        }
    } else {
        while (strlen(line) == 0) {
            fgets(line, MAX_LINE_LENGTH, fp);
        }
    }
    return true;
}

bool Sequence::LoadAct(const char *fileName, const bool normalizeMeas,
                       const bool loadClr, const bool skipCam) {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "r");
    if (fp == nullptr) {
        return false;
    }

    //Parse sequence
    char line[MAX_LINE_LENGTH];
    GetLine(fp, "<Image Sequence>", line);
    m_tag.LoadAct(fp);
    GetLine(fp, "</Image Sequence>", line);
    const FrameIndex nFrms = FrameIndex(m_tag.GetFramesNumber());

    // Parse motion type
    GetLine(fp, "<Motion Type>", line);
    fscanf(fp, "%s", line);
    fscanf(fp, "%s", line);
    const std::string intrinsicType = line;
    if (intrinsicType == "FOCAL_KNOWN") {
        m_intrinsicType = INTRINSIC_USER_FIXED;
    } else if (intrinsicType == "FOCAL_CONSTANT") {
        m_intrinsicType = INTRINSIC_CONSTANT;
    } else if (intrinsicType == "FOCAL_VARIABLE") {
        m_intrinsicType = INTRINSIC_VARIABLE;
    } else {
        m_intrinsicType = INTRINSIC_USER_FIXED;
    }
    GetLine(fp, "</Motion Type>", line);

    // Parse intrinsic
    float fx, fy, cx, cy, skew, aspectRatio, d;
    GetLine(fp, "<intrinsic parameter>", line);
    GetLine(fp, NULL, line);
    if (sscanf(line, "%f %f %f %f %f %f %f", &fx, &fy, &cx, &cy, &skew,
               &aspectRatio,
               &d) == 6) {
        d = 0.0f;
    }
    GetLine(fp, "</intrinsic parameter>", line);
    m_K.Set(fx, fy, cx, cy);
    m_Kr.Set(1.0f, d);

    // Parse tracks
    int dim, int1;
    GetLine(fp, "<Feature Tracks>", line);
    GetLine(fp, NULL, line);
    if (sscanf(line, "kpts: %d", &dim)) {
        fscanf(fp, "%d", &int1);
    } else {
        dim = 0;
        sscanf(line, "%d", &int1);
    }
    const TrackIndex nTrks = TrackIndex(int1);
#if DESCRIPTOR_TRACK
    if (dim != 0) {
        m_descs.Resize(nTrks);
    }
#else
    std::vector<float> descIncompatible;
    AlignedVector<Descriptor> descs;
    if (dim != 0) {
        if (dim == DESCRIPTOR_DIMENSION) {
            descs.Resize(nTrks);
        } else {
            descIncompatible.resize(dim);
        }
    }

#endif

    m_Cs.Resize(nFrms);
    if (m_intrinsicType == INTRINSIC_VARIABLE) {
        m_Krs.Resize(nFrms);
    }
    m_Xs.Resize(nTrks);
    m_mapFrmToMea.resize(nFrms + 1);
    m_mapTrkToMea.assign(nTrks, MeasurementIndexList());
    //m_mapTrkToPlane.assign(nTrks, INVALID_PLANE_INDEX);
    m_frmStates.assign(nFrms,
                       FLAG_FRAME_STATE_SOLVED/* | FLAG_FRAME_STATE_KEY_FRAME*/);
    m_frmStates.front() = m_frmStates.back() |= FLAG_FRAME_STATE_KEY_FRAME;
    m_trkStates.assign(nTrks, FLAG_TRACK_STATE_DEFAULT);

    FrameMeasurementMap frmFtrCnts(nFrms, 0);
    std::vector<Point2D> xs;
    MeasurementFrameMap mapMea2Frm;
    MeasurementTrackMap mapMea2Trk;
    MeasurementStateList meaStates;
    int int3[3];
    Point2D x;
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        fscanf(fp, "%d %d %d", &int3[0], &int3[1], &int3[2]);
        if (int3[1] != -1) {
            m_trkStates[iTrk] |= FLAG_TRACK_STATE_SOLVED;
            if (int3[2] == 1) {
                m_trkStates[iTrk] |= FLAG_TRACK_STATE_INLIER;
            }
        }
        const ubyte inlier = (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER);

        m_Xs[iTrk].Load(fp);

        const FrameIndex nCrsps = FrameIndex(int3[0]);
        for (FrameIndex i = 0; i < nCrsps; ++i) {
            fscanf(fp, "%d", &int1);
            const FrameIndex iFrm = FrameIndex(int1);
            ++frmFtrCnts[iFrm];
            x.Load(fp);
            xs.push_back(x);
            mapMea2Frm.push_back(iFrm);
            mapMea2Trk.push_back(iTrk);
            //meaStates.push_back(inlier ? FLAG_MEASUREMENT_STATE_DEFAULT : FLAG_MEASUREMENT_STATE_OUTLIER);
            meaStates.push_back(FLAG_MEASUREMENT_STATE_DEFAULT);
        }
        if (dim == 0) {
            continue;
        } else if (dim == DESCRIPTOR_DIMENSION) {
#if DESCRIPTOR_TRACK
            Descriptor &desc = m_descs[iTrk];
#else
            Descriptor &desc = descs[iTrk];
#endif
            for (int d = 0; d < dim; d++) {
                fscanf(fp, "%f", &desc[d]);
            }
            desc.Normalize();
        } else {
            for (int d = 0; d < dim; d++) {
                fscanf(fp, "%f", &descIncompatible[d]);
            }
        }
    }
    GetLine(fp, "</Feature Tracks>", line);

    // Reorder measurements from TRACK-ORDER to FRAME_ORDER
    FrameMeasurementMap &frmMeaIdxs = frmFtrCnts;
    m_mapFrmToMea[0] = 0;
    for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        m_mapFrmToMea[iFrm + 1] = m_mapFrmToMea[iFrm] + frmFtrCnts[iFrm];
        frmMeaIdxs[iFrm] = m_mapFrmToMea[iFrm];
    }
    const MeasurementIndex nMeas = MeasurementIndex(xs.size());
    m_xs.Resize(nMeas);
#if DESCRIPTOR_TRACK == 0
    m_descs.Resize(nMeas);
#endif
    m_mapMeaToFrm.resize(nMeas);
    m_mapMeaToTrk.resize(nMeas);
    m_meaStates.resize(nMeas);
    for (MeasurementIndex iMea = 0; iMea < nMeas; ++iMea) {
        const FrameIndex iFrm = mapMea2Frm[iMea];
        const TrackIndex iTrk = mapMea2Trk[iMea];
        const MeasurementIndex iMeaRe = frmMeaIdxs[iFrm]++;
        m_xs[iMeaRe] = xs[iMea];
#if DESCRIPTOR_TRACK == 0
        if (dim != 0 && dim == DESCRIPTOR_DIMENSION) {
            m_descs[iMeaRe] = descs[iTrk];
        }
#endif
        m_mapMeaToFrm[iMeaRe] = iFrm;
        m_mapMeaToTrk[iMeaRe] = iTrk;
        m_meaStates[iMeaRe] = meaStates[iMea];
        m_mapTrkToMea[iTrk].push_back(iMeaRe);
    }
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        std::sort(m_mapTrkToMea[iTrk].begin(), m_mapTrkToMea[iTrk].end());
    }
    m_measNormalized = false;
    if (normalizeMeas) {
        NormalizeMeasurements();
    }

    if (!skipCam) {
        // Parse cameras
        char buf[MAX_LINE_LENGTH];
        GetLine(fp, "<Camera Track>", line);
        for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
            sprintf(buf, "<FRAME%d>", iFrm);
            GetLine(fp, buf, line);
            fscanf(fp, "%f", &fx);
            if (m_intrinsicType == INTRINSIC_VARIABLE) {
                m_Krs[iFrm].Set(fx / ((m_K.fx() + m_K.fy()) * 0.5f), 0.0f);
            }
            m_Cs[iFrm].Load(fp);
            sprintf(buf, "</FRAME%d>", iFrm);
            GetLine(fp, buf, line);
        }
        GetLine(fp, "</Camera Track>", line);
    }
    fclose(fp);

    //SetReferenceFrame(0);

    if (loadClr) {
        LoadTrackColors();
    }

    printf("Loaded \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::SaveNvm(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if (fp == nullptr) {
        return false;
    }
    SaveNvm(fp);
    fclose(fp);
    printf("Saved \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

void Sequence::SaveNvm(FILE *fp) const {
    AlignedVector<Point2D> xs(m_xs.Size());
    if (m_measNormalized) {
        m_K.NormalizedPlaneToImageN(m_xs, xs);
    } else {
        xs.CopyFrom(m_xs);
    }

    if (m_intrinsicType == INTRINSIC_USER_FIXED)
        fprintf(fp, "NVM_V3 FixedK %f %f %f %f 0\n\n", m_K.fx(), m_K.cx(), m_K.fy(),
                m_K.cy());

    const FrameIndex nFrms = GetFramesNumber();
    fprintf(fp, "%d\n", nFrms);

    float f, d, q[4];
    Point3D center;
    FrameIndex iFrm;
    if (m_Cs.Empty()) {
        for (iFrm = 0; iFrm < nFrms; ++iFrm) {
            const std::string imgFileName = m_tag.GetImageFileName(iFrm) == "" ? "*.jpg" :
                                            IO::RemovePrefix(m_tag.GetImageFileName(iFrm), GetDirectory());
            fprintf(fp, "%s 0 0 0 0 0 0 0 0 0 0\n", imgFileName.c_str());
        }
    } else {
        for (iFrm = 0; iFrm < nFrms; ++iFrm) {
            const std::string imgFileName = m_tag.GetImageFileName(iFrm) == "" ? "*.jpg" :
                                            IO::RemovePrefix(m_tag.GetImageFileName(iFrm), GetDirectory());
            switch (m_intrinsicType) {
                case INTRINSIC_USER_FIXED:
                    f = (m_K.fx() + m_K.fy()) * 0.5f;
                    d = 0.0f;
                    break;
                case INTRINSIC_CONSTANT:
                    f = (m_K.fx() + m_K.fy()) * 0.5f * m_Kr.f();
                    d = m_Kr.d() * m_Kr.f() * m_Kr.f();
                    break;
                case INTRINSIC_VARIABLE:
                    f = (m_K.fx() + m_K.fy()) * 0.5f * m_Krs[iFrm].f();
                    d = m_Krs[iFrm].d() * m_Krs[iFrm].f() * m_Krs[iFrm].f();
                    break;
            }
            const Camera &C = m_Cs[iFrm];
            C.ToQuaternion(q);
            C.GetCenter(center);
            fprintf(fp, "%s %f %f %f %f %f %f %f %f %f 0\n", imgFileName.c_str(), f, q[0],
                    q[1], q[2], -q[3], center.X(), center.Y(), center.Z(), d);
        }
    }
    fprintf(fp, "\n");

    const TrackIndex nTrks = GetTracksNumber();
    fprintf(fp, "%d\n", nTrks);

    const Point3D zero1(0, 0, 0);
    const CVD::Rgb<ubyte> zero2(0, 0, 0);
    const Point2D dx(-m_K.cx(), -m_K.cy());
    MeasurementIndex iMea;
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        const Point3D X = m_Xs.Empty() ? zero1 : m_Xs[iTrk];
        const CVD::Rgb<ubyte> clr = m_trkClrs.empty() ? zero2 : m_trkClrs[iTrk];
        const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        fprintf(fp, "%f %f %f %d %d %d %d", X.X(), X.Y(), X.Z(), clr.red, clr.green,
                clr.blue, nCrsps);
        for (FrameIndex i = 0; i < nCrsps; ++i) {
            iMea = iMeas[i];
            iFrm = m_mapMeaToFrm[iMea];
            Point2D &x = xs[iMea];
            x += dx;
            fprintf(fp, " %d %d %f %f", iFrm, iMea - m_mapFrmToMea[iFrm], x.x(), x.y());
        }
        fprintf(fp, "\n");
    }
}

bool Sequence::LoadNvm(const char *fileName, const bool normalizeMeas) {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "r");
    if (fp == nullptr) {
        return false;
    }
    LoadNvm(fp);
    fclose(fp);
    if (normalizeMeas) {
        NormalizeMeasurements();
    }
    printf("Loaded \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

void Sequence::LoadNvm(FILE *fp) {
    int int1;
    char buf[MAX_LINE_LENGTH];
    //fscanf(fp, "%s", buf);
    fgets(buf, MAX_LINE_LENGTH, fp);
    float fx, fy, cx, cy, r;
    if (sscanf(buf, "NVM_V3 FixedK %f %f %f %f %f", &fx, &cx, &fy, &cy, &r) == 5) {
        m_K.Set(fx, fy, cx, cy);
        m_intrinsicType = INTRINSIC_USER_FIXED;
        fscanf(fp, "%d", &int1);
    } else {
        m_intrinsicType = INTRINSIC_VARIABLE;
        sscanf(buf, "%s", buf);
        if (strcmp(buf, "NVM_V3") == 0) {
            fscanf(fp, "%d", &int1);
        } else {
            sscanf(buf, "%d", &int1);
        }
    }
    m_Kr.Set(1.0f, 0.0f);

    FrameIndex iFrm;
    Quaternion q;
    float f, d, t;
    Point3D center;
    const FrameIndex nFrms = FrameIndex(int1);
    AlignedVector<Camera> Cs(nFrms);
    AlignedVector<Camera::IntrinsicParameter> Krs(nFrms);
    std::vector<std::string> imgFileNames(nFrms);
    std::vector<std::pair<int, FrameIndex> > iFrmsSort(nFrms);
    FrameIndexList iFrmsOriToRe(nFrms), iFrmsReToOri(nFrms);
    SequenceTag tag(GetDirectory());
    for (iFrm = 0; iFrm < nFrms; ++iFrm) {
        fscanf(fp, "%s %f %lf %lf %lf %lf %f %f %f %f %f", buf, &f, &q.v3(), &q.v0(),
               &q.v1(), &q.v2(), &center.X(), &center.Y(), &center.Z(), &d, &t);
        Krs[iFrm].Set(f, d / (f * f));
        imgFileNames[iFrm] = GetDirectory() + buf;
        //Camera &C = m_Cs[iFrm];
        Camera &C = Cs[iFrm];
        q.ToRotationMatrixTranspose(C);
        C.SetCenter(center);
        iFrmsSort[iFrm] = std::make_pair(IO::ExtractFileNumber(imgFileNames[iFrm]),
                                         iFrm);
    }
    //m_tag.Set(imgFileNames);
    tag.Set(imgFileNames);
    if (m_intrinsicType == INTRINSIC_VARIABLE) {
        cx = (tag.GetImageWidth() - 1) * 0.5f;
        cy = (tag.GetImageHeight() - 1) * 0.5f;
        //m_K.Set(1.0f, 1.0f, cx, cy);
        std::vector<float> fs(nFrms);
        for (iFrm = 0; iFrm < nFrms; ++iFrm) {
            fs[iFrm] = Krs[iFrm].f();
        }
        const FrameIndex ith = FrameIndex(nFrms >> 1);
        std::nth_element(fs.begin(), fs.begin() + ith, fs.end());
        const float fMed = fs[ith];
        m_K.Set(fMed, fMed, cx, cy);
        const float s = 1 / fMed;
        for (iFrm = 0; iFrm < nFrms; ++iFrm) {
            Krs[iFrm].Scale(s);
        }
    }
    std::sort(iFrmsSort.begin(), iFrmsSort.end());
    iFrmsOriToRe.resize(nFrms);
    iFrmsReToOri.resize(nFrms);
    m_Cs.Resize(nFrms);
    if (m_intrinsicType == INTRINSIC_VARIABLE) {
        m_Krs.Resize(nFrms);
    }
    for (iFrm = 0; iFrm < nFrms; ++iFrm) {
        const FrameIndex iFrmOri = iFrmsSort[iFrm].second;
        iFrmsOriToRe[iFrmOri] = iFrm;
        iFrmsReToOri[iFrm] = iFrmOri;
        m_Cs[iFrm] = Cs[iFrmOri];
        if (m_intrinsicType == INTRINSIC_VARIABLE) {
            m_Krs[iFrm] = Krs[iFrmOri];
        }
    }
    const int step = iFrmsSort[1].first - iFrmsSort[0].first;
    for (iFrm = 1; iFrm < nFrms &&
            iFrmsSort[iFrm].first - iFrmsSort[iFrm - 1].first == step; ++iFrm);
    if (iFrm == nFrms) {
        m_tag.Set(GetDirectory(),
                  IO::RemovePrefix(tag.GetImageFileName(iFrmsReToOri[0]), GetDirectory()),
                  0, step, iFrmsSort.back().first - iFrmsSort.front().first);
    } else {
        tag.GetSubSequence(iFrmsReToOri, m_tag);
    }

    fscanf(fp, "%d", &int1);
    const TrackIndex nTrks = TrackIndex(int1);
    m_Xs.Resize(nTrks);
    m_descs.Resize(0);
    m_mapFrmToMea.resize(nFrms + 1);
    m_mapTrkToMea.assign(nTrks, MeasurementIndexList());
    //m_mapTrkToPlane.assign(nTrks, INVALID_PLANE_INDEX);
    m_frmStates.assign(nFrms,
                       FLAG_FRAME_STATE_SOLVED/* | FLAG_FRAME_STATE_KEY_FRAME*/);
    m_frmStates.front() = m_frmStates.back() |= FLAG_FRAME_STATE_KEY_FRAME;
    m_trkStates.assign(nTrks, FLAG_TRACK_STATE_SOLVED | FLAG_TRACK_STATE_INLIER);
    m_trkClrs.resize(nTrks);

    int int4[4];
    FrameMeasurementMap frmFtrCnts(nFrms, 0);
    std::vector<Point2D> xs;
    MeasurementFrameMap mapMea2Frm;
    MeasurementTrackMap mapMea2Trk;
    MeasurementStateList meaStates;
    Point2D x;
    const Point2D dx(cx, cy);
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        Point3D &X = m_Xs[iTrk];
        CVD::Rgb<ubyte> &clr = m_trkClrs[iTrk];
        fscanf(fp, "%f %f %f %d %d %d %d", &X.X(), &X.Y(), &X.Z(), &int4[0], &int4[1],
               &int4[2], &int4[3]);
        X.reserve() = 1.0f;
        clr.red = ubyte(int4[0]);
        clr.green = ubyte(int4[1]);
        clr.blue = ubyte(int4[2]);
        const FrameIndex nCrsps = FrameIndex(int4[3]);
        for (FrameIndex i = 0; i < nCrsps; ++i) {
            fscanf(fp, "%d", &int1);
            //iFrm = FrameIndex(int1);
            iFrm = iFrmsOriToRe[int1];
            ++frmFtrCnts[iFrm];
            fscanf(fp, "%d", &int1);
            x.Load(fp);
            x += dx;
            xs.push_back(x);
            mapMea2Frm.push_back(iFrm);
            mapMea2Trk.push_back(iTrk);
            meaStates.push_back(FLAG_MEASUREMENT_STATE_DEFAULT);
        }
    }

    // Reorder measurements from TRACK-ORDER to FRAME_ORDER
    FrameMeasurementMap &frmMeaIdxs = frmFtrCnts;
    m_mapFrmToMea[0] = 0;
    for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        m_mapFrmToMea[iFrm + 1] = m_mapFrmToMea[iFrm] + frmFtrCnts[iFrm];
        frmMeaIdxs[iFrm] = m_mapFrmToMea[iFrm];
    }
    const MeasurementIndex nMeas = MeasurementIndex(xs.size());
    m_xs.Resize(nMeas);
    m_mapMeaToFrm.resize(nMeas);
    m_mapMeaToTrk.resize(nMeas);
    m_meaStates.resize(nMeas);
    for (MeasurementIndex iMea = 0; iMea < nMeas; ++iMea) {
        const FrameIndex iFrm = mapMea2Frm[iMea];
        const TrackIndex iTrk = mapMea2Trk[iMea];
        const MeasurementIndex iMeaRe = frmMeaIdxs[iFrm]++;
        m_xs[iMeaRe] = xs[iMea];
        m_mapMeaToFrm[iMeaRe] = iFrm;
        m_mapMeaToTrk[iMeaRe] = iTrk;
        m_meaStates[iMeaRe] = meaStates[iMea];
        m_mapTrkToMea[iTrk].push_back(iMeaRe);
    }
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        std::sort(m_mapTrkToMea[iTrk].begin(), m_mapTrkToMea[iTrk].end());
    }
    m_measNormalized = false;
}

bool Sequence::SaveVsfm(const char *fileName, const bool saveNvm,
                        const bool saveSift) const {
    if (saveNvm) {
        SaveNvm((IO::RemoveFileExtension(fileName) + ".nvm").c_str());
    }


    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if (fp == nullptr) {
        return false;
    }

    FeatureMatchList matches;
    const FrameIndex nFrms = GetFramesNumber();
    //const uint nPairs = (uint(nFrms) * uint(nFrms) - uint(nFrms)) / 2;
    //uint iPair = 0;
    for (FrameIndex iFrm1 = 0; iFrm1 < nFrms; ++iFrm1) {
        printf("\rSaving matches...%d%%", iFrm1 * 100 / nFrms);
        for (FrameIndex iFrm2 = iFrm1 + 1; iFrm2 < nFrms; ++iFrm2/*, ++iPair*/) {
            //printf("\rSaving matches...%d%%", iPair * 100 / nPairs);
            SearchForFrameFeatureMatches(iFrm1, iFrm2, matches);
            const ushort nMatches = ushort(matches.size());
            if (nMatches == 0) {
                continue;
            }
            fprintf(fp, "%s\n%s\n%d\n", IO::RemovePrefix(m_tag.GetImageFileName(iFrm1),
                    GetDirectory()).c_str(),
                    IO::RemovePrefix(m_tag.GetImageFileName(iFrm2), GetDirectory()).c_str(),
                    nMatches);
            for (ushort i = 0; i < nMatches; ++i) {
                fprintf(fp, "%d ", matches[i].GetIndex1());
            }
            fprintf(fp, "\n");
            for (ushort i = 0; i < nMatches; ++i) {
                fprintf(fp, "%d ", matches[i].GetIndex2());
            }
            fprintf(fp, "\n\n");
        }
    }
    printf("\rSaved \'%s\'\n", (GetDirectory() + fileName).c_str());
    fclose(fp);

    if (!saveSift) {
        return true;
    }

    TrackIndex iTrk;
    MeasurementIndex iMea;

    AlignedVector<Point2D> xs;
    if (m_measNormalized) {
        m_K.NormalizedPlaneToImageN(m_xs, xs);
    } else {
        xs = m_xs;
    }

    std::string siftFileName;
    int int5[5] = {'S' + ('I' << 8) + ('F' << 16) + ('T' << 24), 'V' + ('5' << 8) + ('.' << 16) + ('0' << 24), -1, 5, 128};
    int &nFtrs = int5[2];
    float flt5[5] = {-1, -1, -1, 0, 0};
    ubyte *clr = (ubyte *) &flt5[2];
    clr[3] = 1;
    Descriptor128ub desc;

    for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        siftFileName = IO::RemoveFileExtension(m_tag.GetImageFileName(iFrm)) + ".sift";
        FILE *fp = fopen( siftFileName.c_str(), "wb");
        const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm],
                               iMea2 = m_mapFrmToMea[iFrm + 1];
        nFtrs = int(iMea2 - iMea1);
        fwrite(int5, sizeof(int), 5, fp);

        for (iMea = iMea1; iMea < iMea2; ++iMea) {
            flt5[0] = xs[iMea].x();
            flt5[1] = xs[iMea].y();
            if (m_trkClrs.empty() || (iTrk = m_mapMeaToTrk[iMea]) == INVALID_TRACK_INDEX) {
                clr[0] = clr[1] = clr[2] = 0;
            } else {
                memcpy(clr, &m_trkClrs[iTrk], 3);
            }
            fwrite(flt5, sizeof(float), 5, fp);
        }
        for (iMea = iMea1; iMea < iMea2; ++iMea) {
#if DESCRIPTOR_TRACK
            if (m_descs.Empty() || (iTrk = m_mapMeaToTrk[iMea]) == INVALID_TRACK_INDEX) {
                desc.SetZero();
            } else {
                m_descs[iTrk].ConvertTo(desc);
            }
#else
            if (m_descs.Empty()) {
                desc.SetZero();
            } else {
                m_descs[iMea].ConvertTo(desc);
            }
#endif
            fwrite(desc, sizeof(ubyte), 128, fp);
        }
        fclose(fp);

        printf("\rSaved \'%s\'", siftFileName.c_str());
    }
    printf("\n");

    return true;
}

bool Sequence::LoadVsfm(const char *fileName) {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "r");
    if (fp == nullptr) {
        return false;
    }
    char line[MAX_LINE_LENGTH], buf[MAX_LINE_LENGTH];
    std::map<std::string, FrameIndex> map;
    std::map<std::string, FrameIndex>::const_iterator it;
    FrameIndex iFrm1, iFrm2;
    std::string imgFileName1, imgFileName2;
    FrameIndexPairList iFrmPairList;
    std::vector<FeatureMatchList> matchesList;
    int int1;

    while (fgets(line, MAX_LINE_LENGTH, fp)) {
        if (line[0] == '#' || line[0] == ' ' || line[0] == 10) {
            continue;
        }
        sscanf(line, "%s", buf);
        imgFileName1 = IO::RemovePrefix(buf, GetDirectory());
        fscanf(fp, "%s", buf);
        imgFileName2 = IO::RemovePrefix(buf, GetDirectory());

        if ((it = map.find(imgFileName1)) == map.end()) {
            iFrm1 = FrameIndex(map.size());
            map.insert(std::map<std::string, FrameIndex>::value_type(imgFileName1, iFrm1));
        } else {
            iFrm1 = it->second;
        }
        if ((it = map.find(imgFileName2)) == map.end()) {
            iFrm2 = FrameIndex(map.size());
            map.insert(std::map<std::string, FrameIndex>::value_type(imgFileName2, iFrm2));
        } else {
            iFrm2 = it->second;
        }
        iFrmPairList.push_back(FrameIndexPair(iFrm1, iFrm2));
        fscanf(fp, "%d", &int1);
        const ushort nMatches = ushort(int1);
        matchesList.push_back(FeatureMatchList(nMatches));
        FeatureMatchList &matches = matchesList.back();
        for (ushort i = 0; i < nMatches; ++i) {
            fscanf(fp, "%d", &int1);
            matches[i].SetIndex1(FeatureIndex(int1));
        }
        for (ushort i = 0; i < nMatches; ++i) {
            fscanf(fp, "%d", &int1);
            matches[i].SetIndex2(FeatureIndex(int1));
        }
    }

    const FrameIndex nFrms = FrameIndex(map.size());
    std::vector<std::string> imgFileNames(nFrms);
    FrameIndexList iFrmsOriToRe(nFrms);
    FrameIndex iFrm;
    for (it = map.begin(), iFrm = 0; it != map.end(); ++it, ++iFrm) {
        imgFileNames[iFrm] = GetDirectory() + it->first;
        iFrmsOriToRe[it->second] = iFrm;
    }

    Table<FeatureMatchList *, FrameIndex> pMatchesTable(nFrms, nFrms);
    pMatchesTable.SetZero();
    const uint nPairs = uint(matchesList.size());
    FeatureIndex tmp;
    for (uint iPair = 0; iPair < nPairs; ++iPair) {
        iFrmPairList[iPair].Get(iFrm1, iFrm2);
        iFrm1 = iFrmsOriToRe[iFrm1];
        iFrm2 = iFrmsOriToRe[iFrm2];
        if (iFrm1 < iFrm2) {
            pMatchesTable[iFrm1][iFrm2] = &matchesList[iPair];
        } else {
            FeatureMatchList &matches = matchesList[iPair];
            const ushort nMatches = ushort(matches.size());
            for (ushort i = 0; i < nMatches; ++i) {
                matches[i].Invert(tmp);
            }
            pMatchesTable[iFrm2][iFrm1] = &matches;
        }
    }

    std::string siftFileName;
    int int5[5];
    const int &nFtrs = int5[2];
    float flt5[5];
    const ubyte *clr = (ubyte *) &flt5[2];
    Descriptor128ub desc;
    AlignedVector<Point2D> ftrs;
    std::vector<CVD::Rgb<ubyte> > clrs;
#if DESCRIPTOR_TRACK == 0
    AlignedVector<Descriptor> descs;
#else
    std::vector<AlignedVector<Descriptor> > descsList(nFrms);
#endif
    TrackIndex iTrk1, iTrk2, iTrk;
    FeatureIndex iFtr1, iFtr2;
    std::vector<bool> marks;

    SetTag(imgFileNames);
    for (iFrm2 = 0; iFrm2 < nFrms; ++iFrm2) {
        siftFileName = IO::RemoveFileExtension(m_tag.GetImageFileName(iFrm2)) + ".sift";
        FILE *fp = fopen( siftFileName.c_str(), "rb");
        fread(int5, sizeof(int), 5, fp);
        ftrs.Resize(nFtrs);
        clrs.resize(nFtrs);
        for (int iFtr = 0; iFtr < nFtrs; ++iFtr) {
            fread(flt5, sizeof(float), 5, fp);
            ftrs[iFtr].Set(flt5[0], flt5[1]);
            clrs[iFtr].red = clr[0];
            clrs[iFtr].green = clr[1];
            clrs[iFtr].blue = clr[2];
        }
#if DESCRIPTOR_TRACK == 0
        descs.Resize(nFtrs);
#else
        AlignedVector<Descriptor> &descs2 = descsList[iFrm2];
        descs2.Resize(nFtrs);
#endif
        for (int iFtr = 0; iFtr < nFtrs; ++iFtr) {
            fread(desc, sizeof(ubyte), 128, fp);
#if DESCRIPTOR_TRACK == 0
            descs[iFtr].ConvertFrom(desc);
#else
            descs2[iFtr].ConvertFrom(desc);
#endif
        }
        fclose(fp);

#if DESCRIPTOR_TRACK == 0
        PushBackFrame(ftrs.Data(), descs.Data(), nFtrs, nFtrs, false);
#endif
        const TrackIndex *iTrks2 = GetFrameTrackIndexes(iFrm2);
        for (iFrm1 = 0; iFrm1 < iFrm2; ++iFrm1) {
            if (!pMatchesTable[iFrm1][iFrm2]) {
                continue;
            }
#if DESCRIPTOR_TRACK
            const AlignedVector<Descriptor> &descs1 = descsList[iFrm1];
#endif
            const TrackIndex *iTrks1 = GetFrameTrackIndexes(iFrm1);
            FeatureMatchList &matches = *pMatchesTable[iFrm1][iFrm2];
            const ushort nMatches = ushort(matches.size());
            for (ushort i = 0; i < nMatches; ++i) {
                matches[i].Get(iFtr1, iFtr2);
                iTrk1 = iTrks1[iFtr1];
                iTrk2 = iTrks2[iFtr2];
#if DESCRIPTOR_TRACK == 0
                if (iTrk1 == INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX) {
                    PushBackTrack(iFrm1, iFtr1, iFrm2, iFtr2);
                } else if (iTrk1 != INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX &&
                           SearchTrackForFrameMeasurementIndex(iTrk1,
                                   iFrm2) == INVALID_MEASUREMENT_INDEX) {
                    MatchTrackAndFrameFeature(iTrk1, iFrm2, iFtr2);
                } else if (iTrk1 == INVALID_TRACK_INDEX && iTrk2 != INVALID_TRACK_INDEX &&
                           SearchTrackForFrameMeasurementIndex(iTrk2,
                                   iFrm1) == INVALID_MEASUREMENT_INDEX) {
                    MatchTrackAndFrameFeature(iTrk2, iFrm1, iFtr1);
                }
#else
                if (iTrk1 == INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX) {
                    PushBackTrack(iFrm1, iFtr1, iFrm2, iFtr2, descs1[iFtr1], descs2[iFtr2]);
                } else if (iTrk1 != INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX &&
                           SearchTrackForFrameMeasurementIndex(iTrk1,
                                   iFrm2) == INVALID_MEASUREMENT_INDEX) {
                    MatchTrackAndFrameFeature(iTrk1, iFrm2, iFtr2, descs2[iFtr2]);
                } else if (iTrk1 == INVALID_TRACK_INDEX && iTrk2 != INVALID_TRACK_INDEX &&
                           SearchTrackForFrameMeasurementIndex(iTrk2,
                                   iFrm1) == INVALID_MEASUREMENT_INDEX) {
                    MatchTrackAndFrameFeature(iTrk2, iFrm1, iFtr1, descs1[iFtr1]);
                }
#endif
                else if (iTrk1 != INVALID_TRACK_INDEX && iTrk2 != INVALID_TRACK_INDEX &&
                         iTrk1 != iTrk2 && !AreTracksOverlappingInFrames(iTrk1, iTrk2, marks)) {
                    MatchTracks(iTrk1, iTrk2);
                }
            }
        }

        const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm2];
        const TrackIndex nTrksLast = GetTrackColorsNumber();
        const TrackIndex nTrksCurrent = GetTracksNumber();
        SetTrackColorsNumber(nTrksCurrent);
        for (iTrk = nTrksLast; iTrk < nTrksCurrent; ++iTrk) {
            if (GetTrackLength(iTrk) != 0)
                SetTrackColor(iTrk, clrs[GetTrackMeasurementIndexList(iTrk).back() -
                                                                                   iMeaStart]);
        }
    }
    RemoveNullMeasurements();
    RemoveBrokenTracks();
    m_measNormalized = false;
    //if(normalizeMeas)
    //  NormalizeMeasurements();

    AssertConsistency();
    printf("Loaded \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::LoadVsfmF(const char *fileName) {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "r");
    if (fp == nullptr) {
        return false;
    }

    char line[MAX_LINE_LENGTH], buf[MAX_LINE_LENGTH];
    std::map<std::string, FrameIndex> map;
    std::map<std::string, FrameIndex>::const_iterator it;
    FrameIndex iFrm1, iFrm2;
    std::string imgFileName1, imgFileName2;
    FrameIndexPairList iFrmPairList;
    std::vector<FeatureMatchList> matchesList;
    int int1, int2[2];
    float flt4[4];

    while (fgets(line, MAX_LINE_LENGTH, fp)) {
        if (line[0] == '#' || line[0] == ' ' || line[0] == 10) {
            continue;
        }
        sscanf(line, "%d %s", &int1, buf);
        imgFileName1 = IO::RemovePrefix(buf, GetDirectory());
        fscanf(fp, "%d %s", &int1, buf);
        imgFileName2 = IO::RemovePrefix(buf, GetDirectory());

        if ((it = map.find(imgFileName1)) == map.end()) {
            iFrm1 = FrameIndex(map.size());
            map.insert(std::map<std::string, FrameIndex>::value_type(imgFileName1, iFrm1));
        } else {
            iFrm1 = it->second;
        }
        if ((it = map.find(imgFileName2)) == map.end()) {
            iFrm2 = FrameIndex(map.size());
            map.insert(std::map<std::string, FrameIndex>::value_type(imgFileName2, iFrm2));
        } else {
            iFrm2 = it->second;
        }
        iFrmPairList.push_back(FrameIndexPair(iFrm1, iFrm2));
        fscanf(fp, "%d", &int1);
        const ushort nMatches = ushort(int1);
        matchesList.push_back(FeatureMatchList(nMatches));
        FeatureMatchList &matches = matchesList.back();
        for (ushort i = 0; i < nMatches; ++i) {
            fscanf(fp, "%d %f %f %d %f %f", &int2[0], &flt4[0], &flt4[1], &int2[1],
                   &flt4[2], &flt4[3]);
            matches[i].Set(FeatureIndex(int2[0]), FeatureIndex(int2[1]));
        }
    }

    const FrameIndex nFrms = FrameIndex(map.size());
    std::vector<std::string> imgFileNames(nFrms);
    FrameIndexList iFrmsOriToRe(nFrms);
    FrameIndex iFrm;
    for (it = map.begin(), iFrm = 0; it != map.end(); ++it, ++iFrm) {
        imgFileNames[iFrm] = it->first;
        iFrmsOriToRe[it->second] = iFrm;
    }

    Table<FeatureMatchList *, FrameIndex> pMatchesTable(nFrms, nFrms);
    pMatchesTable.SetZero();
    const uint nPairs = uint(matchesList.size());
    FeatureIndex tmp;
    for (uint iPair = 0; iPair < nPairs; ++iPair) {
        iFrmPairList[iPair].Get(iFrm1, iFrm2);
        iFrm1 = iFrmsOriToRe[iFrm1];
        iFrm2 = iFrmsOriToRe[iFrm2];
        if (iFrm1 < iFrm2) {
            pMatchesTable[iFrm1][iFrm2] = &matchesList[iPair];
        } else {
            FeatureMatchList &matches = matchesList[iPair];
            const ushort nMatches = ushort(matches.size());
            for (ushort i = 0; i < nMatches; ++i) {
                matches[i].Invert(tmp);
            }
            pMatchesTable[iFrm2][iFrm1] = &matches;
        }
    }

    std::string siftFileName;
    int int5[5];
    const int &nFtrs = int5[2];
    float flt5[5];
    const ubyte *clr = (ubyte *) &flt5[2];
    Descriptor128ub desc;
    AlignedVector<Point2D> ftrs;
    std::vector<CVD::Rgb<ubyte> > clrs;
#if DESCRIPTOR_TRACK == 0
    AlignedVector<Descriptor> descs;
#else
    std::vector<AlignedVector<Descriptor> > descsList(nFrms);
#endif
    TrackIndex iTrk1, iTrk2, iTrk;
    FeatureIndex iFtr1, iFtr2;
    std::vector<bool> marks;

    SetTag(imgFileNames);
    for (iFrm2 = 0; iFrm2 < nFrms; ++iFrm2) {
        siftFileName = IO::RemoveFileExtension(m_tag.GetImageFileName(iFrm2)) + ".sift";
        FILE *fp = fopen( siftFileName.c_str(), "rb");
        fread(int5, sizeof(int), 5, fp);
        ftrs.Resize(nFtrs);
        clrs.resize(nFtrs);
        for (int iFtr = 0; iFtr < nFtrs; ++iFtr) {
            fread(flt5, sizeof(float), 5, fp);
            ftrs[iFtr].Set(flt5[0], flt5[1]);
            clrs[iFtr].red = clr[0];
            clrs[iFtr].green = clr[1];
            clrs[iFtr].blue = clr[2];
        }
#if DESCRIPTOR_TRACK == 0
        descs.Resize(nFtrs);
#else
        AlignedVector<Descriptor> &descs2 = descsList[iFrm2];
        descs2.Resize(nFtrs);
#endif
        for (int iFtr = 0; iFtr < nFtrs; ++iFtr) {
            fread(desc, sizeof(ubyte), 128, fp);
#if DESCRIPTOR_TRACK == 0
            descs[iFtr].ConvertFrom(desc);
#else
            descs2[iFtr].ConvertFrom(desc);
#endif
        }
        fclose(fp);

#if DESCRIPTOR_TRACK == 0
        PushBackFrame(ftrs.Data(), descs.Data(), nFtrs, nFtrs, false);
#endif
        const TrackIndex *iTrks2 = GetFrameTrackIndexes(iFrm2);
        for (iFrm1 = 0; iFrm1 < iFrm2; ++iFrm1) {
            if (!pMatchesTable[iFrm1][iFrm2]) {
                continue;
            }
#if DESCRIPTOR_TRACK
            const AlignedVector<Descriptor> &descs1 = descsList[iFrm1];
#endif
            const TrackIndex *iTrks1 = GetFrameTrackIndexes(iFrm1);
            FeatureMatchList &matches = *pMatchesTable[iFrm1][iFrm2];

            //int FC;       //feature count of image2, input
            //int NM;       //number of putative matches, output
            //int NF;       //number of inlier matches
            ////Points<int> matches;// feature index of putative matches, output
            //TwoViewGeometry tvg;
            //Points<int> inliers;
            //MatchFile mat(IO::RemoveFileExtension(m_tag.GetImageFileName(iFrm1)).c_str()); //image1's match database
            //FC = nFtrs;
            //if(mat.IsValid())
            //  //mat.GetMatchCount(IO::RemoveFileExtension(m_tag.GetImageFileName(iFrm2)).c_str(), NM, NF);
            //  //mat.GetPMatch(IO::RemoveFileExtension(m_tag.GetImageFileName(iFrm2)).c_str(), FC, NM, matches);
            //  mat.GetIMatch(IO::RemoveFileExtension(m_tag.GetImageFileName(iFrm2)).c_str(), FC, tvg, inliers);

            const ushort nMatches = ushort(matches.size());
            for (ushort i = 0; i < nMatches; ++i) {
                matches[i].Get(iFtr1, iFtr2);
                iTrk1 = iTrks1[iFtr1];
                iTrk2 = iTrks2[iFtr2];
                //if(i == 762 && nMatches == 763)
                //if(iTrk1 == 798 || iTrk2 == 798 || GetTracksNumber() == 798)
                //printf("(%d, %d), %d/%d, (%d/%d, %d/%d), (%d, %d)\n", iFrm1, iFrm2, i, nMatches, iFtr1, GetFrameFeaturesNumber(iFrm1), iFtr2, GetFrameFeaturesNumber(iFrm2), iTrk1, iTrk2);
#if DESCRIPTOR_TRACK == 0
                if (iTrk1 == INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX) {
                    PushBackTrack(iFrm1, iFtr1, iFrm2, iFtr2);
                } else if (iTrk1 != INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX &&
                           SearchTrackForFrameMeasurementIndex(iTrk1,
                                   iFrm2) == INVALID_MEASUREMENT_INDEX) {
                    MatchTrackAndFrameFeature(iTrk1, iFrm2, iFtr2);
                } else if (iTrk1 == INVALID_TRACK_INDEX && iTrk2 != INVALID_TRACK_INDEX &&
                           SearchTrackForFrameMeasurementIndex(iTrk2,
                                   iFrm1) == INVALID_MEASUREMENT_INDEX) {
                    MatchTrackAndFrameFeature(iTrk2, iFrm1, iFtr1);
                }
#else
                if (iTrk1 == INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX) {
                    PushBackTrack(iFrm1, iFtr1, iFrm2, iFtr2, descs1[iFtr1], descs2[iFtr2]);
                } else if (iTrk1 != INVALID_TRACK_INDEX && iTrk2 == INVALID_TRACK_INDEX &&
                           SearchTrackForFrameMeasurementIndex(iTrk1,
                                   iFrm2) == INVALID_MEASUREMENT_INDEX) {
                    MatchTrackAndFrameFeature(iTrk1, iFrm2, iFtr2, descs2[iFtr2]);
                } else if (iTrk1 == INVALID_TRACK_INDEX && iTrk2 != INVALID_TRACK_INDEX &&
                           SearchTrackForFrameMeasurementIndex(iTrk2,
                                   iFrm1) == INVALID_MEASUREMENT_INDEX) {
                    MatchTrackAndFrameFeature(iTrk2, iFrm1, iFtr1, descs1[iFtr1]);
                }
#endif
                else if (iTrk1 != INVALID_TRACK_INDEX && iTrk2 != INVALID_TRACK_INDEX &&
                         iTrk1 != iTrk2 && !AreTracksOverlappingInFrames(iTrk1, iTrk2, marks)) {
                    MatchTracks(iTrk1, iTrk2);
                }
            }
        }

        const MeasurementIndex iMeaStart = m_mapFrmToMea[iFrm2];
        const TrackIndex nTrksLast = GetTrackColorsNumber();
        const TrackIndex nTrksCurrent = GetTracksNumber();
        SetTrackColorsNumber(nTrksCurrent);
        for (iTrk = nTrksLast; iTrk < nTrksCurrent; ++iTrk) {
            if (GetTrackLength(iTrk) != 0)
                SetTrackColor(iTrk, clrs[GetTrackMeasurementIndexList(iTrk).back() -
                                                                                   iMeaStart]);
        }
    }
    RemoveNullMeasurements();
    RemoveBrokenTracks();
    m_measNormalized = false;
    //if(normalizeMeas)
    //  NormalizeMeasurements();

    AssertConsistency();
    printf("Loaded \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::SaveOut(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if (fp == nullptr) {
        return false;
    }

    AlignedVector<Point2D> xs(m_xs.Size());
    if (m_measNormalized) {
        m_K.NormalizedPlaneToImageN(m_xs, xs);
    } else {
        xs.CopyFrom(m_xs);
    }

    fprintf(fp, "# Bundle file v0.3\n");

    FrameIndex iFrm;
    const FrameIndex nFrms = GetFramesNumber();
    fprintf(fp, "%d %d\n", nFrms, CountTracks(FLAG_TRACK_STATE_INLIER));
    const float f = (m_K.fx() + m_K.fy()) * 0.5f;
    for (iFrm = 0; iFrm < nFrms; ++iFrm) {
        if (m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) {
            switch (m_intrinsicType) {
                case INTRINSIC_USER_FIXED:
                    fprintf(fp, "%f 0 0\n", f);
                    break;
                case INTRINSIC_CONSTANT:
                    fprintf(fp, "%f 0 0\n", f * m_Kr.f());
                    break;
                case Sequence::INTRINSIC_VARIABLE:
                    fprintf(fp, "%f 0 0\n", f * m_Krs[iFrm].f());
                    break;
            }
            const Camera &C = m_Cs[iFrm];
            fprintf(fp, "%f %f %f\n", C.r00(), C.r01(), C.r02());
            fprintf(fp, "%f %f %f\n", -C.r10(), -C.r11(), -C.r12());
            fprintf(fp, "%f %f %f\n", -C.r20(), -C.r21(), -C.r22());
            fprintf(fp, "%f %f %f\n", C.tX(), -C.tY(), -C.tZ());
        } else {
            fprintf(fp, "0 0 0\n");
            fprintf(fp, "0 0 0\n0 0 0\n0 0 0\n0 0 0\n");
        }
    }

    TrackIndex iTrk;
    MeasurementIndex iMea;
    const Point2D dx(-m_K.cx(), -m_K.cy());
    const TrackIndex nTrks = GetTracksNumber();
    for (iTrk = 0; iTrk < nTrks; ++iTrk) {
        if (!(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER)) {
            continue;
        }
        m_Xs[iTrk].Save(fp);
        const CVD::Rgb<ubyte> &clr = m_trkClrs[iTrk];
        fprintf(fp, "%d %d %d\n", clr.red, clr.green, clr.blue);
        const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        fprintf(fp, "%d", nCrsps);
        for (FrameIndex i = 0; i < nCrsps; ++i) {
            iMea = iMeas[i];
            iFrm = m_mapMeaToFrm[iMea];
            Point2D &x = xs[iMea];
            x += dx;
            fprintf(fp, " %d %d %f %f", iFrm, iMea - m_mapFrmToMea[iFrm], x.x(), -x.y());
        }
        fprintf(fp, "\n");
    }

    fclose(fp);
    printf("Saved \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::SaveImageLists(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if (fp == nullptr) {
        return false;
    }
    const FrameIndex nFrms = GetFramesNumberTotal();
    for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
        fprintf(fp, "%s\n", IO::RemovePrefix(GetImageFileName(iFrm),
                                             GetDirectory()).c_str());
    fclose(fp);
    printf("Saved \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::SaveCameras(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if (fp == nullptr) {
        return false;
    }
    m_K.Save(fp);
    const FrameIndex nFrms = FrameIndex(m_Cs.Size());
    fprintf(fp, "%d\n", nFrms);
    for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        fprintf(fp, "%s\n", IO::RemovePrefix(m_tag.GetImageFileName(iFrm),
                                             GetDirectory()).c_str());
        m_Cs[iFrm].Save(fp);
    }
    fclose(fp);
    printf("Saved \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::SavePoints(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "w");
    if (fp == nullptr) {
        return false;
    }
    const TrackIndex nTrks = TrackIndex(m_Xs.Size());
    fprintf(fp, "%d\n", nTrks);
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        //m_Xs[iTrk].Save(fp);
        const Point3D &X = m_Xs[iTrk];
        const CVD::Rgb<ubyte> &clr = m_trkClrs[iTrk];
        fprintf(fp, "%f %f %f %d %d %d\n", X.X(), X.Y(), X.Z(), clr.red, clr.green,
                clr.blue);
    }
    fclose(fp);
    printf("Saved \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::SaveB(const char *fileName) const {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "wb");
    if (fp == nullptr) {
        return false;
    }
    SaveB(fp);
    fclose(fp);
    printf("Saved \'%s\'\n", (GetDirectory() + fileName).c_str());
    return true;
}

bool Sequence::SaveBwithDir(const char *fileName) const {

    FILE *fp = fopen( fileName, "wb");
    if (!fp) {
        return false;
    }
    SaveB(fp);
    fclose(fp);
    printf("Saved \'%s\'\n", fileName);
    return true;
}

void Sequence::SaveB(FILE *fp) const {
    m_tag.SaveB(fp);
    m_K.SaveB(fp);
    m_Kr.SaveB(fp);
    fwrite(&m_intrinsicType, sizeof(IntrinsicType), 1, fp);
    m_Cs.SaveB(fp);
    m_Krs.SaveB(fp);
    m_Ps.SaveB(fp);
    m_Xs.SaveB(fp);
    m_xs.SaveB(fp);
    m_descs.SaveB(fp);
    IO::VectorSaveB(m_mapFrmToMea, fp);
    IO::VectorSetSaveB(m_mapTrkToMea, fp);
    IO::VectorSaveB(m_mapMeaToFrm, fp);
    IO::VectorSaveB(m_mapMeaToTrk, fp);
    IO::VectorSaveB(m_frmStates, fp);
    IO::VectorSaveB(m_trkStates, fp);
    IO::VectorSaveB(m_meaStates, fp);
    IO::VectorSaveB(m_trkClrs, fp);
    fwrite(&m_measNormalized, sizeof(m_measNormalized), 1, fp);
}

bool Sequence::LoadB(const char *fileName) {

    FILE *fp = fopen((GetDirectory() + fileName).c_str(), "rb");
    if (fp == nullptr) {
        return false;
    }
    LoadB(fp);
    fclose(fp);
    printf("Loaded \'%s\'\n", (GetDirectory() + fileName).c_str());
    //SaveB(fileName);
    //exit(0);
    return true;
}

bool Sequence::LoadBwithDir(const char *fileName) {

    FILE *fp = fopen(fileName, "rb");
    if (fp == nullptr) {
        return false;
    }
    LoadB(fp);
    fclose(fp);
    printf("Loaded \'%s\'\n", fileName);
    //SaveB(fileName);
    //exit(0);
    return true;
}

void Sequence::LoadB(FILE *fp) {
    m_tag.LoadB(fp);
    m_K.LoadB(fp);
    m_Kr.LoadB(fp);
    fread(&m_intrinsicType, sizeof(IntrinsicType), 1, fp);
    m_Cs.LoadB(fp);
    m_Krs.LoadB(fp);
    m_Ps.LoadB(fp);
    m_Xs.LoadB(fp);
    m_xs.LoadB(fp);
    m_descs.LoadB(fp);
    IO::VectorLoadB(m_mapFrmToMea, fp);
    IO::VectorSetLoadB(m_mapTrkToMea, fp);
    IO::VectorLoadB(m_mapMeaToFrm, fp);
    IO::VectorLoadB(m_mapMeaToTrk, fp);
    IO::VectorLoadB(m_frmStates, fp);
    IO::VectorLoadB(m_trkStates, fp);
    IO::VectorLoadB(m_meaStates, fp);
    IO::VectorLoadB(m_trkClrs, fp);
    fread(&m_measNormalized, sizeof(m_measNormalized), 1, fp);
}

void Sequence::PrintCamera(const FrameIndex &iFrm) const {
    if (iFrm < GetCamerasNumber()
            && (m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED)) {
        printf("----------------------------------------------------------------\n");
        m_Cs[iFrm].Print();
//#if _DEBUG
//      printf("%f\n", m_Cs[iFrm].Determinant());
//#endif
        if (m_intrinsicType == INTRINSIC_VARIABLE) {
            m_Krs[iFrm].Print();
        }
    }
}

void Sequence::PrintFrameFeature(const FrameIndex &iFrm,
                                 const FeatureIndex &iFtr) const {
    const MeasurementIndex iMea = m_mapFrmToMea[iFrm] + iFtr;
    printf("----------------------------------------------------------------\n");
    printf("Frame %d, Feature %d, Measurement %d", iFrm, iFtr, iMea);
    if (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_ENFT) {
        printf(", ENFT");
    } else {
        printf(", SIFT");
    }
    if (m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) {
        if (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER) {
            printf(", OUTLIER");
        } else {
            printf(", INLIER");
        }
    }
    printf("\n");
    Point2D x;
    if (m_measNormalized) {
        m_K.NormalizedPlaneToImage(m_xs[iMea], x);
    } else {
        x = m_xs[iMea];
    }
    printf("  Feature = (%f, %f)\n", x.x(), x.y());
    const TrackIndex iTrk = m_mapMeaToTrk[iMea];
    if ((m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) &&
            iTrk != INVALID_TRACK_INDEX && (m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED)) {
        Point2D xp;
        m_Cs[iFrm].ProjectToNormalizedPlane(m_Xs[iTrk], xp);
//#if _DEBUG
//      m_Cs[iFrm].Print();
//      m_Xs[iTrk].Print();
//#endif
//#if _DEBUG
//      xp.Print();
//#endif
        if (m_intrinsicType == INTRINSIC_CONSTANT) {
            xp *= m_Kr.f();
        } else if (m_intrinsicType == INTRINSIC_VARIABLE) {
            xp *= m_Krs[iFrm].f();
        }
//#if _DEBUG
//      printf("%f\n", m_Krs[iFrm].f());
//      xp.Print();
//#endif
        m_K.NormalizedPlaneToImage(xp);
        printf("  Projection = (%f, %f)\n", xp.x(), xp.y());
        printf("  Error = %f\n", sqrt(x.SquaredDistance(xp)));
    }
//#if DESCRIPTOR_TRACK == 0
//  if(!m_descs.Empty())
//      m_descs[iMea].Print();
//#endif
}

void Sequence::PrintTrack(const TrackIndex &iTrk) const {
    printf("----------------------------------------------------------------\n");
    printf("Track %d", iTrk);
    if (iTrk < GetPointsNumber() && (m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED)) {
        printf(", (%f, %f, %f)", m_Xs[iTrk].X(), m_Xs[iTrk].Y(), m_Xs[iTrk].Z());
        if (m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) {
            printf(", INLIER");
        } else {
            printf(", OUTLIER");
        }
    }
    if (m_trkStates[iTrk] & FLAG_TRACK_STATE_MERGED) {
        printf(", MERGED");
    }
    if (m_trkStates[iTrk] & FLAG_TRACK_STATE_COMMON) {
        if (m_trkStates[iTrk] & FLAG_TRACK_STATE_COMMON_OUTLIER) {
            printf(", COMMON_OUTLIER");
        } else {
            printf(", COMMON_INLIER");
        }
    }
    printf("\n");
    FrameIndex cntSolved = 0;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for (FrameIndex i = 0; i < nCrsps; ++i) {
        const MeasurementIndex iMea = iMeas[i];
        const FrameIndex iFrm = m_mapMeaToFrm[iMea];
        const FeatureIndex iFtr = FeatureIndex(iMea - m_mapFrmToMea[iFrm]);
        printf("  %d: Frame %d, Feature %d, Measurement %d\n", i + 1, iFrm, iFtr, iMea);
        if (m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED) {
            ++cntSolved;
        }
    }
    if (cntSolved >= 2) {
        FrameIndex iFrm1, iFrm2;
        AlignedVector<Point3D> rayDirs;
        const float angle = ComputePointMaximalRayAngle(iTrk, rayDirs, iFrm1, iFrm2,
                            false);
        printf("  Maximal ray angle: %f between Frame (%d, %d)\n", angle, iFrm1, iFrm2);
    }
}

void Sequence::PrintStates() const {
    printf("----------------------------------------------------------------\n");
    printf("Intrinsic Parameter: (%.2f, %.2f, %.2f, %.2f)", m_K.fx(), m_K.fy(),
           m_K.cx(), m_K.cy());
    if (m_intrinsicType == INTRINSIC_CONSTANT) {
        printf(" (%f %f)\n", m_Kr.f(), m_Kr.d());
    } else {
        printf("\n");
    }
    printf("Frames: Total = %d, Keyframe = %d, Solved = %d\n", GetFramesNumber(),
           CountFrames(FLAG_FRAME_STATE_KEY_FRAME), CountFrames(FLAG_FRAME_STATE_SOLVED));
    printf("Tracks: Total = %d, Solved = %d, Inlier = %d\n", GetTracksNumber(),
           CountTracks(FLAG_TRACK_STATE_SOLVED), CountTracks(FLAG_TRACK_STATE_INLIER));
    printf("Measurements: Total = %d, ENFT = %d, Outlier = %d\n",
           GetMeasurementsNumber(), CountMeasurements(FLAG_MEASUREMENT_STATE_ENFT),
           CountMeasurements(FLAG_MEASUREMENT_STATE_OUTLIER));

    FrameIndex iFrmInlierMin, iFrmInlierRatioMin, iFrmMSEMax,
               iFrmInlierAreaRatioMin;
    FeatureIndex nInliers, nInliersMin;
    MeasurementIndex nInliersTotal;
    float inlierRatio, inlierRatioMin, MSE, MSEMax, SSE, SSETotal, inlierAreaRatio,
          inlierAreaRatioMin;
    FeatureIndexList iFtrs;
    Point2D meanFtrTrked, meanFtrInlier;
    LA::Vector3f covFtrTrked, covFtrInlier;
    iFrmInlierMin = iFrmInlierRatioMin = iFrmMSEMax = iFrmInlierAreaRatioMin =
            INVALID_FRAME_INDEX;
    nInliersMin = INVALID_FEATURE_INDEX;
    inlierRatioMin = inlierAreaRatioMin = FLT_MAX;
    MSEMax = 0;
    nInliersTotal = 0;
    SSETotal = 0;
    const FrameIndex nFrms = GetFramesNumber();
    for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        if (!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED)) {
            continue;
        }
        ComputeFrameInlierRatioAndMSE(iFrm, nInliers, inlierRatio, SSE, MSE);
        nInliersTotal += nInliers;
        SSETotal = SSE + SSETotal;
        if (nInliers < nInliersMin) {
            nInliersMin = nInliers;
            iFrmInlierMin = iFrm;
        }
        if (inlierRatio < inlierRatioMin) {
            inlierRatioMin = inlierRatio;
            iFrmInlierRatioMin = iFrm;
        }
        if (MSE > MSEMax) {
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
        if (inlierAreaRatio < inlierAreaRatioMin) {
            inlierAreaRatioMin = inlierAreaRatio;
            iFrmInlierAreaRatioMin = iFrm;
        }
    }
    if (iFrmInlierMin != INVALID_FRAME_INDEX) {
        printf("  Inliers           >= %d (Frame %d)\n", nInliersMin, iFrmInlierMin);
    }
    if (iFrmInlierRatioMin != INVALID_FRAME_INDEX)
        printf("  Inlier ratio      >= %f (Frame %d)\n", inlierRatioMin,
               iFrmInlierRatioMin);
    if (iFrmInlierAreaRatioMin != INVALID_FRAME_INDEX)
        printf("  Inlier area ratio >= %f (Frame %d)\n", inlierAreaRatioMin,
               iFrmInlierAreaRatioMin);
    if (iFrmMSEMax != INVALID_FRAME_INDEX) {
        printf("  Frame MSE         <= %f (Frame %d)\n", MSEMax, iFrmMSEMax);
    }

    MSEMax = 0;
    TrackIndex iTrkMSEMax = INVALID_TRACK_INDEX;
    const TrackIndex nTrks = GetTracksNumber();
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        if (!(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED) ||
                !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER)) {
            continue;
        }
        ComputeTrackMSE(iTrk, m_Xs[iTrk], MSE);
        if (MSE > MSEMax) {
            MSEMax = MSE;
            iTrkMSEMax = iTrk;
        }
    }
    if (iTrkMSEMax != INVALID_TRACK_INDEX) {
        printf("  Track MSE         <= %f (Track %d)\n", MSEMax, iTrkMSEMax);
    }
    if (nInliersTotal != 0) {
        printf("  MSE = %f\n", SSETotal / nInliersTotal);
    }
}

void Sequence::AssertTrackOrdered(const TrackIndex &iTrk) const {
    MeasurementIndex iMea1, iMea2;
    FrameIndex iFrm1, iFrm2;
    const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
    const FrameIndex nCrsps = FrameIndex(iMeas.size());
    for (FrameIndex i = 1; i < nCrsps; ++i) {
        iMea1 = iMeas[i - 1];
        iFrm1 = m_mapMeaToFrm[iMea1];
        iMea2 = iMeas[i];
        iFrm2 = m_mapMeaToFrm[iMea2];
        IO::Assert(iMea1 < iMea2 &&
                   iFrm1 < iFrm2,
                   "iTrk = %d, i = %d, iMea1 = %d, iMea2 = %d, iFrm1 = %d, iFrm2 = %d\n", iTrk, i,
                   iMea1, iMea2, iFrm1, iFrm2);
    }
}

void Sequence::AssertConsistency() const {
    const FrameIndex nFrms = GetFramesNumber();
    IO::Assert(m_mapFrmToMea.size() == nFrms + 1,
               "m_mapFrmToMea.size() = %d, nFrms = %d\n", m_mapFrmToMea.size(), nFrms);
    const TrackIndex nTrks = GetTracksNumber();
    IO::Assert(m_mapTrkToMea.size() == nTrks,
               "m_mapTrkToMea.size() = %d, nTrks = %d\n", m_mapTrkToMea.size(), nTrks);
    const MeasurementIndex nMeas = GetMeasurementsNumber();
    IO::Assert(m_mapMeaToFrm.size() == nMeas &&
               m_mapMeaToTrk.size() == nMeas,
               "m_mapMeaToFrm.size() = %d, m_mapMeaToTrk.size() = %d, nMeas = %d\n",
               m_mapMeaToFrm.size(), m_mapMeaToTrk.size(), nMeas);
    IO::Assert(m_mapFrmToMea.front() == 0 &&
               m_mapFrmToMea.back() == nMeas,
               "m_mapFrmToMea.front() = %d, m_mapFrmToMea.back() = %d, nMeas = %d\n",
               m_mapFrmToMea.front(), m_mapFrmToMea.back(), nMeas);

    for (FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm) {
        const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm],
                               iMea2 = m_mapFrmToMea[iFrm + 1];
        for (MeasurementIndex iMea = iMea1; iMea < iMea2; ++iMea)
            IO::Assert(m_mapMeaToFrm[iMea] == iFrm,
                       "iFrm = %d, iMea = %d, m_mapMeaToFrm[iMea] = %d\n", iFrm, iMea,
                       m_mapMeaToFrm[iMea]);
    }

    std::vector<bool> meaMarks(nMeas, false);
    for (TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk) {
        AssertTrackOrdered(iTrk);
        const MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
        const FrameIndex nCrsps = FrameIndex(iMeas.size());
        for (FrameIndex i = 0; i < nCrsps; ++i) {
            const MeasurementIndex iMea = iMeas[i];
            IO::Assert(m_mapMeaToTrk[iMea] == iTrk,
                       "iTrk = %d, iMea = %d, m_mapMeaToTrk[iMea] = %d\n", iTrk, iMea,
                       m_mapMeaToTrk[iMea]);
            IO::Assert(!meaMarks[iMea], "Already marked! iMea = %d\n", iMea);
            meaMarks[iMea] = true;
        }
    }
    for (MeasurementIndex iMea = 0; iMea < nMeas; ++iMea)
        IO::Assert(m_mapMeaToTrk[iMea] == INVALID_TRACK_INDEX ||
                   meaMarks[iMea], "Not marked yet! iMea = %d\n", iMea);
}

void Sequence::SaveProjectiveMatrixes(const char *fileName,
                                      const AlignedVector<ProjectiveMatrix> &Ps) {

    FILE *fp = fopen( fileName, "w");
    const uint N = Ps.Size();
    fprintf(fp, "%d\n", N);
    for (uint i = 0; i < N; ++i) {
        Ps[i].Save(fp);
    }
    fclose(fp);
    printf("Saved \'%s\'\n", fileName);
}