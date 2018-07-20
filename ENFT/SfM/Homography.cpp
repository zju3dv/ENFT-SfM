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
#include "Homography.h"
#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}

static inline void hv1xv2(const ENFT_SSE::__m128 &v1, const ENFT_SSE::__m128 &v2, ENFT_SSE::__m128 &hv1xv2) {
    hv1xv2.m128_f32[0] = v1.m128_f32[1] * v2.m128_f32[2] - v1.m128_f32[2] *
                         v2.m128_f32[1];
    hv1xv2.m128_f32[1] = v1.m128_f32[2] * v2.m128_f32[0] - v1.m128_f32[0] *
                         v2.m128_f32[2];
    hv1xv2.m128_f32[2] = v1.m128_f32[0] * v2.m128_f32[1] - v1.m128_f32[1] *
                         v2.m128_f32[0];
}

static inline void WxUT(const ENFT_SSE::__m128 &w1, const ENFT_SSE::__m128 &w2, const ENFT_SSE::__m128 &w3,
                        const ENFT_SSE::__m128 &u1, const ENFT_SSE::__m128 &u2, const ENFT_SSE::__m128 &u3,
                        LA::AlignedMatrix3f &WxUT) {
    WxUT.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(
                                         w1.m128_f32[0]), u1), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(w2.m128_f32[0]), u2)),
                                     ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(w3.m128_f32[0]), u3));
    WxUT.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(
                                         w1.m128_f32[1]), u1), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(w2.m128_f32[1]), u2)),
                                     ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(w3.m128_f32[1]), u3));
    WxUT.M_20_21_22_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(
                                         w1.m128_f32[2]), u1), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(w2.m128_f32[2]), u2)),
                                     ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(w3.m128_f32[2]), u3));
}

static inline void HmR_xN(const LA::AlignedMatrix3f &H,
                          const LA::AlignedMatrix3f &R, const LA::AlignedVector3f &N, float &v0,
                          float &v1, float &v2) {
    v0 = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_sub_ps(H.M_00_01_02_x(), R.M_00_01_02_x()),
                                N.v012x()));
    v1 = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_sub_ps(H.M_10_11_12_x(), R.M_10_11_12_x()),
                                N.v012x()));
    v2 = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_sub_ps(H.M_20_21_22_x(), R.M_20_21_22_x()),
                                N.v012x()));
}

static inline void SetColumns(const ENFT_SSE::__m128 &v0, const ENFT_SSE::__m128 &v1,
                              const ENFT_SSE::__m128 &v2, LA::AlignedMatrix3f &M) {
    M.M00() = v0.m128_f32[0];
    M.M01() = v1.m128_f32[0];
    M.M02() = v2.m128_f32[0];
    M.M10() = v0.m128_f32[1];
    M.M11() = v1.m128_f32[1];
    M.M12() = v2.m128_f32[1];
    M.M20() = v0.m128_f32[2];
    M.M21() = v1.m128_f32[2];
    M.M22() = v2.m128_f32[2];
}

static inline void CheckDecomposition(const Homography &H,
                                      const RigidTransformation3D &T, const LA::AlignedVector3f &N) {
    Homography Hchk;
    memcpy(Hchk, T, sizeof(Hchk));
    Hchk.M00() += T.tX() * N.v0();
    Hchk.M01() += T.tX() * N.v1();
    Hchk.M02() += T.tX() * N.v2();
    Hchk.M10() += T.tY() * N.v0();
    Hchk.M11() += T.tY() * N.v1();
    Hchk.M12() += T.tY() * N.v2();
    Hchk.M20() += T.tZ() * N.v0();
    Hchk.M21() += T.tZ() * N.v1();
    Hchk.M22() += T.tZ() * N.v2();

    Homography chk;
    chk.M_00_01_02_x() = ENFT_SSE::_mm_sub_ps(H.M_00_01_02_x(), Hchk.M_00_01_02_x());
    chk.M_10_11_12_x() = ENFT_SSE::_mm_sub_ps(H.M_10_11_12_x(), Hchk.M_10_11_12_x());
    chk.M_20_21_22_x() = ENFT_SSE::_mm_sub_ps(H.M_20_21_22_x(), Hchk.M_20_21_22_x());
    chk.Print();
}

bool Homography::ToRelativePose(const MatchSet2D &xs, RigidTransformation3D &T1,
                                LA::AlignedVector3f &N1, RigidTransformation3D &T2, LA::AlignedVector3f &N2,
                                const float &sccRatioTh, ENFT_SSE::__m128 *work14) {
    ENFT_SSE::__m128 *H = work14;
    float *s = (float *) &work14[3], *work4 = (float *) &work14[4];
    char jobu = 'N', jobvt = 'N';
    integer m = 3, n = 3, lda = 4, ldu = 4, ldvt = 4, lwork = 15, info;
    memcpy(H, this, 48);
    sgesvd_(&jobvt, &jobu, &m, &n, (float *) H, &lda, s, NULL, &ldvt, NULL, &ldu,
            work4, &lwork, &info);
    if(info != 0)
        return false;
    work14[0] = ENFT_SSE::_mm_set1_ps(1 / s[1]);
    Scale(work14[0]);

    GetSSE(H);
    ENFT_SSE::__m128 &dot = work14[10];
    ushort cntPos = 0;
    const ushort nPacks = xs.GetPacksNumber();
    for(ushort i = 0, ip1 = 1, ip2 = 2, ip3 = 3; i < nPacks;
            i += 4, ip1 += 4, ip2 += 4, ip3 += 4) {
        ComputeSign4(H, xs.GetPack(i), xs.GetPack(ip1), xs.GetPack(ip2),
                     xs.GetPack(ip3), dot);
        if(dot.m128_f32[0] > 0)
            ++cntPos;
        if(dot.m128_f32[1] > 0)
            ++cntPos;
        if(dot.m128_f32[2] > 0)
            ++cntPos;
        if(dot.m128_f32[3] > 0)
            ++cntPos;
    }
    for(ushort iRem = 0; iRem < xs.GetRemindersNumber(); ++iRem) {
        if(ComputeSign(xs.GetReminder1(iRem), xs.GetReminder2(iRem)) > 0)
            ++cntPos;
    }
    const ushort N = xs.Size();
    const ushort cntPosTh = ushort(N * sccRatioTh + 0.5f);
    if(cntPos < cntPosTh && (N - cntPos) < cntPosTh)
        return false;
    if(cntPos < cntPosTh) {
        M00() = -M00();
        M01() = -M01();
        M02() = -M02();
        M10() = -M10();
        M11() = -M11();
        M12() = -M12();
        M20() = -M20();
        M21() = -M21();
        M22() = -M22();
    }

    jobvt = 'A';
    ENFT_SSE::__m128 *vt = &work14[8];
    memcpy(H, this, 48);
    sgesvd_(&jobvt, &jobu, &m, &n, (float *) H, &lda, s, (float *) vt, &ldvt, NULL,
            &ldu, work4, &lwork, &info);
    if(info != 0)
        return false;
    const float s2_3 = s[2] * s[2], s2_1 = s[0] * s[0];
    const float t = 1 / sqrt(s2_1 - s2_3);
    const ENFT_SSE::__m128 &v1 = vt[0], &v2 = vt[1], &v3 = vt[2];
    ENFT_SSE::__m128 &t1 = work14[0], &t3 = work14[1];
    t1 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(sqrt(1 - s2_3) * t), v1);
    t3 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(sqrt(s2_1 - 1) * t), v3);
    ENFT_SSE::__m128 &u1 = work14[2], &u2 = work14[3];
    u1 = ENFT_SSE::_mm_add_ps(t1, t3);
    u2 = ENFT_SSE::_mm_sub_ps(t1, t3);
    const ENFT_SSE::__m128 &U1_1 = v2, &U1_2 = u1, &U2_1 = v2, &U2_2 = u2;
    ENFT_SSE::__m128 &U1_3 = work14[4], &U2_3 = work14[5];
    hv1xv2(v2, u1, U1_3);
    hv1xv2(v2, u2, U2_3);

    ENFT_SSE::__m128 &Hv2 = work14[6], &Hu1 = work14[7], &Hu2 = work14[11];
    Apply(v2, Hv2);
    Apply(u1, Hu1);
    Apply(u2, Hu2);

    const ENFT_SSE::__m128 &W1_1 = Hv2, &W1_2 = Hu1, &W2_1 = Hv2, &W2_2 = Hu2;
    ENFT_SSE::__m128 &W1_3 = work14[12], &W2_3 = work14[13];
    hv1xv2(Hv2, Hu1, W1_3);
    hv1xv2(Hv2, Hu2, W2_3);

    WxUT(W1_1, W1_2, W1_3, U1_1, U1_2, U1_3, T1);
    hv1xv2(v2, u1, N1.v012x());
    if(N1.v2() > 0) {
        N1.v0() = -N1.v0();
        N1.v1() = -N1.v1();
        N1.v2() = -N1.v2();
    }
    HmR_xN(*this, T1, N1, T1.tX(), T1.tY(), T1.tZ());

    WxUT(W2_1, W2_2, W2_3, U2_1, U2_2, U2_3, T2);
    hv1xv2(v2, u2, N2.v012x());
    if(N2.v2() > 0) {
        N2.v0() = -N2.v0();
        N2.v1() = -N2.v1();
        N2.v2() = -N2.v2();
    }
    HmR_xN(*this, T2, N2, T2.tX(), T2.tY(), T2.tZ());

//#if _DEBUG
//  CheckDecomposition(*this, T1, N1);
//  CheckDecomposition(*this, T2, N2);
//#endif

    return true;
}