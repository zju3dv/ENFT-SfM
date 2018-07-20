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
#include "EssentialMatrix.h"
#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}

bool EssentialMatrix::EnforceSingularConstraint(ENFT_SSE::__m128 *work11) {
    // s    : work11[0]
    // vt   : work11[1...3]
    // u    : work11[4...6]
    // work4: work11[7...10]
    float *s = (float *) work11, *work4 = (float *) &work11[7];
    ENFT_SSE::__m128 *vt = &work11[1], *u = &work11[4];

    char jobu = 'A', jobvt = 'A';
    integer m = 3, n = 3, lda = 4, ldu = 4, ldvt = 4, lwork = 15, info;
    sgesvd_(&jobu, &jobvt, &m, &n, (float *) this, &lda, s, (float *) vt, &ldu,
            (float *) u, &ldvt, work4, &lwork, &info);
    if(info != 0)
        return false;

    M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[0].m128_f32[0]), vt[0]),
                                ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[0].m128_f32[1]), vt[1]));
    M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[1].m128_f32[0]), vt[0]),
                                ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[1].m128_f32[1]), vt[1]));
    M_20_21_22_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[2].m128_f32[0]), vt[0]),
                                ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[2].m128_f32[1]), vt[1]));

    return true;
}

bool EssentialMatrix::ToRelativePose(const MatchSet2D &xs,
                                     RigidTransformation3D &T, const float &sccRatioTh, ENFT_SSE::__m128 *work38) const {
    // E     : work38[0...2]
    // s     : work38[3]
    // vt    : work38[4...6]
    // u     : work38[7...9]
    // work28: work38[10...37]
    float *E = (float *) work38, *s = (float *) &work38[3];
    ENFT_SSE::__m128 *vt = &work38[4], *u = &work38[7], *work28 = &work38[10];
    memcpy(E, this, 48);

    char jobu = 'A', jobvt = 'A';
    integer m = 3, n = 3, lda = 4, ldu = 4, ldvt = 4, lwork = 15, info;
    sgesvd_(&jobu, &jobvt, &m, &n, E, &lda, s, (float *) vt, &ldu, (float *) u,
            &ldvt, (float *) work28, &lwork, &info);
    if(info != 0)
        return false;

    // Case 1:
    //         [  0 1 0 ]           [ u02 ]
    // R = U * [ -1 0 0 ] * VT, t = [ u12 ]
    //         [  0 0 1 ]           [ u22 ]
    T.M_00_01_02_x() = ENFT_SSE::_mm_sub_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                      u[0].m128_f32[0]), vt[1]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[0].m128_f32[2]), vt[2])),
                                  ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[0].m128_f32[1]), vt[0]));
    T.M_10_11_12_x() = ENFT_SSE::_mm_sub_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                      u[1].m128_f32[0]), vt[1]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[1].m128_f32[2]), vt[2])),
                                  ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[1].m128_f32[1]), vt[0]));
    T.M_20_21_22_x() = ENFT_SSE::_mm_sub_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                      u[2].m128_f32[0]), vt[1]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[2].m128_f32[2]), vt[2])),
                                  ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[2].m128_f32[1]), vt[0]));
    if(T.Determinant() < 0) {
        T.r00() = -T.r00();
        T.r01() = -T.r01();
        T.r02() = -T.r02();
        T.r10() = -T.r10();
        T.r11() = -T.r11();
        T.r12() = -T.r12();
        T.r20() = -T.r20();
        T.r21() = -T.r21();
        T.r22() = -T.r22();
    }
    T.tX() = u[0].m128_f32[2];
    T.tY() = u[1].m128_f32[2];
    T.tZ() = u[2].m128_f32[2];
    ushort cntPos, cntNeg;
    CheckCheirality(*this, T, xs, cntPos, cntNeg, work28);
    const ushort cntPosTh = ushort(xs.Size() * sccRatioTh + 0.5f);
    if(cntPos > cntNeg && cntPos >= cntPosTh)
        return true;

    // Case 2:
    //         [  0 1 0 ]           [ -u02 ]
    // R = U * [ -1 0 0 ] * VT, t = [ -u12 ]
    //         [  0 0 1 ]           [ -u22 ]
    else if(cntPos < cntNeg && cntNeg >= cntPosTh) {
        T.tX() = -u[0].m128_f32[2];
        T.tY() = -u[1].m128_f32[2];
        T.tZ() = -u[2].m128_f32[2];
        return true;
    }

    // Case 3:
    //         [ 0 -1 0 ]           [ u02 ]
    // R = U * [ 1  0 0 ] * VT, t = [ u12 ]
    //         [ 0  0 1 ]           [ u22 ]
    T.M_00_01_02_x() = ENFT_SSE::_mm_sub_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                      u[0].m128_f32[1]), vt[0]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[0].m128_f32[2]), vt[2])),
                                  ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[0].m128_f32[0]), vt[1]));
    T.M_10_11_12_x() = ENFT_SSE::_mm_sub_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                      u[1].m128_f32[1]), vt[0]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[1].m128_f32[2]), vt[2])),
                                  ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[1].m128_f32[0]), vt[1]));
    T.M_20_21_22_x() = ENFT_SSE::_mm_sub_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(
                                      u[2].m128_f32[1]), vt[0]), ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[2].m128_f32[2]), vt[2])),
                                  ENFT_SSE::_mm_mul_ps(_mm_set1_ps(u[2].m128_f32[0]), vt[1]));
    if(T.Determinant() < 0) {
        T.r00() = -T.r00();
        T.r01() = -T.r01();
        T.r02() = -T.r02();
        T.r10() = -T.r10();
        T.r11() = -T.r11();
        T.r12() = -T.r12();
        T.r20() = -T.r20();
        T.r21() = -T.r21();
        T.r22() = -T.r22();
    }
    T.tX() = u[0].m128_f32[2];
    T.tY() = u[1].m128_f32[2];
    T.tZ() = u[2].m128_f32[2];
    CheckCheirality(*this, T, xs, cntPos, cntNeg, work28);
    if(cntPos > cntNeg && cntPos >= cntPosTh)
        return true;

    // Case 4:
    //         [ 0 -1 0 ]           [ -u02 ]
    // R = U * [ 1  0 0 ] * VT, t = [ -u12 ]
    //         [ 0  0 1 ]           [ -u22 ]
    else if(cntPos < cntNeg && cntNeg >= cntPosTh) {
        T.tX() = -u[0].m128_f32[2];
        T.tY() = -u[1].m128_f32[2];
        T.tZ() = -u[2].m128_f32[2];
        return true;
    } else
        return false;
}

void EssentialMatrix::SolveIdealPoint(const EssentialMatrix &E,
                                      const RigidTransformation3D &T, const LA::Vector2f &x1, const LA::Vector2f &x2,
                                      ENFT_SSE::__m128 &X) {
    ENFT_SSE::__m128 &c = X;
    c.m128_f32[0] = -(E.M10() * x1.v0() + E.M11() * x1.v1() + E.M12());
    c.m128_f32[1] = E.M00() * x1.v0() + E.M01() * x1.v1() + E.M02();
    c.m128_f32[2] = -(x2.v0() * c.m128_f32[0] + x2.v1() * c.m128_f32[1]);

    const LA::Vector2f &d = x1;
    ENFT_SSE::__m128 &C = X;
    T.TransposeTimes(c, C);
    X.m128_f32[2] = -1 / (d.v0() * C.m128_f32[0] + d.v1() * C.m128_f32[1] +
                          C.m128_f32[2]);
    X.m128_f32[0] = d.v0() * X.m128_f32[2];
    X.m128_f32[1] = d.v1() * X.m128_f32[2];
    X.m128_f32[3] = 1;
}

void EssentialMatrix::SolveIdealPoint(const EssentialMatrix &E,
                                      const RigidTransformation3D &T, const ENFT_SSE::__m128 &x1x2, ENFT_SSE::__m128 &X) {
    ENFT_SSE::__m128 &c = X;
    c.m128_f32[0] = -(E.M10() * x1x2.m128_f32[0] + E.M11() * x1x2.m128_f32[1] +
                      E.M12());
    c.m128_f32[1] = E.M00() * x1x2.m128_f32[0] + E.M01() * x1x2.m128_f32[1] +
                    E.M02();
    c.m128_f32[2] = -(x1x2.m128_f32[2] * c.m128_f32[0] + x1x2.m128_f32[3] *
                      c.m128_f32[1]);

    //const LA::Vector2f &d = x1;
    ENFT_SSE::__m128 &C = X;
    T.TransposeTimes(c, C);
    X.m128_f32[2] = -1 / (x1x2.m128_f32[0] * C.m128_f32[0] + x1x2.m128_f32[1] *
                          C.m128_f32[1] + C.m128_f32[2]);
    X.m128_f32[0] = x1x2.m128_f32[0] * X.m128_f32[2];
    X.m128_f32[1] = x1x2.m128_f32[1] * X.m128_f32[2];
    X.m128_f32[3] = 1;

    //Point3D _X;
    //const LA::Vector2f x1(x1x2.m128_f32[0], x1x2.m128_f32[1]), x2(x1x2.m128_f32[2], x1x2.m128_f32[3]);
    //SolveIdealPoint(E, T, x1, x2, _X);
    //X = _mm_setr_ps(_X.X(), _X.Y(), _X.Z(), 1);
}

void EssentialMatrix::CheckCheirality(const EssentialMatrix &E,
                                      const RigidTransformation3D &T, const MatchSet2D &xs, ushort &cntPos,
                                      ushort &cntNeg, ENFT_SSE::__m128 *work28) {
    ENFT_SSE::__m128 &e00 = work28[ 0], &e01 = work28[ 1], &e02 = work28[ 2],
            &e10 = work28[ 3], &e11 = work28[ 4], &e12 = work28[ 5];
    ENFT_SSE::__m128 &p00 = work28[ 6], &p10 = work28[ 7], &p20 = work28[ 8],
            &p01 = work28[ 9], &p11 = work28[10], &p21 = work28[11];
    ENFT_SSE::__m128 &p02 = work28[12], &p12 = work28[13], &p22 = work28[14],
            &p03 = work28[15], &p13 = work28[16], &p23 = work28[17];
    ENFT_SSE::__m128 &one = work28[18], &negone = work28[19];
    ENFT_SSE::__m128 &cX = work28[20], &cY = work28[21], &cZ = work28[22], &C0 = work28[23],
            &C1 = work28[24], &C2 = work28[25], &tmp = work28[26];
    ENFT_SSE::__m128 &XX = cX, &XY = cY, &XZ = cZ, &X = work28[27];
    e00 = _mm_set1_ps(E.M00());
    e01 = _mm_set1_ps(E.M01());
    e02 = _mm_set1_ps(E.M02());
    e10 = _mm_set1_ps(E.M10());
    e11 = _mm_set1_ps(E.M11());
    e12 = _mm_set1_ps(E.M12());
    p00 = _mm_set1_ps(T.r00());
    p10 = _mm_set1_ps(T.r10());
    p20 = _mm_set1_ps(T.r20());
    p01 = _mm_set1_ps(T.r01());
    p11 = _mm_set1_ps(T.r11());
    p21 = _mm_set1_ps(T.r21());
    p02 = _mm_set1_ps(T.r02());
    p12 = _mm_set1_ps(T.r12());
    p22 = _mm_set1_ps(T.r22());
    p03 = _mm_set1_ps(T.tX());
    p13 = _mm_set1_ps(T.tY());
    p23 = _mm_set1_ps(T.tZ());
    one = _mm_set1_ps(1);
    negone = _mm_set1_ps(-1);

    bool pos1, pos2;
    cntPos = cntNeg = 0;
    const ushort nPacks = xs.GetPacksNumber();
    for(ushort iPack = 0; iPack < nPacks; iPack += 4) {
        const ENFT_SSE::__m128 &u1 = xs.GetPack(iPack), &v1 = xs.GetPack(iPack + 1),
                      &u2 = xs.GetPack(iPack + 2), &v2 = xs.GetPack(iPack + 3);
        cX = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e10, u1), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e11, v1), e12));
        cY = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e00, u1), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e01, v1), e02));
        cZ = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(u2, cX), ENFT_SSE::_mm_mul_ps(v2, cY));
        tmp = ENFT_SSE::_mm_div_ps(one, ENFT_SSE::_mm_sub_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(p13, cY), ENFT_SSE::_mm_mul_ps(p23,
                                         cZ)), ENFT_SSE::_mm_mul_ps(p03, cX)));
        C0 = ENFT_SSE::_mm_mul_ps(_mm_sub_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(p10, cY), ENFT_SSE::_mm_mul_ps(p20, cZ)),
                                   ENFT_SSE::_mm_mul_ps(p00, cX)), tmp);
        C1 = ENFT_SSE::_mm_mul_ps(_mm_sub_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(p11, cY), ENFT_SSE::_mm_mul_ps(p21, cZ)),
                                   ENFT_SSE::_mm_mul_ps(p01, cX)), tmp);
        C2 = ENFT_SSE::_mm_mul_ps(_mm_sub_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(p12, cY), ENFT_SSE::_mm_mul_ps(p22, cZ)),
                                   ENFT_SSE::_mm_mul_ps(p02, cX)), tmp);
        XZ = ENFT_SSE::_mm_div_ps(negone, ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(u1, C0), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(v1,
                                           C1), C2)));
        XX = ENFT_SSE::_mm_mul_ps(u1, XZ);
        XY = ENFT_SSE::_mm_mul_ps(v1, XZ);

        X = _mm_set_ps(1, XZ.m128_f32[0], XY.m128_f32[0], XX.m128_f32[0]);
        pos1 = X.m128_f32[2] > 0;
        pos2 = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(T.M_20_21_22_x(), X)) > 0;
        if(pos1 && pos2)
            ++cntPos;
        else if(!pos1 && !pos2)
            ++cntNeg;
        X = _mm_set_ps(1, XZ.m128_f32[1], XY.m128_f32[1], XX.m128_f32[1]);
        pos1 = X.m128_f32[2] > 0;
        pos2 = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(T.M_20_21_22_x(), X)) > 0;
        if(pos1 && pos2)
            ++cntPos;
        else if(!pos1 && !pos2)
            ++cntNeg;
        X = _mm_set_ps(1, XZ.m128_f32[2], XY.m128_f32[2], XX.m128_f32[2]);
        pos1 = X.m128_f32[2] > 0;
        pos2 = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(T.M_20_21_22_x(), X)) > 0;
        if(pos1 && pos2)
            ++cntPos;
        else if(!pos1 && !pos2)
            ++cntNeg;
        X = _mm_set_ps(1, XZ.m128_f32[3], XY.m128_f32[3], XX.m128_f32[3]);
        pos1 = X.m128_f32[2] > 0;
        pos2 = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(T.M_20_21_22_x(), X)) > 0;
        if(pos1 && pos2)
            ++cntPos;
        else if(!pos1 && !pos2)
            ++cntNeg;
    }
    for(ushort iRem = 0; iRem < xs.GetRemindersNumber(); ++iRem) {
        SolveIdealPoint(E, T, xs.GetReminder1(iRem), xs.GetReminder2(iRem), X);
        pos1 = X.m128_f32[2] > 0;
        pos2 = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(T.M_20_21_22_x(), X)) > 0;
        if(pos1 && pos2)
            ++cntPos;
        else if(!pos1 && !pos2)
            ++cntNeg;
    }
}

bool EssentialMatrix::CheckCheirality(const EssentialMatrix &E,
                                      const RigidTransformation3D &T, const MatchSet2D &xs, ENFT_SSE::__m128 *work28) {
    ushort cntPos, cntNeg;
    CheckCheirality(E, T, xs, cntPos, cntNeg, work28);
    return cntPos > cntNeg;
}