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
#include "EssentialMatrixFivePointAlgorithm.h"
#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}

bool EssentialMatrixFivePointAlgorithm::Run(const FiveMatches2D &data,
        AlignedVector<EssentialMatrix> &Es, AlignedVector<ENFT_SSE::__m128> &work) {
    Es.Resize(0);
    if(!ExtractNullspace(data, work))
        return false;
    ExpandCubicConstraints(work);
    if(!ComputeGrobnerBasis(work))
        return false;
    ComputeActionMatrix();
    return ComputeExtrinsicMatrixes(Es, work);
}

bool EssentialMatrixFivePointAlgorithm::Run(const MatchSet2D &data,
        AlignedVector<EssentialMatrix> &Es, AlignedVector<ENFT_SSE::__m128> &work) {
    Es.Resize(0);
    if(!ExtractNullspace(data, work))
        return false;
    ExpandCubicConstraints(work);
    if(!ComputeGrobnerBasis(work))
        return false;
    ComputeActionMatrix();
    return ComputeExtrinsicMatrixes(Es, work);
}

bool EssentialMatrixFivePointAlgorithm::ExtractNullspace(
    const FiveMatches2D &data, AlignedVector<ENFT_SSE::__m128> &work) {
    const ENFT_SSE::__m128 &u1_0123 = data.u1_0123(), &v1_0123 = data.v1_0123(),
                  &u2_0123 = data.u2_0123(), &v2_0123 = data.v2_0123();
    ENFT_SSE::__m128 *QT = (ENFT_SSE::__m128 *) &m_QT9x5;

    // Column 0: u1u2
    QT[0] = ENFT_SSE::_mm_mul_ps(u1_0123, u2_0123);
    QT[1].m128_f32[0] = data.u1_4() * data.u2_4();
    // Column 1: v1u2
    QT[2] = ENFT_SSE::_mm_mul_ps(v1_0123, u2_0123);
    QT[3].m128_f32[0] = data.v1_4() * data.u2_4();
    // Column 2: u2
    QT[4] = u2_0123;
    QT[5].m128_f32[0] = data.u2_4();
    // Column 3: u1v2
    QT[6] = ENFT_SSE::_mm_mul_ps(u1_0123, v2_0123);
    QT[7].m128_f32[0] = data.u1_4() * data.v2_4();
    // Column 4: v1v2
    QT[8] = ENFT_SSE::_mm_mul_ps(v1_0123, v2_0123);
    QT[9].m128_f32[0] = data.v1_4() * data.v2_4();
    // Column 5: v2
    QT[10] = v2_0123;
    QT[11].m128_f32[0] = data.v2_4();
    // Column 6: u1
    QT[12] = u1_0123;
    QT[13].m128_f32[0] = data.u1_4();
    // Column 7: v1
    QT[14] = v1_0123;
    QT[15].m128_f32[0] = data.v1_4();
    // Column 8: 1
    QT[16] = _mm_set1_ps(1);
    QT[17].m128_f32[0] = 1;

    //m_QT9x5.Print();

    char jobu = 'N', jobvt = 'A';
    integer m = 5, n = 9, lda = 8, ldu = 1, ldvt = m_vt.Stride(), lwork = 50, info;
    m_s.Resize(5);
    work.Resize(uint(lwork + 3) >> 2);
    // Make 3 elements offset in each row of m_vt to make sure the last four elements are aligned
    sgesvd_(&jobu, &jobvt, &m, &n, &m_QT9x5.M00(), &lda, m_s.Data(), NULL, &ldu,
            &m_vt[0][3], &ldvt, (float *) work.Data(), &lwork, &info);

    //m_vt.Print();

    return info == 0;
}

bool EssentialMatrixFivePointAlgorithm::ExtractNullspace(const MatchSet2D &data,
        AlignedVector<ENFT_SSE::__m128> &work) {
    const uint N = data.Size();
    m_QT9xX.Resize(N);

    ENFT_SSE::__m128 *QT[9] = {(ENFT_SSE::__m128 *) m_QT9xX[0], (ENFT_SSE::__m128 *) m_QT9xX[1], (ENFT_SSE::__m128 *) m_QT9xX[2], (ENFT_SSE::__m128 *) m_QT9xX[3],
                     (ENFT_SSE::__m128 *) m_QT9xX[4], (ENFT_SSE::__m128 *) m_QT9xX[5], (ENFT_SSE::__m128 *) m_QT9xX[6], (ENFT_SSE::__m128 *) m_QT9xX[7], (ENFT_SSE::__m128 *) m_QT9xX[8]
                    };
    const ushort nPacks = data.GetPacksNumber();
    for(ushort iu1 = 0, iv1 = 1, iu2 = 2, iv2 = 3; iu1 < nPacks;
            iu1 += 4, iv1 += 4, iu2 += 4, iv2 += 4) {
        const ENFT_SSE::__m128 &u1 = data.GetPack(iu1), &v1 = data.GetPack(iv1),
                      &u2 = data.GetPack(iu2), &v2 = data.GetPack(iv2);
        // Column 0: u1u2
        *QT[0] = ENFT_SSE::_mm_mul_ps(u1, u2);
        ++QT[0];
        // Column 1: v1u2
        *QT[1] = ENFT_SSE::_mm_mul_ps(v1, u2);
        ++QT[1];
        // Column 2: u2
        *QT[2] = u2;
        ++QT[2];
        // Column 3: u1v2
        *QT[3] = ENFT_SSE::_mm_mul_ps(u1, v2);
        ++QT[3];
        // Column 4: v1v2
        *QT[4] = ENFT_SSE::_mm_mul_ps(v1, v2);
        ++QT[4];
        // Column 5: v2
        *QT[5] = v2;
        ++QT[5];
        // Column 6: u1
        *QT[6] = u1;
        ++QT[6];
        // Column 7: v1
        *QT[7] = v1;
        ++QT[7];
        // Column 8: 1
        *QT[8] = _mm_set1_ps(1);
        ++QT[8];
    }
    for(ushort iRem = 0, iMatch = nPacks; iRem < data.GetRemindersNumber();
            ++iRem, ++iMatch) {
        const LA::Vector2f &x1 = data.GetReminder1(iRem), &x2 = data.GetReminder2(iRem);
        // Column 0: u1u2
        m_QT9xX[0][iMatch] = x1.v0() * x2.v0();
        // Column 1: v1u2
        m_QT9xX[1][iMatch] = x1.v1() * x2.v0();
        // Column 2: u2
        m_QT9xX[2][iMatch] = x2.v0();
        // Column 3: u1v2
        m_QT9xX[3][iMatch] = x1.v0() * x2.v1();
        // Column 4: v1v2
        m_QT9xX[4][iMatch] = x1.v1() * x2.v1();
        // Column 5: v2
        m_QT9xX[5][iMatch] = x2.v1();
        // Column 6: u1
        m_QT9xX[6][iMatch] = x1.v0();
        // Column 7: v1
        m_QT9xX[7][iMatch] = x1.v1();
        // Column 8: 1
        m_QT9xX[8][iMatch] = 1;
    }

    //m_QT9xX.Print();

    char jobu = 'N', jobvt = 'A';
    integer m = N, n = 9, lda = m_QT9xX.Stride(), ldu = 1, ldvt = m_vt.Stride(),
            lwork = -1, info;
    // Make 3 elements offset in each row of m_vt to make sure the last four elements are aligned
    m_s.Resize(N);
    work.Resize(1);
    sgesvd_(&jobu, &jobvt, &m, &n, m_QT9xX[0], &lda, m_s.Data(), NULL, &ldu,
            &m_vt[0][3], &ldvt, (float *) work.Data(), &lwork, &info);
    lwork = integer(work[0].m128_f32[0]);
    work.Resize(uint(lwork + 3) >> 2);
    sgesvd_(&jobu, &jobvt, &m, &n, m_QT9xX[0], &lda, m_s.Data(), NULL, &ldu,
            &m_vt[0][3], &ldvt, (float *) work.Data(), &lwork, &info);

    //m_vt.Print();

    return info == 0;
}

void EssentialMatrixFivePointAlgorithm::ExpandCubicConstraints(
    AlignedVector<ENFT_SSE::__m128> &work) {
    m_polyE[0][0].Set(PTR_TO_SSE(m_vt[0] + 8));
    m_polyE[0][1].Set(PTR_TO_SSE(m_vt[1] + 8));
    m_polyE[0][2].Set(PTR_TO_SSE(m_vt[2] + 8));
    m_polyE[1][0].Set(PTR_TO_SSE(m_vt[3] + 8));
    m_polyE[1][1].Set(PTR_TO_SSE(m_vt[4] + 8));
    m_polyE[1][2].Set(PTR_TO_SSE(m_vt[5] + 8));
    m_polyE[2][0].Set(PTR_TO_SSE(m_vt[6] + 8));
    m_polyE[2][1].Set(PTR_TO_SSE(m_vt[7] + 8));
    m_polyE[2][2].Set(PTR_TO_SSE(m_vt[8] + 8));

    work.Resize(4);
    Polynomial::ABmCD(m_polyE[0][1], m_polyE[1][2], m_polyE[0][2], m_polyE[1][1],
                      m_polyTmp2[0], work[0]);
    Polynomial::ABmCD(m_polyE[0][2], m_polyE[1][0], m_polyE[0][0], m_polyE[1][2],
                      m_polyTmp2[1], work[0]);
    Polynomial::ABmCD(m_polyE[0][0], m_polyE[1][1], m_polyE[0][1], m_polyE[1][0],
                      m_polyTmp2[2], work[0]);
    Polynomial::ABpCDpEF(m_polyE[2][0], m_polyTmp2[0], m_polyE[2][1], m_polyTmp2[1],
                         m_polyE[2][2], m_polyTmp2[2], m_polyConstraints[9],
                         work[0], work[1], work[2], work[3]);

    Polynomial::ABpCDpEF(m_polyE[0][0], m_polyE[0][0], m_polyE[0][1], m_polyE[0][1],
                         m_polyE[0][2], m_polyE[0][2], m_polyEET[0], work[0]);
    Polynomial::ABpCDpEF(m_polyE[0][0], m_polyE[1][0], m_polyE[0][1], m_polyE[1][1],
                         m_polyE[0][2], m_polyE[1][2], m_polyEET[1], work[0]);
    Polynomial::ABpCDpEF(m_polyE[0][0], m_polyE[2][0], m_polyE[0][1], m_polyE[2][1],
                         m_polyE[0][2], m_polyE[2][2], m_polyEET[2], work[0]);
    Polynomial::ABpCDpEF(m_polyE[1][0], m_polyE[1][0], m_polyE[1][1], m_polyE[1][1],
                         m_polyE[1][2], m_polyE[1][2], m_polyEET[3], work[0]);
    Polynomial::ABpCDpEF(m_polyE[1][0], m_polyE[2][0], m_polyE[1][1], m_polyE[2][1],
                         m_polyE[1][2], m_polyE[2][2], m_polyEET[4], work[0]);
    Polynomial::ABpCDpEF(m_polyE[2][0], m_polyE[2][0], m_polyE[2][1], m_polyE[2][1],
                         m_polyE[2][2], m_polyE[2][2], m_polyEET[5], work[0]);
    Polynomial::ApBpC(m_polyEET[0], m_polyEET[3], m_polyEET[5], m_polyTrace);
    Polynomial::sA(0.5f, m_polyTrace, work[0]);

    Polynomial::V3E2f &polyLambda0 = m_polyTmp2[0], &polyLambda3 = m_polyTmp2[1],
                       &polyLambda5 = m_polyTmp2[2];
    Polynomial::AmB(m_polyEET[0], m_polyTrace, polyLambda0);
    Polynomial::AmB(m_polyEET[3], m_polyTrace, polyLambda3);
    Polynomial::AmB(m_polyEET[5], m_polyTrace, polyLambda5);
    Polynomial::V3E2f &polyLambda1 = m_polyEET[1], &polyLambda2 = m_polyEET[2],
                       &polyLambda4 = m_polyEET[4];

    Polynomial::ABpCDpEF(m_polyE[0][0], polyLambda0, m_polyE[1][0], polyLambda1,
                         m_polyE[2][0], polyLambda2, m_polyConstraints[0],
                         work[0], work[1], work[2], work[3]);
    Polynomial::ABpCDpEF(m_polyE[0][1], polyLambda0, m_polyE[1][1], polyLambda1,
                         m_polyE[2][1], polyLambda2, m_polyConstraints[1],
                         work[0], work[1], work[2], work[3]);
    Polynomial::ABpCDpEF(m_polyE[0][2], polyLambda0, m_polyE[1][2], polyLambda1,
                         m_polyE[2][2], polyLambda2, m_polyConstraints[2],
                         work[0], work[1], work[2], work[3]);
    Polynomial::ABpCDpEF(m_polyE[0][0], polyLambda1, m_polyE[1][0], polyLambda3,
                         m_polyE[2][0], polyLambda4, m_polyConstraints[3],
                         work[0], work[1], work[2], work[3]);
    Polynomial::ABpCDpEF(m_polyE[0][1], polyLambda1, m_polyE[1][1], polyLambda3,
                         m_polyE[2][1], polyLambda4, m_polyConstraints[4],
                         work[0], work[1], work[2], work[3]);
    Polynomial::ABpCDpEF(m_polyE[0][2], polyLambda1, m_polyE[1][2], polyLambda3,
                         m_polyE[2][2], polyLambda4, m_polyConstraints[5],
                         work[0], work[1], work[2], work[3]);
    Polynomial::ABpCDpEF(m_polyE[0][0], polyLambda2, m_polyE[1][0], polyLambda4,
                         m_polyE[2][0], polyLambda5, m_polyConstraints[6],
                         work[0], work[1], work[2], work[3]);
    Polynomial::ABpCDpEF(m_polyE[0][1], polyLambda2, m_polyE[1][1], polyLambda4,
                         m_polyE[2][1], polyLambda5, m_polyConstraints[7],
                         work[0], work[1], work[2], work[3]);
    Polynomial::ABpCDpEF(m_polyE[0][2], polyLambda2, m_polyE[1][2], polyLambda4,
                         m_polyE[2][2], polyLambda5, m_polyConstraints[8],
                         work[0], work[1], work[2], work[3]);
}

bool EssentialMatrixFivePointAlgorithm::ComputeGrobnerBasis(
    AlignedVector<ENFT_SSE::__m128> &work) {
    Polynomial::Permulate(m_polyConstraints[0], m_Gb[0]);
    Polynomial::Permulate(m_polyConstraints[1], m_Gb[1]);
    Polynomial::Permulate(m_polyConstraints[2], m_Gb[2]);
    Polynomial::Permulate(m_polyConstraints[3], m_Gb[3]);
    Polynomial::Permulate(m_polyConstraints[4], m_Gb[4]);
    Polynomial::Permulate(m_polyConstraints[5], m_Gb[5]);
    Polynomial::Permulate(m_polyConstraints[6], m_Gb[6]);
    Polynomial::Permulate(m_polyConstraints[7], m_Gb[7]);
    Polynomial::Permulate(m_polyConstraints[8], m_Gb[8]);
    Polynomial::Permulate(m_polyConstraints[9], m_Gb[9]);
    work.Resize(6);
    return Polynomial::GaussianEliminate(m_Gb, work.Data());
}

void EssentialMatrixFivePointAlgorithm::ComputeActionMatrix() {
    memcpy(m_At[0], &m_Gb[0].v10(), 40);
    memcpy(m_At[1], &m_Gb[1].v10(), 40);
    memcpy(m_At[2], &m_Gb[2].v10(), 40);
    memcpy(m_At[3], &m_Gb[4].v10(), 40);
    memcpy(m_At[4], &m_Gb[5].v10(), 40);
    memcpy(m_At[5], &m_Gb[7].v10(), 40);

    memset(m_At[6], 0, 192);
    m_At[6][0] = m_At[7][1] = m_At[8][3] = m_At[9][6] = -1;
    //g_At.Print();
}

bool EssentialMatrixFivePointAlgorithm::ComputeExtrinsicMatrixes(
    AlignedVector<EssentialMatrix> &Es, AlignedVector<ENFT_SSE::__m128> &work) {
    //g_At.Transpose();
    char jobvl = 'V', jobvr = 'N';
    integer n = 10, lda = m_At.Stride(), ldvl = lda, ldvr = 1, lwork = 40, info;
    // Make 2 elements offset in each row of m_vt to make sure the last four elements are aligned
    work.Resize(uint(lwork + 3) >> 2);
    sgeev_(&jobvl, &jobvr, &n, m_At[0], &lda, m_wr, m_wi, &m_vr[0][2], &ldvl, NULL,
           &ldvr, (float *) work.Data(), &lwork, &info);
    if(info != 0)
        return false;

    const ushort nEs = ushort(m_wi[0] == 0) + ushort(m_wi[1] == 0) + ushort(
                           m_wi[2] == 0) + ushort(m_wi[3] == 0) + ushort(m_wi[4] == 0) +
                       ushort(m_wi[5] == 0) + ushort(m_wi[6] == 0) + ushort(m_wi[7] == 0) + ushort(
                           m_wi[8] == 0) + ushort(m_wi[9] == 0);
    Es.Resize(nEs);
    work.Resize(12);
    ENFT_SSE::__m128 &xyz1 = work[0], *work1 = work.Data() + 1;
    for(ushort i = 0, j = 0; i < 10; ++i) {
        if(m_wi[i] != 0)
            continue;
        xyz1 = ENFT_SSE::_mm_div_ps(VALUE_TO_SSE(m_vr[i][8]), _mm_set1_ps(m_vr[i][11]));

        EssentialMatrix &E = Es[j];
        E.M00() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(xyz1, m_polyE[0][0].cx_cy_cz_c()));
        E.M01() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(xyz1, m_polyE[0][1].cx_cy_cz_c()));
        E.M02() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(xyz1, m_polyE[0][2].cx_cy_cz_c()));
        E.M10() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(xyz1, m_polyE[1][0].cx_cy_cz_c()));
        E.M11() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(xyz1, m_polyE[1][1].cx_cy_cz_c()));
        E.M12() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(xyz1, m_polyE[1][2].cx_cy_cz_c()));
        E.M20() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(xyz1, m_polyE[2][0].cx_cy_cz_c()));
        E.M21() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(xyz1, m_polyE[2][1].cx_cy_cz_c()));
        E.M22() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(xyz1, m_polyE[2][2].cx_cy_cz_c()));
        //E.EnforceUnitLastEntry();
        E.EnforceSingularConstraint(work1);
        ++j;

        //printf("\n");
        //E.Print();
    }

    return true;
}