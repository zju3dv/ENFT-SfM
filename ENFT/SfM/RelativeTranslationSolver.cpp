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
#include "RelativeTranslationSolver.h"
#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}

//static inline void PrintError(const float AT3x2[3][2], const RelativeTranslation &T)
//{
//  float e;
//  for(uint i = 0; i < 2; ++i)
//  {
//      e = AT3x2[0][i] * T.tX() + AT3x2[1][i] * T.tY() + AT3x2[2][i] * T.tZ();
//      printf("%f\n", e);
//  }
//}
//
//static inline void PrintError(const LA::AlignedMatrix3xXf &AT3xX, const RelativeTranslation &T)
//{
//  float e;
//  const uint N = AT3xX.GetColumnsNumber();
//  for(uint i = 0; i < N; ++i)
//  {
//      e = AT3xX[0][i] * T.tX() + AT3xX[1][i] * T.tY() + AT3xX[2][i] * T.tZ();
//      printf("%f ", e);
//  }
//  printf("\n");
//}

bool RelativeTranslationSolver::Run(const TwoMatches3DTo2D &data,
                                    RelativeTranslation &T, AlignedVector<ENFT_SSE::__m128> &work) {
    for(ushort i = 0; i < 2; ++i) {
        const Point3D &Rx1 = data.X(i);
        const Point2D &x2 = data.x(i);
        m_AT3x2[0][i] = Rx1.Y() - Rx1.Z() * x2.y();
        m_AT3x2[1][i] = Rx1.Z() * x2.x() - Rx1.X();
        m_AT3x2[2][i] = Rx1.X() * x2.y() - Rx1.Y() * x2.x();
    }
    //PrintError(m_AT3x2, T);
    char jobu = 'N', jobvt = 'A';
    integer m = 2, n = 3, lda = 2, ldu = 1, ldvt = 3, lwork = 14, info;
    work.Resize(4);
    sgesvd_(&jobu, &jobvt, &m, &n, &m_AT3x2[0][0], &lda, m_s, NULL, &ldu,
            &m_vt[0][0], &ldvt, (float *) work.Data(), &lwork, &info);
    if(info != 0)
        return false;
    T.Set(m_vt[0][2], m_vt[1][2], m_vt[2][2]);
    //Run(data, T, work);
    return true;
}

bool RelativeTranslationSolver::Run(const MatchSet3DTo2DX &data,
                                    RelativeTranslation &T, AlignedVector<ENFT_SSE::__m128> &work) {
    const ushort N = data.Size();
    m_AT3xX.Resize(N);
    for(ushort i = 0; i < N; ++i) {
        const Point3D &Rx1 = data.X(i);
        const Point2D &x2 = data.x(i);
        m_AT3xX[0][i] = Rx1.Y() - Rx1.Z() * x2.y();
        m_AT3xX[1][i] = Rx1.Z() * x2.x() - Rx1.X();
        m_AT3xX[2][i] = Rx1.X() * x2.y() - Rx1.Y() * x2.x();
    }
    //PrintError(m_AT3xX, T);
    char jobu = 'N', jobvt = 'S';
    integer m = N, n = 3, lda = m_AT3xX.Stride(), ldu = 1, ldvt = 3, lwork = -1,
            info;
    work.Resize(1);
    sgesvd_(&jobu, &jobvt, &m, &n, m_AT3xX[0], &lda, m_s, NULL, &ldu, &m_vt[0][0],
            &ldvt, (float *) work.Data(), &lwork, &info);
    lwork = integer(work[0].m128_f32[0]);
    work.Resize(uint(lwork + 3) >> 2);
    sgesvd_(&jobu, &jobvt, &m, &n, m_AT3xX[0], &lda, m_s, NULL, &ldu, &m_vt[0][0],
            &ldvt, (float *) work.Data(), &lwork, &info);
    if(info != 0)
        return false;
    T.Set(m_vt[0][2], m_vt[1][2], m_vt[2][2]);
    //Run(data, T, work);
    return true;
}

