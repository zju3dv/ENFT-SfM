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
#include "ProjectiveMatrix.h"
#undef small
extern "C" {
#include <f2c.h>
#include <clapack.h>
}

bool ProjectiveMatrix::ToIdealIntrinsicExtrinsic(float &f, Camera &C,
        ENFT_SSE::__m128 *work4) const {
    integer m = 3, n = 3, lda = 3, lwork = 3, info;
    float *a = (float *) work4, *tau = a + 9;
    a[0] = M00();
    a[3] = M01();
    a[6] = M02();
    a[1] = M10();
    a[4] = M11();
    a[7] = M12();
    a[2] = M20();
    a[5] = M21();
    a[8] = M22();
    sgerqf_(&m, &n, a, &lda, tau, tau + 3, &lwork, &info);
    if(info != 0)
        return false;
    //printf("\n");
    //printf("%f %f %f\n", a[0], a[1], a[2]);
    //printf("%f %f %f\n", a[3], a[4], a[5]);
    //printf("%f %f %f\n", a[6], a[7], a[8]);
    //float m_K[3][3] = {{a[0], a[3], a[6]}, {0, a[4], a[7]}, {0, 0, a[8]}};

    float &k00 = a[0], &k01 = a[3], &k02 = a[6], &k11 = a[4], &k12 = a[7],
           &k22 = a[8];
    const bool neg = ((k00 < 0) + (k11 < 0) + (k22 < 0)) & 1;
    if(neg) {
        k00 = -k00;
        k01 = -k01;
        k02 = -k02;
        k11 = -k11;
        k12 = -k12;
        k22 = -k12;
    }
    if(k00 < 0)
        k00 = -k00;
    if(k11 < 0) {
        k01 = -k01;
        k11 = -k11;
    }
    if(k22 < 0) {
        k02 = -k02;
        k12 = -k12;
        k22 = -k22;
    }

#if 1
    float &kI00 = a[1], &kI11 = a[2], &kI22 = a[5], &kI00kI11 = tau[0];
    kI00 = 1 / k00;
    kI11 = 1 / k11;
    kI22 = 1 / k22;
    kI00kI11 = kI00 * kI11;
    f = (k00 + k11) * 0.5f * kI22;
    C.r00_r01_r02_tX() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(kI00),
                                    M_00_01_02_03()),
                                    ENFT_SSE::_mm_mul_ps(_mm_set1_ps(-k01 * kI00kI11), M_10_11_12_13())),
                                    ENFT_SSE::_mm_mul_ps(_mm_set1_ps((k01 * k12 - k02 * k11) * kI00kI11 * kI22),
                                            M_20_21_22_23()));
    C.r10_r11_r12_tY() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(kI11), M_10_11_12_13()),
                                    ENFT_SSE::_mm_mul_ps(_mm_set1_ps(-k12 * kI11 * kI22), M_20_21_22_23()));
    C.r20_r21_r22_tZ() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(kI22), M_20_21_22_23());
    if(C.Determinant() < 0) {
        work4[0] = ENFT_SSE::_mm_setzero_ps();
        C.r00_r01_r02_tX() = ENFT_SSE::_mm_sub_ps(work4[0], C.r00_r01_r02_tX());
        C.r10_r11_r12_tY() = ENFT_SSE::_mm_sub_ps(work4[0], C.r10_r11_r12_tY());
        C.r20_r21_r22_tZ() = ENFT_SSE::_mm_sub_ps(work4[0], C.r20_r21_r22_tZ());
    }
#else
    f = (k00 + k11) * 0.5f / k22;
    ENFT_SSE::__m128 &fI = work4[0];
    fI = _mm_set1_ps(1 / f);
    C.r00_r01_r02_tX() = ENFT_SSE::_mm_mul_ps(fI, M_00_01_02_03());
    C.r10_r11_r12_tY() = ENFT_SSE::_mm_mul_ps(fI, M_10_11_12_13());
    C.r20_r21_r22_tZ() = M_20_21_22_23();
#endif
//#if _DEBUG
//  printf("----------------------------------------------------------------\n");
//  printf("%f\n", C.Determinant());
//  printf("%f  ", LA::Dot(C.r00_r01_r02_tX(), C.r00_r01_r02_tX()) - 1);
//  printf("%f  ", LA::Dot(C.r10_r11_r12_tY(), C.r10_r11_r12_tY()) - 1);
//  printf("%f\n", LA::Dot(C.r20_r21_r22_tZ(), C.r20_r21_r22_tZ()) - 1);
//  printf("%f  ", LA::Dot(C.r00_r01_r02_tX(), C.r10_r11_r12_tY()));
//  printf("%f  ", LA::Dot(C.r00_r01_r02_tX(), C.r20_r21_r22_tZ()));
//  printf("%f\n", LA::Dot(C.r10_r11_r12_tY(), C.r20_r21_r22_tZ()));
//  RotationTransformation3D RT = C;
//  RT.Transpose();
//  printf("%f  ", LA::Dot(RT.r_00_01_02_x(), RT.r_00_01_02_x()) - 1);
//  printf("%f  ", LA::Dot(RT.r_10_11_12_x(), RT.r_10_11_12_x()) - 1);
//  printf("%f\n", LA::Dot(RT.r_20_21_22_x(), RT.r_20_21_22_x()) - 1);
//  printf("%f  ", LA::Dot(RT.r_00_01_02_x(), RT.r_10_11_12_x()));
//  printf("%f  ", LA::Dot(RT.r_00_01_02_x(), RT.r_20_21_22_x()));
//  printf("%f\n", LA::Dot(RT.r_10_11_12_x(), RT.r_20_21_22_x()));
//#endif
//#if _DEBUG
//  C.Print();
//#endif
//#if _DEBUG
//  printf("----------------------------------------------------------------\n");
//  Print();
//  ProjectiveMatrix P;
//  P.FromIntrinsicExtrinsic(f, C);
//  const ENFT_SSE::__m128 s = P.M23() == 0.0f ? _mm_set1_ps(1.0f) : _mm_set1_ps(M23() / P.M23());
//  P.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(s, P.M_00_01_02_03());
//  P.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(s, P.M_10_11_12_13());
//  P.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(s, P.M_20_21_22_23());
//  P.Print();
//#endif
    return true;
}