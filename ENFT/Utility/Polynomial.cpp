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
#include "Polynomial.h"

static void PrintN(const uint N, const float *ptr)
{
	for(uint i = 0; i < N; ++i)
		printf(" %.4f", *++ptr);
	printf("\n");
}

static void Print(const float *A_4_10, const float *A_5_10, const float *A_6_10, 
				  const float *A_7_10, const float *A_8_10, const float *A_9_10)
{
	PrintN(10, A_4_10);
	PrintN(10, A_5_10);
	PrintN(10, A_6_10);
	PrintN(10, A_7_10);
	PrintN(10, A_8_10);
	PrintN(10, A_9_10);
}

static void Print(const Polynomial::V3E3fPermutation A[10])
{
	Print(&A[4].v10(), &A[5].v10(), &A[6].v10(), &A[7].v10(), &A[8].v10(), &A[9].v10());
}

bool Polynomial::GaussianEliminate(V3E3fPermutation *A, ENFT_SSE::__m128 *work)
{
	V3E3fPermutation &scaledRow = *(V3E3fPermutation*) (work);
	ENFT_SSE::__m128 &tmp = work[5];

	/***************************************************/
	/* Gaussian elimination
	/***************************************************/

	// i = 0
	if(A[0].v0() == 0)
		return false;
	sA_from_0or1(1 / A[0].v0(), A[0], A[0], tmp);
	// i = 0, j = 1
	sA_from_0or1(A[1].v0(), A[0], scaledRow, tmp);	AmB_from_0or1(A[1], scaledRow, A[1]);
	// i = 0, j = 2
	sA_from_0or1(A[2].v0(), A[0], scaledRow, tmp);	AmB_from_0or1(A[2], scaledRow, A[2]);
	// i = 0, j = 3
	sA_from_0or1(A[3].v0(), A[0], scaledRow, tmp);	AmB_from_0or1(A[3], scaledRow, A[3]);
	// i = 0, j = 4
	sA_from_0or1(A[4].v0(), A[0], scaledRow, tmp);	AmB_from_0or1(A[4], scaledRow, A[4]);
	// i = 0, j = 5
	sA_from_0or1(A[5].v0(), A[0], scaledRow, tmp);	AmB_from_0or1(A[5], scaledRow, A[5]);
	// i = 0, j = 6
	sA_from_0or1(A[6].v0(), A[0], scaledRow, tmp);	AmB_from_0or1(A[6], scaledRow, A[6]);
	// i = 0, j = 7
	sA_from_0or1(A[7].v0(), A[0], scaledRow, tmp);	AmB_from_0or1(A[7], scaledRow, A[7]);
	// i = 0, j = 8
	sA_from_0or1(A[8].v0(), A[0], scaledRow, tmp);	AmB_from_0or1(A[8], scaledRow, A[8]);
	// i = 0, j = 9
	sA_from_0or1(A[9].v0(), A[0], scaledRow, tmp);	AmB_from_0or1(A[9], scaledRow, A[9]);

	// i = 1
	if(A[1].v1() == 0)
		return false;
	sA_from_2(1 / A[1].v1(), A[1], A[1], tmp);		/*A[1].v1() = 1;	A[1].v0() = 0;*/
	// i = 1, j = 2
	sA_from_2(A[2].v1(), A[1], scaledRow, tmp);		AmB_from_2(A[2], scaledRow, A[2]);
	// i = 1, j = 3
	sA_from_2(A[3].v1(), A[1], scaledRow, tmp);		AmB_from_2(A[3], scaledRow, A[3]);
	// i = 1, j = 4
	sA_from_2(A[4].v1(), A[1], scaledRow, tmp);		AmB_from_2(A[4], scaledRow, A[4]);
	// i = 1, j = 5
	sA_from_2(A[5].v1(), A[1], scaledRow, tmp);		AmB_from_2(A[5], scaledRow, A[5]);
	// i = 1, j = 6
	sA_from_2(A[6].v1(), A[1], scaledRow, tmp);		AmB_from_2(A[6], scaledRow, A[6]);
	// i = 1, j = 7
	sA_from_2(A[7].v1(), A[1], scaledRow, tmp);		AmB_from_2(A[7], scaledRow, A[7]);
	// i = 1, j = 8
	sA_from_2(A[8].v1(), A[1], scaledRow, tmp);		AmB_from_2(A[8], scaledRow, A[8]);
	// i = 1, j = 9
	sA_from_2(A[9].v1(), A[1], scaledRow, tmp);		AmB_from_2(A[9], scaledRow, A[9]);

	// i = 2
	if(A[2].v2() == 0)
		return false;
	sA_from_3(1 / A[2].v2(), A[2], A[2], tmp);		/*A[2].v2() = 1;	memset(&A[2], 0, 8);*/
	// i = 2, j = 3
	sA_from_3(A[3].v2(), A[2], scaledRow, tmp);		AmB_from_3(A[3], scaledRow, A[3]);
	// i = 2, j = 4
	sA_from_3(A[4].v2(), A[2], scaledRow, tmp);		AmB_from_3(A[4], scaledRow, A[4]);
	// i = 2, j = 5
	sA_from_3(A[5].v2(), A[2], scaledRow, tmp);		AmB_from_3(A[5], scaledRow, A[5]);
	// i = 2, j = 6
	sA_from_3(A[6].v2(), A[2], scaledRow, tmp);		AmB_from_3(A[6], scaledRow, A[6]);
	// i = 2, j = 7
	sA_from_3(A[7].v2(), A[2], scaledRow, tmp);		AmB_from_3(A[7], scaledRow, A[7]);
	// i = 2, j = 8
	sA_from_3(A[8].v2(), A[2], scaledRow, tmp);		AmB_from_3(A[8], scaledRow, A[8]);
	// i = 2, j = 9
	sA_from_3(A[9].v2(), A[2], scaledRow, tmp);		AmB_from_3(A[9], scaledRow, A[9]);

	// i = 3
	if(A[3].v3() == 0)
		return false;
	sA_from_4or5(1 / A[3].v3(), A[3], A[3], tmp);	/*A[3].v3() = 1;	memset(&A[3], 0, 12);*/
	// i = 3, j = 4
	sA_from_4or5(A[4].v3(), A[3], scaledRow, tmp);	AmB_from_4or5(A[4], scaledRow, A[4]);
	// i = 3, j = 5
	sA_from_4or5(A[5].v3(), A[3], scaledRow, tmp);	AmB_from_4or5(A[5], scaledRow, A[5]);
	// i = 3, j = 6
	sA_from_4or5(A[6].v3(), A[3], scaledRow, tmp);	AmB_from_4or5(A[6], scaledRow, A[6]);
	// i = 3, j = 7
	sA_from_4or5(A[7].v3(), A[3], scaledRow, tmp);	AmB_from_4or5(A[7], scaledRow, A[7]);
	// i = 3, j = 8
	sA_from_4or5(A[8].v3(), A[3], scaledRow, tmp);	AmB_from_4or5(A[8], scaledRow, A[8]);
	// i = 3, j = 9
	sA_from_4or5(A[9].v3(), A[3], scaledRow, tmp);	AmB_from_4or5(A[9], scaledRow, A[9]);

	// i = 4
	if(A[4].v4() == 0)
		return false;
	sA_from_4or5(1 / A[4].v4(), A[4], A[4], tmp);	/*A[4].v4() = 1;	memset(&A[4], 0, 16);*/
	// i = 4, j = 5
	sA_from_4or5(A[5].v4(), A[4], scaledRow, tmp);	AmB_from_4or5(A[5], scaledRow, A[5]);
	// i = 4, j = 6
	sA_from_4or5(A[6].v4(), A[4], scaledRow, tmp);	AmB_from_4or5(A[6], scaledRow, A[6]);
	// i = 4, j = 7
	sA_from_4or5(A[7].v4(), A[4], scaledRow, tmp);	AmB_from_4or5(A[7], scaledRow, A[7]);
	// i = 4, j = 8
	sA_from_4or5(A[8].v4(), A[4], scaledRow, tmp);	AmB_from_4or5(A[8], scaledRow, A[8]);
	// i = 4, j = 9
	sA_from_4or5(A[9].v4(), A[4], scaledRow, tmp);	AmB_from_4or5(A[9], scaledRow, A[9]);

	// i = 5
	if(A[5].v5() == 0)
		return false;
	sA_from_6(1 / A[5].v5(), A[5], A[5], tmp);		/*A[5].v5() = 1;	memset(&A[5], 0, 20);*/
	// i = 5, j = 6
	sA_from_6(A[6].v5(), A[5], scaledRow, tmp);		AmB_from_6(A[6], scaledRow, A[6]);
	// i = 5, j = 7
	sA_from_6(A[7].v5(), A[5], scaledRow, tmp);		AmB_from_6(A[7], scaledRow, A[7]);
	// i = 5, j = 8
	sA_from_6(A[8].v5(), A[5], scaledRow, tmp);		AmB_from_6(A[8], scaledRow, A[8]);
	// i = 5, j = 9
	sA_from_6(A[9].v5(), A[5], scaledRow, tmp);		AmB_from_6(A[9], scaledRow, A[9]);

	// i = 6
	if(A[6].v6() == 0)
		return false;
	sA_from_7(1 / A[6].v6(), A[6], A[6], tmp);		/*A[6].v6() = 1;	memset(&A[6], 0, 24);*/
	// i = 6, j = 7
	sA_from_7(A[7].v6(), A[6], scaledRow, tmp);		AmB_from_7(A[7], scaledRow, A[7]);
	// i = 6, j = 8
	sA_from_7(A[8].v6(), A[6], scaledRow, tmp);		AmB_from_7(A[8], scaledRow, A[8]);
	// i = 6, j = 9
	sA_from_7(A[9].v6(), A[6], scaledRow, tmp);		AmB_from_7(A[9], scaledRow, A[9]);

	// i = 7
	if(A[7].v7() == 0)
		return false;
	sA_from_8or9(1 / A[7].v7(), A[7], A[7], tmp);	/*A[7].v7() = 1;	memset(&A[7], 0, 28);*/
	// i = 7, j = 8
	sA_from_8or9(A[8].v7(), A[7], scaledRow, tmp);	AmB_from_8or9(A[8], scaledRow, A[8]);
	// i = 7, j = 9
	sA_from_8or9(A[9].v7(), A[7], scaledRow, tmp);	AmB_from_8or9(A[9], scaledRow, A[9]);

	// i = 8
	if(A[8].v8() == 0)
		return false;
	sA_from_8or9(1 / A[8].v8(), A[8], A[8], tmp);	/*A[8].v8() = 1;	memset(&A[8], 0, 32);*/
	// i = 8, j = 9
	sA_from_8or9(A[9].v8(), A[8], scaledRow, tmp);	AmB_from_8or9(A[9], scaledRow, A[9]);

	// i = 9
	if(A[9].v9() == 0)
		return false;
	sA_from_10(1 / A[9].v9(), A[9], A[9], tmp);		/*A[9].v9() = 1;	memset(&A[9], 0, 36);*/

	
	/***************************************************/
	/* Back substitution
	/***************************************************/

	// i = 9
	// i = 9, j = 8
	sA_from_10(A[8].v9(), A[9], scaledRow, tmp);		AmB_from_10(A[8], scaledRow, A[8]);
	// i = 9, j = 7
	sA_from_10(A[7].v9(), A[9], scaledRow, tmp);		AmB_from_10(A[7], scaledRow, A[7]);
	// i = 9, j = 6
	sA_from_10(A[6].v9(), A[9], scaledRow, tmp);		AmB_from_10(A[6], scaledRow, A[6]);
	// i = 9, j = 5
	sA_from_10(A[5].v9(), A[9], scaledRow, tmp);		AmB_from_10(A[5], scaledRow, A[5]);
	// i = 9, j = 4
	sA_from_10(A[4].v9(), A[9], scaledRow, tmp);		AmB_from_10(A[4], scaledRow, A[4]);
	// i = 9, j = 3
	sA_from_10(A[3].v9(), A[9], scaledRow, tmp);		AmB_from_10(A[3], scaledRow, A[3]);
	// i = 9, j = 2
	sA_from_10(A[2].v9(), A[9], scaledRow, tmp);		AmB_from_10(A[2], scaledRow, A[2]);
	// i = 9, j = 1
	sA_from_10(A[1].v9(), A[9], scaledRow, tmp);		AmB_from_10(A[1], scaledRow, A[1]);
	// i = 9, j = 0
	sA_from_10(A[0].v9(), A[9], scaledRow, tmp);		AmB_from_10(A[0], scaledRow, A[0]);

	// i = 8
	// i = 8, j = 7
	sA_from_10(A[7].v8(), A[8], scaledRow, tmp);		AmB_from_10(A[7], scaledRow, A[7]);
	// i = 8, j = 6
	sA_from_10(A[6].v8(), A[8], scaledRow, tmp);		AmB_from_10(A[6], scaledRow, A[6]);
	// i = 8, j = 5
	sA_from_10(A[5].v8(), A[8], scaledRow, tmp);		AmB_from_10(A[5], scaledRow, A[5]);
	// i = 8, j = 4
	sA_from_10(A[4].v8(), A[8], scaledRow, tmp);		AmB_from_10(A[4], scaledRow, A[4]);
	// i = 8, j = 3
	sA_from_10(A[3].v8(), A[8], scaledRow, tmp);		AmB_from_10(A[3], scaledRow, A[3]);
	// i = 8, j = 2
	sA_from_10(A[2].v8(), A[8], scaledRow, tmp);		AmB_from_10(A[2], scaledRow, A[2]);
	// i = 8, j = 1
	sA_from_10(A[1].v8(), A[8], scaledRow, tmp);		AmB_from_10(A[1], scaledRow, A[1]);
	// i = 8, j = 0
	sA_from_10(A[0].v8(), A[8], scaledRow, tmp);		AmB_from_10(A[0], scaledRow, A[0]);

	// i = 7
	// i = 7, j = 6
	sA_from_10(A[6].v7(), A[7], scaledRow, tmp);		AmB_from_10(A[6], scaledRow, A[6]);
	// i = 7, j = 5
	sA_from_10(A[5].v7(), A[7], scaledRow, tmp);		AmB_from_10(A[5], scaledRow, A[5]);
	// i = 7, j = 4
	sA_from_10(A[4].v7(), A[7], scaledRow, tmp);		AmB_from_10(A[4], scaledRow, A[4]);
	// i = 7, j = 3
	sA_from_10(A[3].v7(), A[7], scaledRow, tmp);		AmB_from_10(A[3], scaledRow, A[3]);
	// i = 7, j = 2
	sA_from_10(A[2].v7(), A[7], scaledRow, tmp);		AmB_from_10(A[2], scaledRow, A[2]);
	// i = 7, j = 1
	sA_from_10(A[1].v7(), A[7], scaledRow, tmp);		AmB_from_10(A[1], scaledRow, A[1]);
	// i = 7, j = 0
	sA_from_10(A[0].v7(), A[7], scaledRow, tmp);		AmB_from_10(A[0], scaledRow, A[0]);

	// i = 6
	// i = 6, j = 5
	sA_from_10(A[5].v6(), A[6], scaledRow, tmp);		AmB_from_10(A[5], scaledRow, A[5]);
	// i = 6, j = 4
	sA_from_10(A[4].v6(), A[6], scaledRow, tmp);		AmB_from_10(A[4], scaledRow, A[4]);
	// i = 6, j = 3
	sA_from_10(A[3].v6(), A[6], scaledRow, tmp);		AmB_from_10(A[3], scaledRow, A[3]);
	// i = 6, j = 2
	sA_from_10(A[2].v6(), A[6], scaledRow, tmp);		AmB_from_10(A[2], scaledRow, A[2]);
	// i = 6, j = 1
	sA_from_10(A[1].v6(), A[6], scaledRow, tmp);		AmB_from_10(A[1], scaledRow, A[1]);
	// i = 6, j = 0
	sA_from_10(A[0].v6(), A[6], scaledRow, tmp);		AmB_from_10(A[0], scaledRow, A[0]);

	// i = 5
	// i = 5, j = 4
	sA_from_10(A[4].v5(), A[5], scaledRow, tmp);		AmB_from_10(A[4], scaledRow, A[4]);
	// i = 5, j = 3
	sA_from_10(A[3].v5(), A[5], scaledRow, tmp);		AmB_from_10(A[3], scaledRow, A[3]);
	// i = 5, j = 2
	sA_from_10(A[2].v5(), A[5], scaledRow, tmp);		AmB_from_10(A[2], scaledRow, A[2]);
	// i = 5, j = 1
	sA_from_10(A[1].v5(), A[5], scaledRow, tmp);		AmB_from_10(A[1], scaledRow, A[1]);
	// i = 5, j = 0
	sA_from_10(A[0].v5(), A[5], scaledRow, tmp);		AmB_from_10(A[0], scaledRow, A[0]);

	// i = 4
	// i = 4, j = 3
	sA_from_10(A[3].v4(), A[4], scaledRow, tmp);		AmB_from_10(A[3], scaledRow, A[3]);
	// i = 4, j = 2
	sA_from_10(A[2].v4(), A[4], scaledRow, tmp);		AmB_from_10(A[2], scaledRow, A[2]);
	// i = 4, j = 1
	sA_from_10(A[1].v4(), A[4], scaledRow, tmp);		AmB_from_10(A[1], scaledRow, A[1]);
	// i = 4, j = 0
	sA_from_10(A[0].v4(), A[4], scaledRow, tmp);		AmB_from_10(A[0], scaledRow, A[0]);

	// i = 3
	// i = 3, j = 2
	sA_from_10(A[2].v3(), A[3], scaledRow, tmp);		AmB_from_10(A[2], scaledRow, A[2]);
	// i = 3, j = 1
	sA_from_10(A[1].v3(), A[3], scaledRow, tmp);		AmB_from_10(A[1], scaledRow, A[1]);
	// i = 3, j = 0
	sA_from_10(A[0].v3(), A[3], scaledRow, tmp);		AmB_from_10(A[0], scaledRow, A[0]);

	// i = 2
	// i = 2, j = 1
	sA_from_10(A[1].v2(), A[2], scaledRow, tmp);		AmB_from_10(A[1], scaledRow, A[1]);
	// i = 2, j = 0
	sA_from_10(A[0].v2(), A[2], scaledRow, tmp);		AmB_from_10(A[0], scaledRow, A[0]);

	// i = 1
	// i = 1, j = 0
	sA_from_10(A[0].v1(), A[1], scaledRow, tmp);		AmB_from_10(A[0], scaledRow, A[0]);

	return true;
}