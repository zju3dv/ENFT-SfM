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

#ifndef _LINEAR_SYSTEM_H_
#define _LINEAR_SYSTEM_H_

namespace LA
{

	template<typename TYPE, int N> inline void SwapRow(TYPE **A, int* iwork, int a, int b)
	{
		int i;
		TYPE vTmp;
		for(i = 0; i < N; ++i)
		{
			vTmp = A[a][i];
			A[a][i] = A[b][i];
			A[b][i] = vTmp;
		}
		int iTmp = iwork[a];
		iwork[a] = iwork[b];
		iwork[b] = iTmp;
	}

	template<typename TYPE, int N> inline void Print(TYPE **A)
	{
		int i, j;
		for(i = 0; i < N; ++i)
		{
			for(j = 0; j < N; ++j)
				printf("%f ", A[i][j]);
			printf("\n");
		}
	}

	// sizeof(iwork) == sizeof(int) * N
	template<typename TYPE, int N> inline bool DecomposeLU(TYPE **A, int *iwork)
	{
		int i, j, k;
		TYPE v, vMax;
		int iMax;
		for(i = 1, vMax = fabs(A[0][0]), iMax = 0; i < N; ++i)
		{
			if((v = fabs(A[i][0])) < vMax)
				continue;
			vMax = v;
			iMax = i;
		}
		SwapRow<TYPE, N>(A, iwork, 0, iMax);
		if(vMax < FLT_EPSILON)
			return false;
		v = 1 / A[0][0];
		for(j = 1; j < N; ++j)
			A[j][0] *= v;
		for(i = 1; i < N - 1; ++i)
		{
			for(k = i + 1, vMax = fabs(A[i][i]), iMax = i; k < N; ++k)
			{
				if((v = fabs(A[k][i])) < vMax)
					continue;
				vMax = v;
				iMax = k;
			}
			if(vMax < FLT_EPSILON)
				return false;
			SwapRow<TYPE, N>(A, iwork, i, iMax);
			for(k = 0; k < i; ++k)
				A[i][i] -= A[i][k] * A[k][i];
			v = 1 / A[i][i];
			for(j = i + 1; j < N; ++j)
			{
				for(k = 0; k < i; ++k)
					A[i][j] -= A[i][k] * A[k][j];
				for(k = 0; k < i; ++k)
					A[j][i] -= A[j][k] * A[k][i];
				A[j][i] *= v;
			}
		}
		for(k = 0; k < N - 1; ++k)
			A[N - 1][N - 1] -= A[N - 1][k] * A[k][N - 1];
		return true;
	}

	template<typename TYPE, int N> inline bool DecomposeLDL(TYPE **A)
	{
		int i, j, k;
		TYPE v;
		if((v = A[0][0]) < FLT_EPSILON)
			return false;
		v = 1 / v;
		for(j = 1; j < N; ++j)
			A[j][0] = (A[0][j] *= v);
		for(i = 1; i < N - 1; ++i)
		{
			for(k = 0; k < i; ++k)
				A[i][i] -= A[k][i] * A[k][i] * A[k][k];
			if((v = A[i][i]) < FLT_EPSILON)
				return false;
			v = 1 / v;
			for(j = i + 1; j < N; ++j)
			{
				for(k = 0; k < i; ++k)
					A[i][j] -= A[k][j] * A[k][i] * A[k][k];
				A[j][i] = (A[i][j] *= v);
			}
		}
		for(k = 0; k < N - 1; ++k)
			A[N - 1][N - 1] -= A[k][N - 1] * A[k][N - 1] * A[k][k];
		return true;
	}

	// sizeof(work) == sizeof(TYPE) * N
	// sizeof(iwork) == sizeof(int) * N
	template<typename TYPE, int N> inline bool SolveLinearSystemLU(TYPE **A, TYPE *b, TYPE *work, int *iwork, bool decomposed = false)
	{
		int i, j;
		if(!decomposed)
		{
			for(i = 0; i < N; ++i)
				iwork[i] = i;
			if(!DecomposeLU<TYPE, N>(A, iwork))
				return false;
		}
		for(i = 0; i < N; ++i)
			work[i] = b[iwork[i]];
		memcpy(b, work, sizeof(TYPE) * N);
		for(i = 0; i < N; ++i)
		for(j = 0; j < i; ++j)
			b[i] -= A[i][j] * b[j];
		for(i = N - 1; i >= 0; --i)
		{
			for(j = i + 1; j < N; ++j)
				b[i] -= A[i][j] * b[j];
			if(fabs(A[i][i]) < FLT_EPSILON)
				return false;
			b[i] /= A[i][i];
		}
		return true;
	}

	template<typename TYPE, int N> inline bool SolveLinearSystemLDL(TYPE **A, TYPE *b, bool decomposed = false)
	{
		if(!decomposed && !DecomposeLDL<TYPE, N>(A))
			return false;
		//printf("\n");
		//Print<TYPE, N>(A);
		int i, j;
		for(i = 0; i < N; ++i)
		for(j = 0; j < i; ++j)
			b[i] -= A[i][j] * b[j];
		for(i = N - 1; i >= 0; --i)
		{
			if(fabs(A[i][i]) < FLT_EPSILON)
				return false;
			b[i] /= A[i][i];
			for(j = i + 1; j < N; ++j)
				b[i] -= A[i][j] * b[j];
		}
		return true;
	}

	// sizeof(work) == sizeof(TYPE) * N * (N + 1)
	// sizeof(iwork) == sizeof(int) * N
	template<typename TYPE, int N> inline bool InvertLU(TYPE **A, TYPE *work, int *iwork)
	{
		int i, j;
		for(i = 0; i < N; ++i)
			iwork[i] = i;
		if(!DecomposeLU<TYPE, N>(A, iwork))
			return false;
		memset(work, 0, sizeof(TYPE) * N * N);
		for(i = 0; i < N; ++i)
		{
			work[i * N + i] = 1;
			if(!SolveLinearSystemLU<TYPE, N>(A, &work[i * N], &work[N * N], iwork, true))
				return false;
		}
		for(i = 0; i < N; ++i)
		for(j = 0; j < N; ++j)
			A[i][j] = work[j * N + i];
		return true;
	}

	// sizeof(work) == sizeof(TYPE) * N * N
	template<typename TYPE, int N> inline bool InvertLDL(TYPE **A, TYPE *work)
	{
		int i;
		DecomposeLDL<TYPE, N>(A);
		memset(work, 0, sizeof(TYPE) * N * N);
		for(i = 0; i < N; ++i)
		{
			work[i * N + i] = 1;
			if(!SolveLinearSystemLDL<TYPE, N>(A, &work[i * N], true))
				return false;
		}
		memcpy(A[0], work, sizeof(TYPE) * N * N);
		return true;
	}

}

#endif