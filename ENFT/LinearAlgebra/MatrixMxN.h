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

#ifndef _MATRIX_MXN_H_
#define _MATRIX_MXN_H_

#include "Utility/SSE.h"
#include "Matrix3.h"
#include "Vector9.h"
#include "VectorN.h"

namespace LA
{
	template<typename TYPE, uint N>  inline const uint _Stride();
	template<>  inline const uint _Stride<float,   4>() { return  4; }
	template<>  inline const uint _Stride<float,   5>() { return  8; }
	template<>  inline const uint _Stride<float,   6>() { return  8; }
	template<>  inline const uint _Stride<float,   7>() { return  8; }
	template<>  inline const uint _Stride<float,   8>() { return  8; }
	template<>  inline const uint _Stride<float,   9>() { return 12; }
	template<>  inline const uint _Stride<float,  10>() { return 12; }
	template<>  inline const uint _Stride<float,  11>() { return 12; }
	template<>  inline const uint _Stride<float,  12>() { return 12; }
	template<>  inline const uint _Stride<double,  4>() { return  4; }
	template<>  inline const uint _Stride<double,  5>() { return  6; }
	template<>  inline const uint _Stride<double,  6>() { return  6; }
	template<>  inline const uint _Stride<double,  7>() { return  8; }
	template<>  inline const uint _Stride<double,  8>() { return  8; }
	template<>  inline const uint _Stride<double,  9>() { return 10; }
	template<>  inline const uint _Stride<double, 10>() { return 10; }
	template<>  inline const uint _Stride<double, 11>() { return 12; }
	template<>  inline const uint _Stride<double, 12>() { return 12; }
	template<typename TYPE, uint M, uint N> class AlignedMatrixMxN
	{

	public:

		AlignedMatrixMxN()
		{
			const uint stride = _Stride<TYPE, N>();
			m_rows[0] = (TYPE *) _aligned_malloc(sizeof(TYPE) * M * stride, SSE_ALIGNMENT);
			for(uint i = 1; i < M; ++i)
				m_rows[i] = m_rows[i - 1] + stride;
		}
		AlignedMatrixMxN(const uint &iRow, const TYPE val)
		{
			const uint stride = _Stride<TYPE, N>();
			m_rows[0] = (TYPE *) _aligned_malloc(sizeof(TYPE) * M * stride, SSE_ALIGNMENT);
			for(uint i = 1; i < M; ++i)
				m_rows[i] = m_rows[i - 1] + stride;
			for(uint j = 0; j < N; j++)
				m_rows[iRow][j] = val;
		}
		~AlignedMatrixMxN() { _aligned_free(m_rows[0]); }

		//inline operator const TYPE* () const { return m_rows[0]; }					inline operator TYPE* ()	{ return m_rows[0]; }
		inline const TYPE* operator[](const uint &i) const { return m_rows[i]; }		inline TYPE* operator[](const uint &i) { return m_rows[i]; }
		inline void SetZero() { memset(m_rows[0], 0, sizeof(TYPE) * M * _Stride<TYPE, N>()); }

		inline void Print() const
		{
			for(uint i = 0; i < M; ++i)
			{
				for(uint j = 0; j < N; j++)
				{
					if(m_rows[i][j] == 0)
						printf("      ");
					else if(m_rows[i][j] > 0)
						printf(" %.2f ", m_rows[i][j]);
					else
						printf("%.2f ", m_rows[i][j]);
				}
				printf("\n");
			}
			printf("\n\n\n");
		}
		inline void PrintTranspose() const
		{
			for(uint j = 0; j < N; j++)
			{
				for(uint i = 0; i < M; ++i)
				{
					if(m_rows[i][j] == 0)
						printf("      ");
					else if(m_rows[i][j] > 0)
						printf(" %.2f ", m_rows[i][j]);
					else
						printf("%.2f ", m_rows[i][j]);
				}
				printf("\n");
			}
			printf("\n\n\n");
		}

		void Transpose();

		//static inline const uint& Stride() { return g_stride; }
		static inline const uint Stride() { return _Stride<TYPE, N>(); }

	protected:

		TYPE* m_rows[M];

	//private:

	//	static const uint g_stride;

	};

	//typedef AlignedMatrixMxN<float,   3,  4> AlignedMatrix3x4f;
	typedef AlignedMatrixMxN<float,   3,  6> AlignedMatrix3x6f;
	//typedef AlignedMatrixMxN<float,   3,  8> AlignedMatrix3x8f;
	typedef AlignedMatrixMxN<float,   4,  9> AlignedMatrix4x9f;
	typedef AlignedMatrixMxN<float,   6,  9> AlignedMatrix6x9f;
	//typedef AlignedMatrixMxN<float,   8,  8> AlignedMatrix8f;
	//typedef AlignedMatrixMxN<float,   9,  5> AlignedMatrix9x5f;
	typedef AlignedMatrixMxN<float,   9,  9> AlignedMatrix9f;
	typedef AlignedMatrixMxN<float,  10,  5> AlignedMatrix10x5f;
	typedef AlignedMatrixMxN<float,  10, 10> AlignedMatrix10f;
	typedef AlignedMatrixMxN<float, 12, 12> AlignedMatrix12f;
	//typedef AlignedMatrixMxN<double, 12, 12> AlignedMatrix12d;

//	inline void AAT(const AlignedMatrix3x4f &A, AlignedMatrix3f &AAT)
//	{
//#ifndef SET_AAT_ENTRY_3x4
//#define SET_AAT_ENTRY_3x4(i, j)\
//	AAT.M##i##j() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[i]), PTR_TO_SSE(A[j])))
//#endif
//		SET_AAT_ENTRY_3x4(0, 0);	SET_AAT_ENTRY_3x4(0, 1);	SET_AAT_ENTRY_3x4(0, 2);
//		SET_AAT_ENTRY_3x4(1, 1);	SET_AAT_ENTRY_3x4(1, 2);
//		SET_AAT_ENTRY_3x4(2, 2);
//	}
	inline void AAT(const AlignedMatrix3x6f &A, AlignedMatrix3f &AAT)
	{
#ifndef SET_AAT_ENTRY_3x6
#define SET_AAT_ENTRY_3x6(i, j)\
	AAT.M##i##j() = A[i][4] * A[j][4] + A[i][5] * A[j][5] + ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[i]), PTR_TO_SSE(A[j])))
#endif
		SET_AAT_ENTRY_3x6(0, 0);	SET_AAT_ENTRY_3x6(0, 1);	SET_AAT_ENTRY_3x6(0, 2);
		SET_AAT_ENTRY_3x6(1, 1);	SET_AAT_ENTRY_3x6(1, 2);
		SET_AAT_ENTRY_3x6(2, 2);
	}
//	inline void AAT(const AlignedMatrix3x8f &A, AlignedMatrix3f &AAT)
//	{
//#ifndef SET_AAT_ENTRY_3x8
//#define SET_AAT_ENTRY_3x8(i, j)\
//	AAT.M##i##j() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[i]), PTR_TO_SSE(A[j])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[i]+4), PTR_TO_SSE(A[j]+4))))
//#endif
//		SET_AAT_ENTRY_3x8(0, 0);	SET_AAT_ENTRY_3x8(0, 1);	SET_AAT_ENTRY_3x8(0, 2);
//		SET_AAT_ENTRY_3x8(1, 1);	SET_AAT_ENTRY_3x8(1, 2);
//		SET_AAT_ENTRY_3x8(2, 2);
//	}
	inline void ATBT(const AlignedVector9f &A, const AlignedMatrix6x9f &B, float *ATBT)
	{
		ATBT[0] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), PTR_TO_SSE(B[0])), ENFT_SSE::_mm_mul_ps(A.v4567(), PTR_TO_SSE(B[0] + 4)))) + A.v8() * B[0][8];
		ATBT[1] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), PTR_TO_SSE(B[1])), ENFT_SSE::_mm_mul_ps(A.v4567(), PTR_TO_SSE(B[1] + 4)))) + A.v8() * B[1][8];
		ATBT[2] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), PTR_TO_SSE(B[2])), ENFT_SSE::_mm_mul_ps(A.v4567(), PTR_TO_SSE(B[2] + 4)))) + A.v8() * B[2][8];
		ATBT[3] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), PTR_TO_SSE(B[3])), ENFT_SSE::_mm_mul_ps(A.v4567(), PTR_TO_SSE(B[3] + 4)))) + A.v8() * B[3][8];
		ATBT[4] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), PTR_TO_SSE(B[4])), ENFT_SSE::_mm_mul_ps(A.v4567(), PTR_TO_SSE(B[4] + 4)))) + A.v8() * B[4][8];
		ATBT[5] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.v0123(), PTR_TO_SSE(B[5])), ENFT_SSE::_mm_mul_ps(A.v4567(), PTR_TO_SSE(B[5] + 4)))) + A.v8() * B[5][8];
	}
	inline void ABT(const AlignedMatrix4x9f &A, const AlignedMatrix6x9f &B, float *ABT)
	{
		ABT[ 0] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0]), PTR_TO_SSE(B[0])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0] + 4), PTR_TO_SSE(B[0] + 4)))) + A[0][8] * B[0][8];
		ABT[ 1] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0]), PTR_TO_SSE(B[1])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0] + 4), PTR_TO_SSE(B[1] + 4)))) + A[0][8] * B[1][8];
		ABT[ 2] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0]), PTR_TO_SSE(B[2])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0] + 4), PTR_TO_SSE(B[2] + 4)))) + A[0][8] * B[2][8];
		ABT[ 3] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0]), PTR_TO_SSE(B[3])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0] + 4), PTR_TO_SSE(B[3] + 4)))) + A[0][8] * B[3][8];
		ABT[ 4] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0]), PTR_TO_SSE(B[4])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0] + 4), PTR_TO_SSE(B[4] + 4)))) + A[0][8] * B[4][8];
		ABT[ 5] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0]), PTR_TO_SSE(B[5])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[0] + 4), PTR_TO_SSE(B[5] + 4)))) + A[0][8] * B[5][8];
		
		ABT[ 6] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1]), PTR_TO_SSE(B[0])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1] + 4), PTR_TO_SSE(B[0] + 4)))) + A[1][8] * B[0][8];
		ABT[ 7] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1]), PTR_TO_SSE(B[1])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1] + 4), PTR_TO_SSE(B[1] + 4)))) + A[1][8] * B[1][8];
		ABT[ 8] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1]), PTR_TO_SSE(B[2])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1] + 4), PTR_TO_SSE(B[2] + 4)))) + A[1][8] * B[2][8];
		ABT[ 9] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1]), PTR_TO_SSE(B[3])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1] + 4), PTR_TO_SSE(B[3] + 4)))) + A[1][8] * B[3][8];
		ABT[10] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1]), PTR_TO_SSE(B[4])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1] + 4), PTR_TO_SSE(B[4] + 4)))) + A[1][8] * B[4][8];
		ABT[11] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1]), PTR_TO_SSE(B[5])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[1] + 4), PTR_TO_SSE(B[5] + 4)))) + A[1][8] * B[5][8];

		ABT[12] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2]), PTR_TO_SSE(B[0])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2] + 4), PTR_TO_SSE(B[0] + 4)))) + A[2][8] * B[0][8];
		ABT[13] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2]), PTR_TO_SSE(B[1])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2] + 4), PTR_TO_SSE(B[1] + 4)))) + A[2][8] * B[1][8];
		ABT[14] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2]), PTR_TO_SSE(B[2])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2] + 4), PTR_TO_SSE(B[2] + 4)))) + A[2][8] * B[2][8];
		ABT[15] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2]), PTR_TO_SSE(B[3])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2] + 4), PTR_TO_SSE(B[3] + 4)))) + A[2][8] * B[3][8];
		ABT[16] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2]), PTR_TO_SSE(B[4])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2] + 4), PTR_TO_SSE(B[4] + 4)))) + A[2][8] * B[4][8];
		ABT[17] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2]), PTR_TO_SSE(B[5])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[2] + 4), PTR_TO_SSE(B[5] + 4)))) + A[2][8] * B[5][8];

		ABT[18] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3]), PTR_TO_SSE(B[0])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3] + 4), PTR_TO_SSE(B[0] + 4)))) + A[3][8] * B[0][8];
		ABT[19] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3]), PTR_TO_SSE(B[1])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3] + 4), PTR_TO_SSE(B[1] + 4)))) + A[3][8] * B[1][8];
		ABT[20] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3]), PTR_TO_SSE(B[2])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3] + 4), PTR_TO_SSE(B[2] + 4)))) + A[3][8] * B[2][8];
		ABT[21] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3]), PTR_TO_SSE(B[3])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3] + 4), PTR_TO_SSE(B[3] + 4)))) + A[3][8] * B[3][8];
		ABT[22] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3]), PTR_TO_SSE(B[4])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3] + 4), PTR_TO_SSE(B[4] + 4)))) + A[3][8] * B[4][8];
		ABT[23] = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3]), PTR_TO_SSE(B[5])), ENFT_SSE::_mm_mul_ps(PTR_TO_SSE(A[3] + 4), PTR_TO_SSE(B[5] + 4)))) + A[3][8] * B[5][8];
	}

	template<typename TYPE, uint M> class AlignedMatrixMxX
	{

	public:

		AlignedMatrixMxX() : m_N(0), m_stride(0), m_capacity(0) { m_rows[0] = NULL; }
		AlignedMatrixMxX(const uint &N) : m_N(N)
		{
			m_stride = N + (16 - ((sizeof(TYPE) * N) & 15)) / sizeof(TYPE);
			m_capacity = m_stride;
			m_rows[0] = (TYPE *) _aligned_malloc(sizeof(TYPE) * M * m_stride, SSE_ALIGNMENT);
			for(uint i = 1; i < M; ++i)
				m_rows[i] = m_rows[i - 1] + m_stride;
		}
		~AlignedMatrixMxX()
		{
			if(m_N)
				_aligned_free(m_rows[0]);
		}

		inline const uint& GetColumnsNumber() const { return m_N; }
		inline void GetColumn(const uint &j, AlignedVectorN<TYPE, M> &col) const
		{
			for(uint i = 0; i < M; ++i)
				col[i] = m_rows[i][j];
		}
		inline void Resize(const uint &N)
		{
			if(m_N == N)
				return;
			const uint stride = N + ((16 - ((sizeof(TYPE) * N) & 15)) & 15) / sizeof(TYPE);

			if(m_capacity < stride)
			{
				if(m_rows[0] != NULL)
					_aligned_free(m_rows[0]);
				m_rows[0] = (TYPE *) _aligned_malloc(sizeof(TYPE) * M * stride, SSE_ALIGNMENT);
				m_capacity = stride;
			}

			m_N = N;
			m_stride = stride;
			for(uint i = 1; i < M; ++i)
				m_rows[i] = m_rows[i - 1] + stride;
		}
		inline const uint& Stride() const { return m_stride; }

		inline TYPE* operator[] (const uint &i) { return m_rows[i]; }
		inline const TYPE* operator[] (const uint &i) const { return m_rows[i]; }

		inline void Print() const
		{
			for(uint i = 0; i < M; ++i)
			{
				for(uint j = 0; j < m_N; j++)
				{
					if(m_rows[i][j] == 0)
						printf("      ");
					else if(m_rows[i][j] > 0)
						printf(" %.2f ", m_rows[i][j]);
					else
						printf("%.2f ", m_rows[i][j]);
				}
				printf("\n");
			}
		}
		inline void PrintTranspose() const
		{
			for(uint j = 0; j < m_N; j++)
			{
				for(uint i = 0; i < M; ++i)
				{
					if(m_rows[i][j] == 0)
						printf("      ");
					else if(m_rows[i][j] > 0)
						printf(" %.2f ", m_rows[i][j]);
					else
						printf("%.2f ", m_rows[i][j]);
				}
				printf("\n");
			}
		}

	protected:

		uint m_N, m_stride, m_capacity;
		TYPE* m_rows[M];

	};

	typedef AlignedMatrixMxX<float,  3> AlignedMatrix3xXf;
	typedef AlignedMatrixMxX<float,  8> AlignedMatrix8xXf;
	typedef AlignedMatrixMxX<float,  9> AlignedMatrix9xXf;
	typedef AlignedMatrixMxX<float, 11> AlignedMatrix11xXf;
	typedef AlignedMatrixMxX<float, 12> AlignedMatrix12xXf;
	typedef AlignedMatrixMxX<double,12> AlignedMatrix12xXd;

	void AAT(const AlignedMatrix3xXf &A, AlignedMatrix3f &AAT);

	template<typename TYPE, uint N>
	class AlignedMatrixXxN
	{

	public:

		AlignedMatrixXxN() : m_M(0), m_capacity(0) {}
		AlignedMatrixXxN(const uint &M) : m_M(M)
		{
			const uint stride = _Stride<TYPE, N>();
			m_capacity = M;
			m_rows.resize(M);
			m_rows[0] = (TYPE *) _aligned_malloc(sizeof(TYPE) * M * stride, SSE_ALIGNMENT);
			for(uint i = 1; i < M; ++i)
				m_rows[i] = m_rows[i - 1] + stride;
		}
		~AlignedMatrixXxN()
		{
			if(!m_rows.empty())
				_aligned_free(m_rows[0]);
		}

		inline const uint GetRowsNumber() const { return m_M; }
		inline void Resize(const uint &M)
		{
			if(m_capacity >= M)
			{
				m_M = M;
				return;
			}
			if(!m_rows.empty())
				_aligned_free(m_rows[0]);
			const uint stride = _Stride<TYPE, N>();
			m_M = m_capacity = M;
			m_rows.resize(M);
			m_rows[0] = (TYPE *) _aligned_malloc(sizeof(TYPE) * M * stride, SSE_ALIGNMENT);
			for(uint i = 1; i < M; ++i)
				m_rows[i] = m_rows[i - 1] + stride;
		}

		inline		 TYPE* operator[] (const uint &i)	    { return m_rows[i]; }
		inline const TYPE* operator[] (const uint &i) const { return m_rows[i]; }

		inline void Print() const
		{
			for(uint i = 0; i < m_M; ++i)
			{
				for(uint j = 0; j < N; j++)
				{
					if(m_rows[i][j] == 0)
						printf("      ");
					else if(m_rows[i][j] > 0)
						printf(" %.2f ", m_rows[i][j]);
					else
						printf("%.2f ", m_rows[i][j]);
				}
				printf("\n");
			}
			printf("\n\n\n");
		}

	public:

		static inline const uint Stride() { return _Stride<TYPE, N>(); }

	protected:

		uint m_M, m_capacity;
		std::vector<TYPE *> m_rows;

	};

	typedef AlignedMatrixXxN<float,   4> AlignedMatrixXx4f;
	typedef AlignedMatrixXxN<float,  11> AlignedMatrixXx11f;
	typedef AlignedMatrixXxN<float,  12> AlignedMatrixXx12f;
	typedef AlignedMatrixXxN<double, 12> AlignedMatrixXx12d;
	
}

#endif