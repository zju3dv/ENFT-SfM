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

#ifndef _MATRIX_4x6_H_
#define _MATRIX_4x6_H_

#include "Utility/SSE.h"
#include "Matrix2x6.h"

namespace LA
{

	class AlignedCompactMatrix4x6f : public AlignedCompactMatrix2x6f
	{

	public:

		inline const ENFT_SSE::__m128& M_20_21_22_23() const { return m_20_21_22_23; }	inline ENFT_SSE::__m128& M_20_21_22_23() { return m_20_21_22_23; }
		inline const ENFT_SSE::__m128& M_24_25_34_35() const { return m_24_25_34_35; }	inline ENFT_SSE::__m128& M_24_25_34_35() { return m_24_25_34_35; }
		inline const ENFT_SSE::__m128& M_30_31_32_33() const { return m_30_31_32_33; }	inline ENFT_SSE::__m128& M_30_31_32_33() { return m_30_31_32_33; }
		inline const float& M20() const { return m_20_21_22_23.m128_f32[0]; }	inline float& M20() { return m_20_21_22_23.m128_f32[0]; }
		inline const float& M21() const { return m_20_21_22_23.m128_f32[1]; }	inline float& M21() { return m_20_21_22_23.m128_f32[1]; }
		inline const float& M22() const { return m_20_21_22_23.m128_f32[2]; }	inline float& M22() { return m_20_21_22_23.m128_f32[2]; }
		inline const float& M23() const { return m_20_21_22_23.m128_f32[3]; }	inline float& M23() { return m_20_21_22_23.m128_f32[3]; }
		inline const float& M24() const { return m_24_25_34_35.m128_f32[0]; }	inline float& M24() { return m_24_25_34_35.m128_f32[0]; }
		inline const float& M25() const { return m_24_25_34_35.m128_f32[1]; }	inline float& M25() { return m_24_25_34_35.m128_f32[1]; }
		inline const float& M30() const { return m_30_31_32_33.m128_f32[0]; }	inline float& M30() { return m_30_31_32_33.m128_f32[0]; }
		inline const float& M31() const { return m_30_31_32_33.m128_f32[1]; }	inline float& M31() { return m_30_31_32_33.m128_f32[1]; }
		inline const float& M32() const { return m_30_31_32_33.m128_f32[2]; }	inline float& M32() { return m_30_31_32_33.m128_f32[2]; }
		inline const float& M33() const { return m_30_31_32_33.m128_f32[3]; }	inline float& M33() { return m_30_31_32_33.m128_f32[3]; }
		inline const float& M34() const { return m_24_25_34_35.m128_f32[2]; }	inline float& M34() { return m_24_25_34_35.m128_f32[2]; }
		inline const float& M35() const { return m_24_25_34_35.m128_f32[3]; }	inline float& M35() { return m_24_25_34_35.m128_f32[3]; }
		inline void ConvertToConventionalStorage(float *work2)
		{
			AlignedCompactMatrix2x6f::ConvertToConventionalStorage(work2);
			memcpy(work2, &M34(), 8);
			memcpy(&M34(), &M30(), 16);
			memcpy(&M32(), work2, 8);
		}
		inline void ConvertToConventionalStorage(float *M) const
		{
			AlignedCompactMatrix2x6f::ConvertToConventionalStorage(M);
			memcpy(M + 12, &M20(), 24);
			memcpy(M + 18, &M30(), 16);
			memcpy(M + 22, &M34(), 8);
		}
		inline void ConvertToSpecialStorage(float *work2)
		{
			AlignedCompactMatrix2x6f::ConvertToSpecialStorage(work2);
			memcpy(work2, &M32(), 8);
			//memcpy(&M30(), &M34(), 16);
			memcpy(&M32(), &M30(), 8);
			memcpy(&M30(), &M34(), 8);
			memcpy(&M34(), work2, 8);
		}

	protected:

		ENFT_SSE::__m128 m_20_21_22_23, m_24_25_34_35, m_30_31_32_33;

	};
}

#endif