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

#ifndef _MATRIX_4x7_H_
#define _MATRIX_4x7_H_

#include "Matrix3x7.h"

namespace LA
{

	class AlignedMatrix4x7f : public AlignedMatrix3x7f
	{

	public:

		inline const ENFT_SSE::__m128& M_30_31_32_33() const { return m_30_31_32_33; }		inline ENFT_SSE::__m128& M_30_31_32_33() { return m_30_31_32_33; }
		inline const ENFT_SSE::__m128& M_34_35_36_x () const { return m_34_35_36_x; }			inline ENFT_SSE::__m128& M_34_35_36_x () { return m_34_35_36_x; }
		inline const float& M30() const { return m_30_31_32_33.m128_f32[0]; }		inline float& M30() { return m_30_31_32_33.m128_f32[0]; }
		inline const float& M31() const { return m_30_31_32_33.m128_f32[1]; }		inline float& M31() { return m_30_31_32_33.m128_f32[1]; }
		inline const float& M32() const { return m_30_31_32_33.m128_f32[2]; }		inline float& M32() { return m_30_31_32_33.m128_f32[2]; }
		inline const float& M33() const { return m_30_31_32_33.m128_f32[3]; }		inline float& M33() { return m_30_31_32_33.m128_f32[3]; }
		inline const float& M34() const { return m_34_35_36_x.m128_f32[0]; }		inline float& M34() { return m_34_35_36_x.m128_f32[0]; }
		inline const float& M35() const { return m_34_35_36_x.m128_f32[1]; }		inline float& M35() { return m_34_35_36_x.m128_f32[1]; }
		inline const float& M36() const { return m_34_35_36_x.m128_f32[2]; }		inline float& M36() { return m_34_35_36_x.m128_f32[2]; }
		inline const float& reserve3() const { return m_34_35_36_x.m128_f32[3]; }	inline float& reserve3() { return m_34_35_36_x.m128_f32[3]; }

	protected:

		ENFT_SSE::__m128 m_30_31_32_33, m_34_35_36_x;

	};

	inline void SetReserve(AlignedMatrix4x7f &M) { M.reserve0() = M.reserve1() = M.reserve2() = M.reserve3() = 0; }
}

#endif
