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

#ifndef _MATRIX_6x3_H_
#define _MATRIX_6x3_H_

#include "Matrix3.h"

namespace LA
{

	class AlignedMatrix6x3f : public AlignedMatrix3f
	{

	public:

		inline const ENFT_SSE::__m128& M_30_31_32_x() const { return m_sse3; }			inline ENFT_SSE::__m128& M_30_31_32_x() { return m_sse3; }
		inline const float& M30()	   const { return m_sse3.m128_f32[0]; }		inline float& M30()		 { return m_sse3.m128_f32[0]; }
		inline const float& M31()	   const { return m_sse3.m128_f32[1]; }		inline float& M31()		 { return m_sse3.m128_f32[1]; }
		inline const float& M32()	   const { return m_sse3.m128_f32[2]; }		inline float& M32()		 { return m_sse3.m128_f32[2]; }
		inline const float& reserve3() const { return m_sse3.m128_f32[3]; }		inline float& reserve3() { return m_sse3.m128_f32[3]; }

		inline const ENFT_SSE::__m128& M_40_41_42_x() const { return m_sse4; }			inline ENFT_SSE::__m128& M_40_41_42_x() { return m_sse4; }
		inline const float& M40()	   const { return m_sse4.m128_f32[0]; }		inline float& M40()		 { return m_sse4.m128_f32[0]; }
		inline const float& M41()	   const { return m_sse4.m128_f32[1]; }		inline float& M41()		 { return m_sse4.m128_f32[1]; }
		inline const float& M42()	   const { return m_sse4.m128_f32[2]; }		inline float& M42()		 { return m_sse4.m128_f32[2]; }
		inline const float& reserve4() const { return m_sse4.m128_f32[3]; }		inline float& reserve4() { return m_sse4.m128_f32[3]; }

		inline const ENFT_SSE::__m128& M_50_51_52_x() const { return m_sse5; }			inline ENFT_SSE::__m128& M_50_51_52_x() { return m_sse5; }
		inline const float& M50()	   const { return m_sse5.m128_f32[0]; }		inline float& M50()		 { return m_sse5.m128_f32[0]; }
		inline const float& M51()	   const { return m_sse5.m128_f32[1]; }		inline float& M51()		 { return m_sse5.m128_f32[1]; }
		inline const float& M52()	   const { return m_sse5.m128_f32[2]; }		inline float& M52()		 { return m_sse5.m128_f32[2]; }
		inline const float& reserve5() const { return m_sse5.m128_f32[3]; }		inline float& reserve5() { return m_sse5.m128_f32[3]; }

		inline void SetZero() { memset(this, 0, 96); }

	private:

		ENFT_SSE::__m128 m_sse3, m_sse4, m_sse5;
	};
}

#endif