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

#ifndef _MATRIX_5x7_H_
#define _MATRIX_5x7_H_

#include "Matrix4x7.h"

namespace LA
{

	class AlignedMatrix5x7f : public AlignedMatrix4x7f
	{

	public:

		inline const ENFT_SSE::__m128& M_40_41_42_43() const { return m_40_41_42_43; }		inline ENFT_SSE::__m128& M_40_41_42_43() { return m_40_41_42_43; }
		inline const ENFT_SSE::__m128& M_44_45_46_x () const { return m_44_45_46_x; }			inline ENFT_SSE::__m128& M_44_45_46_x () { return m_44_45_46_x; }
		inline const float& M40() const { return m_40_41_42_43.m128_f32[0]; }		inline float& M40() { return m_40_41_42_43.m128_f32[0]; }
		inline const float& M41() const { return m_40_41_42_43.m128_f32[1]; }		inline float& M41() { return m_40_41_42_43.m128_f32[1]; }
		inline const float& M42() const { return m_40_41_42_43.m128_f32[2]; }		inline float& M42() { return m_40_41_42_43.m128_f32[2]; }
		inline const float& M43() const { return m_40_41_42_43.m128_f32[3]; }		inline float& M43() { return m_40_41_42_43.m128_f32[3]; }
		inline const float& M44() const { return m_44_45_46_x.m128_f32[0]; }		inline float& M44() { return m_44_45_46_x.m128_f32[0]; }
		inline const float& M45() const { return m_44_45_46_x.m128_f32[1]; }		inline float& M45() { return m_44_45_46_x.m128_f32[1]; }
		inline const float& M46() const { return m_44_45_46_x.m128_f32[2]; }		inline float& M46() { return m_44_45_46_x.m128_f32[2]; }
		inline const float& reserve4() const { return m_44_45_46_x.m128_f32[3]; }	inline float& reserve4() { return m_44_45_46_x.m128_f32[3]; }

	protected:

		ENFT_SSE::__m128 m_40_41_42_43, m_44_45_46_x;

	};

	inline void SetReserve(AlignedMatrix5x7f &M) { M.reserve0() = M.reserve1() = M.reserve2() = M.reserve3() = M.reserve4() = 0; }
}

#endif
