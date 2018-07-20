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

#ifndef _MATRIX_6x7_H_
#define _MATRIX_6x7_H_

#include "Matrix5x7.h"

namespace LA
{

	class AlignedMatrix6x7f : public AlignedMatrix5x7f
	{

	public:

		inline const ENFT_SSE::__m128& M_50_51_52_53() const { return m_50_51_52_53; }		inline ENFT_SSE::__m128& M_50_51_52_53() { return m_50_51_52_53; }
		inline const ENFT_SSE::__m128& M_54_55_56_x () const { return m_54_55_56_x; }			inline ENFT_SSE::__m128& M_54_55_56_x () { return m_54_55_56_x; }
		inline const float& M50() const { return m_50_51_52_53.m128_f32[0]; }		inline float& M50() { return m_50_51_52_53.m128_f32[0]; }
		inline const float& M51() const { return m_50_51_52_53.m128_f32[1]; }		inline float& M51() { return m_50_51_52_53.m128_f32[1]; }
		inline const float& M52() const { return m_50_51_52_53.m128_f32[2]; }		inline float& M52() { return m_50_51_52_53.m128_f32[2]; }
		inline const float& M53() const { return m_50_51_52_53.m128_f32[3]; }		inline float& M53() { return m_50_51_52_53.m128_f32[3]; }
		inline const float& M54() const { return m_54_55_56_x.m128_f32[0]; }		inline float& M54() { return m_54_55_56_x.m128_f32[0]; }
		inline const float& M55() const { return m_54_55_56_x.m128_f32[1]; }		inline float& M55() { return m_54_55_56_x.m128_f32[1]; }
		inline const float& M56() const { return m_54_55_56_x.m128_f32[2]; }		inline float& M56() { return m_54_55_56_x.m128_f32[2]; }
		inline const float& reserve5() const { return m_54_55_56_x.m128_f32[3]; }	inline float& reserve5() { return m_54_55_56_x.m128_f32[3]; }

	protected:

		ENFT_SSE::__m128 m_50_51_52_53, m_54_55_56_x;

	};

	inline void SetReserve(AlignedMatrix6x7f &M) { M.reserve0() = M.reserve1() = M.reserve2() = M.reserve3() = M.reserve4() = M.reserve5() = 0; }
}

#endif
