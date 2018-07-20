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

#ifndef _VECTOR_9_H_
#define _VECTOR_9_H_

#include "Vector8.h"

namespace LA
{

	class AlignedVector9f : public AlignedVector8f
	{

	public:

		inline const float& v8() const { return m_8xxx.m128_f32[0]; }
		inline		 float& v8()	   { return m_8xxx.m128_f32[0]; }

		inline void Set(const float &v0, const float &v1, const float &v2, const float &v3, 
						const float &v4, const float &v5, const float &v6, const float &v7, const float &v8)
		{
			v0123() = ENFT_SSE::_mm_set_ps(v0, v1, v2, v3);
			v4567() = ENFT_SSE::_mm_set_ps(v4, v5, v6, v7);
			m_8xxx.m128_f32[0] = v8;
		}
		inline float SquaredLength() const { return AlignedVector8f::SquaredLength() + v8() * v8(); }

	private:

		ENFT_SSE::__m128 m_8xxx;

	};

};

#endif