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

#ifndef _SSE_H_
#define _SSE_H_

#ifdef _LINUX_
#include "Utility/linux_sse/xmmintrin.h"
#include "Utility/linux_sse/emmintrin.h"
namespace ENFT_SSE {
#else
#include <xmmintrin.h>
#include <emmintrin.h>
#define ENFT_SSE
#endif

#ifndef VALUE_TO_SSE
#define VALUE_TO_SSE(val) *(__m128*) &val
#endif

#ifndef PTR_TO_SSE
#define PTR_TO_SSE(ptr) *(__m128*) (ptr)
#endif

#ifndef VALUE_FROM_SSE
#define VALUE_FROM_SSE(TYPE, sse) *(TYPE *)&sse
#endif

#ifndef PTR_FROM_SSE
#define PTR_FROM_SSE(TYPE, sse) (TYPE *)&sse
#endif

//BYTE-ALIGNMENT for data allocation (16 required for SSE, 32 required for AVX)
//PREVIOUS version uses only SSE. The new version will include AVX. 
//SO the alignment is increased from 16 to 32
#define SSE_ALIGNMENT 32
#define SSE_ALIGNMENT_MASK (SSE_ALIGNMENT - 1)
#define ALIGN_PTR(p)	  (( ((uint) p) + SSE_ALIGNMENT_MASK)  & (~SSE_ALIGNMENT_MASK))


class SSE
{
public:

	static inline void Set01(const float &f0, const float &f1, ENFT_SSE::__m128 &sse)
	{
		sse.m128_f32[0] = f0;	sse.m128_f32[1] = f1;
	}
	static inline void Set012(const float &f0, const float &f1, const float &f2, ENFT_SSE::__m128 &sse)
	{
		sse.m128_f32[0] = f0;	sse.m128_f32[1] = f1;	sse.m128_f32[2] = f2;
	}
	static inline void Set0123(const float &f0, const float &f1, const float &f2, const float &f3, ENFT_SSE::__m128 &sse)
	{
		sse = _mm_set_ps(f3, f2, f1, f0);
	}
	static inline void Set0123(const float &f, ENFT_SSE::__m128 &sse)
	{
		sse = _mm_set_ps1(f);
	}
	static inline void Set01_23(const float &f01, const float &f23, ENFT_SSE::__m128 &sse)
	{
		sse.m128_f32[0] = sse.m128_f32[1] = f01; sse.m128_f32[2] = sse.m128_f32[3] = f23;
	}
	static inline void Get0123(const ENFT_SSE::__m128 &sse, float &f0, float &f1, float &f2, float &f3)
	{
		f0 = sse.m128_f32[0]; f1 = sse.m128_f32[1]; f2 = sse.m128_f32[2]; f3 = sse.m128_f32[3];
	}
	static inline float Sum01(const ENFT_SSE::__m128 &sse)
	{
		return sse.m128_f32[0] + sse.m128_f32[1];
	}
	static inline float Sum012(const ENFT_SSE::__m128 &sse)
	{
		return sse.m128_f32[0] + sse.m128_f32[1] + sse.m128_f32[2];
	}
	static inline float Sum0123(const ENFT_SSE::__m128 &sse)
	{
		return sse.m128_f32[0] + sse.m128_f32[2] + sse.m128_f32[1] + sse.m128_f32[3];
	}
	static inline float Sum23(const ENFT_SSE::__m128 &sse)
	{
		return sse.m128_f32[2] + sse.m128_f32[3];
	}
	static inline float Sum123(const ENFT_SSE::__m128 &sse)
	{
		return sse.m128_f32[1] + sse.m128_f32[2] + sse.m128_f32[3];
	}
	static inline void Sum0123(const ENFT_SSE::__m128 &sse, float &sum)
	{
		sum = sse.m128_f32[0] + sse.m128_f32[2] + sse.m128_f32[1] + sse.m128_f32[3];
	}
	static inline void Sum0123(const ENFT_SSE::__m128 &sse, double &sum)
	{
		sum = double(sse.m128_f32[0] + sse.m128_f32[2] + sse.m128_f32[1] + sse.m128_f32[3]);
	}
	static inline void Sum01_23(const ENFT_SSE::__m128 &sse, float &sum01, float &sum23)
	{
		sum01 = sse.m128_f32[0] + sse.m128_f32[1];
		sum23 = sse.m128_f32[2] + sse.m128_f32[3];
	}
	static inline void Sum01_23(const ENFT_SSE::__m128 &sseA, const float &B0123, float &sum01, float &sum23)
	{
		sum01 = sseA.m128_f32[0] + sseA.m128_f32[1] + B0123;
		sum23 = sseA.m128_f32[2] + sseA.m128_f32[3] + B0123;
	}
	static inline void Sum01_23(const ENFT_SSE::__m128 &sseA, const float &B01, const float &B23, float &sum01, float &sum23)
	{
		sum01 = sseA.m128_f32[0] + sseA.m128_f32[1] + B01;
		sum23 = sseA.m128_f32[2] + sseA.m128_f32[3] + B23;
	}
	static inline void Sum01_23(const ENFT_SSE::__m128 &sseA, const ENFT_SSE::__m128 &sseB, float &sum01, float &sum23)
	{
		sum01 = sseA.m128_f32[0] + sseA.m128_f32[1] + sseB.m128_f32[0] + sseB.m128_f32[1];
		sum23 = sseA.m128_f32[2] + sseA.m128_f32[3] + sseB.m128_f32[2] + sseB.m128_f32[3];
	}
	static inline void Sum02_13(const ENFT_SSE::__m128 &sse, float &sum02, float &sum13)
	{
		sum02 = sse.m128_f32[0] + sse.m128_f32[2];
		sum13 = sse.m128_f32[1] + sse.m128_f32[3];
	}
	static inline void Sum012_0123(const ENFT_SSE::__m128 &sse, float &sum012, float &sum0123)
	{
		sum012 = sse.m128_f32[0] + sse.m128_f32[1] + sse.m128_f32[2];
		sum0123 = sum012 + sse.m128_f32[3];
	}
	static inline void Sum012_3(const ENFT_SSE::__m128 &sse, float &sum012, float &val3)
	{
		sum012 = sse.m128_f32[0] + sse.m128_f32[1] + sse.m128_f32[2];
		val3 = sse.m128_f32[3];
	}
	static inline void Minus01_23(const float &A01, const float &A23, const ENFT_SSE::__m128 &sseB, float &dif01, float &dif23)
	{
		dif01 = A01 - (sseB.m128_f32[0] + sseB.m128_f32[1]);
		dif23 = A23 - (sseB.m128_f32[2] + sseB.m128_f32[3]);
	}
	static inline void Add01_23To(const ENFT_SSE::__m128 &sse, float &to01, float &to23)
	{
		to01 += sse.m128_f32[0] + sse.m128_f32[1];
		to23 += sse.m128_f32[2] + sse.m128_f32[3];
	}
	static inline void Subtract01_23From(const ENFT_SSE::__m128 &sse, float &from01, float &from23)
	{
		from01 -= sse.m128_f32[0] + sse.m128_f32[1];
		from23 -= sse.m128_f32[2] + sse.m128_f32[3];
	}
	static inline void Print(const ENFT_SSE::__m128 &sse)
	{
		printf("(%f, %f, %f, %f)\n", sse.m128_f32[0], sse.m128_f32[1], sse.m128_f32[2], sse.m128_f32[3]);
	}

	// static inline int SumU16(const ENFT_SSE::__m128i &sse)
	// {
	// 	return sse.m128i_u16[0] + sse.m128i_u16[1] + sse.m128i_u16[2] + sse.m128i_u16[3] + sse.m128i_u16[4] + sse.m128i_u16[5] + sse.m128i_u16[6] + sse.m128i_u16[7];
	// }
	// static inline int SumU32(const ENFT_SSE::__m128i &sse)
	// {
	// 	return sse.m128i_u32[0] + sse.m128i_u32[1] + sse.m128i_u32[2] + sse.m128i_u32[3];
	// }
	// static inline int SumI32(const ENFT_SSE::__m128i &sse)
	// {
	// 	return sse.m128i_i32[0] + sse.m128i_i32[1] + sse.m128i_i32[2] + sse.m128i_i32[3];
	// }

};

#ifdef __LINUX__
}  // namespace ENFT_SSE
#endif

#endif
