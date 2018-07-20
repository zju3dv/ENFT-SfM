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

#ifndef _MATRIX_6x5_H_
#define _MATRIX_6x5_H_

#include "Matrix5x6.h"

namespace LA
{

	class AlignedMatrix6x5f : public AlignedMatrix5f
	{

	public:

		inline const ENFT_SSE::__m128& M_50_51_52_53() const { return m_50_51_52_53; }	inline ENFT_SSE::__m128& M_50_51_52_53() { return m_50_51_52_53; }
		inline const ENFT_SSE::__m128& M_54_x_x_x() const { return m_54_x_x_x; }			inline ENFT_SSE::__m128& M_54_x_x_x() { return m_54_x_x_x; }
		inline const float& M50() const { return m_50_51_52_53.m128_f32[0]; }	inline float& M50()	{ return m_50_51_52_53.m128_f32[0]; }
		inline const float& M51() const { return m_50_51_52_53.m128_f32[1]; }	inline float& M51()	{ return m_50_51_52_53.m128_f32[1]; }
		inline const float& M52() const { return m_50_51_52_53.m128_f32[2]; }	inline float& M52()	{ return m_50_51_52_53.m128_f32[2]; }
		inline const float& M53() const { return m_50_51_52_53.m128_f32[3]; }	inline float& M53() { return m_50_51_52_53.m128_f32[3]; }
		inline const float& M54() const { return m_54_x_x_x.m128_f32[0]; }		inline float& M54() { return m_54_x_x_x.m128_f32[0]; }

		inline void GetTranpose(AlignedMatrix5x6f &MT) const
		{
			MT.M_00_01_02_03() = ENFT_SSE::_mm_setr_ps(M00(), M10(), M20(), M30());	MT.M04() = M40();	MT.M05() = M50();
			MT.M_10_11_12_13() = ENFT_SSE::_mm_setr_ps(M01(), M11(), M21(), M31());	MT.M14() = M41();	MT.M15() = M51();
			MT.M_20_21_22_23() = ENFT_SSE::_mm_setr_ps(M02(), M12(), M22(), M32());	MT.M24() = M42();	MT.M25() = M52();
			MT.M_30_31_32_33() = ENFT_SSE::_mm_setr_ps(M03(), M13(), M23(), M33());	MT.M34() = M43();	MT.M35() = M53();
			MT.M_40_41_42_43() = ENFT_SSE::_mm_setr_ps(M04(), M14(), M24(), M34());	MT.M44() = M44();	MT.M45() = M54();
		}
		inline void Print() const
		{
			printf("%f %f %f %f %f\n", M00(), M01(), M02(), M03(), M04());
			printf("%f %f %f %f %f\n", M10(), M11(), M12(), M13(), M14());
			printf("%f %f %f %f %f\n", M20(), M21(), M22(), M23(), M24());
			printf("%f %f %f %f %f\n", M30(), M31(), M32(), M33(), M34());
			printf("%f %f %f %f %f\n", M40(), M41(), M42(), M43(), M44());
			printf("%f %f %f %f %f\n", M50(), M51(), M52(), M53(), M54());
		}

	protected:

		ENFT_SSE::__m128 m_50_51_52_53, m_54_x_x_x;

	};

}

#endif