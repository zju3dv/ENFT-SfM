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

#ifndef _MATRIX_11_H_
#define _MATRIX_11_H_

#include "Vector11.h"
#include "Matrix3x11.h"

namespace LA
{

	class AlignedMatrix11f : public AlignedMatrix3x11f
	{

	public:

		inline operator const float* () const { return (const float *) this; }			inline operator float* () { return (float *) this; }

		inline const ENFT_SSE::__m128& M_30_31_32_33() const { return m_30_31_32_33; }			inline ENFT_SSE::__m128& M_30_31_32_33() { return m_30_31_32_33; }
		inline const ENFT_SSE::__m128& M_34_35_36_37() const { return m_34_35_36_37; }			inline ENFT_SSE::__m128& M_34_35_36_37() { return m_34_35_36_37; }
		inline const ENFT_SSE::__m128& M_38_39_310_x() const { return m_38_39_310_x; }			inline ENFT_SSE::__m128& M_38_39_310_x() { return m_38_39_310_x; }

		inline const ENFT_SSE::__m128& M_40_41_42_43() const { return m_40_41_42_43; }			inline ENFT_SSE::__m128& M_40_41_42_43() { return m_40_41_42_43; }
		inline const ENFT_SSE::__m128& M_44_45_46_47() const { return m_44_45_46_47; }			inline ENFT_SSE::__m128& M_44_45_46_47() { return m_44_45_46_47; }
		inline const ENFT_SSE::__m128& M_48_49_410_x() const { return m_48_49_410_x; }			inline ENFT_SSE::__m128& M_48_49_410_x() { return m_48_49_410_x; }

		inline const ENFT_SSE::__m128& M_50_51_52_53() const { return m_50_51_52_53; }			inline ENFT_SSE::__m128& M_50_51_52_53() { return m_50_51_52_53; }
		inline const ENFT_SSE::__m128& M_54_55_56_57() const { return m_54_55_56_57; }			inline ENFT_SSE::__m128& M_54_55_56_57() { return m_54_55_56_57; }
		inline const ENFT_SSE::__m128& M_58_59_510_x() const { return m_58_59_510_x; }			inline ENFT_SSE::__m128& M_58_59_510_x() { return m_58_59_510_x; }

		inline const ENFT_SSE::__m128& M_60_61_62_63() const { return m_60_61_62_63; }			inline ENFT_SSE::__m128& M_60_61_62_63() { return m_60_61_62_63; }
		inline const ENFT_SSE::__m128& M_64_65_66_67() const { return m_64_65_66_67; }			inline ENFT_SSE::__m128& M_64_65_66_67() { return m_64_65_66_67; }
		inline const ENFT_SSE::__m128& M_68_69_610_x() const { return m_68_69_610_x; }			inline ENFT_SSE::__m128& M_68_69_610_x() { return m_68_69_610_x; }

		inline const ENFT_SSE::__m128& M_70_71_72_73() const { return m_70_71_72_73; }			inline ENFT_SSE::__m128& M_70_71_72_73() { return m_70_71_72_73; }
		inline const ENFT_SSE::__m128& M_74_75_76_77() const { return m_74_75_76_77; }			inline ENFT_SSE::__m128& M_74_75_76_77() { return m_74_75_76_77; }
		inline const ENFT_SSE::__m128& M_78_79_710_x() const { return m_78_79_710_x; }			inline ENFT_SSE::__m128& M_78_79_710_x() { return m_78_79_710_x; }

		inline const ENFT_SSE::__m128& M_80_81_82_83() const { return m_80_81_82_83; }			inline ENFT_SSE::__m128& M_80_81_82_83() { return m_80_81_82_83; }
		inline const ENFT_SSE::__m128& M_84_85_86_87() const { return m_84_85_86_87; }			inline ENFT_SSE::__m128& M_84_85_86_87() { return m_84_85_86_87; }
		inline const ENFT_SSE::__m128& M_88_89_810_x() const { return m_88_89_810_x; }			inline ENFT_SSE::__m128& M_88_89_810_x() { return m_88_89_810_x; }

		inline const ENFT_SSE::__m128& M_90_91_92_93() const { return m_90_91_92_93; }			inline ENFT_SSE::__m128& M_90_91_92_93() { return m_90_91_92_93; }
		inline const ENFT_SSE::__m128& M_94_95_96_97() const { return m_94_95_96_97; }			inline ENFT_SSE::__m128& M_94_95_96_97() { return m_94_95_96_97; }
		inline const ENFT_SSE::__m128& M_98_99_910_x() const { return m_98_99_910_x; }			inline ENFT_SSE::__m128& M_98_99_910_x() { return m_98_99_910_x; }

		inline const ENFT_SSE::__m128& M_100_101_102_103() const { return m_100_101_102_103; }	inline ENFT_SSE::__m128& M_100_101_102_103() { return m_100_101_102_103; }
		inline const ENFT_SSE::__m128& M_104_105_106_107() const { return m_104_105_106_107; }	inline ENFT_SSE::__m128& M_104_105_106_107() { return m_104_105_106_107; }
		inline const ENFT_SSE::__m128& M_108_109_1010_x() const { return m_108_109_1010_x; }		inline ENFT_SSE::__m128& M_108_109_1010_x() { return m_108_109_1010_x; }

		inline const float& M30() const { return m_30_31_32_33.m128_f32[0]; }			inline float& M30() { return m_30_31_32_33.m128_f32[0]; }
		inline const float& M31() const { return m_30_31_32_33.m128_f32[1]; }			inline float& M31() { return m_30_31_32_33.m128_f32[1]; }
		inline const float& M32() const { return m_30_31_32_33.m128_f32[2]; }			inline float& M32() { return m_30_31_32_33.m128_f32[2]; }
		inline const float& M33() const { return m_30_31_32_33.m128_f32[3]; }			inline float& M33() { return m_30_31_32_33.m128_f32[3]; }
		inline const float& M34() const { return m_34_35_36_37.m128_f32[0]; }			inline float& M34() { return m_34_35_36_37.m128_f32[0]; }
		inline const float& M35() const { return m_34_35_36_37.m128_f32[1]; }			inline float& M35() { return m_34_35_36_37.m128_f32[1]; }
		inline const float& M36() const { return m_34_35_36_37.m128_f32[2]; }			inline float& M36() { return m_34_35_36_37.m128_f32[2]; }
		inline const float& M37() const { return m_34_35_36_37.m128_f32[3]; }			inline float& M37() { return m_34_35_36_37.m128_f32[3]; }
		inline const float& M38() const { return m_38_39_310_x.m128_f32[0]; }			inline float& M38() { return m_38_39_310_x.m128_f32[0]; }
		inline const float& M39() const { return m_38_39_310_x.m128_f32[1]; }			inline float& M39() { return m_38_39_310_x.m128_f32[1]; }
		inline const float& M310() const { return m_38_39_310_x.m128_f32[2]; }			inline float& M310() { return m_38_39_310_x.m128_f32[2]; }

		inline const float& M40() const { return m_40_41_42_43.m128_f32[0]; }			inline float& M40() { return m_40_41_42_43.m128_f32[0]; }
		inline const float& M41() const { return m_40_41_42_43.m128_f32[1]; }			inline float& M41() { return m_40_41_42_43.m128_f32[1]; }
		inline const float& M42() const { return m_40_41_42_43.m128_f32[2]; }			inline float& M42() { return m_40_41_42_43.m128_f32[2]; }
		inline const float& M43() const { return m_40_41_42_43.m128_f32[3]; }			inline float& M43() { return m_40_41_42_43.m128_f32[3]; }
		inline const float& M44() const { return m_44_45_46_47.m128_f32[0]; }			inline float& M44() { return m_44_45_46_47.m128_f32[0]; }
		inline const float& M45() const { return m_44_45_46_47.m128_f32[1]; }			inline float& M45() { return m_44_45_46_47.m128_f32[1]; }
		inline const float& M46() const { return m_44_45_46_47.m128_f32[2]; }			inline float& M46() { return m_44_45_46_47.m128_f32[2]; }
		inline const float& M47() const { return m_44_45_46_47.m128_f32[3]; }			inline float& M47() { return m_44_45_46_47.m128_f32[3]; }
		inline const float& M48() const { return m_48_49_410_x.m128_f32[0]; }			inline float& M48() { return m_48_49_410_x.m128_f32[0]; }
		inline const float& M49() const { return m_48_49_410_x.m128_f32[1]; }			inline float& M49() { return m_48_49_410_x.m128_f32[1]; }
		inline const float& M410() const { return m_48_49_410_x.m128_f32[2]; }			inline float& M410() { return m_48_49_410_x.m128_f32[2]; }

		inline const float& M50() const { return m_50_51_52_53.m128_f32[0]; }			inline float& M50() { return m_50_51_52_53.m128_f32[0]; }
		inline const float& M51() const { return m_50_51_52_53.m128_f32[1]; }			inline float& M51() { return m_50_51_52_53.m128_f32[1]; }
		inline const float& M52() const { return m_50_51_52_53.m128_f32[2]; }			inline float& M52() { return m_50_51_52_53.m128_f32[2]; }
		inline const float& M53() const { return m_50_51_52_53.m128_f32[3]; }			inline float& M53() { return m_50_51_52_53.m128_f32[3]; }
		inline const float& M54() const { return m_54_55_56_57.m128_f32[0]; }			inline float& M54() { return m_54_55_56_57.m128_f32[0]; }
		inline const float& M55() const { return m_54_55_56_57.m128_f32[1]; }			inline float& M55() { return m_54_55_56_57.m128_f32[1]; }
		inline const float& M56() const { return m_54_55_56_57.m128_f32[2]; }			inline float& M56() { return m_54_55_56_57.m128_f32[2]; }
		inline const float& M57() const { return m_54_55_56_57.m128_f32[3]; }			inline float& M57() { return m_54_55_56_57.m128_f32[3]; }
		inline const float& M58() const { return m_58_59_510_x.m128_f32[0]; }			inline float& M58() { return m_58_59_510_x.m128_f32[0]; }
		inline const float& M59() const { return m_58_59_510_x.m128_f32[1]; }			inline float& M59() { return m_58_59_510_x.m128_f32[1]; }
		inline const float& M510() const { return m_58_59_510_x.m128_f32[2]; }			inline float& M510() { return m_58_59_510_x.m128_f32[2]; }

		inline const float& M60() const { return m_60_61_62_63.m128_f32[0]; }			inline float& M60() { return m_60_61_62_63.m128_f32[0]; }
		inline const float& M61() const { return m_60_61_62_63.m128_f32[1]; }			inline float& M61() { return m_60_61_62_63.m128_f32[1]; }
		inline const float& M62() const { return m_60_61_62_63.m128_f32[2]; }			inline float& M62() { return m_60_61_62_63.m128_f32[2]; }
		inline const float& M63() const { return m_60_61_62_63.m128_f32[3]; }			inline float& M63() { return m_60_61_62_63.m128_f32[3]; }
		inline const float& M64() const { return m_64_65_66_67.m128_f32[0]; }			inline float& M64() { return m_64_65_66_67.m128_f32[0]; }
		inline const float& M65() const { return m_64_65_66_67.m128_f32[1]; }			inline float& M65() { return m_64_65_66_67.m128_f32[1]; }
		inline const float& M66() const { return m_64_65_66_67.m128_f32[2]; }			inline float& M66() { return m_64_65_66_67.m128_f32[2]; }
		inline const float& M67() const { return m_64_65_66_67.m128_f32[3]; }			inline float& M67() { return m_64_65_66_67.m128_f32[3]; }
		inline const float& M68() const { return m_68_69_610_x.m128_f32[0]; }			inline float& M68() { return m_68_69_610_x.m128_f32[0]; }
		inline const float& M69() const { return m_68_69_610_x.m128_f32[1]; }			inline float& M69() { return m_68_69_610_x.m128_f32[1]; }
		inline const float& M610() const { return m_68_69_610_x.m128_f32[2]; }			inline float& M610() { return m_68_69_610_x.m128_f32[2]; }

		inline const float& M70() const { return m_70_71_72_73.m128_f32[0]; }			inline float& M70() { return m_70_71_72_73.m128_f32[0]; }
		inline const float& M71() const { return m_70_71_72_73.m128_f32[1]; }			inline float& M71() { return m_70_71_72_73.m128_f32[1]; }
		inline const float& M72() const { return m_70_71_72_73.m128_f32[2]; }			inline float& M72() { return m_70_71_72_73.m128_f32[2]; }
		inline const float& M73() const { return m_70_71_72_73.m128_f32[3]; }			inline float& M73() { return m_70_71_72_73.m128_f32[3]; }
		inline const float& M74() const { return m_74_75_76_77.m128_f32[0]; }			inline float& M74() { return m_74_75_76_77.m128_f32[0]; }
		inline const float& M75() const { return m_74_75_76_77.m128_f32[1]; }			inline float& M75() { return m_74_75_76_77.m128_f32[1]; }
		inline const float& M76() const { return m_74_75_76_77.m128_f32[2]; }			inline float& M76() { return m_74_75_76_77.m128_f32[2]; }
		inline const float& M77() const { return m_74_75_76_77.m128_f32[3]; }			inline float& M77() { return m_74_75_76_77.m128_f32[3]; }
		inline const float& M78() const { return m_78_79_710_x.m128_f32[0]; }			inline float& M78() { return m_78_79_710_x.m128_f32[0]; }
		inline const float& M79() const { return m_78_79_710_x.m128_f32[1]; }			inline float& M79() { return m_78_79_710_x.m128_f32[1]; }
		inline const float& M710() const { return m_78_79_710_x.m128_f32[2]; }			inline float& M710() { return m_78_79_710_x.m128_f32[2]; }

		inline const float& M80() const { return m_80_81_82_83.m128_f32[0]; }			inline float& M80() { return m_80_81_82_83.m128_f32[0]; }
		inline const float& M81() const { return m_80_81_82_83.m128_f32[1]; }			inline float& M81() { return m_80_81_82_83.m128_f32[1]; }
		inline const float& M82() const { return m_80_81_82_83.m128_f32[2]; }			inline float& M82() { return m_80_81_82_83.m128_f32[2]; }
		inline const float& M83() const { return m_80_81_82_83.m128_f32[3]; }			inline float& M83() { return m_80_81_82_83.m128_f32[3]; }
		inline const float& M84() const { return m_84_85_86_87.m128_f32[0]; }			inline float& M84() { return m_84_85_86_87.m128_f32[0]; }
		inline const float& M85() const { return m_84_85_86_87.m128_f32[1]; }			inline float& M85() { return m_84_85_86_87.m128_f32[1]; }
		inline const float& M86() const { return m_84_85_86_87.m128_f32[2]; }			inline float& M86() { return m_84_85_86_87.m128_f32[2]; }
		inline const float& M87() const { return m_84_85_86_87.m128_f32[3]; }			inline float& M87() { return m_84_85_86_87.m128_f32[3]; }
		inline const float& M88() const { return m_88_89_810_x.m128_f32[0]; }			inline float& M88() { return m_88_89_810_x.m128_f32[0]; }
		inline const float& M89() const { return m_88_89_810_x.m128_f32[1]; }			inline float& M89() { return m_88_89_810_x.m128_f32[1]; }
		inline const float& M810() const { return m_88_89_810_x.m128_f32[2]; }			inline float& M810() { return m_88_89_810_x.m128_f32[2]; }

		inline const float& M90() const { return m_90_91_92_93.m128_f32[0]; }			inline float& M90() { return m_90_91_92_93.m128_f32[0]; }
		inline const float& M91() const { return m_90_91_92_93.m128_f32[1]; }			inline float& M91() { return m_90_91_92_93.m128_f32[1]; }
		inline const float& M92() const { return m_90_91_92_93.m128_f32[2]; }			inline float& M92() { return m_90_91_92_93.m128_f32[2]; }
		inline const float& M93() const { return m_90_91_92_93.m128_f32[3]; }			inline float& M93() { return m_90_91_92_93.m128_f32[3]; }
		inline const float& M94() const { return m_94_95_96_97.m128_f32[0]; }			inline float& M94() { return m_94_95_96_97.m128_f32[0]; }
		inline const float& M95() const { return m_94_95_96_97.m128_f32[1]; }			inline float& M95() { return m_94_95_96_97.m128_f32[1]; }
		inline const float& M96() const { return m_94_95_96_97.m128_f32[2]; }			inline float& M96() { return m_94_95_96_97.m128_f32[2]; }
		inline const float& M97() const { return m_94_95_96_97.m128_f32[3]; }			inline float& M97() { return m_94_95_96_97.m128_f32[3]; }
		inline const float& M98() const { return m_98_99_910_x.m128_f32[0]; }			inline float& M98() { return m_98_99_910_x.m128_f32[0]; }
		inline const float& M99() const { return m_98_99_910_x.m128_f32[1]; }			inline float& M99() { return m_98_99_910_x.m128_f32[1]; }
		inline const float& M910() const { return m_98_99_910_x.m128_f32[2]; }			inline float& M910() { return m_98_99_910_x.m128_f32[2]; }

		inline const float& M100() const { return m_100_101_102_103.m128_f32[0]; }		inline float& M100() { return m_100_101_102_103.m128_f32[0]; }
		inline const float& M101() const { return m_100_101_102_103.m128_f32[1]; }		inline float& M101() { return m_100_101_102_103.m128_f32[1]; }
		inline const float& M102() const { return m_100_101_102_103.m128_f32[2]; }		inline float& M102() { return m_100_101_102_103.m128_f32[2]; }
		inline const float& M103() const { return m_100_101_102_103.m128_f32[3]; }		inline float& M103() { return m_100_101_102_103.m128_f32[3]; }
		inline const float& M104() const { return m_104_105_106_107.m128_f32[0]; }		inline float& M104() { return m_104_105_106_107.m128_f32[0]; }
		inline const float& M105() const { return m_104_105_106_107.m128_f32[1]; }		inline float& M105() { return m_104_105_106_107.m128_f32[1]; }
		inline const float& M106() const { return m_104_105_106_107.m128_f32[2]; }		inline float& M106() { return m_104_105_106_107.m128_f32[2]; }
		inline const float& M107() const { return m_104_105_106_107.m128_f32[3]; }		inline float& M107() { return m_104_105_106_107.m128_f32[3]; }
		inline const float& M108() const { return m_108_109_1010_x.m128_f32[0]; }		inline float& M108() { return m_108_109_1010_x.m128_f32[0]; }
		inline const float& M109() const { return m_108_109_1010_x.m128_f32[1]; }		inline float& M109() { return m_108_109_1010_x.m128_f32[1]; }
		inline const float& M1010() const { return m_108_109_1010_x.m128_f32[2]; }		inline float& M1010() { return m_108_109_1010_x.m128_f32[2]; }

		inline void SetZero() { memset(this, 0, sizeof(AlignedMatrix11f)); }
		inline void GetDiagonal(AlignedVector11f &d) const
		{
			d.v0() = M00();		d.v1() = M11();		d.v2() = M22();		d.v3() = M33();		d.v4() = M44();		d.v5() = M55();
			d.v6() = M66();		d.v7() = M77();		d.v8() = M88();		d.v9() = M99();		d.v10() = M1010();
		}
		inline void SetDiagonal(const AlignedVector11f &d)
		{
			M00() = d.v0();		M11() = d.v1();		M22() = d.v2();		M33() = d.v3();		M44() = d.v4();		M55() = d.v5();
			M66() = d.v6();		M77() = d.v7();		M88() = d.v8();		M99() = d.v9();		M1010() = d.v10();
		}
		inline void ScaleDiagonal(const float &lambda)
		{
			M00() *= lambda;	M11() *= lambda;	M22() *= lambda;	M33() *= lambda;	M44() *= lambda;	M55() *= lambda;
			M66() *= lambda;	M77() *= lambda;	M88() *= lambda;	M99() *= lambda;	M1010() *= lambda;
		}
		inline void IncreaseDiagonal(const float &lambda)
		{
			M00() += lambda;	M11() += lambda;	M22() += lambda;	M33() += lambda;	M44() += lambda;	M55() += lambda;
			M66() += lambda;	M77() += lambda;	M88() += lambda;	M99() += lambda;	M1010() += lambda;
		}
		inline void SetLowerFromUpper()
		{
			M10() = M01();
			M20() = M02();		M21() = M12();
			M30() = M03();		M31() = M13();		M32() = M23();
			M40() = M04();		M41() = M14();		M42() = M24();		M43() = M34();
			M50() = M05();		M51() = M15();		M52() = M25();		M53() = M35();		M54() = M45();
			M60() = M06();		M61() = M16();		M62() = M26();		M63() = M36();		M64() = M46();		M65() = M56();
			M70() = M07();		M71() = M17();		M72() = M27();		M73() = M37();		M74() = M47();		M75() = M57();		M76() = M67();
			M80() = M08();		M81() = M18();		M82() = M28();		M83() = M38();		M84() = M48();		M85() = M58();		M86() = M68();		M87() = M78();
			M90() = M09();		M91() = M19();		M92() = M29();		M93() = M39();		M94() = M49();		M95() = M59();		M96() = M69();		M97() = M79();		M98() = M89();
			M100() = M010();	M101() = M110();	M102() = M210();	M103() = M310();	M104() = M410();	M105() = M510();	M106() = M610();	M107() = M710();	M108() = M810();	M109() = M910();
		}
		inline void Print() const
		{
			const float *row = &M00();
			for(int i = 0; i < 11; ++i, row += 12)
			{
				for(int j = 0; j < 11; ++j)
					printf(" %.2f", row[j]);
				printf("\n");
			}
		}

	protected:

		ENFT_SSE::__m128 m_30_31_32_33, m_34_35_36_37, m_38_39_310_x;
		ENFT_SSE::__m128 m_40_41_42_43, m_44_45_46_47, m_48_49_410_x;
		ENFT_SSE::__m128 m_50_51_52_53, m_54_55_56_57, m_58_59_510_x;
		ENFT_SSE::__m128 m_60_61_62_63, m_64_65_66_67, m_68_69_610_x;
		ENFT_SSE::__m128 m_70_71_72_73, m_74_75_76_77, m_78_79_710_x;
		ENFT_SSE::__m128 m_80_81_82_83, m_84_85_86_87, m_88_89_810_x;
		ENFT_SSE::__m128 m_90_91_92_93, m_94_95_96_97, m_98_99_910_x;
		ENFT_SSE::__m128 m_100_101_102_103, m_104_105_106_107, m_108_109_1010_x;

	};

	bool InvertSymmetricUpper(const AlignedMatrix11f &A, AlignedMatrix11f &Ainv);
	inline bool InvertSymmetricUpper(const AlignedMatrix11f &A, AlignedMatrix11f &Ainv, float *work0) { return InvertSymmetricUpper(A, Ainv); }
	bool SolveLinearSystemSymmetricUpper(AlignedMatrix11f &A, AlignedVector11f &b);
	inline bool SolveLinearSystemSymmetricUpper(AlignedMatrix11f &A, AlignedVector11f &b, float *work0) { return SolveLinearSystemSymmetricUpper(A, b); }

	inline void AB(const AlignedMatrix11f &A, const AlignedVector11f &B, AlignedVector11f &AB)
	{
#if _DEBUG
		assert(B.reserve() == 0.0f);
#endif
		AB.v0() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), B.v8910x())));
		AB.v1() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), B.v8910x())));
		AB.v2() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_28_29_210_x(), B.v8910x())));
		AB.v3() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_34_35_36_37(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_38_39_310_x(), B.v8910x())));
		AB.v4() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_44_45_46_47(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_48_49_410_x(), B.v8910x())));
		AB.v5() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_54_55_56_57(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_58_59_510_x(), B.v8910x())));
		AB.v6() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_64_65_66_67(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_68_69_610_x(), B.v8910x())));
		AB.v7() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_70_71_72_73(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_74_75_76_77(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_78_79_710_x(), B.v8910x())));
		AB.v8() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_80_81_82_83(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_84_85_86_87(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_88_89_810_x(), B.v8910x())));
		AB.v9() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_90_91_92_93(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_94_95_96_97(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_98_99_910_x(), B.v8910x())));
		AB.v10() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_100_101_102_103(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_104_105_106_107(), B.v4567())), ENFT_SSE::_mm_mul_ps(A.M_108_109_1010_x(), B.v8910x())));
	}

	inline void SetLowerFromUpper(AlignedMatrix11f &A) { A.SetLowerFromUpper(); }
	inline void GetDiagonal(const AlignedMatrix11f &M, AlignedVector11f &d) { M.GetDiagonal(d); }
	inline void SetDiagonal(const AlignedVector11f &d, AlignedMatrix11f &M) { M.SetDiagonal(d); }
	inline void ScaleDiagonal(const float &lambda, AlignedMatrix11f &M) { M.ScaleDiagonal(lambda); }
	inline void IncreaseDiagonal(const float &lambda, AlignedMatrix11f &M) { M.IncreaseDiagonal(lambda); }

	inline void ssTA(const AlignedVector11f &s, AlignedMatrix11f &A, ENFT_SSE::__m128 &work)
	{
		work = _mm_set1_ps(s.v0());
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_00_01_02_03());
		A.M_04_05_06_07() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_04_05_06_07());
		A.M_08_09_010_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_08_09_010_x());

		work = _mm_set1_ps(s.v1());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_10_11_12_13());
		A.M_14_15_16_17() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_14_15_16_17());
		A.M_18_19_110_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_18_19_110_x());

		work = _mm_set1_ps(s.v2());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_20_21_22_23());
		A.M_24_25_26_27() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_24_25_26_27());
		A.M_28_29_210_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_28_29_210_x());

		work = _mm_set1_ps(s.v3());
		A.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_30_31_32_33());
		A.M_34_35_36_37() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_34_35_36_37());
		A.M_38_39_310_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_38_39_310_x());

		work = _mm_set1_ps(s.v4());
		A.M_40_41_42_43() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_40_41_42_43());
		A.M_44_45_46_47() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_44_45_46_47());
		A.M_48_49_410_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_48_49_410_x());

		work = _mm_set1_ps(s.v5());
		A.M_50_51_52_53() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_50_51_52_53());
		A.M_54_55_56_57() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_54_55_56_57());
		A.M_58_59_510_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_58_59_510_x());

		work = _mm_set1_ps(s.v6());
		A.M_60_61_62_63() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_60_61_62_63());
		A.M_64_65_66_67() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_64_65_66_67());
		A.M_68_69_610_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_68_69_610_x());

		work = _mm_set1_ps(s.v7());
		A.M_70_71_72_73() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_70_71_72_73());
		A.M_74_75_76_77() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_74_75_76_77());
		A.M_78_79_710_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_78_79_710_x());

		work = _mm_set1_ps(s.v8());
		A.M_80_81_82_83() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_80_81_82_83());
		A.M_84_85_86_87() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_84_85_86_87());
		A.M_88_89_810_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_88_89_810_x());

		work = _mm_set1_ps(s.v9());
		A.M_90_91_92_93() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_90_91_92_93());
		A.M_94_95_96_97() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_94_95_96_97());
		A.M_98_99_910_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_98_99_910_x());

		work = _mm_set1_ps(s.v10());
		A.M_100_101_102_103() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v0123()), A.M_100_101_102_103());
		A.M_104_105_106_107() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v4567()), A.M_104_105_106_107());
		A.M_108_109_1010_x() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(work, s.v8910x()), A.M_108_109_1010_x());
	}

	inline void SubtractATBFrom(const AlignedMatrix3x11f &A, const AlignedMatrix3x11f &B, AlignedMatrix11f &from, ENFT_SSE::__m128 *work3)
	{
		work3[0] = _mm_set1_ps(A.M00());		work3[1] = _mm_set1_ps(A.M10());	work3[2] = _mm_set1_ps(A.M20());
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_04_05_06_07() = ENFT_SSE::_mm_sub_ps(from.M_04_05_06_07(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_08_09_010_x() = ENFT_SSE::_mm_sub_ps(from.M_08_09_010_x(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));

		work3[0] = _mm_set1_ps(A.M01());		work3[1] = _mm_set1_ps(A.M11());	work3[2] = _mm_set1_ps(A.M21());
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_14_15_16_17() = ENFT_SSE::_mm_sub_ps(from.M_14_15_16_17(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_18_19_110_x() = ENFT_SSE::_mm_sub_ps(from.M_18_19_110_x(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));

		work3[0] = _mm_set1_ps(A.M02());		work3[1] = _mm_set1_ps(A.M12());	work3[2] = _mm_set1_ps(A.M22());
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_24_25_26_27() = ENFT_SSE::_mm_sub_ps(from.M_24_25_26_27(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_28_29_210_x() = ENFT_SSE::_mm_sub_ps(from.M_28_29_210_x(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));

		work3[0] = _mm_set1_ps(A.M03());		work3[1] = _mm_set1_ps(A.M13());	work3[2] = _mm_set1_ps(A.M23());
		from.M_30_31_32_33() = ENFT_SSE::_mm_sub_ps(from.M_30_31_32_33(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_34_35_36_37() = ENFT_SSE::_mm_sub_ps(from.M_34_35_36_37(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_38_39_310_x() = ENFT_SSE::_mm_sub_ps(from.M_38_39_310_x(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));

		work3[0] = _mm_set1_ps(A.M04());		work3[1] = _mm_set1_ps(A.M14());	work3[2] = _mm_set1_ps(A.M24());
		from.M_40_41_42_43() = ENFT_SSE::_mm_sub_ps(from.M_40_41_42_43(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_44_45_46_47() = ENFT_SSE::_mm_sub_ps(from.M_44_45_46_47(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_48_49_410_x() = ENFT_SSE::_mm_sub_ps(from.M_48_49_410_x(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));

		work3[0] = _mm_set1_ps(A.M05());		work3[1] = _mm_set1_ps(A.M15());	work3[2] = _mm_set1_ps(A.M25());
		from.M_50_51_52_53() = ENFT_SSE::_mm_sub_ps(from.M_50_51_52_53(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_54_55_56_57() = ENFT_SSE::_mm_sub_ps(from.M_54_55_56_57(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_58_59_510_x() = ENFT_SSE::_mm_sub_ps(from.M_58_59_510_x(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));

		work3[0] = _mm_set1_ps(A.M06());		work3[1] = _mm_set1_ps(A.M16());	work3[2] = _mm_set1_ps(A.M26());
		from.M_60_61_62_63() = ENFT_SSE::_mm_sub_ps(from.M_60_61_62_63(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_64_65_66_67() = ENFT_SSE::_mm_sub_ps(from.M_64_65_66_67(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_68_69_610_x() = ENFT_SSE::_mm_sub_ps(from.M_68_69_610_x(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));

		work3[0] = _mm_set1_ps(A.M07());		work3[1] = _mm_set1_ps(A.M17());	work3[2] = _mm_set1_ps(A.M27());
		from.M_70_71_72_73() = ENFT_SSE::_mm_sub_ps(from.M_70_71_72_73(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_74_75_76_77() = ENFT_SSE::_mm_sub_ps(from.M_74_75_76_77(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_78_79_710_x() = ENFT_SSE::_mm_sub_ps(from.M_78_79_710_x(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));

		work3[0] = _mm_set1_ps(A.M08());		work3[1] = _mm_set1_ps(A.M18());	work3[2] = _mm_set1_ps(A.M28());
		from.M_80_81_82_83() = ENFT_SSE::_mm_sub_ps(from.M_80_81_82_83(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_84_85_86_87() = ENFT_SSE::_mm_sub_ps(from.M_84_85_86_87(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_88_89_810_x() = ENFT_SSE::_mm_sub_ps(from.M_88_89_810_x(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));

		work3[0] = _mm_set1_ps(A.M09());		work3[1] = _mm_set1_ps(A.M19());	work3[2] = _mm_set1_ps(A.M29());
		from.M_90_91_92_93() = ENFT_SSE::_mm_sub_ps(from.M_90_91_92_93(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_94_95_96_97() = ENFT_SSE::_mm_sub_ps(from.M_94_95_96_97(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_98_99_910_x() = ENFT_SSE::_mm_sub_ps(from.M_98_99_910_x(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));

		work3[0] = _mm_set1_ps(A.M010());		work3[1] = _mm_set1_ps(A.M110());	work3[2] = _mm_set1_ps(A.M210());
		from.M_100_101_102_103() = ENFT_SSE::_mm_sub_ps(from.M_100_101_102_103(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_10_11_12_13())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_20_21_22_23())));
		from.M_104_105_106_107() = ENFT_SSE::_mm_sub_ps(from.M_104_105_106_107(), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_04_05_06_07()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_14_15_16_17())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_24_25_26_27())));
		from.M_108_109_1010_x () = ENFT_SSE::_mm_sub_ps(from.M_108_109_1010_x (), ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work3[1], B.M_18_19_110_x())), ENFT_SSE::_mm_mul_ps(work3[2], B.M_28_29_210_x())));
	}
	inline void AddATBTo(const AlignedMatrix11f &A, const AlignedVector11f &B, AlignedVector11f &to, ENFT_SSE::__m128 *work1)
	{
		work1[0] = _mm_set1_ps(B.v0());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), work1[0]), to.v8910x());

		work1[0] = _mm_set1_ps(B.v1());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), work1[0]), to.v8910x());

		work1[0] = _mm_set1_ps(B.v2());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_28_29_210_x(), work1[0]), to.v8910x());

		work1[0] = _mm_set1_ps(B.v3());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_34_35_36_37(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_38_39_310_x(), work1[0]), to.v8910x());

		work1[0] = _mm_set1_ps(B.v4());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_44_45_46_47(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_48_49_410_x(), work1[0]), to.v8910x());

		work1[0] = _mm_set1_ps(B.v5());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_54_55_56_57(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_58_59_510_x(), work1[0]), to.v8910x());

		work1[0] = _mm_set1_ps(B.v6());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_64_65_66_67(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_68_69_610_x(), work1[0]), to.v8910x());

		work1[0] = _mm_set1_ps(B.v7());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_70_71_72_73(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_74_75_76_77(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_78_79_710_x(), work1[0]), to.v8910x());

		work1[0] = _mm_set1_ps(B.v8());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_80_81_82_83(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_84_85_86_87(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_88_89_810_x(), work1[0]), to.v8910x());

		work1[0] = _mm_set1_ps(B.v9());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_90_91_92_93(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_94_95_96_97(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_98_99_910_x(), work1[0]), to.v8910x());

		work1[0] = _mm_set1_ps(B.v10());
		to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_100_101_102_103(), work1[0]), to.v0123());
		to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_104_105_106_107(), work1[0]), to.v4567());
		to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_108_109_1010_x(), work1[0]), to.v8910x());
	}
	inline void AddABTo(const AlignedMatrix11f &A, const AlignedVector11f &B, AlignedVector11f &to)
	{
#if _DEBUG
		assert(B.reserve() == 0);
#endif
		to.v0() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_04_05_06_07(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), B.v8910x()))) + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), B.v8910x()))) + to.v1();
		to.v2() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_24_25_26_27(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_28_29_210_x(), B.v8910x()))) + to.v2();
		to.v3() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_34_35_36_37(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_38_39_310_x(), B.v8910x()))) + to.v3();
		to.v4() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_40_41_42_43(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_44_45_46_47(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_48_49_410_x(), B.v8910x()))) + to.v4();
		to.v5() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_50_51_52_53(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_54_55_56_57(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_58_59_510_x(), B.v8910x()))) + to.v5();
		to.v6() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_60_61_62_63(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_64_65_66_67(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_68_69_610_x(), B.v8910x()))) + to.v6();
		to.v7() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_70_71_72_73(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_74_75_76_77(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_78_79_710_x(), B.v8910x()))) + to.v7();
		to.v8() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_80_81_82_83(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_84_85_86_87(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_88_89_810_x(), B.v8910x()))) + to.v8();
		to.v9() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_90_91_92_93(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_94_95_96_97(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_98_99_910_x(), B.v8910x()))) + to.v9();
		to.v10() = ENFT_SSE::SSE::Sum0123(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_100_101_102_103(), B.v0123()), ENFT_SSE::_mm_mul_ps(A.M_104_105_106_107(), B.v4567())), 
													 ENFT_SSE::_mm_mul_ps(A.M_108_109_1010_x(), B.v8910x()))) + to.v10();
	}
	inline void AddABTo(const AlignedMatrix11f &A, const AlignedVector11f &B, AlignedVector11f &to, ENFT_SSE::__m128 *work0) { AddABTo(A, B, to); }
}

#endif