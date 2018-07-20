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

#ifndef _POLYNOMIAL_H_
#define _POLYNOMIAL_H_

#include "SSE.h"

class Polynomial
{

public:

	class V3E1f
	{

	public:

		inline const float& cx() const { return m_cx_cy_cz_c.m128_f32[0]; }		inline float& cx() { return m_cx_cy_cz_c.m128_f32[0]; }
		inline const float& cy() const { return m_cx_cy_cz_c.m128_f32[1]; }		inline float& cy() { return m_cx_cy_cz_c.m128_f32[1]; }
		inline const float& cz() const { return m_cx_cy_cz_c.m128_f32[2]; }		inline float& cz() { return m_cx_cy_cz_c.m128_f32[2]; }
		inline const float& c () const { return m_cx_cy_cz_c.m128_f32[3]; }		inline float& c () { return m_cx_cy_cz_c.m128_f32[3]; }
		inline const ENFT_SSE::__m128& cx_cy_cz_c() const { return m_cx_cy_cz_c; }		inline ENFT_SSE::__m128& cx_cy_cz_c() { return m_cx_cy_cz_c; }

		inline void Set(const float &cx, const float &cy, const float &cz, const float &c) { m_cx_cy_cz_c = ENFT_SSE::_mm_set_ps(c, cz, cy, cx); }
		inline void Set(const ENFT_SSE::__m128 &cx_cy_cz_c) { m_cx_cy_cz_c = cx_cy_cz_c; }

	protected:

		ENFT_SSE::__m128 m_cx_cy_cz_c;

		friend class Polynomial;

	};

	class V3E2f
	{

	public:

		inline const float& cxx() const { return m_cxx_cxy_cxz_cx.m128_f32[0]; }	inline float& cxx() { return m_cxx_cxy_cxz_cx.m128_f32[0]; }
		inline const float& cxy() const { return m_cxx_cxy_cxz_cx.m128_f32[1]; }	inline float& cxy() { return m_cxx_cxy_cxz_cx.m128_f32[1]; }
		inline const float& cxz() const { return m_cxx_cxy_cxz_cx.m128_f32[2]; }	inline float& cxz() { return m_cxx_cxy_cxz_cx.m128_f32[2]; }
		inline const float& cx () const { return m_cxx_cxy_cxz_cx.m128_f32[3]; }	inline float& cx () { return m_cxx_cxy_cxz_cx.m128_f32[3]; }
		inline const float& cyy() const { return m_cyy_cyz_cy_czz.m128_f32[0]; }	inline float& cyy() { return m_cyy_cyz_cy_czz.m128_f32[0]; }
		inline const float& cyz() const { return m_cyy_cyz_cy_czz.m128_f32[1]; }	inline float& cyz() { return m_cyy_cyz_cy_czz.m128_f32[1]; }
		inline const float& cy () const { return m_cyy_cyz_cy_czz.m128_f32[2]; }	inline float& cy () { return m_cyy_cyz_cy_czz.m128_f32[2]; }
		inline const float& czz() const { return m_cyy_cyz_cy_czz.m128_f32[3]; }	inline float& czz() { return m_cyy_cyz_cy_czz.m128_f32[3]; }
		inline const float& cz () const { return m_x_x_cz_c1.m128_f32[2]; }			inline float& cz () { return m_x_x_cz_c1.m128_f32[2]; }
		inline const float& c  () const { return m_x_x_cz_c1.m128_f32[3]; }			inline float& c  () { return m_x_x_cz_c1.m128_f32[3]; }
		inline const ENFT_SSE::__m128 &cxx_cxy_cxz_cx() const { return m_cxx_cxy_cxz_cx; }	inline ENFT_SSE::__m128 &cxx_cxy_cxz_cx() { return m_cxx_cxy_cxz_cx; }
		inline const ENFT_SSE::__m128 &cyy_cyz_cy_czz() const { return m_cyy_cyz_cy_czz; }	inline ENFT_SSE::__m128 &cyy_cyz_cy_czz() { return m_cyy_cyz_cy_czz; }
		inline const ENFT_SSE::__m128 &x_x_cz_c	   () const { return m_x_x_cz_c1; }			inline ENFT_SSE::__m128 &x_x_cz_c		 () { return m_x_x_cz_c1; }

	protected:

		ENFT_SSE::__m128 m_cxx_cxy_cxz_cx, m_cyy_cyz_cy_czz, m_x_x_cz_c1;

		friend class Polynomial;
	};

	class V3E3f
	{

	public:

		inline const float& cxxx() const { return m_cxxx_cxxy_cxxz_cxx.m128_f32[0]; }		inline float& cxxx() { return m_cxxx_cxxy_cxxz_cxx.m128_f32[0]; }
		inline const float& cxxy() const { return m_cxxx_cxxy_cxxz_cxx.m128_f32[1]; }		inline float& cxxy() { return m_cxxx_cxxy_cxxz_cxx.m128_f32[1]; }
		inline const float& cxxz() const { return m_cxxx_cxxy_cxxz_cxx.m128_f32[2]; }		inline float& cxxz() { return m_cxxx_cxxy_cxxz_cxx.m128_f32[2]; }
		inline const float& cxx () const { return m_cxxx_cxxy_cxxz_cxx.m128_f32[3]; }		inline float& cxx () { return m_cxxx_cxxy_cxxz_cxx.m128_f32[3]; }
		inline const float& cxyy() const { return m_cxyy_cxyz_cxy_cxzz.m128_f32[0]; }		inline float& cxyy() { return m_cxyy_cxyz_cxy_cxzz.m128_f32[0]; }
		inline const float& cxyz() const { return m_cxyy_cxyz_cxy_cxzz.m128_f32[1]; }		inline float& cxyz() { return m_cxyy_cxyz_cxy_cxzz.m128_f32[1]; }
		inline const float& cxy () const { return m_cxyy_cxyz_cxy_cxzz.m128_f32[2]; }		inline float& cxy () { return m_cxyy_cxyz_cxy_cxzz.m128_f32[2]; }
		inline const float& cxzz() const { return m_cxyy_cxyz_cxy_cxzz.m128_f32[3]; }		inline float& cxzz() { return m_cxyy_cxyz_cxy_cxzz.m128_f32[3]; }
		inline const float& cxz () const { return m_cxz_cx_cyyy_cyyz.m128_f32[0];	}		inline float& cxz () { return m_cxz_cx_cyyy_cyyz.m128_f32[0];	}
		inline const float& cx  () const { return m_cxz_cx_cyyy_cyyz.m128_f32[1];	}		inline float& cx  () { return m_cxz_cx_cyyy_cyyz.m128_f32[1];	}
		inline const float& cyyy() const { return m_cxz_cx_cyyy_cyyz.m128_f32[2];	}		inline float& cyyy() { return m_cxz_cx_cyyy_cyyz.m128_f32[2];	}
		inline const float& cyyz() const { return m_cxz_cx_cyyy_cyyz.m128_f32[3];	}		inline float& cyyz() { return m_cxz_cx_cyyy_cyyz.m128_f32[3];	}
		inline const float& cyy () const { return m_cyy_cyzz_cyz_cy.m128_f32[0];	}		inline float& cyy () { return m_cyy_cyzz_cyz_cy.m128_f32[0];	}
		inline const float& cyzz() const { return m_cyy_cyzz_cyz_cy.m128_f32[1];	}		inline float& cyzz() { return m_cyy_cyzz_cyz_cy.m128_f32[1];	}
		inline const float& cyz () const { return m_cyy_cyzz_cyz_cy.m128_f32[2];	}		inline float& cyz () { return m_cyy_cyzz_cyz_cy.m128_f32[2];	}
		inline const float& cy  () const { return m_cyy_cyzz_cyz_cy.m128_f32[3];	}		inline float& cy  () { return m_cyy_cyzz_cyz_cy.m128_f32[3];	}
		inline const float& czzz() const { return m_czzz_czz_cz_c.m128_f32[0];		}		inline float& czzz() { return m_czzz_czz_cz_c.m128_f32[0];		}
		inline const float& czz () const { return m_czzz_czz_cz_c.m128_f32[1];		}		inline float& czz () { return m_czzz_czz_cz_c.m128_f32[1];		}
		inline const float& cz  () const { return m_czzz_czz_cz_c.m128_f32[2];		}		inline float& cz  () { return m_czzz_czz_cz_c.m128_f32[2];		}
		inline const float& c   () const { return m_czzz_czz_cz_c.m128_f32[3];		}		inline float& c   () { return m_czzz_czz_cz_c.m128_f32[3];		}
		inline const ENFT_SSE::__m128& cxxx_cxxy_cxxz_cxx() const { return m_cxxx_cxxy_cxxz_cxx; }	inline ENFT_SSE::__m128& cxxx_cxxy_cxxz_cxx() { return m_cxxx_cxxy_cxxz_cxx; }
		inline const ENFT_SSE::__m128& cxyy_cxyz_cxy_cxzz() const { return m_cxyy_cxyz_cxy_cxzz; }	inline ENFT_SSE::__m128& cxyy_cxyz_cxy_cxzz() { return m_cxyy_cxyz_cxy_cxzz; }
		inline const ENFT_SSE::__m128& cxz_cx_cyyy_cyyz  () const { return m_cxz_cx_cyyy_cyyz;   }	inline ENFT_SSE::__m128& cxz_cx_cyyy_cyyz	 () { return m_cxz_cx_cyyy_cyyz;   }
		inline const ENFT_SSE::__m128& cyy_cyzz_cyz_cy   () const { return m_cyy_cyzz_cyz_cy;    }	inline ENFT_SSE::__m128& cyy_cyzz_cyz_cy	 () { return m_cyy_cyzz_cyz_cy;	   }
		inline const ENFT_SSE::__m128& czzz_czz_cz_c	   () const { return m_czzz_czz_cz_c;	   }	inline ENFT_SSE::__m128& czzz_czz_cz_c	 () { return m_czzz_czz_cz_c;	   }

	protected:

		ENFT_SSE::__m128 m_cxxx_cxxy_cxxz_cxx, m_cxyy_cxyz_cxy_cxzz, m_cxz_cx_cyyy_cyyz, m_cyy_cyzz_cyz_cy, m_czzz_czz_cz_c;

		friend class Polynomial;

	};

	class V3E3fPermutation
	{

	public:

		inline const float& v0 () const { return m_sse0.m128_f32[0]; }		inline float& v0 () { return m_sse0.m128_f32[0]; }
		inline const float& v1 () const { return m_sse0.m128_f32[1]; }		inline float& v1 () { return m_sse0.m128_f32[1]; }
		inline const float& v2 () const { return m_sse0.m128_f32[2]; }		inline float& v2 () { return m_sse0.m128_f32[2]; }
		inline const float& v3 () const { return m_sse0.m128_f32[3]; }		inline float& v3 () { return m_sse0.m128_f32[3]; }
		inline const float& v4 () const { return m_sse1.m128_f32[0]; }		inline float& v4 () { return m_sse1.m128_f32[0]; }
		inline const float& v5 () const { return m_sse1.m128_f32[1]; }		inline float& v5 () { return m_sse1.m128_f32[1]; }
		inline const float& v6 () const { return m_sse1.m128_f32[2]; }		inline float& v6 () { return m_sse1.m128_f32[2]; }
		inline const float& v7 () const { return m_sse1.m128_f32[3]; }		inline float& v7 () { return m_sse1.m128_f32[3]; }
		inline const float& v8 () const { return m_sse2.m128_f32[0]; }		inline float& v8 () { return m_sse2.m128_f32[0]; }
		inline const float& v9 () const { return m_sse2.m128_f32[1]; }		inline float& v9 () { return m_sse2.m128_f32[1]; }
		inline const float& v10() const { return m_sse2.m128_f32[2]; }		inline float& v10() { return m_sse2.m128_f32[2]; }
		inline const float& v11() const { return m_sse2.m128_f32[3]; }		inline float& v11() { return m_sse2.m128_f32[3]; }
		inline const float& v12() const { return m_sse3.m128_f32[0]; }		inline float& v12() { return m_sse3.m128_f32[0]; }
		inline const float& v13() const { return m_sse3.m128_f32[1]; }		inline float& v13() { return m_sse3.m128_f32[1]; }
		inline const float& v14() const { return m_sse3.m128_f32[2]; }		inline float& v14() { return m_sse3.m128_f32[2]; }
		inline const float& v15() const { return m_sse3.m128_f32[3]; }		inline float& v15() { return m_sse3.m128_f32[3]; }
		inline const float& v16() const { return m_sse4.m128_f32[0]; }		inline float& v16() { return m_sse4.m128_f32[0]; }
		inline const float& v17() const { return m_sse4.m128_f32[1]; }		inline float& v17() { return m_sse4.m128_f32[1]; }
		inline const float& v18() const { return m_sse4.m128_f32[2]; }		inline float& v18() { return m_sse4.m128_f32[2]; }
		inline const float& v19() const { return m_sse4.m128_f32[3]; }		inline float& v19() { return m_sse4.m128_f32[3]; }
		inline const ENFT_SSE::__m128& sse0() const { return m_sse0; }				inline ENFT_SSE::__m128& sse0() { return m_sse0; }
		inline const ENFT_SSE::__m128& sse1() const { return m_sse1; }				inline ENFT_SSE::__m128& sse1() { return m_sse1; }
		inline const ENFT_SSE::__m128& sse2() const { return m_sse2; }				inline ENFT_SSE::__m128& sse2() { return m_sse2; }
		inline const ENFT_SSE::__m128& sse3() const { return m_sse3; }				inline ENFT_SSE::__m128& sse3() { return m_sse3; }
		inline const ENFT_SSE::__m128& sse4() const { return m_sse4; }				inline ENFT_SSE::__m128& sse4() { return m_sse4; }

	protected:

		//ENFT_SSE::__m128 m_sseCoef_x3_x2y_xy2_y3, m_sseCoef_x2z_xyz_y2z_xz2, m_sseCoef_yz2_z3_x2_xy, m_sseCoef_y2_xz_yz_z2, m_cx_cy_cz_c;
		ENFT_SSE::__m128 m_sse0, m_sse1, m_sse2, m_sse3, m_sse4;

		friend class Polynomial;

	};

	static inline void ApB(const V3E3f &A, const V3E3f &B, V3E3f &ApB)
	{
		ApB.cxxx_cxxy_cxxz_cxx() = ENFT_SSE::_mm_add_ps(A.cxxx_cxxy_cxxz_cxx(), B.cxxx_cxxy_cxxz_cxx());
		ApB.cxyy_cxyz_cxy_cxzz() = ENFT_SSE::_mm_add_ps(A.cxyy_cxyz_cxy_cxzz(), B.cxyy_cxyz_cxy_cxzz());
		ApB.cxz_cx_cyyy_cyyz  () = ENFT_SSE::_mm_add_ps(A.cxz_cx_cyyy_cyyz  (), B.cxz_cx_cyyy_cyyz  ());
		ApB.cyy_cyzz_cyz_cy	  () = ENFT_SSE::_mm_add_ps(A.cyy_cyzz_cyz_cy	  (), B.cyy_cyzz_cyz_cy	  ());
		ApB.czzz_czz_cz_c	  () = ENFT_SSE::_mm_add_ps(A.czzz_czz_cz_c	  (), B.czzz_czz_cz_c	  ());
	}
	static inline void ApBpC(const V3E2f &A, const V3E2f &B, const V3E2f &C, V3E2f &ApBpC)
	{
		ApBpC.cxx_cxy_cxz_cx() = ENFT_SSE::_mm_add_ps(A.cxx_cxy_cxz_cx(), ENFT_SSE::_mm_add_ps(B.cxx_cxy_cxz_cx(), C.cxx_cxy_cxz_cx()));
		ApBpC.cyy_cyz_cy_czz() = ENFT_SSE::_mm_add_ps(A.cyy_cyz_cy_czz(), ENFT_SSE::_mm_add_ps(B.cyy_cyz_cy_czz(), C.cyy_cyz_cy_czz()));
		ApBpC.cz() = A.cz() + B.cz() + C.cz();
		ApBpC.c () = A.c () + B.c () + C.c ();
	}
	static inline void ApBpC(const V3E3f &A, const V3E3f &B, const V3E3f &C, V3E3f &ApBpC)
	{
		ApBpC.cxxx_cxxy_cxxz_cxx() = ENFT_SSE::_mm_add_ps(A.cxxx_cxxy_cxxz_cxx(), ENFT_SSE::_mm_add_ps(B.cxxx_cxxy_cxxz_cxx(), C.cxxx_cxxy_cxxz_cxx()));
		ApBpC.cxyy_cxyz_cxy_cxzz() = ENFT_SSE::_mm_add_ps(A.cxyy_cxyz_cxy_cxzz(), ENFT_SSE::_mm_add_ps(B.cxyy_cxyz_cxy_cxzz(), C.cxyy_cxyz_cxy_cxzz()));
		ApBpC.cxz_cx_cyyy_cyyz	() = ENFT_SSE::_mm_add_ps(A.cxz_cx_cyyy_cyyz	(), ENFT_SSE::_mm_add_ps(B.cxz_cx_cyyy_cyyz  (), C.cxz_cx_cyyy_cyyz  ()));
		ApBpC.cyy_cyzz_cyz_cy	() = ENFT_SSE::_mm_add_ps(A.cyy_cyzz_cyz_cy	(), ENFT_SSE::_mm_add_ps(B.cyy_cyzz_cyz_cy   (), C.cyy_cyzz_cyz_cy   ()));
		ApBpC.czzz_czz_cz_c		() = ENFT_SSE::_mm_add_ps(A.czzz_czz_cz_c		(), ENFT_SSE::_mm_add_ps(B.czzz_czz_cz_c	   (), C.czzz_czz_cz_c	   ()));
	}
	static inline void AmB(const V3E2f &A, const V3E2f &B, V3E2f &AmB)
	{
		AmB.cxx_cxy_cxz_cx() = ENFT_SSE::_mm_sub_ps(A.cxx_cxy_cxz_cx(), B.cxx_cxy_cxz_cx());
		AmB.cyy_cyz_cy_czz() = ENFT_SSE::_mm_sub_ps(A.cyy_cyz_cy_czz(), B.cyy_cyz_cy_czz());
		AmB.cz() = A.cz() - B.cz();
		AmB.c () = A.c () - B.c ();
	}
	static inline void sA(const float &s, V3E2f &A, ENFT_SSE::__m128 &tmp)
	{
		tmp = ENFT_SSE::_mm_set1_ps(s);
		A.cxx_cxy_cxz_cx() = ENFT_SSE::_mm_mul_ps(tmp, A.cxx_cxy_cxz_cx());
		A.cyy_cyz_cy_czz() = ENFT_SSE::_mm_mul_ps(tmp, A.cyy_cyz_cy_czz());
		A.cz() *= s;
		A.c () *= s;
	}
	static inline void AB(const V3E1f &A, const V3E1f &B, V3E2f &AB, ENFT_SSE::__m128 &tmp)
	{
		AB.cxx_cxy_cxz_cx() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.cx()), B.m_cx_cy_cz_c);

		tmp = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.cy()), B.m_cx_cy_cz_c);
		AB.cxy() += tmp.m128_f32[0];
		memcpy(&AB.cyy(), &tmp.m128_f32[1], 12);

		tmp = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.cz()), B.m_cx_cy_cz_c);
		AB.cxz() += tmp.m128_f32[0];
		AB.cyz() += tmp.m128_f32[1];
		AB.czz() = tmp.m128_f32[2];

		AB.x_x_cz_c() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.c()), B.m_cx_cy_cz_c);
		AB.cx() += AB.x_x_cz_c().m128_f32[0];
		AB.cy() += AB.x_x_cz_c().m128_f32[1];
		AB.cz() += tmp.m128_f32[3];
	}
	static inline void ABmCD(const V3E1f &A, const V3E1f &B, const V3E1f &C, const V3E1f &D, V3E2f &ABmCD, ENFT_SSE::__m128 &tmp)
	{
		ABmCD.cxx_cxy_cxz_cx() = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.cx()), B.m_cx_cy_cz_c), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(C.cx()), D.m_cx_cy_cz_c));

		tmp = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.cy()), B.m_cx_cy_cz_c), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(C.cy()), D.m_cx_cy_cz_c));
		ABmCD.cxy() += tmp.m128_f32[0];
		memcpy(&ABmCD.cyy(), &tmp.m128_f32[1], 12);

		tmp = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.cz()), B.m_cx_cy_cz_c), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(C.cz()), D.m_cx_cy_cz_c));
		ABmCD.cxz() += tmp.m128_f32[0];
		ABmCD.cyz() += tmp.m128_f32[1];
		ABmCD.czz() = tmp.m128_f32[2];

		ABmCD.x_x_cz_c() = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.c()), B.m_cx_cy_cz_c), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(C.c()), D.m_cx_cy_cz_c));
		ABmCD.cx() += ABmCD.x_x_cz_c().m128_f32[0];
		ABmCD.cy() += ABmCD.x_x_cz_c().m128_f32[1];
		ABmCD.cz() += tmp.m128_f32[3];
	}
	static inline void ABpCDpEF(const V3E1f &A, const V3E1f &B, const V3E1f &C, const V3E1f &D, const V3E1f &E, const V3E1f &F, V3E2f &ABpCDpEF, ENFT_SSE::__m128 &tmp)
	{
		ABpCDpEF.cxx_cxy_cxz_cx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.cx()), B.m_cx_cy_cz_c), 
									ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(C.cx()), D.m_cx_cy_cz_c), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(E.cx()), F.m_cx_cy_cz_c)));
		tmp = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.cy()), B.m_cx_cy_cz_c), 
			  ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(C.cy()), D.m_cx_cy_cz_c), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(E.cy()), F.m_cx_cy_cz_c)));
		ABpCDpEF.cxy() += tmp.m128_f32[0];
		memcpy(&ABpCDpEF.cyy(), &tmp.m128_f32[1], 12);

		tmp = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.cz()), B.m_cx_cy_cz_c), 
			  ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(C.cz()), D.m_cx_cy_cz_c), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(E.cz()), F.m_cx_cy_cz_c)));
		ABpCDpEF.cxz() += tmp.m128_f32[0];
		ABpCDpEF.cyz() += tmp.m128_f32[1];
		ABpCDpEF.czz() = tmp.m128_f32[2];

		ABpCDpEF.x_x_cz_c() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.c()), B.m_cx_cy_cz_c), 
							  ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(C.c()), D.m_cx_cy_cz_c), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(E.c()), F.m_cx_cy_cz_c)));
		ABpCDpEF.cx() += ABpCDpEF.x_x_cz_c().m128_f32[0];
		ABpCDpEF.cy() += ABpCDpEF.x_x_cz_c().m128_f32[1];
		ABpCDpEF.cz() += tmp.m128_f32[3];
	}
	static inline void AB(const V3E1f &A, const V3E2f &B, V3E3f &AB, ENFT_SSE::__m128 &tmp1, ENFT_SSE::__m128 &tmp2)
	{
		tmp1 = ENFT_SSE::_mm_set1_ps(A.cx());
		AB.cxxx_cxxy_cxxz_cxx() = ENFT_SSE::_mm_mul_ps(tmp1, B.cxx_cxy_cxz_cx());
		AB.cxyy_cxyz_cxy_cxzz() = ENFT_SSE::_mm_mul_ps(tmp1, B.cyy_cyz_cy_czz());
		AB.cxz() = A.cx() * B.cz();
		AB.cx () = A.cx() * B.c ();

		tmp1 = ENFT_SSE::_mm_set1_ps(A.cy());
		tmp2 = ENFT_SSE::_mm_mul_ps(tmp1, B.cxx_cxy_cxz_cx());
		AB.cxxy() += tmp2.m128_f32[0];	AB.cxyy() += tmp2.m128_f32[1];	AB.cxyz() += tmp2.m128_f32[2];	AB.cxy() += tmp2.m128_f32[3];
		tmp2 = ENFT_SSE::_mm_mul_ps(tmp1, B.cyy_cyz_cy_czz());
		memcpy(&AB.cyyy(), &tmp2, 16);
		AB.cyz() = A.cy() * B.cz();
		AB.cy () = A.cy() * B.c ();

		tmp1 = ENFT_SSE::_mm_set1_ps(A.cz());
		tmp2 = ENFT_SSE::_mm_mul_ps(tmp1, B.cxx_cxy_cxz_cx());
		AB.cxxz() += tmp2.m128_f32[0];	AB.cxyz() += tmp2.m128_f32[1];	AB.cxzz() += tmp2.m128_f32[2];	AB.cxz() += tmp2.m128_f32[3];
		tmp2 = ENFT_SSE::_mm_mul_ps(tmp1, B.cyy_cyz_cy_czz());
		AB.cyyz() += tmp2.m128_f32[0];	AB.cyzz() += tmp2.m128_f32[1];	AB.cyz() += tmp2.m128_f32[2];	AB.czzz() = tmp2.m128_f32[3];
		AB.czz () = A.cz() * B.cz();
		AB.cz  () = A.cz() * B.c ();

		tmp1 = ENFT_SSE::_mm_set1_ps(A.c());
		tmp2 = ENFT_SSE::_mm_mul_ps(tmp1, B.cxx_cxy_cxz_cx());
		AB.cxx() += tmp2.m128_f32[0];	AB.cxy() += tmp2.m128_f32[1];	AB.cxz() += tmp2.m128_f32[2];	AB.cx() += tmp2.m128_f32[3];
		tmp2 = ENFT_SSE::_mm_mul_ps(tmp1, B.cyy_cyz_cy_czz());
		AB.cyy() += tmp2.m128_f32[0];	AB.cyz() += tmp2.m128_f32[1];	AB.cy() += tmp2.m128_f32[2];	AB.czz() += tmp2.m128_f32[3];
		AB.cz() += A.c() * B.cz();
		AB.c() = A.c() * B.c();
	}
	static inline void ABpCDpEF(const V3E1f &A, const V3E2f &B, const V3E1f &C, const V3E2f &D, const V3E1f &E, const V3E2f &F, V3E3f &ABpCDpEF, 
								ENFT_SSE::__m128 &tmp1, ENFT_SSE::__m128 &tmp2, ENFT_SSE::__m128 &tmp3, ENFT_SSE::__m128 &tmp4)
	{
		tmp1 = ENFT_SSE::_mm_set1_ps(A.cx());		tmp2 = ENFT_SSE::_mm_set1_ps(C.cx());		tmp3 = ENFT_SSE::_mm_set1_ps(E.cx());
		ABpCDpEF.cxxx_cxxy_cxxz_cxx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp1, B.cxx_cxy_cxz_cx()), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp2, D.cxx_cxy_cxz_cx()), ENFT_SSE::_mm_mul_ps(tmp3, F.cxx_cxy_cxz_cx())));
		ABpCDpEF.cxyy_cxyz_cxy_cxzz() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp1, B.cyy_cyz_cy_czz()), 
										ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp2, D.cyy_cyz_cy_czz()), ENFT_SSE::_mm_mul_ps(tmp3, F.cyy_cyz_cy_czz())));
		ABpCDpEF.cxz() = A.cx() * B.cz() + C.cx() * D.cz() + E.cx() * F.cz();
		ABpCDpEF.cx () = A.cx() * B.c () + C.cx() * D.c () + E.cx() * F.c ();

		tmp1 = ENFT_SSE::_mm_set1_ps(A.cy());		tmp2 = ENFT_SSE::_mm_set1_ps(C.cy());		tmp3 = ENFT_SSE::_mm_set1_ps(E.cy());
		tmp4 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp1, B.cxx_cxy_cxz_cx()), 
			   ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp2, D.cxx_cxy_cxz_cx()), ENFT_SSE::_mm_mul_ps(tmp3, F.cxx_cxy_cxz_cx())));
		ABpCDpEF.cxxy() += tmp4.m128_f32[0];	ABpCDpEF.cxyy() += tmp4.m128_f32[1];	ABpCDpEF.cxyz() += tmp4.m128_f32[2];	ABpCDpEF.cxy() += tmp4.m128_f32[3];
		tmp4 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp1, B.cyy_cyz_cy_czz()), 
			   ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp2, D.cyy_cyz_cy_czz()), ENFT_SSE::_mm_mul_ps(tmp3, F.cyy_cyz_cy_czz())));
		memcpy(&ABpCDpEF.cyyy(), &tmp4, 16);
		ABpCDpEF.cyz() = A.cy() * B.cz() + C.cy() * D.cz() + E.cy() * F.cz();
		ABpCDpEF.cy () = A.cy() * B.c () + C.cy() * D.c () + E.cy() * F.c ();

		tmp1 = ENFT_SSE::_mm_set1_ps(A.cz());		tmp2 = ENFT_SSE::_mm_set1_ps(C.cz());		tmp3 = ENFT_SSE::_mm_set1_ps(E.cz());
		tmp4 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp1, B.cxx_cxy_cxz_cx()), 
			   ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp2, D.cxx_cxy_cxz_cx()), ENFT_SSE::_mm_mul_ps(tmp3, F.cxx_cxy_cxz_cx())));
		ABpCDpEF.cxxz() += tmp4.m128_f32[0];	ABpCDpEF.cxyz() += tmp4.m128_f32[1];	ABpCDpEF.cxzz() += tmp4.m128_f32[2];	ABpCDpEF.cxz() += tmp4.m128_f32[3];
		tmp4 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp1, B.cyy_cyz_cy_czz()), 
			   ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp2, D.cyy_cyz_cy_czz()), ENFT_SSE::_mm_mul_ps(tmp3, F.cyy_cyz_cy_czz())));
		ABpCDpEF.cyyz() += tmp4.m128_f32[0];	ABpCDpEF.cyzz() += tmp4.m128_f32[1];	ABpCDpEF.cyz() += tmp4.m128_f32[2];		ABpCDpEF.czzz() = tmp4.m128_f32[3];
		ABpCDpEF.czz() = A.cz() * B.cz() + C.cz() * D.cz() + E.cz() * F.cz();
		ABpCDpEF.cz () = A.cz() * B.c () + C.cz() * D.c () + E.cz() * F.c ();

		tmp1 = ENFT_SSE::_mm_set1_ps(A.c());		tmp2 = ENFT_SSE::_mm_set1_ps(C.c());		tmp3 = ENFT_SSE::_mm_set1_ps(E.c());
		tmp4 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp1, B.cxx_cxy_cxz_cx()), 
			   ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp2, D.cxx_cxy_cxz_cx()), ENFT_SSE::_mm_mul_ps(tmp3, F.cxx_cxy_cxz_cx())));
		ABpCDpEF.cxx() += tmp4.m128_f32[0];	ABpCDpEF.cxy() += tmp4.m128_f32[1];	ABpCDpEF.cxz() += tmp4.m128_f32[2];	ABpCDpEF.cx() += tmp4.m128_f32[3];
		tmp4 = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp1, B.cyy_cyz_cy_czz()), 
			   ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(tmp2, D.cyy_cyz_cy_czz()), ENFT_SSE::_mm_mul_ps(tmp3, F.cyy_cyz_cy_czz())));
		ABpCDpEF.cyy() += tmp4.m128_f32[0];	ABpCDpEF.cyz() += tmp4.m128_f32[1];	ABpCDpEF.cy() += tmp4.m128_f32[2];	ABpCDpEF.czz() += tmp4.m128_f32[3];
		ABpCDpEF.cz() += A.c() * B.cz() + C.c() * D.cz() + E.c() * F.cz();
		ABpCDpEF.c() = A.c() * B.c() + C.c() * D.c() + E.c() * F.c();
	}
	static inline void Permulate(const V3E3f &A, V3E3fPermutation &AP)
	{
		AP.v0 () = A.cxxx();
		AP.v1 () = A.cxxy();
		AP.v2 () = A.cxyy();
		AP.v3 () = A.cyyy();
		AP.v4 () = A.cxxz();
		AP.v5 () = A.cxyz();
		AP.v6 () = A.cyyz();
		AP.v7 () = A.cxzz();
		AP.v8 () = A.cyzz();
		AP.v9 () = A.czzz();
		AP.v10() = A.cxx ();
		AP.v11() = A.cxy ();
		AP.v12() = A.cyy ();
		AP.v13() = A.cxz ();
		AP.v14() = A.cyz ();
		AP.v15() = A.czz ();
		AP.v16() = A.cx	 ();
		AP.v17() = A.cy	 ();
		AP.v18() = A.cz	 ();
		AP.v19() = A.c	 ();
	}
	static inline void sA_from_0or1(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA, ENFT_SSE::__m128 &tmp)
	{
		tmp = ENFT_SSE::_mm_set1_ps(s);
		sA.m_sse0 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse0);
		sA.m_sse1 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse1);
		sA.m_sse2 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse2);
		sA.m_sse3 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse3);
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse4);
	}
	static inline void sA_from_2(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA, ENFT_SSE::__m128 &tmp)
	{
		sA.v2() = s * A.v2();	sA.v3() = s * A.v3();
		tmp = ENFT_SSE::_mm_set1_ps(s);
		sA.m_sse1 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse1);
		sA.m_sse2 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse2);
		sA.m_sse3 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse3);
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse4);
	}
	static inline void sA_from_3(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA, ENFT_SSE::__m128 &tmp)
	{
		sA.v3() = s * A.v3();
		tmp = ENFT_SSE::_mm_set1_ps(s);
		sA.m_sse1 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse1);
		sA.m_sse2 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse2);
		sA.m_sse3 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse3);
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse4);
	}
	static inline void sA_from_4or5(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA, ENFT_SSE::__m128 &tmp)
	{
		tmp = ENFT_SSE::_mm_set1_ps(s);
		sA.m_sse1 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse1);
		sA.m_sse2 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse2);
		sA.m_sse3 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse3);
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse4);
	}
	static inline void sA_from_6(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA, ENFT_SSE::__m128 &tmp)
	{
		sA.v6() = s * A.v6();	sA.v7() = s * A.v7();
		tmp = ENFT_SSE::_mm_set1_ps(s);
		sA.m_sse2 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse2);
		sA.m_sse3 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse3);
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse4);
	}
	static inline void sA_from_7(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA, ENFT_SSE::__m128 &tmp)
	{
		sA.v7() = s * A.v7();
		tmp = ENFT_SSE::_mm_set1_ps(s);
		sA.m_sse2 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse2);
		sA.m_sse3 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse3);
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse4);
	}
	static inline void sA_from_8or9(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA, ENFT_SSE::__m128 &tmp)
	{
		tmp = ENFT_SSE::_mm_set1_ps(s);
		sA.m_sse2 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse2);
		sA.m_sse3 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse3);
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse4);
	}
	static inline void sA_from_10(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA, ENFT_SSE::__m128 &tmp)
	{
		sA.v10() = s * A.v10();	sA.v11() = s * A.v11();
		tmp = ENFT_SSE::_mm_set1_ps(s);
		sA.m_sse3 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse3);
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse4);
	}
	static inline void sA_from_11(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA, ENFT_SSE::__m128 &tmp)
	{
		sA.v11() = s * A.v11();
		tmp = ENFT_SSE::_mm_set1_ps(s);
		sA.m_sse3 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse3);
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse4);
	}
	static inline void sA_from_12or13(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA, ENFT_SSE::__m128 &tmp)
	{
		tmp = ENFT_SSE::_mm_set1_ps(s);
		sA.m_sse3 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse3);
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(tmp, A.m_sse4);
	}
	static inline void sA_from_14(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA)
	{
		sA.v14() = s * A.v14();	sA.v15() = s * A.v15();
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.m_sse4);
	}
	static inline void sA_from_15(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA)
	{
		sA.v15() = s * A.v15();
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.m_sse4);
	}
	static inline void sA_from_16or17(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA)
	{
		sA.m_sse4 = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s), A.m_sse4);
	}
	static inline void sA_from_18(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA)
	{
		sA.v18() = s * A.v18();	sA.v19() = s * A.v19();
	}
	static inline void sA_from_19(const float &s, const V3E3fPermutation &A, V3E3fPermutation &sA)
	{
		sA.v19() = s * A.v19();
	}

	//////////////////////////////
	static inline void AmB_from_0or1(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.m_sse0 = ENFT_SSE::_mm_sub_ps(A.m_sse0, B.m_sse0);
		AmB.m_sse1 = ENFT_SSE::_mm_sub_ps(A.m_sse1, B.m_sse1);
		AmB.m_sse2 = ENFT_SSE::_mm_sub_ps(A.m_sse2, B.m_sse2);
		AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_2(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.v2() = A.v2() - B.v2();	AmB.v3() = A.v3() - B.v3();
		AmB.m_sse1 = ENFT_SSE::_mm_sub_ps(A.m_sse1, B.m_sse1);
		AmB.m_sse2 = ENFT_SSE::_mm_sub_ps(A.m_sse2, B.m_sse2);
		AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_3(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.v3() = A.v3() - B.v3();
		AmB.m_sse1 = ENFT_SSE::_mm_sub_ps(A.m_sse1, B.m_sse1);
		AmB.m_sse2 = ENFT_SSE::_mm_sub_ps(A.m_sse2, B.m_sse2);
		AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_4or5(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.m_sse1 = ENFT_SSE::_mm_sub_ps(A.m_sse1, B.m_sse1);
		AmB.m_sse2 = ENFT_SSE::_mm_sub_ps(A.m_sse2, B.m_sse2);
		AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_6(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.v6() = A.v6() - B.v6();	AmB.v7() = A.v7() - B.v7();
		AmB.m_sse2 = ENFT_SSE::_mm_sub_ps(A.m_sse2, B.m_sse2);
		AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_7(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.v7() = A.v7() - B.v7();
		AmB.m_sse2 = ENFT_SSE::_mm_sub_ps(A.m_sse2, B.m_sse2);
		AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_8or9(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.m_sse2 = ENFT_SSE::_mm_sub_ps(A.m_sse2, B.m_sse2);
		AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_10(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.v10() = A.v10() - B.v10();	AmB.v11() = A.v11() - B.v11();
		AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_11(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.v11() = A.v11() - B.v11();
		AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_12or13(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.m_sse3 = ENFT_SSE::_mm_sub_ps(A.m_sse3, B.m_sse3);
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_14(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.v14() = A.v14() - B.v14();	AmB.v15() = A.v15() - B.v15();
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_15(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.v15() = A.v15() - B.v15();
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_16or17(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.m_sse4 = ENFT_SSE::_mm_sub_ps(A.m_sse4, B.m_sse4);
	}
	static inline void AmB_from_18(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
	{
		AmB.v18() = A.v18() - B.v18();	AmB.v19() = A.v19() - B.v19();
	}
	static inline void AmB_from_19(const V3E3fPermutation &A, const V3E3fPermutation &B, V3E3fPermutation &AmB)
{
		AmB.v19() = A.v19() - B.v19();
	}

	static bool GaussianEliminate(/*V3E3fPermutation A[10], ENFT_SSE::__m128 work[6]*/V3E3fPermutation *A, ENFT_SSE::__m128 *work);

};

#endif