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

#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "LinearAlgebra/Vector4.h"
#include "LinearAlgebra/Matrix3.h"
#include "Point.h"

#if 0
class Quaternion : public LA::AlignedVector4f
{

public:

	inline Quaternion() {}
	inline Quaternion(const float *q) { m_v0123 = ENFT_SSE::_mm_setr_ps(q[0], q[1], q[2], q[3]); }
	inline Quaternion(const float &v0, const float &v1, const float &v2, const float &v3) { v0123() = ENFT_SSE::_mm_setr_ps(v0, v1, v2, v3); }

	inline void Invert(Quaternion &qI) const { qI.Set(-v0(), -v1(), -v2(), v3()); }
	inline void Slerp(const float &a1, const Quaternion &q1, const Quaternion &q2)
	{
		v0() = acos(q1.Dot(q2));
		if(fabs(v0()) < FLT_EPSILON)
		{
			v0123() = q1.v0123();
			return;
		}
		v1() = 1 / sin(v0());
		v2() = sin(a1 * v0()) * v1();
		v3() = sin((1 - a1) * v0()) * v1();
		v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(v2()), q1.v0123()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(v3()), q2.v0123()));
	}

	inline void FromAxisAngle(const Point3D &axis, const float &angle)
	{
		v0123() = ENFT_SSE::_mm_mul_ps(axis.XYZx(), ENFT_SSE::_mm_set1_ps(sin(angle * 0.5f)));
		v3() = cos(angle * 0.5f);
	}
	inline void ToAxisAngle(Point3D &axis, float &angle) const
	{
		axis.Z() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(m_v0123, m_v0123));
		if(axis.Z() > FLT_EPSILON)
		{
			axis.XYZx() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1 / sqrt(axis.Z())), m_v0123);
			axis.reserve() = 1.0f;
			angle = acos(v3());
			angle = angle + angle;
		}
		else
		{
			axis.Set(0.0f, 0.0f, 1.0f);
			angle = 0.0f;
		}
	}

	inline void ToLeftMatrix(LA::AlignedMatrix4f &L) const
	{
		L.M_00_01_02_03() = ENFT_SSE::_mm_setr_ps(v3(), v2(), -v1(), v0());
		L.M_10_11_12_13() = ENFT_SSE::_mm_setr_ps(-v2(), v3(), v0(), v1());
		L.M_20_21_22_23() = ENFT_SSE::_mm_setr_ps(v1(), -v0(), v3(), v2());
		L.M_30_31_32_33() = ENFT_SSE::_mm_setr_ps(-v0(), -v1(), -v2(), v3());
	}
	inline void ToRightMatrix(LA::AlignedMatrix4f &R) const
	{
		R.M_00_01_02_03() = ENFT_SSE::_mm_setr_ps(v3(), -v2(), v1(), v0());
		R.M_10_11_12_13() = ENFT_SSE::_mm_setr_ps(v2(), v3(), -v0(), v1());
		R.M_20_21_22_23() = ENFT_SSE::_mm_setr_ps(-v1(), v0(), v3(), v2());
		R.M_30_31_32_33() = ENFT_SSE::_mm_setr_ps(-v0(), -v1(), -v2(), v3());
	}
	static inline void Multiply(const Quaternion &q1, const Quaternion &q2, Quaternion &q1xq2)
	{
		q1xq2.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(q1.v3(), q1.v2(), -q1.v1(), q1.v0()), q2.v0123()));
		q1xq2.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(-q1.v2(), q1.v3(), q1.v0(), q1.v1()), q2.v0123()));
		q1xq2.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(q1.v1(), -q1.v0(), q1.v3(), q1.v2()), q2.v0123()));
		q1xq2.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_setr_ps(-q1.v0(), -q1.v1(), -q1.v2(), q1.v3()), q2.v0123()));
	}

};
#else
class Quaternion : public LA::Vector4d
{

public:

	inline void Invert(Quaternion &qI) const { qI.Set(-v0(), -v1(), -v2(), v3()); }
	inline void Slerp(const double &a1, const Quaternion &q1, const Quaternion &q2)
	{
		v0() = acos(q1.Dot(q2));
		if(fabs(v0()) < DBL_EPSILON)
		{
			*this = q1;
			return;
		}
		v1() = 1 / sin(v0());
		const double s1 = sin(a1 * v0()) * v1();
		const double s2 = sin((1 - a1) * v0()) * v1();
		v0() = s1 * q1.v0() + s2 * q2.v0();
		v1() = s1 * q1.v1() + s2 * q2.v1();
		v2() = s1 * q1.v2() + s2 * q2.v2();
		v3() = s1 * q1.v3() + s2 * q2.v3();
	}

	inline void FromAxisAngle(const Point3D &axis, const float &angle)
	{
		v3() = sin(angle * 0.5);
		v0() = axis.X() * v3();
		v1() = axis.Y() * v3();
		v2() = axis.Z() * v3();
		v3() = cos(angle * 0.5);
	}
	inline void ToAxisAngle(Point3D &axis, float &angle) const
	{
		const double c2 = v0() * v0() + v1() * v1() + v2() * v2();
		if(c2 > DBL_EPSILON)
		{
			const double s = 1 / sqrt(c2);
			axis.X() = float(v0() * s);
			axis.Y() = float(v1() * s);
			axis.Z() = float(v2() * s);
			angle = float(acos(v3()));
			angle = angle + angle;
		}
		else
		{
			axis.Set(0.0f, 0.0f, 1.0f);
			angle = 0.0f;
		}
	}

	inline void ToRotationMatrix(LA::Matrix3d &R) const
	{
		const double q00 = v0() * v0(), q01 = v0() * v1(), q02 = v0() * v2(), q03 = v0() * v3();
		const double q11 = v1() * v1(), q12 = v1() * v2(), q13 = v1() * v3();
		const double q22 = v2() * v2(), q23 = v2() * v3();
		R.M00() = q11 + q22;				R.M01() = q01 + q23;				R.M02() = q02 - q13;
		R.M10() = q01 - q23;				R.M11() = q00 + q22;				R.M12() = q12 + q03;
		R.M20() = q02 + q13;				R.M21() = q12 - q03;				R.M22() = q00 + q11;
		R.M00() = 1 - R.M00() - R.M00();	R.M01() = R.M01() + R.M01();		R.M02() = R.M02() + R.M02();
		R.M10() = R.M10() + R.M10();		R.M11() = 1 - R.M11() - R.M11();	R.M12() = R.M12() + R.M12();
		R.M20() = R.M20() + R.M20();		R.M21() = R.M21() + R.M21();		R.M22() = 1 - R.M22() - R.M22();
	}
	inline void ToRotationMatrixTranspose(LA::Matrix3d &RT) const
	{
		const double q00 = v0() * v0(), q01 = v0() * v1(), q02 = v0() * v2(), q03 = v0() * v3();
		const double q11 = v1() * v1(), q12 = v1() * v2(), q13 = v1() * v3();
		const double q22 = v2() * v2(), q23 = v2() * v3();
		RT.M00() = q11 + q22;				RT.M10() = q01 + q23;				RT.M20() = q02 - q13;
		RT.M01() = q01 - q23;				RT.M11() = q00 + q22;				RT.M21() = q12 + q03;
		RT.M02() = q02 + q13;				RT.M12() = q12 - q03;				RT.M22() = q00 + q11;
		RT.M00() = 1 - RT.M00() - RT.M00();	RT.M01() = RT.M01() + RT.M01();		RT.M02() = RT.M02() + RT.M02();
		RT.M10() = RT.M10() + RT.M10();		RT.M11() = 1 - RT.M11() - RT.M11();	RT.M12() = RT.M12() + RT.M12();
		RT.M20() = RT.M20() + RT.M20();		RT.M21() = RT.M21() + RT.M21();		RT.M22() = 1 - RT.M22() - RT.M22();
	}
	inline void ToRotationMatrix(LA::AlignedMatrix3f &R) const
	{
		const double q00 = v0() * v0(), q01 = v0() * v1(), q02 = v0() * v2(), q03 = v0() * v3();
		const double q11 = v1() * v1(), q12 = v1() * v2(), q13 = v1() * v3();
		const double q22 = v2() * v2(), q23 = v2() * v3();
		R.M00() = float(q11 + q22);			R.M01() = float(q01 + q23);			R.M02() = float(q02 - q13);
		R.M10() = float(q01 - q23);			R.M11() = float(q00 + q22);			R.M12() = float(q12 + q03);
		R.M20() = float(q02 + q13);			R.M21() = float(q12 - q03);			R.M22() = float(q00 + q11);
		R.M00() = 1 - R.M00() - R.M00();	R.M01() = R.M01() + R.M01();		R.M02() = R.M02() + R.M02();
		R.M10() = R.M10() + R.M10();		R.M11() = 1 - R.M11() - R.M11();	R.M12() = R.M12() + R.M12();
		R.M20() = R.M20() + R.M20();		R.M21() = R.M21() + R.M21();		R.M22() = 1 - R.M22() - R.M22();
	}
	inline void ToRotationMatrixTranspose(LA::AlignedMatrix3f &RT) const
	{
		const double q00 = v0() * v0(), q01 = v0() * v1(), q02 = v0() * v2(), q03 = v0() * v3();
		const double q11 = v1() * v1(), q12 = v1() * v2(), q13 = v1() * v3();
		const double q22 = v2() * v2(), q23 = v2() * v3();
		RT.M00() = float(q11 + q22);		RT.M10() = float(q01 + q23);		RT.M20() = float(q02 - q13);
		RT.M01() = float(q01 - q23);		RT.M11() = float(q00 + q22);		RT.M21() = float(q12 + q03);
		RT.M02() = float(q02 + q13);		RT.M12() = float(q12 - q03);		RT.M22() = float(q00 + q11);
		RT.M00() = 1 - RT.M00() - RT.M00();	RT.M01() = RT.M01() + RT.M01();		RT.M02() = RT.M02() + RT.M02();
		RT.M10() = RT.M10() + RT.M10();		RT.M11() = 1 - RT.M11() - RT.M11();	RT.M12() = RT.M12() + RT.M12();
		RT.M20() = RT.M20() + RT.M20();		RT.M21() = RT.M21() + RT.M21();		RT.M22() = 1 - RT.M22() - RT.M22();
	}
	inline void FromRotationMatrix(const LA::Matrix3d &R)
	{
		v3() = R.M00() + R.M11() + R.M22();
		if(v3() > R.M00() && v3() > R.M11() && v3() > R.M22())
		{
			v3() = sqrt(v3() + 1) * 0.5;
			v2() = 0.25 / v3();
			v0() = (R.M12() - R.M21()) * v2();
			v1() = (R.M20() - R.M02()) * v2();
			v2() = (R.M01() - R.M10()) * v2();
		}
		else if(R.M00() > R.M11() && R.M00() > R.M22())
		{
			v0() = sqrt(R.M00() + R.M00() - v3() + 1) * 0.5;
			v3() = 0.25 / v0();
			v1() = (R.M01() + R.M10()) * v3();
			v2() = (R.M02() + R.M20()) * v3();
			v3() = (R.M12() - R.M21()) * v3();
		}
		else if(R.M11() > R.M22())
		{
			v1() = sqrt(R.M11() + R.M11() - v3() + 1) * 0.5;
			v3() = 0.25 / v1();
			v0() = (R.M01() + R.M10()) * v3();
			v2() = (R.M12() + R.M21()) * v3();
			v3() = (R.M20() - R.M02()) * v3();
		}
		else
		{
			v2() = sqrt(R.M22() + R.M22() - v3() + 1) * 0.5;
			v3() = 0.25 / v2();
			v0() = (R.M02() + R.M20()) * v3();
			v1() = (R.M12() + R.M21()) * v3();
			v3() = (R.M01() - R.M10()) * v3();
		}
	}
	inline void FromRotationMatrix(const LA::AlignedMatrix3f &R)
	{
		v3() = double(R.M00() + R.M11() + R.M22());
		if(v3() > double(R.M00()) && v3() > double(R.M11()) && v3() > double(R.M22()))
		{
			v3() = sqrt(v3() + 1) * 0.5;
			v2() = 0.25 / v3();
			v0() = (R.M12() - R.M21()) * v2();
			v1() = (R.M20() - R.M02()) * v2();
			v2() = (R.M01() - R.M10()) * v2();
		}
		else if(R.M00() > R.M11() && R.M00() > R.M22())
		{
			v0() = sqrt(R.M00() + R.M00() - v3() + 1) * 0.5;
			v3() = 0.25 / v0();
			v1() = (R.M01() + R.M10()) * v3();
			v2() = (R.M02() + R.M20()) * v3();
			v3() = (R.M12() - R.M21()) * v3();
		}
		else if(R.M11() > R.M22())
		{
			v1() = sqrt(R.M11() + R.M11() - v3() + 1) * 0.5;
			v3() = 0.25 / v1();
			v0() = (R.M01() + R.M10()) * v3();
			v2() = (R.M12() + R.M21()) * v3();
			v3() = (R.M20() - R.M02()) * v3();
		}
		else
		{
			v2() = sqrt(R.M22() + R.M22() - v3() + 1) * 0.5;
			v3() = 0.25 / v2();
			v0() = (R.M02() + R.M20()) * v3();
			v1() = (R.M12() + R.M21()) * v3();
			v3() = (R.M01() - R.M10()) * v3();
		}
	}

	static inline void Multiply(const Quaternion &q1, const Quaternion &q2, Quaternion &q1xq2)
	{
		q1xq2.v0() =  q1.v3() * q2.v0() + q1.v2() * q2.v1() - q1.v1() * q2.v2() + q1.v0() * q2.v3();
		q1xq2.v1() = -q1.v2() * q2.v0() + q1.v3() * q2.v1() + q1.v0() * q2.v2() + q1.v1() * q2.v3();
		q1xq2.v2() =  q1.v1() * q2.v0() - q1.v0() * q2.v1() + q1.v3() * q2.v2() + q1.v2() * q2.v3();
		q1xq2.v3() = -q1.v0() * q2.v0() - q1.v1() * q2.v1() - q1.v2() * q2.v2() + q1.v3() * q2.v3();
	}
	static inline void Multiply(const Quaternion &q1, const Quaternion &q2, ENFT_SSE::__m128 &q1xq2)
	{
		q1xq2.m128_f32[0] = float( q1.v3() * q2.v0() + q1.v2() * q2.v1() - q1.v1() * q2.v2() + q1.v0() * q2.v3());
		q1xq2.m128_f32[1] = float(-q1.v2() * q2.v0() + q1.v3() * q2.v1() + q1.v0() * q2.v2() + q1.v1() * q2.v3());
		q1xq2.m128_f32[2] = float( q1.v1() * q2.v0() - q1.v0() * q2.v1() + q1.v3() * q2.v2() + q1.v2() * q2.v3());
		q1xq2.m128_f32[3] = float(-q1.v0() * q2.v0() - q1.v1() * q2.v1() - q1.v2() * q2.v2() + q1.v3() * q2.v3());
	}

};
#endif

#endif