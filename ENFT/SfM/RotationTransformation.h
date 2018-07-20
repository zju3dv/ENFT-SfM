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

#ifndef _ROTATION_TRANSFORMATION_H_
#define _ROTATION_TRANSFORMATION_H_

#include "LinearAlgebra/Matrix2.h"
#include "LinearAlgebra/Matrix3.h"
#include "LinearAlgebra/Matrix4.h"
#include "Point.h"

class RotationTransformation2D : public LA::AlignedMatrix2f
{

public:

	inline const ENFT_SSE::__m128& r_00_01_10_11() const { return M_00_01_10_11(); }
	inline		 ENFT_SSE::__m128& r_00_01_10_11()	   { return M_00_01_10_11(); }
	inline const float& r00() const { return M00(); }		inline float& r00() { return M00(); }
	inline const float& r01() const { return M01(); }		inline float& r01() { return M01(); }
	inline const float& r10() const { return M10(); }		inline float& r10() { return M10(); }
	inline const float& r11() const { return M11(); }		inline float& r11() { return M11(); }

	inline void Invalidate() { r00() = FLT_MAX; }
	inline bool IsValid() const { return r00() != FLT_MAX; }
	inline bool IsInvalid() const { return r00() == FLT_MAX; }

	inline void Set(const float &theta)
	{
		M00() = M11() = cos(theta);
		M10() = sin(theta);
		M01() = -M10();
	}

	inline void Invert(RotationTransformation2D &Rinv) const { Rinv.m_00_01_10_11 = _mm_setr_ps(r00(), r10(), r01(), r11()); }
	inline void Apply(const Point2D &x, Point2D &Rx) const
	{
		Rx.x() = r00() * x.x() + r01() * x.y();
		Rx.y() = r10() * x.x() + r11() * x.y();
	}
	inline void ApplyInversely(const Point2D &x, Point2D &RTx) const
	{
		RTx.x() = r00() * x.x() + r10() * x.y();
		RTx.y() = r01() * x.x() + r11() * x.y();
	}
	inline void Print() const
	{
		printf("%f %f\n", r00(), r01());
		printf("%f %f\n", r10(), r11());
	}

public:

	// x2 = R12 * x1 = R12 * R1 * x
	// R2 = R12 * R1
	// R12 and R2 are allowed to be the same
	static inline void AccumulateTransformation(const RotationTransformation2D &R1, const RotationTransformation2D &R12, RotationTransformation2D &R2, 
												ENFT_SSE::__m128 *work2)
	{
		work2[0] = ENFT_SSE::_mm_mul_ps(R12.r_00_01_10_11(), _mm_setr_ps(R1.r00(), R1.r10(), R1.r00(), R1.r10()));
		work2[1] = ENFT_SSE::_mm_mul_ps(R12.r_00_01_10_11(), _mm_setr_ps(R1.r01(), R1.r11(), R1.r01(), R1.r11()));
		R2.r00() = work2[0].m128_f32[0] + work2[0].m128_f32[1];
		R2.r01() = work2[1].m128_f32[0] + work2[1].m128_f32[1];
		R2.r10() = work2[0].m128_f32[2] + work2[0].m128_f32[3];
		R2.r11() = work2[1].m128_f32[2] + work2[1].m128_f32[3];
	}

};

class RotationTransformation3D : public LA::AlignedMatrix3f
{

public:

	inline const ENFT_SSE::__m128& r_00_01_02_x() const { return M_00_01_02_x(); }		inline ENFT_SSE::__m128& r_00_01_02_x() { return M_00_01_02_x(); }
	inline const ENFT_SSE::__m128& r_10_11_12_x() const { return M_10_11_12_x(); }		inline ENFT_SSE::__m128& r_10_11_12_x() { return M_10_11_12_x(); }
	inline const ENFT_SSE::__m128& r_20_21_22_x() const { return M_20_21_22_x(); }		inline ENFT_SSE::__m128& r_20_21_22_x() { return M_20_21_22_x(); }
	inline const float& r00() const { return M00(); }		inline float& r00() { return M00(); }
	inline const float& r01() const { return M01(); }		inline float& r01() { return M01(); }
	inline const float& r02() const { return M02(); }		inline float& r02() { return M02(); }
	inline const float& r10() const { return M10(); }		inline float& r10() { return M10(); }
	inline const float& r11() const { return M11(); }		inline float& r11() { return M11(); }
	inline const float& r12() const { return M12(); }		inline float& r12() { return M12(); }
	inline const float& r20() const { return M20(); }		inline float& r20() { return M20(); }
	inline const float& r21() const { return M21(); }		inline float& r21() { return M21(); }
	inline const float& r22() const { return M22(); }		inline float& r22() { return M22(); }

	inline void Invalidate() { r00() = FLT_MAX; }
	inline bool IsValid() const { return r00() != FLT_MAX; }
	inline bool IsInvalid() const { return r00() == FLT_MAX; }

	inline void Apply(const Point3D &X, float &RX_X, float &RX_Y, float &RX_Z) const
	{
		RX_X = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(), X.XYZx()));
		RX_Y = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), X.XYZx()));
		RX_Z = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_20_21_22_x(), X.XYZx()));
	}
	inline void Apply(const ENFT_SSE::__m128 &X, float &RX_X, float &RX_Y, float &RX_Z) const
	{
		RX_X = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(), X));
		RX_Y = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), X));
		RX_Z = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_20_21_22_x(), X));
	}
	inline void Apply(const Point3D &X, Point3D &RX) const
	{
		RX.X() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(), X.XYZx()));
		RX.Y() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), X.XYZx()));
		RX.Z() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_20_21_22_x(), X.XYZx()));
	}
	inline void Apply(const Point2D &x, Point3D &RX) const
	{
		RX.X() = r00() * x.x() + r01() * x.y() + r02();
		RX.Y() = r10() * x.x() + r11() * x.y() + r12();
		RX.Z() = r20() * x.x() + r21() * x.y() + r22();
	}
	inline void Apply(const ENFT_SSE::__m128 &X, ENFT_SSE::__m128 &RX) const
	{
		RX.m128_f32[0] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(), X));
		RX.m128_f32[1] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), X));
		RX.m128_f32[2] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_20_21_22_x(), X));
	}
	inline void ApplyInversely(const Point3D &RX, Point3D &X) const
	{
		X.XYZx() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(), _mm_set1_ps(RX.X())), 
										 ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), _mm_set1_ps(RX.Y()))), 
										 ENFT_SSE::_mm_mul_ps(r_20_21_22_x(), _mm_set1_ps(RX.Z())));
		X.reserve() = 1;
	}
	inline void ApplyInversely(const Point2D &RX, Point3D &X) const
	{
		X.XYZx() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(), _mm_set1_ps(RX.x())), 
										 ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), _mm_set1_ps(RX.y()))), r_20_21_22_x());
		X.reserve() = 1;
	}
	inline void ApplyInversely(Point3D &X) const
	{
		X.XYZx() = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(), _mm_set1_ps(X.X())), 
										 ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), _mm_set1_ps(X.Y()))), 
										 ENFT_SSE::_mm_mul_ps(r_20_21_22_x(), _mm_set1_ps(X.Z())));
		X.reserve() = 1;
	}

	//	   R2 * X2 = R1 * X1
	// ==> R2 * (R * X1) = R1 * X1
	// ==> R2 = R1 * R^T
	// R1 and R2 are allowed to be the same
	inline void Apply(const RotationTransformation3D &R1, RotationTransformation3D &R2) const { LA::ABT(R1, *this, R2); }

	inline void Apply(const Point3DCovariance &P, Point3DCovariance &RPRT, ENFT_SSE::__m128 *work4) const
	{
		work4[0] = _mm_setr_ps(P.M00(), P.M01(), P.M02(), 0.0f);
		work4[1] = _mm_setr_ps(P.M10(), P.M11(), P.M12(), 0.0f);
		work4[2] = _mm_setr_ps(P.M20(), P.M21(), P.M22(), 0.0f);
		work4[3].m128_f32[0] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work4[0], r_00_01_02_x()));
		work4[3].m128_f32[1] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work4[1], r_00_01_02_x()));
		work4[3].m128_f32[2] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work4[2], r_00_01_02_x()));
		RPRT.cXX() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(), work4[3]));

		work4[3].m128_f32[0] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work4[0], r_10_11_12_x()));
		work4[3].m128_f32[1] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work4[1], r_10_11_12_x()));
		work4[3].m128_f32[2] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work4[2], r_10_11_12_x()));
		RPRT.cXY() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(), work4[3]));
		RPRT.cYY() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), work4[3]));

		work4[3].m128_f32[0] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work4[0], r_20_21_22_x()));
		work4[3].m128_f32[1] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work4[1], r_20_21_22_x()));
		work4[3].m128_f32[2] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(work4[2], r_20_21_22_x()));
		RPRT.cXZ() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_00_01_02_x(), work4[3]));
		RPRT.cYZ() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_10_11_12_x(), work4[3]));
		RPRT.cZZ() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(r_20_21_22_x(), work4[3]));
	}
	inline void ApplyInversely(const Point3DCovariance &RPRT, Point3DCovariance &P, RotationTransformation3D &RT, ENFT_SSE::__m128 *work4) const
	{
		LA::AlignedMatrix3f::GetTranspose(RT);
		RT.Apply(RPRT, P, work4);
	}

	inline void GetTranspose(RotationTransformation3D &RT) const { LA::AlignedMatrix3f::GetTranspose(RT); }
	inline void GetTranspose(LA::Matrix4f &RT) const
	{
		RT.M00() = r00();	RT.M01() = r10();	RT.M02() = r20();	RT.M03() = 0;
		RT.M10() = r01();	RT.M11() = r11();	RT.M12() = r21();	RT.M13() = 0;
		RT.M20() = r02();	RT.M21() = r12();	RT.M22() = r22();	RT.M23() = 0;
		RT.M30() = 0.0f;	RT.M31() = 0.0f;	RT.M32() = 0.0f;	RT.M33() = 1.0f;
	}

	inline void LeftMultiply(const RotationTransformation3D &Rl, RotationTransformation3D &RlxR, ENFT_SSE::__m128 &work) const
	{
		ENFT_SSE::SSE::Set012(r00(), r10(), r20(), work);
		RlxR.r00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Rl.r_00_01_02_x(), work));
		RlxR.r10() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Rl.r_10_11_12_x(), work));
		RlxR.r20() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Rl.r_20_21_22_x(), work));
		ENFT_SSE::SSE::Set012(r01(), r11(), r21(), work);
		RlxR.r01() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Rl.r_00_01_02_x(), work));
		RlxR.r11() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Rl.r_10_11_12_x(), work));
		RlxR.r21() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Rl.r_20_21_22_x(), work));
		ENFT_SSE::SSE::Set012(r02(), r12(), r22(), work);
		RlxR.r02() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Rl.r_00_01_02_x(), work));
		RlxR.r12() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Rl.r_10_11_12_x(), work));
		RlxR.r22() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(Rl.r_20_21_22_x(), work));
	}
	inline void FromAxisAngle(const Point3D &axis, const float &angle)
	{
		const float Cos = cos(angle);
		const float Sin = angle < PI ? sqrt(1 - Cos * Cos) : -sqrt(1 - Cos * Cos);
		const float w = 1 - Cos;
		const float x2 = axis.X() * axis.X();
		const float y2 = axis.Y() * axis.Y();
		const float z2 = axis.Z() * axis.Z();
		const float wxy = w * axis.X() * axis.Y();
		const float wxz = w * axis.X() * axis.Z();
		const float wyz = w * axis.Y() * axis.Z();
		const float xSin = axis.X() * Sin;
		const float ySin = axis.Y() * Sin;
		const float zSin = axis.Z() * Sin;

		r00() = w * x2 + Cos;
		r01() = wxy - zSin;
		r02() = wxz + ySin;
		r10() = wxy + zSin;
		r11() = w * y2 + Cos;
		r12() = wyz - xSin;
		r20() = wxz - ySin;
		r21() = wyz + xSin;
		r22() = w * z2 + Cos;
	}
	inline void ToAxisAngle(Point3D &axis, float &angle) const
	{
		const float trace = r00() + r11() + r22();
		const float Cos = 0.5f * (trace - 1);
		angle = acos(Cos);  // in [0,PI]
		//if(angle > 0)
		if(Cos < 1)
		{
			//if(angle < PI)
			if(Cos > -1)
			{
				axis.X() = r21() - r12();
				axis.Y() = r02() - r20();
				axis.Z() = r10() - r01();
				axis.Normalize();
			}
			else
			{
				// angle is PI
				if (r00() >= r11())
				{
					if (r00() >= r22())
					{
						// r00 is maximum diagonal term
						axis.X() = 0.5f * sqrt(r00() - r11() - r22() + 1);
						const float halfInverse = 0.5f / axis.X();
						axis.Y() = halfInverse * r01();
						axis.Z() = halfInverse * r02();
					}
					else
					{
						// r22 is maximum diagonal term
						axis.Z() = 0.5f * sqrt(r22() - r00() - r11() + 1);
						const float halfInverse = 0.5f / axis.Z();
						axis.X() = halfInverse * r02();
						axis.Y() = halfInverse * r12();
					}
				}
				else
				{
					// r11 > r00
					if (r11() >= r22())
					{
						// r11 is maximum diagonal term
						axis.Y() = 0.5f * sqrt(r11() - r00() - r22() + 1);
						const float halfInverse  = 0.5f / axis.Y();
						axis.X() = halfInverse * r01();
						axis.Z() = halfInverse * r12();
					}
					else
					{
						// r22 is maximum diagonal term
						axis.Z() = 0.5f * sqrt(r22() - r00() - r11() + 1);
						const float halfInverse = 0.5f / axis.Z();
						axis.X() = halfInverse * r02();
						axis.Y() = halfInverse * r12();
					}
				}
			}
		}
		else
		{
			// The angle is 0 and the matrix is the identity.  Any axis will
			// work, so just use the x-axis.
			axis.Set(1, 0, 0);
		}
	}
	inline void FromEulerAngleX(const float &angle)
	{
		//Rx = [1, 0, 0; 0, cx, -sx; 0, sx, cx]
		//R = Rx^T = [1, 0, 0; 0, cx, sx; 0, -sx, cx]
		MakeIdentity();
		r11() = r22() = cos(angle);
		r12() = sin(angle);
		r21() = -r12();
	}
	inline void FromEulerAngleY(const float &angle)
	{
		//Ry = [cy, 0, sy; 0, 1, 0; -sy, 0, cy]
		//R = Ry^T = [cy, 0, -sy; 0, 1, 0; sy, 0, cy]
		MakeIdentity();
		r00() = r22() = cos(angle);
		r20() = sin(angle);
		r02() = -r20();
	}
	inline void FromEulerAngleZ(const float &angle)
	{
		//Rz = [cz, -sz, 0; sz, cz, 0; 0, 0, 1]
		//R = Rz^T = [cz, sz, 0; -sz, cz, 0; 0, 0, 1]
		MakeIdentity();
		r00() = r11() = cos(angle);
		r01() = sin(angle);
		r10() = -r01();
	}
	inline void FromEulerAnglesZXY(const float &yaw, const float &pitch, const float &roll)
	{
		// R(c->w) = Rz(yaw) * Rx(pitch) * Ry(roll)
		// R = R(c->w)^T
		const float cx = cos(pitch), sx = sin(pitch), cy = cos(roll), sy = sin(roll), cz = cos(yaw), sz = sin(yaw);
		const float cycz = cy * cz, sxsy = sx * sy, cysz = cy * sz;
		//r00() = cycz - sxsy * sz;
		//r10() = -cx * sz;
		//r20() = sy * cz + sx * cysz;
		//r01() = cysz + sxsy * cz;
		//r11() = cx * cz;
		//r21() = sy * sz - sx * cycz;
		//r02() = -cx * sy;
		//r12() = sx;
		//r22() = cx * cy;
		r00() = cycz - sxsy * sz;
		r01() = cysz + sxsy * cz;
		r02() = -cx * sy;
		r10() = -cx * sz;
		r11() = cx * cz;
		r12() = sx;
		r20() = sy * cz + sx * cysz;
		r21() = sy * sz - sx * cycz;
		r22() = cx * cy;
	}
	inline void FromEulerAnglesZYX(const float &yaw, const float &pitch, const float &roll)
	{
		// R(c->w) = Rz(yaw) * Ry(pitch) * Rx(roll)
		// R = R(c->w)^T
		const float cx = cos(roll), sx = sin(roll), cy = cos(pitch), sy = sin(pitch), cz = cos(yaw), sz = sin(yaw);
		const float sxsy = sx * sy, cxsz = cx * sz, cxcz = cx * cz;
		//r00() = cy * cz;
		//r10() = sxsy * cz - cxsz;
		//r20() = sx * sz + cxcz * sy;
		//r01() = cy * sz;
		//r11() = cxcz + sxsy * sz;
		//r21() = cxsz * sy - sx * cz;
		//r02() = -sy;
		//r12() = sx * cy;
		//r22() = cx * cy;
		r00() = cy * cz;
		r01() = cy * sz;
		r02() = -sy;
		r10() = sxsy * cz - cxsz;
		r11() = cxcz + sxsy * sz;
		r12() = sx * cy;
		r20() = sx * sz + cxcz * sy;
		r21() = cxsz * sy - sx * cz;
		r22() = cx * cy;
	}

	inline void FromRodrigues(const float &w0, const float &w1, const float &w2)
	{
		const float w11=w0*w0, w22=w1*w1, w33=w2*w2, the2=w11+w22+w33;
		if(the2 > 1e-10f)
		{
			const float w12=w0*w1, w13=w0*w2, w23=w1*w2;
			const float the=sqrt(the2), s1=1/the, s2=1/the2, co=cos(the), si=sin(the), c1=si*s1, c2=(1-co)*s2;
			const float c1w1=c1*w0, c1w2=c1*w1, c1w3=c1*w2, c2w11=c2*w11, c2w12=c2*w12, c2w13=c2*w13, c2w22=c2*w22, c2w23=c2*w23, c2w33=c2*w33;
			r00() = 1-c2w22-c2w33;	r01() = -c1w3+c2w12;	r02() = c1w2+c2w13;
			r10() = c1w3+c2w12;		r11() = 1-c2w11-c2w33;	r12() = -c1w1+c2w23;
			r20() = -c1w2+c2w13;	r21() = c1w1+c2w23;		r22() = 1-c2w11-c2w22;
		}
		else
			MakeIdentity();
	}
	inline void FromRodrigues(const float &w0, const float &w1, const float &w2, float *work24)
	{
		float &w11 = work24[0], &w22 = work24[1], &w33 = work24[2], &the2 = work24[3];
		w11=w0*w0;	w22=w1*w1;	w33=w2*w2;	the2=w11+w22+w33;
		//if(the2 != 0)
		if(the2 > 1e-10f)
		{
			//const float w11=w0*w0, w12=w0*w1, w13=w0*w2, w22=w1*w1, w23=w1*w2, w33=w2*w2;
			//const float the2=w11+w22+w33, the=sqrt(the2), s1=1/the, s2=1/the2, co=cos(the), si=sin(the), c1=si*s1, c2=(1-co)*s2;
			//const float c1w1=c1*w0, c1w2=c1*w1, c1w3=c1*w2, c2w11=c2*w11, c2w12=c2*w12, c2w13=c2*w13, c2w22=c2*w22, c2w23=c2*w23, c2w33=c2*w33;
				
			float &w12=work24[4], &w13=work24[5], &w23=work24[6];
			w12=w0*w1;	w13=w0*w2;	w23=w1*w2; 

			float &the=work24[7], &s1=work24[8], &s2=work24[9], &co=work24[10], &si=work24[11], &c1=work24[12], &c2=work24[13];
			the=sqrt(the2); s1=1/the; s2=1/the2; co=cos(the); si=sin(the); c1=si*s1; c2=(1-co)*s2;
				
			float &c1w1=work24[14], &c1w2=work24[15], &c1w3=work24[16];
			float &c2w11=work24[17], &c2w12=work24[18], &c2w13=work24[19], &c2w22=work24[20], &c2w23=work24[21], &c2w33=work24[22];
			c1w1=c1*w0; c1w2=c1*w1; c1w3=c1*w2; c2w11=c2*w11; c2w12=c2*w12; c2w13=c2*w13; c2w22=c2*w22; c2w23=c2*w23; c2w33=c2*w33;
				
			r00() = 1-c2w22-c2w33;	r01() = -c1w3+c2w12;	r02() = c1w2+c2w13;
			r10() = c1w3+c2w12;		r11() = 1-c2w11-c2w33;	r12() = -c1w1+c2w23;
			r20() = -c1w2+c2w13;	r21() = c1w1+c2w23;		r22() = 1-c2w11-c2w22;
		}
		else
			MakeIdentity();
	}
	inline void ToRodrigues(float *w) const
	{
		const double a = (r00() + r11() + r22() - 1) / 2;
		if(fabs(r01() - r10()) < 0.01 && fabs(r12() - r21()) < 0.01 && fabs(r02() - r20()) < 0.01)
		{
			if(fabs(r01() + r10()) < 0.1 && fabs(r12() + r21()) < 0.1 && fabs(r02() + r20()) < 0.1 && a > 0.9)
				w[0] = w[1] = w[2] = 0;
			else
			{
				const float ha = (float) (sqrt(0.5f) * PI);
				const double xx = (r00() + 1) * 0.5;
				const double yy = (r11() + 1) * 0.5;
				const double zz = (r22() + 1) * 0.5;
				const double xy = (r01() + r10()) * 0.25;
				const double xz = (r02() + r20()) * 0.25;
				const double yz = (r12() + r21()) * 0.25;

				if((xx > yy) && (xx > zz)) 
				{
					if(xx < 0.01) 
					{
						w[0] = 0;
						w[1] = w[2] = ha; 
					}
					else 
					{
						const double t = sqrt(xx) ;
						w[0] = (float) (t * PI);
						w[1] = (float) (xy / t * PI);
						w[2] = (float) (xz / t * PI);
					}
				}
				else if(yy > zz) 
				{ 
					if (yy < 0.01)
					{
						w[0] = w[2] = ha;
						w[1] = 0;
					}
					else
					{
						const double t = sqrt(yy);
						w[0] = (float) (xy / t* PI);
						w[1] = (float) (t * PI);
						w[2] = (float) (yz / t* PI);
					}    
				} else 
				{
					if (zz < 0.01)
					{
						w[0] = w[1] = ha;
						w[2] = 0;
					}
					else
					{
						const double t = sqrt(zz);
						w[0]  = (float) (xz / t * PI);
						w[1]  = (float) (yz / t * PI);
						w[2]  = (float) (t * PI);
					}
				}
			}
		}
		else
		{
			const double b = acos(a), c = 0.5*b/sin(b);
			w[0] = (float) (c * (r21() - r12()));
			w[1] = (float) (c * (r02() - r20()));
			w[2] = (float) (c * (r10() - r01()));
		}
	}
	inline void FromQuaternion(const ENFT_SSE::__m128 &q, ENFT_SSE::__m128 &work)
	{
#if _DEBUG
		const float l2 = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(q, q));
		assert(EQUAL(l2, 1.0f));
#endif
		work = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(q.m128_f32[0] + q.m128_f32[0]), q);
		r11() = r22() = 1 - work.m128_f32[0];
		r01() = r10() = work.m128_f32[1];
		r02() = r20() = work.m128_f32[2];
		r12() = work.m128_f32[3];
		r21() = -work.m128_f32[3];
		work = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(q.m128_f32[1] + q.m128_f32[1]), q);
		r00() = 1 - work.m128_f32[1];
		r22() -= work.m128_f32[1];
		r12() += work.m128_f32[2];
		r21() += work.m128_f32[2];
		r02() -= work.m128_f32[3];
		r20() += work.m128_f32[3];
		work.m128_f32[0] = q.m128_f32[2] + q.m128_f32[2];
		work.m128_f32[1] = work.m128_f32[0] * q.m128_f32[2];
		work.m128_f32[2] = work.m128_f32[0] * q.m128_f32[3];
		r11() -= work.m128_f32[1];
		r00() -= work.m128_f32[1];
		r01() += work.m128_f32[2];
		r10() -= work.m128_f32[2];
	}
	inline void ToQuaternion(float *q) const
	{
		q[3] = r00() + r11() + r22();
		if(q[3] > r00() && q[3] > r11() && q[3] > r22())
		{
			q[3] = sqrt(q[3] + 1) * 0.5f;
			q[2] = 0.25f / q[3];
			q[0] = (r12() - r21()) * q[2];
			q[1] = (r20() - r02()) * q[2];
			q[2] = (r01() - r10()) * q[2];
		}
		else if(r00() > r11() && r00() > r22())
		{
			q[0] = sqrt(r00() + r00() - q[3] + 1) * 0.5f;
			q[3] = 0.25f / q[0];
			q[1] = (r01() + r10()) * q[3];
			q[2] = (r02() + r20()) * q[3];
			q[3] = (r12() - r21()) * q[3];
		}
		else if(r11() > r22())
		{
			q[1] = sqrt(r11() + r11() - q[3] + 1) * 0.5f;
			q[3] = 0.25f / q[1];
			q[0] = (r01() + r10()) * q[3];
			q[2] = (r12() + r21()) * q[3];
			q[3] = (r20() - r02()) * q[3];
		}
		else
		{
			q[2] = sqrt(r22() + r22() - q[3] + 1) * 0.5f;
			q[3] = 0.25f / q[2];
			q[0] = (r02() + r20()) * q[3];
			q[1] = (r12() + r21()) * q[3];
			q[3] = (r01() - r10()) * q[3];
		}
	}
	inline void FromVectorPair(const LA::AlignedVector3f &vFrom, const LA::AlignedVector3f &vTo)
	{
		Point3D axis;
		LA::Cross(vFrom, vTo, axis);
		axis.Normalize();
		FromAxisAngle(axis, acos(LA::Dot(vFrom, vTo)));
	}

	static inline void ComputeRelativeTransformation(const RotationTransformation3D &R1, const RotationTransformation3D &R2, RotationTransformation3D &R12)
	{
		LA::ABT(R2, R1, R12);
	}
};

#endif