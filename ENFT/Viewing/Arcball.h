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

#ifndef _ARCBALL_H_
#define _ARCBALL_H_

#include "SfM/RotationTransformation.h"
#include "SfM/Point.h"
#include "LinearAlgebra/Matrix4.h"
#include <GL/glew.h>
#include <GL/glut.h>

class Arcball
{
	
public:

	inline void Initialize()
	{
		m_C.SetZero();
		m_OC = m_OC2 = 0;
		m_nC.SetZero();
		//m_r = m_r2 = 0;
		SetRadius(0);
		m_R.MakeIdentity();
		m_RT.MakeIdentity();
		m_Cstart = m_C;
		m_Rstart = m_R;
	}
	inline void Initialize(const RotationTransformation3D &R, const float &zCenter, const float &radius)
	{
		m_C.Set(0, 0, zCenter);
		m_OC = fabs(zCenter);
		m_OC2 = zCenter * zCenter;
		m_nC.Set(0, 0, 1);
		SetRadius(radius);
		m_R = R;
		m_R.GetTranspose(m_RT);
		m_Cstart = m_C;
		m_rStart = m_r;
		m_Rstart = m_R;
	}
	inline void Initialize(const float &zCenter, const float &radius)
	{
		m_C.Set(0, 0, zCenter);
		m_OC = fabs(zCenter);
		m_OC2 = zCenter * zCenter;
		m_nC.Set(0, 0, 1);
		SetRadius(radius);
		m_R.MakeIdentity();
		m_RT.MakeIdentity();
		m_Cstart = m_C;
		m_rStart = m_r;
		m_Rstart = m_R;
	}
	inline void Initialize(const Point3D &center, const float &radius)
	{
		m_C = center;
		m_OC2 = m_C.SquaredLength();
		m_OC = sqrt(m_OC2);
		LA::sA(1 / m_OC, m_C, m_nC);
		SetRadius(radius);
		m_R.MakeIdentity();
		m_RT.MakeIdentity();
		m_Cstart = m_C;
		m_rStart = m_r;
		m_Rstart = m_R;
	}
	//inline void MakeIdentity()
	//{
	//	m_R.MakeIdentity();
	//	m_RT.MakeIdentity();
	//}
	inline void Set(const float &centerZ, const float &radius)
	{
		m_C.Set(0, 0, centerZ);
		m_OC = centerZ;
		m_OC2 = m_OC * m_OC;
		m_nC.Set(0, 0, 1);
		SetRadius(radius);
	}
	inline const Point3D& GetCenter() const { return m_C; }
	inline void SetCenter(const Point3D &center)
	{
		m_C = center;
		m_OC2 = m_C.SquaredLength();
		m_OC = sqrt(m_OC2);
		LA::sA(1 / m_OC, m_C, m_nC);
		ComputeTangentFoot();
	}
	inline float GetCenterZ() const { return m_C.v2(); }
	inline float GetRadius() const { return m_r; }
	inline const RotationTransformation3D& GetRotation() const { return m_R; }

	inline void StartChangingRadius() { m_rStart = m_r; }
	inline void ChangeRadius(const float &ratio) { SetRadius(m_rStart * ratio); }
	inline void StartTranslatingCenter() { m_Cstart = m_C; }
	inline void TranslateCenter(const Point3D &dC)
	{
		LA::ApB(m_Cstart, dC, m_C);
		m_OC2 = m_C.SquaredLength();
		m_OC = sqrt(m_OC2);
		LA::sA(1 / m_OC, m_C, m_nC);
		ComputeTangentFoot();
	}
	inline void TranslateCenterXY(const float &dX, const float &dY)
	{
		m_C.X() = m_Cstart.X() + dX;
		m_C.Y() = m_Cstart.Y() + dY;
		m_OC2 = m_C.SquaredLength();
		m_OC = sqrt(m_OC2);
		LA::sA(1 / m_OC, m_C, m_nC);
		ComputeTangentFoot();
	}
	inline void StartRotation(const Point3D &Xwin)
	{
		m_Rstart = m_R;
		ComputeCenterToIntersectionDirection(Xwin, m_nCXstart);
	}
	inline void ComputeRotation(const Point3D &Xwin)
	{
		Point3D nCX;
		ComputeCenterToIntersectionDirection(Xwin, nCX);

		RotationTransformation3D dR;
		dR.FromVectorPair(m_nCXstart, nCX);

		ENFT_SSE::__m128 work;
		LA::AB(dR, m_Rstart, m_R, work);
		m_R.GetTranspose(m_RT);
	}
	inline void ApplyTransformation() const
	{
		glTranslatef(m_C.X(), m_C.Y(), m_C.Z());
		glMultMatrixf(m_RT);
		glTranslatef(-m_C.X(), -m_C.Y(), -m_C.Z());
	}
	inline void DrawSphere(const GLint &slices, const GLint &stacks) const
	{
		glPushMatrix();
		glTranslatef(m_C.X(), m_C.Y(), m_C.Z());
		glMultMatrixf(m_RT);
		glutWireSphere(GLdouble(m_r), slices, stacks);
		glPopMatrix();
	}
	inline void SaveB(FILE *fp) const { fwrite(this, sizeof(Arcball), 1, fp); }
	inline void LoadB(FILE *fp) { fread(this, sizeof(Arcball), 1, fp); }

private:

	inline void SetRadius(const float &radius)
	{
		m_r = radius;
		m_r2 = radius * radius;
		ComputeTangentFoot();
	}
	inline void ComputeTangentFoot()
	{
		m_OTfoot = m_OC - m_r2 / m_OC;
		LA::sA(m_OC, m_nC, m_Tfoot);
	}
	inline void ComputeCenterToIntersectionDirection(const Point3D &Xwin, Point3D &nCX) const
	{
		float a = Xwin.SquaredLength();
		float b = -Xwin.Dot(m_C);
		float c = m_OC2 - m_r2;
		float root = b * b - a * c;
		float t;
		if(root > 0)
		{
			t = (-b - sqrt(root)) / a;
			LA::sA(t, Xwin, nCX);
			nCX -= m_C;
			nCX.Normalize();
		}
		else
		{
			// Tfoot is perpendicular foot of the tangent point T on OC
			// Xext is the intersection of OXwin and TfootT
			// |OXext| / |OTfoot| = |OXwin| / |OXfoot|
			Point3D Xext;
			LA::sA(m_OTfoot / Xwin.Dot(m_nC), Xwin, Xext);

			// T = Xext + XextTfoot * t
			// TC = XextT - CXext = XextTfoot * t - CXext
			// |TC| = |XextTfoot * t - CXext| = R
			Point3D CXext, XextTfoot, T;
			LA::AmB(Xext, m_C, CXext);
			LA::AmB(m_Tfoot, Xext, XextTfoot);
			a = XextTfoot.SquaredLength();
			b = XextTfoot.Dot(XextTfoot);
			c = CXext.SquaredLength() - m_r2;
			t = (-b - sqrt(b * b - a * c)) / a;
			LA::sApB(t, XextTfoot, Xext, T);
			LA::AmB(T, m_C, nCX);
			nCX.Normalize();
		}
	}

private:

	Point3D m_C, m_Cstart, m_nC, m_nCXstart, m_Tfoot;
	RotationTransformation3D m_R, m_Rstart;
	LA::Matrix4f m_RT;
	float m_OC, m_OC2, m_r, m_r2, m_rStart, m_OTfoot;

};

#endif
