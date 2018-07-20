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

#ifndef _OMEGA_MATRIX_H_
#define _OMEGA_MATRIX_H_

#include "LinearAlgebra/Vector3.h"
#include "Quaternion.h"

class OmegaMatrix : public LA::Vector3d
{

public:

	inline const double& wX() const { return v0(); }	inline double& wX() { return v0(); }
	inline const double& wY() const { return v1(); }	inline double& wY() { return v1(); }
	inline const double& wZ() const { return v2(); }	inline double& wZ() { return v2(); }

	static inline void ABmBA(const OmegaMatrix &A, const OmegaMatrix &B, OmegaMatrix &ABmBA)
	{
		LA::Cross(A, B, ABmBA);
		ABmBA *= 2.0f;
	}

	static inline void AB(const OmegaMatrix &O, const Quaternion &q, Quaternion &Oq)
	{
		Oq.v0() =  O.wZ() * q.v1() - O.wY() * q.v2() + O.wX() * q.v3();
		Oq.v1() =  O.wX() * q.v2() - O.wZ() * q.v0() + O.wY() * q.v3();
		Oq.v2() =  O.wY() * q.v0() - O.wX() * q.v1() + O.wZ() * q.v3();
		Oq.v3() = -O.wX() * q.v0() - O.wY() * q.v1() - O.wZ() * q.v2();
		//Oq.Normalize();
#if _DEBUG
		const double l2 = Oq.SquaredLength();
		assert(l2 == 1.0);
#endif
	}

};

#endif