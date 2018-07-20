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

#ifndef _RELATIVE_TRANSLATION_H_
#define _RELATIVE_TRANSLATION_H_

#include "Translation.h"
#include "EssentialMatrix.h"

class RelativeTranslation : public Translation
{

public:

	inline float ComputeSuqaredError(const Point3D &Rx1, const Point2D &x2, float *work3) const
	{
		float &l2x = work3[0], &l2y = work3[1], &x2TEx1 = work3[2];
		l2x = tY() * Rx1.Z() - tZ() * Rx1.Y();
		l2y = tZ() * Rx1.X() - tX() * Rx1.Z();
		x2TEx1 = x2.x() * l2x + x2.y() * l2y - tY() * Rx1.X() + tX() * Rx1.Y();
		return x2TEx1 * x2TEx1 / (l2x * l2x + l2y * l2y);
	}

	inline bool ToRelativePose(const MatchSet2D &xs, const RotationTransformation3D &R, RigidTransformation3D &T, EssentialMatrix &E, const float &sccRatioTh, 
							   ENFT_SSE::__m128 *work28) const
	{
		T.Set(R, *this);
		E.FromRelativePose(T);
		ushort cntPos, cntNeg;
		EssentialMatrix::CheckCheirality(E, T, xs, cntPos, cntNeg, work28);
		const ushort cntPosTh = ushort(xs.Size() * sccRatioTh + 0.5f);
		if(cntPos > cntNeg && cntPos >= cntPosTh)
			return true;
		else if(cntPos < cntNeg && cntNeg >= cntPosTh)
		{
			T.tX() = -T.tX();
			T.tY() = -T.tY();
			T.tZ() = -T.tZ();
			return true;
		}
		else
			return false;
	}
};

#endif