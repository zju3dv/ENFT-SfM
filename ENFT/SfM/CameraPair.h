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

#ifndef _CAMERA_PAIR_H_
#define _CAMERA_PAIR_H_

#include "Camera.h"
#include "SimilarityTransformation.h"

class CameraPair
{

public:

	inline const SimilarityTransformation3D& S() const { return m_S; }	inline SimilarityTransformation3D& S() { return m_S; }
	inline const Camera& C12() const { return m_C12; }					inline Camera& C12() { return m_C12; }
	inline const Camera& C21() const { return m_C21; }					inline Camera& C21() { return m_C21; }

private:

	SimilarityTransformation3D m_S;
	Camera m_C12, m_C21;
};

#endif