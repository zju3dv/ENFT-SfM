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

#include "LinearAlgebra/Vector4.h"
#include "LinearAlgebra/Matrix3.h"
#include "ProgramGL.h"

template<> inline void ProgramGL::SetInputParameter<int>(const int &param, const GLint &loc) const
{
#if _DEBUG
	assert(loc != -1);
#endif
	glUniform1i(loc, param);
}

template<> inline void ProgramGL::SetInputParameter<float>(const float &param, const GLint &loc) const
{
#if _DEBUG
	assert(loc != -1);
#endif
	glUniform1f(loc, param);
}

template<> inline void ProgramGL::SetInputParameter<LA::Vector2f>(const LA::Vector2f &param, const GLint &loc) const
{
#if _DEBUG
	assert(loc != -1);
#endif
	glUniform2fv(loc, 1, param);
}

template<> inline void ProgramGL::SetInputParameter<LA::Vector4f>(const LA::Vector4f &param, const GLint &loc) const
{
#if _DEBUG
	assert(loc != -1);
#endif
	glUniform4fv(loc, 1, param);
}

template<> inline void ProgramGL::SetInputParameter<LA::Matrix3f>(const LA::Matrix3f &param, const GLint &loc) const
{
#if _DEBUG
	assert(loc != -1);
#endif
	glUniformMatrix3fv(loc, 1, GL_TRUE, param);
}