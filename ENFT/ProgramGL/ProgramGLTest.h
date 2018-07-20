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

#ifndef _PROGRAM_GL_TEST_H_
#define _PROGRAM_GL_TEST_H_

#include "ProgramGL.h"

class ProgramGLTest : public ProgramGL
{

public:

	inline void Initialize()
	{
		std::vector<std::string> inpTexNames(1), inpParamNames;
		inpTexNames[0] = "g_srcTex";
		LoadSource(
			"uniform sampler2DRect g_srcTex;\n"
			"void main()\n"
			"{\n"
			"	vec2 coord = gl_FragCoord.st;\n"
			//"	vec2 coord = floor(gl_FragCoord.st);\n"
			"	gl_FragColor = texture2DRect(g_srcTex, coord);\n"
			"}\n", inpTexNames, inpParamNames);
	}
	template<ushort CHANNELS_NUMBER>
	inline void Run(const TextureGL<CHANNELS_NUMBER> &srcTex, const TextureGL<CHANNELS_NUMBER> &dstTex) const
	{
#if _DEBUG
		assert(dstTex.GetWidth() == srcTex.GetWidth() && dstTex.GetHeight() == srcTex.GetHeight());
#endif
		Activate();
		SetInputTexture(srcTex);
		SetOutputTexture(dstTex);
		DrawQuad(dstTex.GetWidth(), dstTex.GetHeight());
		Deactivate();
	}
};

#endif