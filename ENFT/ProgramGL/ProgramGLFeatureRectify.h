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

#ifndef _PROGRAM_GL_FEATURE_RECTIFY_H_
#define _PROGRAM_GL_FEATURE_RECTIFY_H_

#include "ProgramGL.h"

class ProgramGLFeatureRectify : public ProgramGL
{

public:

	inline void Initialize(const ushort &ftrTexWidth)
	{
		m_ftrTexWidth = ftrTexWidth;
		m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
#endif
		char source[MAX_SOURCE_LENGTH];
		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out <<
			"#pragma optionNV(ifcvt none)\n"
			"uniform sampler2DRect g_ftrTex;\n"
			"uniform mat3 g_H;\n"
			"void main()\n"
			"{\n"
			"	vec4 ftr = texture2DRect(g_ftrTex, gl_FragCoord.st);\n"
			"	vec2 x = ftr.xy;\n"
			"	vec3 Hx = g_H * vec3(x, 1);\n"
			"	vec2 y0 = (Hx.xy * (1 / Hx.z));\n"
			"	ftr.xy = y0;\n"
			"	x += vec2(cos(ftr.w), sin(ftr.w)) * ftr.z;\n"
			"	Hx = g_H * vec3(x, 1);\n"
			"	vec2 y1 = (Hx.xy * (1 / Hx.z));\n"
			"	vec2 dy = y1 - y0;\n"
			"	ftr.z = length(dy);\n"
			"	ftr.w = acos(dy.x / ftr.z);\n"
			"	ftr.w = dy.y > 0 ? ftr.w : -ftr.w;\n"
			"	gl_FragColor = ftr;\n"
			"}\n" << '\0';
		//printf("%s\n", source);

		std::vector<std::string> inpTexNames(1), inpParamNames(1);
		inpTexNames[0] = "g_ftrTex";
		inpParamNames[0] = "g_H";
		LoadSource(source, inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &srcFtrTex, const TextureGL4 &dstFtrTex, const ushort &nFtrs, const LA::Matrix3f &H)
	{
#if _DEBUG
		assert(dstFtrTex.GetWidth() == m_ftrTexWidth);
#endif
		Activate();
		SetInputTexture(srcFtrTex);
		SetInputParameter(H);
		SetOutputTexture(dstFtrTex);
		DrawQuad(nFtrs, m_ftrTexWidth, m_ftrTexWidthLog);
		Deactivate();
	}

private:

	ushort m_ftrTexWidth, m_ftrTexWidthLog;

};

#endif