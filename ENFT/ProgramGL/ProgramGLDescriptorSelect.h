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

#ifndef PROGRAM_GL_DESCRIPTOR_SELECT_H_
#define PROGRAM_GL_DESCRIPTOR_SELECT_H_

#include "ProgramGL.h"

class ProgramGLDescriptorSelect : public ProgramGL
{

public:

	inline void Initialize(const ushort &descTexWidth)
	{
		m_descTexWidth = descTexWidth;
		m_descTexWidthLog = ushort(log(float(descTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_descTexWidthLog) == descTexWidth);
#endif

		char source[MAX_SOURCE_LENGTH];
		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out <<
			"uniform sampler2DRect g_srcDescTex;\n"
			"uniform sampler2DRect g_idxTex;\n"
			"void main()\n"
			"{\n"
			"	ivec2 dstCoord = ivec2(gl_FragCoord.st);\n"
			"	vec2 idxCoord = ivec2(dstCoord.x >> 4, dstCoord.y) + vec2(0.5);\n"
			"	int iFtr = int(texture2DRect(g_idxTex, idxCoord).r);\n"
			"	int iPix = (iFtr << 4) + (dstCoord.x & 15);\n"
			"	vec2 srcCoord = ivec2(iPix & " << descTexWidth - 1 << ", iPix >> " << m_descTexWidthLog << ") + vec2(0.5);\n"
			"	gl_FragColor = texture2DRect(g_srcDescTex, srcCoord);\n"
			"}\n" << '\0';
		std::vector<std::string> inpTexNames(2), inpParamNames;
		inpTexNames[0] = "g_srcDescTex";
		inpTexNames[1] = "g_idxTex";
		LoadSource(source, inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &srcDescTex, const TextureGL4 &dstDescTex, const TextureGL1 &idxTex, const ushort &nFtrs) const
	{
#if _DEBUG
		assert(srcDescTex.GetWidth() == m_descTexWidth && dstDescTex.GetWidth() == m_descTexWidth && idxTex.GetWidth() == (m_descTexWidth >> 4));
		assert(srcDescTex.GetTexture() != dstDescTex.GetTexture());
#endif

		Activate();
		SetInputTextures(srcDescTex, idxTex);
		SetOutputTexture(dstDescTex);
		DrawQuad(uint(nFtrs) << 4, m_descTexWidth, m_descTexWidthLog);
		Deactivate();
		UnbindInputTextures(srcDescTex, idxTex);
	}

private:

	ushort m_descTexWidth, m_descTexWidthLog;

};

#endif