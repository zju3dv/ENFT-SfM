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

#ifndef PROGRAM_GL_FEATURE_COPY_H_
#define PROGRAM_GL_FEATURE_COPY_H_

#include "ProgramGL.h"

class ProgramGLFeatureCopy : public ProgramGL
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
			"uniform sampler2DRect g_srcFtrTex;\n"
			"void main()\n"
			"{\n"
			"	gl_FragColor = texture2DRect(g_srcFtrTex, gl_FragCoord.st);\n"
			"}\n" << '\0';
		std::vector<std::string> inpTexNames(1), inpParamNames;
		inpTexNames[0] = "g_srcFtrTex";
		LoadSource(source, inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &srcFtrTex, const TextureGL4 &dstFtrTex, const uint &nFtrs) const
	{
		if(srcFtrTex.GetTexture() == dstFtrTex.GetTexture())
			return;

#if _DEBUG
		assert(srcFtrTex.GetWidth() == m_ftrTexWidth && dstFtrTex.GetWidth() == m_ftrTexWidth);
#endif

		Activate();
		SetInputTexture(srcFtrTex);
		SetOutputTexture(dstFtrTex);
		DrawQuad(nFtrs, m_ftrTexWidth, m_ftrTexWidthLog);
		Deactivate();
	}

private:

	ushort m_ftrTexWidth, m_ftrTexWidthLog;

};

#endif