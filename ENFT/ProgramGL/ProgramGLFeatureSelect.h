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

#ifndef PROGRAM_GL_FEATURE_SELECT_H_
#define PROGRAM_GL_FEATURE_SELECT_H_

#include "ProgramGL.h"

class ProgramGLFeatureSelect
{

public:

	inline void Initialize(const ushort &ftrTexWidth)
	{
		m_ftrTexWidth = ftrTexWidth;
		m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
#endif

		for(ushort i = 0; i < 2; ++i)
		{
			if(i == 0)
			{
				char source[MAX_SOURCE_LENGTH];
				std::ostrstream out(source, MAX_SOURCE_LENGTH);
				out <<
					"uniform sampler2DRect g_ftrTexSrc;\n"
					"uniform sampler2DRect g_idxTex;\n"
					"void main()\n"
					"{\n"
					"	int idx = int(texture2DRect(g_idxTex, gl_FragCoord.st).r);\n"
					"	vec2 srcCoord = ivec2(idx & " << ftrTexWidth - 1 << ", idx >> " << m_ftrTexWidthLog << ") + vec2(0.5);\n"
					"	gl_FragColor = texture2DRect(g_ftrTexSrc, srcCoord);\n"
					"}\n" << '\0';
				std::vector<std::string> inpTexNames(2), inpParamNames;
				inpTexNames[0] = "g_ftrTexSrc";
				inpTexNames[1] = "g_idxTex";
				m_programs[i].LoadSource(source, inpTexNames, inpParamNames);
			}
			else
			{
				char source[MAX_SOURCE_LENGTH];
				std::ostrstream out(source, MAX_SOURCE_LENGTH);
				out <<
					"uniform sampler2DRect g_ftrTexSrc1;\n"
					"uniform sampler2DRect g_ftrTexSrc2;\n"
					"uniform sampler2DRect g_idxTex12;\n"
					"void main()\n"
					"{\n"
					"	ivec2 idx12 = ivec2(texture2DRect(g_idxTex12, gl_FragCoord.st).rg);\n"
					"	vec4 srcCoord = ivec4(idx12 & " << ftrTexWidth - 1 << ", idx12 >> " << m_ftrTexWidthLog << ") + vec4(0.5);\n"
					"	gl_FragColor = vec4(texture2DRect(g_ftrTexSrc1, srcCoord.rb).rg, texture2DRect(g_ftrTexSrc2, srcCoord.ga).rg);\n"
					"}\n" << '\0';
				//printf("%s\n", source);
				std::vector<std::string> inpTexNames(3), inpParamNames;
				inpTexNames[0] = "g_ftrTexSrc1";
				inpTexNames[1] = "g_ftrTexSrc2";
				inpTexNames[2] = "g_idxTex12";
				m_programs[i].LoadSource(source, inpTexNames, inpParamNames);
			}
		}
	}
	template<ushort CHANNELS_NUMBER>
	inline void Run(const TextureGL<CHANNELS_NUMBER> &ftrTexSrc, const TextureGL<CHANNELS_NUMBER> &ftrTexDst, const TextureGL1 &idxTex, const uint &nFtrs) const
	{
#if _DEBUG
		assert(ftrTexSrc.GetWidth() == m_ftrTexWidth && ftrTexDst.GetWidth() == m_ftrTexWidth && idxTex.GetWidth() == m_ftrTexWidth);
		assert(ftrTexSrc.GetTexture() != ftrTexDst.GetTexture());
#endif
		m_programs[0].Activate();
		m_programs[0].SetInputTextures(ftrTexSrc, idxTex);
		m_programs[0].SetOutputTexture(ftrTexDst);
		m_programs[0].DrawQuad(nFtrs, m_ftrTexWidth, m_ftrTexWidthLog);
		m_programs[0].UnbindInputTextures(ftrTexSrc, idxTex);
		m_programs[0].Deactivate();
	}
	template<ushort CHANNELS_NUMBER>
	inline void Run(const TextureGL<CHANNELS_NUMBER> &ftrTexSrc1, const TextureGL<CHANNELS_NUMBER> &ftrTexSrc2, const TextureGL4 &ftrTexDst, 
					const TextureGL2 &idxTex12, const uint &nFtrs) const
	{
#if _DEBUG
		assert(ftrTexSrc1.GetWidth() == m_ftrTexWidth && ftrTexSrc2.GetWidth() == m_ftrTexWidth && ftrTexDst.GetWidth() == m_ftrTexWidth && idxTex12.GetWidth() == m_ftrTexWidth);
		assert(ftrTexSrc1.GetTexture() != ftrTexDst.GetTexture() && ftrTexSrc2.GetTexture() != ftrTexDst.GetTexture());
#endif

		m_programs[1].Activate();
		m_programs[1].SetInputTextures(ftrTexSrc1, ftrTexSrc2, idxTex12);
		m_programs[1].SetOutputTexture(ftrTexDst);
		m_programs[1].DrawQuad(nFtrs, m_ftrTexWidth, m_ftrTexWidthLog);
		m_programs[1].UnbindInputTextures(ftrTexSrc1, ftrTexSrc2, idxTex12);
		m_programs[1].Deactivate();
	}

private:

	ushort m_ftrTexWidth, m_ftrTexWidthLog;
	ProgramGL m_programs[2];

};

#endif