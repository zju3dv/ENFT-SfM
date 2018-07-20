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

#ifndef _PROGRAM_GL_DESCRIPTOR_NORMALIZE_H_
#define _PROGRAM_GL_DESCRIPTOR_NORMALIZE_H_

#include "ProgramGL.h"

class ProgramGLDescriptorNormalizePass1 : public ProgramGL
{

public:

	inline void Initialize(const ushort &descTexWidth)
	{
		m_normTexWidth = (descTexWidth >> 4);
		m_normTexWidthLog = ushort(log(float(m_normTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_normTexWidthLog) == m_normTexWidth);
#endif

		char source[MAX_SOURCE_LENGTH];
		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out <<
			"#pragma optionNV(ifcvt none)\n"
			"#pragma optionNV(unroll all)\n"
			"uniform sampler2DRect g_descTex;\n"
			"void main()\n"
			"{\n"
			"	ivec2 dstCoord = ivec2(gl_FragCoord.st);\n"
			"	vec2 srcCoord = ivec2(dstCoord.x << 4, dstCoord.y) + vec2(0.5);\n"
			"	vec4 dot = vec4(0);\n"
			"	for(int i = 0; i < 16; ++i, ++srcCoord.x)\n"
			"	{\n"
			"		vec4 d = texture2DRect(g_descTex, srcCoord);\n"
			"		dot += d * d;\n"
			"	}\n"
			"	gl_FragColor.r = 1 / sqrt(dot.r + dot.g + dot.b + dot.a);\n"
			"}\n" << '\0';

		//printf("%s\n", source);

		std::vector<std::string> inpTexNames(1), inpParamNames;
		inpTexNames[0] = "g_descTex";
		LoadSource(source, inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &descTex, const TextureGL1 &normTex, const ushort &nFtrs) const
	{
#if _DEBUG
		assert(descTex.GetWidth() == (normTex.GetWidth() << 4) && descTex.GetHeight() == normTex.GetHeight() && normTex.GetWidth() == m_normTexWidth);
#endif

		Activate();
		SetInputTexture(descTex);
		SetOutputTexture(normTex);
		DrawQuad(uint(nFtrs), m_normTexWidth, m_normTexWidthLog);
		Deactivate();
	}

private:

	ushort m_normTexWidth, m_normTexWidthLog;

};

class ProgramGLDescriptorNormalizePass2
{

public:

	inline void Initialize(const ushort &descTexWidth)
	{
		m_descTexWidth = descTexWidth;
		m_descTexWidthLog = ushort(log(float(descTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_descTexWidthLog) == descTexWidth);
#endif

		for(ushort i = 0; i < 2; ++i)
		{
			char source[MAX_SOURCE_LENGTH];
			std::ostrstream out(source, MAX_SOURCE_LENGTH);
			out <<
				"#pragma optionNV(ifcvt none)\n"
				"uniform sampler2DRect g_descTex;\n"
				"uniform sampler2DRect g_normTex;\n"
				"void main()\n"
				"{\n"
				"	ivec2 dstCoord = ivec2(gl_FragCoord.st);\n"
				"	vec2 srcCoord = ivec2(dstCoord.x >> 4, dstCoord.y) + vec2(0.5);\n"
				"	float norm = texture2DRect(g_normTex, srcCoord).r;\n"
				"	vec4 d = texture2DRect(g_descTex, gl_FragCoord.st);\n";
			if(i == 0)
				out <<
				"	d *= norm;\n";
			else
				out <<
				"	d = min(d * norm, vec4(0.2));\n";
			out <<
				"	gl_FragColor = d;\n"
				"}\n" << '\0';
			//printf("%s\n", source);
			std::vector<std::string> inpTexNames(2), inpParamNames;
			inpTexNames[0] = "g_descTex";
			inpTexNames[1] = "g_normTex";
			m_programs[i].LoadSource(source, inpTexNames, inpParamNames);
		}
	}

	inline void Run(const TextureGL4 &descTex, const TextureGL1 &normTex, const ushort &nFtrs, const bool &truncate) const
	{
#if _DEBUG
		assert(descTex.GetWidth() == (normTex.GetWidth() << 4) && descTex.GetHeight() == normTex.GetHeight() && descTex.GetWidth() == m_descTexWidth);
#endif

		const ProgramGL &program = m_programs[truncate];
		program.Activate();
		program.SetInputTextures(descTex, normTex);
		program.SetOutputTexture(descTex);
		program.DrawQuad(uint(nFtrs) << 4, m_descTexWidth, m_descTexWidthLog);
		program.Deactivate();
		program.UnbindInputTextures(descTex, normTex);
	}

private:

	ushort m_descTexWidth, m_descTexWidthLog;
	ProgramGL m_programs[2];

};

class ProgramGLDescriptorNormalize
{

public:

	inline void Initialize(const ushort &descTexWidth)
	{
		m_pass1.Initialize(descTexWidth);
		m_pass2.Initialize(descTexWidth);
	}

	inline void Run(const TextureGL4 &descTex, const TextureGL1 &normTex, const ushort &nFtrs, const bool &truncate) const
	{
		if(truncate)
		{
			m_pass1.Run(descTex, normTex, nFtrs);
			m_pass2.Run(descTex, normTex, nFtrs, true);
		}
		m_pass1.Run(descTex, normTex, nFtrs);
		m_pass2.Run(descTex, normTex, nFtrs, false);
	}

private:

	ProgramGLDescriptorNormalizePass1 m_pass1;
	ProgramGLDescriptorNormalizePass2 m_pass2;

};

#endif