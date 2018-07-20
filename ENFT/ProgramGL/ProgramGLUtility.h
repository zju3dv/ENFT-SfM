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

#ifndef _PROGRAM_GL_UTILITY_H_
#define _PROGRAM_GL_UTILITY_H_

#include "ProgramGL.h"

class ProgramGLCopy : public ProgramGL
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
			"	gl_FragColor = texture2DRect(g_srcTex, gl_FragCoord.st);\n"
			"}\n", inpTexNames, inpParamNames);
	}
	template<ushort CHANNELS_NUMBER_SRC, ushort CHANNELS_NUMBER_DST>
	inline void Run(const TextureGL<CHANNELS_NUMBER_SRC> &srcTex, const TextureGL<CHANNELS_NUMBER_DST> &dstTex) const
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

class ProgramGLScale : public ProgramGL
{

public:

	inline void Initialize()
	{
		std::vector<std::string> inpTexNames(1), inpParamNames(1);
		inpTexNames[0] = "g_srcTex";
		inpParamNames[0] = "g_scale";
		LoadSource(
			"uniform sampler2DRect g_srcTex;\n"
			"uniform float g_scale;\n"
			"void main()\n"
			"{\n"
			"	gl_FragColor = texture2DRect(g_srcTex, gl_FragCoord.st) * g_scale;\n"
			"}\n", inpTexNames, inpParamNames);
	}
	template<ushort CHANNELS_NUMBER_SRC, ushort CHANNELS_NUMBER_DST>
	inline void Run(const float &scale, const TextureGL<CHANNELS_NUMBER_SRC> &srcTex, const TextureGL<CHANNELS_NUMBER_DST> &dstTex) const
	{
#if _DEBUG
		assert(dstTex.GetWidth() == srcTex.GetWidth() && dstTex.GetHeight() == srcTex.GetHeight());
#endif
		Activate();
		SetInputTexture(srcTex);
		SetInputParameter(scale);
		SetOutputTexture(dstTex);
		DrawQuad(dstTex.GetWidth(), dstTex.GetHeight());
		Deactivate();
	}
	template<ushort CHANNELS_NUMBER>
	inline void Run(const float &scale, const TextureGL<CHANNELS_NUMBER> &tex) const
	{
		Activate();
		SetInputTexture(tex);
		SetInputParameter(scale);
		SetOutputTexture(tex);
		DrawQuad(tex.GetWidth(), tex.GetHeight());
		Deactivate();
	}
};

class ProgramGLResize : public ProgramGL
{

public:

	inline void Initialize()
	{
		char source[MAX_SOURCE_LENGTH];
		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out <<
			"uniform sampler2DRect g_srcTex;\n"
			"uniform vec2 g_sizeRatio;\n"
			"void main()\n"
			"{\n"
			"	vec2 dstCoord = floor(gl_FragCoord.st);\n"
			"	vec2 srcCoord = dstCoord * g_sizeRatio + vec2(0.5);\n"
			"	gl_FragColor = texture2DRect(g_srcTex, srcCoord);\n"
			"}\n" << '\0';

		std::vector<std::string> inpTexNames(1), inpParamNames(1);
		inpTexNames[0] = "g_srcTex";
		inpParamNames[0] = "g_sizeRatio";
		LoadSource(source, inpTexNames, inpParamNames);
	}
	template<ushort CHANNELS_NUMBER>
	inline void Run(const TextureGL<CHANNELS_NUMBER> &srcTex, const TextureGL<CHANNELS_NUMBER> &dstTex) const
	{
		Activate();
		SetInputTexture(srcTex);
		const LA::Vector2f sizeRatio(float(srcTex.GetWidth()) / dstTex.GetWidth(), float(srcTex.GetHeight()) / dstTex.GetHeight());
		SetInputParameter(sizeRatio);
		SetOutputTexture(dstTex);
		DrawQuad(dstTex.GetWidth(), dstTex.GetHeight());
		Deactivate();
	}
};

class ProgramGLClearColorDepth : public ProgramGL
{

public:

	inline void Initialize()
	{
		std::vector<std::string> inpTexNames, inpParamNames;
		LoadSource(
			"void main()\n"
			"{\n"
			"	gl_FragColor = vec4(0);\n"
			"	gl_FragDepth = 0.0f;\n"
			"}\n", inpTexNames, inpParamNames);
	}
	template<ushort CHANNELS_NUMBER>
	inline void Run(const TextureGL<CHANNELS_NUMBER> &tex) const
	{
		Activate();
		SetOutputTexture(tex);
		DrawQuad(tex.GetWidth(), tex.GetHeight());
		Deactivate();
	}
};

class ProgramGLConvertRGB2Gray : public ProgramGL
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
			"	gl_FragColor.r = dot(texture2DRect(g_srcTex, gl_FragCoord.st).rgb, vec3(0.2989959716796875, 0.587005615234375, 0.1139984130859375));\n"
			"}\n", inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &srcTex, const TextureGL1 &dstTex) const
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
	inline void Run(const TextureGL3 &srcTex, const TextureGL1 &dstTex) const
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

class ProgramGLConvertGray2RGB : public ProgramGL
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
			"	gl_FragColor.rgb = vec3(texture2DRect(g_srcTex, gl_FragCoord.st).r);\n"
			"}\n", inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL1 &srcTex, const TextureGL4 &dstTex) const
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

class ProgramGLConvertRGB2GrayResize : public ProgramGL
{

public:

	inline void Initialize()
	{
		char source[MAX_SOURCE_LENGTH];
		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out <<
			"uniform sampler2DRect g_srcTex;\n"
			"uniform vec2 g_sizeRatio;\n"
			"void main()\n"
			"{\n"
			"	vec2 dstCoord = floor(gl_FragCoord.st);\n"
			"	vec2 srcCoord = dstCoord * g_sizeRatio + vec2(0.5);\n"
			"	gl_FragColor.r = dot(texture2DRect(g_srcTex, srcCoord).rgb, vec3(0.2989959716796875, 0.587005615234375, 0.1139984130859375));\n"
			"}\n" << '\0';

		std::vector<std::string> inpTexNames(1), inpParamNames(1);
		inpTexNames[0] = "g_srcTex";
		inpParamNames[0] = "g_sizeRatio";
		LoadSource(source, inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &srcTex, const TextureGL1 &dstTex) const
	{
		Activate();
		SetInputTexture(srcTex);
		const LA::Vector2f sizeRatio(float(srcTex.GetWidth()) / dstTex.GetWidth(), float(srcTex.GetHeight()) / dstTex.GetHeight());
		SetInputParameter(sizeRatio);
		SetOutputTexture(dstTex);
		DrawQuad(dstTex.GetWidth(), dstTex.GetHeight());
		Deactivate();
	}
	inline void Run(const TextureGL3 &srcTex, const TextureGL1 &dstTex) const
	{
		Activate();
		SetInputTexture(srcTex);
		const LA::Vector2f sizeRatio(float(srcTex.GetWidth()) / dstTex.GetWidth(), float(srcTex.GetHeight()) / dstTex.GetHeight());
		SetInputParameter(sizeRatio);
		SetOutputTexture(dstTex);
		DrawQuad(dstTex.GetWidth(), dstTex.GetHeight());
		Deactivate();
	}
};

#endif