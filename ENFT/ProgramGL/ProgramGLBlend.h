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

#ifndef _PROGRAM_GL_BLEND_H_
#define _PROGRAM_GL_BLEND_H_

#include "ProgramGL.h"

class ProgramGLFillAlpha : public ProgramGL
{

public:

	inline void Initialize()
	{
		std::vector<std::string> inpTexNames(1), inpParamNames(0);
		inpTexNames[0] = "g_srcTex";
		LoadSource(
			"uniform sampler2DRect g_srcTex;\n"
			"void main()\n"
			"{\n"
			"	vec4 src = texture2DRect(g_srcTex, gl_FragCoord.st);\n"
			"	gl_FragColor = vec4(src.rgb, 1);\n"
			"}\n", inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &tex) const
	{
		Activate();
		SetInputTexture(tex);
		SetOutputTexture(tex);
		DrawQuad(tex.GetWidth(), tex.GetHeight());
		Deactivate();
	}
};

class ProgramGLAlphaWeightedSum : public ProgramGL
{

public:

	inline void Initialize()
	{
		std::vector<std::string> inpTexNames(3), inpParamNames(0);
		inpTexNames[0] = "g_srcTex";
		inpTexNames[1] = "g_dstTex";
		inpTexNames[2] = "g_alphaTex";
		LoadSource(
			"#pragma optionNV(ifcvt none)\n"
			"uniform sampler2DRect g_srcTex;\n"
			"uniform sampler2DRect g_dstTex;\n"
			"uniform sampler2DRect g_alphaTex;\n"
			"void main()\n"
			"{\n"
			"	if(texture2DRect(g_alphaTex, gl_FragCoord.st).a >= 1)\n"
			"	{\n"
			"		gl_FragColor = vec4(0);\n"
			"		return;\n"
			"	}\n"
			"	vec4 src = texture2DRect(g_srcTex, gl_FragCoord.st);\n"
			"	vec4 dst = texture2DRect(g_dstTex, gl_FragCoord.st);\n"
			"	if(src.a == 0)\n"
			"	{\n"
			"		gl_FragColor = dst;\n"
			"		return;\n"
			"	}\n"
			"	gl_FragColor.rgb = src.rgb * src.a + dst.rgb;\n"
			"	gl_FragColor.a = src.a + dst.a;\n"
			"}\n", inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &srcTex, const TextureGL4 &dstTex, const TextureGL4 &alphaTex) const
	{
#if _DEBUG
		assert(srcTex.GetWidth() == dstTex.GetWidth() && srcTex.GetHeight() == dstTex.GetHeight());
#endif
		Activate();
		SetInputTextures(srcTex, dstTex, alphaTex);
		SetOutputTexture(dstTex);
		DrawQuad(dstTex.GetWidth(), dstTex.GetHeight());
		Deactivate();
	}
};

class ProgramGLDividedByAlpha : public ProgramGL
{

public:

	inline void Initialize()
	{
		std::vector<std::string> inpTexNames(1), inpParamNames(0);
		inpTexNames[0] = "g_srcTex";
		LoadSource(
			"#pragma optionNV(ifcvt none)\n"
			"uniform sampler2DRect g_srcTex;\n"
			"void main()\n"
			"{\n"
			"	vec4 src = texture2DRect(g_srcTex, gl_FragCoord.st);\n"
			"	if(src.a == 0)\n"
			"		gl_FragColor = src;\n"
			"	else\n"
			"		gl_FragColor = vec4(src.rgb / src.a, src.a);\n"
			"}\n", inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &tex) const
	{
		Activate();
		SetInputTexture(tex);
		SetOutputTexture(tex);
		DrawQuad(tex.GetWidth(), tex.GetHeight());
		Deactivate();
	}
};

class ProgramGLAccumulateAlpha : public ProgramGL
{

public:

	inline void Initialize()
	{
		std::vector<std::string> inpTexNames(2), inpParamNames(0);
		inpTexNames[0] = "g_srcTex";
		inpTexNames[1] = "g_dstTex";
		LoadSource(
			"uniform sampler2DRect g_srcTex;\n"
			"uniform sampler2DRect g_dstTex;\n"
			"void main()\n"
			"{\n"
			"	float src = texture2DRect(g_srcTex, gl_FragCoord.st).a;\n"
			"	float dst = texture2DRect(g_dstTex, gl_FragCoord.st).a;\n"
			"	float a = src + dst;\n"
			"	gl_FragColor = vec4(0, 0, 0, min(1.0, a));\n"
			"}\n", inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &srcTex, const TextureGL4 &dstTex) const
	{
#if _DEBUG
		assert(srcTex.GetWidth() == dstTex.GetWidth() && srcTex.GetHeight() == dstTex.GetHeight());
#endif
		Activate();
		SetInputTextures(srcTex, dstTex);
		SetOutputTexture(dstTex);
		DrawQuad(dstTex.GetWidth(), dstTex.GetHeight());
		Deactivate();
	}
};

class ProgramGLAlphaWeightedAccumulate : public ProgramGL
{

public:

	inline void Initialize()
	{
		std::vector<std::string> inpTexNames(3), inpParamNames(0);
		inpTexNames[0] = "g_srcTex";
		inpTexNames[1] = "g_dstTex";
		inpTexNames[2] = "g_alphaTex";
		LoadSource(
			"#pragma optionNV(ifcvt none)\n"
			"uniform sampler2DRect g_srcTex;\n"
			"uniform sampler2DRect g_dstTex;\n"
			"uniform sampler2DRect g_alphaTex;\n"
			"void main()\n"
			"{\n"
			"	if(texture2DRect(g_alphaTex, gl_FragCoord.st).a >= 1)\n"
			"	{\n"
			"		gl_FragColor = vec4(0);\n"
			"		return;\n"
			"	}\n"
			"	vec4 src = texture2DRect(g_srcTex, gl_FragCoord.st);\n"
			"	vec4 dst = texture2DRect(g_dstTex, gl_FragCoord.st);\n"
			"	if(src.a != 0 && dst.a < 1)\n"
			"	{\n"
			"		float a = src.a + dst.a;\n"
			"		gl_FragColor.rgb = src.rgb * (src.a / a) + dst.rgb * (dst.a / a);\n"
			"		gl_FragColor.a = min(1.0, a);\n"
			//"		gl_FragColor = src + (1 - src.a) * dst.a * dst;\n"
			"	}\n"
			"	else\n"
			"		gl_FragColor = dst;\n"
			"}\n", inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &srcTex, const TextureGL4 &dstTex, const TextureGL4 &alphaTex) const
	{
#if _DEBUG
		assert(srcTex.GetWidth() == dstTex.GetWidth() && srcTex.GetHeight() == dstTex.GetHeight());
#endif
		Activate();
		SetInputTextures(srcTex, dstTex, alphaTex);
		SetOutputTexture(dstTex);
		DrawQuad(dstTex.GetWidth(), dstTex.GetHeight());
		Deactivate();
	}
};

#endif