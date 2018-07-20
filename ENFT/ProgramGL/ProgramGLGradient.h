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

#ifndef _PROGRAM_GL_GRADIENT_H_
#define _PROGRAM_GL_GRADIENT_H_

#include "ProgramGL.h"

class ProgramGLGradient : public ProgramGL
{

public:

	inline void Initialize()
	{
		std::vector<std::string> inpTexNames(1), inpParamsNames;
		inpTexNames[0] = "g_imgTex";
		LoadSource(
			"uniform sampler2DRect g_imgTex;\n"
			"void main()\n"
			"{\n"
			"	float center = texture2DRect(g_imgTex, gl_FragCoord.st).r;\n"
			"	float left = texture2DRect(g_imgTex, gl_TexCoord[0].st).r;\n"
			"	float right = texture2DRect(g_imgTex, gl_TexCoord[1].st).r;\n"
			"	float down = texture2DRect(g_imgTex, gl_TexCoord[2].st).r;\n"
			"	float up = texture2DRect(g_imgTex, gl_TexCoord[3].st).r;\n"
			"	gl_FragColor.rgb = vec3(center, (vec2(right, up) - vec2(left, down)) * 0.5);\n"
			"}\n", inpTexNames, inpParamsNames);
	}
	inline void Run(const TextureGL1 &imgTex, const TextureGL4 &imgGradTex) const
	{
		Activate();
		SetInputTexture(imgTex);
		SetOutputTexture(imgGradTex);

		const GLint s1 = 0, t1 = 0, s2 = GLint(imgTex.GetWidth()), t2 = GLint(imgTex.GetHeight());
		const GLint x1 = 0, y1 = 0, x2 = GLint(imgGradTex.GetWidth()), y2 = GLint(imgGradTex.GetHeight());
		glBegin(GL_QUADS);
		glMultiTexCoord2i(GL_TEXTURE0, s1-1, t1  );
		glMultiTexCoord2i(GL_TEXTURE1, s1+1, t1  );
		glMultiTexCoord2i(GL_TEXTURE2, s1  , t1-1);
		glMultiTexCoord2i(GL_TEXTURE3, s1  , t1+1);
		glVertex2i(x1, y1);
		glMultiTexCoord2i(GL_TEXTURE0, s1-1, t2  );
		glMultiTexCoord2i(GL_TEXTURE1, s1+1, t2  );
		glMultiTexCoord2i(GL_TEXTURE2, s1  , t2-1);
		glMultiTexCoord2i(GL_TEXTURE3, s1  , t2+1);
		glVertex2i(x1, y2);
		glMultiTexCoord2i(GL_TEXTURE0, s2-1, t2  );
		glMultiTexCoord2i(GL_TEXTURE1, s2+1, t2  );
		glMultiTexCoord2i(GL_TEXTURE2, s2  , t2-1);
		glMultiTexCoord2i(GL_TEXTURE3, s2  , t2+1);
		glVertex2i(x2, y2);
		glMultiTexCoord2i(GL_TEXTURE0, s2-1, t1  );
		glMultiTexCoord2i(GL_TEXTURE1, s2+1, t1  );
		glMultiTexCoord2i(GL_TEXTURE2, s2  , t1-1);
		glMultiTexCoord2i(GL_TEXTURE3, s2  , t1+1);
		glVertex2i(x2, y1);
		glEnd();
		glFlush();

		Deactivate();
	}
};

class ProgramGLGradientMagnitudeDirectionPacked : public ProgramGL
{

public:

	inline void Initialize()
	{
		std::vector<std::string> inpTexNames(1), inpParamNames;
		inpTexNames[0] = "g_gaussTex";
		LoadSource(
			"uniform sampler2DRect g_gaussTex;\n"
			"void main()\n"
			"{\n"
			"	vec4 center = texture2DRect(g_gaussTex, gl_FragCoord.st);\n"
			"	vec4 left = texture2DRect(g_gaussTex, gl_TexCoord[0].st);\n"
			"	vec4 right = texture2DRect(g_gaussTex, gl_TexCoord[1].st);\n"
			"	vec4 down = texture2DRect(g_gaussTex, gl_TexCoord[2].st);\n"
			"	vec4 up = texture2DRect(g_gaussTex, gl_TexCoord[3].st);\n"
			"	vec4 gx = (vec4(right.rb, center.ga) - vec4(center.rb, left.ga)).brag;\n"
			"	vec4 gy = (vec4(up.rg, center.ba) - vec4(center.rg, down.ba)).barg;\n"
			"	vec4 gradMag = sqrt(gx * gx + gy * gy) * 0.5;\n"
			"	vec4 invalid = vec4(equal(gradMag, vec4(0.0)));\n"
			"	gl_FragData[0] = gradMag;\n"
			"	gl_FragData[1] = atan(gy, gx + invalid);\n"
			"	gl_FragDepth = 1.0f;\n"
			"}\n", inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &gaussTex, const TextureGL4 &gradMagTex, const TextureGL4 &gradDirTex) const
	{
#if _DEBUG
		assert(gaussTex.GetWidth() <= gradMagTex.GetWidth() && gaussTex.GetHeight() <= gradMagTex.GetHeight());
		assert(gaussTex.GetWidth() <= gradDirTex.GetWidth() && gaussTex.GetHeight() <= gradDirTex.GetHeight());
#endif
		Activate();
		SetInputTexture(gaussTex);
		SetOutputTextures(gradMagTex, gradDirTex);

		const GLint s1 = 0, t1 = 0, s2 = GLint(gaussTex.GetWidth()), t2 = GLint(gaussTex.GetHeight());
		const GLint x1 = 0, y1 = 0, x2 = GLint(gaussTex.GetWidth()), y2 = GLint(gaussTex.GetHeight());
		glBegin(GL_QUADS);
		glMultiTexCoord2i(GL_TEXTURE0, s1-1, t1  );
		glMultiTexCoord2i(GL_TEXTURE1, s1+1, t1  );
		glMultiTexCoord2i(GL_TEXTURE2, s1  , t1-1);
		glMultiTexCoord2i(GL_TEXTURE3, s1  , t1+1);
		glVertex2i(x1, y1);
		glMultiTexCoord2i(GL_TEXTURE0, s1-1, t2  );
		glMultiTexCoord2i(GL_TEXTURE1, s1+1, t2  );
		glMultiTexCoord2i(GL_TEXTURE2, s1  , t2-1);
		glMultiTexCoord2i(GL_TEXTURE3, s1  , t2+1);
		glVertex2i(x1, y2);
		glMultiTexCoord2i(GL_TEXTURE0, s2-1, t2  );
		glMultiTexCoord2i(GL_TEXTURE1, s2+1, t2  );
		glMultiTexCoord2i(GL_TEXTURE2, s2  , t2-1);
		glMultiTexCoord2i(GL_TEXTURE3, s2  , t2+1);
		glVertex2i(x2, y2);
		glMultiTexCoord2i(GL_TEXTURE0, s2-1, t1  );
		glMultiTexCoord2i(GL_TEXTURE1, s2+1, t1  );
		glMultiTexCoord2i(GL_TEXTURE2, s2  , t1-1);
		glMultiTexCoord2i(GL_TEXTURE3, s2  , t1+1);
		glVertex2i(x2, y1);
		glEnd();
		glFlush();

		Deactivate();
		DetachOutputTextures(gradMagTex, gradDirTex);
	}
};

#endif