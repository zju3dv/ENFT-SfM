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

#ifndef _PROGRAM_GL_FEATURE_LEVELIZE_H_
#define _PROGRAM_GL_FEATURE_LEVELIZE_H_

#include "ProgramGL.h"

class ProgramGLFeatureLevelize : public ProgramGL
{

public:

	inline void Initialize(const ushort &levelTexWidth, const ushort &nOctave, const ushort &nLevelsDoG, const float &sigma0)
	{
		m_levelTexWidth = levelTexWidth;
		m_levelTexWidthLog = ushort(log(float(levelTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_levelTexWidthLog) == levelTexWidth);
#endif

		char source[MAX_SOURCE_LENGTH];
		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out <<
			"uniform sampler2DRect g_ftrTex;\n"
			"void main()\n"
			"{\n"
			"	vec4 s = vec4(texture2DRect(g_ftrTex, gl_TexCoord[0].st).b, \n"
			"				  texture2DRect(g_ftrTex, gl_TexCoord[1].st).b, \n"
			"				  texture2DRect(g_ftrTex, gl_TexCoord[2].st).b, \n"
			"				  texture2DRect(g_ftrTex, gl_TexCoord[3].st).b);\n"
			"	gl_FragColor = clamp(round(log(s * " << 1 / sigma0 << ") * " << nLevelsDoG / log(2.0f) << "), vec4(0), vec4(" << nOctave * nLevelsDoG - 1 << "));\n"
			//"	vec4 idx = log(s * " << 1 / sigma0 << ") * " << nLevelsDoG / log(2.0f) << ";\n"
			//"	gl_FragColor = idx - round(idx);\n"
			"}\n" << '\0';
		//printf("%s\n", source);

		std::vector<std::string> inpTexNames(1), inpParamNames;
		inpTexNames[0] = "g_ftrTex";
		LoadSource(source, inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &ftrTex, const TextureGL4 &levelTex, const ushort &nFtrs) const
	{
#if _DEBUG
		assert(ftrTex.GetWidth() == (m_levelTexWidth << 2) && levelTex.GetWidth() == m_levelTexWidth);
#endif

		Activate();
		SetInputTexture(ftrTex);
		SetOutputTexture(levelTex);

		glBegin(GL_QUADS);
		const ushort nDrawPixels = ((nFtrs + 3) >> 2);
		const uint drawHeight1 = (nDrawPixels >> m_levelTexWidthLog);
		const uint nDrawPixels1 = (drawHeight1 << m_levelTexWidthLog);
		if(nDrawPixels1 != 0)
		{
			const GLfloat s1 = -1.5f, s2 = (m_levelTexWidth << 2) - 1.5f, t1 = 0, t2 = float(drawHeight1);
			const GLint x1 = 0, x2 = GLint(m_levelTexWidth), y1 = 0, y2 = GLint(drawHeight1);
			glMultiTexCoord2f(GL_TEXTURE0, s1,   t1);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t1);
			glMultiTexCoord2f(GL_TEXTURE2, s1+2, t1);
			glMultiTexCoord2f(GL_TEXTURE3, s1+3, t1);
			glVertex2i(x1, y1);
			glMultiTexCoord2f(GL_TEXTURE0, s1,   t2);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t2);
			glMultiTexCoord2f(GL_TEXTURE2, s1+2, t2);
			glMultiTexCoord2f(GL_TEXTURE3, s1+3, t2);
			glVertex2i(x1, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s2,   t2);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t2);
			glMultiTexCoord2f(GL_TEXTURE2, s2+2, t2);
			glMultiTexCoord2f(GL_TEXTURE3, s2+3, t2);
			glVertex2i(x2, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s2,   t1);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t1);
			glMultiTexCoord2f(GL_TEXTURE2, s2+2, t1);
			glMultiTexCoord2f(GL_TEXTURE3, s2+3, t1);
			glVertex2i(x2, y1);
		}
		const uint nDrawPixels2 = nDrawPixels - nDrawPixels1;
		if(nDrawPixels2 != 0)
		{
			const GLfloat s1 = -1.5f, s2 = (nDrawPixels2 << 2) - 1.5f, t1 = GLfloat(drawHeight1), t2 = t1 + 1;
			const GLint x1 = 0, x2 = GLint(nDrawPixels2), y1 = GLint(drawHeight1), y2 = y1 + 1;
			glBegin(GL_QUADS);
			glMultiTexCoord2f(GL_TEXTURE0, s1,   t1);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t1);
			glMultiTexCoord2f(GL_TEXTURE2, s1+2, t1);
			glMultiTexCoord2f(GL_TEXTURE3, s1+3, t1);
			glVertex2i(x1, y1);
			glMultiTexCoord2f(GL_TEXTURE0, s1,   t2);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t2);
			glMultiTexCoord2f(GL_TEXTURE2, s1+2, t2);
			glMultiTexCoord2f(GL_TEXTURE3, s1+3, t2);
			glVertex2i(x1, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s2,   t2);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t2);
			glMultiTexCoord2f(GL_TEXTURE2, s2+2, t2);
			glMultiTexCoord2f(GL_TEXTURE3, s2+3, t2);
			glVertex2i(x2, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s2,   t1);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t1);
			glMultiTexCoord2f(GL_TEXTURE2, s2+2, t1);
			glMultiTexCoord2f(GL_TEXTURE3, s2+3, t1);
			glVertex2i(x2, y1);
		}
		glEnd();
		glFlush();
		Deactivate();
	}

private:

	ushort m_levelTexWidth, m_levelTexWidthLog;

};

#endif