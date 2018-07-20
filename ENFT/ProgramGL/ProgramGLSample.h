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

#ifndef _PROGRAM_GL_DOWN_SAMPLE_H_
#define _PROGRAM_GL_DOWN_SAMPLE_H_

#include "ProgramGL.h"

class ProgramGLDownSample : public ProgramGL
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
			"	gl_FragColor = vec4(texture2DRect(g_srcTex,gl_TexCoord[0].st).r, "
			"						texture2DRect(g_srcTex,gl_TexCoord[1].st).r, "
			"						texture2DRect(g_srcTex,gl_TexCoord[2].st).r, "
			"						texture2DRect(g_srcTex,gl_TexCoord[3].st).r);\n"
			"	gl_FragDepth = 1.0f;\n"
			"}\n", inpTexNames, inpParamNames);
	};
	template<ushort CHANNELS_NUMBER_SRC>
	inline void Run(const TextureGL<CHANNELS_NUMBER_SRC> &srcTex, const TextureGL4 &dstTex) const
	{
#if _DEBUG
		assert(dstTex.GetWidth() == ((srcTex.GetWidth() + 1) >> 1) && dstTex.GetHeight() == ((srcTex.GetHeight() + 1) >> 1));
#endif

		Activate();
		SetInputTexture(srcTex);
		SetOutputTexture(dstTex);

#if 1
		const GLfloat s1 = -0.5f, t1 = -0.5f, s2 = (dstTex.GetWidth() << 1) - 0.5f, t2 = (dstTex.GetHeight() << 1) - 0.5f;
		const GLint x1 = 0, y1 = 0, x2 = GLint(dstTex.GetWidth()), y2 = GLint(dstTex.GetHeight());
		glBegin(GL_QUADS);
		glMultiTexCoord2f(GL_TEXTURE0, s1,   t1  );
		glMultiTexCoord2f(GL_TEXTURE1, s1+1, t1  );
		glMultiTexCoord2f(GL_TEXTURE2, s1,   t1+1);
		glMultiTexCoord2f(GL_TEXTURE3, s1+1, t1+1);
		glVertex2i(x1, y1);
		glMultiTexCoord2f(GL_TEXTURE0, s1,   t2  );
		glMultiTexCoord2f(GL_TEXTURE1, s1+1, t2  );
		glMultiTexCoord2f(GL_TEXTURE2, s1,   t2+1);
		glMultiTexCoord2f(GL_TEXTURE3, s1+1, t2+1);
		glVertex2i(x1, y2);
		glMultiTexCoord2f(GL_TEXTURE0, s2,   t2  );
		glMultiTexCoord2f(GL_TEXTURE1, s2+1, t2  );
		glMultiTexCoord2f(GL_TEXTURE2, s2,   t2+1);
		glMultiTexCoord2f(GL_TEXTURE3, s2+1, t2+1);
		glVertex2i(x2, y2);
		glMultiTexCoord2f(GL_TEXTURE0, s2,   t1  );
		glMultiTexCoord2f(GL_TEXTURE1, s2+1, t1  );
		glMultiTexCoord2f(GL_TEXTURE2, s2,   t1+1);
		glMultiTexCoord2f(GL_TEXTURE3, s2+1, t1+1);
		glVertex2i(x2, y1);
		glEnd();
		glFlush();
#else
		const GLint s1 = 0, t1 = 0, s2 = GLint(dstTex.GetWidth() << 1), t2 = GLint(dstTex.GetHeight() << 1);
		const GLint x1 = 0, y1 = 0, x2 = GLint(dstTex.GetWidth()), y2 = GLint(dstTex.GetHeight());
		glBegin(GL_QUADS);
		glMultiTexCoord2i(GL_TEXTURE0, s1,   t1  );
		glMultiTexCoord2i(GL_TEXTURE1, s1+1, t1  );
		glMultiTexCoord2i(GL_TEXTURE2, s1,   t1+1);
		glMultiTexCoord2i(GL_TEXTURE3, s1+1, t1+1);
		glVertex2i(x1, y1);
		glMultiTexCoord2i(GL_TEXTURE0, s1,   t2  );
		glMultiTexCoord2i(GL_TEXTURE1, s1+1, t2  );
		glMultiTexCoord2i(GL_TEXTURE2, s1,   t2+1);
		glMultiTexCoord2i(GL_TEXTURE3, s1+1, t2+1);
		glVertex2i(x1, y2);
		glMultiTexCoord2i(GL_TEXTURE0, s2,   t2  );
		glMultiTexCoord2i(GL_TEXTURE1, s2+1, t2  );
		glMultiTexCoord2i(GL_TEXTURE2, s2,   t2+1);
		glMultiTexCoord2i(GL_TEXTURE3, s2+1, t2+1);
		glVertex2i(x2, y2);
		glMultiTexCoord2i(GL_TEXTURE0, s2,   t1  );
		glMultiTexCoord2i(GL_TEXTURE1, s2+1, t1  );
		glMultiTexCoord2i(GL_TEXTURE2, s2,   t1+1);
		glMultiTexCoord2i(GL_TEXTURE3, s2+1, t1+1);
		glVertex2i(x2, y1);
		glEnd();
		glFlush();
#endif

		Deactivate();
	}
};

class ProgramGLUpSample : public ProgramGL
{

public:

	inline void Initialize()
	{
		//std::vector<std::string> inpTexNames(1), inpParamNames;
		//inpTexNames[0] = "g_srcTex";
		//LoadSource(
		//	"uniform sampler2DRect g_srcTex;\n"
		//	"void main()\n"
		//	"{\n"
		//	"	gl_FragColor = vec4(texture2DRect(g_srcTex,gl_TexCoord[0].st).r, "
		//							"texture2DRect(g_srcTex,gl_TexCoord[1].st).r, "
		//							"texture2DRect(g_srcTex,gl_TexCoord[2].st).r, "
		//							"texture2DRect(g_srcTex,gl_TexCoord[3].st).r);\n"
		//	"}\n", inpTexNames, inpParamNames);
		std::vector<std::string> inpTexNames(1), inpParamNames;
		inpTexNames[0] = "g_srcTex";
		LoadSource(
			"uniform sampler2DRect g_srcTex;\n"
			"void main()\n"
			"{\n"
			"	vec4 src = vec4(texture2DRect(g_srcTex,gl_TexCoord[0].st).r, "
			"					texture2DRect(g_srcTex,gl_TexCoord[1].st).r, "
			"					texture2DRect(g_srcTex,gl_TexCoord[2].st).r, "
			"					texture2DRect(g_srcTex,gl_TexCoord[3].st).r);\n"
			"	gl_FragColor = vec4(src.r, (src.r + src.g) * 0.5, (src.r + src.b) * 0.5, (src.r + src.g + src.b + src.a) * 0.25);\n"
			"	gl_FragDepth = 1.0f;\n"
			"}\n", inpTexNames, inpParamNames);
	};
	inline void Run(const TextureGL1 &srcTex, const TextureGL4 &dstTex) const
	{
#if _DEBUG
		assert(dstTex.GetWidth() == srcTex.GetWidth() && dstTex.GetHeight() == srcTex.GetHeight());
#endif

		Activate();
		SetInputTexture(srcTex);
		SetOutputTexture(dstTex);

		const GLint s1 = 0, t1 = 0, s2 = GLint(srcTex.GetWidth()), t2 = GLint(srcTex.GetHeight());
		const GLint x1 = 0, y1 = 0, x2 = GLint(dstTex.GetWidth()), y2 = GLint(dstTex.GetHeight());
		glBegin(GL_QUADS);
		glMultiTexCoord2i(GL_TEXTURE0, s1,   t1  );
		glMultiTexCoord2i(GL_TEXTURE1, s1+1, t1  );
		glMultiTexCoord2i(GL_TEXTURE2, s1,   t1+1);
		glMultiTexCoord2i(GL_TEXTURE3, s1+1, t1+1);
		glVertex2i(x1, y1);
		glMultiTexCoord2i(GL_TEXTURE0, s1,   t2  );
		glMultiTexCoord2i(GL_TEXTURE1, s1+1, t2  );
		glMultiTexCoord2i(GL_TEXTURE2, s1,   t2+1);
		glMultiTexCoord2i(GL_TEXTURE3, s1+1, t2+1);
		glVertex2i(x1, y2);
		glMultiTexCoord2i(GL_TEXTURE0, s2,   t2  );
		glMultiTexCoord2i(GL_TEXTURE1, s2+1, t2  );
		glMultiTexCoord2i(GL_TEXTURE2, s2,   t2+1);
		glMultiTexCoord2i(GL_TEXTURE3, s2+1, t2+1);
		glVertex2i(x2, y2);
		glMultiTexCoord2i(GL_TEXTURE0, s2,   t1  );
		glMultiTexCoord2i(GL_TEXTURE1, s2+1, t1  );
		glMultiTexCoord2i(GL_TEXTURE2, s2,   t1+1);
		glMultiTexCoord2i(GL_TEXTURE3, s2+1, t1+1);
		glVertex2i(x2, y1);
		glEnd();
		glFlush();

		Deactivate();
	}
};

#endif