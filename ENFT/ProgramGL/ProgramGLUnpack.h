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

#ifndef _PROGRAM_GL_UNPACK_H_
#define _PROGRAM_GL_UNPACK_H_

#include "ProgramGL.h"

class ProgramGLUnpack : public ProgramGL
{

public:

	inline void Initialize()
	{
		//char source[MAX_SOURCE_LENGTH];
		//std::ostrstream out(source, MAX_SOURCE_LENGTH);
		//out << "uniform sampler2DRect g_srcTex;\n";
		//out << "void main()\n";
		//out << "{\n";
		//out <<		"uvec2 dstCoord = uvec2(gl_FragCoord.xy);\n";
		//out <<		"uvec2 bit = dstCoord & uvec2(1, 1);\n";
		//out <<		"gl_FragColor.r = texture2DRect(src, vec2(dstCoord >> 1) + vec2(0.5, 0.5))[(bit.y << 1) + bit.x];\n";
		//out << "}\n";

		std::vector<std::string> inpTexNames(1), inpParamNames;
		inpTexNames[0] = "g_srcTex";
		LoadSource(
			"uniform sampler2DRect g_srcTex;\n"
			"void main()\n"
			"{\n"
				"ivec2 dstCoord = ivec2(gl_FragCoord.xy);\n"
				"ivec2 bit = (dstCoord & 1);\n"
				"gl_FragColor = vec4(texture2DRect(g_srcTex, vec2(dstCoord >> 1) + vec2(0.5, 0.5))[(bit.y << 1) + bit.x]);\n"
			"}\n", inpTexNames, inpParamNames);
	};
	template<ushort CHANNELS_NUMBER_DST>
	inline void Run(const TextureGL4 &srcTex, const TextureGL<CHANNELS_NUMBER_DST> &dstTex) const
	{
#if _DEBUG
		assert(srcTex.GetWidth() == ((dstTex.GetWidth() + 1) >> 1) && srcTex.GetHeight() == ((dstTex.GetHeight() + 1) >> 1));
#endif

		Activate();
		SetInputTexture(srcTex);
		SetOutputTexture(dstTex);

		/*
		GLint s1 = 0, t1 = 0, s2 = GLint(srcTex.GetWidth()), t2 = GLint(srcTex.GetHeight());
		GLint x1 = 0, y1 = 0, x2 = GLint(dstTex.GetWidth()), y2 = GLint(dstTex.GetHeight());
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
		*/
		DrawQuad(dstTex.GetWidth(), dstTex.GetHeight());

		Deactivate();
	}
	inline void Draw(const TextureGL4 &srcTex)
	{
		glClear(GL_COLOR_BUFFER_BIT);

		Activate();
		SetInputTexture(srcTex);
		
 		GLint s1 = 0, t1 = 0, s2 = GLint(srcTex.GetWidth()), t2 = GLint(srcTex.GetHeight());
		GLint x1 = 0, y1 = 0, x2 = GLint(srcTex.GetWidth() << 1), y2 = GLint(srcTex.GetHeight() << 1);
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
		//glMultiTexCoord2i(GL_TEXTURE0, s1,   t2  );
		//glMultiTexCoord2i(GL_TEXTURE1, s1+1, t2  );
		//glMultiTexCoord2i(GL_TEXTURE2, s1,   t2+1);
		//glMultiTexCoord2i(GL_TEXTURE3, s1+1, t2+1);
		//glVertex2i(x1, y1);
		//glMultiTexCoord2i(GL_TEXTURE0, s1,   t1  );
		//glMultiTexCoord2i(GL_TEXTURE1, s1+1, t1  );
		//glMultiTexCoord2i(GL_TEXTURE2, s1,   t1+1);
		//glMultiTexCoord2i(GL_TEXTURE3, s1+1, t1+1);
		//glVertex2i(x1, y2);
		//glMultiTexCoord2i(GL_TEXTURE0, s2,   t1  );
		//glMultiTexCoord2i(GL_TEXTURE1, s2+1, t1  );
		//glMultiTexCoord2i(GL_TEXTURE2, s2,   t1+1);
		//glMultiTexCoord2i(GL_TEXTURE3, s2+1, t1+1);
		//glVertex2i(x2, y2);
		//glMultiTexCoord2i(GL_TEXTURE0, s2,   t2  );
		//glMultiTexCoord2i(GL_TEXTURE1, s2+1, t2  );
		//glMultiTexCoord2i(GL_TEXTURE2, s2,   t2+1);
		//glMultiTexCoord2i(GL_TEXTURE3, s2+1, t2+1);
		//glVertex2i(x2, y1);
		glEnd();
		glFlush();

		Deactivate();
	}

};

#endif