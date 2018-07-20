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

#ifndef _PROGRAM_GL_FEATURE_HISTOGRAM_H_
#define _PROGRAM_GL_FEATURE_HISTOGRAM_H_

class ProgramGLFeatureHistogramInitialize
{

public:

	inline void Initialize()
	{
		for(ushort i = 0; i < 4; ++i)
		{
			char source[MAX_SOURCE_LENGTH];
			std::ostrstream out(source, MAX_SOURCE_LENGTH);
			out <<
				"uniform sampler2DRect g_extremeTex;\n"
				"void main()\n"
				"{\n";
			if(i == 0)
			{
				out << 
				"	vec4 src = vec4(texture2DRect(g_extremeTex, gl_TexCoord[0].st).r,\n"
				"					texture2DRect(g_extremeTex, gl_TexCoord[1].st).r,\n"
				"					texture2DRect(g_extremeTex, gl_TexCoord[2].st).r,\n"
				"					texture2DRect(g_extremeTex, gl_TexCoord[3].st).r);\n";
			}
			else if(i == 1) // for the case that the last input column is odd
			{
				out <<
					"	vec4 src = vec4(texture2DRect(g_extremeTex, gl_TexCoord[0].st).r, 0,\n"
					"					texture2DRect(g_extremeTex, gl_TexCoord[1].st).r, 0);\n";
			}
			else if(i == 2)	// for the case that the last input row is odd
			{
				out <<
					"	vec4 src = vec4(texture2DRect(g_extremeTex, gl_TexCoord[0].st).r,\n"
					"					texture2DRect(g_extremeTex, gl_TexCoord[1].st).r, 0, 0);\n";
			}
			else if(i == 3) // for the case that both the last input column and row are odd
			{
				out <<
					"	vec4 src = vec4(texture2DRect(g_extremeTex, gl_TexCoord[0].st).r, 0, 0, 0);\n";
			}
			out <<
				"	gl_FragColor = vec4(notEqual(src, vec4(0)));\n"
				"}\n" << '\0';

			std::vector<std::string> inpTexNames(1), inpParamNames;
			inpTexNames[0] = "g_extremeTex";
			m_programs[i].LoadSource(source, inpTexNames, inpParamNames);
		}
	};

	inline void Run(const TextureGL4 &extremeTex, const TextureGL4 &histTex) const
	{
		m_programs[0].SetInputTexture(extremeTex);
		m_programs[0].SetOutputTexture(histTex);

		const ushort inpWidth = extremeTex.GetWidth(), inpHeight = extremeTex.GetHeight();
		const ushort drawWidth1 = (inpWidth >> 1), drawHeight1 = (inpHeight >> 1);
		const ushort inpWidth1 = (drawWidth1 << 1), inpHeight1 = (drawHeight1 << 1);
		const GLfloat s1 = -0.5f, s2 = inpWidth1 - 0.5f, t1 = -0.5f, t2 = inpHeight1 - 0.5f;
		const GLint x1 = 0, x2 = GLint(drawWidth1), y1 = 0, y2 = GLint(drawHeight1);
		if(drawWidth1 && drawHeight1)
		{
			m_programs[0].Activate();
			glBegin(GL_QUADS);
			glMultiTexCoord2f(GL_TEXTURE0, s1,	 t1);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t1);
			glMultiTexCoord2f(GL_TEXTURE2, s1,	 t1+1);
			glMultiTexCoord2f(GL_TEXTURE3, s1+1, t1+1);
			glVertex2i(x1, y1);
			glMultiTexCoord2f(GL_TEXTURE0, s1,	 t2);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t2);
			glMultiTexCoord2f(GL_TEXTURE2, s1,	 t2+1);
			glMultiTexCoord2f(GL_TEXTURE3, s1+1, t2+1);
			glVertex2i(x1, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s2,	 t2);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t2);
			glMultiTexCoord2f(GL_TEXTURE2, s2,	 t2+1);
			glMultiTexCoord2f(GL_TEXTURE3, s2+1, t2+1);
			glVertex2i(x2, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s2,	 t1);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t1);
			glMultiTexCoord2f(GL_TEXTURE2, s2,	 t1+1);
			glMultiTexCoord2f(GL_TEXTURE3, s2+1, t1+1);
			glVertex2i(x2, y1);
			glEnd();
			glFlush();
			m_programs[0].Deactivate();
		}
		if(inpWidth1 != inpWidth)
		{
			const GLfloat s3 = s2 + 2;
			const GLint x3 = x2 + 1;
			m_programs[1].Activate();
			glBegin(GL_QUADS);
			glMultiTexCoord2f(GL_TEXTURE0, s2, t1);
			glMultiTexCoord2f(GL_TEXTURE1, s2, t1+1);
			glVertex2i(x2, y1);
			glMultiTexCoord2f(GL_TEXTURE0, s2, t2);
			glMultiTexCoord2f(GL_TEXTURE1, s2, t2+1);
			glVertex2i(x2, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s3, t2);
			glMultiTexCoord2f(GL_TEXTURE1, s3, t2+1);
			glVertex2i(x3, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s3, t1);
			glMultiTexCoord2f(GL_TEXTURE1, s3, t1+1);
			glVertex2i(x3, y1);
			glEnd();
			glFlush();
			m_programs[1].Deactivate();
		}
		if(inpHeight1 != inpHeight)
		{
			const GLfloat t3 = t2 + 2;
			const GLint y3 = y2 + 1;
			m_programs[2].Activate();
			glBegin(GL_QUADS);
			glMultiTexCoord2f(GL_TEXTURE0, s1,	 t2);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t2);
			glVertex2i(x1, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s1,	 t3);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t3);
			glVertex2i(x1, y3);
			glMultiTexCoord2f(GL_TEXTURE0, s2,	 t3);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t3);
			glVertex2i(x2, y3);
			glMultiTexCoord2f(GL_TEXTURE0, s2,	 t2);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t2);
			glVertex2i(x2, y2);
			glEnd();
			glFlush();
			m_programs[2].Deactivate();
		}
		if(inpWidth1 != inpWidth && inpHeight1 != inpHeight)
		{
			const GLfloat s3 = s2 + 2, t3 = t2 + 2;
			const GLint x3 = x2 + 1, y3 = y2 + 1;
			m_programs[3].Activate();
			glBegin(GL_QUADS);
			glTexCoord2f(s2, t2);		glVertex2i(x2, y2);
			glTexCoord2f(s2, t3);		glVertex2i(x2, y3);
			glTexCoord2f(s3, t3);		glVertex2i(x3, y3);
			glTexCoord2f(s3, t2);		glVertex2i(x3, y2);
			glEnd();
			glFlush();
			m_programs[3].Deactivate();
		}
	}

private:

	ProgramGL m_programs[4];

};

class ProgramGLFeatureHistogramReduce
{

public:

	inline void Initialize()
	{
		for(ushort i = 0; i < 4; ++i)
		{
			char source[MAX_SOURCE_LENGTH];
			std::ostrstream out(source, MAX_SOURCE_LENGTH);
			out <<
				"uniform sampler2DRect g_srcHistTex;\n"
				"void main(void)\n"
				"{\n"
				"	vec4 src, sum;\n";
			if(i == 0)
			{
				out <<
				"	src = texture2DRect(g_srcHistTex, gl_TexCoord[0].st);		sum.rg = src.rg + src.ba;\n"
				"	src = texture2DRect(g_srcHistTex, gl_TexCoord[1].st);		sum.ba = src.rg + src.ba;\n"
				"	gl_FragColor.rg = sum.rb + sum.ga;\n"
				"	src = texture2DRect(g_srcHistTex, gl_TexCoord[2].st);		sum.rg = src.rg + src.ba;\n"
				"	src = texture2DRect(g_srcHistTex, gl_TexCoord[3].st);		sum.ba = src.rg + src.ba;\n"
				"	gl_FragColor.ba = sum.rb + sum.ga;\n";
			}
			else if(i == 1) // for the case that the last input column is odd
			{
				out <<
				"	src = texture2DRect(g_srcHistTex, gl_TexCoord[0].st);		sum.rg = src.rg + src.ba;\n"
				"	src = texture2DRect(g_srcHistTex, gl_TexCoord[1].st);		sum.ba = src.rg + src.ba;\n"
				"	gl_FragColor = vec4(sum.r + sum.g, 0, sum.b + sum.a, 0);\n";
			}
			else if(i == 2) // for the case that the last input row is odd
			{
				out <<
				"	src = texture2DRect(g_srcHistTex, gl_TexCoord[0].st);		sum.rg = src.rg + src.ba;\n"
				"	src = texture2DRect(g_srcHistTex, gl_TexCoord[1].st);		sum.ba = src.rg + src.ba;\n"
				"	gl_FragColor = vec4(sum.rb + sum.ga, 0, 0);\n";
			}
			else if(i == 3) // for the case that both the last input column and row are odd
			{
				out <<
				"	src = texture2DRect(g_srcHistTex, gl_TexCoord[0].st);		sum.rg = src.rg + src.ba;\n"
				"	gl_FragColor = vec4(sum.r + sum.g, 0, 0, 0);\n";
			}
			out <<
				"}\n" << '\0';

			std::vector<std::string> inpTexNames(1), inpParamNames;
			inpTexNames[0] = "g_srcHistTex";
			m_programs[i].LoadSource(source, inpTexNames, inpParamNames);
		}
	};

	inline void Run(const TextureGL4 &srcHistTex, const TextureGL4 &dstHistTex) const
	{
		m_programs[0].SetInputTexture(srcHistTex);
		m_programs[0].SetOutputTexture(dstHistTex);

		const ushort inpWidth = srcHistTex.GetWidth(), inpHeight = srcHistTex.GetHeight();
		const ushort drawWidth1 = (inpWidth >> 1), drawHeight1 = (inpHeight >> 1);
		const ushort inpWidth1 = (drawWidth1 << 1), inpHeight1 = (drawHeight1 << 1);
		const GLfloat s1 = -0.5f, s2 = inpWidth1 - 0.5f, t1 = -0.5f, t2 = t1 + inpHeight1;
		const GLint x1 = 0, x2 = GLint(drawWidth1), y1 = 0, y2 = GLint(drawHeight1);
		if(drawWidth1 && drawHeight1)
		{
			m_programs[0].Activate();
			glBegin(GL_QUADS);
			glMultiTexCoord2f(GL_TEXTURE0, s1,	 t1);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t1);
			glMultiTexCoord2f(GL_TEXTURE2, s1,	 t1+1);
			glMultiTexCoord2f(GL_TEXTURE3, s1+1, t1+1);
			glVertex2i(x1, y1);
			glMultiTexCoord2f(GL_TEXTURE0, s1,	 t2);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t2);
			glMultiTexCoord2f(GL_TEXTURE2, s1,	 t2+1);
			glMultiTexCoord2f(GL_TEXTURE3, s1+1, t2+1);
			glVertex2i(x1, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s2,	 t2);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t2);
			glMultiTexCoord2f(GL_TEXTURE2, s2,	 t2+1);
			glMultiTexCoord2f(GL_TEXTURE3, s2+1, t2+1);
			glVertex2i(x2, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s2,	 t1);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t1);
			glMultiTexCoord2f(GL_TEXTURE2, s2,	 t1+1);
			glMultiTexCoord2f(GL_TEXTURE3, s2+1, t1+1);
			glVertex2i(x2, y1);
			glEnd();
			glFlush();
			m_programs[0].Deactivate();
		}
		if(inpWidth1 != inpWidth)
		{
			const GLfloat s3 = s2 + 2;
			const GLint x3 = x2 + 1;
			m_programs[1].Activate();
			glBegin(GL_QUADS);
			glMultiTexCoord2f(GL_TEXTURE0, s2, t1);
			glMultiTexCoord2f(GL_TEXTURE1, s2, t1+1);
			glVertex2i(x2, y1);
			glMultiTexCoord2f(GL_TEXTURE0, s2, t2);
			glMultiTexCoord2f(GL_TEXTURE1, s2, t2+1);
			glVertex2i(x2, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s3, t2);
			glMultiTexCoord2f(GL_TEXTURE1, s3, t2+1);
			glVertex2i(x3, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s3, t1);
			glMultiTexCoord2f(GL_TEXTURE1, s3, t1+1);
			glVertex2i(x3, y1);
			glEnd();
			glFlush();
			m_programs[1].Deactivate();
		}
		if(inpHeight1 != inpHeight)
		{
			const GLfloat t3 = t2 + 2;
			const GLint y3 = y2 + 1;
			m_programs[2].Activate();
			glBegin(GL_QUADS);
			glMultiTexCoord2f(GL_TEXTURE0, s1,	 t2);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t2);
			glVertex2i(x1, y2);
			glMultiTexCoord2f(GL_TEXTURE0, s1,	 t3);
			glMultiTexCoord2f(GL_TEXTURE1, s1+1, t3);
			glVertex2i(x1, y3);
			glMultiTexCoord2f(GL_TEXTURE0, s2,	 t3);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t3);
			glVertex2i(x2, y3);
			glMultiTexCoord2f(GL_TEXTURE0, s2,	 t2);
			glMultiTexCoord2f(GL_TEXTURE1, s2+1, t2);
			glVertex2i(x2, y2);
			glEnd();
			glFlush();
			m_programs[2].Deactivate();
		}
		if(inpWidth1 != inpWidth && inpHeight1 != inpHeight)
		{
			const GLfloat s3 = s2 + 2, t3 = t2 + 2;
			const GLint x3 = x2 + 1, y3 = y2 + 1;
			m_programs[3].Activate();
			glBegin(GL_QUADS);
			glTexCoord2f(s2, t2);		glVertex2i(x2, y2);
			glTexCoord2f(s2, t3);		glVertex2i(x2, y3);
			glTexCoord2f(s3, t3);		glVertex2i(x3, y3);
			glTexCoord2f(s3, t2);		glVertex2i(x3, y2);
			glEnd();
			glFlush();
			m_programs[3].Deactivate();
		}
	}

private:

	ProgramGL m_programs[4];
};

#endif