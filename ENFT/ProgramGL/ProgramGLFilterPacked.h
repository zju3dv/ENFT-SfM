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

#ifndef _PROGRAM_GL_FILTER_PACKED_H_
#define _PROGRAM_GL_FILTER_PACKED_H_

#include "ProgramGL.h"

class ProgramGLFilterPackedPass : public ProgramGL
{

public:

	inline void Initialize(const std::vector<float> &kernel)
	{
		char source[MAX_SOURCE_LENGTH];
		std::vector<std::string> inpTexNames, inpParamNames;
		SetSource(kernel, source, inpTexNames, inpParamNames);
		LoadSource(source, inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &srcTex, const TextureGL4 &dstTex) const
	{
		Activate();
		SetInputTexture(srcTex);
		SetOutputTexture(dstTex);
		DrawQuad(srcTex, dstTex);
		Deactivate();
	}

protected:

	virtual void SetSource(const std::vector<float> &kernel, char *source, std::vector<std::string> &inpTexNames, std::vector<std::string> &inpParamNames) = 0;
	virtual void DrawQuad(const TextureGL4 &srcTex, const TextureGL4 &dstTex) const = 0;

protected:

	ushort m_kernalWidth;

};

class ProgramGLFilterPackedPass1 : public ProgramGLFilterPackedPass
{

public:

	virtual void SetSource(const std::vector<float> &kernel, char *source, std::vector<std::string> &inpTexNames, std::vector<std::string> &inpParamNames)
	{
#if _DEBUG
		assert(kernel.size() >= 7 && kernel.size() <= 25 && (kernel.size() & 1) == 1);
#endif
		m_kernalWidth = ushort(kernel.size());

		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out << "uniform sampler2DRect g_srcTex;\n";
		out << "void main()\n";
		out << "{\n";
		switch(m_kernalWidth)
		{
		case 7:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.ga, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src1.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src2.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src3.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.gaga";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[6] << ") * src4.rb);\n";
			break;
		case 9:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.rb, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src0.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src1.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src2.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src3.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.rbrb";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[8] << ") * src4.ga);\n";
			break;
		case 11:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.ga, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src1.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src2.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src3.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src4.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src5.rbrb";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.gaga";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[10] << ") * src6.rb);\n";
			break;
		case 13:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.rb, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src0.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src1.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src2.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src3.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src4.gaga";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.rbrb";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src5.gaga";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.rbrb";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[12] << ") * src6.ga);\n";
			break;
		case 15:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(1, 0));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.ga, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src1.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src2.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src3.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src4.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src5.rbrb";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.gaga";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src6.rbrb";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.gaga";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src7.rbrb";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.gaga";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[14] << ") * src8.rb);\n";
			break;
		case 17:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(1, 0));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.rb, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src0.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src1.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src2.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src3.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src4.gaga";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.rbrb";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src5.gaga";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.rbrb";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src6.gaga";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.rbrb";
			out <<			   " + vec4(vec2(" << kernel[15] << "), vec2(" << kernel[14] << ")) * src7.gaga";
			out <<			   " + vec4(vec2(" << kernel[16] << "), vec2(" << kernel[15] << ")) * src8.rbrb";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[16] << ") * src8.ga);\n";
			break;
		case 19:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(1, 0));\n";
			out << "vec4 src9 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(2, 0));\n";
			out << "vec4 src10 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(3, 0));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.ga, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src1.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src2.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src3.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src4.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src5.rbrb";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.gaga";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src6.rbrb";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.gaga";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src7.rbrb";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.gaga";
			out <<			   " + vec4(vec2(" << kernel[15] << "), vec2(" << kernel[14] << ")) * src8.rbrb";
			out <<			   " + vec4(vec2(" << kernel[16] << "), vec2(" << kernel[15] << ")) * src8.gaga";
			out <<			   " + vec4(vec2(" << kernel[17] << "), vec2(" << kernel[16] << ")) * src9.rbrb";
			out <<			   " + vec4(vec2(" << kernel[18] << "), vec2(" << kernel[17] << ")) * src9.gaga";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[18] << ") * src10.rb);\n";
			break;
		case 21:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(1, 0));\n";
			out << "vec4 src9 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(2, 0));\n";
			out << "vec4 src10 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(3, 0));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.rb, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src0.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src1.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src2.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src3.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src4.gaga";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.rbrb";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src5.gaga";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.rbrb";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src6.gaga";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.rbrb";
			out <<			   " + vec4(vec2(" << kernel[15] << "), vec2(" << kernel[14] << ")) * src7.gaga";
			out <<			   " + vec4(vec2(" << kernel[16] << "), vec2(" << kernel[15] << ")) * src8.rbrb";
			out <<			   " + vec4(vec2(" << kernel[17] << "), vec2(" << kernel[16] << ")) * src8.gaga";
			out <<			   " + vec4(vec2(" << kernel[18] << "), vec2(" << kernel[17] << ")) * src9.rbrb";
			out <<			   " + vec4(vec2(" << kernel[19] << "), vec2(" << kernel[18] << ")) * src9.gaga";
			out <<			   " + vec4(vec2(" << kernel[20] << "), vec2(" << kernel[19] << ")) * src10.rbrb";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[20] << ") * src10.ga);\n";
			break;
		case 23:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(1, 0));\n";
			out << "vec4 src9 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(2, 0));\n";
			out << "vec4 src10 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(3, 0));\n";
			out << "vec4 src11 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(4, 0));\n";
			out << "vec4 src12 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(5, 0));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.ga, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src1.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src2.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src3.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src4.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src5.rbrb";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.gaga";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src6.rbrb";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.gaga";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src7.rbrb";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.gaga";
			out <<			   " + vec4(vec2(" << kernel[15] << "), vec2(" << kernel[14] << ")) * src8.rbrb";
			out <<			   " + vec4(vec2(" << kernel[16] << "), vec2(" << kernel[15] << ")) * src8.gaga";
			out <<			   " + vec4(vec2(" << kernel[17] << "), vec2(" << kernel[16] << ")) * src9.rbrb";
			out <<			   " + vec4(vec2(" << kernel[18] << "), vec2(" << kernel[17] << ")) * src9.gaga";
			out <<			   " + vec4(vec2(" << kernel[19] << "), vec2(" << kernel[18] << ")) * src10.rbrb";
			out <<			   " + vec4(vec2(" << kernel[20] << "), vec2(" << kernel[19] << ")) * src10.gaga";
			out <<			   " + vec4(vec2(" << kernel[21] << "), vec2(" << kernel[20] << ")) * src11.rbrb";
			out <<			   " + vec4(vec2(" << kernel[22] << "), vec2(" << kernel[21] << ")) * src11.gaga";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[22] << ") * src12.rb);\n";
			break;
		case 25:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(1, 0));\n";
			out << "vec4 src9 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(2, 0));\n";
			out << "vec4 src10 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(3, 0));\n";
			out << "vec4 src11 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(4, 0));\n";
			out << "vec4 src12 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(5, 0));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.rb, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src0.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src1.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src2.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src3.gaga";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.rbrb";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src4.gaga";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.rbrb";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src5.gaga";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.rbrb";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src6.gaga";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.rbrb";
			out <<			   " + vec4(vec2(" << kernel[15] << "), vec2(" << kernel[14] << ")) * src7.gaga";
			out <<			   " + vec4(vec2(" << kernel[16] << "), vec2(" << kernel[15] << ")) * src8.rbrb";
			out <<			   " + vec4(vec2(" << kernel[17] << "), vec2(" << kernel[16] << ")) * src8.gaga";
			out <<			   " + vec4(vec2(" << kernel[18] << "), vec2(" << kernel[17] << ")) * src9.rbrb";
			out <<			   " + vec4(vec2(" << kernel[19] << "), vec2(" << kernel[18] << ")) * src9.gaga";
			out <<			   " + vec4(vec2(" << kernel[20] << "), vec2(" << kernel[19] << ")) * src10.rbrb";
			out <<			   " + vec4(vec2(" << kernel[21] << "), vec2(" << kernel[20] << ")) * src10.gaga";
			out <<			   " + vec4(vec2(" << kernel[22] << "), vec2(" << kernel[21] << ")) * src11.rbrb";
			out <<			   " + vec4(vec2(" << kernel[23] << "), vec2(" << kernel[22] << ")) * src11.gaga";
			out <<			   " + vec4(vec2(" << kernel[24] << "), vec2(" << kernel[23] << ")) * src12.rbrb";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[24] << ") * src12.ga);\n";
			break;
		}
		out << "gl_FragColor = gl_FragColor.rbga;\n";
		out << "gl_FragDepth = 1.0f;\n";
		out << "}\n";
		out << '\0';
		//printf("%s\n", source);

		inpTexNames.resize(1);
		inpTexNames[0] = "g_srcTex";
		inpParamNames.resize(0);
	}
	virtual void DrawQuad(const TextureGL4 &srcTex, const TextureGL4 &dstTex) const
	{
#if _DEBUG
		assert(srcTex.GetWidth() == dstTex.GetWidth() && srcTex.GetHeight() == dstTex.GetHeight());
#endif
		GLint s1 = 0, t1 = 0, s2 = GLint(srcTex.GetWidth()), t2 = GLint(srcTex.GetHeight());
		GLint x1 = 0, y1 = 0, x2 = GLint(dstTex.GetWidth()), y2 = GLint(dstTex.GetHeight());
		glBegin(GL_QUADS);
		switch(m_kernalWidth)
		{
		case 7:
		case 9:
			glMultiTexCoord2i(GL_TEXTURE0, s1-2, t1);
			glMultiTexCoord2i(GL_TEXTURE1, s1-1, t1);
			glMultiTexCoord2i(GL_TEXTURE2, s1  , t1);
			glMultiTexCoord2i(GL_TEXTURE3, s1+1, t1);
			glMultiTexCoord2i(GL_TEXTURE4, s1+2, t1);
			glVertex2i(x1, y1);
			glMultiTexCoord2i(GL_TEXTURE0, s1-2, t2);
			glMultiTexCoord2i(GL_TEXTURE1, s1-1, t2);
			glMultiTexCoord2i(GL_TEXTURE2, s1  , t2);
			glMultiTexCoord2i(GL_TEXTURE3, s1+1, t2);
			glMultiTexCoord2i(GL_TEXTURE4, s1+2, t2);
			glVertex2i(x1, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2-2, t2);
			glMultiTexCoord2i(GL_TEXTURE1, s2-1, t2);
			glMultiTexCoord2i(GL_TEXTURE2, s2  , t2);
			glMultiTexCoord2i(GL_TEXTURE3, s2+1, t2);
			glMultiTexCoord2i(GL_TEXTURE4, s2+2, t2);
			glVertex2i(x2, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2-2, t1);
			glMultiTexCoord2i(GL_TEXTURE1, s2-1, t1);
			glMultiTexCoord2i(GL_TEXTURE2, s2  , t1);
			glMultiTexCoord2i(GL_TEXTURE3, s2+1, t1);
			glMultiTexCoord2i(GL_TEXTURE4, s2+2, t1);
			glVertex2i(x2, y1);
			break;
		case 11:
		case 13:
			glMultiTexCoord2i(GL_TEXTURE0, s1-3, t1);
			glMultiTexCoord2i(GL_TEXTURE1, s1-2, t1);
			glMultiTexCoord2i(GL_TEXTURE2, s1-1, t1);
			glMultiTexCoord2i(GL_TEXTURE3, s1  , t1);
			glMultiTexCoord2i(GL_TEXTURE4, s1+1, t1);
			glMultiTexCoord2i(GL_TEXTURE5, s1+2, t1);
			glMultiTexCoord2i(GL_TEXTURE6, s1+3, t1);
			glVertex2i(x1, y1);
			glMultiTexCoord2i(GL_TEXTURE0, s1-3, t2);
			glMultiTexCoord2i(GL_TEXTURE1, s1-2, t2);
			glMultiTexCoord2i(GL_TEXTURE2, s1-1, t2);
			glMultiTexCoord2i(GL_TEXTURE3, s1  , t2);
			glMultiTexCoord2i(GL_TEXTURE4, s1+1, t2);
			glMultiTexCoord2i(GL_TEXTURE5, s1+2, t2);
			glMultiTexCoord2i(GL_TEXTURE6, s1+3, t2);
			glVertex2i(x1, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2-3, t2);
			glMultiTexCoord2i(GL_TEXTURE1, s2-2, t2);
			glMultiTexCoord2i(GL_TEXTURE2, s2-1, t2);
			glMultiTexCoord2i(GL_TEXTURE3, s2  , t2);
			glMultiTexCoord2i(GL_TEXTURE4, s2+1, t2);
			glMultiTexCoord2i(GL_TEXTURE5, s2+2, t2);
			glMultiTexCoord2i(GL_TEXTURE6, s2+3, t2);
			glVertex2i(x2, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2-3, t1);
			glMultiTexCoord2i(GL_TEXTURE1, s2-2, t1);
			glMultiTexCoord2i(GL_TEXTURE2, s2-1, t1);
			glMultiTexCoord2i(GL_TEXTURE3, s2  , t1);
			glMultiTexCoord2i(GL_TEXTURE4, s2+1, t1);
			glMultiTexCoord2i(GL_TEXTURE5, s2+2, t1);
			glMultiTexCoord2i(GL_TEXTURE6, s2+3, t1);
			glVertex2i(x2, y1);
			break;
		case 15:
		case 17:
			glMultiTexCoord2i(GL_TEXTURE0, s1-4, t1);
			glMultiTexCoord2i(GL_TEXTURE1, s1-3, t1);
			glMultiTexCoord2i(GL_TEXTURE2, s1-2, t1);
			glMultiTexCoord2i(GL_TEXTURE3, s1-1, t1);
			glMultiTexCoord2i(GL_TEXTURE4, s1  , t1);
			glMultiTexCoord2i(GL_TEXTURE5, s1+1, t1);
			glMultiTexCoord2i(GL_TEXTURE6, s1+2, t1);
			glMultiTexCoord2i(GL_TEXTURE7, s1+3, t1);
			glVertex2i(x1, y1);
			glMultiTexCoord2i(GL_TEXTURE0, s1-4, t2);
			glMultiTexCoord2i(GL_TEXTURE1, s1-3, t2);
			glMultiTexCoord2i(GL_TEXTURE2, s1-2, t2);
			glMultiTexCoord2i(GL_TEXTURE3, s1-1, t2);
			glMultiTexCoord2i(GL_TEXTURE4, s1  , t2);
			glMultiTexCoord2i(GL_TEXTURE5, s1+1, t2);
			glMultiTexCoord2i(GL_TEXTURE6, s1+2, t2);
			glMultiTexCoord2i(GL_TEXTURE7, s1+3, t2);
			glVertex2i(x1, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2-4, t2);
			glMultiTexCoord2i(GL_TEXTURE1, s2-3, t2);
			glMultiTexCoord2i(GL_TEXTURE2, s2-2, t2);
			glMultiTexCoord2i(GL_TEXTURE3, s2-1, t2);
			glMultiTexCoord2i(GL_TEXTURE4, s2  , t2);
			glMultiTexCoord2i(GL_TEXTURE5, s2+1, t2);
			glMultiTexCoord2i(GL_TEXTURE6, s2+2, t2);
			glMultiTexCoord2i(GL_TEXTURE7, s2+3, t2);
			glVertex2i(x2, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2-4, t1);
			glMultiTexCoord2i(GL_TEXTURE1, s2-3, t1);
			glMultiTexCoord2i(GL_TEXTURE2, s2-2, t1);
			glMultiTexCoord2i(GL_TEXTURE3, s2-1, t1);
			glMultiTexCoord2i(GL_TEXTURE4, s2  , t1);
			glMultiTexCoord2i(GL_TEXTURE5, s2+1, t1);
			glMultiTexCoord2i(GL_TEXTURE6, s2+2, t1);
			glMultiTexCoord2i(GL_TEXTURE7, s2+3, t1);
			glVertex2i(x2, y1);
			break;
		case 19:
		case 21:
			glMultiTexCoord2i(GL_TEXTURE0, s1-5, t1);
			glMultiTexCoord2i(GL_TEXTURE1, s1-4, t1);
			glMultiTexCoord2i(GL_TEXTURE2, s1-3, t1);
			glMultiTexCoord2i(GL_TEXTURE3, s1-2, t1);
			glMultiTexCoord2i(GL_TEXTURE4, s1-1, t1);
			glMultiTexCoord2i(GL_TEXTURE5, s1  , t1);
			glMultiTexCoord2i(GL_TEXTURE6, s1+1, t1);
			glMultiTexCoord2i(GL_TEXTURE7, s1+2, t1);
			glVertex2i(x1, y1);
			glMultiTexCoord2i(GL_TEXTURE0, s1-5, t2);
			glMultiTexCoord2i(GL_TEXTURE1, s1-4, t2);
			glMultiTexCoord2i(GL_TEXTURE2, s1-3, t2);
			glMultiTexCoord2i(GL_TEXTURE3, s1-2, t2);
			glMultiTexCoord2i(GL_TEXTURE4, s1-1, t2);
			glMultiTexCoord2i(GL_TEXTURE5, s1  , t2);
			glMultiTexCoord2i(GL_TEXTURE6, s1+1, t2);
			glMultiTexCoord2i(GL_TEXTURE7, s1+2, t2);
			glVertex2i(x1, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2-5, t2);
			glMultiTexCoord2i(GL_TEXTURE1, s2-4, t2);
			glMultiTexCoord2i(GL_TEXTURE2, s2-3, t2);
			glMultiTexCoord2i(GL_TEXTURE3, s2-2, t2);
			glMultiTexCoord2i(GL_TEXTURE4, s2-1, t2);
			glMultiTexCoord2i(GL_TEXTURE5, s2  , t2);
			glMultiTexCoord2i(GL_TEXTURE6, s2+1, t2);
			glMultiTexCoord2i(GL_TEXTURE7, s2+2, t2);
			glVertex2i(x2, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2-5, t1);
			glMultiTexCoord2i(GL_TEXTURE1, s2-4, t1);
			glMultiTexCoord2i(GL_TEXTURE2, s2-3, t1);
			glMultiTexCoord2i(GL_TEXTURE3, s2-2, t1);
			glMultiTexCoord2i(GL_TEXTURE4, s2-1, t1);
			glMultiTexCoord2i(GL_TEXTURE5, s2  , t1);
			glMultiTexCoord2i(GL_TEXTURE6, s2+1, t1);
			glMultiTexCoord2i(GL_TEXTURE7, s2+2, t1);
			glVertex2i(x2, y1);
			break;
		case 23:
		case 25:
			glMultiTexCoord2i(GL_TEXTURE0, s1-6, t1);
			glMultiTexCoord2i(GL_TEXTURE1, s1-5, t1);
			glMultiTexCoord2i(GL_TEXTURE2, s1-4, t1);
			glMultiTexCoord2i(GL_TEXTURE3, s1-3, t1);
			glMultiTexCoord2i(GL_TEXTURE4, s1-2, t1);
			glMultiTexCoord2i(GL_TEXTURE5, s1-1, t1);
			glMultiTexCoord2i(GL_TEXTURE6, s1  , t1);
			glMultiTexCoord2i(GL_TEXTURE7, s1+1, t1);
			glVertex2i(x1, y1);
			glMultiTexCoord2i(GL_TEXTURE0, s1-6, t2);
			glMultiTexCoord2i(GL_TEXTURE1, s1-5, t2);
			glMultiTexCoord2i(GL_TEXTURE2, s1-4, t2);
			glMultiTexCoord2i(GL_TEXTURE3, s1-3, t2);
			glMultiTexCoord2i(GL_TEXTURE4, s1-2, t2);
			glMultiTexCoord2i(GL_TEXTURE5, s1-1, t2);
			glMultiTexCoord2i(GL_TEXTURE6, s1  , t2);
			glMultiTexCoord2i(GL_TEXTURE7, s1+1, t2);
			glVertex2i(x1, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2-6, t2);
			glMultiTexCoord2i(GL_TEXTURE1, s2-5, t2);
			glMultiTexCoord2i(GL_TEXTURE2, s2-4, t2);
			glMultiTexCoord2i(GL_TEXTURE3, s2-3, t2);
			glMultiTexCoord2i(GL_TEXTURE4, s2-2, t2);
			glMultiTexCoord2i(GL_TEXTURE5, s2-1, t2);
			glMultiTexCoord2i(GL_TEXTURE6, s2  , t2);
			glMultiTexCoord2i(GL_TEXTURE7, s2+1, t2);
			glVertex2i(x2, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2-6, t1);
			glMultiTexCoord2i(GL_TEXTURE1, s2-5, t1);
			glMultiTexCoord2i(GL_TEXTURE2, s2-4, t1);
			glMultiTexCoord2i(GL_TEXTURE3, s2-3, t1);
			glMultiTexCoord2i(GL_TEXTURE4, s2-2, t1);
			glMultiTexCoord2i(GL_TEXTURE5, s2-1, t1);
			glMultiTexCoord2i(GL_TEXTURE6, s2  , t1);
			glMultiTexCoord2i(GL_TEXTURE7, s2+1, t1);
			glVertex2i(x2, y1);
			break;
		}
		glEnd();
		glFlush();
	}

};

class ProgramGLFilterPackedPass2 : public ProgramGLFilterPackedPass
{

public:

	virtual void SetSource(const std::vector<float> &kernel, char *source, std::vector<std::string> &inpTexNames, std::vector<std::string> &inpParamNames)
	{
#if _DEBUG
		assert(kernel.size() >= 7 && kernel.size() <= 25 && (kernel.size() & 1) == 1);
#endif
		m_kernalWidth = ushort(kernel.size());

		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out << "uniform sampler2DRect g_srcTex;\n";
		out << "void main()\n";
		out << "{\n";
		switch(m_kernalWidth)
		{
		case 7:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.ba, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src1.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.baba";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src2.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.baba";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src3.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.baba";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[6] << ") * src4.rg);\n";
			break;
		case 9:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.rg, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src0.baba";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src1.baba";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src2.baba";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src3.baba";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.rgrg";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[8] << ") * src4.ba);\n";
			break;
		case 11:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.ba, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src1.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.baba";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src2.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.baba";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src3.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.baba";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src4.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.baba";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src5.rgrg";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.baba";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[10] << ") * src6.rg);\n";
			break;
		case 13:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.rg, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src0.baba";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src1.baba";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src2.baba";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src3.baba";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src4.baba";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.rgrg";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src5.baba";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.rgrg";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[12] << ") * src6.ba);\n";
			break;
		case 15:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 1));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.ba, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src1.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.baba";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src2.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.baba";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src3.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.baba";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src4.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.baba";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src5.rgrg";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.baba";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src6.rgrg";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.baba";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src7.rgrg";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.baba";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[14] << ") * src8.rg);\n";
			break;
		case 17:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 1));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.rg, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src0.baba";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src1.baba";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src2.baba";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src3.baba";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src4.baba";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.rgrg";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src5.baba";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.rgrg";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src6.baba";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.rgrg";
			out <<			   " + vec4(vec2(" << kernel[15] << "), vec2(" << kernel[14] << ")) * src7.baba";
			out <<			   " + vec4(vec2(" << kernel[16] << "), vec2(" << kernel[15] << ")) * src8.rgrg";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[16] << ") * src8.ba);\n";
			break;
		case 19:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 1));\n";
			out << "vec4 src9 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 2));\n";
			out << "vec4 src10 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 3));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.ba, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src1.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.baba";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src2.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.baba";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src3.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.baba";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src4.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.baba";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src5.rgrg";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.baba";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src6.rgrg";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.baba";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src7.rgrg";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.baba";
			out <<			   " + vec4(vec2(" << kernel[15] << "), vec2(" << kernel[14] << ")) * src8.rgrg";
			out <<			   " + vec4(vec2(" << kernel[16] << "), vec2(" << kernel[15] << ")) * src8.baba";
			out <<			   " + vec4(vec2(" << kernel[17] << "), vec2(" << kernel[16] << ")) * src9.rgrg";
			out <<			   " + vec4(vec2(" << kernel[18] << "), vec2(" << kernel[17] << ")) * src9.baba";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[18] << ") * src10.rg);\n";
			break;
		case 21:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 1));\n";
			out << "vec4 src9 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 2));\n";
			out << "vec4 src10 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 3));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.rg, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src0.baba";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src1.baba";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src2.baba";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src3.baba";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src4.baba";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.rgrg";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src5.baba";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.rgrg";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src6.baba";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.rgrg";
			out <<			   " + vec4(vec2(" << kernel[15] << "), vec2(" << kernel[14] << ")) * src7.baba";
			out <<			   " + vec4(vec2(" << kernel[16] << "), vec2(" << kernel[15] << ")) * src8.rgrg";
			out <<			   " + vec4(vec2(" << kernel[17] << "), vec2(" << kernel[16] << ")) * src8.baba";
			out <<			   " + vec4(vec2(" << kernel[18] << "), vec2(" << kernel[17] << ")) * src9.rgrg";
			out <<			   " + vec4(vec2(" << kernel[19] << "), vec2(" << kernel[18] << ")) * src9.baba";
			out <<			   " + vec4(vec2(" << kernel[20] << "), vec2(" << kernel[19] << ")) * src10.rgrg";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[20] << ") * src10.ba);\n";
			break;
		case 23:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 1));\n";
			out << "vec4 src9 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 2));\n";
			out << "vec4 src10 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 3));\n";
			out << "vec4 src11 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 4));\n";
			out << "vec4 src12 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 5));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.ba, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src1.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.baba";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src2.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.baba";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src3.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.baba";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src4.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.baba";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src5.rgrg";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.baba";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src6.rgrg";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.baba";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src7.rgrg";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.baba";
			out <<			   " + vec4(vec2(" << kernel[15] << "), vec2(" << kernel[14] << ")) * src8.rgrg";
			out <<			   " + vec4(vec2(" << kernel[16] << "), vec2(" << kernel[15] << ")) * src8.baba";
			out <<			   " + vec4(vec2(" << kernel[17] << "), vec2(" << kernel[16] << ")) * src9.rgrg";
			out <<			   " + vec4(vec2(" << kernel[18] << "), vec2(" << kernel[17] << ")) * src9.baba";
			out <<			   " + vec4(vec2(" << kernel[19] << "), vec2(" << kernel[18] << ")) * src10.rgrg";
			out <<			   " + vec4(vec2(" << kernel[20] << "), vec2(" << kernel[19] << ")) * src10.baba";
			out <<			   " + vec4(vec2(" << kernel[21] << "), vec2(" << kernel[20] << ")) * src11.rgrg";
			out <<			   " + vec4(vec2(" << kernel[22] << "), vec2(" << kernel[21] << ")) * src11.baba";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[22] << ") * src12.rg);\n";
			break;
		case 25:
			out << "vec4 src0 = texture2DRect(g_srcTex, gl_TexCoord[0].st);\n";
			out << "vec4 src1 = texture2DRect(g_srcTex, gl_TexCoord[1].st);\n";
			out << "vec4 src2 = texture2DRect(g_srcTex, gl_TexCoord[2].st);\n";
			out << "vec4 src3 = texture2DRect(g_srcTex, gl_TexCoord[3].st);\n";
			out << "vec4 src4 = texture2DRect(g_srcTex, gl_TexCoord[4].st);\n";
			out << "vec4 src5 = texture2DRect(g_srcTex, gl_TexCoord[5].st);\n";
			out << "vec4 src6 = texture2DRect(g_srcTex, gl_TexCoord[6].st);\n";
			out << "vec4 src7 = texture2DRect(g_srcTex, gl_TexCoord[7].st);\n";
			out << "vec4 src8 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 1));\n";
			out << "vec4 src9 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 2));\n";
			out << "vec4 src10 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 3));\n";
			out << "vec4 src11 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 4));\n";
			out << "vec4 src12 = texture2DRect(g_srcTex, gl_TexCoord[7].st + vec2(0, 5));\n";
			out << "gl_FragColor = vec4(vec2(" << kernel[ 0] << ") * src0.rg, vec2(0))";
			out <<			   " + vec4(vec2(" << kernel[ 1] << "), vec2(" << kernel[ 0] << ")) * src0.baba";
			out <<			   " + vec4(vec2(" << kernel[ 2] << "), vec2(" << kernel[ 1] << ")) * src1.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 3] << "), vec2(" << kernel[ 2] << ")) * src1.baba";
			out <<			   " + vec4(vec2(" << kernel[ 4] << "), vec2(" << kernel[ 3] << ")) * src2.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 5] << "), vec2(" << kernel[ 4] << ")) * src2.baba";
			out <<			   " + vec4(vec2(" << kernel[ 6] << "), vec2(" << kernel[ 5] << ")) * src3.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 7] << "), vec2(" << kernel[ 6] << ")) * src3.baba";
			out <<			   " + vec4(vec2(" << kernel[ 8] << "), vec2(" << kernel[ 7] << ")) * src4.rgrg";
			out <<			   " + vec4(vec2(" << kernel[ 9] << "), vec2(" << kernel[ 8] << ")) * src4.baba";
			out <<			   " + vec4(vec2(" << kernel[10] << "), vec2(" << kernel[ 9] << ")) * src5.rgrg";
			out <<			   " + vec4(vec2(" << kernel[11] << "), vec2(" << kernel[10] << ")) * src5.baba";
			out <<			   " + vec4(vec2(" << kernel[12] << "), vec2(" << kernel[11] << ")) * src6.rgrg";
			out <<			   " + vec4(vec2(" << kernel[13] << "), vec2(" << kernel[12] << ")) * src6.baba";
			out <<			   " + vec4(vec2(" << kernel[14] << "), vec2(" << kernel[13] << ")) * src7.rgrg";
			out <<			   " + vec4(vec2(" << kernel[15] << "), vec2(" << kernel[14] << ")) * src7.baba";
			out <<			   " + vec4(vec2(" << kernel[16] << "), vec2(" << kernel[15] << ")) * src8.rgrg";
			out <<			   " + vec4(vec2(" << kernel[17] << "), vec2(" << kernel[16] << ")) * src8.baba";
			out <<			   " + vec4(vec2(" << kernel[18] << "), vec2(" << kernel[17] << ")) * src9.rgrg";
			out <<			   " + vec4(vec2(" << kernel[19] << "), vec2(" << kernel[18] << ")) * src9.baba";
			out <<			   " + vec4(vec2(" << kernel[20] << "), vec2(" << kernel[19] << ")) * src10.rgrg";
			out <<			   " + vec4(vec2(" << kernel[21] << "), vec2(" << kernel[20] << ")) * src10.baba";
			out <<			   " + vec4(vec2(" << kernel[22] << "), vec2(" << kernel[21] << ")) * src11.rgrg";
			out <<			   " + vec4(vec2(" << kernel[23] << "), vec2(" << kernel[22] << ")) * src11.baba";
			out <<			   " + vec4(vec2(" << kernel[24] << "), vec2(" << kernel[23] << ")) * src12.rgrg";
			out <<			   " + vec4(vec2(0), vec2(" << kernel[24] << ") * src12.ba);\n";
			break;
		}
		out << "gl_FragDepth = 1.0f;\n";
		out << "}\n";
		out << '\0';

		//printf("%s", source);

		inpTexNames.resize(1);
		inpTexNames[0] = "g_srcTex";
		inpParamNames.resize(0);
	}
	virtual void DrawQuad(const TextureGL4 &srcTex, const TextureGL4 &dstTex) const
	{
#if _DEBUG
		assert(srcTex.GetWidth() == dstTex.GetWidth() && srcTex.GetHeight() == dstTex.GetHeight());
#endif
		GLint s1 = 0, t1 = 0, s2 = GLint(srcTex.GetWidth()), t2 = GLint(srcTex.GetHeight());
		GLint x1 = 0, y1 = 0, x2 = GLint(dstTex.GetWidth()), y2 = GLint(dstTex.GetHeight());
		glBegin(GL_QUADS);
		switch(m_kernalWidth)
		{
		case 7:
		case 9:
			glMultiTexCoord2i(GL_TEXTURE0, s1, t1-2);
			glMultiTexCoord2i(GL_TEXTURE1, s1, t1-1);
			glMultiTexCoord2i(GL_TEXTURE2, s1, t1  );
			glMultiTexCoord2i(GL_TEXTURE3, s1, t1+1);
			glMultiTexCoord2i(GL_TEXTURE4, s1, t1+2);
			glVertex2i(x1, y1);
			glMultiTexCoord2i(GL_TEXTURE0, s1, t2-2);
			glMultiTexCoord2i(GL_TEXTURE1, s1, t2-1);
			glMultiTexCoord2i(GL_TEXTURE2, s1, t2  );
			glMultiTexCoord2i(GL_TEXTURE3, s1, t2+1);
			glMultiTexCoord2i(GL_TEXTURE4, s1, t2+2);
			glVertex2i(x1, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2, t2-2);
			glMultiTexCoord2i(GL_TEXTURE1, s2, t2-1);
			glMultiTexCoord2i(GL_TEXTURE2, s2, t2  );
			glMultiTexCoord2i(GL_TEXTURE3, s2, t2+1);
			glMultiTexCoord2i(GL_TEXTURE4, s2, t2+2);
			glVertex2i(x2, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2, t1-2);
			glMultiTexCoord2i(GL_TEXTURE1, s2, t1-1);
			glMultiTexCoord2i(GL_TEXTURE2, s2, t1  );
			glMultiTexCoord2i(GL_TEXTURE3, s2, t1+1);
			glMultiTexCoord2i(GL_TEXTURE4, s2, t1+2);
			glVertex2i(x2, y1);
			break;
		case 11:
		case 13:
			glMultiTexCoord2i(GL_TEXTURE0, s1, t1-3);
			glMultiTexCoord2i(GL_TEXTURE1, s1, t1-2);
			glMultiTexCoord2i(GL_TEXTURE2, s1, t1-1);
			glMultiTexCoord2i(GL_TEXTURE3, s1, t1  );
			glMultiTexCoord2i(GL_TEXTURE4, s1, t1+1);
			glMultiTexCoord2i(GL_TEXTURE5, s1, t1+2);
			glMultiTexCoord2i(GL_TEXTURE6, s1, t1+3);
			glVertex2i(x1, y1);
			glMultiTexCoord2i(GL_TEXTURE0, s1, t2-3);
			glMultiTexCoord2i(GL_TEXTURE1, s1, t2-2);
			glMultiTexCoord2i(GL_TEXTURE2, s1, t2-1);
			glMultiTexCoord2i(GL_TEXTURE3, s1, t2  );
			glMultiTexCoord2i(GL_TEXTURE4, s1, t2+1);
			glMultiTexCoord2i(GL_TEXTURE5, s1, t2+2);
			glMultiTexCoord2i(GL_TEXTURE6, s1, t2+3);
			glVertex2i(x1, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2, t2-3);
			glMultiTexCoord2i(GL_TEXTURE1, s2, t2-2);
			glMultiTexCoord2i(GL_TEXTURE2, s2, t2-1);
			glMultiTexCoord2i(GL_TEXTURE3, s2, t2  );
			glMultiTexCoord2i(GL_TEXTURE4, s2, t2+1);
			glMultiTexCoord2i(GL_TEXTURE5, s2, t2+2);
			glMultiTexCoord2i(GL_TEXTURE6, s2, t2+3);
			glVertex2i(x2, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2, t1-3);
			glMultiTexCoord2i(GL_TEXTURE1, s2, t1-2);
			glMultiTexCoord2i(GL_TEXTURE2, s2, t1-1);
			glMultiTexCoord2i(GL_TEXTURE3, s2, t1  );
			glMultiTexCoord2i(GL_TEXTURE4, s2, t1+1);
			glMultiTexCoord2i(GL_TEXTURE5, s2, t1+2);
			glMultiTexCoord2i(GL_TEXTURE6, s2, t1+3);
			glVertex2i(x2, y1);
			break;
		case 15:
		case 17:
			glMultiTexCoord2i(GL_TEXTURE0, s1, t1-4);
			glMultiTexCoord2i(GL_TEXTURE1, s1, t1-3);
			glMultiTexCoord2i(GL_TEXTURE2, s1, t1-2);
			glMultiTexCoord2i(GL_TEXTURE3, s1, t1-1);
			glMultiTexCoord2i(GL_TEXTURE4, s1, t1  );
			glMultiTexCoord2i(GL_TEXTURE5, s1, t1+1);
			glMultiTexCoord2i(GL_TEXTURE6, s1, t1+2);
			glMultiTexCoord2i(GL_TEXTURE7, s1, t1+3);
			glVertex2i(x1, y1);
			glMultiTexCoord2i(GL_TEXTURE0, s1, t2-4);
			glMultiTexCoord2i(GL_TEXTURE1, s1, t2-3);
			glMultiTexCoord2i(GL_TEXTURE2, s1, t2-2);
			glMultiTexCoord2i(GL_TEXTURE3, s1, t2-1);
			glMultiTexCoord2i(GL_TEXTURE4, s1, t2  );
			glMultiTexCoord2i(GL_TEXTURE5, s1, t2+1);
			glMultiTexCoord2i(GL_TEXTURE6, s1, t2+2);
			glMultiTexCoord2i(GL_TEXTURE7, s1, t2+3);
			glVertex2i(x1, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2, t2-4);
			glMultiTexCoord2i(GL_TEXTURE1, s2, t2-3);
			glMultiTexCoord2i(GL_TEXTURE2, s2, t2-2);
			glMultiTexCoord2i(GL_TEXTURE3, s2, t2-1);
			glMultiTexCoord2i(GL_TEXTURE4, s2, t2  );
			glMultiTexCoord2i(GL_TEXTURE5, s2, t2+1);
			glMultiTexCoord2i(GL_TEXTURE6, s2, t2+2);
			glMultiTexCoord2i(GL_TEXTURE7, s2, t2+3);
			glVertex2i(x2, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2, t1-4);
			glMultiTexCoord2i(GL_TEXTURE1, s2, t1-3);
			glMultiTexCoord2i(GL_TEXTURE2, s2, t1-2);
			glMultiTexCoord2i(GL_TEXTURE3, s2, t1-1);
			glMultiTexCoord2i(GL_TEXTURE4, s2, t1  );
			glMultiTexCoord2i(GL_TEXTURE5, s2, t1+1);
			glMultiTexCoord2i(GL_TEXTURE6, s2, t1+2);
			glMultiTexCoord2i(GL_TEXTURE7, s2, t1+3);
			glVertex2i(x2, y1);
			break;
		case 19:
		case 21:
			glMultiTexCoord2i(GL_TEXTURE0, s1, t1-5);
			glMultiTexCoord2i(GL_TEXTURE1, s1, t1-4);
			glMultiTexCoord2i(GL_TEXTURE2, s1, t1-3);
			glMultiTexCoord2i(GL_TEXTURE3, s1, t1-2);
			glMultiTexCoord2i(GL_TEXTURE4, s1, t1-1);
			glMultiTexCoord2i(GL_TEXTURE5, s1, t1  );
			glMultiTexCoord2i(GL_TEXTURE6, s1, t1+1);
			glMultiTexCoord2i(GL_TEXTURE7, s1, t1+2);
			glVertex2i(x1, y1);
			glMultiTexCoord2i(GL_TEXTURE0, s1, t2-5);
			glMultiTexCoord2i(GL_TEXTURE1, s1, t2-4);
			glMultiTexCoord2i(GL_TEXTURE2, s1, t2-3);
			glMultiTexCoord2i(GL_TEXTURE3, s1, t2-2);
			glMultiTexCoord2i(GL_TEXTURE4, s1, t2-1);
			glMultiTexCoord2i(GL_TEXTURE5, s1, t2  );
			glMultiTexCoord2i(GL_TEXTURE6, s1, t2+1);
			glMultiTexCoord2i(GL_TEXTURE7, s1, t2+2);
			glVertex2i(x1, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2, t2-5);
			glMultiTexCoord2i(GL_TEXTURE1, s2, t2-4);
			glMultiTexCoord2i(GL_TEXTURE2, s2, t2-3);
			glMultiTexCoord2i(GL_TEXTURE3, s2, t2-2);
			glMultiTexCoord2i(GL_TEXTURE4, s2, t2-1);
			glMultiTexCoord2i(GL_TEXTURE5, s2, t2  );
			glMultiTexCoord2i(GL_TEXTURE6, s2, t2+1);
			glMultiTexCoord2i(GL_TEXTURE7, s2, t2+2);
			glVertex2i(x2, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2, t1-5);
			glMultiTexCoord2i(GL_TEXTURE1, s2, t1-4);
			glMultiTexCoord2i(GL_TEXTURE2, s2, t1-3);
			glMultiTexCoord2i(GL_TEXTURE3, s2, t1-2);
			glMultiTexCoord2i(GL_TEXTURE4, s2, t1-1);
			glMultiTexCoord2i(GL_TEXTURE5, s2, t1  );
			glMultiTexCoord2i(GL_TEXTURE6, s2, t1+1);
			glMultiTexCoord2i(GL_TEXTURE7, s2, t1+2);
			glVertex2i(x2, y1);
			break;
		case 23:
		case 25:
			glMultiTexCoord2i(GL_TEXTURE0, s1, t1-6);
			glMultiTexCoord2i(GL_TEXTURE1, s1, t1-5);
			glMultiTexCoord2i(GL_TEXTURE2, s1, t1-4);
			glMultiTexCoord2i(GL_TEXTURE3, s1, t1-3);
			glMultiTexCoord2i(GL_TEXTURE4, s1, t1-2);
			glMultiTexCoord2i(GL_TEXTURE5, s1, t1-1);
			glMultiTexCoord2i(GL_TEXTURE6, s1, t1  );
			glMultiTexCoord2i(GL_TEXTURE7, s1, t1+1);
			glVertex2i(x1, y1);
			glMultiTexCoord2i(GL_TEXTURE0, s1, t2-6);
			glMultiTexCoord2i(GL_TEXTURE1, s1, t2-5);
			glMultiTexCoord2i(GL_TEXTURE2, s1, t2-4);
			glMultiTexCoord2i(GL_TEXTURE3, s1, t2-3);
			glMultiTexCoord2i(GL_TEXTURE4, s1, t2-2);
			glMultiTexCoord2i(GL_TEXTURE5, s1, t2-1);
			glMultiTexCoord2i(GL_TEXTURE6, s1, t2  );
			glMultiTexCoord2i(GL_TEXTURE7, s1, t2+1);
			glVertex2i(x1, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2, t2-6);
			glMultiTexCoord2i(GL_TEXTURE1, s2, t2-5);
			glMultiTexCoord2i(GL_TEXTURE2, s2, t2-4);
			glMultiTexCoord2i(GL_TEXTURE3, s2, t2-3);
			glMultiTexCoord2i(GL_TEXTURE4, s2, t2-2);
			glMultiTexCoord2i(GL_TEXTURE5, s2, t2-1);
			glMultiTexCoord2i(GL_TEXTURE6, s2, t2  );
			glMultiTexCoord2i(GL_TEXTURE7, s2, t2+1);
			glVertex2i(x2, y2);
			glMultiTexCoord2i(GL_TEXTURE0, s2, t1-6);
			glMultiTexCoord2i(GL_TEXTURE1, s2, t1-5);
			glMultiTexCoord2i(GL_TEXTURE2, s2, t1-4);
			glMultiTexCoord2i(GL_TEXTURE3, s2, t1-3);
			glMultiTexCoord2i(GL_TEXTURE4, s2, t1-2);
			glMultiTexCoord2i(GL_TEXTURE5, s2, t1-1);
			glMultiTexCoord2i(GL_TEXTURE6, s2, t1  );
			glMultiTexCoord2i(GL_TEXTURE7, s2, t1+1);
			glVertex2i(x2, y1);
			break;
		}
		glEnd();
		glFlush();
	}
};

class ProgramGLFilterPacked
{

public:

	//inline const float& GetSigma(const ushort &iLevel) const { return m_sigmas[iLevel]; }
	//inline const std::vector<float>& GetKernel(const ushort &iLevel) const { return m_kernels[iLevel]; }
	inline void Initialize(const ushort &nLevels, const float &dSigma, const float &sigmak, const bool &upSample)
	{
		//m_sigmas.resize(nLevels);
		//m_kernels.resize(nLevels);
		m_pass1.resize(nLevels);
		m_pass2.resize(nLevels);

		float sigma;
		std::vector<float> kernel;
		for(ushort iLevel = 0; iLevel < nLevels; ++iLevel)
		{
			//float &sigma = m_sigmas[iLevel];
			if(iLevel == 0)
			{
				//float sa = sigma0 / sigmak;
				float sa = 1.6f;
				float sb = upSample ? 1.0f : 0.5f;
				sigma = sqrt(sa * sa - sb * sb);
			}
			else
				sigma = dSigma * powf(sigmak, float(iLevel - 1));

			const ushort widthHalf = ushort(ceil(4.0f * sigma - 0.5f)), width = (widthHalf << 1) + 1;
			//std::vector<float> &kernel = m_kernels[iLevel];
			kernel.resize(width);

			const float one_over_sigma2 = 1 / (sigma * sigma);
			float *p = kernel.data(), sum = 0;
			for(short i = -widthHalf; i <= widthHalf; ++i, ++p)
			{
				*p = exp(-0.5f * i * i * one_over_sigma2);
				sum += *p;
			}
			const float s = 1 / sum;
			for(ushort i = 0; i < width; ++i)
				kernel[i] *= s;

			m_pass1[iLevel].Initialize(kernel);
			m_pass2[iLevel].Initialize(kernel);
		}
	}
	inline void Run(const ushort &iLevel, const TextureGL4 &srcTex, const TextureGL4 &dstTex, const TextureGL4 &tmpTex) const
	{
		m_pass1[iLevel].Run(srcTex, tmpTex);
		m_pass2[iLevel].Run(tmpTex, dstTex);
	}

private:

	//std::vector<float> m_sigmas;
	//std::vector<std::vector<float> > m_kernels;
	std::vector<ProgramGLFilterPackedPass1> m_pass1;
	std::vector<ProgramGLFilterPackedPass2> m_pass2;

};

#endif