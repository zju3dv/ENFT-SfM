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

#ifndef _PROGRAM_GL_DESCRIPTOR_GENERATE_H_
#define _PROGRAM_GL_DESCRIPTOR_GENERATE_H_

#include "ProgramGL.h"

class ProgramGLDescriptorGenerate
{

public:

	inline void Initialize(const ushort &ftrTexWidth, const std::vector<LA::Vector2us> &octaveSizes)
	{
		m_descTexWidth = (ftrTexWidth << 4);
		m_descTexWidthLog = ushort(log(float(m_descTexWidth)) / log(2.0f));

#if _DEBUG
		assert((1 << m_descTexWidthLog) == m_descTexWidth);
#endif
		
		const ushort nOctaves = ushort(octaveSizes.size());
		m_programs.resize(nOctaves);
		for(ushort iOctave = 0; iOctave < nOctaves; ++iOctave)
		{
			char source[MAX_SOURCE_LENGTH];
			std::ostrstream out(source, MAX_SOURCE_LENGTH);

			const ushort width = (octaveSizes[iOctave].v0() << 1), height = (octaveSizes[iOctave].v1() << 1);

			out <<
				"#pragma optionNV(ifcvt none)\n"
				"#pragma optionNV(unroll all)\n"

				"#define FACTOR_RAD_TO_4_BINS " << 2 / PI << "\n"

				"uniform sampler2DRect g_ftrTex;\n"
				"uniform sampler2DRect g_gradMagTex;\n"
				"uniform sampler2DRect g_gradDirTex;\n"

				"void main()\n"
				"{\n"
				"	ivec2 pixCoord = ivec2(gl_FragCoord.st);\n"
				//"	int iFtr = (pixCoord.y << " << m_ftrTexWidthLog << ") + (pixCoord.x >> 4);\n"
				//"	ivec2 ftrCoord = ivec2(iFtr & " << ftrTexWidth - 1 << ", iFtr >> " << m_ftrTexWidthLog << ");\n"
				"	ivec2 ftrCoord = ivec2(pixCoord.x >> 4, pixCoord.y);\n"
				"	vec4 ftr = texture2DRect(g_ftrTex, ftrCoord + vec2(0.5));\n"
				"	vec2 winCoord = vec2((pixCoord.x & 3), ((pixCoord.x & 15) >> 2)) - vec2(1.5);\n"
				"	vec4 tmp;\n"
				"	tmp.x = cos(ftr.a);\n"
				"	tmp.y = sin(ftr.a);\n"
				"	tmp.zw = tmp.xy;\n"
				"	float spt = ftr.b * 3;\n"

				//"	if(pixCoord.x < 2000)\n"
				//"	{\n"
				//"		gl_FragColor = vec4(gl_FragCoord);\n"
				////"		gl_FragColor = vec4(ftrCoord.yx, winCoord.yx);\n"
				//"		return;\n"
				//"	}\n"

				"	tmp *= vec4(vec2(spt), vec2(1 / spt));\n"
				"	vec4 cscs = vec4(tmp.xy, -tmp.xy);\n"
				"	vec4 rots = vec4(tmp.zw, -tmp.zw);\n"
				"	tmp = cscs.xwyx * winCoord.xyxy;\n"
				"	vec2 samCoord = ftr.xy + tmp.xz + tmp.yw;\n"

				//get a horizontal bounding box of the rotated rectangle
				"	tmp.xy = abs(cscs.xy);\n"
				"	tmp.zw = vec2(tmp.x + tmp.y);\n"
				"	tmp.xy = -tmp.zw;\n"
				"	vec4 range = vec4(samCoord, samCoord) + tmp;\n"
				"	range = floor(vec4(max(range.rg, vec2(2)), min(range.ba, vec2(" << width - 3 << ", " << height - 3 << "))) * 0.5) + 0.5;\n"

				//"	if(iFtr == 64)\n"
				//"	{\n"
				//"		gl_FragColor = range;\n"
				//"		return;\n"
				//"	}\n"

				"	vec4 vote = vec4(0);\n"
				"	vec4 nox = vec4(0, rots.xy, rots.x + rots.y);\n"
				"	vec4 noy = vec4(0, rots.wx, rots.w + rots.x);\n"
				"	vec2 pos;\n"
				"	for(pos.y = range.y; pos.y <= range.w; ++pos.y)\n"
				"	{\n"
				"		for(pos.x = range.x; pos.x <= range.z; ++pos.x)\n"
				"		{\n"
				"			vec2 tpt = pos + pos - samCoord - 0.5;\n"
				"			vec4 temp = rots.xywx * tpt.xyxy;\n"
				"			vec2 temp2 = temp.xz + temp.yw;\n"
				"			vec4 nx = temp2.x + nox;\n"
				"			vec4 ny = temp2.y + noy;\n"
				"			vec4 nxn = abs(nx), nyn = abs(ny);\n"
				"			bvec4 inside = lessThan(max(nxn, nyn) , vec4(1));\n"
				"			if(any(inside))\n"
				"			{\n"
				"				vec4 gradMag = texture2DRect(g_gradMagTex, pos);\n"
				"				vec4 gradDir = texture2DRect(g_gradDirTex, pos);\n"
				"				vec4 iBin = (ftr.a - gradDir) * FACTOR_RAD_TO_4_BINS;\n"
				"				iBin += (vec4(lessThan(iBin, vec4(0))) * 4);\n"
				"				ivec4 iBin1 = ivec4(floor(iBin));\n"
				"				ivec4 iBin2 = ((iBin1 + 1) & 3);\n"
				"				vec4 diffx = nx + winCoord.x, diffy = ny + winCoord.y;\n"
				"				vec4 ww = exp(-0.125 * (diffx * diffx + diffy * diffy));\n"
				"				vec4 weight = (1 - nxn) * (1 - nyn) * gradMag * ww;\n"
				"				vec4 weight2 = (iBin - vec4(iBin1)) * weight;\n"
				"				vec4 weight1 = weight - weight2;\n"
				"				if(inside.r)\n"
				"				{\n"
				"					vote[iBin1.r] += weight1.r;\n"
				"					vote[iBin2.r] += weight2.r;\n"
				"				}\n"
				"				if(inside.g)\n"
				"				{\n"
				"					vote[iBin1.g] += weight1.g;\n"
				"					vote[iBin2.g] += weight2.g;\n"
				"				}\n"
				"				if(inside.b)\n"
				"				{\n"
				"					vote[iBin1.b] += weight1.b;\n"
				"					vote[iBin2.b] += weight2.b;\n"
				"				}\n"
				"				if(inside.a)\n"
				"				{\n"
				"					vote[iBin1.a] += weight1.a;\n"
				"					vote[iBin2.a] += weight2.a;\n"
				"				}\n"
				"			}\n"
				"		}\n"
				"	}\n"
				"	gl_FragColor = vote;\n"
				"}\n" << '\0';

			//printf("%s\n", source);

			std::vector<std::string> inpTexNames(3), inpParamNames;
			inpTexNames[0] = "g_ftrTex";
			inpTexNames[1] = "g_gradMagTex";
			inpTexNames[2] = "g_gradDirTex";
			m_programs[iOctave].LoadSource(source, inpTexNames, inpParamNames);
		}
	};
	inline void Run(const ushort &iOctave, const TextureGL4 &ftrTex, const TextureGL4 &gradMagTex, const TextureGL4 &gradDirTex, const TextureGL4 &descTex, 
					const ushort &nFtrs) const
	{
#if _DEBUG
		//assert(descTex.GetWidth() == (ftrTex.GetWidth() << 4) && descTex.GetHeight() == ftrTex.GetHeight() && descTex.GetWidth() == m_descTexWidth);
		assert(descTex.GetWidth() == (ftrTex.GetWidth() << 4) && descTex.GetHeight() <= ftrTex.GetHeight() && descTex.GetWidth() == m_descTexWidth);
#endif

		const ProgramGL &program = m_programs[iOctave];
		program.Activate();
		program.SetInputTextures(ftrTex, gradMagTex, gradDirTex);
		program.SetOutputTexture(descTex);
		program.DrawQuad(uint(nFtrs) << 4, m_descTexWidth, m_descTexWidthLog);
		program.UnbindInputTextures(ftrTex, gradMagTex, gradDirTex);
	}

private:

	ushort m_descTexWidth, m_descTexWidthLog;
	std::vector<ProgramGL> m_programs;
};

#endif