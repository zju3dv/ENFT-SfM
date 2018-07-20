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

#ifndef _PROGRAM_GL_ASSIGN_ORIENTATION_H_
#define _PROGRAM_GL_ASSIGN_ORIENTATION_H_

#include "ProgramGL.h"

class ProgramGLAssignOrientation
{

public:

	inline void Initialize(const ushort &ftrTexWidth, const std::vector<LA::Vector2us> &octaveSizes, const float &gaussFactor = 1.5f, const float &winFactor = 2.0f)
	{
		m_ftrTexWidth = ftrTexWidth;
		m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
#endif
		const ushort nOctaves = ushort(octaveSizes.size());
		m_programs.resize(nOctaves);
		
		for(ushort iOctave = 0; iOctave < nOctaves; ++iOctave)
		{
			const ushort width = (octaveSizes[iOctave].v0() << 1), height = (octaveSizes[iOctave].v1() << 1);
			char source[MAX_SOURCE_LENGTH];
			std::ostrstream out(source, MAX_SOURCE_LENGTH);
			out <<
				"#pragma optionNV(ifcvt none)\n"
				"#pragma optionNV(unroll all)\n"
			
				"#define FACTOR_RAD_TO_36_BINS " << 18 / PI << "\n"
				"#define FACTOR_36_BINS_TO_RAD " << PI / 18 << "\n"

				"uniform sampler2DRect g_ftrTex;\n"
				"uniform sampler2DRect g_gradMagTex;\n"
				"uniform sampler2DRect g_gradDirTex;\n"
				"void main()\n"
				"{\n"
				"	vec4 bins[10];\n"
				"	bins[0] = vec4(0);	bins[1] = vec4(0);	bins[2] = vec4(0);	bins[3] = vec4(0);	bins[4] = vec4(0);\n"
				"	bins[5] = vec4(0);	bins[6] = vec4(0);	bins[7] = vec4(0);	bins[8] = vec4(0);	bins[9] = vec4(0);\n"
				"	vec4 ftr = texture2DRect(g_ftrTex, gl_FragCoord.st);\n"
				"	vec2 center = ftr.rg;\n"
				"	float sigma = ftr.b;\n"
				"	float gsigma = sigma * " << gaussFactor << ";\n"
				"	vec2 win = vec2(sigma * " << gaussFactor * winFactor << ");\n"
				"	vec4 dist2Th = vec4(win.x * win.x + 0.5);\n"
				"	float factor = -0.5 / (gsigma * gsigma);\n"
				//"	vec4 range = floor(vec4(center - win, center + win) * 0.5) + 0.5;\n"
				"	vec4 range = floor(vec4(max(center - win, vec2(2)), min(center + win, vec2(" << width - 3 << ", " << height - 3 << "))) * 0.5) + 0.5;\n"

				//"	if(ftr.x > 191.6 && ftr.x < 191.7 && ftr.y > 128.5 && ftr.y < 128.6)\n"
				//"	{\n"
				////"		gl_FragColor = vec4(max(center - win, vec2(2)), min(center + win, vec2(" << width - 3 << ", " << height - 3 << ")));\n"
				////"		gl_FragColor = floor(gl_FragColor * 0.5) + 0.5;\n"
				//"		gl_FragColor = vec4(" << height - 3 << ");\n"
				//"		return;\n"
				//"	}\n"

				"	vec2 pos;\n"
					//loop to get the histogram
				"	for(pos.y = range.y; pos.y <= range.w;	++pos.y)\n"
				"	{\n"
				"		for(pos.x = range.x; pos.x <= range.z; ++pos.x)\n"
				"		{\n"
				"			vec4 offset;\n"
				"			offset.rg = pos + pos - center - 0.5;\n"
				"			offset.ba = offset.rg + 1;\n"
				"			vec4 offset2 = offset * offset;\n"
				"			vec4 dist2 = offset2.rbrb + offset2.ggaa;\n"
				"			bvec4 inside = lessThan(dist2, dist2Th);\n"
				"			if(any(inside))\n"
				"			{\n"
				"				vec4 gradMag = texture2DRect(g_gradMagTex, pos);\n"
				"				vec4 gradDir = texture2DRect(g_gradDirTex, pos);\n"
				"				vec4 weight = gradMag * exp(dist2 * factor);\n"
				"				ivec4 iBin  = ivec4(floor(gradDir * FACTOR_RAD_TO_36_BINS));\n"
				"				iBin += (ivec4(lessThan(iBin, ivec4(0))) * 36);\n"
				//"				if(iBin.r >= 0)\n"
				//"				{\n"
				//"					gl_FragColor = vec4(iBin);\n"
				//"					return;\n"
				//"				}\n"
				"				ivec4 idx1 = (iBin >> 2);\n"
				"				ivec4 idx2 = (iBin & 3);\n"
				"				bins[idx1.r][idx2.r] = inside.r ? bins[idx1.r][idx2.r] + weight.r : bins[idx1.r][idx2.r];\n"
				"				bins[idx1.g][idx2.g] = inside.g ? bins[idx1.g][idx2.g] + weight.g : bins[idx1.g][idx2.g];\n"
				"				bins[idx1.b][idx2.b] = inside.b ? bins[idx1.b][idx2.b] + weight.b : bins[idx1.b][idx2.b];\n"
				"				bins[idx1.a][idx2.a] = inside.a ? bins[idx1.a][idx2.a] + weight.a : bins[idx1.a][idx2.a];\n"
				"			}\n"
				"		}\n"
				"	}\n";

			//smooth histogram and find the largest
			/*
				smoothing kernel:	 (1 3 6 7 6 3 1 ) / 27
				the same as 3 pass of (1 1 1) / 3 averaging
				maybe better to use 4 pass on the vectors...
			*/
			//the inner loop on different array numbers is always unrolled in fp40
			out <<
				"	mat3 m1 = mat3(1, 3, 6, 0, 1, 3, 0, 0, 1) / 27.0;\n"
				"	mat4 m2 = mat4(7, 6, 3, 1, 6, 7, 6, 3, 3, 6, 7, 6, 1, 3, 6, 7) / 27.0;\n"
				"	#define FILTER_CODE(i) {						\\\n"
				"			vec4 newb	=	(bins[i] * m2);			\\\n"
				"			newb.xyz	+=	(prev.yzw * m1);		\\\n"
				"			prev = bins[i];							\\\n"
				"			newb.wzy	+=	(bins[i+1].zyx * m1);	\\\n"
				"			bins[i] = newb;}\n"
				"	for (int j=0; j<2; j++)\n"
				"	{\n"
				"		vec4 prev = bins[8];\n"
				"		bins[9] = bins[0];\n"
				"		for (int i=0; i<9; i++)\n"
				"		{\n"
				"			FILTER_CODE(i);\n"
				"		}\n"
				"	}\n";

			//find the maximum voting
			out <<
				"	vec4 maxh; vec2 maxh2;\n"
				"	vec4 maxh4 = max(max(max(max(max(max(max(max(bins[0], bins[1]), bins[2]), \n"
				"			bins[3]), bins[4]), bins[5]), bins[6]), bins[7]), bins[8]);\n"
				"	maxh2 = max(maxh4.xy, maxh4.zw); maxh = vec4(max(maxh2.x, maxh2.y));\n";

			//find the peaks
			out <<
				"	bvec4 test;\n"
				"	bins[9] = bins[0];\n"
				"	float prevb	= bins[8].w;\n"
				"	float di, th;\n"
				"	for (int i = 0, k = 0; i < 9; i++, k += 4)\n"
				"	{\n"
				"		test = equal(bins[i], maxh);\n"
				"		if(any(test))\n"
				"		{\n"
				"			if(test.r)\n"
				"			{\n"
				"				di = -0.5 * (bins[i].y - prevb) / (bins[i].y + prevb - bins[i].x - bins[i].x);\n"
				"				th = (k + di + 0.5);\n"
				"			}\n"
				"			else if(test.g)\n"
				"			{\n"
				"				di = -0.5 * (bins[i].z - bins[i].x) / (bins[i].z + bins[i].x - bins[i].y -  bins[i].y);\n"
				"				th = (k + di + 1.5);\n"
				"			}\n"
				"			else if(test.b)\n"
				"			{\n"
				"				di = -0.5 * (bins[i].w - bins[i].y) / (bins[i].w + bins[i].y - bins[i].z - bins[i].z);\n"
				"				th = (k + di + 2.5);\n"
				"			}\n"
				"			else\n"
				"			{\n"
				"				di = -0.5 * (bins[i + 1].x - bins[i].z) / (bins[i + 1].x + bins[i].z - bins[i].w - bins[i].w);\n"
				"				th = (k+di+3.5);\n"
				"			}\n"
				"			float rot = th * FACTOR_36_BINS_TO_RAD;\n"
				"			rot = rot > " << PI << " ? rot - " << PIx2 << " : rot;\n"
				"			gl_FragColor = vec4(center, sigma, rot);\n"
				//"			gl_FragColor = vec4(center, sigma, th);\n"
				"			return;\n"
				"		}\n"
				"		prevb = bins[i].w;\n"
				"	}\n";
			//end
			out <<
				"}\n" << '\0';

			//printf("%s\n", source);

			std::vector<std::string> inpTexNames(3), inpParamNames;
			inpTexNames[0] = "g_ftrTex";
			inpTexNames[1] = "g_gradMagTex";
			inpTexNames[2] = "g_gradDirTex";
			m_programs[iOctave].LoadSource(source, inpTexNames, inpParamNames);
		}
	}
	inline void Run(const ushort &iOctave, const TextureGL4 &ftrTex, const TextureGL4 &gradMagTex, const TextureGL4 &gradDirTex, const ushort &nFtrs) const
	{
#if _DEBUG
		assert(m_ftrTexWidth == ftrTex.GetWidth());
#endif

		const ProgramGL &program = m_programs[iOctave];
		program.Activate();
		program.SetInputTextures(ftrTex, gradMagTex, gradDirTex);
		program.SetOutputTexture(ftrTex);
		program.DrawQuad(nFtrs, m_ftrTexWidth, m_ftrTexWidthLog);
		program.Deactivate();
		program.UnbindInputTextures(ftrTex, gradMagTex, gradDirTex);
	}

private:

	ushort m_ftrTexWidth, m_ftrTexWidthLog;
	std::vector<ProgramGL> m_programs;
};

#endif