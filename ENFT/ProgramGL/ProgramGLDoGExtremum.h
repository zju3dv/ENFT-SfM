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

#ifndef _PROGRAM_GL_DOT_EXTREMUM_h_
#define _PROGRAM_GL_DOT_EXTREMUM_h_

#include "ProgramGL.h"

class ProgramGLDoGExtremum
{

public:

	inline void Initialize(const std::vector<LA::Vector2us> &octaveSizes, const float &dogTh, const float &edgeTh)
	{
		const ushort &nOctaves = ushort(octaveSizes.size());
		m_octaveSizes = octaveSizes;
		m_programs.resize(nOctaves);
		for(ushort iOctave = 0; iOctave < nOctaves; ++iOctave)
		{
			char source[MAX_SOURCE_LENGTH];
			std::ostrstream out(source, MAX_SOURCE_LENGTH);

			const float threshold2 = (edgeTh + 1) * (edgeTh + 1) / edgeTh;

			out << 
				"#pragma optionNV(ifcvt none)\n"
				"#pragma optionNV(unroll all)\n"
				"#define REPEAT4(FUNCTION)\\\n"
				"for(int i = 0; i < 4; ++i)\\\n"
				"{\\\n"
				"	FUNCTION(i);\\\n"
				"}\n";
			//X: (CLR) (CENTER 0, LEFT -1, RIGHT +1)  
			//Y: (CDU) (CENTER 0, DOWN -1, UP    +1) 

			out <<
				"uniform sampler2DRect g_dogTex0;\n"
				"uniform sampler2DRect g_dogTex1;\n"
				"uniform sampler2DRect g_dogTex2;\n"
				//"uniform sampler2DRect g_maskTex;\n"
				"void main ()\n"
				"{\n"
				//"	gl_FragDepth = 1.0f;\n"
				//"	if(any(equal(vec4(texture2DRect(g_maskTex, gl_TexCoord[0].st).r, "
				//"					  texture2DRect(g_maskTex, gl_TexCoord[1].st).r, "
				//"					  texture2DRect(g_maskTex, gl_TexCoord[2].st).r, "
				//"					  texture2DRect(g_maskTex, gl_TexCoord[3].st).r), vec4(0))) || "
				//"	   any(equal(vec4(texture2DRect(g_maskTex, gl_TexCoord[4].st).r, "
				//"					  texture2DRect(g_maskTex, gl_TexCoord[5].st).r, "
				//"					  texture2DRect(g_maskTex, gl_TexCoord[6].st).r, "
				//"					  texture2DRect(g_maskTex, gl_TexCoord[7].st).r), vec4(0))))"
				//"	{\n"
				//"		gl_FragColor = vec4(0);\n"
				//"		return;\n"
				//"	}\n"

				"	vec4 ccc = texture2DRect(g_dogTex1, gl_FragCoord.st);\n"
				"	vec4 clc = texture2DRect(g_dogTex1, gl_TexCoord[0].st);\n"
				"	vec4 crc = texture2DRect(g_dogTex1, gl_TexCoord[1].st);\n"
				"	vec4 ccd = texture2DRect(g_dogTex1, gl_TexCoord[2].st);\n"
				"	vec4 ccu = texture2DRect(g_dogTex1, gl_TexCoord[3].st);\n"
				"	vec4 cld = texture2DRect(g_dogTex1, gl_TexCoord[4].st);\n"
				"	vec4 clu = texture2DRect(g_dogTex1, gl_TexCoord[5].st);\n"
				"	vec4 crd = texture2DRect(g_dogTex1, gl_TexCoord[6].st);\n"
				"	vec4 cru = texture2DRect(g_dogTex1, gl_TexCoord[7].st);\n"
				"	vec4  cc = ccc;\n"
				"	vec4  v1[4], v2[4];\n"
				"	v1[0] = vec4(clc.g, ccc.g, ccd.b, ccc.b);\n"
				"	v1[1] = vec4(ccc.r, crc.r, ccd.a, ccc.a);\n"
				"	v1[2] = vec4(clc.a, ccc.a, ccc.r, ccu.r);\n"
				"	v1[3] = vec4(ccc.b, crc.b, ccc.g, ccu.g);\n"
				"	v2[0] = vec4(cld.a, clc.a, ccd.a, ccc.a);\n"
				"	v2[1] = vec4(ccd.b, ccc.b, crd.b, crc.b);\n"
				"	v2[2] = vec4(clc.g, clu.g, ccc.g, ccu.g);\n"
				"	v2[3] = vec4(ccc.r, ccu.r, crc.r, cru.r);\n";

			//test against 8 neighbours
			//use variable to identify type of extremum
			//1.0 for local maximum and -1.0 for minimum
			out <<
				"	vec4 extremum = vec4(0.0); \n"
				"	#define KEYTEST_STEP0(i) \\\n"
				"	{\\\n"
				"		bvec4 test1 = greaterThan(vec4(cc[i]), max(v1[i], v2[i])), test2 = lessThan(vec4(cc[i]), min(v1[i], v2[i]));\\\n"
				"		extremum[i] = cc[i] > " <<  dogTh * 0.8f << " && all(test1)?1.0: 0.0;\\\n"
				"		extremum[i] = cc[i] < " << -dogTh * 0.8f << " && all(test2)? -1.0: extremum[i];\\\n"
				"	}\n"
				"	REPEAT4(KEYTEST_STEP0);\n"
				"	if(gl_FragCoord.s < 1.0)\n"
				"		extremum.rb = vec2(0.0);\n"
				"	if(gl_FragCoord.s > float(" << octaveSizes[iOctave].v0() - 1.0<< "))\n"
				"		extremum.ga = vec2(0.0);\n"
				"	if(gl_FragCoord.t < 1.0)\n"
				"		extremum.rg = vec2(0.0);\n"
				"	if(gl_FragCoord.t > float(" << octaveSizes[iOctave].v1() - 1.0<< "))\n"
				"		extremum.ba = vec2(0.0);\n"
				"	gl_FragColor = vec4(0);\n"
				"	if(any(notEqual(extremum, vec4(0.0)))) {\n";

			//do edge supression first.. 
			//vector v1 is < (-1, 0), (1, 0), (0,-1), (0, 1)>
			//vector v2 is < (-1,-1), (-1,1), (1,-1), (1, 1)>

			out <<
				"	float fxx[4], fyy[4], fxy[4], fx[4], fy[4];\n"
				"	#define EDGE_SUPPRESION(i) \\\n"
				"	if(extremum[i] != 0.0f)\\\n"
				"	{\\\n"
				"		vec4 D2 = v1[i].xyzw - cc[i];\\\n"
				"		vec2 D4 = v2[i].xw - v2[i].yz;\\\n"
				"		vec2 D5 = 0.5*(v1[i].yw-v1[i].xz); \\\n"
				"		fx[i] = D5.x;	fy[i] = D5.y ;\\\n"
				"		fxx[i] = D2.x + D2.y;\\\n"
				"		fyy[i] = D2.z + D2.w;\\\n"
				"		fxy[i] = 0.25*(D4.x + D4.y);\\\n"
				"		float fxx_plus_fyy = fxx[i] + fyy[i];\\\n"
				"		float score_up = fxx_plus_fyy*fxx_plus_fyy; \\\n"
				"		float score_down = (fxx[i]*fyy[i] - fxy[i]*fxy[i]);\\\n"
				"		if( score_down <= 0.0 || score_up > " << (edgeTh+1)*(edgeTh+1)/edgeTh << " * score_down)extremum[i] = 0.0;\\\n"
				"	}\n"
				"	REPEAT4(EDGE_SUPPRESION);\n"
				"	if(any(notEqual(extremum, vec4(0.0)))) {\n";

			////////////////////////////////////////////////
			// Read 9 pixels of upper/lower level
			out <<
				"	vec4  v4[4], v5[4], v6[4];\n"
				"	ccc = texture2DRect(g_dogTex2, gl_FragCoord.st);\n"
				"	clc = texture2DRect(g_dogTex2, gl_TexCoord[0].st);\n"
				"	crc = texture2DRect(g_dogTex2, gl_TexCoord[1].st);\n"
				"	ccd = texture2DRect(g_dogTex2, gl_TexCoord[2].st);\n"
				"	ccu = texture2DRect(g_dogTex2, gl_TexCoord[3].st);\n"
				"	cld = texture2DRect(g_dogTex2, gl_TexCoord[4].st);\n"
				"	clu = texture2DRect(g_dogTex2, gl_TexCoord[5].st);\n"
				"	crd = texture2DRect(g_dogTex2, gl_TexCoord[6].st);\n"
				"	cru = texture2DRect(g_dogTex2, gl_TexCoord[7].st);\n"
				"	vec4 cu = ccc;\n"
				"	v4[0] = vec4(clc.g, ccc.g, ccd.b, ccc.b);\n"
				"	v4[1] = vec4(ccc.r, crc.r, ccd.a, ccc.a);\n"
				"	v4[2] = vec4(clc.a, ccc.a, ccc.r, ccu.r);\n"
				"	v4[3] = vec4(ccc.b, crc.b, ccc.g, ccu.g);\n"
				"	v6[0] = vec4(cld.a, clc.a, ccd.a, ccc.a);\n"
				"	v6[1] = vec4(ccd.b, ccc.b, crd.b, crc.b);\n"
				"	v6[2] = vec4(clc.g, clu.g, ccc.g, ccu.g);\n"
				"	v6[3] = vec4(ccc.r, ccu.r, crc.r, cru.r);\n"
				<<
				"	#define KEYTEST_STEP1(i)\\\n"
				"	if(extremum[i] == 1.0)\\\n"
				"	{\\\n"
				"		bvec4 test = lessThan(vec4(cc[i]), max(v4[i], v6[i])); \\\n"
				"		if(cc[i] < cu[i] || any(test))extremum[i] = 0.0; \\\n"
				"	}else if(extremum[i] == -1.0)\\\n"
				"	{\\\n"
				"		bvec4 test = greaterThan(vec4(cc[i]), min(v4[i], v6[i])); \\\n"
				"		if(cc[i] > cu[i] || any(test) )extremum[i] = 0.0; \\\n"
				"	}\n"
				"	REPEAT4(KEYTEST_STEP1);\n"
				"	if(any(notEqual(extremum, vec4(0.0)))) { \n"
				<<
				"	ccc = texture2DRect(g_dogTex0, gl_FragCoord.st);\n"
				"	clc = texture2DRect(g_dogTex0, gl_TexCoord[0].st);\n"
				"	crc = texture2DRect(g_dogTex0, gl_TexCoord[1].st);\n"
				"	ccd = texture2DRect(g_dogTex0, gl_TexCoord[2].st);\n"
				"	ccu = texture2DRect(g_dogTex0, gl_TexCoord[3].st);\n"
				"	cld = texture2DRect(g_dogTex0, gl_TexCoord[4].st);\n"
				"	clu = texture2DRect(g_dogTex0, gl_TexCoord[5].st);\n"
				"	crd = texture2DRect(g_dogTex0, gl_TexCoord[6].st);\n"
				"	cru = texture2DRect(g_dogTex0, gl_TexCoord[7].st);\n"
				"	vec4 cd = ccc;\n"
				"	v5[0] = vec4(clc.g, ccc.g, ccd.b, ccc.b);\n"
				"	v5[1] = vec4(ccc.r, crc.r, ccd.a, ccc.a);\n"
				"	v5[2] = vec4(clc.a, ccc.a, ccc.r, ccu.r);\n"
				"	v5[3] = vec4(ccc.b, crc.b, ccc.g, ccu.g);\n"
				"	v6[0] = vec4(cld.a, clc.a, ccd.a, ccc.a);\n"
				"	v6[1] = vec4(ccd.b, ccc.b, crd.b, crc.b);\n"
				"	v6[2] = vec4(clc.g, clu.g, ccc.g, ccu.g);\n"
				"	v6[3] = vec4(ccc.r, ccu.r, crc.r, cru.r);\n"
				<<
				"	#define KEYTEST_STEP2(i)\\\n"
				"	if(extremum[i] == 1.0)\\\n"
				"	{\\\n"
				"		bvec4 test = lessThan(vec4(cc[i]), max(v5[i], v6[i]));\\\n"
				"		if(cc[i] < cd[i] || any(test))extremum[i] = 0.0; \\\n"
				"	}else if(extremum[i] == -1.0)\\\n"
				"	{\\\n"
				"		bvec4 test = greaterThan(vec4(cc[i]), min(v5[i], v6[i]));\\\n"
				"		if(cc[i] > cd[i] || any(test))extremum[i] = 0.0; \\\n"
				"	}\n"
				"	REPEAT4(KEYTEST_STEP2);\n"
				"	float keysum = dot(abs(extremum), vec4(1, 1, 1, 1)) ;\n"
				"	//assume there is only one keypoint in the four. \n"
				"	if(keysum==1.0) {\n";

			//////////////////////////////////////////////////////////////////////
			// Subpixel localization
			out <<
				"	vec3 offset = vec3(0, 0, 0); \n"
				"	#define TESTMOVE_KEYPOINT(idx) \\\n"
				"	if(extremum[idx] != 0.0f) \\\n"
				"	{\\\n"
				"		cu[0] = cu[idx];	cd[0] = cd[idx];	cc[0] = cc[idx];	\\\n"
				"		v4[0] = v4[idx];	v5[0] = v5[idx];						\\\n"
				"		fxy[0] = fxy[idx];	fxx[0] = fxx[idx];	fyy[0] = fyy[idx];	\\\n"
				"		fx[0] = fx[idx];	fy[0] = fy[idx];\\\n"
				"	}\n"
				"	TESTMOVE_KEYPOINT(1);\n"
				"	TESTMOVE_KEYPOINT(2);\n"
				"	TESTMOVE_KEYPOINT(3);\n"
				<<

				"	float fs = 0.5*( cu[0] - cd[0] );				\n"
				"	float fss = cu[0] + cd[0] - cc[0] - cc[0];\n"
				"	float fxs = 0.25 * (v4[0].y + v5[0].x - v4[0].x - v5[0].y);\n"
				"	float fys = 0.25 * (v4[0].w + v5[0].z - v4[0].z - v5[0].w);\n"
				"	vec4 A0, A1, A2 ;			\n"
				"	A0 = vec4(fxx[0], fxy[0], fxs, -fx[0]);	\n"
				"	A1 = vec4(fxy[0], fyy[0], fys, -fy[0]);	\n"
				"	A2 = vec4(fxs, fys, fss, -fs);	\n"
				"	vec3 x3 = abs(vec3(fxx[0], fxy[0], fxs));		\n"
				"	float maxa = max(max(x3.x, x3.y), x3.z);	\n"
				"	if(maxa >= 1e-10 ) \n"
				"	{												\n"
				"		if(x3.y ==maxa )							\n"
				"		{											\n"
				"			vec4 TEMP = A1; A1 = A0; A0 = TEMP;	\n"
				"		}else if( x3.z == maxa )					\n"
				"		{											\n"
				"			vec4 TEMP = A2; A2 = A0; A0 = TEMP;	\n"
				"		}											\n"
				"		A0 /= A0.x;									\n"
				"		A1 -= A1.x * A0;							\n"
				"		A2 -= A2.x * A0;							\n"
				"		vec2 x2 = abs(vec2(A1.y, A2.y));		\n"
				"		if( x2.y > x2.x )							\n"
				"		{											\n"
				"			vec3 TEMP = A2.yzw;					\n"
				"			A2.yzw = A1.yzw;						\n"
				"			A1.yzw = TEMP;							\n"
				"			x2.x = x2.y;							\n"
				"		}											\n"
				"		if(x2.x >= 1e-10) {								\n"
				"			A1.yzw /= A1.y;								\n"
				"			A2.yzw -= A2.y * A1.yzw;					\n"
				"			if(abs(A2.z) >= 1e-10) {\n"
				"				offset.z = A2.w /A2.z;				    \n"
				"				offset.y = A1.w - offset.z*A1.z;			    \n"
				"				offset.x = A0.w - offset.z*A0.z - offset.y*A0.y;	\n"
				"				bool test = (abs(cc[0] + 0.5*dot(vec3(fx[0], fy[0], fs), offset )) > " << dogTh << ") ;\n"
				"				if(!test || any( greaterThan(abs(offset), vec3(1.0)))) extremum = vec4(0.0);\n"
				"			}\n"
				"		}\n"
				"	}\n"
				<<"\n"
				"	gl_FragColor = vec4(dot(extremum, vec4(1.0, 2.0, 3.0, 4.0)), offset);\n"
				"	}}}}\n"
				"}\n" << '\0';

			std::vector<std::string> inpTexNames(/*4*/3), inpParamNames;
			inpTexNames[0] = "g_dogTex0";
			inpTexNames[1] = "g_dogTex1";
			inpTexNames[2] = "g_dogTex2";
			//inpTexNames[3] = "g_maskTex";
			m_programs[iOctave].LoadSource(source, inpTexNames, inpParamNames);
		}
	};
	inline void Run(const ushort &iOctave, const TextureGL4 &dogTex0, const TextureGL4 &dogTex1, const TextureGL4 &dogTex2/*, const TextureGLDepthMask &maskTex*/, 
					const TextureGL4 &extremeTex) const
	{
		const ushort width = m_octaveSizes[iOctave].v0(), height = m_octaveSizes[iOctave].v1();

#if _DEBUG
		assert(dogTex0.GetWidth() >= width && dogTex0.GetHeight() >= height);
		assert(dogTex1.GetWidth() >= width && dogTex1.GetHeight() >= height);
		assert(dogTex2.GetWidth() >= width && dogTex2.GetHeight() >= height);
		//assert(maskTex.GetWidth() >= width && maskTex.GetHeight() >= height);
		assert(extremeTex.GetWidth() >= width && extremeTex.GetHeight() >= height);
#endif

		const ProgramGL &program = m_programs[iOctave];
		program.Activate();
		program.SetInputTextures(dogTex0, dogTex1, dogTex2/*, maskTex*/);
		program.SetOutputTexture(extremeTex);

		const GLint s1 = 0, t1 = 0, s2 = GLint(width), t2 = GLint(height);
		const GLint x1 = 0, y1 = 0, x2 = GLint(width), y2 = GLint(height);
		glBegin(GL_QUADS);
		glMultiTexCoord2i(GL_TEXTURE0, s1-1, t1  );
		glMultiTexCoord2i(GL_TEXTURE1, s1+1, t1  );
		glMultiTexCoord2i(GL_TEXTURE2, s1  , t1-1);
		glMultiTexCoord2i(GL_TEXTURE3, s1  , t1+1);
		glMultiTexCoord2i(GL_TEXTURE4, s1-1, t1-1);
		glMultiTexCoord2i(GL_TEXTURE5, s1-1, t1+1);
		glMultiTexCoord2i(GL_TEXTURE6, s1+1, t1-1);
		glMultiTexCoord2i(GL_TEXTURE7, s1+1, t1+1);
		glVertex2i(x1, y1);
		glMultiTexCoord2i(GL_TEXTURE0, s1-1, t2  );
		glMultiTexCoord2i(GL_TEXTURE1, s1+1, t2  );
		glMultiTexCoord2i(GL_TEXTURE2, s1  , t2-1);
		glMultiTexCoord2i(GL_TEXTURE3, s1  , t2+1);
		glMultiTexCoord2i(GL_TEXTURE4, s1-1, t2-1);
		glMultiTexCoord2i(GL_TEXTURE5, s1-1, t2+1);
		glMultiTexCoord2i(GL_TEXTURE6, s1+1, t2-1);
		glMultiTexCoord2i(GL_TEXTURE7, s1+1, t2+1);
		glVertex2i(x1, y2);
		glMultiTexCoord2i(GL_TEXTURE0, s2-1, t2  );
		glMultiTexCoord2i(GL_TEXTURE1, s2+1, t2  );
		glMultiTexCoord2i(GL_TEXTURE2, s2  , t2-1);
		glMultiTexCoord2i(GL_TEXTURE3, s2  , t2+1);
		glMultiTexCoord2i(GL_TEXTURE4, s2-1, t2-1);
		glMultiTexCoord2i(GL_TEXTURE5, s2-1, t2+1);
		glMultiTexCoord2i(GL_TEXTURE6, s2+1, t2-1);
		glMultiTexCoord2i(GL_TEXTURE7, s2+1, t2+1);
		glVertex2i(x2, y2);
		glMultiTexCoord2i(GL_TEXTURE0, s2-1, t1  );
		glMultiTexCoord2i(GL_TEXTURE1, s2+1, t1  );
		glMultiTexCoord2i(GL_TEXTURE2, s2  , t1-1);
		glMultiTexCoord2i(GL_TEXTURE3, s2  , t1+1);
		glMultiTexCoord2i(GL_TEXTURE4, s2-1, t1-1);
		glMultiTexCoord2i(GL_TEXTURE5, s2-1, t1+1);
		glMultiTexCoord2i(GL_TEXTURE6, s2+1, t1-1);
		glMultiTexCoord2i(GL_TEXTURE7, s2+1, t1+1);
		glVertex2i(x2, y1);
		glEnd();
		glFlush();

		program.Deactivate();
		program.UnbindInputTextures(dogTex0, dogTex1, dogTex2/*, maskTex*/);
	}

private:

	std::vector<LA::Vector2us> m_octaveSizes;
	std::vector<ProgramGL> m_programs;

};

#endif