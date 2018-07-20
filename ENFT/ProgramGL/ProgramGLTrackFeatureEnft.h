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

#ifndef _PROGRAM_GL_TRACK_FEATURE_ENFT_H_
#define _PROGRAM_GL_TRACK_FEATURE_ENFT_H_

#include "ProgramGL.h"

// Compute rectified feature position, epipolar line and error
// Only for first iteration
class ProgramGLTrackFeatureEnftPass1 : public ProgramGL
{

public:

	inline void Initialize(const ushort &ftrTexWidth)
	{
		m_ftrTexWidth = ftrTexWidth;
		m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
#endif

		char source[MAX_SOURCE_LENGTH];
		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out <<
			"uniform sampler2DRect g_ftrTexSrc;\n"
			"uniform mat3 g_H;\n"
			"uniform mat3 g_FHinv;\n"
			"void main()\n"
			"{\n"
			"	vec2 x1 = texture2DRect(g_ftrTexSrc, gl_FragCoord.st).rg;\n"
			"	vec3 Hx1 = g_H * vec3(x1, 1);\n"
			"	x1 = Hx1.xy / Hx1.z;\n"
			"	vec3 Fx1 = g_FHinv * Hx1;\n"
			"	vec3 l = Fx1 / length(Fx1.xy);\n"
			"	gl_FragData[0] = vec4(x1, x1);\n"
			"	gl_FragData[1] = vec4(l, 0);\n"
			"	gl_FragData[2] = vec4(dot(l, vec3(x1, 1)), vec3(0));\n"
			"}\n" << '\0';
		std::vector<std::string> inpTexNames(1), inpParamNames(2);
		inpTexNames[0] = "g_ftrTexSrc";
		inpParamNames[0] = "g_H";
		inpParamNames[1] = "g_FHinv";
		LoadSource(source, inpTexNames, inpParamNames);
	}
	template<ushort CHANNELS_NUMBER_SRC>
	inline void Run(const TextureGL<CHANNELS_NUMBER_SRC> &ftrTexSrc, const TextureGL4 &ftrTexDst, const TextureGL4 &epLineTex, const TextureGL4 &errTex, const ushort &nFtrs, 
					const LA::Matrix3f &H, const LA::Matrix3f &FHinv) const
	{
#if _DEBUG
		assert(ftrTexSrc.GetWidth() == m_ftrTexWidth && ftrTexDst.GetWidth() == m_ftrTexWidth && epLineTex.GetWidth() == m_ftrTexWidth && errTex.GetWidth() == m_ftrTexWidth);
#endif
		Activate();
		SetInputTexture(ftrTexSrc);
		SetInputParameters(H, FHinv);
		SetOutputTextures(ftrTexDst, epLineTex, errTex);
		DrawQuad(uint(nFtrs), m_ftrTexWidth, m_ftrTexWidthLog);
		DetachOutputTextures(ftrTexDst, epLineTex, errTex);
		Deactivate();
	}

private:

	ushort m_ftrTexWidth, m_ftrTexWidthLog;

};

// Construct floating window of rect(I1)
// Only for first iteration
class ProgramGLTrackFeatureEnftPass2 : public ProgramGL
{

public:

	inline void Initialize(const ushort &fltWinTexWidth, const ushort &winSz)
	{
		m_fltWinTexWidth = fltWinTexWidth;
		m_nPixelsPerWin = winSz * winSz;

		char source[MAX_SOURCE_LENGTH];
		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out <<
			"uniform sampler2DRect g_ftrTex;\n"
			"uniform sampler2DRect g_imgTex1;\n"
			"uniform mat3 g_Hinv;\n"
			"uniform float g_gainRatio;\n"
			"void main()\n"
			"{\n"
			"	vec2 ftrCoord = gl_FragCoord.st;\n"
			"	float tmp = ftrCoord.x * " << 1.0f / m_nPixelsPerWin << ";\n"
			"	ftrCoord.x = floor(tmp) + 0.5;\n"

			"	tmp = fract(tmp) * " << winSz << ";\n"
			"	vec2 winCoord = floor(vec2(fract(tmp) * " << winSz << ", tmp));\n"

			//"	gl_FragColor = vec4(ftrCoord.x, 0, winCoord.yx);\n"

			"	vec2 xr = texture2DRect(g_ftrTex, ftrCoord).rg + winCoord - " << (winSz - 1) * 0.5 << ";\n"
			"	vec3 Hinvxr = g_Hinv * vec3(xr, 1);\n"
			"	vec2 xo = Hinvxr.xy / Hinvxr.z;\n"
			"	vec2 x00 = floor(xo) + 0.5;\n"
			"	vec2 x11 = x00 + 1;\n"
			"	vec2 alpha = fract(xo);\n"
			"	float w11 = alpha.x * alpha.y;\n"
			"	float w10 = alpha.x - w11;\n"
			"	float w01 = alpha.y - w11;\n"
			"	float w00 = 1 - alpha.y - w10;\n"
			"	gl_FragColor.r = dot(vec4(texture2DRect(g_imgTex1, x00).r, \n"
			"							  texture2DRect(g_imgTex1, vec2(x00.x, x11.y)).r, \n"
			"							  texture2DRect(g_imgTex1, vec2(x11.x, x00.y)).r, \n"
			"							  texture2DRect(g_imgTex1, x11).r), vec4(w00, w01, w10, w11)) * g_gainRatio;\n"
			"}\n" << '\0';

		std::vector<std::string> inpTexNames(2), inpParamNames(2);
		inpTexNames[0] = "g_ftrTex";
		inpTexNames[1] = "g_imgTex1";
		inpParamNames[0] = "g_Hinv";
		inpParamNames[1] = "g_gainRatio";
		LoadSource(source, inpTexNames, inpParamNames);
	};
	inline void Run(const TextureGL4 &ftrTex, const TextureGL1 &imgTex1, const TextureGL1 &ITex1, const ushort &nFtrs, const LA::Matrix3f &Hinv, const float &gainRatio) const
	{
#if _DEBUG
		assert(ftrTex.GetWidth() * m_nPixelsPerWin == m_fltWinTexWidth && ITex1.GetWidth() == m_fltWinTexWidth);
#endif
		Activate();
		SetInputTextures(ftrTex, imgTex1);
		SetInputParameters(Hinv, gainRatio);
		SetOutputTexture(ITex1);
		DrawQuad(uint(m_nPixelsPerWin) * uint(nFtrs), m_fltWinTexWidth);
		UnbindInputTextures(ftrTex, imgTex1);
		Deactivate();
	}

private:

	ushort m_fltWinTexWidth, m_nPixelsPerWin;

};

// Construct floating window of [g*g^T, d*gT]
// For each iteration
class ProgramGLTrackFeatureEnftPass3 : public ProgramGL
{

public:

	inline void Initialize(const ushort &fltWinTexWidth, const ushort &winSz)
	{
		m_fltWinTexWidth = fltWinTexWidth;
		m_nPixelsPerWin = winSz * winSz;

		char source[MAX_SOURCE_LENGTH];
		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out <<
			"uniform sampler2DRect g_ftrTex;\n"
			"uniform sampler2DRect g_ITex1;\n"
			"uniform sampler2DRect g_imgGradTex2;\n"
			"void main()\n"
			"{\n"
			"	vec2 dstCoord = gl_FragCoord.st;\n"
			"	float tmp = dstCoord.x * " << 1.0f / m_nPixelsPerWin << ";\n"
			"	vec2 ftrCoord = vec2(floor(tmp) + 0.5, dstCoord.y);\n"

			"	tmp = fract(tmp) * " << winSz << ";\n"
			"	vec2 winCoord = floor(vec2(fract(tmp) * " << winSz << ", tmp));\n"

			"	vec2 x = texture2DRect(g_ftrTex, ftrCoord).ba + winCoord - " << (winSz - 1) * 0.5 << ";\n"
			"	vec2 x00 = floor(x) + 0.5;\n"
			"	vec2 x11 = x00 + 1;\n"
			"	vec2 alpha = fract(x);\n"
			"	float w11 = alpha.x * alpha.y;\n"
			"	float w10 = alpha.x - w11;\n"
			"	float w01 = alpha.y - w11;\n"
			"	float w00 = 1 - alpha.y - w10;\n"
			"	vec4 p2 = texture2DRect(g_imgGradTex2, x00) * w00\n"
			"			+ texture2DRect(g_imgGradTex2, vec2(x00.x, x11.y)) * w01\n"
			"			+ texture2DRect(g_imgGradTex2, vec2(x11.x, x00.y)) * w10\n"
			"			+ texture2DRect(g_imgGradTex2, x11) * w11;\n"
			"	float d = texture2DRect(g_ITex1, dstCoord).r - p2.r;\n"
			"	vec2 g = p2.gb;\n"
			"	gl_FragData[0].rgb = vec3(g.x * g.x, g.x * g.y, g.y * g.y);\n"
			"	gl_FragData[1].rgb = vec3(d * g, d);\n"
			"}\n" << '\0';

		//printf("%s\n", source);
		std::vector<std::string> inpTexNames(3), inpParamNames;
		inpTexNames[0] = "g_ftrTex";
		inpTexNames[1] = "g_ITex1";
		inpTexNames[2] = "g_imgGradTex2";
		LoadSource(source, inpTexNames, inpParamNames);
	};
	inline void Run(const TextureGL4 &ftrTex, const TextureGL1 &ITex1, const TextureGL4 &imgGradTex2, const TextureGL4 &GTex, const TextureGL4 &eTex, 
					const ushort &nFtrs) const
	{
#if _DEBUG
		assert(ftrTex.GetWidth() * m_nPixelsPerWin == m_fltWinTexWidth && ITex1.GetWidth() == m_fltWinTexWidth);
		assert(GTex.GetWidth() == m_fltWinTexWidth && eTex.GetWidth() == m_fltWinTexWidth);
#endif
		Activate();
		SetInputTextures(ftrTex, ITex1, imgGradTex2);
		SetOutputTextures(GTex, eTex);
		DrawQuad(uint(m_nPixelsPerWin) * uint(nFtrs), m_fltWinTexWidth);
		UnbindInputTextures(ftrTex, ITex1, imgGradTex2);
		DetachOutputTextures(GTex, eTex);
		Deactivate();
	}

private:

	ushort m_fltWinTexWidth, m_nPixelsPerWin;

};

// Update feature location and error
// For each iteration
class ProgramGLTrackFeatureEnftPass4
{

public:

	// GTex: [gxx, gxy, gyy]
	// eTex: [dgx, dgy, d]
	// ftrTex2: [x2, y2]
	// epLineTex: [A, B, C]. A^2 + B^2 = 1. The distance of a point(x, y) to the line is |Ax + By + C|
	// errTex: [epErr, homoErr, SAD]. epErr = Ax1 + By2 + C (SIGNED!!!), homoErr = [x1 - x2, y1 - y2], SAD = Sigma_|d|
	inline void Initialize(const ushort &ftrTexWidth, const ushort &winSz, const ushort &width, const ushort &height, const float &lambdaEp, const float &lambdaHomo, 
						   const float &deltaTh, const float &SADTh, const float &errThEp, const float &errThHomo)
	{
		m_ftrTexWidth = ftrTexWidth;
		m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
		m_nPixelsPerWin = winSz * winSz;
#if _DEBUG
		assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
#endif

		for(ushort i = 0; i < 2; ++i)
		{
			char source[MAX_SOURCE_LENGTH];
			std::ostrstream out(source, MAX_SOURCE_LENGTH);
			out <<
				"#pragma optionNV(ifcvt none)\n"
				"#pragma optionNV(unroll all)\n"

				"uniform sampler2DRect g_ftrTex;\n"
				"uniform sampler2DRect g_GTex;\n"
				"uniform sampler2DRect g_eTex;\n"
				"uniform sampler2DRect g_epLineTex;\n"
				"uniform sampler2DRect g_errTex;\n"

				"void main()\n"
				"{\n"
				"	vec2 dstCoord = gl_FragCoord.st;\n"
				"	float xSrc1 = floor(dstCoord.x) * " << m_nPixelsPerWin << " + 0.5;\n"
				"	float xSrc2 = xSrc1 + " << m_nPixelsPerWin << ";\n"
				"	vec3 G = vec3(0);\n"
				"	vec2 e = vec2(0);\n";

			if(i == 1)
				out <<
				"	float SAD = 0;\n"
				"	vec3 ed;\n";
			out <<
				"	vec2 srcCoord = dstCoord;\n"
				"	for(srcCoord.x = xSrc1; srcCoord.x < xSrc2; ++srcCoord.x)\n"
				"	{\n"
				"		G += texture2DRect(g_GTex, srcCoord).rgb;\n";
			if(i == 0)
				out <<
				"		e += texture2DRect(g_eTex, srcCoord).rg;\n";
			else
				out <<
				"		ed = texture2DRect(g_eTex, srcCoord).rgb;\n"
				"		e += ed.rg;\n"
				"		SAD += abs(ed.b);\n";
			out <<
				"	}\n"
				"	vec3 l = texture2DRect(g_epLineTex, dstCoord).rgb;\n"
				"	G += vec3(l.x * l.x, l.x * l.y, l.y * l.y) * " << lambdaEp << " + vec3(" << lambdaHomo << ", 0, " << lambdaHomo << ");\n"
				"	vec4 err = texture2DRect(g_errTex, dstCoord);\n"
				"	e += l.rg * (-err.r * " << lambdaEp << ") + err.gb * " << lambdaHomo << ";\n"

				"	float det = G.r * G.b - G.g * G.g;\n"
				"	vec2 delta = det == 0 ? vec2(0) : vec2(G.b * e.x - G.g * e.y, G.r * e.y - G.g * e.x) / det;\n"
				"	vec4 ftr = texture2DRect(g_ftrTex, dstCoord);\n"
				"	vec2 x1 = ftr.rg;\n"
				"	vec2 x2 = ftr.ba + delta;\n";
			if(i == 0)
				out <<
				"	ftr.ba = x2;\n"
				"	err = vec4(dot(l, vec3(x2, 1)), x1 - x2, 0);\n";
			else
				out <<
				"	err = vec4(dot(l, vec3(x2, 1)), x1 - x2, SAD);\n"
				"	if(any(lessThan(x2, vec2(" << winSz << "))) || any(greaterThan(x2, vec2(" << width - winSz - 1 << ", " << height - winSz - 1 << ")))\n"
				"	|| any(greaterThan(delta, vec2(" << deltaTh << ")))\n"
				"	|| any(greaterThan(abs(err), vec4(" << errThEp << ", vec2(" << errThHomo << "), " << SADTh << "))))\n"
				//"		ftr = vec4(vec3(-1), " << FLT_MAX << ");\n"
				"		ftr = vec4(vec2(-1), " << FLT_MAX << ", -1);\n"
				"	else\n"
				//"		ftr = vec4(x2, 0, SAD);\n";
				"		ftr = vec4(x2, SAD, 0);\n";
			out <<
				"	gl_FragData[0] = ftr;\n"
				"	gl_FragData[1] = err;\n"
				"}\n" << '\0';

			//if(i == 1)
			//printf("%s\n", source);

			std::vector<std::string> inpTexNames(5), inpParamNames;
			inpTexNames[0] = "g_ftrTex";
			inpTexNames[1] = "g_GTex";
			inpTexNames[2] = "g_eTex";
			inpTexNames[3] = "g_epLineTex";
			inpTexNames[4] = "g_errTex";
			m_programs[i].LoadSource(source, inpTexNames, inpParamNames);
		}
	}

	inline void Run(const TextureGL4 &ftrTex, const TextureGL4 &GTex, const TextureGL4 &eTex, const TextureGL4 &epLineTex, const TextureGL4 &errTex, 
					const ushort &nFtrs, const bool &final) const
	{
#if _DEBUG
		assert(ftrTex.GetWidth() == epLineTex.GetWidth() && ftrTex.GetWidth() == errTex.GetWidth() && ftrTex.GetWidth() == m_ftrTexWidth);
		assert(GTex.GetWidth() == m_ftrTexWidth * m_nPixelsPerWin && eTex.GetWidth() == m_ftrTexWidth * m_nPixelsPerWin);
#endif
		const ProgramGL &program = m_programs[final];
		program.Activate();
		program.SetInputTextures(ftrTex, GTex, eTex, epLineTex, errTex);
		program.SetOutputTextures(ftrTex, errTex);
		program.DrawQuad(uint(nFtrs), m_ftrTexWidth, m_ftrTexWidthLog);
		program.UnbindInputTextures(ftrTex, GTex, eTex, epLineTex, errTex);
		program.DetachOutputTextures(ftrTex, errTex);
		program.Deactivate();
	}

private:

	ushort m_ftrTexWidth, m_ftrTexWidthLog, m_nPixelsPerWin;
	ProgramGL m_programs[2];
};

// Find the best plane
class ProgramGLTrackFeatureEnftPass5 : public ProgramGL
{

public:

	inline void Initialize(const ushort &ftrTexWidth)
	{
		m_ftrTexWidth = ftrTexWidth;
		m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));

#if _DEBUG
		assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
#endif

		char source[MAX_SOURCE_LENGTH];
		std::ostrstream out(source, MAX_SOURCE_LENGTH);
		out <<
			"#pragma optionNV(ifcvt none)\n"
			"uniform sampler2DRect g_ftrTex;\n"
			"uniform sampler2DRect g_ftrTexNew;\n"
			"uniform float g_iNewPlane;\n"
			"void main()\n"
			"{\n"
			"	vec4 ftr = texture2DRect(g_ftrTex, gl_FragCoord.st);\n"
			"	vec4 ftrNew = texture2DRect(g_ftrTexNew, gl_FragCoord.st);\n"
			//"	ftr = ftrNew.a < ftr.a ? vec4(ftrNew.rg, g_iNewPlane, ftrNew.a) : ftr;\n"
			"	ftr = ftrNew.b < ftr.b ? vec4(ftrNew.rg, ftrNew.b, g_iNewPlane) : ftr;\n"
			"	gl_FragColor = ftr;\n"
			"}\n" << '\0';
		std::vector<std::string> inpTexNames(2), inpParamNames(1);
		inpTexNames[0] = "g_ftrTex";
		inpTexNames[1] = "g_ftrTexNew";
		inpParamNames[0] = "g_iNewPlane";
		LoadSource(source, inpTexNames, inpParamNames);
	}
	inline void Run(const TextureGL4 &ftrTex, const TextureGL4 &ftrTexNew, const ushort &nFtrs, const ushort &iPlane)
	{
#if _DEBUG
		assert(ftrTex.GetWidth() == m_ftrTexWidth && ftrTexNew.GetWidth() == m_ftrTexWidth);
#endif
		Activate();
		SetInputTextures(ftrTex, ftrTexNew);
		SetInputParameter(float(iPlane));
		SetOutputTexture(ftrTex);
		DrawQuad(uint(nFtrs), m_ftrTexWidth, m_ftrTexWidthLog);
		UnbindInputTextures(ftrTex, ftrTexNew);
		Deactivate();
	}

private:

	ushort m_ftrTexWidth, m_ftrTexWidthLog;

};

class ProgramGLTrackFeatureEnft
{

public:

	inline void Initialize(const ushort &ftrTexWidth, const ushort &fltWinTexWidth, const ushort &winSz, const ushort &nIters, const float &lambdaEp, 
						   const float &lambdaHomo, const ushort &width, const ushort &height, const float &deltaTh, const float &SADTh, const float &errThEp, 
						   const float &errThHomo)
	{
		m_nIters = nIters;
		m_pass1.Initialize(ftrTexWidth);
		m_pass2.Initialize(fltWinTexWidth, winSz);
		m_pass3.Initialize(fltWinTexWidth, winSz);
		m_pass4.Initialize(ftrTexWidth, winSz, width, height, lambdaEp, lambdaHomo, deltaTh, SADTh, errThEp, errThHomo);
		m_pass5.Initialize(ftrTexWidth);
	}
	template<ushort CHANNELS_NUMBER_SRC>
	inline void Run(const TextureGL<CHANNELS_NUMBER_SRC> &ftrTexSrc, const TextureGL1 &imgTex1, const TextureGL4 &imgGradTex2, const TextureGL4 &ftrTexDst, 
					const TextureGL4 &tmpFtrTex, const TextureGL4 &epLineTex, const TextureGL4 &errTex, const TextureGL1 &ITex1, const TextureGL4 &GTex, 
					const TextureGL4 &eTex, const ushort nFtrs, const FundamentalMatrix &F, const AlignedVector<Homography> &Hs, const float &gainRatio)
	{
#if _DEBUG
		assert(Hs.Size() != 0);
#endif
		Hs[0].Invert(m_Hinv);
		ENFT_SSE::__m128 work;
		LA::AB(F, m_Hinv, m_FHinv, work);
		Hs[0].Get(m_M1);
		m_FHinv.Get(m_M2);
		m_pass1.Run(ftrTexSrc, ftrTexDst, epLineTex, errTex, nFtrs, m_M1, m_M2);

		m_Hinv.Get(m_M1);
		m_pass2.Run(ftrTexDst, imgTex1, ITex1, nFtrs, m_M1, gainRatio);
		for(ushort iter = 1; iter < m_nIters; ++iter)
		{
			m_pass3.Run(ftrTexDst, ITex1, imgGradTex2, GTex, eTex, nFtrs);
			m_pass4.Run(ftrTexDst, GTex, eTex, epLineTex, errTex, nFtrs, false);
		}
		m_pass3.Run(ftrTexDst, ITex1, imgGradTex2, GTex, eTex, nFtrs);
		m_pass4.Run(ftrTexDst, GTex, eTex, epLineTex, errTex, nFtrs, true);

//#if _DEBUG
//		CVD::Image<LA::Vector4f> img;
//		img.resize(CVD::ImageRef(int(ftrTexDst.GetWidth()), int(ftrTexDst.GetHeight())));
//		ftrTexDst.DownloadToCPU((float *) img.data());
//		LA::Vector4f *chk = img.data() + 293;
//		errTex.DownloadToCPU((float *) img.data());
//#endif

		const ushort nPlanes = ushort(Hs.Size());
		for(ushort iPlane = 1; iPlane < nPlanes; ++iPlane)
		{
			Hs[iPlane].Invert(m_Hinv);
			LA::AB(F, m_Hinv, m_FHinv, work);
			Hs[iPlane].Get(m_M1);
			m_FHinv.Get(m_M2);
			m_pass1.Run(ftrTexSrc, tmpFtrTex, epLineTex, errTex, nFtrs, m_M1, m_M2);

			m_Hinv.Get(m_M1);
			m_pass2.Run(tmpFtrTex, imgTex1, ITex1, nFtrs, m_M1, gainRatio);
			for(ushort iter = 1; iter < m_nIters; ++iter)
			{
				m_pass3.Run(tmpFtrTex, ITex1, imgGradTex2, GTex, eTex, nFtrs);
				m_pass4.Run(tmpFtrTex, GTex, eTex, epLineTex, errTex, nFtrs, false);
			}
			m_pass3.Run(tmpFtrTex, ITex1, imgGradTex2, GTex, eTex, nFtrs);
			m_pass4.Run(tmpFtrTex, GTex, eTex, epLineTex, errTex, nFtrs, true);
			m_pass5.Run(ftrTexDst, tmpFtrTex, nFtrs, iPlane);
//#if _DEBUG
//			tmpFtrTex.DownloadToCPU((float *) img.data());
//			errTex.DownloadToCPU((float *) img.data());
//#endif
		}
	}

#if _DEBUG
	const ProgramGLTrackFeatureEnftPass1& GetProgramPass1() const { return m_pass1; }
	const ProgramGLTrackFeatureEnftPass2& GetProgramPass2() const { return m_pass2; }
	const ProgramGLTrackFeatureEnftPass3& GetProgramPass3() const { return m_pass3; }
	const ProgramGLTrackFeatureEnftPass4& GetProgramPass4() const { return m_pass4; }
#endif

private:

	ProgramGLTrackFeatureEnftPass1 m_pass1;
	ProgramGLTrackFeatureEnftPass2 m_pass2;
	ProgramGLTrackFeatureEnftPass3 m_pass3;
	ProgramGLTrackFeatureEnftPass4 m_pass4;
	ProgramGLTrackFeatureEnftPass5 m_pass5;
	ushort m_nIters;
	Homography m_Hinv;
	FundamentalMatrix m_FHinv;
	LA::Matrix3f m_M1, m_M2;

};

#endif