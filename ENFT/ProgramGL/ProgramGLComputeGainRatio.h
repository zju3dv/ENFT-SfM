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

#ifndef _PROGRAM_GL_COMPUTE_GAIN_RATIO_H_
#define _PROGRAM_GL_COMPUTE_GAIN_RATIO_H_

#include "ProgramGL.h"

// Compute rectified feature position
class ProgramGLComputeGainRatioPass1 : public ProgramGL {

  public:

    inline void Initialize(const ushort &ftrTexWidth) {
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
            "void main()\n"
            "{\n"
            "	vec4 x12 = texture2DRect(g_ftrTexSrc, gl_FragCoord.st);\n"
            "	vec3 Hx1 = g_H * vec3(x12.rg, 1);\n"
            "	gl_FragColor = vec4(Hx1.xy / Hx1.z, x12.ba);\n"
            "}\n" << '\0';
        std::vector<std::string> inpTexNames(1), inpParamNames(1);
        inpTexNames[0] = "g_ftrTexSrc";
        inpParamNames[0] = "g_H";
        LoadSource(source, inpTexNames, inpParamNames);
    }
    template<ushort CHANNELS_NUMBER_SRC>
    inline void Run(const TextureGL<CHANNELS_NUMBER_SRC> &ftrTexSrc, const TextureGL4 &ftrTexDst, const ushort &nFtrs, const LA::Matrix3f &H) const {
#if _DEBUG
        assert(ftrTexSrc.GetWidth() == m_ftrTexWidth && ftrTexDst.GetWidth() == m_ftrTexWidth);
#endif
        Activate();
        SetInputTexture(ftrTexSrc);
        SetInputParameter(H);
        SetOutputTexture(ftrTexDst);
        DrawQuad(uint(nFtrs), m_ftrTexWidth, m_ftrTexWidthLog);
        Deactivate();
    }

  private:

    ushort m_ftrTexWidth, m_ftrTexWidthLog;

};

// Construct floating window of I1 and I2
class ProgramGLComputeGainRatioPass2 : public ProgramGL {

  public:

    inline void Initialize(const ushort &fltWinTexWidth, const ushort &winSz) {
        m_fltWinTexWidth = fltWinTexWidth;
        m_nPixelsPerWin = winSz * winSz;

        char source[MAX_SOURCE_LENGTH];
        std::ostrstream out(source, MAX_SOURCE_LENGTH);
        out <<
            "uniform sampler2DRect g_ftrTex12;\n"
            "uniform sampler2DRect g_imgTex1;\n"
            "uniform sampler2DRect g_imgTex2;\n"
            "uniform mat3 g_Hinv;\n"
            "void main()\n"
            "{\n"
            "	vec2 ftrCoord = gl_FragCoord.st;\n"
            "	float tmp = ftrCoord.x * " << 1.0f / m_nPixelsPerWin << ";\n"
            "	ftrCoord.x = floor(tmp) + 0.5;\n"

            "	tmp = fract(tmp) * " << winSz << ";\n"
            "	vec2 winCoord = floor(vec2(fract(tmp) * " << winSz << ", tmp));\n"

            "	vec2 y = winCoord - " << (winSz - 1) * 0.5 << ";\n"
            "	vec4 x = texture2DRect(g_ftrTex12, ftrCoord) + vec4(y, y);\n"
            "	vec2 xr1 = x.rg;\n"
            "	vec3 Hinvxr1 = g_Hinv * vec3(xr1, 1);\n"
            "	x.rg = Hinvxr1.xy / Hinvxr1.z;\n"

            "	vec4 x00 = floor(x) + 0.5;\n"
            "	vec4 x11 = x00 + 1;\n"
            "	vec4 alpha = fract(x);\n"
            "	vec2 w11 = alpha.rb * alpha.ga;\n"
            "	vec2 w10 = alpha.rb - w11;\n"
            "	vec2 w01 = alpha.ga - w11;\n"
            "	vec2 w00 = 1 - alpha.ga - w10;\n"
            "	gl_FragColor.r = dot(vec4(texture2DRect(g_imgTex1, x00.rg).r, \n"
            "							  texture2DRect(g_imgTex1, vec2(x00.r, x11.g)).r, \n"
            "							  texture2DRect(g_imgTex1, vec2(x11.r, x00.g)).r, \n"
            "							  texture2DRect(g_imgTex1, x11.rg).r), vec4(w00.r, w01.r, w10.r, w11.r));\n"
            "	gl_FragColor.g = dot(vec4(texture2DRect(g_imgTex2, x00.ba).r, \n"
            "							  texture2DRect(g_imgTex2, vec2(x00.b, x11.a)).r, \n"
            "							  texture2DRect(g_imgTex2, vec2(x11.b, x00.a)).r, \n"
            "							  texture2DRect(g_imgTex2, x11.ba).r), vec4(w00.g, w01.g, w10.g, w11.g));\n"
            "}\n" << '\0';

        std::vector<std::string> inpTexNames(3), inpParamNames(1);
        inpTexNames[0] = "g_ftrTex12";
        inpTexNames[1] = "g_imgTex1";
        inpTexNames[2] = "g_imgTex2";
        inpParamNames[0] = "g_Hinv";
        LoadSource(source, inpTexNames, inpParamNames);
    }
    inline void Run(const TextureGL4 &ftrTex12, const TextureGL1 &imgTex1, const TextureGL1 &imgTex2, const TextureGL2 &ITex12, const ushort &nFtrs,
                    const LA::Matrix3f &Hinv) const {
#if _DEBUG
        assert(ftrTex12.GetWidth() * m_nPixelsPerWin == m_fltWinTexWidth && ITex12.GetWidth() == m_fltWinTexWidth);
#endif
        Activate();
        SetInputTextures(ftrTex12, imgTex1, imgTex2);
        SetInputParameter(Hinv);
        SetOutputTexture(ITex12);
        DrawQuad(uint(m_nPixelsPerWin) * uint(nFtrs), m_fltWinTexWidth);
        UnbindInputTextures(ftrTex12, imgTex1, imgTex2);
        Deactivate();
    }

  private:

    ushort m_fltWinTexWidth, m_nPixelsPerWin;

};

// Compute gain ratio for each floating window
class ProgramGLComputeGainRatioPass3 : public ProgramGL {

  public:

    inline void Initialize(const ushort &ftrTexWidth, const ushort &winSz) {
        m_ftrTexWidth = ftrTexWidth;
        m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
        m_nPixelsPerWin = winSz * winSz;
#if _DEBUG
        assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
#endif
        char source[MAX_SOURCE_LENGTH];
        std::ostrstream out(source, MAX_SOURCE_LENGTH);
        out <<
            "#pragma optionNV(ifcvt none)\n"
            "#pragma optionNV(unroll all)\n"

            "uniform sampler2DRect g_ITex12;\n"

            "void main()\n"
            "{\n"
            "	vec2 dstCoord = gl_FragCoord.st;\n"
            "	float xSrc1 = floor(dstCoord.x) * " << m_nPixelsPerWin << " + 0.5;\n"
            "	float xSrc2 = xSrc1 + " << m_nPixelsPerWin << ";\n"
            "	vec2 sum12 = vec2(0);\n"
            "	vec2 srcCoord = dstCoord;\n"
            "	for(srcCoord.x = xSrc1; srcCoord.x < xSrc2; ++srcCoord.x)\n"
            "		sum12 += texture2DRect(g_ITex12, srcCoord).rg;\n"
            "	gl_FragColor.r = sum12.g / sum12.r;\n"
            "}\n" << '\0';

        std::vector<std::string> inpTexNames(1), inpParamNames;
        inpTexNames[0] = "g_ITex12";
        LoadSource(source, inpTexNames, inpParamNames);
    }

    inline void Run(const TextureGL2 &ITex12, const TextureGL1 &gainRatioTex, const ushort &nFtrs) const {
#if _DEBUG
        assert(ITex12.GetWidth() == m_ftrTexWidth * m_nPixelsPerWin && gainRatioTex.GetWidth() == m_ftrTexWidth);
#endif
        Activate();
        SetInputTexture(ITex12);
        SetOutputTexture(gainRatioTex);
        DrawQuad(uint(nFtrs), m_ftrTexWidth, m_ftrTexWidthLog);
        Deactivate();
    }

  private:

    ushort m_ftrTexWidth, m_ftrTexWidthLog, m_nPixelsPerWin;
    ProgramGL m_programs[2];
};

class ProgramGLComputeGainRatio {

  public:

    inline void Initialize(const ushort &ftrTexWidth, const ushort &fltWinTexWidth, const ushort &winSz) {
        m_pass1.Initialize(ftrTexWidth);
        m_pass2.Initialize(fltWinTexWidth, winSz);
        m_pass3.Initialize(ftrTexWidth, winSz);
    }

    inline void Run(const TextureGL4 &ftrTex12, const TextureGL1 &imgTex1, const TextureGL1 &imgTex2, const TextureGL4 &ftrTex12Rect, const TextureGL2 &ITex12,
                    const TextureGL1 &gainRatioTex, const ushort nFtrs, const Homography &H) {
        H.Get(m_M);
        m_pass1.Run(ftrTex12, ftrTex12Rect, nFtrs, m_M);
        H.Invert(m_Hinv);
        m_Hinv.Get(m_M);
        m_pass2.Run(ftrTex12Rect, imgTex1, imgTex2, ITex12, nFtrs, m_M);
        m_pass3.Run(ITex12, gainRatioTex, nFtrs);
    }

  private:

    ProgramGLComputeGainRatioPass1 m_pass1;
    ProgramGLComputeGainRatioPass2 m_pass2;
    ProgramGLComputeGainRatioPass3 m_pass3;
    Homography m_Hinv;
    LA::Matrix3f m_M;
};

#endif