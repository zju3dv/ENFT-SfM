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

#ifndef _PROGRAM_GL_FEATURE_LOCALIZE_H_
#define _PROGRAM_GL_FEATURE_LOCALIZE_H_

#include "ProgramGL.h"

class ProgramGLFeatureInitialize : public ProgramGL {

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
            //"#pragma optionNV(ifcvt none)\n"
            "uniform vec4 g_sum;\n"
            "void main(void)\n"
            "{\n"
            "	ivec2 coord = ivec2(gl_FragCoord.st);\n"
            "	float idx = float((coord.y << " << m_ftrTexWidthLog << ") + coord.x);\n"
            "	if(idx < g_sum.r)\n"
            "		gl_FragColor.rgb = vec3(idx, 0.5, 0.5);\n"
            "	else if(idx < g_sum.g)\n"
            "		gl_FragColor.rgb = vec3(idx - g_sum.r, 1.5, 0.5);\n"
            "	else if(idx < g_sum.b)\n"
            "		gl_FragColor.rgb = vec3(idx - g_sum.g, 0.5, 1.5);\n"
            "	else\n"
            "		gl_FragColor.rgb = vec3(idx - g_sum.b, 1.5, 1.5);\n"

            //" gl_FragColor.a = idx;\n"

            "}\n" << '\0';

        std::vector<std::string> inpTexNames, inpParamNames(1);
        inpParamNames[0] = "g_sum";
        LoadSource(source, inpTexNames, inpParamNames);
    }
    inline void Run(const LA::Vector4f &hist, const TextureGL4 &ftrTex, const ushort &nFtrs) const {
#if _DEBUG
        assert(ftrTex.GetWidth() == m_ftrTexWidth);
#endif

        Activate();
        LA::Vector4f sum;
        sum.v0() = hist.v0();
        sum.v1() = hist.v1() + sum.v0();
        sum.v2() = hist.v2() + sum.v1();
        SetInputParameter(sum);
        SetOutputTexture(ftrTex);
        DrawQuad(uint(nFtrs), m_ftrTexWidth, m_ftrTexWidthLog);
        Deactivate();
    }

  private:

    ushort m_ftrTexWidth, m_ftrTexWidthLog;

};

class ProgramGLFeatureLocalize : public ProgramGL {

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
            "#pragma optionNV(ifcvt none)\n"
            "uniform sampler2DRect g_ftrTex;\n"
            "uniform sampler2DRect g_histTex;\n"
            "void main(void)\n"
            "{\n"
            "	vec4 ftr = texture2DRect(g_ftrTex, gl_FragCoord.st);\n"
            "	float idx = ftr.r;\n"
            "	vec2 loc = ftr.gb;\n"
            "	vec4 hist = texture2DRect(g_histTex, loc);\n"
            "	float sum01 = hist.r + hist.g;\n"
            "	float sum012 = sum01 + hist.b;\n"
            "	if(idx < hist.r)\n"
            "		loc = loc + loc + vec2(-0.5, -0.5);\n"
            "	else if(idx < sum01)\n"
            "	{\n"
            "		idx -= hist.r;\n"
            "		loc = loc + loc + vec2(0.5, -0.5);\n"
            "	}\n"
            "	else if(idx < sum012)\n"
            "	{\n"
            "		idx -= sum01;\n"
            "		loc = loc + loc + vec2(-0.5, 0.5);\n"
            "	}\n"
            "	else\n"
            "	{\n"
            "		idx -= sum012;\n"
            "		loc = loc + loc + vec2(0.5, 0.5);\n"
            "	}\n"
            "	gl_FragColor.rgb = vec3(idx, loc);\n"

            //" if(idx >= 0)\n"
            //"     gl_FragColor = hist;\n"

            "}\n" << '\0';

        std::vector<std::string> inpTexNames(2), inpParamNames;
        inpTexNames[0] = "g_ftrTex";
        inpTexNames[1] = "g_histTex";
        LoadSource(source, inpTexNames, inpParamNames);
    }
    inline void Run(const TextureGL4 &histTex, const TextureGL4 &ftrTex, const ushort &nFtrs) const {
#if _DEBUG
        assert(ftrTex.GetWidth() == m_ftrTexWidth);
        AssertTextureOutput(ftrTex);
#endif

        Activate();
        SetInputTextures(ftrTex, histTex);
        //SetOutputTexture(ftrTex);
        DrawQuad(uint(nFtrs), m_ftrTexWidth, m_ftrTexWidthLog);
        Deactivate();
        UnbindInputTextures(ftrTex, histTex);
    }

  private:

    ushort m_ftrTexWidth, m_ftrTexWidthLog;
};

class ProgramGLFeatureFinalize {

  public:

    inline void Initialize(const ushort &ftrTexWidth, const std::vector<float> &sigmas, const float &sigmak) {
        m_ftrTexWidth = ftrTexWidth;
        m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
#if _DEBUG
        assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
#endif

        const ushort nLevelsDoG = ushort(sigmas.size());
        m_programs.resize(nLevelsDoG);

        for(ushort iLevelDoG = 0; iLevelDoG < nLevelsDoG; ++iLevelDoG) {
            char source[MAX_SOURCE_LENGTH];
            std::ostrstream out(source, MAX_SOURCE_LENGTH);
            out <<
                "#pragma optionNV(ifcvt none)\n"
                "uniform sampler2DRect g_ftrTex;\n"
                "uniform sampler2DRect g_histTex;\n"
                "uniform sampler2DRect g_dogTex;\n"
                "uniform sampler2DRect g_extremeTex;\n"
                "void main(void)\n"
                "{\n"
                "	vec4 ftr = texture2DRect(g_ftrTex, gl_FragCoord.st);\n"
                "	float idx = ftr.r;\n"
                "	vec2 loc = ftr.gb;\n"
                "	vec4 hist = texture2DRect(g_histTex, loc);\n"
                "	if(hist.r == 1)\n"
                "		loc = loc + loc + vec2(-0.5, -0.5);\n"
                "	else if(hist.g == 1)\n"
                "		loc = loc + loc + vec2(0.5, -0.5);\n"
                "	else if(hist.b == 1)\n"
                "		loc = loc + loc + vec2(-0.5, 0.5);\n"
                "	else if(hist.a == 1)\n"
                "		loc = loc + loc + vec2(0.5, 0.5);\n"
                "	vec4 extremum = texture2DRect(g_extremeTex, loc);\n"
                "	int c = int(abs(extremum.r)) - 1;\n"
                "	gl_FragColor = vec4(loc + loc + vec2(float(c & 1), float(c >> 1)) - vec2(0.5) + extremum.gb, " << sigmas[iLevelDoG] << " * pow(" << sigmak <<
                ", extremum.a), abs(texture2DRect(g_dogTex, loc)[c]));\n"
                "}\n" << '\0';

            std::vector<std::string> inpTexNames(4), inpParamNames;
            inpTexNames[0] = "g_ftrTex";
            inpTexNames[1] = "g_histTex";
            inpTexNames[2] = "g_dogTex";
            inpTexNames[3] = "g_extremeTex";
            m_programs[iLevelDoG].LoadSource(source, inpTexNames, inpParamNames);
        }
    }
    inline void Run(const ushort &iLevelDoG, const TextureGL4 &histTex, const TextureGL4 &dogTex, const TextureGL4 &extremeTex, const TextureGL4 &ftrTex,
                    const ushort &nFtrs) const {
#if _DEBUG
        assert(ftrTex.GetWidth() == m_ftrTexWidth);
        m_programs[iLevelDoG].AssertTextureOutput(ftrTex);
#endif

        const ProgramGL &program = m_programs[iLevelDoG];
        program.Activate();
        program.SetInputTextures(ftrTex, histTex, dogTex, extremeTex);
        //program.SetOutputTexture(ftrTex);
        program.DrawQuad(uint(nFtrs), m_ftrTexWidth, m_ftrTexWidthLog);
        program.Deactivate();
        program.UnbindInputTextures(ftrTex, histTex, dogTex, extremeTex);
    }

  private:

    ushort m_ftrTexWidth, m_ftrTexWidthLog;
    std::vector<ProgramGL> m_programs;

};

#endif