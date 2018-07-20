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

#ifndef _PROGRAM_GL_MATCH_FEATURE_H_
#define _PROGRAM_GL_MATCH_FEATURE_H_

#include "ProgramGL.h"
#include "SfM/FundamentalMatrix.h"

// const ushort &searchRangeNbr = USHRT_MAX
// const ushort &searchRangeNbrEp = USHRT_MAX,
// const float &epErrTh = FLT_MAX
// const float &descDotTh = FLT_MAX

// Compute pairwise similarity
class ProgramGLMatchFeaturePass1 : public ProgramGL {

  public:
    inline void Initialize(const ushort &ftrTexWidth, const ushort &searchRange = USHRT_MAX, const float &epErrTh = FLT_MAX) {
        m_ftrTexWidth = ftrTexWidth;
        m_epErrTh = epErrTh;
        const ushort ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
        const ushort descTexWidth = (ftrTexWidth << 4);
        const ushort descTexWidthLog = ftrTexWidthLog + 4;
#if _DEBUG
        assert((1 << ftrTexWidthLog) == ftrTexWidth);
#endif

        m_searchRange = searchRange;

        char source[MAX_SOURCE_LENGTH];
        std::ostrstream out(source, MAX_SOURCE_LENGTH);
        /*out <<
            "#pragma optionNV(ifcvt none)\n"
            "#pragma optionNV(unroll all)\n";*/
        if(searchRange != USHRT_MAX || epErrTh != FLT_MAX) {
            out <<
                "uniform sampler2DRect g_ftrTex1;\n"
                "uniform sampler2DRect g_ftrTex2;\n";
        }
        out <<
            "uniform sampler2DRect g_descTex1;\n"
            "uniform sampler2DRect g_descTex2;\n";
        if(epErrTh != FLT_MAX)
            out <<
                "uniform mat3 g_F;\n";
        out <<
            "void main()\n"
            "{\n"
            "	ivec2 iFtr12 = ivec2(gl_FragCoord.yx);\n"
            "	ivec2 xFtr12 = (iFtr12 & " << ftrTexWidth - 1 << ");\n"
            "	ivec2 yFtr12 = (iFtr12 >> " << ftrTexWidthLog << ");\n";
        if(searchRange != USHRT_MAX || epErrTh != FLT_MAX) {
            out <<
                "	vec2 ftr1 = texture2DRect(g_ftrTex1, vec2(xFtr12.r, yFtr12.r) + vec2(0.5)).rg;\n"
                "	vec2 ftr2 = texture2DRect(g_ftrTex2, vec2(xFtr12.g, yFtr12.g) + vec2(0.5)).rg;\n";
        }
        if(searchRange != USHRT_MAX) {
            out <<
                "	if(any(greaterThan(abs(ftr1 - ftr2), vec2(" << searchRange << "))))\n"
                "	{\n"
                "		gl_FragColor.r = -1;\n"
                "		return;\n"
                "	}\n";
        }
        if(epErrTh != FLT_MAX) {
            out <<
                "	vec3 x1 = vec3(ftr1, 1), x2 = vec3(ftr2, 1);\n"
                "	vec3 Fx1 = g_F * x1, x2TF = x2 * g_F;\n"
                "	float x2TFx1 = dot(x2, Fx1);\n"
                "	vec4 tmp = vec4(Fx1.xy, x2TF.xy);\n"
                "	tmp = tmp * tmp;\n"
                "	float d2 = x2TFx1 * x2TFx1;\n"
                "	float epErr2 = d2 / (tmp.x + tmp.y) + d2 / (tmp.z + tmp.w);\n"
                "	if(epErr2 > " << epErrTh *epErrTh << ")\n"
                "	{\n"
                "		gl_FragColor.r = -1;\n"
                "		return;\n"
                "	}\n";
        }
        out <<
            "	vec2 xDesc12 = (xFtr12 << 4) + vec2(0.5);\n"
            "	vec2 yDesc12 = yFtr12 + vec2(0.5);\n"
            "	vec4 dot = vec4(0);\n"
            " for(int i = 0; i < 16; ++i, ++xDesc12)\n"
            "     dot += texture2DRect(g_descTex1, vec2(xDesc12.r, yDesc12.r)) * texture2DRect(g_descTex2, vec2(xDesc12.g, yDesc12.g));\n"
            "	gl_FragColor.r = dot.r + dot.g + dot.b + dot.a;\n"
            "}\n" << '\0';

        //printf("%s\n", source);

        std::vector<std::string> inpTexNames, inpParamNames;
        if(searchRange != USHRT_MAX || epErrTh != FLT_MAX) {
            inpTexNames.push_back("g_ftrTex1");
            inpTexNames.push_back("g_ftrTex2");
        }
        inpTexNames.push_back("g_descTex1");
        inpTexNames.push_back("g_descTex2");
        if(epErrTh != FLT_MAX)
            inpParamNames.push_back("g_F");
        LoadSource(source, inpTexNames, inpParamNames);
    }
    inline const ushort &GetSearchRange() const {
        return m_searchRange;
    }
    inline const float &GetEpipolarErrorThreshold() const {
        return m_epErrTh;
    }
    inline void Run(const TextureGL4 &descTex1, const TextureGL4 &descTex2, const ushort &nFtrs1, const ushort &nFtrs2, const TextureGL2 &matchTex) const {
#if _DEBUG
        assert(descTex1.GetWidth() == (m_ftrTexWidth << 4) && descTex2.GetWidth() == (m_ftrTexWidth << 4));
        assert(nFtrs1 <= matchTex.GetHeight() - 2);
        assert(nFtrs2 <= matchTex.GetWidth());
        assert(m_searchRange == USHRT_MAX&&m_epErrTh == FLT_MAX);
#endif
        Activate();
        SetInputTextures(descTex1, descTex2);
        SetOutputTexture(matchTex);
        DrawQuad(nFtrs2, nFtrs1);
        Deactivate();
        UnbindInputTextures(descTex1, descTex2);
    }
    template<ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    inline void Run(const TextureGL<CHANNELS_NUMBER_1> &ftrTex1, const TextureGL<CHANNELS_NUMBER_2> &ftrTex2,
                    const TextureGL4 &descTex1, const TextureGL4 &descTex2,
                    const ushort &nFtrs1, const ushort &nFtrs2, const TextureGL2 &matchTex) {
        if(m_searchRange == USHRT_MAX) {
            Run(descTex1, descTex2, nFtrs1, nFtrs2, matchTex);
            return;
        }
#if _DEBUG
        assert(ftrTex1.GetWidth() == m_ftrTexWidth && ftrTex2.GetWidth() == m_ftrTexWidth);
        assert(descTex1.GetWidth() == (m_ftrTexWidth << 4) && descTex2.GetWidth() == (m_ftrTexWidth << 4));
        assert(nFtrs1 <= matchTex.GetHeight());
        assert(nFtrs2 <= matchTex.GetWidth());
        assert(m_searchRange != USHRT_MAX);
#endif
        Activate();
        SetInputTextures(ftrTex1, ftrTex2, descTex1, descTex2);
        SetOutputTexture(matchTex);
        DrawQuad(nFtrs2, nFtrs1);
        Deactivate();
        UnbindInputTextures(ftrTex1, ftrTex2, descTex1, descTex2);
    }
    template<ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    inline void Run(const TextureGL<CHANNELS_NUMBER_1> &ftrTex1, const TextureGL<CHANNELS_NUMBER_2> &ftrTex2,
                    const TextureGL4 &descTex1, const TextureGL4 &descTex2,
                    const ushort &nFtrs1, const ushort &nFtrs2, const TextureGL2 &matchTex, const FundamentalMatrix &F) {
#if _DEBUG
        assert(ftrTex1.GetWidth() == m_ftrTexWidth && ftrTex2.GetWidth() == m_ftrTexWidth);
        assert(descTex1.GetWidth() == (m_ftrTexWidth << 4) && descTex2.GetWidth() == (m_ftrTexWidth << 4));
        assert(nFtrs1 <= matchTex.GetHeight() - 2);
        assert(nFtrs2 <= matchTex.GetWidth());
        assert(m_epErrTh != FLT_MAX);
#endif
        Activate();
        SetInputTextures(ftrTex1, ftrTex2, descTex1, descTex2);
        F.Get(m_F);
        SetInputParameter(m_F);
        SetOutputTexture(matchTex);
        DrawQuad(nFtrs2, nFtrs1);
        Deactivate();
        UnbindInputTextures(ftrTex1, ftrTex2, descTex1, descTex2);
    }

  private:

    ushort m_ftrTexWidth, m_searchRange;
    float m_epErrTh;
    LA::Matrix3f m_F;

};

// Filter by epipolar geometry
class ProgramGLMatchFeaturePass2 : public ProgramGL {

  public:

    inline void Initialize(const ushort &ftrTexWidth, const float &epErrTh) {
        m_ftrTexWidth = ftrTexWidth;
        const ushort ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
#if _DEBUG
        assert((1 << ftrTexWidthLog) == ftrTexWidth);
#endif
        char source[MAX_SOURCE_LENGTH];
        std::ostrstream out(source, MAX_SOURCE_LENGTH);
        out <<
            "#pragma optionNV(ifcvt none)\n"
            "#pragma optionNV(unroll all)\n"
            "uniform sampler2DRect g_ftrTex1;\n"
            "uniform sampler2DRect g_ftrTex2;\n"
            "uniform sampler2DRect g_matchTex;\n"
            "uniform mat3 g_F;\n"
            "void main()\n"
            "{\n"
            "	ivec2 iFtr12 = ivec2(gl_FragCoord.yx);\n"
            "	ivec2 xFtr12 = (iFtr12 & " << ftrTexWidth - 1 << ");\n"
            "	ivec2 yFtr12 = (iFtr12 >> " << ftrTexWidthLog << ");\n"
            "	vec2 ftr1 = texture2DRect(g_ftrTex1, vec2(xFtr12.r, yFtr12.r) + vec2(0.5)).rg;\n"
            "	vec2 ftr2 = texture2DRect(g_ftrTex2, vec2(xFtr12.g, yFtr12.g) + vec2(0.5)).rg;\n"
            "	vec3 x1 = vec3(ftr1, 1), x2 = vec3(ftr2, 1);\n"
            "	vec3 Fx1 = g_F * x1, x2TF = x2 * g_F;\n"
            "	float x2TFx1 = dot(x2, Fx1);\n"
            "	vec4 tmp = vec4(Fx1.xy, x2TF.xy);\n"
            "	tmp = tmp * tmp;\n"
            "	float d2 = x2TFx1 * x2TFx1;\n"
            "	float epErr2 = d2 / (tmp.x + tmp.y) + d2 / (tmp.z + tmp.w);\n"
            "	if(epErr2 > " << epErrTh *epErrTh << ")\n"
            "		gl_FragColor.r = -1;\n"
            "	else\n"
            "		gl_FragColor.r = texture2DRect(g_matchTex, gl_FragCoord.xy).r;\n"
            "}\n" << '\0';

        //printf("%s\n", source);

        std::vector<std::string> inpTexNames(3), inpParamNames(1);
        inpTexNames[0] = "g_ftrTex1";
        inpTexNames[1] = "g_ftrTex2";
        inpTexNames[2] = "g_matchTex";
        inpParamNames[0] = "g_F";
        LoadSource(source, inpTexNames, inpParamNames);
    }
    template<ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    inline void Run(const TextureGL<CHANNELS_NUMBER_1> &ftrTex1, const TextureGL<CHANNELS_NUMBER_2> &ftrTex2, const TextureGL2 &matchTex, const ushort &nFtrs1,
                    const ushort &nFtrs2, const FundamentalMatrix &F) {
#if _DEBUG
        assert(ftrTex1.GetWidth() == m_ftrTexWidth && ftrTex2.GetWidth() == m_ftrTexWidth);
        assert(nFtrs1 <= matchTex.GetHeight() - 2);
        assert(nFtrs2 <= matchTex.GetWidth());
#endif
        Activate();
        SetInputTextures(ftrTex1, ftrTex2, matchTex);
        F.Get(m_F);
        SetInputParameter(m_F);
        SetOutputTexture(matchTex);
        DrawQuad(nFtrs2, nFtrs1);
        Deactivate();
        UnbindInputTextures(ftrTex1, ftrTex2, matchTex);
    }

  private:

    ushort m_ftrTexWidth;
    LA::Matrix3f m_F;

};

// Search for most similar feature in second image for each feature in first image
class ProgramGLMatchFeaturePass3 : public ProgramGL {

  public:

    inline void Initialize(const float &nearest1To2RatioTh) {
        char source[MAX_SOURCE_LENGTH];
        std::ostrstream out(source, MAX_SOURCE_LENGTH);
        out <<
            "#pragma optionNV(ifcvt none)\n"
            "#pragma optionNV(unroll all)\n"
            "uniform sampler2DRect g_matchTex;\n"
            "uniform int g_nFtrs2;\n"
            "void main(void)\n"
            "{\n"
            "	float y = gl_FragCoord.x;\n"
            "	float dot, x2 = g_nFtrs2 + 0.5f;\n"
            "	vec3 nearest = vec3(-1);\n"
            "	for(float x = 0.5f; x < x2; ++x)\n"
            "	{\n"
            "		dot = texture2DRect(g_matchTex, vec2(x, y)).r;\n"
            "		nearest = dot > nearest.g ? vec3(x, dot, nearest.g) : vec3(nearest.rg, max(nearest.b, dot));\n"
            "	}\n"
            //2NN Ration
            " gl_FragColor.rg = (nearest.g > 1 - " << nearest1To2RatioTh *nearest1To2RatioTh << " * (1 - nearest.b)) ? nearest.rg : vec2(-1);\n"
            "}\n" << '\0';
        std::vector<std::string> inpTexNames(1), inpParamNames(1);
        inpTexNames[0] = "g_matchTex";
        inpParamNames[0] = "g_nFtrs2";
        LoadSource(source, inpTexNames, inpParamNames);
    }
    inline void Run(const TextureGL2 &matchTex, const ushort &nFtrs1, const ushort &nFtrs2) const {
#if _DEBUG
        AssertTextureOutput(matchTex);
#endif
        Activate();
        SetInputTexture(matchTex);
        SetInputParameter(int(nFtrs2));
        const GLint x1 = 0, x2 = GLint(nFtrs1), y1 = GLint(nFtrs1), y2 = y1 + 1;
        glBegin(GL_QUADS);
        glVertex2i(x1, y1);
        glVertex2i(x1, y2);
        glVertex2i(x2, y2);
        glVertex2i(x2, y1);
        glEnd();
        glFlush();
        Deactivate();
    }
};

class ProgramGLMatchFeaturePass4 : public ProgramGL {

  public:

    inline void Initialize(const float &nearest1To2RatioTh, const float &descDotTh) {
        char source[MAX_SOURCE_LENGTH];
        std::ostrstream out(source, MAX_SOURCE_LENGTH);
        out <<
            "#pragma optionNV(ifcvt none)\n"
            "#pragma optionNV(unroll all)\n"
            "uniform sampler2DRect g_matchTex;\n"
            "uniform int g_nFtrs1;\n"
            "void main(void)\n"
            "{\n"
            "	float x = gl_FragCoord.x;\n"
            "	float dot, y2 = g_nFtrs1 + 0.5f;\n"
            "	vec3 nearest = vec3(-1);\n"
            "	for(float y = 0.5f; y < y2; ++y)\n"
            "	{\n"
            "		dot = texture2DRect(g_matchTex, vec2(x, y)).r;\n"
            "		nearest = dot > nearest.g ? vec3(y, dot, nearest.g) : vec3(nearest.rg, max(nearest.b, dot));\n"
            "	}\n"
            "	nearest = (nearest.g > 1 - " << nearest1To2RatioTh *nearest1To2RatioTh << " * (1 - nearest.b)) ? nearest : vec3(-1);\n"
            "	int iFtr1 = int(nearest.r);\n"
            "	if(iFtr1 != -1 && texture2DRect(g_matchTex, vec2(iFtr1 + 0.5, y2)).r == x && nearest.g >= " << descDotTh << ")\n"
            "		gl_FragColor.rg = vec2(iFtr1, nearest.g);\n"
            "	else\n"
            "		gl_FragColor.rg = vec2(-1);\n"
            "}\n" << '\0';
        std::vector<std::string> inpTexNames(1), inpParamNames(1);
        inpTexNames[0] = "g_matchTex";
        inpParamNames[0] = "g_nFtrs1";
        LoadSource(source, inpTexNames, inpParamNames);
    }
    inline void Run(const TextureGL2 &matchTex, const ushort &nFtrs1, const ushort &nFtrs2) const {
#if _DEBUG
        AssertTextureInput(matchTex);
        AssertTextureOutput(matchTex);
#endif
        Activate();
        SetInputParameter(int(nFtrs1));
        const GLint x1 = 0, x2 = GLint(nFtrs2), y1 = GLint(nFtrs1 + 1), y2 = y1 + 1;
        glBegin(GL_QUADS);
        glVertex2i(x1, y1);
        glVertex2i(x1, y2);
        glVertex2i(x2, y2);
        glVertex2i(x2, y1);
        glEnd();
        glFlush();
        Deactivate();
    }
};

#endif