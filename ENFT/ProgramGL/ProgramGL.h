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

#ifndef _PROGRAM_GL_H_
#define _PROGRAM_GL_H_

#include "TextureGL.h"
#include <strstream>
#include <cvd/glwindow.h>

#define MAX_SOURCE_LENGTH 100000

class ProgramGL {

  public:

    ProgramGL() : m_program(0) {}
    ~ProgramGL() {
        if(m_program != 0)
            glDeleteProgram(m_program);
    }

    inline bool IsInitialized() const {
        return m_program != 0;
    }
    template<ushort CHANNELS_NUMBER>
    inline void SetInputTexture(const TextureGL<CHANNELS_NUMBER> &tex) const {
#if _DEBUG
        assert(m_inpTexs.size() == 1 && m_inpTexs[0] != -1);
#endif
        tex.Bind();
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1>
    inline void SetInputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1) const {
#if _DEBUG
        assert(m_inpTexs.size() == 2 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1);
#endif
        glActiveTexture(GL_TEXTURE1);
        tex1.Bind();
        glUniform1i(m_inpTexs[1], 1);
        glActiveTexture(GL_TEXTURE0);
        tex0.Bind();
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    inline void SetInputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1,
                                 const TextureGL<CHANNELS_NUMBER_2> &tex2) const {
#if _DEBUG
        assert(m_inpTexs.size() == 3 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1 && m_inpTexs[2] != -1);
#endif
        glActiveTexture(GL_TEXTURE1);
        tex1.Bind();
        glUniform1i(m_inpTexs[1], 1);
        glActiveTexture(GL_TEXTURE2);
        tex2.Bind();
        glUniform1i(m_inpTexs[2], 2);
        glActiveTexture(GL_TEXTURE0);
        tex0.Bind();
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2, ushort CHANNELS_NUMBER_3>
    inline void SetInputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1,
                                 const TextureGL<CHANNELS_NUMBER_2> &tex2, const TextureGL<CHANNELS_NUMBER_3> &tex3) const {
#if _DEBUG
        assert(m_inpTexs.size() == 4 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1 && m_inpTexs[2] != -1 && m_inpTexs[3] != -1);
#endif
        glActiveTexture(GL_TEXTURE1);
        tex1.Bind();
        glUniform1i(m_inpTexs[1], 1);
        glActiveTexture(GL_TEXTURE2);
        tex2.Bind();
        glUniform1i(m_inpTexs[2], 2);
        glActiveTexture(GL_TEXTURE3);
        tex3.Bind();
        glUniform1i(m_inpTexs[3], 3);
        glActiveTexture(GL_TEXTURE0);
        tex0.Bind();
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    inline void SetInputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1,
                                 const TextureGL<CHANNELS_NUMBER_2> &tex2, const TextureGLDepthMask &tex3) const {
#if _DEBUG
        assert(m_inpTexs.size() == 4 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1 && m_inpTexs[2] != -1 && m_inpTexs[3] != -1);
#endif
        glActiveTexture(GL_TEXTURE1);
        tex1.Bind();
        glUniform1i(m_inpTexs[1], 1);
        glActiveTexture(GL_TEXTURE2);
        tex2.Bind();
        glUniform1i(m_inpTexs[2], 2);
        glActiveTexture(GL_TEXTURE3);
        tex3.Bind();
        glUniform1i(m_inpTexs[3], 3);
        glActiveTexture(GL_TEXTURE0);
        tex0.Bind();
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2, ushort CHANNELS_NUMBER_3, ushort CHANNELS_NUMBER_4>
    inline void SetInputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1, const TextureGL<CHANNELS_NUMBER_2> &tex2,
                                 const TextureGL<CHANNELS_NUMBER_3> &tex3, const TextureGL<CHANNELS_NUMBER_4> &tex4) const {
#if _DEBUG
        assert(m_inpTexs.size() == 5 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1 && m_inpTexs[2] != -1 && m_inpTexs[3] != -1 && m_inpTexs[4] != -1);
#endif
        glActiveTexture(GL_TEXTURE1);
        tex1.Bind();
        glUniform1i(m_inpTexs[1], 1);
        glActiveTexture(GL_TEXTURE2);
        tex2.Bind();
        glUniform1i(m_inpTexs[2], 2);
        glActiveTexture(GL_TEXTURE3);
        tex3.Bind();
        glUniform1i(m_inpTexs[3], 3);
        glActiveTexture(GL_TEXTURE4);
        tex4.Bind();
        glUniform1i(m_inpTexs[4], 4);
        glActiveTexture(GL_TEXTURE0);
        tex0.Bind();
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1>
    inline void UnbindInputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1) const {
#if _DEBUG
        assert(m_inpTexs.size() == 2 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1);
#endif
        glActiveTexture(GL_TEXTURE1);
        tex1.Unbind();
        glActiveTexture(GL_TEXTURE0);
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    inline void UnbindInputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1,
                                    const TextureGL<CHANNELS_NUMBER_2> &tex2) const {
#if _DEBUG
        assert(m_inpTexs.size() == 3 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1 && m_inpTexs[2] != -1);
#endif
        glActiveTexture(GL_TEXTURE1);
        tex1.Unbind();
        glActiveTexture(GL_TEXTURE2);
        tex2.Unbind();
        glActiveTexture(GL_TEXTURE0);
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    inline void UnbindInputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1,
                                    const TextureGL<CHANNELS_NUMBER_2> &tex2, const TextureGLDepthMask &tex3) const {
#if _DEBUG
        assert(m_inpTexs.size() == 4 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1 && m_inpTexs[2] != -1 && m_inpTexs[3] != -1);
#endif
        glActiveTexture(GL_TEXTURE1);
        tex1.Unbind();
        glActiveTexture(GL_TEXTURE2);
        tex2.Unbind();
        glActiveTexture(GL_TEXTURE3);
        tex3.Unbind();
        glActiveTexture(GL_TEXTURE0);
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2, ushort CHANNELS_NUMBER_3>
    inline void UnbindInputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1,
                                    const TextureGL<CHANNELS_NUMBER_2> &tex2, const TextureGL<CHANNELS_NUMBER_3> &tex3) const {
#if _DEBUG
        assert(m_inpTexs.size() == 4 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1 && m_inpTexs[2] != -1 && m_inpTexs[3] != -1);
#endif
        glActiveTexture(GL_TEXTURE1);
        tex1.Unbind();
        glActiveTexture(GL_TEXTURE2);
        tex2.Unbind();
        glActiveTexture(GL_TEXTURE3);
        tex3.Unbind();
        glActiveTexture(GL_TEXTURE0);
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2, ushort CHANNELS_NUMBER_3, ushort CHANNELS_NUMBER_4>
    inline void UnbindInputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1, const TextureGL<CHANNELS_NUMBER_2> &tex2,
                                    const TextureGL<CHANNELS_NUMBER_3> &tex3, const TextureGL<CHANNELS_NUMBER_4> &tex4) const {
#if _DEBUG
        assert(m_inpTexs.size() == 5 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1 && m_inpTexs[2] != -1 && m_inpTexs[3] != -1 && m_inpTexs[4] != -1);
#endif
        glActiveTexture(GL_TEXTURE1);
        tex1.Unbind();
        glActiveTexture(GL_TEXTURE2);
        tex2.Unbind();
        glActiveTexture(GL_TEXTURE3);
        tex3.Unbind();
        glActiveTexture(GL_TEXTURE4);
        tex4.Unbind();
        glActiveTexture(GL_TEXTURE0);
    }
    template<class TYPE>
    inline void SetInputParameter(const TYPE &param, const GLint &loc) const;
    template<class TYPE>
    inline void SetInputParameter(const TYPE &param) const {
#if _DEBUG
        assert(m_inpParams.size() == 1 && m_inpParams[0] != -1);
#endif
        SetInputParameter(param, m_inpParams[0]);
    }
    template<class TYPE_0, class TYPE_1>
    inline void SetInputParameters(const TYPE_0 &param0, const TYPE_1 &param1) const {
#if _DEBUG
        assert(m_inpParams.size() == 2 && m_inpParams[0] != -1 && m_inpParams[1] != -1);
#endif
        SetInputParameter(param0, m_inpParams[0]);
        SetInputParameter(param1, m_inpParams[1]);
    }
    template<class TYPE_0, class TYPE_1, class TYPE_2>
    inline void SetInputParameters(const TYPE_0 &param0, const TYPE_1 &param1, const TYPE_2 &param2) const {
#if _DEBUG
        assert(m_inpParams.size() == 3 && m_inpParams[0] != -1 && m_inpParams[1] != -1 && m_inpParams[2] != -1);
#endif
        SetInputParameter(param0, m_inpParams[0]);
        SetInputParameter(param1, m_inpParams[1]);
        SetInputParameter(param2, m_inpParams[2]);
    }
    template<class TYPE_0, class TYPE_1, class TYPE_2, class TYPE_3>
    inline void SetInputParameters(const TYPE_0 &param0, const TYPE_1 &param1, const TYPE_2 &param2, const TYPE_3 &param3) const {
#if _DEBUG
        assert(m_inpParams.size() == 4 && m_inpParams[0] != -1 && m_inpParams[1] != -1 && m_inpParams[2] != -1 && m_inpParams[3] != -1);
#endif
        SetInputParameter(param0, m_inpParams[0]);
        SetInputParameter(param1, m_inpParams[1]);
        SetInputParameter(param2, m_inpParams[2]);
        SetInputParameter(param3, m_inpParams[3]);
    }
    template<ushort CHANNELS_NUMBER>
    static inline void SetOutputTexture(const TextureGL<CHANNELS_NUMBER> &tex) {
#if _DEBUG
        AssertFrameBufferBound();
#endif
        GLint attachedTexture;
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        if(attachedTexture != tex.GetTexture())
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tex.GetTexture(), 0);

        GLint drawnAttachment;
        glGetIntegerv(GL_DRAW_BUFFER, &drawnAttachment);
        if(drawnAttachment != GL_COLOR_ATTACHMENT0_EXT)
            glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1>
    static inline void SetOutputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1) {
#if _DEBUG
        AssertFrameBufferBound();
#endif
        GLint attachedTexture;
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        if(attachedTexture != tex0.GetTexture())
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tex0.GetTexture(), 0);
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        if(attachedTexture != tex1.GetTexture())
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, tex1.GetTexture(), 0);

        GLint attachment;
        glGetIntegerv(GL_DRAW_BUFFER1_ARB, &attachment);
        if(attachment != GL_COLOR_ATTACHMENT1_EXT)
            glDrawBuffers(2, g_attachments);
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    static inline void SetOutputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1,
                                         const TextureGL<CHANNELS_NUMBER_2> &tex2) {
#if _DEBUG
        AssertFrameBufferBound();
#endif
        GLint attachedTexture;
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        if(attachedTexture != tex0.GetTexture())
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tex0.GetTexture(), 0);
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        if(attachedTexture != tex1.GetTexture())
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, tex1.GetTexture(), 0);
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT2_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        if(attachedTexture != tex2.GetTexture())
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT2_EXT, GL_TEXTURE_RECTANGLE_ARB, tex2.GetTexture(), 0);

        GLint attachment;
        glGetIntegerv(GL_DRAW_BUFFER2_ARB, &attachment);
        if(attachment != GL_COLOR_ATTACHMENT2_EXT)
            glDrawBuffers(3, g_attachments);
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1>
    static inline void DetachOutputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1) {
#if _DEBUG
        AssertFrameBufferBound();
        GLint attachedTexture;
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        assert(attachedTexture == tex0.GetTexture());
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        assert(attachedTexture == tex1.GetTexture());
#endif
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, 0, 0);
        glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2>
    static inline void DetachOutputTextures(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1,
                                            const TextureGL<CHANNELS_NUMBER_2> &tex2) {
#if _DEBUG
        AssertFrameBufferBound();
        GLint attachedTexture;
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        assert(attachedTexture == tex0.GetTexture());
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        assert(attachedTexture == tex1.GetTexture());
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT2_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        assert(attachedTexture == tex2.GetTexture());
#endif
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT2_EXT, GL_TEXTURE_RECTANGLE_ARB, 0, 0);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, 0, 0);
        glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
    }
    static inline void AttachDepthMaskTexture(const TextureGLDepthMask &tex) {
#if _DEBUG
        AssertFrameBufferBound();
#endif
        GLint attachedTexture;
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        if(attachedTexture != tex.GetTexture())
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_RECTANGLE_ARB, tex.GetTexture(), 0);
    }
    static inline void DetachDepthMaskTexture(const TextureGLDepthMask &tex) {
#if _DEBUG
        AssertFrameBufferBound();
        GLint attachedTexture;
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        assert(attachedTexture == tex.GetTexture());
#endif
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_RECTANGLE_ARB, 0, 0);
    }
    inline void Activate() const {
        glUseProgram(m_program);
    }
    inline void Deactivate() const {
        glUseProgram(0);
    }
    inline void LoadSource(const char *source, const std::vector<std::string> &inpTexNames,
                           const std::vector<std::string> &inpParamNames/*, const bool &printSource = false*/) {
        //if(printSource)
        //  printf("%s\n", source);
        if(m_program != 0)
            glDeleteProgram(m_program);
        m_program = glCreateProgram();
        GLuint shader = glCreateShader(GL_FRAGMENT_SHADER);
#ifdef __LINUX__
        //specify the version of glsl,otherwise the default version is glsl 1.10
        std::string sourceVersion = "#version 130\n";
        sourceVersion = sourceVersion+source;
        const char *tempchar = sourceVersion.c_str();
        glShaderSource(shader, 1, &tempchar, NULL);
#else
        std::string sourceVersion = "#version 130\n";
        sourceVersion = sourceVersion + source;
        const char *tempchar = sourceVersion.c_str();
        glShaderSource(shader, 1, &tempchar, NULL);
        //glShaderSource(shader, 1,&source, NULL);
#endif
        glCompileShader(shader);

        GLint status;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
        if(status != GL_TRUE) {
            GLint len;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
            char *infoLog = new char[len + 1];
            glGetShaderInfoLog(shader, len, &len, infoLog);
            printf("Compile error:\n%s\n", infoLog);
            delete infoLog;
            exit(0);
        }

        glAttachShader(m_program, shader);
        glLinkProgram(m_program);

        glGetProgramiv(m_program, GL_LINK_STATUS, &status);
        if(status != GL_TRUE) {
            GLint len;
            glGetProgramiv(m_program, GL_INFO_LOG_LENGTH, &len);
            char *infoLog = new char[len + 1];
            glGetProgramInfoLog(m_program, len, &len, infoLog);
            printf("Link error: %s\n", infoLog);
            delete infoLog;
            exit(0);
        }

        const ushort nInpTexs = ushort(inpTexNames.size());
        m_inpTexs.resize(nInpTexs);
        for(ushort iInpTex = 0; iInpTex < nInpTexs; ++iInpTex) {
            m_inpTexs[iInpTex] = glGetUniformLocation(m_program, inpTexNames[iInpTex].c_str());
#if _DEBUG
            assert(m_inpTexs[iInpTex] != -1);
#endif
        }
        const ushort nInpParams = ushort(inpParamNames.size());
        m_inpParams.resize(nInpParams);
        for(ushort iInpParam = 0; iInpParam < nInpParams; ++iInpParam) {
            m_inpParams[iInpParam] = glGetUniformLocation(m_program, inpParamNames[iInpParam].c_str());
#if _DEBUG
            assert(m_inpParams[iInpParam] != -1);
#endif
        }
    }
    static inline void DrawQuad(const ushort &drawWidth, const ushort &drawHeight) {
        const GLint x1 = 0, y1 = 0, x2 = GLint(drawWidth), y2 = GLint(drawHeight);
        glBegin(GL_QUADS);
        glVertex2i(x1, y1);
        glVertex2i(x1, y2);
        glVertex2i(x2, y2);
        glVertex2i(x2, y1);
        glEnd();
        glFlush();
    }
    static inline void DrawQuad(const uint nDrawPixels, const ushort &texWidth) {
        glBegin(GL_QUADS);
        const uint drawHeight1 = nDrawPixels / texWidth;
        const uint nDrawPixels1 = drawHeight1 * texWidth;
        if(nDrawPixels1 != 0) {
            const GLint x1 = 0, x2 = GLint(texWidth), y1 = 0, y2 = GLint(drawHeight1);
            glVertex2i(x1, y1);
            glVertex2i(x1, y2);
            glVertex2i(x2, y2);
            glVertex2i(x2, y1);
        }
        const uint nDrawPixels2 = nDrawPixels - nDrawPixels1;
        if(nDrawPixels2 != 0) {
            const GLint x1 = 0, x2 = GLint(nDrawPixels2), y1 = GLint(drawHeight1), y2 = y1 + 1;
            glVertex2i(x1, y1);
            glVertex2i(x1, y2);
            glVertex2i(x2, y2);
            glVertex2i(x2, y1);
        }
        glEnd();
        glFlush();
    }
    static inline void DrawQuad(const uint nDrawPixels, const ushort &texWidth, const ushort &texWidthLog) {
#if _DEBUG
        assert((1 << texWidthLog) == texWidth);
#endif
        glBegin(GL_QUADS);
        const uint drawHeight1 = (nDrawPixels >> texWidthLog);
        const uint nDrawPixels1 = (drawHeight1 << texWidthLog);
        if(nDrawPixels1 != 0) {
            const GLint x1 = 0, x2 = GLint(texWidth), y1 = 0, y2 = GLint(drawHeight1);
            glVertex2i(x1, y1);
            glVertex2i(x1, y2);
            glVertex2i(x2, y2);
            glVertex2i(x2, y1);
        }
        const uint nDrawPixels2 = nDrawPixels - nDrawPixels1;
        if(nDrawPixels2 != 0) {
            const GLint x1 = 0, x2 = GLint(nDrawPixels2), y1 = GLint(drawHeight1), y2 = y1 + 1;
            glVertex2i(x1, y1);
            glVertex2i(x1, y2);
            glVertex2i(x2, y2);
            glVertex2i(x2, y1);
        }
        glEnd();
        glFlush();
    }
//  static inline void DrawQuad(const uint nDrawPixels, const ushort &texWidth, const ushort &texWidthLog)
//  {
//#if _DEBUG
//      assert((1 << texWidthLog) == texWidth);
//#endif
//      const ushort drawHeight = ((nDrawPixels + texWidth - 1) >> texWidthLog);
//      const GLint x1 = 0, y1 = 0, x2 = GLint(texWidth), y2 = GLint(drawHeight);
//      glBegin(GL_QUADS);
//      glVertex2i(x1, y1);
//      glVertex2i(x1, y2);
//      glVertex2i(x2, y2);
//      glVertex2i(x2, y1);
//      glEnd();
//      glFlush();
//  }
    static inline void DrawQuad(const uint nSkipPixels, const uint nDrawPixels, const ushort &texWidth, const ushort &texWidthLog) {
#if _DEBUG
        assert((1 << texWidthLog) == texWidth);
#endif
        glBegin(GL_QUADS);
        uint nRemPixels = nDrawPixels;
        GLint x1, x2, y1, y2 = (nSkipPixels >> texWidthLog);
        if(nSkipPixels != 0) {
            x1 = GLint(nSkipPixels & (texWidth - 1));
            x2 = x1 + nRemPixels;
            if(x2 > texWidth)
                x2 = GLint(texWidth);
            y1 = y2;
            ++y2;
            glVertex2i(x1, y1);
            glVertex2i(x1, y2);
            glVertex2i(x2, y2);
            glVertex2i(x2, y1);
            nRemPixels -= (x2 - x1);
        }
        const uint drawHeight2 = (nRemPixels >> texWidthLog);
        if(drawHeight2 != 0) {
            x1 = 0;
            x2 = GLint(texWidth);
            y1 = y2;
            y2 += GLint(drawHeight2);
            glVertex2i(x1, y1);
            glVertex2i(x1, y2);
            glVertex2i(x2, y2);
            glVertex2i(x2, y1);
            nRemPixels -= (drawHeight2 << texWidthLog);
        }
        if(nRemPixels != 0) {
            x1 = 0;
            x2 = GLint(nRemPixels);
            y1 = y2;
            ++y2;
            glVertex2i(x1, y1);
            glVertex2i(x1, y2);
            glVertex2i(x2, y2);
            glVertex2i(x2, y1);
        }
        glEnd();
        glFlush();
    }

#if _DEBUG
  public:
    template<ushort CHANNELS_NUMBER>
    inline void AssertTextureInput(const TextureGL<CHANNELS_NUMBER> &tex) const {
        assert(m_inpTexs.size() == 1 && m_inpTexs[0] != -1);
        tex.AssertBound();
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1>
    inline void AssertTexturesInput(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1) const {
        assert(m_inpTexs.size() == 2 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1);
        GLint param;
        glActiveTexture(GL_TEXTURE1);
        tex1.AssertBound();
        glGetUniformiv(m_program, m_inpTexs[1], &param);
        assert(param == 1);
        glActiveTexture(GL_TEXTURE0);
        tex0.AssertBound();
    }
    template<ushort CHANNELS_NUMBER_0, ushort CHANNELS_NUMBER_1, ushort CHANNELS_NUMBER_2, ushort CHANNELS_NUMBER_3>
    inline void AssertTexturesInput(const TextureGL<CHANNELS_NUMBER_0> &tex0, const TextureGL<CHANNELS_NUMBER_1> &tex1,
                                    const TextureGL<CHANNELS_NUMBER_2> &tex2, const TextureGL<CHANNELS_NUMBER_3> &tex3) const {
        assert(m_inpTexs.size() == 4 && m_inpTexs[0] != -1 && m_inpTexs[1] != -1 && m_inpTexs[2] != -1 && m_inpTexs[3] != -1);
        GLint param;
        glActiveTexture(GL_TEXTURE1);
        tex1.AssertBound();
        glGetUniformiv(m_program, m_inpTexs[1], &param);
        assert(param == 1);
        glActiveTexture(GL_TEXTURE2);
        tex2.AssertBound();
        glGetUniformiv(m_program, m_inpTexs[2], &param);
        assert(param == 2);
        glActiveTexture(GL_TEXTURE3);
        tex3.AssertBound();
        glGetUniformiv(m_program, m_inpTexs[3], &param);
        assert(param == 3);
        glActiveTexture(GL_TEXTURE0);
        tex0.AssertBound();
    }
    template<ushort CHANNELS_NUMBER>
    inline void AssertTextureOutput(const TextureGL<CHANNELS_NUMBER> &tex) const {
        AssertFrameBufferBound();
        GLint attachedTexture;
        glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
        assert(attachedTexture == tex.GetTexture());

        GLint drawnAttachment;
        glGetIntegerv(GL_DRAW_BUFFER, &drawnAttachment);
        assert(drawnAttachment == GL_COLOR_ATTACHMENT0_EXT);
    }
#endif

  private:

    GLuint m_program;
    std::vector<GLint> m_inpTexs, m_inpParams;

  public:

    static void Initialize(const ushort &width = 0, const ushort &height = 0);
    static void BindFrameBuffer();
    static void UnbindFrameBuffer();
    static inline void FitViewportGL(const ushort &width, const ushort &height) {
        glViewport(0, 0, width, height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluOrtho2D(0, width, 0, height);
    }
    template<ushort CHANNELS_NUMBER>
    static inline void FitViewportGL(const TextureGL<CHANNELS_NUMBER> &tex) {
        FitViewportGL(tex.GetWidth(), tex.GetHeight());
    }
    static void FitViewportWindows(const ushort &width, const ushort &height) {
        glViewport(0, 0, width, height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluOrtho2D(0, width, height, 0);
    }
    static inline CVD::GLWindow *GetGLWindow() {
        return g_pWnd;
    }

#if _DEBUG
    static void AssertAttachmentAvailable(const GLenum &attachment);
    static void AssertFrameBufferBound();
    static void AssertFrameBufferUnbound();
#endif

  protected:

    static const GLenum g_attachments[8];
    static CVD::GLWindow *g_pWnd;
    static GLuint g_frameBuffer;
    static GLint g_drawBuffer, g_readBuffer;
};

#include "ProgramGL.hpp"

#endif