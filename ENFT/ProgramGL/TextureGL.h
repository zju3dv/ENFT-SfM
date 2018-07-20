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

#ifndef _TEXTURE_GL_H_
#define _TEXTURE_GL_H_

#include "Utility/Table.h"
#include <GL/glew.h>

#if _DEBUG
#include <assert.h>
#endif

template<ushort CHANNELS_NUMBER>
class TextureGL {

  public:

    TextureGL() {
        memset(this, 0, sizeof(TextureGL<CHANNELS_NUMBER>));
    }
    ~TextureGL() {
        Delete();
    }
    inline void Generate() {
#if _DEBUG
        assert(m_texture == 0);
#endif
        glGenTextures(1, &m_texture);
        Bind();
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER_ARB);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER_ARB);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    }
    inline void Generate(const ushort &width, const ushort &height) {
        if(m_texture == 0)
            Generate();
        else
            Bind();
        Resize(width, height);
    }
    inline void Bind() const {
#if _DEBUG
        assert(m_texture != 0);
#endif
        GLint boundTexture;
        glGetIntegerv(GL_TEXTURE_BINDING_RECTANGLE_ARB, &boundTexture);
        if(boundTexture != m_texture)
            glBindTexture(GL_TEXTURE_RECTANGLE_ARB, m_texture);
    }
    inline void Unbind() const {
#if _DEBUG
        assert(m_texture != 0);
        GLint boundTexture;
        glGetIntegerv(GL_TEXTURE_BINDING_RECTANGLE_ARB, &boundTexture);
        assert(boundTexture == m_texture);
#endif
        glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
    }
    inline void SetFilterType(const GLint &filterType) const {
#if _DEBUG
        AssertBound();
#endif
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, filterType);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, filterType);
    }
    inline void Delete() {
        if(m_texture != 0) {
            GLint boundTexture;
            glGetIntegerv(GL_TEXTURE_BINDING_RECTANGLE_ARB, &boundTexture);
            if(boundTexture == m_texture)
                glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
            glDeleteTextures(1, &m_texture);
            m_texture = 0;
        }
    }

    inline void Resize(const ushort &width, const ushort &height);

    template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels) const;
    template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels, const GLenum &components) const;
    template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels, const ushort &width, const ushort &height) const;
    template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels, const ushort &width, const ushort &height, const GLenum &components) const;
    template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels, const ushort &x, const ushort &y, const ushort &width, const ushort &height) const;
    template<typename TYPE> inline void UploadFromCPU(const TYPE *pixels, const ushort &x, const ushort &y, const ushort &width, const ushort &height,
            const GLenum &components) const;
    template<typename TYPE> inline void DownloadToCPU(TYPE *pixels) const;
    template<typename TYPE> inline void DownloadToCPU(TYPE *pixels, const GLenum &components) const;
    template<typename TYPE> inline void DownloadToCPU(TYPE *pixels, const ushort &width, const ushort &height) const;
    template<typename TYPE> inline void DownloadToCPU(TYPE *pixels, const ushort &width, const ushort &height, const GLenum &components) const;
    template<typename TYPE> inline void DownloadToCPU(TYPE *pixels, const ushort &x, const ushort &y, const ushort &width, const ushort &height) const;
    template<typename TYPE> inline void DownloadToCPU(TYPE *pixels, const ushort &x, const ushort &y, const ushort &width, const ushort &height,
            const GLenum &components) const;

    inline GLenum GetAttachment() const {
        GLint attachedTexture;
        GLenum i = 0, attachment;
        for(i = 0, attachment = GL_COLOR_ATTACHMENT0_EXT; i < 8; ++i, ++attachment) {
            glGetFramebufferAttachmentParameterivEXT(GL_FRAMEBUFFER_EXT, attachment, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &attachedTexture);
            if(attachedTexture == m_texture)
                return attachment;
        }
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, m_texture, 0);
        return GL_COLOR_ATTACHMENT0_EXT;
    }

    inline const ushort &GetWidth() const {
        return m_width;
    }
    inline const ushort &GetHeight() const {
        return m_height;
    }
    inline const uint &GetPixelsNumber() const {
        return m_nPixels;
    }
    inline const uint &GetTotalSize() const {
        return m_totalSize;
    }
    inline const GLuint &GetTexture() const {
        return m_texture;
    }
    inline void ComputeTotalSize();

    // inverse : inverse width and height order
    inline void Display_CV(std::string window_name, bool inverse = false) const;
    inline void Save(const char *fileName) const;
    inline void Save(const char *fileName, const ushort &width, const ushort &height) const;
    inline void Save(const char *fileName, const ushort &x, const ushort &y, const ushort &width, const ushort &height) const;
    inline void Save(const char *fileName, const GLenum &components) const;
    inline void SaveB(const char *fileName) const;
    inline void SaveB(FILE *fp) const;
    inline void SaveB(const char *fileName, const uint &nPixels) const;
    inline void SaveB(FILE *fp, const uint &nPixels) const;
    //inline void SaveB(const char *fileName, const GLenum &components) const;
    inline void LoadB(const char *fileName) const;
    inline void LoadB(FILE *fp) const;
    inline void LoadB(const char *fileName, const uint &nPixels) const;
    inline void LoadB(FILE *fp, const uint &nPixels) const;

    //inline void SaveImage(const char *imgFile, CVD::Image<ubyte

#if _DEBUG
    template<typename TYPE> inline void DownloadToCPUWithoutFBO(TYPE *pixels) const;
    template<typename TYPE> inline void DownloadToCPUWithoutFBO(TYPE *pixels, const GLenum &components) const;
    inline void AssertBound() const {
        assert(m_texture != 0);
        GLint boundTexture;
        glGetIntegerv(GL_TEXTURE_BINDING_RECTANGLE_ARB, &boundTexture);
        assert(boundTexture == m_texture);
    }
    inline void AssertComponentsCompitable(const GLenum &components) const {
        assert(components == GL_RED || components == GL_GREEN || components == GL_BLUE || components == GL_ALPHA
               || CHANNELS_NUMBER >= 2 && components == GL_RG
               || CHANNELS_NUMBER >= 3 && components == GL_RGB
               || CHANNELS_NUMBER == 4 && components == GL_RGBA);
    }
#endif

  protected:

    GLuint m_texture;
    ushort m_width, m_height;
    uint m_nPixels, m_totalSize;

};

typedef TextureGL<1> TextureGL1;
typedef TextureGL<2> TextureGL2;
typedef TextureGL<3> TextureGL3;
typedef TextureGL<4> TextureGL4;
template<> inline void TextureGL<1>::ComputeTotalSize() {
    m_totalSize = m_nPixels;
}
template<> inline void TextureGL<2>::ComputeTotalSize() {
    m_totalSize = (m_nPixels << 1);
}
template<> inline void TextureGL<3>::ComputeTotalSize() {
    m_totalSize = (m_nPixels << 1) + m_nPixels;
}
template<> inline void TextureGL<4>::ComputeTotalSize() {
    m_totalSize = (m_nPixels << 2);
}

class TextureGLDepthMask : private TextureGL1 {

  public:

    inline void Generate(const Table<ubyte, ushort> &maskTable) {
        if(m_texture == 0)
            TextureGL1::Generate();
        else
            Bind();
        m_width = maskTable.GetColsNumber();
        m_height = maskTable.GetRowsNumber();
        m_nPixels = uint(m_width) * uint(m_height);
        ComputeTotalSize();
        glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_DEPTH_COMPONENT, m_width, m_height, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, maskTable[0]);
    }
    inline void Bind() const {
        TextureGL1::Bind();
    }
    inline void Unbind() const {
        TextureGL1::Unbind();
    }
    inline const ushort &GetWidth() const {
        return TextureGL1::GetWidth();
    }
    inline const ushort &GetHeight() const {
        return TextureGL1::GetHeight();
    }
    inline const GLuint &GetTexture() const {
        return TextureGL1::GetTexture();
    }
    inline const uint &GetPixelsNumber() const {
        return TextureGL1::GetPixelsNumber();
    }
    template<typename TYPE> inline void DownloadToCPU(TYPE *pixels) const;
};

#include "TextureGL.hpp"

#endif