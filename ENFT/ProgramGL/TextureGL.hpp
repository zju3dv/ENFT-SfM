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
#include <string>
#include <array>

template<ushort CHANNELS_NUMBER>  inline const GLenum GetFormat();
template<>  inline const GLenum GetFormat<1>() {
    return GL_LUMINANCE;
}
template<>  inline const GLenum GetFormat<2>() {
    return GL_RG;
}
template<>  inline const GLenum GetFormat<3>() {
    return GL_RGB;
}
template<>  inline const GLenum GetFormat<4>() {
    return GL_RGBA;
}

template<ushort CHANNELS_NUMBER>  inline const GLint GetInternalFormat();
template<>  inline const GLint GetInternalFormat<1>() {
    return GL_LUMINANCE32F_ARB;
}
template<>  inline const GLint GetInternalFormat<2>() {
    return GL_RG32F;
}
template<>  inline const GLint GetInternalFormat<3>() {
    return GL_RGB32F_ARB;
}
template<>  inline const GLint GetInternalFormat<4>() {
    return GL_RGBA32F_ARB;
}

template<typename TYPE>  inline const GLenum GetType();
template<>  inline const GLenum GetType<ubyte >() {
    return GL_UNSIGNED_BYTE;
}
template<>  inline const GLenum GetType<ushort>() {
    return GL_UNSIGNED_SHORT;
}
template<>  inline const GLenum GetType<float >() {
    return GL_FLOAT;
}

template<ushort CHANNELS_NUMBER> inline void TextureGL<CHANNELS_NUMBER>::Resize(const ushort &width, const ushort &height) {
    if(m_width == width && m_height == height)
        return;
    m_width = width;
    m_height = height;
    m_nPixels = uint(width) * uint(height);
    ComputeTotalSize();
//#if _DEBUG
//  AssertBound();
//#endif
    Bind();
    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GetInternalFormat<CHANNELS_NUMBER>(), m_width, m_height, 0, GetFormat<CHANNELS_NUMBER>(), GL_FLOAT, NULL);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::UploadFromCPU(const TYPE *pixels) const {
#if _DEBUG
    AssertBound();
#endif
    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GetInternalFormat<CHANNELS_NUMBER>(), m_width, m_height, 0, GetFormat<CHANNELS_NUMBER>(), GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::UploadFromCPU(const TYPE *pixels, const GLenum &components) const {
#if _DEBUG
    AssertBound();
    AssertComponentsCompitable(components);
#endif
    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GetInternalFormat<CHANNELS_NUMBER>(), m_width, m_height, 0, components, GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::UploadFromCPU(const TYPE *pixels, const ushort &width, const ushort &height) const {
#if _DEBUG
    AssertBound();
#endif
    glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 0, 0, width, height, GetFormat<CHANNELS_NUMBER>(), GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::UploadFromCPU(const TYPE *pixels, const ushort &width, const ushort &height, const GLenum &components) const {
#if _DEBUG
    AssertBound();
    AssertComponentsCompitable(components);
#endif
    glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 0, 0, width, height, components, GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::UploadFromCPU(const TYPE *pixels, const ushort &x, const ushort &y, const ushort &width, const ushort &height) const {
#if _DEBUG
    AssertBound();
#endif
    glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, x, y, width, height, GetFormat<CHANNELS_NUMBER>(), GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::UploadFromCPU(const TYPE *pixels, const ushort &x, const ushort &y, const ushort &width, const ushort &height,
        const GLenum &components) const {
#if _DEBUG
    AssertBound();
    AssertComponentsCompitable(components);
#endif
    glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, x, y, width, height, components, GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::DownloadToCPU(TYPE *pixels) const {
    glReadBuffer(GetAttachment());
    glReadPixels(0, 0, m_width, m_height, GetFormat<CHANNELS_NUMBER>(), GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::DownloadToCPU(TYPE *pixels, const GLenum &components) const {
#if _DEBUG
    AssertComponentsCompitable(components);
#endif
    glReadBuffer(GetAttachment());
    glReadPixels(0, 0, m_width, m_height, components, GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::DownloadToCPU(TYPE *pixels, const ushort &width, const ushort &height) const {
    glReadBuffer(GetAttachment());
    glReadPixels(0, 0, width, height, GetFormat<CHANNELS_NUMBER>(), GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::DownloadToCPU(TYPE *pixels, const ushort &width, const ushort &height, const GLenum &components) const {
#if _DEBUG
    AssertComponentsCompitable(components);
#endif
    glReadBuffer(GetAttachment());
    glReadPixels(0, 0, width, height, components, GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::DownloadToCPU(TYPE *pixels, const ushort &x, const ushort &y, const ushort &width, const ushort &height) const {
    glReadBuffer(GetAttachment());
    glReadPixels(x, y, width, height, GetFormat<CHANNELS_NUMBER>(), GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::DownloadToCPU(TYPE *pixels, const ushort &x, const ushort &y, const ushort &width, const ushort &height,
        const GLenum &components) const {
#if _DEBUG
    AssertComponentsCompitable(components);
#endif
    glReadBuffer(GetAttachment());
    glReadPixels(x, y, width, height, components, GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER>  inline void SavePixel(FILE *const &fp, const ushort &x, const ushort &y, const float *const &p);
template<>  inline void SavePixel<1>(FILE *const &fp, const ushort &x, const ushort &y, const float *const &p) {
    fprintf(fp, "(%d, %d): %f\n", y, x, p[0]);
}
template<>  inline void SavePixel<2>(FILE *const &fp, const ushort &x, const ushort &y, const float *const &p) {
    fprintf(fp, "(%d, %d): %f %f\n", y, x, p[0], p[1]);
}
template<>  inline void SavePixel<3>(FILE *const &fp, const ushort &x, const ushort &y, const float *const &p) {
    fprintf(fp, "(%d, %d): %f %f %f\n", y, x, p[0], p[1], p[2]);
}
template<>  inline void SavePixel<4>(FILE *const &fp, const ushort &x, const ushort &y, const float *const &p) {
    fprintf(fp, "(%d, %d): %f %f %f %f\n", y, x, p[0], p[1], p[2], p[3]);
}

inline ushort GetChannelsNumber(const GLenum &components) {
    switch(components) {
        case GL_RED:
        case GL_GREEN:
        case GL_BLUE:
        case GL_ALPHA:
        case GL_LUMINANCE:
            return 1;
        case GL_RG:
            return 2;
        case GL_RGB:
            return 3;
        case GL_RGBA:
            return 4;
        default:
            return SHRT_MAX;
    }
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::Save(const char *fileName) const {
    std::vector<float> pixels(GetTotalSize());
    DownloadToCPU(pixels.data());

    FILE *fp = fopen(fileName, "w");
    fprintf(fp, "%d %d\n", m_width, m_height);
    float *p = pixels.data();
    for(ushort y = 0; y < m_height; ++y) {
        for(ushort x = 0; x < m_width; ++x, p += CHANNELS_NUMBER)
            SavePixel<CHANNELS_NUMBER>(fp, x, y, p);
        fprintf(fp, "\n");
    }
    fclose(fp);
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::Save(const char *fileName, const ushort &width, const ushort &height) const {
    std::vector<float> pixels(GetTotalSize());
    DownloadToCPU(pixels);

    FILE *fp = fopen(fileName, "w");
    fprintf(fp, "%d %d\n", width, height);
    const ushort stride = m_width * CHANNELS_NUMBER;
    float *row = pixels.data();
    for(ushort y = 0; y < height; ++y, row += stride) {
        float *p = row;
        for(ushort x = 0; x < width; ++x, p += CHANNELS_NUMBER)
            SavePixel<CHANNELS_NUMBER>(fp, x, y, p);
        fprintf(fp, "\n");
    }
    fclose(fp);
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::Save(const char *fileName, const ushort &x, const ushort &y, const ushort &width, const ushort &height) const {
    std::vector<float> pixels(GetTotalSize());
    DownloadToCPU(pixels);

    FILE *fp = fopen(fileName, "w");
    fprintf(fp, "%d %d\n", width, height);
    const ushort stride = m_width * CHANNELS_NUMBER;
    float *row = pixels.data() + (y * m_width + x) * CHANNELS_NUMBER;
    for(ushort y = 0; y < height; ++y, row += stride) {
        float *p = row;
        for(ushort x = 0; x < width; ++x, p += CHANNELS_NUMBER)
            SavePixel<CHANNELS_NUMBER>(fp, x, y, p);
        fprintf(fp, "\n");
    }
    fclose(fp);
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::Save(const char *fileName, const GLenum &components) const {
    uint nChannels = uint(GetChannelsNumber(components));
#if _DEBUG
    assert(nChannels <= CHANNELS_NUMBER);
#endif

    std::vector<float> pixels(m_nPixels * nChannels);
    DownloadToCPU(pixels.data(), components);

    FILE *fp = fopen(fileName, "w");
    float *p = pixels.data();
    switch(nChannels) {
        case 1:
            for(ushort y = 0; y < m_height; ++y) {
                for(ushort x = 0; x < m_width; ++x, p += nChannels)
                    SavePixel<1>(fp, x, y, p);
                fprintf(fp, "\n");
            }
            break;
        case 2:
            for(ushort y = 0; y < m_height; ++y) {
                for(ushort x = 0; x < m_width; ++x, p += nChannels)
                    SavePixel<2>(fp, x, y, p);
                fprintf(fp, "\n");
            }
            break;
        case 3:
            for(ushort y = 0; y < m_height; ++y) {
                for(ushort x = 0; x < m_width; ++x, p += nChannels)
                    SavePixel<3>(fp, x, y, p);
                fprintf(fp, "\n");
            }
            break;
        case 4:
            for(ushort y = 0; y < m_height; ++y) {
                for(ushort x = 0; x < m_width; ++x, p += nChannels)
                    SavePixel<4>(fp, x, y, p);
                fprintf(fp, "\n");
            }
            break;
    }
    fclose(fp);
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::SaveB(const char *fileName) const {
    FILE *fp = fopen(fileName, "wb");
    SaveB(fp);
    fclose(fp);
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::SaveB(FILE *fp) const {
    std::vector<float> pixels(GetTotalSize());
    DownloadToCPU(pixels.data());
    fwrite(pixels.data(), 4, GetTotalSize(), fp);
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::SaveB(const char *fileName, const uint &nPixels) const {
    FILE *fp = fopen(fileName, "wb");
    SaveB(fp, nPixels);
    fclose(fp);
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::SaveB(FILE *fp, const uint &nPixels) const {
    const ushort heightCeil = ushort((nPixels + m_width - 1) / m_width);
    const uint nPixelsCeil = uint(m_width) * uint(heightCeil);
    std::vector<float> pixels(nPixelsCeil * CHANNELS_NUMBER);
    DownloadToCPU(pixels.data(), m_width, heightCeil);
    fwrite(pixels.data(), sizeof(float), nPixels * CHANNELS_NUMBER, fp);
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::LoadB(const char *fileName) const {
    FILE *fp = fopen(fileName, "rb");
    LoadB(fp);
    fclose(fp);
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::LoadB(FILE *fp) const {
    std::vector<float> pixels(GetTotalSize());
    fread(pixels.data(), sizeof(float), GetTotalSize(), fp);
    Bind();
    UploadFromCPU(pixels.data());
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::LoadB(const char *fileName, const uint &nPixels) const {
    FILE *fp = fopen(fileName, "rb");
    LoadB(fp, nPixels);
    fclose(fp);
}

template<ushort CHANNELS_NUMBER>
inline void TextureGL<CHANNELS_NUMBER>::LoadB(FILE *fp, const uint &nPixels) const {
    const ushort heightCeil = ushort((nPixels + m_width - 1) / m_width);
    const uint nPixelsCeil = uint(m_width) * uint(heightCeil);
    std::vector<float> pixels(nPixelsCeil * CHANNELS_NUMBER);
    fread(pixels.data(), sizeof(float), nPixels * CHANNELS_NUMBER, fp);
    Bind();
    UploadFromCPU(pixels.data(), m_width, heightCeil);
}

#if _DEBUG
template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::DownloadToCPUWithoutFBO(TYPE *pixels) const {
    AssertBound();
    glGetTexImage(GL_TEXTURE_RECTANGLE_ARB, 0, GetFormat<CHANNELS_NUMBER>(), GetType<TYPE>(), pixels);
}

template<ushort CHANNELS_NUMBER> template<typename TYPE>
inline void TextureGL<CHANNELS_NUMBER>::DownloadToCPUWithoutFBO(TYPE *pixels, const GLenum &components) const {
    AssertBound();
    AssertComponentsCompitable(components);
    glGetTexImage(GL_TEXTURE_RECTANGLE_ARB, 0, components, GetType<TYPE>(), pixels);
}
#endif

template<typename TYPE>
inline void TextureGLDepthMask::DownloadToCPU(TYPE *pixels) const {
#if _DEBUG
    AssertBound();
#endif
    glGetTexImage(GL_TEXTURE_RECTANGLE_ARB, 0, GL_DEPTH_COMPONENT, GetType<TYPE>(), pixels);
}

