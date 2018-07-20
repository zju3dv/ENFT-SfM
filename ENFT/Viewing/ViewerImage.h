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

#ifndef _VIEWER_IMAGE_H_
#define _VIEWER_IMAGE_H_

#include "Viewer.h"
#include "Utility/SSE.h"
#include "ProgramGL/ProgramGLSample.h"
#include "ProgramGL/ProgramGLUnpack.h"
#include <cvd/image.h>
#include <cvd/rgba.h>

template<ImageType IMAGE_TYPE>
class ViewerImage : public Viewer
{

public:

	inline void Initialize(const ushort &width, const ushort &height) { Viewer::Initialize(); m_tex.Generate(width, height); }

	inline void SetImage(const ubyte *img) { m_tex.Bind(); m_tex.UploadFromCPU(img); }
	inline void SetImage(const float *img) { m_tex.Bind(); m_tex.UploadFromCPU(img); }
	inline void SetImage(const ushort *img)
	{
		const uint nPixels = m_tex.GetPixelsNumber();
		ushort maxVal = 0;
		for(uint i = 0; i < nPixels; ++i)
		{
			if(img[i] > maxVal)
				maxVal = src[i];
		}
		const ENFT_SSE::__m128 norm = ENFT_SSE::_mm_set1_ps(255.0f / maxVal);

		m_buffer.resize(nPixels);
		const ushort *src = img;
		ubyte *dst = m_buffer.data();

		ENFT_SSE::__m128 tmp;
		const uint _nPixels = (nPixels & (~3));
		for(uint i = 0; i < _nPixels; i += 4, src += 4, dst += 4)
		{
			tmp = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set_ps(float(src[3]), float(src[2]), float(src[1]), float(src[0])), norm);
			dst[0] = ubyte(tmp.m128_f32[0]);
			dst[1] = ubyte(tmp.m128_f32[1]);
			dst[2] = ubyte(tmp.m128_f32[2]);
			dst[3] = ubyte(tmp.m128_f32[3]);
		}
		for(uint i = _nPixels; i < nPixels; ++i)
			m_buffer[i] = ubyte(img[i] * norm.m128_f32[0]);

		m_tex.Bind();
		m_tex.UploadFromCPU(m_buffer.data());
	}
	template<ushort CHANNELS_NUMBER>
	inline void SetImage(const TextureGL<CHANNELS_NUMBER> &tex);
	inline void SetString(const char *str) { m_str = str; }

protected:

	virtual void OnDraw()
	{
		m_tex.Bind();
		Viewer::DrawTexture(m_tex.GetWidth(), m_tex.GetHeight());
		//glEnable(GL_TEXTURE_RECTANGLE_ARB);
		//m_tex.Bind();
		//glBegin(GL_QUADS);
		//glTexCoord2i(0, 0);									glVertex2i(0, 0);
		//glTexCoord2i(0, m_tex.GetHeight());					glVertex2i(0, m_size.y);
		//glTexCoord2i(m_tex.GetWidth(), m_tex.GetHeight());	glVertex2i(m_size.x, m_size.y);
		//glTexCoord2i(m_tex.GetWidth(), 0);					glVertex2i(m_size.x, 0);
		//glEnd();
		//glDisable(GL_TEXTURE_RECTANGLE_ARB);

		//glEnable(GL_TEXTURE_2D);
		//glBindTexture(GL_TEXTURE_2D, m_texture);
		//glBegin(GL_QUADS);
		//glTexCoord2i(0, 0);		glVertex2i(0, 0);
		//glTexCoord2i(0, 1);		glVertex2i(0, m_size.y);
		//glTexCoord2i(1, 1);		glVertex2i(m_size.x, m_size.y);
		//glTexCoord2i(1, 0);		glVertex2i(m_size.x, 0);
		//glEnd();
		//glDisable(GL_TEXTURE_2D);
	}
	virtual void OnDrawString()
	{
		if(m_str.empty())
			return;
		glColor3ub(0, 255, 0);
		Viewer::DrawStringBottomLeftLarge("%s", m_str.c_str());
	}

private:

	template<class PROGRAM, ushort CHANNELS_NUMBER_SRC, ushort CHANNELS_NUMBER_DST>
	static inline void RunProgram(PROGRAM &program, const TextureGL<CHANNELS_NUMBER_SRC> &srcTex, const TextureGL<CHANNELS_NUMBER_DST> &dstTex)
	{
		if(!program.IsInitialized())
			program.Initialize();
		ProgramGL::FitViewportGL(dstTex.GetWidth(), dstTex.GetHeight());
		program.Run(srcTex, dstTex);

//#if _DEBUG
//		std::vector<ubyte> buffer1, buffer2;
//		buffer1.resize(srcTex.GetTotalSize());
//		srcTex.DownloadToCPU(buffer1.data());
//		buffer2.resize(dstTex.GetTotalSize());
//		dstTex.DownloadToCPU(buffer2.data());
//#endif
	}

protected:

	TextureGL<IMAGE_TYPE == IMAGE_TYPE_RGB ? 4 : 1> m_tex;
	std::string m_str;
	std::vector<ubyte> m_buffer;
	ProgramGLCopy m_programCopy;
	ProgramGLConvertRGB2Gray m_programConvertRGB2Gray;
	ProgramGLConvertGray2RGB m_programConvertGray2RGB;
	ProgramGLUnpack m_programUnpack;
};

template<> template<> inline void ViewerImage<IMAGE_TYPE_RGB>::SetImage<4>(const TextureGL4 &tex) { RunProgram(m_programCopy, tex, m_tex); }
template<> template<> inline void ViewerImage<IMAGE_TYPE_RGB>::SetImage<1>(const TextureGL1 &tex) { RunProgram(m_programConvertGray2RGB, tex, m_tex); }
template<> template<> inline void ViewerImage<IMAGE_TYPE_GRAY>::SetImage<4>(const TextureGL4 &tex) { RunProgram(m_programConvertRGB2Gray, tex, m_tex); }
template<> template<> inline void ViewerImage<IMAGE_TYPE_GRAY>::SetImage<1>(const TextureGL1 &tex) { RunProgram(m_programCopy, tex, m_tex); }
template<> template<> inline void ViewerImage<IMAGE_TYPE_GRAY_PACKED>::SetImage<4>(const TextureGL4 &tex) { RunProgram(m_programUnpack, tex, m_tex); }

#endif