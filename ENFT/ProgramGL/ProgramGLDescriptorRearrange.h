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

#ifndef _DESCRIPTOR_REARRANGE_H_
#define _DESCRIPTOR_REARRANGE_H_

#include "ProgramGL.h"

enum DescriptorRearrangeMode { DRM_LEVEL_TO_GLOBAL, DRM_GLOBAL_TO_LEVEL };

class ProgramGLDescriptorRearrange
{

public:

	inline void Initialize(const ushort &descTexWidth)
	{
		m_descTexWidth = descTexWidth;
		m_descTexWidthLog = ushort(log(float(descTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_descTexWidthLog) == descTexWidth);
#endif

		for(ushort i = 0; i < 2; ++i)
		{
			char source[MAX_SOURCE_LENGTH];
			std::ostrstream out(source, MAX_SOURCE_LENGTH);
			out <<
				"uniform sampler2DRect g_srcDescTex;\n"
				"uniform int g_iPixStart;\n"
				"void main()\n"
				"{\n"
				"	ivec2 dstCoord = ivec2(gl_FragCoord.st);\n"
				"	int iDstPix = (dstCoord.y << " << m_descTexWidthLog << ") + dstCoord.x;\n";

			if(i == DRM_LEVEL_TO_GLOBAL)
				out <<
				"	int iSrcPix = iDstPix - g_iPixStart;\n";
			else
				out <<
				"	int iSrcPix = iDstPix + g_iPixStart;\n";

			out <<
				"	ivec2 srcCoord = ivec2(iSrcPix & " << descTexWidth - 1 << ", iSrcPix >> " << m_descTexWidthLog << ");\n"
				"	gl_FragColor = texture2DRect(g_srcDescTex, srcCoord + vec2(0.5));\n"
				"}\n" << '\0';

			//printf("%s\n", source);

			std::vector<std::string> inpTexNames(1), inpParamNames(1);
			inpTexNames[0] = "g_srcDescTex";
			inpParamNames[0] = "g_iPixStart";
			m_programs[i].LoadSource(source, inpTexNames, inpParamNames);
		}
	}
	inline void Run(const DescriptorRearrangeMode &mode, const TextureGL4 &srcDescTex, const TextureGL4 &dstDescTex, const ushort &iFtrStart, const ushort &nFtrs) const
	{
#if _DEBUG
		assert(srcDescTex.GetWidth() == m_descTexWidth && dstDescTex.GetWidth() == m_descTexWidth);
#endif

		const ProgramGL &program = m_programs[mode];
		program.Activate();
		program.SetInputTexture(srcDescTex);
		program.SetInputParameter(int(iFtrStart) << 4);
		program.SetOutputTexture(dstDescTex);
		if(mode == FRM_LEVEL_TO_GLOBAL)
			program.DrawQuad(uint(iFtrStart) << 4, uint(nFtrs) << 4, m_descTexWidth, m_descTexWidthLog);
		else
			program.DrawQuad(uint(nFtrs) << 4, m_descTexWidth, m_descTexWidthLog);
		program.Deactivate();
	}

private:

	ProgramGL m_programs[2];
	ushort m_descTexWidth, m_descTexWidthLog;

};

#endif