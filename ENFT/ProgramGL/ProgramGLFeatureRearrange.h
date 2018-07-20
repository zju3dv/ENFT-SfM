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

#ifndef _PROGRAM_GL_FEATURE_REARRANGE_H_
#define _PROGRAM_GL_FEATURE_REARRANGE_H_

#include "ProgramGL.h"
#include "Utility/Table.h"

enum FeatureRearrangeMode { FRM_LEVEL_TO_GLOBAL, FRM_GLOBAL_TO_LEVEL };

class ProgramGLFeatureRearrange
{

public:

	inline void Initialize(const ushort &ftrTexWidth, const ushort &nOctaves, const bool &upSample)
	{
		m_ftrTexWidth = ftrTexWidth;
		m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
#endif

		const float s0 = upSample ? 0.5f : 1.0f;

		m_programTable.Resize(2, nOctaves);
		for(ushort i = 0; i < 2; ++i)
		{
			float s = i == FRM_LEVEL_TO_GLOBAL ? s0 : 1 / s0;
			float sk = i == FRM_LEVEL_TO_GLOBAL ? 2.0f : 0.5f;
			for(ushort iOctave = 0; iOctave < nOctaves; ++iOctave, s *= sk)
			{
				char source[MAX_SOURCE_LENGTH];
				std::ostrstream out(source, MAX_SOURCE_LENGTH);
				out <<
					"uniform sampler2DRect g_srcFtrTex;\n"
					"uniform int g_iFtrStart;\n"
					"void main()\n"
					"{\n"
					"	ivec2 dstCoord = ivec2(gl_FragCoord.st);\n"
					"	int iDstPix = (dstCoord.y << " << m_ftrTexWidthLog << ") + dstCoord.x;\n";

				if(i == FRM_LEVEL_TO_GLOBAL)
					out <<
					"	int iSrcPix = iDstPix - g_iFtrStart;\n";
				else
					out <<
					"	int iSrcPix = iDstPix + g_iFtrStart;\n";

				out <<
					"	ivec2 srcCoord = ivec2(iSrcPix & " << ftrTexWidth - 1 << ", iSrcPix >> " << m_ftrTexWidthLog << ");\n"
					"	vec4 ftr = texture2DRect(g_srcFtrTex, srcCoord + vec2(0.5));\n";

				if(s == 1.0f)
					out <<
					"	gl_FragColor = ftr;\n";
				else
					out <<
					"	gl_FragColor = vec4((ftr.xy - vec2(0.5)) * " << s << " + vec2(0.5), ftr.z * " << s << ", ftr.w);\n";
				out <<
					"}\n" << '\0';

				//printf("%s\n", source);

				std::vector<std::string> inpTexNames(1), inpParamNames(1);
				inpTexNames[0] = "g_srcFtrTex";
				inpParamNames[0] = "g_iFtrStart";
				m_programTable[i][iOctave].LoadSource(source, inpTexNames, inpParamNames);
			}
		}
	}

	inline void Run(const FeatureRearrangeMode &mode, const ushort &iOctave, const TextureGL4 &srcFtrTex, const TextureGL4 &dstFtrTex, 
					const uint &iFtrStart, const uint &nFtrs) const
	{
#if _DEBUG
		assert(srcFtrTex.GetWidth() == m_ftrTexWidth && dstFtrTex.GetWidth() == m_ftrTexWidth);
#endif
		const ProgramGL &program = m_programTable[mode][iOctave];
		program.Activate();
		program.SetInputTexture(srcFtrTex);
		program.SetInputParameter(int(iFtrStart));
		program.SetOutputTexture(dstFtrTex);
		if(mode == FRM_LEVEL_TO_GLOBAL)
			program.DrawQuad(iFtrStart, nFtrs, m_ftrTexWidth, m_ftrTexWidthLog);
		else
			program.DrawQuad(nFtrs, m_ftrTexWidth, m_ftrTexWidthLog);
		program.Deactivate();
	}

private:

	Table<ProgramGL, ushort> m_programTable;
	ushort m_ftrTexWidth, m_ftrTexWidthLog;

};

#endif