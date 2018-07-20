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

#ifndef _FEATURE_TRACKER_ENFT_H_
#define _FEATURE_TRACKER_ENFT_H_

#include "SfM/Homography.h"
#include "SfM/FundamentalMatrix.h"
#include "ProgramGL/ProgramGLUnpack.h"
#include "ProgramGL/ProgramGLUtility.h"
#include "ProgramGL/ProgramGLGradient.h"
#include "ProgramGL/ProgramGLComputeGainRatio.h"
#include "ProgramGL/ProgramGLTrackFeatureEnft.h"

class FeatureTrackerEnft
{

public:

	inline void Initialize(const ushort &width, const ushort &height, const ushort &bufferSize, const ushort &maxNumFtrs, const ushort &ftrTexWidth, 
						   const ushort &winSz, const ushort &nIters, const float &lambdaEp, const float &lambdaHomo, const float &deltaTh, const float &SADTh, 
						   const float &errThEp, const float &errThHomo)
	{
		m_bufferSize = bufferSize;
		const ushort bufferSize_p1 = bufferSize + 1;
		m_imgTexs.resize(bufferSize_p1);
		for(ushort iBuffer = 0; iBuffer < bufferSize_p1; ++iBuffer)
			m_imgTexs[iBuffer].Generate(width, height);
		m_imgGradTex.Generate(width, height);

		const ushort ftrTexHeight = (maxNumFtrs + ftrTexWidth - 1) / ftrTexWidth;
		//m_ftrTexSrc.Generate(ftrTexWidth, ftrTexHeight);
		m_ftrTex.Generate(ftrTexWidth, ftrTexHeight);
		m_tmpFtrTex.Generate(ftrTexWidth, ftrTexHeight);
		m_epLineTex.Generate(ftrTexWidth, ftrTexHeight);
		m_errTex.Generate(ftrTexWidth, ftrTexHeight);
		m_gainRatioTex.Generate(ftrTexWidth, ftrTexHeight);

		m_ftrTexWidth = ftrTexWidth;
		m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
#if _DEBUG
		assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
#endif

		const ushort fltWinTexWidth = ftrTexWidth * winSz * winSz;
		const ushort fltWinTexHeight = ftrTexHeight;
		m_GTex.Generate(fltWinTexWidth, fltWinTexHeight);
		m_eTex.Generate(fltWinTexWidth, fltWinTexHeight);
		m_ITex.Generate(fltWinTexWidth, fltWinTexHeight);
		m_ITex12.Generate(fltWinTexWidth, fltWinTexHeight);

		m_programUnpack.Initialize();
		m_programScale.Initialize();
		m_programGradient.Initialize();
		m_programComputeGainRatio.Initialize(ftrTexWidth, fltWinTexWidth, winSz);
		m_programTrackFeature.Initialize(ftrTexWidth, fltWinTexWidth, winSz, nIters, lambdaEp, lambdaHomo, width, height, deltaTh, SADTh, errThEp, errThHomo);
	}
	inline void SetImage(const ushort &iBuffer, const TextureGL4 &gaussTex/*, const bool &compGrad*/)
	{
		ProgramGL::FitViewportGL(m_imgTexs[iBuffer]);
		m_programUnpack.Run(gaussTex, m_imgTexs[iBuffer]);
		//if(compGrad)
		m_programGradient.Run(m_imgTexs[iBuffer], m_imgGradTex);
	}
	inline void SetImage(const ushort &iBuffer, const TextureGL4 &gaussTex, const float &scale/*, const bool &compGrad*/)
	{
		ProgramGL::FitViewportGL(m_imgTexs[iBuffer]);
		m_programUnpack.Run(gaussTex, m_imgTexs[iBuffer]);
		m_programScale.Run(scale, m_imgTexs[iBuffer]);
		//if(compGrad)
		m_programGradient.Run(m_imgTexs[iBuffer], m_imgGradTex);
	}
	inline void SetImage(const ushort &iBuffer, const float *gaussImg/*, const bool &compGrad*/)
	{
		ProgramGL::FitViewportGL(m_imgTexs[iBuffer]);
		m_imgTexs[iBuffer].Bind();
		m_imgTexs[iBuffer].UploadFromCPU(gaussImg);
		//if(compGrad)
		m_programGradient.Run(m_imgTexs[iBuffer], m_imgGradTex);
	}
	inline const TextureGL1& GetImageTexture(const ushort &iBuffer) const { return m_imgTexs[iBuffer]; }
	inline const TextureGL4& GetFeatureTexture() const { return m_ftrTex; }
	inline void PushBackGainRatios(const ushort iBufferSrc, const ushort iBufferDst, const TextureGL4 &ftrTexSrcDst, const ushort &nFtrs, const Homography &H, 
								   std::vector<float> &gainRatios)
	{
		ProgramGL::FitViewportGL(m_ITex12);
		m_programComputeGainRatio.Run(ftrTexSrcDst, m_imgTexs[iBufferSrc], m_imgTexs[iBufferDst], m_ftrTex, m_ITex12, m_gainRatioTex, nFtrs, H);
		DownloadGainRatiosToCPU(m_gainRatios, nFtrs);
		gainRatios.insert(gainRatios.end(), m_gainRatios.begin(), m_gainRatios.end());
	}
	template<ushort CHANNELS_NUMBER>
	inline void TrackFeatures(const ushort iBufferSrc, const ushort iBufferDst, const TextureGL<CHANNELS_NUMBER> &ftrTex, const ushort nFtrs, 
							  const FundamentalMatrix &F, const AlignedVector<Homography> &Hs, const float &gainRatio, std::vector<FeatureEnftMatch> &matches)
	{
		if(Hs.Empty())
		{
			matches.resize(0);
			return;
		}
		ProgramGL::FitViewportGL(m_ITex);
		m_programTrackFeature.Run(ftrTex, m_imgTexs[iBufferSrc], m_imgGradTex, m_ftrTex, m_tmpFtrTex, m_epLineTex, m_errTex, m_ITex, m_GTex, m_eTex, nFtrs, 
			F, Hs, gainRatio);
		DownloadFeaturesToCPU(m_ftrs, nFtrs);
		matches.resize(0);
		for(ushort i = 0; i < nFtrs; ++i)
		{
			if(m_ftrs[i].IsValid())
				matches.push_back(FeatureEnftMatch(i, m_ftrs[i]));
		}
	}
	template<ushort CHANNELS_NUMBER>
	inline void TrackFeatures(const ushort iBufferSrc, const ushort iBufferDst, const TextureGL<CHANNELS_NUMBER> &ftrTex, const ushort nFtrs, 
							  const FundamentalMatrix &F, const AlignedVector<Homography> &Hs, const float &gainRatio, std::vector<FeatureEnftPlane> &ftrs)
	{
		if(Hs.Empty())
		{
			ftrs.resize(0);
			return;
		}
		ProgramGL::FitViewportGL(m_ITex);
		m_programTrackFeature.Run(ftrTex, m_imgTexs[iBufferSrc], m_imgGradTex, m_ftrTex, m_tmpFtrTex, m_epLineTex, m_errTex, m_ITex, m_GTex, m_eTex, nFtrs, 
			F, Hs, gainRatio);
		DownloadFeaturesToCPU(ftrs, nFtrs);
	}
	//inline void TrackFeatures(const ushort iBufferSrc, const ushort iBufferDst, const FundamentalMatrix &F, const AlignedVector<Homography> &Hs, 
	//						  const float &gainRatio, AlignedVector<Feature> &ftrsSrcDst)
	//{
	//	const ushort nFtrs = ushort(ftrsSrcDst.Size());
	//	if(Hs.Empty())
	//	{
	//		ftrsSrcDst.Resize(nFtrs);
	//		for(ushort i = 0; i < nFtrs; ++i)
	//			ftrsSrcDst[i].Invalidate();
	//		return;
	//	}
	//	ProgramGL::FitViewportGL(m_ITex);
	//	const uint drawHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
	//	const uint nDrawPixels1 = (drawHeight1 << m_ftrTexWidthLog);
	//	m_ftrTexSrc.Bind();
	//	if(nDrawPixels1 != 0)
	//		m_ftrTexSrc.UploadFromCPU((float *) ftrsSrcDst.Data(), m_ftrTexWidth, drawHeight1);
	//	const uint nRemDrawPixels = nFtrs - nDrawPixels1;
	//	if(nRemDrawPixels != 0)
	//		m_ftrTexSrc.UploadFromCPU((float *) (ftrsSrcDst.Data() + nDrawPixels1), 0, drawHeight1, nRemDrawPixels, 1);

	//	m_programTrackFeature.Run(m_ftrTexSrc, m_imgTexs[iBufferSrc], m_imgGradTex, m_ftrTex, m_tmpFtrTex, m_epLineTex, m_errTex, m_ITex, m_GTex, m_eTex, nFtrs, 
	//		F, Hs, gainRatio);
	//	DownloadFeaturesToCPU(ftrsSrcDst, nFtrs);
	//}
	inline void DownloadFeaturesToCPU(AlignedVector<Point2D> &ftrs, const ushort &nFtrs)
	{
		ftrs.Resize(nFtrs);
		const uint readHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
		const uint nReadPixels1 = (readHeight1 << m_ftrTexWidthLog);
		if(nReadPixels1 != 0)
			m_ftrTex.DownloadToCPU((float *) ftrs.Data(), m_ftrTexWidth, readHeight1, GL_RG);
		const uint nReadPixels2 = nFtrs - nReadPixels1;
		if(nReadPixels2 != 0)
			m_ftrTex.DownloadToCPU((float *) (ftrs.Data() + nReadPixels1), 0, readHeight1, nReadPixels2, 1, GL_RG);
	}
	inline void DownloadFeaturesToCPU(std::vector<FeatureEnft> &ftrs, const ushort &nFtrs)
	{
		ftrs.resize(nFtrs);
		const uint readHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
		const uint nReadPixels1 = (readHeight1 << m_ftrTexWidthLog);
		if(nReadPixels1 != 0)
			m_ftrTex.DownloadToCPU((float *) ftrs.data(), m_ftrTexWidth, readHeight1, GL_RGB);
		const uint nReadPixels2 = nFtrs - nReadPixels1;
		if(nReadPixels2 != 0)
			m_ftrTex.DownloadToCPU((float *) (ftrs.data() + nReadPixels1), 0, readHeight1, nReadPixels2, 1, GL_RGB);
	}
	inline void DownloadFeaturesToCPU(std::vector<FeatureEnftPlane> &ftrs, const ushort &nFtrs)
	{
		ftrs.resize(nFtrs);
		const uint readHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
		const uint nReadPixels1 = (readHeight1 << m_ftrTexWidthLog);
		if(nReadPixels1 != 0)
			m_ftrTex.DownloadToCPU((float *) ftrs.data(), m_ftrTexWidth, readHeight1);
		const uint nReadPixels2 = nFtrs - nReadPixels1;
		if(nReadPixels2 != 0)
			m_ftrTex.DownloadToCPU((float *) (ftrs.data() + nReadPixels1), 0, readHeight1, nReadPixels2, 1);
	}
	//inline void DownloadTextureToCPU(const TextureGL2 &tex, std::vector<LA::Vector2f> &buffer, const uint nPixels)
	//{
	//	buffer.resize(nPixels);
	//	const uint readHeight1 = (nPixels >> m_ftrTexWidthLog);
	//	const uint nReadPixels1 = (readHeight1 << m_ftrTexWidthLog);
	//	if(nReadPixels1 != 0)
	//		tex.DownloadToCPU((float *) buffer.data(), m_ftrTexWidth, readHeight1);
	//	const uint nReadPixels2 = nPixels - nReadPixels1;
	//	if(nReadPixels2 != 0)
	//		tex.DownloadToCPU((float *) (buffer.data() + nReadPixels1), 0, readHeight1, nReadPixels2, 1);
	//}
	//inline void DownloadTextureToCPU(const TextureGL4 &tex, std::vector<LA::Vector4f> &buffer, const uint nPixels)
	//{
	//	buffer.resize(nPixels);
	//	const uint readHeight1 = (nPixels >> m_ftrTexWidthLog);
	//	const uint nReadPixels1 = (readHeight1 << m_ftrTexWidthLog);
	//	if(nReadPixels1 != 0)
	//		tex.DownloadToCPU((float *) buffer.data(), m_ftrTexWidth, readHeight1);
	//	const uint nReadPixels2 = nPixels - nReadPixels1;
	//	if(nReadPixels2 != 0)
	//		tex.DownloadToCPU((float *) (buffer.data() + nReadPixels1), 0, readHeight1, nReadPixels2, 1);
	//}
	inline void DownloadGainRatiosToCPU(std::vector<float> &gainRatios, const ushort &nFtrs)
	{
		gainRatios.resize(nFtrs);
		const uint readHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
		const uint nReadPixels1 = (readHeight1 << m_ftrTexWidthLog);
		if(nReadPixels1 != 0)
			m_gainRatioTex.DownloadToCPU(gainRatios.data(), m_ftrTexWidth, readHeight1);
		const uint nReadPixels2 = nFtrs - nReadPixels1;
		if(nReadPixels2 != 0)
			m_gainRatioTex.DownloadToCPU(gainRatios.data() + nReadPixels1, 0, readHeight1, nReadPixels2, 1);
	}

	template<class MATCH_TYPE>
	static inline void FromMatches12ToMatchMarks1(const ushort &nFtrs1, const std::vector<MATCH_TYPE> &matches, const std::vector<FeatureEnftMatch> &matchesEnft, 
												  std::vector<bool> &matchMarks1)
	{
		matchMarks1.assign(nFtrs1, false);
		const ushort nMatches = ushort(matches.size());
		for(ushort i = 0; i < nMatches; ++i)
			matchMarks1[matches[i].GetIndex1()] = true;
		const ushort nMatchesEnft = ushort(matchesEnft.size());
		for(ushort i = 0; i < nMatchesEnft; ++i)
			matchMarks1[matchesEnft[i].GetFeatureIndex1()] = true;
	}
	static inline void FromMatchMarksToUnmatchedFeatureIndexes(const std::vector<bool> &matchMarks, const ushort nFtrs, std::vector<ushort> &iFtrs)
	{
		iFtrs.resize(0);
		for(ushort iFtr = 0; iFtr < nFtrs; ++iFtr)
		{
			if(!matchMarks[iFtr])
				iFtrs.push_back(iFtr);
		}
	}
	template<class MATCH_TYPE>
	static inline void FromMatches12ToUnmatchedFeatureIndexes1(const ushort &nFtrs1, const std::vector<MATCH_TYPE> &matches, 
															   const std::vector<FeatureEnftMatch> &matchesEnft, std::vector<ushort> &iFtrs1, 
															   std::vector<bool> &matchMarks1)
	{
		FromMatches12ToMatchMarks1<MATCH_TYPE>(nFtrs1, matches, matchesEnft, matchMarks1);
		FromMatchMarksToUnmatchedFeatureIndexes(matchMarks1, nFtrs1, iFtrs1);
	}
	static inline void FromMatches13To23(const std::vector<ushort> &iFtrs1To2, std::vector<FeatureEnftMatch> &matches13, std::vector<FeatureEnftMatch> &matches23)
	{
		matches23.resize(0);
		const ushort nMatches13 = ushort(matches13.size());
		ushort i, j, iFtr2;
		for(i = j = 0; i < nMatches13; ++i)
		{
			if((iFtr2 = iFtrs1To2[matches13[i].GetFeatureIndex1()]) == USHRT_MAX)
				matches13[j++] = matches13[i];
			else
				matches23.push_back(FeatureEnftMatch(iFtr2, matches13[i].GetFeature2()));
		}
		matches13.resize(j);
	}
	static inline void FromMatches13To23(const std::vector<ushort> &iFtrs1To2, std::vector<FeatureEnftMatch> &matches)
	{
		const ushort nMatches = ushort(matches.size());
		for(ushort i = 0; i < nMatches; ++i)
			matches[i].SetFeatureIndex1(iFtrs1To2[matches[i].GetFeatureIndex1()]);
	}

private:

	ushort m_bufferSize, m_ftrTexWidth, m_ftrTexWidthLog;
	std::vector<TextureGL1> m_imgTexs;
	TextureGL2 /*m_ftrTexSrc, */m_ITex12;
	TextureGL4 m_imgGradTex, m_ftrTex, m_tmpFtrTex, m_epLineTex, m_errTex, m_GTex, m_eTex;
	TextureGL1 m_ITex, m_gainRatioTex;
	std::vector<float> m_gainRatios;
	std::vector<FeatureEnft> m_ftrs;

	ProgramGLUnpack				m_programUnpack;
	ProgramGLScale				m_programScale;
	ProgramGLGradient			m_programGradient;
	ProgramGLComputeGainRatio	m_programComputeGainRatio;
	ProgramGLTrackFeatureEnft	m_programTrackFeature;

};

#endif