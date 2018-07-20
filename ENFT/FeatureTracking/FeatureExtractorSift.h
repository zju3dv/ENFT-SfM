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

#ifndef _FEATURE_EXTRACTOR_SIFT_H_
#define _FEATURE_EXTRACTOR_SIFT_H_

#include "SfM/Match.h"
#include "Sequence/Feature.h"
#include "Sequence/Descriptor.h"
#include "Utility/FillMap.h"
#include "ProgramGL/ProgramGLSample.h"
#include "ProgramGL/ProgramGLFilterPacked.h"
#include "ProgramGL/ProgramGLGradient.h"
#include "ProgramGL/ProgramGLDoG.h"
#include "ProgramGL/ProgramGLDoGExtremum.h"
#include "ProgramGL/ProgramGLFeatureHistogram.h"
#include "ProgramGL/ProgramGLFeatureLocalize.h"
#include "ProgramGL/ProgramGLFeatureRearrange.h"
#include "ProgramGL/ProgramGLFeatureCopy.h"
#include "ProgramGL/ProgramGLFeatureSelect.h"
#include "ProgramGL/ProgramGLAssignOrientation.h"
#include "ProgramGL/ProgramGLDescriptorGenerate.h"
#include "ProgramGL/ProgramGLDescriptorRearrange.h"
#include "ProgramGL/ProgramGLDescriptorNormalize.h"
#include "ProgramGL/ProgramGLDescriptorCopy.h"
#include "ProgramGL/ProgramGLDescriptorSelect.h"
#include <cvd/image.h>
#include "Utility/Timer.h"

#define FE_PRINT_TIMING 0

#if FE_PRINT_TIMING
#include "Table.h"
#include "Utility/Timer.h"
#endif

class FeatureExtractorSift
{

public:

	class Octave
	{

	public:

		void Initialize(const ushort &widthPacked, const ushort &heightPacked, const ushort &nLevelsDoG, const ushort &maxNumFtrs, const ushort &ftrTexWidth);

		inline const ushort& GetPackedWidth () const { return m_tmpTex.GetWidth (); }
		inline const ushort& GetPackedHeight() const { return m_tmpTex.GetHeight(); }
		inline const TextureGL4& GetGaussianTexture(const ushort &iGauss) const { return m_gaussTexs[iGauss]; }
		inline const TextureGL4& GetDoGTexture(const ushort &iDoG) const { return m_dogTexs[iDoG]; }
		inline const TextureGL4& GetTemporaryTexture() const { return m_tmpTex; }
		inline const TextureGL4& GetGradientMagnitudeTexture(const ushort &iLevelDoG) const { return m_gradMagTexs[iLevelDoG]; }
		inline const TextureGL4& GetGradientDirectionTexture(const ushort &iLevelDoG) const { return m_gradDirTexs[iLevelDoG]; }
		inline const TextureGL4& GetExtremeTexture(const ushort &iLevelDoG) const { return m_extremeTexs[iLevelDoG]; }
		inline const TextureGL4& GetFeatureTexture(const ushort &iLevelDoG) const { return m_ftrTexs[iLevelDoG]; }
		inline		 TextureGL4& GetFeatureTexture(const ushort &iLevelDoG)		  { return m_ftrTexs[iLevelDoG]; }
		inline const TextureGL4& GetDescriptorTexture(const ushort &iLevelDoG) const { return m_descTexs[iLevelDoG]; }

	private:

		TextureGL4 m_tmpTex;
		std::vector<TextureGL4> m_gaussTexs, m_dogTexs, m_gradMagTexs, m_gradDirTexs, m_extremeTexs, m_ftrTexs, m_descTexs;

	};

public:

	FeatureExtractorSift();
	~FeatureExtractorSift();
	void Initialize(const ushort &widthInit, const ushort &heightInit, const ushort &bufferSize, const ushort &maxNumFtrsExtracted, 
		const ushort &maxNumFtrsAdditional, const ushort &ftrTexWidth, const ushort &nOctaves, const ushort &nLevelsDoG, const bool &upSample = false, 
		const float &dogTh = 0.02f, const float &edgeTh = 10.0f, const float &hessianTh = 500.0f, const ushort &minDist = 10);
	void ExtractFeatures(const TextureGL1 &inpTex, const ushort &iBuffer);
	void SelectFeatures(const ushort &iBufferSrc, const ushort &iBufferDst, const std::vector<ushort> &idxs);
	void SelectFeatures(const ushort &iBufferSrc1, const ushort &iBufferSrc2, const ushort &iBufferDst, const std::vector<Match<ushort> > &matches);
	void SelectDescriptors(const ushort &iBufferSrc, const ushort &iBufferDst, const std::vector<ushort> &idxs);
	void SelectFeaturesAndDescriptors(const ushort &iBufferSrc, const ushort &iBufferDst, const std::vector<ushort> &idxs);

#if FE_PRINT_TIMING
	void PrintTiming(const char *str, const ushort &iRecord);
	void PrintTiming();
#endif

	inline const TextureGL4& GetGaussianTexture(const ushort &iOctave, const ushort &iGauss) const { return m_octaves[iOctave].GetGaussianTexture(iGauss); }
	inline const TextureGL4& GetFeatureTexture(const ushort &iBuffer) const { return m_ftrTexs[iBuffer]; }
	inline const TextureGL4& GetDescriptorTexture(const ushort &iBuffer) const { return m_descTexs[iBuffer]; }
	inline const uint& GetFeaturesNumber(const ushort &iBuffer) const { return m_nFtrsTotal[iBuffer]; }
	inline const std::vector<uint>& GetFeaturesNumberBuffer() const { return m_nFtrsTotal; }
	inline		 std::vector<uint>& GetFeaturesNumberBuffer()		  { return m_nFtrsTotal; }
	inline const std::vector<uint>& GetFeaturesNumberOctave(const ushort &iBuffer) const { return m_nFtrsOctave[iBuffer]; }
	inline const Table<uint, ushort>& GetFeaturesNumberLevel(const ushort &iBuffer) const { return m_nFtrsLevel[iBuffer]; }
	template<class Feature, typename Score>
	inline void MarkMaximalFeatures(const Feature *ftrs, const std::vector<std::pair<Score, uint> > &iFtrsScored, const ushort &maxNumFtrs, 
									std::vector<bool> &ftrMarks)
	{
		m_fillMap.MarkMaximalPoints(ftrs, iFtrsScored, maxNumFtrs, ftrMarks, m_minDist);
	}
	inline void DownloadFeaturesToCPU(const ushort &iBuffer, AlignedVector<Point2D> &ftrs)
	{
		ftrs.Resize(m_nFtrsTotal[iBuffer]);
		DownloadFeaturesToCPU(m_ftrTexs[iBuffer], m_nFtrsTotal[iBuffer], ftrs.Data());
	}
	inline void DownloadFeaturesToCPU(const ushort &iBuffer, std::vector<FeatureSift> &ftrs)
	{
		ftrs.resize(m_nFtrsTotal[iBuffer]);
		DownloadFeaturesToCPU(m_ftrTexs[iBuffer], m_nFtrsTotal[iBuffer], ftrs.data());
	}
	inline void DownloadDescriptorsToCPU(const ushort &iBuffer, AlignedVector<Descriptor> &descs)
	{
		descs.Resize(m_nFtrsTotal[iBuffer]);
		DownloadDescriptorsToCPU(m_descTexs[iBuffer], m_nFtrsTotal[iBuffer], descs.Data());
	}
	
	inline void UploadFeaturesAndDescriptorsFromCPU(const ushort &iBuffer, const AlignedVector<Point2D> &ftrs, const AlignedVector<Descriptor> &descs)
	{
		m_nFtrsTotal[iBuffer] = uint(ftrs.Size());
		m_nFtrsOctave[iBuffer].assign(m_nFtrsOctave[iBuffer].size(), 0);
		m_nFtrsLevel[iBuffer].SetZero();
		UploadFeaturesFromCPU(m_ftrTexs[iBuffer], m_nFtrsTotal[iBuffer], ftrs.Data());
		UploadDescriptorsFromCPU(m_descTexs[iBuffer], m_nFtrsTotal[iBuffer], descs.Data());
	}
	inline void PushBackFeaturesAndDescriptorsFromCPU(const ushort &iBuffer, const uint &nFtrs, const Point2D *ftrs, const Descriptor *descs)
	{
		PushBackFeaturesFromCPU(m_ftrTexs[iBuffer], m_nFtrsTotal[iBuffer], nFtrs, ftrs);
		PushBackDescriptorsFromCPU(m_descTexs[iBuffer], m_nFtrsTotal[iBuffer], nFtrs, descs);
		m_nFtrsTotal[iBuffer] += nFtrs;
		m_nFtrsOctave[iBuffer].assign(m_nFtrsOctave[iBuffer].size(), 0);
		m_nFtrsLevel[iBuffer].SetZero();
	}
	inline void PrintFeaturesNumber(const ushort &iBuffer) const
	{
		printf("Total: %d\n", m_nFtrsTotal[iBuffer]);
		printf("Octave:\n");
		for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave)
			printf(" %d", m_nFtrsOctave[iBuffer][iOctave]);
		printf("\n");
		printf("Level:\n");
		for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave)
		{
			for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG)
				printf(" %d", m_nFtrsLevel[iBuffer][iOctave][iLevelDoG]);
			printf("\n");
		}
	}

private:

	void BuildGaussianPyramid(const TextureGL1 &inpTex);
	void LocalizeFeatures(const ushort &iBuffer);
	uint LocalizeFeatures(const ushort &iOctave, const ushort &iLevelDoG, const TextureGL4 &dogTex, const TextureGL4 &extremeTex, /*const */TextureGL4 &ftrTex);
	void NonMaximalSuppress(const ushort &iBuffer);
	void AssignOrientations(const ushort &iBuffer);
	void GenerateDescriptors(const ushort &iBuffer);
	void RearrangeFeaturesLevelToGlobal(const ushort &iBuffer);
	void RearrangeFeaturesGlobalToLevel(const ushort &iBuffer);
	void RearrangeDescriptorsLevelToGlobal(const ushort &iBuffer);
	void RearrangeDescriptorsGlobalToLevel(const ushort &iBuffer);
	void SelectFeatures(const TextureGL4 &ftrTexSrc, const TextureGL4 &ftrTexDst, const TextureGL1 &idxTex, const uint &nFtrsSrc, const uint &nFtrsDst);
	void SelectFeatures(const TextureGL4 &ftrTexSrc1, const TextureGL4 &ftrTexSrc2, const TextureGL4 &ftrTexDst, const TextureGL2 &idxTex, const uint &nFtrsDst);
	void SelectDescriptors(const TextureGL4 &descTexSrc, const TextureGL4 &descTexDst, const TextureGL1 &idxTex, const uint &nFtrsSrc, const uint &nFtrsDst);
	void DownloadFeaturesToCPU(const TextureGL4 &ftrTex, const uint &nFtrs, FeatureSift* const &ftrs);
	void DownloadFeaturesToCPU(const TextureGL4 &ftrTex, const uint &nFtrs, Point2D* const &ftrs);
	void DownloadDescriptorsToCPU(const TextureGL4 &descTex, const uint &nFtrs, Descriptor* const &descs);
	void DownloadIndexesToCPU(const TextureGL1 &idxTex, const uint &nFtrs, float* const &idxs);
	void UploadFeaturesFromCPU(const TextureGL4 &ftrTex, const uint &nFtrs, const Point2D* const &ftrs);
	void UploadFeaturesFromCPU(const TextureGL4 &ftrTex, const uint &nFtrs, const FeatureSift* const &ftrs);
	void UploadDescriptorsFromCPU(const TextureGL4 &descTex, const uint &nFtrs, const Descriptor* const &descs);
	void UploadIndexesFromCPU(const TextureGL1 &idxTex, const uint &nFtrs, const float* const &idxs);
	void UploadIndexesFromCPU(const TextureGL2 &idxTex, const uint &nFtrs, const float* const &idxs);
	void PushBackFeaturesFromCPU(const TextureGL4 &ftrTex, const uint &nFtrsExist, const uint &nFtrsNew, const Point2D* const &ftrsNew);
	void PushBackDescriptorsFromCPU(const TextureGL4 &descTex, const uint &nFtrsExist, const uint &nFtrsNew, const Descriptor* const &descsNew);

private:
	
	std::vector<Octave> m_octaves;
	std::vector<TextureGL4> m_ftrTexs, m_descTexs, m_ftrHistTexs;
	TextureGL1 m_idxTex1;
	TextureGL2 m_idxTex2;
	TextureGL4 m_tmpTex;
	ushort m_widthInit, m_heightInit, m_widthInitPacked, m_heightInitPacked, m_nOctaves, m_nLevelsDoG, m_nLevels, m_bufferSize;
	ushort m_maxNumFtrsExtracted, m_maxNumFtrsAdditional, m_minDist;
	ushort m_ftrTexWidth, m_ftrTexWidthLog, m_descTexWidth, m_descTexWidthLog, m_idxTexWidth, m_idxTexWidthLog;
	bool m_upSample;

	std::vector<uint> m_nFtrsTotal;
	std::vector<std::vector<uint> > m_nFtrsOctave;
	std::vector<Table<uint, ushort> > m_nFtrsLevel;

	std::vector<FeatureSift> m_ftrs;
	std::vector<std::pair<float, uint> > m_iFtrsScored;
	FillMap m_fillMap;
	std::vector<bool> m_ftrMarks;
	std::vector<float> m_idxs1, m_idxs2;
	std::vector<uint> m_iFtrs;
	LA::Matrix3f m_H;

	ProgramGLUpSample								m_programUpSample;
	ProgramGLDownSample								m_programDownSamle;
	ProgramGLFilterPacked							m_programFilter;
	ProgramGLGradientMagnitudeDirectionPacked		m_programGradient;
	ProgramGLDoG									m_programDoG;
	ProgramGLDoGExtremum							m_programDoGExtremum;
	ProgramGLFeatureHistogramInitialize				m_programFtrHistInitialize;
	ProgramGLFeatureHistogramReduce					m_programFtrHistReduce;
	ProgramGLFeatureInitialize						m_programFtrInitialize;
	ProgramGLFeatureLocalize						m_programFtrLocalize;
	ProgramGLFeatureFinalize						m_programFtrFinalize;
	ProgramGLFeatureRearrange						m_programFtrRearrange;
	ProgramGLFeatureCopy							m_programFtrCopy;
	ProgramGLFeatureSelect							m_programFtrSelect;
	ProgramGLAssignOrientation						m_programAssignOrientation;
	ProgramGLDescriptorGenerate						m_programDescGenerate;
	ProgramGLDescriptorRearrange					m_programDescRearrange;
	ProgramGLDescriptorNormalize					m_programDescNormalize;
	ProgramGLDescriptorCopy							m_programDescCopy;
	ProgramGLDescriptorSelect						m_programDescSelect;

	float m_dogTh, m_edgeTh;


#if FE_PRINT_TIMING == 1
	Timer m_timer;
#elif FE_PRINT_TIMING == 2
	std::vector<Timer> m_timerOctave;
#elif FE_PRINT_TIMING == 3
	Table<Timer> m_timerLevel;
#endif
};

#endif