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

#include "stdafx.h"
#include "FeatureExtractorSift.h"

#if FE_PRINT_TIMING
#define FE_TIMING_GAUSS_PYRAMID         0
#define FE_TIMING_FEATURE_LOCALIZE      1
#define FE_TIMING_ORIENTATION           2
#define FE_TIMING_DESCRIPTOR_GENERATE   3
#define FE_TIMING_NON_MAXIMAL_SUPPRESS  4
#define FE_TIMING_FEATURE_REARRANGE     5
#define FE_TIMING_FEATURE_DOWNLOAD      6
#define FE_TIMING_DESCRIPTOR_REARRANGE  7
#define FE_TIMING_DESCRIPTOR_NORMALIZE  8
#define FE_TIMING_DESCRIPTOR_DOWNLOAD   9
#define FE_TIMING_TOTAL                 10
#define FE_TIMING_RECORDS_NUMBER        11
#endif

static inline void InitializeTextures(const ushort nLevels, const ushort &width, const ushort &height, std::vector<TextureGL4> &texs) {
    texs.resize(nLevels);
    for(ushort iLevel = 0; iLevel < nLevels; ++iLevel)
        texs[iLevel].Generate(width, height);
}

static inline void InitializeTextures(const ushort &nLevels, const ushort &width, const ushort &height, std::vector<TextureGL4> &texs1,
                                      std::vector<TextureGL4> &texs2) {
    texs1.resize(nLevels);
    texs2.resize(nLevels);
    for(ushort iLevel = 0; iLevel < nLevels; ++iLevel) {
        texs1[iLevel].Generate(width, height);
        texs2[iLevel].Generate(width, height);
    }
}

void FeatureExtractorSift::Octave::Initialize(const ushort &widthPacked, const ushort &heightPacked, const ushort &nLevelsDoG, const ushort &maxNumFtrs,
        const ushort &ftrTexWidth) {
    m_tmpTex.Generate(widthPacked, heightPacked);
    InitializeTextures(nLevelsDoG + 3, widthPacked, heightPacked, m_gaussTexs);
    InitializeTextures(nLevelsDoG + 2, widthPacked, heightPacked, m_dogTexs);
    InitializeTextures(nLevelsDoG, widthPacked, heightPacked, m_gradMagTexs, m_gradDirTexs);
    InitializeTextures(nLevelsDoG, widthPacked, heightPacked, m_extremeTexs);
    const ushort ftrTexHeight = (maxNumFtrs + ftrTexWidth - 1) / ftrTexWidth, descTexWidth = (ftrTexWidth << 4), descTexHeight = ftrTexHeight;
    InitializeTextures(nLevelsDoG, ftrTexWidth, ftrTexHeight, m_ftrTexs);
    InitializeTextures(nLevelsDoG, descTexWidth, descTexHeight, m_descTexs);
}

FeatureExtractorSift::FeatureExtractorSift() {
}

FeatureExtractorSift::~FeatureExtractorSift() {
}

void FeatureExtractorSift::Initialize(const ushort &widthInit, const ushort &heightInit, const ushort &bufferSize, const ushort &maxNumFtrsExtracted,
                                      const ushort &maxNumFtrsAdditional, const ushort &ftrTexWidth, const ushort &nOctaves, const ushort &nLevelsDoG,
                                      const bool &upSample, const float &dogTh, const float &edgeTh, const float &hessianTh, const ushort &minDist) {
    m_widthInit = widthInit;
    m_heightInit = heightInit;
    m_nOctaves = nOctaves;
    m_nLevelsDoG = nLevelsDoG;
    m_nLevels = nLevelsDoG + 3;
    m_bufferSize = bufferSize;
    m_maxNumFtrsExtracted = maxNumFtrsExtracted;
    m_maxNumFtrsAdditional = maxNumFtrsAdditional;
    m_upSample = upSample;
    m_minDist = minDist;

    m_widthInitPacked = ((widthInit + 1) >> 1);
    m_heightInitPacked = ((heightInit + 1) >> 1);
    if(upSample) {
        m_widthInitPacked = (m_widthInitPacked << 1);
        m_heightInitPacked = (m_heightInitPacked << 1);
    }

    const ushort ftrTexHeight = (maxNumFtrsExtracted + m_maxNumFtrsAdditional + ftrTexWidth - 1) / ftrTexWidth;
    const ushort descTexWidth = (ftrTexWidth << 4), descTexHeight = ftrTexHeight;
    const ushort idxTexWidth = ftrTexWidth, idxTexHeight = ftrTexHeight;
    m_ftrTexWidth = ftrTexWidth;
    m_ftrTexWidthLog = ushort(log(float(ftrTexWidth)) / log(2.0f));
    m_descTexWidth = descTexWidth;
    m_descTexWidthLog = m_ftrTexWidthLog + 4;
    m_idxTexWidth = idxTexWidth;
    m_idxTexWidthLog = m_ftrTexWidthLog;
#if _DEBUG
    assert(ftrTexWidth <= m_widthInitPacked);
    assert((1 << m_ftrTexWidthLog) == ftrTexWidth);
    assert((1 << m_descTexWidthLog) == descTexWidth);
#endif
    const ushort bufferSize_p1 = bufferSize + 1;
    m_ftrTexs.resize(bufferSize_p1);
    m_descTexs.resize(bufferSize_p1);
    for(ushort iBuffer = 0; iBuffer < bufferSize_p1; ++iBuffer) {
        m_ftrTexs[iBuffer].Generate(ftrTexWidth, ftrTexHeight);
        m_descTexs[iBuffer].Generate(descTexWidth, descTexHeight);
    }
    m_idxTex1.Generate(idxTexWidth, idxTexHeight);
    m_idxTex2.Generate(idxTexWidth, idxTexHeight);
    m_tmpTex.Generate();

    m_octaves.resize(nOctaves);
    ushort widthPacked = m_widthInitPacked, heightPacked = m_heightInitPacked;
    std::vector<LA::Vector2us> octaveSizes(nOctaves);
    for(ushort iOctave = 0; iOctave < nOctaves; ++iOctave, widthPacked = ((widthPacked + 1) >> 1), heightPacked = ((heightPacked + 1) >> 1)) {
        m_octaves[iOctave].Initialize(widthPacked, heightPacked, nLevelsDoG, maxNumFtrsExtracted, ftrTexWidth);
        octaveSizes[iOctave].Set(widthPacked, heightPacked);
    }
    ushort nLevelsFtrHist;
    for(nLevelsFtrHist = nOctaves - 1; widthPacked != 1 || heightPacked != 1;
            ++nLevelsFtrHist, widthPacked = ((widthPacked + 1) >> 1), heightPacked = ((heightPacked + 1) >> 1));
    ++nLevelsFtrHist;
    widthPacked = m_octaves[1].GetPackedWidth();
    heightPacked = m_octaves[1].GetPackedHeight();
    m_ftrHistTexs.resize(nLevelsFtrHist);
    for(ushort iLevelFtrHist = 0; iLevelFtrHist < nLevelsFtrHist; ++iLevelFtrHist, widthPacked = ((widthPacked + 1) >> 1), heightPacked = ((heightPacked + 1) >> 1))
        m_ftrHistTexs[iLevelFtrHist].Generate(widthPacked, heightPacked);

    const float sigmak = powf(2.0f, 1.0f / nLevelsDoG);
    const float sigma0 = 1.6f * sigmak;
    const float dSigma = sigma0 * sqrt(1.0f - 1.0f / (sigmak * sigmak));
    std::vector<float> sigmas;
    sigmas.resize(nLevelsDoG);
    for(ushort iLevelDoG = 0; iLevelDoG < nLevelsDoG; ++iLevelDoG)
        sigmas[iLevelDoG] = sigma0 * powf(2.0f, float(iLevelDoG) / float(nLevelsDoG));

    m_programUpSample.Initialize();
    m_programDownSamle.Initialize();
    m_programFilter.Initialize(m_nLevels, dSigma, sigmak, upSample);
    m_programGradient.Initialize();
    m_programDoG.Initialize();
    m_programDoGExtremum.Initialize(octaveSizes, dogTh / nLevelsDoG, edgeTh);
    m_programFtrHistInitialize.Initialize();
    m_programFtrHistReduce.Initialize();
    m_programFtrInitialize.Initialize(ftrTexWidth);
    m_programFtrLocalize.Initialize(ftrTexWidth);
    m_programFtrFinalize.Initialize(ftrTexWidth, sigmas, sigmak);
    m_programFtrRearrange.Initialize(ftrTexWidth, nOctaves, upSample);
    m_programFtrCopy.Initialize(ftrTexWidth);
    m_programAssignOrientation.Initialize(ftrTexWidth, octaveSizes);
    m_programDescGenerate.Initialize(ftrTexWidth, octaveSizes);
    m_programDescRearrange.Initialize(descTexWidth);
    m_programDescNormalize.Initialize(descTexWidth);
    m_programDescCopy.Initialize(descTexWidth);

    m_nFtrsTotal.resize(bufferSize_p1);
    m_nFtrsOctave.resize(bufferSize_p1);
    m_nFtrsLevel.resize(bufferSize_p1);
    for(ushort iBuffer = 0; iBuffer < bufferSize_p1; ++iBuffer) {
        m_nFtrsOctave[iBuffer].resize(nOctaves);
        m_nFtrsLevel[iBuffer].Resize(nOctaves, nLevelsDoG);
    }

    m_fillMap.Resize(widthInit, heightInit);
    //m_idxs1.reserve(maxNumFtrs + 3);
    //m_idxs2.reserve(maxNumFtrs + 3);
    m_iFtrs.resize(nOctaves * nLevelsDoG + 1);

    m_programFtrSelect.Initialize(ftrTexWidth);
    m_programDescSelect.Initialize(descTexWidth);

#if FE_PRINT_TIMING == 1
    m_timer.Resize(FE_TIMING_RECORDS_NUMBER);
#elif FE_PRINT_TIMING == 2
    m_timerOctave.resize(nOctaves);
    for(ushort iOctave = 0; iOctave < nOctaves; ++iOctave)
        m_timerOctave[iOctave].Resize(FE_TIMING_RECORDS_NUMBER);
#elif FE_PRINT_TIMING == 3
    m_timerLevel.Resize(nOctaves, nLevelsDoG);
    for(ushort iOctave = 0; iOctave < nOctaves; ++iOctave)
        for(ushort iLevelDoG = 0; iLevelDoG < nLevelsDoG; ++iLevelDoG)
            m_timerLevel[iOctave][iLevelDoG].Resize(FE_TIMING_RECORDS_NUMBER);
#endif
}

void FeatureExtractorSift::ExtractFeatures(const TextureGL1 &inpTex, const ushort &iBuffer) {
#if FE_PRINT_TIMING == 1
    Timer &timer = m_timer;
#elif FE_PRINT_TIMING == 2
    Timer &timer = m_timerOctave[0];
#elif FE_PRINT_TIMING == 3
    Timer &timer = m_timerLevel[0][0];
#endif

#if FE_PRINT_TIMING
    timer.Start(FE_TIMING_TOTAL);
#endif

    BuildGaussianPyramid(inpTex);
    LocalizeFeatures(iBuffer);
    if(m_nFtrsTotal[iBuffer] > m_maxNumFtrsExtracted)
        NonMaximalSuppress(iBuffer);
    //PrintFeaturesNumber(iBuffer);

    AssignOrientations(iBuffer);
    GenerateDescriptors(iBuffer);

#if FE_PRINT_TIMING
    timer.Stop(FE_TIMING_TOTAL);
#endif
}

void FeatureExtractorSift::SelectFeatures(const ushort &iBufferSrc, const ushort &iBufferDst, const std::vector<ushort> &idxs) {
#if _DEBUG
    assert(iBufferSrc != iBufferDst);
#endif

    const uint nFtrsDst = uint(idxs.size());
    m_idxs1.resize(nFtrsDst);

    //PrintFeaturesNumber(iBufferSrc);

#if 1
    m_idxs1.resize(nFtrsDst);
    for(uint i = 0; i < nFtrsDst; ++i)
        m_idxs1[i] = float(idxs[i]);
    m_nFtrsTotal[iBufferDst] = nFtrsDst;
    m_nFtrsOctave[iBufferDst].assign(m_nFtrsOctave[iBufferDst].size(), 0);
    m_nFtrsLevel[iBufferDst].SetZero();
#else
#if _DEBUG
    for(uint i = 1; i < nFtrsDst; ++i)
        assert(idxs[i - 1] < idxs[i]);
#endif

    uint iFtrDst = 0;
    for(ushort iOctave = 0, iFtrSrcEnd = 0; iOctave < m_nOctaves; ++iOctave) {
        uint nFtrsOctaveDst = 0;
        for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG) {
            iFtrSrcEnd += m_nFtrsLevel[iBufferSrc][iOctave][iLevelDoG];
            uint nFtrsLevelDst = 0;
            while(iFtrDst != nFtrsDst && idxs[iFtrDst] < iFtrSrcEnd) {
                ++nFtrsLevelDst;
                m_idxs1[iFtrDst] = float(idxs[iFtrDst]);
                ++iFtrDst;
            }
            m_nFtrsLevel[iBufferDst][iOctave][iLevelDoG] = nFtrsLevelDst;
            nFtrsOctaveDst += nFtrsLevelDst;
        }
        m_nFtrsOctave[iBufferDst][iOctave] = nFtrsOctaveDst;
        //nFtrsDst += nFtrsOctaveDst;
    }
    m_nFtrsTotal[iBufferDst] = iFtrDst;
#endif

    //PrintFeaturesNumber(iBufferDst);

    UploadIndexesFromCPU(m_idxTex1, nFtrsDst, m_idxs1.data());
    SelectFeatures(m_ftrTexs[iBufferSrc], m_ftrTexs[iBufferDst], m_idxTex1, m_nFtrsTotal[iBufferSrc], m_nFtrsTotal[iBufferDst]);
}

void FeatureExtractorSift::SelectFeatures(const ushort &iBufferSrc1, const ushort &iBufferSrc2, const ushort &iBufferDst,
        const std::vector<Match<ushort> > &matches) {
    const ushort nMatches = ushort(matches.size()), N = (nMatches << 1);
    m_idxs1.resize(N);
    for(ushort i = 0, j = 0; i < nMatches; ++i) {
        m_idxs1[j++] = float(matches[i].GetIndex1());
        m_idxs1[j++] = float(matches[i].GetIndex2());
    }
    UploadIndexesFromCPU(m_idxTex2, nMatches, m_idxs1.data());
    SelectFeatures(m_ftrTexs[iBufferSrc1], m_ftrTexs[iBufferSrc2], m_ftrTexs[iBufferDst], m_idxTex2, nMatches);
}

void FeatureExtractorSift::SelectDescriptors(const ushort &iBufferSrc, const ushort &iBufferDst, const std::vector<ushort> &idxs) {
#if _DEBUG
    assert(iBufferSrc != iBufferDst);
#endif

    const uint nFtrsDst = uint(idxs.size());
    m_idxs1.resize(nFtrsDst);

    //PrintFeaturesNumber(iBufferSrc);

    m_idxs1.resize(nFtrsDst);
    for(uint i = 0; i < nFtrsDst; ++i)
        m_idxs1[i] = float(idxs[i]);
    m_nFtrsTotal[iBufferDst] = nFtrsDst;
    m_nFtrsOctave[iBufferDst].assign(m_nFtrsOctave[iBufferDst].size(), 0);
    m_nFtrsLevel[iBufferDst].SetZero();

    UploadIndexesFromCPU(m_idxTex1, nFtrsDst, m_idxs1.data());
    SelectDescriptors(m_descTexs[iBufferSrc], m_descTexs[iBufferDst], m_idxTex1, m_nFtrsTotal[iBufferSrc], m_nFtrsTotal[iBufferDst]);
}

void FeatureExtractorSift::SelectFeaturesAndDescriptors(const ushort &iBufferSrc, const ushort &iBufferDst, const std::vector<ushort> &idxs) {
#if _DEBUG
    assert(idxs.back() < m_nFtrsTotal[iBufferSrc]);
#endif
    SelectFeatures(iBufferSrc, iBufferDst, idxs);
    SelectDescriptors(m_descTexs[iBufferSrc], m_descTexs[iBufferDst], m_idxTex1, m_nFtrsTotal[iBufferSrc], m_nFtrsTotal[iBufferDst]);
}

#if FE_PRINT_TIMING
void FeatureExtractorSift::PrintTiming(const char *str, const ushort &iRecord) {
#if FE_PRINT_TIMING == 1
    printf("%s", str);
    m_timer.PrintTotalAndStableMeanTiming(iRecord);
#elif FE_PRINT_TIMING == 2
    printf("%s\n", str);
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
        printf("    Octave %d: ", iOctave);
        m_timerOctave[iOctave].PrintTotalAndStableMeanTiming(iRecord);
    }
#elif FE_PRINT_TIMING == 3
    printf("%s\n", str);
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave)
        for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG) {
            printf("    Octave %d, level %d: ", iOctave, iLevelDoG);
            m_timerLevel[iOctave][0].PrintTotalAndStableMeanTiming(iRecord);
        }
#endif
}
void FeatureExtractorSift::PrintTiming() {
    printf("[FeatureExtractorSift]\n");

#if FE_PRINT_TIMING == 1
    printf("  FE_TIMING_GAUSS_PYRAMID:          ");
    m_timer.PrintTotalAndStableMeanTiming(FE_TIMING_GAUSS_PYRAMID);
#elif FE_PRINT_TIMING == 2
    printf("  FE_TIMING_GAUSS_PYRAMID:\n");
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
        printf("    Octave %d: ", iOctave);
        m_timerOctave[iOctave].PrintTotalAndStableMeanTiming(FE_TIMING_GAUSS_PYRAMID);
    }
#elif FE_PRINT_TIMING == 3
    printf("  FE_TIMING_GAUSS_PYRAMID:\n");
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
        printf("    Octave %d: ", iOctave);
        m_timerLevel[iOctave][0].PrintTotalAndStableMeanTiming(FE_TIMING_GAUSS_PYRAMID);
    }
#endif

#if FE_PRINT_TIMING == 1
    Timer &timer = m_timer;
#elif FE_PRINT_TIMING == 2
    Timer &timer = m_timerOctave[0];
#elif FE_PRINT_TIMING == 3
    Timer &timer = m_timerLevel[0][0];
#endif
    PrintTiming("  FE_TIMING_FEATURE_LOCALIZE:     ", FE_TIMING_FEATURE_LOCALIZE);
    PrintTiming("  FE_TIMING_ORIENTATION:          ", FE_TIMING_ORIENTATION);
    PrintTiming("  FE_TIMING_DESCRIPTOR_GENERATE:  ", FE_TIMING_DESCRIPTOR_GENERATE);
    printf("  FE_TIMING_NON_MAXIMAL_SUPPRESS: ");
    timer.PrintTotalAndStableMeanTiming(FE_TIMING_NON_MAXIMAL_SUPPRESS);
    printf("  FE_TIMING_FEATURE_REARRANGE:    ");
    timer.PrintTotalAndStableMeanTiming(FE_TIMING_FEATURE_REARRANGE);
    printf("  FE_TIMING_DESCRIPTOR_REARRANGE: ");
    timer.PrintTotalAndStableMeanTiming(FE_TIMING_DESCRIPTOR_REARRANGE);
    printf("  FE_TIMING_DESCRIPTOR_NORMALIZE: ");
    timer.PrintTotalAndStableMeanTiming(FE_TIMING_DESCRIPTOR_NORMALIZE);
    printf("  FE_TIMING_FEATURE_DOWNLOAD:     ");
    timer.PrintTotalAndStableMeanTiming(FE_TIMING_FEATURE_DOWNLOAD);
    printf("  FE_TIMING_DESCRIPTOR_DOWNLOAD:  ");
    timer.PrintTotalAndStableMeanTiming(FE_TIMING_DESCRIPTOR_DOWNLOAD);
    printf("  FE_TIMING_TOTAL:                ");
    timer.PrintTotalAndStableMeanTiming(FE_TIMING_TOTAL);
}
#endif

void FeatureExtractorSift::BuildGaussianPyramid(const TextureGL1 &inpTex) {
#if FE_PRINT_TIMING == 1
    m_timer.Start(FE_TIMING_GAUSS_PYRAMID);
#endif

    ProgramGL::FitViewportGL(m_widthInitPacked, m_heightInitPacked);
    //glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_EQUAL);
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
#if FE_PRINT_TIMING == 2
        m_timerOctave[iOctave].Start(FE_TIMING_GAUSS_PYRAMID);
#elif FE_PRINT_TIMING == 3
        m_timerLevel[iOctave][0].Start(FE_TIMING_GAUSS_PYRAMID);
#endif

        const Octave &octave = m_octaves[iOctave];
        //ProgramGL::FitViewportGL(octave.GetPackedWidth(), octave.GetPackedHeight());
        //ProgramGL::AttachDepthMaskTexture(octave.GetMaskTexture(iCam));
        if(iOctave == 0) {
            if(m_upSample)
                m_programUpSample.Run(inpTex, octave.GetGaussianTexture(0));
            else
                m_programDownSamle.Run(inpTex, octave.GetGaussianTexture(0));
            m_programFilter.Run(0, octave.GetGaussianTexture(0), octave.GetGaussianTexture(0), octave.GetTemporaryTexture());
        } else
            m_programDownSamle.Run(m_octaves[iOctave - 1].GetGaussianTexture(m_nLevelsDoG), octave.GetGaussianTexture(0));

        const ushort nLevels = m_nLevelsDoG + 3;
        for(ushort iGauss0 = 0, iGauss1 = 1; iGauss1 < nLevels; ++iGauss0, ++iGauss1)
            m_programFilter.Run(iGauss1, octave.GetGaussianTexture(iGauss0), octave.GetGaussianTexture(iGauss1), octave.GetTemporaryTexture());
        for(ushort iGauss0 = 0, iGauss1 = 1, iDoG = 0; iGauss1 < nLevels; ++iGauss0, ++iGauss1, ++iDoG)
            m_programDoG.Run(octave.GetGaussianTexture(iGauss0), octave.GetGaussianTexture(iGauss1), octave.GetDoGTexture(iDoG));
        for(ushort iGauss = 1, iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iGauss, ++iLevelDoG)
            m_programGradient.Run(octave.GetGaussianTexture(iGauss), octave.GetGradientMagnitudeTexture(iLevelDoG), octave.GetGradientDirectionTexture(iLevelDoG));
        for(ushort iLevelDoG = 0, iDoG0 = 0, iDoG1 = 1, iDoG2 = 2; iLevelDoG < m_nLevelsDoG; ++iLevelDoG, ++iDoG0, ++iDoG1, ++iDoG2) {
            //m_programClearColorDepth.Run(octave.GetExtremeTexture(iLevelDoG));
            m_programDoGExtremum.Run(iOctave, octave.GetDoGTexture(iDoG0), octave.GetDoGTexture(iDoG1), octave.GetDoGTexture(iDoG2)/*, octave.GetMaskTexture(iCam)*/,
                                     octave.GetExtremeTexture(iLevelDoG));
        }

        //ProgramGL::DetachDepthMaskTexture(octave.GetMaskTexture(iCam));

#if FE_PRINT_TIMING == 2
        m_timerOctave[iOctave].Stop(FE_TIMING_GAUSS_PYRAMID);
#elif FE_PRINT_TIMING == 3
        m_timerLevel[iOctave][0].Stop(FE_TIMING_GAUSS_PYRAMID);
#endif
    }
    //glDisable(GL_DEPTH_TEST);

#if FE_PRINT_TIMING == 1
    m_timer.Stop(FE_TIMING_GAUSS_PYRAMID);
#endif
}

void FeatureExtractorSift::LocalizeFeatures(const ushort &iBuffer) {
    uint nFtrsTotal = 0;
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
        /*const */Octave &octave = m_octaves[iOctave];
        uint nFtrsOctave = 0;
        for(ushort iLevelDoG = 0, iDoG1 = 1; iLevelDoG < m_nLevelsDoG; ++iLevelDoG, ++iDoG1) {
            const uint nFtrs = LocalizeFeatures(iOctave, iLevelDoG, octave.GetDoGTexture(iDoG1),
                                                octave.GetExtremeTexture(iLevelDoG), octave.GetFeatureTexture(iLevelDoG));
            m_nFtrsLevel[iBuffer][iOctave][iLevelDoG] = nFtrs;
            nFtrsOctave += nFtrs;
        }
        m_nFtrsOctave[iBuffer][iOctave] = nFtrsOctave;
        nFtrsTotal += nFtrsOctave;
    }
    m_nFtrsTotal[iBuffer] = nFtrsTotal;

}

uint FeatureExtractorSift::LocalizeFeatures(const ushort &iOctave, const ushort &iLevelDoG, const TextureGL4 &dogTex,
        const TextureGL4 &extremeTex, /*const */TextureGL4 &ftrTex) {
    m_programFtrHistInitialize.Run(extremeTex, m_ftrHistTexs[iOctave]);
    const ushort nLevelsFtrHist = ushort(m_ftrHistTexs.size());
    for(ushort i0 = iOctave, i1 = i0 + 1; i1 < nLevelsFtrHist; ++i0, ++i1) {
        m_programFtrHistReduce.Run(m_ftrHistTexs[i0], m_ftrHistTexs[i1]);
    }

    LA::Vector4f hist;
    m_ftrHistTexs.back().DownloadToCPU((float *) hist);
    const uint nFtrs = uint(hist.v0() + hist.v1() + hist.v2() + hist.v3());
    if(nFtrs == 0)
        return 0;
    else if(nFtrs >= ftrTex.GetPixelsNumber()) {
        const ushort ftrTexHeight = (nFtrs + m_ftrTexWidth - 1) / m_ftrTexWidth;
        ftrTex.Bind();
        ftrTex.Resize(m_ftrTexWidth, ftrTexHeight);
    }

    //ProgramGL::FitViewportGL(m_ftrTexWidth, (nFtrs + m_ftrTexWidth - 1) >> m_ftrTexWidthLog);
    m_programFtrInitialize.Run(hist, ftrTex, nFtrs);
    for(ushort iLevelFtrHist = nLevelsFtrHist - 2; iLevelFtrHist > iOctave; --iLevelFtrHist)
        m_programFtrLocalize.Run(m_ftrHistTexs[iLevelFtrHist], ftrTex, nFtrs);
    m_programFtrFinalize.Run(iLevelDoG, m_ftrHistTexs[iOctave], dogTex, extremeTex, ftrTex, nFtrs);

    return nFtrs;
}

void FeatureExtractorSift::NonMaximalSuppress(const ushort &iBuffer) {
#if FE_PRINT_TIMING == 1
    Timer &timer = m_timer;
#elif FE_PRINT_TIMING == 2
    Timer &timer = m_timerOctave[0];
#elif FE_PRINT_TIMING == 3
    Timer &timer = m_timerLevel[0][0];
#endif

#if FE_PRINT_TIMING
    timer.Start(FE_TIMING_NON_MAXIMAL_SUPPRESS);
#endif

    m_ftrs.resize(m_nFtrsTotal[iBuffer]);
    m_iFtrsScored.resize(m_nFtrsTotal[iBuffer]);
    for(ushort iOctave = 0, iFtrStart = 0; iOctave < m_nOctaves; ++iOctave) {
        const Octave &octave = m_octaves[iOctave];
        for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG) {
            const uint nFtrs = m_nFtrsLevel[iBuffer][iOctave][iLevelDoG];
            if(nFtrs == 0)
                continue;
            //m_programFtrRearrange.Run(FRM_LEVEL_TO_GLOBAL, iOctave, octave.GetFeatureTexture(iLevelDoG), m_ftrTexs[iBuffer], 0, nFtrs);
            if(nFtrs > m_tmpTex.GetPixelsNumber()) {
                m_tmpTex.Bind();
                m_tmpTex.Resize(octave.GetFeatureTexture(iLevelDoG).GetWidth(), octave.GetFeatureTexture(iLevelDoG).GetHeight());
            }
            m_programFtrRearrange.Run(FRM_LEVEL_TO_GLOBAL, iOctave, octave.GetFeatureTexture(iLevelDoG), m_tmpTex, 0, nFtrs);

            FeatureSift *ftrs = m_ftrs.data() + iFtrStart;
            std::pair<float, uint> *iFtrsScored = m_iFtrsScored.data() + iFtrStart;
            //DownloadFeaturesToCPU(m_ftrTexs[iBuffer], nFtrs, ftrs);
            DownloadFeaturesToCPU(m_tmpTex, nFtrs, ftrs);
            for(uint iFtr = 0; iFtr < nFtrs; ++iFtr)
                iFtrsScored[iFtr] = std::make_pair(fabs(ftrs[iFtr].d()), iFtrStart + iFtr);
            //iFtrsScored[iFtr] = std::make_pair(fabs(ftrs[iFtr].d() * (m_nOctaves - iOctave)), iFtrStart + iFtr);
            iFtrStart += nFtrs;
        }
    }

    std::sort(m_iFtrsScored.begin(), m_iFtrsScored.end(), std::greater<std::pair<float, uint> >());
    MarkMaximalFeatures(m_ftrs.data(), m_iFtrsScored, m_maxNumFtrsExtracted, m_ftrMarks);

    uint nFtrsTotal = 0, iFtrSrc = 0;
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
        const Octave &octave = m_octaves[iOctave];
        uint nFtrsOctaveDst = 0;
        for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG) {
            m_idxs1.resize(0);
            const uint nFtrsLevelSrc = m_nFtrsLevel[iBuffer][iOctave][iLevelDoG];
            for(uint i = 0; i < nFtrsLevelSrc; ++i, ++iFtrSrc) {
                if(m_ftrMarks[iFtrSrc])
                    m_idxs1.push_back(float(i));
            }
            const uint nFtrsLevelDst = uint(m_idxs1.size());
            m_nFtrsLevel[iBuffer][iOctave][iLevelDoG] = nFtrsLevelDst;
            nFtrsOctaveDst += nFtrsLevelDst;

            UploadIndexesFromCPU(m_idxTex1, nFtrsLevelDst, m_idxs1.data());
            SelectFeatures(octave.GetFeatureTexture(iLevelDoG), octave.GetFeatureTexture(iLevelDoG), m_idxTex1, nFtrsLevelSrc, nFtrsLevelDst);
        }
        m_nFtrsOctave[iBuffer][iOctave] = nFtrsOctaveDst;
        nFtrsTotal += nFtrsOctaveDst;
    }
    m_nFtrsTotal[iBuffer] = nFtrsTotal;

#if FE_PRINT_TIMING
    timer.Stop(FE_TIMING_NON_MAXIMAL_SUPPRESS);
#endif
}

void FeatureExtractorSift::AssignOrientations(const ushort &iBuffer) {
#if FE_PRINT_TIMING == 1
    m_timer.Start(FE_TIMING_ORIENTATION);
#endif
    ProgramGL::FitViewportGL(m_ftrTexs[iBuffer]);
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
#if FE_PRINT_TIMING == 2
        m_timerOctave[iOctave].Start(FE_TIMING_ORIENTATION);
#endif
        const Octave &octave = m_octaves[iOctave];
        for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG) {
            const uint nFtrs = m_nFtrsLevel[iBuffer][iOctave][iLevelDoG];
            if(nFtrs == 0)
                continue;
#if FE_PRINT_TIMING == 3
            m_timerLevel[iOctave][iLevelDoG].Start(FE_TIMING_ORIENTATION);
#endif
            //ProgramGL::FitViewportGL(m_ftrTexWidth, (nFtrs + m_ftrTexWidth - 1) >> m_ftrTexWidthLog);
            m_programAssignOrientation.Run(iOctave, octave.GetFeatureTexture(iLevelDoG), octave.GetGradientMagnitudeTexture(iLevelDoG),
                                           octave.GetGradientDirectionTexture(iLevelDoG), nFtrs);
#if FE_PRINT_TIMING == 3
            m_timerLevel[iOctave][iLevelDoG].Stop(FE_TIMING_ORIENTATION);
#endif
        }
#if FE_PRINT_TIMING == 2
        m_timerOctave[iOctave].Stop(FE_TIMING_ORIENTATION);
#endif
    }
#if FE_PRINT_TIMING == 1
    m_timer.Stop(FE_TIMING_ORIENTATION);
#endif

#if FE_PRINT_TIMING == 1
    m_timer.Start(FE_TIMING_FEATURE_REARRANGE);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Start(FE_TIMING_FEATURE_REARRANGE);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Start(FE_TIMING_FEATURE_REARRANGE);
#endif

    RearrangeFeaturesLevelToGlobal(iBuffer);

#if FE_PRINT_TIMING == 1
    m_timer.Stop(FE_TIMING_FEATURE_REARRANGE);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Stop(FE_TIMING_FEATURE_REARRANGE);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Stop(FE_TIMING_FEATURE_REARRANGE);
#endif
}

void FeatureExtractorSift::GenerateDescriptors(const ushort &iBuffer) {
#if FE_PRINT_TIMING == 1
    m_timer.Start(FE_TIMING_DESCRIPTOR_GENERATE);
#endif
    ProgramGL::FitViewportGL(m_descTexs[iBuffer]);
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
#if FE_PRINT_TIMING == 2
        m_timerOctave[iOctave].Start(FE_TIMING_DESCRIPTOR_GENERATE);
#endif
        const Octave &octave = m_octaves[iOctave];
        for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG) {
            const uint nFtrs = m_nFtrsLevel[iBuffer][iOctave][iLevelDoG];
            if(nFtrs == 0)
                continue;
#if FE_PRINT_TIMING == 3
            m_timerLevel[iOctave][iLevelDoG].Start(FE_TIMING_DESCRIPTOR_GENERATE);
#endif
            //ProgramGL::FitViewportGL(m_descTexWidth, ((nFtrs << 4) + m_descTexWidth - 1) >> m_descTexWidthLog);
            m_programDescGenerate.Run(iOctave, octave.GetFeatureTexture(iLevelDoG), octave.GetGradientMagnitudeTexture(iLevelDoG),
                                      octave.GetGradientDirectionTexture(iLevelDoG), octave.GetDescriptorTexture(iLevelDoG), nFtrs);
#if FE_PRINT_TIMING == 3
            m_timerLevel[iOctave][iLevelDoG].Stop(FE_TIMING_DESCRIPTOR_GENERATE);
#endif
        }
#if FE_PRINT_TIMING == 2
        m_timerOctave[iOctave].Stop(FE_TIMING_DESCRIPTOR_GENERATE);
#endif
    }
#if FE_PRINT_TIMING == 1
    m_timer.Stop(FE_TIMING_DESCRIPTOR_GENERATE);
#endif

#if FE_PRINT_TIMING == 1
    m_timer.Start(FE_TIMING_DESCRIPTOR_REARRANGE);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Start(FE_TIMING_DESCRIPTOR_REARRANGE);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Start(FE_TIMING_DESCRIPTOR_REARRANGE);
#endif
    RearrangeDescriptorsLevelToGlobal(iBuffer);
#if FE_PRINT_TIMING == 1
    m_timer.Stop(FE_TIMING_DESCRIPTOR_REARRANGE);
    m_timer.Start(FE_TIMING_DESCRIPTOR_NORMALIZE);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Stop(FE_TIMING_DESCRIPTOR_REARRANGE);
    m_timerOctave[0].Start(FE_TIMING_DESCRIPTOR_NORMALIZE);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Stop(FE_TIMING_DESCRIPTOR_REARRANGE);
    m_timerLevel[0][0].Start(FE_TIMING_DESCRIPTOR_NORMALIZE);
#endif
    m_programDescNormalize.Run(m_descTexs[iBuffer], m_idxTex1, m_nFtrsTotal[iBuffer], true);
#if FE_PRINT_TIMING == 1
    m_timer.Stop(FE_TIMING_DESCRIPTOR_NORMALIZE);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Stop(FE_TIMING_DESCRIPTOR_NORMALIZE);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Stop(FE_TIMING_DESCRIPTOR_NORMALIZE);
#endif
}

void FeatureExtractorSift::RearrangeFeaturesGlobalToLevel(const ushort &iBuffer) {
    uint iFtrStart = 0;
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
        const Octave &octave = m_octaves[iOctave];
        for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG) {
            const uint nFtrs = m_nFtrsLevel[iBuffer][iOctave][iLevelDoG];
            //ProgramGL::FitViewportGL(m_ftrTexWidth, (nFtrs + m_ftrTexWidth - 1) >> m_ftrTexWidthLog);
            if(nFtrs == 0)
                continue;
            m_programFtrRearrange.Run(FRM_GLOBAL_TO_LEVEL, iOctave, m_ftrTexs[iBuffer], octave.GetFeatureTexture(iLevelDoG), iFtrStart, nFtrs);
            iFtrStart += nFtrs;
        }
    }
}

void FeatureExtractorSift::RearrangeDescriptorsLevelToGlobal(const ushort &iBuffer) {
    //ProgramGL::FitViewportGL(m_descTexWidth, ((m_nFtrs[iBuffer] << 4) + m_descTexWidth - 1) >> m_descTexWidthLog);
    uint iFtrStart = 0;
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
        const Octave &octave = m_octaves[iOctave];
        for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG) {
            const uint nFtrs = m_nFtrsLevel[iBuffer][iOctave][iLevelDoG];
            if(nFtrs == 0)
                continue;
            m_programDescRearrange.Run(DRM_LEVEL_TO_GLOBAL, octave.GetDescriptorTexture(iLevelDoG), m_descTexs[iBuffer], iFtrStart, nFtrs);
            iFtrStart += nFtrs;
        }
    }
}

void FeatureExtractorSift::RearrangeDescriptorsGlobalToLevel(const ushort &iBuffer) {
    uint iFtrStart = 0;
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
        const Octave &octave = m_octaves[iOctave];
        for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG) {
            const uint nFtrs = m_nFtrsLevel[iBuffer][iOctave][iLevelDoG];
            if(nFtrs == 0)
                continue;
            //ProgramGL::FitViewportGL(m_descTexWidth, ((nFtrs << 4) + m_descTexWidth - 1) >> m_descTexWidthLog);
            m_programDescRearrange.Run(DRM_GLOBAL_TO_LEVEL, m_descTexs[iBuffer], octave.GetDescriptorTexture(iLevelDoG), iFtrStart, nFtrs);
            iFtrStart += nFtrs;
        }
    }
}

void FeatureExtractorSift::SelectFeatures(const TextureGL4 &ftrTexSrc, const TextureGL4 &ftrTexDst, const TextureGL1 &idxTex, const uint &nFtrsSrc,
        const uint &nFtrsDst) {
    ProgramGL::FitViewportGL(ftrTexDst);
    if(ftrTexSrc.GetTexture() == ftrTexDst.GetTexture()) {
        //ProgramGL::FitViewportGL(m_ftrTexWidth, (nFtrsSrc + m_ftrTexWidth - 1) >> m_ftrTexWidthLog);
        if(nFtrsSrc > m_tmpTex.GetPixelsNumber()) {
            m_tmpTex.Bind();
            m_tmpTex.Resize(ftrTexSrc.GetWidth(), ftrTexSrc.GetHeight());
        }
        m_programFtrCopy.Run(ftrTexSrc, m_tmpTex, nFtrsSrc);
        //ProgramGL::FitViewportGL(m_ftrTexWidth, (nFtrsDst + m_ftrTexWidth - 1) >> m_ftrTexWidthLog);
        m_programFtrSelect.Run(m_tmpTex, ftrTexDst, idxTex, nFtrsDst);
    } else {
        //ProgramGL::FitViewportGL(m_ftrTexWidth, (nFtrsDst + m_ftrTexWidth - 1) >> m_ftrTexWidthLog);
        m_programFtrSelect.Run(ftrTexSrc, ftrTexDst, idxTex, nFtrsDst);
    }
}

void FeatureExtractorSift::SelectFeatures(const TextureGL4 &ftrTexSrc1, const TextureGL4 &ftrTexSrc2, const TextureGL4 &ftrTexDst, const TextureGL2 &idxTex12,
        const uint &nFtrsDst) {
    ProgramGL::FitViewportGL(ftrTexDst);
#if _DEBUG
    assert(ftrTexSrc1.GetTexture() != ftrTexDst.GetTexture() && ftrTexSrc2.GetTexture() != ftrTexDst.GetTexture());
#endif
    m_programFtrSelect.Run(ftrTexSrc1, ftrTexSrc2, ftrTexDst, idxTex12, nFtrsDst);
}

void FeatureExtractorSift::SelectDescriptors(const TextureGL4 &descTexSrc, const TextureGL4 &descTexDst, const TextureGL1 &idxTex, const uint &nFtrsSrc,
        const uint &nFtrsDst) {
    ProgramGL::FitViewportGL(descTexDst);
    if(descTexSrc.GetTexture() == descTexDst.GetTexture()) {
        //ProgramGL::FitViewportGL(m_descTexWidth, ((nFtrsSrc << 4) + m_descTexWidth - 1) >> m_descTexWidthLog);
        m_programDescCopy.Run(descTexSrc, m_descTexs[m_bufferSize], nFtrsSrc);
        //ProgramGL::FitViewportGL(m_descTexWidth, ((nFtrsDst << 4) + m_descTexWidth - 1) >> m_descTexWidthLog);
        m_programDescSelect.Run(m_descTexs[m_bufferSize], descTexDst, idxTex, nFtrsDst);
    } else {
        //ProgramGL::FitViewportGL(m_descTexWidth, ((nFtrsDst << 4) + m_descTexWidth - 1) >> m_descTexWidthLog);
        m_programDescSelect.Run(descTexSrc, descTexDst, idxTex, nFtrsDst);
    }
}

void FeatureExtractorSift::DownloadFeaturesToCPU(const TextureGL4 &ftrTex, const uint &nFtrs, FeatureSift *const &ftrs) {
#if FE_PRINT_TIMING == 1
    m_timer.Start(FE_TIMING_FEATURE_DOWNLOAD);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Start(FE_TIMING_FEATURE_DOWNLOAD);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Start(FE_TIMING_FEATURE_DOWNLOAD);
#endif
    const uint readHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
    const uint nReadPixels1 = (readHeight1 << m_ftrTexWidthLog);
    if(nReadPixels1 != 0)
        ftrTex.DownloadToCPU((float *) ftrs, m_ftrTexWidth, readHeight1);
    const uint nReadPixels2 = nFtrs - nReadPixels1;
    if(nReadPixels2 != 0)
        ftrTex.DownloadToCPU((float *) (ftrs + nReadPixels1), 0, readHeight1, nReadPixels2, 1);
#if FE_PRINT_TIMING == 1
    m_timer.Stop(FE_TIMING_FEATURE_DOWNLOAD);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Stop(FE_TIMING_FEATURE_DOWNLOAD);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Stop(FE_TIMING_FEATURE_DOWNLOAD);
#endif
}

void FeatureExtractorSift::DownloadFeaturesToCPU(const TextureGL4 &ftrTex, const uint &nFtrs, Point2D *const &ftrs) {
#if FE_PRINT_TIMING == 1
    m_timer.Start(FE_TIMING_FEATURE_DOWNLOAD);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Start(FE_TIMING_FEATURE_DOWNLOAD);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Start(FE_TIMING_FEATURE_DOWNLOAD);
#endif
    const uint readHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
    const uint nReadPixels1 = (readHeight1 << m_ftrTexWidthLog);
    if(nReadPixels1 != 0)
        ftrTex.DownloadToCPU((float *) ftrs, m_ftrTexWidth, readHeight1, GL_RG);
    const uint nReadPixels2 = nFtrs - nReadPixels1;
    if(nReadPixels2 != 0)
        ftrTex.DownloadToCPU((float *) (ftrs + nReadPixels1), 0, readHeight1, nReadPixels2, 1, GL_RG);
#if FE_PRINT_TIMING == 1
    m_timer.Stop(FE_TIMING_FEATURE_DOWNLOAD);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Stop(FE_TIMING_FEATURE_DOWNLOAD);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Stop(FE_TIMING_FEATURE_DOWNLOAD);
#endif
}

void FeatureExtractorSift::DownloadDescriptorsToCPU(const TextureGL4 &descTex, const uint &nFtrs, Descriptor *const &descs) {
#if FE_PRINT_TIMING == 1
    m_timer.Start(FE_TIMING_DESCRIPTOR_DOWNLOAD);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Start(FE_TIMING_DESCRIPTOR_DOWNLOAD);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Start(FE_TIMING_DESCRIPTOR_DOWNLOAD);
#endif
    const uint nReadPixels = (uint(nFtrs) << 4);
    const uint readHeight1 = (nReadPixels >> m_descTexWidthLog);
    const uint nReadPixels1 = (readHeight1 << m_descTexWidthLog);
    if(nReadPixels1 != 0)
        descTex.DownloadToCPU((float *) descs, m_descTexWidth, readHeight1);
    const uint nReadPixels2 = nReadPixels - nReadPixels1;
    if(nReadPixels2 != 0)
        descTex.DownloadToCPU((float *) (descs + (nReadPixels1 >> 4)), 0, readHeight1, nReadPixels2, 1);
#if FE_PRINT_TIMING == 1
    m_timer.Stop(FE_TIMING_DESCRIPTOR_DOWNLOAD);
#elif FE_PRINT_TIMING == 2
    m_timerOctave[0].Stop(FE_TIMING_DESCRIPTOR_DOWNLOAD);
#elif FE_PRINT_TIMING == 3
    m_timerLevel[0][0].Stop(FE_TIMING_DESCRIPTOR_DOWNLOAD);
#endif
}

void FeatureExtractorSift::DownloadIndexesToCPU(const TextureGL1 &idxTex, const uint &nFtrs, float *const &idxs) {
    const uint readHeight1 = (uint(nFtrs) >> m_idxTexWidthLog);
    const uint nReadPixels1 = (readHeight1 << m_idxTexWidthLog);
    if(nReadPixels1 != 0)
        idxTex.DownloadToCPU(idxs, m_idxTexWidth, readHeight1);
    const uint nReadPixels2 = nFtrs - nReadPixels1;
    if(nReadPixels2 != 0)
        idxTex.DownloadToCPU(idxs + nReadPixels1, 0, readHeight1, nReadPixels2, 1);
}

void FeatureExtractorSift::UploadFeaturesFromCPU(const TextureGL4 &ftrTex, const uint &nFtrs, const Point2D *const &ftrs) {
    ftrTex.Bind();
    const uint drawHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
    const uint nDrawPixels1 = (drawHeight1 << m_ftrTexWidthLog);
    if(nDrawPixels1 != 0)
        ftrTex.UploadFromCPU((const float *) ftrs, m_ftrTexWidth, drawHeight1, GL_RG);
    const uint nDrawPixels2 = nFtrs - nDrawPixels1;
    if(nDrawPixels2 != 0)
        ftrTex.UploadFromCPU((const float *) (ftrs + nDrawPixels1), 0, drawHeight1, nDrawPixels2, 1, GL_RG);
}

void FeatureExtractorSift::UploadFeaturesFromCPU(const TextureGL4 &ftrTex, const uint &nFtrs, const FeatureSift *const &ftrs) {
    ftrTex.Bind();
    const uint drawHeight1 = (uint(nFtrs) >> m_ftrTexWidthLog);
    const uint nDrawPixels1 = (drawHeight1 << m_ftrTexWidthLog);
    if(nDrawPixels1 != 0)
        ftrTex.UploadFromCPU((const float *) ftrs, m_ftrTexWidth, drawHeight1);
    const uint nDrawPixels2 = nFtrs - nDrawPixels1;
    if(nDrawPixels2 != 0)
        ftrTex.UploadFromCPU((const float *) (ftrs + nDrawPixels1), 0, drawHeight1, nDrawPixels2, 1);
}

void FeatureExtractorSift::UploadDescriptorsFromCPU(const TextureGL4 &descTex, const uint &nFtrs, const Descriptor *const &descs) {
    descTex.Bind();
    const uint nDrawPixels = (uint(nFtrs) << 4);
    const uint drawHeight1 = (nDrawPixels >> m_descTexWidthLog);
    const uint nDrawPixels1 = (drawHeight1 << m_descTexWidthLog);
    if(nDrawPixels1 != 0)
        descTex.UploadFromCPU((const float *) descs, m_descTexWidth, drawHeight1);
    const uint nDrawPixels2 = nDrawPixels - nDrawPixels1;
    if(nDrawPixels2 != 0)
        descTex.UploadFromCPU((const float *) (descs + (nDrawPixels1 >> 4)), 0, drawHeight1, nDrawPixels2, 1);
}

void FeatureExtractorSift::UploadIndexesFromCPU(const TextureGL1 &idxTex, const uint &nFtrs, const float *const &idxs) {
    idxTex.Bind();
    const uint drawHeight1 = (uint(nFtrs) >> m_idxTexWidthLog);
    const uint nDrawPixels1 = (drawHeight1 << m_idxTexWidthLog);
    if(nDrawPixels1 != 0)
        idxTex.UploadFromCPU(idxs, m_idxTexWidth, drawHeight1);
    const uint nDrawPixels2 = nFtrs - nDrawPixels1;
    if(nDrawPixels2 != 0)
        idxTex.UploadFromCPU(idxs + nDrawPixels1, 0, drawHeight1, nDrawPixels2, 1);
}

void FeatureExtractorSift::UploadIndexesFromCPU(const TextureGL2 &idxTex, const uint &nFtrs, const float *const &idxs) {
    idxTex.Bind();
    const uint drawHeight1 = (uint(nFtrs) >> m_idxTexWidthLog);
    const uint nDrawPixels1 = (drawHeight1 << m_idxTexWidthLog);
    if(nDrawPixels1 != 0)
        idxTex.UploadFromCPU(idxs, m_idxTexWidth, drawHeight1);
    const uint nDrawPixels2 = nFtrs - nDrawPixels1;
    if(nDrawPixels2 != 0)
        idxTex.UploadFromCPU(idxs + (nDrawPixels1 << 1), 0, drawHeight1, nDrawPixels2, 1);
}

void FeatureExtractorSift::RearrangeFeaturesLevelToGlobal(const ushort &iBuffer) {
    //ProgramGL::FitViewportGL(m_ftrTexWidth, (m_nFtrs[iBuffer] + m_ftrTexWidth - 1) >> m_ftrTexWidthLog);
    uint iFtrStart = 0;
    for(ushort iOctave = 0; iOctave < m_nOctaves; ++iOctave) {
        const Octave &octave = m_octaves[iOctave];
        for(ushort iLevelDoG = 0; iLevelDoG < m_nLevelsDoG; ++iLevelDoG) {
            const uint nFtrs = m_nFtrsLevel[iBuffer][iOctave][iLevelDoG];
            if(nFtrs == 0)
                continue;
            m_programFtrRearrange.Run(FRM_LEVEL_TO_GLOBAL, iOctave, octave.GetFeatureTexture(iLevelDoG), m_ftrTexs[iBuffer], iFtrStart, nFtrs);
            iFtrStart += nFtrs;
        }
    }
}

void FeatureExtractorSift::PushBackFeaturesFromCPU(const TextureGL4 &ftrTex, const uint &nFtrsExist, const uint &nFtrsNew, const Point2D *const &ftrsNew) {
    ftrTex.Bind();
    const Point2D *pFtrs = ftrsNew;
    const uint nSkipPixels = uint(nFtrsExist);
    uint nRemPixels = uint(nFtrsNew);
    ushort x = ushort(nSkipPixels & (m_ftrTexWidth - 1)), y = (nSkipPixels >> m_ftrTexWidthLog);
    if(x != 0) {
        const uint nPixels = x + nRemPixels > m_ftrTexWidth ? m_ftrTexWidth - x : nRemPixels;
        ftrTex.UploadFromCPU((const float *) pFtrs, x, y, nPixels, 1, GL_RG);
        pFtrs += nPixels;
        ++y;
        nRemPixels -= nPixels;
    }
    const uint drawHeight2 = (nRemPixels >> m_ftrTexWidthLog);
    if(drawHeight2 != 0) {
        const uint nPixels = (drawHeight2 << m_ftrTexWidthLog);
        ftrTex.UploadFromCPU((const float *) pFtrs, 0, y, m_ftrTexWidth, drawHeight2, GL_RG);
        pFtrs += nPixels;
        y += ushort(drawHeight2);
        nRemPixels -= nPixels;
    }
    if(nRemPixels != 0)
        ftrTex.UploadFromCPU((const float *) pFtrs, 0, y, nRemPixels, 1, GL_RG);
}

void FeatureExtractorSift::PushBackDescriptorsFromCPU(const TextureGL4 &descTex, const uint &nFtrsExist, const uint &nFtrsNew,
        const Descriptor *const &descsNew) {
    //PushBackPixelsFromCPU((const float *) descsNew, descTex, uint(nFtrsExist << 4), uint(nFtrsNew << 4), m_descTexWidth, m_descTexWidthLog, GL_RGBA);
    descTex.Bind();
    const Descriptor *pDescs = descsNew;
    const uint nSkipPixels = uint(nFtrsExist << 4);
    uint nRemPixels = uint(nFtrsNew << 4);
    ushort x = ushort(nSkipPixels & (m_descTexWidth - 1)), y = (nSkipPixels >> m_descTexWidthLog);
    if(x != 0) {
        const uint nPixels = x + nRemPixels > m_descTexWidth ? m_descTexWidth - x : nRemPixels;
        descTex.UploadFromCPU((const float *) pDescs, x, y, nPixels, 1);
        pDescs += (nPixels >> 4);
        ++y;
        nRemPixels -= nPixels;
    }
    const uint drawHeight2 = (nRemPixels >> m_descTexWidthLog);
    if(drawHeight2 != 0) {
        const uint nPixels = (drawHeight2 << m_descTexWidthLog);
        descTex.UploadFromCPU((const float *) pDescs, 0, y, m_descTexWidth, drawHeight2);
        pDescs += (nPixels >> 4);
        y += ushort(drawHeight2);
        nRemPixels -= nPixels;
    }
    if(nRemPixels != 0)
        descTex.UploadFromCPU((const float *) pDescs, 0, y, nRemPixels, 1);
}