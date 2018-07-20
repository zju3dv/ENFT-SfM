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
#include "Sequence/Sequence.h"
template<BA_TEMPLATE_PARAMETER>
void BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT>::Resize(const CameraIndex &nCams, const PointIndex &nPts, const MeasurementIndex &nMeas)
{
	m_Cs.Resize(nCams);
	m_Xs.Resize(nPts);
	m_xs.Resize(nMeas);

	m_mapCamToMea.resize(nCams + 1);
	m_mapPtToMea.resize(nPts);
	for(PointIndex iPt = 0; iPt < nPts; ++iPt)
		m_mapPtToMea[iPt].resize(0);
	m_mapMeaToCam.resize(nMeas);
	m_mapMeaToPt.resize(nMeas);
}

template<BA_TEMPLATE_PARAMETER>
void BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT>::RemovePoints(const std::vector<bool> &ptMarksRemove, std::vector<PointIndex> &iPtsOriToNew)
{
	PointIndex iPtOri, iPtNew;
	const PointIndex nPtsOri = GetPointsNumber();
	iPtsOriToNew.assign(nPtsOri, nPtsOri);
	for(iPtOri = iPtNew = 0; iPtOri < nPtsOri; ++iPtOri)
	{
		if(ptMarksRemove[iPtOri])
			continue;
		m_Xs[iPtNew] = m_Xs[iPtOri];
		m_mapPtToMea[iPtNew].resize(0);
		iPtsOriToNew[iPtOri] = iPtNew++;
	}
	const PointIndex nPtsNew = iPtNew;
	m_Xs.Resize(nPtsNew);
	m_mapPtToMea.resize(nPtsNew);

	MeasurementIndex iMeaOri, iMeaNew = 0;
	const CameraIndex nCams = GetCamerasNumber();
	for(CameraIndex iCam = 0; iCam < nCams; ++iCam)
	{
		const MeasurementIndex iMeaOri1 = m_mapCamToMea[iCam], iMeaOri2 = m_mapCamToMea[iCam + 1];
		m_mapCamToMea[iCam] = iMeaNew;
		for(iMeaOri = iMeaOri1; iMeaOri < iMeaOri2; ++iMeaOri)
		{
			iPtOri = m_mapMeaToPt[iMeaOri];
			if((iPtNew = iPtsOriToNew[iPtOri]) >= nPtsOri)
				continue;
			m_mapPtToMea[iPtNew].push_back(iMeaNew);
			m_mapMeaToCam[iMeaNew] = iCam;
			m_mapMeaToPt[iMeaNew] = iPtNew;
			m_xs[iMeaNew] = m_xs[iMeaOri];
			++iMeaNew;
		}
	}
	const MeasurementIndex nMeasNew = iMeaNew;
	m_mapCamToMea.back() = nMeasNew;
	m_mapMeaToCam.resize(nMeasNew);
	m_mapMeaToPt.resize(nMeasNew);
	m_xs.Resize(nMeasNew);
}

template<BA_TEMPLATE_PARAMETER>
void BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT>::ReorderMeasurements(const std::vector<MeasurementIndex> &iMeasOriToNew)
{
	MeasurementIndex iMea;
	const MeasurementIndex nMeas = GetMeasurementsNumber();
	m_xs.Swap(m_xsOri);
	m_xs.Resize(nMeas);
	for(iMea = 0; iMea < nMeas; ++iMea)
		m_xs[iMeasOriToNew[iMea]] = m_xsOri[iMea];

	CameraIndex i;
	const PointIndex nPts = GetPointsNumber();
	for(PointIndex iPt = 0; iPt < nPts; ++iPt)
	{
		std::vector<MeasurementIndex> &iMeas = m_mapPtToMea[iPt];
		const CameraIndex nCrsps = CameraIndex(iMeas.size());
		for(i = 0; i < nCrsps; ++i)
		{
			iMeas[i] = iMeasOriToNew[iMeas[i]];
			m_mapMeaToPt[iMeas[i]] = iPt;
		}
	}
}

template<BA_TEMPLATE_PARAMETER>
void BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT>::GetSubData(const std::vector<CameraIndex> &iCamsAdj, BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> &dataSub, 
																  std::vector<CameraIndex> &iCamsSub, std::vector<PointIndex> &iPtsSub, std::vector<bool> &camMarks, 
																  std::vector<PointIndex> &iPtsSrcToDst) const
{
	// Step1: mark adjusted cameras
	const CameraIndex nCamsSrc = GetCamerasNumber();
	camMarks.assign(nCamsSrc, false);
	const CameraIndex nCamsAdj = CameraIndex(iCamsAdj.size());
	for(CameraIndex i = 0; i < nCamsAdj; ++i)
		camMarks[iCamsAdj[i]] = true;

	// Step2: collect BA points, and create reverse index list iPtsSrcToDst
	CameraIndex iCamSrc, iCamDst;
	PointIndex iPtSrc, iPtDst = 0;
	MeasurementIndex iMeaSrc, iMeaDst = 0;
	const PointIndex nPtsSrc = GetPointsNumber();
	iPtsSrcToDst.assign(nPtsSrc, nPtsSrc);
	iPtsSub.resize(0);
	for(CameraIndex i = 0; i < nCamsAdj; ++i)
	{
		iCamSrc = iCamsAdj[i];
		const MeasurementIndex iMeaSrc1 = m_mapCamToMea[iCamSrc], iMeaSrc2 = m_mapCamToMea[iCamSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iPtSrc = m_mapMeaToPt[iMeaSrc];
			if(iPtsSrcToDst[iPtSrc] < nPtsSrc)
				continue;
			iPtsSrcToDst[iPtSrc] = iPtDst++;
			iPtsSub.push_back(iPtSrc);
		}
		iMeaDst += iMeaSrc2 - iMeaSrc1;
	}

	// Step3: collect BA cameras, and count BA measurements
	iCamsSub.resize(0);
	for(iCamSrc = 0; iCamSrc < nCamsSrc; ++iCamSrc)
	{
		if(camMarks[iCamSrc])
			continue;
		const MeasurementIndex iMeaSrc1 = m_mapCamToMea[iCamSrc], iMeaSrc2 = m_mapCamToMea[iCamSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2 && iPtsSrcToDst[m_mapMeaToPt[iMeaSrc]] == nPtsSrc; ++iMeaSrc);
		if(iMeaSrc == iMeaSrc2)
			continue;
		iCamsSub.push_back(iCamSrc);
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			if(iPtsSrcToDst[m_mapMeaToPt[iMeaSrc]] != nPtsSrc)
				++iMeaDst;
		}
	}
	iCamsSub.insert(iCamsSub.end(), iCamsAdj.begin(), iCamsAdj.end());

	// Step4: copy BA cameras, points and planes
	const CameraIndex nCamsDst = CameraIndex(iCamsSub.size());
	const PointIndex nPtsDst = iPtDst;
	const MeasurementIndex nMeasDst = iMeaDst;
	dataSub.Resize(nCamsDst, nPtsDst, nMeasDst);
	for(iCamDst = 0; iCamDst < nCamsDst; ++iCamDst)
		dataSub.m_Cs[iCamDst] = m_Cs[iCamsSub[iCamDst]];
	for(iPtDst = 0; iPtDst < nPtsDst; ++iPtDst)
		dataSub.m_Xs[iPtDst] = m_Xs[iPtsSub[iPtDst]];

	// Step5: copy measurements and create correspondence maps
	iMeaDst = 0;
	dataSub.m_mapCamToMea[0] = 0;
	for(iCamDst = 0; iCamDst < nCamsDst; ++iCamDst)
	{
		iCamSrc = iCamsSub[iCamDst];
		const MeasurementIndex iMeaSrc1 = m_mapCamToMea[iCamSrc], iMeaSrc2 = m_mapCamToMea[iCamSrc + 1];
		for(iMeaSrc = iMeaSrc1; iMeaSrc < iMeaSrc2; ++iMeaSrc)
		{
			iPtSrc = m_mapMeaToPt[iMeaSrc];
			if((iPtDst = iPtsSrcToDst[iPtSrc]) == nPtsSrc)
				continue;
			dataSub.m_xs[iMeaDst] = m_xs[iMeaSrc];
			dataSub.m_mapPtToMea[iPtDst].push_back(iMeaDst);
			dataSub.m_mapMeaToCam[iMeaDst] = iCamDst;
			dataSub.m_mapMeaToPt[iMeaDst] = iPtDst;
			++iMeaDst;
		}
		dataSub.m_mapCamToMea[iCamDst + 1] = iMeaDst;
	}
#if _DEBUG
	assert(iMeaDst == nMeasDst);
#endif
}

template<BA_TEMPLATE_PARAMETER>
void BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT>::SetSubData(const BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> &dataSub, 
																  const CameraIndex &nCamsFix, const std::vector<CameraIndex> &iCamsSub, 
																  const std::vector<PointIndex> &iPtsSub)
{
	const CameraIndex nCamsSub = FrameIndex(iCamsSub.size());
	for(CameraIndex i = nCamsFix; i < nCamsSub; ++i)
		m_Cs[iCamsSub[i]] = dataSub.m_Cs[i];
	const PointIndex nPtsSub = TrackIndex(iPtsSub.size());
	for(PointIndex i = 0; i < nPtsSub; ++i)
		m_Xs[iPtsSub[i]] = dataSub.m_Xs[i];
}