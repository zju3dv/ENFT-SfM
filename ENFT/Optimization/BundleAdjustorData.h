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

#ifndef _BUNDLE_ADJUSTOR_DATA_H_
#define _BUNDLE_ADJUSTOR_DATA_H_

#include "Utility/AlignedVector.h"

#define BA_STAGE_DEFAULT	0
#define BA_STAGE_S			1
#define BA_STAGE_B			2
#define BA_STAGE_P			3
#define BA_STAGE_Z			4
#define BA_STAGE_X			5
#define BA_STAGE_R			6

#ifndef BA_TEMPLATE_PARAMETER
#define BA_TEMPLATE_PARAMETER typename CameraIndex, class Camera, class CameraParameter, class CameraBlock, \
							  typename PointIndex, class Point, class PointParameter, class PointBlock, \
							  typename MeasurementIndex, class Measurement, class PointCameraBlock, typename FeatureIndex, \
							  class Global, class GlobalParameter, class GlobalBlock, class GlobalCameraBlock, class GlobalPointBlock
#endif
#ifndef BA_TEMPLATE_ARGUMENT
#define BA_TEMPLATE_ARGUMENT CameraIndex, Camera, CameraParameter, CameraBlock, \
							 PointIndex, Point, PointParameter, PointBlock, \
							 MeasurementIndex, Measurement, PointCameraBlock, FeatureIndex, \
							 Global, GlobalParameter, GlobalBlock, GlobalCameraBlock, GlobalPointBlock
#endif
template<BA_TEMPLATE_PARAMETER>
class BundleAdjustorDataTemplate
{

public:

	inline const AlignedVector<Camera>& GetCameras() const { return m_Cs; }
	inline const AlignedVector<Point>& GetPoints() const { return m_Xs; }
	inline const AlignedVector<Measurement>& GetMeasurements() const { return m_xs; }
	inline CameraIndex GetCamerasNumber() const { return CameraIndex(m_Cs.Size()); }
	inline PointIndex GetPointsNumber() const { return PointIndex(m_Xs.Size()); }
	inline MeasurementIndex GetMeasurementsNumber() const { return MeasurementIndex(m_xs.Size()); }
	inline const Camera& GetCamera(const CameraIndex &iCam) const { return m_Cs[iCam]; }
	inline const Point& GetPoint(const PointIndex &iPt) const { return m_Xs[iPt]; }
	inline const Measurement& GetMeasurement(const MeasurementIndex &iMea) const { return m_xs[iMea]; }
	inline const Global& GetGlobal() const { return m_G; }
	inline const MeasurementIndex& GetCameraFirstMeasurementIndex(const CameraIndex &iCam) const { return m_mapCamToMea[iCam]; }
	inline const PointIndex* GetCameraPointIndexes(const CameraIndex &iCam) const { return m_mapMeaToPt.data() + m_mapCamToMea[iCam]; }
	inline FeatureIndex GetCameraFeaturesNumber(const CameraIndex &iCam) const { return FeatureIndex(m_mapCamToMea[iCam + 1] - m_mapCamToMea[iCam]); }
	inline const std::vector<MeasurementIndex>& GetPointMeasurementIndexes(const PointIndex &iPt) const { return m_mapPtToMea[iPt]; }
	inline const CameraIndex& GetMeasurementCameraIndex(const MeasurementIndex &iMea) const { return m_mapMeaToCam[iMea]; }
	inline const PointIndex& GetMeasurementPointIndex(const MeasurementIndex &iMea) const { return m_mapMeaToPt[iMea]; }
	inline void SwapCameras(AlignedVector<Camera> &Cs) { m_Cs.Swap(Cs); }
	inline void SwapPoints(AlignedVector<Point> &Xs) { m_Xs.Swap(Xs); }
	inline void SetCamera(const CameraIndex &iCam, const Camera &C) { m_Cs[iCam] = C; }
	inline void SetPoint(const PointIndex &iPt, const Point &X) { m_Xs[iPt] = X; }
	inline void SetGlobal(const Global &G) { m_G = G; }

	virtual void Resize(const CameraIndex &nCams, const PointIndex &nPts, const MeasurementIndex &nMeas);
	virtual void RemovePoints(const std::vector<bool> &ptMarksRemove, std::vector<PointIndex> &iPtsOriToNew);
	virtual void ReorderMeasurements(const std::vector<MeasurementIndex> &iMeasOriToNew);
	virtual void GetSubData(const std::vector<CameraIndex> &iCamsAdj, BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> &dataSub, 
		std::vector<CameraIndex> &iCamsSub, std::vector<PointIndex> &iPtsSub, std::vector<bool> &camMarks, 
		std::vector<PointIndex> &iPtsSrcToDst) const;
	virtual void SetSubData(const BundleAdjustorDataTemplate<BA_TEMPLATE_ARGUMENT> &dataSub, const CameraIndex &nCamsFix, 
		const std::vector<CameraIndex> &iCamsSub, const std::vector<PointIndex> &iPtsSub);

	virtual float GetFactorSSEToMSE() const = 0;
	virtual void ValidateGlobal() = 0;
	virtual void InvalidateGlobal() = 0;
	virtual bool IsGlobalValid() const = 0;

	virtual void NormalizeData(const float dataNormalizeMedian) = 0;
	virtual void DenormalizeData() = 0;

	virtual double ComputeSSE(std::vector<float> &ptSSEs) = 0;
	virtual void ConstructNormalEquation(AlignedVector<CameraBlock> &Dcs, AlignedVector<PointBlock> &Dxs, GlobalBlock &Dg, AlignedVector<PointCameraBlock> &Wxcs, 
		AlignedVector<GlobalCameraBlock> &Wgcs, AlignedVector<GlobalPointBlock> &Wgxs, AlignedVector<CameraParameter> &bcs, AlignedVector<PointParameter> &bxs, 
		GlobalParameter &bg, AlignedVector<CameraParameter> &scs, AlignedVector<PointParameter> &sxs, GlobalParameter &sg) = 0;
	virtual void ConstructNormalEquation(const AlignedVector<CameraParameter> &scs, const AlignedVector<PointParameter> &sxs, const GlobalParameter &sg, 
		AlignedVector<CameraBlock> &Dcs, AlignedVector<PointBlock> &Dxs, GlobalBlock &Dg, AlignedVector<PointCameraBlock> &Wxcs, 
		AlignedVector<GlobalCameraBlock> &Wgcs, AlignedVector<GlobalPointBlock> &Wgxs, AlignedVector<CameraParameter> &bcs, AlignedVector<PointParameter> &bxs, 
		GlobalParameter &bg) = 0;
	virtual void UpdateCameras(const AlignedVector<CameraParameter> &scs, const AlignedVector<CameraParameter> &xcs, const AlignedVector<Camera> &CsOld) = 0;
	virtual void UpdatePoints(const AlignedVector<PointParameter> &sxs, const AlignedVector<PointParameter> &xxs, const AlignedVector<Point> &XsOld) = 0;
	virtual void UpdateGlobal(const GlobalParameter &sg, const GlobalParameter &xg, const Global &Gold) = 0;

#if _DEBUG
	virtual void DebugSchurComplement(const float damping, const std::vector<std::pair<CameraIndex, CameraIndex> > &iPairs, 
		const AlignedVector<CameraBlock> &Accs, const GlobalBlock &Agg, const AlignedVector<GlobalCameraBlock> &Agcs, 
		const AlignedVector<CameraParameter> &bcs, const GlobalParameter &bg) {}
	virtual void DebugSolution(const float damping, const AlignedVector<CameraParameter> &xcs, const AlignedVector<PointParameter> &xxs, 
		const GlobalParameter &xg) {}
	virtual void DebugAx(const AlignedVector<CameraParameter> &xcs, const GlobalParameter &xg, const AlignedVector<CameraParameter> &Axcs, 
		const GlobalParameter &Axg) {}
#endif

protected:

	AlignedVector<Camera>		m_Cs;
	AlignedVector<Point>		m_Xs;
	AlignedVector<Measurement>	m_xs, m_xsOri;
	Global m_G;

	std::vector<MeasurementIndex>				m_mapCamToMea;
	std::vector<std::vector<MeasurementIndex> >	m_mapPtToMea;
	std::vector<CameraIndex>					m_mapMeaToCam;
	std::vector<PointIndex>						m_mapMeaToPt;

};

#include "BundleAdjustorData.hpp"

#endif