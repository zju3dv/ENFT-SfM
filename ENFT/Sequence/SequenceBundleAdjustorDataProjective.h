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

#ifndef _SEQUENCE_BUNDLE_ADJUSTOR_DATA_PROJECTIVE_H_
#define _SEQUENCE_BUNDLE_ADJUSTOR_DATA_PROJECTIVE_H_

#include "Optimization/BundleAdjustorData.h"
#include "Sequence/Sequence.h"
#include "LinearAlgebra/Matrix11.h"

#define SEQUENCE_BA_ARGUMENT_PROJECTIVE FrameIndex, ProjectiveMatrix, LA::AlignedVector11f, LA::AlignedMatrix11f, \
										TrackIndex, Point3D, LA::Vector3f, LA::SymmetricMatrix3f, \
										MeasurementIndex, Point2D, LA::AlignedMatrix3x11f, FeatureIndex, \
										Camera::IntrinsicParameter, LA::Vector2f, LA::AlignedMatrix2f, LA::AlignedMatrix2x11f, LA::AlignedMatrix2x3f
class SequenceBundleAdjustorDataProjective : public BundleAdjustorDataTemplate<SEQUENCE_BA_ARGUMENT_PROJECTIVE>
{

public:

	SequenceBundleAdjustorDataProjective() : m_mapFrmToMea(m_mapCamToMea), m_mapTrkToMea(m_mapPtToMea), m_mapMeaToFrm(m_mapMeaToCam), m_mapMeaToTrk(m_mapMeaToPt) {}
	
	virtual float GetFactorSSEToMSE() const { return m_fxy / m_xs.Size(); }
	virtual void ValidateGlobal();
	virtual void InvalidateGlobal();
	virtual bool IsGlobalValid() const;
	virtual void ComputeMeasurementDepths(std::vector<float> &depths);

	virtual void ComputeSceneCenter(Point3D &center);
	virtual void TranslateScene(const Point3D &translation);
	virtual void ScaleScene(const float &scale);

	virtual void NormalizeData(const float dataNormalizeMedian);
	virtual void DenormalizeData();
	virtual double ComputeSSE();
	virtual double ComputeSSE(std::vector<float> &ptSSEs);
	virtual void ConstructNormalEquation(AlignedVector<LA::AlignedMatrix11f> &Dcs, AlignedVector<LA::SymmetricMatrix3f> &Dxs, LA::AlignedMatrix2f &Dg, 
		AlignedVector<LA::AlignedMatrix3x11f> &Wxcs, AlignedVector<LA::AlignedMatrix2x11f> &Wgcs, AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, 
		AlignedVector<LA::AlignedVector11f> &bcs, AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg, AlignedVector<LA::AlignedVector11f> &scs, 
		AlignedVector<LA::Vector3f> &sxs, LA::Vector2f &sg);
	virtual void ConstructNormalEquation(const AlignedVector<LA::AlignedVector11f> &scs, const AlignedVector<LA::Vector3f> &sxs, const LA::Vector2f &sg, 
		AlignedVector<LA::AlignedMatrix11f> &Dcs, AlignedVector<LA::SymmetricMatrix3f> &Dxs, LA::AlignedMatrix2f &Dg, 
		AlignedVector<LA::AlignedMatrix3x11f> &Wxcs, AlignedVector<LA::AlignedMatrix2x11f> &Wgcs, AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, 
		AlignedVector<LA::AlignedVector11f> &bcs, AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg);
	virtual void UpdateCameras(const AlignedVector<LA::AlignedVector11f> &scs, const AlignedVector<LA::AlignedVector11f> &xcs, const AlignedVector<ProjectiveMatrix> &PsOld);
	virtual void UpdatePoints(const AlignedVector<LA::Vector3f> &sxs, const AlignedVector<LA::Vector3f> &xxs, const AlignedVector<Point3D> &XsOld);
	virtual void UpdateGlobal(const LA::Vector2f &sg, const LA::Vector2f &xg, const Camera::IntrinsicParameter &Gold);

#if _DEBUG
	virtual void DebugSchurComplement(const float damping, const std::vector<std::pair<FrameIndex, FrameIndex> > &iPairs, 
		const AlignedVector<LA::AlignedMatrix11f> &Accs, const LA::AlignedMatrix2f &Agg, const AlignedVector<LA::AlignedMatrix2x11f> &Agcs, 
		const AlignedVector<LA::AlignedVector11f> &bcs, const LA::Vector2f &bg);
	virtual void DebugSolution(const float damping, const AlignedVector<LA::AlignedVector11f> &xcs, const AlignedVector<LA::Vector3f> &xxs, const LA::Vector2f &xg);
	virtual void DebugAx(const AlignedVector<LA::AlignedVector11f> &xcs, const LA::Vector2f &xg, const AlignedVector<LA::AlignedVector11f> &Axcs, const LA::Vector2f &Axg);
#endif

protected:

	MeasurementIndexList &m_mapFrmToMea;
	std::vector<MeasurementIndexList> &m_mapTrkToMea;
	FrameIndexList &m_mapMeaToFrm;
	TrackIndexList &m_mapMeaToTrk;

	float m_fxy;

	Point3D m_translation;
	float m_scale;
	std::vector<float> m_ds;

	friend Sequence;

};

#endif