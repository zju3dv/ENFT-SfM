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

#ifndef _SEQUENCE_SET_BUNDLE_ADJUSTOR_DATA_3D_SIMILARITY_H_
#define _SEQUENCE_SET_BUNDLE_ADJUSTOR_DATA_3D_SIMILARITY_H_

#include "SequenceSet.h"
#include "Optimization/BundleAdjustorData.h"
#include "LinearAlgebra/Matrix7.h"

#define SEQUENCE_SET_BA_ARGUMENT_3D_SIMILARITY	SequenceIndex, SimilarityTransformation3D, LA::AlignedVector7f, LA::AlignedMatrix7f, \
												TrackIndex, Point3D, LA::Vector3f, LA::SymmetricMatrix3f, \
												TrackIndex, Point3D, LA::AlignedMatrix3x7f, TrackIndex, \
												Camera::IntrinsicParameter, LA::Vector2f, LA::AlignedMatrix2f, LA::AlignedMatrix2x7f, LA::AlignedMatrix2x3f
class SequenceSetBundleAdjustorData3DSimilarity : public BundleAdjustorDataTemplate<SEQUENCE_SET_BA_ARGUMENT_3D_SIMILARITY>
{

public:

	SequenceSetBundleAdjustorData3DSimilarity() : m_mapSeqToIdvTrk(m_mapCamToMea), m_mapCmnTrkToIdvTrk(m_mapPtToMea), m_mapIdvTrkToSeq(m_mapMeaToCam), m_mapIdvTrkToCmnTrk(m_mapMeaToPt) {}

	virtual float GetFactorSSEToMSE() const { return 1.0f / m_xs.Size(); }

	virtual void NormalizeData(const float dataNormalizeMedian);
	virtual void DenormalizeData();
	virtual void ValidateGlobal();
	virtual void InvalidateGlobal();
	virtual bool IsGlobalValid() const;

	virtual double ComputeSSE(std::vector<float> &ptSSEs);
	virtual void ConstructNormalEquation(AlignedVector<LA::AlignedMatrix7f> &Dcs, AlignedVector<LA::SymmetricMatrix3f> &Dxs, LA::AlignedMatrix2f &Dg, 
		AlignedVector<LA::AlignedMatrix3x7f> &Wxcs, AlignedVector<LA::AlignedMatrix2x7f> &Wgcs, AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, 
		AlignedVector<LA::AlignedVector7f> &bcs, AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg, AlignedVector<LA::AlignedVector7f> &scs, 
		AlignedVector<LA::Vector3f> &sxs, LA::Vector2f &sg);
	virtual void ConstructNormalEquation(const AlignedVector<LA::AlignedVector7f> &scs, const AlignedVector<LA::Vector3f> &sxs, const LA::Vector2f &sg, 
		AlignedVector<LA::AlignedMatrix7f> &Dcs, AlignedVector<LA::SymmetricMatrix3f> &Dxs, LA::AlignedMatrix2f &Dg, AlignedVector<LA::AlignedMatrix3x7f> &Wxcs, 
		AlignedVector<LA::AlignedMatrix2x7f> &Wgcs, AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, AlignedVector<LA::AlignedVector7f> &bcs, AlignedVector<LA::Vector3f> &bxs, 
		LA::Vector2f &bg);
	virtual void UpdateCameras(const AlignedVector<LA::AlignedVector7f> &scs, const AlignedVector<LA::AlignedVector7f> &xcs, 
		const AlignedVector<SimilarityTransformation3D> &SsOld);
	virtual void UpdatePoints(const AlignedVector<LA::Vector3f> &sxs, const AlignedVector<LA::Vector3f> &xxs, const AlignedVector<Point3D> &XsOld);
	virtual void UpdateGlobal(const LA::Vector2f &sg, const LA::Vector2f &xg, const Camera::IntrinsicParameter &Gold);

protected:

	TrackIndexList				&m_mapSeqToIdvTrk;
	std::vector<TrackIndexList>	&m_mapCmnTrkToIdvTrk;
	SequenceIndexList			&m_mapIdvTrkToSeq;
	TrackIndexList				&m_mapIdvTrkToCmnTrk;

	Point3D m_translation;
	float m_scale;
	std::vector<float> m_distSqs;

	friend SequenceSet;

};

#endif