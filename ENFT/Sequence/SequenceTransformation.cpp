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
#include "Sequence/Sequence.h"

void Sequence::SetReferenceFrame(const FrameIndex &iFrmRef)
{
	ENFT_SSE::__m128 work[2];
	Camera &Cr = m_Cs[iFrmRef];
	const FrameIndex nFrms = GetCamerasNumber();
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		if(iFrm != iFrmRef && (m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
			m_Cs[iFrm].ChangeReference(Cr, work);
	}
	const TrackIndex nTrks = GetPointsNumber();
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		if(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED)
			Cr.Apply(m_Xs[iTrk], work);
	}
	Cr.MakeIdentity();
	ComputeProjectiveMatrixes();
}

void Sequence::ComputeSceneSize(Point3D &size) const
{
	Point3D center;
	center.XYZx() = size.XYZx() = ENFT_SSE::_mm_setr_ps(0, 0, 0, 1);
	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	TrackIndex nInliers = 0;
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		if(!(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
			continue;
		LA::ApB(m_Xs[iTrk], center, center);
		++nInliers;
	}
	if(nInliers == 0)
		return;
	const float norm = 1.0f / nInliers;
	center.Scale(norm);

	Point3D dX;
	std::vector<float> dXs(nInliers), dYs(nInliers), dZs(nInliers);
	for(TrackIndex iTrk = 0, i = 0; iTrk < nTrks; ++iTrk)
	{
		if(!(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER))
			continue;
		LA::AmB(m_Xs[iTrk], center, dX);
		dXs[i] = fabs(dX.v0());
		dYs[i] = fabs(dX.v1());
		dZs[i] = fabs(dX.v2());
		++i;
	}

	const TrackIndex ith = TrackIndex((nInliers - 1) * 0.5f);
	std::nth_element(dXs.begin(), dXs.begin() + ith, dXs.end());
	std::nth_element(dYs.begin(), dYs.begin() + ith, dYs.end());
	std::nth_element(dZs.begin(), dZs.begin() + ith, dZs.end());

	size.Set(dXs[ith], dYs[ith], dZs[ith]);
	size.Scale(1.4f);
	LA::ApB(size, size, size);
	//printf("Center = (%f, %f, %f), Size = (%f, %f, %f)\n", center.X(), center.Y(), center.Z(), size.X(), size.Y(), size.Z());
}

void Sequence::ComputeMeasurementDepths(std::vector<float> &depths) const
{
	const MeasurementIndex nMeas = GetMeasurementsNumber();
	depths.resize(nMeas);
	for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea)
		depths[iMea] = m_Cs[m_mapMeaToFrm[iMea]].ComputeDepth(m_Xs[m_mapMeaToTrk[iMea]]);
}

void Sequence::TranslateScene(const Point3D &translation)
{
	if(translation.SquaredLength() == 0)
		return;
	const ENFT_SSE::__m128 t = ENFT_SSE::_mm_setr_ps(translation.X(), translation.Y(), translation.Z(), 1);
	Point3D dt;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
			continue;
		m_Cs[iFrm].ApplyRotation(t, dt.XYZx());
		m_Cs[iFrm].DecreaseTranslation(dt);
	}
	ComputeProjectiveMatrixes();

	const ENFT_SSE::__m128 dX = ENFT_SSE::_mm_setr_ps(translation.X(), translation.Y(), translation.Z(), 0);
	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		if(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED)
			m_Xs[iTrk].XYZx() = ENFT_SSE::_mm_add_ps(m_Xs[iTrk].XYZx(), dX);
	}
}

void Sequence::ScaleScene(const float &scale)
{
	if(scale == 1)
		return;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		if(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED)
			m_Cs[iFrm].Scale(scale);
	}
	ComputeProjectiveMatrixes();

	const ENFT_SSE::__m128 s = ENFT_SSE::_mm_setr_ps(scale, scale, scale, 1);
	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		if(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED)
			m_Xs[iTrk].Scale(s);
	}
}

void Sequence::TransformScene(const RigidTransformation3D &T)
{
	ENFT_SSE::__m128 work[3];
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		if(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED)
			T.Apply(m_Cs[iFrm], m_Cs[iFrm], work);
	}
	ComputeProjectiveMatrixes();

	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		if(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED)
			T.Apply(m_Xs[iTrk], work);
	}
}

void Sequence::TransformScene(const SimilarityTransformation3D &S)
{
	ENFT_SSE::__m128 work[3];
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		if(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED)
			S.Apply(m_Cs[iFrm], m_Cs[iFrm], work);
	}
	ComputeProjectiveMatrixes();

	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		if(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED)
			S.Apply(m_Xs[iTrk], work);
	}
}

static inline void UpdateToMetric(const float &f, const float &nfp0, const float &nfp1, const float &np2, const ProjectiveMatrix &Pp, ProjectiveMatrix &Pm)
{
	Pm.M00() = Pp.M00() * f + Pp.M03() * nfp0;
	Pm.M01() = Pp.M01() * f + Pp.M03() * nfp1;
	Pm.M02() = Pp.M02() + Pp.M03() * np2;
	Pm.M03() = Pp.M03();
	Pm.M10() = Pp.M10() * f + Pp.M13() * nfp0;
	Pm.M11() = Pp.M11() * f + Pp.M13() * nfp1;
	Pm.M12() = Pp.M12() + Pp.M13() * np2;
	Pm.M13() = Pp.M13();
	Pm.M20() = Pp.M20() * f + Pp.M23() * nfp0;
	Pm.M21() = Pp.M21() * f + Pp.M23() * nfp1;
	Pm.M22() = Pp.M22() + Pp.M23() * np2;
	Pm.M23() = Pp.M23();
}

void Sequence::UpdateToMetric(const AbsoluteQuadric &Q)
{
#if _DEBUG
	assert(m_intrinsicType != INTRINSIC_USER_FIXED);
#endif
	const float f = sqrt(Q.f2()), fI = 1 / f, fI2 = fI * fI;
	const float nfp0 = Q.a0() * fI, nfp1 = Q.a1() * fI, np2 = Q.a2();
	const ENFT_SSE::__m128 p = ENFT_SSE::_mm_setr_ps(-Q.a0() * fI2, -Q.a1() * fI2, -Q.a2(), 1.0f);

	FrameIndex iFrm;
	ProjectiveMatrix Pp;
	std::vector<float> fs;
	ENFT_SSE::__m128 work[4];
	const FrameIndex nFrms = FrameIndex(m_Ps.Size());
	m_Cs.Resize(nFrms);
	if(m_intrinsicType == INTRINSIC_VARIABLE)
		m_Krs.Resize(nFrms);
	for(iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
			continue;
		Pp = m_Ps[iFrm];
		ProjectiveMatrix &Pm = m_Ps[iFrm];
		::UpdateToMetric(f, nfp0, nfp1, np2, Pp, Pm);
		if(m_intrinsicType == INTRINSIC_CONSTANT)
		{
			if(!Pm.ToIdealIntrinsicExtrinsic(m_Kr.f(), m_Cs[iFrm], work))
				m_frmStates[iFrm] &= ~FLAG_FRAME_STATE_SOLVED;
			else
				fs.push_back(m_Kr.f());
		}
		else if(m_intrinsicType == INTRINSIC_VARIABLE)
		{
//#if _DEBUG
//			const float f = m_Krs[iFrm].f();
//#endif
			if(!Pm.ToIdealIntrinsicExtrinsic(m_Krs[iFrm].f(), m_Cs[iFrm], work))
				m_frmStates[iFrm] &= ~FLAG_FRAME_STATE_SOLVED;
//#if _DEBUG
//			else
//				printf("Frame %d: %f\n", iFrm, m_Krs[iFrm].f());
//			//else
//			//	printf("Frame %d: %f - %f = %f\n", iFrm, f, m_Krs[iFrm].f(), f - m_Krs[iFrm].f());
//#endif
		}
	}
	if(m_intrinsicType == INTRINSIC_CONSTANT)
	{
		const FrameIndex ith = FrameIndex(fs.size() >> 1);
		std::nth_element(fs.begin(), fs.begin() + ith, fs.end());
		m_Kr.f() = fs[ith];
	}

	TrackIndex iTrk;
	Point3D Xp;
	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		if(!(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED))
			continue;
		Xp = m_Xs[iTrk];
#if _DEBUG
		assert(Xp.reserve() == 1.0f);
#endif
		Point3D &Xm = m_Xs[iTrk];
		Xm.X() = fI * Xp.X();
		Xm.Y() = fI * Xp.Y();
		Xm.Z() = Xp.Z();
		Xm *= 1 / ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(p, Xp.XYZx()));
//#if _DEBUG
//		Xm.XYZx() = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_setzero_ps(), Xm.XYZx());
//		Xm.reserve() = 1.0f;
//#endif
		Xm.reserve() = 1.0f;
	}

//#if _DEBUG
//	FrameIndexList iFrmsSolve;
//	GetFrameIndexList(FLAG_FRAME_STATE_SOLVED, iFrmsSolve);
//	AlignedVector<ProjectiveMatrix> Ps;
//	const FrameIndex nFrmsSolve = FrameIndex(iFrmsSolve.size());
//	Ps.Resize(nFrmsSolve);
//	for(FrameIndex i = 0; i < nFrmsSolve; ++i)
//		Ps[i] = m_Ps[iFrmsSolve[i]];
//	SaveProjectiveMatrixes("F:/tmp/test.txt", Ps);
//	exit(0);
//#endif

	MeasurementIndex iMea, cntPos = 0, cntNeg = 0;
	for(iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
			continue;
		const Camera &C = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			if((iTrk = m_mapMeaToTrk[iMea]) == INVALID_TRACK_INDEX || !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) || (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
				continue;
			else if(C.ComputeDepth(m_Xs[iTrk]) > 0)
				++cntPos;
			else
				++cntNeg;
		}
	}
	const ENFT_SSE::__m128 zero = ENFT_SSE::_mm_setzero_ps();
	if(cntPos < cntNeg)
	{
		for(iFrm = 0; iFrm < nFrms; ++iFrm)
		{
			if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
				continue;
			Camera &C = m_Cs[iFrm];
			C.tX() = -C.tX();
			C.tY() = -C.tY();
			C.tZ() = -C.tZ();
			ProjectiveMatrix &P = m_Ps[iFrm];
			P.M03() = -P.M03();
			P.M13() = -P.M13();
			P.M23() = -P.M23();
		}
		for(iTrk = 0; iTrk < nTrks; ++iTrk)
		{
			if(!(m_trkStates[iTrk] & FLAG_TRACK_STATE_SOLVED))
				continue;
			Point3D &X = m_Xs[iTrk];
			X.XYZx() = ENFT_SSE::_mm_sub_ps(zero, X.XYZx());
			X.reserve() = 1.0f;
		}
	}
	//for(iFrm = 0; iFrm < nFrms; ++iFrm)
	//{
	//	if(!(m_frmStates[iFrm] & FLAG_FRAME_STATE_SOLVED))
	//		continue;
	//	const Camera &C = m_Cs[iFrm];
	//	const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
	//	for(iMea = iMea1; iMea < iMea2; ++iMea)
	//	{
	//		if((iTrk = m_mapMeaToTrk[iMea]) == INVALID_TRACK_INDEX || !(m_trkStates[iTrk] & FLAG_TRACK_STATE_INLIER) || (m_meaStates[iMea] & FLAG_MEASUREMENT_STATE_OUTLIER))
	//			continue;
	//		else if(C.ComputeDepth(m_Xs[iTrk]) < 0)
	//		{
	//			Point3D &X = m_Xs[iTrk];
	//			X.XYZx() = ENFT_SSE::_mm_sub_ps(zero, X.XYZx());
	//			X.reserve() = 1.0f;
	//			//MarkTrackOutlier(iTrk);
	//		}
	//	}
	//}
}

void Sequence::UpdateToMetric(const AbsoluteQuadric &Q, AlignedVector<ProjectiveMatrix> &Ps)
{
	const float f = sqrt(Q.f2()), fI = 1 / f;
	const float nfp0 = Q.a0() * fI, nfp1 = Q.a1() * fI, np2 = Q.a2();

	ProjectiveMatrix Pp;
	const uint N = Ps.Size();
	for(uint i = 0; i < N; ++i)
	{
		Pp = Ps[i];
		::UpdateToMetric(f, nfp0, nfp1, np2, Pp, Ps[i]);
	}
}