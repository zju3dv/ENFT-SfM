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
#include "SequenceBundleAdjustorData.h"
#if _DEBUG
//#define _DEBUG_WITH_EIGEN
#ifdef _DEBUG_WITH_EIGEN
//#define _DEBUG_WITH_EIGEN_JTJ_JTE
#define _DEBUG_WITH_EIGEN_AX
#include <Eigen>
#endif
#endif

void SequenceBundleAdjustorData::ValidateGlobal()
{
	m_r2s.Resize(m_xs.Size());
	m_r2xs.Resize(0);
}

void SequenceBundleAdjustorData::InvalidateGlobal()
{
	m_r2s.Resize(0);
	m_r2xs.Resize(0);
}

bool SequenceBundleAdjustorData::IsGlobalValid() const
{
	return !m_r2s.Empty();
}

void SequenceBundleAdjustorData::UndistortMeasurements()
{
#if _DEBUG
	assert(IsGlobalValid());
#endif
	const uint N = m_xs.Size(), _N2 = N - (N & 1), _N4 = N - (N & 3);

	uint i;
	if(m_r2xs.Empty())
	{
		m_r2xs.Resize(N);

		ENFT_SSE::__m128 r2;
		const ENFT_SSE::__m128 *x = (ENFT_SSE::__m128 *) m_xs.Data();
		ENFT_SSE::__m128 *r2x = (ENFT_SSE::__m128 *) m_r2xs.Data();
		for(i = 0; i < _N2; i += 2, ++x, ++r2x)
		{
			r2 = ENFT_SSE::_mm_mul_ps(*x, *x);
			m_r2s[i  ] = r2.m128_f32[0] = r2.m128_f32[1] = r2.m128_f32[0] + r2.m128_f32[1];
			m_r2s[i+1] = r2.m128_f32[2] = r2.m128_f32[3] = r2.m128_f32[2] + r2.m128_f32[3];
			*r2x = ENFT_SSE::_mm_mul_ps(r2, *x);
		}
		if(_N2 != N)
		{
			const Point2D &x = m_xs[_N2];
			float &r2 = m_r2s[_N2];
			r2 = x.SquaredLength();
			m_r2xs[_N2].Set(r2 * x.x(), r2 * x.y());
		}
	}

	m_cs.Resize(N);
	m_xus.Resize(N);

	const ENFT_SSE::__m128 d = ENFT_SSE::_mm_set1_ps(m_G.d()), one = ENFT_SSE::_mm_set1_ps(1.0f);
	const ENFT_SSE::__m128 *r2 = (ENFT_SSE::__m128 *) m_r2s.Data();
	ENFT_SSE::__m128 *ps = (ENFT_SSE::__m128 *) m_cs.Data();
	for(i = 0; i < _N4; i += 4, ++r2, ++ps)
		*ps = ENFT_SSE::_mm_add_ps(one, ENFT_SSE::_mm_mul_ps(d, *r2));
	for(; i < N; ++i)
		m_cs[i] = 1 + m_G.d() * m_r2s[i];

	ENFT_SSE::__m128 c;
	const ENFT_SSE::__m128 *x = (ENFT_SSE::__m128 *) m_xs.Data();
	ENFT_SSE::__m128 *xu = (ENFT_SSE::__m128 *) m_xus.Data();
	for(i = 0; i < _N2; i += 2, ++x, ++xu)
	{
		c.m128_f32[0] = c.m128_f32[1] = m_cs[i];
		c.m128_f32[2] = c.m128_f32[3] = m_cs[i + 1];
		*xu = ENFT_SSE::_mm_mul_ps(c, *x);
	}
	if(_N2 != N)
	{
		const float &c = m_cs[_N2];
		const Point2D &x = m_xs[_N2];
		m_xus[_N2].Set(c * x.x(), c * x.y());
	}

	//printf("%f, %f\n", m_r2s[0], m_cs[0]);
	//m_xs[0].Print();
	//m_xus[0].Print();
}

void SequenceBundleAdjustorData::ScaleMeasurements(const float scale)
{
	if(scale == 1.0f)
		return;
	const ENFT_SSE::__m128 s = ENFT_SSE::_mm_set1_ps(scale);
	const uint N = m_xs.Size(), _N = N - (N & 1);
	ENFT_SSE::__m128 *x = (ENFT_SSE::__m128 *) m_xs.Data();
	for(uint i = 0; i < _N; i += 2, ++x)
		*x = ENFT_SSE::_mm_mul_ps(s, *x);
	if(_N != N)
		m_xs[_N] *= scale;
}

void SequenceBundleAdjustorData::RectifyMeasurements()
{
	ValidateGlobal();
	UndistortMeasurements();
	m_xs.Swap(m_xus);
	m_xus.Resize(0);
	ScaleMeasurements(1 / m_G.f());
	m_G.Set(1.0f, 0.0f);
}

void SequenceBundleAdjustorData::ComputeMeasurementDepths(std::vector<float> &depths)
{
	depths.resize(m_xs.Size());

	FrameIndex iFrm;
	MeasurementIndex iMea;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		const Camera &C = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
			depths[iMea] = C.ComputeDepth(m_Xs[m_mapMeaToTrk[iMea]]);
	}
}

void SequenceBundleAdjustorData::ComputeSceneCenter(Point3D &center)
{
	center.XYZx() = ENFT_SSE::_mm_setr_ps(0, 0, 0, 1);
	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	if(nTrks == 0)
		return;
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
		LA::ApB(m_Xs[iTrk], center, center);
	const float norm = 1.0f / nTrks;
	center.Scale(norm);
	center.reserve() = 1;
}

void SequenceBundleAdjustorData::TranslateScene(const Point3D &translation)
{
	if(translation.SquaredLength() == 0)
		return;
	const ENFT_SSE::__m128 t = ENFT_SSE::_mm_setr_ps(translation.X(), translation.Y(), translation.Z(), 1);
	Point3D dt;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		m_Cs[iFrm].ApplyRotation(t, dt.XYZx());
		m_Cs[iFrm].DecreaseTranslation(dt);
	}

	const ENFT_SSE::__m128 dX = ENFT_SSE::_mm_setr_ps(translation.X(), translation.Y(), translation.Z(), 0);
	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
		m_Xs[iTrk].XYZx() = ENFT_SSE::_mm_add_ps(m_Xs[iTrk].XYZx(), dX);
}

void SequenceBundleAdjustorData::ScaleScene(const float &scale)
{
	if(scale == 1.0f)
		return;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
		m_Cs[iFrm].Scale(scale);
	const ENFT_SSE::__m128 s = ENFT_SSE::_mm_setr_ps(scale, scale, scale, 1);
	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
		m_Xs[iTrk].Scale(s);
}

void SequenceBundleAdjustorData::NormalizeData(const float dataNormalizeMedian)
{
	if(dataNormalizeMedian == 0)
	{
		m_scaleScene = m_scaleFocal = 1.0f;
		m_translation.SetZero();
		return;
	}
	Point3D center;
	ComputeSceneCenter(center);
	//printf("...\n");
	//center.SetZero();
	m_translation.Set(-center.X(), -center.Y(), -center.Z());
	TranslateScene(m_translation);

	ComputeMeasurementDepths(m_ds);//bug m_ds.size() == 0
	if (m_ds.size() == 0) {
		std::cout << "ERROR: m_ds.size() == 0" << std::endl;
		return;
	}
	const MeasurementIndex ith = MeasurementIndex(m_ds.size() >> 1);
	std::nth_element(m_ds.begin(), m_ds.begin() + ith, m_ds.end());
	const float dMed = m_ds[ith];
	m_scaleScene = (1 / dMed) / dataNormalizeMedian;
	ScaleScene(m_scaleScene);

	if(IsGlobalValid())
	{
		m_scaleFocal = dataNormalizeMedian / m_G.f();
		m_G.Scale(m_scaleFocal);
		ScaleMeasurements(m_scaleFocal);
	}
	else
		m_scaleFocal = 1.0f;
}

void SequenceBundleAdjustorData::DenormalizeData()
{
	m_scaleScene = 1 / m_scaleScene;
	ScaleScene(m_scaleScene);
	m_scaleScene = 1;

	m_translation.v0() = -m_translation.v0();
	m_translation.v1() = -m_translation.v1();
	m_translation.v2() = -m_translation.v2();
	TranslateScene(m_translation);
	m_translation.SetZero();

	if(IsGlobalValid())
	{
		m_scaleFocal = 1 / m_scaleFocal;
		m_G.Scale(m_scaleFocal);
	}
}

double SequenceBundleAdjustorData::ComputeSSE()
{
	double SSE = 0;
	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	if(IsGlobalValid())
	{
		Point2D xp;
		UndistortMeasurements();
		const float f = m_G.f();
		for(iFrm = 0; iFrm < nFrms; ++iFrm)
		{
			const Camera &C = m_Cs[iFrm];
			const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
			for(iMea = iMea1; iMea < iMea2; ++iMea)
			{
				iTrk = m_mapMeaToTrk[iMea];
				C.ProjectToNormalizedPlane(m_Xs[iTrk], xp);
				xp *= f;
				SSE += m_xus[iMea].SquaredDistance(xp);
			}
		}
	}
	else
	{
		Point2D e;
		for(iFrm = 0; iFrm < nFrms; ++iFrm)
		{
			const Camera &C = m_Cs[iFrm];
			const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
			for(iMea = iMea1; iMea < iMea2; ++iMea)
			{
				iTrk = m_mapMeaToTrk[iMea];
				SSE += C.ComputeProjectionSquaredError(m_Xs[iTrk], m_xs[iMea], e);
			}
		}
	}

#if 0
	if(IsGlobalValid())
	{
		Point2D xp;
		float errSq, errSqMax = 0;
		MeasurementIndex iMeaMax = 0;
		const float f = m_G.f();
		for(iFrm = 0; iFrm < nFrms; ++iFrm)
		{
			const Camera &C = m_Cs[iFrm];
			const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
			for(iMea = iMea1; iMea < iMea2; ++iMea)
			{
				iTrk = m_mapMeaToTrk[iMea];
				C.ProjectToNormalizedPlane(m_Xs[iTrk], xp);
				xp *= f;
				if((errSq = m_xus[iMea].SquaredDistance(xp)) < errSqMax)
					continue;
				errSqMax = errSq;
				iMeaMax = iMea;
			}
		}
		const FrameIndex iFrmMax = m_mapMeaToFrm[iMeaMax];
		const TrackIndex iTrkMax = m_mapMeaToTrk[iMeaMax];
		m_Cs[iFrmMax].ProjectToNormalizedPlane(m_Xs[iTrkMax], xp);
		xp *= f;
		printf("----------------------------------------------------------------\n");
		printf("Error <= ||(%f, %f) - (%f, %f)|| == %f --> %f\n", m_xus[iMeaMax].x(), m_xus[iMeaMax].y(), xp.x(), xp.y(), errSqMax, errSqMax * m_fxy);
		printf("iFrm = %d, iTrk = %d, iMea = %d\n", iFrmMax, iTrkMax, iMeaMax);
	}
	else
	{
		Point2D xp, e;
		float errSq, errSqMax = 0;
		MeasurementIndex iMeaMax = 0;
		for(iFrm = 0; iFrm < nFrms; ++iFrm)
		{
			const Camera &C = m_Cs[iFrm];
			const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
			for(iMea = iMea1; iMea < iMea2; ++iMea)
			{
				iTrk = m_mapMeaToTrk[iMea];
				if((errSq = C.ComputeProjectionSquaredError(m_Xs[iTrk], m_xs[iMea], e)) < errSqMax)
					continue;
				errSqMax = errSq;
				iMeaMax = iMea;
			}
		}
		const FrameIndex iFrmMax = m_mapMeaToFrm[iMeaMax];
		const TrackIndex iTrkMax = m_mapMeaToTrk[iMeaMax];
		m_Cs[iFrmMax].ProjectToNormalizedPlane(m_Xs[iTrkMax], xp);
		printf("----------------------------------------------------------------\n");
		printf("Error <= ||(%f, %f) - (%f, %f)|| == %f --> %f\n", m_xs[iMeaMax].x(), m_xs[iMeaMax].y(), xp.x(), xp.y(), errSqMax, errSqMax * m_fxy);
		printf("iFrm = %d, iTrk = %d, iMea = %d\n", iFrmMax, iTrkMax, iMeaMax);
	}
#endif

	return SSE;
}

double SequenceBundleAdjustorData::ComputeSSE(std::vector<float> &ptSSEs)
{
	const TrackIndex nTrks = GetPointsNumber();
	ptSSEs.assign(nTrks, 0.0f);

	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	if(IsGlobalValid())
	{
		Point2D xp;
		UndistortMeasurements();
		const float f = m_G.f();
		for(iFrm = 0; iFrm < nFrms; ++iFrm)
		{
			const Camera &C = m_Cs[iFrm];
			const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
			for(iMea = iMea1; iMea < iMea2; ++iMea)
			{
				iTrk = m_mapMeaToTrk[iMea];
				C.ProjectToNormalizedPlane(m_Xs[iTrk], xp);
				xp *= f;
				ptSSEs[iTrk] += m_xus[iMea].SquaredDistance(xp);
			}
		}
	}
	else
	{
		Point2D e;
		for(iFrm = 0; iFrm < nFrms; ++iFrm)
		{
			const Camera &C = m_Cs[iFrm];
			const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
			for(iMea = iMea1; iMea < iMea2; ++iMea)
			{
				iTrk = m_mapMeaToTrk[iMea];
				ptSSEs[iTrk] += C.ComputeProjectionSquaredError(m_Xs[iTrk], m_xs[iMea], e);
			}
		}
	}

	double SSE = 0;
	for(iTrk = 0; iTrk < nTrks; ++iTrk)
		SSE += ptSSEs[iTrk];
	return SSE;
}

static inline void Project(const Camera &C, const Point3D &X, Point3D &RX, float &ZcI, Point2D &x, LA::AlignedCompactMatrix2x6f &Jc, LA::AlignedMatrix2x3f &Jx, 
						   ENFT_SSE::__m128 *work3)
{
	C.ProjectToNormalizedPlane(X, RX.X(), RX.Y(), RX.Z(), ZcI, x.x(), x.y());
	work3[0] = ENFT_SSE::_mm_set1_ps(ZcI);
	work3[1] = ENFT_SSE::_mm_set1_ps(-x.x() * ZcI);
	work3[2] = ENFT_SSE::_mm_set1_ps(-x.y() * ZcI);
	Jc.M03() = ZcI;									Jc.M04() = 0;									Jc.M05() = work3[1].m128_f32[0];
	Jc.M13() = 0;									Jc.M14() = ZcI;									Jc.M15() = work3[2].m128_f32[0];
	Jc.M00() = Jc.M05()*RX.Y();						Jc.M01() = Jc.M03()*RX.Z()-Jc.M05()*RX.X();		Jc.M02() = -Jc.M03()*RX.Y();
	Jc.M10() = -Jc.M14()*RX.Z()+Jc.M15()*RX.Y();	Jc.M11() = -Jc.M15()*RX.X();					Jc.M12() = Jc.M14()*RX.X();
	Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(work3[1], C.r_20_21_22_x()));
	Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(work3[2], C.r_20_21_22_x()));
}
static inline void Project(const Camera &C, const Point3D &X, float &ZcI, Point2D &x, LA::AlignedMatrix2x3f &Jx, ENFT_SSE::__m128 *work3)
{
	C.ProjectToNormalizedPlane(X, ZcI, x.x(), x.y());
	work3[0] = ENFT_SSE::_mm_set1_ps(ZcI);
	work3[1] = ENFT_SSE::_mm_set1_ps(-x.x() * ZcI);
	work3[2] = ENFT_SSE::_mm_set1_ps(-x.y() * ZcI);
	Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(work3[1], C.r_20_21_22_x()));
	Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(work3[2], C.r_20_21_22_x()));
}
static inline void Project(const Camera &C, const float &f, const Point3D &X, const Point2D &r2x, Point3D &RX, float &ZcI, Point2D &x, 
						   LA::AlignedCompactMatrix2x6f &Jc, LA::AlignedMatrix2x3f &Jx, LA::AlignedMatrix2f &Jg, ENFT_SSE::__m128 *work3, 
						   const bool &distortionInvalid)
{
	C.ProjectToNormalizedPlane(X, RX.X(), RX.Y(), RX.Z(), ZcI, x.x(), x.y());
	Jg.M00() = x.x();
	Jg.M10() = x.y();
	if(distortionInvalid)
	{
		Jg.M01() = 0.0f;
		Jg.M11() = 0.0f;
	}
	else
	{
		Jg.M01() = -r2x.x();
		Jg.M11() = -r2x.y();
	}
	x *= f;
	work3[0] = ENFT_SSE::_mm_set1_ps(f * ZcI);
	work3[1] = ENFT_SSE::_mm_set1_ps(-x.x() * ZcI);
	work3[2] = ENFT_SSE::_mm_set1_ps(-x.y() * ZcI);
	Jc.M03() = work3[0].m128_f32[0];				Jc.M04() = 0;									Jc.M05() = work3[1].m128_f32[0];
	Jc.M13() = 0;									Jc.M14() = work3[0].m128_f32[0];				Jc.M15() = work3[2].m128_f32[0];
	Jc.M00() = Jc.M05()*RX.Y();						Jc.M01() = Jc.M03()*RX.Z()-Jc.M05()*RX.X();		Jc.M02() = -Jc.M03()*RX.Y();
	Jc.M10() = -Jc.M14()*RX.Z()+Jc.M15()*RX.Y();	Jc.M11() = -Jc.M15()*RX.X();					Jc.M12() = Jc.M14()*RX.X();
	Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(work3[1], C.r_20_21_22_x()));
	Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(work3[2], C.r_20_21_22_x()));
}
static inline void Project(const Camera &C, const float &f, const Point3D &X, const Point2D &r2x, float &ZcI, Point2D &x, LA::AlignedMatrix2x3f &Jx, 
						   LA::AlignedMatrix2f &Jg, ENFT_SSE::__m128 *work3, const bool &distortionInvalid)
{
	C.ProjectToNormalizedPlane(X, ZcI, x.x(), x.y());
	Jg.M00() = x.x();
	Jg.M10() = x.y();
	if(distortionInvalid)
	{
		Jg.M01() = 0.0f;
		Jg.M11() = 0.0f;
	}
	else
	{
		Jg.M01() = -r2x.x();
		Jg.M11() = -r2x.y();
	}
	x *= f;
	work3[0] = ENFT_SSE::_mm_set1_ps(f * ZcI);
	work3[1] = ENFT_SSE::_mm_set1_ps(-x.x() * ZcI);
	work3[2] = ENFT_SSE::_mm_set1_ps(-x.y() * ZcI);
	Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(work3[1], C.r_20_21_22_x()));
	Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(work3[2], C.r_20_21_22_x()));
}

#ifdef _DEBUG_WITH_EIGEN
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenMatrix;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> EigenVector;
EigenMatrix g_JTJ, g_A;
EigenVector g_JTE, g_b, g_x;
static inline void Print(const EigenMatrix &M)
{
	int r, c;
	const int nRows = int(M.rows()), nCols = int(M.cols());
	for(r = 0; r < nRows; ++r)
	{
		for(c = 0; c < nCols; ++c)
			printf("%f ", M(r, c));
		printf("\n");
	}
}
static inline void SetMc(const int iEq, const int iC, const LA::AlignedCompactMatrix2x6f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iEq;	c = iC;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();	Mdst(r,c++)=Msrc.M02();	Mdst(r,c++)=Msrc.M03();	Mdst(r,c++)=Msrc.M04();	Mdst(r,c++)=Msrc.M05();
	++r;		c = iC;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();	Mdst(r,c++)=Msrc.M12();	Mdst(r,c++)=Msrc.M13();	Mdst(r,c++)=Msrc.M14();	Mdst(r,c++)=Msrc.M15();
}
static inline void SetMx(const int iEq, const int iX, const LA::AlignedMatrix2x3f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iEq;	c = iX;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();	Mdst(r,c++)=Msrc.M02();
	++r;		c = iX;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();	Mdst(r,c++)=Msrc.M12();
}
static inline void SetMg(const int iEq, const int iG, const LA::AlignedMatrix2f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iEq;	c = iG;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();
	++r;		c = iG;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();
}
static inline void SetVE(const int iEq, const Point2D &Vsrc, EigenVector &Vdst)
{
	int i = iEq;
	Vdst(i++) = Vsrc.x();
	Vdst(i++) = Vsrc.y();
}
static inline void SetMcc(const int iCr, const int iCc, const LA::AlignedCompactMatrix6f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iCr;	c = iCc;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();	Mdst(r,c++)=Msrc.M02();	Mdst(r,c++)=Msrc.M03();	Mdst(r,c++)=Msrc.M04();	Mdst(r,c++)=Msrc.M05();
	++r;		c = iCc;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();	Mdst(r,c++)=Msrc.M12();	Mdst(r,c++)=Msrc.M13();	Mdst(r,c++)=Msrc.M14();	Mdst(r,c++)=Msrc.M15();
	++r;		c = iCc;
	Mdst(r,c++)=Msrc.M20();	Mdst(r,c++)=Msrc.M21();	Mdst(r,c++)=Msrc.M22();	Mdst(r,c++)=Msrc.M23();	Mdst(r,c++)=Msrc.M24();	Mdst(r,c++)=Msrc.M25();
	++r;		c = iCc;
	Mdst(r,c++)=Msrc.M30();	Mdst(r,c++)=Msrc.M31();	Mdst(r,c++)=Msrc.M32();	Mdst(r,c++)=Msrc.M33();	Mdst(r,c++)=Msrc.M34();	Mdst(r,c++)=Msrc.M35();
	++r;		c = iCc;
	Mdst(r,c++)=Msrc.M40();	Mdst(r,c++)=Msrc.M41();	Mdst(r,c++)=Msrc.M42();	Mdst(r,c++)=Msrc.M43();	Mdst(r,c++)=Msrc.M44();	Mdst(r,c++)=Msrc.M45();
	++r;		c = iCc;
	Mdst(r,c++)=Msrc.M50();	Mdst(r,c++)=Msrc.M51();	Mdst(r,c++)=Msrc.M52();	Mdst(r,c++)=Msrc.M53();	Mdst(r,c++)=Msrc.M54();	Mdst(r,c++)=Msrc.M55();
	if(iCr == iCc)
		return;
	Mdst.block<6, 6>(iCc, iCr) = Mdst.block<6, 6>(iCr, iCc).transpose();
}
static inline void SetMxx(const int iXr, const int iXc, const LA::SymmetricMatrix3f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iXr;	c = iXc;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();	Mdst(r,c++)=Msrc.M02();
	++r;		c = iXc;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();	Mdst(r,c++)=Msrc.M12();
	++r;		c = iXc;
	Mdst(r,c++)=Msrc.M20();	Mdst(r,c++)=Msrc.M21();	Mdst(r,c++)=Msrc.M22();
	if(iXr == iXc)
		return;
	Mdst.block<3, 3>(iXc, iXr) = Mdst.block<3, 3>(iXr, iXc).transpose();
}
static inline void SetMgg(const int iGr, const int iGc, const LA::AlignedMatrix2f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iGr;	c = iGc;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();
	++r;		c = iGc;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();
	if(iGr == iGc)
		return;
	Mdst.block<2, 2>(iGc, iGr) = Mdst.block<2, 2>(iGr, iGc).transpose();
}
static inline void SetMcx(const int iC, const int iX, const LA::AlignedCompactMatrix3x6f Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iX;		c = iC;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();	Mdst(r,c++)=Msrc.M02();	Mdst(r,c++)=Msrc.M03();	Mdst(r,c++)=Msrc.M04();	Mdst(r,c++)=Msrc.M05();
	++r;		c = iC;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();	Mdst(r,c++)=Msrc.M12();	Mdst(r,c++)=Msrc.M13();	Mdst(r,c++)=Msrc.M14();	Mdst(r,c++)=Msrc.M15();
	++r;		c = iC;
	Mdst(r,c++)=Msrc.M20();	Mdst(r,c++)=Msrc.M21();	Mdst(r,c++)=Msrc.M22();	Mdst(r,c++)=Msrc.M23();	Mdst(r,c++)=Msrc.M24();	Mdst(r,c++)=Msrc.M25();
	Mdst.block<6, 3>(iC, iX) = Mdst.block<3, 6>(iX, iC).transpose();
	//printf("----------------------------------------------------------------\n");
	//Msrc.Print();
	//printf("----------------------------------------------------------------\n");
	//Print(Mdst.block<6, 3>(iC, iX));
	//printf("----------------------------------------------------------------\n");
	//Print(Mdst.block<3, 6>(iX, iC));
}
static inline void SetMcg(const int iC, const int iG, const LA::AlignedCompactMatrix2x6f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iG;		c = iC;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();	Mdst(r,c++)=Msrc.M02();	Mdst(r,c++)=Msrc.M03();	Mdst(r,c++)=Msrc.M04();	Mdst(r,c++)=Msrc.M05();
	++r;		c = iC;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();	Mdst(r,c++)=Msrc.M12();	Mdst(r,c++)=Msrc.M13();	Mdst(r,c++)=Msrc.M14();	Mdst(r,c++)=Msrc.M15();
	Mdst.block<6, 2>(iC, iG) = Mdst.block<2, 6>(iG, iC).transpose();
}
static inline void SetMxg(const int iX, const int iG, const LA::AlignedMatrix2x3f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iG;		c = iX;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();	Mdst(r,c++)=Msrc.M02();
	++r;		c = iX;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();	Mdst(r,c++)=Msrc.M12();
	Mdst.block<3, 2>(iX, iG) = Mdst.block<2, 3>(iG, iX).transpose();
}
static inline void SetVc(const int iC, const LA::AlignedVector6f &Vsrc, EigenVector &Vdst)
{
	int i = iC;
	Vdst(i++) = Vsrc.v0();
	Vdst(i++) = Vsrc.v1();
	Vdst(i++) = Vsrc.v2();
	Vdst(i++) = Vsrc.v3();
	Vdst(i++) = Vsrc.v4();
	Vdst(i++) = Vsrc.v5();
}
static inline void SetVx(const int iX, const LA::Vector3f &Vsrc, EigenVector &Vdst)
{
	int i = iX;
	Vdst(i++) = Vsrc.v0();
	Vdst(i++) = Vsrc.v1();
	Vdst(i++) = Vsrc.v2();
}
static inline void SetVg(const int iG, const LA::Vector2f &Vsrc, EigenVector &Vdst)
{
	int i = iG;
	Vdst(i++) = Vsrc.v0();
	Vdst(i++) = Vsrc.v1();
}
static inline void GetIndex(const int iC, const int iX, const int iG, const int p, int &i, int &j, char &s)
{
	if(p < iX)
	{
		s = 'C';
		i = p / 6;
		j = p % 6;
	}
	else if(p < iG)
	{
		s = 'X';
		i = (p - iX) / 3;
		j = (p - iX) % 3;
	}
	else
	{
		s = 'G';
		i = 0;
		j = p - iG;
	}
}
static inline void CheckMcxg(const int iC, const int iX, const int iG, const EigenMatrix &M1, const EigenMatrix &M2)
{
	int r, c, ir, jr, ic, jc;
	char sr, sc;
	float v1, v2;
	const int nRows = int(M1.rows()), nCols = int(M1.cols());
	for(r = 0; r < nRows; ++r)
	for(c = 0; c < nCols; ++c)
	{
		v1 = M1(r, c);
		v2 = M2(r, c);
		if(EQUAL(v1, v2))
			continue;
		GetIndex(iC, iX, iG, r, ir, jr, sr);
		GetIndex(iC, iX, iG, c, ic, jc, sc);
		printf("(%c%d, %c%d)(%d, %d): %f - %f = %f\n", sr, ir, sc, ic, jr, jc, v1, v2, v1 - v2);
	}
}
static inline void CheckVcxg(const int iC, const int iX, const int iG, const EigenVector &V1, const EigenVector &V2)
{
	int r, i, j;
	char s;
	float v1, v2;
	const int nRows = int(V1.rows());
	for(r = 0; r < nRows; ++r)
	{
		v1 = V1(r);
		v2 = V2(r);
		if(EQUAL(v1, v2))
			continue;
		GetIndex(iC, iX, iG, r, i, j, s);
		printf("(%c%d)(%d): %f - %f = %f\n", s, i, j, v1, v2, v1 - v2);
	}
}
static inline void PrintA(const int iC, const int iX, const int iG, const EigenMatrix &A)
{
	const int nRows = int(A.rows()), nCols = int(A.cols());
	int r, c, i, j;
	char s;
	float v;
	for(r = 0; r < nRows; ++r)
	for(c = 0; c < nCols; ++c)
	{
		if((v = A(r, c)) == 0)
			continue;
		GetIndex(iC, iX, iG, c, i, j, s);
		printf("[%d](%c%d)(%d): %f\n", r, s, i, j, v);
	}
}
#endif

void SequenceBundleAdjustorData::ConstructNormalEquation(AlignedVector<LA::AlignedCompactMatrix6f> &Dcs, AlignedVector<LA::SymmetricMatrix3f> &Dxs, 
														 LA::AlignedMatrix2f &Dg, AlignedVector<LA::AlignedCompactMatrix3x6f> &Wxcs, 
														 AlignedVector<LA::AlignedCompactMatrix2x6f> &Wgcs, AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, 
														 AlignedVector<LA::AlignedVector6f> &bcs, AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg, 
														 AlignedVector<LA::AlignedVector6f> &scs, AlignedVector<LA::Vector3f> &sxs, LA::Vector2f &sg)
{
#ifdef _DEBUG_WITH_EIGEN
	const int nEqs = int(m_xs.Size() * 2), nVars = int(scs.Size()) * 6 + int(sxs.Size()) * 3 + 2;
	const int iC = 0, iX = int(scs.Size()) * 6, iG = int(scs.Size()) * 6 + int(sxs.Size()) * 3;
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
	EigenMatrix J;
	EigenVector E;
	int iEq = 0;
	J.resize(nEqs, nVars);
	J.setZero();
	E.resize(nEqs);
	E.setZero();
	Wxcs.SetZero();
#endif
#endif

	const bool global = IsGlobalValid();
	if(global)
		UndistortMeasurements();

	Dcs.SetZero();
	Dxs.SetZero();
	Dg.SetZero();
	Wgcs.SetZero();
	Wgxs.SetZero();
	bcs.SetZero();
	bxs.SetZero();
	bg.SetZero();
	scs.SetZero();
	sxs.SetZero();
	sg.SetZero();

	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	float ZcI;
	Point2D xp, e;
	Point3D RX;
	LA::AlignedCompactMatrix2x6f Jc;
	LA::AlignedMatrix2x3f Jx;
	LA::AlignedMatrix2f Jg;
	ENFT_SSE::__m128 work[3];
	const float f = m_G.f();
	const FrameIndex nFrms = FrameIndex(m_Cs.Size()), nFrmsFix = nFrms - FrameIndex(scs.Size());
	for(iFrm = 0; iFrm < nFrmsFix; ++iFrm)
	{
		const Camera &C = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			if(global)
			{
				Project(C, f, m_Xs[iTrk], m_r2xs[iMea], ZcI, xp, Jx, Jg, work, m_distortionInvalid);
				LA::AmB(m_xus[iMea], xp, e);
				LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
				LA::AddAij2To(Jx, sxs[iTrk], work[0]);
				LA::AddATAToUpper(Jg, Dg, work[0]);
				LA::AddATBTo(Jg, Jx, Wgxs[iTrk]);
				LA::AddATBTo(Jg, e, bg, work[0]);
				LA::AddAij2To(Jg, sg, work[0]);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
				SetMx(iEq, iX + iTrk * 3, Jx, J);
				SetMg(iEq, iG, Jg, J);
				SetVE(iEq, e, E);
				iEq += 2;
#endif
			}
			else
			{
				Project(C, m_Xs[iTrk], ZcI, xp, Jx, work);
				LA::AmB(m_xs[iMea], xp, e);
				LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
				LA::AddAij2To(Jx, sxs[iTrk], work[0]);
			}
		}
	}
	FrameIndex i;
	MeasurementIndex j;
	for(i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		const Camera &C = m_Cs[iFrm];
		LA::AlignedCompactMatrix6f &Dc = Dcs[i];
		LA::AlignedVector6f &bc = bcs[i], &sc = scs[i];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			if(global)
			{
				Project(C, f, m_Xs[iTrk], m_r2xs[iMea], RX, ZcI, xp, Jc, Jx, Jg, work, m_distortionInvalid);
				LA::AmB(m_xus[iMea], xp, e);
				LA::AddATAToUpper(Jc, Dc);
				LA::AddATBTo(Jc, e, bc, work);
				LA::AddAij2To(Jc, sc);
				LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
				LA::AddAij2To(Jx, sxs[iTrk], work[0]);
				LA::ATB(Jx, Jc, Wxcs[j], work);
				LA::AddATAToUpper(Jg, Dg, work[0]);
				LA::AddATBTo(Jg, Jc, Wgcs[i], work);
				LA::AddATBTo(Jg, Jx, Wgxs[iTrk]);
				LA::AddATBTo(Jg, e, bg, work[0]);
				LA::AddAij2To(Jg, sg, work[0]);
				//printf("Jc\n");
				//Jc.Print();
				//printf("Jc\n");
				//Jx.Print();
				//printf("Jg\n");
				//Jg.Print();
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
				SetMc(iEq, iC + i * 6, Jc, J);
				SetMx(iEq, iX + iTrk * 3, Jx, J);
				SetMg(iEq, iG, Jg, J);
				SetVE(iEq, e, E);
				iEq += 2;
#endif
			}
			else
			{
				Project(C, m_Xs[iTrk], RX, ZcI, xp, Jc, Jx, work);
				LA::AmB(m_xs[iMea], xp, e);
				LA::AddATAToUpper(Jc, Dc);
				LA::AddATBTo(Jc, e, bc, work);
				LA::AddAij2To(Jc, sc);
				LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
				LA::AddAij2To(Jx, sxs[iTrk], work[0]);
				LA::ATB(Jx, Jc, Wxcs[j], work);
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedCompactMatrix2x6f>(Dc);
		LA::SetLowerFromUpper(Dc);
		LA::FinishAdditionATBTo<LA::AlignedCompactMatrix2x6f, LA::Vector2f>(bc);
		LA::SetReserve<BA_STAGE_B>(bc);
		LA::FinishAdditionAij2To<LA::AlignedCompactMatrix2x6f>(sc);
		LA::MakeReciprocal(sc);
		LA::MakeSquareRoot(sc);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
		sc.v0123() = sc.v45xx() = ENFT_SSE::_mm_set1_ps(1.0f);
#endif
		LA::SetReserve<BA_STAGE_S>(sc);
		LA::ssTA(sc, Dc, work);
		LA::sA(sc, bc);
	}

	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		LA::SymmetricMatrix3f &Dx = Dxs[iTrk];
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x3f>(Dx);
		LA::SetLowerFromUpper(Dx);
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x3f, LA::Vector2f>(bxs[iTrk]);
		LA::Vector3f &sx = sxs[iTrk];
		LA::FinishAdditionAij2To<LA::AlignedMatrix2x3f>(sx);
		LA::MakeReciprocal(sx);
		LA::MakeSquareRoot(sx);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
		sx.v0() = sx.v1() = sx.v2() = 1.0f;
#endif
		LA::SetReserve<BA_STAGE_S>(sx);
		LA::ssTA(sx, Dx);
		LA::sA(sx, bxs[iTrk]);
		LA::SetReserve<BA_STAGE_B>(bxs[iTrk]);
	}

	for(iFrm = nFrmsFix, i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		const LA::AlignedVector6f &sc = scs[i];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			LA::s1s2TA(sxs[iTrk], sc, Wxcs[j], work);
		}
	}
	if(global)
	{
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2f>(Dg);
		LA::SetLowerFromUpper(Dg);
		LA::FinishAdditionATBTo<LA::AlignedMatrix2f, LA::Vector2f>(bg);
		LA::SetReserve<BA_STAGE_B>(bg);
		LA::FinishAdditionAij2To<LA::AlignedMatrix2f>(sg);
		LA::MakeReciprocal(sg);
		LA::MakeSquareRoot(sg);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
		sg.v0() = sg.v1() = 1.0f;
#endif
		LA::SetReserve<BA_STAGE_S>(sg);
		LA::ssTA(sg, Dg);
		LA::sA(sg, bg);
		for(iFrm = nFrmsFix, i = 0; iFrm < nFrms; ++iFrm, ++i)
			LA::s1s2TA(sg, scs[i], Wgcs[i], work);
		for(iTrk = 0; iTrk < nTrks; ++iTrk)
			LA::s1s2TA(sg, sxs[iTrk], Wgxs[iTrk], work[0]);
	}

//#if _DEBUG
//	printf("...\n");
//	Dg.SetZero();
//	Dg.MakeIdentity();
//	Wgcs.SetZero();
//	Wgxs.SetZero();
//	bg.SetZero();
//#endif

#ifdef _DEBUG_WITH_EIGEN
	g_JTJ.resize(nVars, nVars);
	g_JTJ.setZero();
	g_JTE.resize(nVars);
	g_JTE.setZero();
	for(iFrm = nFrmsFix, i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		SetMcc(iC + i * 6, iC + i * 6, Dcs[i], g_JTJ);
		SetVc(iC + i * 6, bcs[i], g_JTE);
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			SetMcx(iC + i * 6, iX + iTrk * 3, Wxcs[j], g_JTJ);
		}
		SetMcg(iC + i * 6, iG, Wgcs[i], g_JTJ);
	}
	for(iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		SetMxx(iX + iTrk * 3, iX + iTrk * 3, Dxs[iTrk], g_JTJ);
		SetVx(iX + iTrk * 3, bxs[iTrk], g_JTE);
		SetMxg(iX + iTrk * 3, iG, Wgxs[iTrk], g_JTJ);
	}
	SetMgg(iG, iG, Dg, g_JTJ);
	SetVg(iG, bg, g_JTE);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
	//PrintA(iC, iX, iG, J);
	const EigenMatrix JT = J.transpose();
	const EigenMatrix JTJ1 = JT * J, &JTJ2 = g_JTJ;
	const EigenMatrix JTE1 = JT * E, &JTE2 = g_JTE;
	CheckMcxg(iC, iX, iG, JTJ1, JTJ2);
	CheckVcxg(iC, iX, iG, JTE1, JTE2);
#endif
#endif
}

void SequenceBundleAdjustorData::ConstructNormalEquation(const AlignedVector<LA::AlignedVector6f> &scs, const AlignedVector<LA::Vector3f> &sxs, 
														 const LA::Vector2f &sg, AlignedVector<LA::AlignedCompactMatrix6f> &Dcs, 
														 AlignedVector<LA::SymmetricMatrix3f> &Dxs, LA::AlignedMatrix2f &Dg, 
														 AlignedVector<LA::AlignedCompactMatrix3x6f> &Wxcs, AlignedVector<LA::AlignedCompactMatrix2x6f> &Wgcs, 
														 AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, AlignedVector<LA::AlignedVector6f> &bcs, 
														 AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg)
{
#ifdef _DEBUG_WITH_EIGEN
	const int nEqs = int(m_xs.Size() * 2), nVars = int(scs.Size()) * 6 + int(sxs.Size()) * 3 + 2;
	const int iC = 0, iX = int(scs.Size()) * 6, iG = int(scs.Size()) * 6 + int(sxs.Size()) * 3;
#endif

	const bool global = IsGlobalValid();
	if(global)
		UndistortMeasurements();

	Dcs.SetZero();
	Dxs.SetZero();
	Dg.SetZero();
	Wgcs.SetZero();
	Wgxs.SetZero();
	bcs.SetZero();
	bxs.SetZero();
	bg.SetZero();

	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	float ZcI;
	Point2D xp, e;
	Point3D RX;
	LA::AlignedCompactMatrix2x6f Jc;
	LA::AlignedMatrix2x3f Jx;
	LA::AlignedMatrix2f Jg;
	ENFT_SSE::__m128 work[3];
	const float f = m_G.f();
	const FrameIndex nFrms = FrameIndex(m_Cs.Size()), nFrmsFix = nFrms - FrameIndex(scs.Size());
	for(iFrm = 0; iFrm < nFrmsFix; ++iFrm)
	{
		const Camera &C = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			if(global)
			{
				Project(C, f, m_Xs[iTrk], m_r2xs[iMea], ZcI, xp, Jx, Jg, work, m_distortionInvalid);
				LA::AmB(m_xus[iMea], xp, e);
				LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
				LA::AddATAToUpper(Jg, Dg, work[0]);
				LA::AddATBTo(Jg, Jx, Wgxs[iTrk]);
				LA::AddATBTo(Jg, e, bg, work[0]);
			}
			else
			{
				Project(C, m_Xs[iTrk], ZcI, xp, Jx, work);
				LA::AmB(m_xs[iMea], xp, e);
				LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
			}
		}
	}
	FrameIndex i;
	MeasurementIndex j;
	for(i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		const Camera &C = m_Cs[iFrm];
		LA::AlignedCompactMatrix6f &Dc = Dcs[i];
		LA::AlignedVector6f &bc = bcs[i];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			if(global)
			{
				Project(C, f, m_Xs[iTrk], m_r2xs[iMea], RX, ZcI, xp, Jc, Jx, Jg, work, m_distortionInvalid);
				LA::AmB(m_xus[iMea], xp, e);
				LA::AddATAToUpper(Jc, Dc);
				LA::AddATBTo(Jc, e, bc, work);
				LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
				LA::ATB(Jx, Jc, Wxcs[j], work);
				LA::AddATAToUpper(Jg, Dg, work[0]);
				LA::AddATBTo(Jg, Jc, Wgcs[i], work);
				LA::AddATBTo(Jg, Jx, Wgxs[iTrk]);
				LA::AddATBTo(Jg, e, bg, work[0]);
				//printf("Jc\n");
				//Jc.Print();
				//printf("Jc\n");
				//Jx.Print();
				//printf("Jg\n");
				//Jg.Print();
			}
			else
			{
				Project(C, m_Xs[iTrk], RX, ZcI, xp, Jc, Jx, work);
				LA::AmB(m_xs[iMea], xp, e);
				LA::AddATAToUpper(Jc, Dc);
				LA::AddATBTo(Jc, e, bc, work);
				LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
				LA::ATB(Jx, Jc, Wxcs[j], work);
			}
		}
		LA::FinishAdditionATAToUpper<LA::AlignedCompactMatrix2x6f>(Dc);
		LA::SetLowerFromUpper(Dc);
		LA::FinishAdditionATBTo<LA::AlignedCompactMatrix2x6f, LA::Vector2f>(bc);
		LA::SetReserve<BA_STAGE_B>(bc);
		const LA::AlignedVector6f &sc = scs[i];
		LA::ssTA(sc, Dc, work);
		LA::sA(sc, bc);
	}

	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		LA::SymmetricMatrix3f &Dx = Dxs[iTrk];
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2x3f>(Dx);
		LA::SetLowerFromUpper(Dx);
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x3f, LA::Vector2f>(bxs[iTrk]);
		const LA::Vector3f &sx = sxs[iTrk];
		LA::ssTA(sx, Dx);
		LA::sA(sx, bxs[iTrk]);
		LA::SetReserve<BA_STAGE_B>(bxs[iTrk]);
	}

	for(iFrm = nFrmsFix, i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		const LA::AlignedVector6f &sc = scs[i];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			LA::s1s2TA(sxs[iTrk], sc, Wxcs[j], work);
		}
	}
	if(global)
	{
		LA::FinishAdditionATAToUpper<LA::AlignedMatrix2f>(Dg);
		LA::SetLowerFromUpper(Dg);
		LA::FinishAdditionATBTo<LA::AlignedMatrix2f, LA::Vector2f>(bg);
		LA::SetReserve<BA_STAGE_B>(bg);
		LA::ssTA(sg, Dg);
		LA::sA(sg, bg);
		for(iFrm = nFrmsFix, i = 0; iFrm < nFrms; ++iFrm, ++i)
			LA::s1s2TA(sg, scs[i], Wgcs[i], work);
		for(iTrk = 0; iTrk < nTrks; ++iTrk)
			LA::s1s2TA(sg, sxs[iTrk], Wgxs[iTrk], work[0]);
	}

#ifdef _DEBUG_WITH_EIGEN
	g_JTJ.resize(nVars, nVars);
	g_JTJ.setZero();
	g_JTE.resize(nVars);
	g_JTE.setZero();
	for(iFrm = nFrmsFix, i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		SetMcc(iC + i * 6, iC + i * 6, Dcs[i], g_JTJ);
		SetVc(iC + i * 6, bcs[i], g_JTE);
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			SetMcx(iC + i * 6, iX + iTrk * 3, Wxcs[j], g_JTJ);
		}
		SetMcg(iC + i * 6, iG, Wgcs[i], g_JTJ);
	}
	for(iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		SetMxx(iX + iTrk * 3, iX + iTrk * 3, Dxs[iTrk], g_JTJ);
		SetVx(iX + iTrk * 3, bxs[iTrk], g_JTE);
		SetMxg(iX + iTrk * 3, iG, Wgxs[iTrk], g_JTJ);
	}
	SetMgg(iG, iG, Dg, g_JTJ);
	SetVg(iG, bg, g_JTE);
#endif
}

void SequenceBundleAdjustorData::UpdateCameras(const AlignedVector<LA::AlignedVector6f> &scs, const AlignedVector<LA::AlignedVector6f> &xcs, 
											   const AlignedVector<Camera> &CsOld)
{
	RotationTransformation3D dR;
	ENFT_SSE::__m128 workm;
	float workf[24];
	const FrameIndex nFrms = FrameIndex(m_Cs.Size()), nFrmsFix = nFrms - FrameIndex(scs.Size());
	for(FrameIndex iFrm = nFrmsFix, i = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		const LA::AlignedVector6f &sc = scs[i], &xc = xcs[i];
		const Camera &Cold = CsOld[iFrm];
		Camera &Cnew = m_Cs[iFrm];
		workm = ENFT_SSE::_mm_mul_ps(sc.v0123(), xc.v0123());
		Cnew.tX() = workm.m128_f32[3] + Cold.tX();
		Cnew.tY() = sc.v4() * xc.v4() + Cold.tY();
		Cnew.tZ() = sc.v5() * xc.v5() + Cold.tZ();
		dR.FromRodrigues(workm.m128_f32[0], workm.m128_f32[1], workm.m128_f32[2], workf);
		Cold.LeftMultiplyRotation(dR, Cnew, workm);
	}
}

void SequenceBundleAdjustorData::UpdatePoints(const AlignedVector<LA::Vector3f> &sxs, const AlignedVector<LA::Vector3f> &xxs, const AlignedVector<Point3D> &XsOld)
{
	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		const LA::Vector3f &sx = sxs[iTrk], &xx = xxs[iTrk];
		const Point3D &Xold = XsOld[iTrk];
		Point3D &Xnew = m_Xs[iTrk];
		Xnew.X() = sx.v0() * xx.v0() + Xold.X();
		Xnew.Y() = sx.v1() * xx.v1() + Xold.Y();
		Xnew.Z() = sx.v2() * xx.v2() + Xold.Z();
	}
}

void SequenceBundleAdjustorData::UpdateGlobal(const LA::Vector2f &sg, const LA::Vector2f &xg, const Camera::IntrinsicParameter &Gold)
{
#if _DEBUG
	assert(IsGlobalValid());
#endif
	m_G.f() = Gold.f() + sg.v0() * xg.v0();
	m_G.d() = Gold.d() + sg.v1() * xg.v1();
}

#if _DEBUG
void SequenceBundleAdjustorData::DebugSchurComplement(const float damping, const std::vector<std::pair<FrameIndex, FrameIndex> > &iPairs, 
													  const AlignedVector<LA::AlignedCompactMatrix6f> &Accs, const LA::AlignedMatrix2f &Agg, 
													  const AlignedVector<LA::AlignedCompactMatrix2x6f> &Agcs, const AlignedVector<LA::AlignedVector6f> &bcs, 
													  const LA::Vector2f &bg)
{
#ifdef _DEBUG_WITH_EIGEN
	const int nC = int(bcs.Size()) * 6, nX = int(m_Xs.Size()) * 3, nG = 2, nVars = nC + nX + nG;
	const int iC = 0, iX = nC, iG = nC + nX;
	
	int i;
	g_A.resize(nVars, nVars);
	g_A.setZero();
	g_b.resize(nVars);
	g_b.setZero();
	const int nPairs = int(iPairs.size());
	for(i = 0; i < nPairs; ++i)
		SetMcc(iC + iPairs[i].first * 6, iC + iPairs[i].second * 6, Accs[i], g_A);
	SetMgg(iG, iG, Agg, g_A);
	const int nFrmsAdj = int(bcs.Size());
	for(i = 0; i < nFrmsAdj; ++i)
	{
		SetMcg(iC + i * 6, iG, Agcs[i], g_A);
		SetVc(iC + i * 6, bcs[i], g_b);
	}
	SetVg(iG, bg, g_b);

	const float lambda = damping + 1;
	EigenMatrix Dc = g_JTJ.block(iC, iC, nC, nC), Dx = g_JTJ.block(iX, iX, nX, nX), Dg = g_JTJ.block(iG, iG, nG, nG);
	for(i = 0; i < nC; ++i)
		Dc(i, i) *= lambda;
	for(i = 0; i < nX; ++i)
		Dx(i, i) *= lambda;
	for(i = 0; i < nG; ++i)
		Dg(i, i) *= lambda;
	const EigenMatrix DxI = Dx.inverse(), Wxc = g_JTJ.block(iX, iC, nX, nC), Wgc = g_JTJ.block(iG, iC, nG, nC), Wgx = g_JTJ.block(iG, iX, nG, nX);
	const EigenMatrix Ycx = Wxc.transpose() * DxI, Ygx = Wgx * DxI;
	const EigenVector JTEc = g_JTE.block(iC, 0, nC, 1), JTEx = g_JTE.block(iX, 0, nX, 1), JTEg = g_JTE.block(iG, 0, nG, 1);

	EigenMatrix A1;
	EigenVector b1;
	A1.resize(nVars, nVars);
	A1.setZero();
	b1.resize(nVars);
	b1.setZero();
	A1.block(iC, iC, nC, nC) = Dc - Ycx * Wxc;
	A1.block(iG, iC, nG, nC) = Wgc - Wgx * DxI * Wxc;
	A1.block(iC, iG, nC, nG) = A1.block(iG, iC, nG, nC).transpose();
	A1.block(iG, iG, nG, nG) = Dg - Ygx * Wgx.transpose();
	b1.block(iC, 0, nC, 1) = JTEc - Ycx * JTEx;
	b1.block(iG, 0, nG, 1) = JTEg - Ygx * JTEx;
	const EigenMatrix &A2 = g_A;
	const EigenVector &b2 = g_b;
	CheckMcxg(iC, iX, iG, A1, A2);
	CheckVcxg(iC, iX, iG, b1, b2);
#endif
}

void SequenceBundleAdjustorData::DebugSolution(const float damping, const AlignedVector<LA::AlignedVector6f> &xcs, const AlignedVector<LA::Vector3f> &xxs, 
											   const LA::Vector2f &xg)
{
#ifdef _DEBUG_WITH_EIGEN
	const int nC = int(xcs.Size()) * 6, nX = int(xxs.Size()) * 3, nG = 2, nVars = nC + nX + nG;
	const int iC = 0, iX = nC, iG = nC + nX;
	g_x.resize(nVars);
	g_x.setZero();
	
	int i;
	const int nFrmsAdj = int(xcs.Size());
	for(i = 0; i < nFrmsAdj; ++i)
		SetVc(iC + i * 6, xcs[i], g_x);
	const int nTrks = int(xxs.Size());
	for(i = 0; i < nTrks; ++i)
		SetVx(iX + i * 3, xxs[i], g_x);
	SetVg(iG, xg, g_x);

	EigenMatrix A = g_JTJ;
	EigenVector b = g_JTE;
	const float lambda = damping + 1;
	for(i = 0; i < nVars; ++i)
		A(i, i) *= lambda;
	const EigenVector x1 = A.ldlt().solve(b), &x2 = g_x;
	CheckVcxg(iC, iX, iG, x1, x2);
	//const EigenVector Ax1 = A * x1, Ax2 = A * x2;
	//CheckVcxg(iC, iX, iG, Ax1, b);
	//CheckVcxg(iC, iX, iG, Ax2, b);
#endif
}

void SequenceBundleAdjustorData::DebugAx(const AlignedVector<LA::AlignedVector6f> &xcs, const LA::Vector2f &xg, const AlignedVector<LA::AlignedVector6f> &Axcs, 
										 const LA::Vector2f &Axg)
{
#ifdef _DEBUG_WITH_EIGEN_AX
	const int nC = int(xcs.Size()) * 6, nX = int(m_Xs.Size()) * 3, nG = 2;
	const int iC = 0, iX = nC, iG = nC + nX;

	EigenMatrix A;
	A.resize(nC + nG, nC + nG);
	A.setZero();
	A.block(iC, iC, nC, nC) = g_A.block(iC, iC, nC, nC);
	A.block(nC, nC, nG, nG) = g_A.block(iG, iG, nG, nG);
	A.block(iC, nC, nC, nG) = g_A.block(iC, iG, nC, nG);
	A.block(nC, iC, nG, nC) = g_A.block(iG, iC, nG, nC);

	EigenVector x;
	x.resize(nC + nG);
	x.setZero();
	const int nFrmsAdj = int(xcs.Size());
	for(int i = 0; i < nFrmsAdj; ++i)
		SetVc(iC + i * 6, xcs[i], x);
	SetVg(nC, xg, x);

	EigenVector Ax;
	Ax.resize(nC + nG);
	Ax.setZero();
	for(int i = 0; i < nFrmsAdj; ++i)
		SetVc(iC + i * 6, Axcs[i], Ax);
	SetVg(nC, Axg, Ax);

	const EigenVector Ax1 = A * x, &Ax2 = Ax;
	CheckVcxg(iC, nC, nC, Ax1, Ax2);
#endif
}
#endif