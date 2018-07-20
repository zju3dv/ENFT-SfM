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
#include "Sequence/SequenceBundleAdjustorDataDepth.h"
#if _DEBUG
//#define _DEBUG_WITH_EIGEN
#ifdef _DEBUG_WITH_EIGEN
#define _DEBUG_WITH_EIGEN_JTJ_JTE
#define _DEBUG_WITH_EIGEN_AX
#include <Eigen>
#endif
#endif

void SequenceBundleAdjustorDataDepth::ValidateGlobal()
{
}

void SequenceBundleAdjustorDataDepth::InvalidateGlobal()
{
}

bool SequenceBundleAdjustorDataDepth::IsGlobalValid() const
{
	return false;
}

void SequenceBundleAdjustorDataDepth::ComputeSceneCenter(Point3D &center)
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

void SequenceBundleAdjustorDataDepth::TranslateScene(const Point3D &translation)
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

void SequenceBundleAdjustorDataDepth::ScaleScene(const float &scale)
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
	const MeasurementIndex nMeas = MeasurementIndex(m_xs.Size());
	for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea)
	{
		LA::Vector3f &x = m_xs[iMea];
		if(x.v2() != 0.0f)
			x.v2() *= scale;
	}
}

void SequenceBundleAdjustorDataDepth::NormalizeData(const float dataNormalizeMedian)
{
	if(dataNormalizeMedian == 0)
	{
		m_scaleScene = 1.0f;
		m_translation.SetZero();
		return;
	}
	Point3D center;
	ComputeSceneCenter(center);
	//printf("...\n");
	//center.SetZero();
	m_translation.Set(-center.X(), -center.Y(), -center.Z());
	TranslateScene(m_translation);

	MeasurementIndex i, j;
	const MeasurementIndex nMeas = MeasurementIndex(m_xs.Size());
	m_ds.resize(nMeas);
	for(i = j = 0; i < nMeas; ++i)
	{
		if((m_ds[j] = m_xs[i].v2()) > 0.0f)
			++j;
	}
	const MeasurementIndex nDeps = j;
	if(nDeps == 0)
	{
		m_scaleScene = 1.0f;
		return;
	}
	m_ds.resize(nDeps);
	const MeasurementIndex ith = MeasurementIndex(nDeps >> 1);
	std::nth_element(m_ds.begin(), m_ds.begin() + ith, m_ds.end());
	const float dMed = m_ds[ith];
	m_scaleScene = (1 / dMed) / dataNormalizeMedian;
	ScaleScene(m_scaleScene);
}

void SequenceBundleAdjustorDataDepth::DenormalizeData()
{
	m_scaleScene = 1 / m_scaleScene;
	ScaleScene(m_scaleScene);
	m_scaleScene = 1;

	m_translation.v0() = -m_translation.v0();
	m_translation.v1() = -m_translation.v1();
	m_translation.v2() = -m_translation.v2();
	TranslateScene(m_translation);
	m_translation.SetZero();
}

double SequenceBundleAdjustorDataDepth::ComputeSSE()
{
	double SSE1 = 0.0, SSE2 = 0.0;
	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	Point2D xp, e1;
	float Zc, e2;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		const Camera &C = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			C.ProjectToNormalizedPlane(m_Xs[iTrk], Zc, xp);
			const LA::Vector3f &x = m_xs[iMea];
			e1.x() = x.v0() - xp.x();
			e1.y() = x.v1() - xp.y();
			SSE1 += e1.SquaredLength();
			if(x.v2() == 0.0f)
				continue;
			e2 = x.v2() - Zc;
			SSE2 += e2 * e2;
		}
	}
	return SSE1 + m_depthWeight * SSE2;
}

double SequenceBundleAdjustorDataDepth::ComputeSSE(std::vector<float> &ptSSEs)
{
	const TrackIndex nTrks = GetPointsNumber();
	ptSSEs.assign(nTrks, 0.0f);

	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	Point2D xp, e1;
	float Zc, e2;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		const Camera &C = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			//ptSSEs[iTrk] += C.ComputeProjectionSquaredError(m_Xs[iTrk], m_xs[iMea], e);

			C.ProjectToNormalizedPlane(m_Xs[iTrk], Zc, xp);
			const LA::Vector3f &x = m_xs[iMea];
			e1.x() = x.v0() - xp.x();
			e1.y() = x.v1() - xp.y();
			ptSSEs[iTrk] += e1.SquaredLength();
			if(x.v2() == 0.0f)
				continue;
			e2 = x.v2() - Zc;
			ptSSEs[iTrk] += m_depthWeight * e2 * e2;
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
static inline void Project(const ENFT_SSE::__m128 &w, const Camera &C, const Point3D &X, Point3D &RX, float &ZcI, Point3D &x, LA::AlignedCompactMatrix3x6f &Jc, 
						   LA::AlignedMatrix3f &Jx, ENFT_SSE::__m128 *work3)
{
	C.Apply(X, RX, x);
	ZcI = 1 / x.Z();
	x.X() *= ZcI;
	x.Y() *= ZcI;
	work3[0] = ENFT_SSE::_mm_set1_ps(ZcI);
	work3[1] = ENFT_SSE::_mm_set1_ps(-x.X() * ZcI);
	work3[2] = ENFT_SSE::_mm_set1_ps(-x.Y() * ZcI);
	Jc.M03() = ZcI;									Jc.M04() = 0;									Jc.M05() = work3[1].m128_f32[0];
	Jc.M13() = 0;									Jc.M14() = ZcI;									Jc.M15() = work3[2].m128_f32[0];
	Jc.M23() = 0;									Jc.M24() = 0;									Jc.M25() = w.m128_f32[0];
	Jc.M00() = Jc.M05()*RX.Y();						Jc.M01() = Jc.M03()*RX.Z()-Jc.M05()*RX.X();		Jc.M02() = -Jc.M03()*RX.Y();
	Jc.M10() = -Jc.M14()*RX.Z()+Jc.M15()*RX.Y();	Jc.M11() = -Jc.M15()*RX.X();					Jc.M12() = Jc.M14()*RX.X();
	Jc.M20() = w.m128_f32[0] * RX.Y();				Jc.M21() = -w.m128_f32[0] * RX.X();				Jc.M22() = 0;
	Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(work3[1], C.r_20_21_22_x()));
	Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(work3[2], C.r_20_21_22_x()));
	Jx.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(w, C.r_20_21_22_x());
}
static inline void Project(const ENFT_SSE::__m128 &w, const Camera &C, const Point3D &X, float &ZcI, Point3D &x, LA::AlignedMatrix3f &Jx, ENFT_SSE::__m128 *work3)
{
	C.Apply(X, x);
	ZcI = 1 / x.Z();
	x.X() *= ZcI;
	x.Y() *= ZcI;
	work3[0] = ENFT_SSE::_mm_set1_ps(ZcI);
	work3[1] = ENFT_SSE::_mm_set1_ps(-x.X() * ZcI);
	work3[2] = ENFT_SSE::_mm_set1_ps(-x.Y() * ZcI);
	Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_00_01_02_x()), ENFT_SSE::_mm_mul_ps(work3[1], C.r_20_21_22_x()));
	Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], C.r_10_11_12_x()), ENFT_SSE::_mm_mul_ps(work3[2], C.r_20_21_22_x()));
	Jx.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(w, C.r_20_21_22_x());
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
static inline void SetMc(const int iEq, const int iC, const LA::AlignedCompactMatrix3x6f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iEq;	c = iC;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();	Mdst(r,c++)=Msrc.M02();	Mdst(r,c++)=Msrc.M03();	Mdst(r,c++)=Msrc.M04();	Mdst(r,c++)=Msrc.M05();
	++r;		c = iC;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();	Mdst(r,c++)=Msrc.M12();	Mdst(r,c++)=Msrc.M13();	Mdst(r,c++)=Msrc.M14();	Mdst(r,c++)=Msrc.M15();
	++r;		c = iC;
	Mdst(r,c++)=Msrc.M20();	Mdst(r,c++)=Msrc.M21();	Mdst(r,c++)=Msrc.M22();	Mdst(r,c++)=Msrc.M23();	Mdst(r,c++)=Msrc.M24();	Mdst(r,c++)=Msrc.M25();
}
static inline void SetMx(const int iEq, const int iX, const LA::AlignedMatrix2x3f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iEq;	c = iX;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();	Mdst(r,c++)=Msrc.M02();
	++r;		c = iX;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();	Mdst(r,c++)=Msrc.M12();
}
static inline void SetMx(const int iEq, const int iX, const LA::AlignedMatrix3f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iEq;	c = iX;
	Mdst(r,c++)=Msrc.M00();	Mdst(r,c++)=Msrc.M01();	Mdst(r,c++)=Msrc.M02();
	++r;		c = iX;
	Mdst(r,c++)=Msrc.M10();	Mdst(r,c++)=Msrc.M11();	Mdst(r,c++)=Msrc.M12();
	++r;		c = iX;
	Mdst(r,c++)=Msrc.M20();	Mdst(r,c++)=Msrc.M21();	Mdst(r,c++)=Msrc.M22();
}
static inline void SetVE(const int iEq, const Point2D &Vsrc, EigenVector &Vdst)
{
	int i = iEq;
	Vdst(i++) = Vsrc.x();
	Vdst(i++) = Vsrc.y();
}
static inline void SetVE(const int iEq, const Point3D &Vsrc, EigenVector &Vdst)
{
	int i = iEq;
	Vdst(i++) = Vsrc.X();
	Vdst(i++) = Vsrc.Y();
	Vdst(i++) = Vsrc.Z();
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
	Mdst(r, c++) = Msrc.M00();	Mdst(r, c++) = Msrc.M01();	Mdst(r, c++) = Msrc.M02();
	++r;		c = iXc;
	Mdst(r, c++) = Msrc.M10();	Mdst(r, c++) = Msrc.M11();	Mdst(r, c++) = Msrc.M12();
	++r;		c = iXc;
	Mdst(r, c++) = Msrc.M20();	Mdst(r, c++) = Msrc.M21();	Mdst(r, c++) = Msrc.M22();
	if(iXr == iXc)
		return;
	Mdst.block<3, 3>(iXc, iXr) = Mdst.block<3, 3>(iXr, iXc).transpose();
}
static inline void SetMcx(const int iC, const int iX, const LA::AlignedCompactMatrix3x6f Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iX;		c = iC;
	Mdst(r, c++) = Msrc.M00();	Mdst(r, c++) = Msrc.M01();	Mdst(r, c++) = Msrc.M02();	Mdst(r, c++) = Msrc.M03();	Mdst(r, c++) = Msrc.M04();	Mdst(r, c++) = Msrc.M05();
	++r;		c = iC;
	Mdst(r, c++) = Msrc.M10();	Mdst(r, c++) = Msrc.M11();	Mdst(r, c++) = Msrc.M12();	Mdst(r, c++) = Msrc.M13();	Mdst(r, c++) = Msrc.M14();	Mdst(r, c++) = Msrc.M15();
	++r;		c = iC;
	Mdst(r, c++) = Msrc.M20();	Mdst(r, c++) = Msrc.M21();	Mdst(r, c++) = Msrc.M22();	Mdst(r, c++) = Msrc.M23();	Mdst(r, c++) = Msrc.M24();	Mdst(r, c++) = Msrc.M25();
	Mdst.block<6, 3>(iC, iX) = Mdst.block<3, 6>(iX, iC).transpose();
	//printf("----------------------------------------------------------------\n");
	//Msrc.Print();
	//printf("----------------------------------------------------------------\n");
	//Print(Mdst.block<6, 3>(iC, iX));
	//printf("----------------------------------------------------------------\n");
	//Print(Mdst.block<3, 6>(iX, iC));
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
static inline void GetIndex(const int iX, const int p, int &i, int &j, char &s)
{
	if(p < iX)
	{
		s = 'C';
		i = p / 6;
		j = p % 6;
	}
	else
	{
		s = 'X';
		i = (p - iX) / 3;
		j = (p - iX) % 3;
	}
}
static inline void CheckMcx(const int iX, const EigenMatrix &M1, const EigenMatrix &M2)
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
		GetIndex(iX, r, ir, jr, sr);
		GetIndex(iX, c, ic, jc, sc);
		printf("(%c%d, %c%d)(%d, %d): %f - %f = %f\n", sr, ir, sc, ic, jr, jc, v1, v2, v1 - v2);
	}
}
static inline void CheckVcx(const int iX, const EigenVector &V1, const EigenVector &V2)
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
		GetIndex(iX, r, i, j, s);
		printf("(%c%d)(%d): %f - %f = %f\n", s, i, j, v1, v2, v1 - v2);
	}
}
static inline void PrintA(const int iX, const EigenMatrix &A)
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
		GetIndex(iX, c, i, j, s);
		printf("[%d](%c%d)(%d): %f\n", r, s, i, j, v);
	}
}
static uint CountDepths(const AlignedVector<LA::Vector3f> &xs)
{
	uint cnt = 0;
	const uint N = xs.Size();
	for(uint i = 0; i < N; ++i)
	{
		if(xs[i].v2() != 0.0f)
			++cnt;
	}
	return cnt;
}
#endif

void SequenceBundleAdjustorDataDepth::ConstructNormalEquation(AlignedVector<LA::AlignedCompactMatrix6f> &Dcs, AlignedVector<LA::SymmetricMatrix3f> &Dxs, 
															  LA::AlignedMatrix2f &Dg, AlignedVector<LA::AlignedCompactMatrix3x6f> &Wxcs, 
															  AlignedVector<LA::AlignedCompactMatrix2x6f> &Wgcs, AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, 
															  AlignedVector<LA::AlignedVector6f> &bcs, AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg, 
															  AlignedVector<LA::AlignedVector6f> &scs, AlignedVector<LA::Vector3f> &sxs, LA::Vector2f &sg)
{
#ifdef _DEBUG_WITH_EIGEN
	const int nEqs = int(m_xs.Size() * 2) + int(CountDepths(m_xs)), nVars = int(scs.Size()) * 6 + int(sxs.Size()) * 3;
	const int iX = int(scs.Size()) * 6;
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
	Point2D x1, e1;
	Point3D x2, e2, RX;
	LA::AlignedCompactMatrix2x6f Jc1;
	LA::AlignedMatrix2x3f Jx1;
	LA::AlignedCompactMatrix3x6f Jc2;
	LA::AlignedMatrix3f Jx2;
	ENFT_SSE::__m128 work[3];
	Jc2.SetReserveZero();
	const ENFT_SSE::__m128 w = ENFT_SSE::_mm_set1_ps(m_depthWeight);
	const FrameIndex nFrms = FrameIndex(m_Cs.Size()), nFrmsFix = nFrms - FrameIndex(scs.Size());
	for(iFrm = 0; iFrm < nFrmsFix; ++iFrm)
	{
		const Camera &C = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			const LA::Vector3f &x = m_xs[iMea];
			if(x.v2() == 0.0f)
			{
				Project(C, m_Xs[iTrk], ZcI, x1, Jx1, work);
				e1.x() = x.v0() - x1.x();
				e1.y() = x.v1() - x1.y();
				LA::AddATAToUpper(Jx1, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx1, e1, bxs[iTrk], work[0]);
				LA::AddAij2To(Jx1, sxs[iTrk], work[0]);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
				SetMx(iEq, iX + iTrk * 3, Jx1, J);
				SetVE(iEq, e1, E);
				iEq += 2;
#endif
			}
			else
			{
				Project(w, C, m_Xs[iTrk], ZcI, x2, Jx2, work);
				e2.X() = x.v0() - x2.X();
				e2.Y() = x.v1() - x2.Y();
				e2.Z() = m_depthWeight * (x.v2() - x2.Z());
				LA::AddATAToUpper(Jx2, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx2, e2, bxs[iTrk], work[0]);
				LA::AddAij2To(Jx2, sxs[iTrk], work[0]);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
				SetMx(iEq, iX + iTrk * 3, Jx2, J);
				SetVE(iEq, e2, E);
				iEq += 3;
#endif
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
			const LA::Vector3f &x = m_xs[iMea];
			if(x.v2() == 0.0f)
			{
				Project(C, m_Xs[iTrk], RX, ZcI, x1, Jc1, Jx1, work);
				e1.x() = x.v0() - x1.x();
				e1.y() = x.v1() - x1.y();
				LA::AddATAToUpper(Jc1, Dc);
				LA::AddATBTo(Jc1, e1, bc, work);
				LA::AddAij2To(Jc1, sc);
				LA::AddATAToUpper(Jx1, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx1, e1, bxs[iTrk], work[0]);
				LA::AddAij2To(Jx1, sxs[iTrk], work[0]);
				LA::ATB(Jx1, Jc1, Wxcs[j], work);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
				SetMc(iEq, i * 6, Jc1, J);
				SetMx(iEq, iX + iTrk * 3, Jx1, J);
				SetVE(iEq, e1, E);
				iEq += 2;
#endif
			}
			else
			{
				Project(w, C, m_Xs[iTrk], RX, ZcI, x2, Jc2, Jx2, work);
				e2.X() = x.v0() - x2.X();
				e2.Y() = x.v1() - x2.Y();
				e2.Z() = m_depthWeight * (x.v2() - x2.Z());
				LA::AddATAToUpper(Jc2, Dc);
				LA::AddATBTo(Jc2, e2, bc, work);
				LA::AddAij2To(Jc2, sc);
				LA::AddATAToUpper(Jx2, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx2, e2, bxs[iTrk], work[0]);
				LA::AddAij2To(Jx2, sxs[iTrk], work[0]);
				LA::ATB(Jx2, Jc2, Wxcs[j], work);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
				SetMc(iEq, i * 6, Jc2, J);
				SetMx(iEq, iX + iTrk * 3, Jx2, J);
				SetVE(iEq, e2, E);
				iEq += 3;
#endif
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
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
	assert(iEq == nEqs);
#endif

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

#ifdef _DEBUG_WITH_EIGEN
	g_JTJ.resize(nVars, nVars);
	g_JTJ.setZero();
	g_JTE.resize(nVars);
	g_JTE.setZero();
	for(iFrm = nFrmsFix, i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		SetMcc(i * 6, i * 6, Dcs[i], g_JTJ);
		SetVc(i * 6, bcs[i], g_JTE);
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			SetMcx(i * 6, iX + iTrk * 3, Wxcs[j], g_JTJ);
		}
	}
	for(iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		SetMxx(iX + iTrk * 3, iX + iTrk * 3, Dxs[iTrk], g_JTJ);
		SetVx(iX + iTrk * 3, bxs[iTrk], g_JTE);
	}
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
	//PrintA(iC, iX, iG, J);
	const EigenMatrix JT = J.transpose();
	const EigenMatrix JTJ1 = JT * J, &JTJ2 = g_JTJ;
	const EigenMatrix JTE1 = JT * E, &JTE2 = g_JTE;
	CheckMcx(iX, JTJ1, JTJ2);
	CheckVcx(iX, JTE1, JTE2);
#endif
#endif
}

void SequenceBundleAdjustorDataDepth::ConstructNormalEquation(const AlignedVector<LA::AlignedVector6f> &scs, const AlignedVector<LA::Vector3f> &sxs, 
															  const LA::Vector2f &sg, AlignedVector<LA::AlignedCompactMatrix6f> &Dcs, 
															  AlignedVector<LA::SymmetricMatrix3f> &Dxs, LA::AlignedMatrix2f &Dg, 
															  AlignedVector<LA::AlignedCompactMatrix3x6f> &Wxcs, AlignedVector<LA::AlignedCompactMatrix2x6f> &Wgcs,
															  AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, AlignedVector<LA::AlignedVector6f> &bcs, 
															  AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg)
{
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
	Point2D x1, e1;
	Point3D x2, e2, RX;
	LA::AlignedCompactMatrix2x6f Jc1;
	LA::AlignedMatrix2x3f Jx1;
	LA::AlignedCompactMatrix3x6f Jc2;
	LA::AlignedMatrix3f Jx2;
	ENFT_SSE::__m128 work[3];
	Jc2.SetReserveZero();
	const ENFT_SSE::__m128 w = ENFT_SSE::_mm_set1_ps(m_depthWeight);
	const FrameIndex nFrms = FrameIndex(m_Cs.Size()), nFrmsFix = nFrms - FrameIndex(scs.Size());
	for(iFrm = 0; iFrm < nFrmsFix; ++iFrm)
	{
		const Camera &C = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			const LA::Vector3f &x = m_xs[iMea];
			if(x.v2() == 0.0f)
			{
				Project(C, m_Xs[iTrk], ZcI, x1, Jx1, work);
				e1.x() = x.v0() - x1.x();
				e1.y() = x.v1() - x1.y();
				LA::AddATAToUpper(Jx1, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx1, e1, bxs[iTrk], work[0]);
			}
			else
			{
				Project(w, C, m_Xs[iTrk], ZcI, x2, Jx2, work);
				e2.X() = x.v0() - x2.X();
				e2.Y() = x.v1() - x2.Y();
				e2.Z() = m_depthWeight * (x.v2() - x2.Z());
				LA::AddATAToUpper(Jx2, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx2, e2, bxs[iTrk], work[0]);
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
			const LA::Vector3f &x = m_xs[iMea];
			if(x.v2() == 0.0f)
			{
				Project(C, m_Xs[iTrk], RX, ZcI, x1, Jc1, Jx1, work);
				e1.x() = x.v0() - x1.x();
				e1.y() = x.v1() - x1.y();
				LA::AddATAToUpper(Jc1, Dc);
				LA::AddATBTo(Jc1, e1, bc, work);
				LA::AddATAToUpper(Jx1, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx1, e1, bxs[iTrk], work[0]);
				LA::ATB(Jx1, Jc1, Wxcs[j], work);
			}
			else
			{
				Project(w, C, m_Xs[iTrk], RX, ZcI, x2, Jc2, Jx2, work);
				e2.X() = x.v0() - x2.X();
				e2.Y() = x.v1() - x2.Y();
				e2.Z() = m_depthWeight * (x.v2() - x2.Z());
				LA::AddATAToUpper(Jc2, Dc);
				LA::AddATBTo(Jc2, e2, bc, work);
				LA::AddATAToUpper(Jx2, Dxs[iTrk], work[0]);
				LA::AddATBTo(Jx2, e2, bxs[iTrk], work[0]);
				LA::ATB(Jx2, Jc2, Wxcs[j], work);
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
}

void SequenceBundleAdjustorDataDepth::UpdateCameras(const AlignedVector<LA::AlignedVector6f> &scs, const AlignedVector<LA::AlignedVector6f> &xcs, 
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

void SequenceBundleAdjustorDataDepth::UpdatePoints(const AlignedVector<LA::Vector3f> &sxs, const AlignedVector<LA::Vector3f> &xxs, 
												   const AlignedVector<Point3D> &XsOld)
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

void SequenceBundleAdjustorDataDepth::UpdateGlobal(const LA::Vector2f &sg, const LA::Vector2f &xg, const Camera::IntrinsicParameter &Gold)
{
}