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
#include "SequenceBundleAdjustorDataProjective.h"
#if _DEBUG
//#define _DEBUG_WITH_EIGEN
#ifdef _DEBUG_WITH_EIGEN
//#define _DEBUG_WITH_EIGEN_JTJ_JTE
#define _DEBUG_WITH_EIGEN_AX
#include <Eigen>
#endif
#endif

void SequenceBundleAdjustorDataProjective::ValidateGlobal()
{
}

void SequenceBundleAdjustorDataProjective::InvalidateGlobal()
{
}

bool SequenceBundleAdjustorDataProjective::IsGlobalValid() const
{
	return false;
}

void SequenceBundleAdjustorDataProjective::ComputeMeasurementDepths(std::vector<float> &depths)
{
	depths.resize(m_xs.Size());

	FrameIndex iFrm;
	MeasurementIndex iMea;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		const ProjectiveMatrix &P = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
			depths[iMea] = P.ComputeDepth(m_Xs[m_mapMeaToTrk[iMea]]);
	}
}

void SequenceBundleAdjustorDataProjective::ComputeSceneCenter(Point3D &center)
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

void SequenceBundleAdjustorDataProjective::TranslateScene(const Point3D &translation)
{
	if(translation.SquaredLength() == 0)
		return;
	const ENFT_SSE::__m128 t = ENFT_SSE::_mm_setr_ps(translation.X(), translation.Y(), translation.Z(), 1);
	Point3D dt;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		ProjectiveMatrix &P = m_Cs[iFrm];
		dt.X() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(P.M_00_01_02_03(), t));
		dt.Y() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(P.M_10_11_12_13(), t));
		dt.Z() = -ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(P.M_20_21_22_23(), t));
		P.M03() += dt.X();
		P.M13() += dt.Y();
		P.M23() += dt.Z();
	}

	const ENFT_SSE::__m128 dX = ENFT_SSE::_mm_setr_ps(translation.X(), translation.Y(), translation.Z(), 0);
	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
		m_Xs[iTrk].XYZx() = ENFT_SSE::_mm_add_ps(m_Xs[iTrk].XYZx(), dX);
}

void SequenceBundleAdjustorDataProjective::ScaleScene(const float &scale)
{
	if(scale == 1.0f)
		return;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		ProjectiveMatrix &P = m_Cs[iFrm];
		P.M03() *= scale;
		P.M13() *= scale;
		P.M23() *= scale;
	}
	const ENFT_SSE::__m128 s = ENFT_SSE::_mm_setr_ps(scale, scale, scale, 1);
	const TrackIndex nTrks = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
		m_Xs[iTrk].Scale(s);
}

void SequenceBundleAdjustorDataProjective::NormalizeData(const float dataNormalizeMedian)
{
	if(dataNormalizeMedian == 0)
	{
		m_scale = 1.0f;
		m_translation.SetZero();
		return;
	}
	Point3D center;
	ComputeSceneCenter(center);
	//printf("...\n");
	//center.SetZero();
	m_translation.Set(-center.X(), -center.Y(), -center.Z());
	TranslateScene(m_translation);

	ComputeMeasurementDepths(m_ds);
	const MeasurementIndex ith = MeasurementIndex(m_ds.size() >> 1);
	std::nth_element(m_ds.begin(), m_ds.begin() + ith, m_ds.end());
	const float dMed = m_ds[ith];
	m_scale = (1 / dMed) / dataNormalizeMedian;
	ScaleScene(m_scale);
}

void SequenceBundleAdjustorDataProjective::DenormalizeData()
{
	m_scale = 1 / m_scale;
	ScaleScene(m_scale);
	m_scale = 1;

	m_translation.v0() = -m_translation.v0();
	m_translation.v1() = -m_translation.v1();
	m_translation.v2() = -m_translation.v2();
	TranslateScene(m_translation);
	m_translation.SetZero();
}

double SequenceBundleAdjustorDataProjective::ComputeSSE()
{
	double SSE = 0;
	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	Point2D e;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		const ProjectiveMatrix &P = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			SSE += P.ComputeProjectionSquaredError(m_Xs[iTrk], m_xs[iMea], e);
		}
	}

#if 0
	float errSq, errSqMax = 0;
	MeasurementIndex iMeaMax = 0;
	const float f = m_G.f();
	for(iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		const ProjectiveMatrix &P = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			if((errSq = P.ComputeProjectionSquaredError(m_Xs[iTrk], m_xs[iMea], e)) < errSqMax)
				continue;
			errSqMax = errSq;
			iMeaMax = iMea;
		}
	}
	Point2D x;
	const FrameIndex iFrmMax = m_mapMeaToFrm[iMeaMax];
	const TrackIndex iTrkMax = m_mapMeaToTrk[iMeaMax];
	m_Cs[iFrmMax].Project(m_Xs[iTrkMax], x);
	printf("----------------------------------------------------------------\n");
	printf("Error <= ||(%f, %f) - (%f, %f)|| == %f --> %f\n", m_xs[iMeaMax].x(), m_xs[iMeaMax].y(), x.x(), x.y(), errSqMax, errSqMax * m_fxy);
	printf("iFrm = %d, iTrk = %d, iMea = %d\n", iFrmMax, iTrkMax, iMeaMax);
#endif

	return SSE;
}

double SequenceBundleAdjustorDataProjective::ComputeSSE(std::vector<float> &ptSSEs)
{
	const TrackIndex nTrks = GetPointsNumber();
	ptSSEs.assign(nTrks, 0.0f);

	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	Point2D e;
	const FrameIndex nFrms = FrameIndex(m_Cs.Size());
	for(iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		const ProjectiveMatrix &P = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			ptSSEs[iTrk] += P.ComputeProjectionSquaredError(m_Xs[iTrk], m_xs[iMea], e);
		}
	}

	double SSE = 0;
	for(iTrk = 0; iTrk < nTrks; ++iTrk)
		SSE += ptSSEs[iTrk];
	return SSE;
}

static inline void Project(const ProjectiveMatrix &P, const Point3D &X, float &ZcI, Point2D &x, LA::AlignedMatrix2x11f &Jc, LA::AlignedMatrix2x3f &Jx, ENFT_SSE::__m128 *work3)
{
	P.Project(X, ZcI, x);
	work3[0] = ENFT_SSE::_mm_set1_ps(ZcI);
	work3[1] = ENFT_SSE::_mm_set1_ps(-x.x() * ZcI);
	work3[2] = ENFT_SSE::_mm_set1_ps(-x.y() * ZcI);
#if _DEBUG
	assert(X.reserve() == 1.0f);
#endif
	Jc.M_00_01_02_03() = Jc.M_14_15_16_17() = ENFT_SSE::_mm_mul_ps(work3[0], X.XYZx());
	Jc.M_04_05_06_07() = Jc.M_10_11_12_13() = ENFT_SSE::_mm_setzero_ps();
	Jc.M_08_09_010_x() = ENFT_SSE::_mm_mul_ps(work3[1], X.XYZx());
	Jc.M_18_19_110_x() = ENFT_SSE::_mm_mul_ps(work3[2], X.XYZx());
	Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], P.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], P.M_20_21_22_23()));
	Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], P.M_10_11_12_13()), ENFT_SSE::_mm_mul_ps(work3[2], P.M_20_21_22_23()));
}
static inline void Project(const ProjectiveMatrix &P, const Point3D &X, float &ZcI, Point2D &x, LA::AlignedMatrix2x3f &Jx, ENFT_SSE::__m128 *work3)
{
	P.Project(X, ZcI, x);
	work3[0] = ENFT_SSE::_mm_set1_ps(ZcI);
	work3[1] = ENFT_SSE::_mm_set1_ps(-x.x() * ZcI);
	work3[2] = ENFT_SSE::_mm_set1_ps(-x.y() * ZcI);
	Jx.M_00_01_02_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], P.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(work3[1], P.M_20_21_22_23()));
	Jx.M_10_11_12_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work3[0], P.M_10_11_12_13()), ENFT_SSE::_mm_mul_ps(work3[2], P.M_20_21_22_23()));
}

static inline void AddATAToUpper(const LA::AlignedMatrix2x11f &A, LA::AlignedMatrix11f &to, ENFT_SSE::__m128 &work)
{
	work = ENFT_SSE::_mm_set1_ps(A.M00());
	to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.M_00_01_02_03()), to.M_00_01_02_03());
	to.M_08_09_010_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.M_08_09_010_x()), to.M_08_09_010_x());
	work = ENFT_SSE::_mm_set1_ps(A.M01());
	to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.M_00_01_02_03()), to.M_10_11_12_13());
	to.M_18_19_110_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work, A.M_08_09_010_x()), to.M_18_19_110_x());
	to.M22() = A.M02() * A.M02() + to.M22();
	to.M23() = A.M02() * A.M03() + to.M23();
	to.M_28_29_210_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), A.M_08_09_010_x()), to.M_28_29_210_x());
	to.M33() = A.M03() * A.M03() + to.M33();
	to.M_38_39_310_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M03()), A.M_08_09_010_x()), to.M_38_39_310_x());
	to.M_48_49_410_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M14()), A.M_18_19_110_x()), to.M_48_49_410_x());
	to.M_58_59_510_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M15()), A.M_18_19_110_x()), to.M_58_59_510_x());
	to.M_68_69_610_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M16()), A.M_18_19_110_x()), to.M_68_69_610_x());
	to.M_78_79_710_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M17()), A.M_18_19_110_x()), to.M_78_79_710_x());
	to.M_88_89_810_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M08()), A.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M18()), A.M_18_19_110_x())), to.M_88_89_810_x());
	to.M99() = A.M09() * A.M09() + A.M19() * A.M19() + to.M99();
	to.M910() = A.M09() * A.M010() + A.M19() * A.M110() + to.M910();
	to.M1010() = A.M010() * A.M010() + A.M110() * A.M110() + to.M1010();
}
static inline void FinishAdditionATAToUpper(LA::AlignedMatrix11f &to)
{
	to.M_44_45_46_47() = to.M_00_01_02_03();
	to.M_54_55_56_57() = to.M_10_11_12_13();
	to.M66() = to.M22();
	to.M67() = to.M23();
	to.M77() = to.M33();
}
static inline void AddATBTo(const LA::AlignedMatrix2x11f &A, const LA::Vector2f &B, LA::AlignedVector11f &to, ENFT_SSE::__m128 *work2)
{
	work2[0] = ENFT_SSE::_mm_set1_ps(B.v0());
	work2[1] = ENFT_SSE::_mm_set1_ps(B.v1());
	to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), work2[0]), to.v0123());
	to.v4567() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_14_15_16_17(), work2[1]), to.v4567());
	to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), work2[0]), ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), work2[1])), to.v8910x());
}
static inline void AddAij2To(const LA::AlignedMatrix2x11f &A, LA::AlignedVector11f &to)
{
	to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), A.M_00_01_02_03()), to.v0123());
	to.v8910x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_08_09_010_x(), A.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(A.M_18_19_110_x(), A.M_18_19_110_x())), to.v8910x());
}
static inline void FinishAdditionAij2To(LA::AlignedVector11f &to)
{
	to.v4567() = to.v0123();
}
static inline void ATB(const LA::AlignedMatrix2x3f &A, const LA::AlignedMatrix2x11f &B, LA::AlignedMatrix3x11f &ATB, ENFT_SSE::__m128 *work2)
{
	work2[0] = ENFT_SSE::_mm_set1_ps(A.M00());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M10());
	ATB.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03());
	ATB.M_04_05_06_07() = ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17());
	ATB.M_08_09_010_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_18_19_110_x()));
	work2[0] = ENFT_SSE::_mm_set1_ps(A.M01());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M11());
	ATB.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03());
	ATB.M_14_15_16_17() = ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17());
	ATB.M_18_19_110_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_18_19_110_x()));
	work2[0] = ENFT_SSE::_mm_set1_ps(A.M02());	work2[1] = ENFT_SSE::_mm_set1_ps(A.M12());
	ATB.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(work2[0], B.M_00_01_02_03());
	ATB.M_24_25_26_27() = ENFT_SSE::_mm_mul_ps(work2[1], B.M_14_15_16_17());
	ATB.M_28_29_210_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(work2[0], B.M_08_09_010_x()), ENFT_SSE::_mm_mul_ps(work2[1], B.M_18_19_110_x()));
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
static inline void SetMc(const int iEq, const int iC, const LA::AlignedMatrix2x11f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iEq;	c = iC;
	Mdst(r, c++) = Msrc.M00();	Mdst(r, c++) = Msrc.M01();	Mdst(r, c++) = Msrc.M02();	Mdst(r, c++) = Msrc.M03();	Mdst(r, c++) = Msrc.M04();	Mdst(r, c++) = Msrc.M05();
	Mdst(r, c++) = Msrc.M06();	Mdst(r, c++) = Msrc.M07();	Mdst(r, c++) = Msrc.M08();	Mdst(r, c++) = Msrc.M09();	Mdst(r, c++) = Msrc.M010();
	++r;		c = iC;
	Mdst(r, c++) = Msrc.M10();	Mdst(r, c++) = Msrc.M11();	Mdst(r, c++) = Msrc.M12();	Mdst(r, c++) = Msrc.M13();	Mdst(r, c++) = Msrc.M14();	Mdst(r, c++) = Msrc.M15();
	Mdst(r, c++) = Msrc.M16();	Mdst(r, c++) = Msrc.M17();	Mdst(r, c++) = Msrc.M18();	Mdst(r, c++) = Msrc.M19();	Mdst(r, c++) = Msrc.M110();
}
static inline void SetMx(const int iEq, const int iX, const LA::AlignedMatrix2x3f &Msrc, EigenMatrix &Mdst)
{
	int r, c;
	r = iEq;	c = iX;
	Mdst(r, c++) = Msrc.M00();	Mdst(r, c++) = Msrc.M01();	Mdst(r, c++) = Msrc.M02();
	++r;		c = iX;
	Mdst(r, c++) = Msrc.M10();	Mdst(r, c++) = Msrc.M11();	Mdst(r, c++) = Msrc.M12();
}
static inline void SetVE(const int iEq, const Point2D &Vsrc, EigenVector &Vdst)
{
	int i = iEq;
	Vdst(i++) = Vsrc.x();
	Vdst(i++) = Vsrc.y();
}
static inline void SetMcc(const int iCr, const int iCc, const LA::AlignedMatrix11f &Msrc, EigenMatrix &Mdst)
{
	int i, j, r, c;
	const float *pMsrc = Msrc;
	for(i = 0, r = iCr; i < 11; ++i, ++r, pMsrc += 12)
	for(j = 0, c = iCc; j < 11; ++j, ++c)
		Mdst(r, c) = pMsrc[j];
	if(iCr == iCc)
		return;
	Mdst.block<11, 11>(iCc, iCr) = Mdst.block<11, 11>(iCr, iCc).transpose();
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
static inline void SetMcx(const int iC, const int iX, const LA::AlignedMatrix3x11f Msrc, EigenMatrix &Mdst)
{
	int i, j, r, c;
	const float *pMsrc = Msrc;
	for(i = 0, r = iX; i < 3; ++i, ++r, pMsrc += 12)
	for(j = 0, c = iC; j < 11; ++j, ++c)
		Mdst(r, c) = pMsrc[j];
	Mdst.block<11, 3>(iC, iX) = Mdst.block<3, 11>(iX, iC).transpose();
}
static inline void SetVc(const int iC, const LA::AlignedVector11f &Vsrc, EigenVector &Vdst)
{
	int i = iC;
	Vdst(i++) = Vsrc.v0();
	Vdst(i++) = Vsrc.v1();
	Vdst(i++) = Vsrc.v2();
	Vdst(i++) = Vsrc.v3();
	Vdst(i++) = Vsrc.v4();
	Vdst(i++) = Vsrc.v5();
	Vdst(i++) = Vsrc.v6();
	Vdst(i++) = Vsrc.v7();
	Vdst(i++) = Vsrc.v8();
	Vdst(i++) = Vsrc.v9();
	Vdst(i++) = Vsrc.v10();
}
static inline void SetVx(const int iX, const LA::Vector3f &Vsrc, EigenVector &Vdst)
{
	int i = iX;
	Vdst(i++) = Vsrc.v0();
	Vdst(i++) = Vsrc.v1();
	Vdst(i++) = Vsrc.v2();
}
static inline void GetIndex(const int iC, const int iX, const int p, int &i, int &j, char &s)
{
	if(p < iX)
	{
		s = 'C';
		i = p / 11;
		j = p % 11;
	}
	else
	{
		s = 'X';
		i = (p - iX) / 3;
		j = (p - iX) % 3;
	}
}
static inline void CheckMcx(const int iC, const int iX, const EigenMatrix &M1, const EigenMatrix &M2)
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
		GetIndex(iC, iX, r, ir, jr, sr);
		GetIndex(iC, iX, c, ic, jc, sc);
		printf("(%c%d, %c%d)(%d, %d): %f - %f = %f\n", sr, ir, sc, ic, jr, jc, v1, v2, v1 - v2);
	}
}
static inline void CheckVcx(const int iC, const int iX, const EigenVector &V1, const EigenVector &V2)
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
		GetIndex(iC, iX, r, i, j, s);
		printf("(%c%d)(%d): %f - %f = %f\n", s, i, j, v1, v2, v1 - v2);
	}
}
static inline void PrintA(const int iC, const int iX, const EigenMatrix &A)
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
		GetIndex(iC, iX, c, i, j, s);
		printf("[%d](%c%d)(%d): %f\n", r, s, i, j, v);
	}
}
#endif

void SequenceBundleAdjustorDataProjective::ConstructNormalEquation(AlignedVector<LA::AlignedMatrix11f> &Dcs, AlignedVector<LA::SymmetricMatrix3f> &Dxs, 
																   LA::AlignedMatrix2f &Dg, AlignedVector<LA::AlignedMatrix3x11f> &Wxcs, 
																   AlignedVector<LA::AlignedMatrix2x11f> &Wgcs, AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, 
																   AlignedVector<LA::AlignedVector11f> &bcs, AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg, 
																   AlignedVector<LA::AlignedVector11f> &scs, AlignedVector<LA::Vector3f> &sxs, LA::Vector2f &sg)
{
#ifdef _DEBUG_WITH_EIGEN
	const int nEqs = int(m_xs.Size() * 2), nVars = int(scs.Size()) * 11 + int(sxs.Size()) * 3;
	const int iC = 0, iX = int(scs.Size()) * 11;
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
	bcs.SetZero();
	bxs.SetZero();
	scs.SetZero();
	sxs.SetZero();

	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	float ZcI;
	Point2D xp, e;
	LA::AlignedMatrix2x11f Jc;
	LA::AlignedMatrix2x3f Jx;
	ENFT_SSE::__m128 work[3];
	const FrameIndex nFrms = FrameIndex(m_Cs.Size()), nFrmsFix = nFrms - FrameIndex(scs.Size());
	for(iFrm = 0; iFrm < nFrmsFix; ++iFrm)
	{
		const ProjectiveMatrix &P = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			Project(P, m_Xs[iTrk], ZcI, xp, Jx, work);
			LA::AmB(m_xs[iMea], xp, e);
			LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
			LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
			LA::AddAij2To(Jx, sxs[iTrk], work[0]);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
			SetMx(iEq, iX + iTrk * 3, Jx, J);
			SetVE(iEq, e, E);
			iEq += 2;
#endif
		}
	}
	FrameIndex i;
	MeasurementIndex j;
	for(i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		const ProjectiveMatrix &P = m_Cs[iFrm];
		LA::AlignedMatrix11f &Dc = Dcs[i];
		LA::AlignedVector11f &bc = bcs[i], &sc = scs[i];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			Project(P, m_Xs[iTrk], ZcI, xp, Jc, Jx, work);
//#if _DEBUG
//			Jc.Print();
//#endif
			LA::AmB(m_xs[iMea], xp, e);
			AddATAToUpper(Jc, Dc, work[0]);
			::AddATBTo(Jc, e, bc, work);
			AddAij2To(Jc, sc);
			LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
			LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
			LA::AddAij2To(Jx, sxs[iTrk], work[0]);
			ATB(Jx, Jc, Wxcs[j], work);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
			SetMc(iEq, iC + i * 11, Jc, J);
			SetMx(iEq, iX + iTrk * 3, Jx, J);
			SetVE(iEq, e, E);
			iEq += 2;
#endif
		}
		FinishAdditionATAToUpper(Dc);
		LA::SetLowerFromUpper(Dc);
//#if _DEBUG
//		Dc.Print();
//#endif
		LA::SetReserve<BA_STAGE_B>(bc);
		FinishAdditionAij2To(sc);
		LA::MakeReciprocal(sc);
		LA::MakeSquareRoot(sc);
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
		sc.v0123() = sc.v4567() = sc.v8910x() = ENFT_SSE::_mm_set1_ps(1.0f);
#endif
		LA::SetReserve<BA_STAGE_S>(sc);
		LA::ssTA(sc, Dc, work[0]);
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
		const LA::AlignedVector11f &sc = scs[i];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			LA::s1s2TA(sxs[iTrk], sc, Wxcs[j], work[0]);
		}
	}

#ifdef _DEBUG_WITH_EIGEN
	g_JTJ.resize(nVars, nVars);
	g_JTJ.setZero();
	g_JTE.resize(nVars);
	g_JTE.setZero();
	for(iFrm = nFrmsFix, i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		SetMcc(iC + i * 11, iC + i * 11, Dcs[i], g_JTJ);
		SetVc(iC + i * 11, bcs[i], g_JTE);
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			SetMcx(iC + i * 11, iX + iTrk * 3, Wxcs[j], g_JTJ);
		}
	}
	for(iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		SetMxx(iX + iTrk * 3, iX + iTrk * 3, Dxs[iTrk], g_JTJ);
		SetVx(iX + iTrk * 3, bxs[iTrk], g_JTE);
	}
#ifdef _DEBUG_WITH_EIGEN_JTJ_JTE
	//PrintA(iC, iX, J);
	const EigenMatrix JT = J.transpose();
	const EigenMatrix JTJ1 = JT * J, &JTJ2 = g_JTJ;
	const EigenMatrix JTE1 = JT * E, &JTE2 = g_JTE;
	CheckMcx(iC, iX, JTJ1, JTJ2);
	CheckVcx(iC, iX, JTE1, JTE2);
#endif
#endif
}

void SequenceBundleAdjustorDataProjective::ConstructNormalEquation(const AlignedVector<LA::AlignedVector11f> &scs, const AlignedVector<LA::Vector3f> &sxs, const LA::Vector2f &sg, 
														 AlignedVector<LA::AlignedMatrix11f> &Dcs, AlignedVector<LA::SymmetricMatrix3f> &Dxs, 
														 LA::AlignedMatrix2f &Dg, AlignedVector<LA::AlignedMatrix3x11f> &Wxcs, 
														 AlignedVector<LA::AlignedMatrix2x11f> &Wgcs, AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, 
														 AlignedVector<LA::AlignedVector11f> &bcs, AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg)
{
#ifdef _DEBUG_WITH_EIGEN
	const int nEqs = int(m_xs.Size() * 2), nVars = int(scs.Size()) * 11 + int(sxs.Size()) * 3;
	const int iC = 0, iX = int(scs.Size()) * 11;
#endif

	Dcs.SetZero();
	Dxs.SetZero();
	bcs.SetZero();
	bxs.SetZero();

	FrameIndex iFrm;
	TrackIndex iTrk;
	MeasurementIndex iMea;
	float ZcI;
	Point2D xp, e;
	LA::AlignedMatrix2x11f Jc;
	LA::AlignedMatrix2x3f Jx;
	ENFT_SSE::__m128 work[3];
	const FrameIndex nFrms = FrameIndex(m_Cs.Size()), nFrmsFix = nFrms - FrameIndex(scs.Size());
	for(iFrm = 0; iFrm < nFrmsFix; ++iFrm)
	{
		const ProjectiveMatrix &P = m_Cs[iFrm];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea)
		{
			iTrk = m_mapMeaToTrk[iMea];
			Project(P, m_Xs[iTrk], ZcI, xp, Jx, work);
			LA::AmB(m_xs[iMea], xp, e);
			LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
			LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
		}
	}
	FrameIndex i;
	MeasurementIndex j;
	for(i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		const ProjectiveMatrix &P = m_Cs[iFrm];
		LA::AlignedMatrix11f &Dc = Dcs[i];
		LA::AlignedVector11f &bc = bcs[i];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			Project(P, m_Xs[iTrk], ZcI, xp, Jc, Jx, work);
			LA::AmB(m_xs[iMea], xp, e);
			AddATAToUpper(Jc, Dc, work[0]);
			::AddATBTo(Jc, e, bc, work);
			LA::AddATAToUpper(Jx, Dxs[iTrk], work[0]);
			LA::AddATBTo(Jx, e, bxs[iTrk], work[0]);
			ATB(Jx, Jc, Wxcs[j], work);
		}
		FinishAdditionATAToUpper(Dc);
		LA::SetLowerFromUpper(Dc);
		LA::SetReserve<BA_STAGE_B>(bc);
		const LA::AlignedVector11f &sc = scs[i];
		LA::ssTA(sc, Dc, work[0]);
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
		const LA::AlignedVector11f &sc = scs[i];
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			LA::s1s2TA(sxs[iTrk], sc, Wxcs[j], work[0]);
		}
	}

#ifdef _DEBUG_WITH_EIGEN
	g_JTJ.resize(nVars, nVars);
	g_JTJ.setZero();
	g_JTE.resize(nVars);
	g_JTE.setZero();
	for(iFrm = nFrmsFix, i = 0, j = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		SetMcc(iC + i * 11, iC + i * 11, Dcs[i], g_JTJ);
		SetVc(iC + i * 11, bcs[i], g_JTE);
		const MeasurementIndex iMea1 = m_mapFrmToMea[iFrm], iMea2 = m_mapFrmToMea[iFrm + 1];
		for(iMea = iMea1; iMea < iMea2; ++iMea, ++j)
		{
			iTrk = m_mapMeaToTrk[iMea];
			SetMcx(iC + i * 11, iX + iTrk * 3, Wxcs[j], g_JTJ);
		}
	}
	for(iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		SetMxx(iX + iTrk * 3, iX + iTrk * 3, Dxs[iTrk], g_JTJ);
		SetVx(iX + iTrk * 3, bxs[iTrk], g_JTE);
	}
#endif
}

void SequenceBundleAdjustorDataProjective::UpdateCameras(const AlignedVector<LA::AlignedVector11f> &scs, const AlignedVector<LA::AlignedVector11f> &xcs, 
														 const AlignedVector<ProjectiveMatrix> &PsOld)
{
	const FrameIndex nFrms = FrameIndex(m_Cs.Size()), nFrmsFix = nFrms - FrameIndex(scs.Size());
	for(FrameIndex iFrm = nFrmsFix, i = 0; iFrm < nFrms; ++iFrm, ++i)
	{
		const LA::AlignedVector11f &sc = scs[i], &xc = xcs[i];
		const ProjectiveMatrix &Pold = PsOld[iFrm];
		ProjectiveMatrix &Pnew = m_Cs[iFrm];
		Pnew.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(sc.v0123(), xc.v0123()), Pold.M_00_01_02_03());
		Pnew.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(sc.v4567(), xc.v4567()), Pold.M_10_11_12_13());
		Pnew.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(sc.v8910x(), xc.v8910x()), Pold.M_20_21_22_23());
#if _DEBUG
		assert(Pnew.M23() == Pold.M23());
#endif
	}
}

void SequenceBundleAdjustorDataProjective::UpdatePoints(const AlignedVector<LA::Vector3f> &sxs, const AlignedVector<LA::Vector3f> &xxs, const AlignedVector<Point3D> &XsOld)
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

void SequenceBundleAdjustorDataProjective::UpdateGlobal(const LA::Vector2f &sg, const LA::Vector2f &xg, const Camera::IntrinsicParameter &Gold)
{
}

#if _DEBUG
void SequenceBundleAdjustorDataProjective::DebugSchurComplement(const float damping, const std::vector<std::pair<FrameIndex, FrameIndex> > &iPairs, 
																const AlignedVector<LA::AlignedMatrix11f> &Accs, const LA::AlignedMatrix2f &Agg, 
																const AlignedVector<LA::AlignedMatrix2x11f> &Agcs, const AlignedVector<LA::AlignedVector11f> &bcs, 
																const LA::Vector2f &bg)
{
#ifdef _DEBUG_WITH_EIGEN
	const int nC = int(bcs.Size()) * 11, nX = int(m_Xs.Size()) * 3, nVars = nC + nX;
	const int iC = 0, iX = nC;
	
	int i;
	g_A.resize(nVars, nVars);
	g_A.setZero();
	g_b.resize(nVars);
	g_b.setZero();
	const int nPairs = int(iPairs.size());
	for(i = 0; i < nPairs; ++i)
		SetMcc(iC + iPairs[i].first * 11, iC + iPairs[i].second * 11, Accs[i], g_A);
	const int nFrmsAdj = int(bcs.Size());
	for(i = 0; i < nFrmsAdj; ++i)
		SetVc(iC + i * 11, bcs[i], g_b);

	const float lambda = damping + 1;
	EigenMatrix Dc = g_JTJ.block(iC, iC, nC, nC), Dx = g_JTJ.block(iX, iX, nX, nX);
	for(i = 0; i < nC; ++i)
		Dc(i, i) *= lambda;
	for(i = 0; i < nX; ++i)
		Dx(i, i) *= lambda;
	const EigenMatrix DxI = Dx.inverse(), Wxc = g_JTJ.block(iX, iC, nX, nC);
	const EigenMatrix Ycx = Wxc.transpose() * DxI;
	const EigenVector JTEc = g_JTE.block(iC, 0, nC, 1), JTEx = g_JTE.block(iX, 0, nX, 1);

	EigenMatrix A1;
	EigenVector b1;
	A1.resize(nVars, nVars);
	A1.setZero();
	b1.resize(nVars);
	b1.setZero();
	A1.block(iC, iC, nC, nC) = Dc - Ycx * Wxc;
	b1.block(iC, 0, nC, 1) = JTEc - Ycx * JTEx;
	const EigenMatrix &A2 = g_A;
	const EigenVector &b2 = g_b;
	CheckMcx(iC, iX, A1, A2);
	CheckVcx(iC, iX, b1, b2);
#endif
}

void SequenceBundleAdjustorDataProjective::DebugSolution(const float damping, const AlignedVector<LA::AlignedVector11f> &xcs, const AlignedVector<LA::Vector3f> &xxs, 
														 const LA::Vector2f &xg)
{
#ifdef _DEBUG_WITH_EIGEN
	const int nC = int(xcs.Size()) * 11, nX = int(xxs.Size()) * 3, nVars = nC + nX;
	const int iC = 0, iX = nC;
	g_x.resize(nVars);
	g_x.setZero();
	
	int i;
	const int nFrmsAdj = int(xcs.Size());
	for(i = 0; i < nFrmsAdj; ++i)
		SetVc(iC + i * 11, xcs[i], g_x);
	const int nTrks = int(xxs.Size());
	for(i = 0; i < nTrks; ++i)
		SetVx(iX + i * 3, xxs[i], g_x);

	EigenMatrix A = g_JTJ;
	EigenVector b = g_JTE;
	const float lambda = damping + 1;
	for(i = 0; i < nVars; ++i)
		A(i, i) *= lambda;
	const EigenVector x1 = A.ldlt().solve(b), &x2 = g_x;
	CheckVcx(iC, iX, x1, x2);
	//const EigenVector Ax1 = A * x1, Ax2 = A * x2;
	//CheckVcx(iC, iX, Ax1, b);
	//CheckVcx(iC, iX, Ax2, b);
#endif
}

void SequenceBundleAdjustorDataProjective::DebugAx(const AlignedVector<LA::AlignedVector11f> &xcs, const LA::Vector2f &xg, const AlignedVector<LA::AlignedVector11f> &Axcs, 
												   const LA::Vector2f &Axg)
{
#ifdef _DEBUG_WITH_EIGEN_AX
	const int nC = int(xcs.Size()) * 11, nX = int(m_Xs.Size()) * 3;
	const int iC = 0, iX = nC;

	EigenMatrix A;
	A.resize(nC, nC);
	A.setZero();
	A.block(iC, iC, nC, nC) = g_A.block(iC, iC, nC, nC);;

	EigenVector x;
	x.resize(nC);
	x.setZero();
	const int nFrmsAdj = int(xcs.Size());
	for(int i = 0; i < nFrmsAdj; ++i)
		SetVc(iC + i * 11, xcs[i], x);

	EigenVector Ax;
	Ax.resize(nC);
	Ax.setZero();
	for(int i = 0; i < nFrmsAdj; ++i)
		SetVc(iC + i * 11, Axcs[i], Ax);

	const EigenVector Ax1 = A * x, &Ax2 = Ax;
	CheckVcx(iC, iX, Ax1, Ax2);
#endif
}
#endif