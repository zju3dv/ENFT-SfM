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
#include "SequenceSetBundleAdjustorData3DSimilarity.h"

void SequenceSetBundleAdjustorData3DSimilarity::ValidateGlobal()
{
}

void SequenceSetBundleAdjustorData3DSimilarity::InvalidateGlobal()
{
}

bool SequenceSetBundleAdjustorData3DSimilarity::IsGlobalValid() const
{
	return false;
}

void SequenceSetBundleAdjustorData3DSimilarity::NormalizeData(const float dataNormalizeMedian)
{
	if(dataNormalizeMedian == 0)
	{
		m_scale = 1;
		m_translation.SetZero();
		return;
	}
	Point3D sum;
	sum.SetZero();
	const TrackIndex nTrksCmn = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
		LA::ApB(m_Xs[iTrkCmn], sum, sum);
	Point3D &mean = sum;
	mean.XYZx() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1.0f / nTrksCmn), sum.XYZx());
	m_translation.Set(-mean.X(), -mean.Y(), -mean.Z());

	Point3D dX;
	m_distSqs.resize(nTrksCmn);
	for(TrackIndex iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		LA::AmB(m_Xs[iTrkCmn], mean, dX);
		m_distSqs[iTrkCmn] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(dX.XYZx(), dX.XYZx()));
	}
	const TrackIndex ith = (nTrksCmn >> 1);
	std::nth_element(m_distSqs.begin(), m_distSqs.begin() + ith, m_distSqs.end());
	const float distSqMed = m_distSqs[ith];
	m_scale = (1 / sqrt(distSqMed)) / dataNormalizeMedian;

	ENFT_SSE::__m128 dt;
	const ENFT_SSE::__m128 translation = ENFT_SSE::_mm_setr_ps(m_translation.X(), m_translation.Y(), m_translation.Z(), 0);
	const SequenceIndex nSeqs = SequenceIndex(m_Cs.Size());
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		SimilarityTransformation3D &S = m_Cs[iSeq];
		S.ApplyRotation(translation, dt);
		dt = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1 / S.s()), translation), dt);
		S.tX() = m_scale * (S.tX() + dt.m128_f32[0]);
		S.tY() = m_scale * (S.tY() + dt.m128_f32[1]);
		S.tZ() = m_scale * (S.tZ() + dt.m128_f32[2]);
	}

	const ENFT_SSE::__m128 scale = ENFT_SSE::_mm_setr_ps(m_scale, m_scale, m_scale, 1.0f);
	for(TrackIndex iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		m_Xs[iTrkCmn] += translation;
		m_Xs[iTrkCmn] *= scale;
	}
	const TrackIndex nTrksIdv = TrackIndex(m_xs.Size());
	for(TrackIndex iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
	{
		m_xs[iTrkIdv] += translation;
		m_xs[iTrkIdv] *= scale;
	}
}

void SequenceSetBundleAdjustorData3DSimilarity::DenormalizeData()
{
	if(m_scale == 1 && m_translation.SquaredLength() == 0)
		return;
	m_scale = 1 / m_scale;
	ENFT_SSE::__m128 dt;
	const ENFT_SSE::__m128 translation = ENFT_SSE::_mm_setr_ps(m_translation.v0(), m_translation.v1(), m_translation.v2(), 0);
	const SequenceIndex nSeqs = SequenceIndex(m_Cs.Size());
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		SimilarityTransformation3D &S = m_Cs[iSeq];
		S.ApplyRotation(translation, dt);
		dt =ENFT_SSE::_mm_sub_ps(dt, ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(1 / S.s()), translation));
		S.tX() = m_scale * S.tX() + dt.m128_f32[0];
		S.tY() = m_scale * S.tY() + dt.m128_f32[1];
		S.tZ() = m_scale * S.tZ() + dt.m128_f32[2];
	}

	const ENFT_SSE::__m128 scale = ENFT_SSE::_mm_setr_ps(m_scale, m_scale, m_scale, 1.0f);
	const TrackIndex nTrksCmn = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		m_Xs[iTrkCmn] *= scale;
		m_Xs[iTrkCmn] -= translation;
	}
	const TrackIndex nTrksIdv = TrackIndex(m_xs.Size());
	for(TrackIndex iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
	{
		m_xs[iTrkIdv] *= scale;
		m_xs[iTrkIdv] -= translation;
	}
}

double SequenceSetBundleAdjustorData3DSimilarity::ComputeSSE(std::vector<float> &ptSSEs)
{
	const TrackIndex nTrksCmn = GetPointsNumber();
	ptSSEs.assign(nTrksCmn, 0.0f);

	//static int g_cnt = 0;
	TrackIndex iTrkCmn, iTrkIdv;
	Point3D e;
	const TrackIndex nTrksIdv = GetMeasurementsNumber();
	for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
	{
		iTrkCmn = m_mapIdvTrkToCmnTrk[iTrkIdv];
		m_Cs[m_mapIdvTrkToSeq[iTrkIdv]].ComputeTransformationError(m_Xs[iTrkCmn], m_xs[iTrkIdv], e);
		ptSSEs[iTrkCmn] += e.SquaredLength();
		//if(g_cnt == 1)
		//{
		//	m_Cs[m_mapIdvTrkToSeq[iTrkIdv]].Print();
		//	m_Xs[iTrkCmn].Print();
		//	m_xs[iTrkIdv].Print();
		//	e.Print();
		//}
	}
	//++g_cnt;

	//float errSq, errSqMax = 0;
	//TrackIndex iTrkIdvMax;
	//for(iTrkIdv = 0; iTrkIdv < nTrksIdv; ++iTrkIdv)
	//{
	//	m_Cs[m_mapIdvTrkToSeq[iTrkIdv]].ComputeTransformationError(m_Xs[m_mapIdvTrkToCmnTrk[iTrkIdv]], m_xs[iTrkIdv], e);
	//	if((errSq = e.SquaredLength()) > errSqMax)
	//	{
	//		errSqMax = errSq;
	//		iTrkIdvMax = iTrkIdv;
	//	}
	//}
	//const TrackIndex iTrkCmn = m_mapIdvTrkToCmnTrk[iTrkIdvMax];
	//printf("iTrkCmn = %d, ", iTrkCmn);
	//m_Xs[iTrkCmn].Print();
	//const TrackIndexList &iTrksIdv = m_mapCmnTrkToIdvTrk[iTrkCmn];
	//const SequenceIndex nCrsps = SequenceIndex(iTrksIdv.size());
	//for(SequenceIndex i = 0; i < nCrsps; ++i)
	//{
	//	const TrackIndex iTrkIdv = iTrksIdv[i];
	//	const SequenceIndex iSeq = m_mapIdvTrkToSeq[iTrkIdv];
	//	printf("iSeq = %d, iTrkIdv = %d,  ", iSeq, iTrkIdv);
	//	m_xs[iTrkIdv].Print();
	//}

	double SSE = 0.0;
	for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
		SSE += ptSSEs[iTrkCmn];
	return SSE;
}

static inline void ATA(const LA::AlignedMatrix3f &A, LA::SymmetricMatrix3f &ATA, ENFT_SSE::__m128 &work)
{
	work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), A.M_00_01_02_x()), 
								 ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), A.M_10_11_12_x())), 
								 ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), A.M_20_21_22_x()));
	//ATA.M00() = work.m128_f32[0];
	//ATA.M01() = work.m128_f32[1];
	//ATA.M02() = work.m128_f32[2];
	memcpy(&ATA.M00(), &work, 12);
	ATA.M11() = A.M01() * A.M01() + A.M11() * A.M11() + A.M21() * A.M21();
	ATA.M12() = A.M01() * A.M02() + A.M11() * A.M12() + A.M21() * A.M22();
	ATA.M22() = A.M02() * A.M02() + A.M12() * A.M12() + A.M22() * A.M22();
}

static inline void Aij2(const LA::AlignedMatrix3f &A, LA::Vector3f &Aij2, ENFT_SSE::__m128 &work)
{
	work = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), A.M_00_01_02_x()), 
								 ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), A.M_10_11_12_x())), 
								 ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), A.M_20_21_22_x()));
	memcpy(Aij2, &work, 12);
}

static inline void AddATAToUpper00(const LA::AlignedMatrix3x4f &A0, LA::AlignedMatrix7f &to)
{
#if _DEBUG
	assert(A0.M01() == 0.0f);
	assert(A0.M12() == 0.0f);
	assert(A0.M23() == 0.0f);
#endif
	to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A0.M00()), A0.M_00_01_02_03()), 
											   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A0.M10()), A0.M_10_11_12_13())), 
									ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A0.M20()), A0.M_20_21_22_23()), to.M_00_01_02_03()));
	to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A0.M11()), A0.M_10_11_12_13()), 
											   ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A0.M21()), A0.M_20_21_22_23())), to.M_10_11_12_13());
	to.M22() = A0.M02() * A0.M02() + A0.M22() * A0.M22() + to.M22();
	to.M23() = A0.M02() * A0.M03() + to.M23();
	to.M33() = A0.M03() * A0.M03() + A0.M13() * A0.M13() + to.M33();
}

static inline void AddATAToUpper01(const LA::AlignedMatrix3x4f &A0, const ENFT_SSE::__m128 &A1, LA::AlignedMatrix7f &to, ENFT_SSE::__m128 &work)
{
	work = ENFT_SSE::_mm_mul_ps(A0.M_00_01_02_03(), A1);
	to.M04() = work.m128_f32[0] + to.M04();
	to.M14() = work.m128_f32[1] + to.M14();
	to.M24() = work.m128_f32[2] + to.M24();
	to.M34() = work.m128_f32[3] + to.M34();
	work = ENFT_SSE::_mm_mul_ps(A0.M_10_11_12_13(), A1);
	to.M05() = work.m128_f32[0] + to.M05();
	to.M15() = work.m128_f32[1] + to.M15();
	to.M25() = work.m128_f32[2] + to.M25();
	to.M35() = work.m128_f32[3] + to.M35();
	work = ENFT_SSE::_mm_mul_ps(A0.M_20_21_22_23(), A1);
	to.M06() = work.m128_f32[0] + to.M06();
	to.M16() = work.m128_f32[1] + to.M16();
	to.M26() = work.m128_f32[2] + to.M26();
	to.M36() = work.m128_f32[3] + to.M36();
}

static inline void AddATAToUpper11(const float &A1, const uint &N, LA::AlignedMatrix7f &to)
{
	to.reserve0() = A1 * A1 * N;
	to.M44() = to.reserve0() + to.M44();
	to.M55() = to.reserve0() + to.M55();
	to.M66() = to.reserve0() + to.M66();
}

static inline void AddATBTo0(const LA::AlignedMatrix3x4f &A0, const ENFT_SSE::__m128 &B, LA::AlignedVector7f &to)
{
	to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A0.M_00_01_02_03(), ENFT_SSE::_mm_set1_ps(B.m128_f32[0])), ENFT_SSE::_mm_mul_ps(A0.M_10_11_12_13(), ENFT_SSE::_mm_set1_ps(B.m128_f32[1]))), 
							ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A0.M_20_21_22_23(), ENFT_SSE::_mm_set1_ps(B.m128_f32[2])), to.v0123()));
}

static inline void AddATBTo1(const ENFT_SSE::__m128 &A1, const ENFT_SSE::__m128 &B, LA::AlignedVector7f &to)
{
	to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A1, B), to.v456x());
}

static inline void AddAij2To0(const LA::AlignedMatrix3x4f &A0, LA::AlignedVector7f &to)
{
	to.v0123() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A0.M_00_01_02_03(), A0.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(A0.M_10_11_12_13(), A0.M_10_11_12_13())), 
							ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A0.M_20_21_22_23(), A0.M_20_21_22_23()), to.v0123()));
}

static inline void AddAij2To1(const float &A1, const uint &N, LA::AlignedVector7f &to)
{
	to.reserve() = A1 * A1 * N;
	to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_set1_ps(to.reserve()), to.v456x());
}

//static inline void FinishAdditionAij2To(LA::AlignedVector7f &to)
//{
//	to.v1() = to.v2() = to.v3() = (to.v1() + to.v2() + to.v3()) / 3;
//}

static inline void ATB(const LA::AlignedMatrix3f &A, const LA::AlignedMatrix3x4f &B0, const ENFT_SSE::__m128 &B1, LA::AlignedMatrix3x7f &ATB, ENFT_SSE::__m128 &work)
{
	ATB.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B0.M_00_01_02_03()), 
												ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B0.M_10_11_12_13())), 
												ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), B0.M_20_21_22_23()));
	ATB.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B0.M_00_01_02_03()), 
												ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B0.M_10_11_12_13())), 
												ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M21()), B0.M_20_21_22_23()));
	ATB.M_20_21_22_23() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), B0.M_00_01_02_03()), 
												ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B0.M_10_11_12_13())), 
												ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M22()), B0.M_20_21_22_23()));
	work = ENFT_SSE::_mm_mul_ps(A.M_00_01_02_x(), B1);
	ATB.M04() = work.m128_f32[0];
	ATB.M14() = work.m128_f32[1];
	ATB.M24() = work.m128_f32[2];
	work = ENFT_SSE::_mm_mul_ps(A.M_10_11_12_x(), B1);
	ATB.M05() = work.m128_f32[0];
	ATB.M15() = work.m128_f32[1];
	ATB.M25() = work.m128_f32[2];
	work = ENFT_SSE::_mm_mul_ps(A.M_20_21_22_x(), B1);
	ATB.M06() = work.m128_f32[0];
	ATB.M16() = work.m128_f32[1];
	ATB.M26() = work.m128_f32[2];
}

void SequenceSetBundleAdjustorData3DSimilarity::ConstructNormalEquation(AlignedVector<LA::AlignedMatrix7f> &Dcs, AlignedVector<LA::SymmetricMatrix3f> &Dxs, 
																		LA::AlignedMatrix2f &Dg, AlignedVector<LA::AlignedMatrix3x7f> &Wxcs, 
																		AlignedVector<LA::AlignedMatrix2x7f> &Wgcs, AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, 
																		AlignedVector<LA::AlignedVector7f> &bcs, AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg, 
																		AlignedVector<LA::AlignedVector7f> &scs, AlignedVector<LA::Vector3f> &sxs, LA::Vector2f &sg)
{
	Dcs.SetZero();
	Dxs.SetZero();
	bcs.SetZero();
	bxs.SetZero();
	scs.SetZero();
	sxs.SetZero();

#if _DEBUG
	assert(Wgcs.Empty() && Wgxs.Empty());
#endif

	SequenceIndex iSeq;
	TrackIndex iTrkCmn, iTrkIdv;
	LA::AlignedMatrix3x4f Jc0;
	LA::AlignedMatrix3f Jx;
	LA::SymmetricMatrix3f Dx;
	LA::Vector3f sx;
	ENFT_SSE::__m128 Jc1, RX, TX, SX, sRX, e, eSum, work;
	const SequenceIndex nSeqs = SequenceIndex(m_Cs.Size()), nSeqsFix = nSeqs - SequenceIndex(scs.Size());
	for(iSeq = 0; iSeq < nSeqsFix; ++iSeq)
	{
		const SimilarityTransformation3D &S = m_Cs[iSeq];
		Jx.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_00_01_02_x());
		Jx.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_10_11_12_x());
		Jx.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_20_21_22_x());
		ATA(Jx, Dx, work);
		Aij2(Jx, sx, work);
		const TrackIndex iTrkIdv1 = m_mapSeqToIdvTrk[iSeq], iTrkIdv2 = m_mapSeqToIdvTrk[iSeq + 1];
		for(iTrkIdv = iTrkIdv1; iTrkIdv < iTrkIdv2; ++iTrkIdv)
		{
			iTrkCmn = m_mapIdvTrkToCmnTrk[iTrkIdv];
			S.Apply(m_Xs[iTrkCmn].XYZx(), SX);
			e = ENFT_SSE::_mm_sub_ps(m_xs[iTrkIdv].XYZx(), SX);
			Dxs[iTrkCmn] += Dx;
			LA::AddATBTo(Jx, e, bxs[iTrkCmn], work);
			sxs[iTrkCmn] += sx;
		}
	}
	SequenceIndex i;
	MeasurementIndex j;
	for(i = 0, j = 0; iSeq < nSeqs; ++iSeq, ++i)
	{
		const SimilarityTransformation3D &S = m_Cs[iSeq];
		LA::AlignedMatrix7f &Dc = Dcs[i];
		LA::AlignedVector7f &bc = bcs[i], &sc = scs[i];
		Jc1 = ENFT_SSE::_mm_set1_ps(S.s());
		Jx.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_00_01_02_x());
		Jx.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_10_11_12_x());
		Jx.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_20_21_22_x());
		ATA(Jx, Dx, work);
		Aij2(Jx, sx, work);
		eSum = ENFT_SSE::_mm_setzero_ps();
		const TrackIndex iTrkIdv1 = m_mapSeqToIdvTrk[iSeq], iTrkIdv2 = m_mapSeqToIdvTrk[iSeq + 1];
		for(iTrkIdv = iTrkIdv1; iTrkIdv < iTrkIdv2; ++iTrkIdv, ++j)
		{
			iTrkCmn = m_mapIdvTrkToCmnTrk[iTrkIdv];
			S.Apply(m_Xs[iTrkCmn].XYZx(), RX, TX, SX);
			sRX = ENFT_SSE::_mm_mul_ps(S.sss1(), RX);
			Jc0.M00() = TX.m128_f32[0];		Jc0.M01() = 0.0f;				Jc0.M02() = sRX.m128_f32[2];	Jc0.M03() = -sRX.m128_f32[1];
			Jc0.M10() = TX.m128_f32[1];		Jc0.M11() = -sRX.m128_f32[2];	Jc0.M12() = 0.0f;				Jc0.M13() = sRX.m128_f32[0];
			Jc0.M20() = TX.m128_f32[2];		Jc0.M21() = sRX.m128_f32[1];	Jc0.M22() = -sRX.m128_f32[0];	Jc0.M23() = 0.0f;
			e = ENFT_SSE::_mm_sub_ps(m_xs[iTrkIdv].XYZx(), SX);
			AddATAToUpper00(Jc0, Dc);
			AddATAToUpper01(Jc0, Jc1, Dc, work);
			AddATBTo0(Jc0, e, bc);
			eSum = ENFT_SSE::_mm_add_ps(e, eSum);
			AddAij2To0(Jc0, sc);
			Dxs[iTrkCmn] += Dx;
			LA::AddATBTo(Jx, e, bxs[iTrkCmn], work);
			sxs[iTrkCmn] += sx;
			ATB(Jx, Jc0, Jc1, Wxcs[j], work);
		}
		AddATAToUpper11(Jc1.m128_f32[0], iTrkIdv2 - iTrkIdv1, Dc);
		AddATBTo1(Jc1, eSum, bc);
		AddAij2To1(Jc1.m128_f32[0], iTrkIdv2 - iTrkIdv1, sc);
		//FinishAdditionAij2To(sc);
		LA::SetLowerFromUpper(Dc);
		LA::SetReserve<BA_STAGE_B>(bc);
		LA::MakeReciprocal(sc);
		LA::MakeSquareRoot(sc);
		//sc.v0123() = sc.v456x() = ENFT_SSE::_mm_set1_ps(1.0f);
		LA::SetReserve<BA_STAGE_S>(sc);
		LA::ssTA(sc, Dc, work);
		LA::sA(sc, bc);
	}

	const TrackIndex nTrksCmn = TrackIndex(m_Xs.Size());
	for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		LA::SymmetricMatrix3f &Dx = Dxs[iTrkCmn];
		LA::SetLowerFromUpper(Dx);
		LA::Vector3f &bx = bxs[iTrkCmn];
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x3f, ENFT_SSE::__m128>(bx);
		LA::Vector3f &sx = sxs[iTrkCmn];
		LA::MakeReciprocal(sx);
		LA::MakeSquareRoot(sx);
		//sx.v0() = sx.v1() = sx.v2() = 1.0f;
		LA::SetReserve<BA_STAGE_S>(sx);
		LA::ssTA(sx, Dx);
		LA::sA(sx, bx);
		LA::SetReserve<BA_STAGE_B>(bx);
	}

	for(iSeq = nSeqsFix, i = 0, j = 0; iSeq < nSeqs; ++iSeq, ++i)
	{
		const LA::AlignedVector7f &sc = scs[i];
		const TrackIndex iTrkIdv1 = m_mapSeqToIdvTrk[iSeq], iTrkIdv2 = m_mapSeqToIdvTrk[iSeq + 1];
		for(iTrkIdv = iTrkIdv1; iTrkIdv < iTrkIdv2; ++iTrkIdv, ++j)
		{
			iTrkCmn = m_mapIdvTrkToCmnTrk[iTrkIdv];
			LA::s1s2TA(sxs[iTrkCmn], sc, Wxcs[j], work);
		}
	}
}

void SequenceSetBundleAdjustorData3DSimilarity::ConstructNormalEquation(const AlignedVector<LA::AlignedVector7f> &scs, const AlignedVector<LA::Vector3f> &sxs, 
																		const LA::Vector2f &sg, AlignedVector<LA::AlignedMatrix7f> &Dcs, 
																		AlignedVector<LA::SymmetricMatrix3f> &Dxs, LA::AlignedMatrix2f &Dg, 
																		AlignedVector<LA::AlignedMatrix3x7f> &Wxcs, AlignedVector<LA::AlignedMatrix2x7f> &Wgcs, 
																		AlignedVector<LA::AlignedMatrix2x3f> &Wgxs, AlignedVector<LA::AlignedVector7f> &bcs, 
																		AlignedVector<LA::Vector3f> &bxs, LA::Vector2f &bg)
{
	Dcs.SetZero();
	Dxs.SetZero();
	bcs.SetZero();
	bxs.SetZero();

#if _DEBUG
	assert(Wgcs.Empty() && Wgxs.Empty());
#endif

	SequenceIndex iSeq;
	TrackIndex iTrkCmn, iTrkIdv;
	LA::AlignedMatrix3x4f Jc0;
	LA::AlignedMatrix3f Jx;
	LA::SymmetricMatrix3f Dx;
	ENFT_SSE::__m128 Jc1, RX, TX, SX, sRX, e, eSum, work;
	const SequenceIndex nSeqs = SequenceIndex(m_Cs.Size()), nSeqsFix = nSeqs - SequenceIndex(scs.Size());
	for(iSeq = 0; iSeq < nSeqsFix; ++iSeq)
	{
		const SimilarityTransformation3D &S = m_Cs[iSeq];
		Jx.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_00_01_02_x());
		Jx.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_10_11_12_x());
		Jx.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_20_21_22_x());
		ATA(Jx, Dx, work);
		const TrackIndex iTrkIdv1 = m_mapSeqToIdvTrk[iSeq], iTrkIdv2 = m_mapSeqToIdvTrk[iSeq + 1];
		for(iTrkIdv = iTrkIdv1; iTrkIdv < iTrkIdv2; ++iTrkIdv)
		{
			iTrkCmn = m_mapIdvTrkToCmnTrk[iTrkIdv];
			S.Apply(m_Xs[iTrkCmn].XYZx(), SX);
			e = ENFT_SSE::_mm_sub_ps(m_xs[iTrkIdv].XYZx(), SX);
			Dxs[iTrkCmn] += Dx;
			LA::AddATBTo(Jx, e, bxs[iTrkCmn], work);
		}
	}
	SequenceIndex i;
	MeasurementIndex j;
	for(i = 0, j = 0; iSeq < nSeqs; ++iSeq, ++i)
	{
		const SimilarityTransformation3D &S = m_Cs[iSeq];
		LA::AlignedMatrix7f &Dc = Dcs[i];
		LA::AlignedVector7f &bc = bcs[i];
		Jc1 = ENFT_SSE::_mm_set1_ps(S.s());
		Jx.M_00_01_02_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_00_01_02_x());
		Jx.M_10_11_12_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_10_11_12_x());
		Jx.M_20_21_22_x() = ENFT_SSE::_mm_mul_ps(S.sss1(), S.r_20_21_22_x());
		ATA(Jx, Dx, work);
		eSum = ENFT_SSE::_mm_setzero_ps();
		const TrackIndex iTrkIdv1 = m_mapSeqToIdvTrk[iSeq], iTrkIdv2 = m_mapSeqToIdvTrk[iSeq + 1];
		for(iTrkIdv = iTrkIdv1; iTrkIdv < iTrkIdv2; ++iTrkIdv, ++j)
		{
			iTrkCmn = m_mapIdvTrkToCmnTrk[iTrkIdv];
			S.Apply(m_Xs[iTrkCmn].XYZx(), RX, TX, SX);
			sRX = ENFT_SSE::_mm_mul_ps(S.sss1(), RX);
			Jc0.M00() = TX.m128_f32[0];		Jc0.M01() = 0.0f;				Jc0.M02() = sRX.m128_f32[2];	Jc0.M03() = -sRX.m128_f32[1];
			Jc0.M10() = TX.m128_f32[1];		Jc0.M11() = -sRX.m128_f32[2];	Jc0.M12() = 0.0f;				Jc0.M13() = sRX.m128_f32[0];
			Jc0.M20() = TX.m128_f32[2];		Jc0.M21() = sRX.m128_f32[1];	Jc0.M22() = -sRX.m128_f32[0];	Jc0.M23() = 0.0f;
			e = ENFT_SSE::_mm_sub_ps(m_xs[iTrkIdv].XYZx(), SX);
			AddATAToUpper00(Jc0, Dc);
			AddATAToUpper01(Jc0, Jc1, Dc, work);
			AddATBTo0(Jc0, e, bc);
			eSum = ENFT_SSE::_mm_add_ps(e, eSum);
			Dxs[iTrkCmn] += Dx;
			LA::AddATBTo(Jx, e, bxs[iTrkCmn], work);
			ATB(Jx, Jc0, Jc1, Wxcs[j], work);
		}
		AddATAToUpper11(Jc1.m128_f32[0], iTrkIdv2 - iTrkIdv1, Dc);
		AddATBTo1(Jc1, eSum, bc);
		//FinishAdditionAij2To(sc);
		LA::SetLowerFromUpper(Dc);
		LA::SetReserve<BA_STAGE_B>(bc);
		const LA::AlignedVector7f &sc = scs[i];
		LA::ssTA(sc, Dc, work);
		LA::sA(sc, bc);
	}

	const TrackIndex nTrksCmn = TrackIndex(m_Xs.Size());
	for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		LA::SymmetricMatrix3f &Dx = Dxs[iTrkCmn];
		LA::SetLowerFromUpper(Dx);
		LA::Vector3f &bx = bxs[iTrkCmn];
		LA::FinishAdditionATBTo<LA::AlignedMatrix2x3f, ENFT_SSE::__m128>(bx);
		const LA::Vector3f &sx = sxs[iTrkCmn];
		LA::ssTA(sx, Dx);
		LA::sA(sx, bx);
		LA::SetReserve<BA_STAGE_B>(bx);
	}

	for(iSeq = nSeqsFix, i = 0, j = 0; iSeq < nSeqs; ++iSeq, ++i)
	{
		const LA::AlignedVector7f &sc = scs[i];
		const TrackIndex iTrkIdv1 = m_mapSeqToIdvTrk[iSeq], iTrkIdv2 = m_mapSeqToIdvTrk[iSeq + 1];
		for(iTrkIdv = iTrkIdv1; iTrkIdv < iTrkIdv2; ++iTrkIdv, ++j)
		{
			iTrkCmn = m_mapIdvTrkToCmnTrk[iTrkIdv];
			LA::s1s2TA(sxs[iTrkCmn], sc, Wxcs[j], work);
		}
	}
}

void SequenceSetBundleAdjustorData3DSimilarity::UpdateCameras(const AlignedVector<LA::AlignedVector7f> &scs, const AlignedVector<LA::AlignedVector7f> &xcs, 
															  const AlignedVector<SimilarityTransformation3D> &SsOld)
{
	RotationTransformation3D dR;
	ENFT_SSE::__m128 workm;
	float workf[24];
	const SequenceIndex nSeqs = SequenceIndex(m_Cs.Size()), nSeqsFix = nSeqs - FrameIndex(scs.Size());
	for(SequenceIndex iSeq = nSeqsFix, i = 0; iSeq < nSeqs; ++iSeq, ++i)
	{
		const LA::AlignedVector7f &sc = scs[i], &xc = xcs[i];
		const SimilarityTransformation3D &Sold = SsOld[iSeq];
		SimilarityTransformation3D &Snew = m_Cs[iSeq];
		workm = ENFT_SSE::_mm_mul_ps(sc.v0123(), xc.v0123());
		Snew.SetScale(Sold.s() + workm.m128_f32[0]);
		dR.FromRodrigues(workm.m128_f32[1], workm.m128_f32[2], workm.m128_f32[3], workf);
		Sold.LeftMultiplyRotation(dR, Snew, workm);
		workm = ENFT_SSE::_mm_mul_ps(sc.v456x(), xc.v456x());
		Snew.tX() = workm.m128_f32[0] + Sold.tX();
		Snew.tY() = workm.m128_f32[1] + Sold.tY();
		Snew.tZ() = workm.m128_f32[2] + Sold.tZ();
	}
}

void SequenceSetBundleAdjustorData3DSimilarity::UpdatePoints(const AlignedVector<LA::Vector3f> &sxs, const AlignedVector<LA::Vector3f> &xxs, const AlignedVector<Point3D> &XsOld)
{
	const TrackIndex nTrksCmn = TrackIndex(m_Xs.Size());
	for(TrackIndex iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		const LA::Vector3f &sx = sxs[iTrkCmn], &xx = xxs[iTrkCmn];
		const Point3D &Xold = XsOld[iTrkCmn];
		Point3D &Xnew = m_Xs[iTrkCmn];
		Xnew.X() = sx.v0() * xx.v0() + Xold.X();
		Xnew.Y() = sx.v1() * xx.v1() + Xold.Y();
		Xnew.Z() = sx.v2() * xx.v2() + Xold.Z();
	}
}

void SequenceSetBundleAdjustorData3DSimilarity::UpdateGlobal(const LA::Vector2f &sg, const LA::Vector2f &xg, const Camera::IntrinsicParameter &Gold)
{
#if _DEBUG
	assert(0);
#endif
}