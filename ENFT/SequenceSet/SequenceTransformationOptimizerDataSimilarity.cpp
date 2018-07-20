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
#include "SequenceTransformationOptimizerDataSimilarity.h"

void SequenceTransformationOptimizerDataSimilarity::NormalizeData(const float dataNormalizeMedian)
{
	if(dataNormalizeMedian == 0)
	{
		m_scale = 1;
		m_translation.SetZero();
		return;
	}
	Point3D sum;
	sum.SetZero();
	const uint nSeqPairs = uint(m_mapPairs.size());
	for(uint i = 0; i < nSeqPairs; ++i)
	{
		const AlignedVector<PointPair> &Xs = m_mapPairs[i].Xs();
		const TrackIndex nPtPairs = TrackIndex(Xs.Size());
		for(TrackIndex j = 0; j < nPtPairs; ++j)
		{
			LA::ApB(Xs[j].X1(), sum, sum);
			LA::ApB(Xs[j].X2(), sum, sum);
		}
	}
	Point3D &mean = sum;
	mean.XYZx() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1.0f / m_nPts), sum.XYZx());
	m_translation.Set(-mean.X(), -mean.Y(), -mean.Z());

	Point3D dX;
	std::vector<float> distSqs;
	distSqs.reserve(m_nPts);
	for(uint i = 0; i < nSeqPairs; ++i)
	{
		const AlignedVector<PointPair> &Xs = m_mapPairs[i].Xs();
		const TrackIndex nPtPairs = TrackIndex(Xs.Size());
		for(TrackIndex j = 0; j < nPtPairs; ++j)
		{
			LA::AmB(Xs[j].X1(), mean, dX);		distSqs.push_back(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(dX.XYZx(), dX.XYZx())));
			LA::AmB(Xs[j].X2(), mean, dX);		distSqs.push_back(ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(dX.XYZx(), dX.XYZx())));
		}
	}
	const TrackIndex ith = (m_nPts >> 1);
	std::nth_element(distSqs.begin(), distSqs.begin() + ith, distSqs.end());
	const float distSqMed = distSqs[ith];
	m_scale = (1 / sqrt(distSqMed)) / dataNormalizeMedian;

	ENFT_SSE::__m128 dt;
	const ENFT_SSE::__m128 translation = _mm_setr_ps(m_translation.v0(), m_translation.v1(), m_translation.v2(), 0);
	const SequenceIndex nSeqs = SequenceIndex(m_Ts.Size());
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		SimilarityTransformation3D &S = m_Ts[iSeq];
		S.ApplyRotation(translation, dt);
		dt = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / S.s()), translation), dt);
		S.tX() = m_scale * (S.tX() + dt.m128_f32[0]);
		S.tY() = m_scale * (S.tY() + dt.m128_f32[1]);
		S.tZ() = m_scale * (S.tZ() + dt.m128_f32[2]);
	}

	const ENFT_SSE::__m128 scale = _mm_setr_ps(m_scale, m_scale, m_scale, 1.0f);
	for(uint i = 0; i < nSeqPairs; ++i)
	{
		AlignedVector<PointPair> &Xs = m_mapPairs[i].Xs();
		const TrackIndex nPtPairs = TrackIndex(Xs.Size());
		for(TrackIndex j = 0; j < nPtPairs; ++j)
		{
			Xs[j].X1() += translation;		Xs[j].X1() *= scale;
			Xs[j].X2() += translation;		Xs[j].X2() *= scale;
		}
	}
}

void SequenceTransformationOptimizerDataSimilarity::DenormalizeData()
{
	if(m_scale == 1 && m_translation.SquaredLength() == 0)
		return;
	m_scale = 1 / m_scale;
	ENFT_SSE::__m128 dt;
	const ENFT_SSE::__m128 translation = _mm_setr_ps(m_translation.v0(), m_translation.v1(), m_translation.v2(), 0);
	const SequenceIndex nSeqs = SequenceIndex(m_Ts.Size());
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		SimilarityTransformation3D &S = m_Ts[iSeq];
		S.ApplyRotation(translation, dt);
		dt =_mm_sub_ps(dt, ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1 / S.s()), translation));
		S.tX() = m_scale * S.tX() + dt.m128_f32[0];
		S.tY() = m_scale * S.tY() + dt.m128_f32[1];
		S.tZ() = m_scale * S.tZ() + dt.m128_f32[2];
	}
}

//void SequenceTransformationOptimizerDataSimilarity::InvertTransformations()
//{
//	ENFT_SSE::__m128 work[2];
//	const SequenceIndex nSeqs = SequenceIndex(m_Ts.Size());
//	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
//		m_Ts[iSeq].Invert(m_Tinvs[iSeq], work);
//}

// Similarity transformation: s * (dR * R * X + t)
// Similarity transformation is parameterized as: [s, w^T, t^T]^T, where w are Rodrigues parameters of dR
// J = [Js, Jw, Jt]
// Js = RX + t
//      [    0    sRX_Z -sRX_Y ]
// Jw = [ -sRX_Z    0    sRX_X ]
//      [  sRX_Y -sRX_X    0   ]
// Jt = diag(s)
class JacobianMatrix3x7f
{
public:

	inline const ENFT_SSE::__m128& TX() const { return m_TX; }					inline ENFT_SSE::__m128& TX() { return m_TX; }
	inline const float& TX_X() const { return m_TX.m128_f32[0]; }		inline float& TX_X() { return m_TX.m128_f32[0]; }
	inline const float& TX_Y() const { return m_TX.m128_f32[1]; }		inline float& TX_Y() { return m_TX.m128_f32[1]; }
	inline const float& TX_Z() const { return m_TX.m128_f32[2]; }		inline float& TX_Z() { return m_TX.m128_f32[2]; }
	inline const ENFT_SSE::__m128& sRX() const { return m_sRX_s; }				inline ENFT_SSE::__m128& sRX() { return m_sRX_s; }
	inline const float& sRX_X() const { return m_sRX_s.m128_f32[0]; }	inline float& sRX_X() { return m_sRX_s.m128_f32[0]; }
	inline const float& sRX_Y() const { return m_sRX_s.m128_f32[1]; }	inline float& sRX_Y() { return m_sRX_s.m128_f32[1]; }
	inline const float& sRX_Z() const { return m_sRX_s.m128_f32[2]; }	inline float& sRX_Z() { return m_sRX_s.m128_f32[2]; }
	inline const float&	s() const { return m_sRX_s.m128_f32[3]; }		inline float& s() { return m_sRX_s.m128_f32[3]; }
	inline void SetZero() { m_TX = m_sRX_s = ENFT_SSE::_mm_setzero_ps(); }

protected:

	ENFT_SSE::__m128 m_TX, m_sRX_s;

};

static inline void ComputeTransformationError(const SimilarityTransformation3D &S1, const Point3D &X1, const SimilarityTransformation3D &S2, const Point3D &X2, 
											  Point3D &S1X1, Point3D &S2X2, Point3D &e)
{
	S1.Apply(X1, S1X1);
	S2.Apply(X2, S2X2);
	LA::AmB(S1X1, S2X2, e);
}

double SequenceTransformationOptimizerDataSimilarity::ComputeTransformationSSE() const
{
	Point3D S1X1, S2X2, e, sum;
	sum.SetZero();
	const uint nSeqPairs = uint(m_mapPairs.size());
	for(uint i = 0; i < nSeqPairs; ++i)
	{
		const SequenceIndex iSeq1 = m_mapPairs[i].GetMapIndex1(), iSeq2 = m_mapPairs[i].GetMapIndex2();
		const SimilarityTransformation3D &S1 = m_Ts[iSeq1], &S2 = m_Ts[iSeq2];
		const AlignedVector<PointPair> &Xs = m_mapPairs[i].Xs();
		const TrackIndex nPtPairs = TrackIndex(Xs.Size());
		for(TrackIndex j = 0; j < nPtPairs; ++j)
		{
			ComputeTransformationError(S1, Xs[j].X1(), S2, Xs[j].X2(), S1X1, S2X2, e);
			sum.XYZx() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(e.XYZx(), e.XYZx()), sum.XYZx());
		}
	}

	return ENFT_SSE::SSE::Sum012(sum.XYZx());
}

static inline void ComputeTransformationErrorAndJacobian(const SimilarityTransformation3D &S1, const Point3D &X1, const SimilarityTransformation3D &S2, const Point3D &X2, 
														 Point3D &e, JacobianMatrix3x7f &J1, JacobianMatrix3x7f &J2, ENFT_SSE::__m128 &work/*, const bool fixScales*/)
{
	ENFT_SSE::__m128 &RX = work;
	S1.ApplyRigidTransformation(X1.XYZx(), RX, J1.TX());
	RX.m128_f32[3] = S1.s();
	J1.sRX() = ENFT_SSE::_mm_mul_ps(S1.sss1(), RX);

	S2.ApplyRigidTransformation(X2.XYZx(), RX, J2.TX());
	RX.m128_f32[3] = S2.s();
	J2.sRX() = ENFT_SSE::_mm_mul_ps(S2.sss1(), RX);

	e.XYZx() = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(S1.sss1(), J1.TX()), ENFT_SSE::_mm_mul_ps(S2.sss1(), J2.TX()));
	J1.TX() = ENFT_SSE::_mm_sub_ps(_mm_setzero_ps(), J1.TX());
	J1.sRX() = ENFT_SSE::_mm_sub_ps(_mm_setzero_ps(), J1.sRX());
	//if(fixScales)
	//	J1.TX() = J2.TX() = ENFT_SSE::_mm_setzero_ps();
}

static inline void ComputeTransformationErrorAndJacobian(const SimilarityTransformation3D &S1, const Point3D &X1, const SimilarityTransformation3D &S2, const Point3D &X2, 
														 Point3D &e, JacobianMatrix3x7f &J2, ENFT_SSE::__m128 &work/*, const bool fixScales*/)
{
	ENFT_SSE::__m128 &RX = work;
	S2.ApplyRigidTransformation(X2.XYZx(), RX, J2.TX());
	RX.m128_f32[3] = S2.s();
	J2.sRX() = ENFT_SSE::_mm_mul_ps(S2.sss1(), RX);
	S1.Apply(X1, e);
	e.XYZx() = ENFT_SSE::_mm_sub_ps(e.XYZx(), ENFT_SSE::_mm_mul_ps(S2.sss1(), J2.TX()));
	//if(fixScales)
	//	J2.TX() = ENFT_SSE::_mm_setzero_ps();
}

static inline void AddATAToUpper(const JacobianMatrix3x7f &A, LA::AlignedMatrix7f &to, ENFT_SSE::__m128* const &work2)
{
	to.M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.TX(), A.TX())) + to.M00();
	to.M01() = A.TX_Z() * A.sRX_Y() - A.TX_Y() * A.sRX_Z() + to.M01();
	to.M02() = A.TX_X() * A.sRX_Z() - A.TX_Z() * A.sRX_X() + to.M02();
	to.M03() = A.TX_Y() * A.sRX_X() - A.TX_X() * A.sRX_Y() + to.M03();

	ENFT_SSE::__m128 &s = work2[0];
	s = _mm_set1_ps(A.s());
	to.M_04_05_06_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.TX(), s), to.M_04_05_06_x());

	ENFT_SSE::__m128 &sRX_s_2 = work2[1];
	sRX_s_2 = ENFT_SSE::_mm_mul_ps(A.sRX(), A.sRX());
	to.M11() = sRX_s_2.m128_f32[1] + sRX_s_2.m128_f32[2] + to.M11();
	to.M12() = -A.sRX_X() * A.sRX_Y() + to.M12();
	to.M13() = -A.sRX_X() * A.sRX_Z() + to.M13();
	to.M22() = sRX_s_2.m128_f32[0] + sRX_s_2.m128_f32[2] + to.M22();
	to.M23() = -A.sRX_Y() * A.sRX_Z() + to.M23();
	to.M33() = sRX_s_2.m128_f32[0] + sRX_s_2.m128_f32[1] + to.M33();

	ENFT_SSE::__m128 &sRX_s_s = work2[1];
	sRX_s_s = ENFT_SSE::_mm_mul_ps(A.sRX(), s);
	to.M15() = -sRX_s_s.m128_f32[2] + to.M15();
	to.M16() = sRX_s_s.m128_f32[1] + to.M16();
	to.M24() = sRX_s_s.m128_f32[2] + to.M24();
	to.M26() = -sRX_s_s.m128_f32[0] + to.M26();
	to.M34() = -sRX_s_s.m128_f32[1] + to.M34();
	to.M35() = sRX_s_s.m128_f32[0] + to.M35();

	to.M44() = sRX_s_s.m128_f32[3] + to.M44();
	to.M55() = sRX_s_s.m128_f32[3] + to.M55();
	to.M66() = sRX_s_s.m128_f32[3] + to.M66();
}

static inline void AddATBTo(const JacobianMatrix3x7f &A, const LA::AlignedVector3f &B, LA::AlignedVector7f &to)
{
	to.v0() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.TX(), B.v012x())) + to.v0();
	to.v1() = A.sRX_Y() * B.v2() - A.sRX_Z() * B.v1() + to.v1();
	to.v2() = A.sRX_Z() * B.v0() - A.sRX_X() * B.v2() + to.v2();
	to.v3() = A.sRX_X() * B.v1() - A.sRX_Y() * B.v0() + to.v3();
	to.v456x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(_mm_set1_ps(A.s()), B.v012x()), to.v456x());
}

static inline void AddATBTo(const JacobianMatrix3x7f &A, const JacobianMatrix3x7f &B, LA::AlignedMatrix7f &to, ENFT_SSE::__m128 *work2)
{
	to.M00() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.TX(), B.TX())) + to.M00();
	to.M01() = A.TX_Z() * B.sRX_Y() - A.TX_Y() * B.sRX_Z() + to.M01();
	to.M02() = A.TX_X() * B.sRX_Z() - A.TX_Z() * B.sRX_X() + to.M02();
	to.M03() = A.TX_Y() * B.sRX_X() - A.TX_X() * B.sRX_Y() + to.M03();

	ENFT_SSE::__m128 &sb = work2[0];
	sb = _mm_set1_ps(B.s());
	to.M_04_05_06_x() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(A.TX(), sb), to.M_04_05_06_x());

	ENFT_SSE::__m128 &sRXa_sa_sb = work2[1];
	sRXa_sa_sb = ENFT_SSE::_mm_mul_ps(A.sRX(), sb);
	to.M10() = A.sRX_Y() * B.TX_Z() - A.sRX_Z() * B.TX_Y() + to.M10();
	to.M11() = A.sRX_Y() * B.sRX_Y() + A.sRX_Z() * B.sRX_Z() + to.M11();
	to.M12() = -A.sRX_Y() * B.sRX_X() + to.M12();
	to.M13() = -A.sRX_Z() * B.sRX_X() + to.M13();
	to.M15() = -sRXa_sa_sb.m128_f32[2] + to.M15();
	to.M16() =  sRXa_sa_sb.m128_f32[1] + to.M16();

	to.M20() = A.sRX_Z() * B.TX_X() - A.sRX_X() * B.TX_Z() + to.M20();
	to.M21() = -A.sRX_X() * B.sRX_Y() + to.M21();
	to.M22() = A.sRX_Z() * B.sRX_Z() + A.sRX_X() * B.sRX_X() + to.M22();
	to.M23() = -A.sRX_Z() * B.sRX_Y() + to.M23();
	to.M24() = sRXa_sa_sb.m128_f32[2] + to.M24();
	to.M26() = -sRXa_sa_sb.m128_f32[0] + to.M26();

	to.M30() = A.sRX_X() * B.TX_Y() - A.sRX_Y() * B.TX_X() + to.M30();
	to.M31() = -A.sRX_X() * B.sRX_Z() + to.M31();
	to.M32() = -A.sRX_Y() * B.sRX_Z() + to.M32();
	to.M33() = A.sRX_Y() * B.sRX_Y() + A.sRX_X() * B.sRX_X() + to.M33();
	to.M34() = -sRXa_sa_sb.m128_f32[1] + to.M34();
	to.M35() = sRXa_sa_sb.m128_f32[0] + to.M35();

	ENFT_SSE::__m128 &sa = work2[0], &sa_TXb = work2[1], &sa_sRX_sb = work2[0];
	sa = _mm_set1_ps(A.s());
	sa_TXb = ENFT_SSE::_mm_mul_ps(sa, B.TX());
	sa_sRX_sb = ENFT_SSE::_mm_mul_ps(sa, B.sRX());
	to.M40() = sa_TXb.m128_f32[0] + to.M40();
	to.M42() = sa_sRX_sb.m128_f32[2] + to.M42();
	to.M43() = -sa_sRX_sb.m128_f32[1] + to.M43();
	to.M44() = sa_sRX_sb.m128_f32[3] + to.M44();

	to.M50() = sa_TXb.m128_f32[1] + to.M50();
	to.M51() = -sa_sRX_sb.m128_f32[2] + to.M51();
	to.M53() = sa_sRX_sb.m128_f32[0] + to.M53();
	to.M55() = sa_sRX_sb.m128_f32[3] + to.M55();

	to.M60() = sa_TXb.m128_f32[2] + to.M60();
	to.M61() = sa_sRX_sb.m128_f32[1] + to.M61();
	to.M62() = -sa_sRX_sb.m128_f32[0] + to.M62();
	to.M66() = sa_sRX_sb.m128_f32[3] + to.M66();
}

static inline void AddAij2To(const JacobianMatrix3x7f &A, LA::AlignedVector7f &to, ENFT_SSE::__m128 &work)
{
	to.v0() = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(A.TX(), A.TX())) + to.v0();
	work = ENFT_SSE::_mm_mul_ps(A.sRX(), A.sRX());
	to.v1() = work.m128_f32[1] + work.m128_f32[2] + to.v1();
	to.v2() = work.m128_f32[2] + work.m128_f32[0] + to.v2();
	to.v3() = work.m128_f32[0] + work.m128_f32[1] + to.v3();
	to.v4() = work.m128_f32[3] + to.v4();
	to.v5() = work.m128_f32[3] + to.v5();
	to.v6() = work.m128_f32[3] + to.v6();
}

template<class MATRIX> inline void FinishAdditionAij2To(LA::AlignedVector7f &to) {}
template<> inline void FinishAdditionAij2To<JacobianMatrix3x7f>(LA::AlignedVector7f &to)
{
	to.v1() = to.v2() = to.v3() = (to.v1() + to.v2() + to.v3()) / 3;
	//to.v5() = to.v6() = to.v4();
}

void SequenceTransformationOptimizerDataSimilarity::ConstructNormalEquation(AlignedVector<LA::AlignedMatrix7f> &As, AlignedVector<LA::AlignedVector7f> &bs, 
																			AlignedVector<LA::AlignedVector7f> &ss) const
{
	As.SetZero();
	bs.SetZero();
	ss.SetZero();

	ENFT_SSE::__m128 work[2];
	Point3D e;
	JacobianMatrix3x7f J1, J2;

	const uint nSeqPairs = uint(m_mapPairs.size());
	for(uint i = 0; i < nSeqPairs; ++i)
	{
		const SequenceIndex iSeq1 = m_mapPairs[i].GetMapIndex1(), iSeq2 = m_mapPairs[i].GetMapIndex2();
		const SimilarityTransformation3D &S1 = m_Ts[iSeq1], &S2 = m_Ts[iSeq2];
		if(iSeq1 == 0)
		{
			const uint iBlock2 = uint(iSeq2);
			LA::AlignedMatrix7f &A2 = As[iBlock2];
			LA::AlignedVector7f &b2 = bs[iSeq2], &s2 = ss[iSeq2];
			const AlignedVector<PointPair> &Xs = m_mapPairs[i].Xs();
			const TrackIndex nPtPairs = TrackIndex(Xs.Size());
			for(TrackIndex j = 0; j < nPtPairs; ++j)
			{
				ComputeTransformationErrorAndJacobian(S1, Xs[j].X1(), S2, Xs[j].X2(), e, J2, work[0]/*, m_fixScales*/);
				AddATAToUpper(J2, A2, work);
				AddATBTo(J2, e, b2);
				AddAij2To(J2, s2, work[0]);
			}
		}
		else
		{
			const uint iBlock1 = uint(iSeq1), iBlock2 = uint(iSeq2), iBlock12 = m_mapPairs[i].GetBlockIndex();
			LA::AlignedMatrix7f &A1 = As[iBlock1], &A2 = As[iBlock2], &A12 = As[iBlock12];
			LA::AlignedVector7f &b1 = bs[iSeq1], &b2 = bs[iSeq2], &s1 = ss[iSeq1], &s2 = ss[iSeq2];
			const AlignedVector<PointPair> &Xs = m_mapPairs[i].Xs();
			const TrackIndex nPtPairs = TrackIndex(Xs.Size());
			for(TrackIndex j = 0; j < nPtPairs; ++j)
			{
				ComputeTransformationErrorAndJacobian(S1, Xs[j].X1(), S2, Xs[j].X2(), e, J1, J2, work[0]/*, m_fixScales*/);
				AddATAToUpper(J1, A1, work);
				AddATAToUpper(J2, A2, work);
				AddATBTo(J1, J2, A12, work);
				AddATBTo(J1, e, b1);
				AddATBTo(J2, e, b2);
 				AddAij2To(J1, s1, work[0]);
				AddAij2To(J2, s2, work[0]);
			}
		}
	}

	LA::AlignedMatrix7f s12;
	const SequenceIndex nSeqs = GetMapsNumber();
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		LA::AlignedVector7f &s = ss[iSeq];
		LA::SetReserve<GTO_STAGE_S>(s);

		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ABT(s, s, s12, work);

		const uint iBlock = uint(iSeq);
		LA::SetLowerFromUpper(As[iBlock]);
		LA::sA(s12, As[iBlock]);
		LA::SetReserve(As[iBlock]);

		LA::sA(s, bs[iBlock]);
		LA::SetReserve<GTO_STAGE_JTE>(bs[iSeq]);
	}
	for(uint i = 0; i < nSeqPairs; ++i)
	{
		const SequenceIndex iSeq1 = m_mapPairs[i].GetMapIndex1(), iSeq2 = m_mapPairs[i].GetMapIndex2();
		LA::ABT(ss[iSeq1], ss[iSeq2], s12, work);
		const uint iBlock12 = m_mapPairs[i].GetBlockIndex();
		LA::sA(s12, As[iBlock12]);
		LA::SetReserve(As[iBlock12]);
	}
}

void SequenceTransformationOptimizerDataSimilarity::ConstructNormalEquation(const AlignedVector<LA::AlignedVector7f> &ss, AlignedVector<LA::AlignedMatrix7f> &As, 
																			AlignedVector<LA::AlignedVector7f> &bs) const
{
	As.SetZero();
	bs.SetZero();

	ENFT_SSE::__m128 work[2];
	Point3D e;
	JacobianMatrix3x7f J1, J2;

	const uint nSeqPairs = uint(m_mapPairs.size());
	for(uint i = 0; i < nSeqPairs; ++i)
	{
		const SequenceIndex iSeq1 = m_mapPairs[i].GetMapIndex1(), iSeq2 = m_mapPairs[i].GetMapIndex2();
		const SimilarityTransformation3D &S1 = m_Ts[iSeq1], &S2 = m_Ts[iSeq2];
		if(iSeq1 == 0)
		{
			const uint iBlock2 = uint(iSeq2);
			LA::AlignedMatrix7f &A2 = As[iBlock2];
			LA::AlignedVector7f &b2 = bs[iSeq2];
			const AlignedVector<PointPair> &Xs = m_mapPairs[i].Xs();
			const TrackIndex nPtPairs = TrackIndex(Xs.Size());
			for(TrackIndex j = 0; j < nPtPairs; ++j)
			{
				ComputeTransformationErrorAndJacobian(S1, Xs[j].X1(), S2, Xs[j].X2(), e, J2, work[0]/*, m_fixScales*/);
				AddATAToUpper(J2, A2, work);
				AddATBTo(J2, e, b2);
			}
		}
		else
		{
			const uint iBlock1 = uint(iSeq1), iBlock2 = uint(iSeq2), iBlock12 = m_mapPairs[i].GetBlockIndex();
			LA::AlignedMatrix7f &A1 = As[iBlock1], &A2 = As[iBlock2], &A12 = As[iBlock12];
			LA::AlignedVector7f &b1 = bs[iSeq1], &b2 = bs[iSeq2];
			const AlignedVector<PointPair> &Xs = m_mapPairs[i].Xs();
			const TrackIndex nPtPairs = TrackIndex(Xs.Size());
			for(TrackIndex j = 0; j < nPtPairs; ++j)
			{
				ComputeTransformationErrorAndJacobian(S1, Xs[j].X1(), S2, Xs[j].X2(), e, J1, J2, work[0]/*, m_fixScales*/);
				AddATAToUpper(J1, A1, work);
				AddATAToUpper(J2, A2, work);
				AddATBTo(J1, J2, A12, work);
				AddATBTo(J1, e, b1);
				AddATBTo(J2, e, b2);
			}
		}
	}

	LA::AlignedMatrix7f s12;
	const SequenceIndex nSeqs = GetMapsNumber();
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		const LA::AlignedVector7f &s = ss[iSeq];
		LA::ABT(s, s, s12, work);

		const uint iBlock = uint(iSeq);
		LA::SetLowerFromUpper(As[iBlock]);
		LA::sA(s12, As[iBlock]);
		LA::SetReserve(As[iBlock]);

		LA::sA(s, bs[iBlock]);
		LA::SetReserve<GTO_STAGE_JTE>(bs[iSeq]);
	}
	for(uint i = 0; i < nSeqPairs; ++i)
	{
		const SequenceIndex iSeq1 = m_mapPairs[i].GetMapIndex1(), iSeq2 = m_mapPairs[i].GetMapIndex2();
		LA::ABT(ss[iSeq1], ss[iSeq2], s12, work);
		const uint iBlock12 = m_mapPairs[i].GetBlockIndex();
		LA::sA(s12, As[iBlock12]);
		LA::SetReserve(As[iBlock12]);
	}
}

static inline void UpdateSimilarity(const LA::AlignedVector7f &w, const LA::AlignedVector7f &dp, const SimilarityTransformation3D &Sold, SimilarityTransformation3D &Snew, 
									RotationTransformation3D &dR, ENFT_SSE::__m128 &work1m, float* const &work24f)
{
	work1m = ENFT_SSE::_mm_mul_ps(w.v0123(), dp.v0123());
	Snew.SetScale(Sold.s() + work1m.m128_f32[0]);
	dR.FromRodrigues(work1m.m128_f32[1], work1m.m128_f32[2], work1m.m128_f32[3], work24f);
	work1m = ENFT_SSE::_mm_mul_ps(w.v456x(), dp.v456x());
	Snew.tX() = work1m.m128_f32[0] + Sold.tX();
	Snew.tY() = work1m.m128_f32[1] + Sold.tY();
	Snew.tZ() = work1m.m128_f32[2] + Sold.tZ();
	Sold.LeftMultiplyRotation(dR, Snew, work1m);
}

void SequenceTransformationOptimizerDataSimilarity::UpdateTransformations(const AlignedVector<LA::AlignedVector7f> &ws, const AlignedVector<LA::AlignedVector7f> &dps, 
																		  const AlignedVector<SimilarityTransformation3D> &TsOld)
{
	RotationTransformation3D dR;
	ENFT_SSE::__m128 work1m;
	float work24f[24];
	const SequenceIndex nSeqs = SequenceIndex(ws.Size());
	for(SequenceIndex i = 0; i < nSeqs; ++i)
		UpdateSimilarity(ws[i], dps[i], TsOld[i], m_Ts[i], dR, work1m, work24f);
}

void SequenceTransformationOptimizerDataSimilarity::SetScales(const AlignedVector<float> &scales)
{
	const SegmentIndex nSegs = SegmentIndex(m_Ts.Size());
	for(SegmentIndex iSeg = 0; iSeg < nSegs; ++iSeg)
		m_Ts[iSeg].SetScale(scales[iSeg]);
	//m_fixScales = true;
}