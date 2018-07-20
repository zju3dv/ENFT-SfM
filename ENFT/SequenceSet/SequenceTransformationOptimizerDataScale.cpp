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
#include "SequenceTransformationOptimizerDataScale.h"

void SequenceTransformationOptimizerDataScale::NormalizeData(const float dataNormalizeMedian)
{
	if(dataNormalizeMedian == 0)
	{
		m_scale = 1;
		return;
	}
	std::vector<float> lens;
	lens.reserve(m_nPts);
	const uint nMapPairs = uint(m_mapPairs.size());
	for(uint i = 0; i < nMapPairs; ++i)
	{
		const AlignedVector<PointPair> &ls = m_mapPairs[i].Xs();
		const TrackIndex nLinePairs = TrackIndex(ls.Size());
		for(TrackIndex j = 0; j < nLinePairs; ++j)
		{
			const float &l1 = ls[j].X1(), &l2 = ls[j].X2();
			lens.push_back(l1);
			lens.push_back(l2);
		}
	}
	const TrackIndex ith = (m_nPts >> 1);
	std::nth_element(lens.begin(), lens.begin() + ith, lens.end());
	const float lenMed = lens[ith];
	m_scale = (1 / lenMed) / dataNormalizeMedian;

	for(uint i = 0; i < nMapPairs; ++i)
	{
		AlignedVector<PointPair> &ls = m_mapPairs[i].Xs();
		const TrackIndex nLinePairs = TrackIndex(ls.Size());
		for(TrackIndex j = 0; j < nLinePairs; ++j)
		{
			ls[j].X1() *= m_scale;
			ls[j].X2() *= m_scale;
		}
	}
}

void SequenceTransformationOptimizerDataScale::DenormalizeData() {}

//void SequenceTransformationOptimizerDataScale::InvertTransformations()
//{
//	const ENFT_SSE::__m128 one = ENFT_SSE::_mm_set1_ps(1.0f);
//	const ENFT_SSE::__m128 *ps = (ENFT_SSE::__m128 *) m_Ts.Data();
//	ENFT_SSE::__m128 *psinv = (ENFT_SSE::__m128 *) m_Tinvs.Data();
//	const SequenceIndex N = SequenceIndex(m_Ts.Size()), _N = N - (N & 3);
//	for(SequenceIndex i = 0; i < _N; i += 4, ++ps, ++psinv)
//		*psinv = ENFT_SSE::_mm_div_ps(one, *ps);
//	for(SequenceIndex i = _N; i < N; ++i)
//		m_Tinvs[i] = 1 / m_Ts[i];
//}

static inline void ComputeTransformationError(const float &s1, const float &l1, const float &s2, const float &l2, float &e)
{
	e = s1 * l1 - s2 * l2;
}

double SequenceTransformationOptimizerDataScale::ComputeTransformationSSE() const
{
	float e, sum = 0;
	const uint nMapPairs = uint(m_mapPairs.size());
	for(uint i = 0; i < nMapPairs; ++i)
	{
		const SequenceIndex iSeq1 = m_mapPairs[i].GetMapIndex1(), iSeq2 = m_mapPairs[i].GetMapIndex2();
		const float &s1 = m_Ts[iSeq1], &s2 = m_Ts[iSeq2];
		const AlignedVector<PointPair> &ls = m_mapPairs[i].Xs();
		const TrackIndex nLinePairs = TrackIndex(ls.Size());
		for(TrackIndex j = 0; j < nLinePairs; ++j)
		{
			ComputeTransformationError(s1, ls[j].X1(), s2, ls[j].X2(), e);
			sum = e * e + sum;
		}
	}
	return sum;
}

static inline void ComputeTransformationErrorAndJacobian(const float &s1, const float &l1, const float &s2, const float &l2, float &e, float &J1, float &J2)
{
	//e = s1 * l1 - s2 * l2;
	e = s2 * l2 - s1 * l1;
	J1 = l1;
	J2 = -l2;
}

static inline void ComputeTransformationErrorAndJacobian(const float &s1, const float &l1, const float &s2, const float &l2, float &e, float &J2)
{
	//e = s1 * l1 - s2 * l2;
	e = s2 * l2 - s1 * l1;
	J2 = -l2;
}

void SequenceTransformationOptimizerDataScale::ConstructNormalEquation(AlignedVector<LA::Matrix1f> &As, AlignedVector<float> &bs, AlignedVector<float> &ss) const
{
	As.SetZero();
	bs.SetZero();

	float e, J1, J2;
	const uint nMapPairs = uint(m_mapPairs.size());
	for(uint i = 0; i < nMapPairs; ++i)
	{
		const SequenceIndex iSeq1 = m_mapPairs[i].GetMapIndex1(), iSeq2 = m_mapPairs[i].GetMapIndex2();
		const float &s1 = m_Ts[iSeq1], &s2 = m_Ts[iSeq2];
		if(iSeq1 == 0)
		{
			const uint iBlock2 = uint(iSeq2);
			float &A2 = As[iBlock2];
			float &b2 = bs[iSeq2];
			const AlignedVector<PointPair> &ls = m_mapPairs[i].Xs();
			const TrackIndex nLinePairs = TrackIndex(ls.Size());
			for(TrackIndex j = 0; j < nLinePairs; ++j)
			{
				ComputeTransformationErrorAndJacobian(s1, ls[j].X1(), s2, ls[j].X2(), e, J2);
				A2 = J2 * J2 + A2;
				b2 = J2 * e + b2;
			}
		}
		else
		{
			const uint iBlock1 = uint(iSeq1), iBlock2 = uint(iSeq2), iBlock12 = m_mapPairs[i].GetBlockIndex();
			float &A1 = As[iBlock1], &A2 = As[iBlock2], &A12 = As[iBlock12];
			float &b1 = bs[iSeq1], &b2 = bs[iSeq2];
			const AlignedVector<PointPair> &ls = m_mapPairs[i].Xs();
			const TrackIndex nLinePairs = TrackIndex(ls.Size());
			for(TrackIndex j = 0; j < nLinePairs; ++j)
			{
				ComputeTransformationErrorAndJacobian(s1, ls[j].X1(), s2, ls[j].X2(), e, J1, J2);
				A1 = J1 * J1 + A1;
				A2 = J2 * J2 + A2;
				A12 = J1 * J2 + A12;
				b1 = J1 * e + b1;
				b2 = J2 * e + b2;
			}
		}
	}

	const SequenceIndex nSeqs = GetMapsNumber();
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		const uint iBlock = uint(iSeq);
		float &s = ss[iSeq];
		s = As[iBlock];
		s = s == 0 ? 0 : sqrt(1 / s);

		As[iBlock].Set(1.0f);
		bs[iBlock] *= s;
	}
	float s12;
	for(uint i = 0; i < nMapPairs; ++i)
	{
		const SequenceIndex iSeq1 = m_mapPairs[i].GetMapIndex1(), iSeq2 = m_mapPairs[i].GetMapIndex2();
		s12 = ss[iSeq1] * ss[iSeq2];
		const uint iBlock12 = m_mapPairs[i].GetBlockIndex();
		As[iBlock12] *= s12;
	}
}

void SequenceTransformationOptimizerDataScale::ConstructNormalEquation(const AlignedVector<float> &ss, AlignedVector<LA::Matrix1f> &As, AlignedVector<float> &bs) const
{
	As.SetZero();
	bs.SetZero();

	float e, J1, J2;
	const uint nMapPairs = uint(m_mapPairs.size());
	for(uint i = 0; i < nMapPairs; ++i)
	{
		const SequenceIndex iSeq1 = m_mapPairs[i].GetMapIndex1(), iSeq2 = m_mapPairs[i].GetMapIndex2();
		const float &s1 = m_Ts[iSeq1], &s2 = m_Ts[iSeq2];
		if(iSeq1 == 0)
		{
			const uint iBlock2 = uint(iSeq2);
			float &A2 = As[iBlock2];
			float &b2 = bs[iSeq2];
			const AlignedVector<PointPair> &ls = m_mapPairs[i].Xs();
			const TrackIndex nLinePairs = TrackIndex(ls.Size());
			for(TrackIndex j = 0; j < nLinePairs; ++j)
			{
				ComputeTransformationErrorAndJacobian(s1, ls[j].X1(), s2, ls[j].X2(), e, J2);
				A2 = J2 * J2 + A2;
				b2 = J2 * e + b2;
			}
		}
		else
		{
			const uint iBlock1 = uint(iSeq1), iBlock2 = uint(iSeq2), iBlock12 = m_mapPairs[i].GetBlockIndex();
			float &A1 = As[iBlock1], &A2 = As[iBlock2], &A12 = As[iBlock12];
			float &b1 = bs[iSeq1], &b2 = bs[iSeq2];
			const AlignedVector<PointPair> &ls = m_mapPairs[i].Xs();
			const TrackIndex nLinePairs = TrackIndex(ls.Size());
			for(TrackIndex j = 0; j < nLinePairs; ++j)
			{
				ComputeTransformationErrorAndJacobian(s1, ls[j].X1(), s2, ls[j].X2(), e, J1, J2);
				A1 = J1 * J1 + A1;
				A2 = J2 * J2 + A2;
				A12 = J1 * J2 + A12;
				b1 = J1 * e + b1;
				b2 = J2 * e + b2;
			}
		}
	}

	float s12;
	const SequenceIndex nSeqs = GetMapsNumber();
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		const float &s = ss[iSeq];
		s12 = s * s;

		const uint iBlock = uint(iSeq);
		As[iBlock] *= s12;
		bs[iBlock] *= s;
	}
	for(uint i = 0; i < nMapPairs; ++i)
	{
		const SequenceIndex iSeq1 = m_mapPairs[i].GetMapIndex1(), iSeq2 = m_mapPairs[i].GetMapIndex2();
		s12 = ss[iSeq1] * ss[iSeq2];
		const uint iBlock12 = m_mapPairs[i].GetBlockIndex();
		As[iBlock12] *= s12;
	}
}

static inline void UpdateScale(const float &w, const float &dp, const float &sOld, float &sNew)
{
	sNew = sOld + w * dp;
}

void SequenceTransformationOptimizerDataScale::UpdateTransformations(const AlignedVector<float> &ws, const AlignedVector<float> &dps, 
																	 const AlignedVector<float> &TsOld)
{
	const SequenceIndex nSeqs = SequenceIndex(ws.Size());
	for(SequenceIndex i = 0; i < nSeqs; ++i)
		UpdateScale(ws[i], dps[i], TsOld[i], m_Ts[i]);
}