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
#include "Optimization/Optimizer.h"
#include "SfM/Point2DDistortionData.h"
#include "Utility/Utility.h"

void Sequence::InitializeMeasurements()
{
	const MeasurementIndex nMeas = MeasurementIndex(m_meaStates.size());
	for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea)
		MarkMeasurementInlier(iMea);
}

void Sequence::SetMeasurementStates(const MeasurementIndexList &iMeas, const MeasurementStateList &meaStates)
{
	const MeasurementIndex nMeas = MeasurementIndex(iMeas.size());
	for(MeasurementIndex i = 0; i < nMeas; ++i)
		m_meaStates[iMeas[i]] = meaStates[i];
}

MeasurementIndex Sequence::CountMeasurements(const MeasurementState meaState) const
{
	MeasurementIndex cnt = 0;
	const MeasurementIndex nMeas = GetMeasurementsNumber();
	for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea)
	{
		if(m_meaStates[iMea] & meaState)
			++cnt;
	}
	return cnt;
}

void Sequence::NullifyMeasurement(const MeasurementIndex &iMea)
{
	const TrackIndex iTrk = m_mapMeaToTrk[iMea];
	m_mapMeaToTrk[iMea] = INVALID_TRACK_INDEX;
	MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
	iMeas.erase(std::lower_bound(iMeas.begin(), iMeas.end(), iMea));
}

void Sequence::RemoveNullMeasurements()
{
	const MeasurementIndex nMeasOri = GetMeasurementsNumber();
	MeasurementIndexList iMeasOriToNew(nMeasOri, INVALID_MEASUREMENT_INDEX);

	MeasurementIndex iMeaOri, iMeaNew = 0;
	const FrameIndex nFrms = GetFramesNumber();
	for(FrameIndex iFrm = 0; iFrm < nFrms; ++iFrm)
	{
		const MeasurementIndex iMeaOri1 = m_mapFrmToMea[iFrm], iMeaOri2 = m_mapFrmToMea[iFrm + 1];
		m_mapFrmToMea[iFrm] = iMeaNew;
		for(iMeaOri = iMeaOri1; iMeaOri < iMeaOri2; ++iMeaOri)
		{
			if(m_mapMeaToTrk[iMeaOri] == INVALID_TRACK_INDEX)
				continue;
			m_xs[iMeaNew] = m_xs[iMeaOri];
			m_mapMeaToFrm[iMeaNew] = m_mapMeaToFrm[iMeaOri];
			m_mapMeaToTrk[iMeaNew] = m_mapMeaToTrk[iMeaOri];
			m_meaStates[iMeaNew] = m_meaStates[iMeaOri];
#if DESCRIPTOR_TRACK == 0
			if(!m_descs.Empty())
				m_descs[iMeaNew] = m_descs[iMeaOri];
#endif
			iMeasOriToNew[iMeaOri] = iMeaNew;
			++iMeaNew;
		}
	}
	const MeasurementIndex nMeasNew = iMeaNew;
	if(nMeasNew == nMeasOri)
		return;
	m_xs.Resize(nMeasNew);
	m_mapFrmToMea.back() = nMeasNew;
	m_mapMeaToFrm.resize(nMeasNew);
	m_mapMeaToTrk.resize(nMeasNew);
	m_meaStates.resize(nMeasNew);
#if DESCRIPTOR_TRACK == 0
	if(!m_descs.Empty())
		m_descs.Resize(nMeasNew);
#endif

	const TrackIndex nTrks = GetTracksNumber();
	for(TrackIndex iTrk = 0; iTrk < nTrks; ++iTrk)
	{
		MeasurementIndexList &iMeas = m_mapTrkToMea[iTrk];
		const FrameIndex nCrsps = FrameIndex(iMeas.size());
		for(FrameIndex i = 0; i < nCrsps; ++i)
		{
			iMeaNew = iMeasOriToNew[iMeas[i]];
#if _DEBUG
			assert(iMeaNew != INVALID_MEASUREMENT_INDEX);
#endif
			iMeas[i] = iMeaNew;
		}
	}
}

void Sequence::ComputeMeasurementsDistribution(const MeasurementIndexList &iMeas, Point2D &mean, LA::Vector3f &cov) const
{
	mean.SetZero();
	const MeasurementIndex nMeas = MeasurementIndex(iMeas.size());
	for(MeasurementIndex i = 0; i < nMeas; ++i)
		mean += m_xs[iMeas[i]];
	float norm = 1.0f / nMeas;
	mean *= norm;

	Point2D dx;
	cov.SetZero();
	for(MeasurementIndex i = 0; i < nMeas; ++i)
	{
		const Point2D &x = m_xs[iMeas[i]];
		dx.Set(x.x() - mean.x(), x.y() - mean.y());
		cov.v0() += dx.x() * dx.x();
		cov.v1() += dx.x() * dx.y();
		cov.v2() += dx.y() * dx.y();
	}
	norm = 1.0f / (nMeas - 1);
	cov *= norm;

	if(m_measNormalized)
	{
		m_K.NormalizedPlaneToImage(mean);
		cov.v0() *= m_K.fx() * m_K.fx();
		cov.v1() *= m_K.fxy();
		cov.v2() *= m_K.fy() * m_K.fy();
	}
}

void Sequence::BucketMeasurements(const ubyte nBinsX, const ubyte nBinsY, std::vector<ubyte> &mapMeaToBin) const
{
	IO::Assert(!m_measNormalized, "m_meaNormalized = %d\n", m_measNormalized);
	IO::Assert(uint(nBinsX) * uint(nBinsY) <= UCHAR_MAX, "nBinsX = %d, nBinsY = %d\n", nBinsX, nBinsY);

	// Create bucket
	std::vector<ubyte> iBinsX, iBinsY;
	iBinsX.resize(m_tag.GetImageWidth() + 1);
	const float binWidth = float(m_tag.GetImageWidth()) / nBinsX;
	float x1 = 0, x2 = binWidth;
	ubyte x1i = 0, x2i = ubyte(x2 + 0.5f);
	for(ubyte i = 0; i < nBinsX; ++i, x1 = x2, x2 += binWidth, x1i = x2i, x2i = ubyte(x2 + 0.5f))
	for(ubyte x = x1i; x < x2i; ++x)
		iBinsX[x] = i;
	iBinsY.resize(m_tag.GetImageHeight() + 1);
	const float binHeight = float(m_tag.GetImageHeight()) / nBinsY;
	float y1 = 0, y2 = binHeight;
	ubyte y1i = 0, y2i = ubyte(y2 + 0.5f);
	int iBin = 0;
	for(uint i = 0; i < nBinsY; ++i, iBin += nBinsX, y1 = y2, y2 += binHeight, y1i = y2i, y2i = ubyte(y2 + 0.5f))
	for(ubyte y = y1i; y < y2i; ++y)
		iBinsY[y] = iBin;

	// Bucket measurements
	const MeasurementIndex nMeas = GetMeasurementsNumber();
	mapMeaToBin.resize(nMeas);
	for(MeasurementIndex iMea = 0; iMea < nMeas; ++iMea)
		mapMeaToBin[iMea] = iBinsY[size_t(m_xs[iMea].y() + 0.5f)] + iBinsX[size_t(m_xs[iMea].x() + 0.5f)];
}

void Sequence::NormalizeMeasurements()
{
	if(m_measNormalized)
		return;
	m_K.ImageToNormalizedPlaneN(m_xs);
	m_measNormalized = true;
}

void Sequence::DenormalizeMeasurements()
{
	if(!m_measNormalized)
		return;
	m_K.NormalizedPlaneToImageN(m_xs);
	m_measNormalized = false;
}

void Sequence::ScaleMeasurements(const float scale, AlignedVector<Point2D> &xs)
{
	if(scale == 1.0f)
		return;
	ENFT_SSE::__m128 s = ENFT_SSE::_mm_set1_ps(scale);
	const uint N = xs.Size(), _N = N - (N & 1);
	ENFT_SSE::__m128 *x = (ENFT_SSE::__m128 *) xs.Data();
	for(uint i = 0; i < _N; i += 2, ++x)
		*x = ENFT_SSE::_mm_mul_ps(s, *x);
	if(_N != N)
		xs[_N] *= scale;
}

void Sequence::DistortMeasurements(const float fxy, const float distortion, AlignedVector<Point2D> &xs)
{
	if(distortion == 0.0f)
		return;
	OptimizerTemplate<Point2D, LA::Vector2f, LA::AlignedMatrix2f> optor;
	Point2DDistortionData data;
	data.SetFocal(fxy);
	data.SetDistortion(distortion);
	const uint N = xs.Size();
	for(uint i = 0; i < N; ++i)
	{
		data.SetPoint(xs[i]);
		//printf("%d", i);
		optor.Run(data, xs[i]/*, 1*/);
		//printf("%f\n", data.ComputeSSE(m_xs[i]));
	}
}

void Sequence::UndistortMeasurements(const float distortion, AlignedVector<Point2D> &xs)
{
	if(distortion == 0.0f)
		return;
	ENFT_SSE::__m128 r2, c;
	const uint N = xs.Size(), _N = N - (N & 1);
	ENFT_SSE::__m128 *x = (ENFT_SSE::__m128 *) xs.Data();
	for(uint i = 0; i < _N; i += 2, ++x)
	{
		r2 = ENFT_SSE::_mm_mul_ps(*x, *x);
		c.m128_f32[0] = c.m128_f32[1] = 1 + distortion * (r2.m128_f32[0] + r2.m128_f32[1]);
		c.m128_f32[2] = c.m128_f32[3] = 1 + distortion * (r2.m128_f32[2] + r2.m128_f32[3]);
		*x = ENFT_SSE::_mm_mul_ps(c, *x);
	}
	if(_N != N)
		xs[_N] *= 1 + distortion * xs[_N].SquaredLength();
}

void Sequence::RectifyMeasurements(const Camera::IntrinsicParameter &Kr, AlignedVector<Point2D> &xs)
{
	const float d = Kr.d(), finv = 1 / Kr.f();
	if(d == 0.0f)
	{
		ScaleMeasurements(finv, xs);
		return;
	}
	ENFT_SSE::__m128 r2, c;
	const uint N = xs.Size(), _N = N - (N & 1);
	ENFT_SSE::__m128 *x = (ENFT_SSE::__m128 *) xs.Data();
	for(uint i = 0; i < _N; i += 2, ++x)
	{
		r2 = ENFT_SSE::_mm_mul_ps(*x, *x);
		c.m128_f32[0] = c.m128_f32[1] = (1 + d * (r2.m128_f32[0] + r2.m128_f32[1])) * finv;
		c.m128_f32[2] = c.m128_f32[3] = (1 + d * (r2.m128_f32[2] + r2.m128_f32[3])) * finv;
		*x = ENFT_SSE::_mm_mul_ps(c, *x);
	}
	if(_N != N)
		xs[_N] *= (1 + d * xs[_N].SquaredLength()) * finv;
}