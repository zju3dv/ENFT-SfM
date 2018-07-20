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

#ifndef _HOMOGRAPHY_ESTIMATOR_DATA_H_
#define _HOMOGRAPHY_ESTIMATOR_DATA_H_

#include "SfM/Match.h"
#include "SfM/Homography.h"
#include "Optimization/OptimizerData.h"
#include "LinearAlgebra/Matrix6.h"
#include "LinearAlgebra/Matrix7.h"
#include "LinearAlgebra/Matrix8.h"

class HomographyEstimatorData : public MatchSet2D, public OptimizerDataTemplate<Homography, LA::AlignedVector8f, LA::AlignedMatrix8f>
{

public:

	inline void ValidateWeights() { m_w2s.Resize(Size()); m_ws.Resize(Size()); }
	inline void InvalidateWeights() { m_w2s.Resize(0); m_ws.Resize(0); }
	inline bool AreWeightsValid() const { return !m_w2s.Empty(); }
	inline const float& GetWeight(const ushort &i) const { return m_ws[i]; }
	inline const float& GetSquaredWeight(const ushort &i) const { return m_w2s[i]; }
	inline void SetWeight(const ushort &i, const float &w) { m_ws[i] = w; }
	inline void FinishSettingWeights()
	{
		ENFT_SSE::__m128 wSum = ENFT_SSE::_mm_setzero_ps();
		ENFT_SSE::__m128 *w = (ENFT_SSE::__m128 *) m_ws.Data();
		const ushort N = ushort(m_ws.Size()), _N = N - (N & 3);
		for(ushort i = 0; i < _N; i += 4, ++w)
			wSum = ENFT_SSE::_mm_add_ps(*w, wSum);
		for(ushort i = _N; i < N; ++i)
			wSum.m128_f32[0] += m_ws[i];
		const ENFT_SSE::__m128 s = _mm_set1_ps(1 / ENFT_SSE::SSE::Sum0123(wSum));
		w = (ENFT_SSE::__m128 *) m_ws.Data();
		ENFT_SSE::__m128 *w2 = (ENFT_SSE::__m128 *) m_w2s.Data();
		for(ushort i = 0; i < _N; i += 4, ++w, ++w2)
		{
			*w = ENFT_SSE::_mm_mul_ps(s, *w);
			*w2 = ENFT_SSE::_mm_mul_ps(*w, *w);
		}
		for(ushort i = _N; i < N; ++i)
		{
			m_ws[i] *= s.m128_f32[0];
			m_w2s[i] = m_ws[i] * m_ws[i];
		}
	}
	inline void GetSubset(const std::vector<ushort> &idxs, HomographyEstimatorData &subset) const
	{
		MatchSet2D::GetSubset(idxs, subset);
		if(m_w2s.Empty())
			return;
		subset.ValidateWeights();
		const ushort N = ushort(idxs.size());
		for(ushort i = 0; i < N; ++i)
		{
			subset.m_w2s[i] = m_w2s[idxs[i]];
			subset.m_ws[i] = m_ws[idxs[i]];
		}
	}

	virtual void NormalizeData(const float &dataNormalizeMedian, Homography &H)
	{
		if(dataNormalizeMedian == 0)
		{
			m_scale1 = m_scale2 = 1.0f;
			m_normalized = false;
		}
		else
		{
			Normalize(m_work);
			H.Normalize(mean_u1v1u2v2(), scale1(), scale2(), m_work);
			m_normalized = true;
		}
	}
	virtual void DenormalizeData(Homography &H)
	{
		if(m_normalized)
			H.Denormalize(mean_u1v1u2v2(), scale1(), scale2(), m_work);
	}
	virtual double ComputeSSE(const Homography &H)
	{
		ENFT_SSE::__m128 *h = m_work;
		H.GetSSE(h);

		ENFT_SSE::__m128 &one = m_work[9], &ex = m_work[10], &ey = m_work[11], &sum = m_work[12];
		one = _mm_set1_ps(1.0f);
		sum = ENFT_SSE::_mm_setzero_ps();
		if(m_w2s.Empty())
		{
			const ushort nPacks = GetPacksNumber(), nRems = GetRemindersNumber();
			for(ushort iu1 = 0, iv1 = 1, iu2 = 2, iv2 = 3; iu1 < nPacks; iu1 += 4, iv1 += 4, iu2 += 4, iv2 += 4)
			{
				const ENFT_SSE::__m128 &u1 = GetPack(iu1), &v1 = GetPack(iv1), &u2 = GetPack(iu2), &v2 = GetPack(iv2);
				ey = ENFT_SSE::_mm_div_ps(one, ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[6], u1), ENFT_SSE::_mm_mul_ps(h[7], v1)), h[8]));
				ex = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[0], u1), ENFT_SSE::_mm_mul_ps(h[1], v1)), h[2]), ey), u2);
				ey = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[3], u1), ENFT_SSE::_mm_mul_ps(h[4], v1)), h[5]), ey), v2);
				sum = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(ex, ex), ENFT_SSE::_mm_mul_ps(ey, ey)), sum);
			}
			for(ushort iRem = 0; iRem < nRems; ++iRem)
			{
				float *work = (float *) m_work;
				sum.m128_f32[0] = H.ComputeSquaredError(GetReminder1(iRem), GetReminder2(iRem), work) + sum.m128_f32[0];
			}
		}
		else
		{
			const ENFT_SSE::__m128 *w2 = (ENFT_SSE::__m128 *) m_w2s.Data();
			const ushort nPacks = GetPacksNumber(), nRems = GetRemindersNumber();
			for(ushort iu1 = 0, iv1 = 1, iu2 = 2, iv2 = 3; iu1 < nPacks; iu1 += 4, iv1 += 4, iu2 += 4, iv2 += 4, ++w2)
			{
				const ENFT_SSE::__m128 &u1 = GetPack(iu1), &v1 = GetPack(iv1), &u2 = GetPack(iu2), &v2 = GetPack(iv2);
				ey = ENFT_SSE::_mm_div_ps(one, ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[6], u1), ENFT_SSE::_mm_mul_ps(h[7], v1)), h[8]));
				ex = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[0], u1), ENFT_SSE::_mm_mul_ps(h[1], v1)), h[2]), ey), u2);
				ey = ENFT_SSE::_mm_sub_ps(ENFT_SSE::_mm_mul_ps(_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[3], u1), ENFT_SSE::_mm_mul_ps(h[4], v1)), h[5]), ey), v2);
				sum = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(*w2, ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ex, ex), ENFT_SSE::_mm_mul_ps(ey, ey))), sum);
			}
			for(ushort iRem = 0; iRem < nRems; ++iRem)
			{
				float *work = (float *) m_work;
				sum.m128_f32[0] = w2->m128_f32[iRem] * H.ComputeSquaredError(GetReminder1(iRem), GetReminder2(iRem), work) + sum.m128_f32[0];
			}
		}
		return double(ENFT_SSE::SSE::Sum0123(sum));
	}
	virtual double GetFactorSSEToMSE()
	{
		return double(1 / (scale1() * scale2() * Size()));
	}
	virtual void ConstructNormalEquation(const Homography &H, LA::AlignedMatrix8f &A, LA::AlignedVector8f &b, LA::AlignedVector8f &s)
	{
		ENFT_SSE::__m128 *h = m_work;
		H.GetSSE(h);
		A.SetZero();
		b.SetZero();
		s.SetZero();

		if(m_w2s.Empty())
		{
			ENFT_SSE::__m128 &o = m_work[9], &x = m_work[10], &y = m_work[11], &o_z = m_work[12], &o_z2 = m_work[13], &x_z = m_work[14], &y_z = m_work[15], &x2py2_z2 = m_work[16];
			ENFT_SSE::__m128 &u_z = m_work[17], &u_z2 = m_work[18], &u2_z2 = m_work[19], &v_z = m_work[20], &v_z2 = m_work[21], &v2_z2 = m_work[22], &uv_z2 = m_work[23];
			ENFT_SSE::__m128 &ex = m_work[24], &ey = m_work[25], &t = m_work[26];
			o = _mm_set1_ps(1.0f);
			const ushort nPacks = GetPacksNumber(), nRems = GetRemindersNumber();
			for(ushort iu1 = 0, iv1 = 1, iu2 = 2, iv2 = 3; iu1 < nPacks; iu1 += 4, iv1 += 4, iu2 += 4, iv2 += 4)
			{
				const ENFT_SSE::__m128 &u = GetPack(iu1), &v = GetPack(iv1);
				x = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[0], u), ENFT_SSE::_mm_mul_ps(h[1], v)), h[2]);
				y = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[3], u), ENFT_SSE::_mm_mul_ps(h[4], v)), h[5]);
				o_z = ENFT_SSE::_mm_div_ps(o, ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[6], u), ENFT_SSE::_mm_mul_ps(h[7], v)), h[8]));
				o_z2 = ENFT_SSE::_mm_mul_ps(o_z, o_z);
				x2py2_z2 = ENFT_SSE::_mm_mul_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(x, x), ENFT_SSE::_mm_mul_ps(y, y)), o_z2);
				x_z = ENFT_SSE::_mm_mul_ps(x, o_z);	u_z = ENFT_SSE::_mm_mul_ps(u, o_z);	u_z2 = ENFT_SSE::_mm_mul_ps(u_z, o_z);	u2_z2 = ENFT_SSE::_mm_mul_ps(u_z, u_z);
				y_z = ENFT_SSE::_mm_mul_ps(y, o_z);	v_z = ENFT_SSE::_mm_mul_ps(v, o_z);	v_z2 = ENFT_SSE::_mm_mul_ps(v_z, o_z);	v2_z2 = ENFT_SSE::_mm_mul_ps(v_z, v_z);
				uv_z2 = ENFT_SSE::_mm_mul_ps(u_z, v_z);

				A.M00() = ENFT_SSE::SSE::Sum0123(u2_z2) + A.M00();
				A.M01() = ENFT_SSE::SSE::Sum0123(uv_z2) + A.M01();
				A.M02() = ENFT_SSE::SSE::Sum0123( u_z2) + A.M02();
				A.M11() = ENFT_SSE::SSE::Sum0123(v2_z2) + A.M11();
				A.M12() = ENFT_SSE::SSE::Sum0123( v_z2) + A.M12();
				A.M22() = ENFT_SSE::SSE::Sum0123( o_z2) + A.M22();
				A.M06() = A.M06() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, u2_z2));
				A.M07() = A.M07() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, uv_z2));
				A.M17() = A.M17() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, v2_z2));
				A.M26() = A.M26() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z,  u_z2));
				A.M27() = A.M27() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z,  v_z2));
				A.M36() = A.M36() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, u2_z2));
				A.M37() = A.M37() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, uv_z2));
				A.M47() = A.M47() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, v2_z2));
				A.M56() = A.M56() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z,  u_z2));
				A.M57() = A.M57() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z,  v_z2));
				A.M66() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, u2_z2)) + A.M66();
				A.M67() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, uv_z2)) + A.M67();
				A.M77() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, v2_z2)) + A.M77();

				ex = ENFT_SSE::_mm_sub_ps(GetPack(iu2), x_z);
				ey = ENFT_SSE::_mm_sub_ps(GetPack(iv2), y_z);
				t = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(x_z, ex), ENFT_SSE::_mm_mul_ps(y_z, ey));
				b.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ex, u_z)) + b.v0();
				b.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ex, v_z)) + b.v1();
				b.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ex, o_z)) + b.v2();
				b.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ey, u_z)) + b.v3();
				b.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ey, v_z)) + b.v4();
				b.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ey, o_z)) + b.v5();
				b.v6() = b.v6() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(t, u_z));
				b.v7() = b.v7() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(t, v_z));

				s.v0() = ENFT_SSE::SSE::Sum0123(u2_z2) + s.v0();
				s.v1() = ENFT_SSE::SSE::Sum0123(v2_z2) + s.v1();
				s.v2() = ENFT_SSE::SSE::Sum0123( o_z2) + s.v2();
				s.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, u2_z2)) + s.v6();
				s.v7() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, v2_z2)) + s.v7();
			}
			for(ushort iRem = 0; iRem < nRems; ++iRem)
			{
				float *work = (float *) m_work;
				float &x = work[0], &y = work[1], &o_z = work[2], &o_z2 = work[3], &x_z = work[4], &y_z = work[5], &x2py2_z2 = work[6];
				float &u_z = work[7], &u_z2 = work[8], &u2_z2 = work[9], &v_z = work[10], &v_z2 = work[11], &v2_z2 = work[12], &uv_z2 = work[13];
				float &ex = work[14], &ey = work[15], &t = work[16];

				const float &u = GetReminder1(iRem).x(), &v = GetReminder1(iRem).y();
				x = H.M00() * u + H.M01() * v + H.M02();
				y = H.M10() * u + H.M11() * v + H.M12();
				o_z = 1 / (H.M20() * u + H.M21() * v + H.M22());
				o_z2 = o_z * o_z;
				x2py2_z2 = (x * x + y * y) * o_z2;
				x_z = x * o_z;	u_z = u * o_z;	u_z2 = u_z * o_z;	u2_z2 = u_z * u_z;
				y_z = y * o_z;	v_z = v * o_z;	v_z2 = v_z * o_z;	v2_z2 = v_z * v_z;
				uv_z2 = u_z * v_z;

				A.M00() = u2_z2 + A.M00();
				A.M01() = uv_z2 + A.M01();
				A.M02() =  u_z2 + A.M02();
				A.M11() = v2_z2 + A.M11();
				A.M12() =  v_z2 + A.M12();
				A.M22() =  o_z2 + A.M22();
				A.M06() = A.M06() - x_z * u2_z2;
				A.M07() = A.M07() - x_z * uv_z2;
				A.M17() = A.M17() - x_z * v2_z2;
				A.M26() = A.M26() - x_z *  u_z2;
				A.M27() = A.M27() - x_z *  v_z2;
				A.M36() = A.M36() - y_z * u2_z2;
				A.M37() = A.M37() - y_z * uv_z2;
				A.M47() = A.M47() - y_z * v2_z2;
				A.M56() = A.M56() - y_z *  u_z2;
				A.M57() = A.M57() - y_z *  v_z2;
				A.M66() = x2py2_z2 * u2_z2 + A.M66();
				A.M67() = x2py2_z2 * uv_z2 + A.M67();
				A.M77() = x2py2_z2 * v2_z2 + A.M77();

				ex = GetReminder2(iRem).x() - x_z;
				ey = GetReminder2(iRem).y() - y_z;
				t = x_z * ex + y_z * ey;
				b.v0() = ex * u_z + b.v0();
				b.v1() = ex * v_z + b.v1();
				b.v2() = ex * o_z + b.v2();
				b.v3() = ey * u_z + b.v3();
				b.v4() = ey * v_z + b.v4();
				b.v5() = ey * o_z + b.v5();
				b.v6() = b.v6() - t * u_z;
				b.v7() = b.v7() - t * v_z;

				s.v0() = u2_z2 + s.v0();
				s.v1() = v2_z2 + s.v1();
				s.v2() =  o_z2 + s.v2();
				s.v6() = x2py2_z2 * u2_z2 + s.v6();
				s.v7() = x2py2_z2 * v2_z2 + s.v7();
			}
		}
		else
		{
			ENFT_SSE::__m128 &o = m_work[9], &x = m_work[10], &y = m_work[11], &o_z = m_work[12], &o_z2 = m_work[13], &x_z = m_work[14], &y_z = m_work[15], &x2py2_z2 = m_work[16];
			ENFT_SSE::__m128 &w_z = m_work[17], &w2_z2 = m_work[18];
			ENFT_SSE::__m128 &wu_z = m_work[19], &w2u_z2 = m_work[20], &w2u2_z2 = m_work[21], &wv_z = m_work[22], &w2v_z2 = m_work[23], &w2v2_z2 = m_work[24], &w2uv_z2 = m_work[25];
			ENFT_SSE::__m128 &wex = m_work[26], &wey = m_work[27], &t = m_work[28];
			o = _mm_set1_ps(1.0f);
			const ENFT_SSE::__m128 *w = (ENFT_SSE::__m128 *) m_ws.Data();
			const ushort nPacks = GetPacksNumber(), nRems = GetRemindersNumber();
			for(ushort iu1 = 0, iv1 = 1, iu2 = 2, iv2 = 3; iu1 < nPacks; iu1 += 4, iv1 += 4, iu2 += 4, iv2 += 4, ++w)
			{
				const ENFT_SSE::__m128 &u = GetPack(iu1), &v = GetPack(iv1);
				x = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[0], u), ENFT_SSE::_mm_mul_ps(h[1], v)), h[2]);
				y = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[3], u), ENFT_SSE::_mm_mul_ps(h[4], v)), h[5]);
				o_z = ENFT_SSE::_mm_div_ps(o, ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[6], u), ENFT_SSE::_mm_mul_ps(h[7], v)), h[8]));
				o_z2 = ENFT_SSE::_mm_mul_ps(o_z, o_z);
				x2py2_z2 = ENFT_SSE::_mm_mul_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(x, x), ENFT_SSE::_mm_mul_ps(y, y)), o_z2);
				w_z = ENFT_SSE::_mm_mul_ps(*w, o_z);
				w2_z2 = ENFT_SSE::_mm_mul_ps(w_z, w_z);
				x_z = ENFT_SSE::_mm_mul_ps(x, o_z);	wu_z = ENFT_SSE::_mm_mul_ps(u, w_z);	w2u_z2 = ENFT_SSE::_mm_mul_ps(wu_z, w_z);	w2u2_z2 = ENFT_SSE::_mm_mul_ps(wu_z, wu_z);
				y_z = ENFT_SSE::_mm_mul_ps(y, o_z);	wv_z = ENFT_SSE::_mm_mul_ps(v, w_z);	w2v_z2 = ENFT_SSE::_mm_mul_ps(wu_z, w_z);	w2v2_z2 = ENFT_SSE::_mm_mul_ps(wv_z, wv_z);
				w2uv_z2 = ENFT_SSE::_mm_mul_ps(wu_z, wv_z);

				A.M00() = ENFT_SSE::SSE::Sum0123(w2u2_z2) + A.M00();
				A.M01() = ENFT_SSE::SSE::Sum0123(w2uv_z2) + A.M01();
				A.M02() = ENFT_SSE::SSE::Sum0123( w2u_z2) + A.M02();
				A.M11() = ENFT_SSE::SSE::Sum0123(w2v2_z2) + A.M11();
				A.M12() = ENFT_SSE::SSE::Sum0123( w2v_z2) + A.M12();
				A.M22() = ENFT_SSE::SSE::Sum0123(  w2_z2) + A.M22();
				A.M06() = A.M06() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, w2u2_z2));
				A.M07() = A.M07() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, w2uv_z2));
				A.M17() = A.M17() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, w2v2_z2));
				A.M26() = A.M26() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z,  w2u_z2));
				A.M27() = A.M27() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z,  w2v_z2));
				A.M36() = A.M36() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, w2u2_z2));
				A.M37() = A.M37() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, w2uv_z2));
				A.M47() = A.M47() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, w2v2_z2));
				A.M56() = A.M56() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z,  w2u_z2));
				A.M57() = A.M57() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z,  w2v_z2));
				A.M66() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, w2u2_z2)) + A.M66();
				A.M67() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, w2uv_z2)) + A.M67();
				A.M77() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, w2v2_z2)) + A.M77();

				wex = ENFT_SSE::_mm_mul_ps(*w, ENFT_SSE::_mm_sub_ps(GetPack(iu2), x_z));
				wey = ENFT_SSE::_mm_mul_ps(*w, ENFT_SSE::_mm_sub_ps(GetPack(iv2), y_z));
				t = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(x_z, wex), ENFT_SSE::_mm_mul_ps(y_z, wey));
				b.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wex, wu_z)) + b.v0();
				b.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wex, wv_z)) + b.v1();
				b.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wex,  w_z)) + b.v2();
				b.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wey, wu_z)) + b.v3();
				b.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wey, wv_z)) + b.v4();
				b.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wey,  w_z)) + b.v5();
				b.v6() = b.v6() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(t, wu_z));
				b.v7() = b.v7() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(t, wv_z));

				s.v0() = ENFT_SSE::SSE::Sum0123(w2u2_z2) + s.v0();
				s.v1() = ENFT_SSE::SSE::Sum0123(w2v2_z2) + s.v1();
				s.v2() = ENFT_SSE::SSE::Sum0123(  w2_z2) + s.v2();
				s.v6() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, w2u2_z2)) + s.v6();
				s.v7() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, w2v2_z2)) + s.v7();
			}
			for(ushort iRem = 0; iRem < nRems; ++iRem)
			{
				float *work = (float *) m_work;
				float &x = work[0], &y = work[1], &o_z = work[2], &o_z2 = work[3], &x_z = work[4], &y_z = work[5], &x2py2_z2 = work[6];
				float &w_z = work[4], &w2_z2 = work[5];
				float &wu_z = work[7], &w2u_z2 = work[8], &w2u2_z2 = work[9], &wv_z = work[10], &w2v_z2 = work[11], &w2v2_z2 = work[12], &w2uv_z2 = work[13];
				float &wex = work[14], &wey = work[15], &t = work[16];

				const float &u = GetReminder1(iRem).x(), &v = GetReminder1(iRem).y();
				x = H.M00() * u + H.M01() * v + H.M02();
				y = H.M10() * u + H.M11() * v + H.M12();
				o_z = 1 / (H.M20() * u + H.M21() * v + H.M22());
				o_z2 = o_z * o_z;
				x2py2_z2 = (x * x + y * y) * o_z2;
				w_z = w->m128_f32[iRem] * o_z;
				w2_z2 = w_z * w_z;
				x_z = x * o_z;		wu_z = u * w_z;		w2u_z2 = wu_z * w_z;	w2u2_z2 = wu_z * wu_z;
				y_z = y * o_z;		wv_z = v * w_z;		w2v_z2 = wv_z * w_z;	w2v2_z2 = wv_z * wv_z;
				w2uv_z2 = wu_z * wv_z;

				A.M00() = w2u2_z2 + A.M00();
				A.M01() = w2uv_z2 + A.M01();
				A.M02() =  w2u_z2 + A.M02();
				A.M11() = w2v2_z2 + A.M11();
				A.M12() =  w2v_z2 + A.M12();
				A.M22() =   w2_z2 + A.M22();
				A.M06() = A.M06() - x_z * w2u2_z2;
				A.M07() = A.M07() - x_z * w2uv_z2;
				A.M17() = A.M17() - x_z * w2v2_z2;
				A.M26() = A.M26() - x_z *  w2u_z2;
				A.M27() = A.M27() - x_z *  w2v_z2;
				A.M36() = A.M36() - y_z * w2u2_z2;
				A.M37() = A.M37() - y_z * w2uv_z2;
				A.M47() = A.M47() - y_z * w2v2_z2;
				A.M56() = A.M56() - y_z *  w2u_z2;
				A.M57() = A.M57() - y_z *  w2v_z2;
				A.M66() = x2py2_z2 * w2u2_z2 + A.M66();
				A.M67() = x2py2_z2 * w2uv_z2 + A.M67();
				A.M77() = x2py2_z2 * w2v2_z2 + A.M77();

				wex = w->m128_f32[iRem] * (GetReminder2(iRem).x() - x_z);
				wey = w->m128_f32[iRem] * (GetReminder2(iRem).y() - y_z);
				t = x_z * wex + y_z * wey;
				b.v0() = wex * wu_z + b.v0();
				b.v1() = wex * wv_z + b.v1();
				b.v2() = wex *  w_z + b.v2();
				b.v3() = wey * wu_z + b.v3();
				b.v4() = wey * wv_z + b.v4();
				b.v5() = wey *  w_z + b.v5();
				b.v6() = b.v6() - t * wu_z;
				b.v7() = b.v7() - t * wv_z;

				s.v0() = w2u2_z2 + s.v0();
				s.v1() = w2v2_z2 + s.v1();
				s.v2() =   w2_z2 + s.v2();
				s.v6() = x2py2_z2 * w2u2_z2 + s.v6();
				s.v7() = x2py2_z2 * w2v2_z2 + s.v7();
			}
		}
		A.M33() = A.M00();		A.M34() = A.M01();		A.M35() = A.M02();
								A.M44() = A.M11();		A.M45() = A.M12();
														A.M55() = A.M22();
		A.M16() = A.M07();
		A.M46() = A.M37();
		A.SetLowerFromUpper();

		s.v3() = s.v0();
		s.v4() = s.v1();
		s.v5() = s.v2();
		LA::MakeReciprocal(s);
		LA::MakeSquareRoot(s);
		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}
	virtual void ConstructNormalEquation(const Homography &H, const LA::AlignedVector8f &s, LA::AlignedMatrix8f &A, LA::AlignedVector8f &b)
	{
		ENFT_SSE::__m128 *h = m_work;
		H.GetSSE(h);
		A.SetZero();
		b.SetZero();

		if(m_w2s.Empty())
		{
			ENFT_SSE::__m128 &o = m_work[9], &x = m_work[10], &y = m_work[11], &o_z = m_work[12], &o_z2 = m_work[13], &x_z = m_work[14], &y_z = m_work[15], &x2py2_z2 = m_work[16];
			ENFT_SSE::__m128 &u_z = m_work[17], &u_z2 = m_work[18], &u2_z2 = m_work[19], &v_z = m_work[20], &v_z2 = m_work[21], &v2_z2 = m_work[22], &uv_z2 = m_work[23];
			ENFT_SSE::__m128 &ex = m_work[24], &ey = m_work[25], &t = m_work[26];
			o = _mm_set1_ps(1.0f);
			const ushort nPacks = GetPacksNumber(), nRems = GetRemindersNumber();
			for(ushort iu1 = 0, iv1 = 1, iu2 = 2, iv2 = 3; iu1 < nPacks; iu1 += 4, iv1 += 4, iu2 += 4, iv2 += 4)
			{
				const ENFT_SSE::__m128 &u = GetPack(iu1), &v = GetPack(iv1);
				x = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[0], u), ENFT_SSE::_mm_mul_ps(h[1], v)), h[2]);
				y = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[3], u), ENFT_SSE::_mm_mul_ps(h[4], v)), h[5]);
				o_z = ENFT_SSE::_mm_div_ps(o, ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[6], u), ENFT_SSE::_mm_mul_ps(h[7], v)), h[8]));
				o_z2 = ENFT_SSE::_mm_mul_ps(o_z, o_z);
				x2py2_z2 = ENFT_SSE::_mm_mul_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(x, x), ENFT_SSE::_mm_mul_ps(y, y)), o_z2);
				x_z = ENFT_SSE::_mm_mul_ps(x, o_z);	u_z = ENFT_SSE::_mm_mul_ps(u, o_z);	u_z2 = ENFT_SSE::_mm_mul_ps(u_z, o_z);	u2_z2 = ENFT_SSE::_mm_mul_ps(u_z, u_z);
				y_z = ENFT_SSE::_mm_mul_ps(y, o_z);	v_z = ENFT_SSE::_mm_mul_ps(v, o_z);	v_z2 = ENFT_SSE::_mm_mul_ps(v_z, o_z);	v2_z2 = ENFT_SSE::_mm_mul_ps(v_z, v_z);
				uv_z2 = ENFT_SSE::_mm_mul_ps(u_z, v_z);

				A.M00() = ENFT_SSE::SSE::Sum0123(u2_z2) + A.M00();
				A.M01() = ENFT_SSE::SSE::Sum0123(uv_z2) + A.M01();
				A.M02() = ENFT_SSE::SSE::Sum0123( u_z2) + A.M02();
				A.M11() = ENFT_SSE::SSE::Sum0123(v2_z2) + A.M11();
				A.M12() = ENFT_SSE::SSE::Sum0123( v_z2) + A.M12();
				A.M22() = ENFT_SSE::SSE::Sum0123( o_z2) + A.M22();
				A.M06() = A.M06() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, u2_z2));
				A.M07() = A.M07() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, uv_z2));
				A.M17() = A.M17() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, v2_z2));
				A.M26() = A.M26() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z,  u_z2));
				A.M27() = A.M27() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z,  v_z2));
				A.M36() = A.M36() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, u2_z2));
				A.M37() = A.M37() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, uv_z2));
				A.M47() = A.M47() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, v2_z2));
				A.M56() = A.M56() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z,  u_z2));
				A.M57() = A.M57() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z,  v_z2));
				A.M66() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, u2_z2)) + A.M66();
				A.M67() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, uv_z2)) + A.M67();
				A.M77() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, v2_z2)) + A.M77();

				ex = ENFT_SSE::_mm_sub_ps(GetPack(iu2), x_z);
				ey = ENFT_SSE::_mm_sub_ps(GetPack(iv2), y_z);
				t = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(x_z, ex), ENFT_SSE::_mm_mul_ps(y_z, ey));
				b.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ex, u_z)) + b.v0();
				b.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ex, v_z)) + b.v1();
				b.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ex, o_z)) + b.v2();
				b.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ey, u_z)) + b.v3();
				b.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ey, v_z)) + b.v4();
				b.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(ey, o_z)) + b.v5();
				b.v6() = b.v6() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(t, u_z));
				b.v7() = b.v7() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(t, v_z));
			}
			for(ushort iRem = 0; iRem < nRems; ++iRem)
			{
				float *work = (float *) m_work;
				float &x = work[0], &y = work[1], &o_z = work[2], &o_z2 = work[3], &x_z = work[4], &y_z = work[5], &x2py2_z2 = work[6];
				float &u_z = work[7], &u_z2 = work[8], &u2_z2 = work[9], &v_z = work[10], &v_z2 = work[11], &v2_z2 = work[12], &uv_z2 = work[13];
				float &ex = work[14], &ey = work[15], &t = work[16];

				const float &u = GetReminder1(iRem).x(), &v = GetReminder1(iRem).y();
				x = H.M00() * u + H.M01() * v + H.M02();
				y = H.M10() * u + H.M11() * v + H.M12();
				o_z = 1 / (H.M20() * u + H.M21() * v + H.M22());
				o_z2 = o_z * o_z;
				x2py2_z2 = (x * x + y * y) * o_z2;
				x_z = x * o_z;	u_z = u * o_z;	u_z2 = u_z * o_z;	u2_z2 = u_z * u_z;
				y_z = y * o_z;	v_z = v * o_z;	v_z2 = v_z * o_z;	v2_z2 = v_z * v_z;
				uv_z2 = u_z * v_z;

				A.M00() = u2_z2 + A.M00();
				A.M01() = uv_z2 + A.M01();
				A.M02() =  u_z2 + A.M02();
				A.M11() = v2_z2 + A.M11();
				A.M12() =  v_z2 + A.M12();
				A.M22() =  o_z2 + A.M22();
				A.M06() = A.M06() - x_z * u2_z2;
				A.M07() = A.M07() - x_z * uv_z2;
				A.M17() = A.M17() - x_z * v2_z2;
				A.M26() = A.M26() - x_z *  u_z2;
				A.M27() = A.M27() - x_z *  v_z2;
				A.M36() = A.M36() - y_z * u2_z2;
				A.M37() = A.M37() - y_z * uv_z2;
				A.M47() = A.M47() - y_z * v2_z2;
				A.M56() = A.M56() - y_z *  u_z2;
				A.M57() = A.M57() - y_z *  v_z2;
				A.M66() = x2py2_z2 * u2_z2 + A.M66();
				A.M67() = x2py2_z2 * uv_z2 + A.M67();
				A.M77() = x2py2_z2 * v2_z2 + A.M77();

				ex = GetReminder2(iRem).x() - x_z;
				ey = GetReminder2(iRem).y() - y_z;
				t = x_z * ex + y_z * ey;
				b.v0() = ex * u_z + b.v0();
				b.v1() = ex * v_z + b.v1();
				b.v2() = ex * o_z + b.v2();
				b.v3() = ey * u_z + b.v3();
				b.v4() = ey * v_z + b.v4();
				b.v5() = ey * o_z + b.v5();
				b.v6() = b.v6() - t * u_z;
				b.v7() = b.v7() - t * v_z;
			}
		}
		else
		{
			ENFT_SSE::__m128 &o = m_work[9], &x = m_work[10], &y = m_work[11], &o_z = m_work[12], &o_z2 = m_work[13], &x_z = m_work[14], &y_z = m_work[15], &x2py2_z2 = m_work[16];
			ENFT_SSE::__m128 &w_z = m_work[17], &w2_z2 = m_work[18];
			ENFT_SSE::__m128 &wu_z = m_work[19], &w2u_z2 = m_work[20], &w2u2_z2 = m_work[21], &wv_z = m_work[22], &w2v_z2 = m_work[23], &w2v2_z2 = m_work[24], &w2uv_z2 = m_work[25];
			ENFT_SSE::__m128 &wex = m_work[26], &wey = m_work[27], &t = m_work[28];
			o = _mm_set1_ps(1.0f);
			const ENFT_SSE::__m128 *w = (ENFT_SSE::__m128 *) m_ws.Data();
			const ushort nPacks = GetPacksNumber(), nRems = GetRemindersNumber();
			for(ushort iu1 = 0, iv1 = 1, iu2 = 2, iv2 = 3; iu1 < nPacks; iu1 += 4, iv1 += 4, iu2 += 4, iv2 += 4, ++w)
			{
				const ENFT_SSE::__m128 &u = GetPack(iu1), &v = GetPack(iv1);
				x = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[0], u), ENFT_SSE::_mm_mul_ps(h[1], v)), h[2]);
				y = ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[3], u), ENFT_SSE::_mm_mul_ps(h[4], v)), h[5]);
				o_z = ENFT_SSE::_mm_div_ps(o, ENFT_SSE::_mm_add_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(h[6], u), ENFT_SSE::_mm_mul_ps(h[7], v)), h[8]));
				o_z2 = ENFT_SSE::_mm_mul_ps(o_z, o_z);
				x2py2_z2 = ENFT_SSE::_mm_mul_ps(_mm_add_ps(ENFT_SSE::_mm_mul_ps(x, x), ENFT_SSE::_mm_mul_ps(y, y)), o_z2);
				w_z = ENFT_SSE::_mm_mul_ps(*w, o_z);
				w2_z2 = ENFT_SSE::_mm_mul_ps(w_z, w_z);
				x_z = ENFT_SSE::_mm_mul_ps(x, o_z);	wu_z = ENFT_SSE::_mm_mul_ps(u, w_z);	w2u_z2 = ENFT_SSE::_mm_mul_ps(wu_z, w_z);	w2u2_z2 = ENFT_SSE::_mm_mul_ps(wu_z, wu_z);
				y_z = ENFT_SSE::_mm_mul_ps(y, o_z);	wv_z = ENFT_SSE::_mm_mul_ps(v, w_z);	w2v_z2 = ENFT_SSE::_mm_mul_ps(wu_z, w_z);	w2v2_z2 = ENFT_SSE::_mm_mul_ps(wv_z, wv_z);
				w2uv_z2 = ENFT_SSE::_mm_mul_ps(wu_z, wv_z);

				A.M00() = ENFT_SSE::SSE::Sum0123(w2u2_z2) + A.M00();
				A.M01() = ENFT_SSE::SSE::Sum0123(w2uv_z2) + A.M01();
				A.M02() = ENFT_SSE::SSE::Sum0123( w2u_z2) + A.M02();
				A.M11() = ENFT_SSE::SSE::Sum0123(w2v2_z2) + A.M11();
				A.M12() = ENFT_SSE::SSE::Sum0123( w2v_z2) + A.M12();
				A.M22() = ENFT_SSE::SSE::Sum0123(  w2_z2) + A.M22();
				A.M06() = A.M06() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, w2u2_z2));
				A.M07() = A.M07() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, w2uv_z2));
				A.M17() = A.M17() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z, w2v2_z2));
				A.M26() = A.M26() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z,  w2u_z2));
				A.M27() = A.M27() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x_z,  w2v_z2));
				A.M36() = A.M36() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, w2u2_z2));
				A.M37() = A.M37() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, w2uv_z2));
				A.M47() = A.M47() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z, w2v2_z2));
				A.M56() = A.M56() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z,  w2u_z2));
				A.M57() = A.M57() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(y_z,  w2v_z2));
				A.M66() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, w2u2_z2)) + A.M66();
				A.M67() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, w2uv_z2)) + A.M67();
				A.M77() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(x2py2_z2, w2v2_z2)) + A.M77();

				wex = ENFT_SSE::_mm_mul_ps(*w, ENFT_SSE::_mm_sub_ps(GetPack(iu2), x_z));
				wey = ENFT_SSE::_mm_mul_ps(*w, ENFT_SSE::_mm_sub_ps(GetPack(iv2), y_z));
				t = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(x_z, wex), ENFT_SSE::_mm_mul_ps(y_z, wey));
				b.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wex, wu_z)) + b.v0();
				b.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wex, wv_z)) + b.v1();
				b.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wex,  w_z)) + b.v2();
				b.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wey, wu_z)) + b.v3();
				b.v4() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wey, wv_z)) + b.v4();
				b.v5() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(wey,  w_z)) + b.v5();
				b.v6() = b.v6() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(t, wu_z));
				b.v7() = b.v7() - ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(t, wv_z));
			}
			for(ushort iRem = 0; iRem < nRems; ++iRem)
			{
				float *work = (float *) m_work;
				float &x = work[0], &y = work[1], &o_z = work[2], &o_z2 = work[3], &x_z = work[4], &y_z = work[5], &x2py2_z2 = work[6];
				float &w_z = work[4], &w2_z2 = work[5];
				float &wu_z = work[7], &w2u_z2 = work[8], &w2u2_z2 = work[9], &wv_z = work[10], &w2v_z2 = work[11], &w2v2_z2 = work[12], &w2uv_z2 = work[13];
				float &wex = work[14], &wey = work[15], &t = work[16];

				const float &u = GetReminder1(iRem).x(), &v = GetReminder1(iRem).y();
				x = H.M00() * u + H.M01() * v + H.M02();
				y = H.M10() * u + H.M11() * v + H.M12();
				o_z = 1 / (H.M20() * u + H.M21() * v + H.M22());
				o_z2 = o_z * o_z;
				x2py2_z2 = (x * x + y * y) * o_z2;
				w_z = w->m128_f32[iRem] * o_z;
				w2_z2 = w_z * w_z;
				x_z = x * o_z;		wu_z = u * w_z;		w2u_z2 = wu_z * w_z;	w2u2_z2 = wu_z * wu_z;
				y_z = y * o_z;		wv_z = v * w_z;		w2v_z2 = wv_z * w_z;	w2v2_z2 = wv_z * wv_z;
				w2uv_z2 = wu_z * wv_z;

				A.M00() = w2u2_z2 + A.M00();
				A.M01() = w2uv_z2 + A.M01();
				A.M02() =  w2u_z2 + A.M02();
				A.M11() = w2v2_z2 + A.M11();
				A.M12() =  w2v_z2 + A.M12();
				A.M22() =   w2_z2 + A.M22();
				A.M06() = A.M06() - x_z * w2u2_z2;
				A.M07() = A.M07() - x_z * w2uv_z2;
				A.M17() = A.M17() - x_z * w2v2_z2;
				A.M26() = A.M26() - x_z *  w2u_z2;
				A.M27() = A.M27() - x_z *  w2v_z2;
				A.M36() = A.M36() - y_z * w2u2_z2;
				A.M37() = A.M37() - y_z * w2uv_z2;
				A.M47() = A.M47() - y_z * w2v2_z2;
				A.M56() = A.M56() - y_z *  w2u_z2;
				A.M57() = A.M57() - y_z *  w2v_z2;
				A.M66() = x2py2_z2 * w2u2_z2 + A.M66();
				A.M67() = x2py2_z2 * w2uv_z2 + A.M67();
				A.M77() = x2py2_z2 * w2v2_z2 + A.M77();

				wex = w->m128_f32[iRem] * (GetReminder2(iRem).x() - x_z);
				wey = w->m128_f32[iRem] * (GetReminder2(iRem).y() - y_z);
				t = x_z * wex + y_z * wey;
				b.v0() = wex * wu_z + b.v0();
				b.v1() = wex * wv_z + b.v1();
				b.v2() = wex *  w_z + b.v2();
				b.v3() = wey * wu_z + b.v3();
				b.v4() = wey * wv_z + b.v4();
				b.v5() = wey *  w_z + b.v5();
				b.v6() = b.v6() - t * wu_z;
				b.v7() = b.v7() - t * wv_z;
			}
		}
		A.M33() = A.M00();		A.M34() = A.M01();		A.M35() = A.M02();
								A.M44() = A.M11();		A.M45() = A.M12();
														A.M55() = A.M22();
		A.M16() = A.M07();
		A.M46() = A.M37();
		A.SetLowerFromUpper();

		LA::ssTA(s, A, m_work[0]);
		LA::sA(s, b);
	}
	virtual void UpdateModel(const LA::AlignedVector8f &s, const LA::AlignedVector8f &x, const Homography &Hold, Homography &Hnew)
	{
		m_work[0] = ENFT_SSE::_mm_mul_ps(s.v0123(), x.v0123());
		Hnew.M00() = m_work[0].m128_f32[0] + Hold.M00();
		Hnew.M01() = m_work[0].m128_f32[1] + Hold.M01();
		Hnew.M02() = m_work[0].m128_f32[2] + Hold.M02();
		Hnew.M10() = m_work[0].m128_f32[3] + Hold.M10();
		m_work[0] = ENFT_SSE::_mm_mul_ps(s.v4567(), x.v4567());
		Hnew.M11() = m_work[0].m128_f32[0] + Hold.M11();
		Hnew.M12() = m_work[0].m128_f32[1] + Hold.M12();
		Hnew.M20() = m_work[0].m128_f32[2] + Hold.M20();
		Hnew.M21() = m_work[0].m128_f32[3] + Hold.M21();
	}

protected:

	ENFT_SSE::__m128 m_work[29];
	AlignedVector<float> m_ws, m_w2s;
	bool m_normalized;

};

#endif