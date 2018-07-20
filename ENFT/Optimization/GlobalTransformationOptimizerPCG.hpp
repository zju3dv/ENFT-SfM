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

template<GTO_TEMPLATE_PARAMETER>
inline uint GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::RunPCG()
{
	Compute_r0_b();
	Compute_z0p0_Mr0();

	double rTzPre, rTzCur = Compute_r0Tz0();
	double rTzMin = m_pcgStopRTZRatioMin * fabs(rTzCur), rTzMax = m_pcgStopRTZRatioMax * fabs(rTzCur);
	float a = float(rTzCur / Compute_pTAp());
	//printf("%f, %f\n", rTzCur, a);
	if(!std::isfinite(a))
		return rTzCur < rTzMin && rTzCur > -rTzMin ? 1 : 0;

	//const double rTzInit = rTzCur;

	Compute_x1_a0p0(a);
	Compute_r1_r0ma0Ap0(a);

	uint k = 1;
	bool scc;
	while(true)
	{
		Compute_z_Mr();

		rTzPre = rTzCur;
		rTzCur = Compute_rTz();
		if(rTzCur <= rTzMin && rTzCur >= -rTzMin && k >= m_pcgMinNumIters)
		{
			scc = true;
			break;
		}
		else if(rTzCur >= rTzMax || rTzCur <= -rTzMax)
		{
			scc = true;
			//scc = false;
			break;
		}

		float b = float(rTzCur / rTzPre);
		Update_p_zpbp(b);

		a = float(rTzCur / Compute_pTAp());

		//printf("%f, %f, %f\n", rTzCur, a, b);

		if(!std::isfinite(a))
		{
			//scc = false;
			scc = rTzCur < rTzMin && rTzCur > -rTzMin;
			break;
		}

		Update_x_xpap(a);
		if(++k >= m_pcgMaxNumIters)
		{
			scc = true;
			break;
		}

		Update_r_rmaAp(a);
	}

	//printf("\r%f\n", rTzCur / rTzInit);

	//printf("%d\n", k);
	if(!scc)
		k = 0;

	//ComputeAx<GTO_STAGE_JTE>(m_xs, m_Aps);
	//LA::AmB<TransformationParameter, GTO_STAGE_JTE>(m_Aps, m_bs, m_ps, 1);
	return k;
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Compute_r0_b()
{
	//m_rs.CopyFrom(m_bs);
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Compute_z0p0_Mr0()
{
	ComputeMx<GTO_STAGE_P>(m_rs, m_ps);
}

template<GTO_TEMPLATE_PARAMETER>
inline double GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Compute_r0Tz0()
{
	return Dot(m_rs, m_ps);
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Compute_x1_a0p0(const float &a0)
{
	sA<GTO_STAGE_X>(a0, m_ps, m_xs);
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Compute_r1_r0ma0Ap0(const float &a0)
{
	sApB<GTO_STAGE_R>(-a0, m_Aps, m_rs, m_rs);
}

template<GTO_TEMPLATE_PARAMETER>
double GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Compute_pTAp()
{
	ComputeAx<GTO_STAGE_GENERAL>(m_ps, m_Aps);
	return Dot(m_ps, m_Aps);
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Compute_z_Mr()
{
	ComputeMx<GTO_STAGE_Z>(m_rs, m_zs);
}

template<GTO_TEMPLATE_PARAMETER>
inline double GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Compute_rTz()
{
	return Dot(m_rs, m_zs);
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Update_p_zpbp(const float &b)
{
	sApB<GTO_STAGE_P>(b, m_ps, m_zs, m_ps);
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Update_x_xpap(const float &a)
{
	sApB<GTO_STAGE_X>(a, m_ps, m_xs, m_xs);
}

template<GTO_TEMPLATE_PARAMETER>
inline void GlobalTransformationOptimizerTemplate<GTO_TEMPLATE_ARGUMENT>::Update_r_rmaAp(const float &a)
{
	sApB<GTO_STAGE_R>(-a, m_Aps, m_rs, m_rs);
}