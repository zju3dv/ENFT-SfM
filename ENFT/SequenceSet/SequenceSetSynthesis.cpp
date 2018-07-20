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
#include "SequenceSet.h"
#include "Utility/Random.h"

void SequenceSet::AddNoiseToTransformations(const SequenceIndex nSeqsFix, const float noisePercentRigid, const float maxScale, 
											AlignedVector<SimilarityTransformation3D> &Ss)
{
	Point3D sum;
	sum.SetZero();
	const TrackIndex nTrksCmn = GetCommonTracksNumber();
	SequenceIndex iSeq;
	TrackIndex iTrkCmn, iTrkIdv;
	for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		m_mapCmnTrkToIdvTrk[iTrkCmn][0].Get(iSeq, iTrkIdv);
		LA::ApB(GetSequence(iSeq).GetPoint(iTrkIdv), sum, sum);
	}
	Point3D mean;
	mean.XYZx() = ENFT_SSE::_mm_mul_ps(_mm_set1_ps(1.0f / nTrksCmn), sum.XYZx());

	Point3D dX;
	std::vector<float> distSqs(nTrksCmn);
	for(iTrkCmn = 0; iTrkCmn < nTrksCmn; ++iTrkCmn)
	{
		m_mapCmnTrkToIdvTrk[iTrkCmn][0].Get(iSeq, iTrkIdv);
		LA::AmB(GetSequence(iSeq).GetPoint(iTrkIdv), mean, dX);
		distSqs[iTrkCmn] = ENFT_SSE::SSE::Sum012(ENFT_SSE::_mm_mul_ps(dX.XYZx(), dX.XYZx()));
	}
	const TrackIndex ith = (nTrksCmn >> 1);
	std::nth_element(distSqs.begin(), distSqs.begin() + ith, distSqs.end());
	const float distSqMed = distSqs[ith];
	const float noiseRangeRigid = sqrt(distSqMed) * noisePercentRigid;

	const float sMin = 1 / maxScale, sMax = maxScale;
	Point3D center;
	float w[3], work[24];
	const SequenceIndex nSeqs = GetSequencesNumber();
	Ss.Resize(nSeqs);
	for(iSeq = 0; iSeq < nSeqsFix; ++iSeq)
		Ss[iSeq].MakeIdentity();
	for(iSeq = nSeqsFix; iSeq < nSeqs; ++iSeq)
	{
		w[0] = Random::GenerateProbability() > 0.5f ? noisePercentRigid * (Random::GenerateProbability() - 0.5f) : noisePercentRigid * (Random::GenerateProbability() - 0.5f);
		w[1] = Random::GenerateProbability() > 0.5f ? noisePercentRigid * (Random::GenerateProbability() - 0.5f) : noisePercentRigid * (Random::GenerateProbability() - 0.5f);
		w[2] = Random::GenerateProbability() > 0.5f ? noisePercentRigid * (Random::GenerateProbability() - 0.5f) : noisePercentRigid * (Random::GenerateProbability() - 0.5f);
		//memset(w, 0, sizeof(w));
		center.X() = noiseRangeRigid * 2 * (Random::GenerateProbability() - 0.5f);
		center.Y() = noiseRangeRigid * 2 * (Random::GenerateProbability() - 0.5f);
		center.Z() = noiseRangeRigid * 2 * (Random::GenerateProbability() - 0.5f);
		//center.SetZero();
		SimilarityTransformation3D &S = Ss[iSeq];
		S.FromRodrigues(w[0], w[1], w[2], work);
		S.SetCenter(center);
		S.SetScale(Random::GenerateFloat(sMin, sMax));
		m_pSeqs[iSeq]->TransformScene(S);
	}
	for(iSeq = 0; iSeq < nSeqs; ++iSeq)
		MarkSequenceRegistered(iSeq);
}

void SequenceSet::SynthesizeLoop(const ushort &width, const ushort &height, const float &fx, const float &fy, const float &cx, const float &cy, const SequenceIndex nSeqs, 
								 const SegmentIndex nSegsPerSeq, const FrameIndex nFrmsPerSeq, const TrackIndex nTrks, const FeatureIndex nFtrsPerFrm, 
								 const float dCenterPercentMax, const float dAngleMax)
{
	Sequence seqCat;
	const FrameIndex nFrmsCat = nSeqs * nFrmsPerSeq;
	seqCat.SynthesizeLoop(width, height, fx, fy, cx, cy, nFrmsCat, nTrks, nFtrsPerFrm, dCenterPercentMax, dAngleMax);
	SplitSequences(seqCat, nFrmsPerSeq);

#if _DEBUG
	assert(nFrmsPerSeq % nSegsPerSeq == 0);
#endif
	FrameIndex iFrmDst;
	const FrameIndex nFrmsPerSeg = nFrmsPerSeq / nSegsPerSeq;
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		Sequence &seqDst = *m_pSeqs[iSeq];
		iFrmDst = nFrmsPerSeg - 1;
		for(SegmentIndex i = 1; i < nSegsPerSeq; ++i, iFrmDst += nFrmsPerSeg)
		{
			seqDst.MarkFrameSplitPoint(iFrmDst);
			seqDst.MarkFrameSplitPoint(iFrmDst + 1);
		}
	}
	SetCommonPointsNumber(GetCommonTracksNumber());
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
		MarkSequenceRegistered(iSeq);
}

void SequenceSet::SynthesizeIntrinsicRectification(const float maxFocal, const float maxDistortion, Camera::IntrinsicParameter &G)
{
	const float minFocal = 1 / maxFocal;
	G.f() = Random::GenerateFloat(minFocal, maxFocal);
	G.d() = Random::GenerateFloat(-maxDistortion, maxDistortion);
	const SequenceIndex nSeqs = GetSequencesNumber();
	for(SequenceIndex iSeq = 0; iSeq < nSeqs; ++iSeq)
	{
		Sequence &seq = *m_pSeqs[iSeq];
		seq.SetIntrinsicType(Sequence::INTRINSIC_CONSTANT);
		seq.NormalizeMeasurements();
		Sequence::ScaleMeasurements(G.f(), seq.m_xs);
		Sequence::DistortMeasurements(seq.GetIntrinsicMatrix().fxy(), G.d(), seq.m_xs);
	}
	m_intrinsicType = Sequence::INTRINSIC_CONSTANT;
}

void SequenceSet::PushBackSimilarityError(const SimilarityTransformation3D &S1, const SimilarityTransformation3D &S2, std::vector<float> &sAbsErrs, 
										  std::vector<float> &sRelErrs, std::vector<float> &qAbsErrs, std::vector<float> &qRelErrs, std::vector<float> &wAbsErrs, 
										  std::vector<float> &wRelErrs, std::vector<float> &tAbsErrs, std::vector<float> &tRelErrs, std::vector<float> &centerAbsErrs, 
										  std::vector<float> &centerRelErrs)
{
	float absErr, relErr;
	LA::Vector4f q1, q2;
	LA::Vector3f w1, w2;
	Point3D t1, t2, center1, center2;

	sAbsErrs.push_back(fabs(S1.s() - S2.s()));
	sRelErrs.push_back(fabs(S1.s() - S2.s()) / S1.s());

	S1.ToQuaternion(q1);
	S2.ToQuaternion(q2);
	LA::ComputeError(q1, q2, absErr, relErr);
	qAbsErrs.push_back(absErr);
	qRelErrs.push_back(relErr);

	S1.ToRodrigues(w1);
	S2.ToRodrigues(w2);
	LA::ComputeError(w1, w2, absErr, relErr);
	wAbsErrs.push_back(absErr);
	wRelErrs.push_back(relErr);

	S1.GetTranslation(t1);
	S2.GetTranslation(t2);
	LA::ComputeError(t1, t2, absErr, relErr);
	tAbsErrs.push_back(absErr);
	tRelErrs.push_back(relErr);

	S1.GetCenter(center1);
	S2.GetCenter(center2);
	LA::ComputeError(center1, center2, absErr, relErr);
	centerAbsErrs.push_back(absErr);
	centerRelErrs.push_back(relErr);
}

void SequenceSet::PrintSimilarityErrors(const std::vector<float> &sAbsErrs, const std::vector<float> &sRelErrs, const std::vector<float> &qAbsErrs, 
										const std::vector<float> &qRelErrs, const std::vector<float> &wAbsErrs, const std::vector<float> &wRelErrs, 
										const std::vector<float> &tAbsErrs, const std::vector<float> &tRelErrs, const std::vector<float> &centerAbsErrs, 
										const std::vector<float> &centerRelErrs, const float errMedScaleFactor)
{
	printf("Similarity absolute error\n");
	printf("  s = ");			Sequence::AnalyzeAndPrintError(sAbsErrs, errMedScaleFactor);
	printf("  q = ");			Sequence::AnalyzeAndPrintError(qAbsErrs, errMedScaleFactor);
	printf("  w = ");			Sequence::AnalyzeAndPrintError(wAbsErrs, errMedScaleFactor);
	printf("  t = ");			Sequence::AnalyzeAndPrintError(tAbsErrs, errMedScaleFactor);
	printf("  center = ");		Sequence::AnalyzeAndPrintError(centerAbsErrs, errMedScaleFactor);
	printf("Similarity relative error\n");
	printf("  s = ");			Sequence::AnalyzeAndPrintError(sRelErrs, errMedScaleFactor);
	printf("  q = ");			Sequence::AnalyzeAndPrintError(qRelErrs, errMedScaleFactor);
	printf("  w = ");			Sequence::AnalyzeAndPrintError(wRelErrs, errMedScaleFactor);
	printf("  t = ");			Sequence::AnalyzeAndPrintError(tRelErrs, errMedScaleFactor);
	printf("  center = ");		Sequence::AnalyzeAndPrintError(centerRelErrs, errMedScaleFactor);
}

void SequenceSet::PushBackRigidError(const RigidTransformation3D &T1, const RigidTransformation3D &T2, std::vector<float> &qAbsErrs, std::vector<float> &qRelErrs, 
									 std::vector<float> &wAbsErrs, std::vector<float> &wRelErrs, std::vector<float> &tAbsErrs, std::vector<float> &tRelErrs, 
									 std::vector<float> &centerAbsErrs, std::vector<float> &centerRelErrs)
{
	float absErr, relErr;
	LA::Vector4f q1, q2;
	LA::Vector3f w1, w2;
	Point3D t1, t2, center1, center2;

	T1.ToQuaternion(q1);
	T2.ToQuaternion(q2);
	LA::ComputeError(q1, q2, absErr, relErr);
	qAbsErrs.push_back(absErr);
	qRelErrs.push_back(relErr);

	T1.ToRodrigues(w1);
	T2.ToRodrigues(w2);
	LA::ComputeError(w1, w2, absErr, relErr);
	wAbsErrs.push_back(absErr);
	wRelErrs.push_back(relErr);

	T1.GetTranslation(t1);
	T2.GetTranslation(t2);
	LA::ComputeError(t1, t2, absErr, relErr);
	tAbsErrs.push_back(absErr);
	tRelErrs.push_back(relErr);

	T1.GetCenter(center1);
	T2.GetCenter(center2);
	LA::ComputeError(center1, center2, absErr, relErr);
	centerAbsErrs.push_back(absErr);
	centerRelErrs.push_back(relErr);
}

void SequenceSet::PrintRigidErrors(const std::vector<float> &qAbsErrs, const std::vector<float> &qRelErrs, const std::vector<float> &wAbsErrs, 
								   const std::vector<float> &wRelErrs, const std::vector<float> &tAbsErrs, const std::vector<float> &tRelErrs, 
								   const std::vector<float> &centerAbsErrs, const std::vector<float> &centerRelErrs, const float errMedScaleFactor)
{
	printf("Rigid absolute error\n");
	printf("  q = ");			Sequence::AnalyzeAndPrintError(qAbsErrs, errMedScaleFactor);
	printf("  w = ");			Sequence::AnalyzeAndPrintError(wAbsErrs, errMedScaleFactor);
	printf("  t = ");			Sequence::AnalyzeAndPrintError(tAbsErrs, errMedScaleFactor);
	printf("  center = ");		Sequence::AnalyzeAndPrintError(centerAbsErrs, errMedScaleFactor);
	printf("Rigid relative error\n");
	printf("  q = ");			Sequence::AnalyzeAndPrintError(qRelErrs, errMedScaleFactor);
	printf("  w = ");			Sequence::AnalyzeAndPrintError(wRelErrs, errMedScaleFactor);
	printf("  t = ");			Sequence::AnalyzeAndPrintError(tRelErrs, errMedScaleFactor);
	printf("  center = ");		Sequence::AnalyzeAndPrintError(centerRelErrs, errMedScaleFactor);
}

void SequenceSet::PushBackScaleError(const float &scale1, const float &scale2, std::vector<float> &sAbsErrs, std::vector<float> &sRelErrs)
{
	sAbsErrs.push_back(fabs(scale1 - scale2));
	sRelErrs.push_back(fabs(scale1 - scale2) / scale1);
}

void SequenceSet::PrintScaleErrors(const std::vector<float> &sAbsErrs, const std::vector<float> &sRelErrs, const float errMedScaleFactor /* = 2.0f */)
{
	printf("Scale absolute error\n");
	Sequence::AnalyzeAndPrintError(sAbsErrs, errMedScaleFactor);
	printf("Scale relative error\n");
	Sequence::AnalyzeAndPrintError(sRelErrs, errMedScaleFactor);
}