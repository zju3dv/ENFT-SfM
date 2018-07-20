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

#ifndef _RANDOM_H_
#define _RANDOM_H_

#define UT_RANDOM_MAX 32767

class Random
{

public:

	// rand() = [0, 32767] = [0x0000, 0x7fff]
	//static int Z;
	//static int I;
	//static int L;
	//static inline void Start()
	//{
	//	Z = 5173;
	//	I = 3849;
	//	L = 0;
	//}
	//static inline ushort rand() { L = (Z * L + I) & 32767; return ushort(L); }
	static inline ubyte	 GenerateUbyte() { return ubyte(rand() & 255); }
	static inline ushort GenerateUshort() { return ushort(rand() % UT_RANDOM_MAX); }
	static inline ushort GenerateUshort(const ushort &max) { return ushort(rand() % max); }
	static inline ushort Generate(const ushort &max) { return GenerateUshort(max); }
	//static inline uint	 GenerateUint  ()									{ return (rand() << 15) + rand(); }
	static inline uint	 GenerateUint() { return uint(rand() % UT_RANDOM_MAX); }
	static inline uint	 GenerateUint(const uint   &max) { return GenerateUint() % max; }
	static inline uint	 Generate(const uint &max) { return GenerateUint(max); }
	static inline float  GenerateProbability() { return GenerateUshort() / 32767.0f; }
	static inline float  GenerateFloat(const float &max) { return GenerateProbability() * max; }
	static inline float  GenerateFloat(const float &min, const float &max) { return GenerateProbability() * (max - min) + min; }
	static void GenerateFloats(const float &min, const float &max, const ushort &N, float *floats)
	{
		for (ushort i = 0; i < N; ++i)
			floats[i] = GenerateFloat(min, max);
	}
	static void GenerateFloats(const float &min, const float &max, const ushort &N, double *floats)
	{
		for (ushort i = 0; i < N; ++i)
			floats[i] = double(GenerateFloat(min, max));
	}
	static void GenerateAngles(const float &min, const float &max, const ushort &N, float *angles)
	{
		for (ushort i = 0; i < N; ++i)
			angles[i] = GenerateFloat(min, max) * FACTOR_DEG_TO_RAD;
	}

	// Generate an ushort number between [0, N);
	// The probability of generating i is (accumulatedWeights[i+1] - accumulatedWeights[i]) / accumulatedWeights[N], 
	// where accumulatedWeights[i+1] > accumulatedWeights[i] and SIGMA_{i=0~N-1}(accumulatedWeights[i+1] - accumulatedWeights[i]) = accumulatedWeights[N]
	// So the we randomly generate j between [0, accumulatedWeights[N]), then i is index of the last element in accumulatedWeights that is less than or equal to j
	static inline ushort GenerateUshort(const std::vector<uint> &accumulatedWeights)
	{
		const uint j = accumulatedWeights.back() < 32768 ? uint(GenerateUshort(ushort(accumulatedWeights.back()))) : GenerateUint(accumulatedWeights.back());
		return ushort(std::upper_bound(accumulatedWeights.begin() + 1, accumulatedWeights.end(), j) - accumulatedWeights.begin() - 1);
	}
	// Assume probabilities have been normalized, i.e., accumulatedWeights[N] = 1
	static inline uint GenerateUint(const std::vector<float> &accumulatedWeights)
	{
		const float j = GenerateProbability();
		const uint i = uint(std::upper_bound(accumulatedWeights.begin() + 1, accumulatedWeights.end(), j) - accumulatedWeights.begin() - 1);
		return std::min(i, uint(accumulatedWeights.size()) - 2);
	}
	//static inline ushort GenerateUint(const std::vector<float> &accumulatedWeights, const bool verbose)
	//{
	//	const float j = GenerateProbability();
	//	if(verbose)
	//		printf("%f, %d\n", j, std::upper_bound(accumulatedWeights.begin() + 1, accumulatedWeights.end(), j) - accumulatedWeights.begin() - 1);
	//	return std::upper_bound(accumulatedWeights.begin() + 1, accumulatedWeights.end(), j) - accumulatedWeights.begin() - 1;
	//}
};

#endif