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

#ifndef _MATRIX_4_H_
#define _MATRIX_4_H_

#include "Matrix3x4.h"
#include "Vector4.h"

namespace LA
{

	class AlignedMatrix4f : public AlignedMatrix3x4f
	{

	public:

		inline const ENFT_SSE::__m128& M_30_31_32_33() const { return m_30_31_32_33; }	inline ENFT_SSE::__m128& M_30_31_32_33() { return m_30_31_32_33; }
		inline const float& M30() const { return m_30_31_32_33.m128_f32[0]; }	inline float& M30()	{ return m_30_31_32_33.m128_f32[0]; }
		inline const float& M31() const { return m_30_31_32_33.m128_f32[1]; }	inline float& M31()	{ return m_30_31_32_33.m128_f32[1]; }
		inline const float& M32() const { return m_30_31_32_33.m128_f32[2]; }	inline float& M32()	{ return m_30_31_32_33.m128_f32[2]; }
		inline const float& M33() const { return m_30_31_32_33.m128_f32[3]; }	inline float& M33() { return m_30_31_32_33.m128_f32[3]; }
		inline void SetZero() { memset(this, 0, sizeof(AlignedMatrix4f)); }
		inline void GetDiagonal(AlignedVector4f &d) const { d.v0() = M00();	d.v1() = M11();	d.v2() = M22(); d.v3() = M33(); }
		inline void SetDiagonal(const AlignedVector4f &d) { M00() = d.v0(); M11() = d.v1(); M22() = d.v2(); M33() = d.v3(); }
		inline void ScaleDiagonal(const float &lambda) { M00() *= lambda; M11() *= lambda; M22() *= lambda; M33() *= lambda; }
		inline void IncreaseDiagonal(const float &lambda) { M00() += lambda; M11() += lambda; M22() += lambda; M33() += lambda; }
		inline void operator *= (const ENFT_SSE::__m128 &s)
		{
			M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(s, M_00_01_02_03());
			M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(s, M_10_11_12_13());
			M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(s, M_20_21_22_23());
			M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(s, M_30_31_32_33());
		}
		inline void SetLowerFromUpper()
		{
			M10() = M01();
			M20() = M02();	M21() = M12();
			M30() = M03();	M31() = M13();	M32() = M23();
		}
		inline void Print() const
		{
			printf("%f %f %f %f\n", M00(), M01(), M02(), M03());
			printf("%f %f %f %f\n", M10(), M11(), M12(), M13());
			printf("%f %f %f %f\n", M20(), M21(), M22(), M23());
			printf("%f %f %f %f\n", M30(), M31(), M32(), M33());
		}
		inline void SaveB(FILE *fp) const { fwrite(this, sizeof(LA::AlignedMatrix4f), 1, fp); }
		inline void LoadB(FILE *fp) { fread(this, sizeof(LA::AlignedMatrix4f), 1, fp); }

	protected:

		ENFT_SSE::__m128 m_30_31_32_33;

	};

	inline void AddAATToUpper(const AlignedVector3f &A, AlignedMatrix4f &to)
	{
#if _DEBUG
		assert(A.reserve() == 1.0f);
#endif
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v0()), A.v012x()), to.M_00_01_02_03());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v1()), A.v012x()), to.M_10_11_12_13());
		to.M22() = A.v2() * A.v2() + to.M22();
		to.M23() = A.v2() + to.M23();
		to.M33() = 1.0f + to.M33();
	}
	inline void AddATAToUpper(const AlignedMatrix2x4f &A, AlignedMatrix4f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), A.M_10_11_12_13())), 
										to.M_00_01_02_03());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), A.M_00_01_02_03()), ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), A.M_10_11_12_13())), 
										to.M_10_11_12_13());
		to.M22() = A.M02() * A.M02() + A.M12() * A.M12() + to.M22();
		to.M23() = A.M02() * A.M03() + A.M12() * A.M13() + to.M23();
		to.M33() = A.M03() * A.M03() + A.M13() * A.M13() + to.M33();
	}
	template<class MATRIX> inline void FinishAdditionATAToUpper(AlignedMatrix4f &to) {}
	inline void GetDiagonal(const AlignedMatrix4f &M, AlignedVector4f &d) { M.GetDiagonal(d); }
	inline void SetDiagonal(const AlignedVector4f &d, AlignedMatrix4f &M) { M.SetDiagonal(d); }
	inline void ScaleDiagonal(const float &lambda, AlignedMatrix4f &M) { M.ScaleDiagonal(lambda); }
	inline void IncreaseDiagonal(const float &lambda, AlignedMatrix4f &M) { M.IncreaseDiagonal(lambda); }
	inline void SetLowerFromUpper(AlignedMatrix4f &A) { A.SetLowerFromUpper(); }

	inline void ssTA(const AlignedVector4f &s, AlignedMatrix4f &A)
	{
		A.M_00_01_02_03() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v0()), s.v0123()), A.M_00_01_02_03());
		A.M_10_11_12_13() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v1()), s.v0123()), A.M_10_11_12_13());
		A.M_20_21_22_23() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v2()), s.v0123()), A.M_20_21_22_23());
		A.M_30_31_32_33() = ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(s.v3()), s.v0123()), A.M_30_31_32_33());
	}
	inline void AB(const AlignedMatrix4f &A, const AlignedVector4f &B, AlignedVector4f &AB)
	{
		AB.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123()));
		AB.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123()));
		AB.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123()));
		AB.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123()));
	}
	inline void AddABTo(const AlignedMatrix4f &A, const AlignedVector4f &B, AlignedVector4f &to)
	{
		to.v0() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_00_01_02_03(), B.v0123())) + to.v0();
		to.v1() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_10_11_12_13(), B.v0123())) + to.v1();
		to.v2() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_20_21_22_23(), B.v0123())) + to.v2();
		to.v3() = ENFT_SSE::SSE::Sum0123(ENFT_SSE::_mm_mul_ps(A.M_30_31_32_33(), B.v0123())) + to.v3();
	}
	inline void SubtractATBFrom(const AlignedMatrix3x4f &A, const AlignedMatrix3x4f &B, AlignedMatrix4f &from)
	{
		from.M_00_01_02_03() = ENFT_SSE::_mm_sub_ps(from.M_00_01_02_03(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M00()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M10()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M20()), B.M_20_21_22_23())));
		from.M_10_11_12_13() = ENFT_SSE::_mm_sub_ps(from.M_10_11_12_13(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M01()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M11()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M21()), B.M_20_21_22_23())));
		from.M_20_21_22_23() = ENFT_SSE::_mm_sub_ps(from.M_20_21_22_23(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M02()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M12()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M22()), B.M_20_21_22_23())));
		from.M_30_31_32_33() = ENFT_SSE::_mm_sub_ps(from.M_30_31_32_33(), ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M03()), B.M_00_01_02_03()), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M13()), B.M_10_11_12_13())), 
																					  ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.M23()), B.M_20_21_22_23())));
	}

	inline void AddAATToUpper(const AlignedVector4f &A, AlignedMatrix4f &to)
	{
		to.M_00_01_02_03() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v0()), A.v0123()), to.M_00_01_02_03());
		to.M_10_11_12_13() = ENFT_SSE::_mm_add_ps(ENFT_SSE::_mm_mul_ps(ENFT_SSE::_mm_set1_ps(A.v1()), A.v0123()), to.M_10_11_12_13());
		to.M22() = A.v2() * A.v2() + to.M22();
		to.M23() = A.v2() * A.v3() + to.M23();
		to.M33() = A.v3() * A.v3() + to.M33();
	}

	bool InvertSymmetricUpper(AlignedMatrix4f &A);
	bool InvertSymmetricUpper(const AlignedMatrix4f &A, AlignedMatrix4f &Ainv);
	bool SolveLinearSystemSymmetricUpper(AlignedMatrix4f &A, AlignedVector4f &b);
	inline bool SolveLinearSystemSymmetricUpper(AlignedMatrix4f &A, AlignedVector4f &b, float *work0) { return SolveLinearSystemSymmetricUpper(A, b); }

	template<typename TYPE> class Matrix4
	{
	public:
		Matrix4() { MakeIdentity(); }
		inline const TYPE& M00() const { return m_data[ 0]; }	inline TYPE& M00() { return m_data[ 0]; }
		inline const TYPE& M01() const { return m_data[ 1]; }	inline TYPE& M01() { return m_data[ 1]; }
		inline const TYPE& M02() const { return m_data[ 2]; }	inline TYPE& M02() { return m_data[ 2]; }
		inline const TYPE& M03() const { return m_data[ 3]; }	inline TYPE& M03() { return m_data[ 3]; }
		inline const TYPE& M10() const { return m_data[ 4]; }	inline TYPE& M10() { return m_data[ 4]; }
		inline const TYPE& M11() const { return m_data[ 5]; }	inline TYPE& M11() { return m_data[ 5]; }
		inline const TYPE& M12() const { return m_data[ 6]; }	inline TYPE& M12() { return m_data[ 6]; }
		inline const TYPE& M13() const { return m_data[ 7]; }	inline TYPE& M13() { return m_data[ 7]; }
		inline const TYPE& M20() const { return m_data[ 8]; }	inline TYPE& M20() { return m_data[ 8]; }
		inline const TYPE& M21() const { return m_data[ 9]; }	inline TYPE& M21() { return m_data[ 9]; }
		inline const TYPE& M22() const { return m_data[10]; }	inline TYPE& M22() { return m_data[10]; }
		inline const TYPE& M23() const { return m_data[11]; }	inline TYPE& M23() { return m_data[11]; }
		inline const TYPE& M30() const { return m_data[12]; }	inline TYPE& M30() { return m_data[12]; }
		inline const TYPE& M31() const { return m_data[13]; }	inline TYPE& M31() { return m_data[13]; }
		inline const TYPE& M32() const { return m_data[14]; }	inline TYPE& M32() { return m_data[14]; }
		inline const TYPE& M33() const { return m_data[15]; }	inline TYPE& M33() { return m_data[15]; }
		inline operator const TYPE* () const { return (const TYPE *) this; }
		inline operator		  TYPE* ()		  { return		 (TYPE *) this; }
		inline void MakeIdentity() { memset(this, 0, sizeof(Matrix4<TYPE>)); M00() = M11() = M22() = M33() = 1; }
		inline void Transpose()
		{
			TYPE tmp;
			SWAP(M01(), M10(), tmp);	SWAP(M02(), M20(), tmp);	SWAP(M03(), M30(), tmp);
			SWAP(M12(), M21(), tmp);	SWAP(M13(), M31(), tmp);	SWAP(M23(), M32(), tmp);
		}
		inline void Set(const LA::AlignedMatrix3f &M)
		{
			M00() = M.M00();	M01() = M.M01();	M02() = M.M02();	M03() = 0;
			M10() = M.M10();	M11() = M.M11();	M12() = M.M12();	M13() = 0;
			M20() = M.M20();	M21() = M.M21();	M22() = M.M22();	M23() = 0;
			M30() = 0;			M31() = 0;			M32() = 0;			M33() = 1;
		}
	private:
		TYPE m_data[16];
	};

	typedef Matrix4<float > Matrix4f;
	typedef Matrix4<double> Matrix4d;

#if _DEBUG
	inline void SubtractATBFrom_Debug(const AlignedMatrix3x4f &A, const AlignedMatrix3x4f &B, AlignedMatrix4f &from)
	{
		const float* _A[3] = {&A.M00(), &A.M10(), &A.M20()};
		const float* _B[3] = {&B.M00(), &B.M10(), &B.M20()};
		float* _from[4] = {&from.M00(), &from.M10(), &from.M20(), &from.M30()};
		for(int i = 0; i < 4; ++i)
		for(int j = 0; j < 4; ++j)
		for(int k = 0; k < 3; ++k)
			_from[i][j] -= _A[k][i] * _B[k][j];
	}
#endif

}

#endif