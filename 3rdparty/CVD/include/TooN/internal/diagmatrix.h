//-*- c++ -*-
//
// Copyright (C) 2009 Tom Drummond (twd20@cam.ac.uk),
// Ed Rosten (er258@cam.ac.uk)
//
// This file is part of the TooN Library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along
// with this library; see the file COPYING.  If not, write to the Free
// Software Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307,
// USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU General Public License.



namespace TooN {


/**
@class DiagonalMatrix 
A diagonal matrix

Support is limited but diagonal matrices can be multiplied by vectors, matrices
or diagonal matrices on either side.

Diagonal matrices can be created from vectors by using the <code> as_diagonal() 
</code> member function:

@code
Vector<3> v = makeVector(1,2,3);
Vector<3> v2 = v.as_diagonal() * v;   // v2 = (1,4,9)
@endcode

A vector can be obtained from the diagonal matrix by using the
<code> diagonal_slice() </code> member function.
@ingroup gLinAlg
 **/
template<int Size=Dynamic, typename Precision=DefaultPrecision, typename Base=Internal::VBase>
struct DiagonalMatrix {
public:
	///@name Constructors
	///@{
	
	inline DiagonalMatrix() {}
	inline DiagonalMatrix(int size_in) : my_vector(size_in) {}
	inline DiagonalMatrix(Precision* data) : my_vector(data) {}
	inline DiagonalMatrix(Precision* data, int size_in) : my_vector(data,size_in) {}
	inline DiagonalMatrix(Precision* data_in, int size_in, int stride_in, Internal::Slicing)
		: my_vector(data_in, size_in, stride_in, Internal::Slicing() ) {}

	// constructors to allow return value optimisations
	// construction from 0-ary operator
	///my_vector constructed from a TooN::Operator 
	template <class Op>
	inline DiagonalMatrix(const Operator<Op>& op)
		: my_vector (op)
	{
		op.eval(my_vector);
	}
	
	// constructor from arbitrary vector
	template<int Size2, typename Precision2, typename Base2>
	inline DiagonalMatrix(const Vector<Size2,Precision2,Base2>& from)
		: my_vector(from.size())
	{
		my_vector=from;
	}
	///@}



	///Index the leading elements on the diagonal 
	Precision& operator[](int i){return my_vector[i];}
	///Index the leading elements on the diagonal 
	const Precision& operator[](int i) const {return my_vector[i];}
	
	///Return the leading diagonal as a vector.
	typename Vector<Size, Precision, Base>::as_slice_type diagonal_slice() {
		return my_vector.as_slice();
	}

	///Return the leading diagonal as a vector.
	const typename Vector<Size, Precision, Base>::as_slice_type diagonal_slice() const {
		return my_vector.as_slice();
	}
	
	///The vector used to hold the leading diagonal.
	Vector<Size,Precision,Base> my_vector;
};


template<int S1, typename P1, typename B1, int S2, typename P2, typename B2>
inline Vector<Internal::Sizer<S1,S2>::size, typename Internal::MultiplyType<P1,P2>::type>
operator*(const DiagonalMatrix<S1,P1,B1>& d, const Vector<S2,P2,B2>& v){
	return diagmult(d.my_vector,v);
}

template<int S1, typename P1, typename B1, int S2, typename P2, typename B2>
inline Vector<Internal::Sizer<S1,S2>::size, typename Internal::MultiplyType<P1,P2>::type>
operator*( const Vector<S1,P1,B1>& v, const DiagonalMatrix<S2,P2,B2>& d){
	return diagmult(v,d.my_vector);
}

// perhaps not the safest way to do this as we're returning the same operator used to normally make vectors
template<int S1, typename P1, typename B1, int S2, typename P2, typename B2>
inline DiagonalMatrix<Internal::Sizer<S1,S2>::size, typename Internal::MultiplyType<P1,P2>::type>
operator*( const DiagonalMatrix<S1,P1,B1>& d1, const DiagonalMatrix<S2,P2,B2>& d2){
	SizeMismatch<S1,S2>::test(d1.my_vector.size(),d2.my_vector.size());
	return Operator<Internal::VPairwise<Internal::Multiply,S1,P1,B1,S2,P2,B2> >(d1.my_vector,d2.my_vector);
}

template<int R, int C, int Size, typename P1, typename P2, typename B1, typename B2>
Matrix<R, C, typename Internal::MultiplyType<P1,P2>::type>
operator* (const Matrix<R, C, P1, B1>& m, const DiagonalMatrix<Size, P2, B2>& d){
	return diagmult(m,d.my_vector);
}

template<int R, int C, typename P1, typename B1, int Size, typename P2, typename B2> 
Matrix<R, C, typename Internal::MultiplyType<P1,P2>::type>
operator* (const DiagonalMatrix<Size,P1,B1>& d, const Matrix<R,C,P2,B2>& m)
{
	return diagmult(d.my_vector, m);
}

}
