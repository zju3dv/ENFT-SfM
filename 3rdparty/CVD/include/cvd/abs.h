#ifndef CVD_ABS_H
#define CVD_ABS_H

namespace CVD
{
	/// Local templated version of abs. Returns the absolute value of a variable.
	/// @param t The input parameter
	/// @ingroup gCPP
	template <class T> 
	inline T abs(T t) { return t<0 ? -t : t; }
	/// Fast instantiation for unsigned datatype which avoids a comparison
	/// @param b The input parameter
	/// @ingroup gCPP
	inline unsigned char abs(unsigned char b) { return b; }
	/// Fast instantiation for unsigned datatype which avoids a comparison
	/// @param u The input parameter
	/// @ingroup gCPP
	inline unsigned short abs(unsigned short u) { return u; }
	/// Fast instantiation for unsigned datatype which avoids a comparison
	/// @param u The input parameter
	/// @ingroup gCPP
	inline unsigned int abs(unsigned int u) { return u; }
	/// Fast instantiation for unsigned datatype which avoids a comparison
	/// @param u The input parameter
	/// @ingroup gCPP
	inline unsigned long abs(unsigned long u) { return u; }
}
#endif

