#ifndef CVD_IS_POD_H
#define CVD_IS_POD_H
#include <limits>

namespace CVD{
namespace Internal{

	//Define is_POD for all builtin data types, all pointer types and
	//arrays.
	template<class C> struct is_POD
	{
	  enum { is_pod = std::numeric_limits<C>::is_specialized };
	};

	template<class C> struct is_POD<C*>
	{
	  enum { is_pod = true };
	};

	template<class C, int i> struct is_POD<C[i]>
	{
	  enum { is_pod = is_POD<C>::is_pod };
	};

}
}

#endif
