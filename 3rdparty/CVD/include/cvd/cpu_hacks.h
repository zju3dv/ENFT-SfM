#ifndef CVD_CPU_HACKS_H
#define CVD_CPU_HACKS_H


#include <cvd/config.h>

#if CVD_HAVE_FENV_H
	#include <fenv.h>
#endif

#if CVD_HAVE_MMXEXT
    #include <xmmintrin.h>
#endif

namespace CVD
{


	/// Enable floating point exceptions. This function may
	/// do nothing, depending on the architecture
	inline void enableFPE();

	#ifndef DOXYGEN_IGNORE_INTERNAL
		#ifdef CVD_HAVE_FENV_H
			inline void enableFPE() 
			{ 
				feclearexcept(FE_ALL_EXCEPT);
				feenableexcept(FE_DIVBYZERO|FE_INVALID); 
			}
		#else
			/// Enable floating point exceptions. This function may
			/// do nothing, depending on the architecture
			inline void enableFPE() 
			{ 
			}
		#endif
	#endif

	/// Prefetch memory. This function might do nothing, depending on the
	/// architecture, or it might prefetch memory. Either way it will have
	/// no effect on the computation except to (possibly) speed it up.
	/// @param ptr The address of the memory to prefetch.
	/// @param I   The type of prefetch. This depends on the architecture.
	///            	- x86, MMXEXT
	///					- 0: prefetcht0
	///					- 1: prefetcht1
	///					- 2: prefetcht2
	///					- *: prefetchnta  (default)
	///				- Everything else
	///					- *: nothing
	template<int I> inline void prefetch(const void* ptr);
	
	inline void prefetch(const void* ptr);


	#ifndef DOXYGEN_IGNORE_INTERNAL
		#ifdef  CVD_HAVE_MMXEXT	
			template<int I> inline void prefetch(const void* ptr)
			{
                _mm_prefetch((char *)ptr, _MM_HINT_NTA);
			}

			template<> inline void prefetch<0>(const void* ptr)
			{
				_mm_prefetch((char *)ptr, _MM_HINT_T0);
			}
			
			template<> inline void prefetch<1>(const void* ptr)
			{
				_mm_prefetch((char *)ptr, _MM_HINT_T1);
			}

			template<> inline void prefetch<2>(const void* ptr)
			{
                _mm_prefetch((char *)ptr, _MM_HINT_T2);
			}
		
			inline void prefetch(const void* ptr)
			{
				prefetch<-1>(ptr);
			}

		#else
			template<int i> inline void prefetch(const void*){}
			inline void prefetch(const void*){}

		#endif
	#endif
}
#endif


