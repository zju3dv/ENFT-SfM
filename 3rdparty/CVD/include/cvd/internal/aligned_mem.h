#ifndef CVD_ALIGNED_MEM_H
#define CVD_ALIGNED_MEM_H
#include <cassert>
#include <cvd/config.h>

#ifdef CVD_IMAGE_DEBUG_INITIALIZE_RANDOM
	#include <ctime>
#endif

namespace CVD {
	namespace Internal
	{
		template <class T, int N=20> struct placement_delete
		{
			enum { Size = (1<<N) };

			struct Array { 
				T data[Size]; 
			};

			static inline void destruct(T* buf) 
			{
				(*(Array*)buf).~Array();
			}

			static inline void free(T* buf, size_t M) 
			{
				if (M >= Size) {
					placement_delete<T,N>::free(buf+Size,M-Size);
					placement_delete<T,N>::destruct(buf);
				} else {
					placement_delete<T,N-1>::free(buf, M);
				}
			}
		};




		#if defined CVD_IMAGE_DEBUG_INITIALIZE_RANDOM
			union intbits
			{
				unsigned long i;
				char c[4];
			};

			inline static void debug_initialize(void* data, size_t bytes)
			{
				//Get a random seed. Precision limited to 1 second.
				static intbits random = { ((std::time(NULL) & 0xffffffff) *1664525L + 1013904223L)& 0xffffffff};
				unsigned char* cdata = static_cast<unsigned char*>(data);

				size_t i=0;
				
				for(i=0; i < bytes/4;i+=4)
				{
					//Really evil random number generator from NR.
					//About 4x faster than Mersenne twister. Good quality
					//is not needed, since it's only used to shake up the program
					//state to test for uninitialized values. Also, it doesn't disturb
					//the standard library random number generator.

					//Also completely not threadsafe, but this probably doesn't matter
					//since it will just mess up the already terrible quality
					//of the PRNG.
					
					cdata[i+0] = random.c[0];	
					cdata[i+1] = random.c[1];	
					cdata[i+2] = random.c[2];	
					cdata[i+3] = random.c[3];	
					random.i = (1664525L * random.i + 1013904223L) & 0xffffffff;
				}

				for(int n=0; i+n < bytes; n++)
					cdata[n] = random.c[n];
			}
		#else
			///@internal
			///@brief This function is called on any uninitialized data. By default, no action is taken. 
			///See \ref sDebug
			inline static void debug_initialize(void *, int)
			{
			}
		#endif






		template <class T> struct placement_delete<T,-1>
		{
			static inline void free(T*, size_t ) {}
		};

		void * aligned_alloc(size_t count, size_t alignment);
		void aligned_free(void * memory);

		template <class T> 
		inline T * aligned_alloc(size_t count, size_t alignment){
			void * data = aligned_alloc(sizeof(T)* count, alignment);
			debug_initialize(data, sizeof(T)*count);
			return new (data) T[count];
		}

		template <class T>
		inline void aligned_free(T * memory, size_t count){
			placement_delete<T>::free(memory, count);   
			aligned_free(memory);
		}

	} // namespace Internal

	template <class T, int N> struct AlignedMem {
		T* mem;
		size_t count;
		AlignedMem(size_t c) : count(c) {
			mem = Internal::aligned_alloc<T>(count, N);
		}
		~AlignedMem() {
			Internal::aligned_free<T>(mem, count);
		}
		T* data() { return mem; }
		const T* data() const { return mem; }
	};    
}

#endif
