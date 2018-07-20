#ifndef CVD_INC_INTERNAL_SHARED_PTR_H
#define CVD_INC_INTERNAL_SHARED_PTR_H

#include "cvd/config.h"

#if (defined CVD_INTERNAL_NEED_TR1) && !(defined WIN32 && !defined __MINGW32__)
	#include <tr1/memory>
	namespace CVD{
		namespace STD{
			using std::tr1::shared_ptr;
		};
	};
#else
	#include <memory>
	namespace CVD{
		namespace STD{
			using std::shared_ptr;
		}
	}
#endif
#endif
