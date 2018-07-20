#ifndef CVD_RANDOM_H
#define CVD_RANDOM_H

#include <cstdlib>
#include <cmath>

namespace CVD {
	/// Uniform random numbers between 0 and 1
	///
	/// This uses rand() internally, so set thw seed with srand()
	/// @ingroup gMaths
	inline double rand_u()
	{
		return ((double) std::rand()/ RAND_MAX);
	}

	/// Gaussian random numbers with zero mean and unit standard deviation.
	///
	/// This uses rand() internally, so set thw seed with srand()
	/// @ingroup gMaths
	inline double rand_g()
	{
		static bool use_old=false;
		static double y2;
		double r;


		if(!use_old)
		{
			double x1, x2, w, y1;
			do {
				x1 = 2.0 * rand_u() - 1.0;
				x2 = 2.0 * rand_u() - 1.0;
				w = x1 * x1 + x2 * x2;
			} while ( w >= 1.0 );

			w = std::sqrt( (-2.0 * std::log( w ) ) / w );
			y1 = x1 * w;
			y2 = x2 * w;

			r = y1;
			use_old = true;
		}
		else
		{
			r = y2;
			use_old = false;
		}


		return r;
	}

}

#endif
