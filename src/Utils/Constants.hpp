#ifndef MP_CONSTANTS_HPP_
#define MP_CONSTANTS_HPP_

#include "Utils/Definitions.hpp"
#include <cmath>

namespace MP
{
    namespace Constants
    {
	enum
	    {
		NEITHER_PARALLEL_NOR_ANTI_PARALLEL = 0,
		PARALLEL                           = 1,
		ANTI_PARALLEL                      = 2
	    };
	
	
	const double EPSILON         = ldexp(1.0, -36);
	const double EPSILON_SQUARED = ldexp(1.0, -72);
	const double SQRT_EPSILON    = ldexp(1.0, -18);
	
	const double RAD2DEG         = 180 / M_PI;
	const double DEG2RAD         = M_PI / 180;

	const int ID_UNDEFINED       = -1;
    }
}

#endif
    
    
    
    







