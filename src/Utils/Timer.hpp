#ifndef GP__TIMER_HPP_
#define GP__TIMER_HPP_

#include "Utils/Definitions.hpp"

namespace MP
{
    namespace Timer
    {

#ifdef OS_WINDOWS
#include <windows.h>
typedef long long Clock;
#else
#include <time.h>
typedef struct timespec Clock;
#endif

	static inline void Start(Clock * const c)
	{
#ifdef OS_WINDOWS
	    QueryPerformanceCounter((LARGE_INTEGER *) c);
#else
	    clock_gettime(CLOCK_MONOTONIC, c);
#endif
	}

	static inline double Elapsed(Clock * const c)
	{
#ifdef OS_WINDOWS
	    long long end;
	    long long freq;
	    QueryPerformanceCounter((LARGE_INTEGER *) &end);
	    QueryPerformanceFrequency((LARGE_INTEGER *) &freq);
	    return ((double) (end - (*c))) / freq;
#else
	    struct timespec end;
	    clock_gettime(CLOCK_MONOTONIC, &end);
	    return (end.tv_sec - c->tv_sec) + (end.tv_nsec - c->tv_nsec) * (1e-9);
#endif
	}
    }
}

#endif
