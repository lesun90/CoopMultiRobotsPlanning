#ifndef MP_STATISTICS_HPP_
#define MP_STATISTICS_HPP_

#include <vector>

namespace MP
{
    template <typename Type>
    static inline double Sum(const int n, const Type items[])
    {
	double s = 0;	
	for(int i = 0; i < n; ++i)
	    s += items[i];
	return s;
    }
    
    template <typename Type>
    static inline double Mean(const int n, const Type items[])
    {
	return Sum(n, items) / n;	
    }

    template <typename Type>
    static inline double Variance(const int n, const Type items[], const double mean)
    {
	double s = 0;
	for(int i = 0; i < n; ++i)
	    s += (items[i] - mean) * (items[i] - mean);
	return s / n;
    }
    
    template <typename Type>
    static inline double Variance(const int n, const Type items[])
    {
	return Variance(n, items, Mean(n, items));
    }
    
    template <typename Type>
    static inline double MedianInSorted(const int n, const Type items[])
    {
	return n % 2 ? items[n / 2] : ((items[n / 2] + items[n / 2 - 1]) / 2.0);
    }
    
    template <typename Type>
    static inline double Q1InSorted(const int n, const Type items[])
    {
	return MedianInSorted(n/2, items);
    }

    template <typename Type>
    static inline double Q3InSorted(const int n, const Type items[])
    {
	//n = 7: 0 1 2   3   4 5 6 
        //n/2=3  

	//n=8: 0 1 2 3   4 5 6 7
	//n/2=4
	return MedianInSorted(n/2, &items[(n%2) + n/2]);
    }
    
    
}	


#endif



