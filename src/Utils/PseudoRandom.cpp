#include "Utils/PseudoRandom.hpp"
#include <cstdio>
#include <ctime>

namespace MP
{
    unsigned int RandomSeed(void)
    {
	FILE        *fp = fopen("/dev/urandom", "r");    
	unsigned int s;
	int          ignore;
	
	if(fp != NULL)
	{
	    ignore = fread(&s, sizeof(unsigned int), 1, fp);    
	    fclose(fp);    	        
	}
	else
	    s = (unsigned int) time(NULL);
	
	printf("using seed: %u\n", s);
	
	
	RandomSeed(s);
	
	return s;
    }
    
    void RandomSeed(const unsigned int s)
    {
	srandom(s);
    }
    
    double RandomGaussianReal(const double mean, const double stddev)
    {
	double        x1, x2, w, y1;    
	static double y2;
	static int    use_last = 0;
	
	if (use_last)                                /* use gaussian sample from previous time. */
	{
	    y1       = y2;
	    use_last = 0;
	}
	else
	{
	    do
	    {
		x1 = 2.0 * RandomUniformReal() - 1.0;   /* select uniformly at random a point inside */
		x2 = 2.0 * RandomUniformReal() - 1.0;   /* the square [-1, 1] x [-1, 1]. */	    
		w  = x1 * x1 + x2 * x2;              /* continue the selection as long as */
	    }                                        /* the point is not inside */
	    while (w >= 1.0 || w == 0.0);            /* the unit circle. */
	    
	    w = sqrt ((-2.0 * log(w) ) / w);  
	    
	    y1       = x1 * w;                       /* complete the Box-Muller transformations to get */
	    y2       = x2 * w;                       /* two gaussian samples. */
	    use_last = 1;                            /* use one now and save the other for the next time. */
	} 
	/* transform the sample to a Gaussian distribution */
	return mean + y1 * stddev;                   /* with mean mean and standard deviation stddev. */
    }
    
    int RandomSelectWeighted(const int n, const double weights[], const double tw)
    {
	const double wr = RandomUniformReal(0, tw);
	double       w  = 0.0;
	for(int i = 0; i < n; ++i)
	{
	    w += weights[i];
	    if((w + Constants::EPSILON) >= wr)
		return i;
	}
	assert(false);
	return -1;
    }
}





    
    
    
    
    
    
