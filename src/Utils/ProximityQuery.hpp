#ifndef MP_PROXIMITY_QUERY_HPP_
#define MP_PROXIMITY_QUERY_HPP_

#include <cmath>

namespace MP
{
    template <typename Key>
    class ProximityQuery
    {
    public:
	ProximityQuery(void)
	{
	    m_k       = 0;
	    m_range   = HUGE_VAL;	    
	}

	virtual ~ProximityQuery(void) 
	{
	}

	int GetNrNeighbors(void) const
	{
	    return m_k;
	}

	double GetRange(void)
	{
	    return m_range;
	}

	Key GetKey(void) const
	{
	    return m_key;
	}

	virtual void Clear(void)
	{
	}

	virtual void SetNrNeighbors(const int k)
	{
	    m_k = k;
	}


	virtual void SetRange(const double range)
	{
	    m_range = range;
	}

	virtual void SetKey(Key key)
	{
	    m_key = key;
	}

   protected:
	int    m_k;
	double m_range;
	Key    m_key;
   };
}

#endif 








