#ifndef MP_PROXIMITY_BRUTE_FORCE_HPP_
#define MP_PROXIMITY_BRUTE_FORCE_HPP_

#include "Utils/Proximity.hpp"

namespace MP
{
    template <typename Key, typename DistFnData>
    class ProximityBruteForce : public Proximity<Key, DistFnData>
    { 
    public:
	ProximityBruteForce(void) : Proximity<Key, DistFnData>()
	{
	}


	virtual ~ProximityBruteForce(void)
	{
	}
	
	virtual void Neighbors(ProximityQuery<Key> * const query, 
			       ProximityResults<Key> * const results)
	{
	    const Key  qkey = query->GetKey();
	    const int  size = this->m_keys.size();

	    results->Clear();
	    results->SetNrNeighborsAndRange(query->GetNrNeighbors(), query->GetRange());
	    for(int i = 0; i < size; i++)
	    {
		const Key ikey = this->m_keys[i];
	    	results->Insert(ikey, this->m_distFn(qkey, ikey, this->m_distFnData));
	    }
	}
    };
}

#endif








