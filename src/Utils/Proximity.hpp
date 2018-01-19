#ifndef MP_PROXIMITY_HPP_
#define MP_PROXIMITY_HPP_

#include "Utils/ProximityQuery.hpp"
#include "Utils/ProximityResults.hpp"
#include <vector>
#include <cstdlib>

namespace MP
{
    template <typename Key, typename DistFnData>
    class Proximity
    {
    public:
	Proximity(void)
	{
	    m_construct = false;	    
	}

	virtual ~Proximity(void) 
	{
	}

	typedef double (*DistFn) (const Key, const Key, DistFnData);

	DistFn     m_distFn;
	DistFnData m_distFnData;
	
	bool IsDataStructureConstructed(void) const
	{
	    return m_construct;
	}

	const std::vector<Key>* GetKeys(void) const
	{
	    return &m_keys;
	}

	virtual void AddKey(const Key key)
	{
	    m_keys.push_back(key);	    
	}
	
	virtual void ConstructDataStructure(void)
	{
	    m_construct = true;
	}

	virtual void ClearDataStructure(void)
	{
	    m_construct = false;
	    m_keys.clear();
	}

	virtual void Clear(void)
	{
	    ClearDataStructure();
	}

	virtual Key Neighbor(ProximityQuery<Key> * const query, double * const d = NULL)
	{
	    ProximityResults<Key> pr;
	    
	    query->SetNrNeighbors(1);	    
	    Neighbors(query, &pr);

	    if(d)
		*d = pr.GetDistance(0);	    
	    return pr.GetKey(0);
	}
	
	virtual void Neighbors(ProximityQuery<Key>   * const query, 
			       ProximityResults<Key> * const results) = 0;

    protected:
	bool             m_construct;
	std::vector<Key> m_keys;	
    };      
}

#endif 








