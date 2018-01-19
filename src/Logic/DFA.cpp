#include "Logic/DFA.hpp"
#include "Utils/Constants.hpp"

namespace MP
{
    int DFA::State::Satisfied(const int n, const int props[]) const
    {
	const int ne = m_edges.size();
	
	for(int i = 0; i < ne; ++i)
	    if(m_edges[i]->m_formula->IsSatisfied(n, props))
		return m_edges[i]->m_zto;
	return Constants::ID_UNDEFINED;
    }
    
}    



