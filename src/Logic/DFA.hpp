#ifndef MP_DFA_HPP_
#define MP_DFA_HPP_

#include "Logic/Formula.hpp"
#include "Utils/Misc.hpp"
#include <vector>

namespace MP
{
    class DFA
    {
    public:
	struct Edge
	{
	    int      m_zto;
	    Formula *m_formula;
	};
	
	struct State
	{
	    State(void) : m_isAccept(false)
	    {
	    }
	    
	    ~State(void)
	    {
		DeleteItems<Edge*>(&m_edges);
	    }

	    int Satisfied(const int n, const int props[]) const;
	    
	    bool               m_isAccept;
	    std::vector<Edge*> m_edges;
	};

	std::vector<State*> m_states;
	int                 m_init;

	DFA(void) : m_init(Constants::ID_UNDEFINED)
	{
	}
	
	~DFA(void)
	{
	    DeleteItems<State*>(&m_states);
	}

	void CompleteSetup(void)
	{
	}
    };


}    

#endif



