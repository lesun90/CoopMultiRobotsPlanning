#ifndef MP_LOGIC_DFA_IO_HPP_
#define MP_LOGIC_DFA_IO_HPP_

#include "Logic/DFA.hpp"
#include <cstdio>

namespace MP
{
    class DFAIO
    {
    public:
	static DFA* ReadLBT(FILE * const in);
	static void PrintDotty(const DFA * const dfa, FILE * const out);
	
    };
    
}
    
#endif



