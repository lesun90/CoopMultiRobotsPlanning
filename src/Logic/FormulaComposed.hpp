#ifndef MP_LOGIC_FORMULA_COMPOSED_HPP_
#define MP_LOGIC_FORMULA_COMPOSED_HPP_

#include "Logic/Formula.hpp"
#include "Utils/Misc.hpp"
#include <vector>

namespace MP
{
    class FormulaComposed : public Formula
    {
    public:
	FormulaComposed(void) : Formula()
	{
	}
	
	virtual ~FormulaComposed(void)
	{
	    DeleteItems<Formula *>(&m_subformulas);		
	}
	
	std::vector<Formula*> m_subformulas;
    };
}    

#endif



